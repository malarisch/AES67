library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity wallclock is
    generic(
        -- nanoseconds added per clock tick (250 MHz -> 4 ns)
        increment_interval : natural := 4
    );
    port(
        clk                     : in  std_logic;
        reset_n                 : in  std_logic;

        wallclock_seconds_o     : out unsigned(31 downto 0);
        wallclock_nanoseconds_o : out unsigned(31 downto 0);

        wallclock_set_i         : in  std_logic;
        wallclock_seconds_i     : in  unsigned(31 downto 0);
        wallclock_nanoseconds_i : in  unsigned(31 downto 0);

        second_pulse_o          : out std_logic
    );
end wallclock;

architecture Behavioral of wallclock is
    constant NS_PER_SEC : unsigned(31 downto 0) := to_unsigned(1_000_000_000, 32);
    constant INC_NS     : unsigned(3 downto 0)  := to_unsigned(8, 4);  -- 250 MHz -> 4ns
    
begin
    process(clk, reset_n)
        variable new_nsec : integer range 0 to 1_000_500_000;
        variable new_sec : unsigned(31 downto 0);
    begin
        if reset_n = '0' then
            second_pulse_o     <= '0';
            new_nsec          := 0;
            new_sec           := (others => '0');

        elsif rising_edge(clk) then
            second_pulse_o <= '0';

            if wallclock_set_i = '1' then
                
                new_nsec := to_integer(wallclock_nanoseconds_i);
                new_sec  := wallclock_seconds_i;
            else
                
                new_nsec := new_nsec + 8;
                if new_nsec >= NS_PER_SEC then
                    new_nsec := 0;
                    new_sec  := new_sec + 1;
                    second_pulse_o     <= '1';
                end if;

            end if;
            wallclock_nanoseconds_o <= TO_UNSIGNED(new_nsec, 32);
            wallclock_seconds_o     <= new_sec;
        end if;
    end process;

    
    
end Behavioral;
