library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity wallclock is
    generic(
        -- nanoseconds added per clock tick (125 MHz -> 8 ns)
        increment_interval : natural := 8
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
    signal wallclock_sec_reg  : unsigned(31 downto 0) := (others => '0');
    signal wallclock_nsec_reg : unsigned(31 downto 0) := (others => '0');

    constant NS_PER_SEC : unsigned(31 downto 0) := to_unsigned(1_000_000_000, 32);
    constant INC_NS     : unsigned(31 downto 0) := to_unsigned(increment_interval, 32);
begin
    process(clk, reset_n)
        
    
    begin
        if reset_n = '0' then
            wallclock_sec_reg  <= (others => '0');
            wallclock_nsec_reg <= (others => '0');
            second_pulse_o     <= '0';

        elsif rising_edge(clk) then
            second_pulse_o <= '0';

            

            
                

                if wallclock_nsec_reg >= NS_PER_SEC then
                    wallclock_nsec_reg <= wallclock_nsec_reg - NS_PER_SEC;
                    wallclock_sec_reg  <= wallclock_sec_reg + 1;
                    second_pulse_o     <= '1';
                else
                    wallclock_nsec_reg <= wallclock_nsec_reg + INC_NS;
                end if;
            
        end if;
    end process;

    wallclock_seconds_o     <= wallclock_sec_reg;
    wallclock_nanoseconds_o <= wallclock_nsec_reg;
end Behavioral;
