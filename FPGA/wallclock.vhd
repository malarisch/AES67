library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity wallclock is
    generic(
        -- no generics
        increment_interval : integer := 8  -- number of clock cycles per increment

    );
    port(
        clk			: in std_logic;
        reset_n		: in std_logic;
        
        wallclock_seconds_o	: out unsigned(31 downto 0);
        wallclock_nanoseconds_o : out unsigned(31 downto 0);
        wallclock_set_i : in std_logic;
        wallclock_seconds_i : in unsigned(31 downto 0);
        wallclock_nanoseconds_i : in unsigned(31 downto 0)
    );
end wallclock;

architecture Behavioral of wallclock is
    signal wallclock_sec_reg : unsigned(31 downto 0) := (others => '0');
    signal wallclock_nsec_reg : unsigned(31 downto 0) := (others => '0');
    signal clock_counter : unsigned(8 downto 0) := (others => '0');
begin
    process(clk, reset_n)
    begin
        if reset_n = '0' then
            wallclock_sec_reg <= (others => '0');
            wallclock_nsec_reg <= (others => '0');
        elsif rising_edge(clk) then
            if wallclock_set_i = '1' then
                wallclock_sec_reg <= wallclock_seconds_i;
                wallclock_nsec_reg <= wallclock_nanoseconds_i;
            else
                if clock_counter = to_unsigned(increment_interval - 1, clock_counter'length) then
                    clock_counter <= (others => '0');
                else
                    clock_counter <= clock_counter + 1;
                end if;
                if clock_counter = to_unsigned(increment_interval - 1, clock_counter'length) then
                    wallclock_nsec_reg <= wallclock_nsec_reg + 1;
                end if;
                if wallclock_nsec_reg = to_unsigned(999_999_999, wallclock_nsec_reg'length) and clock_counter = to_unsigned(increment_interval - 1, clock_counter'length) then
                    wallclock_nsec_reg <= (others => '0');
                    wallclock_sec_reg <= wallclock_sec_reg + 1;
                end if;
            end if;
        end if;
    end process;
    wallclock_seconds_o <= wallclock_sec_reg;
    wallclock_nanoseconds_o <= wallclock_nsec_reg;
end Behavioral;