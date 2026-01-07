library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity ethernet_timestamp is
    port(
        clk			: in std_logic;
        reset_n			: in std_logic;
        
        wallclock_seconds_i : in unsigned(31 downto 0);
        wallclock_nanoseconds_i : in unsigned(31 downto 0);
        timestamp_seconds_o	: out unsigned(31 downto 0);
        timestamp_nanoseconds_o : out unsigned(31 downto 0);
        
        timestamp_set_i : in std_logic

        
    );
end ethernet_timestamp;
architecture Behavioral of ethernet_timestamp is
    signal timestamp_sec_reg : unsigned(31 downto 0) := (others => '0');
    signal timestamp_nsec_reg : unsigned(31 downto 0) := (others => '0');
begin
    process(clk, reset_n)
    begin
        if reset_n = '0' then
            timestamp_sec_reg <= (others => '0');
            timestamp_nsec_reg <= (others => '0');
        elsif rising_edge(clk) then
            if timestamp_set_i = '1' then
                timestamp_sec_reg <= wallclock_seconds_i;
                timestamp_nsec_reg <= wallclock_nanoseconds_i;
            end if;
        end if;
    end process;
    timestamp_seconds_o <= timestamp_sec_reg;
    timestamp_nanoseconds_o <= timestamp_nsec_reg;
end Behavioral;