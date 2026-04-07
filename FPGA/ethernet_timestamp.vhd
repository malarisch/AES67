library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- ============================================================
-- Ethernet RX Timestamp Latch
-- ============================================================

entity ethernet_timestamp is
    port(
        clk			: in std_logic; -- sys clock domain
        reset_n			: in std_logic;
        
        wallclock_seconds_i : in unsigned(47 downto 0);
        wallclock_nanoseconds_i : in unsigned(31 downto 0);
        timestamp_seconds_o	: out unsigned(47 downto 0);
        timestamp_nanoseconds_o : out unsigned(31 downto 0);
        

        -- signals from ethernet clock domain
        sof_tog_i : in std_logic


        
    );
end ethernet_timestamp;

architecture Behavioral of ethernet_timestamp is

    -- synchronizers
    signal sof_tog_i_meta : std_logic := '0';
    signal sof_tog_i_sync : std_logic := '0';
    signal sof_tog_i_prev : std_logic := '0';

    

    
    
begin
    process(clk, reset_n)
    begin
        if reset_n = '0' then
            sof_tog_i_meta <= '0';
            sof_tog_i_sync <= '0';
            sof_tog_i_prev <= '0';
        elsif rising_edge(clk) then
            -- Synchronize timestamp_set_i into clk domain
            sof_tog_i_meta <= sof_tog_i;
            sof_tog_i_sync <= sof_tog_i_meta;
            sof_tog_i_prev <= sof_tog_i_sync;


            if (sof_tog_i_prev /= sof_tog_i_sync) then
                timestamp_seconds_o <= wallclock_seconds_i;
                timestamp_nanoseconds_o <= wallclock_nanoseconds_i;
            end if;


        end if;
    end process;
    
end Behavioral;