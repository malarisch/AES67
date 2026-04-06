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
        mac_rx_frame_i : in std_logic;
        ethernet_parser_sync_in_i : in std_logic;
        is_ptp_packet_i : in std_logic

        
    );
end ethernet_timestamp;

architecture Behavioral of ethernet_timestamp is
    signal timestamp_sec_reg  : unsigned(47 downto 0) := (others => '0');
    signal timestamp_nsec_reg : unsigned(31 downto 0) := (others => '0');

    signal timestamp_sec_latch     : unsigned(47 downto 0) := (others => '0');
    signal timestamp_nsec_latch    : unsigned(31 downto 0) := (others => '0');

    -- synchronizers
    signal mac_rx_frame_meta : std_logic := '0';
    signal mac_rx_frame_sync : std_logic := '0';
    signal mac_rx_frame_prev : std_logic := '0';

    signal eth_parser_sync_meta : std_logic := '0';
    signal eth_parser_sync_sync : std_logic := '0';
    signal eth_parser_sync_prev : std_logic := '0';

    signal is_ptp_packet_meta : std_logic := '0';
    signal is_ptp_packet_sync : std_logic := '0';
    signal is_ptp_packet_prev : std_logic := '0';
    

    
    
begin
    process(clk, reset_n)
    begin
        if reset_n = '0' then
            timestamp_sec_reg <= (others => '0');
            timestamp_nsec_reg <= (others => '0');

            timestamp_sec_latch <= (others => '0');
            timestamp_nsec_latch <= (others => '0');
            mac_rx_frame_meta <= '0';
            mac_rx_frame_sync <= '0';
            mac_rx_frame_prev <= '0';
            eth_parser_sync_meta <= '0';
            eth_parser_sync_sync <= '0';
            eth_parser_sync_prev <= '0';
            is_ptp_packet_meta <= '0';
            is_ptp_packet_sync <= '0';
            is_ptp_packet_prev <= '0';
        elsif rising_edge(clk) then
            -- Synchronize timestamp_set_i into clk domain
            mac_rx_frame_meta <= mac_rx_frame_i;
            mac_rx_frame_sync <= mac_rx_frame_meta;
            mac_rx_frame_prev <= mac_rx_frame_sync;

            eth_parser_sync_meta <= ethernet_parser_sync_in_i;
            eth_parser_sync_sync <= eth_parser_sync_meta;
            eth_parser_sync_prev <= eth_parser_sync_sync;

            is_ptp_packet_meta <= is_ptp_packet_i;
            is_ptp_packet_sync <= is_ptp_packet_meta;
            is_ptp_packet_prev <= is_ptp_packet_sync;

            if (mac_rx_frame_prev = '0' and mac_rx_frame_sync = '1') then
                timestamp_sec_latch <= wallclock_seconds_i;
                timestamp_nsec_latch <= wallclock_nanoseconds_i;
            end if;

            if (eth_parser_sync_prev = '0' and eth_parser_sync_sync = '1') then
                timestamp_seconds_o <= timestamp_sec_latch;
                timestamp_nanoseconds_o <= timestamp_nsec_latch;
            end if;


        end if;
    end process;
    
end Behavioral;