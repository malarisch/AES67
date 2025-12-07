library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- ####################################################################
-- # PTP Timestamper Module
-- # Captures timestamps for transmitted and received Ethernet packets
-- # for use with PTPv2 (IEEE 1588) protocol
-- ####################################################################

entity ptp_timestamper is
  port (
    -- System Clock
    clk_i       : in  std_ulogic;
    rst_i       : in  std_ulogic;
    
    -- Free-running timestamp counter input (from PTP clock)
    -- 64-bit nanosecond counter
    timestamp_ns_i : in unsigned(63 downto 0);
    
    -- TX Interface (MAC TX Domain)
    tx_clk_i        : in  std_ulogic;
    tx_rst_i        : in  std_ulogic;
    tx_enable_i     : in  std_ulogic;  -- TX enable signal from MAC
    tx_byte_sent_i  : in  std_ulogic;  -- Byte sent signal from MAC
    
    -- TX Timestamp Output (synchronized to clk_i)
    tx_timestamp_o     : out unsigned(63 downto 0);
    tx_timestamp_valid_o : out std_ulogic;
    tx_timestamp_ack_i : in  std_ulogic;  -- Acknowledge from reader
    
    -- RX Interface (MAC RX Domain)
    rx_clk_i        : in  std_ulogic;
    rx_rst_i        : in  std_ulogic;
    rx_frame_i      : in  std_ulogic;  -- RX frame active signal
    rx_byte_rcv_i   : in  std_ulogic;  -- Byte received signal
    
    -- RX Timestamp Output (synchronized to clk_i)
    rx_timestamp_o     : out unsigned(63 downto 0);
    rx_timestamp_valid_o : out std_ulogic;
    rx_timestamp_ack_i : in  std_ulogic   -- Acknowledge from reader
  );
end entity ptp_timestamper;

architecture rtl of ptp_timestamper is

  -- TX timestamp capture in TX clock domain
  signal tx_timestamp_cap : unsigned(63 downto 0) := (others => '0');
  signal tx_frame_start   : std_ulogic := '0';
  signal tx_enable_d      : std_ulogic := '0';
  signal tx_timestamp_valid_tx : std_ulogic := '0';
  
  -- RX timestamp capture in RX clock domain
  signal rx_timestamp_cap : unsigned(63 downto 0) := (others => '0');
  signal rx_frame_start   : std_ulogic := '0';
  signal rx_frame_d       : std_ulogic := '0';
  signal rx_timestamp_valid_rx : std_ulogic := '0';
  
  -- Synchronization from TX domain to system clock domain
  signal tx_timestamp_sync_1 : unsigned(63 downto 0) := (others => '0');
  signal tx_timestamp_sync_2 : unsigned(63 downto 0) := (others => '0');
  signal tx_valid_sync_1     : std_ulogic := '0';
  signal tx_valid_sync_2     : std_ulogic := '0';
  signal tx_valid_sys        : std_ulogic := '0';
  signal tx_ack_sync_1       : std_ulogic := '0';
  signal tx_ack_sync_2       : std_ulogic := '0';
  
  -- Synchronization from RX domain to system clock domain
  signal rx_timestamp_sync_1 : unsigned(63 downto 0) := (others => '0');
  signal rx_timestamp_sync_2 : unsigned(63 downto 0) := (others => '0');
  signal rx_valid_sync_1     : std_ulogic := '0';
  signal rx_valid_sync_2     : std_ulogic := '0';
  signal rx_valid_sys        : std_ulogic := '0';
  signal rx_ack_sync_1       : std_ulogic := '0';
  signal rx_ack_sync_2       : std_ulogic := '0';
  
  -- Timestamp counter synchronized to TX/RX domains
  signal timestamp_ns_tx_1 : unsigned(63 downto 0) := (others => '0');
  signal timestamp_ns_tx_2 : unsigned(63 downto 0) := (others => '0');
  signal timestamp_ns_rx_1 : unsigned(63 downto 0) := (others => '0');
  signal timestamp_ns_rx_2 : unsigned(63 downto 0) := (others => '0');

begin

  --------------------------------------------------------------------
  -- TX Timestamp Capture (in TX clock domain)
  -- Captures timestamp at the start of frame transmission
  --------------------------------------------------------------------
  process(tx_clk_i, tx_rst_i)
  begin
    if tx_rst_i = '1' then
      tx_enable_d <= '0';
      tx_frame_start <= '0';
      tx_timestamp_cap <= (others => '0');
      tx_timestamp_valid_tx <= '0';
      timestamp_ns_tx_1 <= (others => '0');
      timestamp_ns_tx_2 <= (others => '0');
      tx_ack_sync_1 <= '0';
      tx_ack_sync_2 <= '0';
    elsif rising_edge(tx_clk_i) then
      -- Synchronize timestamp counter to TX domain (2-stage)
      timestamp_ns_tx_1 <= timestamp_ns_i;
      timestamp_ns_tx_2 <= timestamp_ns_tx_1;
      
      -- Synchronize acknowledge signal from system domain
      tx_ack_sync_1 <= tx_timestamp_ack_i;
      tx_ack_sync_2 <= tx_ack_sync_1;
      
      -- Detect rising edge of tx_enable (start of frame)
      tx_enable_d <= tx_enable_i;
      tx_frame_start <= tx_enable_i and not tx_enable_d;
      
      -- Capture timestamp on frame start
      if tx_frame_start = '1' then
        tx_timestamp_cap <= timestamp_ns_tx_2;
        tx_timestamp_valid_tx <= '1';
      elsif tx_ack_sync_2 = '1' then
        -- Clear valid flag when acknowledged
        tx_timestamp_valid_tx <= '0';
      end if;
    end if;
  end process;
  
  --------------------------------------------------------------------
  -- RX Timestamp Capture (in RX clock domain)
  -- Captures timestamp at the start of frame reception
  --------------------------------------------------------------------
  process(rx_clk_i, rx_rst_i)
  begin
    if rx_rst_i = '1' then
      rx_frame_d <= '0';
      rx_frame_start <= '0';
      rx_timestamp_cap <= (others => '0');
      rx_timestamp_valid_rx <= '0';
      timestamp_ns_rx_1 <= (others => '0');
      timestamp_ns_rx_2 <= (others => '0');
      rx_ack_sync_1 <= '0';
      rx_ack_sync_2 <= '0';
    elsif rising_edge(rx_clk_i) then
      -- Synchronize timestamp counter to RX domain (2-stage)
      timestamp_ns_rx_1 <= timestamp_ns_i;
      timestamp_ns_rx_2 <= timestamp_ns_rx_1;
      
      -- Synchronize acknowledge signal from system domain
      rx_ack_sync_1 <= rx_timestamp_ack_i;
      rx_ack_sync_2 <= rx_ack_sync_1;
      
      -- Detect rising edge of rx_frame (start of frame)
      rx_frame_d <= rx_frame_i;
      rx_frame_start <= rx_frame_i and not rx_frame_d;
      
      -- Capture timestamp on frame start
      if rx_frame_start = '1' then
        rx_timestamp_cap <= timestamp_ns_rx_2;
        rx_timestamp_valid_rx <= '1';
      elsif rx_ack_sync_2 = '1' then
        -- Clear valid flag when acknowledged
        rx_timestamp_valid_rx <= '0';
      end if;
    end if;
  end process;
  
  --------------------------------------------------------------------
  -- TX Timestamp Synchronization to System Clock Domain
  --------------------------------------------------------------------
  process(clk_i, rst_i)
  begin
    if rst_i = '1' then
      tx_timestamp_sync_1 <= (others => '0');
      tx_timestamp_sync_2 <= (others => '0');
      tx_valid_sync_1 <= '0';
      tx_valid_sync_2 <= '0';
      tx_valid_sys <= '0';
      tx_timestamp_o <= (others => '0');
      tx_timestamp_valid_o <= '0';
    elsif rising_edge(clk_i) then
      -- Two-stage synchronization for timestamp
      tx_timestamp_sync_1 <= tx_timestamp_cap;
      tx_timestamp_sync_2 <= tx_timestamp_sync_1;
      
      -- Two-stage synchronization for valid flag
      tx_valid_sync_1 <= tx_timestamp_valid_tx;
      tx_valid_sync_2 <= tx_valid_sync_1;
      
      -- Latch timestamp and valid when valid signal rises
      if (tx_valid_sync_2 = '1') and (tx_valid_sys = '0') then
        tx_timestamp_o <= tx_timestamp_sync_2;
        tx_timestamp_valid_o <= '1';
        tx_valid_sys <= '1';
      elsif tx_timestamp_ack_i = '1' then
        tx_timestamp_valid_o <= '0';
        tx_valid_sys <= '0';
      end if;
    end if;
  end process;
  
  --------------------------------------------------------------------
  -- RX Timestamp Synchronization to System Clock Domain
  --------------------------------------------------------------------
  process(clk_i, rst_i)
  begin
    if rst_i = '1' then
      rx_timestamp_sync_1 <= (others => '0');
      rx_timestamp_sync_2 <= (others => '0');
      rx_valid_sync_1 <= '0';
      rx_valid_sync_2 <= '0';
      rx_valid_sys <= '0';
      rx_timestamp_o <= (others => '0');
      rx_timestamp_valid_o <= '0';
    elsif rising_edge(clk_i) then
      -- Two-stage synchronization for timestamp
      rx_timestamp_sync_1 <= rx_timestamp_cap;
      rx_timestamp_sync_2 <= rx_timestamp_sync_1;
      
      -- Two-stage synchronization for valid flag
      rx_valid_sync_1 <= rx_timestamp_valid_rx;
      rx_valid_sync_2 <= rx_valid_sync_1;
      
      -- Latch timestamp and valid when valid signal rises
      if (rx_valid_sync_2 = '1') and (rx_valid_sys = '0') then
        rx_timestamp_o <= rx_timestamp_sync_2;
        rx_timestamp_valid_o <= '1';
        rx_valid_sys <= '1';
      elsif rx_timestamp_ack_i = '1' then
        rx_timestamp_valid_o <= '0';
        rx_valid_sys <= '0';
      end if;
    end if;
  end process;

end architecture rtl;
