-- ####################################################################
-- # LiteX Ethernet Buffer Bridge
-- #
-- # Bridges the LiteX EthPacketBuffer FPGA-side signals to the
-- # Ethernet MAC TX/RX interface.
-- #
-- # The EthPacketBuffer dual-port RAMs have their FPGA-side ports
-- # directly clocked by mac_rx_clock (RX) and mac_tx_clock (TX).
-- # This module therefore operates entirely in the MAC clock domains
-- # with no CDC or intermediate buffers needed for packet data.
-- #
-- # TX: SoC writes packet, sets tx_len, pulses eth_tx_request.
-- #     TX process reads buffer directly on mac_tx_clock and feeds MAC.
-- #
-- # RX: MAC receives frame.  RX process writes bytes directly into
-- #     buffer on mac_rx_clock, then asserts rx_valid + rx_len.
-- ####################################################################
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.ethernet_types.all;

entity litex_eth_buffer_bridge is
  port (
    -- ================================================================
    -- LiteX EthPacketBuffer FPGA-side signals
    -- RX port clocked by mac_rx_clock, TX port clocked by mac_tx_clock
    -- ================================================================

    -- RX buffer: this module writes received frames
    buf_rx_data_o   : out std_ulogic_vector(7 downto 0);
    buf_rx_addr_o   : out unsigned(10 downto 0);
    buf_rx_we_o     : out std_ulogic;
    buf_rx_len_o    : out unsigned(10 downto 0);
    buf_rx_valid_o  : out std_ulogic;
    buf_rx_ack_i    : in  std_ulogic;

    -- TX buffer: this module reads packet data

    buf_tx_addr_o   : out unsigned(10 downto 0);
    buf_tx_len_i    : in  unsigned(10 downto 0);
    buf_tx_dat_i    : in STD_ULOGIC_VECTOR(7 downto 0);
    

    -- ================================================================
    -- Control signals from/to SoC pads (SoC sys_clk domain)
    -- CDC into MAC domains where needed.
    -- ================================================================
    eth_tx_request_i  : in  std_ulogic;   -- level from SoC: send TX
    eth_tx_done_o     : out std_ulogic;   -- level: TX completed
    eth_rx_overflow_o : out std_ulogic;   -- level: RX frame dropped

    -- ================================================================
    -- MAC TX interface (mac_tx_clock_i domain)
    -- ================================================================
    mac_tx_clock_i     : in  std_ulogic;
    mac_tx_reset_i     : in  std_ulogic;
    mac_tx_enable_o    : out std_ulogic;

    mac_tx_byte_sent_i : in  std_ulogic;
    mac_tx_busy_i      : in  std_ulogic;
    mac_tx_dat_o    : out STD_ULOGIC_VECTOR(7 downto 0);

    -- TX arbitration
    tx_allow_req_o     : out std_ulogic;
    tx_allow_i         : in  std_ulogic;

    -- ================================================================
    -- MAC RX interface (mac_rx_clock_i domain)
    -- ================================================================
    mac_rx_clock_i    : in  std_ulogic;
    mac_rx_reset_i    : in  std_ulogic;
    mac_rx_frame_i    : in  std_ulogic;
    mac_rx_data_i     : in  t_ethernet_data;
    mac_rx_byte_rcv_i : in  std_ulogic;
    mac_rx_error_i    : in  std_ulogic
  );
end entity;

architecture rtl of litex_eth_buffer_bridge is

  constant MAX_FRAME_LEN : natural := 1518;

  -- ================================================================
  -- TX signals (mac_tx_clock domain)
  -- ================================================================

  -- CDC for eth_tx_request (SoC sys_clk → mac_tx_clock)
  signal tx_req_meta       : std_ulogic := '0';
  signal tx_req_sync       : std_ulogic := '0';
  signal tx_req_sync_d     : std_ulogic := '0';

  signal tx_done_reg       : std_ulogic := '0';
  

  type t_tx_sm is (TX_IDLE, TX_WAIT_ALLOW, TX_PRIME, TX_TRANSMIT, TX_END);
  signal sm_tx : t_tx_sm := TX_IDLE;


  -- ================================================================
  -- RX signals (mac_rx_clock domain)
  -- ================================================================

  signal rx_wr_addr    : unsigned(10 downto 0) := (others => '0');
  signal rx_frame_d    : std_ulogic := '0';
  signal rx_error_flag : std_ulogic := '0';
  signal rx_valid_reg  : std_ulogic := '0';
  signal rx_overflow_reg : std_ulogic := '0';

  -- CDC for buf_rx_ack (SoC sys_clk → mac_rx_clock)
  signal rx_ack_meta : std_ulogic := '0';
  signal rx_ack_sync : std_ulogic := '0';
  signal rx_ack_sync_d : std_ulogic := '0';

begin

  -- ================================================================
  -- Output assignments
  -- ================================================================
  eth_tx_done_o     <= tx_done_reg;
  eth_rx_overflow_o <= rx_overflow_reg;
  buf_rx_valid_o    <= rx_valid_reg;

  -- ================================================================
  -- TX process (mac_tx_clock domain)
  --
  -- Reads directly from LiteX TX buffer via buf_tx_addr_o/buf_tx_data_i.
  -- The buffer's FPGA read port is clocked by mac_tx_clock inside the
  -- SoC, so this is a same-clock-domain access.
  -- ================================================================
  p_tx : process(mac_tx_clock_i, mac_tx_reset_i)
  variable tx_preamble_bytes : UNSIGNED(4 downto 0);
  begin
    if mac_tx_reset_i = '1' then
      tx_req_meta       <= '0';
      tx_req_sync       <= '0';
      tx_req_sync_d     <= '0';
      tx_done_reg       <= '0';
      
      buf_tx_addr_o     <= (others => '0');
      mac_tx_enable_o   <= '0';
      tx_allow_req_o    <= '0';
      sm_tx             <= TX_IDLE;
      tx_preamble_bytes := (others => '0');
      mac_tx_dat_o <= (others => '0');
    elsif rising_edge(mac_tx_clock_i) then
      -- CDC: sync eth_tx_request into mac_tx domain
      tx_req_meta   <= eth_tx_request_i;
      tx_req_sync   <= tx_req_meta;
      tx_req_sync_d <= tx_req_sync;
      -- Only drive data during active TX, otherwise hold at 0x00
      -- to avoid OR-mux corruption in ethernet_packet_aggregator
      case sm_tx is
        when TX_IDLE =>
          mac_tx_dat_o <= (others => '0');
          mac_tx_enable_o <= '0';
          tx_allow_req_o  <= '0';
          -- Detect rising edge of tx_request
          if tx_req_sync = '1' and tx_req_sync_d = '0' then
            tx_done_reg    <= '0';
            tx_allow_req_o <= '1';
            sm_tx          <= TX_WAIT_ALLOW;
            buf_tx_addr_o      <= (others => '0');
          end if;

        when TX_WAIT_ALLOW =>
          mac_tx_dat_o <= (others => '0');
          if tx_allow_i = '1' then
            sm_tx              <= TX_PRIME;
          end if;
        when TX_PRIME =>
              mac_tx_dat_o <= buf_tx_dat_i;
              mac_tx_enable_o <= '1';
              if (mac_tx_busy_i = '1') then
                tx_preamble_bytes := tx_preamble_bytes + 1;
                if (tx_preamble_bytes >= 6) then
                  buf_tx_addr_o <= buf_tx_addr_o + 1;
                  
                  if (tx_preamble_bytes = 7) then
                  sm_tx              <= TX_TRANSMIT;
                  tx_preamble_bytes := (others => '0');
                  end if;
                end if;
              end if;

        when TX_TRANSMIT =>
          mac_tx_dat_o <= buf_tx_dat_i;
          mac_tx_enable_o <= '1';
          if mac_tx_byte_sent_i = '1' then
            if buf_tx_addr_o = buf_tx_len_i then
              sm_tx <= TX_END;
            else
              buf_tx_addr_o <= buf_tx_addr_o + 1;
            end if;
          end if;

        when TX_END =>
          mac_tx_dat_o <= (others => '0');
          mac_tx_enable_o <= '0';
          tx_allow_req_o  <= '0';
          tx_done_reg     <= '1';
          sm_tx           <= TX_IDLE;
      end case;
    end if;
  end process;

  -- ================================================================
  -- RX process (mac_rx_clock domain)
  --
  -- Writes directly into LiteX RX buffer via buf_rx_addr_o/buf_rx_data_o.
  -- The buffer's FPGA write port is clocked by mac_rx_clock inside the
  -- SoC, so this is a same-clock-domain access.
  -- ================================================================
  p_rx : process(mac_rx_clock_i, mac_rx_reset_i)
  begin
    if mac_rx_reset_i = '1' then
      rx_ack_meta    <= '0';
      rx_ack_sync    <= '0';
      rx_ack_sync_d  <= '0';
      buf_rx_data_o  <= (others => '0');
      buf_rx_addr_o  <= (others => '0');
      buf_rx_we_o    <= '0';
      buf_rx_len_o   <= (others => '0');
      rx_valid_reg   <= '0';
      rx_overflow_reg <= '0';
      rx_wr_addr     <= (others => '0');
      rx_frame_d     <= '0';
      rx_error_flag  <= '0';
    elsif rising_edge(mac_rx_clock_i) then
      buf_rx_we_o <= '0';

      -- CDC: sync buf_rx_ack into mac_rx domain
      rx_ack_meta   <= buf_rx_ack_i;
      rx_ack_sync   <= rx_ack_meta;
      rx_ack_sync_d <= rx_ack_sync;

      -- SoC acknowledged: release buffer
      if rx_ack_sync = '1' and rx_ack_sync_d = '0' then
        rx_valid_reg    <= '0';
        rx_overflow_reg <= '0';
      end if;

      rx_frame_d <= mac_rx_frame_i;

      -- Frame start: reset state
      if mac_rx_frame_i = '1' and rx_frame_d = '0' then
        rx_wr_addr    <= (others => '0');
        rx_error_flag <= '0';
        -- If previous frame not consumed, this is an overflow
        if rx_valid_reg = '1' then
          rx_overflow_reg <= '1';
        end if;
      end if;

      -- Receive bytes: write directly into buffer
      -- NOTE: when mac_rx_byte_rcv_i is asserted on the same cycle as
      -- frame start (rx_frame_d='0'), we must use address 0 explicitly
      -- because the frame-start reset above would be overridden by the
      -- increment (VHDL "last assignment wins").
      if mac_rx_frame_i = '1' and mac_rx_byte_rcv_i = '1' then
        if rx_wr_addr < MAX_FRAME_LEN and rx_valid_reg = '0' then
          buf_rx_data_o <= mac_rx_data_i;
          if rx_frame_d = '0' then
            -- First byte of new frame: write to address 0
            buf_rx_addr_o <= (others => '0');
            buf_rx_we_o   <= '1';
            rx_wr_addr    <= to_unsigned(1, rx_wr_addr'length);
          else
            buf_rx_addr_o <= rx_wr_addr;
            buf_rx_we_o   <= '1';
            rx_wr_addr    <= rx_wr_addr + 1;
          end if;
        else
          rx_error_flag <= '1';
        end if;
      end if;

      -- Track MAC error
      if mac_rx_frame_i = '1' and mac_rx_error_i = '1' then
        rx_error_flag <= '1';
      end if;

      -- Frame end: assert valid if no error
      if mac_rx_frame_i = '0' and rx_frame_d = '1' then
        if rx_error_flag = '0' and rx_wr_addr /= 0 then
          buf_rx_len_o <= rx_wr_addr;
          rx_valid_reg <= '1';
        end if;
      end if;
    end if;
  end process;

end architecture;
