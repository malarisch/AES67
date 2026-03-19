-- ####################################################################
-- # LiteX Ethernet Buffer Bridge
-- #
-- # Bridges the LiteX EthPacketBuffer FPGA-side signals to the
-- # Ethernet MAC TX/RX interface.
-- #
-- # TX: SoC writes packet, sets tx_len, pulses eth_tx_request.
-- #     TX process reads buffer directly on mac_tx_clock and feeds MAC.
-- #
-- # RX: ethernet_packet_parser asserts parse_mcu_packet when a non-RTP
-- #     frame is ready in eth_ram.  This module copies the frame from
-- #     eth_ram into the LiteX RX buffer, then asserts rx_valid + rx_len.
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
    -- RX from eth_ram / packet parser (mac_rx_clock_i domain)
    -- ================================================================
    mac_rx_clock_i      : in  std_ulogic;
    mac_rx_reset_i      : in  std_ulogic;
    parse_mcu_packet_i  : in  std_ulogic;   -- 1-cycle pulse: non-RTP frame ready
    pkt_len_i           : in  unsigned(10 downto 0);  -- frame length from ethernet_receive
    eth_ram_data_i      : in  std_ulogic_vector(7 downto 0);  -- read data from eth_ram read port
    eth_ram_addr_o      : out unsigned(10 downto 0);  -- read address to eth_ram

    mcu_clk_i         : in STD_ULOGIC
  );
end entity;

architecture rtl of litex_eth_buffer_bridge is

  -- ================================================================
  -- TX signals (mac_tx_clock domain)
  -- ================================================================

  -- CDC for eth_tx_request (SoC sys_clk → mac_tx_clock)
  signal tx_req_meta       : std_ulogic := '0';
  signal tx_req_sync       : std_ulogic := '0';
  signal tx_req_sync_d     : std_ulogic := '0';

  signal tx_done_reg       : std_ulogic := '0';
  signal tx_done_reg_1cdc  : std_ulogic := '0';
  signal tx_done_reg_2cdc  : std_ulogic := '0';


  type t_tx_sm is (TX_IDLE, TX_WAIT_ALLOW, TX_PRIME, TX_TRANSMIT, TX_END);
  signal sm_tx : t_tx_sm := TX_IDLE;


  -- ================================================================
  -- RX signals (mac_rx_clock domain)
  -- ================================================================

  type t_rx_sm is (RX_IDLE, RX_COPY, RX_DONE);
  signal sm_rx : t_rx_sm := RX_IDLE;

  signal rx_copy_addr    : unsigned(10 downto 0) := (others => '0');
  signal rx_copy_len     : unsigned(10 downto 0) := (others => '0');
  signal rx_valid_reg    : std_ulogic := '0';
  signal rx_valid_reg_1cdc  : std_ulogic := '0';
  signal rx_valid_reg_2cdc  : std_ulogic := '0';
  signal rx_overflow_reg : std_ulogic := '0';
  signal rx_overflow_reg_1cdc : std_ulogic := '0';
  signal rx_overflow_reg_2cdc : std_ulogic := '0';

  signal parse_mcu_d : std_ulogic := '0';  -- delayed for rising-edge detection

  -- CDC for buf_rx_ack (SoC sys_clk → mac_rx_clock)
  signal rx_ack_meta : std_ulogic := '0';
  signal rx_ack_sync : std_ulogic := '0';
  signal rx_ack_sync_d : std_ulogic := '0';

begin

  -- ================================================================
  -- Output assignments & cdc
  -- ================================================================

  mcu_cdc : process (mcu_clk_i)
  begin
    if (rising_edge(mcu_clk_i)) then
      rx_valid_reg_1cdc <= rx_valid_reg;
      rx_valid_reg_2cdc <= rx_valid_reg_1cdc;
      buf_rx_valid_o    <= rx_valid_reg_2cdc;

      rx_overflow_reg_1cdc <= rx_overflow_reg;
      rx_overflow_reg_2cdc <= rx_overflow_reg_1cdc;
      eth_rx_overflow_o <= rx_overflow_reg_2cdc;

      tx_done_reg_1cdc <= tx_done_reg;
      tx_done_reg_2cdc <= tx_done_reg_1cdc;
      eth_tx_done_o     <= tx_done_reg_2cdc;
    end if;
  end process;

  -- ================================================================
  -- TX process (mac_tx_clock domain)
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
  -- Copies frames from eth_ram into the LiteX RX buffer when
  -- ethernet_packet_parser signals a non-RTP packet is ready.
  --
  -- eth_ram read ports are combinatorial (async), so the data for
  -- a given address is available in the same cycle.  We set up
  -- address 0 in the IDLE→COPY transition, then copy one byte
  -- per clock cycle.
  -- ================================================================
  p_rx : process(mac_rx_clock_i, mac_rx_reset_i)
  begin
    if mac_rx_reset_i = '1' then
      rx_ack_meta      <= '0';
      rx_ack_sync      <= '0';
      rx_ack_sync_d    <= '0';
      buf_rx_data_o    <= (others => '0');
      buf_rx_addr_o    <= (others => '0');
      buf_rx_we_o      <= '0';
      buf_rx_len_o     <= (others => '0');
      rx_valid_reg     <= '0';
      rx_overflow_reg  <= '0';
      rx_copy_addr     <= (others => '0');
      rx_copy_len      <= (others => '0');
      eth_ram_addr_o   <= (others => '0');
      parse_mcu_d      <= '0';
      sm_rx            <= RX_IDLE;
    elsif rising_edge(mac_rx_clock_i) then
      buf_rx_we_o <= '0';

      -- Edge detection for parse_mcu_packet (it stays high as a level
      -- until the next packet arrives, so we must detect the rising edge)
      parse_mcu_d <= parse_mcu_packet_i;

      -- CDC: sync buf_rx_ack into mac_rx domain
      rx_ack_meta   <= buf_rx_ack_i;
      rx_ack_sync   <= rx_ack_meta;
      rx_ack_sync_d <= rx_ack_sync;

      -- SoC acknowledged: release buffer
      if rx_ack_sync = '1' and rx_ack_sync_d = '0' then
        rx_valid_reg    <= '0';
        rx_overflow_reg <= '0';
      end if;

      case sm_rx is
        when RX_IDLE =>
          if parse_mcu_packet_i = '1' and parse_mcu_d = '0' then
            if rx_valid_reg = '1' then
              -- Previous frame not yet consumed by SoC -> overflow
              rx_overflow_reg <= '1';
            else
              -- Latch frame length, start copying
              rx_copy_len    <= pkt_len_i;
              rx_copy_addr   <= (others => '0');
              eth_ram_addr_o <= (others => '0');  -- present address 0 to async read port
              sm_rx          <= RX_COPY;
            end if;
          end if;

        when RX_COPY =>
          -- eth_ram read port is combinatorial (async):
          -- eth_ram_data_i = ram[eth_ram_addr_o] in the SAME cycle.
          -- So right now eth_ram_data_i holds data for rx_copy_addr.
          buf_rx_data_o  <= eth_ram_data_i;
          buf_rx_addr_o  <= rx_copy_addr;
          buf_rx_we_o    <= '1';

          if rx_copy_addr + 1 >= rx_copy_len then
            -- This was the last byte
            sm_rx <= RX_DONE;
          else
            -- Advance to next byte
            rx_copy_addr   <= rx_copy_addr + 1;
            eth_ram_addr_o <= rx_copy_addr + 1;
          end if;

        when RX_DONE =>
          buf_rx_len_o <= rx_copy_len;
          rx_valid_reg <= '1';
          sm_rx        <= RX_IDLE;

      end case;
    end if;
  end process;

end architecture;
