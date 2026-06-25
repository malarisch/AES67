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

use work.wallclock_signals_pkg.all;

entity litex_eth_buffer_bridge is
  generic (
    
    ADD_RX_TIMESTAMP : boolean := false;
    RX_NUM_SLOTS : integer := 2
  );
  port (
    -- ================================================================
    -- LiteX EthPacketBuffer FPGA-side signals
    -- RX port clocked by mac_rx_clock, TX port clocked by mac_tx_clock
    -- ================================================================

    -- RX buffer: this module writes received frames
    buf_rx_data_o   : out std_logic_vector(7 downto 0);
    buf_rx_addr_o   : out std_logic_vector(10 downto 0);
    buf_rx_we_o     : out std_logic;
    buf_rx_len_o    : out std_logic_vector(10 downto 0);
    buf_rx_valid_o  : out std_logic;
    buf_rx_ack_i    : in  std_logic;
    timestamps_i : in t_eth_timestamps;
    tx_timestamp_o : out t_eth_timestamp;
    -- TX buffer: this module reads packet data

    buf_tx_addr_o   : out std_logic_vector(10 downto 0);
    buf_tx_len_i    : in  std_logic_vector(10 downto 0);
    buf_tx_dat_i    : in  std_logic_vector(7 downto 0);


    -- ================================================================
    -- Control signals from/to SoC pads (SoC sys_clk domain)
    -- CDC into MAC domains where needed.
    -- ================================================================
    eth_tx_request_i  : in  std_logic;   -- level from SoC: send TX
    eth_tx_done_o     : out std_logic;   -- level: TX completed
    eth_rx_overflow_o : out std_logic;   -- level: RX frame dropped

    -- ================================================================
    -- MAC TX interface (mac_tx_clock_i domain)
    -- ================================================================
    mac_tx_clock_i     : in  std_logic;
    mac_tx_reset_i     : in  std_logic;
    mac_tx_enable_o    : out std_logic;

    mac_tx_byte_sent_i : in  std_logic;
    mac_tx_busy_i      : in  std_logic;
    mac_tx_dat_o    : out std_logic_vector(7 downto 0);
    mac_start_prefetch_i : in std_logic;
    mac_speed_in        : in std_logic_vector(1 downto 0);

    -- TX arbitration
    tx_allow_req_o     : out std_logic;
    tx_allow_i         : in  std_logic;

    -- ================================================================
    -- RX from eth_ram / packet parser (mac_rx_clock_i domain)
    -- ================================================================
    mac_rx_clock_i      : in  std_logic;
    mac_rx_reset_i      : in  std_logic;
    parse_mcu_packet_tog_i  : in  std_logic;  -- toggles when packet ready
    pkt_len_i           : in  std_logic_vector(10 downto 0);  -- frame length from ethernet_receive
    eth_ram_data_i      : in  std_logic_vector(7 downto 0);  -- read data from eth_ram read port
    eth_ram_addr_o      : out std_logic_vector(10 downto 0);  -- read address to eth_ram

    mcu_clk_i         : in std_logic;
    sys_clk_i         : in std_logic;
    packet_length_valid_i : in std_logic
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


  type t_tx_sm is (TX_IDLE, TX_WAIT_ALLOW, TX_PRIME, TX_PRIME_PREFETCH, TX_TRANSMIT, TX_END);
  signal sm_tx : t_tx_sm := TX_IDLE;


  -- ================================================================
  -- RX signals (mac_rx_clock domain)
  -- ================================================================

  -- Two independent state machines share the mac_rx_clock domain (no CDC):
  --   FILL  : eth_ram -> internal frame FIFO  (runs at each parser toggle)
  --   DRAIN : frame FIFO -> LiteX single buffer (runs as the SoC acks)
  type t_fill_sm is (FILL_IDLE, FILL_WAIT, FILL_COPY, FILL_TS_SEC, FILL_TS_NS, FILL_COMMIT);
  signal sm_fill : t_fill_sm := FILL_IDLE;
  type t_drain_sm is (DR_IDLE, DR_PRIME, DR_COPY, DR_SET_VALID, DR_WAIT_ACK);
  signal sm_drain : t_drain_sm := DR_IDLE;

  signal rx_copy_addr    : unsigned(10 downto 0) := (others => '0');
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
  signal buf_tx_addr   : unsigned(10 downto 0);
  -- Internal unsigned working copies of the address/length outputs; the entity
  -- ports are std_logic_vector and driven from these via concurrent casts.
  signal eth_ram_addr  : unsigned(10 downto 0);
  signal buf_rx_addr   : unsigned(10 downto 0);
  signal buf_rx_len    : unsigned(10 downto 0);
  signal packet_length_latch : UNSIGNED(10 downto 0);
  signal packet_lenght_valid_latch : std_ulogic;
  signal packet_length_valid_counter : unsigned(3 downto 0);
  signal tx_zsof : STD_LOGIC := '0';
  signal ts_write_index : integer range 0 to 3;

  -- RX hardware-timestamp trailer (only when ADD_RX_TIMESTAMP).
  constant TS_TRAILER_LEN : natural := 5;  -- 1 byte seconds + 4 bytes nanoseconds
  -- Latched at frame detection so the 5-cycle trailer write can't be torn by a
  -- newer frame's timestamp arriving mid-write.
  signal rx_ts_sec_latch : unsigned(3 downto 0)  := (others => '0');
  signal rx_ts_ns_latch  : unsigned(29 downto 0) := (others => '0');

  -- Internal RX frame FIFO (mac_rx_clock domain only).
  constant SLOT_SIZE : integer := 2048;  -- bytes per slot (>= 1518 + trailer)
  type t_fifo_ram is array(0 to RX_NUM_SLOTS * SLOT_SIZE - 1) of std_logic_vector(7 downto 0);
  signal fifo_ram   : t_fifo_ram;
  signal fifo_raddr : integer range 0 to RX_NUM_SLOTS * SLOT_SIZE - 1 := 0;
  signal fifo_rdata : std_logic_vector(7 downto 0) := (others => '0');
  type t_len_arr is array(0 to RX_NUM_SLOTS - 1) of unsigned(10 downto 0);
  signal slot_len   : t_len_arr := (others => (others => '0'));
  signal wr_slot    : integer range 0 to RX_NUM_SLOTS - 1 := 0;
  signal rd_slot    : integer range 0 to RX_NUM_SLOTS - 1 := 0;
  signal fifo_count : integer range 0 to RX_NUM_SLOTS := 0;
  signal drain_addr : unsigned(10 downto 0) := (others => '0');
  signal drain_len  : unsigned(10 downto 0) := (others => '0');
begin

  -- ================================================================
  -- Output assignments & cdc
  -- ================================================================
  buf_tx_addr_o  <= std_logic_vector(buf_tx_addr);
  eth_ram_addr_o <= std_logic_vector(eth_ram_addr);
  buf_rx_addr_o  <= std_logic_vector(buf_rx_addr);
  buf_rx_len_o   <= std_logic_vector(buf_rx_len);
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

      buf_tx_addr     <= (others => '0');
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

      tx_zsof <= mac_start_prefetch_i;
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
            buf_tx_addr      <= (others => '0');
          end if;

        when TX_WAIT_ALLOW =>
          mac_tx_dat_o <= (others => '0');
          if tx_allow_i = '1' then
              sm_tx              <= TX_PRIME;
          end if;
        when TX_PRIME =>
              mac_tx_dat_o <= buf_tx_dat_i; -- output byte 0
              mac_tx_enable_o <= '1';
              if (mac_speed_in = b"10") then
                -- gigabit
                sm_tx <= TX_PRIME_PREFETCH;
              else
                sm_tx <= TX_TRANSMIT;
                buf_tx_addr <= buf_tx_addr + 1;

              end if;
        when TX_PRIME_PREFETCH => 
            if ((mac_start_prefetch_i = '1' and tx_zsof = '0') or (mac_start_prefetch_i = '0' and tx_zsof = '1')) then
                mac_tx_dat_o <= buf_tx_dat_i; -- output byte 0
                buf_tx_addr <= buf_tx_addr + 1;
                if (mac_start_prefetch_i = '0' and tx_zsof = '1') then
                sm_tx <= TX_TRANSMIT;

                end if;
              end if;

        when TX_TRANSMIT =>
          
          if mac_tx_byte_sent_i = '1' then
            mac_tx_dat_o <= buf_tx_dat_i;
            
            if buf_tx_addr = unsigned(buf_tx_len_i) then
              sm_tx <= TX_END;
              tx_timestamp_o <= timestamps_i.tx;
            else
              buf_tx_addr <= buf_tx_addr + 1;
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
  -- RX process (sys_clk domain)
  --
  -- Copies frames from eth_ram into the LiteX RX buffer when
  -- packet parser signals a non-RTP packet is ready.
  --
  -- eth_ram read port is synchronous (1 clock latency):
  -- address presented in cycle N, data available in cycle N+1.
  -- We use RX_WAIT to absorb that latency before entering RX_COPY.
  -- ================================================================
  p_rx : process(mac_rx_clock_i, mac_rx_reset_i)
    variable v_commit : boolean;   -- a frame was committed to the FIFO this cycle
    variable v_pop    : boolean;   -- a frame was released from the FIFO this cycle
  begin
    if mac_rx_reset_i = '1' then
      rx_ack_meta      <= '0';
      rx_ack_sync      <= '0';
      rx_ack_sync_d    <= '0';
      buf_rx_data_o    <= (others => '0');
      buf_rx_addr      <= (others => '0');
      buf_rx_we_o      <= '0';
      buf_rx_len       <= (others => '0');
      rx_valid_reg     <= '0';
      rx_overflow_reg  <= '0';
      rx_copy_addr     <= (others => '0');
      eth_ram_addr     <= (others => '0');
      parse_mcu_d      <= '0';
      sm_fill          <= FILL_IDLE;
      sm_drain         <= DR_IDLE;
      wr_slot          <= 0;
      rd_slot          <= 0;
      fifo_count       <= 0;
      fifo_raddr       <= 0;
      drain_addr       <= (others => '0');
      drain_len        <= (others => '0');
      packet_lenght_valid_latch <= '0';
      packet_length_latch <= (others => '0');
    elsif rising_edge(mac_rx_clock_i) then
      v_commit := false;
      v_pop    := false;
      buf_rx_we_o <= '0';

      -- Edge detection for parse_mcu_packet toggle
      parse_mcu_d <= parse_mcu_packet_tog_i;

      -- CDC: sync buf_rx_ack into this domain
      rx_ack_meta   <= buf_rx_ack_i;
      rx_ack_sync   <= rx_ack_meta;
      rx_ack_sync_d <= rx_ack_sync;

      if (packet_length_valid_i = '1') then
        packet_lenght_valid_latch <= '1';
        packet_length_latch <= unsigned(pkt_len_i);
      end if;

      -- Internal FIFO RAM read port (1-cycle latency). The write port is driven
      -- inline from the FILL states below; Quartus infers a simple dual-port RAM.
      fifo_rdata <= fifo_ram(fifo_raddr);

      -- ============================================================
      -- FILL: copy a completed frame from eth_ram into the next free FIFO slot.
      -- Same eth_ram read timing as the legacy single-buffer copy; only the
      -- write destination changed. Drops (overflow) only when all slots are full.
      -- ============================================================
      case sm_fill is
        when FILL_IDLE =>
          if parse_mcu_packet_tog_i /= parse_mcu_d then
            if fifo_count >= RX_NUM_SLOTS then
              rx_overflow_reg <= '1';   -- no free slot: drop the frame
            else
              rx_copy_addr    <= (others => '0');
              eth_ram_addr    <= (others => '0');
              -- Capture this frame's RX timestamp now (the TSU updates it only
              -- at the next frame's SOF, so it is stable for the trailer write).
              rx_ts_sec_latch <= timestamps_i.rx.seconds;
              rx_ts_ns_latch  <= timestamps_i.rx.nanoseconds;
              sm_fill         <= FILL_WAIT;
            end if;
          end if;

        when FILL_WAIT =>
          -- RAM latency cycle: data for addr 0 valid next cycle; prefetch addr 1.
          eth_ram_addr <= to_unsigned(1, eth_ram_addr'length);
          rx_copy_addr <= to_unsigned(1, rx_copy_addr'length);
          sm_fill      <= FILL_COPY;

        when FILL_COPY =>
          -- eth_ram_data_i holds the byte for (rx_copy_addr - 1).
          fifo_ram(wr_slot * SLOT_SIZE + to_integer(rx_copy_addr) - 1) <= eth_ram_data_i;
          if rx_copy_addr >= packet_length_latch and packet_lenght_valid_latch = '1' then
            if ADD_RX_TIMESTAMP then
              sm_fill <= FILL_TS_SEC;
            else
              sm_fill <= FILL_COMMIT;
            end if;
          end if;
          eth_ram_addr <= rx_copy_addr + 1;
          rx_copy_addr <= rx_copy_addr + 1;

        -- Trailer byte 0: seconds[3:0] in the low nibble.
        when FILL_TS_SEC =>
          fifo_ram(wr_slot * SLOT_SIZE + to_integer(rx_copy_addr) - 1) <=
            "0000" & std_logic_vector(rx_ts_sec_latch);
          rx_copy_addr   <= rx_copy_addr + 1;
          ts_write_index <= 0;
          sm_fill        <= FILL_TS_NS;

        -- Trailer bytes 1..4: nanoseconds[29:0], little-endian (top byte's high
        -- 2 bits zero), one byte per cycle.
        when FILL_TS_NS =>
          case ts_write_index is
            when 0      => fifo_ram(wr_slot*SLOT_SIZE + to_integer(rx_copy_addr) - 1) <= std_logic_vector(rx_ts_ns_latch(7 downto 0));
            when 1      => fifo_ram(wr_slot*SLOT_SIZE + to_integer(rx_copy_addr) - 1) <= std_logic_vector(rx_ts_ns_latch(15 downto 8));
            when 2      => fifo_ram(wr_slot*SLOT_SIZE + to_integer(rx_copy_addr) - 1) <= std_logic_vector(rx_ts_ns_latch(23 downto 16));
            when others => fifo_ram(wr_slot*SLOT_SIZE + to_integer(rx_copy_addr) - 1) <= "00" & std_logic_vector(rx_ts_ns_latch(29 downto 24));
          end case;
          rx_copy_addr <= rx_copy_addr + 1;
          if ts_write_index = 3 then
            ts_write_index <= 0;
            sm_fill        <= FILL_COMMIT;
          else
            ts_write_index <= ts_write_index + 1;
          end if;

        when FILL_COMMIT =>
          if ADD_RX_TIMESTAMP then
            slot_len(wr_slot) <= packet_length_latch
                                 + to_unsigned(TS_TRAILER_LEN, packet_length_latch'length);
          else
            slot_len(wr_slot) <= packet_length_latch;
          end if;
          if wr_slot = RX_NUM_SLOTS - 1 then
            wr_slot <= 0;
          else
            wr_slot <= wr_slot + 1;
          end if;
          v_commit := true;
          packet_lenght_valid_latch <= '0';
          sm_fill <= FILL_IDLE;
      end case;

      -- ============================================================
      -- DRAIN: feed the head FIFO slot into the LiteX single buffer, one frame
      -- at a time, advancing only when the SoC acks the previous frame.
      -- ============================================================
      case sm_drain is
        when DR_IDLE =>
          if fifo_count > 0 and rx_valid_reg = '0' then
            drain_len  <= slot_len(rd_slot);
            drain_addr <= (others => '0');
            fifo_raddr <= rd_slot * SLOT_SIZE;        -- present byte 0
            sm_drain   <= DR_PRIME;
          end if;

        when DR_PRIME =>
          fifo_raddr <= rd_slot * SLOT_SIZE + 1;      -- prefetch byte 1
          drain_addr <= to_unsigned(1, drain_addr'length);
          sm_drain   <= DR_COPY;

        when DR_COPY =>
          -- fifo_rdata holds the byte for (drain_addr - 1).
          buf_rx_data_o <= fifo_rdata;
          buf_rx_addr   <= drain_addr - 1;
          buf_rx_we_o   <= '1';
          if drain_addr >= drain_len then
            sm_drain <= DR_SET_VALID;
          end if;
          fifo_raddr <= rd_slot * SLOT_SIZE + to_integer(drain_addr) + 1;
          drain_addr <= drain_addr + 1;

        when DR_SET_VALID =>
          buf_rx_len   <= drain_len;
          rx_valid_reg <= '1';
          sm_drain     <= DR_WAIT_ACK;

        when DR_WAIT_ACK =>
          if rx_ack_sync = '1' and rx_ack_sync_d = '0' then
            rx_valid_reg    <= '0';
            rx_overflow_reg <= '0';
            if rd_slot = RX_NUM_SLOTS - 1 then
              rd_slot <= 0;
            else
              rd_slot <= rd_slot + 1;
            end if;
            v_pop    := true;
            sm_drain <= DR_IDLE;
          end if;
      end case;

      -- Single update of the shared occupancy counter (handles simultaneous
      -- commit + pop as a no-op).
      if v_commit and not v_pop then
        fifo_count <= fifo_count + 1;
      elsif v_pop and not v_commit then
        fifo_count <= fifo_count - 1;
      end if;
    end if;
  end process;

end architecture;
