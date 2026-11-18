-- ####################################################################
-- # LiteX Ethernet Buffer Bridge
-- #
-- # Bridges the LiteX EthPacketBuffer FPGA-side signals to the
-- # Ethernet MAC TX/RX interface.
-- #
-- # TX: SoC writes packet, sets tx_len, pulses eth_tx_request.
-- #     TX process reads buffer directly on mac_tx_clock and feeds MAC.
-- #
-- # RX: one dual-clock ring buffer is the only MCU frame storage. It has an
-- #     8-bit write port on mac_rx_clock and a 32-bit read port on the LiteX
-- #     sys clock (mcu_clk_i), built from four byte-lane RAMs (portable
-- #     mixed-width RAM: one 9/18-kbit block per lane at 4 KB).
-- #
-- #     WRITER (mac_rx): every byte ethernet_receive stores (rx_byte_we_i) is
-- #     written speculatively into free ring space as it arrives. When the
-- #     completed, error-free frame is classified as MCU traffic
-- #     (parse_mcu_packet_tog_i) the entry is committed: length header word
-- #     written, write pointer advanced. RTP / filtered PTP / errored frames
-- #     are never committed and get overwritten by the next frame.
-- #
-- #     READER (sys): reads the head entry's header word itself, publishes
-- #     rx_len + rx_valid, and maps the CPU's Wishbone word reads onto the
-- #     entry body (frame at RX offset 0, as before). The SoC ack (rising
-- #     edge) pops the entry.
-- #
-- #     The two pointers cross domains with a toggle handshake each (they
-- #     jump by whole entries, so Gray coding does not apply).
-- #
-- #     ethernet_receive latches the RX timestamp per frame and appends
-- #     it as a 5-byte trailer behind the frame data, so the frame and its
-- #     timestamp travel together (no shared-register race during bursts).
-- #     pkt_len_i therefore already includes those 5 bytes.
-- ####################################################################
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;

entity litex_eth_buffer_bridge is
  generic (

    -- true (SW PTP): the RX-timestamp trailer is handed to the SoC as part
    -- of the frame.  false (HW PTP): the trailer is not committed and the
    -- SoC sees the bare frame.
    ADD_RX_TIMESTAMP : boolean := false;
    -- RX ring buffer size in bytes. MUST be a power of two and >= 2048 so
    -- any single entry (max frame + trailer + header word) always fits.
    -- 4096 = 4 lanes x 1024 x 8 bit = one block RAM per lane.
    RX_RING_SIZE : integer := 4096
  );
  port (
    -- ================================================================
    -- LiteX EthPacketBuffer FPGA-side signals
    -- RX read side on mcu_clk_i (= LiteX sys clock), TX on mac_tx_clock
    -- ================================================================

    -- RX: the SoC reads the published frame straight out of the ring
    buf_rx_rd_addr_i : in  std_logic_vector(10 downto 0); -- Wishbone word offset into the frame
    buf_rx_rd_data_o : out std_logic_vector(31 downto 0); -- frame bytes 4i..4i+3 (LE lanes), 1-cycle latency
    buf_rx_len_o    : out std_logic_vector(10 downto 0);  -- valid while buf_rx_valid_o
    buf_rx_valid_o  : out std_logic;
    buf_rx_ack_i    : in  std_logic;   -- rising edge releases the frame
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
    eth_rx_overflow_o : out std_logic;   -- level: RX frame dropped (cleared by the next pop)

    -- ================================================================
    -- MAC TX interface (mac_tx_clock_i domain)
    -- ================================================================
    mac_tx_clock_i     : in  std_logic;
    mac_tx_reset_i     : in  std_logic;
    mac_tx_enable_o    : out std_logic;

    mac_tx_byte_sent_i : in  std_logic;
    mac_tx_dat_o    : out std_logic_vector(7 downto 0);
    mac_start_prefetch_i : in std_logic;
    mac_speed_in        : in std_logic_vector(1 downto 0);

    -- TX arbitration
    tx_allow_req_o     : out std_logic;
    tx_allow_i         : in  std_logic;

    -- ================================================================
    -- RX byte stream + classification from ethernet_receive
    -- (mac_rx_clock_i domain)
    -- ================================================================
    mac_rx_clock_i      : in  std_logic;
    mac_rx_reset_i      : in  std_logic;  -- async; resets both RX domains
    rx_byte_we_i        : in  std_logic;  -- strobe: new byte at rx_byte_index_i
    rx_byte_index_i     : in  std_logic_vector(10 downto 0);  -- byte offset in the frame (0 = first byte, gapless)
    rx_byte_data_i      : in  std_logic_vector(7 downto 0);
    parse_mcu_packet_tog_i  : in  std_logic;  -- toggles when the completed frame is for the MCU
    pkt_len_i           : in  std_logic_vector(10 downto 0);  -- frame length from ethernet_receive (incl. 5-byte timestamp trailer)

    mcu_clk_i         : in std_logic
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
  signal buf_tx_addr   : unsigned(10 downto 0);
  signal tx_zsof : STD_LOGIC := '0';

  -- ================================================================
  -- RX ring
  --
  -- Word-aligned entries, packed back-to-back, wrapping at RING_WORDS:
  --   word 0          : header, lanes 1:0 = len (LE), lanes 3:2 unused
  --   word 1 ..       : frame bytes, byte i in word 1 + i/4, lane i mod 4
  -- 'len' is the MCU-visible length (= buf_rx_len). The 5-byte rx-timestamp
  -- trailer is part of it when ADD_RX_TIMESTAMP; otherwise it is written
  -- behind the body but not committed.
  -- ================================================================
  constant RING_WORDS : natural := RX_RING_SIZE / 4;
  constant WADDR_W    : natural := integer(ceil(log2(real(RING_WORDS))));
  constant PTR_W      : natural := WADDR_W + 1;  -- extra wrap bit: full /= empty
  constant TS_TRAILER_LEN : natural := 5;        -- 1 byte seconds + 4 bytes nanoseconds

  type t_lane_bytes is array (0 to 3) of std_logic_vector(7 downto 0);

  -- write port (mac_rx), registered
  signal lane_we    : std_logic_vector(3 downto 0) := (others => '0');
  signal lane_waddr : unsigned(WADDR_W - 1 downto 0) := (others => '0');
  signal lane_wdata : t_lane_bytes := (others => (others => '0'));
  -- read port (mcu_clk)
  signal lane_raddr : unsigned(WADDR_W - 1 downto 0);
  signal lane_rdata : t_lane_bytes;

  -- ---------------- WRITER (mac_rx domain) ----------------
  signal wr_ptr       : unsigned(PTR_W - 1 downto 0) := (others => '0'); -- next entry's header word
  signal rd_ptr_w     : unsigned(PTR_W - 1 downto 0) := (others => '0'); -- reader's pointer, as seen here
  -- bytes of the current frame that landed in the ring in order (index 0
  -- restarts it; a byte that does not fit or arrives out of order freezes
  -- it, so COMMIT sees a short frame and drops it)
  signal fill_written : integer range 0 to 2048 := 0;
  signal parse_mcu_d  : std_ulogic := '0';
  signal rx_overflow_reg : std_ulogic := '0';
  signal rx_overflow_reg_1cdc : std_ulogic := '0';
  signal rx_overflow_reg_2cdc : std_ulogic := '0';

  -- wr_ptr -> reader (toggle handshake; wp_pub stable while wp_req /= ack)
  signal wp_pub      : unsigned(PTR_W - 1 downto 0) := (others => '0');
  signal wp_req      : std_ulogic := '0';
  signal wp_ack_meta : std_ulogic := '0';
  signal wp_ack_sync : std_ulogic := '0';
  -- rd_ptr from reader
  signal rp_req_meta : std_ulogic := '0';
  signal rp_req_sync : std_ulogic := '0';
  signal rp_ack      : std_ulogic := '0';

  -- ---------------- READER (mcu_clk domain) ----------------
  signal rd_rst_meta : std_ulogic := '1';
  signal rd_rst      : std_ulogic := '1';   -- mac_rx_reset_i, deassert synced
  type t_rd_sm is (RD_IDLE, RD_HDR, RD_VALID);
  signal sm_rd       : t_rd_sm := RD_IDLE;
  signal rd_ptr      : unsigned(PTR_W - 1 downto 0) := (others => '0'); -- head entry's header word
  signal rd_body     : unsigned(WADDR_W - 1 downto 0) := (others => '0'); -- head entry's first body word
  signal wr_ptr_r    : unsigned(PTR_W - 1 downto 0) := (others => '0'); -- writer's pointer, as seen here
  signal rx_len_reg  : unsigned(10 downto 0) := (others => '0');
  signal rx_valid_reg : std_ulogic := '0';
  signal rx_ack_d    : std_ulogic := '0';

  -- rd_ptr -> writer (toggle handshake; rp_pub stable while rp_req /= ack)
  signal rp_pub      : unsigned(PTR_W - 1 downto 0) := (others => '0');
  signal rp_req      : std_ulogic := '0';
  signal rp_ack_meta : std_ulogic := '0';
  signal rp_ack_sync : std_ulogic := '0';
  -- wr_ptr from writer
  signal wp_req_meta : std_ulogic := '0';
  signal wp_req_sync : std_ulogic := '0';
  signal wp_ack      : std_ulogic := '0';

  -- words an entry of 'len' bytes occupies (header + body)
  function entry_words(len : unsigned(10 downto 0)) return unsigned is
  begin
    return to_unsigned(1 + (to_integer(len) + 3) / 4, PTR_W);
  end function;

begin

  -- ================================================================
  -- Output assignments & cdc
  -- ================================================================
  buf_tx_addr_o    <= std_logic_vector(buf_tx_addr);
  buf_rx_len_o     <= std_logic_vector(rx_len_reg);
  buf_rx_valid_o   <= rx_valid_reg;
  buf_rx_rd_data_o <= lane_rdata(3) & lane_rdata(2) & lane_rdata(1) & lane_rdata(0);

  mcu_cdc : process (mcu_clk_i)
  begin
    if (rising_edge(mcu_clk_i)) then
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
  -- RX ring: four byte-lane RAMs, simple dual port, dual clock
  -- ================================================================
  gen_lanes : for k in 0 to 3 generate
    type t_lane_ram is array (0 to RING_WORDS - 1) of std_logic_vector(7 downto 0);
    signal lane_ram : t_lane_ram;
  begin
    p_lane_wr : process (mac_rx_clock_i)
    begin
      if rising_edge(mac_rx_clock_i) then
        if lane_we(k) = '1' then
          lane_ram(to_integer(lane_waddr)) <= lane_wdata(k);
        end if;
      end if;
    end process;

    p_lane_rd : process (mcu_clk_i)
    begin
      if rising_edge(mcu_clk_i) then
        lane_rdata(k) <= lane_ram(to_integer(lane_raddr));
      end if;
    end process;
  end generate;

  -- ================================================================
  -- WRITER (mac_rx domain): INGEST + COMMIT, pointer exchange
  --
  -- The lane write port is registered; COMMIT's header write falls into the
  -- gap between frames and takes priority over INGEST, so a byte colliding
  -- with it is refused and the frame dropped instead of misplaced.
  -- ================================================================
  p_wr : process (mac_rx_clock_i, mac_rx_reset_i)
    variable v_pkt     : unsigned(10 downto 0);
    variable v_idx     : integer range 0 to 2047;
    variable v_written : integer range 0 to 2048;
    variable v_free    : integer range 0 to RING_WORDS;
    variable v_commit  : boolean;
  begin
    if mac_rx_reset_i = '1' then
      lane_we         <= (others => '0');
      lane_waddr      <= (others => '0');
      lane_wdata      <= (others => (others => '0'));
      wr_ptr          <= (others => '0');
      rd_ptr_w        <= (others => '0');
      fill_written    <= 0;
      parse_mcu_d     <= '0';
      rx_overflow_reg <= '0';
      wp_pub          <= (others => '0');
      wp_req          <= '0';
      wp_ack_meta     <= '0';
      wp_ack_sync     <= '0';
      rp_req_meta     <= '0';
      rp_req_sync     <= '0';
      rp_ack          <= '0';
    elsif rising_edge(mac_rx_clock_i) then
      lane_we  <= (others => '0');
      v_commit := false;

      -- ---- reader's pointer: a pop arrived (also clears the drop flag) ----
      rp_req_meta <= rp_req;
      rp_req_sync <= rp_req_meta;
      if rp_req_sync /= rp_ack then
        rd_ptr_w        <= rp_pub;
        rp_ack          <= rp_req_sync;
        rx_overflow_reg <= '0';
      end if;
      -- free words, conservative: rd_ptr_w only lags the real read pointer
      v_free := RING_WORDS - to_integer(wr_ptr - rd_ptr_w);

      -- ---- COMMIT ----
      -- ethernet_receive toggles parse_mcu_packet_tog_i after the last byte
      -- (incl. trailer) of an error-free MCU frame went through INGEST;
      -- pkt_len_i is valid from then until the next frame's toggle.
      parse_mcu_d <= parse_mcu_packet_tog_i;
      if parse_mcu_packet_tog_i /= parse_mcu_d then
        -- pkt_len_i includes the 5-byte rx-timestamp trailer; strip it
        -- unless the SoC is supposed to see it.
        if ADD_RX_TIMESTAMP then
          v_pkt := unsigned(pkt_len_i);
        else
          v_pkt := unsigned(pkt_len_i) - to_unsigned(TS_TRAILER_LEN, 11);
        end if;
        if (not ADD_RX_TIMESTAMP and unsigned(pkt_len_i) < TS_TRAILER_LEN)
           or to_integer(v_pkt) > fill_written then
          -- body did not fully land in the ring (no room, or longer than
          -- ethernet_receive stores): drop the frame
          rx_overflow_reg <= '1';
        else
          v_commit      := true;
          lane_we       <= "0011";
          lane_waddr    <= wr_ptr(WADDR_W - 1 downto 0);
          lane_wdata(0) <= std_logic_vector(v_pkt(7 downto 0));
          lane_wdata(1) <= "00000" & std_logic_vector(v_pkt(10 downto 8));
          wr_ptr        <= wr_ptr + entry_words(v_pkt);
        end if;
      end if;

      -- ---- INGEST ----
      -- Every stored byte goes straight to its place in the entry at wr_ptr,
      -- behind the header word. Only free ring space is written, and only in
      -- order: index 0 restarts the frame, any refused byte stops
      -- fill_written for the rest of it.
      v_written := fill_written;
      if rx_byte_we_i = '1' then
        v_idx := to_integer(unsigned(rx_byte_index_i));
        if v_idx = 0 then
          v_written := 0;
        end if;
        if not v_commit and v_idx = v_written and 1 + v_idx / 4 < v_free then
          lane_we(v_idx mod 4) <= '1';
          lane_waddr <= wr_ptr(WADDR_W - 1 downto 0) + 1
                        + to_unsigned(v_idx / 4, WADDR_W);
          lane_wdata(v_idx mod 4) <= rx_byte_data_i;
          v_written := v_idx + 1;
        end if;
      end if;
      fill_written <= v_written;

      -- ---- publish wr_ptr once the previous value was taken ----
      -- wr_ptr moves together with the registered header write, so the
      -- header lands in the RAM one cycle before wp_pub can show it.
      wp_ack_meta <= wp_ack;
      wp_ack_sync <= wp_ack_meta;
      if wp_ack_sync = wp_req and wp_pub /= wr_ptr then
        wp_pub <= wr_ptr;
        wp_req <= not wp_req;
      end if;
    end if;
  end process;

  -- ================================================================
  -- READER (mcu_clk domain)
  -- ================================================================
  p_rd_rst : process (mcu_clk_i, mac_rx_reset_i)
  begin
    if mac_rx_reset_i = '1' then
      rd_rst_meta <= '1';
      rd_rst      <= '1';
    elsif rising_edge(mcu_clk_i) then
      rd_rst_meta <= '0';
      rd_rst      <= rd_rst_meta;
    end if;
  end process;

  -- Read address: the CPU's word offset into the published body while a
  -- frame is valid, otherwise the head entry's header word (RD_IDLE reads
  -- it, RD_HDR takes it one cycle later).
  lane_raddr <= rd_body + resize(unsigned(buf_rx_rd_addr_i), WADDR_W)
                  when sm_rd = RD_VALID
                else rd_ptr(WADDR_W - 1 downto 0);

  p_rd : process (mcu_clk_i, rd_rst)
  begin
    if rd_rst = '1' then
      sm_rd        <= RD_IDLE;
      rd_ptr       <= (others => '0');
      rd_body      <= (others => '0');
      wr_ptr_r     <= (others => '0');
      rx_len_reg   <= (others => '0');
      rx_valid_reg <= '0';
      rx_ack_d     <= '0';
      rp_pub       <= (others => '0');
      rp_req       <= '0';
      rp_ack_meta  <= '0';
      rp_ack_sync  <= '0';
      wp_req_meta  <= '0';
      wp_req_sync  <= '0';
      wp_ack       <= '0';
    elsif rising_edge(mcu_clk_i) then
      -- ---- writer's pointer: entries were committed ----
      wp_req_meta <= wp_req;
      wp_req_sync <= wp_req_meta;
      if wp_req_sync /= wp_ack then
        wr_ptr_r <= wp_pub;
        wp_ack   <= wp_req_sync;
      end if;

      rx_ack_d <= buf_rx_ack_i;
      case sm_rd is
        when RD_IDLE =>
          if rd_ptr /= wr_ptr_r then
            sm_rd <= RD_HDR;             -- header word is being read now
          end if;

        when RD_HDR =>
          rx_len_reg   <= unsigned(lane_rdata(1)(2 downto 0)) & unsigned(lane_rdata(0));
          rd_body      <= rd_ptr(WADDR_W - 1 downto 0) + 1;
          rx_valid_reg <= '1';
          sm_rd        <= RD_VALID;

        when RD_VALID =>
          if buf_rx_ack_i = '1' and rx_ack_d = '0' then
            rx_valid_reg <= '0';
            rd_ptr       <= rd_ptr + entry_words(rx_len_reg);
            sm_rd        <= RD_IDLE;
          end if;
      end case;

      -- ---- publish rd_ptr once the previous value was taken ----
      rp_ack_meta <= rp_ack;
      rp_ack_sync <= rp_ack_meta;
      if rp_ack_sync = rp_req and rp_pub /= rd_ptr then
        rp_pub <= rd_ptr;
        rp_req <= not rp_req;
      end if;
    end if;
  end process;

end architecture;
