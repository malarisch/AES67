-- ####################################################################
-- # FMC Parallel Ethernet Client (simplified, no PTP)
-- # Register map matches spi_ethernet_client for basic TX/RX.
-- ####################################################################
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.ethernet_types.all;

entity fmc_ethernet_client is
  generic (
    TX_ADDR_WIDTH : positive := 11;  -- 2**11 = 2048 Bytes
    RX_ADDR_WIDTH : positive := 11
  );
  port (
    clk_sys_i   : in  std_ulogic;
    rst_sys_i   : in  std_ulogic;

    -- Simple asynchronous FMC-like bus
    fmc_addr_i  : in  std_ulogic_vector(6 downto 0);
    fmc_data_io : inout std_ulogic_vector(7 downto 0);
    fmc_ne_n_i  : in  std_ulogic;
    fmc_noe_n_i : in  std_ulogic;
    fmc_nwe_n_i : in  std_ulogic;
    fmc_int_o   : out std_ulogic;

    -- MAC TX
    mac_tx_clock_i     : in  std_ulogic;
    mac_tx_reset_i     : in  std_ulogic;
    mac_tx_enable_o    : out std_ulogic;
    mac_tx_data_o      : out t_ethernet_data;
    mac_tx_byte_sent_i : in  std_ulogic;
    mac_tx_busy_i      : in  std_ulogic;

    -- MAC RX
    mac_rx_clock_i        : in  std_ulogic;
    mac_rx_reset_i        : in  std_ulogic;
    mac_rx_frame_i        : in  std_ulogic;
    mac_rx_data_i         : in  t_ethernet_data;
    mac_rx_byte_rcv_i     : in  std_ulogic;
    mac_rx_error_i        : in  std_ulogic;
    system_config_wr_en   : out std_ulogic;
    system_config_wr_addr : out unsigned(10 downto 0);
    system_config_wr_data : out std_ulogic_vector(7 downto 0);

    system_config_rd_addr : out unsigned(10 downto 0);
    system_config_rd_data : in  std_ulogic_vector(7 downto 0);

    system_config_done_o : out std_ulogic;

    -- Write 0x50 outputs
    pll_ppb_measurement_start_o : out std_ulogic;
    reset_wallclock_o           : out std_ulogic;
    reset_ptp_o                 : out std_ulogic;
    reset_ethernet_o            : out std_ulogic;

    -- Read 0x50 inputs (clocking flags, async -> need CDC)
    pll_ppb_measurement_valid_i : in  std_ulogic;
    wallclock_locked_i          : in  std_ulogic;
    wallclock_phasejump_i       : in  std_ulogic;
    wallclock_configured_i      : in  std_ulogic;
    ptp_leader_lost_i           : in  std_ulogic;

    -- Read 0x51 inputs (ethernet flags, async -> need CDC)
    eth_link_up_i               : in  std_ulogic;
    eth_link_speed_i            : in  std_ulogic_vector(1 downto 0);

    -- Read 0x52..0x54 (32-bit values)
    path_delay_i                : in  std_ulogic_vector(31 downto 0);
    leader_offset_i             : in  std_ulogic_vector(31 downto 0);
    clock_ppb_meter_i           : in  std_ulogic_vector(31 downto 0);

    ptp_is_leader_o               : out STD_ULOGIC;
    ptp_is_follower_o             : out STD_ULOGIC;

    -- TX stream config (written via 0x58, 20 bytes per stream -> tx_router config RAM)
    tx_stream_config_wr_en_o      : out std_ulogic;
    tx_stream_config_wr_addr_o    : out std_ulogic_vector(7 downto 0);
    tx_stream_config_wr_data_o    : out std_ulogic_vector(7 downto 0);

    tx_allow_req_o : out std_ulogic;
    tx_allow_i : in std_ulogic;

    -- RX stream config (written via 0x59, 16 bytes per stream -> rx_ringbuffer stream_ram)
    rx_stream_config_wr_clk_o  : out std_ulogic;
    rx_stream_config_wr_addr_o : out std_ulogic_vector(7 downto 0);
    rx_stream_config_wr_data_o : out std_ulogic_vector(7 downto 0)
  );
end entity;

architecture rtl of fmc_ethernet_client is

  signal reg_tx_len      : unsigned(15 downto 0) := (others => '0');
  signal reg_tx_start    : std_ulogic := '0';
  signal reg_tx_start_toggle : std_ulogic := '0'; -- Toggle for CDC

  signal reg_rx_len      : unsigned(15 downto 0) := (others => '0');
  signal reg_rx_ready    : std_ulogic := '0';
  signal reg_rx_overflow : std_ulogic := '0';

  -- RX clear handshake
  signal rx_clear_req_sys       : std_ulogic := '0';
  signal rx_clear_ack_mac       : std_ulogic := '0';
  signal rx_clear_ack_sync1_sys : std_ulogic := '0';
  signal rx_clear_ack_sync2_sys : std_ulogic := '0';
  signal rx_clear_sys_pulse     : std_ulogic := '0';
  signal rx_clear_req_sync1_mac : std_ulogic := '0';
  signal rx_clear_req_sync2_mac : std_ulogic := '0';
  signal rx_clear_req_mac_d     : std_ulogic := '0';
  signal rx_clear_mac_pulse     : std_ulogic := '0';

  signal reg_rx_len_sys      : unsigned(15 downto 0) := (others => '0');
  signal reg_rx_ready_sys    : std_ulogic := '0';
  signal reg_rx_overflow_sys : std_ulogic := '0';

  -- 2-FF CDC synchronizers for reg_rx_ready and reg_rx_overflow (mac_rx -> clk_sys)
  signal rx_ready_meta       : std_ulogic := '0';
  signal rx_ready_sync       : std_ulogic := '0';
  signal rx_overflow_meta    : std_ulogic := '0';
  signal rx_overflow_sync    : std_ulogic := '0';
  signal rx_len_latched      : unsigned(15 downto 0) := (others => '0');

  signal reg_tx_len_reset : std_ulogic := '0';

  -- TX busy flag: set when tx_start_toggle is issued, cleared by tx_clear_sys_pulse
  signal tx_busy_sys : std_ulogic := '0';

  -- TX FIFO
  signal txfifo_wr_en    : std_ulogic := '0';
  signal txfifo_wr_data  : std_ulogic_vector(7 downto 0) := (others => '0');
  signal txfifo_wr_full  : std_ulogic;
  signal txfifo_rd_en    : std_ulogic := '0';
  signal txfifo_rd_data  : std_ulogic_vector(7 downto 0);
  signal txfifo_rd_empty : std_ulogic;

  -- RX FIFO
  signal rxfifo_wr_en    : std_ulogic := '0';
  signal rxfifo_wr_data  : std_ulogic_vector(7 downto 0) := (others => '0');
  signal rxfifo_wr_full  : std_ulogic;
  signal rxfifo_rd_en    : std_ulogic := '0';
  signal rxfifo_rd_data  : std_ulogic_vector(7 downto 0);
  signal rxfifo_rd_empty : std_ulogic;
  signal fmc_rx_bytes_sent   : unsigned(15 downto 0) := (others => '0');

  -- MAC TX
  signal mac_tx_active   : std_ulogic := '0';
  signal tx_start_sync_1 : std_ulogic := '0';
  signal tx_start_sync_2 : std_ulogic := '0';
  signal tx_start_pulse  : std_ulogic := '0';
  signal tx_prefetch_valid : std_ulogic := '0';
  signal tx_byte_sent_count : unsigned(15 downto 0) := (others => '0');
  signal tx_clear_toggle_mac : std_ulogic := '0';
  signal tx_clear_sync_sys_1 : std_ulogic := '0';
  signal tx_clear_sync_sys_2 : std_ulogic := '0';
  signal tx_clear_sys_pulse  : std_ulogic := '0';
  signal tx_clear_mac_d      : std_ulogic := '0';
  signal tx_clear_mac_pulse  : std_ulogic := '0';
  signal mac_tx_preamble_bytes_sent : unsigned(3 downto 0) := (others => '0');

  type t_tx_SM is (s_Idle, s_waitForAllow, s_PrimeTx, s_Transmit, s_End);
  signal sm_tx_ethernet : t_tx_SM := s_Idle;

  -- RX bookkeeping
  signal mac_rx_frame_d : std_ulogic := '0';
  signal debug_rx_byte_cnt : unsigned(15 downto 0) := (others => '0');

  -- FMC signals
  signal fmc_data_out : std_ulogic_vector(7 downto 0) := (others => '0');
  signal fmc_data_oe  : std_ulogic := '0';
  signal fmc_data_in  : std_ulogic_vector(7 downto 0);
  signal nwe_d        : std_ulogic := '1';
  signal fmc_noe_n_sync_d : std_ulogic := '1';
  signal fmc_nwe_n_sync_d : std_ulogic := '1';
  signal fmc_ne_n_sync_d  : std_ulogic := '1';
  signal fmc_ne_n_meta  : std_ulogic := '1';
  signal fmc_ne_n_sync  : std_ulogic := '1';
  signal fmc_noe_n_meta : std_ulogic := '1';
  signal fmc_noe_n_sync : std_ulogic := '1';
  signal fmc_nwe_n_meta : std_ulogic := '1';
  signal fmc_nwe_n_sync : std_ulogic := '1';
  signal fmc_addr_meta  : std_ulogic_vector(6 downto 0) := (others => '0');
  signal fmc_addr_sync  : std_ulogic_vector(6 downto 0) := (others => '0');
  signal fmc_addr_sync_d : std_ulogic_vector(6 downto 0) := (others => '0');
  signal fmc_din_meta   : std_ulogic_vector(7 downto 0) := (others => '0');
  signal fmc_din_sync   : std_ulogic_vector(7 downto 0) := (others => '0');
  signal fmc_din_sync_d : std_ulogic_vector(7 downto 0) := (others => '0');

  signal fmc_wr_addr_lat : unsigned(6 downto 0) := (others => '0');
  signal fmc_wr_data_lat : std_ulogic_vector(7 downto 0) := (others => '0');
  signal fmc_read_latched : std_ulogic := '0';
  signal fmc_read_addr_lat : unsigned(6 downto 0) := (others => '0');
  signal fmc_read_data_lat : std_ulogic_vector(7 downto 0) := (others => '0');
  signal fmc_ne_qual_low : std_ulogic := '0';

  -- NWE-edge write capture (CDC from NWE domain → clk_sys domain)
  signal wr_cap_addr   : unsigned(6 downto 0) := (others => '0');
  signal wr_cap_data   : std_ulogic_vector(7 downto 0) := (others => '0');
  signal wr_cap_toggle : std_ulogic := '0';
  signal wr_cap_toggle_meta : std_ulogic := '0';
  signal wr_cap_toggle_sync : std_ulogic := '0';
  signal wr_cap_toggle_d    : std_ulogic := '0';
  signal wr_pending         : std_ulogic := '0';


  signal overflow_interrupt : std_ulogic := '0';

  signal register_byte_count : unsigned(3 downto 0) := (others => '0');

  -- CDC registers for status inputs (2-FF synchronisers)
  signal ppb_valid_meta, ppb_valid_sync               : std_ulogic := '0';
  signal wc_locked_meta, wc_locked_sync               : std_ulogic := '0';
  signal wc_phasejump_meta, wc_phasejump_sync         : std_ulogic := '0';
  signal wc_configured_meta, wc_configured_sync       : std_ulogic := '0';
  signal ptp_leader_lost_meta, ptp_leader_lost_sync   : std_ulogic := '0';
  signal eth_link_up_meta, eth_link_up_sync           : std_ulogic := '0';
  signal eth_speed_meta, eth_speed_sync               : std_ulogic_vector(1 downto 0) := (others => '0');

  -- CDC for 32-bit status inputs
  signal path_delay_meta, path_delay_sync             : std_ulogic_vector(31 downto 0) := (others => '0');
  signal leader_offset_meta, leader_offset_sync       : std_ulogic_vector(31 downto 0) := (others => '0');
  signal ppb_meter_meta, ppb_meter_sync               : std_ulogic_vector(31 downto 0) := (others => '0');

  -- PPB measurement start handshake
  signal ppb_start_reg : std_ulogic := '0';

  -- TX stream config state (0x58 write, 20 bytes per stream)
  signal stream_cfg_byte_count : unsigned(7 downto 0) := (others => '0');
  signal stream_cfg_base_addr  : unsigned(7 downto 0) := (others => '0');

  -- RX stream config state (0x59 write, 16 bytes per stream -> rx_ringbuffer stream_ram)
  signal rx_stream_cfg_byte_count : unsigned(7 downto 0) := (others => '0');
  signal rx_stream_cfg_base_addr  : unsigned(7 downto 0) := (others => '0');
  signal rx_stream_cfg_wr_en      : std_ulogic := '0';



  -- Read byte counter for multi-byte registers (0x52..0x54)
  signal read_byte_count    : unsigned(1 downto 0) := (others => '0');
  signal read_reg_addr_prev : std_ulogic_vector(6 downto 0) := (others => '0');

begin

  fmc_data_in <= fmc_data_io;
  -- Drive data bus only when internal OE is asserted *and* external raw strobes
  -- indicate a read cycle. This prevents stale data from a previous read from
  -- appearing at the very beginning of the next read cycle.
  fmc_data_io <= fmc_data_out
    when (fmc_data_oe = '1') and (fmc_ne_n_i = '0') and (fmc_noe_n_i = '0') and (fmc_nwe_n_i = '1')
    else (others => 'Z');

  -- Concurrent output assignments for status/control ports
  pll_ppb_measurement_start_o <= ppb_start_reg;

  -- RX stream config clock: gate clk_sys_i with wr_en so rx_ringbuffer sees a rising edge per write
  rx_stream_config_wr_clk_o <= clk_sys_i and rx_stream_cfg_wr_en;


  -- Config RAM read address: combinational so data is ready when latched
  system_config_rd_addr <= resize(unsigned(fmc_addr_meta) - to_unsigned(16#60#, 7), 11)
                           when unsigned(fmc_addr_meta) >= 16#60#
                           else (others => '0');

  -- Qualify NE low using both raw and synchronized versions to tolerate phase differences
  fmc_ne_qual_low <= '1' when (fmc_ne_n_i = '0') or (fmc_ne_n_sync = '0') or (fmc_ne_n_sync_d = '0') else '0';

  --------------------------------------------------------------------
  -- TX-FIFO
  --------------------------------------------------------------------
  tx_fifo_inst : entity work.async_fifo
    generic map (
      DATA_WIDTH => 8,
      ADDR_WIDTH => TX_ADDR_WIDTH
    )
    port map (
      wr_clk_i   => clk_sys_i,
      wr_rst_i   => rst_sys_i or tx_clear_sys_pulse,
      wr_en_i    => txfifo_wr_en,
      wr_data_i  => txfifo_wr_data,
      wr_full_o  => txfifo_wr_full,

      rd_clk_i   => mac_tx_clock_i,
      rd_rst_i   => mac_tx_reset_i or tx_clear_mac_pulse,
      rd_en_i    => txfifo_rd_en,
      rd_data_o  => txfifo_rd_data,
      rd_empty_o => txfifo_rd_empty
    );

  --------------------------------------------------------------------
  -- RX-FIFO
  --------------------------------------------------------------------
  rx_fifo_inst : entity work.async_fifo
    generic map (
      DATA_WIDTH => 8,
      ADDR_WIDTH => RX_ADDR_WIDTH
    )
    port map (
      wr_clk_i   => mac_rx_clock_i,
      wr_rst_i   => mac_rx_reset_i or rx_clear_mac_pulse,
      wr_en_i    => rxfifo_wr_en,
      wr_data_i  => rxfifo_wr_data,
      wr_full_o  => rxfifo_wr_full,

      rd_clk_i   => clk_sys_i,
      rd_rst_i   => rst_sys_i or rx_clear_sys_pulse,
      rd_en_i    => rxfifo_rd_en,
      rd_data_o  => rxfifo_rd_data,
      rd_empty_o => rxfifo_rd_empty
    );

  --------------------------------------------------------------------
  -- Sync clear between domains
  --------------------------------------------------------------------
  process(clk_sys_i, rst_sys_i)
  begin
    if rst_sys_i = '1' then
      rx_clear_sys_pulse <= '0';
      rx_clear_ack_sync1_sys <= '0';
      rx_clear_ack_sync2_sys <= '0';
      reg_rx_len_sys      <= (others => '0');
      reg_rx_ready_sys    <= '0';
      reg_rx_overflow_sys <= '0';
      rx_ready_meta       <= '0';
      rx_ready_sync       <= '0';
      rx_overflow_meta    <= '0';
      rx_overflow_sync    <= '0';
      rx_len_latched      <= (others => '0');

    elsif rising_edge(clk_sys_i) then
      -- RX clear handshake (ack from MAC domain)
      rx_clear_ack_sync1_sys <= rx_clear_ack_mac;
      rx_clear_ack_sync2_sys <= rx_clear_ack_sync1_sys;
      rx_clear_sys_pulse <= rx_clear_ack_sync1_sys and not rx_clear_ack_sync2_sys;

      -- 2-FF CDC for reg_rx_ready and reg_rx_overflow (mac_rx -> clk_sys)
      rx_ready_meta    <= reg_rx_ready;
      rx_ready_sync    <= rx_ready_meta;
      rx_overflow_meta <= reg_rx_overflow;
      rx_overflow_sync <= rx_overflow_meta;

      if rx_clear_sys_pulse = '1' then
        reg_rx_ready_sys    <= '0';
        reg_rx_overflow_sys <= '0';
        reg_rx_len_sys      <= (others => '0');
      else
        reg_rx_ready_sys    <= rx_ready_sync;
        reg_rx_overflow_sys <= rx_overflow_sync;
        -- Latch length when ready rises — reg_rx_len is stable in the
        -- MAC domain while reg_rx_ready is high, so sampling it a few
        -- clk_sys cycles after the ready sync is safe.
        if rx_ready_sync = '1' then
          reg_rx_len_sys <= reg_rx_len;
        end if;
      end if;

      tx_clear_sync_sys_1 <= tx_clear_toggle_mac;
      tx_clear_sync_sys_2 <= tx_clear_sync_sys_1;
      tx_clear_sys_pulse  <= tx_clear_sync_sys_1 xor tx_clear_sync_sys_2;
    end if;
  end process;

  --------------------------------------------------------------------
  -- FMC signal synchronization into clk_sys_i domain
  -- (NE, NOE still needed for read path)
  --------------------------------------------------------------------
  process(clk_sys_i, rst_sys_i)
  begin
    if rst_sys_i = '1' then
      fmc_ne_n_meta <= '1';
      fmc_ne_n_sync <= '1';
      fmc_ne_n_sync_d <= '1';
      fmc_noe_n_meta <= '1';
      fmc_noe_n_sync <= '1';
      fmc_noe_n_sync_d <= '1';
      fmc_nwe_n_meta <= '1';
      fmc_nwe_n_sync <= '1';
      fmc_nwe_n_sync_d <= '1';

      fmc_addr_meta <= (others => '0');
      fmc_addr_sync <= (others => '0');
      fmc_addr_sync_d <= (others => '0');
      fmc_din_meta  <= (others => '0');
      fmc_din_sync  <= (others => '0');
      fmc_din_sync_d <= (others => '0');

      nwe_d <= '1';

      wr_cap_toggle_meta <= '0';
      wr_cap_toggle_sync <= '0';
      wr_cap_toggle_d    <= '0';
      wr_pending         <= '0';
    elsif rising_edge(clk_sys_i) then
      fmc_ne_n_meta <= fmc_ne_n_i;
      fmc_ne_n_sync <= fmc_ne_n_meta;
      fmc_ne_n_sync_d <= fmc_ne_n_sync;

      fmc_noe_n_meta <= fmc_noe_n_i;
      fmc_noe_n_sync <= fmc_noe_n_meta;
      fmc_noe_n_sync_d <= fmc_noe_n_sync;

      fmc_nwe_n_meta <= fmc_nwe_n_i;
      fmc_nwe_n_sync <= fmc_nwe_n_meta;
      fmc_nwe_n_sync_d <= fmc_nwe_n_sync;
      nwe_d <= fmc_nwe_n_sync_d;

      fmc_addr_meta <= fmc_addr_i;
      fmc_addr_sync <= fmc_addr_meta;
      fmc_addr_sync_d <= fmc_addr_sync;

      fmc_din_meta  <= fmc_data_in;
      fmc_din_sync  <= fmc_din_meta;
      fmc_din_sync_d <= fmc_din_sync;

      -- 2-stage sync of write-capture toggle into clk_sys domain
      wr_cap_toggle_meta <= wr_cap_toggle;
      wr_cap_toggle_sync <= wr_cap_toggle_meta;
      wr_cap_toggle_d    <= wr_cap_toggle_sync;
      wr_pending         <= wr_cap_toggle_sync xor wr_cap_toggle_d;
    end if;
  end process;

  --------------------------------------------------------------------
  -- CDC: Synchronise asynchronous status inputs into clk_sys_i
  --------------------------------------------------------------------
  p_status_cdc : process(clk_sys_i, rst_sys_i)
  begin
    if rst_sys_i = '1' then
      ppb_valid_meta       <= '0';  ppb_valid_sync       <= '0';
      wc_locked_meta       <= '0';  wc_locked_sync       <= '0';
      wc_phasejump_meta    <= '0';  wc_phasejump_sync    <= '0';
      wc_configured_meta   <= '0';  wc_configured_sync   <= '0';
      ptp_leader_lost_meta <= '0';  ptp_leader_lost_sync <= '0';
      eth_link_up_meta     <= '0';  eth_link_up_sync     <= '0';
      eth_speed_meta       <= (others => '0'); eth_speed_sync       <= (others => '0');
      path_delay_meta      <= (others => '0'); path_delay_sync      <= (others => '0');
      leader_offset_meta   <= (others => '0'); leader_offset_sync   <= (others => '0');
      ppb_meter_meta       <= (others => '0'); ppb_meter_sync       <= (others => '0');
    elsif rising_edge(clk_sys_i) then
      -- 1-bit flags
      ppb_valid_meta       <= pll_ppb_measurement_valid_i;
      ppb_valid_sync       <= ppb_valid_meta;
      wc_locked_meta       <= wallclock_locked_i;
      wc_locked_sync       <= wc_locked_meta;
      wc_phasejump_meta    <= wallclock_phasejump_i;
      wc_phasejump_sync    <= wc_phasejump_meta;
      wc_configured_meta   <= wallclock_configured_i;
      wc_configured_sync   <= wc_configured_meta;
      ptp_leader_lost_meta <= ptp_leader_lost_i;
      ptp_leader_lost_sync <= ptp_leader_lost_meta;
      eth_link_up_meta     <= eth_link_up_i;
      eth_link_up_sync     <= eth_link_up_meta;
      eth_speed_meta       <= eth_link_speed_i;
      eth_speed_sync       <= eth_speed_meta;
      -- 32-bit values (slow-changing, per-bit 2-FF is sufficient)
      path_delay_meta      <= path_delay_i;
      path_delay_sync      <= path_delay_meta;
      leader_offset_meta   <= leader_offset_i;
      leader_offset_sync   <= leader_offset_meta;
      ppb_meter_meta       <= clock_ppb_meter_i;
      ppb_meter_sync       <= ppb_meter_meta;
    end if;
  end process p_status_cdc;

  --------------------------------------------------------------------
  -- Write capture on NWE rising edge.
  -- FMC spec guarantees address and data are stable at this point.
  -- wr_cap_addr / wr_cap_data stay constant until the NEXT write
  -- cycle (~75 ns later), so they are rock-stable by the time the
  -- toggle-CDC delivers wr_pending in the clk_sys domain (~12 ns).
  --------------------------------------------------------------------
  process(fmc_nwe_n_i, rst_sys_i)
  begin
    if rst_sys_i = '1' then
      wr_cap_addr   <= (others => '0');
      wr_cap_data   <= (others => '0');
      wr_cap_toggle <= '0';
    elsif rising_edge(fmc_nwe_n_i) then
      if fmc_ne_n_i = '0' then
        wr_cap_addr   <= unsigned(fmc_addr_i);
        wr_cap_data   <= fmc_data_in;      -- use the dedicated input alias
        wr_cap_toggle <= not wr_cap_toggle;
      end if;
    end if;
  end process;

  --------------------------------------------------------------------
  -- FMC bus handling (byte accesses, lower byte used)
  --------------------------------------------------------------------
  process(clk_sys_i, rst_sys_i)
    variable addr_v    : unsigned(6 downto 0);
    variable v_rd_byte : unsigned(1 downto 0);
  begin
    if rst_sys_i = '1' then
      txfifo_wr_en    <= '0';
      rxfifo_rd_en    <= '0';
      reg_tx_len      <= (others => '0');
      reg_tx_start    <= '0';
      reg_tx_start_toggle <= '0';
      reg_tx_len_reset <= '0';
      tx_busy_sys     <= '0';
      rx_clear_req_sys   <= '0';
      fmc_rx_bytes_sent <= (others => '0');
      fmc_data_out    <= (others => '0');
      fmc_data_oe     <= '0';
      fmc_read_latched <= '0';
      fmc_read_addr_lat <= (others => '0');
      fmc_read_data_lat <= (others => '0');
      system_config_wr_en <= '0';
      system_config_wr_addr <= (others => '0');
      system_config_wr_data <= (others => '0');
      system_config_done_o <= '0';
      ppb_start_reg         <= '0';
      stream_cfg_byte_count <= (others => '0');
      stream_cfg_base_addr  <= (others => '0');
      tx_stream_config_wr_en_o   <= '0';
      tx_stream_config_wr_addr_o <= (others => '0');
      tx_stream_config_wr_data_o <= (others => '0');
      rx_stream_cfg_byte_count   <= (others => '0');
      rx_stream_cfg_base_addr    <= (others => '0');
      rx_stream_cfg_wr_en        <= '0';
      rx_stream_config_wr_addr_o <= (others => '0');
      rx_stream_config_wr_data_o <= (others => '0');
      read_byte_count       <= (others => '0');
      read_reg_addr_prev    <= (others => '0');
    elsif rising_edge(clk_sys_i) then
      system_config_wr_en <= '0';
      tx_stream_config_wr_en_o <= '0';
      rx_stream_cfg_wr_en <= '0';
      txfifo_wr_en <= '0';
      rxfifo_rd_en <= '0';
      fmc_data_out <= fmc_read_data_lat;
      fmc_data_oe  <= '0';


      -- TX busy: cleared when MAC side signals completion via toggle CDC
      if tx_clear_sys_pulse = '1' then
        tx_busy_sys <= '0';
      end if;

      -- PPB measurement start handshake:
      -- start_o stays high until valid_i goes low (measurement accepted)
      if ppb_start_reg = '1' and ppb_valid_sync = '0' then
        ppb_start_reg <= '0';
      end if;
      -- Clear request once MAC side acknowledged
      if rx_clear_sys_pulse = '1' then
        rx_clear_req_sys <= '0';
      end if;

      -- Drop read latch when NOE deasserts or chip deselects (raw levels to avoid phase races)
      if (fmc_noe_n_i = '1') or (fmc_ne_n_i = '1') then
        fmc_read_latched <= '0';
      end if;

      -- Process write when toggle-CDC signals a completed FMC write cycle.
      -- wr_cap_addr / wr_cap_data were captured on NWE rising edge and are
      -- guaranteed stable (next write is at least ~75 ns away).
      if wr_pending = '1' then
        case wr_cap_addr is
          when "0000000" =>  -- 0x00 TX_LEN low
            reg_tx_len(7 downto 0) <= unsigned(wr_cap_data);
            reg_tx_len_reset <= '1';

            register_byte_count <= (others => '0');
          when "0000001" =>  -- 0x01 TX_LEN high
            reg_tx_len(15 downto 8) <= unsigned(wr_cap_data);
            reg_tx_len_reset <= '1';

            register_byte_count <= (others => '0');
          when "0000010" =>  -- 0x02 TX_CTRL
            if wr_cap_data(0) = '1' and reg_tx_start = '0' then
              reg_tx_start_toggle <= not reg_tx_start_toggle;
              tx_busy_sys <= '1';
            end if;
            reg_tx_start <= wr_cap_data(0);

            register_byte_count <= (others => '0');
          when "0100010" =>  -- 0x22 RX_STATUS clear
            if wr_cap_data(0) = '1' then
              rx_clear_req_sys <= '1';
              overflow_interrupt <= '0';
              fmc_rx_bytes_sent   <= (others => '0');
              register_byte_count <= (others => '0');
            end if;
          when "1000000" =>  -- 0x40 Write Register for MAC Address (6 Byte)
                system_config_done_o <= '0'; -- Indicate write in progress
                system_config_wr_en <= '1';
                system_config_wr_addr <= resize(register_byte_count, 11);
                system_config_wr_data <= wr_cap_data;
                if (register_byte_count >= 5) then
                  register_byte_count <= (others => '0');
                  system_config_done_o <= '1'; -- Indicate write done
                else
                  register_byte_count <= register_byte_count + 1;
                end if;
          when "1000001" =>  -- 0x41 Write Register for IP Address  (4 Byte)
                system_config_done_o <= '0'; -- Indicate write in progress
                system_config_wr_en <= '1';
                system_config_wr_addr <= to_unsigned(6, 11) + resize(register_byte_count, 11);
                system_config_wr_data <= wr_cap_data;
                if (register_byte_count >= 3) then
                  register_byte_count <= (others => '0');
                  system_config_done_o <= '1'; -- Indicate write done
                else
                  register_byte_count <= register_byte_count + 1;
                end if;
          when "1000010" =>  -- 0x42 Write Register for Audio Status
            null;
          when "1000011" =>  -- 0x43
            null;

          when "1010000" =>  -- 0x50 Status flag write
            -- Bit[0]: Start PLL PPB Measurement
            if wr_cap_data(0) = '1' then
              ppb_start_reg <= '1';
            end if;
            -- Bit[1]: Reset Wallclock
            reset_wallclock_o <= wr_cap_data(1);
            -- Bit[2]: Reset PTP
            reset_ptp_o <= wr_cap_data(2);
            
            -- Bit[3]: Reset Ethernet
            reset_ethernet_o <= wr_cap_data(3);
            
            -- Bit[4]: PTP Is leader
            ptp_is_leader_o <= wr_cap_data(4);

            -- Bit[5]: PTP Is follower
            ptp_is_follower_o <= wr_cap_data(5);
            
            register_byte_count <= (others => '0');
          when "1010101" =>  -- 0x55 PTP config write (11 bytes, starts at RAM 0x10)
              system_config_done_o <= '0'; -- Indicate write in progress
              system_config_wr_en <= '1';
              system_config_wr_addr <= to_unsigned(10, 11) + resize(register_byte_count, 11);
              system_config_wr_data <= wr_cap_data;
              if (register_byte_count >= 11) then
                register_byte_count <= (others => '0');
                system_config_done_o <= '1'; -- Indicate write done
              else
                register_byte_count <= register_byte_count + 1;
              end if;
            null;
          when "1011000" =>  -- 0x58 TX stream config write (20 bytes per stream -> tx_router config RAM)
              -- Byte 0 is stream_id: latch base address = stream_id * 32
              -- Bytes 0..19 are written to config_ram[base + byte_offset]
              tx_stream_config_wr_en_o <= '1';
              tx_stream_config_wr_data_o <= wr_cap_data;
              if stream_cfg_byte_count = 0 then
                -- First byte: stream_id -> compute base addr and write to offset 0
                stream_cfg_base_addr <= unsigned(wr_cap_data(2 downto 0)) & "00000";
                tx_stream_config_wr_addr_o <= std_ulogic_vector(unsigned(wr_cap_data(2 downto 0)) & "00000");
              else
                -- Subsequent bytes: use latched base addr + offset
                tx_stream_config_wr_addr_o <= std_ulogic_vector(stream_cfg_base_addr + resize(stream_cfg_byte_count, 8));
              end if;
              if stream_cfg_byte_count >= 19 then
                stream_cfg_byte_count <= (others => '0');
              else
                stream_cfg_byte_count <= stream_cfg_byte_count + 1;
              end if;
          when "1011001" =>  -- 0x59 RX stream config write (16 bytes per stream -> rx_ringbuffer stream_ram)
              -- Byte 0 is the base address in stream_ram (caller computes stream_id * 32)
              -- Bytes 1..15 are the actual config data written to stream_ram[base + 0..14]
              rx_stream_config_wr_data_o <= wr_cap_data;
              if rx_stream_cfg_byte_count = 0 then
                -- First byte: base address only, do NOT write to stream_ram
                rx_stream_cfg_base_addr <= unsigned(wr_cap_data);
              else
                -- Subsequent bytes: actual config data at base addr + (byte_count - 1)
                rx_stream_cfg_wr_en <= '1';
                rx_stream_config_wr_addr_o <= std_ulogic_vector(rx_stream_cfg_base_addr + resize(rx_stream_cfg_byte_count - 1, 8));
              end if;
              if rx_stream_cfg_byte_count >= 15 then
                rx_stream_cfg_byte_count <= (others => '0');
              else
                rx_stream_cfg_byte_count <= rx_stream_cfg_byte_count + 1;
              end if;
          when others =>
            if wr_cap_addr >= 16#10# and wr_cap_addr < 16#20# then
              if txfifo_wr_full = '0' then
                txfifo_wr_data <= wr_cap_data;
                txfifo_wr_en   <= '1';
              end if;
            end if;
        end case;
      end if;

      -- Latch address and prepare read data when NOE is low and not yet latched.
      -- We still use the synchronized NOE level, but do not require a clean falling edge,
      -- so a simultaneous NE/NOE assertion will still produce data in the next clk_sys_i.
      if (fmc_ne_qual_low = '1')and (fmc_noe_n_sync = '1') and (fmc_noe_n_meta = '0') then
        -- Use the most recent address (1-stage synced) to handle NE/NOE asserting together.
        --addr_v := unsigned(fmc_addr_meta);
        --fmc_read_addr_lat <= addr_v;
        fmc_read_data_lat <= (others => '0');

        -- Determine byte position for multi-byte sequential registers
        if fmc_addr_meta /= read_reg_addr_prev then
          v_rd_byte := "00";
        else
          v_rd_byte := read_byte_count;
        end if;

        case fmc_addr_meta is
          when "0000010" => -- 0x02 Read: TX status
            fmc_read_data_lat(0) <= tx_busy_sys;

          when "0100000" => fmc_read_data_lat <= std_ulogic_vector(reg_rx_len_sys(7 downto 0)); -- 0x20
          when "0100001" => fmc_read_data_lat <= std_ulogic_vector(reg_rx_len_sys(15 downto 8)); -- 0x21
          when "0100010" => -- 0x22
            fmc_read_data_lat(0) <= reg_rx_ready_sys;
            fmc_read_data_lat(1) <= reg_rx_overflow_sys;

          when "1010000" => -- 0x50 Read: Clocking flags
            fmc_read_data_lat(0) <= ppb_valid_sync;
            fmc_read_data_lat(1) <= wc_locked_sync;
            fmc_read_data_lat(2) <= wc_phasejump_sync;
            fmc_read_data_lat(3) <= wc_configured_sync;
            fmc_read_data_lat(4) <= ptp_leader_lost_sync;

          when "1010001" => -- 0x51 Read: Ethernet flags
            fmc_read_data_lat(0) <= eth_link_up_sync;
            fmc_read_data_lat(1) <= eth_speed_sync(0);
            fmc_read_data_lat(2) <= eth_speed_sync(1);

          when "1010010" => -- 0x52 Read: Path Delay (4-byte sequential)
            case v_rd_byte is
              when "00"   => fmc_read_data_lat <= path_delay_sync(7 downto 0);
              when "01"   => fmc_read_data_lat <= path_delay_sync(15 downto 8);
              when "10"   => fmc_read_data_lat <= path_delay_sync(23 downto 16);
              when others => fmc_read_data_lat <= path_delay_sync(31 downto 24);
            end case;
            read_byte_count <= v_rd_byte + 1;

          when "1010011" => -- 0x53 Read: Leader Offset (4-byte sequential)
            case v_rd_byte is
              when "00"   => fmc_read_data_lat <= leader_offset_sync(7 downto 0);
              when "01"   => fmc_read_data_lat <= leader_offset_sync(15 downto 8);
              when "10"   => fmc_read_data_lat <= leader_offset_sync(23 downto 16);
              when others => fmc_read_data_lat <= leader_offset_sync(31 downto 24);
            end case;
            read_byte_count <= v_rd_byte + 1;

          when "1010100" => -- 0x54 Read: PLL PPB meter (4-byte, zeros if not valid)
            if ppb_valid_sync = '1' then
              case v_rd_byte is
                when "00"   => fmc_read_data_lat <= ppb_meter_sync(7 downto 0);
                when "01"   => fmc_read_data_lat <= ppb_meter_sync(15 downto 8);
                when "10"   => fmc_read_data_lat <= ppb_meter_sync(23 downto 16);
                when others => fmc_read_data_lat <= ppb_meter_sync(31 downto 24);
              end case;
            end if;
            -- fmc_read_data_lat stays zeros when ppb_valid_sync = '0'
            read_byte_count <= v_rd_byte + 1;

          when others =>
            if unsigned(fmc_addr_meta) >= 16#30# and unsigned(fmc_addr_meta) < 16#40# then
              if rxfifo_rd_empty = '0' then
                fmc_read_data_lat <= rxfifo_rd_data;
                rxfifo_rd_en <= '1';
                fmc_rx_bytes_sent <= fmc_rx_bytes_sent + 1;
              end if;
            elsif unsigned(fmc_addr_meta) >= 16#60# then
              -- 0x60..0x7F: Direct config RAM read (address - 0x60)
              fmc_read_data_lat <= system_config_rd_data;
            end if;
        end case;

        read_reg_addr_prev <= fmc_addr_meta;
        fmc_read_latched <= '1';
      end if;

      -- Drive bus during read while latched and external strobes indicate read
      if (fmc_read_latched = '1') and (fmc_ne_qual_low = '1') and (fmc_noe_n_sync = '0') and (fmc_nwe_n_sync = '1') then
        fmc_data_out <= fmc_read_data_lat;
        fmc_data_oe  <= '1';
      end if;

      if reg_tx_len_reset = '1' then
        reg_tx_len_reset <= '0';
      end if;
    end if;
  end process;

  --------------------------------------------------------------------
  -- TX start sync to MAC clock
  --------------------------------------------------------------------
  process(mac_tx_clock_i, mac_tx_reset_i)
  begin
    if mac_tx_reset_i = '1' then
      tx_start_sync_1 <= '0';
      tx_start_sync_2 <= '0';
      tx_start_pulse  <= '0';
    elsif rising_edge(mac_tx_clock_i) then
      tx_start_sync_1 <= reg_tx_start_toggle;
      tx_start_sync_2 <= tx_start_sync_1;
      tx_start_pulse  <= tx_start_sync_1 xor tx_start_sync_2;
    end if;
  end process;

  --------------------------------------------------------------------
  -- TX MAC state machine (derived from spi_ethernet_client)
  --------------------------------------------------------------------
  process(mac_tx_clock_i, mac_tx_reset_i)
  begin
    if mac_tx_reset_i = '1' then
      tx_clear_toggle_mac <= '0';
      mac_tx_active   <= '0';
      mac_tx_enable_o <= '0';
      mac_tx_data_o   <= (others => '0');
      txfifo_rd_en    <= '0';
      tx_prefetch_valid <= '0';
      mac_tx_preamble_bytes_sent <= (others => '0');
      tx_byte_sent_count <= (others => '0');
    elsif rising_edge(mac_tx_clock_i) then
      txfifo_rd_en <= '0';
      case sm_tx_ethernet is
        when s_Idle =>
          mac_tx_data_o   <= (others => '0');
          mac_tx_enable_o <= '0';
          tx_allow_req_o <= '0';
          if (tx_start_pulse = '1') and (mac_tx_active = '0') then
            if txfifo_rd_empty = '0' then
              sm_tx_ethernet <= s_waitForAllow;
              tx_allow_req_o <= '1';
            end if;
          end if;
        when s_waitForAllow =>
          if tx_allow_i = '1' then
            if txfifo_rd_empty = '0' then
              sm_tx_ethernet <= s_PrimeTx;
              mac_tx_data_o <= txfifo_rd_data;
              mac_tx_enable_o <= '1';
              tx_prefetch_valid <= '1';
            else
              -- FIFO empty (e.g. after reset glitch) — abort back to idle
              sm_tx_ethernet <= s_Idle;
              tx_allow_req_o <= '0';
            end if;
          end if;
        when s_PrimeTx =>
          
          mac_tx_preamble_bytes_sent <= mac_tx_preamble_bytes_sent + 1;
          mac_tx_active <= '1';
          tx_prefetch_valid <= '1';
          if (mac_tx_preamble_bytes_sent = 12) then
            txfifo_rd_en <= '1';
          end if;
          if mac_tx_preamble_bytes_sent = 14 then
            if txfifo_rd_en = '0' then
              sm_tx_ethernet <= s_Transmit;
              txfifo_rd_en <= '1';
              mac_tx_preamble_bytes_sent <= (others => '0');
            end if;
          end if;
          mac_tx_data_o <= txfifo_rd_data;
          
        when s_Transmit =>
          mac_tx_enable_o <= '1';
          if mac_tx_byte_sent_i = '1' then
            if tx_prefetch_valid = '1' then
              mac_tx_data_o <= txfifo_rd_data;
              tx_byte_sent_count <= tx_byte_sent_count + 1;
            end if;
            if txfifo_rd_empty = '0' then
              txfifo_rd_en <= '1';
              tx_prefetch_valid <= '0';
            else
              if (tx_byte_sent_count + 1) >= reg_tx_len then
                sm_tx_ethernet <= s_End;
              end if;
            end if;
          end if;
          if txfifo_rd_en = '1' then
            tx_prefetch_valid <= '1';
          end if;

        when s_End =>
          mac_tx_enable_o <= '0';
          mac_tx_data_o   <= (others => '0');
          mac_tx_active   <= '0';
          sm_tx_ethernet  <= s_Idle;
          tx_prefetch_valid <= '0';
          tx_byte_sent_count <= (others => '0');
          tx_clear_toggle_mac <= not tx_clear_toggle_mac;
      end case;
    end if;
  end process;

  --------------------------------------------------------------------
  -- TX FIFO clear sync
  --------------------------------------------------------------------
  process(mac_tx_clock_i, mac_tx_reset_i)
  begin
    if mac_tx_reset_i = '1' then
      tx_clear_mac_d     <= '0';
      tx_clear_mac_pulse <= '0';
    elsif rising_edge(mac_tx_clock_i) then
      tx_clear_mac_pulse <= tx_clear_toggle_mac xor tx_clear_mac_d;
      tx_clear_mac_d     <= tx_clear_toggle_mac;
    end if;
  end process;

  --------------------------------------------------------------------
  -- RX path
  --------------------------------------------------------------------
  process(mac_rx_clock_i, mac_rx_reset_i)
    variable len_next    : unsigned(reg_rx_len'range);
    variable ready_next  : std_ulogic;
    variable ovf_next    : std_ulogic;
  begin
    if mac_rx_reset_i = '1' then
      rxfifo_wr_en      <= '0';
      rxfifo_wr_data    <= (others => '0');
      reg_rx_len        <= (others => '0');
      reg_rx_ready      <= '0';
      reg_rx_overflow   <= '0';
      mac_rx_frame_d    <= '0';
      rx_clear_req_sync1_mac <= '0';
      rx_clear_req_sync2_mac <= '0';
      rx_clear_req_mac_d     <= '0';
      rx_clear_ack_mac       <= '0';
      rx_clear_mac_pulse     <= '0';
    elsif rising_edge(mac_rx_clock_i) then
      rxfifo_wr_en <= '0';
      rx_clear_req_sync1_mac <= rx_clear_req_sys;
      rx_clear_req_sync2_mac <= rx_clear_req_sync1_mac;
      rx_clear_req_mac_d     <= rx_clear_req_sync2_mac;
      rx_clear_mac_pulse     <= rx_clear_req_sync2_mac and not rx_clear_req_mac_d;
      rx_clear_ack_mac       <= rx_clear_req_sync2_mac;
      mac_rx_frame_d  <= mac_rx_frame_i;

      len_next   := reg_rx_len;
      ready_next := reg_rx_ready;
      ovf_next   := reg_rx_overflow;

      if rx_clear_mac_pulse = '1' then
        len_next   := (others => '0');
        ready_next := '0';
        ovf_next   := '0';
      end if;

      if (mac_rx_frame_i = '1') and (mac_rx_frame_d = '0') then
        len_next   := (others => '0');
        ready_next := '0';
        if reg_rx_ready = '1' then
          ovf_next := '1';
        else
          ovf_next := '0';
        end if;
      end if;

      if ready_next = '0' then
        if (mac_rx_frame_i = '1') and (mac_rx_byte_rcv_i = '1') then
          debug_rx_byte_cnt <= debug_rx_byte_cnt + 1;
          if rxfifo_wr_full = '0' then
            rxfifo_wr_data <= mac_rx_data_i;
            rxfifo_wr_en   <= '1';
            len_next       := len_next + 1;
          else
            ovf_next := '1';
          end if;
        end if;

        if (mac_rx_frame_i = '0') and (mac_rx_frame_d = '1') then
          if len_next /= 0 then
            ready_next := '1';
          end if;
        end if;
      end if;

      reg_rx_len      <= len_next;
      reg_rx_ready    <= ready_next;
      reg_rx_overflow <= ovf_next;
    end if;
  end process;

  -- All signals on fmc_int_o must be in the clk_sys domain to avoid
  -- metastability glitches that can cause the MCU to miss edge-triggered IRQs.
  fmc_int_o <= reg_rx_ready_sys or reg_rx_overflow_sys or overflow_interrupt;

end architecture;
