library ieee;
use ieee.std_logic_1164.all;
use IEEE.NUMERIC_STD.all;
use work.miim_types.all;

use work.audioclks_pkg.all;
use work.wallclock_signals_pkg.all;
entity aes67_wb_bridge is
  generic (
		MII_WIDTH : integer := 2;
		ETHERNET_TYPE : string := "RMII";
		SYS_CLK_NS_PER_TICK : integer := 8; -- 125 MHz
        MII_CLK_NS_PER_TICK : integer := 20; -- 50 MHz
		TX_MAX_STREAMS : natural := 8;
		RX_MAX_STREAMS : natural := 8;
		RX_CHANNELS		: natural := 16;
		TX_CHANNELS		: natural := 16; -- must be multiple of two for i2s, multiple of 8 for tdm8
		RX_BYTE_DEPTH	: natural := 3;
		TX_BYTE_DEPTH	: natural := 3;
		RX_SAMPLE_BUFFER_DEPTH : natural := 256;
		TX_SAMPLE_BUFFER_DEPTH : natural := 64; -- must be power of two (media-clock-derived TX write pointer)
		  
    	MIIM_CLOCK_DIVIDER : POSITIVE := 50;

    	MIIM_PHY_ADDRESS      : t_phy_address := (others => '0');
		

		
		AUDIO_RX_USE_PARALLEL_INTERFACE : boolean := false;
		AUDIO_RX_TDM_OUTPUTS : natural := 2;

		AUDIO_TX_USE_PARALLEL_INTERFACE : boolean := false;
		AUDIO_TX_TDM_INPUTS : natural := 2;

		USE_EXTERNAL_PLL : BOOLEAN := true;
		ENABLE_METERING: BOOLEAN := true;
    PTP_MOVING_AVERAGE_DEPTH : INTEGER := 8;
    PTP_IN_SOFTWARE : BOOLEAN := false;
    PHY_TYPE : STRING := "UNKNOWN";
    AUDIO_TDM_CONFIG : t_audio_clock_cfg

	);
	PORT
	(
		-- system clocks
		sys_clk_125MHz_i :  IN  STD_LOGIC;
		enet_clk_i       :  IN  STD_LOGIC;
		clk_mcu_i	   :  IN  STD_LOGIC;
		rst_n		: IN STD_LOGIC;

		
    -- raw mii/rmii/rgmii interface for eth timestamping
    phy_mii_rx_clk_in : IN STD_LOGIC;
		phy_mii_tx_clk_in : IN STD_LOGIC;
		phy_mii_tx_data_in : IN STD_LOGIC_VECTOR(MII_WIDTH - 1 downto 0);
		phy_mii_rx_data_in : IN STD_LOGIC_VECTOR(MII_WIDTH - 1 downto 0);
		phy_mii_tx_en_i : IN STD_LOGIC;
		phy_mii_rx_en_i : IN STD_LOGIC;
		
    -- to mii/gmii converted interface for mac
    	mii_rx_clock_i : IN STD_LOGIC;
    	mii_tx_clock_i : IN STD_LOGIC;
    	mii_rx_err_i : IN STD_LOGIC;
    	mii_rx_dv_i : IN STD_LOGIC;
    	mii_rxd_i : IN STD_LOGIC_VECTOR(7 downto 0);

    	mii_tx_err_o : OUT STD_LOGIC;
    	mii_tx_en_o : OUT STD_LOGIC;
    	mii_txd_o : OUT std_logic_vector(7 downto 0);

    	enet_mdio : INOUT std_logic;
    	enet_mdc : OUT std_logic;



		-- audio clocks

		pll_512fs_i : IN STD_LOGIC;
    audioclocks_o: OUT t_audio_clocks;
    selected_audio_clock_o : OUT t_audio_clocks_selected;



-- wishbone bus
      aes67_wb_ack                              : out std_logic;
      aes67_wb_adr                              : in std_logic_vector(29 downto 0);
      aes67_wb_bte                              : in std_logic_vector(1 downto 0);
      aes67_wb_cti                              : in std_logic_vector(2 downto 0);
      aes67_wb_cyc                              : in std_logic;
      aes67_wb_dat_r                            : out std_logic_vector(31 downto 0);
      aes67_wb_dat_w                            : in std_logic_vector(31 downto 0);
      aes67_wb_err                              : out std_logic;
      aes67_wb_sel                              : in std_logic_vector(3 downto 0);
      aes67_wb_stb                              : in std_logic;
      aes67_wb_we                               : in std_logic;
      eth_irq_o                           : out std_logic;
		-- audio outputs
		tdm8out_o : OUT STD_LOGIC_VECTOR(AUDIO_RX_TDM_OUTPUTS - 1 downto 0);
		rx_sample_register : OUT STD_LOGIC_VECTOR((RX_BYTE_DEPTH * 8) * RX_CHANNELS - 1 downto 0);
		

		-- audio inputs
		tdm8in_i : IN STD_LOGIC_VECTOR(AUDIO_TX_TDM_INPUTS - 1 downto 0);
		tx_sample_register : IN STD_LOGIC_VECTOR((TX_BYTE_DEPTH * 8) * TX_CHANNELS - 1 downto 0);

    dbg_mac_tx_clk_o : out std_logic

	);
end entity;

architecture rtl of aes67_wb_bridge is

component litex_soc_aes67_bridge
  port (
    aes67_ctrl_adda_nrst : out std_logic;
    aes67_ctrl_eth_link_up : in std_logic;
    aes67_ctrl_eth_reset : out std_logic;
    aes67_ctrl_eth_rx_overflow : in std_logic;
    aes67_ctrl_eth_speed : in std_logic_vector     (1 downto 0);
    aes67_ctrl_eth_tx_done : in std_logic;
    aes67_ctrl_eth_tx_request : out std_logic;
    aes67_ctrl_ip_addr : out std_logic_vector    (31 downto 0);
    aes67_ctrl_mac_addr : out std_logic_vector    (47 downto 0);
    aes67_ctrl_meter_clear : out std_logic;
    aes67_ctrl_parser_delay_asymmetry_ns : out std_logic_vector    (31 downto 0);
    aes67_ctrl_parser_min_filter_active_depth : out std_logic_vector     (7 downto 0);
    aes67_ctrl_parser_min_filter_enable : out std_logic;
    aes67_ctrl_pll_ppb_pll_count : in std_logic_vector    (24 downto 0);
    aes67_ctrl_pll_ppb_start : out std_logic;
    aes67_ctrl_pll_ppb_valid : in std_logic;
    aes67_ctrl_pll_ppb_wc_count : in std_logic_vector    (24 downto 0);
    aes67_ctrl_ptp_announce_msg_interval : out std_logic_vector     (7 downto 0);
    aes67_ctrl_ptp_gm_clock_accuracy : out std_logic_vector     (7 downto 0);
    aes67_ctrl_ptp_gm_clock_class : out std_logic_vector     (7 downto 0);
    aes67_ctrl_ptp_gm_priority1 : out std_logic_vector     (7 downto 0);
    aes67_ctrl_ptp_gm_priority2 : out std_logic_vector     (7 downto 0);
    aes67_ctrl_ptp_is_follower : in std_logic;
    aes67_ctrl_ptp_is_leader : in std_logic;
    aes67_ctrl_ptp_leader_id : in std_logic_vector    (63 downto 0);
    aes67_ctrl_ptp_log_msg_interval : out std_logic_vector     (7 downto 0);
    aes67_ctrl_ptp_offset : in std_logic_vector    (31 downto 0);
    aes67_ctrl_ptp_path_delay : in std_logic_vector    (31 downto 0);
    aes67_ctrl_ptp_reset : out std_logic;
    aes67_ctrl_ptp_time_source : out std_logic_vector     (7 downto 0);
    aes67_ctrl_rx_meter_clip : in std_logic_vector    (15 downto 0);
    aes67_ctrl_rx_meter_signal : in std_logic_vector    (15 downto 0);
    aes67_ctrl_rx_reset : out std_logic;
    aes67_ctrl_servo_filter_shift : out std_logic_vector     (4 downto 0);
    aes67_ctrl_servo_gain_shift : out std_logic_vector     (4 downto 0);
    aes67_ctrl_servo_gain_shift_locked : out std_logic_vector     (4 downto 0);
    aes67_ctrl_servo_ki_extra_shift : out std_logic_vector     (4 downto 0);
    aes67_ctrl_servo_ki_gain : out std_logic_vector     (7 downto 0);
    aes67_ctrl_servo_kp_gain : out std_logic_vector     (7 downto 0);
    aes67_ctrl_servo_lock_count_threshold : out std_logic_vector     (7 downto 0);
    aes67_ctrl_servo_lock_threshold_ns : out std_logic_vector    (31 downto 0);
    aes67_ctrl_servo_mon_effective_gain_shift : in std_logic_vector     (7 downto 0);
    aes67_ctrl_servo_mon_filtered_offset : in std_logic_vector    (31 downto 0);
    aes67_ctrl_servo_mon_first_lock_achieved : in std_logic;
    aes67_ctrl_servo_mon_integral_sum : in std_logic_vector    (31 downto 0);
    aes67_ctrl_servo_mon_lock_counter : in std_logic_vector    (15 downto 0);
    aes67_ctrl_servo_mon_pi_proportional : in std_logic_vector    (31 downto 0);
    aes67_ctrl_servo_mon_pi_sum_raw : in std_logic_vector    (31 downto 0);
    aes67_ctrl_servo_mon_sample_count : in std_logic_vector    (15 downto 0);
    aes67_ctrl_servo_unlock_threshold_ns : out std_logic_vector    (31 downto 0);
    aes67_ctrl_servo_warmup_samples : out std_logic_vector     (7 downto 0);
    aes67_ctrl_tx_meter_clip : in std_logic_vector    (15 downto 0);
    aes67_ctrl_tx_meter_signal : in std_logic_vector    (15 downto 0);
    aes67_ctrl_tx_reset : out std_logic;
    aes67_ctrl_tx_timestamp_nsec_in : in std_logic_vector    (29 downto 0);
    aes67_ctrl_tx_timestamp_sec_in : in std_logic_vector     (3 downto 0);
    aes67_ctrl_wallclock_configured : in std_logic;
    aes67_ctrl_wallclock_locked : in std_logic;
    aes67_ctrl_wallclock_nanoseconds_in : in std_logic_vector    (29 downto 0);
    aes67_ctrl_wallclock_nanoseconds_out : out std_logic_vector    (29 downto 0);
    aes67_ctrl_wallclock_phasejump : out std_logic;
    aes67_ctrl_wallclock_ppb : out std_logic_vector    (19 downto 0);
    aes67_ctrl_wallclock_seconds_in : in std_logic_vector    (47 downto 0);
    aes67_ctrl_wallclock_seconds_out : out std_logic_vector    (47 downto 0);
    aes67_ctrl_wallclock_set : out std_logic;
    aes67_wb_ack : out std_logic;
    aes67_wb_adr : in std_logic_vector    (29 downto 0);
    aes67_wb_bte : in std_logic_vector     (1 downto 0);
    aes67_wb_cti : in std_logic_vector     (2 downto 0);
    aes67_wb_cyc : in std_logic;
    aes67_wb_dat_r : out std_logic_vector    (31 downto 0);
    aes67_wb_dat_w : in std_logic_vector    (31 downto 0);
    aes67_wb_err : out std_logic;
    aes67_wb_sel : in std_logic_vector     (3 downto 0);
    aes67_wb_stb : in std_logic;
    aes67_wb_we : in std_logic;
    clk_mac_rx : in std_logic;
    clk_mac_tx : in std_logic;
    clk_sys : in std_logic;
    eth_buf_irq : out std_logic;
    eth_buf_rx_ack : out std_logic;
    eth_buf_rx_addr : in std_logic_vector    (10 downto 0);
    eth_buf_rx_data : in std_logic_vector     (7 downto 0);
    eth_buf_rx_len : in std_logic_vector    (10 downto 0);
    eth_buf_rx_valid : in std_logic;
    eth_buf_rx_we : in std_logic;
    eth_buf_tx_addr : in std_logic_vector    (10 downto 0);
    eth_buf_tx_data : out std_logic_vector     (7 downto 0);
    eth_buf_tx_len : out std_logic_vector    (10 downto 0);
    rx_stream_cfg_wr_addr : out std_logic_vector     (7 downto 0);
    rx_stream_cfg_wr_data : out std_logic_vector     (7 downto 0);
    rx_stream_cfg_wr_en : out std_logic;
    tx_stream_cfg_wr_addr : out std_logic_vector     (7 downto 0);
    tx_stream_cfg_wr_data : out std_logic_vector     (7 downto 0);
    tx_stream_cfg_wr_en : out std_logic
  );
end component;


signal mac_address_i : STD_LOGIC_VECTOR(47 downto 0);
signal ip_address_i : STD_LOGIC_VECTOR(31 downto 0);

signal ptp_announce_interval_i : STD_LOGIC_VECTOR(7 downto 0);
signal ptp_clock_accuracy_i : STD_LOGIC_VECTOR(7 downto 0);
signal ptp_clock_class_i : STD_LOGIC_VECTOR(7 downto 0);
signal ptp_gm_prioone_i : STD_LOGIC_VECTOR(7 downto 0);
signal ptp_gm_priotwo_i : STD_LOGIC_VECTOR(7 downto 0);
signal ptp_current_leader_id_o : STD_LOGIC_VECTOR(63 downto 0);
signal ptp_log_message_interval_i : STD_LOGIC_VECTOR(7 downto 0);
signal ptp_time_source_i : STD_LOGIC_VECTOR(7 downto 0);
signal ptp_is_follower_o : STD_LOGIC;
signal ptp_is_leader_o : STD_LOGIC;
signal ptp_mean_path_delay_o : STD_LOGIC_VECTOR(31 downto 0);
signal ptp_offset_from_master_o : STD_LOGIC_VECTOR(31 downto 0);
signal wallclock_locked_o : STD_LOGIC;
signal wallclock_configured_o : STD_LOGIC;

signal ptp_reset_i : STD_LOGIC := '0';

signal servo_kp_gain_i              : STD_LOGIC_VECTOR(7 downto 0)  := std_logic_vector(to_signed(40, 8));
signal servo_ki_gain_i              : STD_LOGIC_VECTOR(7 downto 0)  := std_logic_vector(to_signed(5, 8));
signal servo_gain_shift_i           : STD_LOGIC_VECTOR(4 downto 0)  := std_logic_vector(to_unsigned(3, 5));
signal servo_gain_shift_locked_i    : STD_LOGIC_VECTOR(4 downto 0)  := (others => '0');
signal servo_ki_extra_shift_i       : STD_LOGIC_VECTOR(4 downto 0)  := std_logic_vector(to_unsigned(3, 5));
signal servo_filter_shift_i         : STD_LOGIC_VECTOR(4 downto 0)  := (others => '0');
signal servo_warmup_samples_i       : STD_LOGIC_VECTOR(7 downto 0)  := std_logic_vector(to_unsigned(16, 8));
signal servo_lock_threshold_ns_i    : STD_LOGIC_VECTOR(31 downto 0) := std_logic_vector(to_unsigned(500, 32));
signal servo_unlock_threshold_ns_i  : STD_LOGIC_VECTOR(31 downto 0) := std_logic_vector(to_unsigned(5000, 32));
signal servo_lock_count_threshold_i : STD_LOGIC_VECTOR(7 downto 0)  := std_logic_vector(to_unsigned(24, 8));
signal parser_delay_asymmetry_ns_i       : STD_LOGIC_VECTOR(31 downto 0) := (others => '0');

signal servo_mon_filtered_offset_o      : STD_LOGIC_VECTOR(31 downto 0);
signal servo_mon_integral_sum_o         : STD_LOGIC_VECTOR(31 downto 0);
signal servo_mon_pi_proportional_o      : STD_LOGIC_VECTOR(31 downto 0);
signal servo_mon_pi_sum_raw_o           : STD_LOGIC_VECTOR(31 downto 0);
signal servo_mon_effective_gain_shift_o : STD_LOGIC_VECTOR(7 downto 0);
signal servo_mon_lock_counter_o         : STD_LOGIC_VECTOR(15 downto 0);
signal servo_mon_sample_count_o         : STD_LOGIC_VECTOR(15 downto 0);
signal servo_mon_first_lock_achieved_o  : STD_LOGIC;

signal ppb_meter_start_i : STD_LOGIC;
signal pll_meas_valid_o : STD_LOGIC;
signal pll_counter_o : unsigned(24 downto 0);
signal wc_counter_o : unsigned(24 downto 0);

signal audio_meter_clear_i : STD_LOGIC;
signal audio_meter_rx_clip_o : STD_LOGIC_VECTOR(RX_CHANNELS - 1 downto 0);
signal audio_meter_rx_signal_o : STD_LOGIC_VECTOR(RX_CHANNELS - 1 downto 0);
signal audio_meter_tx_clip_o : STD_LOGIC_VECTOR(TX_CHANNELS - 1 downto 0);
signal audio_meter_tx_signal_o : STD_LOGIC_VECTOR(TX_CHANNELS - 1 downto 0);

signal audio_tx_cfg_wr_en_i : STD_LOGIC;
signal audio_tx_cfg_wr_data_i : STD_LOGIC_VECTOR(7 downto 0);
signal audio_tx_cfg_wr_addr_i : STD_LOGIC_VECTOR(7 downto 0);

signal audio_rx_cfg_wr_en_i : STD_LOGIC;
signal audio_rx_cfg_wr_data_i : STD_LOGIC_VECTOR(7 downto 0);
signal audio_rx_cfg_wr_addr_i : STD_LOGIC_VECTOR(7 downto 0);

signal mac_rx_clk : std_logic;
signal mac_tx_clk : std_logic;
signal mac_tx_rst: std_logic;
signal mac_tx_busy: std_logic;
signal mac_tx_byte_sent: std_logic;
signal mac_speed : STD_LOGIC_VECTOR(1 downto 0);
signal mac_linkup : STD_LOGIC;


signal is_mcu_pkt_tog : std_logic;
signal mcu_tx_en : std_logic;
signal mcu_tx_allow_req : std_logic;
signal mcu_tx_allow : std_logic;
signal mcu_tx_start_prefetch : std_logic;

signal mcu_received_packet_length : unsigned(10 downto 0); 
signal mcu_ethbuf_ram_addr: unsigned(10 downto 0);
-- std_logic_vector view of the eth-ram read address driven by the litex bridge
-- (its eth_ram_addr_o port is std_logic_vector; ethernet_top expects unsigned).
signal mcu_ethbuf_ram_addr_slv: std_logic_vector(10 downto 0);
signal mcu_ethbuf_ram_data: std_logic_vector(7 downto 0);
signal mcu_tx_data : std_logic_vector(7 downto 0);
signal mcu_rx_overflow : std_logic;
signal mcu_tx_done : std_logic;
signal mcu_tx_req : std_logic;


signal mcu_buf_rx_ack : std_logic;
signal mcu_buf_rx_we : std_logic;
signal mcu_buf_rx_valid : std_logic;
signal mcu_buf_tx_len: STD_LOGIC_VECTOR(10 downto 0);
signal mcu_buf_tx_addr : STD_LOGIC_VECTOR(10 downto 0);
signal mcu_buf_tx_data : STD_LOGIC_VECTOR(7 downto 0);

signal mcu_buf_rx_len: STD_LOGIC_VECTOR(10 downto 0);
signal mcu_buf_rx_addr : STD_LOGIC_VECTOR(10 downto 0);
signal mcu_buf_rx_data : STD_LOGIC_VECTOR(7 downto 0);

signal audiorx_reset : STD_LOGIC;
signal audiotx_reset : STD_LOGIC;
signal mac_resetn : STD_LOGIC;

signal mac_reset : STD_LOGIC;

signal wallclock_signals : t_wallclock_signals;
signal timestamps : t_eth_timestamps;
signal aes67_ctrl_wallclock_ppb_reg : STD_LOGIC_VECTOR(19 downto 0);
begin

    -- eth-ram read address: litex bridge drives std_logic_vector, ethernet_top
    -- consumes unsigned.
    dbg_mac_tx_clk_o <= mac_rx_clk;
    mcu_ethbuf_ram_addr <= unsigned(mcu_ethbuf_ram_addr_slv);
    mac_resetn <= not mac_reset;
    litex_soc_aes67_bridge_inst: litex_soc_aes67_bridge
     port map(
        aes67_ctrl_eth_link_up => mac_linkup,
        aes67_ctrl_eth_rx_overflow => mcu_rx_overflow,
        aes67_ctrl_eth_speed => mac_speed,
        aes67_ctrl_eth_tx_done => mcu_tx_done,
        aes67_ctrl_eth_tx_request => mcu_tx_req,
        aes67_ctrl_ip_addr => ip_address_i,
        aes67_ctrl_mac_addr => mac_address_i,
        aes67_ctrl_meter_clear => audio_meter_clear_i,
        aes67_ctrl_parser_delay_asymmetry_ns => parser_delay_asymmetry_ns_i,
        aes67_ctrl_pll_ppb_pll_count => STD_LOGIC_VECTOR(pll_counter_o),
        aes67_ctrl_pll_ppb_start => ppb_meter_start_i,
        aes67_ctrl_pll_ppb_valid => pll_meas_valid_o,
        aes67_ctrl_pll_ppb_wc_count => STD_LOGIC_VECTOR(wc_counter_o),
        aes67_ctrl_ptp_announce_msg_interval => ptp_announce_interval_i,
        aes67_ctrl_ptp_gm_clock_accuracy => ptp_clock_accuracy_i,
        aes67_ctrl_ptp_gm_clock_class => ptp_clock_class_i,
        aes67_ctrl_ptp_gm_priority1 => ptp_gm_prioone_i,
        aes67_ctrl_ptp_gm_priority2 => ptp_gm_priotwo_i,
        aes67_ctrl_ptp_is_follower => ptp_is_follower_o,
        aes67_ctrl_ptp_is_leader => ptp_is_leader_o,
        aes67_ctrl_ptp_leader_id => ptp_current_leader_id_o,
        aes67_ctrl_ptp_log_msg_interval => ptp_log_message_interval_i,
        aes67_ctrl_ptp_offset => ptp_offset_from_master_o,
        aes67_ctrl_ptp_path_delay => ptp_mean_path_delay_o,
        aes67_ctrl_ptp_reset => ptp_reset_i,
        aes67_ctrl_ptp_time_source => ptp_time_source_i,
        aes67_ctrl_rx_meter_clip => (others => '0'),
        aes67_ctrl_rx_meter_signal => (others => '0'),
        aes67_ctrl_servo_filter_shift => servo_filter_shift_i,
        aes67_ctrl_servo_gain_shift => servo_gain_shift_i,
        aes67_ctrl_servo_gain_shift_locked => servo_gain_shift_locked_i,
        aes67_ctrl_servo_ki_extra_shift => servo_ki_extra_shift_i,
        aes67_ctrl_servo_ki_gain => servo_ki_gain_i,
        aes67_ctrl_servo_kp_gain => servo_kp_gain_i,
        aes67_ctrl_servo_lock_count_threshold => servo_lock_count_threshold_i,
        aes67_ctrl_servo_lock_threshold_ns => servo_lock_threshold_ns_i,
        aes67_ctrl_servo_mon_effective_gain_shift => servo_mon_effective_gain_shift_o,
        aes67_ctrl_servo_mon_filtered_offset => servo_mon_filtered_offset_o,
        aes67_ctrl_servo_mon_first_lock_achieved => servo_mon_first_lock_achieved_o,
        aes67_ctrl_servo_mon_integral_sum => servo_mon_integral_sum_o,
        aes67_ctrl_servo_mon_lock_counter => servo_mon_lock_counter_o,
        aes67_ctrl_servo_mon_pi_proportional => servo_mon_pi_proportional_o,
        aes67_ctrl_servo_mon_pi_sum_raw => servo_mon_pi_sum_raw_o,
        aes67_ctrl_servo_mon_sample_count => servo_mon_sample_count_o,
        aes67_ctrl_servo_unlock_threshold_ns => servo_unlock_threshold_ns_i,
        aes67_ctrl_servo_warmup_samples => servo_warmup_samples_i,
        aes67_ctrl_tx_meter_clip => (others => '0'),
        aes67_ctrl_tx_meter_signal => (others => '0'),
        aes67_ctrl_wallclock_configured => wallclock_configured_o,
        aes67_ctrl_wallclock_locked => wallclock_locked_o,
        aes67_wb_ack => aes67_wb_ack,
        aes67_wb_adr => aes67_wb_adr,
        aes67_wb_bte => aes67_wb_bte,
        aes67_wb_cti => aes67_wb_cti,
        aes67_wb_cyc => aes67_wb_cyc,
        aes67_wb_dat_r => aes67_wb_dat_r,
        aes67_wb_dat_w => aes67_wb_dat_w,
        aes67_wb_err => aes67_wb_err,
        aes67_wb_sel => aes67_wb_sel,
        aes67_wb_stb => aes67_wb_stb,
        aes67_wb_we => aes67_wb_we,
        clk_mac_rx => mac_rx_clk,
        clk_mac_tx => mac_tx_clk,
        clk_sys => clk_mcu_i,
        eth_buf_irq => eth_irq_o,
        eth_buf_rx_ack => mcu_buf_rx_ack,
        eth_buf_rx_addr => mcu_buf_rx_addr,
        eth_buf_rx_data => mcu_buf_rx_data,
        eth_buf_rx_len => mcu_buf_rx_len,
        eth_buf_rx_valid => mcu_buf_rx_valid,
        eth_buf_rx_we => mcu_buf_rx_we,
        eth_buf_tx_addr => mcu_buf_tx_addr,
        eth_buf_tx_data => mcu_buf_tx_data,
        eth_buf_tx_len => mcu_buf_tx_len,
        rx_stream_cfg_wr_addr => audio_rx_cfg_wr_addr_i,
        rx_stream_cfg_wr_data => audio_rx_cfg_wr_data_i,
        rx_stream_cfg_wr_en => audio_rx_cfg_wr_en_i,
        tx_stream_cfg_wr_addr => audio_tx_cfg_wr_addr_i,
        tx_stream_cfg_wr_data => audio_tx_cfg_wr_data_i,
        tx_stream_cfg_wr_en => audio_tx_cfg_wr_en_i,
        aes67_ctrl_tx_reset => audiotx_reset,
        aes67_ctrl_rx_reset => audiorx_reset,
        aes67_ctrl_eth_reset => mac_reset,
        -- Feed the CSR from the live TSU capture register (sys domain, same as
        -- this CSR logic) instead of the buffer bridge's tx_timestamp_o copy:
        -- that copy is latched in the mac_tx domain when the LAST BYTE has been
        -- fed to the MAC, which for short frames can precede the on-wire SOF —
        -- the latch then keeps the PREVIOUS frame's capture until the next
        -- transmit (host reads a one-frame-stale egress timestamp). The host
        -- attributes the live value to its frame by polling for a change
        -- against a pre-transmit snapshot.
        aes67_ctrl_tx_timestamp_nsec_in => STD_LOGIC_VECTOR(timestamps.tx.nanoseconds),
        aes67_ctrl_tx_timestamp_sec_in => STD_LOGIC_VECTOR(timestamps.tx.seconds),
        aes67_ctrl_wallclock_seconds_in => STD_LOGIC_VECTOR(wallclock_signals.wallclock_seconds_o),
        aes67_ctrl_wallclock_nanoseconds_in => STD_LOGIC_VECTOR(wallclock_signals.wallclock_nanoseconds_o),
        aes67_ctrl_wallclock_nanoseconds_out => wallclock_signals.wallclock_nanoseconds_i,
        aes67_ctrl_wallclock_seconds_out => wallclock_signals.wallclock_seconds_i,
        aes67_ctrl_wallclock_set => wallclock_signals.wallclock_set_i,
        aes67_ctrl_wallclock_phasejump => wallclock_signals.wallclock_do_phasejump_i,
        aes67_ctrl_wallclock_ppb => aes67_ctrl_wallclock_ppb_reg
        
    );
    wallclock_signals.freq_correction_ppb_i <= signed(aes67_ctrl_wallclock_ppb_reg);
    aes67_top_inst: entity work.aes67_top
     generic map(
        MII_WIDTH => MII_WIDTH,
        ETHERNET_TYPE => ETHERNET_TYPE,
        SYS_CLK_NS_PER_TICK => SYS_CLK_NS_PER_TICK,
        MII_CLK_NS_PER_TICK => MII_CLK_NS_PER_TICK,
        TX_MAX_STREAMS => TX_MAX_STREAMS,
        RX_MAX_STREAMS => RX_MAX_STREAMS,
        RX_CHANNELS => RX_CHANNELS,
        TX_CHANNELS => TX_CHANNELS,
        RX_BYTE_DEPTH => RX_BYTE_DEPTH,
        TX_BYTE_DEPTH => TX_BYTE_DEPTH,
        RX_SAMPLE_BUFFER_DEPTH => RX_SAMPLE_BUFFER_DEPTH,
        TX_SAMPLE_BUFFER_DEPTH => TX_SAMPLE_BUFFER_DEPTH,
        MIIM_CLOCK_DIVIDER => MIIM_CLOCK_DIVIDER,
        MIIM_PHY_ADDRESS => MIIM_PHY_ADDRESS,
        AUDIO_RX_USE_PARALLEL_INTERFACE => AUDIO_RX_USE_PARALLEL_INTERFACE,
        AUDIO_RX_TDM_OUTPUTS => AUDIO_RX_TDM_OUTPUTS,
        AUDIO_TX_USE_PARALLEL_INTERFACE => AUDIO_TX_USE_PARALLEL_INTERFACE,
        AUDIO_TX_TDM_INPUTS => AUDIO_TX_TDM_INPUTS,
        USE_EXTERNAL_PLL => USE_EXTERNAL_PLL,
        ENABLE_METERING => ENABLE_METERING,
        PTP_MOVING_AVERAGE_DEPTH => PTP_MOVING_AVERAGE_DEPTH,
        PTP_IN_SOFTWARE => PTP_IN_SOFTWARE,
        PHY_TYPE => PHY_TYPE,
        AUDIO_TDM_CONFIG => AUDIO_TDM_CONFIG
    )
     port map(
        sys_clk_125MHz_i => sys_clk_125MHz_i,
        enet_clk_i => enet_clk_i,
        clk_mcu_i => clk_mcu_i,
        rst_n => rst_n,
        
        phy_mii_rx_clk_in => phy_mii_rx_clk_in,
        phy_mii_tx_clk_in => phy_mii_tx_clk_in,
        phy_mii_tx_data_in => phy_mii_tx_data_in,
        phy_mii_rx_data_in => phy_mii_rx_data_in,
        phy_mii_tx_en_i => phy_mii_tx_en_i,
        phy_mii_rx_en_i => phy_mii_rx_en_i,
        mii_rx_clock_i => mii_rx_clock_i,
        mii_tx_clock_i => mii_tx_clock_i,
        mii_rx_err_i => mii_rx_err_i,
        mii_rx_dv_i => mii_rx_dv_i,
        mii_rxd_i => mii_rxd_i,
        mii_tx_err_o => mii_tx_err_o,
        mii_tx_en_o => mii_tx_en_o,
        mii_txd_o => mii_txd_o,
        enet_mdio => enet_mdio,
        enet_mdc => enet_mdc,
        mac_rx_clock_o => mac_rx_clk,
        mac_tx_clock_o => mac_tx_clk,
        mac_tx_reset_o => mac_tx_rst,
        is_mcu_pkt_tog_o => is_mcu_pkt_tog,
        mcu_ram_addr_i => mcu_ethbuf_ram_addr,
        eth_ram_data_rx_mcu_o => mcu_ethbuf_ram_data,
        eth_tx_en_mcu_i => mcu_tx_en,
        eth_tx_data_mcu_i => mcu_tx_data,
        eth_tx_allow_req_mcu_i => mcu_tx_allow_req,
        eth_tx_allow_mcu_o => mcu_tx_allow,
        mac_tx_busy_o => mac_tx_busy,
        mac_tx_byte_sent_o => mac_tx_byte_sent,
        mac_speed_o => mac_speed,
        mac_linkup_o => mac_linkup,
        mac_received_packet_length_o => mcu_received_packet_length,
        mac_tx_start_prefetch_o => mcu_tx_start_prefetch,
        pll_512fs_i => pll_512fs_i,
        audioclocks_o => audioclocks_o,
        selected_audio_clock_o => selected_audio_clock_o,
        mac_address_i => mac_address_i,
        ip_address_i => ip_address_i,
        ptp_announce_interval_i => ptp_announce_interval_i,
        ptp_clock_accuracy_i => ptp_clock_accuracy_i,
        ptp_clock_class_i => ptp_clock_class_i,
        ptp_gm_prioone_i => ptp_gm_prioone_i,
        ptp_gm_priotwo_i => ptp_gm_priotwo_i,
        ptp_current_leader_id_o => ptp_current_leader_id_o,
        ptp_log_message_interval_i => ptp_log_message_interval_i,
        ptp_time_source_i => ptp_time_source_i,
        ptp_is_follower_o => ptp_is_follower_o,
        ptp_is_leader_o => ptp_is_leader_o,
        ptp_mean_path_delay_o => ptp_mean_path_delay_o,
        ptp_offset_from_master_o => ptp_offset_from_master_o,
        wallclock_locked_o => wallclock_locked_o,
        wallclock_configured_o => wallclock_configured_o,
        servo_kp_gain_i => servo_kp_gain_i,
        servo_ki_gain_i => servo_ki_gain_i,
        servo_gain_shift_i => servo_gain_shift_i,
        servo_gain_shift_locked_i => servo_gain_shift_locked_i,
        servo_ki_extra_shift_i => servo_ki_extra_shift_i,
        servo_filter_shift_i => servo_filter_shift_i,
        servo_warmup_samples_i => servo_warmup_samples_i,
        servo_lock_threshold_ns_i => servo_lock_threshold_ns_i,
        servo_unlock_threshold_ns_i => servo_unlock_threshold_ns_i,
        servo_lock_count_threshold_i => servo_lock_count_threshold_i,
        parser_delay_asymmetry_ns_i => signed(parser_delay_asymmetry_ns_i),
        servo_mon_filtered_offset_o => servo_mon_filtered_offset_o,
        servo_mon_integral_sum_o => servo_mon_integral_sum_o,
        servo_mon_pi_proportional_o => servo_mon_pi_proportional_o,
        servo_mon_pi_sum_raw_o => servo_mon_pi_sum_raw_o,
        servo_mon_effective_gain_shift_o => servo_mon_effective_gain_shift_o,
        servo_mon_lock_counter_o => servo_mon_lock_counter_o,
        servo_mon_sample_count_o => servo_mon_sample_count_o,
        servo_mon_first_lock_achieved_o => servo_mon_first_lock_achieved_o,
        ppb_meter_start_i => ppb_meter_start_i,
        pll_meas_valid_o => pll_meas_valid_o,
        pll_counter_o => pll_counter_o,
        wc_counter_o => wc_counter_o,
        audio_meter_clear_i => audio_meter_clear_i,
        audio_meter_rx_clip_o => audio_meter_rx_clip_o,
        audio_meter_rx_signal_o => audio_meter_rx_signal_o,
        audio_meter_tx_clip_o => audio_meter_tx_clip_o,
        audio_meter_tx_signal_o => audio_meter_tx_signal_o,
        audio_tx_cfg_wr_en_i => audio_tx_cfg_wr_en_i,
        audio_tx_cfg_wr_data_i => audio_tx_cfg_wr_data_i,
        audio_tx_cfg_wr_addr_i => audio_tx_cfg_wr_addr_i,
        audio_rx_cfg_wr_en_i => audio_rx_cfg_wr_en_i,
        audio_rx_cfg_wr_data_i => audio_rx_cfg_wr_data_i,
        audio_rx_cfg_wr_addr_i => audio_rx_cfg_wr_addr_i,
        tdm8out_o => tdm8out_o,
        rx_sample_register => rx_sample_register,
        tdm8in_i => tdm8in_i,
        tx_sample_register => tx_sample_register,
        audiorx_reset_i => audiorx_reset,
        audiotx_reset_i => audiotx_reset,
        mac_resetn_i => mac_resetn,
        wallclock_signals => wallclock_signals,
        timestamps => timestamps
        
    );
    litex_eth_buffer_bridge_inst: entity work.litex_eth_buffer_bridge
     generic map(
        ADD_RX_TIMESTAMP => PTP_IN_SOFTWARE
     )
     port map(
        timestamps_i => timestamps,
        buf_rx_data_o => mcu_buf_rx_data,
        buf_rx_addr_o => mcu_buf_rx_addr,
        buf_rx_we_o => mcu_buf_rx_we,
        buf_rx_len_o => mcu_buf_rx_len,
        buf_rx_valid_o => mcu_buf_rx_valid,
        buf_rx_ack_i => mcu_buf_rx_ack,
        buf_tx_addr_o => mcu_buf_tx_addr,
        buf_tx_len_i => mcu_buf_tx_len,
        buf_tx_dat_i => mcu_buf_tx_data,
        eth_tx_request_i => mcu_tx_req,
        eth_tx_done_o => mcu_tx_done,
        eth_rx_overflow_o => mcu_rx_overflow,
        mac_tx_clock_i => mac_tx_clk,
        mac_tx_reset_i => mac_tx_rst,
        mac_tx_enable_o => mcu_tx_en,
        mac_tx_byte_sent_i => mac_tx_byte_sent,
        mac_tx_dat_o => mcu_tx_data,
        mac_start_prefetch_i => mcu_tx_start_prefetch,
        mac_speed_in => mac_speed,
        tx_allow_req_o => mcu_tx_allow_req,
        tx_allow_i => mcu_tx_allow,
        mac_rx_clock_i => mac_rx_clk,
        mac_rx_reset_i => not rst_n,
        parse_mcu_packet_tog_i => is_mcu_pkt_tog,
        pkt_len_i => std_logic_vector(mcu_received_packet_length),
        eth_ram_data_i => mcu_ethbuf_ram_data,
        eth_ram_addr_o => mcu_ethbuf_ram_addr_slv,
        mcu_clk_i => clk_mcu_i
    );
end architecture;
