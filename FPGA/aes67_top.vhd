-- top level for the logic core. needs external RGMII / RMII to MII Converters and clocks.
-- exposes an ethernet interface for mcu connectivity
LIBRARY ieee;
USE ieee.std_logic_1164.all; 
use IEEE.NUMERIC_STD.all;
use work.miim_types.all;

use work.audioclks_pkg.all;
use work.wallclock_signals_pkg.all;
use work.system_cfg_pkg.all;

ENTITY aes67_top IS  
	generic (
		syscfg : t_global_system_cfg := global_system_cfg_cyc
	);
	PORT
	(
		-- system clocks
		sys_clk_125MHz_i :  IN  STD_LOGIC;
		enet_clk_i       :  IN  STD_LOGIC;
		clk_mcu_i	   :  IN  STD_LOGIC;
		rst_n		: IN STD_LOGIC;
		
		phy_mii_rx_clk_in : IN STD_LOGIC;
		phy_mii_tx_clk_in : IN STD_LOGIC;
		phy_mii_tx_data_in : IN STD_LOGIC_VECTOR(syscfg.PHY_CONFIG.NETWORK_CONFIG.MII_WIDTH - 1 downto 0);
		phy_mii_rx_data_in : IN STD_LOGIC_VECTOR(syscfg.PHY_CONFIG.NETWORK_CONFIG.MII_WIDTH - 1 downto 0);
		phy_mii_tx_en_i : IN STD_LOGIC;
		phy_mii_rx_en_i : IN STD_LOGIC;
		

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


		-- signals for ethernet for the SOC
		mac_rx_clock_o : OUT STD_LOGIC;
		mac_tx_clock_o : OUT STD_LOGIC;
		mac_tx_reset_o : OUT STD_LOGIC;
		is_mcu_pkt_tog_o : OUT STD_LOGIC;
		mcu_ram_addr_i : IN UNSIGNED(10 downto 0);
		eth_ram_data_rx_mcu_o : OUT STD_LOGIC_VECTOR(7 downto 0);
		eth_tx_en_mcu_i : IN STD_LOGIC;
		eth_tx_data_mcu_i : IN STD_LOGIC_VECTOR(7 downto 0);
		eth_tx_allow_req_mcu_i : IN STD_LOGIC;
		eth_tx_allow_mcu_o : OUT STD_LOGIC;
		
		mac_tx_busy_o : OUT STD_LOGIC;
		mac_tx_byte_sent_o : OUT STD_LOGIC;
		mac_speed_o : OUT STD_LOGIC_VECTOR(1 downto 0);
		mac_linkup_o : OUT STD_LOGIC;
		mac_received_packet_length_o : OUT UNSIGNED(10 downto 0);
		mac_tx_start_prefetch_o: OUT STD_LOGIC;
		-- audio clocks

		pll_512fs_i : IN STD_LOGIC;
		audioclocks_o : out t_audio_clocks;
		selected_audio_clock_o : out t_audio_clocks_selected;


		-- control registers - basic

		mac_address_i : IN STD_LOGIC_VECTOR(47 downto 0);
		ip_address_i : IN STD_LOGIC_VECTOR(31 downto 0);

		-- control registers - ptp
		ptp_announce_interval_i : IN STD_LOGIC_VECTOR(7 downto 0);
		ptp_clock_accuracy_i : IN STD_LOGIC_VECTOR(7 downto 0);
		ptp_clock_class_i : IN STD_LOGIC_VECTOR(7 downto 0);
		ptp_gm_prioone_i : IN STD_LOGIC_VECTOR(7 downto 0);
		ptp_gm_priotwo_i : IN STD_LOGIC_VECTOR(7 downto 0);
		ptp_current_leader_id_o : OUT STD_LOGIC_VECTOR(63 downto 0);
		ptp_log_message_interval_i : IN STD_LOGIC_VECTOR(7 downto 0);
		ptp_time_source_i : IN STD_LOGIC_VECTOR(7 downto 0);
		ptp_is_follower_o : OUT STD_LOGIC;
		ptp_is_leader_o : OUT STD_LOGIC;
		ptp_mean_path_delay_o : OUT STD_LOGIC_VECTOR(31 downto 0);
		ptp_offset_from_master_o : OUT STD_LOGIC_VECTOR(31 downto 0);
		wallclock_locked_o : OUT STD_LOGIC;
		wallclock_configured_o : OUT STD_LOGIC;



		-- PTP servo + parser tuning inputs (live-tunable from SoC)
		servo_kp_gain_i              : IN STD_LOGIC_VECTOR(7 downto 0)  := std_logic_vector(to_signed(40, 8));
		servo_ki_gain_i              : IN STD_LOGIC_VECTOR(7 downto 0)  := std_logic_vector(to_signed(5, 8));
		servo_gain_shift_i           : IN STD_LOGIC_VECTOR(4 downto 0)  := std_logic_vector(to_unsigned(3, 5));
		servo_gain_shift_locked_i    : IN STD_LOGIC_VECTOR(4 downto 0)  := (others => '0');
		servo_ki_extra_shift_i       : IN STD_LOGIC_VECTOR(4 downto 0)  := std_logic_vector(to_unsigned(3, 5));
		servo_filter_shift_i         : IN STD_LOGIC_VECTOR(4 downto 0)  := (others => '0');
		servo_warmup_samples_i       : IN STD_LOGIC_VECTOR(7 downto 0)  := std_logic_vector(to_unsigned(16, 8));
		servo_lock_threshold_ns_i    : IN STD_LOGIC_VECTOR(31 downto 0) := std_logic_vector(to_unsigned(500, 32));
		servo_unlock_threshold_ns_i  : IN STD_LOGIC_VECTOR(31 downto 0) := std_logic_vector(to_unsigned(5000, 32));
		servo_lock_count_threshold_i : IN STD_LOGIC_VECTOR(7 downto 0)  := std_logic_vector(to_unsigned(24, 8));
		-- IEEE 1588 delayAsymmetry (ns, signed). +ve = M->S path longer.
		parser_delay_asymmetry_ns_i       : IN signed(31 downto 0) := (others => '0');

		-- PTP servo monitoring outputs (live PI internal state)
		servo_mon_filtered_offset_o      : OUT STD_LOGIC_VECTOR(31 downto 0);
		servo_mon_integral_sum_o         : OUT STD_LOGIC_VECTOR(31 downto 0);
		servo_mon_pi_proportional_o      : OUT STD_LOGIC_VECTOR(31 downto 0);
		servo_mon_pi_sum_raw_o           : OUT STD_LOGIC_VECTOR(31 downto 0);
		servo_mon_effective_gain_shift_o : OUT STD_LOGIC_VECTOR(7 downto 0);
		servo_mon_lock_counter_o         : OUT STD_LOGIC_VECTOR(15 downto 0);
		servo_mon_sample_count_o         : OUT STD_LOGIC_VECTOR(15 downto 0);
		servo_mon_first_lock_achieved_o  : OUT STD_LOGIC;

		-- ppb meter for external pll
		ppb_meter_start_i : IN STD_LOGIC;
		pll_meas_valid_o : OUT STD_LOGIC;
		pll_counter_o : OUT unsigned(24 downto 0);
		wc_counter_o : OUT unsigned(24 downto 0);
		

		-- audio metering
		audio_meter_clear_i : IN STD_LOGIC;
		audio_meter_rx_clip_o : OUT STD_LOGIC_VECTOR(syscfg.AUDIO_CONFIG.RX_DA_CFG.CHANNELS - 1 downto 0);
		audio_meter_rx_signal_o : OUT STD_LOGIC_VECTOR(syscfg.AUDIO_CONFIG.RX_DA_CFG.CHANNELS - 1 downto 0);
		audio_meter_tx_clip_o : OUT STD_LOGIC_VECTOR(syscfg.AUDIO_CONFIG.TX_AD_CFG.CHANNELS - 1 downto 0);
		audio_meter_tx_signal_o : OUT STD_LOGIC_VECTOR(syscfg.AUDIO_CONFIG.TX_AD_CFG.CHANNELS - 1 downto 0);

		-- stream configuration
		audio_tx_cfg_wr_en_i : IN STD_LOGIC;
		audio_tx_cfg_wr_data_i : IN STD_LOGIC_VECTOR(7 downto 0);
		audio_tx_cfg_wr_addr_i : IN STD_LOGIC_VECTOR(7 downto 0);

		audio_rx_cfg_wr_en_i : IN STD_LOGIC;
		audio_rx_cfg_wr_data_i : IN STD_LOGIC_VECTOR(7 downto 0);
		audio_rx_cfg_wr_addr_i : IN STD_LOGIC_VECTOR(7 downto 0);

		tdm_in :  IN  STD_LOGIC_VECTOR(syscfg.AUDIO_CONFIG.TX_AD_CFG.TDM_PINS - 1 downto 0);
		tdm_out :  OUT  STD_LOGIC_VECTOR(syscfg.AUDIO_CONFIG.RX_DA_CFG.TDM_PINS - 1 downto 0);
		


    rx_sample_register : OUT STD_LOGIC_VECTOR((syscfg.AUDIO_CONFIG.PARALLEL_BYTE_DEPTH * 8) * syscfg.AUDIO_CONFIG.RX_DA_CFG.CHANNELS - 1 downto 0);
    tx_sample_register : IN STD_LOGIC_VECTOR((syscfg.AUDIO_CONFIG.PARALLEL_BYTE_DEPTH * 8) * syscfg.AUDIO_CONFIG.TX_AD_CFG.CHANNELS - 1 downto 0) := (others => '0');
		

		-- RX underrun-mute diagnostics (fixed width for the status CSR:
		-- lower 4 streams / lower 8 channels, zero-padded)
		rx_stream_underrun_o : OUT STD_LOGIC_VECTOR(3 downto 0);
		rx_mute_channels_o : OUT STD_LOGIC_VECTOR(7 downto 0);
		


		ptp_reset_i : IN STD_LOGIC := '0';
		audiotx_reset_i : IN STD_LOGIC := '0';
		audiorx_reset_i : IN STD_LOGIC := '0';
		mac_resetn_i : IN STD_LOGIC := '1';

		wallclock_signals : INOUT t_wallclock_signals;
		timestamps : OUT t_eth_timestamps

	);
END aes67_top;

ARCHITECTURE rtl OF aes67_top IS 

signal audioclks : t_audio_clocks := AUDIO_CLOCKS_RESET;
signal selected_audio_clock : t_audio_clocks_selected := AUDIO_CLOCKS_RESET_SELECTED;
signal media_clock : STD_LOGIC_VECTOR(31 downto 0);
signal media_tick : STD_LOGIC;
signal rx_stream_underrun : STD_LOGIC_VECTOR(syscfg.AUDIO_CONFIG.RX_DA_CFG.MAX_STREAMS - 1 downto 0);
signal rx_mute_channels : STD_LOGIC_VECTOR(syscfg.AUDIO_CONFIG.RX_DA_CFG.CHANNELS - 1 downto 0);
signal second_pulse_sys : STD_LOGIC;
signal mac_linkup : STD_LOGIC;
-- mac_linkup_o is driven from internal mac_linkup below

-- sample registers for in/output




signal mii_txd_o_reg : STD_LOGIC_VECTOR(7 downto 0);

signal mac_tx_clock : STD_LOGIC;
signal mac_rx_clock : STD_LOGIC;
signal mac_rx_data: STD_LOGIC_VECTOR(7 downto 0);
signal mac_rx_byte_received : STD_LOGIC;
signal mac_rx_byte_receive_index : unsigned(10 downto 0);
signal mac_is_ptp_frame : std_logic;

-- ethernet_top signals

signal is_rtp_pkt_tog : STD_LOGIC;
signal rtp_ram_addr : unsigned(10 downto 0);
signal eth_ram_data_sys_rtp : STD_LOGIC_VECTOR(7 downto 0);
signal eth_tx_en_ptp : STD_LOGIC;
signal eth_tx_en_rtp : STD_LOGIC;
signal eth_tx_data_ptp : STD_LOGIC_VECTOR(7 downto 0);
signal eth_tx_data_rtp : STD_LOGIC_VECTOR(7 downto 0);
signal eth_tx_req_ptp : STD_LOGIC;
signal eth_tx_req_rtp : STD_LOGIC;
signal eth_tx_allow_ptp : STD_LOGIC;
signal eth_tx_allow_rtp : STD_LOGIC;
signal mac_reset : std_logic;
signal mac_tx_byte_sent : STD_LOGIC;
signal mac_tx_busy : STD_LOGIC;
signal mac_rx_reset : STD_LOGIC;
signal mac_sof_sent_tog : STD_LOGIC;
signal mac_sof_recv_tog : STD_LOGIC;
signal mac_speed : STD_LOGIC_VECTOR(1 downto 0);

-- reset signals
signal ptp_module_rst_n : STD_LOGIC;
signal audiotx_reset_n : STD_LOGIC;
signal audiorx_reset_n : STD_LOGIC;

-- Servo monitoring (typed) signals between ptp_module and top-level pads
signal servo_mon_filtered_offset_signed       : signed(31 downto 0);
signal servo_mon_integral_sum_signed          : signed(31 downto 0);
signal servo_mon_pi_proportional_signed       : signed(31 downto 0);
signal servo_mon_pi_sum_raw_signed            : signed(31 downto 0);
signal servo_mon_effective_gain_shift_unsigned: unsigned(7 downto 0);
signal servo_mon_lock_counter_unsigned        : unsigned(15 downto 0);
signal servo_mon_sample_count_unsigned        : unsigned(15 downto 0);


BEGIN

audioclocks_o <= audioclks;
selected_audio_clock_o <= selected_audio_clock;
mii_txd_o <= mii_txd_o_reg;
ptp_module_rst_n <= rst_n and not ptp_reset_i;
audiotx_reset_n <= rst_n and not audiotx_reset_i;
audiorx_reset_n <= rst_n and not audiorx_reset_i;

mac_speed_o <= mac_speed;


mac_tx_clock_o <= mac_tx_clock;
mac_reset <= not mac_resetn_i;

mac_rx_clock_o <= mac_rx_clock;

mac_tx_busy_o <= mac_tx_busy;
mac_tx_byte_sent_o <= mac_tx_byte_sent;
selected_audio_clock <= audioclks.clk_256fs when syscfg.AUDIO_CONFIG.bclk_speed = audio_clock_12_28 
					else audioclks.clk_128fs when syscfg.AUDIO_CONFIG.bclk_speed = audio_clock_06_14 else audioclks.clk_64fs
						WHEN syscfg.AUDIO_CONFIG.bclk_speed = audio_clock_03_07;
mclk_switch_extern: if USE_EXTERNAL_PLL = true generate
--	clk_512fs <= pll_512fs_i;
--	audioclocks_inst: entity work.audioclock_generator
--	PORT MAP(mclk => clk_512fs,
--		 rst_n => ptp_module_rst_n,
--		 clk_64fs => pll_64fs,
--		 fs => pll_48k_fs,
--		 bclk_r => pll_256fs_rising,
--		 bclk_f => pll_256fs_falling,
--		 fs_pulse => pll_48k_fs_tdm);
	
	ppb_meter_inst: entity work.clock_ppb_meter
	PORT MAP(sys_clk => sys_clk_125MHz_i,
		 reset_n => rst_n,
		 wallclock_512fs_in => audioclks.mclk,
		 pll_512fs_in => pll_512fs_i,
		 wallclock_second_pulse_i => second_pulse_sys,
		 start_i => ppb_meter_start_i,
		 valid_o => pll_meas_valid_o,
		 count_pll_o => pll_counter_o,
		 count_wc_o => wc_counter_o);
end generate;

mclk_switch_INTERNAL: if USE_EXTERNAL_PLL = false generate
	wc_counter_o <= (others => '0');
	pll_counter_o <= (others => '0');
	pll_meas_valid_o <= '0';
end generate;


no_audio_tx_gen : if (syscfg.AUDIO_CONFIG.TX_AD_CFG.CHANNELS = 0) generate
eth_tx_en_rtp <= '0';
eth_tx_req_rtp <= '0';
eth_tx_data_rtp <= (others => '0');

end generate;
audiotx_gen: if (syscfg.AUDIO_CONFIG.TX_AD_CFG.CHANNELS /= 0) generate
audiotx_inst: entity work.audio_tx_module
GENERIC MAP(bytes_per_sample => syscfg.AUDIO_CONFIG.PARALLEL_BYTE_DEPTH,
			global_channel_count => syscfg.AUDIO_CONFIG.TX_AD_CFG.CHANNELS,
			samples_per_channel_depth => syscfg.AUDIO_CONFIG.TX_AD_CFG.BUFFER_DEPTH,
			max_streams => syscfg.AUDIO_CONFIG.TX_AD_CFG.MAX_STREAMS,
			ENABLE_METERING => syscfg.ENABLE_METERING,
			-- TDM demux integrated unless the legacy parallel interface is forced.
			TDM_INPUT => not syscfg.AUDIO_CONFIG.USE_PARALLEL_INTERFACE,
			TDM_INPUTS => syscfg.AUDIO_CONFIG.TX_AD_CFG.TDM_PINS,
			TDM_CONFIG => syscfg.AUDIO_CONFIG.TX_AD_CFG.ADDA_CFG
			)
PORT MAP(sys_clk => sys_clk_125MHz_i,
		 ctrl_plane_clk => clk_mcu_i,
		 rst_n => audiotx_reset_n,

		 audioclocks_i => selected_audio_clock,
		 fs_halfduty_clk_i => audioclks.fsclk_50,
 
		 tdm_in_i => tdm_in(syscfg.AUDIO_CONFIG.TX_AD_CFG.TDM_PINS - 1 downto 0),

		 mac_tx_clock => mac_tx_clock,
		 mac_tx_busy => mac_tx_busy,
		 mac_tx_byte_sent => mac_tx_byte_sent,
		 mac_audio_allow_i => eth_tx_allow_rtp,

		 tx_en_o => eth_tx_en_rtp,
		 tx_req_o => eth_tx_req_rtp,
		 ip_addr_i => ip_address_i,
		 mac_addr_i => mac_address_i,

		 cfg_wr_en_i => audio_tx_cfg_wr_en_i,
		 cfg_wr_addr_i => audio_tx_cfg_wr_addr_i,
		 cfg_wr_data_i => audio_tx_cfg_wr_data_i,
		audio_i => tx_sample_register,

		 
		 media_clock_i => media_clock,


 		 metering_clear_i => audio_meter_clear_i,
		 metering_clip_o => audio_meter_tx_clip_o,
		 metering_signal_o => audio_meter_tx_signal_o,
		 tx_data_o => eth_tx_data_rtp,
		 mac_speed_i => mac_speed);

end generate;

mac_linkup_o <= mac_linkup;

ptp_inst: entity work.ptp_module
generic map (
	MII_WIDTH => syscfg.PHY_CONFIG.NETWORK_CONFIG.MII_WIDTH,
	SYS_CLK_NS_PER_TICK => SYS_CLK_NS_PER_TICK,
	MII_CLK_NS_PER_TICK => syscfg.PHY_CONFIG.NETWORK_CONFIG.MII_CLK_NS_PER_TICK,
	MII_TYPE => syscfg.PHY_CONFIG.NETWORK_CONFIG.MII_TYPE,
	STATIC_PTP_CONF => syscfg.STATIC_PTP_CONFIG,
	PTP_MOVING_AVERAGE_DEPTH => syscfg.PTP_MOVING_AVERAGE_DEPTH,
	PTP_IN_SOFTWARE => syscfg.PTP_IN_SOFTWARE
)
PORT MAP(sys_clk => sys_clk_125MHz_i,
		 rst_n => ptp_module_rst_n,

		 -- mac signals
		 mac_link_up_i => mac_linkup,
		 mac_tx_clock => mac_tx_clock,
		 mac_tx_busy => mac_tx_busy,
		 mac_tx_byte_sent => mac_tx_byte_sent,
		 mac_tx_allow_i => eth_tx_allow_ptp,
		 mcu_tx_en_i => eth_tx_en_mcu_i,


		rx_clk_i => mac_rx_clock,
		rx_data_i => mac_rx_data,
		rx_byte_receive_index_i => mac_rx_byte_receive_index(7 downto 0),
		rx_byte_received_i => mac_rx_byte_received,
		rx_ptp_frame_i => mac_is_ptp_frame,

		tx_data_ptpfu => eth_tx_data_ptp,
		
		tx_en_ptpfu => eth_tx_en_ptp,
		 ptp_allow_req => eth_tx_req_ptp,
		
		 
		-- configuration registers 
		ip_address => ip_address_i,
		mac_address => mac_address_i,

		 ptp_announce_interval => ptp_announce_interval_i,
		 ptp_clock_accuracy => ptp_clock_accuracy_i,
		 ptp_clock_class => ptp_clock_class_i,
		 ptp_clock_priorityone => ptp_gm_prioone_i,
		 ptp_clock_prioritytwo => ptp_gm_priotwo_i,
		 ptp_current_leader_id_o => ptp_current_leader_id_o,
		 ptp_log_message_interval => ptp_log_message_interval_i,
		 ptp_time_source => ptp_time_source_i,
		ptp_is_follower_o => ptp_is_follower_o,
		ptp_is_leader_o => ptp_is_leader_o,
		 wallclock_locked => wallclock_locked_o,
		 wallclock_configured => wallclock_configured_o,
		 
		 
		 ptp_mean_path_delay => ptp_mean_path_delay_o,
		 ptp_offset_from_master => ptp_offset_from_master_o,
		 

		 audioclocks_o => audioclks,
		 wallclock_signals_io => wallclock_signals,
		 timestamps_o => timestamps,
		 second_pulse_sys => second_pulse_sys,
		 media_clock => media_clock,
		 media_tick => media_tick,
		 mac_speed_i => mac_speed,

		 -- Servo / parser tuning inputs (live-tunable from SoC)
		 servo_kp_gain_i              => signed(servo_kp_gain_i),
		 servo_ki_gain_i              => signed(servo_ki_gain_i),
		 servo_gain_shift_i           => unsigned(servo_gain_shift_i),
		 servo_gain_shift_locked_i    => unsigned(servo_gain_shift_locked_i),
		 servo_ki_extra_shift_i       => unsigned(servo_ki_extra_shift_i),
		 servo_filter_shift_i         => unsigned(servo_filter_shift_i),
		 servo_warmup_samples_i       => unsigned(servo_warmup_samples_i),
		 servo_lock_threshold_ns_i    => unsigned(servo_lock_threshold_ns_i),
		 servo_unlock_threshold_ns_i  => unsigned(servo_unlock_threshold_ns_i),
		 servo_lock_count_threshold_i => unsigned(servo_lock_count_threshold_i),
		 parser_delay_asymmetry_ns_i       => parser_delay_asymmetry_ns_i,

		 -- Servo monitoring outputs to top-level
		 servo_mon_filtered_offset_o      => servo_mon_filtered_offset_signed,
		 servo_mon_integral_sum_o         => servo_mon_integral_sum_signed,
		 servo_mon_pi_proportional_o      => servo_mon_pi_proportional_signed,
		 servo_mon_pi_sum_raw_o           => servo_mon_pi_sum_raw_signed,
		 servo_mon_effective_gain_shift_o => servo_mon_effective_gain_shift_unsigned,
		 servo_mon_lock_counter_o         => servo_mon_lock_counter_unsigned,
		 servo_mon_sample_count_o         => servo_mon_sample_count_unsigned,
		 servo_mon_first_lock_achieved_o  => servo_mon_first_lock_achieved_o,

		 phy_mii_rx_clk_in => phy_mii_rx_clk_in,
		 phy_mii_tx_clk_in => phy_mii_tx_clk_in,
		 phy_mii_rx_data_in => phy_mii_rx_data_in,
		 phy_mii_tx_data_in => phy_mii_tx_data_in,
		 phy_gmii_rx_data_in => mii_rxd_i,
		 phy_gmii_tx_data_in => mii_txd_o_reg,
		 phy_mii_tx_en_i => phy_mii_tx_en_i,
		 phy_mii_rx_en_i => phy_mii_rx_en_i
		 );


gen_ptp_metering: if (syscfg.ENABLE_METERING = true) generate
servo_mon_filtered_offset_o      <= std_logic_vector(servo_mon_filtered_offset_signed);
servo_mon_integral_sum_o         <= std_logic_vector(servo_mon_integral_sum_signed);
servo_mon_pi_proportional_o      <= std_logic_vector(servo_mon_pi_proportional_signed);
servo_mon_pi_sum_raw_o           <= std_logic_vector(servo_mon_pi_sum_raw_signed);
servo_mon_effective_gain_shift_o <= std_logic_vector(servo_mon_effective_gain_shift_unsigned);
servo_mon_lock_counter_o         <= std_logic_vector(servo_mon_lock_counter_unsigned);
servo_mon_sample_count_o         <= std_logic_vector(servo_mon_sample_count_unsigned);
end generate; 









rx_gen: if (syscfg.AUDIO_CONFIG.RX_DA_CFG.CHANNELS /= 0) generate
rx_ringbuffer_inst: entity work.rx_ringbuffer
GENERIC MAP(audio_buffer_sample_depth => syscfg.AUDIO_CONFIG.RX_DA_CFG.BUFFER_DEPTH,
			bytes_per_sample => syscfg.AUDIO_CONFIG.PARALLEL_BYTE_DEPTH,
			global_channel_count => syscfg.AUDIO_CONFIG.RX_DA_CFG.CHANNELS,
			max_streams => syscfg.AUDIO_CONFIG.RX_DA_CFG.MAX_STREAMS,
			ENABLE_METERING => syscfg.ENABLE_METERING,
			PARALLEL_OUT => syscfg.AUDIO_CONFIG.USE_PARALLEL_INTERFACE,
			TDM_OUTPUTS => syscfg.AUDIO_CONFIG.RX_DA_CFG.TDM_PINS,
			TDM_CONFIG => syscfg.AUDIO_CONFIG.RX_DA_CFG.ADDA_CFG
			)
PORT MAP(sys_clk => sys_clk_125MHz_i,
		 reset_n => audiorx_reset_n,
		 

		 -- network
 		 eth_read_addr_o => rtp_ram_addr,
		 packet_ready_i => is_rtp_pkt_tog,
		 eth_read_data_i => eth_ram_data_sys_rtp,

		 -- clocking
		audioclocks_i => selected_audio_clock,
		media_clock_i => media_clock,
		 
		 -- configuration
		 stream_config_addr_i => audio_rx_cfg_wr_addr_i,
		 stream_config_data_i => audio_rx_cfg_wr_data_i,
		 stream_config_wr_en_i => audio_rx_cfg_wr_en_i,
		 stream_config_wr_clk_i => clk_mcu_i,

		 -- audio
		 
		 audio_out => rx_sample_register,
		 tdm_out => tdm_out,

		 -- metering
		 metering_clear_i => audio_meter_clear_i,
		 metering_clip_o => audio_meter_rx_clip_o,
		 metering_signal_o => audio_meter_rx_signal_o,

		 -- underrun-mute diagnostics
		 stream_underrun_o => rx_stream_underrun,
		 mute_channels_o => rx_mute_channels);

	rx_stream_underrun_o <= std_logic_vector(resize(unsigned(rx_stream_underrun), 4));
	rx_mute_channels_o <= std_logic_vector(resize(unsigned(rx_mute_channels), 8));
end generate;
no_rx_gen: if syscfg.AUDIO_CONFIG.RX_DA_CFG.CHANNELS = 0 generate
	audio_meter_rx_clip_o <= (others => '0');
	audio_meter_rx_signal_o <= (others => '0');
	rx_sample_register <= (others => '0');
	rx_stream_underrun_o <= (others => '0');
	rx_mute_channels_o <= (others => '0');
	tdm_out <= (others => '0');
	rtp_ram_addr <= (others => '0');
end generate;











ethernet_top_inst: entity work.ethernet_top
 GENERIC MAP(MIIM_CLOCK_DIVIDER => syscfg.PHY_CONFIG.NETWORK_CONFIG.MIIM_CLOCK_DIVIDER,
 MIIM_PHY_ADDRESS => syscfg.PHY_CONFIG.MIIM_PHY_ADDRESS,
 MII_TYPE => syscfg.PHY_CONFIG.NETWORK_CONFIG.MII_TYPE,
 PTP_IN_SOFTWARE => syscfg.PTP_IN_SOFTWARE,
 PHY_TYPE => syscfg.PHY_CONFIG.PHY_TYPE)
 port map(
	sys_clk125MHz_i => sys_clk_125MHz_i,
	enet_clk_i => enet_clk_i,
	mac_rx_clock_o => mac_rx_clock,
	mac_tx_clock_o => mac_tx_clock,
	rst_n => mac_resetn_i,
	received_packet_length_o => mac_received_packet_length_o,
	is_ptp_frame_o => mac_is_ptp_frame,
	mac_rx_data_o => mac_rx_data,
	mac_rx_byte_received_o => mac_rx_byte_received,
	mac_rx_byte_receive_index_o => mac_rx_byte_receive_index,

	is_mcu_pkt_tog_o => is_mcu_pkt_tog_o,
	is_rtp_pkt_tog_o => is_rtp_pkt_tog,
	rtp_ram_addr_i => rtp_ram_addr,
	mcu_ram_addr_i => mcu_ram_addr_i,
	eth_ram_data_sys_rtp_o => eth_ram_data_sys_rtp,
	eth_ram_data_rx_mcu_o => eth_ram_data_rx_mcu_o,
	eth_tx_en_mcu_i => eth_tx_en_mcu_i,
	eth_tx_en_ptp_i => eth_tx_en_ptp,
	eth_tx_en_rtp_i => eth_tx_en_rtp,
	eth_tx_data_mcu_i => eth_tx_data_mcu_i,
	eth_tx_data_ptp_i => eth_tx_data_ptp,
	eth_tx_data_rtp_i => eth_tx_data_rtp,
	eth_tx_req_mcu_i => eth_tx_allow_req_mcu_i,
	eth_tx_req_ptp_i => eth_tx_req_ptp,
	eth_tx_req_rtp_i => eth_tx_req_rtp,
	eth_tx_allow_mcu_o => eth_tx_allow_mcu_o,
	eth_tx_allow_ptp_o => eth_tx_allow_ptp,
	eth_tx_allow_rtp_o => eth_tx_allow_rtp,
	mac_reset_i => mac_reset,
	mac_addr_i => mac_address_i,
	mac_speed_o => mac_speed,
	mac_linkup_o => mac_linkup,
	mac_tx_byte_sent_o => mac_tx_byte_sent,
	mac_tx_reset_o => mac_tx_reset_o,
	mac_tx_busy_o => mac_tx_busy,
	mac_rx_reset_o => mac_rx_reset,
	mac_sof_sent_tog_o => mac_sof_sent_tog,
	mac_tx_start_prefetch_o => mac_tx_start_prefetch_o,
	mac_sof_recv_tog_o => mac_sof_recv_tog,
	mii_rx_clock_i => mii_rx_clock_i,
	mii_tx_clock_i => mii_tx_clock_i,
	mii_rx_err_i => mii_rx_err_i,
	mii_rx_dv_i => mii_rx_dv_i,
	mii_rxd_i => mii_rxd_i,
	mii_tx_err_o => mii_tx_err_o,
	mii_tx_en_o => mii_tx_en_o,
	mii_txd_o => mii_txd_o_reg,
	enet_mdio => enet_mdio,
	enet_mdc => enet_mdc
);


END rtl;