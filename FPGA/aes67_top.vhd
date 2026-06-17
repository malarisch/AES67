-- top level for the logic core. needs external RGMII / RMII to MII Converters and clocks.
-- exposes an ethernet interface for mcu connectivity
LIBRARY ieee;
USE ieee.std_logic_1164.all; 
use IEEE.NUMERIC_STD.all;
use work.miim_types.all;


ENTITY aes67_top IS  
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
		TX_SAMPLE_BUFFER_DEPTH : natural := 64;
		  
    	MIIM_CLOCK_DIVIDER : POSITIVE := 50;

    	MIIM_PHY_ADDRESS      : t_phy_address := (others => '0');
		

		
		AUDIO_RX_USE_PARALLEL_INTERFACE : boolean := false;
		AUDIO_RX_TDM_OUTPUTS : natural := 2;
		AUDIO_RX_TDM_CHANNELS : natural  := 8;

		AUDIO_TX_USE_PARALLEL_INTERFACE : boolean := false;
		AUDIO_TX_TDM_INPUTS : natural := 1;
		AUDIO_TX_TDM_CHANNELS : natural  := 8;

		USE_EXTERNAL_PLL : BOOLEAN := true;
		ENABLE_METERING: BOOLEAN := true

	);
	PORT
	(
		-- system clocks
		sys_clk_125MHz_i :  IN  STD_LOGIC;
		enet_clk_i       :  IN  STD_LOGIC;
		clk_mcu_i	   :  IN  STD_LOGIC;
		rst_n		: IN STD_LOGIC;
		mac_resetn_i : IN STD_LOGIC := '1';
		phy_mii_rx_clk_in : IN STD_LOGIC;
		phy_mii_tx_clk_in : IN STD_LOGIC;
		phy_mii_tx_data_in : IN STD_LOGIC_VECTOR(MII_WIDTH - 1 downto 0);
		phy_mii_rx_data_in : IN STD_LOGIC_VECTOR(MII_WIDTH - 1 downto 0);
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
		wc_512fs_o : OUT STD_LOGIC;
		pll_64fs_o : OUT STD_LOGIC;
		pll_48k_fs_o : OUT STD_LOGIC;
		pll_48k_fs_tdm_o : OUT STD_LOGIC;

		pll_256fs_rising_o : OUT STD_LOGIC;
		pll_256fs_falling_o : OUT STD_LOGIC; -- 50% phase shift - some ADC/DACs require it


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

		-- PTP module reset (active high, async). Combined with global rst_n
		-- to reset everything time-related (servo, parser, wallclock).
		ptp_reset_i : IN STD_LOGIC := '0';

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
		audio_meter_rx_clip_o : OUT STD_LOGIC_VECTOR(RX_CHANNELS - 1 downto 0);
		audio_meter_rx_signal_o : OUT STD_LOGIC_VECTOR(RX_CHANNELS - 1 downto 0);
		audio_meter_tx_clip_o : OUT STD_LOGIC_VECTOR(TX_CHANNELS - 1 downto 0);
		audio_meter_tx_signal_o : OUT STD_LOGIC_VECTOR(TX_CHANNELS - 1 downto 0);

		-- stream configuration
		audio_tx_cfg_wr_en_i : IN STD_LOGIC;
		audio_tx_cfg_wr_data_i : IN STD_LOGIC_VECTOR(7 downto 0);
		audio_tx_cfg_wr_addr_i : IN STD_LOGIC_VECTOR(7 downto 0);

		audio_rx_cfg_wr_en_i : IN STD_LOGIC;
		audio_rx_cfg_wr_data_i : IN STD_LOGIC_VECTOR(7 downto 0);
		audio_rx_cfg_wr_addr_i : IN STD_LOGIC_VECTOR(7 downto 0);

		-- audio outputs
		tdm8out_o : OUT STD_LOGIC_VECTOR(AUDIO_RX_TDM_OUTPUTS - 1 downto 0);
		rx_sample_register : OUT STD_LOGIC_VECTOR((RX_BYTE_DEPTH * 8) * RX_CHANNELS - 1 downto 0);
		

		-- audio inputs
		tdm8in_i : IN STD_LOGIC_VECTOR(AUDIO_TX_TDM_INPUTS - 1 downto 0) := (others => '0');
		tx_sample_register : IN STD_LOGIC_VECTOR((TX_BYTE_DEPTH * 8) * TX_CHANNELS - 1 downto 0) := (others => '0')

	);
END aes67_top;

ARCHITECTURE rtl OF aes67_top IS 


-- audio clocks
SIGNAL pll_64fs : STD_LOGIC;
SIGNAL pll_48k_fs : STD_LOGIC;
SIGNAL pll_48k_fs_tdm : STD_LOGIC;

signal pll_256fs_falling : STD_LOGIC;
signal pll_256fs_rising : STD_LOGIC;
signal clk_512fs : STD_LOGIC;
signal wc_mclk : STD_LOGIC; -- 512·fs NCO generated from wallclock

-- Wallclock-NCO-derived audio sub-clocks (phase-coherent with fs epoch).
-- Used in the internal-NCO path; the external-PLL path uses pll_* directly.
signal wc_clk_256fs   : STD_LOGIC;
signal wc_clk_128fs   : STD_LOGIC;
signal wc_clk_64fs_int : STD_LOGIC;
signal wc_fs_int       : STD_LOGIC;
signal wc_bclk_r_int   : STD_LOGIC;
signal wc_bclk_f_int   : STD_LOGIC;
signal wc_fs_pulse     : STD_LOGIC;
signal wc_fs_tdm_int   : STD_LOGIC;
signal media_clock : STD_LOGIC_VECTOR(31 downto 0);
signal media_tick : STD_LOGIC;
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

-- PTP module reset: global rst_n AND NOT software-asserted ptp_reset_i.
-- Resets servo, parser, wallclock — everything time-related.
signal ptp_module_rst_n : STD_LOGIC;

-- Servo monitoring (typed) signals between ptp_module and top-level pads
signal servo_mon_filtered_offset_signed       : signed(31 downto 0);
signal servo_mon_integral_sum_signed          : signed(31 downto 0);
signal servo_mon_pi_proportional_signed       : signed(31 downto 0);
signal servo_mon_pi_sum_raw_signed            : signed(31 downto 0);
signal servo_mon_effective_gain_shift_unsigned: unsigned(7 downto 0);
signal servo_mon_lock_counter_unsigned        : unsigned(15 downto 0);
signal servo_mon_sample_count_unsigned        : unsigned(15 downto 0);


BEGIN


mii_txd_o <= mii_txd_o_reg;
ptp_module_rst_n <= '1';--rst_n and not ptp_reset_i;

mac_speed_o <= mac_speed;

pll_64fs_o <= pll_64fs;
pll_48k_fs_o <= pll_48k_fs;
pll_48k_fs_tdm_o <= pll_48k_fs_tdm;
pll_256fs_rising_o <= pll_256fs_rising;
pll_256fs_falling_o <= pll_256fs_falling;

mac_tx_clock_o <= mac_tx_clock;
mac_reset <= not mac_resetn_i;

mac_rx_clock_o <= mac_rx_clock;

mac_tx_busy_o <= mac_tx_busy;
mac_tx_byte_sent_o <= mac_tx_byte_sent;

mclk_switch_extern: if USE_EXTERNAL_PLL = true generate
	clk_512fs <= pll_512fs_i;
	audioclocks_inst: entity work.audioclock_generator
	PORT MAP(mclk => clk_512fs,
		 rst_n => ptp_module_rst_n,
		 clk_64fs => pll_64fs,
		 fs => pll_48k_fs,
		 bclk_r => pll_256fs_rising,
		 bclk_f => pll_256fs_falling,
		 fs_pulse => pll_48k_fs_tdm);
	
	ppb_meter_inst: entity work.clock_ppb_meter
	PORT MAP(sys_clk => sys_clk_125MHz_i,
		 reset_n => rst_n,
		 wallclock_512fs_in => wc_mclk,
		 pll_512fs_in => pll_512fs_i,
		 wallclock_second_pulse_i => second_pulse_sys,
		 start_i => ppb_meter_start_i,
		 valid_o => pll_meas_valid_o,
		 count_pll_o => pll_counter_o,
		 count_wc_o => wc_counter_o);
end generate;

mclk_switch_INTERNAL: if USE_EXTERNAL_PLL = false generate
	-- Sub-clocks come straight from the wallclock NCO (ptp_module).
	-- They are sys_clk-synchronous, phase-coherent with fs/sample_pulse,
	-- and reset cleanly on PTP resync — no separate divider needed.
	clk_512fs         <= wc_mclk;
	pll_64fs          <= wc_clk_64fs_int;
	pll_48k_fs        <= wc_fs_int;
	pll_256fs_rising  <= wc_bclk_r_int;
	pll_256fs_falling <= wc_bclk_f_int;
	pll_48k_fs_tdm    <= wc_fs_tdm_int;
	wc_counter_o <= (others => '0');
	pll_counter_o <= (others => '0');
	pll_meas_valid_o <= '0';
end generate;

wc_512fs_o <= clk_512fs;



audiotx_inst: entity work.audio_tx_module
GENERIC MAP(bytes_per_sample => TX_BYTE_DEPTH,
			global_channel_count => TX_CHANNELS,
			samples_per_channel_depth => TX_SAMPLE_BUFFER_DEPTH,
			max_streams => TX_MAX_STREAMS,
			ENABLE_METERING => ENABLE_METERING,
			-- TDM demux integrated unless the legacy parallel interface is forced.
			TDM_INPUT => not AUDIO_TX_USE_PARALLEL_INTERFACE,
			TDM_INPUTS => AUDIO_TX_TDM_INPUTS,
			TDM_CHANNELS => AUDIO_TX_TDM_CHANNELS
			)
PORT MAP(sys_clk => sys_clk_125MHz_i,
		 ctrl_plane_clk => clk_mcu_i,
		 rst_n => rst_n,
		 -- In integrated TDM mode the buffer's frame sync must be the TDM fsync
		 -- (pll_48k_fs_tdm), matching what the external tdm8_in used; the parallel
		 -- capture path uses the regular pll_48k_fs.
		 fs_tdm_clk_i => pll_48k_fs_tdm,
		 fs_halfduty_clk_i => wc_fs_int,
 
		 bclk_i => pll_256fs_falling,
		 tdm_in_i => tdm8in_i(AUDIO_TX_TDM_INPUTS - 1 downto 0),

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



mac_linkup_o <= mac_linkup;

ptp_inst: entity work.ptp_module
generic map (
	MII_WIDTH => MII_WIDTH,
	SYS_CLK_NS_PER_TICK => SYS_CLK_NS_PER_TICK,
	MII_CLK_NS_PER_TICK => MII_CLK_NS_PER_TICK,
	ETHERNET_TYPE => ETHERNET_TYPE
)
PORT MAP(sys_clk => sys_clk_125MHz_i,
		 rst_n => ptp_module_rst_n,

		 -- mac signals
		 mac_link_up_i => mac_linkup,
		 mac_tx_clock => mac_tx_clock,
		 mac_tx_busy => mac_tx_busy,
		 mac_tx_byte_sent => mac_tx_byte_sent,
		 mac_tx_allow_i => eth_tx_allow_ptp,


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
		 

		 -- generated clocks
		 wc_mclk => wc_mclk,
		 -- Audio sub-clocks straight from the wallclock NCO. Phase-coherent
		 -- with the fs/sample epoch (replaces the divider in
		 -- audioclock_generator(_sysclk) for the internal-NCO path).
		 wc_clk_256fs    => wc_clk_256fs,
		 wc_clk_128fs    => wc_clk_128fs,
		 wc_clk_64fs     => wc_clk_64fs_int,
		 wc_fs           => wc_fs_int,
		 wc_bclk_r       => wc_bclk_r_int,
		 wc_bclk_f       => wc_bclk_f_int,
		 wc_fs_pulse     => wc_fs_pulse,
		 wc_fs_tdm_pulse => wc_fs_tdm_int,
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


gen_ptp_metering: if (ENABLE_METERING = true) generate
servo_mon_filtered_offset_o      <= std_logic_vector(servo_mon_filtered_offset_signed);
servo_mon_integral_sum_o         <= std_logic_vector(servo_mon_integral_sum_signed);
servo_mon_pi_proportional_o      <= std_logic_vector(servo_mon_pi_proportional_signed);
servo_mon_pi_sum_raw_o           <= std_logic_vector(servo_mon_pi_sum_raw_signed);
servo_mon_effective_gain_shift_o <= std_logic_vector(servo_mon_effective_gain_shift_unsigned);
servo_mon_lock_counter_o         <= std_logic_vector(servo_mon_lock_counter_unsigned);
servo_mon_sample_count_o         <= std_logic_vector(servo_mon_sample_count_unsigned);
end generate; 










rx_ringbuffer_inst: entity work.rx_ringbuffer
GENERIC MAP(audio_buffer_sample_depth => RX_SAMPLE_BUFFER_DEPTH,
			bytes_per_sample => RX_BYTE_DEPTH,
			global_channel_count => RX_CHANNELS,
			max_streams => RX_MAX_STREAMS,
			ENABLE_METERING => ENABLE_METERING,
			PARALLEL_OUT => AUDIO_RX_USE_PARALLEL_INTERFACE,
			TDM_OUTPUTS => AUDIO_RX_TDM_OUTPUTS,
			TDM_CHANNELS => AUDIO_RX_TDM_CHANNELS
			)
PORT MAP(sys_clk => sys_clk_125MHz_i,
		 reset_n => rst_n,
		 

		 -- network
 		 eth_read_addr_o => rtp_ram_addr,
		 packet_ready_i => is_rtp_pkt_tog,
		 eth_read_data_i => eth_ram_data_sys_rtp,

		 -- clocking
		fs_clk_sync_i => pll_48k_fs_tdm,
		fs_clk_50duty_i => pll_48k_fs,
		bclk_sync_i => pll_256fs_rising,
		media_clock_i => media_clock,
		 
		 -- configuration
		 stream_config_addr_i => audio_rx_cfg_wr_addr_i,
		 stream_config_data_i => audio_rx_cfg_wr_data_i,
		 stream_config_wr_en_i => audio_rx_cfg_wr_en_i,
		 stream_config_wr_clk_i => clk_mcu_i,

		 -- audio
		 
		 audio_out => rx_sample_register,
		 tdm_out => tdm8out_o,

		 -- metering
		 metering_clear_i => audio_meter_clear_i,
		 metering_clip_o => audio_meter_rx_clip_o,
		 metering_signal_o => audio_meter_rx_signal_o);












ethernet_top_inst: entity work.ethernet_top
 GENERIC MAP(MIIM_CLOCK_DIVIDER => MIIM_CLOCK_DIVIDER,
 MIIM_PHY_ADDRESS => MIIM_PHY_ADDRESS,
 ETHERNET_TYPE => ETHERNET_TYPE)
 port map(
	sys_clk125MHz_i => sys_clk_125MHz_i,
	enet_clk_i => enet_clk_i,
	mac_rx_clock_o => mac_rx_clock,
	mac_tx_clock_o => mac_tx_clock,
	rst_n => rst_n,
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