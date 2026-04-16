-- top level for the logic core. needs external RGMII / RMII to MII Converters and clocks.
-- exposes an ethernet interface for mcu connectivity
LIBRARY ieee;
USE ieee.std_logic_1164.all; 
use IEEE.NUMERIC_STD.all;
 

ENTITY aes67_top IS 
	generic (
		TX_MAX_STREAMS : natural := 8;
		RX_MAX_STREAMS : natural := 8;
		RX_CHANNELS		: natural := 16;
		TX_CHANNELS		: natural := 16;
		RX_BYTE_DEPTH	: natural := 3;
		TX_BYTE_DEPTH	: natural := 3;
		RX_SAMPLE_BUFFER_DEPTH : natural := 96;
		TX_SAMPLE_BUFFER_DEPTH : natural := 48;
		  
    MIIM_CLOCK_DIVIDER : POSITIVE := 50
  
	);
	PORT
	(
		-- system clocks
		sys_clk_125MHz_i :  IN  STD_LOGIC;
		enet_clk_i       :  IN  STD_LOGIC;
		clk_mcu_i	   :  IN  STD_LOGIC;
		rst_n		: IN STD_LOGIC;
		mac_resetn_i : IN STD_LOGIC;

		

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
		mac_sof_sent_pulse_o: OUT STD_LOGIC;
		-- audio clocks

		pll_512fs_i : IN STD_LOGIC;
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
		ptp_current_leader_id_i : IN STD_LOGIC_VECTOR(63 downto 0);
		ptp_log_message_interval_i : IN STD_LOGIC_VECTOR(7 downto 0);
		ptp_time_source_i : IN STD_LOGIC_VECTOR(7 downto 0);
		ptp_is_follower_i : IN STD_LOGIC;
		ptp_is_leader_i : IN STD_LOGIC;
		ptp_sync_lost_o : OUT STD_LOGIC;
		ptp_mean_path_delay_o : OUT STD_LOGIC_VECTOR(31 downto 0);
		ptp_offset_from_master_o : OUT STD_LOGIC_VECTOR(31 downto 0);
		wallclock_locked_o : OUT STD_LOGIC;
		wallclock_configured_o : OUT STD_LOGIC;
		wallclock_phasejump_o : OUT STD_LOGIC;

		-- ppb meter for external pll
		ppb_meter_start_i : IN STD_LOGIC;
		pll_meas_valid_o : OUT STD_LOGIC;
		pll_counter_o : OUT unsigned(21 downto 0);
		wc_counter_o : OUT unsigned(21 downto 0);
		

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
		tdm8out_0_o : OUT STD_LOGIC;
		tdm8out_1_o : OUT STD_LOGIC;

		-- audio inputs
		tdm8in_0_i : IN STD_LOGIC;
		tdm8in_1_i : IN STD_LOGIC

	);
END aes67_top;

ARCHITECTURE rtl OF aes67_top IS 


-- audio clocks
SIGNAL pll_64fs : STD_LOGIC;
SIGNAL pll_48k_fs : STD_LOGIC;
SIGNAL pll_48k_fs_tdm : STD_LOGIC;

signal pll_256fs_falling : STD_LOGIC;
signal pll_256fs_rising : STD_LOGIC;

signal wc_64fs : STD_LOGIC; -- nco generated from wallclock
signal media_clock : STD_LOGIC_VECTOR(31 downto 0);
signal second_pulse_sys : STD_LOGIC;

-- sample registers for in/output
type t_rx_sample_register is array (0 to RX_CHANNELS) of std_logic_vector(RX_BYTE_DEPTH*8-1 downto 0);
    signal rx_sample_register : t_rx_sample_register := (others => (others => '0'));
type t_tx_sample_register is array (0 to TX_CHANNELS) of std_logic_vector(TX_BYTE_DEPTH*8-1 downto 0);
    signal tx_sample_register : t_tx_sample_register := (others => (others => '0'));


signal mac_tx_clock : STD_LOGIC;

-- ethernet_top signals

signal is_rtp_pkt_tog : STD_LOGIC;
signal is_ptp_pkt_tog : STD_LOGIC;
signal ptp_ram_addr : STD_LOGIC_VECTOR(10 downto 0);
signal rtp_ram_addr : unsigned(10 downto 0);
signal eth_ram_data_sys_rtp : STD_LOGIC_VECTOR(7 downto 0);
signal eth_ram_data_sys_ptp : STD_LOGIC_VECTOR(7 downto 0);
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
BEGIN 

pll_64fs_o <= pll_64fs;
pll_48k_fs_o <= pll_48k_fs;
pll_48k_fs_tdm_o <= pll_48k_fs_tdm;
pll_256fs_rising_o <= pll_256fs_rising;
pll_256fs_falling_o <= pll_256fs_falling;

mac_tx_clock_o <= mac_tx_clock;
mac_reset <= not mac_resetn_i;

mac_tx_busy_o <= mac_tx_busy;
mac_tx_byte_sent_o <= mac_tx_byte_sent;

audioclocks_inst: entity work.audioclock_generator
PORT MAP(mclk => pll_512fs_i,
		 rst_n => rst_n,
		 clk_64fs => pll_64fs,
		 fs => pll_48k_fs,
		 bclk_r => pll_256fs_rising,
		 bclk_f => pll_256fs_falling,
		 fs_pulse => pll_48k_fs_tdm);


audiotx_inst: entity work.audio_tx_module
GENERIC MAP(bytes_per_sample => TX_BYTE_DEPTH,
			global_channel_count => TX_CHANNELS,
			samples_per_channel_depth => TX_SAMPLE_BUFFER_DEPTH,
			max_streams => TX_MAX_STREAMS
			)
PORT MAP(sys_clk => sys_clk_125MHz_i,
		 fmc_clk => clk_mcu_i,
		 rst_n => rst_n,
		 fs_clk_i => pll_48k_fs,
		 
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


		 ch0i => tx_sample_register(0),
		 ch1i => tx_sample_register(1),
		 ch2i => tx_sample_register(2),
		 ch3i => tx_sample_register(3),
		 ch4i => tx_sample_register(4),
		 ch5i => tx_sample_register(5),
		 ch6i => tx_sample_register(6),
		 ch7i => tx_sample_register(7),
		 ch8i => tx_sample_register(8),
		 ch9i => tx_sample_register(9),
		 ch10i => tx_sample_register(10),
		 ch11i => tx_sample_register(11),
		 ch12i => tx_sample_register(12),
		 ch13i => tx_sample_register(13),
		 ch14i => tx_sample_register(14),
		 ch15i => tx_sample_register(15),
		 
		 media_clock_i => media_clock,


 		 metering_clear_i => audio_meter_clear_i,
		 metering_clip_o => audio_meter_tx_clip_o,
		 metering_signal_o => audio_meter_tx_signal_o,
		 tx_data_o => eth_tx_data_rtp);

















ppb_meter_inst: entity work.clock_ppb_meter
PORT MAP(sys_clk => sys_clk_125MHz_i,
		 reset_n => rst_n,
		 wallclock_64fs_in => wc_64fs,
		 pll_64fs_in => pll_64fs,
		 wallclock_second_pulse_i => second_pulse_sys,
		 start_i => ppb_meter_start_i,
		 valid_o => pll_meas_valid_o,
		 count_pll_o => pll_counter_o,
		 count_wc_o => wc_counter_o);


ptp_inst: entity work.ptp_module
PORT MAP(sys_clk => sys_clk_125MHz_i,
		 rst_n => rst_n,

		 -- mac signals
		 mac_tx_clock => mac_tx_clock,
		 mac_tx_busy => mac_tx_busy,
		 mac_tx_byte_sent => mac_tx_byte_sent,
		 mac_tx_allow_i => eth_tx_allow_ptp,
		sof_recv_tog_i => mac_sof_recv_tog,
		sof_sent_tog_i => mac_sof_sent_tog,
		parse_ptp_packet_tog => is_ptp_pkt_tog,
		tx_data_ptpfu => eth_tx_data_ptp,
		mac_ram_data => eth_ram_data_sys_ptp,
		
		tx_en_ptpfu => eth_tx_en_ptp,
		 ptp_allow_req => eth_tx_req_ptp,
		ptp_ram_addr => ptp_ram_addr,
		
		 
		-- configuration registers 
		ip_address => ip_address_i,
		mac_address => mac_address_i,

		 ptp_announce_interval => ptp_announce_interval_i,
		 ptp_clock_accuracy => ptp_clock_accuracy_i,
		 ptp_clock_class => ptp_clock_class_i,
		 ptp_clock_priorityone => ptp_gm_prioone_i,
		 ptp_clock_prioritytwo => ptp_gm_priotwo_i,
		 ptp_current_leader_id => ptp_current_leader_id_i,
		 ptp_log_message_interval => ptp_log_message_interval_i,
		 ptp_time_source => ptp_time_source_i,
		ptp_is_follower => ptp_is_follower_i,
		ptp_is_leader => ptp_is_leader_i,
		 ptp_sync_lost => ptp_sync_lost_o,
		 wallclock_locked => wallclock_locked_o,
		 wallclock_configured => wallclock_configured_o,
		 wallclock_phasejump => wallclock_phasejump_o,
		 
		 
		 ptp_mean_path_delay => ptp_mean_path_delay_o,
		 ptp_offset_from_master => ptp_offset_from_master_o,
		 

		 -- generated clocks
		 wc_64fs => wc_64fs,
		 second_pulse_sys => second_pulse_sys,
		 media_clock => media_clock
		 );
		 










rx_ringbuffer_inst: entity work.rx_ringbuffer
GENERIC MAP(audio_buffer_sample_depth => RX_SAMPLE_BUFFER_DEPTH,
			bytes_per_sample => RX_BYTE_DEPTH,
			global_channel_count => RX_CHANNELS,
			max_streams => RX_MAX_STREAMS
			
			)
PORT MAP(sys_clk => sys_clk_125MHz_i,
		 reset_n => rst_n,
		 

		 -- network
 		 eth_read_addr_o => rtp_ram_addr,
		 packet_ready_i => is_rtp_pkt_tog,
		 eth_read_data_i => eth_ram_data_sys_rtp,

		 -- clocking
		fs_clk_i => pll_48k_fs,
		media_clock_i => media_clock,
		 
		 -- configuration
		 stream_config_addr_i => audio_rx_cfg_wr_addr_i,
		 stream_config_data_i => audio_rx_cfg_wr_data_i,
		 stream_config_wr_en_i => audio_rx_cfg_wr_en_i,
		 stream_config_wr_clk_i => clk_mcu_i,

		 -- audio
		 -- TODO: MAKE THIS DYNAMIC!
		 audio_ch0_out => rx_sample_register(0),
		 audio_ch1_out => rx_sample_register(1),
		 audio_ch2_out => rx_sample_register(2),
		 audio_ch3_out => rx_sample_register(3),
		 audio_ch4_out => rx_sample_register(4),
		 audio_ch5_out => rx_sample_register(5),
		 audio_ch6_out => rx_sample_register(6),
		 audio_ch7_out => rx_sample_register(7),
		 audio_ch8_out => rx_sample_register(8),
		 audio_ch9_out => rx_sample_register(9),
		 audio_ch10_out => rx_sample_register(10),
		 audio_ch11_out => rx_sample_register(11),
		 audio_ch12_out => rx_sample_register(12),
		 audio_ch13_out => rx_sample_register(13),
		 audio_ch14_out => rx_sample_register(14),
		 audio_ch15_out => rx_sample_register(15),

		 -- metering
		 metering_clear_i => audio_meter_clear_i,
		 metering_clip_o => audio_meter_rx_clip_o,
		 metering_signal_o => audio_meter_rx_signal_o);




tdm8_mux1_inst: entity work.tdm8_out
GENERIC MAP(width => 24
			)
PORT MAP(FSYNC => pll_48k_fs_tdm,
		 BIT_CLK => pll_256fs_falling,
		 RESET => rst_n,
		 DATA_CH0 => rx_sample_register(0),
		 DATA_CH1 => rx_sample_register(1),
		 DATA_CH2 => rx_sample_register(2),
		 DATA_CH3 => rx_sample_register(3),
		 DATA_CH4 => rx_sample_register(4),
		 DATA_CH5 => rx_sample_register(5),
		 DATA_CH6 => rx_sample_register(6),
		 DATA_CH7 => rx_sample_register(7),
		 DOUT => tdm8out_0_o);


tdm8_mux2_inst: entity work.tdm8_out
GENERIC MAP(width => 24
			)
PORT MAP(FSYNC => pll_48k_fs_tdm,
		 BIT_CLK => pll_256fs_falling,
		 RESET => rst_n,
		 DATA_CH0 => rx_sample_register(8),
		 DATA_CH1 => rx_sample_register(9),
		 DATA_CH2 => rx_sample_register(10),
		 DATA_CH3 => rx_sample_register(11),
		 DATA_CH4 => rx_sample_register(12),
		 DATA_CH5 => rx_sample_register(13),
		 DATA_CH6 => rx_sample_register(14),
		 DATA_CH7 => rx_sample_register(15),
		 DOUT => tdm8out_1_o);


tdm8demux1_inst: entity work.tdm8_in
GENERIC MAP(width => 24
			)
PORT MAP(FSYNC => pll_48k_fs_tdm,
		 BIT_CLK => pll_256fs_falling,
		 DIN => tdm8in_0_i,
		 RESET => rst_n,
		 SYS_CLK => sys_clk_125MHz_i,
		 DATA_CH0 => tx_sample_register(0),
		 DATA_CH1 => tx_sample_register(1),
		 DATA_CH2 => tx_sample_register(2),
		 DATA_CH3 => tx_sample_register(3),
		 DATA_CH4 => tx_sample_register(4),
		 DATA_CH5 => tx_sample_register(5),
		 DATA_CH6 => tx_sample_register(6),
		 DATA_CH7 => tx_sample_register(7)
);

tdm8demux2_inst: entity work.tdm8_in
GENERIC MAP(width => 24
			)
PORT MAP(FSYNC => pll_48k_fs_tdm,
		 BIT_CLK => pll_256fs_falling,
		 DIN => tdm8in_1_i,
		 RESET => rst_n,
		 SYS_CLK => sys_clk_125MHz_i,
		 DATA_CH0 => tx_sample_register(8),
		 DATA_CH1 => tx_sample_register(9),
		 DATA_CH2 => tx_sample_register(10),
		 DATA_CH3 => tx_sample_register(11),
		 DATA_CH4 => tx_sample_register(12),
		 DATA_CH5 => tx_sample_register(13),
		 DATA_CH6 => tx_sample_register(14),
		 DATA_CH7 => tx_sample_register(15)
);

ethernet_top_inst: entity work.ethernet_top
 GENERIC MAP(MIIM_CLOCK_DIVIDER => MIIM_CLOCK_DIVIDER)
 port map(
	sys_clk125MHz_i => sys_clk_125MHz_i,
	enet_clk_i => enet_clk_i,
	mac_rx_clock_o => mac_rx_clock_o,
	mac_tx_clock_o => mac_tx_clock,
	rst_n => rst_n,
	received_packet_length_o => mac_received_packet_length_o,
	is_mcu_pkt_tog_o => is_mcu_pkt_tog_o,
	is_rtp_pkt_tog_o => is_rtp_pkt_tog,
	is_ptp_pkt_tog_o => is_ptp_pkt_tog,
	ptp_ram_addr_i => unsigned(ptp_ram_addr),
	rtp_ram_addr_i => rtp_ram_addr,
	mcu_ram_addr_i => mcu_ram_addr_i,
	eth_ram_data_sys_rtp_o => eth_ram_data_sys_rtp,
	eth_ram_data_sys_ptp_o => eth_ram_data_sys_ptp,
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
	mac_speed_o => mac_speed_o,
	mac_linkup_o => mac_linkup_o,
	mac_tx_byte_sent_o => mac_tx_byte_sent,
	mac_tx_reset_o => mac_tx_reset_o,
	mac_tx_busy_o => mac_tx_busy,
	mac_rx_reset_o => mac_rx_reset,
	mac_sof_sent_tog_o => mac_sof_sent_tog,
	mac_sof_sent_pulse_o => mac_sof_sent_pulse_o,
	mac_sof_recv_tog_o => mac_sof_recv_tog,
	mii_rx_clock_i => mii_rx_clock_i,
	mii_tx_clock_i => mii_tx_clock_i,
	mii_rx_err_i => mii_rx_err_i,
	mii_rx_dv_i => mii_rx_dv_i,
	mii_rxd_i => mii_rxd_i,
	mii_tx_err_o => mii_tx_err_o,
	mii_tx_en_o => mii_tx_en_o,
	mii_txd_o => mii_txd_o,
	enet_mdio => enet_mdio,
	enet_mdc => enet_mdc
);


END rtl;