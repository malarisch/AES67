LIBRARY ieee;
USE ieee.std_logic_1164.all; 
USE ieee.numeric_std.all;

use work.audioclks_pkg.all;
use work.wallclock_signals_pkg.all;

ENTITY ptp_module IS 
	generic (
        MII_WIDTH : integer := 2;
		ETHERNET_TYPE : string := "RMII";
        SYS_CLK_NS_PER_TICK : integer := 8; -- 125 MHz
        MII_CLK_NS_PER_TICK : integer := 20; -- 50 MHz
		STATIC_PTP_CONF : BOOLEAN := true;
		PTP_MOVING_AVERAGE_DEPTH : INTEGER := 8; 
		PTP_IN_SOFTWARE : BOOLEAN := false
	);
	PORT
	(
		sys_clk :  IN  STD_LOGIC;
		rst_n :  IN  STD_LOGIC;
		phy_mii_rx_clk_in : IN STD_LOGIC;
		phy_mii_tx_clk_in : IN STD_LOGIC;
		phy_mii_tx_data_in : IN STD_LOGIC_VECTOR(MII_WIDTH - 1 downto 0);
		phy_mii_rx_data_in : IN STD_LOGIC_VECTOR(MII_WIDTH - 1 downto 0);
		phy_gmii_tx_data_in : IN STD_LOGIC_VECTOR(7 downto 0);
		phy_gmii_rx_data_in : IN STD_LOGIC_VECTOR(7 downto 0);
		phy_mii_tx_en_i : IN STD_LOGIC;
		phy_mii_rx_en_i : IN STD_LOGIC;
		mac_link_up_i :  IN  STD_LOGIC;
		mac_tx_clock :  IN  STD_LOGIC;
		mac_tx_busy :  IN  STD_LOGIC;
		mac_tx_byte_sent :  IN  STD_LOGIC;
		mcu_tx_en_i : IN STD_LOGIC;
		rx_clk_i            : in std_logic;
        rx_data_i           : in STD_LOGIC_VECTOR(7 downto 0);
        rx_byte_received_i  : in std_logic;
        rx_byte_receive_index_i : in unsigned(7 downto 0); -- max length is 105 for ptpv2
        rx_ptp_frame_i      : in std_logic;

		ptp_is_leader_o :  OUT  STD_LOGIC;
		ptp_is_follower_o :  OUT  STD_LOGIC;
		mac_tx_allow_i :  IN  STD_LOGIC;
		ip_address :  IN  STD_LOGIC_VECTOR(31 DOWNTO 0);
		mac_address :  IN  STD_LOGIC_VECTOR(47 DOWNTO 0);
		ptp_announce_interval :  IN  STD_LOGIC_VECTOR(7 DOWNTO 0);
		ptp_clock_accuracy :  IN  STD_LOGIC_VECTOR(7 DOWNTO 0);
		ptp_clock_class :  IN  STD_LOGIC_VECTOR(7 DOWNTO 0);
		ptp_clock_priorityone :  IN  STD_LOGIC_VECTOR(7 DOWNTO 0);
		ptp_clock_prioritytwo :  IN  STD_LOGIC_VECTOR(7 DOWNTO 0);
		ptp_current_leader_id_o : OUT  STD_LOGIC_VECTOR(63 DOWNTO 0);
		ptp_log_message_interval :  IN  STD_LOGIC_VECTOR(7 DOWNTO 0);
		ptp_time_source :  IN  STD_LOGIC_VECTOR(7 DOWNTO 0);
		tx_en_ptpfu :  OUT  STD_LOGIC;
		ptp_allow_req :  OUT  STD_LOGIC;
		ptp_locked :  OUT  STD_LOGIC;
		wallclock_locked :  OUT  STD_LOGIC;
		wallclock_configured :  OUT  STD_LOGIC;
		
		audioclocks_o : OUT t_audio_clocks;
		wallclock_signals_io : INOUT t_wallclock_signals;
		timestamps_o : OUT t_eth_timestamps;

		second_pulse_sys :  OUT  STD_LOGIC;
		media_clock :  OUT  STD_LOGIC_VECTOR(31 DOWNTO 0);
		-- 1 sys-clk pulse on every media_clock increment (= media-clock fs tick).
		-- Used by the packetizer so its sample count and RTP timestamp share the
		-- media clock, independent of the NCO audio-sample boundary.
		media_tick :  OUT  STD_LOGIC;
		ptp_mean_path_delay :  OUT  STD_LOGIC_VECTOR(31 DOWNTO 0);
		ptp_offset_from_master :  OUT  STD_LOGIC_VECTOR(31 DOWNTO 0);
		ptp_ram_addr :  OUT  STD_LOGIC_VECTOR(10 DOWNTO 0);
		tx_data_ptpfu :  OUT  STD_LOGIC_VECTOR(7 DOWNTO 0);
		mac_speed_i : IN STD_LOGIC_VECTOR(1 downto 0);

		-- ============================================================
		-- Servo + parser tuning inputs (live-tunable from SoC)
		-- ============================================================
		servo_kp_gain_i              : IN signed(7 downto 0);
		servo_ki_gain_i              : IN signed(7 downto 0);
		servo_gain_shift_i           : IN unsigned(4 downto 0);
		servo_gain_shift_locked_i    : IN unsigned(4 downto 0);
		servo_ki_extra_shift_i       : IN unsigned(4 downto 0);
		servo_filter_shift_i         : IN unsigned(4 downto 0);
		servo_warmup_samples_i       : IN unsigned(7 downto 0);
		servo_lock_threshold_ns_i    : IN unsigned(31 downto 0);
		servo_unlock_threshold_ns_i  : IN unsigned(31 downto 0);
		servo_lock_count_threshold_i : IN unsigned(7 downto 0);


		-- IEEE 1588 delayAsymmetry (ns, signed). Compensates PHY/MAC
		-- TX vs RX latency mismatch. 0 = symmetric link.
		parser_delay_asymmetry_ns_i       : IN signed(31 downto 0) := (others => '0');

		-- ============================================================
		-- Servo monitoring outputs (live PI internal state)
		-- ============================================================
		servo_mon_filtered_offset_o      : OUT signed(31 downto 0);
		servo_mon_integral_sum_o         : OUT signed(31 downto 0);
		servo_mon_pi_proportional_o      : OUT signed(31 downto 0);
		servo_mon_pi_sum_raw_o           : OUT signed(31 downto 0);
		servo_mon_effective_gain_shift_o : OUT unsigned(7 downto 0);
		servo_mon_lock_counter_o         : OUT unsigned(15 downto 0);
		servo_mon_sample_count_o         : OUT unsigned(15 downto 0);
		servo_mon_first_lock_achieved_o  : OUT STD_LOGIC
	);
END ptp_module;

ARCHITECTURE bdf_type OF ptp_module IS 



SIGNAL	log_msg_interval :  SIGNED(7 DOWNTO 0);
SIGNAL	log_msg_interval_valid :  STD_LOGIC;
SIGNAL	powerGood :  STD_LOGIC;
SIGNAL	ptp_allow :  STD_LOGIC;
SIGNAL	ptp_calc_valid :  STD_LOGIC;
SIGNAL	ptp_locked_ALTERA_SYNTHESIZED :  STD_LOGIC;
SIGNAL	ptp_mean_path_delay_ALTERA_SYNTHESIZED :  SIGNED(31 DOWNTO 0);
SIGNAL	ptp_offset_from_master_ALTERA_SYNTHESIZED :  SIGNED(31 DOWNTO 0);
SIGNAL	rx_follower_identity :  STD_LOGIC_VECTOR(79 DOWNTO 0);
SIGNAL	rx_send_delay_req :  STD_LOGIC;
SIGNAL	rx_send_delay_resp :  STD_LOGIC;
SIGNAL	rx_sequence_id :  STD_LOGIC_VECTOR(15 DOWNTO 0);

SIGNAL	rx_ts_ns :  unsigned(29 DOWNTO 0);
SIGNAL	rx_ts_s :  UNSIGNED(3 DOWNTO 0);
SIGNAL	second_pulse_sys_ALTERA_SYNTHESIZED :  STD_LOGIC;
SIGNAL	tx_en_ptpfu_ALTERA_SYNTHESIZED :  STD_LOGIC;
SIGNAL	tx_frame_start :  STD_LOGIC;
SIGNAL	tx_msg_type :  STD_LOGIC_VECTOR(3 DOWNTO 0);
SIGNAL	tx_ptp_log_interval :  STD_LOGIC_VECTOR(7 DOWNTO 0);
SIGNAL	tx_req_port_identity :  STD_LOGIC_VECTOR(79 DOWNTO 0);
SIGNAL	tx_seq_id :  UNSIGNED(15 DOWNTO 0);
SIGNAL	tx_t3_valid :  STD_LOGIC;

SIGNAL	timestamp_ns :  UNSIGNED(29 DOWNTO 0);
SIGNAL	timestamp_s :  UNSIGNED(47 DOWNTO 0);
SIGNAL	ptp_ram_addr_u :  UNSIGNED(10 DOWNTO 0);
SIGNAL	media_clock_u :  UNSIGNED(31 DOWNTO 0);
SIGNAL	servo_request_clock_reconfigure :  STD_LOGIC;

SIGNAL tx_done_sys: STD_LOGIC := '0';
SIGNAL ms_pulse_sys : STD_LOGIC;

-- BMC signals
SIGNAL bmc_is_leader         : STD_LOGIC;
SIGNAL bmc_is_follower       : STD_LOGIC;
SIGNAL bmc_current_leader_id : STD_LOGIC_VECTOR(63 DOWNTO 0);
SIGNAL my_clock_identity     : STD_LOGIC_VECTOR(63 DOWNTO 0);

-- Effective selection inputs consumed by controller/parser: BMC output.
SIGNAL eff_is_leader         : STD_LOGIC;
SIGNAL eff_is_follower       : STD_LOGIC;
SIGNAL eff_current_leader_id : STD_LOGIC_VECTOR(63 DOWNTO 0);

-- Parser announce outputs
SIGNAL ann_valid            : STD_LOGIC;
SIGNAL ann_clock_identity   : STD_LOGIC_VECTOR(63 DOWNTO 0);
SIGNAL ann_priority1        : STD_LOGIC_VECTOR(7 DOWNTO 0);
SIGNAL ann_clock_class      : STD_LOGIC_VECTOR(7 DOWNTO 0);
SIGNAL ann_clock_accuracy   : STD_LOGIC_VECTOR(7 DOWNTO 0);
SIGNAL ann_oslv             : STD_LOGIC_VECTOR(15 DOWNTO 0);
SIGNAL ann_priority2        : STD_LOGIC_VECTOR(7 DOWNTO 0);
SIGNAL ann_steps_removed    : STD_LOGIC_VECTOR(15 DOWNTO 0);
SIGNAL ann_time_source      : STD_LOGIC_VECTOR(7 DOWNTO 0);
SIGNAL ann_log_msg_interval : STD_LOGIC_VECTOR(7 DOWNTO 0);

signal tx_en_switch : std_logic;

		-- ============================================================
		-- Servo + parser tuningputs (live-tunable from SoC)
		-- ============================================================
SIGNAL		servo_kp_gain_reg              : signed(7 downto 0);
SIGNAL		servo_ki_gain_reg              : signed(7 downto 0);
SIGNAL		servo_gain_shift_reg           : unsigned(4 downto 0);
SIGNAL		servo_gain_shift_locked_reg    : unsigned(4 downto 0);
SIGNAL		servo_ki_extra_shift_reg       : unsigned(4 downto 0);
SIGNAL		servo_filter_shift_reg         : unsigned(4 downto 0);
SIGNAL		servo_warmup_samples_reg       : unsigned(7 downto 0);
SIGNAL		servo_lock_threshold_ns_reg    : unsigned(31 downto 0);
SIGNAL		servo_unlock_threshold_ns_reg  : unsigned(31 downto 0);
SIGNAL		servo_lock_count_threshold_reg : unsigned(7 downto 0);


		-- IEEE 1588 delayAsymmetry (ns, signed). Compensates PHY/MAC
		-- TX vs RX latency mismatch. 0 = symmetric link.
SIGNAL		parser_delay_asymmetry_ns_reg       : signed(31 downto 0) := (others => '0');

		-- ============================================================
		-- Servo monitoringputs (live PIternal state)
		-- ============================================================
SIGNAL		servo_mon_filtered_offset_reg      : signed(31 downto 0);
SIGNAL		servo_mon_integral_sum_reg         : signed(31 downto 0);
SIGNAL		servo_mon_pi_proportional_reg      : signed(31 downto 0);
SIGNAL		servo_mon_pi_sum_raw_reg           : signed(31 downto 0);
SIGNAL		servo_mon_effective_gain_shift_reg : unsigned(7 downto 0);
SIGNAL		servo_mon_lock_counter_reg         : unsigned(15 downto 0);
SIGNAL		servo_mon_sample_count_reg         : unsigned(15 downto 0);
SIGNAL		servo_mon_first_lock_achieved_reg  : STD_LOGIC;


signal delta_m2s : signed(31 downto 0);
signal delta_s2m : signed(31 downto 0);
signal delta_m2s_avg : signed(31 downto 0);
signal delta_s2m_avg : signed(31 downto 0);
signal delta_m2s_valid : std_logic;
signal delta_s2m_valid : std_logic;
signal delta_m2s_valid_avg : std_logic;
signal delta_s2m_valid_avg : std_logic;

signal eth_timestamps : t_eth_timestamps;

signal wallclock_signals : t_wallclock_signals;

BEGIN


rx_tsu: entity work.ethernet_timestamp_mii
 generic map(
	PATH => "RX",
	MII_WIDTH => MII_WIDTH,
	SYS_CLK_NS_PER_TICK => SYS_CLK_NS_PER_TICK,
	MII_CLK_NS_PER_TICK => MII_CLK_NS_PER_TICK
)
 port map(
	sys_clk_i => sys_clk,
	mii_clk_i => phy_mii_rx_clk_in,
	reset_n => powerGood,
	wallclock_i => wallclock_signals,
	timestamp_o => eth_timestamps.rx,
	mii_in => phy_mii_rx_data_in,
	gmii_in => phy_gmii_rx_data_in,
	mii_en_in => phy_mii_rx_en_i
);

tx_tsu: entity work.ethernet_timestamp_mii
 generic map(
	PATH => "TX",
	MII_WIDTH => MII_WIDTH,
	SYS_CLK_NS_PER_TICK => SYS_CLK_NS_PER_TICK,
	MII_CLK_NS_PER_TICK => MII_CLK_NS_PER_TICK
)
 port map(
	sys_clk_i => sys_clk,
	mii_clk_i => phy_mii_tx_clk_in,
	reset_n => powerGood,
	wallclock_i => wallclock_signals,
	timestamp_o => eth_timestamps.tx,
	mii_in => phy_mii_tx_data_in,
	gmii_in => phy_gmii_tx_data_in,
	mii_en_in => tx_en_switch
);
ptp_sw_gen: if (PTP_IN_SOFTWARE = true) generate
	tx_en_switch <= mcu_tx_en_i;
	-- wallclock_signals is a mixed-direction record. The wallclock OUTPUTS (_o)
	-- flow out to the bridge (gettime / RX-TX timestamping); the control INPUTS
	-- (_i) come back from the bridge CSRs (settime / phasejump / ppb). Each
	-- direction must be assigned explicitly: a single `io <= internal` would
	-- drive the _i fields the wrong way, leaving the wallclock inputs unsourced
	-- (Quartus warning 12161: "stuck at GND ... node is in wire loop").
	wallclock_signals_io.wallclock_seconds_o     <= wallclock_signals.wallclock_seconds_o;
	wallclock_signals_io.wallclock_nanoseconds_o <= wallclock_signals.wallclock_nanoseconds_o;

	wallclock_signals.wallclock_seconds_i      <= wallclock_signals_io.wallclock_seconds_i;
	wallclock_signals.wallclock_nanoseconds_i  <= wallclock_signals_io.wallclock_nanoseconds_i;
	wallclock_signals.wallclock_set_i          <= wallclock_signals_io.wallclock_set_i;
	wallclock_signals.wallclock_do_phasejump_i <= wallclock_signals_io.wallclock_do_phasejump_i;
	wallclock_signals.freq_correction_ppb_i    <= wallclock_signals_io.freq_correction_ppb_i;

	timestamps_o <= eth_timestamps;
	tx_en_ptpfu <= '0';
	ptp_allow_req <= '0';
	tx_data_ptpfu <= (others => '0');
end generate;

ptp_hw_gen: if (PTP_IN_SOFTWARE = false) generate
tx_en_switch <= tx_en_ptpfu_ALTERA_SYNTHESIZED;
-- EUI-64 clock identity from MAC (same mapping as ptpv2_parser)
my_clock_identity <= (mac_address(47 downto 40) XOR x"02")
                   & mac_address(39 downto 24)
                   & x"FFFE"
                   & mac_address(23 downto 0);

eff_is_leader         <= bmc_is_leader;
eff_is_follower       <= bmc_is_follower;
eff_current_leader_id <= bmc_current_leader_id;

ptp_is_leader_o <= bmc_is_leader;
ptp_is_follower_o <= bmc_is_follower;
ptp_current_leader_id_o <= bmc_current_leader_id;

b2v_controller : entity work.ptpv2_controller
PORT MAP(clk => sys_clk,
		 reset_n => powerGood,
		 send_delay_resp_in => rx_send_delay_resp,
		 wait_amount_i => unsigned(mac_address(8 downto 3)), -- just use some lower bits of the mac adress for wait timer since it's different for every device
		 ms_pulse_i => ms_pulse_sys,
		 is_leader_i => eff_is_leader,
		 is_follower_i => eff_is_follower,
		 tx_en_i => tx_en_ptpfu_ALTERA_SYNTHESIZED,
		 send_delay_req_i => second_pulse_sys_ALTERA_SYNTHESIZED,
		 ptp_announce_log_message_interval_i => ptp_announce_interval,
		 ptp_log_message_interval_i => ptp_log_message_interval,
		 request_port_identity_i => rx_follower_identity,
		 rx_timestamp_nanoseconds_i => rx_ts_ns,
		 rx_timestamp_seconds_i => rx_ts_s,
		 sequence_id_i => unsigned(rx_sequence_id),
		 wallclock_nanoseconds_i => wallclock_signals.wallclock_nanoseconds_o,
		 wallclock_seconds_i => wallclock_signals.wallclock_seconds_o,
		 frame_start_o => tx_frame_start,
		 t3_valid_o => tx_t3_valid,
		 ptp_log_interval_o => tx_ptp_log_interval,
		 request_port_identity_o => tx_req_port_identity,
		 sequence_id_o => tx_seq_id,
		 timestamp_nanoseconds_o => timestamp_ns,
		 timestamp_seconds_o => timestamp_s,
		 tx_timestamp_nanoseconds_i => eth_timestamps.tx.nanoseconds,
		 tx_timestamp_seconds_i => eth_timestamps.tx.seconds,
		 tx_message_type_o => tx_msg_type,
		 tx_done_sys_i => tx_done_sys,
		 parser_log_msg_interval_i => STD_LOGIC_VECTOR(log_msg_interval));


wallclock_locked <= eff_is_leader OR ptp_locked_ALTERA_SYNTHESIZED;


b2v_pptx :  entity work.ptpv2_sender
PORT MAP(sys_clk => sys_clk,
		 frame_start => tx_frame_start,
		 tx_clk => mac_tx_clock,
		 tx_busy => mac_tx_busy,
		 tx_byte_sent => mac_tx_byte_sent,
		 tx_allow_i => ptp_allow,
		 message_type_i => tx_msg_type,
		 ptp_clockaccuracy => ptp_clock_accuracy,
		 ptp_clockclass => ptp_clock_class,
		 ptp_log_interval_i => tx_ptp_log_interval,
		 ptp_prioone => ptp_clock_priorityone,
		 ptp_priotwo => ptp_clock_prioritytwo,
		 ptp_time_source_i => ptp_time_source,
		 request_port_identity => tx_req_port_identity,
		 sequence_id => tx_seq_id,
		 src_ip_address => ip_address,
		 src_mac_address => mac_address,
		 timestamp_nanoseconds_i => timestamp_ns,
		 timestamp_seconds_i => timestamp_s,
		 tx_enable => tx_en_ptpfu_ALTERA_SYNTHESIZED,
		 tx_allow_req_o => ptp_allow_req,
		 tx_data => tx_data_ptpfu,
		 tx_done_sys_o => tx_done_sys,
		 mac_speed_i => mac_speed_i);

s2m_average_inst: entity work.average
 generic map(
	DATA_WIDTH => 32,
	DEPTH => PTP_MOVING_AVERAGE_DEPTH
)
 port map(
	clk_i => sys_clk,
	rst_n_i => powerGood,
	data_i => delta_s2m,
	data_valid_i => delta_s2m_valid,
	data_o => delta_s2m_avg,
	data_valid_o => delta_s2m_valid_avg
);
m2s_average_inst: entity work.average
 generic map(
	DATA_WIDTH => 32,
	DEPTH => PTP_MOVING_AVERAGE_DEPTH
)
 port map(
	clk_i => sys_clk,
	rst_n_i => powerGood,
	data_i => delta_m2s,
	data_valid_i => delta_m2s_valid,
	data_o => delta_m2s_avg,
	data_valid_o => delta_m2s_valid_avg
);

ptp_calc_proc: process (sys_clk, rst_n)
begin
if (rst_n = '0') then
	ptp_calc_valid <= '0';
	ptp_mean_path_delay_ALTERA_SYNTHESIZED <= (others => '0');
	ptp_offset_from_master_ALTERA_SYNTHESIZED <= (others => '0');
elsif (rising_edge(sys_clk)) then
	if (ptp_locked_ALTERA_SYNTHESIZED = '1') then
		ptp_mean_path_delay_ALTERA_SYNTHESIZED <= shift_right(delta_m2s_avg + delta_s2m_avg - parser_delay_asymmetry_ns_i, 1);
		ptp_offset_from_master_ALTERA_SYNTHESIZED <= shift_right(delta_m2s_avg - delta_s2m_avg + parser_delay_asymmetry_ns_i, 1);
	else
		ptp_mean_path_delay_ALTERA_SYNTHESIZED <= shift_right(delta_m2s + delta_s2m - parser_delay_asymmetry_ns_i, 1);
		ptp_offset_from_master_ALTERA_SYNTHESIZED <= shift_right(delta_m2s - delta_s2m + parser_delay_asymmetry_ns_i, 1);
	end if;
	ptp_calc_valid <= delta_m2s_valid_avg;
end if;
end process;

b2v_ptpparser :  entity work.ptpv2_parser

PORT MAP(clk => sys_clk,
		 is_leader => eff_is_leader,
		 reset_n => powerGood,
		 t3_valid_i => tx_t3_valid,
		 ptp_is_follower_i => eff_is_follower,
		 ptp_current_leader_id_i => eff_current_leader_id,
		 clock_reconfigure_req_i => servo_request_clock_reconfigure,
		 rx_timestamp_nanoseconds_i => eth_timestamps.rx.nanoseconds,
		 rx_timestamp_seconds_i => eth_timestamps.rx.seconds,
		 src_mac_address => mac_address,
		 tx_timestamp_nanoseconds_i => eth_timestamps.tx.nanoseconds,
		 tx_timestamp_seconds_i => eth_timestamps.tx.seconds,
		 send_delay_resp_o => rx_send_delay_resp,
		 send_delay_req_o => rx_send_delay_req,
		 --ptp_calc_valid_o => ptp_calc_valid,
		 log_msg_interval_valid_o => log_msg_interval_valid,
		 clock_set_o => wallclock_signals.wallclock_set_i,
		 clock_configured_o => wallclock_configured,
		 clock_configure_timestamp_nanoseconds_o => wallclock_signals.wallclock_nanoseconds_i,
		 clock_configure_timestamp_seconds_o => wallclock_signals.wallclock_seconds_i,
		 log_msg_interval_o => log_msg_interval,
		 delta_m2s_o => delta_m2s,--, => ptp_mean_path_delay_ALTERA_SYNTHESIZED,
		 delta_s2m_o => delta_s2m,
		 delta_m2s_valid_o => delta_m2s_valid,
		 delta_s2m_valid_o => delta_s2m_valid,
		 --offset_from_master_ns_o => ptp_offset_from_master_ALTERA_SYNTHESIZED,
		 rx_follower_identity_o => rx_follower_identity,
		 rx_timestamp_nanoseconds_o => rx_ts_ns,
		 rx_timestamp_seconds_o => rx_ts_s,
		 sequence_id_o => rx_sequence_id,
		 announce_valid_o => ann_valid,
		 announce_clock_identity_o => ann_clock_identity,
		 announce_priority1_o => ann_priority1,
		 announce_clock_class_o => ann_clock_class,
		 announce_clock_accuracy_o => ann_clock_accuracy,
		 announce_offset_scaled_log_var_o => ann_oslv,
		 announce_priority2_o => ann_priority2,
		 announce_steps_removed_o => ann_steps_removed,
		 announce_time_source_o => ann_time_source,
		 announce_log_msg_interval_o => ann_log_msg_interval,
		 
		 rx_clk_i => rx_clk_i,
		 rx_data_i => rx_data_i,
		 rx_byte_receive_index_i => rx_byte_receive_index_i,
		 rx_byte_received_i => rx_byte_received_i,
		 rx_ptp_frame_i => rx_ptp_frame_i
		 );


b2v_bmc :  entity work.ptpv2_bmc
PORT MAP(clk => sys_clk,
		 reset_n => powerGood,
		 ms_pulse_i => ms_pulse_sys,
		 mac_link_up_i => mac_link_up_i,
		 my_clock_identity_i => my_clock_identity,
		 my_priority1_i => ptp_clock_priorityone,
		 my_clock_class_i => ptp_clock_class,
		 my_clock_accuracy_i => ptp_clock_accuracy,
		 my_offset_scaled_log_var_i => x"FFFF",
		 my_priority2_i => ptp_clock_prioritytwo,
		 announce_valid_i => ann_valid,
		 announce_clock_identity_i => ann_clock_identity,
		 announce_priority1_i => ann_priority1,
		 announce_clock_class_i => ann_clock_class,
		 announce_clock_accuracy_i => ann_clock_accuracy,
		 announce_offset_scaled_log_var_i => ann_oslv,
		 announce_priority2_i => ann_priority2,
		 announce_log_msg_interval_i => ann_log_msg_interval,
		 is_leader_o => bmc_is_leader,
		 is_follower_o => bmc_is_follower,
		 current_leader_id_o => bmc_current_leader_id);



static_ptp_conf_gen: if (STATIC_PTP_CONF) generate
static_ptp_conf_inst: entity work.static_ptp_conf
 generic map(
	ETHERNET_TYPE => ETHERNET_TYPE
)
 port map(
	servo_kp_gain_o => servo_kp_gain_reg,
	servo_ki_gain_o => servo_ki_gain_reg,
	servo_gain_shift_o => servo_gain_shift_reg,
	servo_gain_shift_locked_o => servo_gain_shift_locked_reg,
	servo_ki_extra_shift_o => servo_ki_extra_shift_reg,
	servo_filter_shift_o => servo_filter_shift_reg,
	servo_warmup_samples_o => servo_warmup_samples_reg,
	servo_lock_threshold_ns_o => servo_lock_threshold_ns_reg,
	servo_unlock_threshold_ns_o => servo_unlock_threshold_ns_reg,
	servo_lock_count_threshold_o => servo_lock_count_threshold_reg,
	parser_delay_asymmetry_ns_o => parser_delay_asymmetry_ns_reg
);
 end generate;
dynamic_ptp_conf_gen: if (NOT STATIC_PTP_CONF) generate
servo_kp_gain_reg <= servo_kp_gain_i;
servo_ki_gain_reg <= servo_ki_gain_i;
servo_gain_shift_reg <= servo_gain_shift_i;
servo_gain_shift_locked_reg <= servo_gain_shift_locked_i;
servo_ki_extra_shift_reg <= servo_ki_extra_shift_i;
servo_filter_shift_reg <= servo_filter_shift_i;
servo_warmup_samples_reg <= servo_warmup_samples_i;
servo_lock_threshold_ns_reg <= servo_lock_threshold_ns_i;
servo_unlock_threshold_ns_reg <= servo_unlock_threshold_ns_i;
servo_lock_count_threshold_reg <= servo_lock_count_threshold_i;


servo_mon_filtered_offset_o <= servo_mon_filtered_offset_reg;
servo_mon_integral_sum_o <= servo_mon_integral_sum_reg;
servo_mon_pi_proportional_o <= servo_mon_pi_proportional_reg;
servo_mon_pi_sum_raw_o <= servo_mon_pi_sum_raw_reg;
servo_mon_effective_gain_shift_o <= servo_mon_effective_gain_shift_reg;
servo_mon_sample_count_o <= servo_mon_lock_counter_reg;
servo_mon_sample_count_o <= servo_mon_sample_count_reg;
servo_mon_first_lock_achieved_o <= servo_mon_first_lock_achieved_reg;
end generate;

	
b2v_servo :  entity work.ptpv2_servo
PORT MAP(clk => sys_clk,
		 reset_n => powerGood,
		 calc_valid_i => ptp_calc_valid,
		 log_msg_interval_valid_i => log_msg_interval_valid,
		 log_msg_interval_i => log_msg_interval,
		 offset_from_master_i => ptp_offset_from_master_ALTERA_SYNTHESIZED,
		 
		 locked_o => ptp_locked_ALTERA_SYNTHESIZED,
		 freq_correction_o => wallclock_signals.freq_correction_ppb_i,
		 
		 request_clock_reconfigure_o => servo_request_clock_reconfigure,
		 -- Live-tuning inputs from top-level
		 kp_gain_i              => servo_kp_gain_reg,
		 ki_gain_i              => servo_ki_gain_reg,
		 gain_shift_i           => servo_gain_shift_reg,
		 gain_shift_locked_i    => servo_gain_shift_locked_reg,
		 ki_extra_shift_i       => servo_ki_extra_shift_reg,
		 filter_shift_i         => servo_filter_shift_reg,
		 warmup_samples_i       => servo_warmup_samples_reg,
		 lock_threshold_ns_i    => servo_lock_threshold_ns_reg,
		 unlock_threshold_ns_i  => servo_unlock_threshold_ns_reg,
		 lock_count_threshold_i => servo_lock_count_threshold_reg,
		 -- Monitoring outputs to top-level
		 mon_filtered_offset_o      => servo_mon_filtered_offset_reg,
		 mon_integral_sum_o         => servo_mon_integral_sum_reg,
		 mon_pi_proportional_o      => servo_mon_pi_proportional_reg,
		 mon_pi_sum_raw_o           => servo_mon_pi_sum_raw_reg,
		 mon_effective_gain_shift_o => servo_mon_effective_gain_shift_reg,
		 mon_lock_counter_o         => servo_mon_lock_counter_reg,
		 mon_sample_count_o         => servo_mon_sample_count_reg,
		 mon_first_lock_achieved_o  => servo_mon_first_lock_achieved_reg);

tx_en_ptpfu <= tx_en_ptpfu_ALTERA_SYNTHESIZED;
ptp_allow <= mac_tx_allow_i;
ptp_locked <= ptp_locked_ALTERA_SYNTHESIZED;
media_clock <= std_logic_vector(media_clock_u);
ptp_mean_path_delay <= std_logic_vector(ptp_mean_path_delay_ALTERA_SYNTHESIZED);
ptp_offset_from_master <= std_logic_vector(ptp_offset_from_master_ALTERA_SYNTHESIZED);
ptp_ram_addr <= std_logic_vector(ptp_ram_addr_u);
end generate;

media_clock <= std_logic_vector(media_clock_u);
second_pulse_sys <= second_pulse_sys_ALTERA_SYNTHESIZED;

b2v_wallclock :  entity work.wallclock
GENERIC MAP(audio_fs => 48000,
			increment_interval => 8,
			sys_clk_hz => 125000000
			)
PORT MAP(clk => sys_clk,
		 reset_n => powerGood,
		 wallclock_signals => wallclock_signals,
		 second_pulse_o => second_pulse_sys_ALTERA_SYNTHESIZED,
		 ms_pulse_o => ms_pulse_sys,
		 clocks_o => audioclocks_o,
		 media_clock_o => media_clock_u,
		 mclk_cnt_o => open,
		 media_edge_tick_o => media_tick,
		 ppb_adj_dbg_o => open,
		 ppb_trim_dbg_o => open,
		 bias_dbg_o => open,
		 sample_pulse_int_o => open,
		 nco_phase_dbg_o => open,
		 nco_inc_dbg_o => open);

powerGood <= rst_n;



END bdf_type;