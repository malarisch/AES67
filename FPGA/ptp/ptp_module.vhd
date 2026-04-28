-- Copyright (C) 2025  Altera Corporation. All rights reserved.
-- Your use of Altera Corporation's design tools, logic functions 
-- and other software and tools, and any partner logic 
-- functions, and any output files from any of the foregoing 
-- (including device programming or simulation files), and any 
-- associated documentation or information are expressly subject 
-- to the terms and conditions of the Altera Program License 
-- Subscription Agreement, the Altera Quartus Prime License Agreement,
-- the Altera IP License Agreement, or other applicable license
-- agreement, including, without limitation, that your use is for
-- the sole purpose of programming logic devices manufactured by
-- Altera and sold by Altera or its authorized distributors.  Please
-- refer to the Altera Software License Subscription Agreements 
-- on the Quartus Prime software download page.

-- PROGRAM		"Quartus Prime"
-- VERSION		"Version 25.1std.0 Build 1129 10/21/2025 SC Lite Edition"
-- CREATED		"Sat Apr  4 16:30:01 2026"

LIBRARY ieee;
USE ieee.std_logic_1164.all; 
USE ieee.numeric_std.all;


ENTITY ptp_module IS 
	PORT
	(
		sys_clk :  IN  STD_LOGIC;
		rst_n :  IN  STD_LOGIC;
		mac_link_up_i :  IN  STD_LOGIC;
		mac_tx_clock :  IN  STD_LOGIC;
		mac_tx_busy :  IN  STD_LOGIC;
		mac_tx_byte_sent :  IN  STD_LOGIC;
		parse_ptp_packet_tog :  IN  STD_LOGIC;
		ptp_is_leader_o :  OUT  STD_LOGIC;
		ptp_is_follower_o :  OUT  STD_LOGIC;
		sof_recv_tog_i :  IN  STD_LOGIC; -- signal at sof rx delimiter
		mac_tx_allow_i :  IN  STD_LOGIC;
		sof_sent_tog_i :  IN  STD_LOGIC; -- signal at sof tx delimiter
		ip_address :  IN  STD_LOGIC_VECTOR(31 DOWNTO 0);
		mac_address :  IN  STD_LOGIC_VECTOR(47 DOWNTO 0);
		mac_ram_data :  IN  STD_LOGIC_VECTOR(7 DOWNTO 0);
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
		ptp_sync_lost :  OUT  STD_LOGIC;
		wallclock_locked :  OUT  STD_LOGIC;
		wallclock_configured :  OUT  STD_LOGIC;
		wc_mclk :  OUT  STD_LOGIC;
		second_pulse_sys :  OUT  STD_LOGIC;
		wallclock_phasejump :  OUT  STD_LOGIC;
		media_clock :  OUT  STD_LOGIC_VECTOR(31 DOWNTO 0);
		ptp_mean_path_delay :  OUT  STD_LOGIC_VECTOR(31 DOWNTO 0);
		ptp_offset_from_master :  OUT  STD_LOGIC_VECTOR(31 DOWNTO 0);
		ptp_ram_addr :  OUT  STD_LOGIC_VECTOR(10 DOWNTO 0);
		tx_data_ptpfu :  OUT  STD_LOGIC_VECTOR(7 DOWNTO 0);
		mac_speed_i : IN STD_LOGIC_VECTOR(1 downto 0)
	);
END ptp_module;

ARCHITECTURE bdf_type OF ptp_module IS 



SIGNAL	freq_correction :  SIGNED(31 DOWNTO 0);
SIGNAL	log_msg_interval :  SIGNED(7 DOWNTO 0);
SIGNAL	log_msg_interval_valid :  STD_LOGIC;
SIGNAL	phase_jump :  SIGNED(31 DOWNTO 0);
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
SIGNAL	rx_timestamp_ns :  UNSIGNED(31 DOWNTO 0);
SIGNAL	rx_timestamp_s :  UNSIGNED(47 DOWNTO 0);
SIGNAL	rx_ts_ns :  STD_LOGIC_VECTOR(31 DOWNTO 0);
SIGNAL	rx_ts_s :  STD_LOGIC_VECTOR(47 DOWNTO 0);
SIGNAL	second_pulse_sys_ALTERA_SYNTHESIZED :  STD_LOGIC;
SIGNAL	tx_en_ptpfu_ALTERA_SYNTHESIZED :  STD_LOGIC;
SIGNAL	tx_frame_start :  STD_LOGIC;
SIGNAL	tx_msg_type :  STD_LOGIC_VECTOR(3 DOWNTO 0);
SIGNAL	tx_ptp_log_interval :  STD_LOGIC_VECTOR(7 DOWNTO 0);
SIGNAL	tx_req_port_identity :  STD_LOGIC_VECTOR(79 DOWNTO 0);
SIGNAL	tx_seq_id :  UNSIGNED(15 DOWNTO 0);
SIGNAL	tx_t3_valid :  STD_LOGIC;
SIGNAL	tx_timestamp_ns :  UNSIGNED(31 DOWNTO 0);
SIGNAL	tx_timestamp_s :  UNSIGNED(47 DOWNTO 0);
SIGNAL	timestamp_ns :  UNSIGNED(31 DOWNTO 0);
SIGNAL	timestamp_s :  UNSIGNED(47 DOWNTO 0);
SIGNAL	wallclock_nanoseconds :  UNSIGNED(31 DOWNTO 0);
SIGNAL	ptp_ram_addr_u :  UNSIGNED(10 DOWNTO 0);
SIGNAL	media_clock_u :  UNSIGNED(31 DOWNTO 0);
SIGNAL	wallclock_phasejump_ALTERA_SYNTHESIZED :  STD_LOGIC;
SIGNAL	servo_request_clock_reconfigure :  STD_LOGIC;
SIGNAL	wallclock_seconds :  UNSIGNED(47 DOWNTO 0);
SIGNAL	wallclock_set :  STD_LOGIC;
SIGNAL	wallclock_set_ns :  STD_LOGIC_VECTOR(31 DOWNTO 0);
SIGNAL	wallclock_set_s :  STD_LOGIC_VECTOR(47 DOWNTO 0);

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


BEGIN

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
		 ms_pulse_i => ms_pulse_sys,
		 is_leader_i => eff_is_leader,
		 is_follower_i => eff_is_follower,
		 tx_en_i => tx_en_ptpfu_ALTERA_SYNTHESIZED,
		 send_delay_req_i => rx_send_delay_req,
		 ptp_announce_log_message_interval_i => ptp_announce_interval,
		 ptp_log_message_interval_i => ptp_log_message_interval,
		 request_port_identity_i => rx_follower_identity,
		 rx_timestamp_nanoseconds_i => unsigned(rx_ts_ns),
		 rx_timestamp_seconds_i => unsigned(rx_ts_s),
		 sequence_id_i => unsigned(rx_sequence_id),
		 wallclock_nanoseconds_i => wallclock_nanoseconds,
		 wallclock_seconds_i => wallclock_seconds,
		 frame_start_o => tx_frame_start,
		 t3_valid_o => tx_t3_valid,
		 ptp_log_interval_o => tx_ptp_log_interval,
		 request_port_identity_o => tx_req_port_identity,
		 sequence_id_o => tx_seq_id,
		 timestamp_nanoseconds_o => timestamp_ns,
		 timestamp_seconds_o => timestamp_s,
		 tx_timestamp_nanoseconds_i => tx_timestamp_ns,
		 tx_timestamp_seconds_i => tx_timestamp_s,
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


b2v_ptpparser :  entity work.ptpv2_parser
GENERIC MAP(MIN_FILTER_DEPTH => 6
			)
PORT MAP(clk => sys_clk,
		 parse_ptp_packet_tog => parse_ptp_packet_tog,
		 is_leader => eff_is_leader,
		 reset_n => powerGood,
		 t3_valid_i => tx_t3_valid,
		 ptp_is_follower_i => eff_is_follower,
		 ptp_locked_i => ptp_locked_ALTERA_SYNTHESIZED,
		 ptp_current_leader_id_i => eff_current_leader_id,
		 clock_reconfigure_req_i => servo_request_clock_reconfigure,
		 ram_data => mac_ram_data,
		 rx_timestamp_nanoseconds_i => std_logic_vector(rx_timestamp_ns),
		 rx_timestamp_seconds_i => std_logic_vector(rx_timestamp_s),
		 src_mac_address => mac_address,
		 tx_timestamp_nanoseconds_i => std_logic_vector(tx_timestamp_ns),
		 tx_timestamp_seconds_i => std_logic_vector(tx_timestamp_s),
		 send_delay_resp_o => rx_send_delay_resp,
		 send_delay_req_o => rx_send_delay_req,
		 ptp_calc_valid_o => ptp_calc_valid,
		 log_msg_interval_valid_o => log_msg_interval_valid,
		 clock_set_o => wallclock_set,
		 clock_configured_o => wallclock_configured,
		 clock_configure_timestamp_nanoseconds_o => wallclock_set_ns,
		 clock_configure_timestamp_seconds_o => wallclock_set_s,
		 log_msg_interval_o => log_msg_interval,
		 mean_path_delay_ns_o => ptp_mean_path_delay_ALTERA_SYNTHESIZED,
		 offset_from_master_ns_o => ptp_offset_from_master_ALTERA_SYNTHESIZED,
		 ram_read_address => ptp_ram_addr_u,
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
		 announce_log_msg_interval_o => ann_log_msg_interval);


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


b2v_rx_tsu :  entity work.ethernet_timestamp
generic map (
	PATH => "RX",
	MAC_TO_PHY_NS_100M => 163,
	MAC_TO_PHY_NS_1G => 20,
	PHY_TX_TO_WIRE_NS => 0,
   PHY_WIRE_TO_RX_NS => 0
)
PORT MAP(clk => sys_clk,
		 reset_n => powerGood,
		 mac_speed_i => mac_speed_i,
		 sof_tog_i => sof_recv_tog_i,
		 wallclock_nanoseconds_i => wallclock_nanoseconds,
		 wallclock_seconds_i => wallclock_seconds,
		 timestamp_nanoseconds_o => rx_timestamp_ns,
		 timestamp_seconds_o => rx_timestamp_s);
b2v_tx_tsu :  entity work.ethernet_timestamp
generic map (
	PATH => "TX",
	MAC_TO_PHY_NS_100M => 200,
	MAC_TO_PHY_NS_1G => 20,
	PHY_TX_TO_WIRE_NS => 0,
   PHY_WIRE_TO_RX_NS => 0
)
PORT MAP(clk => sys_clk,
		 reset_n => powerGood,
		 mac_speed_i => mac_speed_i,
		 sof_tog_i => sof_sent_tog_i,
		 wallclock_nanoseconds_i => wallclock_nanoseconds,
		 wallclock_seconds_i => wallclock_seconds,
		 timestamp_nanoseconds_o => tx_timestamp_ns,
		 timestamp_seconds_o => tx_timestamp_s);


b2v_servo :  entity work.ptpv2_servo
PORT MAP(clk => sys_clk,
		 reset_n => powerGood,
		 calc_valid_i => ptp_calc_valid,
		 log_msg_interval_valid_i => log_msg_interval_valid,
		 log_msg_interval_i => log_msg_interval,
		 mean_path_delay_i => ptp_mean_path_delay_ALTERA_SYNTHESIZED,
		 offset_from_master_i => ptp_offset_from_master_ALTERA_SYNTHESIZED,
		 phase_jump_valid_o => wallclock_phasejump_ALTERA_SYNTHESIZED,
		 locked_o => ptp_locked_ALTERA_SYNTHESIZED,
		 sync_timeout_o => ptp_sync_lost,
		 freq_correction_o => freq_correction,
		 phase_jump_o => phase_jump,
		 request_clock_reconfigure_o => servo_request_clock_reconfigure);


b2v_wallclock :  entity work.wallclock
GENERIC MAP(audio_fs => 48000,
			increment_interval => 8,
			sys_clk_hz => 125000000
			)
PORT MAP(clk => sys_clk,
		 reset_n => powerGood,
		 wallclock_set_i => wallclock_set,
		 phase_jump_valid_i => wallclock_phasejump_ALTERA_SYNTHESIZED,
		 freq_correction_ppb_i => freq_correction,
		 phase_jump_ns_i => phase_jump,
		 wallclock_nanoseconds_i => unsigned(wallclock_set_ns),
		 wallclock_seconds_i => unsigned(wallclock_set_s),
		 second_pulse_o => second_pulse_sys_ALTERA_SYNTHESIZED,
		 ms_pulse_o => ms_pulse_sys,
		 audio_mclk_o => wc_mclk,
		 media_clock_o => media_clock_u,
		 wallclock_nanoseconds_o => wallclock_nanoseconds,
		 wallclock_seconds_o => wallclock_seconds);

tx_en_ptpfu <= tx_en_ptpfu_ALTERA_SYNTHESIZED;
powerGood <= rst_n;

ptp_allow <= mac_tx_allow_i;
ptp_locked <= ptp_locked_ALTERA_SYNTHESIZED;
second_pulse_sys <= second_pulse_sys_ALTERA_SYNTHESIZED;
wallclock_phasejump <= wallclock_phasejump_ALTERA_SYNTHESIZED;
media_clock <= std_logic_vector(media_clock_u);
ptp_mean_path_delay <= std_logic_vector(ptp_mean_path_delay_ALTERA_SYNTHESIZED);
ptp_offset_from_master <= std_logic_vector(ptp_offset_from_master_ALTERA_SYNTHESIZED);
ptp_ram_addr <= std_logic_vector(ptp_ram_addr_u);

END bdf_type;