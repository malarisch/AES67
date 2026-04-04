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
-- CREATED		"Sat Apr  4 16:19:53 2026"

LIBRARY ieee;
USE ieee.std_logic_1164.all; 

LIBRARY work;

ENTITY ptp_module IS 
	PORT
	(
		sys_clk :  IN  STD_LOGIC;
		rst_n :  IN  STD_LOGIC;
		mac_tx_clock :  IN  STD_LOGIC;
		mac_tx_busy :  IN  STD_LOGIC;
		mac_tx_byte_sent :  IN  STD_LOGIC;
		parse_ptp_packet :  IN  STD_LOGIC;
		ptp_is_leader :  IN  STD_LOGIC;
		ptp_is_follower :  IN  STD_LOGIC;
		ethernet_parser_sync :  IN  STD_LOGIC;
		eth_frame_i :  IN  STD_LOGIC;
		mac_tx_allow_i :  IN  STD_LOGIC;
		sof_sent_i :  IN  STD_LOGIC;
		ip_address :  IN  STD_LOGIC_VECTOR(31 DOWNTO 0);
		mac_address :  IN  STD_LOGIC_VECTOR(47 DOWNTO 0);
		mac_ram_data :  IN  STD_LOGIC_VECTOR(7 DOWNTO 0);
		ptp_announce_interval :  IN  STD_LOGIC_VECTOR(7 DOWNTO 0);
		ptp_clock_accuracy :  IN  STD_LOGIC_VECTOR(7 DOWNTO 0);
		ptp_clock_class :  IN  STD_LOGIC_VECTOR(7 DOWNTO 0);
		ptp_clock_priorityone :  IN  STD_LOGIC_VECTOR(7 DOWNTO 0);
		ptp_clock_prioritytwo :  IN  STD_LOGIC_VECTOR(7 DOWNTO 0);
		ptp_current_leader_id :  IN  STD_LOGIC_VECTOR(63 DOWNTO 0);
		ptp_log_message_interval :  IN  STD_LOGIC_VECTOR(7 DOWNTO 0);
		ptp_time_source :  IN  STD_LOGIC_VECTOR(7 DOWNTO 0);
		tx_en_ptpfu :  OUT  STD_LOGIC;
		ptp_allow_req :  OUT  STD_LOGIC;
		ptp_locked :  OUT  STD_LOGIC;
		ptp_sync_lost :  OUT  STD_LOGIC;
		wallclock_locked :  OUT  STD_LOGIC;
		wallclock_configured :  OUT  STD_LOGIC;
		wc_64fs :  OUT  STD_LOGIC;
		second_pulse_sys :  OUT  STD_LOGIC;
		wallclock_phasejump :  OUT  STD_LOGIC;
		pin_name1 :  OUT  STD_LOGIC;
		media_clock :  OUT  STD_LOGIC_VECTOR(31 DOWNTO 0);
		ptp_mean_path_delay :  OUT  STD_LOGIC_VECTOR(31 DOWNTO 0);
		ptp_offset_from_master :  OUT  STD_LOGIC_VECTOR(31 DOWNTO 0);
		ptp_ram_addr :  OUT  STD_LOGIC_VECTOR(10 DOWNTO 0);
		tx_data_ptpfu :  OUT  STD_LOGIC_VECTOR(7 DOWNTO 0)
	);
END ptp_module;

ARCHITECTURE bdf_type OF ptp_module IS 

COMPONENT ptpv2_sender
	PORT(sys_clk : IN STD_LOGIC;
		 frame_start : IN STD_LOGIC;
		 tx_clk : IN STD_LOGIC;
		 tx_busy : IN STD_LOGIC;
		 tx_byte_sent : IN STD_LOGIC;
		 tx_allow_i : IN STD_LOGIC;
		 message_type_i : IN STD_LOGIC_VECTOR(3 DOWNTO 0);
		 ptp_clockaccuracy : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
		 ptp_clockclass : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
		 ptp_log_interval_i : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
		 ptp_prioone : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
		 ptp_priotwo : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
		 ptp_time_source_i : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
		 request_port_identity : IN STD_LOGIC_VECTOR(79 DOWNTO 0);
		 sequence_id : IN STD_LOGIC_VECTOR(15 DOWNTO 0);
		 src_ip_address : IN STD_LOGIC_VECTOR(31 DOWNTO 0);
		 src_mac_address : IN STD_LOGIC_VECTOR(47 DOWNTO 0);
		 timestamp_nanoseconds_i : IN STD_LOGIC_VECTOR(31 DOWNTO 0);
		 timestamp_seconds_i : IN STD_LOGIC_VECTOR(47 DOWNTO 0);
		 tx_enable : OUT STD_LOGIC;
		 tx_ready_o : OUT STD_LOGIC;
		 tx_allow_req_o : OUT STD_LOGIC;
		 tx_data : OUT STD_LOGIC_VECTOR(7 DOWNTO 0)
	);
END COMPONENT;

COMPONENT ptpv2_parser
GENERIC (MIN_FILTER_DEPTH : INTEGER
			);
	PORT(clk : IN STD_LOGIC;
		 parse_ptp_packet : IN STD_LOGIC;
		 is_leader : IN STD_LOGIC;
		 reset_n : IN STD_LOGIC;
		 t3_valid_i : IN STD_LOGIC;
		 ptp_is_follower_i : IN STD_LOGIC;
		 ptp_locked_i : IN STD_LOGIC;
		 ptp_current_leader_id_i : IN STD_LOGIC_VECTOR(63 DOWNTO 0);
		 ram_data : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
		 rx_timestamp_nanoseconds_i : IN STD_LOGIC_VECTOR(31 DOWNTO 0);
		 rx_timestamp_seconds_i : IN STD_LOGIC_VECTOR(47 DOWNTO 0);
		 src_mac_address : IN STD_LOGIC_VECTOR(47 DOWNTO 0);
		 tx_timestamp_nanoseconds_i : IN STD_LOGIC_VECTOR(31 DOWNTO 0);
		 tx_timestamp_seconds_i : IN STD_LOGIC_VECTOR(47 DOWNTO 0);
		 send_delay_resp_o : OUT STD_LOGIC;
		 send_delay_req_o : OUT STD_LOGIC;
		 ptp_calc_valid_o : OUT STD_LOGIC;
		 log_msg_interval_valid_o : OUT STD_LOGIC;
		 clock_set_o : OUT STD_LOGIC;
		 clock_configured_o : OUT STD_LOGIC;
		 clock_configure_timestamp_nanoseconds_o : OUT STD_LOGIC_VECTOR(31 DOWNTO 0);
		 clock_configure_timestamp_seconds_o : OUT STD_LOGIC_VECTOR(47 DOWNTO 0);
		 log_msg_interval_o : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
		 mean_path_delay_ns_o : OUT STD_LOGIC_VECTOR(31 DOWNTO 0);
		 offset_from_master_ns_o : OUT STD_LOGIC_VECTOR(31 DOWNTO 0);
		 ram_read_address : OUT STD_LOGIC_VECTOR(10 DOWNTO 0);
		 rx_follower_identity_o : OUT STD_LOGIC_VECTOR(79 DOWNTO 0);
		 rx_timestamp_nanoseconds_o : OUT STD_LOGIC_VECTOR(31 DOWNTO 0);
		 rx_timestamp_seconds_o : OUT STD_LOGIC_VECTOR(47 DOWNTO 0);
		 sequence_id_o : OUT STD_LOGIC_VECTOR(15 DOWNTO 0)
	);
END COMPONENT;

COMPONENT ptpv2_servo
	PORT(clk : IN STD_LOGIC;
		 reset_n : IN STD_LOGIC;
		 calc_valid_i : IN STD_LOGIC;
		 log_msg_interval_valid_i : IN STD_LOGIC;
		 log_msg_interval_i : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
		 mean_path_delay_i : IN STD_LOGIC_VECTOR(31 DOWNTO 0);
		 offset_from_master_i : IN STD_LOGIC_VECTOR(31 DOWNTO 0);
		 phase_jump_valid_o : OUT STD_LOGIC;
		 locked_o : OUT STD_LOGIC;
		 sync_timeout_o : OUT STD_LOGIC;
		 freq_correction_o : OUT STD_LOGIC_VECTOR(31 DOWNTO 0);
		 phase_jump_o : OUT STD_LOGIC_VECTOR(31 DOWNTO 0)
	);
END COMPONENT;

COMPONENT ptpv2_controller
	PORT(clk : IN STD_LOGIC;
		 reset_n : IN STD_LOGIC;
		 send_delay_resp_in : IN STD_LOGIC;
		 second_pulse_i : IN STD_LOGIC;
		 is_leader_i : IN STD_LOGIC;
		 is_follower_i : IN STD_LOGIC;
		 tx_en_i : IN STD_LOGIC;
		 send_delay_req_i : IN STD_LOGIC;
		 config_valid_i : IN STD_LOGIC;
		 sof_sent_i : IN STD_LOGIC;
		 ptp_announce_log_message_interval_i : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
		 ptp_log_message_interval_i : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
		 request_port_identity_i : IN STD_LOGIC_VECTOR(79 DOWNTO 0);
		 rx_timestamp_nanoseconds_i : IN STD_LOGIC_VECTOR(31 DOWNTO 0);
		 rx_timestamp_seconds_i : IN STD_LOGIC_VECTOR(47 DOWNTO 0);
		 sequence_id_i : IN STD_LOGIC_VECTOR(15 DOWNTO 0);
		 wallclock_nanoseconds_i : IN STD_LOGIC_VECTOR(31 DOWNTO 0);
		 wallclock_seconds_i : IN STD_LOGIC_VECTOR(47 DOWNTO 0);
		 frame_start_o : OUT STD_LOGIC;
		 t3_valid_o : OUT STD_LOGIC;
		 ptp_log_interval_o : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
		 request_port_identity_o : OUT STD_LOGIC_VECTOR(79 DOWNTO 0);
		 sequence_id_o : OUT STD_LOGIC_VECTOR(15 DOWNTO 0);
		 timestamp_nanoseconds_o : OUT STD_LOGIC_VECTOR(31 DOWNTO 0);
		 timestamp_seconds_o : OUT STD_LOGIC_VECTOR(47 DOWNTO 0);
		 tx_message_type_o : OUT STD_LOGIC_VECTOR(3 DOWNTO 0)
	);
END COMPONENT;

COMPONENT wallclock
GENERIC (audio_fs : INTEGER;
			increment_interval : INTEGER;
			sys_clk_hz : INTEGER
			);
	PORT(clk : IN STD_LOGIC;
		 reset_n : IN STD_LOGIC;
		 wallclock_set_i : IN STD_LOGIC;
		 phase_jump_valid_i : IN STD_LOGIC;
		 freq_correction_ppb_i : IN STD_LOGIC_VECTOR(31 DOWNTO 0);
		 phase_jump_ns_i : IN STD_LOGIC_VECTOR(31 DOWNTO 0);
		 wallclock_nanoseconds_i : IN STD_LOGIC_VECTOR(31 DOWNTO 0);
		 wallclock_seconds_i : IN STD_LOGIC_VECTOR(47 DOWNTO 0);
		 second_pulse_o : OUT STD_LOGIC;
		 audio_bclk_o : OUT STD_LOGIC;
		 audio_lrck_o : OUT STD_LOGIC;
		 sample_pulse_o : OUT STD_LOGIC;
		 media_clock_o : OUT STD_LOGIC_VECTOR(31 DOWNTO 0);
		 wallclock_nanoseconds_o : OUT STD_LOGIC_VECTOR(31 DOWNTO 0);
		 wallclock_seconds_o : OUT STD_LOGIC_VECTOR(47 DOWNTO 0)
	);
END COMPONENT;

COMPONENT ethernet_timestamp
	PORT(clk : IN STD_LOGIC;
		 reset_n : IN STD_LOGIC;
		 mac_rx_frame_i : IN STD_LOGIC;
		 ethernet_parser_sync_in_i : IN STD_LOGIC;
		 is_ptp_packet_i : IN STD_LOGIC;
		 wallclock_nanoseconds_i : IN STD_LOGIC_VECTOR(31 DOWNTO 0);
		 wallclock_seconds_i : IN STD_LOGIC_VECTOR(47 DOWNTO 0);
		 timestamp_nanoseconds_o : OUT STD_LOGIC_VECTOR(31 DOWNTO 0);
		 timestamp_seconds_o : OUT STD_LOGIC_VECTOR(47 DOWNTO 0)
	);
END COMPONENT;

SIGNAL	eth_frame_o :  STD_LOGIC;
SIGNAL	log_msg_interval :  STD_LOGIC_VECTOR(7 DOWNTO 0);
SIGNAL	log_msg_interval_valid :  STD_LOGIC;
SIGNAL	powerGood :  STD_LOGIC;
SIGNAL	ptp_allow :  STD_LOGIC;
SIGNAL	ptp_calc_valid :  STD_LOGIC;
SIGNAL	ptp_locked_ALTERA_SYNTHESIZED :  STD_LOGIC;
SIGNAL	ptp_mean_path_delay_ALTERA_SYNTHESIZED :  STD_LOGIC_VECTOR(31 DOWNTO 0);
SIGNAL	ptp_offset_from_master_ALTERA_SYNTHESIZED :  STD_LOGIC_VECTOR(31 DOWNTO 0);
SIGNAL	ptp_txready :  STD_LOGIC;
SIGNAL	rx_timestamp_ns :  STD_LOGIC_VECTOR(31 DOWNTO 0);
SIGNAL	rx_timestamp_s :  STD_LOGIC_VECTOR(47 DOWNTO 0);
SIGNAL	second_pulse_sys_ALTERA_SYNTHESIZED :  STD_LOGIC;
SIGNAL	sof_sent :  STD_LOGIC;
SIGNAL	tx_en_ptpfu_ALTERA_SYNTHESIZED :  STD_LOGIC;
SIGNAL	wallclock_phasejump_ALTERA_SYNTHESIZED :  STD_LOGIC;
SIGNAL	wallclock_set :  STD_LOGIC;
SIGNAL	wallclock_set_ns :  STD_LOGIC_VECTOR(31 DOWNTO 0);
SIGNAL	wallclock_set_s :  STD_LOGIC_VECTOR(47 DOWNTO 0);
SIGNAL	wc_fs :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_0 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_1 :  STD_LOGIC_VECTOR(3 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_2 :  STD_LOGIC_VECTOR(7 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_3 :  STD_LOGIC_VECTOR(79 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_4 :  STD_LOGIC_VECTOR(15 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_22 :  STD_LOGIC_VECTOR(31 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_23 :  STD_LOGIC_VECTOR(47 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_7 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_10 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_11 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_12 :  STD_LOGIC_VECTOR(79 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_13 :  STD_LOGIC_VECTOR(31 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_14 :  STD_LOGIC_VECTOR(47 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_15 :  STD_LOGIC_VECTOR(15 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_24 :  STD_LOGIC_VECTOR(31 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_25 :  STD_LOGIC_VECTOR(47 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_18 :  STD_LOGIC_VECTOR(31 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_19 :  STD_LOGIC_VECTOR(31 DOWNTO 0);


BEGIN 



b2v_inst : ptpv2_sender
PORT MAP(sys_clk => sys_clk,
		 frame_start => SYNTHESIZED_WIRE_0,
		 tx_clk => mac_tx_clock,
		 tx_busy => mac_tx_busy,
		 tx_byte_sent => mac_tx_byte_sent,
		 tx_allow_i => ptp_allow,
		 message_type_i => SYNTHESIZED_WIRE_1,
		 ptp_clockaccuracy => ptp_clock_accuracy,
		 ptp_clockclass => ptp_clock_class,
		 ptp_log_interval_i => SYNTHESIZED_WIRE_2,
		 ptp_prioone => ptp_clock_priorityone,
		 ptp_priotwo => ptp_clock_prioritytwo,
		 ptp_time_source_i => ptp_time_source,
		 request_port_identity => SYNTHESIZED_WIRE_3,
		 sequence_id => SYNTHESIZED_WIRE_4,
		 src_ip_address => ip_address,
		 src_mac_address => mac_address,
		 timestamp_nanoseconds_i => SYNTHESIZED_WIRE_22,
		 timestamp_seconds_i => SYNTHESIZED_WIRE_23,
		 tx_enable => tx_en_ptpfu_ALTERA_SYNTHESIZED,
		 tx_allow_req_o => ptp_allow_req,
		 tx_data => tx_data_ptpfu);


b2v_inst14 : ptpv2_parser
GENERIC MAP(MIN_FILTER_DEPTH => 3
			)
PORT MAP(clk => sys_clk,
		 parse_ptp_packet => parse_ptp_packet,
		 is_leader => ptp_is_leader,
		 reset_n => powerGood,
		 t3_valid_i => SYNTHESIZED_WIRE_7,
		 ptp_is_follower_i => ptp_is_follower,
		 ptp_locked_i => ptp_locked_ALTERA_SYNTHESIZED,
		 ptp_current_leader_id_i => ptp_current_leader_id,
		 ram_data => mac_ram_data,
		 rx_timestamp_nanoseconds_i => rx_timestamp_ns,
		 rx_timestamp_seconds_i => rx_timestamp_s,
		 src_mac_address => mac_address,
		 tx_timestamp_nanoseconds_i => SYNTHESIZED_WIRE_22,
		 tx_timestamp_seconds_i => SYNTHESIZED_WIRE_23,
		 send_delay_resp_o => SYNTHESIZED_WIRE_10,
		 send_delay_req_o => SYNTHESIZED_WIRE_11,
		 ptp_calc_valid_o => ptp_calc_valid,
		 log_msg_interval_valid_o => log_msg_interval_valid,
		 clock_set_o => wallclock_set,
		 clock_configured_o => wallclock_configured,
		 clock_configure_timestamp_nanoseconds_o => wallclock_set_ns,
		 clock_configure_timestamp_seconds_o => wallclock_set_s,
		 log_msg_interval_o => log_msg_interval,
		 mean_path_delay_ns_o => ptp_mean_path_delay_ALTERA_SYNTHESIZED,
		 offset_from_master_ns_o => ptp_offset_from_master_ALTERA_SYNTHESIZED,
		 ram_read_address => ptp_ram_addr,
		 rx_follower_identity_o => SYNTHESIZED_WIRE_12,
		 rx_timestamp_nanoseconds_o => SYNTHESIZED_WIRE_13,
		 rx_timestamp_seconds_o => SYNTHESIZED_WIRE_14,
		 sequence_id_o => SYNTHESIZED_WIRE_15);


b2v_inst25 : ptpv2_servo
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
		 freq_correction_o => SYNTHESIZED_WIRE_18,
		 phase_jump_o => SYNTHESIZED_WIRE_19);


wallclock_locked <= ptp_is_leader OR ptp_locked_ALTERA_SYNTHESIZED;


b2v_inst34 : ptpv2_controller
PORT MAP(clk => sys_clk,
		 reset_n => powerGood,
		 send_delay_resp_in => SYNTHESIZED_WIRE_10,
		 second_pulse_i => second_pulse_sys_ALTERA_SYNTHESIZED,
		 is_leader_i => ptp_is_leader,
		 is_follower_i => ptp_is_follower,
		 tx_en_i => tx_en_ptpfu_ALTERA_SYNTHESIZED,
		 send_delay_req_i => SYNTHESIZED_WIRE_11,
		 sof_sent_i => sof_sent,
		 ptp_announce_log_message_interval_i => ptp_announce_interval,
		 ptp_log_message_interval_i => ptp_log_message_interval,
		 request_port_identity_i => SYNTHESIZED_WIRE_12,
		 rx_timestamp_nanoseconds_i => SYNTHESIZED_WIRE_13,
		 rx_timestamp_seconds_i => SYNTHESIZED_WIRE_14,
		 sequence_id_i => SYNTHESIZED_WIRE_15,
		 wallclock_nanoseconds_i => SYNTHESIZED_WIRE_24,
		 wallclock_seconds_i => SYNTHESIZED_WIRE_25,
		 frame_start_o => SYNTHESIZED_WIRE_0,
		 t3_valid_o => SYNTHESIZED_WIRE_7,
		 ptp_log_interval_o => SYNTHESIZED_WIRE_2,
		 request_port_identity_o => SYNTHESIZED_WIRE_3,
		 sequence_id_o => SYNTHESIZED_WIRE_4,
		 timestamp_nanoseconds_o => SYNTHESIZED_WIRE_22,
		 timestamp_seconds_o => SYNTHESIZED_WIRE_23,
		 tx_message_type_o => SYNTHESIZED_WIRE_1);


b2v_inst9 : wallclock
GENERIC MAP(audio_fs => 48000,
			increment_interval => 8,
			sys_clk_hz => 125000000
			)
PORT MAP(clk => sys_clk,
		 reset_n => powerGood,
		 wallclock_set_i => wallclock_set,
		 phase_jump_valid_i => wallclock_phasejump_ALTERA_SYNTHESIZED,
		 freq_correction_ppb_i => SYNTHESIZED_WIRE_18,
		 phase_jump_ns_i => SYNTHESIZED_WIRE_19,
		 wallclock_nanoseconds_i => wallclock_set_ns,
		 wallclock_seconds_i => wallclock_set_s,
		 second_pulse_o => second_pulse_sys_ALTERA_SYNTHESIZED,
		 audio_bclk_o => wc_64fs,
		 audio_lrck_o => wc_fs,
		 media_clock_o => media_clock,
		 wallclock_nanoseconds_o => SYNTHESIZED_WIRE_24,
		 wallclock_seconds_o => SYNTHESIZED_WIRE_25);


b2v_rx_tsu : ethernet_timestamp
PORT MAP(clk => sys_clk,
		 reset_n => powerGood,
		 mac_rx_frame_i => eth_frame_o,
		 ethernet_parser_sync_in_i => ethernet_parser_sync,
		 is_ptp_packet_i => parse_ptp_packet,
		 wallclock_nanoseconds_i => SYNTHESIZED_WIRE_24,
		 wallclock_seconds_i => SYNTHESIZED_WIRE_25,
		 timestamp_nanoseconds_o => rx_timestamp_ns,
		 timestamp_seconds_o => rx_timestamp_s);

tx_en_ptpfu <= tx_en_ptpfu_ALTERA_SYNTHESIZED;
powerGood <= rst_n;
eth_frame_o <= eth_frame_i;
sof_sent <= sof_sent_i;
ptp_allow <= mac_tx_allow_i;
ptp_locked <= ptp_locked_ALTERA_SYNTHESIZED;
second_pulse_sys <= second_pulse_sys_ALTERA_SYNTHESIZED;
wallclock_phasejump <= wallclock_phasejump_ALTERA_SYNTHESIZED;
pin_name1 <= wc_fs;
ptp_mean_path_delay <= ptp_mean_path_delay_ALTERA_SYNTHESIZED;
ptp_offset_from_master <= ptp_offset_from_master_ALTERA_SYNTHESIZED;

END bdf_type;