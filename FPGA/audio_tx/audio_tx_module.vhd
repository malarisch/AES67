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
-- CREATED		"Sat Apr  4 16:20:34 2026"

LIBRARY ieee;
USE ieee.std_logic_1164.all; 

LIBRARY work;

ENTITY audio_tx_module IS 
	PORT
	(
		sys_clk :  IN  STD_LOGIC;
		fmc_clk :  IN  STD_LOGIC;
		rst_n :  IN  STD_LOGIC;
		fs_clk_i :  IN  STD_LOGIC;
		cfg_wr_en_i :  IN  STD_LOGIC;
		mac_tx_clock :  IN  STD_LOGIC;
		mac_tx_busy :  IN  STD_LOGIC;
		mac_tx_byte_sent :  IN  STD_LOGIC;
		mac_audio_allow_i :  IN  STD_LOGIC;
		metering_clear_i :  IN  STD_LOGIC;
		cfg_wr_addr_i :  IN  STD_LOGIC_VECTOR(7 DOWNTO 0);
		cfg_wr_data_i :  IN  STD_LOGIC_VECTOR(7 DOWNTO 0);
		ch0i :  IN  STD_LOGIC_VECTOR(23 DOWNTO 0);
		ch10i :  IN  STD_LOGIC_VECTOR(23 DOWNTO 0);
		ch11i :  IN  STD_LOGIC_VECTOR(23 DOWNTO 0);
		ch12i :  IN  STD_LOGIC_VECTOR(23 DOWNTO 0);
		ch13i :  IN  STD_LOGIC_VECTOR(23 DOWNTO 0);
		ch14i :  IN  STD_LOGIC_VECTOR(23 DOWNTO 0);
		ch15i :  IN  STD_LOGIC_VECTOR(23 DOWNTO 0);
		ch1i :  IN  STD_LOGIC_VECTOR(23 DOWNTO 0);
		ch2i :  IN  STD_LOGIC_VECTOR(23 DOWNTO 0);
		ch3i :  IN  STD_LOGIC_VECTOR(23 DOWNTO 0);
		ch4i :  IN  STD_LOGIC_VECTOR(23 DOWNTO 0);
		ch5i :  IN  STD_LOGIC_VECTOR(23 DOWNTO 0);
		ch6i :  IN  STD_LOGIC_VECTOR(23 DOWNTO 0);
		ch7i :  IN  STD_LOGIC_VECTOR(23 DOWNTO 0);
		ch8i :  IN  STD_LOGIC_VECTOR(23 DOWNTO 0);
		ch9i :  IN  STD_LOGIC_VECTOR(23 DOWNTO 0);
		ip_addr_i :  IN  STD_LOGIC_VECTOR(31 DOWNTO 0);
		mac_addr_i :  IN  STD_LOGIC_VECTOR(47 DOWNTO 0);
		media_clock_i :  IN  STD_LOGIC_VECTOR(31 DOWNTO 0);
		tx_en_o :  OUT  STD_LOGIC;
		tx_req_o :  OUT  STD_LOGIC;
		metering_clip_o :  OUT  STD_LOGIC_VECTOR(15 DOWNTO 0);
		metering_signal_o :  OUT  STD_LOGIC_VECTOR(15 DOWNTO 0);
		tx_data_o :  OUT  STD_LOGIC_VECTOR(7 DOWNTO 0)
	);
END audio_tx_module;

ARCHITECTURE bdf_type OF audio_tx_module IS 

COMPONENT tx_router
GENERIC (bytes_per_sample : INTEGER;
			global_channel_count : INTEGER;
			max_streams : INTEGER;
			samples_per_channel_depth : INTEGER
			);
	PORT(sys_clk_i : IN STD_LOGIC;
		 reset_n : IN STD_LOGIC;
		 config_wr_en_i : IN STD_LOGIC;
		 sample_ready_i : IN STD_LOGIC;
		 tx_en_i : IN STD_LOGIC;
		 config_wr_clk_i : IN STD_LOGIC;
		 config_wr_addr_i : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
		 config_wr_data_i : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
		 packet_time_i : IN STD_LOGIC_VECTOR(31 DOWNTO 0);
		 sample_buffer_wr_ptr_i : IN STD_LOGIC_VECTOR(15 DOWNTO 0);
		 audio_packet_tx_start_o : OUT STD_LOGIC;
		 ch_ids_o : OUT STD_LOGIC_VECTOR(63 DOWNTO 0);
		 channel_count_o : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
		 ip_addr_o : OUT STD_LOGIC_VECTOR(31 DOWNTO 0);
		 packet_time_o : OUT STD_LOGIC_VECTOR(31 DOWNTO 0);
		 sample_buffer_tx_start_addr_o : OUT STD_LOGIC_VECTOR(15 DOWNTO 0);
		 samples_per_packet_per_channel_o : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
		 ssrc_o : OUT STD_LOGIC_VECTOR(31 DOWNTO 0)
	);
END COMPONENT;

COMPONENT tx_sample_buffer
GENERIC (bytes_per_sample : INTEGER;
			global_channel_count : INTEGER;
			samples_per_channel_depth : INTEGER
			);
	PORT(sys_clk : IN STD_LOGIC;
		 reset_n : IN STD_LOGIC;
		 fs_clk_i : IN STD_LOGIC;
		 metering_clear_i : IN STD_LOGIC;
		 audio_ch0_in : IN STD_LOGIC_VECTOR(23 DOWNTO 0);
		 audio_ch10_in : IN STD_LOGIC_VECTOR(23 DOWNTO 0);
		 audio_ch11_in : IN STD_LOGIC_VECTOR(23 DOWNTO 0);
		 audio_ch12_in : IN STD_LOGIC_VECTOR(23 DOWNTO 0);
		 audio_ch13_in : IN STD_LOGIC_VECTOR(23 DOWNTO 0);
		 audio_ch14_in : IN STD_LOGIC_VECTOR(23 DOWNTO 0);
		 audio_ch15_in : IN STD_LOGIC_VECTOR(23 DOWNTO 0);
		 audio_ch1_in : IN STD_LOGIC_VECTOR(23 DOWNTO 0);
		 audio_ch2_in : IN STD_LOGIC_VECTOR(23 DOWNTO 0);
		 audio_ch3_in : IN STD_LOGIC_VECTOR(23 DOWNTO 0);
		 audio_ch4_in : IN STD_LOGIC_VECTOR(23 DOWNTO 0);
		 audio_ch5_in : IN STD_LOGIC_VECTOR(23 DOWNTO 0);
		 audio_ch6_in : IN STD_LOGIC_VECTOR(23 DOWNTO 0);
		 audio_ch7_in : IN STD_LOGIC_VECTOR(23 DOWNTO 0);
		 audio_ch8_in : IN STD_LOGIC_VECTOR(23 DOWNTO 0);
		 audio_ch9_in : IN STD_LOGIC_VECTOR(23 DOWNTO 0);
		 read0Addr : IN STD_LOGIC_VECTOR(15 DOWNTO 0);
		 wr_ready_o : OUT STD_LOGIC;
		 data0_out : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
		 metering_clip_o : OUT STD_LOGIC_VECTOR(15 DOWNTO 0);
		 metering_signal_o : OUT STD_LOGIC_VECTOR(15 DOWNTO 0);
		 wr_ptr_o : OUT STD_LOGIC_VECTOR(15 DOWNTO 0)
	);
END COMPONENT;

COMPONENT tx_transmitter
GENERIC (bytes_per_sample : INTEGER;
			global_channel_count : INTEGER;
			samples_per_channel_depth : INTEGER
			);
	PORT(sys_clk : IN STD_LOGIC;
		 tx_clk : IN STD_LOGIC;
		 tx_busy : IN STD_LOGIC;
		 tx_byte_sent : IN STD_LOGIC;
		 start_i : IN STD_LOGIC;
		 tx_allow_i : IN STD_LOGIC;
		 ch_ids_i : IN STD_LOGIC_VECTOR(63 DOWNTO 0);
		 channel_count_i : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
		 dst_ip_address : IN STD_LOGIC_VECTOR(31 DOWNTO 0);
		 sample_buffer_tx_start_addr_i : IN STD_LOGIC_VECTOR(15 DOWNTO 0);
		 sample_counter : IN STD_LOGIC_VECTOR(31 DOWNTO 0);
		 sample_ram_data_in_i : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
		 samples_per_packet_per_channel_i : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
		 src_ip_address : IN STD_LOGIC_VECTOR(31 DOWNTO 0);
		 src_mac_address : IN STD_LOGIC_VECTOR(47 DOWNTO 0);
		 ssrc_i : IN STD_LOGIC_VECTOR(31 DOWNTO 0);
		 tx_enable : OUT STD_LOGIC;
		 tx_req_o : OUT STD_LOGIC;
		 sample_ram_read_addr_o : OUT STD_LOGIC_VECTOR(15 DOWNTO 0);
		 tx_data : OUT STD_LOGIC_VECTOR(7 DOWNTO 0)
	);
END COMPONENT;

SIGNAL	audio_allow_req :  STD_LOGIC;
SIGNAL	ch_ids :  STD_LOGIC_VECTOR(63 DOWNTO 0);
SIGNAL	channel_count :  STD_LOGIC_VECTOR(7 DOWNTO 0);
SIGNAL	packet_time :  STD_LOGIC_VECTOR(31 DOWNTO 0);
SIGNAL	sample_buffer_rd_ptr :  STD_LOGIC_VECTOR(15 DOWNTO 0);
SIGNAL	sample_buffer_start_addr :  STD_LOGIC_VECTOR(15 DOWNTO 0);
SIGNAL	sample_ram_read_addr :  STD_LOGIC_VECTOR(15 DOWNTO 0);
SIGNAL	sample_ram_read_data :  STD_LOGIC_VECTOR(7 DOWNTO 0);
SIGNAL	sample_ready :  STD_LOGIC;
SIGNAL	samples_per_packet_per_ch :  STD_LOGIC_VECTOR(7 DOWNTO 0);
SIGNAL	ssrc :  STD_LOGIC_VECTOR(31 DOWNTO 0);
SIGNAL	tx_data_audiotx :  STD_LOGIC_VECTOR(7 DOWNTO 0);
SIGNAL	tx_en_audiotx :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_0 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_1 :  STD_LOGIC_VECTOR(31 DOWNTO 0);


BEGIN 



b2v_inst37 : tx_router
GENERIC MAP(bytes_per_sample => 3,
			global_channel_count => 16,
			max_streams => 8,
			samples_per_channel_depth => 48
			)
PORT MAP(sys_clk_i => sys_clk,
		 reset_n => rst_n,
		 config_wr_en_i => cfg_wr_en_i,
		 sample_ready_i => sample_ready,
		 tx_en_i => tx_en_audiotx,
		 config_wr_clk_i => fmc_clk,
		 config_wr_addr_i => cfg_wr_addr_i,
		 config_wr_data_i => cfg_wr_data_i,
		 packet_time_i => media_clock_i,
		 sample_buffer_wr_ptr_i => sample_buffer_rd_ptr,
		 audio_packet_tx_start_o => SYNTHESIZED_WIRE_0,
		 ch_ids_o => ch_ids,
		 channel_count_o => channel_count,
		 ip_addr_o => SYNTHESIZED_WIRE_1,
		 packet_time_o => packet_time,
		 sample_buffer_tx_start_addr_o => sample_buffer_start_addr,
		 samples_per_packet_per_channel_o => samples_per_packet_per_ch,
		 ssrc_o => ssrc);


b2v_inst38 : tx_sample_buffer
GENERIC MAP(bytes_per_sample => 3,
			global_channel_count => 16,
			samples_per_channel_depth => 48
			)
PORT MAP(sys_clk => sys_clk,
		 reset_n => rst_n,
		 fs_clk_i => fs_clk_i,
		 metering_clear_i => metering_clear_i,
		 audio_ch0_in => ch0i,
		 audio_ch10_in => ch10i,
		 audio_ch11_in => ch11i,
		 audio_ch12_in => ch12i,
		 audio_ch13_in => ch13i,
		 audio_ch14_in => ch14i,
		 audio_ch15_in => ch15i,
		 audio_ch1_in => ch1i,
		 audio_ch2_in => ch2i,
		 audio_ch3_in => ch3i,
		 audio_ch4_in => ch4i,
		 audio_ch5_in => ch5i,
		 audio_ch6_in => ch6i,
		 audio_ch7_in => ch7i,
		 audio_ch8_in => ch8i,
		 audio_ch9_in => ch9i,
		 read0Addr => sample_ram_read_addr,
		 wr_ready_o => sample_ready,
		 data0_out => sample_ram_read_data,
		 metering_clip_o => metering_clip_o,
		 metering_signal_o => metering_signal_o,
		 wr_ptr_o => sample_buffer_rd_ptr);


b2v_inst39 : tx_transmitter
GENERIC MAP(bytes_per_sample => 3,
			global_channel_count => 16,
			samples_per_channel_depth => 48
			)
PORT MAP(sys_clk => sys_clk,
		 tx_clk => mac_tx_clock,
		 tx_busy => mac_tx_busy,
		 tx_byte_sent => mac_tx_byte_sent,
		 start_i => SYNTHESIZED_WIRE_0,
		 tx_allow_i => mac_audio_allow_i,
		 ch_ids_i => ch_ids,
		 channel_count_i => channel_count,
		 dst_ip_address => SYNTHESIZED_WIRE_1,
		 sample_buffer_tx_start_addr_i => sample_buffer_start_addr,
		 sample_counter => packet_time,
		 sample_ram_data_in_i => sample_ram_read_data,
		 samples_per_packet_per_channel_i => samples_per_packet_per_ch,
		 src_ip_address => ip_addr_i,
		 src_mac_address => mac_addr_i,
		 ssrc_i => ssrc,
		 tx_enable => tx_en_audiotx,
		 tx_req_o => audio_allow_req,
		 sample_ram_read_addr_o => sample_ram_read_addr,
		 tx_data => tx_data_audiotx);

tx_en_o <= tx_en_audiotx;
tx_req_o <= audio_allow_req;
tx_data_o <= tx_data_audiotx;

END bdf_type;