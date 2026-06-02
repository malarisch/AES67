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
-- CREATED		"Sat Apr  4 16:36:23 2026"

LIBRARY ieee;
USE ieee.std_logic_1164.all; 
USE ieee.numeric_std.all;

ENTITY audio_tx_module IS 
	GENERIC (bytes_per_sample : INTEGER := 3;
			global_channel_count : INTEGER := 16;
			max_streams : INTEGER := 8;
			samples_per_channel_depth : INTEGER := 48;
			ENABLE_METERING : BOOLEAN := true;
			-- TDM serial input frontend (forwarded to tx_sample_buffer)
			TDM_INPUT : BOOLEAN := true;
			TDM_INPUTS : INTEGER := 2;
			TDM_CHANNELS : INTEGER := 8
	);
	PORT
	(
		sys_clk :  IN  STD_LOGIC;
		ctrl_plane_clk :  IN  STD_LOGIC;
		rst_n :  IN  STD_LOGIC;
		fs_tdm_clk_i :  IN  STD_LOGIC;
		fs_halfduty_clk_i :  IN  STD_LOGIC;
		cfg_wr_en_i :  IN  STD_LOGIC;
		mac_tx_clock :  IN  STD_LOGIC;
		mac_tx_busy :  IN  STD_LOGIC;
		mac_tx_byte_sent :  IN  STD_LOGIC;
		mac_audio_allow_i :  IN  STD_LOGIC;
		metering_clear_i :  IN  STD_LOGIC;
		cfg_wr_addr_i :  IN  STD_LOGIC_VECTOR(7 DOWNTO 0);
		cfg_wr_data_i :  IN  STD_LOGIC_VECTOR(7 DOWNTO 0);
		audio_i : IN STD_LOGIC_VECTOR((bytes_per_sample * 8) * global_channel_count - 1 downto 0) := (others => '0');
		-- TDM serial input (only used when TDM_INPUT=true)
		bclk_i : IN STD_LOGIC := '0';
		tdm_in_i : IN STD_LOGIC_VECTOR(TDM_INPUTS - 1 downto 0) := (others => '0');
		ip_addr_i :  IN  STD_LOGIC_VECTOR(31 DOWNTO 0);
		mac_addr_i :  IN  STD_LOGIC_VECTOR(47 DOWNTO 0);
		media_clock_i :  IN  STD_LOGIC_VECTOR(31 DOWNTO 0);
		tx_en_o :  OUT  STD_LOGIC;
		tx_req_o :  OUT  STD_LOGIC;
		metering_clip_o :  OUT  STD_LOGIC_VECTOR(global_channel_count - 1 DOWNTO 0);
		metering_signal_o :  OUT  STD_LOGIC_VECTOR(global_channel_count - 1 DOWNTO 0);
		tx_data_o :  OUT  STD_LOGIC_VECTOR(7 DOWNTO 0);
		mac_speed_i : IN STD_LOGIC_VECTOR(1 downto 0)
	);
END audio_tx_module;

ARCHITECTURE bdf_type OF audio_tx_module IS 



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

SIGNAL sequence_id : UNSIGNED(15 downto 0);


BEGIN 



b2v_tx_router : entity work.tx_router
GENERIC MAP(bytes_per_sample => bytes_per_sample,
			global_channel_count => global_channel_count,
			samples_per_channel_depth => samples_per_channel_depth,
			max_streams => max_streams
			)
PORT MAP(sys_clk_i => sys_clk,
		 reset_n => rst_n,
		 config_wr_en_i => cfg_wr_en_i,
		 sample_ready_i => fs_halfduty_clk_i,
		 tx_en_i => tx_en_audiotx,
		 tx_busy_i => mac_tx_busy,
		 config_wr_clk_i => ctrl_plane_clk,
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
		 ssrc_o => ssrc,
		 sequence_id_o => sequence_id);


b2v_tx_sample_buffer : entity work.tx_sample_buffer
GENERIC MAP(bytes_per_sample => bytes_per_sample,
			global_channel_count => global_channel_count,
			samples_per_channel_depth => samples_per_channel_depth,
			ENABLE_METERING => ENABLE_METERING,
			TDM_INPUT => TDM_INPUT,
			TDM_INPUTS => TDM_INPUTS,
			TDM_CHANNELS => TDM_CHANNELS
			)
PORT MAP(sys_clk => sys_clk,
		 reset_n => rst_n,
		 fs_clk_i => fs_tdm_clk_i,
		 bclk_sync_i => bclk_i,
		 tdm_in => tdm_in_i,
		 metering_clear_i => metering_clear_i,
		 audio_in => audio_i,
		 read0Addr => unsigned(sample_ram_read_addr),
		 wr_ready_o => sample_ready,
		 data0_out => sample_ram_read_data,
		 metering_clip_o => metering_clip_o,
		 metering_signal_o => metering_signal_o,
		 wr_ptr_o => sample_buffer_rd_ptr);


b2v_tx_transmitter : entity work.tx_transmitter
GENERIC MAP(bytes_per_sample => bytes_per_sample,
			global_channel_count => global_channel_count,
			samples_per_channel_depth => samples_per_channel_depth
			)
PORT MAP(sys_clk => sys_clk,
		 tx_clk => mac_tx_clock,
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
		 tx_data => tx_data_audiotx,
		 sequence_id_in => sequence_id,
		 mac_speed_i => mac_speed_i);

tx_en_o <= tx_en_audiotx;
tx_req_o <= audio_allow_req;
tx_data_o <= tx_data_audiotx;

END bdf_type;