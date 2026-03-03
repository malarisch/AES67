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
-- CREATED		"Tue Mar  3 16:00:45 2026"

LIBRARY ieee;
USE ieee.std_logic_1164.all; 

LIBRARY work;

ENTITY FPGA IS 
	PORT
	(
		c10_resetn :  IN  STD_LOGIC;
		c10_clk50m :  IN  STD_LOGIC;
		gpio31 :  IN  STD_LOGIC;
		gpio23 :  IN  STD_LOGIC;
		gpio24 :  IN  STD_LOGIC;
		gpio22 :  IN  STD_LOGIC;
		gpio21 :  IN  STD_LOGIC;
		gpio0 :  IN  STD_LOGIC;
		gpio1 :  IN  STD_LOGIC;
		gpio2 :  IN  STD_LOGIC;
		gpio9 :  IN  STD_LOGIC;
		gpio8 :  IN  STD_LOGIC;
		gpio7 :  IN  STD_LOGIC;
		gpio6 :  IN  STD_LOGIC;
		gpio5 :  IN  STD_LOGIC;
		gpio4 :  IN  STD_LOGIC;
		gpio3 :  IN  STD_LOGIC;
		gpio34 :  IN  STD_LOGIC;
		gpio28 :  INOUT  STD_LOGIC;
		gpio17 :  INOUT  STD_LOGIC;
		gpio16 :  INOUT  STD_LOGIC;
		gpio15 :  INOUT  STD_LOGIC;
		gpio14 :  INOUT  STD_LOGIC;
		gpio13 :  INOUT  STD_LOGIC;
		gpio12 :  INOUT  STD_LOGIC;
		gpio11 :  INOUT  STD_LOGIC;
		gpio10 :  INOUT  STD_LOGIC;
		arduino_io4 :  OUT  STD_LOGIC;
		arduino_io5 :  OUT  STD_LOGIC;
		arduino_io :  OUT  STD_LOGIC_VECTOR(0 TO 0);
		gpio27 :  OUT  STD_LOGIC;
		gpio20 :  OUT  STD_LOGIC;
		gpio19 :  OUT  STD_LOGIC;
		gpio18 :  OUT  STD_LOGIC;
		gpio33 :  OUT  STD_LOGIC;
		gpio32 :  OUT  STD_LOGIC;
		gpio30 :  OUT  STD_LOGIC;
		user_led :  OUT  STD_LOGIC_VECTOR(3 TO 3)
	);
END FPGA;

ARCHITECTURE bdf_type OF FPGA IS 

COMPONENT ethernet_reset
	PORT(clk : IN STD_LOGIC;
		 power_good : IN STD_LOGIC;
		 phy_rstn : OUT STD_LOGIC;
		 mac_rst : OUT STD_LOGIC;
		 sendArpRequest : OUT STD_LOGIC;
		 tx_online : OUT STD_LOGIC
	);
END COMPONENT;

COMPONENT system_config_reg
GENERIC (lastAddress : INTEGER
			);
	PORT(wr_clk : IN STD_LOGIC;
		 wr_enable : IN STD_LOGIC;
		 done_in : IN STD_LOGIC;
		 sys_clock_i : IN STD_LOGIC;
		 data_in : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
		 read0Addr : IN STD_LOGIC_VECTOR(10 DOWNTO 0);
		 read1Addr : IN STD_LOGIC_VECTOR(10 DOWNTO 0);
		 read2Addr : IN STD_LOGIC_VECTOR(10 DOWNTO 0);
		 writeAddr : IN STD_LOGIC_VECTOR(10 DOWNTO 0);
		 valid_out : OUT STD_LOGIC;
		 data0_out : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
		 data1_out : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
		 data2_out : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
		 ip_addr_out : OUT STD_LOGIC_VECTOR(31 DOWNTO 0);
		 mac_addr_out : OUT STD_LOGIC_VECTOR(47 DOWNTO 0);
		 ptp_announce_log_message_interval_o : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
		 ptp_current_leader_id : OUT STD_LOGIC_VECTOR(63 DOWNTO 0);
		 ptp_log_message_interval_o : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
		 ptp_time_source_o : OUT STD_LOGIC_VECTOR(7 DOWNTO 0)
	);
END COMPONENT;

COMPONENT ethernet_packet_parser
GENERIC (udp_port : INTEGER;
			udp_port_ptpv2_event : INTEGER;
			udp_port_ptpv2_general : INTEGER;
			udp_port_rtp : INTEGER
			);
	PORT(clk : IN STD_LOGIC;
		 sync_in : IN STD_LOGIC;
		 arp_dst_ip : IN STD_LOGIC_VECTOR(31 DOWNTO 0);
		 arp_src_ip : IN STD_LOGIC_VECTOR(31 DOWNTO 0);
		 arp_type : IN STD_LOGIC_VECTOR(15 DOWNTO 0);
		 dst_ip_address : IN STD_LOGIC_VECTOR(31 DOWNTO 0);
		 ip_address : IN STD_LOGIC_VECTOR(31 DOWNTO 0);
		 ip_type : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
		 mac_address : IN STD_LOGIC_VECTOR(47 DOWNTO 0);
		 pkt_dst_mac : IN STD_LOGIC_VECTOR(47 DOWNTO 0);
		 pkt_src_mac : IN STD_LOGIC_VECTOR(47 DOWNTO 0);
		 pkt_type : IN STD_LOGIC_VECTOR(15 DOWNTO 0);
		 ram_data : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
		 udp_dst_port : IN STD_LOGIC_VECTOR(15 DOWNTO 0);
		 udp_length : IN STD_LOGIC_VECTOR(15 DOWNTO 0);
		 udp_src_port : IN STD_LOGIC_VECTOR(15 DOWNTO 0);
		 send_arp_response : OUT STD_LOGIC;
		 send_icmp_response : OUT STD_LOGIC;
		 send_udp_response : OUT STD_LOGIC;
		 parse_ptp_packet : OUT STD_LOGIC;
		 parse_rtp_packet : OUT STD_LOGIC;
		 arp_ip_address : OUT STD_LOGIC_VECTOR(31 DOWNTO 0);
		 arp_mac_address : OUT STD_LOGIC_VECTOR(47 DOWNTO 0);
		 dst_mac_address : OUT STD_LOGIC_VECTOR(47 DOWNTO 0);
		 icmp_dst_ip : OUT STD_LOGIC_VECTOR(31 DOWNTO 0);
		 icmp_dst_mac : OUT STD_LOGIC_VECTOR(47 DOWNTO 0);
		 icmp_id : OUT STD_LOGIC_VECTOR(15 DOWNTO 0);
		 icmp_sequence : OUT STD_LOGIC_VECTOR(15 DOWNTO 0);
		 ram_read_address : OUT STD_LOGIC_VECTOR(10 DOWNTO 0);
		 udp_payload : OUT STD_LOGIC_VECTOR(31 DOWNTO 0)
	);
END COMPONENT;

COMPONENT reverse_mac
	PORT(mac_address_i : IN STD_LOGIC_VECTOR(47 DOWNTO 0);
		 mac_address_o : OUT STD_LOGIC_VECTOR(47 DOWNTO 0)
	);
END COMPONENT;

COMPONENT audio_tx_module
	PORT(sys_clk : IN STD_LOGIC;
		 fmc_clk : IN STD_LOGIC;
		 rst_n : IN STD_LOGIC;
		 fs_clk_i : IN STD_LOGIC;
		 cfg_wr_en_i : IN STD_LOGIC;
		 mac_tx_clock : IN STD_LOGIC;
		 mac_tx_busy : IN STD_LOGIC;
		 mac_tx_byte_sent : IN STD_LOGIC;
		 mac_audio_allow_i : IN STD_LOGIC;
		 cfg_wr_addr_i : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
		 cfg_wr_data_i : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
		 ch0i : IN STD_LOGIC_VECTOR(23 DOWNTO 0);
		 ch10i : IN STD_LOGIC_VECTOR(23 DOWNTO 0);
		 ch11i : IN STD_LOGIC_VECTOR(23 DOWNTO 0);
		 ch12i : IN STD_LOGIC_VECTOR(23 DOWNTO 0);
		 ch13i : IN STD_LOGIC_VECTOR(23 DOWNTO 0);
		 ch14i : IN STD_LOGIC_VECTOR(23 DOWNTO 0);
		 ch15i : IN STD_LOGIC_VECTOR(23 DOWNTO 0);
		 ch1i : IN STD_LOGIC_VECTOR(23 DOWNTO 0);
		 ch2i : IN STD_LOGIC_VECTOR(23 DOWNTO 0);
		 ch3i : IN STD_LOGIC_VECTOR(23 DOWNTO 0);
		 ch4i : IN STD_LOGIC_VECTOR(23 DOWNTO 0);
		 ch5i : IN STD_LOGIC_VECTOR(23 DOWNTO 0);
		 ch6i : IN STD_LOGIC_VECTOR(23 DOWNTO 0);
		 ch7i : IN STD_LOGIC_VECTOR(23 DOWNTO 0);
		 ch8i : IN STD_LOGIC_VECTOR(23 DOWNTO 0);
		 ch9i : IN STD_LOGIC_VECTOR(23 DOWNTO 0);
		 ip_addr_i : IN STD_LOGIC_VECTOR(31 DOWNTO 0);
		 mac_addr_i : IN STD_LOGIC_VECTOR(47 DOWNTO 0);
		 media_clock_i : IN STD_LOGIC_VECTOR(31 DOWNTO 0);
		 tx_en_o : OUT STD_LOGIC;
		 tx_req_o : OUT STD_LOGIC;
		 tx_data_o : OUT STD_LOGIC_VECTOR(7 DOWNTO 0)
	);
END COMPONENT;

COMPONENT const_eth_speed
	PORT(		 result : OUT STD_LOGIC_VECTOR(1 DOWNTO 0)
	);
END COMPONENT;

COMPONENT clk_by_x
GENERIC (divider : INTEGER
			);
	PORT(clk_in : IN STD_LOGIC;
		 clk_out : OUT STD_LOGIC
	);
END COMPONENT;

COMPONENT eth_tx_arbiter
	PORT(clk_i : IN STD_LOGIC;
		 rst_n_i : IN STD_LOGIC;
		 ptp_req_i : IN STD_LOGIC;
		 audio_req_i : IN STD_LOGIC;
		 mcu_req_i : IN STD_LOGIC;
		 ptp_allow_o : OUT STD_LOGIC;
		 audio_allow_o : OUT STD_LOGIC;
		 mcu_allow_o : OUT STD_LOGIC
	);
END COMPONENT;

COMPONENT ethernet_packet_aggregator
	PORT(tx_en0_i : IN STD_LOGIC;
		 tx_en1_i : IN STD_LOGIC;
		 tx_en2_i : IN STD_LOGIC;
		 tx_en3_i : IN STD_LOGIC;
		 tx_en4_i : IN STD_LOGIC;
		 tx_en5_i : IN STD_LOGIC;
		 tx_en6_i : IN STD_LOGIC;
		 tx_en7_i : IN STD_LOGIC;
		 data0_i : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
		 data1_i : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
		 data2_i : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
		 data3_i : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
		 data4_i : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
		 data5_i : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
		 data6_i : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
		 data7_i : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
		 tx_en_o : OUT STD_LOGIC;
		 data_o : OUT STD_LOGIC_VECTOR(7 DOWNTO 0)
	);
END COMPONENT;

COMPONENT rx_ringbuffer
GENERIC (audio_buffer_sample_depth : INTEGER;
			bytes_per_sample : INTEGER;
			global_channel_count : INTEGER;
			max_streams : INTEGER
			);
	PORT(sys_clk : IN STD_LOGIC;
		 reset_n : IN STD_LOGIC;
		 fs_clk_i : IN STD_LOGIC;
		 packet_ready_i : IN STD_LOGIC;
		 stream_config_wr_clk_i : IN STD_LOGIC;
		 stream_config_wr_en_i : IN STD_LOGIC;
		 eth_read_data_i : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
		 media_clock_i : IN STD_LOGIC_VECTOR(31 DOWNTO 0);
		 stream_config_addr_i : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
		 stream_config_data_i : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
		 audio_ch0_out : OUT STD_LOGIC_VECTOR(23 DOWNTO 0);
		 audio_ch10_out : OUT STD_LOGIC_VECTOR(23 DOWNTO 0);
		 audio_ch11_out : OUT STD_LOGIC_VECTOR(23 DOWNTO 0);
		 audio_ch12_out : OUT STD_LOGIC_VECTOR(23 DOWNTO 0);
		 audio_ch13_out : OUT STD_LOGIC_VECTOR(23 DOWNTO 0);
		 audio_ch14_out : OUT STD_LOGIC_VECTOR(23 DOWNTO 0);
		 audio_ch15_out : OUT STD_LOGIC_VECTOR(23 DOWNTO 0);
		 audio_ch1_out : OUT STD_LOGIC_VECTOR(23 DOWNTO 0);
		 audio_ch2_out : OUT STD_LOGIC_VECTOR(23 DOWNTO 0);
		 audio_ch3_out : OUT STD_LOGIC_VECTOR(23 DOWNTO 0);
		 audio_ch4_out : OUT STD_LOGIC_VECTOR(23 DOWNTO 0);
		 audio_ch5_out : OUT STD_LOGIC_VECTOR(23 DOWNTO 0);
		 audio_ch6_out : OUT STD_LOGIC_VECTOR(23 DOWNTO 0);
		 audio_ch7_out : OUT STD_LOGIC_VECTOR(23 DOWNTO 0);
		 audio_ch8_out : OUT STD_LOGIC_VECTOR(23 DOWNTO 0);
		 audio_ch9_out : OUT STD_LOGIC_VECTOR(23 DOWNTO 0);
		 eth_read_addr_o : OUT STD_LOGIC_VECTOR(10 DOWNTO 0)
	);
END COMPONENT;

COMPONENT i2s_out
GENERIC (width : INTEGER
			);
	PORT(LR_CLK : IN STD_LOGIC;
		 BIT_CLK : IN STD_LOGIC;
		 RESET : IN STD_LOGIC;
		 SYS_CLK : IN STD_LOGIC;
		 DATA_L : IN STD_LOGIC_VECTOR(23 DOWNTO 0);
		 DATA_R : IN STD_LOGIC_VECTOR(23 DOWNTO 0);
		 DOUT : OUT STD_LOGIC
	);
END COMPONENT;

COMPONENT eth_ram
GENERIC (lastAddress : INTEGER
			);
	PORT(rx_clk : IN STD_LOGIC;
		 sync_in : IN STD_LOGIC;
		 data_in : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
		 read0Addr : IN STD_LOGIC_VECTOR(10 DOWNTO 0);
		 read1Addr : IN STD_LOGIC_VECTOR(10 DOWNTO 0);
		 read2Addr : IN STD_LOGIC_VECTOR(10 DOWNTO 0);
		 writeAddr : IN STD_LOGIC_VECTOR(10 DOWNTO 0);
		 sync_out : OUT STD_LOGIC;
		 arp_dst_ip : OUT STD_LOGIC_VECTOR(31 DOWNTO 0);
		 arp_src_ip : OUT STD_LOGIC_VECTOR(31 DOWNTO 0);
		 arp_type : OUT STD_LOGIC_VECTOR(15 DOWNTO 0);
		 data0_out : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
		 data1_out : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
		 data2_out : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
		 ip_type : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
		 pkt_dst_mac : OUT STD_LOGIC_VECTOR(47 DOWNTO 0);
		 pkt_src_mac : OUT STD_LOGIC_VECTOR(47 DOWNTO 0);
		 pkt_type : OUT STD_LOGIC_VECTOR(15 DOWNTO 0);
		 udp_dst_port : OUT STD_LOGIC_VECTOR(15 DOWNTO 0);
		 udp_length : OUT STD_LOGIC_VECTOR(15 DOWNTO 0);
		 udp_src_port : OUT STD_LOGIC_VECTOR(15 DOWNTO 0)
	);
END COMPONENT;

COMPONENT ptp_module
	PORT(sys_clk : IN STD_LOGIC;
		 rst_n : IN STD_LOGIC;
		 mac_tx_clock : IN STD_LOGIC;
		 mac_tx_busy : IN STD_LOGIC;
		 mac_tx_byte_sent : IN STD_LOGIC;
		 mac_tx_allow_i : IN STD_LOGIC;
		 parse_ptp_packet : IN STD_LOGIC;
		 ptp_is_follower : IN STD_LOGIC;
		 ptp_is_leader : IN STD_LOGIC;
		 eth_frame_i : IN STD_LOGIC;
		 ethernet_parser_sync : IN STD_LOGIC;
		 ip_address : IN STD_LOGIC_VECTOR(31 DOWNTO 0);
		 mac_address : IN STD_LOGIC_VECTOR(47 DOWNTO 0);
		 mac_ram_data : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
		 ptp_announce_interval : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
		 ptp_current_leader_id : IN STD_LOGIC_VECTOR(63 DOWNTO 0);
		 ptp_log_message_interval : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
		 ptp_time_source : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
		 tx_en_ptpfu : OUT STD_LOGIC;
		 ptp_allow_req : OUT STD_LOGIC;
		 ptp_locked : OUT STD_LOGIC;
		 ptp_sync_lost : OUT STD_LOGIC;
		 wallclock_locked : OUT STD_LOGIC;
		 wallclock_configured : OUT STD_LOGIC;
		 wallclock_phasejump : OUT STD_LOGIC;
		 wc_64fs : OUT STD_LOGIC;
		 second_pulse_sys : OUT STD_LOGIC;
		 media_clock : OUT STD_LOGIC_VECTOR(31 DOWNTO 0);
		 ptp_mean_path_delay : OUT STD_LOGIC_VECTOR(31 DOWNTO 0);
		 ptp_offset_from_master : OUT STD_LOGIC_VECTOR(31 DOWNTO 0);
		 ptp_ram_addr : OUT STD_LOGIC_VECTOR(10 DOWNTO 0);
		 tx_data_ptpfu : OUT STD_LOGIC_VECTOR(7 DOWNTO 0)
	);
END COMPONENT;

COMPONENT clock_ppb_meter
	PORT(sys_clk : IN STD_LOGIC;
		 reset_n : IN STD_LOGIC;
		 wallclock_64fs_in : IN STD_LOGIC;
		 pll_64fs_in : IN STD_LOGIC;
		 wallclock_second_pulse_i : IN STD_LOGIC;
		 start_i : IN STD_LOGIC;
		 valid_o : OUT STD_LOGIC;
		 count_pll_o : OUT STD_LOGIC_VECTOR(21 DOWNTO 0);
		 count_wc_o : OUT STD_LOGIC_VECTOR(21 DOWNTO 0)
	);
END COMPONENT;

COMPONENT ethernet_receive
GENERIC (lastRamAddress : INTEGER
			);
	PORT(rx_clk : IN STD_LOGIC;
		 rx_frame : IN STD_LOGIC;
		 rx_byte_received : IN STD_LOGIC;
		 rx_error : IN STD_LOGIC;
		 rx_data : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
		 frame_rdy : OUT STD_LOGIC;
		 ram_addr : OUT STD_LOGIC_VECTOR(10 DOWNTO 0);
		 ram_data : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
		 rx_byte_count : OUT STD_LOGIC_VECTOR(10 DOWNTO 0)
	);
END COMPONENT;

COMPONENT ethernet
GENERIC (MIIM_CLOCK_DIVIDER : INTEGER;
			MIIM_DISABLE : BOOLEAN;
			MIIM_PHY_ADDRESS : STD_LOGIC_VECTOR(4 DOWNTO 0);
			MIIM_POLL_WAIT_TICKS : INTEGER;
			MIIM_RESET_WAIT_TICKS : INTEGER
			);
	PORT(clock_125_i : IN STD_LOGIC;
		 reset_i : IN STD_LOGIC;
		 mii_tx_clk_i : IN STD_LOGIC;
		 mii_rx_clk_i : IN STD_LOGIC;
		 mii_rx_er_i : IN STD_LOGIC;
		 mii_rx_dv_i : IN STD_LOGIC;
		 rgmii_rx_ctl_i : IN STD_LOGIC;
		 miim_clock_i : IN STD_LOGIC;
		 tx_enable_i : IN STD_LOGIC;
		 mdio_io : INOUT STD_LOGIC;
		 mac_address_i : IN STD_LOGIC_VECTOR(47 DOWNTO 0);
		 mii_rxd_i : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
		 speed_override_i : IN STD_LOGIC_VECTOR(1 DOWNTO 0);
		 tx_data_i : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
		 reset_o : OUT STD_LOGIC;
		 mii_tx_er_o : OUT STD_LOGIC;
		 mii_tx_en_o : OUT STD_LOGIC;
		 gmii_gtx_clk_o : OUT STD_LOGIC;
		 rgmii_tx_ctl_o : OUT STD_LOGIC;
		 mdc_o : OUT STD_LOGIC;
		 link_up_o : OUT STD_LOGIC;
		 tx_clock_o : OUT STD_LOGIC;
		 tx_reset_o : OUT STD_LOGIC;
		 tx_byte_sent_o : OUT STD_LOGIC;
		 tx_busy_o : OUT STD_LOGIC;
		 rx_clock_o : OUT STD_LOGIC;
		 rx_reset_o : OUT STD_LOGIC;
		 rx_frame_o : OUT STD_LOGIC;
		 rx_byte_received_o : OUT STD_LOGIC;
		 rx_error_o : OUT STD_LOGIC;
		 frame_o : OUT STD_LOGIC;
		 mii_txd_o : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
		 rx_data_o : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
		 speed_o : OUT STD_LOGIC_VECTOR(1 DOWNTO 0)
	);
END COMPONENT;

COMPONENT miirmii
	PORT(ena_10 : IN STD_LOGIC;
		 RefClk : IN STD_LOGIC;
		 m_tx_en : IN STD_LOGIC;
		 m_tx_err : IN STD_LOGIC;
		 Rstn : IN STD_LOGIC;
		 rmii_crs_dv : IN STD_LOGIC;
		 rmii_rx_err : IN STD_LOGIC;
		 m_tx_d : IN STD_LOGIC_VECTOR(3 DOWNTO 0);
		 rmii_rx_d : IN STD_LOGIC_VECTOR(1 DOWNTO 0);
		 m_rx_en : OUT STD_LOGIC;
		 m_rx_err : OUT STD_LOGIC;
		 m_rx_crs : OUT STD_LOGIC;
		 m_rx_col : OUT STD_LOGIC;
		 rx_clk : OUT STD_LOGIC;
		 tx_clk : OUT STD_LOGIC;
		 rmii_tx_en : OUT STD_LOGIC;
		 m_rx_d : OUT STD_LOGIC_VECTOR(3 DOWNTO 0);
		 rmii_tx_d : OUT STD_LOGIC_VECTOR(1 DOWNTO 0)
	);
END COMPONENT;

COMPONENT pll_systemclocks
	PORT(inclk0 : IN STD_LOGIC;
		 c0 : OUT STD_LOGIC;
		 c1 : OUT STD_LOGIC;
		 c3 : OUT STD_LOGIC
	);
END COMPONENT;

COMPONENT const_ch_output_debug
	PORT(		 ch0_o : OUT STD_LOGIC_VECTOR(23 DOWNTO 0);
		 ch10_o : OUT STD_LOGIC_VECTOR(23 DOWNTO 0);
		 ch11_o : OUT STD_LOGIC_VECTOR(23 DOWNTO 0);
		 ch12_o : OUT STD_LOGIC_VECTOR(23 DOWNTO 0);
		 ch13_o : OUT STD_LOGIC_VECTOR(23 DOWNTO 0);
		 ch14_o : OUT STD_LOGIC_VECTOR(23 DOWNTO 0);
		 ch15_o : OUT STD_LOGIC_VECTOR(23 DOWNTO 0);
		 ch1_o : OUT STD_LOGIC_VECTOR(23 DOWNTO 0);
		 ch2_o : OUT STD_LOGIC_VECTOR(23 DOWNTO 0);
		 ch3_o : OUT STD_LOGIC_VECTOR(23 DOWNTO 0);
		 ch4_o : OUT STD_LOGIC_VECTOR(23 DOWNTO 0);
		 ch5_o : OUT STD_LOGIC_VECTOR(23 DOWNTO 0);
		 ch6_o : OUT STD_LOGIC_VECTOR(23 DOWNTO 0);
		 ch7_o : OUT STD_LOGIC_VECTOR(23 DOWNTO 0);
		 ch8_o : OUT STD_LOGIC_VECTOR(23 DOWNTO 0);
		 ch9_o : OUT STD_LOGIC_VECTOR(23 DOWNTO 0)
	);
END COMPONENT;

COMPONENT fmc_ethernet_client
GENERIC (RX_ADDR_WIDTH : INTEGER;
			TX_ADDR_WIDTH : INTEGER
			);
	PORT(clk_sys_i : IN STD_LOGIC;
		 rst_sys_i : IN STD_LOGIC;
		 fmc_ne_n_i : IN STD_LOGIC;
		 fmc_noe_n_i : IN STD_LOGIC;
		 fmc_nwe_n_i : IN STD_LOGIC;
		 mac_tx_clock_i : IN STD_LOGIC;
		 mac_tx_reset_i : IN STD_LOGIC;
		 mac_tx_byte_sent_i : IN STD_LOGIC;
		 mac_tx_busy_i : IN STD_LOGIC;
		 mac_rx_clock_i : IN STD_LOGIC;
		 mac_rx_reset_i : IN STD_LOGIC;
		 mac_rx_frame_i : IN STD_LOGIC;
		 mac_rx_byte_rcv_i : IN STD_LOGIC;
		 mac_rx_error_i : IN STD_LOGIC;
		 pll_ppb_measurement_valid_i : IN STD_LOGIC;
		 wallclock_locked_i : IN STD_LOGIC;
		 wallclock_phasejump_i : IN STD_LOGIC;
		 wallclock_configured_i : IN STD_LOGIC;
		 ptp_leader_lost_i : IN STD_LOGIC;
		 eth_link_up_i : IN STD_LOGIC;
		 tx_allow_i : IN STD_LOGIC;
		 clock_count_pll_i : IN STD_LOGIC_VECTOR(21 DOWNTO 0);
		 clock_count_wc_i : IN STD_LOGIC_VECTOR(21 DOWNTO 0);
		 eth_link_speed_i : IN STD_LOGIC_VECTOR(1 DOWNTO 0);
		 fmc_addr_i : IN STD_LOGIC_VECTOR(6 DOWNTO 0);
		 fmc_data_io : INOUT STD_LOGIC_VECTOR(7 DOWNTO 0);
		 leader_offset_i : IN STD_LOGIC_VECTOR(31 DOWNTO 0);
		 mac_rx_data_i : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
		 path_delay_i : IN STD_LOGIC_VECTOR(31 DOWNTO 0);
		 system_config_rd_data : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
		 fmc_int_o : OUT STD_LOGIC;
		 mac_tx_enable_o : OUT STD_LOGIC;
		 system_config_wr_en : OUT STD_LOGIC;
		 system_config_done_o : OUT STD_LOGIC;
		 pll_ppb_measurement_start_o : OUT STD_LOGIC;
		 reset_wallclock_o : OUT STD_LOGIC;
		 reset_ptp_o : OUT STD_LOGIC;
		 reset_ethernet_o : OUT STD_LOGIC;
		 ptp_is_leader_o : OUT STD_LOGIC;
		 ptp_is_follower_o : OUT STD_LOGIC;
		 tx_stream_config_wr_en_o : OUT STD_LOGIC;
		 tx_allow_req_o : OUT STD_LOGIC;
		 rx_stream_config_wr_clk_o : OUT STD_LOGIC;
		 rx_stream_config_wr_en_o : OUT STD_LOGIC;
		 mac_tx_data_o : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
		 rx_stream_config_wr_addr_o : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
		 rx_stream_config_wr_data_o : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
		 system_config_rd_addr : OUT STD_LOGIC_VECTOR(10 DOWNTO 0);
		 system_config_wr_addr : OUT STD_LOGIC_VECTOR(10 DOWNTO 0);
		 system_config_wr_data : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
		 tx_stream_config_wr_addr_o : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
		 tx_stream_config_wr_data_o : OUT STD_LOGIC_VECTOR(7 DOWNTO 0)
	);
END COMPONENT;

COMPONENT i2s_in
GENERIC (fifo_addrbits : INTEGER;
			width : INTEGER
			);
	PORT(LR_CLK : IN STD_LOGIC;
		 BIT_CLK : IN STD_LOGIC;
		 DIN : IN STD_LOGIC;
		 RESET : IN STD_LOGIC;
		 SYS_CLK : IN STD_LOGIC;
		 DATA_RDY_L : OUT STD_LOGIC;
		 DATA_RDY_R : OUT STD_LOGIC;
		 DATA_L : OUT STD_LOGIC_VECTOR(23 DOWNTO 0);
		 DATA_R : OUT STD_LOGIC_VECTOR(23 DOWNTO 0)
	);
END COMPONENT;

SIGNAL	arp_ip_address :  STD_LOGIC_VECTOR(31 DOWNTO 0);
SIGNAL	arp_mac_address :  STD_LOGIC_VECTOR(47 DOWNTO 0);
SIGNAL	audio_allow :  STD_LOGIC;
SIGNAL	audio_allow_req :  STD_LOGIC;
SIGNAL	ch0i :  STD_LOGIC_VECTOR(23 DOWNTO 0);
SIGNAL	ch0o :  STD_LOGIC_VECTOR(23 DOWNTO 0);
SIGNAL	ch10i :  STD_LOGIC_VECTOR(23 DOWNTO 0);
SIGNAL	ch11i :  STD_LOGIC_VECTOR(23 DOWNTO 0);
SIGNAL	ch12i :  STD_LOGIC_VECTOR(23 DOWNTO 0);
SIGNAL	ch13i :  STD_LOGIC_VECTOR(23 DOWNTO 0);
SIGNAL	ch14i :  STD_LOGIC_VECTOR(23 DOWNTO 0);
SIGNAL	ch15i :  STD_LOGIC_VECTOR(23 DOWNTO 0);
SIGNAL	ch1i :  STD_LOGIC_VECTOR(23 DOWNTO 0);
SIGNAL	ch1o :  STD_LOGIC_VECTOR(23 DOWNTO 0);
SIGNAL	ch4i :  STD_LOGIC_VECTOR(23 DOWNTO 0);
SIGNAL	ch5i :  STD_LOGIC_VECTOR(23 DOWNTO 0);
SIGNAL	ch6i :  STD_LOGIC_VECTOR(23 DOWNTO 0);
SIGNAL	ch7i :  STD_LOGIC_VECTOR(23 DOWNTO 0);
SIGNAL	ch8i :  STD_LOGIC_VECTOR(23 DOWNTO 0);
SIGNAL	ch9i :  STD_LOGIC_VECTOR(23 DOWNTO 0);
SIGNAL	clk50m :  STD_LOGIC;
SIGNAL	clk_125MHz :  STD_LOGIC;
SIGNAL	clk_1kHz :  STD_LOGIC;
SIGNAL	clk_1MHz :  STD_LOGIC;
SIGNAL	clk_250MHz :  STD_LOGIC;
SIGNAL	const_eth_speed :  STD_LOGIC_VECTOR(1 DOWNTO 0);
SIGNAL	dst_mac_address :  STD_LOGIC_VECTOR(47 DOWNTO 0);
SIGNAL	eth_byte_cnt :  STD_LOGIC_VECTOR(10 DOWNTO 0);
SIGNAL	eth_frame_o :  STD_LOGIC;
SIGNAL	eth_frame_rdy :  STD_LOGIC;
SIGNAL	eth_send_arp_response :  STD_LOGIC;
SIGNAL	eth_send_icmp_response :  STD_LOGIC;
SIGNAL	ethernet_parser_sync :  STD_LOGIC;
SIGNAL	fmc_addr :  STD_LOGIC_VECTOR(6 DOWNTO 0);
SIGNAL	fmc_data :  STD_LOGIC_VECTOR(7 DOWNTO 0);
SIGNAL	fmc_int_o :  STD_LOGIC;
SIGNAL	fmc_ne :  STD_LOGIC;
SIGNAL	fmc_noe :  STD_LOGIC;
SIGNAL	fmc_nwe :  STD_LOGIC;
SIGNAL	fs :  STD_LOGIC;
SIGNAL	i2s_i1 :  STD_LOGIC;
SIGNAL	i2s_o1 :  STD_LOGIC;
SIGNAL	icmp_dst_ip_address :  STD_LOGIC_VECTOR(31 DOWNTO 0);
SIGNAL	icmp_dst_mac_address :  STD_LOGIC_VECTOR(47 DOWNTO 0);
SIGNAL	icmp_id :  STD_LOGIC_VECTOR(15 DOWNTO 0);
SIGNAL	icmp_sequence :  STD_LOGIC_VECTOR(15 DOWNTO 0);
SIGNAL	inch2i :  STD_LOGIC_VECTOR(23 DOWNTO 0);
SIGNAL	inch3i :  STD_LOGIC_VECTOR(23 DOWNTO 0);
SIGNAL	ip_address :  STD_LOGIC_VECTOR(31 DOWNTO 0);
SIGNAL	mac_address :  STD_LOGIC_VECTOR(47 DOWNTO 0);
SIGNAL	mac_linkup :  STD_LOGIC;
SIGNAL	mac_reset :  STD_LOGIC;
SIGNAL	mac_rx_byte_received :  STD_LOGIC;
SIGNAL	mac_rx_clock :  STD_LOGIC;
SIGNAL	mac_rx_data :  STD_LOGIC_VECTOR(7 DOWNTO 0);
SIGNAL	mac_rx_error :  STD_LOGIC;
SIGNAL	mac_rx_frame :  STD_LOGIC;
SIGNAL	mac_rx_reset :  STD_LOGIC;
SIGNAL	mac_speed :  STD_LOGIC_VECTOR(1 DOWNTO 0);
SIGNAL	mac_tx_busy :  STD_LOGIC;
SIGNAL	mac_tx_byte_sent :  STD_LOGIC;
SIGNAL	mac_tx_byte_sent_n :  STD_LOGIC;
SIGNAL	mac_tx_clock :  STD_LOGIC;
SIGNAL	mac_tx_data :  STD_LOGIC_VECTOR(7 DOWNTO 0);
SIGNAL	mac_tx_enable :  STD_LOGIC;
SIGNAL	mac_tx_reset :  STD_LOGIC;
SIGNAL	mcu_allow :  STD_LOGIC;
SIGNAL	mcu_allow_req :  STD_LOGIC;
SIGNAL	mcu_tx_data :  STD_LOGIC_VECTOR(7 DOWNTO 0);
SIGNAL	mcu_tx_enable :  STD_LOGIC;
SIGNAL	media_clock :  STD_LOGIC_VECTOR(31 DOWNTO 0);
SIGNAL	MII_RX_CLK :  STD_LOGIC;
SIGNAL	MII_RX_DV :  STD_LOGIC;
SIGNAL	MII_RX_ERR :  STD_LOGIC;
SIGNAL	MII_TX_CLK :  STD_LOGIC;
SIGNAL	parse_ptp_packet :  STD_LOGIC;
SIGNAL	parse_rtp_packet :  STD_LOGIC;
SIGNAL	phy_reset :  STD_LOGIC;
SIGNAL	PHY_RX_D :  STD_LOGIC_VECTOR(7 DOWNTO 0);
SIGNAL	PHY_TX_D :  STD_LOGIC_VECTOR(7 DOWNTO 0);
SIGNAL	pll_counter :  STD_LOGIC_VECTOR(21 DOWNTO 0);
SIGNAL	pll_fs128 :  STD_LOGIC;
SIGNAL	pll_fs256 :  STD_LOGIC;
SIGNAL	pll_fs512 :  STD_LOGIC;
SIGNAL	pll_fs64 :  STD_LOGIC;
SIGNAL	pll_meas_valid :  STD_LOGIC;
SIGNAL	powerGood :  STD_LOGIC;
SIGNAL	ptp_allow :  STD_LOGIC;
SIGNAL	ptp_allow_req :  STD_LOGIC;
SIGNAL	ptp_announce_interval :  STD_LOGIC_VECTOR(7 DOWNTO 0);
SIGNAL	ptp_current_leader_id :  STD_LOGIC_VECTOR(63 DOWNTO 0);
SIGNAL	ptp_is_follower :  STD_LOGIC;
SIGNAL	ptp_is_leader :  STD_LOGIC;
SIGNAL	ptp_log_message_interval :  STD_LOGIC_VECTOR(7 DOWNTO 0);
SIGNAL	ptp_mean_path_delay :  STD_LOGIC_VECTOR(31 DOWNTO 0);
SIGNAL	ptp_offset_from_master :  STD_LOGIC_VECTOR(31 DOWNTO 0);
SIGNAL	ptp_ram_addr :  STD_LOGIC_VECTOR(10 DOWNTO 0);
SIGNAL	ptp_ram_data :  STD_LOGIC_VECTOR(7 DOWNTO 0);
SIGNAL	ptp_sync_lost :  STD_LOGIC;
SIGNAL	ptp_time_source :  STD_LOGIC_VECTOR(7 DOWNTO 0);
SIGNAL	rmii_crs :  STD_LOGIC;
SIGNAL	rtp_ram_addr :  STD_LOGIC_VECTOR(10 DOWNTO 0);
SIGNAL	rtp_ram_data :  STD_LOGIC_VECTOR(7 DOWNTO 0);
SIGNAL	rx_conf_wr_addr :  STD_LOGIC_VECTOR(7 DOWNTO 0);
SIGNAL	rx_conf_wr_clk :  STD_LOGIC;
SIGNAL	rx_conf_wr_data :  STD_LOGIC_VECTOR(7 DOWNTO 0);
SIGNAL	rx_conf_wr_en :  STD_LOGIC;
SIGNAL	second_pulse_sys :  STD_LOGIC;
SIGNAL	sys_conf_rd_addr :  STD_LOGIC_VECTOR(10 DOWNTO 0);
SIGNAL	sys_conf_rd_data :  STD_LOGIC_VECTOR(7 DOWNTO 0);
SIGNAL	sys_conf_wr_addr :  STD_LOGIC_VECTOR(10 DOWNTO 0);
SIGNAL	sys_conf_wr_data :  STD_LOGIC_VECTOR(7 DOWNTO 0);
SIGNAL	sys_conf_wr_done :  STD_LOGIC;
SIGNAL	sys_conf_wr_en :  STD_LOGIC;
SIGNAL	tx_data_audiotx :  STD_LOGIC_VECTOR(7 DOWNTO 0);
SIGNAL	tx_data_ptpfu :  STD_LOGIC_VECTOR(7 DOWNTO 0);
SIGNAL	tx_en_audiotx :  STD_LOGIC;
SIGNAL	tx_en_ptpfu :  STD_LOGIC;
SIGNAL	tx_wr_addr :  STD_LOGIC_VECTOR(7 DOWNTO 0);
SIGNAL	tx_wr_data :  STD_LOGIC_VECTOR(7 DOWNTO 0);
SIGNAL	tx_wr_en :  STD_LOGIC;
SIGNAL	wallclock_configured :  STD_LOGIC;
SIGNAL	wallclock_locked :  STD_LOGIC;
SIGNAL	wallclock_phasejump :  STD_LOGIC;
SIGNAL	wc_64fs :  STD_LOGIC;
SIGNAL	wc_counter :  STD_LOGIC_VECTOR(21 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_0 :  STD_LOGIC_VECTOR(31 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_1 :  STD_LOGIC_VECTOR(31 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_2 :  STD_LOGIC_VECTOR(15 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_3 :  STD_LOGIC_VECTOR(7 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_4 :  STD_LOGIC_VECTOR(47 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_5 :  STD_LOGIC_VECTOR(47 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_6 :  STD_LOGIC_VECTOR(15 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_7 :  STD_LOGIC_VECTOR(7 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_8 :  STD_LOGIC_VECTOR(15 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_9 :  STD_LOGIC_VECTOR(15 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_10 :  STD_LOGIC_VECTOR(15 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_11 :  STD_LOGIC_VECTOR(7 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_12 :  STD_LOGIC_VECTOR(10 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_13 :  STD_LOGIC_VECTOR(10 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_14 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_15 :  STD_LOGIC_VECTOR(47 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_16 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_17 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_18 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_19 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_20 :  STD_LOGIC_VECTOR(1 DOWNTO 0);

SIGNAL	GDFX_TEMP_SIGNAL_0 :  STD_LOGIC_VECTOR(1 DOWNTO 0);

BEGIN 
gpio20 <= SYNTHESIZED_WIRE_20(1);
gpio19 <= SYNTHESIZED_WIRE_20(0);
SYNTHESIZED_WIRE_18 <= '0';

GDFX_TEMP_SIGNAL_0 <= (gpio22 & gpio21);



b2v_inst1 : ethernet_reset
PORT MAP(clk => clk_1kHz,
		 power_good => powerGood,
		 phy_rstn => phy_reset,
		 mac_rst => mac_reset);


b2v_inst10 : system_config_reg
GENERIC MAP(lastAddress => 512
			)
PORT MAP(wr_clk => clk_250MHz,
		 wr_enable => sys_conf_wr_en,
		 done_in => sys_conf_wr_done,
		 sys_clock_i => clk_125MHz,
		 data_in => sys_conf_wr_data,
		 read0Addr => sys_conf_rd_addr,
		 writeAddr => sys_conf_wr_addr,
		 data0_out => sys_conf_rd_data,
		 ip_addr_out => ip_address,
		 mac_addr_out => mac_address,
		 ptp_announce_log_message_interval_o => ptp_announce_interval,
		 ptp_current_leader_id => ptp_current_leader_id,
		 ptp_log_message_interval_o => ptp_log_message_interval,
		 ptp_time_source_o => ptp_time_source);


b2v_inst11 : ethernet_packet_parser
GENERIC MAP(udp_port => 4023,
			udp_port_ptpv2_event => 319,
			udp_port_ptpv2_general => 320,
			udp_port_rtp => 5004
			)
PORT MAP(clk => mac_rx_clock,
		 sync_in => ethernet_parser_sync,
		 arp_dst_ip => SYNTHESIZED_WIRE_0,
		 arp_src_ip => SYNTHESIZED_WIRE_1,
		 arp_type => SYNTHESIZED_WIRE_2,
		 ip_address => ip_address,
		 ip_type => SYNTHESIZED_WIRE_3,
		 mac_address => mac_address,
		 pkt_dst_mac => SYNTHESIZED_WIRE_4,
		 pkt_src_mac => SYNTHESIZED_WIRE_5,
		 pkt_type => SYNTHESIZED_WIRE_6,
		 ram_data => SYNTHESIZED_WIRE_7,
		 udp_dst_port => SYNTHESIZED_WIRE_8,
		 udp_length => SYNTHESIZED_WIRE_9,
		 udp_src_port => SYNTHESIZED_WIRE_10,
		 parse_ptp_packet => parse_ptp_packet,
		 parse_rtp_packet => parse_rtp_packet,
		 ram_read_address => SYNTHESIZED_WIRE_12);


b2v_inst12 : reverse_mac
PORT MAP(mac_address_i => mac_address,
		 mac_address_o => SYNTHESIZED_WIRE_15);


b2v_inst13 : audio_tx_module
PORT MAP(sys_clk => clk_125MHz,
		 fmc_clk => clk_250MHz,
		 rst_n => powerGood,
		 fs_clk_i => fs,
		 cfg_wr_en_i => tx_wr_en,
		 mac_tx_clock => mac_tx_clock,
		 mac_tx_busy => mac_tx_busy,
		 mac_tx_byte_sent => mac_tx_byte_sent,
		 mac_audio_allow_i => audio_allow_req,
		 cfg_wr_addr_i => tx_wr_addr,
		 cfg_wr_data_i => tx_wr_data,
		 ch0i => ch0i,
		 ch10i => ch10i,
		 ch11i => ch11i,
		 ch12i => ch12i,
		 ch13i => ch13i,
		 ch14i => ch14i,
		 ch15i => ch15i,
		 ch1i => ch1i,
		 ch2i => inch2i,
		 ch3i => inch3i,
		 ch4i => ch4i,
		 ch5i => ch5i,
		 ch6i => ch6i,
		 ch7i => ch7i,
		 ch8i => ch8i,
		 ch9i => ch9i,
		 ip_addr_i => ip_address,
		 mac_addr_i => mac_address,
		 media_clock_i => media_clock,
		 tx_en_o => tx_en_audiotx,
		 tx_req_o => audio_allow_req,
		 tx_data_o => tx_data_audiotx);


b2v_inst15 : const_eth_speed
PORT MAP(		 result => const_eth_speed);


b2v_inst16 : clk_by_x
GENERIC MAP(divider => 2
			)
PORT MAP(clk_in => pll_fs256,
		 clk_out => pll_fs128);


arduino_io4 <= NOT(pll_fs512);



b2v_inst18 : eth_tx_arbiter
PORT MAP(clk_i => mac_tx_clock,
		 rst_n_i => powerGood,
		 ptp_req_i => ptp_allow_req,
		 audio_req_i => audio_allow_req,
		 mcu_req_i => mcu_allow_req,
		 ptp_allow_o => ptp_allow,
		 mcu_allow_o => mcu_allow);


b2v_inst19 : ethernet_packet_aggregator
PORT MAP(tx_en3_i => mcu_tx_enable,
		 tx_en4_i => tx_en_audiotx,
		 tx_en6_i => tx_en_ptpfu,
		 data3_i => mcu_tx_data,
		 data4_i => tx_data_audiotx,
		 data6_i => tx_data_ptpfu,
		 tx_en_o => mac_tx_enable,
		 data_o => mac_tx_data);


SYNTHESIZED_WIRE_19 <= NOT(powerGood);



b2v_inst20 : rx_ringbuffer
GENERIC MAP(audio_buffer_sample_depth => 128,
			bytes_per_sample => 3,
			global_channel_count => 16,
			max_streams => 4
			)
PORT MAP(sys_clk => clk_125MHz,
		 reset_n => powerGood,
		 fs_clk_i => fs,
		 packet_ready_i => parse_rtp_packet,
		 stream_config_wr_clk_i => rx_conf_wr_clk,
		 stream_config_wr_en_i => rx_conf_wr_en,
		 eth_read_data_i => rtp_ram_data,
		 media_clock_i => media_clock,
		 stream_config_addr_i => rx_conf_wr_addr,
		 stream_config_data_i => rx_conf_wr_data,
		 audio_ch0_out => ch0o,
		 audio_ch1_out => ch1o,
		 eth_read_addr_o => rtp_ram_addr);


b2v_inst21 : i2s_out
GENERIC MAP(width => 24
			)
PORT MAP(LR_CLK => fs,
		 BIT_CLK => pll_fs64,
		 RESET => powerGood,
		 SYS_CLK => clk_125MHz,
		 DATA_L => ch0o,
		 DATA_R => ch1o,
		 DOUT => i2s_o1);


b2v_inst22 : eth_ram
GENERIC MAP(lastAddress => 1500
			)
PORT MAP(rx_clk => mac_rx_clock,
		 sync_in => eth_frame_rdy,
		 data_in => SYNTHESIZED_WIRE_11,
		 read0Addr => SYNTHESIZED_WIRE_12,
		 read1Addr => ptp_ram_addr,
		 read2Addr => rtp_ram_addr,
		 writeAddr => SYNTHESIZED_WIRE_13,
		 sync_out => ethernet_parser_sync,
		 arp_dst_ip => SYNTHESIZED_WIRE_0,
		 arp_src_ip => SYNTHESIZED_WIRE_1,
		 arp_type => SYNTHESIZED_WIRE_2,
		 data0_out => SYNTHESIZED_WIRE_7,
		 data1_out => ptp_ram_data,
		 data2_out => rtp_ram_data,
		 ip_type => SYNTHESIZED_WIRE_3,
		 pkt_dst_mac => SYNTHESIZED_WIRE_4,
		 pkt_src_mac => SYNTHESIZED_WIRE_5,
		 pkt_type => SYNTHESIZED_WIRE_6,
		 udp_dst_port => SYNTHESIZED_WIRE_8,
		 udp_length => SYNTHESIZED_WIRE_9,
		 udp_src_port => SYNTHESIZED_WIRE_10);


b2v_inst24 : ptp_module
PORT MAP(sys_clk => clk_125MHz,
		 rst_n => powerGood,
		 mac_tx_clock => mac_tx_clock,
		 mac_tx_busy => mac_tx_busy,
		 mac_tx_byte_sent => mac_tx_byte_sent,
		 mac_tx_allow_i => ptp_allow,
		 parse_ptp_packet => parse_ptp_packet,
		 ptp_is_follower => ptp_is_follower,
		 ptp_is_leader => ptp_is_leader,
		 eth_frame_i => eth_frame_o,
		 ethernet_parser_sync => ethernet_parser_sync,
		 ip_address => ip_address,
		 mac_address => mac_address,
		 mac_ram_data => ptp_ram_data,
		 ptp_announce_interval => ptp_announce_interval,
		 ptp_current_leader_id => ptp_current_leader_id,
		 ptp_log_message_interval => ptp_log_message_interval,
		 ptp_time_source => ptp_time_source,
		 tx_en_ptpfu => tx_en_ptpfu,
		 ptp_allow_req => ptp_allow_req,
		 ptp_sync_lost => ptp_sync_lost,
		 wallclock_locked => wallclock_locked,
		 wallclock_configured => wallclock_configured,
		 wallclock_phasejump => wallclock_phasejump,
		 wc_64fs => wc_64fs,
		 second_pulse_sys => second_pulse_sys,
		 media_clock => media_clock,
		 ptp_mean_path_delay => ptp_mean_path_delay,
		 ptp_offset_from_master => ptp_offset_from_master,
		 ptp_ram_addr => ptp_ram_addr,
		 tx_data_ptpfu => tx_data_ptpfu);


mac_tx_byte_sent_n <= NOT(mac_tx_byte_sent);



b2v_inst27 : clk_by_x
GENERIC MAP(divider => 2
			)
PORT MAP(clk_in => pll_fs128,
		 clk_out => pll_fs64);


b2v_inst29 : clk_by_x
GENERIC MAP(divider => 64
			)
PORT MAP(clk_in => pll_fs64,
		 clk_out => fs);


b2v_inst31 : clock_ppb_meter
PORT MAP(sys_clk => clk_125MHz,
		 reset_n => powerGood,
		 wallclock_64fs_in => wc_64fs,
		 pll_64fs_in => pll_fs64,
		 wallclock_second_pulse_i => second_pulse_sys,
		 start_i => SYNTHESIZED_WIRE_14,
		 valid_o => pll_meas_valid,
		 count_pll_o => pll_counter,
		 count_wc_o => wc_counter);


b2v_inst32 : ethernet_receive
GENERIC MAP(lastRamAddress => 1500
			)
PORT MAP(rx_clk => mac_rx_clock,
		 rx_frame => mac_rx_frame,
		 rx_byte_received => mac_rx_byte_received,
		 rx_error => mac_rx_error,
		 rx_data => mac_rx_data,
		 frame_rdy => eth_frame_rdy,
		 ram_addr => SYNTHESIZED_WIRE_13,
		 ram_data => SYNTHESIZED_WIRE_11);


b2v_inst33 : clk_by_x
GENERIC MAP(divider => 2
			)
PORT MAP(clk_in => pll_fs512,
		 clk_out => pll_fs256);


b2v_inst4 : ethernet
GENERIC MAP(MIIM_CLOCK_DIVIDER => 50,
			MIIM_DISABLE => false,
			MIIM_PHY_ADDRESS => "00001",
			MIIM_POLL_WAIT_TICKS => 10000000,
			MIIM_RESET_WAIT_TICKS => 0
			)
PORT MAP(clock_125_i => clk_125MHz,
		 reset_i => mac_reset,
		 mii_tx_clk_i => MII_TX_CLK,
		 mii_rx_clk_i => MII_RX_CLK,
		 mii_rx_er_i => MII_RX_ERR,
		 mii_rx_dv_i => MII_RX_DV,
		 miim_clock_i => clk_125MHz,
		 tx_enable_i => mac_tx_enable,
		 mdio_io => gpio28,
		 mac_address_i => SYNTHESIZED_WIRE_15,
		 mii_rxd_i => PHY_RX_D,
		 speed_override_i => const_eth_speed,
		 tx_data_i => mac_tx_data,
		 mii_tx_er_o => SYNTHESIZED_WIRE_17,
		 mii_tx_en_o => SYNTHESIZED_WIRE_16,
		 mdc_o => gpio27,
		 link_up_o => mac_linkup,
		 tx_clock_o => mac_tx_clock,
		 tx_reset_o => mac_tx_reset,
		 tx_byte_sent_o => mac_tx_byte_sent,
		 tx_busy_o => mac_tx_busy,
		 rx_clock_o => mac_rx_clock,
		 rx_reset_o => mac_rx_reset,
		 rx_frame_o => mac_rx_frame,
		 rx_byte_received_o => mac_rx_byte_received,
		 rx_error_o => mac_rx_error,
		 frame_o => eth_frame_o,
		 mii_txd_o => PHY_TX_D,
		 rx_data_o => mac_rx_data,
		 speed_o => mac_speed);


b2v_inst43 : miirmii
PORT MAP(RefClk => gpio23,
		 m_tx_en => SYNTHESIZED_WIRE_16,
		 m_tx_err => SYNTHESIZED_WIRE_17,
		 Rstn => phy_reset,
		 rmii_crs_dv => rmii_crs,
		 rmii_rx_err => SYNTHESIZED_WIRE_18,
		 m_tx_d => PHY_TX_D(3 DOWNTO 0),
		 rmii_rx_d => GDFX_TEMP_SIGNAL_0,
		 m_rx_en => MII_RX_DV,
		 m_rx_err => MII_RX_ERR,
		 rx_clk => MII_RX_CLK,
		 tx_clk => MII_TX_CLK,
		 rmii_tx_en => gpio18,
		 m_rx_d => PHY_RX_D(3 DOWNTO 0),
		 rmii_tx_d => SYNTHESIZED_WIRE_20);


b2v_inst5 : clk_by_x
GENERIC MAP(divider => 48
			)
PORT MAP(clk_in => fs,
		 clk_out => clk_1kHz);


b2v_inst6 : pll_systemclocks
PORT MAP(inclk0 => clk50m,
		 c0 => clk_125MHz,
		 c1 => clk_250MHz);


b2v_inst7 : const_ch_output_debug
PORT MAP(		 ch0_o => ch0i,
		 ch10_o => ch10i,
		 ch11_o => ch11i,
		 ch12_o => ch12i,
		 ch13_o => ch13i,
		 ch1_o => ch1i,
		 ch2_o => inch2i,
		 ch3_o => inch3i,
		 ch4_o => ch4i,
		 ch5_o => ch5i,
		 ch6_o => ch6i,
		 ch7_o => ch7i,
		 ch8_o => ch8i,
		 ch9_o => ch9i);


b2v_inst8 : fmc_ethernet_client
GENERIC MAP(RX_ADDR_WIDTH => 11,
			TX_ADDR_WIDTH => 11
			)
PORT MAP(clk_sys_i => clk_250MHz,
		 rst_sys_i => SYNTHESIZED_WIRE_19,
		 fmc_ne_n_i => fmc_ne,
		 fmc_noe_n_i => fmc_noe,
		 fmc_nwe_n_i => fmc_nwe,
		 mac_tx_clock_i => mac_tx_clock,
		 mac_tx_reset_i => mac_tx_reset,
		 mac_tx_byte_sent_i => mac_tx_byte_sent,
		 mac_tx_busy_i => mac_tx_busy,
		 mac_rx_clock_i => mac_rx_clock,
		 mac_rx_reset_i => mac_rx_reset,
		 mac_rx_frame_i => mac_rx_frame,
		 mac_rx_byte_rcv_i => mac_rx_byte_received,
		 mac_rx_error_i => mac_rx_error,
		 pll_ppb_measurement_valid_i => pll_meas_valid,
		 wallclock_locked_i => wallclock_locked,
		 wallclock_phasejump_i => wallclock_phasejump,
		 wallclock_configured_i => wallclock_configured,
		 ptp_leader_lost_i => ptp_sync_lost,
		 eth_link_up_i => mac_linkup,
		 tx_allow_i => mcu_allow,
		 clock_count_pll_i => pll_counter,
		 clock_count_wc_i => wc_counter,
		 eth_link_speed_i => mac_speed,
		 fmc_addr_i => fmc_addr,
		 fmc_data_io => fmc_data,
		 leader_offset_i => ptp_offset_from_master,
		 mac_rx_data_i => mac_rx_data,
		 path_delay_i => ptp_mean_path_delay,
		 system_config_rd_data => sys_conf_rd_data,
		 fmc_int_o => fmc_int_o,
		 mac_tx_enable_o => mcu_tx_enable,
		 system_config_wr_en => sys_conf_wr_en,
		 system_config_done_o => sys_conf_wr_done,
		 pll_ppb_measurement_start_o => SYNTHESIZED_WIRE_14,
		 ptp_is_leader_o => ptp_is_leader,
		 ptp_is_follower_o => ptp_is_follower,
		 tx_stream_config_wr_en_o => tx_wr_en,
		 tx_allow_req_o => mcu_allow_req,
		 rx_stream_config_wr_clk_o => rx_conf_wr_clk,
		 rx_stream_config_wr_en_o => rx_conf_wr_en,
		 mac_tx_data_o => mcu_tx_data,
		 rx_stream_config_wr_addr_o => rx_conf_wr_addr,
		 rx_stream_config_wr_data_o => rx_conf_wr_data,
		 system_config_rd_addr => sys_conf_rd_addr,
		 system_config_wr_addr => sys_conf_wr_addr,
		 system_config_wr_data => sys_conf_wr_data,
		 tx_stream_config_wr_addr_o => tx_wr_addr,
		 tx_stream_config_wr_data_o => tx_wr_data);


b2v_inst_i2s_in_1 : i2s_in
GENERIC MAP(fifo_addrbits => 3,
			width => 24
			)
PORT MAP(LR_CLK => fs,
		 BIT_CLK => pll_fs64,
		 DIN => i2s_i1,
		 RESET => powerGood,
		 SYS_CLK => clk_125MHz,
		 DATA_L => ch14i,
		 DATA_R => ch15i);

pll_fs512 <= gpio34;
arduino_io5 <= pll_fs512;
arduino_io(0) <= i2s_o1;
powerGood <= c10_resetn;
clk50m <= c10_clk50m;
rmii_crs <= gpio24;
fmc_ne <= gpio0;
fmc_noe <= gpio1;
fmc_nwe <= gpio2;
i2s_i1 <= gpio31;
gpio17 <= fmc_data(7);
gpio16 <= fmc_data(6);
gpio15 <= fmc_data(5);
gpio14 <= fmc_data(4);
gpio13 <= fmc_data(3);
gpio12 <= fmc_data(2);
gpio11 <= fmc_data(1);
gpio10 <= fmc_data(0);
gpio33 <= pll_fs64;
gpio32 <= fs;
gpio30 <= fmc_int_o;
user_led(3) <= mac_tx_byte_sent_n;

END bdf_type;