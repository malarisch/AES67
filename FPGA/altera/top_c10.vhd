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
-- CREATED		"Sat Apr  4 17:24:55 2026"

LIBRARY ieee;
USE ieee.std_logic_1164.all; 

LIBRARY work;

ENTITY top_c10 IS 
	PORT
	(
		c10_resetn :  IN  STD_LOGIC;
		c10_clk50m :  IN  STD_LOGIC;
		enet_clk_125m :  IN  STD_LOGIC;
		enet_rx_clk :  IN  STD_LOGIC;
		enet_rx_dv :  IN  STD_LOGIC;
		enet_resetn :  IN  STD_LOGIC;
		arduino_io1 :  IN  STD_LOGIC;
		enet_mdio :  INOUT  STD_LOGIC;
		hbus_rwds :  INOUT  STD_LOGIC;
		enet_rx_d :  IN  STD_LOGIC_VECTOR(3 DOWNTO 0);
		gpio1 :  IN  STD_LOGIC;
		gpio0 :  IN  STD_LOGIC;
		gpio2 :  IN  STD_LOGIC;
		gpio33 :  IN  STD_LOGIC;
		gpio28 :  IN  STD_LOGIC;
		gpio26 :  INOUT  STD_LOGIC;
		gpio27 :  INOUT  STD_LOGIC;
		gpio23 :  INOUT  STD_LOGIC;
		gpio24 :  INOUT  STD_LOGIC;
		hbus_dq :  INOUT  STD_LOGIC_VECTOR(7 DOWNTO 0);
		enet_tx_clk :  OUT  STD_LOGIC;
		enet_tx_en :  OUT  STD_LOGIC;
		enet_mdc :  OUT  STD_LOGIC;
		hbus_rstn :  OUT  STD_LOGIC;
		hbus_cs2n :  OUT  STD_LOGIC;
		arduino_io2 :  OUT  STD_LOGIC;
		hbus_clk0_p :  OUT  STD_LOGIC;
		hbus_clk0_n :  OUT  STD_LOGIC;
		enet_tx_d :  OUT  STD_LOGIC_VECTOR(3 DOWNTO 0);
		gpio9 :  OUT  STD_LOGIC;
		gpio13 :  OUT  STD_LOGIC;
		gpio10 :  OUT  STD_LOGIC;
		gpio12 :  OUT  STD_LOGIC;
		gpio11 :  OUT  STD_LOGIC;
		gpio5 :  OUT  STD_LOGIC;
		gpio6 :  OUT  STD_LOGIC;
		gpio34 :  OUT  STD_LOGIC;
		gpio32 :  OUT  STD_LOGIC;
		gpio35 :  OUT  STD_LOGIC;
		gpio25 :  OUT  STD_LOGIC;
		gpio29 :  OUT  STD_LOGIC;
		user_led :  OUT  STD_LOGIC_VECTOR(3 DOWNTO 0)
	);
END top_c10;

ARCHITECTURE bdf_type OF top_c10 IS 

COMPONENT clk_by_x
GENERIC (divider : INTEGER
			);
	PORT(clk_in : IN STD_LOGIC;
		 clk_out : OUT STD_LOGIC
	);
END COMPONENT;

COMPONENT audioclock_generator
	PORT(mclk : IN STD_LOGIC;
		 rst_n : IN STD_LOGIC;
		 clk_256fs : OUT STD_LOGIC;
		 clk_128fs : OUT STD_LOGIC;
		 clk_64fs : OUT STD_LOGIC;
		 fs : OUT STD_LOGIC;
		 bclk_r : OUT STD_LOGIC;
		 bclk_f : OUT STD_LOGIC;
		 fs_pulse : OUT STD_LOGIC
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
		 metering_clear_i : IN STD_LOGIC;
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
		 metering_clip_o : OUT STD_LOGIC_VECTOR(15 DOWNTO 0);
		 metering_signal_o : OUT STD_LOGIC_VECTOR(15 DOWNTO 0);
		 tx_data_o : OUT STD_LOGIC_VECTOR(7 DOWNTO 0)
	);
END COMPONENT;

COMPONENT eth_ram
GENERIC (lastAddress : INTEGER
			);
	PORT(rx_clk : IN STD_LOGIC;
		 sync_in : IN STD_LOGIC;
		 data_in : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
		 readAddrPTP : IN STD_LOGIC_VECTOR(10 DOWNTO 0);
		 readAddrRTP : IN STD_LOGIC_VECTOR(10 DOWNTO 0);
		 readAddrMCU : IN STD_LOGIC_VECTOR(10 DOWNTO 0);
		 
		 writeAddr : IN STD_LOGIC_VECTOR(10 DOWNTO 0);
		 sync_out : OUT STD_LOGIC;
		 dataOut_sysclk_rtp : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
		 dataOut_sysclk_ptp : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
		 dataOut_rxclk : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
		 
		 is_mcu_pkt_o	: out std_logic;
		is_ptp_pkt_o	: out std_logic;
		is_rtp_pkt_o	: out std_logic;
		is_mcu_pkt_tog_o	: out std_logic;
		--is_ptp_pkt_tog_o	: out std_logic;
		--is_rtp_pkt_tog_o	: out std_logic;

		 sys_clk_i : IN STD_LOGIC
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
		 rx_byte_count : OUT STD_LOGIC_VECTOR(10 DOWNTO 0);

		is_ptp_pkt_tog_o	: out std_logic;
		is_rtp_pkt_tog_o	: out std_logic;
		is_mcu_pkt_tog_o	: out std_logic
	);
END COMPONENT;

COMPONENT ethernet_reset
	PORT(clk : IN STD_LOGIC;
		 enet_clk : IN STD_LOGIC;
		 power_good : IN STD_LOGIC;
		 phy_rstn : OUT STD_LOGIC;
		 mac_rst : OUT STD_LOGIC;
		 sendArpRequest : OUT STD_LOGIC;
		 tx_online : OUT STD_LOGIC
	);
END COMPONENT;

COMPONENT litex_soc
	PORT(aes67_ctrl_eth_link_up : IN STD_LOGIC;
		 aes67_ctrl_eth_rx_overflow : IN STD_LOGIC;
		 aes67_ctrl_eth_tx_done : IN STD_LOGIC;
		 aes67_ctrl_pll_ppb_valid : IN STD_LOGIC;
		 aes67_ctrl_ptp_sync_lost : IN STD_LOGIC;
		 aes67_ctrl_wallclock_configured : IN STD_LOGIC;
		 aes67_ctrl_wallclock_locked : IN STD_LOGIC;
		 aes67_ctrl_wallclock_phasejump : IN STD_LOGIC;
		 clk50 : IN STD_LOGIC;
		 clk_mac_rx : IN STD_LOGIC;
		 clk_mac_tx : IN STD_LOGIC;
		 eth_buf_rx_valid : IN STD_LOGIC;
		 eth_buf_rx_we : IN STD_LOGIC;
		 serial1_rx : IN STD_LOGIC;
		 serial_rx : IN STD_LOGIC;
		 spi_miso : IN STD_LOGIC;
		 spiflash_miso : IN STD_LOGIC;
		 hyperram_rwds : INOUT STD_LOGIC;
		 i2c0_scl : INOUT STD_LOGIC;
		 i2c0_sda : INOUT STD_LOGIC;
		 i2c1_scl : INOUT STD_LOGIC;
		 i2c1_sda : INOUT STD_LOGIC;
		 aes67_ctrl_eth_speed : IN STD_LOGIC_VECTOR(1 DOWNTO 0);
		 aes67_ctrl_pll_ppb_pll_count : IN STD_LOGIC_VECTOR(21 DOWNTO 0);
		 aes67_ctrl_pll_ppb_wc_count : IN STD_LOGIC_VECTOR(21 DOWNTO 0);
		 aes67_ctrl_ptp_offset : IN STD_LOGIC_VECTOR(31 DOWNTO 0);
		 aes67_ctrl_ptp_path_delay : IN STD_LOGIC_VECTOR(31 DOWNTO 0);
		 aes67_ctrl_rx_meter_clip : IN STD_LOGIC_VECTOR(15 DOWNTO 0);
		 aes67_ctrl_rx_meter_signal : IN STD_LOGIC_VECTOR(15 DOWNTO 0);
		 aes67_ctrl_tx_meter_clip : IN STD_LOGIC_VECTOR(15 DOWNTO 0);
		 aes67_ctrl_tx_meter_signal : IN STD_LOGIC_VECTOR(15 DOWNTO 0);
		 eth_buf_rx_addr : IN STD_LOGIC_VECTOR(10 DOWNTO 0);
		 eth_buf_rx_data : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
		 eth_buf_rx_len : IN STD_LOGIC_VECTOR(10 DOWNTO 0);
		 eth_buf_tx_addr : IN STD_LOGIC_VECTOR(10 DOWNTO 0);
		 hyperram_dq : INOUT STD_LOGIC_VECTOR(7 DOWNTO 0);
		 aes67_ctrl_adda_nrst : OUT STD_LOGIC;
		 aes67_ctrl_eth_tx_request : OUT STD_LOGIC;
		 aes67_ctrl_meter_clear : OUT STD_LOGIC;
		 aes67_ctrl_pll_ppb_start : OUT STD_LOGIC;
		 aes67_ctrl_ptp_is_follower : OUT STD_LOGIC;
		 aes67_ctrl_ptp_is_leader : OUT STD_LOGIC;
		 eth_buf_rx_ack : OUT STD_LOGIC;
		 hyperram_clk : OUT STD_LOGIC;
		 hyperram_cs_n : OUT STD_LOGIC;
		 hyperram_rst_n : OUT STD_LOGIC;
		 rx_stream_cfg_wr_en : OUT STD_LOGIC;
		 serial1_tx : OUT STD_LOGIC;
		 serial_tx : OUT STD_LOGIC;
		 spi_clk : OUT STD_LOGIC;
		 spi_cs_n : OUT STD_LOGIC;
		 spi_mosi : OUT STD_LOGIC;
		 spiflash_clk : OUT STD_LOGIC;
		 spiflash_cs_n : OUT STD_LOGIC;
		 spiflash_mosi : OUT STD_LOGIC;
		 sys_clk_out : OUT STD_LOGIC;
		 tx_stream_cfg_wr_en : OUT STD_LOGIC;
		 aes67_ctrl_ip_addr : OUT STD_LOGIC_VECTOR(31 DOWNTO 0);
		 aes67_ctrl_mac_addr : OUT STD_LOGIC_VECTOR(47 DOWNTO 0);
		 aes67_ctrl_ptp_announce_msg_interval : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
		 aes67_ctrl_ptp_gm_clock_accuracy : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
		 aes67_ctrl_ptp_gm_clock_class : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
		 aes67_ctrl_ptp_gm_priority1 : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
		 aes67_ctrl_ptp_gm_priority2 : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
		 aes67_ctrl_ptp_leader_id : OUT STD_LOGIC_VECTOR(63 DOWNTO 0);
		 aes67_ctrl_ptp_log_msg_interval : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
		 aes67_ctrl_ptp_time_source : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
		 eth_buf_tx_data : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
		 eth_buf_tx_len : OUT STD_LOGIC_VECTOR(10 DOWNTO 0);
		 rx_stream_cfg_wr_addr : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
		 rx_stream_cfg_wr_data : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
		 tx_stream_cfg_wr_addr : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
		 tx_stream_cfg_wr_data : OUT STD_LOGIC_VECTOR(7 DOWNTO 0)
	);
END COMPONENT;

COMPONENT litex_eth_buffer_bridge
	PORT(buf_rx_ack_i : IN STD_LOGIC;
		 eth_tx_request_i : IN STD_LOGIC;
		 mac_tx_clock_i : IN STD_LOGIC;
		 mac_tx_reset_i : IN STD_LOGIC;
		 mac_tx_byte_sent_i : IN STD_LOGIC;
		 mac_tx_busy_i : IN STD_LOGIC;
		 tx_allow_i : IN STD_LOGIC;
		 mac_rx_clock_i : IN STD_LOGIC;
		 mac_rx_reset_i : IN STD_LOGIC;
		 parse_mcu_packet_tog_i : IN STD_LOGIC;
		 mcu_clk_i : IN STD_LOGIC;
		 buf_tx_dat_i : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
		 buf_tx_len_i : IN STD_LOGIC_VECTOR(10 DOWNTO 0);
		 eth_ram_data_i : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
		 pkt_len_i : IN STD_LOGIC_VECTOR(10 DOWNTO 0);
		 buf_rx_we_o : OUT STD_LOGIC;
		 buf_rx_valid_o : OUT STD_LOGIC;
		 eth_tx_done_o : OUT STD_LOGIC;
		 eth_rx_overflow_o : OUT STD_LOGIC;
		 mac_tx_enable_o : OUT STD_LOGIC;
		 tx_allow_req_o : OUT STD_LOGIC;
		 buf_rx_addr_o : OUT STD_LOGIC_VECTOR(10 DOWNTO 0);
		 buf_rx_data_o : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
		 buf_rx_len_o : OUT STD_LOGIC_VECTOR(10 DOWNTO 0);
		 buf_tx_addr_o : OUT STD_LOGIC_VECTOR(10 DOWNTO 0);
		 eth_ram_addr_o : OUT STD_LOGIC_VECTOR(10 DOWNTO 0);
		 mac_tx_dat_o : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
		 sys_clk_i : IN STD_LOGIC;
        packet_length_valid_i : in STD_ULOGIC

	);
END COMPONENT;

COMPONENT ethernet_packet_aggregator
	PORT(tx_en0_i : IN STD_LOGIC;
		 tx_en1_i : IN STD_LOGIC;
		 tx_en2_i : IN STD_LOGIC;
		 data0_i : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
		 data1_i : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
		 data2_i : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
		 tx_en_o : OUT STD_LOGIC;
		 data_o : OUT STD_LOGIC_VECTOR(7 DOWNTO 0)
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

COMPONENT ptp_module
	PORT(sys_clk : IN STD_LOGIC;
		 rst_n : IN STD_LOGIC;
		 mac_tx_clock : IN STD_LOGIC;
		 mac_tx_busy : IN STD_LOGIC;
		 mac_tx_byte_sent : IN STD_LOGIC;
		 mac_tx_allow_i : IN STD_LOGIC;
		 parse_ptp_packet_tog : IN STD_LOGIC;
		 ptp_is_follower : IN STD_LOGIC;
		 ptp_is_leader : IN STD_LOGIC;
		 sof_recv_tog_i : IN STD_LOGIC; -- sof rx delim
		 ethernet_parser_sync : IN STD_LOGIC;
		 sof_sent_tog_i : IN STD_LOGIC; -- sof tx delim
		 ip_address : IN STD_LOGIC_VECTOR(31 DOWNTO 0);
		 mac_address : IN STD_LOGIC_VECTOR(47 DOWNTO 0);
		 mac_ram_data : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
		 ptp_announce_interval : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
		 ptp_clock_accuracy : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
		 ptp_clock_class : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
		 ptp_clock_priorityone : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
		 ptp_clock_prioritytwo : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
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
		 pin_name1 : OUT STD_LOGIC;
		 media_clock : OUT STD_LOGIC_VECTOR(31 DOWNTO 0);
		 ptp_mean_path_delay : OUT STD_LOGIC_VECTOR(31 DOWNTO 0);
		 ptp_offset_from_master : OUT STD_LOGIC_VECTOR(31 DOWNTO 0);
		 ptp_ram_addr : OUT STD_LOGIC_VECTOR(10 DOWNTO 0);
		 tx_data_ptpfu : OUT STD_LOGIC_VECTOR(7 DOWNTO 0)
	);
END COMPONENT;

COMPONENT reverse_mac
	PORT(mac_address_i : IN STD_LOGIC_VECTOR(47 DOWNTO 0);
		 mac_address_o : OUT STD_LOGIC_VECTOR(47 DOWNTO 0)
	);
END COMPONENT;

COMPONENT rgmii_phy_if
GENERIC (CLOCK_INPUT_STYLE : STRING;
			IODDR_STYLE : STRING;
			TARGET : STRING;
			USE_CLK90 : STRING
			);
	PORT(clk : IN STD_LOGIC;
		 clk90 : IN STD_LOGIC;
		 rst : IN STD_LOGIC;
		 mac_gmii_tx_en : IN STD_LOGIC;
		 mac_gmii_tx_er : IN STD_LOGIC;
		 phy_rgmii_rx_clk : IN STD_LOGIC;
		 phy_rgmii_rx_ctl : IN STD_LOGIC;
		 mac_gmii_txd : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
		 phy_rgmii_rxd : IN STD_LOGIC_VECTOR(3 DOWNTO 0);
		 speed : IN STD_LOGIC_VECTOR(1 DOWNTO 0);
		 mac_gmii_rx_clk : OUT STD_LOGIC;
		 mac_gmii_rx_rst : OUT STD_LOGIC;
		 mac_gmii_rx_dv : OUT STD_LOGIC;
		 mac_gmii_rx_er : OUT STD_LOGIC;
		 mac_gmii_tx_clk : OUT STD_LOGIC;
		 mac_gmii_tx_rst : OUT STD_LOGIC;
		 mac_gmii_tx_clk_en : OUT STD_LOGIC;
		 phy_rgmii_tx_clk : OUT STD_LOGIC;
		 phy_rgmii_tx_ctl : OUT STD_LOGIC;
		 mac_gmii_rxd : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
		 phy_rgmii_txd : OUT STD_LOGIC_VECTOR(3 DOWNTO 0)
	);
END COMPONENT;

COMPONENT ethernet_clks
	PORT(inclk0 : IN STD_LOGIC;
		 c0 : OUT STD_LOGIC;
		 c1 : OUT STD_LOGIC;
		 locked : OUT STD_LOGIC
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
		 metering_clear_i : IN STD_LOGIC;
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
		 eth_read_addr_o : OUT STD_LOGIC_VECTOR(10 DOWNTO 0);
		 metering_clip_o : OUT STD_LOGIC_VECTOR(15 DOWNTO 0);
		 metering_signal_o : OUT STD_LOGIC_VECTOR(15 DOWNTO 0)
	);
END COMPONENT;

COMPONENT pll_systemclocks
	PORT(inclk0 : IN STD_LOGIC;
		 c0 : OUT STD_LOGIC;
		 c1 : OUT STD_LOGIC;
		 c3 : OUT STD_LOGIC
	);
END COMPONENT;

COMPONENT tdm8_out
GENERIC (width : INTEGER
			);
	PORT(FSYNC : IN STD_LOGIC;
		 BIT_CLK : IN STD_LOGIC;
		 RESET : IN STD_LOGIC;
		 DATA_CH0 : IN STD_LOGIC_VECTOR(23 DOWNTO 0);
		 DATA_CH1 : IN STD_LOGIC_VECTOR(23 DOWNTO 0);
		 DATA_CH2 : IN STD_LOGIC_VECTOR(23 DOWNTO 0);
		 DATA_CH3 : IN STD_LOGIC_VECTOR(23 DOWNTO 0);
		 DATA_CH4 : IN STD_LOGIC_VECTOR(23 DOWNTO 0);
		 DATA_CH5 : IN STD_LOGIC_VECTOR(23 DOWNTO 0);
		 DATA_CH6 : IN STD_LOGIC_VECTOR(23 DOWNTO 0);
		 DATA_CH7 : IN STD_LOGIC_VECTOR(23 DOWNTO 0);
		 DOUT : OUT STD_LOGIC;
		 LRCK : OUT STD_LOGIC
	);
END COMPONENT;

COMPONENT tdm8_in
GENERIC (width : INTEGER
			);
	PORT(FSYNC : IN STD_LOGIC;
		 BIT_CLK : IN STD_LOGIC;
		 DIN : IN STD_LOGIC;
		 RESET : IN STD_LOGIC;
		 SYS_CLK : IN STD_LOGIC;
		 DATA_RDY : OUT STD_LOGIC;
		 DATA_CH0 : OUT STD_LOGIC_VECTOR(23 DOWNTO 0);
		 DATA_CH1 : OUT STD_LOGIC_VECTOR(23 DOWNTO 0);
		 DATA_CH2 : OUT STD_LOGIC_VECTOR(23 DOWNTO 0);
		 DATA_CH3 : OUT STD_LOGIC_VECTOR(23 DOWNTO 0);
		 DATA_CH4 : OUT STD_LOGIC_VECTOR(23 DOWNTO 0);
		 DATA_CH5 : OUT STD_LOGIC_VECTOR(23 DOWNTO 0);
		 DATA_CH6 : OUT STD_LOGIC_VECTOR(23 DOWNTO 0);
		 DATA_CH7 : OUT STD_LOGIC_VECTOR(23 DOWNTO 0)
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
		 miim_clock_i : IN STD_LOGIC;
		 tx_enable_i : IN STD_LOGIC;
		 mdio_io : INOUT STD_LOGIC;
		 mac_address_i : IN STD_LOGIC_VECTOR(47 DOWNTO 0);
		 mii_rxd_i : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
		 tx_data_i : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
		 reset_o : OUT STD_LOGIC;
		 mii_tx_er_o : OUT STD_LOGIC;
		 mii_tx_en_o : OUT STD_LOGIC;
		 gmii_gtx_clk_o : OUT STD_LOGIC;
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
	     tx_sof_delim_o		   : out std_ulogic;
		 rx_sof_delim_o		   : out std_ulogic;
		 tx_sof_delim_tog_o		   : out std_ulogic;
		 rx_sof_delim_tog_o		   : out std_ulogic;
		 mii_txd_o : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
		 rx_data_o : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
		 speed_o : OUT STD_LOGIC_VECTOR(1 DOWNTO 0)
	);
END COMPONENT;

SIGNAL	adda_nRST :  STD_LOGIC;
SIGNAL	audio_allow :  STD_LOGIC;
SIGNAL	audio_allow_req :  STD_LOGIC;
SIGNAL	audio_meter_clear :  STD_LOGIC;
SIGNAL	audio_meter_rx_clip :  STD_LOGIC_VECTOR(15 DOWNTO 0);
SIGNAL	audio_meter_rx_signal :  STD_LOGIC_VECTOR(15 DOWNTO 0);
SIGNAL	audio_meter_tx_clip :  STD_LOGIC_VECTOR(15 DOWNTO 0);
SIGNAL	audio_meter_tx_signal :  STD_LOGIC_VECTOR(15 DOWNTO 0);
SIGNAL	bclk_f :  STD_LOGIC;
SIGNAL	bclk_r :  STD_LOGIC;
SIGNAL	buf_rx_a :  STD_LOGIC_VECTOR(10 DOWNTO 0);
SIGNAL	buf_rx_ack :  STD_LOGIC;
SIGNAL	buf_rx_d :  STD_LOGIC_VECTOR(7 DOWNTO 0);
SIGNAL	buf_rx_len :  STD_LOGIC_VECTOR(10 DOWNTO 0);
SIGNAL	buf_rx_valid :  STD_LOGIC;
SIGNAL	buf_rx_we :  STD_LOGIC;
SIGNAL	buf_tx_a :  STD_LOGIC_VECTOR(10 DOWNTO 0);
SIGNAL	buf_tx_dat :  STD_LOGIC_VECTOR(7 DOWNTO 0);
SIGNAL	buf_tx_len :  STD_LOGIC_VECTOR(10 DOWNTO 0);
SIGNAL	ch0i :  STD_LOGIC_VECTOR(23 DOWNTO 0);
SIGNAL	ch10i :  STD_LOGIC_VECTOR(23 DOWNTO 0);
SIGNAL	ch11i :  STD_LOGIC_VECTOR(23 DOWNTO 0);
SIGNAL	ch12i :  STD_LOGIC_VECTOR(23 DOWNTO 0);
SIGNAL	ch13i :  STD_LOGIC_VECTOR(23 DOWNTO 0);
SIGNAL	ch14i :  STD_LOGIC_VECTOR(23 DOWNTO 0);
SIGNAL	ch15i :  STD_LOGIC_VECTOR(23 DOWNTO 0);
SIGNAL	ch1i :  STD_LOGIC_VECTOR(23 DOWNTO 0);
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
SIGNAL	enet_clk :  STD_LOGIC;
SIGNAL	enet_clk_90 :  STD_LOGIC;
SIGNAL	eth_byte_cnt :  STD_LOGIC_VECTOR(10 DOWNTO 0);
SIGNAL	eth_sof_sent_tog :  STD_LOGIC;
SIGNAL	eth_sof_recv_tog :  STD_LOGIC;
SIGNAL	eth_frame_rdy :  STD_LOGIC;
SIGNAL	eth_ip_type :  STD_LOGIC_VECTOR(7 DOWNTO 0);
SIGNAL	eth_pkt_type :  STD_LOGIC_VECTOR(15 DOWNTO 0);
SIGNAL	eth_ram_wr_addr :  STD_LOGIC_VECTOR(10 DOWNTO 0);
SIGNAL	eth_ram_wr_data :  STD_LOGIC_VECTOR(7 DOWNTO 0);
SIGNAL	eth_udp_dst_prt :  STD_LOGIC_VECTOR(15 DOWNTO 0);
SIGNAL	eth_udp_length :  STD_LOGIC_VECTOR(15 DOWNTO 0);
SIGNAL	eth_udp_src_prt :  STD_LOGIC_VECTOR(15 DOWNTO 0);
SIGNAL	ethernet_parser_sync :  STD_LOGIC;
SIGNAL	fs :  STD_LOGIC;
SIGNAL	fs_tdm :  STD_LOGIC;
SIGNAL	gmii_rx_clk :  STD_LOGIC;
SIGNAL	gmii_rx_dv :  STD_LOGIC;
SIGNAL	gmii_rx_err :  STD_LOGIC;
SIGNAL	gmii_rx_rst :  STD_LOGIC;
SIGNAL	gmii_rxd :  STD_LOGIC_VECTOR(7 DOWNTO 0);
SIGNAL	gmii_tx_clk :  STD_LOGIC;
SIGNAL	gmii_tx_clk_en :  STD_LOGIC;
SIGNAL	gmii_tx_en :  STD_LOGIC;
SIGNAL	gmii_tx_err :  STD_LOGIC;
SIGNAL	gmii_tx_rst :  STD_LOGIC;
SIGNAL	gmii_txd :  STD_LOGIC_VECTOR(7 DOWNTO 0);
SIGNAL	hram_clk :  STD_LOGIC;
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
SIGNAL	mcu_clk :  STD_LOGIC;
SIGNAL	mcu_ram_data :  STD_LOGIC_VECTOR(7 DOWNTO 0);
SIGNAL	mcu_read_addr :  STD_LOGIC_VECTOR(10 DOWNTO 0);
SIGNAL	mcu_rx_overflow :  STD_LOGIC;
SIGNAL	mcu_tx_data :  STD_LOGIC_VECTOR(7 DOWNTO 0);
SIGNAL	mcu_tx_done :  STD_LOGIC;
SIGNAL	mcu_tx_enable :  STD_LOGIC;
SIGNAL	mcu_tx_req_i :  STD_LOGIC;
SIGNAL	media_clock :  STD_LOGIC_VECTOR(31 DOWNTO 0);
SIGNAL	nhram_clk :  STD_LOGIC;  -- inverted hyperram clock for differential pair
SIGNAL	parse_mcu_packet_tog :  STD_LOGIC;
SIGNAL	parse_ptp_packet_tog :  STD_LOGIC;
SIGNAL	parse_rtp_packet :  STD_LOGIC;
SIGNAL	parser_ram_addr :  STD_LOGIC_VECTOR(10 DOWNTO 0);
SIGNAL	parser_ram_data :  STD_LOGIC_VECTOR(7 DOWNTO 0);
SIGNAL	phy_reset :  STD_LOGIC;
SIGNAL	pll_counter :  STD_LOGIC_VECTOR(21 DOWNTO 0);
SIGNAL	pll_fs128 :  STD_LOGIC;
SIGNAL	pll_fs256 :  STD_LOGIC;
SIGNAL	pll_fs512 :  STD_LOGIC;
SIGNAL	pll_fs64 :  STD_LOGIC;
SIGNAL	pll_meas_valid :  STD_LOGIC;
SIGNAL	powerGood :  STD_LOGIC;
SIGNAL	ppb_meter_start :  STD_LOGIC;
SIGNAL	ptp_allow :  STD_LOGIC;
SIGNAL	ptp_allow_req :  STD_LOGIC;
SIGNAL	ptp_announce_interval :  STD_LOGIC_VECTOR(7 DOWNTO 0);
SIGNAL	ptp_clock_accuracy :  STD_LOGIC_VECTOR(7 DOWNTO 0);
SIGNAL	ptp_clock_class :  STD_LOGIC_VECTOR(7 DOWNTO 0);
SIGNAL	ptp_current_leader_id :  STD_LOGIC_VECTOR(63 DOWNTO 0);
SIGNAL	ptp_gm_prioone :  STD_LOGIC_VECTOR(7 DOWNTO 0);
SIGNAL	ptp_gm_priotwo :  STD_LOGIC_VECTOR(7 DOWNTO 0);
SIGNAL	ptp_is_follower :  STD_LOGIC;
SIGNAL	ptp_is_leader :  STD_LOGIC;
SIGNAL	ptp_log_message_interval :  STD_LOGIC_VECTOR(7 DOWNTO 0);
SIGNAL	ptp_mean_path_delay :  STD_LOGIC_VECTOR(31 DOWNTO 0);
SIGNAL	ptp_offset_from_master :  STD_LOGIC_VECTOR(31 DOWNTO 0);
SIGNAL	ptp_ram_addr :  STD_LOGIC_VECTOR(10 DOWNTO 0);

SIGNAL	eth_ram_data_sys_rtp :  STD_LOGIC_VECTOR(7 DOWNTO 0);
SIGNAL	eth_ram_data_sys_ptp :  STD_LOGIC_VECTOR(7 DOWNTO 0);
SIGNAL	eth_ram_data_rx :  STD_LOGIC_VECTOR(7 DOWNTO 0);
SIGNAL	ptp_sync_lost :  STD_LOGIC;
SIGNAL	ptp_time_source :  STD_LOGIC_VECTOR(7 DOWNTO 0);
SIGNAL	rev_mac_address :  STD_LOGIC_VECTOR(47 DOWNTO 0);
SIGNAL	rtp_ram_addr :  STD_LOGIC_VECTOR(10 DOWNTO 0);
SIGNAL	rtp_ram_data :  STD_LOGIC_VECTOR(7 DOWNTO 0);
SIGNAL	rx_conf_wr_addr :  STD_LOGIC_VECTOR(7 DOWNTO 0);
SIGNAL	rx_conf_wr_data :  STD_LOGIC_VECTOR(7 DOWNTO 0);
SIGNAL	rx_conf_wr_en :  STD_LOGIC;
SIGNAL	second_pulse_sys :  STD_LOGIC;
SIGNAL	sof_sent :  STD_LOGIC;
SIGNAL	spi0_clk :  STD_LOGIC;
SIGNAL	spi0_cs :  STD_LOGIC;
SIGNAL	spi0_miso :  STD_LOGIC;
SIGNAL	spi0_mosi :  STD_LOGIC;
SIGNAL	spiflash_clk :  STD_LOGIC;
SIGNAL	spiflash_cs :  STD_LOGIC;
SIGNAL	spiflash_miso :  STD_LOGIC;
SIGNAL	spiflash_mosi :  STD_LOGIC;
SIGNAL	tx_data_audiotx :  STD_LOGIC_VECTOR(7 DOWNTO 0);
SIGNAL	tx_data_ptpfu :  STD_LOGIC_VECTOR(7 DOWNTO 0);
SIGNAL	tx_en_audiotx :  STD_LOGIC;
SIGNAL	tx_en_ptpfu :  STD_LOGIC;
SIGNAL	tx_wr_addr :  STD_LOGIC_VECTOR(7 DOWNTO 0);
SIGNAL	tx_wr_data :  STD_LOGIC_VECTOR(7 DOWNTO 0);
SIGNAL	tx_wr_en :  STD_LOGIC;
SIGNAL	uart0_rx :  STD_LOGIC;
SIGNAL	uart0_tx :  STD_LOGIC;
SIGNAL	uart1_rx :  STD_LOGIC;
SIGNAL	uart1_tx :  STD_LOGIC;
SIGNAL	wallclock_configured :  STD_LOGIC;
SIGNAL	wallclock_locked :  STD_LOGIC;
SIGNAL	wallclock_phasejump :  STD_LOGIC;
SIGNAL	wc_64fs :  STD_LOGIC;
SIGNAL	wc_counter :  STD_LOGIC_VECTOR(21 DOWNTO 0);
SIGNAL	wc_fs :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_0 :  STD_LOGIC_VECTOR(23 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_1 :  STD_LOGIC_VECTOR(23 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_2 :  STD_LOGIC_VECTOR(23 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_3 :  STD_LOGIC_VECTOR(23 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_4 :  STD_LOGIC_VECTOR(23 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_5 :  STD_LOGIC_VECTOR(23 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_6 :  STD_LOGIC_VECTOR(23 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_7 :  STD_LOGIC_VECTOR(23 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_8 :  STD_LOGIC_VECTOR(23 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_9 :  STD_LOGIC_VECTOR(23 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_10 :  STD_LOGIC_VECTOR(23 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_11 :  STD_LOGIC_VECTOR(23 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_12 :  STD_LOGIC_VECTOR(23 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_13 :  STD_LOGIC_VECTOR(23 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_14 :  STD_LOGIC_VECTOR(23 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_15 :  STD_LOGIC_VECTOR(23 DOWNTO 0);


BEGIN 



b2v_1khzclkdiv : clk_by_x
GENERIC MAP(divider => 48
			)
PORT MAP(clk_in => fs,
		 clk_out => clk_1kHz);


b2v_audioclocks : audioclock_generator
PORT MAP(mclk => pll_fs512,
		 rst_n => powerGood,
		 clk_64fs => pll_fs64,
		 fs => fs,
		 bclk_r => bclk_r,
		 bclk_f => bclk_f,
		 fs_pulse => fs_tdm);


b2v_audiotx : audio_tx_module
PORT MAP(sys_clk => clk_125MHz,
		 fmc_clk => mcu_clk,
		 rst_n => powerGood,
		 fs_clk_i => fs,
		 cfg_wr_en_i => tx_wr_en,
		 mac_tx_clock => mac_tx_clock,
		 mac_tx_busy => mac_tx_busy,
		 mac_tx_byte_sent => mac_tx_byte_sent,
		 mac_audio_allow_i => audio_allow_req,
		 metering_clear_i => audio_meter_clear,
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
		 metering_clip_o => audio_meter_tx_clip,
		 metering_signal_o => audio_meter_tx_signal,
		 tx_data_o => tx_data_audiotx);


b2v_eth_buf : eth_ram
GENERIC MAP(lastAddress => 1500
			)
PORT MAP(rx_clk => mac_rx_clock,
		 sync_in => eth_frame_rdy,
		 data_in => eth_ram_wr_data,
		 readAddrPTP => ptp_ram_addr,
		 readAddrRTP => rtp_ram_addr,
		 readAddrMCU => mcu_read_addr,
		 writeAddr => eth_ram_wr_addr,
		 sync_out => ethernet_parser_sync,
		 dataOut_sysclk_rtp => eth_ram_data_sys_rtp,
		 dataOut_sysclk_ptp => eth_ram_data_sys_ptp,
		 
		 dataOut_rxclk => eth_ram_data_rx,
		 
		--is_mcu_pkt_tog_o => parse_mcu_packet_tog,
		--is_ptp_pkt_tog_o => parse_ptp_packet_tog,
		--is_rtp_pkt_tog_o => parse_rtp_packet,
				 sys_clk_i => clk_125MHz
);





b2v_eth_rx : ethernet_receive
GENERIC MAP(lastRamAddress => 1500
			)
PORT MAP(rx_clk => mac_rx_clock,
		 rx_frame => mac_rx_frame,
		 rx_byte_received => mac_rx_byte_received,
		 rx_error => mac_rx_error,
		 rx_data => mac_rx_data,
		 frame_rdy => eth_frame_rdy,
		 ram_addr => eth_ram_wr_addr,
		 ram_data => eth_ram_wr_data,
		 rx_byte_count => eth_byte_cnt,
		is_mcu_pkt_tog_o => parse_mcu_packet_tog,
		is_ptp_pkt_tog_o => parse_ptp_packet_tog,
		is_rtp_pkt_tog_o => parse_rtp_packet);


b2v_ethrst : ethernet_reset
PORT MAP(clk => clk_1kHz,
		 enet_clk => enet_clk,
		 power_good => powerGood,
		 mac_rst => mac_reset);


nhram_clk <= NOT(hram_clk);



mac_tx_byte_sent_n <= NOT(mac_tx_byte_sent);




b2v_litex : litex_soc
PORT MAP(aes67_ctrl_eth_link_up => mac_linkup,
		 aes67_ctrl_eth_rx_overflow => mcu_rx_overflow,
		 aes67_ctrl_eth_tx_done => mcu_tx_done,
		 aes67_ctrl_pll_ppb_valid => pll_meas_valid,
		 aes67_ctrl_ptp_sync_lost => ptp_sync_lost,
		 aes67_ctrl_wallclock_configured => wallclock_configured,
		 aes67_ctrl_wallclock_locked => wallclock_locked,
		 aes67_ctrl_wallclock_phasejump => wallclock_phasejump,
		 clk50 => clk50m,
		 clk_mac_rx => mac_rx_clock,
		 clk_mac_tx => mac_tx_clock,
		 eth_buf_rx_valid => buf_rx_valid,
		 eth_buf_rx_we => buf_rx_we,
		 serial1_rx => uart1_rx,
		 serial_rx => uart0_rx,
		 spi_miso => spi0_miso,
		 spiflash_miso => spiflash_miso,
		 hyperram_rwds => hbus_rwds,
		 i2c0_scl => gpio26,
		 i2c0_sda => gpio27,
		 i2c1_scl => gpio23,
		 i2c1_sda => gpio24,
		 aes67_ctrl_eth_speed => mac_speed,
		 aes67_ctrl_pll_ppb_pll_count => pll_counter,
		 aes67_ctrl_pll_ppb_wc_count => wc_counter,
		 aes67_ctrl_ptp_offset => ptp_offset_from_master,
		 aes67_ctrl_ptp_path_delay => ptp_mean_path_delay,
		 aes67_ctrl_rx_meter_clip => audio_meter_rx_clip,
		 aes67_ctrl_rx_meter_signal => audio_meter_rx_signal,
		 aes67_ctrl_tx_meter_clip => audio_meter_tx_clip,
		 aes67_ctrl_tx_meter_signal => audio_meter_tx_signal,
		 eth_buf_rx_addr => buf_rx_a,
		 eth_buf_rx_data => buf_rx_d,
		 eth_buf_rx_len => buf_rx_len,
		 eth_buf_tx_addr => buf_tx_a,
		 hyperram_dq => hbus_dq,
		 aes67_ctrl_adda_nrst => adda_nRST,
		 aes67_ctrl_eth_tx_request => mcu_tx_req_i,
		 aes67_ctrl_meter_clear => audio_meter_clear,
		 aes67_ctrl_pll_ppb_start => ppb_meter_start,
		 aes67_ctrl_ptp_is_follower => ptp_is_follower,
		 aes67_ctrl_ptp_is_leader => ptp_is_leader,
		 eth_buf_rx_ack => buf_rx_ack,
		 hyperram_clk => hram_clk,
		 hyperram_cs_n => hbus_cs2n,
		 hyperram_rst_n => hbus_rstn,
		 rx_stream_cfg_wr_en => rx_conf_wr_en,
		 serial1_tx => uart1_tx,
		 serial_tx => uart0_tx,
		 spiflash_clk => spiflash_clk,
		 spiflash_cs_n => spiflash_cs,
		 spiflash_mosi => spiflash_mosi,
		 sys_clk_out => mcu_clk,
		 tx_stream_cfg_wr_en => tx_wr_en,
		 aes67_ctrl_ip_addr => ip_address,
		 aes67_ctrl_mac_addr => mac_address,
		 aes67_ctrl_ptp_announce_msg_interval => ptp_announce_interval,
		 aes67_ctrl_ptp_gm_clock_accuracy => ptp_clock_accuracy,
		 aes67_ctrl_ptp_gm_clock_class => ptp_clock_class,
		 aes67_ctrl_ptp_gm_priority1 => ptp_gm_prioone,
		 aes67_ctrl_ptp_gm_priority2 => ptp_gm_priotwo,
		 aes67_ctrl_ptp_leader_id => ptp_current_leader_id,
		 aes67_ctrl_ptp_log_msg_interval => ptp_log_message_interval,
		 aes67_ctrl_ptp_time_source => ptp_time_source,
		 eth_buf_tx_data => buf_tx_dat,
		 eth_buf_tx_len => buf_tx_len,
		 rx_stream_cfg_wr_addr => rx_conf_wr_addr,
		 rx_stream_cfg_wr_data => rx_conf_wr_data,
		 tx_stream_cfg_wr_addr => tx_wr_addr,
		 tx_stream_cfg_wr_data => tx_wr_data);


b2v_litex_eth : litex_eth_buffer_bridge
PORT MAP(buf_rx_ack_i => buf_rx_ack,
		 eth_tx_request_i => mcu_tx_req_i,
		 mac_tx_clock_i => mac_tx_clock,
		 mac_tx_reset_i => mac_reset,
		 mac_tx_byte_sent_i => mac_tx_byte_sent,
		 mac_tx_busy_i => mac_tx_busy,
		 tx_allow_i => mcu_allow,
		 mac_rx_clock_i => mac_rx_clock,
		 mac_rx_reset_i => mac_rx_reset,
		 parse_mcu_packet_tog_i => parse_mcu_packet_tog,
		 mcu_clk_i => mcu_clk,
		 buf_tx_dat_i => buf_tx_dat,
		 buf_tx_len_i => buf_tx_len,
		 eth_ram_data_i => eth_ram_data_rx,
		 pkt_len_i => eth_byte_cnt,
		 buf_rx_we_o => buf_rx_we,
		 buf_rx_valid_o => buf_rx_valid,
		 eth_tx_done_o => mcu_tx_done,
		 eth_rx_overflow_o => mcu_rx_overflow,
		 mac_tx_enable_o => mcu_tx_enable,
		 tx_allow_req_o => mcu_allow_req,
		 buf_rx_addr_o => buf_rx_a,
		 buf_rx_data_o => buf_rx_d,
		 buf_rx_len_o => buf_rx_len,
		 buf_tx_addr_o => buf_tx_a,
		 eth_ram_addr_o => mcu_read_addr,
		 mac_tx_dat_o => mcu_tx_data,
		 sys_clk_i => clk_125MHz,
		 packet_length_valid_i => ethernet_parser_sync);


b2v_packetaggregator : ethernet_packet_aggregator
PORT MAP(tx_en0_i => mcu_tx_enable,
		 tx_en1_i => tx_en_audiotx,
		 tx_en2_i => tx_en_ptpfu,
		 data0_i => mcu_tx_data,
		 data1_i => tx_data_audiotx,
		 data2_i => tx_data_ptpfu,
		 tx_en_o => mac_tx_enable,
		 data_o => mac_tx_data);


b2v_packetarbiter : eth_tx_arbiter
PORT MAP(clk_i => mac_tx_clock,
		 rst_n_i => powerGood,
		 ptp_req_i => ptp_allow_req,
		 audio_req_i => audio_allow_req,
		 mcu_req_i => mcu_allow_req,
		 ptp_allow_o => ptp_allow,
		 mcu_allow_o => mcu_allow);


b2v_ppb_meter : clock_ppb_meter
PORT MAP(sys_clk => clk_125MHz,
		 reset_n => powerGood,
		 wallclock_64fs_in => wc_64fs,
		 pll_64fs_in => pll_fs64,
		 wallclock_second_pulse_i => second_pulse_sys,
		 start_i => ppb_meter_start,
		 valid_o => pll_meas_valid,
		 count_pll_o => pll_counter,
		 count_wc_o => wc_counter);


b2v_ptp : ptp_module
PORT MAP(sys_clk => clk_125MHz,
		 rst_n => powerGood,
		 mac_tx_clock => mac_tx_clock,
		 mac_tx_busy => mac_tx_busy,
		 mac_tx_byte_sent => mac_tx_byte_sent,
		 mac_tx_allow_i => ptp_allow,
		 parse_ptp_packet_tog => parse_ptp_packet_tog,
		 ptp_is_follower => ptp_is_follower,
		 ptp_is_leader => ptp_is_leader,
		 ethernet_parser_sync => ethernet_parser_sync,
		 sof_recv_tog_i => eth_sof_recv_tog,
		 sof_sent_tog_i => eth_sof_sent_tog,
		 ip_address => ip_address,
		 mac_address => mac_address,
		 mac_ram_data => eth_ram_data_sys_ptp,
		 ptp_announce_interval => ptp_announce_interval,
		 ptp_clock_accuracy => ptp_clock_accuracy,
		 ptp_clock_class => ptp_clock_class,
		 ptp_clock_priorityone => ptp_gm_prioone,
		 ptp_clock_prioritytwo => ptp_gm_priotwo,
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


b2v_revmac : reverse_mac
PORT MAP(mac_address_i => mac_address,
		 mac_address_o => rev_mac_address);


b2v_rgmii_if : rgmii_phy_if
GENERIC MAP(CLOCK_INPUT_STYLE => "BUFG",
			IODDR_STYLE => "IODDR2",
			TARGET => "ALTERA",
			USE_CLK90 => "TRUE"
			)
PORT MAP(clk => enet_clk,
		 clk90 => enet_clk_90,
		 rst => enet_resetn,
		 mac_gmii_tx_en => gmii_tx_en,
		 mac_gmii_tx_er => gmii_tx_err,
		 phy_rgmii_rx_clk => enet_rx_clk,
		 phy_rgmii_rx_ctl => enet_rx_dv,
		 mac_gmii_txd => gmii_txd,
		 phy_rgmii_rxd => enet_rx_d,
		 speed => mac_speed,
		 mac_gmii_rx_clk => gmii_rx_clk,
		 mac_gmii_rx_dv => gmii_rx_dv,
		 mac_gmii_rx_er => gmii_rx_err,
		 mac_gmii_tx_clk => gmii_tx_clk,
		 phy_rgmii_tx_clk => enet_tx_clk,
		 phy_rgmii_tx_ctl => enet_tx_en,
		 mac_gmii_rxd => gmii_rxd,
		 phy_rgmii_txd => enet_tx_d);


b2v_rgmiiclks : ethernet_clks
PORT MAP(inclk0 => enet_clk_125m,
		 c0 => enet_clk,
		 c1 => enet_clk_90);


b2v_rx_ringbuffer : rx_ringbuffer
GENERIC MAP(audio_buffer_sample_depth => 256,
			bytes_per_sample => 3,
			global_channel_count => 16,
			max_streams => 8
			)
PORT MAP(sys_clk => clk_125MHz,
		 reset_n => powerGood,
		 fs_clk_i => fs,
		 packet_ready_i => parse_rtp_packet,
		 stream_config_wr_clk_i => mcu_clk,
		 stream_config_wr_en_i => rx_conf_wr_en,
		 metering_clear_i => audio_meter_clear,
		 eth_read_data_i => eth_ram_data_sys_rtp,
		 media_clock_i => media_clock,
		 stream_config_addr_i => rx_conf_wr_addr,
		 stream_config_data_i => rx_conf_wr_data,
		 audio_ch0_out => SYNTHESIZED_WIRE_0,
		 audio_ch10_out => SYNTHESIZED_WIRE_10,
		 audio_ch11_out => SYNTHESIZED_WIRE_11,
		 audio_ch12_out => SYNTHESIZED_WIRE_12,
		 audio_ch13_out => SYNTHESIZED_WIRE_13,
		 audio_ch14_out => SYNTHESIZED_WIRE_14,
		 audio_ch15_out => SYNTHESIZED_WIRE_15,
		 audio_ch1_out => SYNTHESIZED_WIRE_1,
		 audio_ch2_out => SYNTHESIZED_WIRE_2,
		 audio_ch3_out => SYNTHESIZED_WIRE_3,
		 audio_ch4_out => SYNTHESIZED_WIRE_4,
		 audio_ch5_out => SYNTHESIZED_WIRE_5,
		 audio_ch6_out => SYNTHESIZED_WIRE_6,
		 audio_ch7_out => SYNTHESIZED_WIRE_7,
		 audio_ch8_out => SYNTHESIZED_WIRE_8,
		 audio_ch9_out => SYNTHESIZED_WIRE_9,
		 eth_read_addr_o => rtp_ram_addr,
		 metering_clip_o => audio_meter_rx_clip,
		 metering_signal_o => audio_meter_rx_signal);


b2v_sysclocks : pll_systemclocks
PORT MAP(inclk0 => clk50m,
		 c0 => clk_125MHz);


b2v_tdm8_mux1 : tdm8_out
GENERIC MAP(width => 24
			)
PORT MAP(FSYNC => fs_tdm,
		 BIT_CLK => bclk_r,
		 RESET => powerGood,
		 DATA_CH0 => SYNTHESIZED_WIRE_0,
		 DATA_CH1 => SYNTHESIZED_WIRE_1,
		 DATA_CH2 => SYNTHESIZED_WIRE_2,
		 DATA_CH3 => SYNTHESIZED_WIRE_3,
		 DATA_CH4 => SYNTHESIZED_WIRE_4,
		 DATA_CH5 => SYNTHESIZED_WIRE_5,
		 DATA_CH6 => SYNTHESIZED_WIRE_6,
		 DATA_CH7 => SYNTHESIZED_WIRE_7,
		 DOUT => gpio5);


b2v_tdm8_mux2 : tdm8_out
GENERIC MAP(width => 24
			)
PORT MAP(FSYNC => fs_tdm,
		 BIT_CLK => bclk_r,
		 RESET => powerGood,
		 DATA_CH0 => SYNTHESIZED_WIRE_8,
		 DATA_CH1 => SYNTHESIZED_WIRE_9,
		 DATA_CH2 => SYNTHESIZED_WIRE_10,
		 DATA_CH3 => SYNTHESIZED_WIRE_11,
		 DATA_CH4 => SYNTHESIZED_WIRE_12,
		 DATA_CH5 => SYNTHESIZED_WIRE_13,
		 DATA_CH6 => SYNTHESIZED_WIRE_14,
		 DATA_CH7 => SYNTHESIZED_WIRE_15,
		 DOUT => gpio6);


b2v_tdm8demux1 : tdm8_in
GENERIC MAP(width => 24
			)
PORT MAP(FSYNC => fs_tdm,
		 BIT_CLK => bclk_f,
		 DIN => gpio0,
		 RESET => powerGood,
		 SYS_CLK => clk_125MHz,
		 DATA_CH0 => ch0i,
		 DATA_CH1 => ch1i,
		 DATA_CH2 => inch2i,
		 DATA_CH3 => inch3i,
		 DATA_CH4 => ch4i,
		 DATA_CH5 => ch5i,
		 DATA_CH6 => ch6i,
		 DATA_CH7 => ch7i);


b2v_tdm8demux2 : tdm8_in
GENERIC MAP(width => 24
			)
PORT MAP(FSYNC => fs_tdm,
		 BIT_CLK => bclk_f,
		 DIN => gpio2,
		 RESET => powerGood,
		 SYS_CLK => clk_125MHz,
		 DATA_CH0 => ch8i,
		 DATA_CH1 => ch9i,
		 DATA_CH2 => ch10i,
		 DATA_CH3 => ch11i,
		 DATA_CH4 => ch12i,
		 DATA_CH5 => ch13i,
		 DATA_CH6 => ch14i,
		 DATA_CH7 => ch15i);


b2v_yol_mac : ethernet
GENERIC MAP(MIIM_CLOCK_DIVIDER => 50,
			MIIM_DISABLE => false,
			MIIM_PHY_ADDRESS => "00000",
			MIIM_POLL_WAIT_TICKS => 10000000,
			MIIM_RESET_WAIT_TICKS => 0
			)
PORT MAP(clock_125_i => enet_clk,
		 reset_i => mac_reset,
		 mii_tx_clk_i => gmii_tx_clk,
		 mii_rx_clk_i => gmii_rx_clk,
		 mii_rx_er_i => gmii_rx_err,
		 mii_rx_dv_i => gmii_rx_dv,
		 miim_clock_i => enet_clk,
		 tx_enable_i => mac_tx_enable,
		 mdio_io => enet_mdio,
		 mac_address_i => rev_mac_address,
		 mii_rxd_i => gmii_rxd,
		 tx_data_i => mac_tx_data,
		 mii_tx_er_o => gmii_tx_err,
		 mii_tx_en_o => gmii_tx_en,
		 mdc_o => enet_mdc,
		 link_up_o => mac_linkup,
		 tx_clock_o => mac_tx_clock,
		 tx_byte_sent_o => mac_tx_byte_sent,
		 tx_busy_o => mac_tx_busy,
		 rx_clock_o => mac_rx_clock,
		 rx_reset_o => mac_rx_reset,
		 rx_frame_o => mac_rx_frame,
		 rx_byte_received_o => mac_rx_byte_received,
		 rx_error_o => mac_rx_error,
		 tx_sof_delim_tog_o => eth_sof_sent_tog,
		 rx_sof_delim_tog_o => eth_sof_recv_tog,
		 mii_txd_o => gmii_txd,
		 rx_data_o => mac_rx_data,
		 speed_o => mac_speed);

pll_fs512 <= gpio1;
powerGood <= c10_resetn;
clk50m <= c10_clk50m;
uart1_rx <= gpio28;
uart0_rx <= arduino_io1;
spiflash_miso <= gpio33;
arduino_io2 <= uart0_tx;
hbus_clk0_p <= hram_clk;
hbus_clk0_n <= nhram_clk;
gpio9 <= bclk_r;
gpio13 <= fs_tdm;
gpio10 <= fs_tdm;
gpio12 <= bclk_f;
gpio11 <= pll_fs512;
gpio34 <= spiflash_clk;
gpio32 <= spiflash_cs;
gpio35 <= spiflash_mosi;
gpio25 <= adda_nRST;
gpio29 <= uart1_tx;
user_led(3) <= mac_tx_byte_sent_n;
user_led(0) <= wallclock_locked;
user_led(1) <= ptp_is_leader;
user_led(2) <= ptp_is_follower;

spi0_miso <= '0';
END bdf_type;