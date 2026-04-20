-- Cyclone 10 LP board top level
-- Instantiates aes67_top, litex_soc, litex_eth_buffer_bridge,
-- RMII Interface; SDRAM

LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.numeric_std.all;

ENTITY top_cyc1000 IS
	PORT
	(
		c10_resetn :  IN  STD_LOGIC := '1';
		c10_clk12m :  IN  STD_LOGIC;
		enet_resetn :  OUT  STD_LOGIC;
		
		enet_mdio :  INOUT  STD_LOGIC; -- pmod 8
		enet_mdc :  OUT  STD_LOGIC; -- ain7
		uart0_tx :  OUT  STD_LOGIC; --bdbus1

		pll_64fs_o : OUT STD_LOGIC; -- ain5
		pll_256fs_rising : OUT STD_LOGIC;
		uart0_rx :  IN  STD_LOGIC;  -- bdbus0
		pll_512fs_i :  IN  STD_LOGIC; -- ain0
		tdm8in_0_i :  IN  STD_LOGIC; -- d0
		tdm8in_1_i :  IN  STD_LOGIC; -- d1
		uart1_rx :  IN  STD_LOGIC; -- d8
		i2c0_scl :  INOUT  STD_LOGIC; -- a1
		i2c0_sda :  INOUT  STD_LOGIC; -- a2
		i2c1_scl :  INOUT  STD_LOGIC; -- d6
		i2c1_sda :  INOUT  STD_LOGIC; -- d7
		lrclk1 :  OUT  STD_LOGIC;
		lrclk0 :  OUT  STD_LOGIC; -- ain6
		pll_256fs_falling :  OUT  STD_LOGIC; -- ain4
		pll_512fs_o :  OUT  STD_LOGIC; -- ain3
		tdm8out_0_o :  OUT  STD_LOGIC; -- d2
		tdm8out_1_o :  OUT  STD_LOGIC; -- d3
		spiflash_miso :  IN  STD_LOGIC; -- d14
		spiflash_clk :  OUT  STD_LOGIC; -- d12
		spiflash_cs :  OUT  STD_LOGIC; -- d11
		spiflash_mosi :  OUT  STD_LOGIC; -- d13
		adda_nRST :  OUT  STD_LOGIC; -- d10
		uart1_tx :  OUT  STD_LOGIC; --d9
		user_led :  OUT  STD_LOGIC_VECTOR(3 DOWNTO 0);

		
		phy_rmii_ref_clk : IN  STD_LOGIC; -- pmod6
		phy_rmii_crsdv   : IN  STD_LOGIC; -- pmod7
		phy_rmii_rxer    : IN  STD_LOGIC; -- n/c
		phy_rmii_rxd     : IN  STD_LOGIC_VECTOR(1 DOWNTO 0); -- pmod 4,5
		phy_rmii_txen    : OUT STD_LOGIC; -- pmod3
		phy_rmii_txd     : OUT STD_LOGIC_VECTOR(1 DOWNTO 0); -- pmod 1,0


		sdram_a                             : OUT STD_LOGIC_VECTOR(13 DOWNTO 0);
		sdram_ba                            : OUT STD_LOGIC_VECTOR(1 DOWNTO 0);
		sdram_cas_n                         : OUT STD_LOGIC;
		sdram_cke                           : OUT STD_LOGIC;
		sdram_clock                         : OUT STD_LOGIC;
		sdram_cs_n                          : OUT STD_LOGIC;
		sdram_dm                            : OUT STD_LOGIC_VECTOR(1 DOWNTO 0);
		sdram_dq                            : INOUT STD_LOGIC_VECTOR(15 DOWNTO 0);
		sdram_ras_n                         : OUT STD_LOGIC;
		sdram_we_n                          : OUT STD_LOGIC
	);
END top_cyc1000;








ARCHITECTURE bdf_type OF top_cyc1000 IS

COMPONENT rmii_phy_if
	PORT (
		rstn_async       : IN  STD_LOGIC;
		mode_speed       : IN  STD_LOGIC;
		mac_mii_crs      : OUT STD_LOGIC;
		mac_mii_rxrst    : OUT STD_LOGIC;
		mac_mii_rxc      : OUT STD_LOGIC;
		mac_mii_rxdv     : OUT STD_LOGIC;
		mac_mii_rxer     : OUT STD_LOGIC;
		mac_mii_rxd      : OUT STD_LOGIC_VECTOR(3 DOWNTO 0);
		mac_mii_txrst    : OUT STD_LOGIC;
		mac_mii_txc      : OUT STD_LOGIC;
		mac_mii_txen     : IN  STD_LOGIC;
		mac_mii_txer     : IN  STD_LOGIC;
		mac_mii_txd      : IN  STD_LOGIC_VECTOR(3 DOWNTO 0);
		phy_rmii_ref_clk : IN  STD_LOGIC;
		phy_rmii_crsdv   : IN  STD_LOGIC;
		phy_rmii_rxer    : IN  STD_LOGIC;
		phy_rmii_rxd     : IN  STD_LOGIC_VECTOR(1 DOWNTO 0);
		phy_rmii_txen    : OUT STD_LOGIC;
		phy_rmii_txd     : OUT STD_LOGIC_VECTOR(1 DOWNTO 0)
	);
END COMPONENT;

COMPONENT litex_soc
	PORT (
		aes67_ctrl_adda_nrst                : OUT STD_LOGIC;
		aes67_ctrl_eth_link_up              : IN  STD_LOGIC;
		aes67_ctrl_eth_rx_overflow          : IN  STD_LOGIC;
		aes67_ctrl_eth_speed                : IN  STD_LOGIC_VECTOR(1 DOWNTO 0);
		aes67_ctrl_eth_tx_done              : IN  STD_LOGIC;
		aes67_ctrl_eth_tx_request           : OUT STD_LOGIC;
		aes67_ctrl_ip_addr                  : OUT STD_LOGIC_VECTOR(31 DOWNTO 0);
		aes67_ctrl_mac_addr                 : OUT STD_LOGIC_VECTOR(47 DOWNTO 0);
		aes67_ctrl_meter_clear              : OUT STD_LOGIC;
		aes67_ctrl_pll_ppb_pll_count        : IN  STD_LOGIC_VECTOR(21 DOWNTO 0);
		aes67_ctrl_pll_ppb_start            : OUT STD_LOGIC;
		aes67_ctrl_pll_ppb_valid            : IN  STD_LOGIC;
		aes67_ctrl_pll_ppb_wc_count         : IN  STD_LOGIC_VECTOR(21 DOWNTO 0);
		aes67_ctrl_ptp_announce_msg_interval: OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
		aes67_ctrl_ptp_gm_clock_accuracy    : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
		aes67_ctrl_ptp_gm_clock_class       : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
		aes67_ctrl_ptp_gm_priority1         : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
		aes67_ctrl_ptp_gm_priority2         : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
		aes67_ctrl_ptp_is_follower          : OUT STD_LOGIC;
		aes67_ctrl_ptp_is_leader            : OUT STD_LOGIC;
		aes67_ctrl_ptp_leader_id            : OUT STD_LOGIC_VECTOR(63 DOWNTO 0);
		aes67_ctrl_ptp_log_msg_interval     : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
		aes67_ctrl_ptp_offset               : IN  STD_LOGIC_VECTOR(31 DOWNTO 0);
		aes67_ctrl_ptp_path_delay           : IN  STD_LOGIC_VECTOR(31 DOWNTO 0);
		aes67_ctrl_ptp_sync_lost            : IN  STD_LOGIC;
		aes67_ctrl_ptp_time_source          : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
		aes67_ctrl_rx_meter_clip            : IN  STD_LOGIC_VECTOR(15 DOWNTO 0);
		aes67_ctrl_rx_meter_signal          : IN  STD_LOGIC_VECTOR(15 DOWNTO 0);
		aes67_ctrl_tx_meter_clip            : IN  STD_LOGIC_VECTOR(15 DOWNTO 0);
		aes67_ctrl_tx_meter_signal          : IN  STD_LOGIC_VECTOR(15 DOWNTO 0);
		aes67_ctrl_wallclock_configured     : IN  STD_LOGIC;
		aes67_ctrl_wallclock_locked         : IN  STD_LOGIC;
		aes67_ctrl_wallclock_phasejump      : IN  STD_LOGIC;
		clk12                               : IN  STD_LOGIC;
		clk_mac_rx                          : IN  STD_LOGIC;
		clk_mac_tx                          : IN  STD_LOGIC;
		eth_buf_rx_ack                      : OUT STD_LOGIC;
		eth_buf_rx_addr                     : IN  STD_LOGIC_VECTOR(10 DOWNTO 0);
		eth_buf_rx_data                     : IN  STD_LOGIC_VECTOR(7 DOWNTO 0);
		eth_buf_rx_len                      : IN  STD_LOGIC_VECTOR(10 DOWNTO 0);
		eth_buf_rx_valid                    : IN  STD_LOGIC;
		eth_buf_rx_we                       : IN  STD_LOGIC;
		eth_buf_tx_addr                     : IN  STD_LOGIC_VECTOR(10 DOWNTO 0);
		eth_buf_tx_data                     : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
		eth_buf_tx_len                      : OUT STD_LOGIC_VECTOR(10 DOWNTO 0);
		i2c0_scl                            : INOUT STD_LOGIC;
		i2c0_sda                            : INOUT STD_LOGIC;
		i2c1_scl                            : INOUT STD_LOGIC;
		i2c1_sda                            : INOUT STD_LOGIC;
		rx_stream_cfg_wr_addr               : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
		rx_stream_cfg_wr_data               : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
		rx_stream_cfg_wr_en                 : OUT STD_LOGIC;
		sdram_a                             : OUT STD_LOGIC_VECTOR(13 DOWNTO 0);
		sdram_ba                            : OUT STD_LOGIC_VECTOR(1 DOWNTO 0);
		sdram_cas_n                         : OUT STD_LOGIC;
		sdram_cke                           : OUT STD_LOGIC;
		sdram_clock                         : OUT STD_LOGIC;
		sdram_cs_n                          : OUT STD_LOGIC;
		sdram_dm                            : OUT STD_LOGIC_VECTOR(1 DOWNTO 0);
		sdram_dq                            : INOUT STD_LOGIC_VECTOR(15 DOWNTO 0);
		sdram_ras_n                         : OUT STD_LOGIC;
		sdram_we_n                          : OUT STD_LOGIC;
		serial1_rx                          : IN  STD_LOGIC;
		serial1_tx                          : OUT STD_LOGIC;
		serial_rx                           : IN  STD_LOGIC;
		serial_tx                           : OUT STD_LOGIC;
		spi_clk                             : OUT STD_LOGIC;
		spi_cs_n                            : OUT STD_LOGIC;
		spi_miso                            : IN  STD_LOGIC;
		spi_mosi                            : OUT STD_LOGIC;
		spiflash_clk                        : OUT STD_LOGIC;
		spiflash_cs_n                       : OUT STD_LOGIC;
		spiflash_miso                       : IN  STD_LOGIC;
		spiflash_mosi                       : OUT STD_LOGIC;
		sys_clk_out                         : OUT STD_LOGIC;
		tx_stream_cfg_wr_addr               : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
		tx_stream_cfg_wr_data               : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
		tx_stream_cfg_wr_en                 : OUT STD_LOGIC
	);
END COMPONENT;


-- board clocks
signal clk_125MHz    : STD_LOGIC;
signal reset_p 		 : STD_LOGIC;



-- MAC status/control between aes67_top and litex_soc/board
signal mac_rx_clock  : STD_LOGIC;
signal mac_tx_clock  : STD_LOGIC;
signal mac_speed     : STD_LOGIC_VECTOR(1 downto 0);
signal mac_linkup    : STD_LOGIC;
signal mac_tx_busy   : STD_LOGIC;
signal mac_tx_byte_sent : STD_LOGIC;

-- aes67_top <-> litex_eth_buffer_bridge (MCU ethernet path)
signal is_mcu_pkt_tog     : STD_LOGIC;
signal mcu_ram_addr       : unsigned(10 downto 0);
signal eth_ram_data_rx_mcu : STD_LOGIC_VECTOR(7 downto 0);
signal eth_tx_en_mcu      : STD_ULOGIC;
signal eth_tx_data_mcu    : STD_ULOGIC_VECTOR(7 downto 0);
signal eth_tx_allow_req_mcu : STD_ULOGIC;
signal eth_tx_allow_mcu   : STD_LOGIC;
signal received_packet_length : unsigned(10 downto 0);
signal mac_tx_reset : std_logic;
signal mac_sof_sent_pulse : STD_LOGIC;
-- litex_eth_buffer_bridge internal signals
signal mcu_tx_req_i    : STD_LOGIC;
signal mcu_tx_done     : STD_ULOGIC;
signal mcu_rx_overflow : STD_ULOGIC;
-- litex_soc <-> litex_eth_buffer_bridge (buffer interface)
-- bridge-side signals (matching litex_eth_buffer_bridge types)
signal buf_rx_data    : STD_ULOGIC_VECTOR(7 downto 0);
signal buf_rx_addr    : unsigned(10 downto 0);
signal buf_rx_we      : STD_ULOGIC;
signal buf_rx_len     : unsigned(10 downto 0);
signal buf_rx_valid   : STD_ULOGIC;
signal buf_rx_ack     : STD_LOGIC;
signal buf_tx_addr    : unsigned(10 downto 0);
-- soc TX buffer outputs (std_logic_vector from litex_soc component)
signal soc_tx_dat     : STD_LOGIC_VECTOR(7 downto 0);
signal soc_tx_len     : STD_LOGIC_VECTOR(10 downto 0);

-- litex_soc control registers -> aes67_top
signal mac_address         : STD_LOGIC_VECTOR(47 downto 0);
signal ip_address          : STD_LOGIC_VECTOR(31 downto 0);
signal ptp_announce_interval : STD_LOGIC_VECTOR(7 downto 0);
signal ptp_clock_accuracy  : STD_LOGIC_VECTOR(7 downto 0);
signal ptp_clock_class     : STD_LOGIC_VECTOR(7 downto 0);
signal ptp_gm_prioone      : STD_LOGIC_VECTOR(7 downto 0);
signal ptp_gm_priotwo      : STD_LOGIC_VECTOR(7 downto 0);
signal ptp_current_leader_id : STD_LOGIC_VECTOR(63 downto 0);
signal ptp_log_message_interval : STD_LOGIC_VECTOR(7 downto 0);
signal ptp_time_source     : STD_LOGIC_VECTOR(7 downto 0);
signal ptp_is_follower     : STD_LOGIC;
signal ptp_is_leader       : STD_LOGIC;
signal ppb_meter_start     : STD_LOGIC;
signal audio_meter_clear   : STD_LOGIC;

-- aes67_top status -> litex_soc
signal ptp_sync_lost       : STD_LOGIC;
signal ptp_mean_path_delay : STD_LOGIC_VECTOR(31 downto 0);
signal ptp_offset_from_master : STD_LOGIC_VECTOR(31 downto 0);
signal wallclock_locked    : STD_LOGIC;
signal wallclock_configured : STD_LOGIC;
signal wallclock_phasejump : STD_LOGIC;
signal pll_meas_valid      : STD_LOGIC;
signal pll_counter         : unsigned(21 downto 0);
signal wc_counter          : unsigned(21 downto 0);
signal audio_meter_rx_clip   : STD_LOGIC_VECTOR(15 downto 0);
signal audio_meter_rx_signal : STD_LOGIC_VECTOR(15 downto 0);
signal audio_meter_tx_clip   : STD_LOGIC_VECTOR(15 downto 0);
signal audio_meter_tx_signal : STD_LOGIC_VECTOR(15 downto 0);

-- stream configuration
signal tx_wr_en       : STD_LOGIC;
signal tx_wr_data     : STD_LOGIC_VECTOR(7 downto 0);
signal tx_wr_addr     : STD_LOGIC_VECTOR(7 downto 0);
signal rx_conf_wr_en  : STD_LOGIC;
signal rx_conf_wr_data : STD_LOGIC_VECTOR(7 downto 0);
signal rx_conf_wr_addr : STD_LOGIC_VECTOR(7 downto 0);

-- audio clocks to board pins

signal pll_48k_fs      : STD_LOGIC;
signal pll_48k_fs_tdm  : STD_LOGIC;

-- litex_soc misc
signal mcu_clk       : STD_LOGIC;
signal mac_mii_crs   : STD_LOGIC;
signal mac_mii_rxrst : STD_LOGIC;
signal mac_mii_rxclk : STD_LOGIC;
signal mac_mii_rxdv : STD_LOGIC;
signal mac_mii_rxer : STD_LOGIC;
signal mac_mii_rxd : STD_LOGIC_VECTOR(3 downto 0);
signal mac_mii_txrst : STD_LOGIC;
signal mac_mii_txen : STD_LOGIC;
signal mac_mii_txclk : STD_LOGIC;
signal mac_mii_txer : STD_LOGIC;
signal mac_mii_txd : STD_LOGIC_VECTOR(3 downto 0);
signal mii_txd_full : STD_LOGIC_VECTOR(7 downto 0);


BEGIN


enet_resetn <= c10_resetn;
reset_p <= not c10_resetn;

-- MII TX: MAC outputs 8 bits (GMII), only lower 4 bits are used for MII
mac_mii_txd <= mii_txd_full(3 downto 0);


rmii_phy_if_inst: rmii_phy_if
 port map(
	mode_speed => '1',
	rstn_async => c10_resetn,
	mac_mii_crs => mac_mii_crs,
	mac_mii_rxrst => mac_mii_rxrst,
	mac_mii_rxc => mac_mii_rxclk,
	mac_mii_rxdv => mac_mii_rxdv,
	mac_mii_rxer => mac_mii_rxer,
	mac_mii_rxd => mac_mii_rxd,
	mac_mii_txrst => mac_mii_txrst,
	mac_mii_txc => mac_mii_txclk,
	mac_mii_txen => mac_mii_txen,
	mac_mii_txer => mac_mii_txer,
	mac_mii_txd => mac_mii_txd,
	phy_rmii_ref_clk => phy_rmii_ref_clk,
	phy_rmii_crsdv => phy_rmii_crsdv,
	phy_rmii_rxer => phy_rmii_rxer,
	phy_rmii_rxd => phy_rmii_rxd,
	phy_rmii_txen => phy_rmii_txen,
	phy_rmii_txd => phy_rmii_txd
);

aes67_top_inst: entity work.aes67_top
generic map(
	MIIM_CLOCK_DIVIDER => 10,
	TX_SAMPLE_BUFFER_DEPTH => 48,
	RX_SAMPLE_BUFFER_DEPTH => 128
)
 port map(
	sys_clk_125MHz_i       => clk_125MHz,
	enet_clk_i             => mac_mii_rxclk,
	clk_mcu_i              => mcu_clk,
	rst_n                  => c10_resetn,
	mac_resetn_i            => c10_resetn,

	-- G/MII from R(G)MII converter
	mii_rx_clock_i         => mac_mii_rxclk,
	mii_tx_clock_i         => mac_mii_txclk,
	mii_rx_err_i           => mac_mii_rxer,
	mii_rx_dv_i            => mac_mii_rxdv,
	mii_rxd_i              => x"0" & mac_mii_rxd,
	mii_tx_err_o           => mac_mii_txer,
	mii_tx_en_o            => mac_mii_txen,
	mii_txd_o              => mii_txd_full,
	enet_mdio              => enet_mdio,
	enet_mdc               => enet_mdc,

	-- SOC ethernet interface
	mac_rx_clock_o         => mac_rx_clock,
	mac_tx_clock_o         => mac_tx_clock,
	is_mcu_pkt_tog_o       => is_mcu_pkt_tog,
	mcu_ram_addr_i         => mcu_ram_addr,
	eth_ram_data_rx_mcu_o  => eth_ram_data_rx_mcu,
	eth_tx_en_mcu_i        => std_logic(eth_tx_en_mcu),
	eth_tx_data_mcu_i      => std_logic_vector(eth_tx_data_mcu),
	eth_tx_allow_req_mcu_i => std_logic(eth_tx_allow_req_mcu),
	eth_tx_allow_mcu_o     => eth_tx_allow_mcu,
	mac_received_packet_length_o => received_packet_length,
	mac_tx_reset_o => mac_tx_reset,
	mac_tx_busy_o          => mac_tx_busy,
	mac_tx_byte_sent_o     => mac_tx_byte_sent,
	mac_speed_o            => mac_speed,
	mac_linkup_o           => mac_linkup,
	mac_sof_sent_pulse_o => mac_sof_sent_pulse,

	-- audio clocks
	pll_512fs_i            => pll_512fs_i,
	pll_64fs_o             => pll_64fs_o,
	pll_48k_fs_o           => pll_48k_fs,
	pll_48k_fs_tdm_o       => pll_48k_fs_tdm,
	pll_256fs_rising_o     => pll_256fs_rising,
	pll_256fs_falling_o    => pll_256fs_falling,

	-- control registers from SoC
	mac_address_i          => mac_address,
	ip_address_i           => ip_address,
	ptp_announce_interval_i     => ptp_announce_interval,
	ptp_clock_accuracy_i        => ptp_clock_accuracy,
	ptp_clock_class_i           => ptp_clock_class,
	ptp_gm_prioone_i            => ptp_gm_prioone,
	ptp_gm_priotwo_i            => ptp_gm_priotwo,
	ptp_current_leader_id_i     => ptp_current_leader_id,
	ptp_log_message_interval_i  => ptp_log_message_interval,
	ptp_time_source_i           => ptp_time_source,
	ptp_is_follower_i           => ptp_is_follower,
	ptp_is_leader_i             => ptp_is_leader,

	-- status to SoC
	ptp_sync_lost_o             => ptp_sync_lost,
	ptp_mean_path_delay_o       => ptp_mean_path_delay,
	ptp_offset_from_master_o    => ptp_offset_from_master,
	wallclock_locked_o          => wallclock_locked,
	wallclock_configured_o      => wallclock_configured,
	wallclock_phasejump_o       => wallclock_phasejump,

	-- ppb meter
	ppb_meter_start_i           => ppb_meter_start,
	pll_meas_valid_o            => pll_meas_valid,
	pll_counter_o               => pll_counter,
	wc_counter_o                => wc_counter,

	-- audio metering
	audio_meter_clear_i         => audio_meter_clear,
	audio_meter_rx_clip_o       => audio_meter_rx_clip,
	audio_meter_rx_signal_o     => audio_meter_rx_signal,
	audio_meter_tx_clip_o       => audio_meter_tx_clip,
	audio_meter_tx_signal_o     => audio_meter_tx_signal,

	-- stream configuration
	audio_tx_cfg_wr_en_i        => tx_wr_en,
	audio_tx_cfg_wr_data_i      => tx_wr_data,
	audio_tx_cfg_wr_addr_i      => tx_wr_addr,
	audio_rx_cfg_wr_en_i        => rx_conf_wr_en,
	audio_rx_cfg_wr_data_i      => rx_conf_wr_data,
	audio_rx_cfg_wr_addr_i      => rx_conf_wr_addr,

	-- TDM audio I/O
	tdm8out_0_o                 => tdm8out_0_o,
	tdm8out_1_o                 => tdm8out_1_o,
	tdm8in_0_i                  => tdm8in_0_i,
	tdm8in_1_i                  => tdm8in_1_i
);


litex_soc_inst: litex_soc
PORT MAP(
	-- clocks
	clk12                              => c10_clk12m,
	clk_mac_rx                         => mac_rx_clock,
	clk_mac_tx                         => mac_tx_clock,
	sys_clk_out                        => mcu_clk,

	-- aes67 control/status
	aes67_ctrl_eth_link_up             => mac_linkup,
	aes67_ctrl_eth_rx_overflow         => mcu_rx_overflow,
	aes67_ctrl_eth_tx_done             => mcu_tx_done,
	aes67_ctrl_eth_speed               => mac_speed,
	aes67_ctrl_eth_tx_request          => mcu_tx_req_i,

	aes67_ctrl_pll_ppb_valid           => pll_meas_valid,
	aes67_ctrl_pll_ppb_pll_count       => std_logic_vector(pll_counter),
	aes67_ctrl_pll_ppb_wc_count        => std_logic_vector(wc_counter),
	aes67_ctrl_pll_ppb_start           => ppb_meter_start,

	aes67_ctrl_ptp_sync_lost           => ptp_sync_lost,
	aes67_ctrl_ptp_offset              => ptp_offset_from_master,
	aes67_ctrl_ptp_path_delay          => ptp_mean_path_delay,
	aes67_ctrl_wallclock_configured    => wallclock_configured,
	aes67_ctrl_wallclock_locked        => wallclock_locked,
	aes67_ctrl_wallclock_phasejump     => wallclock_phasejump,

	aes67_ctrl_rx_meter_clip           => audio_meter_rx_clip,
	aes67_ctrl_rx_meter_signal         => audio_meter_rx_signal,
	aes67_ctrl_tx_meter_clip           => audio_meter_tx_clip,
	aes67_ctrl_tx_meter_signal         => audio_meter_tx_signal,
	aes67_ctrl_meter_clear             => audio_meter_clear,

	aes67_ctrl_adda_nrst               => adda_nRST,
	aes67_ctrl_ptp_is_follower         => ptp_is_follower,
	aes67_ctrl_ptp_is_leader           => ptp_is_leader,

	aes67_ctrl_ip_addr                 => ip_address,
	aes67_ctrl_mac_addr                => mac_address,
	aes67_ctrl_ptp_announce_msg_interval => ptp_announce_interval,
	aes67_ctrl_ptp_gm_clock_accuracy   => ptp_clock_accuracy,
	aes67_ctrl_ptp_gm_clock_class      => ptp_clock_class,
	aes67_ctrl_ptp_gm_priority1        => ptp_gm_prioone,
	aes67_ctrl_ptp_gm_priority2        => ptp_gm_priotwo,
	aes67_ctrl_ptp_leader_id           => ptp_current_leader_id,
	aes67_ctrl_ptp_log_msg_interval    => ptp_log_message_interval,
	aes67_ctrl_ptp_time_source         => ptp_time_source,

	-- ethernet buffer interface
	eth_buf_rx_valid                   => buf_rx_valid,
	eth_buf_rx_we                      => buf_rx_we,
	eth_buf_rx_addr                    => std_logic_vector(buf_rx_addr),
	eth_buf_rx_data                    => std_logic_vector(buf_rx_data),
	eth_buf_rx_len                     => std_logic_vector(buf_rx_len),
	eth_buf_rx_ack                     => buf_rx_ack,
	eth_buf_tx_addr                    => std_logic_vector(buf_tx_addr),
	eth_buf_tx_data                    => soc_tx_dat,
	eth_buf_tx_len                     => soc_tx_len,

	-- stream configuration
	rx_stream_cfg_wr_en                => rx_conf_wr_en,
	rx_stream_cfg_wr_addr              => rx_conf_wr_addr,
	rx_stream_cfg_wr_data              => rx_conf_wr_data,
	tx_stream_cfg_wr_en                => tx_wr_en,
	tx_stream_cfg_wr_addr              => tx_wr_addr,
	tx_stream_cfg_wr_data              => tx_wr_data,

	-- peripherals
	serial_rx                          => uart0_rx,
	serial_tx                          => uart0_tx,
	serial1_rx                         => uart1_rx,
	serial1_tx                         => uart1_tx,
	spi_miso                           => '0',
	spiflash_miso                      => spiflash_miso,
	spiflash_clk                       => spiflash_clk,
	spiflash_cs_n                      => spiflash_cs,
	spiflash_mosi                      => spiflash_mosi,



	-- I2C
	i2c0_scl                           => i2c0_scl, -- gpio26
	i2c0_sda                           => i2c0_sda, -- gpio27
	i2c1_scl                           => i2c1_scl, -- gpio23
	i2c1_sda                           => i2c1_sda, -- gpio24

	-- sdram
	sdram_a                            => sdram_a,
	sdram_ba                           => sdram_ba,
	sdram_cas_n                        => sdram_cas_n,
	sdram_cke                          => sdram_cke,
	sdram_clock                        => sdram_clock,
	sdram_cs_n                         => sdram_cs_n,
	sdram_dm                           => sdram_dm,
	sdram_dq                           => sdram_dq,
	sdram_ras_n                        => sdram_ras_n,
	sdram_we_n                         => sdram_we_n
);


litex_eth_inst: entity work.litex_eth_buffer_bridge
PORT MAP(
	sys_clk_i              => clk_125MHz,
	mcu_clk_i              => mcu_clk,

	-- LiteX buffer RX (bridge writes received frames)
	buf_rx_data_o          => buf_rx_data,
	buf_rx_addr_o          => buf_rx_addr,
	buf_rx_we_o            => buf_rx_we,
	buf_rx_len_o           => buf_rx_len,
	buf_rx_valid_o         => buf_rx_valid,
	buf_rx_ack_i           => buf_rx_ack,

	-- LiteX buffer TX (bridge reads packet data)
	buf_tx_addr_o          => buf_tx_addr,
	buf_tx_len_i           => unsigned(soc_tx_len),
	buf_tx_dat_i           => std_ulogic_vector(soc_tx_dat),

	-- SoC control
	eth_tx_request_i       => mcu_tx_req_i,
	eth_tx_done_o          => mcu_tx_done,
	eth_rx_overflow_o      => mcu_rx_overflow,

	-- MAC TX
	mac_tx_clock_i         => mac_tx_clock,
	mac_tx_reset_i         => mac_tx_reset,
	mac_tx_enable_o        => eth_tx_en_mcu,
	mac_tx_byte_sent_i     => mac_tx_byte_sent,
	mac_tx_busy_i          => mac_tx_busy,
	mac_tx_dat_o           => eth_tx_data_mcu,
	mac_sof_sent_pulse_i	=> mac_sof_sent_pulse,
	mac_speed_in			=> mac_speed,
	tx_allow_req_o         => eth_tx_allow_req_mcu,
	tx_allow_i             => eth_tx_allow_mcu,

	-- MAC RX
	mac_rx_clock_i         => mac_rx_clock,
	mac_rx_reset_i         => reset_p,
	parse_mcu_packet_tog_i => is_mcu_pkt_tog,
	pkt_len_i              => received_packet_length,
	eth_ram_data_i         => std_ulogic_vector(eth_ram_data_rx_mcu),
	eth_ram_addr_o         => mcu_ram_addr,
	packet_length_valid_i  => '1'
);





sysclocks_inst: entity work.pll_systemclocks
PORT MAP(
	inclk0 => c10_clk12m,
	c0     => clk_125MHz
);


-- GPIO / pin assignments


lrclk1        <= pll_48k_fs_tdm; -- gpio13
lrclk0        <= pll_48k_fs_tdm; -- gpio11
pll_512fs_o        <= pll_512fs_i;-- gpio 11 <= gpio 1 pll_512fs passthrough

-- LEDs
user_led(0)   <= wallclock_locked;
user_led(1)   <= ptp_is_leader;
user_led(2)   <= ptp_is_follower;
user_led(3)   <= NOT mac_tx_byte_sent;

END bdf_type;
