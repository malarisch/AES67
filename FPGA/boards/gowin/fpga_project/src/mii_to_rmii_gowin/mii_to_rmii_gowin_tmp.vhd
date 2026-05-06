--Copyright (C)2014-2025 Gowin Semiconductor Corporation.
--All rights reserved.
--File Title: Template file for instantiation
--Tool Version: V1.9.12
--Part Number: GW2A-LV18PG256C8/I7
--Device: GW2A-18
--Created Time: Thu Apr 30 12:00:58 2026

--Change the instance name and port connections to the signal names
----------Copy here to design--------

component mii_to_rmii_gowin
	port (
		refclk: in std_logic;
		rstn: in std_logic;
		speedis_100: in std_logic;
		rmii_rx_crs_dv: in std_logic;
		rmii_rx_er: in std_logic;
		rmii_rxd: in std_logic_vector(1 downto 0);
		rmii_tx_en: out std_logic;
		rmii_txd: out std_logic_vector(1 downto 0);
		mii_rx_clk: out std_logic;
		mii_rx_dv: out std_logic;
		mii_rx_er: out std_logic;
		mii_rxd: out std_logic_vector(3 downto 0);
		mii_tx_clk: out std_logic;
		mii_tx_en: in std_logic;
		mii_tx_er: in std_logic;
		mii_txd: in std_logic_vector(3 downto 0);
		mii_col: out std_logic;
		mii_crs: out std_logic
	);
end component;

your_instance_name: mii_to_rmii_gowin
	port map (
		refclk => refclk,
		rstn => rstn,
		speedis_100 => speedis_100,
		rmii_rx_crs_dv => rmii_rx_crs_dv,
		rmii_rx_er => rmii_rx_er,
		rmii_rxd => rmii_rxd,
		rmii_tx_en => rmii_tx_en,
		rmii_txd => rmii_txd,
		mii_rx_clk => mii_rx_clk,
		mii_rx_dv => mii_rx_dv,
		mii_rx_er => mii_rx_er,
		mii_rxd => mii_rxd,
		mii_tx_clk => mii_tx_clk,
		mii_tx_en => mii_tx_en,
		mii_tx_er => mii_tx_er,
		mii_txd => mii_txd,
		mii_col => mii_col,
		mii_crs => mii_crs
	);

----------Copy end-------------------
