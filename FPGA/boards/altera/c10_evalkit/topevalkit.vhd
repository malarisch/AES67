LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.numeric_std.all;

ENTITY topevalkit IS 
	generic (
		SOC_TYPE : string := "LITEX_HRAM"; -- LITEX_HRAM, LITEX_SDRAM, SPIBONE; WISHBONE_ONLY
		platform : string := "ALTERA"; -- ALTERA, GOWIN, LATTICE

		-- clocking
		STATIC_PTP_CONF : 		BOOLEAN := true;
        clk_in_speed : natural := 50; -- input clock speed in mhz (for now only 12, 27, 50)
		SYS_CLK_NS_PER_TICK : integer := 8; -- 125 MHz
		USE_EXTERNAL_PLL : boolean := FALSE; -- when disabled it will use the nco-generated clocks on the outputs
		ENABLE_METERING: BOOLEAN := false;
		PTP_MOVING_AVERAGE_DEPTH : integer := 8;

		-- ethernet cfg
		MII_WIDTH : integer := 4; -- 2 for rmii, 8 gmii, 4 rgmii
		MII_CLK_NS_PER_TICK : integer := 8; -- 20 for rmii, 40 mii, 8 rgmii/gmii
		MIIM_CLOCK_DIVIDER : POSITIVE := 50;
		ethernet_type	 : string := "RGMII"; -- RMII; RGMII
		
		-- audio rx
		RX_SAMPLE_BUFFER_DEPTH : INTEGER := 256; -- must be power of two
		RX_MAX_STREAMS : natural := 8; -- max concurrent streams being received
		RX_CHANNELS		: natural := 16; -- receive channel count
		AUDIO_RX_USE_PARALLEL_INTERFACE : boolean := false;
        RX_BYTE_DEPTH	: natural := 3; -- width for parallel interface
		AUDIO_RX_TDM_OUTPUTS : natural := 2; -- tdm pin count
		AUDIO_RX_TDM_CHANNELS : natural  := 8; -- tdm channel count

		-- audio tx
		TX_SAMPLE_BUFFER_DEPTH : INTEGER := 64; -- must be power of two
		TX_MAX_STREAMS : natural := 8;
        TX_CHANNELS		: natural := 8; -- must be multiple of two for i2s, multiple of 8 for tdm8
		AUDIO_TX_USE_PARALLEL_INTERFACE : boolean := false;
        TX_BYTE_DEPTH	: natural := 3; -- with for parallel interface
		AUDIO_TX_TDM_INPUTS : natural := 2;
		AUDIO_TX_TDM_CHANNELS : natural  := 8

		
	);
	PORT
	(
		rst_n_i :  IN  STD_LOGIC;
		clock_i :  IN  STD_LOGIC; 

		-- rgmii if (use when ethernet_type  = RGMII)
		phy_rgmii_enet_clk_125m :  IN  STD_LOGIC;
		phy_rgmii_enet_rx_clk :  IN  STD_LOGIC;
		phy_rgmii_enet_rx_dv :  IN  STD_LOGIC;
		phy_rgmii_enet_resetn :  OUT  STD_LOGIC;
		phy_rgmii_enet_rx_d :  IN  STD_LOGIC_VECTOR(3 DOWNTO 0);
		phy_rgmii_enet_tx_clk :  OUT  STD_LOGIC;
		phy_rgmii_enet_tx_en :  OUT  STD_LOGIC;
		phy_rgmii_enet_tx_d :  OUT  STD_LOGIC_VECTOR(3 DOWNTO 0);



		enet_mdc :  OUT  STD_LOGIC;
		enet_mdio :  INOUT  STD_LOGIC;
		
		-- hyperram for litex hram target
		hbus_rwds :  INOUT  STD_LOGIC;
		hbus_dq :  INOUT  STD_LOGIC_VECTOR(7 DOWNTO 0);
		hbus_rstn :  OUT  STD_LOGIC;
		hbus_cs2n :  OUT  STD_LOGIC;
		hbus_clk0_p :  OUT  STD_LOGIC;
		hbus_clk0_n :  OUT  STD_LOGIC;
		


		-- litex console
		uart0_tx :  OUT  STD_LOGIC;
		uart0_rx :  IN  STD_LOGIC;

		-- litex uart interface for adda boards
		uart1_rx :  IN  STD_LOGIC;
		uart1_tx :  OUT  STD_LOGIC;

		-- litex i2c master
		i2c0_scl :  INOUT  STD_LOGIC;
		i2c0_sda :  INOUT  STD_LOGIC;
		i2c1_scl :  INOUT  STD_LOGIC;
		i2c1_sda :  INOUT  STD_LOGIC;


		-- litex spi flash
		spiflash_clk :  OUT  STD_LOGIC;
		spiflash_cs :  OUT  STD_LOGIC;
		spiflash_mosi :  OUT  STD_LOGIC;
		spiflash_miso :  IN  STD_LOGIC;


		-- audio clock in from external pll - only used when USE_EXTERNAL_PLL is true
		pll_512fs_i :  IN  STD_LOGIC; -- gpio 1

		-- audio clocks outputs
		pll_512fs_o :  OUT  STD_LOGIC;
		pll_256fs_rising : OUT STD_LOGIC;
		pll_256fs_falling :  OUT  STD_LOGIC; -- gpio 12
		lrclk_o :  OUT  STD_LOGIC;
		lrclk_tdm_o :  OUT  STD_LOGIC;

		-- audio interface (TODO: MAKE THIS VECTORS!)
		tdm8in_0_i :  IN  STD_LOGIC;
		tdm8in_1_i :  IN  STD_LOGIC;
		tdm8out_0_o :  OUT  STD_LOGIC;
		tdm8out_1_o :  OUT  STD_LOGIC;

		-- spictrl interface (only active if soc_type is spi)
		spictrl_clk_i : IN STD_LOGIC;
		spictrl_mosi_i : IN STD_LOGIC;
		spictrl_cs_n_i : IN STD_LOGIC;
		spictrl_miso_o : INOUT STD_LOGIC;
		spictrl_irq_n_o : OUT STD_LOGIC;
		
		
		
		
		
		-- misc

		--adda_nRST :  OUT  STD_LOGIC;
		user_led :  OUT  STD_LOGIC_VECTOR(3 DOWNTO 0)
		
	);
END topevalkit;

architecture rtl of topevalkit is
signal tdm_in : STD_LOGIC_VECTOR (AUDIO_TX_TDM_INPUTS - 1 downto 0);
signal tdm_out : STD_LOGIC_VECTOR (AUDIO_RX_TDM_OUTPUTS - 1 downto 0);
begin

    tdm_in(0) <= tdm8in_0_i;
    tdm_in(1) <= tdm8in_1_i;
    tdm8out_0_o <= tdm_out(0);
    tdm8out_1_o <= tdm_out(1);
    soc_top_inst: entity work.soc_top
     generic map(
        clk_in_speed => clk_in_speed,
        platform => platform,
		SOC_TYPE => SOC_TYPE,
        MII_WIDTH => MII_WIDTH,
        ETHERNET_TYPE => ETHERNET_TYPE,
        SYS_CLK_NS_PER_TICK => SYS_CLK_NS_PER_TICK,
        MII_CLK_NS_PER_TICK => MII_CLK_NS_PER_TICK,
        TX_MAX_STREAMS => TX_MAX_STREAMS,
        RX_MAX_STREAMS => RX_MAX_STREAMS,
        MIIM_CLOCK_DIVIDER => MIIM_CLOCK_DIVIDER,
        RX_CHANNELS => RX_CHANNELS,
        RX_SAMPLE_BUFFER_DEPTH => RX_SAMPLE_BUFFER_DEPTH,
        AUDIO_RX_USE_PARALLEL_INTERFACE => AUDIO_RX_USE_PARALLEL_INTERFACE,
        RX_BYTE_DEPTH => RX_BYTE_DEPTH,
        AUDIO_RX_TDM_OUTPUTS => AUDIO_RX_TDM_OUTPUTS,
        AUDIO_RX_TDM_CHANNELS => AUDIO_RX_TDM_CHANNELS,
        TX_CHANNELS => TX_CHANNELS,
        TX_SAMPLE_BUFFER_DEPTH => TX_SAMPLE_BUFFER_DEPTH,
        AUDIO_TX_USE_PARALLEL_INTERFACE => AUDIO_TX_USE_PARALLEL_INTERFACE,
        TX_BYTE_DEPTH => TX_BYTE_DEPTH,
        AUDIO_TX_TDM_INPUTS => AUDIO_TX_TDM_INPUTS,
        AUDIO_TX_TDM_CHANNELS => AUDIO_TX_TDM_CHANNELS,
        USE_EXTERNAL_PLL => USE_EXTERNAL_PLL,
        ENABLE_METERING => ENABLE_METERING,
		PTP_MOVING_AVERAGE_DEPTH => PTP_MOVING_AVERAGE_DEPTH
    )
     port map(
        rst_n_i => rst_n_i,
        clock_i => clock_i,
        phy_refclk_i => phy_rgmii_enet_clk_125m,
        phy_mii_enet_rx_clk => phy_rgmii_enet_rx_clk,
        phy_mii_enet_rx_dv => phy_rgmii_enet_rx_dv,
        phy_mii_enet_resetn => phy_rgmii_enet_resetn,
        phy_mii_enet_rx_d => phy_rgmii_enet_rx_d,
        phy_mii_enet_tx_clk => phy_rgmii_enet_tx_clk,
        phy_mii_enet_tx_en => phy_rgmii_enet_tx_en,
        phy_mii_enet_tx_d => phy_rgmii_enet_tx_d,
        enet_mdc => enet_mdc,
        enet_mdio => enet_mdio,
        hbus_rwds => hbus_rwds,
        hbus_dq => hbus_dq,
        hbus_rstn => hbus_rstn,
        hbus_cs2n => hbus_cs2n,
        hbus_clk0_p => hbus_clk0_p,
        hbus_clk0_n => hbus_clk0_n,
        uart0_tx => uart0_tx,
        uart0_rx => uart0_rx,
        uart1_rx => uart1_rx,
        uart1_tx => uart1_tx,
        i2c0_scl => i2c0_scl,
        i2c0_sda => i2c0_sda,
        i2c1_scl => i2c1_scl,
        i2c1_sda => i2c1_sda,
        spiflash_clk => spiflash_clk,
        spiflash_cs => spiflash_cs,
        spiflash_mosi => spiflash_mosi,
        spiflash_miso => spiflash_miso,
        pll_512fs_i => pll_512fs_i,
        audioclk_512fs_o => pll_512fs_o,
        audioclk_256fs_rising_o => pll_256fs_rising,
        audioclk_256fs_falling_o => pll_256fs_falling,
        --audioclk_64fs_o => ,
        audioclk_lrclk_o => lrclk_tdm_o,
        tdm_in => tdm_in,
        tdm_out => tdm_out,
		spibone_clk => spictrl_clk_i,
		spibone_cs_n => spictrl_cs_n_i,
		spibone_miso => spictrl_miso_o,
		spibone_mosi => spictrl_mosi_i,
		spibone_irq_o => spictrl_irq_n_o
    );
    

end architecture;