LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.numeric_std.all;

ENTITY topevalkit IS 
	generic (
		soctype : string := "litex_c10_hram"; -- spi or litex_c10_hram or litex_c10_sdram or litex_tang_primer_20k
		platform : string := "ALTERA";
        board : string := "C10EVALKIT";
        MII_WIDTH : integer := 8; -- 2 for rmii, 8 gmii/rgmii
		MII_CLK_NS_PER_TICK : integer := 8; -- 20 for rmii, 40 mii, 8 rgmii/gmii
		clk_in_speed : natural := 50; -- input clock speed in mhz (for now only 12, 27, 50)
		ethernet_type	 : string := "RGMII"; -- RMII; RGMII
		USE_EXTERNAL_PLL : boolean := FALSE; -- when disabled it will use the nco-generated clocks on the outputs
		FPGAVERSIONMSB : integer := 1;
        FPGAVERSIONLSB : integer := 123;
        TXSTREAMS      : integer := 8;
        RXSTREAMS       : INTEGER := 8;
        TXCHANNELS      : INTEGER := 8;
        RXCHANNELS      : INTEGER := 8;
        BITDEPTH        : INTEGER := 24;
        SAMPLERATE      : INTEGER := 48;
		TX_SAMPLE_BUFFER_DEPTH : INTEGER := 128; -- must be power of two (media-clock-derived TX write pointer)
		RX_SAMPLE_BUFFER_DEPTH : INTEGER := 256;
		STATIC_PTP_CONF : 		BOOLEAN := true;
        ENABLE_METERING: BOOLEAN := false
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

		phy_rmii_ref_clk : IN  STD_LOGIC; -- pmod6
		phy_rmii_crsdv   : IN  STD_LOGIC; -- pmod7
		phy_rmii_rxer    : IN  STD_LOGIC; -- n/c
		phy_rmii_rxd     : IN  STD_LOGIC_VECTOR(1 DOWNTO 0); -- pmod 4,5
		phy_rmii_txen    : OUT STD_LOGIC; -- pmod3
		phy_rmii_txd     : OUT STD_LOGIC_VECTOR(1 DOWNTO 0); -- pmod 1,0


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
		spictrl_miso_o : OUT STD_LOGIC;
		spictrl_irq_n_o : OUT STD_LOGIC;
		
		
		
		
		
		-- misc

		adda_nRST :  OUT  STD_LOGIC;
		user_led :  OUT  STD_LOGIC_VECTOR(3 DOWNTO 0)
		
	);
END topevalkit;

architecture rtl of topevalkit is

begin

    top_inst: entity work.top
     generic map(
        soctype => soctype,
        platform => platform,
        clk_in_speed => clk_in_speed,
        ethernet_type => ethernet_type,
        USE_EXTERNAL_PLL => USE_EXTERNAL_PLL,
        FPGAVERSIONMSB => FPGAVERSIONMSB,
        FPGAVERSIONLSB => FPGAVERSIONLSB,
        TXSTREAMS => TXSTREAMS,
        RXSTREAMS => RXSTREAMS,
        TXCHANNELS => TXCHANNELS,
        RXCHANNELS => RXCHANNELS,
        BITDEPTH => BITDEPTH,
        SAMPLERATE => SAMPLERATE,
        TX_SAMPLE_BUFFER_DEPTH => TX_SAMPLE_BUFFER_DEPTH,
        RX_SAMPLE_BUFFER_DEPTH => RX_SAMPLE_BUFFER_DEPTH,
		MII_WIDTH => MII_WIDTH,
		MII_CLK_NS_PER_TICK => MII_CLK_NS_PER_TICK,
		STATIC_PTP_CONF => STATIC_PTP_CONF,
        ENABLE_METERING => ENABLE_METERING
    )
     port map(
        rst_n_i => rst_n_i,
        clock_i => clock_i,
        phy_rgmii_enet_clk_125m => phy_rgmii_enet_clk_125m,
        phy_rgmii_enet_rx_clk => phy_rgmii_enet_rx_clk,
        phy_rgmii_enet_rx_dv => phy_rgmii_enet_rx_dv,
        phy_rgmii_enet_resetn => phy_rgmii_enet_resetn,
        phy_rgmii_enet_rx_d => phy_rgmii_enet_rx_d,
        phy_rgmii_enet_tx_clk => phy_rgmii_enet_tx_clk,
        phy_rgmii_enet_tx_en => phy_rgmii_enet_tx_en,
        phy_rgmii_enet_tx_d => phy_rgmii_enet_tx_d,
        phy_rmii_ref_clk => phy_rmii_ref_clk,
        phy_rmii_crsdv => phy_rmii_crsdv,
        phy_rmii_rxer => phy_rmii_rxer,
        phy_rmii_rxd => phy_rmii_rxd,
        phy_rmii_txen => phy_rmii_txen,
        phy_rmii_txd => phy_rmii_txd,
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
        pll_512fs_o => pll_512fs_o,
        pll_256fs_rising => pll_256fs_rising,
        pll_256fs_falling => pll_256fs_falling,
        lrclk_o => lrclk_o,
        lrclk_tdm_o => lrclk_tdm_o,
        tdm8in_0_i => tdm8in_0_i,
        tdm8in_1_i => tdm8in_1_i,
        tdm8out_0_o => tdm8out_0_o,
        tdm8out_1_o => tdm8out_1_o,
        spictrl_clk_i => spictrl_clk_i,
        spictrl_mosi_i => spictrl_mosi_i,
        spictrl_cs_n_i => spictrl_cs_n_i,
        spictrl_miso_o => spictrl_miso_o,
        spictrl_irq_n_o => spictrl_irq_n_o,
        adda_nRST => adda_nRST,
        user_led => user_led
    );

end architecture;