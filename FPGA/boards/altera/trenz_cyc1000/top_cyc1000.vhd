-- Cyclone 10 LP board top level
-- Instantiates aes67_top, litex_soc, litex_eth_buffer_bridge,
-- RMII Interface; SDRAM

LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.numeric_std.all;

use work.miim_types.all;

ENTITY top_cyc1000 IS
	generic (
		MIIM_PHY_ADDRESS      : t_phy_address := "00001"; --lan8720a is on 1
		soctype : string := "litex_c10_sdram"; -- spi or litex_c10_hram or litex_c10_sdram or litex_tang_primer_20k
		platform : string := "ALTERA";
		clk_in_speed : natural := 12; -- input clock speed in mhz (for now only 12, 27, 50)
		ethernet_type	 : string := "RMII"; -- RMII; RGMII
		MII_WIDTH : integer := 2; -- 2 for rmii, 4 rgmii or mii, 8 gmii
		MII_CLK_NS_PER_TICK : integer := 20; -- 20 for rmii, 40 mii, 8 rgmii/gmii
		USE_EXTERNAL_PLL : string := "FALSE"; -- when disabled it will use the nco-generated clocks on the outputs
		FPGAVERSIONMSB : integer := 1;
        FPGAVERSIONLSB : integer := 123;
        TXSTREAMS      : integer := 8;
        RXSTREAMS       : INTEGER := 8;
        TXCHANNELS      : INTEGER := 16;
        RXCHANNELS      : INTEGER := 16;
        BITDEPTH        : INTEGER := 24;
        SAMPLERATE      : INTEGER := 48;
		STATIC_PTP_CONF : 		string := "FALSE";
		TX_SAMPLE_BUFFER_DEPTH : INTEGER := 8;
		RX_SAMPLE_BUFFER_DEPTH : INTEGER := 8
	);
	PORT 
	(
		c10_resetn :  IN  STD_LOGIC := '1';
		c10_clk12m :  IN  STD_LOGIC;
		
		enet_mdio :  INOUT  STD_LOGIC; -- pmod 8
		enet_mdc :  OUT  STD_LOGIC; -- ain7
		uart0_tx :  OUT  STD_LOGIC; --bdbus1

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
		lrclk_tdm_o :  OUT  STD_LOGIC;
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
		MIIM_PHY_ADDRESS => MIIM_PHY_ADDRESS,
		MII_WIDTH => MII_WIDTH,
		MII_CLK_NS_PER_TICK => MII_CLK_NS_PER_TICK
	)
	 port map(
		rst_n_i => '1',
		clock_i => c10_clk12m,
		phy_rgmii_enet_clk_125m => open,
		phy_rgmii_enet_rx_clk => open,
		phy_rgmii_enet_rx_dv => open,
		phy_rgmii_enet_resetn => open,
		phy_rgmii_enet_rx_d => open,
		phy_rgmii_enet_tx_clk => open,
		phy_rgmii_enet_tx_en => open,
		phy_rgmii_enet_tx_d => open,
		phy_rmii_ref_clk => phy_rmii_ref_clk,
		phy_rmii_crsdv => phy_rmii_crsdv,
		phy_rmii_rxer => '0',
		phy_rmii_rxd => phy_rmii_rxd,
		phy_rmii_txen => phy_rmii_txen,
		phy_rmii_txd => phy_rmii_txd,
		enet_mdc => enet_mdc,
		enet_mdio => enet_mdio,
		hbus_rwds => open,
		hbus_dq => open,
		hbus_rstn => open,
		hbus_cs2n => open,
		hbus_clk0_p => open,
		hbus_clk0_n => open,
		sdram_a => sdram_a,
		sdram_ba => sdram_ba,
		sdram_cas_n => sdram_cas_n,
		sdram_cke => sdram_cke,
		sdram_clock => sdram_clock,
		sdram_cs_n => sdram_cs_n,
		sdram_dm => sdram_dm,
		sdram_dq => sdram_dq,
		sdram_ras_n => sdram_ras_n,
		sdram_we_n => sdram_we_n,
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
		lrclk_o => open,
		lrclk_tdm_o => lrclk_tdm_o,
		tdm8in_0_i => tdm8in_0_i,
		tdm8in_1_i => tdm8in_1_i,
		tdm8out_0_o => tdm8out_0_o,
		tdm8out_1_o => tdm8out_1_o,
		spictrl_clk_i => open,
		spictrl_mosi_i => open,
		spictrl_cs_n_i => open,
		spictrl_miso_o => open,
		spictrl_irq_n_o => open,
		adda_nRST => adda_nRST,
		user_led => user_led
	);
END bdf_type;
