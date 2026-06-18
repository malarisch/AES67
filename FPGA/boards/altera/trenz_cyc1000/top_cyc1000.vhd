-- Cyclone 10 LP board top level
-- Instantiates aes67_top, litex_soc, litex_eth_buffer_bridge,
-- RMII Interface; SDRAM

LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.numeric_std.all;

use work.miim_types.all;

ENTITY top_cyc1000 IS
	generic (
		SOC_TYPE : string := "LITEX_SDRAM"; 
		platform : string := "ALTERA";
        MII_WIDTH : integer := 2; -- 2 for rmii, 4 rgmii
		MII_CLK_NS_PER_TICK : integer := 20; -- 20 for rmii, 40 mii, 8 rgmii/gmii
		clk_in_speed : natural := 12; -- input clock speed in mhz (for now only 12, 27, 50)
		ethernet_type	 : string := "RMII"; -- RMII; RGMII
		USE_EXTERNAL_PLL : boolean := FALSE; -- when disabled it will use the nco-generated clocks on the outputs

		TX_SAMPLE_BUFFER_DEPTH : INTEGER := 64; -- must be power of two (media-clock-derived TX write pointer)
		RX_SAMPLE_BUFFER_DEPTH : INTEGER := 64;
		STATIC_PTP_CONF : 		BOOLEAN := true;
        ENABLE_METERING: BOOLEAN := false;

		SYS_CLK_NS_PER_TICK : integer := 8; -- 125 MHz
        
		TX_MAX_STREAMS : natural := 8;
		RX_MAX_STREAMS : natural := 8;
		
		
		
		  
    	MIIM_CLOCK_DIVIDER : POSITIVE := 50;

    	--MIIM_PHY_ADDRESS      : t_phy_address := (others => '0');
		

		RX_CHANNELS		: natural := 16;
        
		AUDIO_RX_USE_PARALLEL_INTERFACE : boolean := false;
        RX_BYTE_DEPTH	: natural := 3; -- width for parallel interface
		AUDIO_RX_TDM_OUTPUTS : natural := 2;
		AUDIO_RX_TDM_CHANNELS : natural  := 8;

        TX_CHANNELS		: natural := 8; -- must be multiple of two for i2s, multiple of 8 for tdm8
        
		AUDIO_TX_USE_PARALLEL_INTERFACE : boolean := false;
        TX_BYTE_DEPTH	: natural := 3; -- with for parallel interface
		AUDIO_TX_TDM_INPUTS : natural := 2;
		AUDIO_TX_TDM_CHANNELS : natural  := 8
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
signal tdm_in : STD_LOGIC_VECTOR (AUDIO_TX_TDM_INPUTS - 1 downto 0);
signal tdm_out : STD_LOGIC_VECTOR (AUDIO_RX_TDM_OUTPUTS - 1 downto 0);
begin
	    tdm_in(0) <= tdm8in_0_i;
    tdm_in(1) <= tdm8in_1_i;
    tdm8out_0_o <= tdm_out(0);
    tdm8out_1_o <= tdm_out(1);
	soc_top_inst : entity work.soc_top
  generic map (
    clk_in_speed => clk_in_speed,
    platform => platform,
    SOC_TYPE => SOC_TYPE,
    MII_WIDTH => MII_WIDTH,
    ETHERNET_TYPE => ETHERNET_TYPE,
    SYS_CLK_NS_PER_TICK => SYS_CLK_NS_PER_TICK,
    MII_CLK_NS_PER_TICK => MII_CLK_NS_PER_TICK,
    MIIM_CLOCK_DIVIDER => MIIM_CLOCK_DIVIDER,
    MIIM_PHY_ADDRESS => "00001", --lan8720a is on 1,
    RX_MAX_STREAMS => RX_MAX_STREAMS,
    RX_CHANNELS => RX_CHANNELS,
    RX_SAMPLE_BUFFER_DEPTH => RX_SAMPLE_BUFFER_DEPTH,
    AUDIO_RX_USE_PARALLEL_INTERFACE => AUDIO_RX_USE_PARALLEL_INTERFACE,
    RX_BYTE_DEPTH => RX_BYTE_DEPTH,
    AUDIO_RX_TDM_OUTPUTS => AUDIO_RX_TDM_OUTPUTS,
    AUDIO_RX_TDM_CHANNELS => AUDIO_RX_TDM_CHANNELS,
    TX_MAX_STREAMS => TX_MAX_STREAMS,
    TX_CHANNELS => TX_CHANNELS,
    TX_SAMPLE_BUFFER_DEPTH => TX_SAMPLE_BUFFER_DEPTH,
    AUDIO_TX_USE_PARALLEL_INTERFACE => AUDIO_TX_USE_PARALLEL_INTERFACE,
    TX_BYTE_DEPTH => TX_BYTE_DEPTH,
    AUDIO_TX_TDM_INPUTS => AUDIO_TX_TDM_INPUTS,
    AUDIO_TX_TDM_CHANNELS => AUDIO_TX_TDM_CHANNELS,
    USE_EXTERNAL_PLL => USE_EXTERNAL_PLL,
    ENABLE_METERING => ENABLE_METERING
  )
  port map (
    rst_n_i => c10_resetn,
    clock_i => c10_clk12m,
    phy_refclk_i => phy_rmii_ref_clk,
    phy_mii_enet_rx_clk => phy_rmii_ref_clk,
    phy_mii_enet_rx_dv => phy_rmii_crsdv,
    phy_mii_enet_resetn => open,
    phy_mii_enet_rx_d => phy_rmii_rxd,
    phy_mii_enet_tx_clk => open,
    phy_mii_enet_tx_en => phy_rmii_txen,
    phy_mii_enet_tx_d => phy_rmii_txd,
    enet_mdc => enet_mdc,
    enet_mdio => enet_mdio,
    hbus_rwds => open,
    hbus_dq => open,
    hbus_rstn => open,
    hbus_cs2n => open,
    hbus_clk0_p => open,
    hbus_clk0_n => open,
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
    audioclk_64fs_o => open,
    audioclk_lrclk_o => lrclk_tdm_o,
    tdm_in => tdm_in,
    tdm_out => tdm_out,
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
    spibone_clk => open,
    spibone_cs_n => open,
    spibone_miso => open,
    spibone_mosi => open,
    spibone_irq_o => open
  );

END bdf_type;
