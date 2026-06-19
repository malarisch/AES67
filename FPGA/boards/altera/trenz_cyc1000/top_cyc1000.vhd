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
        
		TX_MAX_STREAMS : natural := 2;
		RX_MAX_STREAMS : natural := 2;
		
		
		
		  
    	MIIM_CLOCK_DIVIDER : POSITIVE := 50;

    	--MIIM_PHY_ADDRESS      : t_phy_address := (others => '0');
		

		RX_CHANNELS		: natural := 2;
        
		AUDIO_RX_USE_PARALLEL_INTERFACE : boolean := false;
        RX_BYTE_DEPTH	: natural := 3; -- width for parallel interface
		AUDIO_RX_TDM_OUTPUTS : natural := 1;
		AUDIO_RX_TDM_CHANNELS : natural  := 2;

        TX_CHANNELS		: natural := 2; -- must be multiple of two for i2s, multiple of 8 for tdm8
        
		AUDIO_TX_USE_PARALLEL_INTERFACE : boolean := false;
        TX_BYTE_DEPTH	: natural := 3; -- with for parallel interface
		AUDIO_TX_TDM_INPUTS : natural := 1;
		AUDIO_TX_TDM_CHANNELS : natural  := 2;
        TDM_BCLK_MULT : INTEGER := 64


	);
	PORT 
	(
		c10_resetn :  IN  STD_LOGIC := '1';
		c10_clk12m :  IN  STD_LOGIC;
		

		uart0_tx :  OUT  STD_LOGIC; --bdbus1
		uart0_rx :  IN  STD_LOGIC;  -- bdbus0
		user_led :  OUT  STD_LOGIC_VECTOR(3 DOWNTO 0);

        AIN0 :  OUT STD_LOGIC; -- mosi flash
        AIN1 : OUT STD_LOGIC; -- sck flash
        AIN2: IN STD_LOGIC; -- flash MISO
        AIN3: OUT STD_LOGIC; -- flash cs
        AIN4: OUT STD_LOGIC; -- lrclk
        AIN5: OUT STD_LOGIC; -- tdm out
        AIN6: OUT STD_LOGIC; -- bclk
        AIN7: OUT STD_LOGIC; -- sclk

        D1 : IN STD_LOGIC; -- n/c tdm in
        D2: INOUT STD_LOGIC; -- i2c clk 0
        D3: INOUT STD_LOGIC; -- i2c clk 1
        D4: INOUT STD_LOGIC; -- i2c data 0
        D5: INOUT STD_LOGIC; -- i2c data 1
        D6 : OUT STD_LOGIC; -- mdc
        D7 : INOUT STD_LOGIC; --mdio
        D8 : IN STD_LOGIC; -- refclk
        D9 : IN STD_LOGIC; --crsdv
        D10 : IN STD_LOGIC; -- rx0
        D11 : IN STD_LOGIC; -- rx1
        D12 : OUT STD_LOGIC; -- txen
        D13 : OUT STD_LOGIC; -- tx0
        D14 : OUT STD_LOGIC; -- tx1



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
signal mii_txd : STD_LOGIC_VECTOR(MII_WIDTH -1 downto 0);
signal mii_rxd : STD_LOGIC_VECTOR(MII_WIDTH -1 downto 0);
begin
	tdm_in(0) <= D1;
    AIN5 <= tdm_out(0);
    mii_rxd(0) <= D10;
    mii_rxd(1) <= D11;
    D13 <= mii_txd(0);
    D14 <= mii_txd(1);
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
    ENABLE_METERING => ENABLE_METERING,
    TDM_BCLK_MULT => TDM_BCLK_MULT
  )
  port map (
    rst_n_i => c10_resetn,
    clock_i => c10_clk12m,
    phy_refclk_i => D8,
    phy_mii_enet_rx_clk => '0',
    phy_mii_enet_rx_dv => D9,
    phy_mii_enet_resetn => open,
    phy_mii_enet_rx_d => mii_rxd,
    phy_mii_enet_tx_clk => open,
    phy_mii_enet_tx_en => D12,
    phy_mii_enet_tx_d => mii_txd,
    enet_mdc => D6,
    enet_mdio => D7,
    hbus_rwds => open,
    hbus_dq => open,
    hbus_rstn => open,
    hbus_cs2n => open,
    hbus_clk0_p => open,
    hbus_clk0_n => open,
    uart0_tx => uart0_tx,
    uart0_rx => uart0_rx,
    uart1_rx => '0',
    uart1_tx => open,
    i2c0_scl => D2,
    i2c0_sda => D4,
    i2c1_scl => D3,
    i2c1_sda => D5,
    spiflash_clk => AIN1,
    spiflash_cs => AIN3,
    spiflash_mosi => AIN0,
    spiflash_miso => AIN2,
    pll_512fs_i => '0',
    audioclk_512fs_o => AIN7,
    audioclk_256fs_rising_o => open,
    audioclk_256fs_falling_o => open,
    audioclk_64fs_o => AIN6,
    audioclk_lrclk_o => AIN4,
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
