-- Cyclone 10 LP board top level
-- Instantiates aes67_top, litex_soc, litex_eth_buffer_bridge,
-- RMII Interface; SDRAM

LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.numeric_std.all;

use work.miim_types.all;

ENTITY top_c10_006 IS
	generic (
		SOC_TYPE : string := "LITEX_SPIBONE"; 
		platform : string := "ALTERA";
        MII_WIDTH : integer := 2; -- 2 for rmii, 4 rgmii
		MII_CLK_NS_PER_TICK : integer := 20; -- 20 for rmii, 40 mii, 8 rgmii/gmii
		clk_in_speed : natural := 50; -- input clock speed in mhz (for now only 12, 27, 50)
		ethernet_type	 : string := "RMII"; -- RMII; RGMII
		USE_EXTERNAL_PLL : boolean := FALSE; -- when disabled it will use the nco-generated clocks on the outputs

		TX_SAMPLE_BUFFER_DEPTH : INTEGER := 8; -- must be power of two (media-clock-derived TX write pointer)
		RX_SAMPLE_BUFFER_DEPTH : INTEGER := 256;
		STATIC_PTP_CONF : 		BOOLEAN := true;
        ENABLE_METERING: BOOLEAN := false;

		SYS_CLK_NS_PER_TICK : integer := 8; -- 125 MHz
        
		TX_MAX_STREAMS : natural := 1;
		RX_MAX_STREAMS : natural := 8;
		
		
		
		  
    	MIIM_CLOCK_DIVIDER : POSITIVE := 50;

    	--MIIM_PHY_ADDRESS      : t_phy_address := (others => '0');
		

		RX_CHANNELS		: natural := 8;
        
		AUDIO_RX_USE_PARALLEL_INTERFACE : boolean := false;
        RX_BYTE_DEPTH	: natural := 3; -- width for parallel interface
		AUDIO_RX_TDM_OUTPUTS : natural := 1;
		AUDIO_RX_TDM_CHANNELS : natural  := 2;

        TX_CHANNELS		: natural := 2; -- must be multiple of two for i2s, multiple of 8 for tdm8
        
		AUDIO_TX_USE_PARALLEL_INTERFACE : boolean := false;
        TX_BYTE_DEPTH	: natural := 3; -- with for parallel interface
		AUDIO_TX_TDM_INPUTS : natural := 1;
		AUDIO_TX_TDM_CHANNELS : natural  := 2;
        TDM_BCLK_MULT : INTEGER := 64;
            TDM_I2S_MODE : BOOLEAN := false;
    TDM_FSCLK_50DUTY : BOOLEAN := true;
    PTP_IN_SOFTWARE : BOOLEAN := true


	);
	PORT 
	(
		c10_resetn :  IN  STD_LOGIC := '1';
		c10_clk50m :  IN  STD_LOGIC;
		
        lrclk: OUT STD_LOGIC; -- lrclk
        tdm_out0: OUT STD_LOGIC; -- tdm out
        bclk: OUT STD_LOGIC; -- bclk
        mclk: OUT STD_LOGIC; -- sclk
        
        tdm_in0 : IN STD_LOGIC; -- n/c tdm in
        mdc : OUT STD_LOGIC; -- mdc
        mdio : INOUT STD_LOGIC; --mdio
        refclk : IN STD_LOGIC; -- refclk
        crsdv : IN STD_LOGIC; --crsdv
        rx0 : IN STD_LOGIC; -- rx0
        rx1 : IN STD_LOGIC; -- rx1
        txen : OUT STD_LOGIC; -- txen
        tx0 : OUT STD_LOGIC; -- tx0
        tx1 : OUT STD_LOGIC; -- tx1

        spictrl_clk : IN STD_LOGIC; -- spictrl clk
        spictrl_cs : IN STD_LOGIC; -- spictrl cs
        spictrl_mosi : IN STD_LOGIC; -- spictrl mosi
        spictrl_miso : OUT STD_LOGIC; -- spictrl miso
        spictrl_irq : OUT STD_LOGIC; -- spictrl irq

        led_o : OUT std_logic
	);
END top_c10_006;








ARCHITECTURE bdf_type OF top_c10_006 IS
signal tdm_in : STD_LOGIC_VECTOR (AUDIO_TX_TDM_INPUTS - 1 downto 0);
signal tdm_out : STD_LOGIC_VECTOR (AUDIO_RX_TDM_OUTPUTS - 1 downto 0);
signal mii_txd : STD_LOGIC_VECTOR(MII_WIDTH -1 downto 0);
signal mii_rxd : STD_LOGIC_VECTOR(MII_WIDTH -1 downto 0);
signal uart0_tx_reg : std_logic;
signal uart0_rx_reg : std_logic;
signal uart_ctrl_rx_reg : std_logic;
signal uart_ctrl_tx_reg : std_logic;
signal miso : std_logic;

begin
    led_o <= '1';
	tdm_in(0) <= tdm_in0;
    tdm_out0 <= tdm_out(0);
    mii_rxd(0) <= rx0;
    mii_rxd(1) <= rx1;
    tx0 <= mii_txd(0);
    tx1 <= mii_txd(1);
    uart_soc_gen: if SOC_TYPE /= "LITEX_UARTBONE" generate
		  spictrl_miso <= miso;
    end generate;
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
    TDM_BCLK_MULT => TDM_BCLK_MULT,
    TDM_FSCLK_50DUTY => TDM_FSCLK_50DUTY,
    TDM_I2S_MODE => TDM_I2S_MODE,
    PTP_IN_SOFTWARE => PTP_IN_SOFTWARE
  )
  port map (
    rst_n_i => c10_resetn,
    clock_i => c10_clk50m,
    phy_refclk_i => refclk,
    phy_mii_enet_rx_clk => '0',
    phy_mii_enet_rx_dv => crsdv,
    phy_mii_enet_resetn => open,
    phy_mii_enet_rx_d => mii_rxd,
    phy_mii_enet_tx_clk => open,
    phy_mii_enet_tx_en => txen,
    phy_mii_enet_tx_d => mii_txd,
    enet_mdc => mdc,
    enet_mdio => mdio,
    hbus_rwds => open,
    hbus_dq => open,
    hbus_rstn => open,
    hbus_cs2n => open,
    hbus_clk0_p => open,
    hbus_clk0_n => open,
    pll_512fs_i => '0',
    --audioclk_512fs_o => AIN7,
    audioclk_mclk_o => mclk,
    audioclk_bclk_o => bclk,
    audioclk_lrclk_o => lrclk,
    spibone_clk => spictrl_clk,
    spibone_cs_n => spictrl_cs,
    spibone_miso => miso,
    spibone_mosi => spictrl_mosi,
    spibone_irq_o => spictrl_irq,
    uartbone_rx => uart_ctrl_rx_reg,
    uartbone_tx => uart_ctrl_tx_reg,
    tdm_in => tdm_in,
    tdm_out => tdm_out
  );

END bdf_type;
