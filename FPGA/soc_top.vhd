
LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.numeric_std.all;

use work.miim_types.all;

ENTITY soc_top IS
generic (
    	clk_in_speed : natural := 50; -- input clock speed in mhz (for now only 12, 27, 50)
		platform : string := "ALTERA"; -- "ALTERA" or "GOWIN"
		MII_WIDTH : integer := 2;
		ETHERNET_TYPE : string := "RMII";
		SYS_CLK_NS_PER_TICK : integer := 8; -- 125 MHz
        MII_CLK_NS_PER_TICK : integer := 20; -- 50 MHz
		TX_MAX_STREAMS : natural := 8;
		RX_MAX_STREAMS : natural := 8;
		
		
		
		  
    	MIIM_CLOCK_DIVIDER : POSITIVE := 50;

    	MIIM_PHY_ADDRESS      : t_phy_address := (others => '0');
		

		RX_CHANNELS		: natural := 16;
        RX_SAMPLE_BUFFER_DEPTH : natural := 256;
		AUDIO_RX_USE_PARALLEL_INTERFACE : boolean := false;
        RX_BYTE_DEPTH	: natural := 3; -- width for parallel interface
		AUDIO_RX_TDM_OUTPUTS : natural := 2;
		AUDIO_RX_TDM_CHANNELS : natural  := 8;

        TX_CHANNELS		: natural := 16; -- must be multiple of two for i2s, multiple of 8 for tdm8
        TX_SAMPLE_BUFFER_DEPTH : natural := 64; -- must be power of two (media-clock-derived TX write pointer)
		AUDIO_TX_USE_PARALLEL_INTERFACE : boolean := false;
        TX_BYTE_DEPTH	: natural := 3; -- with for parallel interface
		AUDIO_TX_TDM_INPUTS : natural := 2;
		AUDIO_TX_TDM_CHANNELS : natural  := 8;

		USE_EXTERNAL_PLL : BOOLEAN := true;
		ENABLE_METERING: BOOLEAN := true

	);
	
	PORT
	(
		rst_n_i :  IN  STD_LOGIC;
		clock_i :  IN  STD_LOGIC;

		-- rgmii if (use when ethernet_type  = RGMII)
		phy_refclk_i :  IN  STD_LOGIC := '0';
		phy_mii_enet_rx_clk :  IN  STD_LOGIC  := '0';
		phy_mii_enet_rx_dv :  IN  STD_LOGIC := '0';
		phy_mii_enet_resetn :  OUT  STD_LOGIC := '0';
		phy_mii_enet_rx_d :  IN  STD_LOGIC_VECTOR(MII_WIDTH - 1 DOWNTO 0)  := (others => '0');
		phy_mii_enet_tx_clk :  OUT  STD_LOGIC  := '0';
		phy_mii_enet_tx_en :  OUT  STD_LOGIC := '0';
		phy_mii_enet_tx_d :  OUT  STD_LOGIC_VECTOR(MII_WIDTH - 1 DOWNTO 0) := (others => '0');

		enet_mdc :  OUT  STD_LOGIC;
		enet_mdio :  INOUT  STD_LOGIC;
		
		-- hyperram for litex hram target
		hbus_rwds :  INOUT  STD_LOGIC := '0';
		hbus_dq :  INOUT  STD_LOGIC_VECTOR(7 DOWNTO 0) := (others => '0') ;
		hbus_rstn :  OUT  STD_LOGIC  := '0';
		hbus_cs2n :  OUT  STD_LOGIC := '0';
		hbus_clk0_p :  OUT  STD_LOGIC := '0';
		hbus_clk0_n :  OUT  STD_LOGIC := '0';


		-- litex console
		uart0_tx :  OUT  STD_LOGIC  := '0';
		uart0_rx :  IN  STD_LOGIC := '0';

		-- litex uart interface for adda boards
		uart1_rx :  IN  STD_LOGIC  := '0';
		uart1_tx :  OUT  STD_LOGIC := '0';

		-- litex i2c master
		i2c0_scl :  INOUT  STD_LOGIC := '0';
		i2c0_sda :  INOUT  STD_LOGIC := '0';
		i2c1_scl :  INOUT  STD_LOGIC := '0';
		i2c1_sda :  INOUT  STD_LOGIC := '0';


		-- litex spi flash
		spiflash_clk :  OUT  STD_LOGIC := '0';
		spiflash_cs :  OUT  STD_LOGIC := '0';
		spiflash_mosi :  OUT  STD_LOGIC := '0';
		spiflash_miso :  IN  STD_LOGIC := '0';


		-- audio clock in from external pll - only used when USE_EXTERNAL_PLL is true
		pll_512fs_i :  IN  STD_LOGIC := '0'; -- gpio 1

		-- audio clocks outputs
		audioclk_512fs_o :  OUT  STD_LOGIC;
		audioclk_256fs_rising_o : OUT STD_LOGIC;
		audioclk_256fs_falling_o :  OUT  STD_LOGIC; -- gpio 12
        audioclk_64fs_o : OUT STD_LOGIC;
		audioclk_lrclk_o :  OUT  STD_LOGIC;
		

		tdm_in :  IN  STD_LOGIC_VECTOR(AUDIO_TX_TDM_INPUTS - 1 downto 0);
		tdm_out :  OUT  STD_LOGIC_VECTOR(AUDIO_RX_TDM_OUTPUTS - 1 downto 0);
		


        rx_sample_register : OUT STD_LOGIC_VECTOR((RX_BYTE_DEPTH * 8) * RX_CHANNELS - 1 downto 0);
        tx_sample_register : IN STD_LOGIC_VECTOR((TX_BYTE_DEPTH * 8) * TX_CHANNELS - 1 downto 0) := (others => '0')
		
		
	);
END soc_top;

architecture rtl of soc_top is
		signal aes67_wb_ack		:	  STD_LOGIC;
		signal aes67_wb_adr		:	  STD_LOGIC_VECTOR(29 DOWNTO 0);
		signal aes67_wb_bte		:	  STD_LOGIC_VECTOR(1 DOWNTO 0);
		signal aes67_wb_cti		:	  STD_LOGIC_VECTOR(2 DOWNTO 0);
		signal aes67_wb_cyc		:	  STD_LOGIC;
		signal aes67_wb_dat_r		:	  STD_LOGIC_VECTOR(31 DOWNTO 0);
		signal aes67_wb_dat_w		:	  STD_LOGIC_VECTOR(31 DOWNTO 0);
		signal aes67_wb_err		:	  STD_LOGIC;
		signal aes67_wb_sel		:	  STD_LOGIC_VECTOR(3 DOWNTO 0);
		signal aes67_wb_stb		:	  STD_LOGIC;
		signal aes67_wb_we		:	  STD_LOGIC;
COMPONENT litex_soc_cyclone10
	PORT
	(
		aes67_wb_ack		:	 IN STD_LOGIC;
		aes67_wb_adr		:	 OUT STD_LOGIC_VECTOR(29 DOWNTO 0);
		aes67_wb_bte		:	 OUT STD_LOGIC_VECTOR(1 DOWNTO 0);
		aes67_wb_cti		:	 OUT STD_LOGIC_VECTOR(2 DOWNTO 0);
		aes67_wb_cyc		:	 OUT STD_LOGIC;
		aes67_wb_dat_r		:	 IN STD_LOGIC_VECTOR(31 DOWNTO 0);
		aes67_wb_dat_w		:	 OUT STD_LOGIC_VECTOR(31 DOWNTO 0);
		aes67_wb_err		:	 IN STD_LOGIC;
		aes67_wb_sel		:	 OUT STD_LOGIC_VECTOR(3 DOWNTO 0);
		aes67_wb_stb		:	 OUT STD_LOGIC;
		aes67_wb_we		:	 OUT STD_LOGIC;
		clk_mac_rx		:	 IN STD_LOGIC;
		clk_mac_tx		:	 IN STD_LOGIC;
		clk_sys		:	 IN STD_LOGIC;
		eth_buf_irq		:	 IN STD_LOGIC;
		hyperram_clk		:	 OUT STD_LOGIC;
		hyperram_cs_n		:	 OUT STD_LOGIC;
		hyperram_dq		:	 INOUT STD_LOGIC_VECTOR(7 DOWNTO 0);
		hyperram_rst_n		:	 OUT STD_LOGIC;
		hyperram_rwds		:	 INOUT STD_LOGIC;
		i2c0_scl		:	 INOUT STD_LOGIC;
		i2c0_sda		:	 INOUT STD_LOGIC;
		i2c1_scl		:	 INOUT STD_LOGIC;
		i2c1_sda		:	 INOUT STD_LOGIC;
		serial1_rx		:	 IN STD_LOGIC;
		serial1_tx		:	 OUT STD_LOGIC;
		serial_rx		:	 IN STD_LOGIC;
		serial_tx		:	 OUT STD_LOGIC;
		spi_clk		:	 OUT STD_LOGIC;
		spi_cs_n		:	 OUT STD_LOGIC;
		spi_miso		:	 IN STD_LOGIC;
		spi_mosi		:	 OUT STD_LOGIC;
		spiflash_clk		:	 OUT STD_LOGIC;
		spiflash_cs_n		:	 OUT STD_LOGIC;
		spiflash_miso		:	 IN STD_LOGIC;
		spiflash_mosi		:	 OUT STD_LOGIC;
		sys_clk_out		:	 OUT STD_LOGIC
	);
END COMPONENT;
signal hbus_clk : std_logic;
signal sys_clk_125MHz : std_logic;
signal mcu_clk : std_logic;
signal mcu_clk_90 : std_logic;

signal mac_speed_i             : std_logic_vector(1 downto 0);
signal mii_rx_clock          : std_logic;
signal mii_tx_clock          : std_logic;
signal mii_rx_err            : std_logic;
signal mii_rx_dv             : std_logic;
signal mii_rxd               : std_logic_vector(7 downto 0);
signal mii_tx_err            : std_logic;
signal mii_tx_en             : std_logic;
signal mii_txd               : std_logic_vector(7 downto 0);
signal rst_n : std_logic;
signal sysclk_pll_locked : std_logic;
signal eth_buf_irq : std_logic;
signal phy_mii_enet_tx_d_sig : std_logic_vector(MII_WIDTH - 1 downto 0);
signal phy_mii_enet_rx_d_sig : std_logic_vector(MII_WIDTH - 1 downto 0);

signal phy_mii_enet_tx_en_sig : std_logic;
begin
    rst_n <= rst_n_i and sysclk_pll_locked;
    phy_mii_enet_tx_d <= phy_mii_enet_tx_d_sig;
    phy_mii_enet_rx_d_sig <= phy_mii_enet_rx_d;
    phy_mii_enet_tx_en <= phy_mii_enet_tx_en_sig;

aes67_wb_bridge_inst: entity work.aes67_wb_bridge
 generic map(
    MII_WIDTH => MII_WIDTH,
    ETHERNET_TYPE => ETHERNET_TYPE,
    SYS_CLK_NS_PER_TICK => SYS_CLK_NS_PER_TICK,
    MII_CLK_NS_PER_TICK => MII_CLK_NS_PER_TICK,
    TX_MAX_STREAMS => TX_MAX_STREAMS,
    RX_MAX_STREAMS => RX_MAX_STREAMS,
    RX_CHANNELS => RX_CHANNELS,
    TX_CHANNELS => TX_CHANNELS,
    RX_BYTE_DEPTH => RX_BYTE_DEPTH,
    TX_BYTE_DEPTH => TX_BYTE_DEPTH,
    RX_SAMPLE_BUFFER_DEPTH => RX_SAMPLE_BUFFER_DEPTH,
    TX_SAMPLE_BUFFER_DEPTH => TX_SAMPLE_BUFFER_DEPTH,
    MIIM_CLOCK_DIVIDER => MIIM_CLOCK_DIVIDER,
    MIIM_PHY_ADDRESS => MIIM_PHY_ADDRESS,
    AUDIO_RX_USE_PARALLEL_INTERFACE => AUDIO_RX_USE_PARALLEL_INTERFACE,
    AUDIO_RX_TDM_OUTPUTS => AUDIO_RX_TDM_OUTPUTS,
    AUDIO_RX_TDM_CHANNELS => AUDIO_RX_TDM_CHANNELS,
    AUDIO_TX_USE_PARALLEL_INTERFACE => AUDIO_TX_USE_PARALLEL_INTERFACE,
    AUDIO_TX_TDM_INPUTS => AUDIO_TX_TDM_INPUTS,
    AUDIO_TX_TDM_CHANNELS => AUDIO_TX_TDM_CHANNELS,
    USE_EXTERNAL_PLL => USE_EXTERNAL_PLL,
    ENABLE_METERING => ENABLE_METERING
)
 port map(
    sys_clk_125MHz_i => sys_clk_125MHz,
    enet_clk_i => phy_refclk_i,
    clk_mcu_i => mcu_clk,
    rst_n => rst_n,
    mac_resetn_i => rst_n,
    phy_mii_rx_clk_in => mii_rx_clock,
    phy_mii_tx_clk_in => mii_tx_clock,
    phy_mii_tx_data_in => phy_mii_enet_tx_d_sig(MII_WIDTH -1 downto 0),
    phy_mii_rx_data_in => phy_mii_enet_rx_d_sig(MII_WIDTH -1 downto 0),
    phy_mii_tx_en_i => phy_mii_enet_tx_en_sig,
    phy_mii_rx_en_i => phy_mii_enet_rx_dv,
    mii_rx_clock_i => mii_rx_clock,
    mii_tx_clock_i => mii_tx_clock,
    mii_rx_err_i => mii_rx_err,
    mii_rx_dv_i => mii_rx_dv,
    mii_rxd_i => mii_rxd,
    mii_tx_err_o => mii_tx_err,
    mii_tx_en_o => mii_tx_en,
    mii_txd_o => mii_txd,
    enet_mdio => enet_mdio,
    enet_mdc => enet_mdc,
    pll_512fs_i => pll_512fs_i,
    wc_512fs_o => audioclk_512fs_o,
    pll_64fs_o => audioclk_64fs_o,
    pll_48k_fs_tdm_o => audioclk_lrclk_o,
    pll_256fs_rising_o => audioclk_256fs_rising_o,
    pll_256fs_falling_o => audioclk_256fs_falling_o,
    aes67_wb_ack => aes67_wb_ack,
    aes67_wb_adr => aes67_wb_adr,
    aes67_wb_bte => aes67_wb_bte,
    aes67_wb_cti => aes67_wb_cti,
    aes67_wb_cyc => aes67_wb_cyc,
    aes67_wb_dat_r => aes67_wb_dat_r,
    aes67_wb_dat_w => aes67_wb_dat_w,
    aes67_wb_err => aes67_wb_err,
    aes67_wb_sel => aes67_wb_sel,
    aes67_wb_stb => aes67_wb_stb,
    aes67_wb_we => aes67_wb_we,
    tdm8out_o => tdm_out,
    rx_sample_register => rx_sample_register,
    tdm8in_i => tdm_in,
    tx_sample_register => tx_sample_register,
    eth_irq_o => eth_buf_irq
);
    hbus_clk0_p <= hbus_clk;
    hbus_clk0_n <= not hbus_clk;
  litex_soc_cyclone10_inst: litex_soc_cyclone10
   port map(
      aes67_wb_ack => aes67_wb_ack,
      aes67_wb_adr => aes67_wb_adr,
      aes67_wb_bte => aes67_wb_bte,
      aes67_wb_cti => aes67_wb_cti,
      aes67_wb_cyc => aes67_wb_cyc,
      aes67_wb_dat_r => aes67_wb_dat_r,
      aes67_wb_dat_w => aes67_wb_dat_w,
      aes67_wb_err => aes67_wb_err,
      aes67_wb_sel => aes67_wb_sel,
      aes67_wb_stb => aes67_wb_stb,
      aes67_wb_we => aes67_wb_we,
      clk_mac_rx => mii_rx_clock,
      clk_mac_tx => mii_tx_clock,
      clk_sys => mcu_clk,
      eth_buf_irq => eth_buf_irq,
      hyperram_clk => hbus_clk,
      hyperram_cs_n => hbus_cs2n,
      hyperram_dq => hbus_dq,
      hyperram_rst_n => hbus_rstn,
      hyperram_rwds => hbus_rwds,
      i2c0_scl => i2c0_scl,
      i2c0_sda => i2c0_sda,
      i2c1_scl => i2c1_scl,
      i2c1_sda => i2c1_sda,
      serial1_rx => uart1_rx,
      serial1_tx => uart1_tx,
      serial_rx => uart0_rx,
      serial_tx => uart0_tx,
      --spi_clk => spi_clk,
      --spi_cs_n => spi_cs_n,
      spi_miso => '0',
      --spi_mosi => spi_mosi,
      spiflash_clk => spiflash_clk,
      spiflash_cs_n => spiflash_cs,
      spiflash_miso => spiflash_miso,
      spiflash_mosi => spiflash_mosi
  );  
  sysclk_pll_gen_inst: entity work.sysclk_pll_gen
   generic map(
      platform => platform,
      clk_in_speed => clk_in_speed
  )
   port map(
      clock_i => clock_i,
      rst_n_i => rst_n,
      sys_clk_125MHz_o => sys_clk_125MHz,
      mcu_clk_o => mcu_clk,
      mcu_clk2_o => mcu_clk_90,
      locked_o => sysclk_pll_locked
  );
  mii_converters_inst: entity work.mii_converters
   generic map(
      MII_WIDTH => MII_WIDTH,
      ETHERNET_TYPE => ETHERNET_TYPE,
      PLATFORM => PLATFORM
  )
   port map(
      rst_n_i => rst_n,
      phy_refclk_i => phy_refclk_i,
      phy_mii_enet_rx_clk => phy_mii_enet_rx_clk,
      phy_mii_enet_rx_dv => phy_mii_enet_rx_dv,
      phy_mii_enet_rx_err => '0',
      phy_mii_enet_resetn => phy_mii_enet_resetn,
      phy_mii_enet_rx_d => phy_mii_enet_rx_d,
      phy_mii_enet_tx_clk => phy_mii_enet_tx_clk,
      phy_mii_enet_tx_en => phy_mii_enet_tx_en_sig,
      phy_mii_enet_tx_d => phy_mii_enet_tx_d_sig,
      mac_speed_i => mac_speed_i,
      mii_rx_clock_o => mii_rx_clock,
      mii_tx_clock_o => mii_tx_clock,
      mii_rx_err_o => mii_rx_err,
      mii_rx_dv_o => mii_rx_dv,
      mii_rxd_o => mii_rxd,
      mii_tx_err_i => mii_tx_err,
      mii_tx_en_i => mii_tx_en,
      mii_txd_i => mii_txd
  );

end architecture;
