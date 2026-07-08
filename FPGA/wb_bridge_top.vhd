
LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.numeric_std.all;

use work.miim_types.all;

use work.audioclks_pkg.all;
ENTITY wb_bridge_top IS
generic (
    	clk_in_speed : natural := 50; -- input clock speed in mhz (for now only 12, 27, 50)
		platform : string := "ALTERA"; -- "ALTERA" or "GOWIN"
		MII_WIDTH : integer := 2;
		ETHERNET_TYPE : string := "RMII";
		SYS_CLK_NS_PER_TICK : integer := 8; -- 125 MHz
        MII_CLK_NS_PER_TICK : integer := 20; -- 50 MHz
    	MIIM_CLOCK_DIVIDER : POSITIVE := 50;
    	MIIM_PHY_ADDRESS      : t_phy_address := (others => '0');
		

		RX_MAX_STREAMS : natural := 8;
        RX_CHANNELS		: natural := 16;
        RX_SAMPLE_BUFFER_DEPTH : natural := 256;
		AUDIO_RX_USE_PARALLEL_INTERFACE : boolean := false;
        RX_BYTE_DEPTH	: natural := 3; -- width for parallel interface
		AUDIO_RX_TDM_OUTPUTS : natural := 2;
		AUDIO_RX_TDM_CHANNELS : natural  := 8;


        TX_MAX_STREAMS : natural := 8;
        TX_CHANNELS		: natural := 16; -- must be multiple of two for i2s, multiple of 8 for tdm8
        TX_SAMPLE_BUFFER_DEPTH : natural := 64; -- must be power of two (media-clock-derived TX write pointer)
		AUDIO_TX_USE_PARALLEL_INTERFACE : boolean := false;
        TX_BYTE_DEPTH	: natural := 3; -- with for parallel interface
		AUDIO_TX_TDM_INPUTS : natural := 2;
		AUDIO_TX_TDM_CHANNELS : natural  := 8;

		USE_EXTERNAL_PLL : BOOLEAN := false;
		ENABLE_METERING: BOOLEAN := false;
        PTP_MOVING_AVERAGE_DEPTH : INTEGER := 8;
        TDM_BCLK_MULT : INTEGER := 256;
        PTP_IN_SOFTWARE : BOOLEAN := false;
        PHY_TYPE : STRING

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
        phy_mii_enet_tx_clk_i :  IN  STD_LOGIC  := '0';
		phy_mii_enet_tx_en :  OUT  STD_LOGIC := '0';
		phy_mii_enet_tx_d :  OUT  STD_LOGIC_VECTOR(MII_WIDTH - 1 DOWNTO 0) := (others => '0');

		enet_mdc :  OUT  STD_LOGIC;
		enet_mdio :  INOUT  STD_LOGIC;




		-- audio clock in from external pll - only used when USE_EXTERNAL_PLL is true
		pll_512fs_i :  IN  STD_LOGIC := '0'; -- gpio 1

		-- audio clocks outputs
		audioclocks_o : out t_audio_clocks;
        selected_audio_clock_o : out t_audio_clocks_selected;
		

		tdm_in :  IN  STD_LOGIC_VECTOR(AUDIO_TX_TDM_INPUTS - 1 downto 0);
		tdm_out :  OUT  STD_LOGIC_VECTOR(AUDIO_RX_TDM_OUTPUTS - 1 downto 0);
		


        rx_sample_register : OUT STD_LOGIC_VECTOR((RX_BYTE_DEPTH * 8) * RX_CHANNELS - 1 downto 0);
        tx_sample_register : IN STD_LOGIC_VECTOR((TX_BYTE_DEPTH * 8) * TX_CHANNELS - 1 downto 0) := (others => '0');
		
		mcu_clk_o : OUT std_logic;
        mcu_clk_90_o : OUT std_logic;
        mcu_irq_o : OUT STD_LOGIC;


        -- wishbone bus
      aes67_wb_ack                              : out std_logic;
      aes67_wb_adr                              : in std_logic_vector(29 downto 0);
      aes67_wb_bte                              : in std_logic_vector(1 downto 0);
      aes67_wb_cti                              : in std_logic_vector(2 downto 0);
      aes67_wb_cyc                              : in std_logic;
      aes67_wb_dat_r                            : out std_logic_vector(31 downto 0);
      aes67_wb_dat_w                            : in std_logic_vector(31 downto 0);
      aes67_wb_err                              : out std_logic;
      aes67_wb_sel                              : in std_logic_vector(3 downto 0);
      aes67_wb_stb                              : in std_logic;
      aes67_wb_we                               : in std_logic;
      dbg_mac_tx_clk_o : out std_logic
	);
END wb_bridge_top;

architecture rtl of wb_bridge_top is

signal sys_clk_125MHz : std_logic;
signal mcu_clk : std_logic;

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
signal phy_mii_enet_tx_d_sig : std_logic_vector(MII_WIDTH - 1 downto 0);
signal phy_mii_enet_rx_d_sig : std_logic_vector(MII_WIDTH - 1 downto 0);

signal phy_mii_enet_tx_en_sig : std_logic;
signal phy_rxclk : std_logic;
signal phy_txclk: std_logic;
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
    ENABLE_METERING => ENABLE_METERING,
    PTP_MOVING_AVERAGE_DEPTH => PTP_MOVING_AVERAGE_DEPTH,
    TDM_BCLK_MULT => TDM_BCLK_MULT,
    PTP_IN_SOFTWARE => PTP_IN_SOFTWARE,
    PHY_TYPE => PHY_TYPE
)
 port map(
    sys_clk_125MHz_i => sys_clk_125MHz,
    enet_clk_i => mii_rx_clock,
    clk_mcu_i => mcu_clk,
    rst_n => rst_n,
    phy_mii_rx_clk_in => phy_rxclk,
    phy_mii_tx_clk_in => phy_txclk,
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
    audioclocks_o => audioclocks_o,
    selected_audio_clock_o => selected_audio_clock_o,
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
    eth_irq_o => mcu_irq_o,
    dbg_mac_tx_clk_o => dbg_mac_tx_clk_o
);

  sysclk_pll_gen_inst: entity work.sysclk_pll_gen
   generic map(
      platform => platform,
      clk_in_speed => clk_in_speed
  )
   port map(
      clock_i => clock_i,
      rst_n_i => rst_n_i,
      sys_clk_125MHz_o => sys_clk_125MHz,
      mcu_clk_o => mcu_clk,
      mcu_clk2_o => mcu_clk_90_o,
      locked_o => sysclk_pll_locked
  );
  mcu_clk_o <= mcu_clk;
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
      phy_mii_enet_tx_clk_i => phy_mii_enet_tx_clk_i,
      phy_mii_enet_tx_en => phy_mii_enet_tx_en_sig,
      phy_mii_enet_tx_d => phy_mii_enet_tx_d_sig,
      phy_clk_rx_o => phy_rxclk,
      phy_clk_tx_o => phy_txclk,
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
