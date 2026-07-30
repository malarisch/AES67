
LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.numeric_std.all;

use work.miim_types.all;
use work.audioclks_pkg.all;
use work.system_cfg_pkg.all;
ENTITY cyc2top IS
	generic (
		syscfg: t_global_system_cfg := global_system_cfg_mi
	);
	PORT 
	(
		c2_resetn :  IN  STD_LOGIC := '1';
		phy_rstn_o : out std_logic := '1';
		
        fs_adc: OUT STD_LOGIC; -- lrclk
		  fs_dac: OUT STD_LOGIC; -- lrclk
		  fs_ext : OUT STD_LOGIC;
        tdm_out: OUT STD_LOGIC_VECTOR(3 downto 0); -- tdm out
        bclk_adc: OUT STD_LOGIC; -- bclk
		  bclk_dac : out std_logic;
        mclk: OUT STD_LOGIC; -- sclk
        
        tdm_in : IN STD_LOGIC_VECTOR(3 downto 0);
        mdc : OUT STD_LOGIC; -- mdc
        mdio : INOUT STD_LOGIC; --mdio
        rxclk_i : IN STD_LOGIC; -- refclk
        txclk_i : IN STD_LOGIC;
        crsdv : IN STD_LOGIC_VECTOR(3 downto 0); --crsdv
        phy0rx : IN STD_LOGIC_VECTOR(3 downto 0);
		  phy1rx : IN STD_LOGIC_VECTOR(3 downto 0);
		  phy2rx : IN STD_LOGIC_VECTOR(3 downto 0);
		  phy3rx : IN STD_LOGIC_VECTOR(3 downto 0);
        txen : OUT STD_LOGIC_VECTOR(3 downto 0); -- txen
        phy0tx : OUT STD_LOGIC_VECTOR(3 downto 0);
		  phy1tx : OUT STD_LOGIC_VECTOR(3 downto 0);
		  phy2tx : OUT STD_LOGIC_VECTOR(3 downto 0);
		  phy3tx : OUT STD_LOGIC_VECTOR(3 downto 0);
		  tx_err_o : out STD_LOGIC_VECTOR(3 downto 0) := (others => '0');

        spictrl_clk : IN STD_LOGIC; -- spictrl clk
        spictrl_cs : IN STD_LOGIC; -- spictrl cs
        spictrl_mosi : IN STD_LOGIC; -- spictrl mosi
        spictrl_miso : OUT STD_LOGIC; -- spictrl miso
        spictrl_irq : OUT STD_LOGIC; -- spictrl irq
        dbg_o : out STD_LOGIC_VECTOR(7 downto 0);
		  INIT_DONE : out STD_LOGIC
	);
END cyc2top;








ARCHITECTURE bdf_type OF cyc2top IS

signal tdm_out_reg : STD_LOGIC_VECTOR(syscfg.AUDIO_CONFIG.RX_DA_CFG.TDM_PINS - 1 downto 0);
signal tdm_in_reg : STD_LOGIC_VECTOR(syscfg.AUDIO_CONFIG.TX_AD_CFG.TDM_PINS - 1 downto 0);
signal uart0_tx_reg : std_logic;
signal uart0_rx_reg : std_logic;
signal uart_ctrl_rx_reg : std_logic;
signal uart_ctrl_tx_reg : std_logic;
signal miso : std_logic;
signal fs : std_logic;
signal bclk : std_logic;
signal phy_rx : std_logic_vector(3 downto 0);
signal phy_tx : std_logic_vector(3 downto 0);
signal phy_rxdv : std_logic;
signal phy_txen : std_logic;
signal mclk_reg : std_logic;
signal dbg_mac_tx_clk_o : std_logic;

begin
	tdm_in_reg <= tdm_in(syscfg.AUDIO_CONFIG.TX_AD_CFG.TDM_PINS - 1 downto 0);
	INIT_DONE <= '1';
    tx_err_o <= (others => '0');
	tdm_out(syscfg.AUDIO_CONFIG.RX_DA_CFG.TDM_PINS - 1 downto 0) <= tdm_out_reg;
	mclk <= mclk_reg;
	 phy_rstn_o <= '1';
	 bclk_adc <= not bclk;
	 bclk_dac <= not bclk;
	 txen(0) <= phy_txen;
	 phy0tx <= phy_tx;
	 phy1tx <= phy_tx;
	 phy2tx <= phy_tx;
	 phy3tx <= phy_tx;
	 phy_rxdv <= crsdv(0); -- or crsdv(1) or crsdv(2) or crsdv(3);
	 phy_rx <= phy0rx(0) & phy0rx(1) & phy0rx(2) &phy0rx(3); -- or phy1rx or phy2rx or phy3rx;
     --dbg_o <= mclk_reg & not bclk & fs & tdm_out_reg & '0';
    uart_soc_gen: if syscfg.SOC_TYPE /= LITEX_UARTBONE generate
		  spictrl_miso <= miso;
    end generate;
	 fs_adc <= fs;
	 fs_dac <= fs;
	soc_top_inst : entity work.soc_top
  generic map (
    syscfg => syscfg
  )
  port map (
    rst_n_i => '1',
    clock_i => txclk_i,
    phy_mii_enet_rx_clk => rxclk_i,
    phy_mii_enet_rx_dv => phy_rxdv,
    phy_mii_enet_resetn => open,
    phy_mii_enet_rx_d => phy_rx,
    phy_mii_enet_tx_clk => open,
    phy_mii_enet_tx_clk_i => txclk_i,
    phy_mii_enet_tx_en => phy_txen,
    phy_mii_enet_tx_d => phy_tx,
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
    audioclk_mclk_o => mclk_reg,
    audioclk_bclk_o => bclk,
    audioclk_lrclk_adc_o => fs,
    --audioclk_lrclk_dac_o => fs_dac,
    audioclk_lrclk_ext_o => fs_ext,
    spibone_clk => spictrl_clk,
    spibone_cs_n => spictrl_cs,
    spibone_miso => miso,
    spibone_mosi => spictrl_mosi,
    spibone_irq_o => spictrl_irq,
    uartbone_rx => uart_ctrl_rx_reg,
    uartbone_tx => uart_ctrl_tx_reg,
    tdm_in => tdm_in_reg,
    tdm_out => tdm_out_reg,
    dbg_mac_tx_clk_o => dbg_mac_tx_clk_o
  );

END bdf_type;
