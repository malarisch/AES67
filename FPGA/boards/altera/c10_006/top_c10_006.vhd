-- Cyclone 10 LP board top level
-- Instantiates aes67_top, litex_soc, litex_eth_buffer_bridge,
-- RMII Interface; SDRAM

LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.numeric_std.all;
 

use work.miim_types.all;
use work.audioclks_pkg.all;
use work.system_cfg_pkg.all;

ENTITY top_c10_006 IS

	generic (

		syscfg: t_global_system_cfg := global_system_cfg_c006
	);
	PORT 
	(
    sys_clk_i : in std_logic;
rst_n_i : in std_logic;
		
		  esp_fsclk : OUT STD_LOGIC;
		  esp_bclk : OUT STD_LOGIC;
        tdm_out: OUT STD_LOGIC_VECTOR(syscfg.AUDIO_CONFIG.RX_DA_CFG.TDM_PINS - 1 downto 0); -- tdm out
        aura_fsclk : OUT STD_LOGIC;
		  aura_bclk : OUT STD_LOGIC;
		  aura_data : OUT STD_LOGIC;
        
        mclk: OUT STD_LOGIC;
        
        tdm_in : IN STD_LOGIC_VECTOR(syscfg.AUDIO_CONFIG.TX_AD_CFG.TDM_PINS - 1 downto 0);
        mdc : OUT STD_LOGIC; -- mdc
        mdio : INOUT STD_LOGIC; --mdio
        phy_refclk_i : IN STD_LOGIC;
        phy_rxdv : IN STD_LOGIC; --crsdv
        phy0rx : IN STD_LOGIC_VECTOR(1 downto 0);
        txen : OUT STD_LOGIC; -- txen
        phy0tx : OUT STD_LOGIC_VECTOR(1 downto 0);

        spictrl_clk : IN STD_LOGIC; -- spictrl clk
        spictrl_cs : IN STD_LOGIC; -- spictrl cs
        spictrl_mosi : IN STD_LOGIC; -- spictrl mosi
        spictrl_miso : out STD_LOGIC; -- spictrl miso
        spictrl_irq : OUT STD_LOGIC -- spictrl irq
       -- dbg_mac_tx_clk_o: OUT STD_LOGIC
	);
END top_c10_006;








ARCHITECTURE bdf_type OF top_c10_006 IS
signal miso : STD_LOGIC;
signal bclk : std_logic;
signal fs : std_logic;
signal tdm_outs : STD_LOGIC_VECTOR(syscfg.AUDIO_CONFIG.RX_DA_CFG.TDM_PINS - 1 downto 0);


begin
esp_fsclk <= fs;
aura_fsclk <= fs;
esp_bclk <= bclk;
aura_bclk <= bclk;
tdm_out <= tdm_outs;
aura_data <= tdm_outs(0);
    spictrl_miso <= miso;
	soc_top_inst : entity work.soc_top
  generic map (
    syscfg => syscfg
  )
  port map (
    rst_n_i => '1',
    clock_i => sys_clk_i,
    phy_refclk_i => phy_refclk_i,
    --phy_mii_enet_rx_clk => rxclk_i,
    phy_mii_enet_rx_dv => phy_rxdv,
    phy_mii_enet_resetn => open,
    phy_mii_enet_rx_d => phy0rx,
    phy_mii_enet_tx_clk => open,
    --phy_mii_enet_tx_clk_i => txclk_i,
    phy_mii_enet_tx_en => txen,
    phy_mii_enet_tx_d => phy0tx,
    enet_mdc => mdc,
    enet_mdio => mdio,
    hbus_rwds => open,
    hbus_dq => open,
    hbus_rstn => open,
    hbus_cs2n => open,
    hbus_clk0_p => open,
    hbus_clk0_n => open,
    pll_512fs_i => '0',
    vcxo_clk_i => open,
    --audioclk_512fs_o => AIN7,
    audioclk_mclk_o => mclk,
    audioclk_bclk_o => bclk,
    audioclk_lrclk_adc_o => fs,
    --audioclk_lrclk_dac_o => fs_dac,
    audioclk_lrclk_ext_o => open,
    spibone_clk => spictrl_clk,
    spibone_cs_n => spictrl_cs,
    spibone_miso => miso,
    spibone_mosi => spictrl_mosi,
    spibone_irq_o => spictrl_irq,
    uartbone_rx => open,
    uartbone_tx => open,
    tdm_in => tdm_in,
    tdm_out => tdm_outs
  );

END bdf_type;
