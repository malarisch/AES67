-- ============================================================
-- top_rmii_tdm8_100m: AES67 core for integration into another design
-- (no board / Quartus project of its own)
--
--   sys_clk : 100 MHz, used directly (no PLL; the spibone SoC runs on it
--             too)
--   network : RMII (50 MHz reference from PHY / oscillator), no MIIM --
--             the PHY must be strapped for 100 Mbit/s full duplex; the
--             link is reported as always up
--   audio   : 8 in / 8 out, one TDM8 data pin per direction, BCLK 256fs
--             (12.288 MHz), frame sync one BCLK wide
--   MCLK    : default = external PLL / jitter cleaner (e.g. CS2000):
--             pll_ref_o (NCO MCLK, PTP-disciplined) -> PLL reference,
--             PLL output (24.576 MHz) -> ext_mclk_i. BCLK / frame sync
--             are generated from ext_mclk_i itself (VCXO_DOMAIN_CLOCKS).
--             For an NCO-only MCLK set syscfg => global_system_cfg_rmii_tdm8_100m
--             (mclk_o is then the NCO MCLK, ext_mclk_i / pll_ref_o unused).
--   control : spibone (SPI -> Wishbone) for the Zephyr firmware on an
--             external MCU (ESP32-S3) or the Linux driver
-- ============================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.miim_types.all;
use work.audioclks_pkg.all;
use work.system_cfg_pkg.all;

entity top_rmii_tdm8_100m is
    generic(
        syscfg : t_global_system_cfg :=
            with_ext_pll_mclk(global_system_cfg_rmii_tdm8_100m, true)
    );
    port(
        sys_clk_i      : in  std_logic;   -- 100 MHz
        rst_n_i        : in  std_logic;   -- async, active low
        ext_mcu_clk_i : in std_logic;
        ext_locked_i  : in std_logic;
        -- RMII
        rmii_ref_clk_i : in  std_logic;   -- 50 MHz
        rmii_crs_dv_i  : in  std_logic;
        rmii_rxd_i     : in  std_logic_vector(1 downto 0);
        rmii_tx_en_o   : out std_logic;
        rmii_txd_o     : out std_logic_vector(1 downto 0);

        -- audio clocks
        mclk_o         : out std_logic;   -- 24.576 MHz (= ext_mclk_i with ext PLL)
        bclk_o         : out std_logic;   -- 12.288 MHz
        fsync_adc_o    : out std_logic;   -- TDM frame sync, ADC side
        fsync_dac_o    : out std_logic;   -- TDM frame sync, DAC side
        -- TDM8 data
        tdm_in_i       : in  std_logic;   -- 8 channels from the ADC side
        tdm_out_o      : out std_logic;   -- 8 channels to the DAC side

        -- external PLL (MCLK_SRC_EXT_PLL)
        pll_ref_o      : out std_logic;   -- reference to the PLL (NCO MCLK)
        ext_mclk_i     : in  std_logic;   -- PLL output, 24.576 MHz

        -- control (spibone)
        spi_clk_i      : in  std_logic;
        spi_cs_n_i     : in  std_logic;
        spi_mosi_i     : in  std_logic;
        spi_miso_o     : out std_logic;
        irq_o          : out std_logic    -- Ethernet buffer interrupt
    );
end entity top_rmii_tdm8_100m;

architecture rtl of top_rmii_tdm8_100m is
    signal miso      : std_logic;
    signal pump      : t_vcxo_pump;
    signal tdm_in_v  : std_logic_vector(syscfg.AUDIO_CONFIG.TX_AD_CFG.TDM_PINS - 1 downto 0);
    signal tdm_out_v : std_logic_vector(syscfg.AUDIO_CONFIG.RX_DA_CFG.TDM_PINS - 1 downto 0);
begin
    assert syscfg.SYS_CLK_NS_PER_TICK = 10 and syscfg.CLK_IN_SPEED = 100
        report "top_rmii_tdm8_100m: syscfg must describe a direct 100 MHz sys_clk"
        severity failure;
    assert syscfg.AUDIO_CONFIG.TX_AD_CFG.TDM_PINS = 1 and syscfg.AUDIO_CONFIG.RX_DA_CFG.TDM_PINS = 1
        report "top_rmii_tdm8_100m: one TDM pin per direction expected"
        severity failure;

    spi_miso_o  <= miso;
    pll_ref_o   <= pump.pll_ref;
    tdm_in_v(0) <= tdm_in_i;
    tdm_out_o   <= tdm_out_v(0);

    soc_top_inst: entity work.soc_top
        generic map(
            syscfg => syscfg
        )
        port map(
            rst_n_i              => rst_n_i,
            ext_mcu_clk_i => ext_mcu_clk_i,
            ext_locked_i => ext_locked_i,
            clock_i              => sys_clk_i,

            phy_refclk_i         => rmii_ref_clk_i,
            phy_mii_enet_rx_dv   => rmii_crs_dv_i,
            phy_mii_enet_rx_d    => rmii_rxd_i,
            phy_mii_enet_resetn  => open,
            phy_mii_enet_tx_clk  => open,
            phy_mii_enet_tx_en   => rmii_tx_en_o,
            phy_mii_enet_tx_d    => rmii_txd_o,
            enet_mdc             => open,
            enet_mdio            => open,

            hbus_rwds            => open,
            hbus_dq              => open,
            hbus_rstn            => open,
            hbus_cs2n            => open,
            hbus_clk0_p          => open,
            hbus_clk0_n          => open,
            i2c0_scl             => open,
            i2c0_sda             => open,

            pll_512fs_i          => '0',
            vcxo_clk_i           => ext_mclk_i,
            vcxo_pump_o          => pump,

            audioclk_mclk_o      => mclk_o,
            audioclk_bclk_o      => bclk_o,
            audioclk_lrclk_dac_o => fsync_dac_o,
            audioclk_lrclk_adc_o => fsync_adc_o,
            audioclk_lrclk_ext_o => open,

            tdm_in               => tdm_in_v,
            tdm_out              => tdm_out_v,
            rx_sample_register   => open,

            spibone_clk          => spi_clk_i,
            spibone_cs_n         => spi_cs_n_i,
            spibone_miso         => miso,
            spibone_mosi         => spi_mosi_i,
            spibone_irq_o        => irq_o,
            dbg_mac_tx_clk_o     => open
        );

end architecture rtl;
