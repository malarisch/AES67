-- AES67 core as a component of OpenAudioFPGA: RMII PHY, TDM8, 100 MHz sys_clk,
-- control over an external Wishbone master.
--
-- Like top_rmii_tdm8_100m, but without the SPIBone bridge: the AES67 register surface
-- (byte window 0x90000000 + 128 KiB, LiteX csr.csv of the aes67_bridge target) is a
-- Wishbone slave on the wb_* ports. The master is the SPIBone SoC of OpenAudioFPGA,
-- which serves the other slaves of the console (routing matrix, ...) on the same bus.
--
-- wb_* are synchronous to ext_mcu_clk_i (50 MHz on the X32): with a 100 MHz sys_clk the
-- mcu clock is passed through from ext_mcu_clk_i (sysclk_pll_gen, CLK_IN_IS_SYS_CLK).
-- wb_adr_i is the 30-bit word address of the full bus; the core decodes the window
-- itself, so the master has to present addresses inside 0x90000000 .. 0x9001FFFF.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.miim_types.all;
use work.audioclks_pkg.all;
use work.system_cfg_pkg.all;

entity aes67_openaudiofpga is
    generic(
        syscfg : t_global_system_cfg :=
            with_soc_type(with_ext_pll_mclk(global_system_cfg_x32, true), EXTERNAL_WB)
    );
    port(
        sys_clk_i      : in  std_logic;   -- 100 MHz
        rst_n_i        : in  std_logic;   -- async, active low
        ext_mcu_clk_i  : in  std_logic;   -- Wishbone clock
        ext_locked_i   : in  std_logic;
        rmii_ref_clk_i : in  std_logic;   -- 50 MHz
        rmii_crs_dv_i  : in  std_logic;
        rmii_rxd_i     : in  std_logic_vector(1 downto 0);
        rmii_tx_en_o   : out std_logic;
        rmii_txd_o     : out std_logic_vector(1 downto 0);

        mclk_o         : out std_logic;   -- 24.576 MHz (= ext_mclk_i with ext PLL)
        bclk_o         : out std_logic;   -- 12.288 MHz
        fsync_adc_o    : out std_logic;   -- TDM frame sync, ADC side
        fsync_dac_o    : out std_logic;   -- TDM frame sync, DAC side
        tdm_in_i       : in  std_logic_vector(3 downto 0);
        tdm_out_o      : out std_logic_vector(3 downto 0);

        pll_ref_o      : out std_logic;   -- reference to the PLL (NCO MCLK)
        ext_mclk_i     : in  std_logic;   -- PLL output, 24.576 MHz

        -- Wishbone slave (classic, 32 bit, word address)
        wb_adr_i       : in  std_logic_vector(29 downto 0);
        wb_dat_i       : in  std_logic_vector(31 downto 0);
        wb_dat_o       : out std_logic_vector(31 downto 0);
        wb_sel_i       : in  std_logic_vector(3 downto 0);
        wb_cyc_i       : in  std_logic;
        wb_stb_i       : in  std_logic;
        wb_we_i        : in  std_logic;
        wb_cti_i       : in  std_logic_vector(2 downto 0);
        wb_bte_i       : in  std_logic_vector(1 downto 0);
        wb_ack_o       : out std_logic;
        wb_err_o       : out std_logic;
        irq_o          : out std_logic    -- Ethernet buffer interrupt
    );
end entity aes67_openaudiofpga;

architecture rtl of aes67_openaudiofpga is
    signal bclk : std_logic;
    signal pump : t_vcxo_pump;
    -- spibone_miso of soc_top is an inout with an initial value: Synplify wants it
    -- connected even though there is no SPIBone in EXTERNAL_WB mode
    signal spibone_miso_nc : std_logic;
begin
    assert syscfg.SYS_CLK_NS_PER_TICK = 10 and syscfg.CLK_IN_SPEED = 100
        report "aes67_openaudiofpga: syscfg must describe a direct 100 MHz sys_clk"
        severity failure;
    assert syscfg.SOC_TYPE = EXTERNAL_WB
        report "aes67_openaudiofpga: syscfg.SOC_TYPE must be EXTERNAL_WB"
        severity failure;

    bclk_o    <= bclk;
    pll_ref_o <= pump.pll_ref;

    soc_top_inst: entity work.soc_top
        generic map(
            syscfg => syscfg
        )
        port map(
            rst_n_i              => rst_n_i,
            ext_mcu_clk_i        => ext_mcu_clk_i,
            ext_locked_i         => ext_locked_i,
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
            audioclk_bclk_o      => bclk,
            audioclk_lrclk_dac_o => fsync_dac_o,
            audioclk_lrclk_adc_o => fsync_adc_o,
            audioclk_lrclk_ext_o => open,

            tdm_in               => tdm_in_i,
            tdm_out              => tdm_out_o,
            rx_sample_register   => open,

            spibone_miso         => spibone_miso_nc,

            ext_wb_adr           => wb_adr_i,
            ext_wb_dat_w         => wb_dat_i,
            ext_wb_dat_r         => wb_dat_o,
            ext_wb_sel           => wb_sel_i,
            ext_wb_cyc           => wb_cyc_i,
            ext_wb_stb           => wb_stb_i,
            ext_wb_we            => wb_we_i,
            ext_wb_cti           => wb_cti_i,
            ext_wb_bte           => wb_bte_i,
            ext_wb_ack           => wb_ack_o,
            ext_wb_err           => wb_err_o,
            ext_wb_irq_o         => irq_o,
            dbg_mac_tx_clk_o     => open
        );

end architecture rtl;
