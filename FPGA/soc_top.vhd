
LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.numeric_std.all;

use work.miim_types.all;

use work.audioclks_pkg.all;
use work.system_cfg_pkg.all;

ENTITY soc_top IS
 generic (
		syscfg : t_global_system_cfg := global_system_cfg_cyc

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
		phy_mii_enet_rx_d :  IN  STD_LOGIC_VECTOR(syscfg.PHY_CONFIG.NETWORK_CONFIG.MII_WIDTH - 1 DOWNTO 0)  := (others => '0');
		phy_mii_enet_tx_clk :  OUT  STD_LOGIC  := '0';
    phy_mii_enet_tx_clk_i :  IN  STD_LOGIC  := '0';
		phy_mii_enet_tx_en :  OUT  STD_LOGIC := '0';
		phy_mii_enet_tx_d :  OUT  STD_LOGIC_VECTOR(syscfg.PHY_CONFIG.NETWORK_CONFIG.MII_WIDTH - 1 DOWNTO 0) := (others => '0');

		enet_mdc :  OUT  STD_LOGIC;
		enet_mdio :  INOUT  STD_LOGIC;
		
		-- hyperram for litex hram target
		hbus_rwds :  INOUT  STD_LOGIC;
		hbus_dq :  INOUT  STD_LOGIC_VECTOR(7 DOWNTO 0);
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

		-- litex i2c master (single shared bus)
		i2c0_scl :  INOUT  STD_LOGIC;
		i2c0_sda :  INOUT  STD_LOGIC;


		-- litex spi flash
		spiflash_clk :  OUT  STD_LOGIC := '0';
		spiflash_cs :  OUT  STD_LOGIC := '0';
		spiflash_mosi :  OUT  STD_LOGIC := '0';
		spiflash_miso :  IN  STD_LOGIC := '0';


		-- audio clock in from external pll - only used when USE_EXTERNAL_PLL is true
		pll_512fs_i :  IN  STD_LOGIC := '0'; -- gpio 1

		-- audio clocks outputs
		audioclk_mclk_o :  OUT  STD_LOGIC;
		audioclk_bclk_o : OUT STD_LOGIC;
		audioclk_lrclk_dac_o :  OUT  STD_LOGIC;
    audioclk_lrclk_adc_o : OUT STD_LOGIC;
    audioclk_lrclk_ext_o : OUT STD_LOGIC;
    
		

		tdm_in :  IN  STD_LOGIC_VECTOR(syscfg.AUDIO_CONFIG.TX_AD_CFG.TDM_PINS - 1 downto 0);
		tdm_out :  OUT  STD_LOGIC_VECTOR(syscfg.AUDIO_CONFIG.RX_DA_CFG.TDM_PINS - 1 downto 0);
		


        rx_sample_register : OUT STD_LOGIC_VECTOR((syscfg.AUDIO_CONFIG.PARALLEL_BYTE_DEPTH * 8) * syscfg.AUDIO_CONFIG.RX_DA_CFG.CHANNELS - 1 downto 0);
        tx_sample_register : IN STD_LOGIC_VECTOR((syscfg.AUDIO_CONFIG.PARALLEL_BYTE_DEPTH * 8) * syscfg.AUDIO_CONFIG.TX_AD_CFG.CHANNELS - 1 downto 0) := (others => '0');
		
		sdram_a : out std_logic_vector    (13 downto 0) := (others => '0');
    sdram_ba : out std_logic_vector     (1 downto 0) := (others => '0');
    sdram_cas_n : out std_logic := '0'; 
    sdram_cke : out std_logic := '0';
    sdram_clock : out std_logic := '0';
    sdram_cs_n : out std_logic := '0';
    sdram_dm : out std_logic_vector     (1 downto 0) := (others => '0');
    sdram_dq : inout std_logic_vector    (15 downto 0);
    sdram_ras_n : out std_logic := '0';
    sdram_we_n : out std_logic := '0';

	spibone_clk : in std_logic := '0';
    spibone_cs_n : in std_logic := '1';
    spibone_miso : inout std_logic := '0';
    spibone_mosi : in std_logic := '0';
	spibone_irq_o : out std_logic := '1';
  uartbone_rx : IN STD_LOGIC :='0';
  uartbone_tx : OUT STD_LOGIC :='0';
  dbg_mac_tx_clk_o : OUT STD_LOGIC
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
		clk_sys		:	 IN STD_LOGIC;
		eth_buf_irq		:	 IN STD_LOGIC;
		hyperram_clk		:	 OUT STD_LOGIC;
		hyperram_cs_n		:	 OUT STD_LOGIC;
		hyperram_dq		:	 INOUT STD_LOGIC_VECTOR(7 DOWNTO 0);
		hyperram_rst_n		:	 OUT STD_LOGIC;
		hyperram_rwds		:	 INOUT STD_LOGIC;
		i2c0_scl		:	 INOUT STD_LOGIC;
		i2c0_sda		:	 INOUT STD_LOGIC;
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
		spiflash_mosi		:	 OUT STD_LOGIC
	);
END COMPONENT;
component litex_soc_cyc1000
  port (
    aes67_wb_ack : in std_logic;
    aes67_wb_adr : out std_logic_vector    (29 downto 0);
    aes67_wb_bte : out std_logic_vector     (1 downto 0);
    aes67_wb_cti : out std_logic_vector     (2 downto 0);
    aes67_wb_cyc : out std_logic;
    aes67_wb_dat_r : in std_logic_vector    (31 downto 0);
    aes67_wb_dat_w : out std_logic_vector    (31 downto 0);
    aes67_wb_err : in std_logic;
    aes67_wb_sel : out std_logic_vector     (3 downto 0);
    aes67_wb_stb : out std_logic;
    aes67_wb_we : out std_logic;
    clk_sys : in std_logic;
    clk_sys_ps : in std_logic;
    eth_buf_irq : in std_logic;
    i2c0_scl : inout std_logic;
    i2c0_sda : inout std_logic;
    sdram_a : out std_logic_vector    (13 downto 0);
    sdram_ba : out std_logic_vector     (1 downto 0);
    sdram_cas_n : out std_logic;
    sdram_cke : out std_logic;
    sdram_clock : out std_logic;
    sdram_cs_n : out std_logic;
    sdram_dm : out std_logic_vector     (1 downto 0);
    sdram_dq : inout std_logic_vector    (15 downto 0);
    sdram_ras_n : out std_logic;
    sdram_we_n : out std_logic;
    serial1_rx : in std_logic;
    serial1_tx : out std_logic;
    serial_rx : in std_logic;
    serial_tx : out std_logic;
    spi_clk : out std_logic;
    spi_cs_n : out std_logic;
    spi_miso : in std_logic;
    spi_mosi : out std_logic;
    spiflash_clk : out std_logic;
    spiflash_cs_n : out std_logic;
    spiflash_miso : in std_logic;
    spiflash_mosi : out std_logic
  );
end component;
component litex_soc_spibone
  port (
    aes67_wb_ack : in std_logic;
    aes67_wb_adr : out std_logic_vector    (29 downto 0);
    aes67_wb_bte : out std_logic_vector     (1 downto 0);
    aes67_wb_cti : out std_logic_vector     (2 downto 0);
    aes67_wb_cyc : out std_logic;
    aes67_wb_dat_r : in std_logic_vector    (31 downto 0);
    aes67_wb_dat_w : out std_logic_vector    (31 downto 0);
    aes67_wb_err : in std_logic;
    aes67_wb_sel : out std_logic_vector     (3 downto 0);
    aes67_wb_stb : out std_logic;
    aes67_wb_we : out std_logic;
    clk_sys : in std_logic;
    spibone_clk : in std_logic;
    spibone_cs_n : in std_logic;
    spibone_miso : inout std_logic;
    spibone_mosi : in std_logic
  );
end component;
component litex_soc_uartbone
  port (
    aes67_wb_ack : in std_logic;
    aes67_wb_adr : out std_logic_vector    (29 downto 0);
    aes67_wb_bte : out std_logic_vector     (1 downto 0);
    aes67_wb_cti : out std_logic_vector     (2 downto 0);
    aes67_wb_cyc : out std_logic;
    aes67_wb_dat_r : in std_logic_vector    (31 downto 0);
    aes67_wb_dat_w : out std_logic_vector    (31 downto 0);
    aes67_wb_err : in std_logic;
    aes67_wb_sel : out std_logic_vector     (3 downto 0);
    aes67_wb_stb : out std_logic;
    aes67_wb_we : out std_logic;
    clk_sys : in std_logic;
    uartbone_rx : in std_logic;
    uartbone_tx : out std_logic
  );
end component;
signal hbus_clk : std_logic;
signal mcu_clk : std_logic;
signal mcu_clk_90 : std_logic;
signal eth_buf_irq : std_logic;
signal audioclocks : t_audio_clocks;
signal selected_audio_clock : t_audio_clocks_selected;
begin

  audioclk_mclk_o <= audioclocks.mclk WHEN syscfg.AUDIO_CONFIG.mclk_speed = audio_clock_24_57
                ELSE audioclocks.clk_256fs.bclk WHEN syscfg.AUDIO_CONFIG.mclk_speed = audio_clock_12_28
                ELSE audioclocks.clk_128fs.bclk WHEN syscfg.AUDIO_CONFIG.mclk_speed = audio_clock_06_14
                ELSE audioclocks.clk_64fs.bclk WHEN syscfg.AUDIO_CONFIG.mclk_speed = audio_clock_03_07;
                
  audioclk_bclk_o <= audioclocks.mclk WHEN syscfg.AUDIO_CONFIG.bclk_speed = audio_clock_24_57
                ELSE audioclocks.clk_256fs.bclk WHEN syscfg.AUDIO_CONFIG.bclk_speed = audio_clock_12_28
                ELSE audioclocks.clk_128fs.bclk WHEN syscfg.AUDIO_CONFIG.bclk_speed = audio_clock_06_14
                ELSE audioclocks.clk_64fs.bclk WHEN syscfg.AUDIO_CONFIG.bclk_speed = audio_clock_03_07;
  
  audioclk_lrclk_dac_o <= audioclocks.fsclk_50 when syscfg.AUDIO_CONFIG.RX_DA_CFG.ADDA_CFG.bits_are_right_shifted_to_fs = false and syscfg.AUDIO_CONFIG.RX_DA_CFG.ADDA_CFG.fs_is_one_bclk_high = false
else selected_audio_clock.fsclk_i2s_50_dac when syscfg.AUDIO_CONFIG.RX_DA_CFG.ADDA_CFG.bits_are_right_shifted_to_fs = true and syscfg.AUDIO_CONFIG.RX_DA_CFG.ADDA_CFG.fs_is_one_bclk_high = false
  else selected_audio_clock.fsclk_tdm when syscfg.AUDIO_CONFIG.RX_DA_CFG.ADDA_CFG.bits_are_right_shifted_to_fs = false and syscfg.AUDIO_CONFIG.RX_DA_CFG.ADDA_CFG.fs_is_one_bclk_high = true
    else selected_audio_clock.fsclk_i2s_tdm when syscfg.AUDIO_CONFIG.RX_DA_CFG.ADDA_CFG.bits_are_right_shifted_to_fs = true and syscfg.AUDIO_CONFIG.RX_DA_CFG.ADDA_CFG.fs_is_one_bclk_high = true;
  audioclk_lrclk_adc_o <= audioclocks.fsclk_50 when syscfg.AUDIO_CONFIG.TX_AD_CFG.ADDA_CFG.bits_are_right_shifted_to_fs = false and syscfg.AUDIO_CONFIG.TX_AD_CFG.ADDA_CFG.fs_is_one_bclk_high = false
else selected_audio_clock.fsclk_i2s_50 when syscfg.AUDIO_CONFIG.TX_AD_CFG.ADDA_CFG.bits_are_right_shifted_to_fs = true and syscfg.AUDIO_CONFIG.TX_AD_CFG.ADDA_CFG.fs_is_one_bclk_high = false
  else selected_audio_clock.fsclk_tdm when syscfg.AUDIO_CONFIG.TX_AD_CFG.ADDA_CFG.bits_are_right_shifted_to_fs = false and syscfg.AUDIO_CONFIG.TX_AD_CFG.ADDA_CFG.fs_is_one_bclk_high = true
    else selected_audio_clock.fsclk_i2s_tdm when syscfg.AUDIO_CONFIG.TX_AD_CFG.ADDA_CFG.bits_are_right_shifted_to_fs = true and syscfg.AUDIO_CONFIG.TX_AD_CFG.ADDA_CFG.fs_is_one_bclk_high = true;

      audioclk_lrclk_ext_o <= audioclocks.fsclk_50;
wb_bridge_top_inst : entity work.wb_bridge_top
  generic map (
    syscfg => syscfg
  )
  port map (
    rst_n_i => rst_n_i,
    clock_i => clock_i,
    phy_refclk_i => phy_refclk_i,
    phy_mii_enet_rx_clk => phy_mii_enet_rx_clk,
    phy_mii_enet_rx_dv => phy_mii_enet_rx_dv,
    phy_mii_enet_resetn => phy_mii_enet_resetn,
    phy_mii_enet_rx_d => phy_mii_enet_rx_d,
    phy_mii_enet_tx_clk => phy_mii_enet_tx_clk,
    phy_mii_enet_tx_clk_i => phy_mii_enet_tx_clk_i,
    phy_mii_enet_tx_en => phy_mii_enet_tx_en,
    phy_mii_enet_tx_d => phy_mii_enet_tx_d,
    enet_mdc => enet_mdc,
    enet_mdio => enet_mdio,
    pll_512fs_i => pll_512fs_i,
    audioclocks_o => audioclocks,
    selected_audio_clock_o => selected_audio_clock,
    tdm_in => tdm_in,
    tdm_out => tdm_out,
    rx_sample_register => rx_sample_register,
    tx_sample_register => tx_sample_register,
    mcu_clk_o => mcu_clk,
    mcu_clk_90_o => mcu_clk_90,
    mcu_irq_o => eth_buf_irq,
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
    dbg_mac_tx_clk_o => dbg_mac_tx_clk_o
  );


litex_soc_gen: if (syscfg.SOC_TYPE = LITEX_VEXRISCV_HRAM) generate
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
      clk_sys => mcu_clk,
      eth_buf_irq => eth_buf_irq,
      hyperram_clk => hbus_clk,
      hyperram_cs_n => hbus_cs2n,
      hyperram_dq => hbus_dq,
      hyperram_rst_n => hbus_rstn,
      hyperram_rwds => hbus_rwds,
      i2c0_scl => i2c0_scl,
      i2c0_sda => i2c0_sda,
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
   end generate;
litex_soc_gen_sdram: if (syscfg.SOC_TYPE = LITEX_VEXRISCV_SDRAM) generate
litex_soc_cyc1000_inst : litex_soc_cyc1000
  port map (
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
    clk_sys => mcu_clk,
    clk_sys_ps => mcu_clk_90,
    eth_buf_irq => eth_buf_irq,
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
      i2c0_scl => i2c0_scl,
      i2c0_sda => i2c0_sda,
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
  end generate;
litex_soc_gen_spibone: if (syscfg.SOC_TYPE=LITEX_SPIBONE) generate
	spibone_irq_o <= eth_buf_irq;
litex_soc_spibone_inst : litex_soc_spibone

  port map (
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
    clk_sys => mcu_clk,
    spibone_clk => spibone_clk,
    spibone_cs_n => spibone_cs_n,
    spibone_miso => spibone_miso,
    spibone_mosi => spibone_mosi
  );


end generate;
litex_soc_gen_uartbone : if (syscfg.SOC_TYPE = LITEX_UARTBONE) generate
litex_soc_uartbone_inst : litex_soc_uartbone
  port map (
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
    clk_sys => mcu_clk,
    uartbone_rx => uartbone_rx,
    uartbone_tx => uartbone_tx
  );

end generate;
end architecture;
