
library IEEE;
use IEEE.std_logic_1164.all;

entity top_tang_primer_25k is
    port (
		clk_50m :  IN  STD_LOGIC;
		-- RMII PHY interface
		rmii_ref_clk :  IN  STD_LOGIC;          -- 50 MHz reference clock from PHY
		rmii_crsdv :  IN  STD_LOGIC;
		rmii_rxd :  IN  STD_LOGIC_VECTOR(1 DOWNTO 0);
		rmii_rxer :  IN  STD_LOGIC;
		enet_mdio :  INOUT  STD_LOGIC;
		rmii_txen :  OUT  STD_LOGIC;
		rmii_txd :  OUT  STD_LOGIC_VECTOR(1 DOWNTO 0);
		enet_mdc :  OUT  STD_LOGIC;
		-- DDR3
		--ddram_a :  OUT  STD_LOGIC_VECTOR(13 DOWNTO 0);
		--ddram_ba :  OUT  STD_LOGIC_VECTOR(2 DOWNTO 0);
		--ddram_ras_n :  OUT  STD_LOGIC;
		--ddram_cas_n :  OUT  STD_LOGIC;
		--ddram_we_n :  OUT  STD_LOGIC;
		--ddram_cs_n :  OUT  STD_LOGIC;
		--ddram_dm :  OUT  STD_LOGIC_VECTOR(1 DOWNTO 0);
		--ddram_dq :  INOUT  STD_LOGIC_VECTOR(15 DOWNTO 0);
		--ddram_dqs_p :  INOUT  STD_LOGIC_VECTOR(1 DOWNTO 0);
		--ddram_dqs_n :  INOUT  STD_LOGIC_VECTOR(1 DOWNTO 0);
		--ddram_clk_p :  OUT  STD_LOGIC;
		--ddram_clk_n :  OUT  STD_LOGIC;
		--ddram_cke :  OUT  STD_LOGIC;
		--ddram_odt :  OUT  STD_LOGIC;
		--ddram_reset_n :  OUT  STD_LOGIC;
		-- I2C
		--i2c0_scl :  INOUT  STD_LOGIC;
		--i2c0_sda :  INOUT  STD_LOGIC;
		--i2c1_scl :  INOUT  STD_LOGIC;
		--i2c1_sda :  INOUT  STD_LOGIC;
		-- Audio clocks
		--pll_fs512_in :  IN  STD_LOGIC;
		tdm_bclk_r :  OUT  STD_LOGIC;
		tdm_bclk_f :  OUT  STD_LOGIC;
		tdm_fsync :  OUT  STD_LOGIC;
		pll_fs512_out :  OUT  STD_LOGIC;
		-- TDM audio data
		tdm_din0 :  IN  STD_LOGIC;
		tdm_din1 :  IN  STD_LOGIC;
		tdm_dout0 :  OUT  STD_LOGIC;
		tdm_dout1 :  OUT  STD_LOGIC;
		-- UART
		--uart0_rx :  IN  STD_LOGIC;
		--uart0_tx :  OUT  STD_LOGIC;
		--uart1_rx :  IN  STD_LOGIC;
		--uart1_tx :  OUT  STD_LOGIC;
		-- SPI flash
		spictrl_miso_o :  OUT  STD_LOGIC;
		spictrl_mosi_i :  IN  STD_LOGIC;
		spictrl_cs_n_i :  IN  STD_LOGIC;
		spictrl_clk_i :  IN  STD_LOGIC;
        spictrl_irq_n_o : OUT STD_LOGIC;
		-- Misc
		--adda_nrst :  OUT  STD_LOGIC;
		--user_led :  OUT  STD_LOGIC_VECTOR(3 DOWNTO 0);
		debug_mac_tx_clk_o : OUT STD_LOGIC;
		debug_mac_tx_byte_sent_o: OUT STD_LOGIC;
		debug_rmii_clk_o : OUT STD_LOGIC
        
    );
end entity top_tang_primer_25k;

architecture rtl of top_tang_primer_25k is


    
begin
	debug_rmii_clk_o <= rmii_ref_clk;
    top_inst: entity work.top
     generic map(
        soctype => "spi",
        platform => "GOWIN",
		board => "TANG_PRIMER_25k",
        clk_in_speed => 50,
        ethernet_type => "RMII",
        USE_EXTERNAL_PLL => "false",
        FPGAVERSIONMSB => 13,
        FPGAVERSIONLSB => 12,
        TXSTREAMS => 8,
        RXSTREAMS => 8,
        TXCHANNELS => 8,
        RXCHANNELS => 8,
        BITDEPTH => 24,
        SAMPLERATE => 48,
        TX_SAMPLE_BUFFER_DEPTH => 48,
        RX_SAMPLE_BUFFER_DEPTH => 48,
        MIIM_PHY_ADDRESS => "00001"
    )
     port map(
        rst_n_i => '1',
        clock_i => clk_50m,
        phy_rmii_ref_clk => rmii_ref_clk,
        phy_rmii_crsdv => rmii_crsdv,
        phy_rmii_rxer => rmii_rxer,
        phy_rmii_rxd => rmii_rxd,
        phy_rmii_txen => rmii_txen,
        phy_rmii_txd => rmii_txd,
        enet_mdc => enet_mdc,
        enet_mdio => enet_mdio,
        pll_256fs_rising => tdm_bclk_r,
        pll_256fs_falling => tdm_bclk_f,
        lrclk_tdm_o => tdm_fsync,
        tdm8in_0_i => tdm_din0,
        tdm8in_1_i => tdm_din1,
        tdm8out_0_o => tdm_dout0,
        tdm8out_1_o => tdm_dout1,
        spictrl_clk_i => spictrl_clk_i,
        spictrl_mosi_i => spictrl_mosi_i,
        spictrl_cs_n_i => spictrl_cs_n_i,
        spictrl_miso_o => spictrl_miso_o,
        spictrl_irq_n_o => spictrl_irq_n_o,
        --adda_nRST => adda_nRST,
--        user_led => user_led,
		debug_mac_tx_byte_sent_o => debug_mac_tx_byte_sent_o,
		debug_mac_tx_clk_o => debug_mac_tx_clk_o
    );
    

end architecture;