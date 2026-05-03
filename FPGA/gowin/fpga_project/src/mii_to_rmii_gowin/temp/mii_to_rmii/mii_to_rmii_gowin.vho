--
--Written by GowinSynthesis
--Tool Version "V1.9.12"
--Thu Apr 30 12:00:58 2026

--Source file index table:
--file0 "\/home/wolff/Gowin_V1.9.12_linux/IDE/ipcore/MII_to_RMII/data/mii_to_rmii_wrap.v"
--file1 "\/home/wolff/Gowin_V1.9.12_linux/IDE/ipcore/MII_to_RMII/data/mii_to_rmii.v"
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
library gw2a;
use gw2a.components.all;

entity mii_to_rmii_gowin is
port(
  refclk :  in std_logic;
  rstn :  in std_logic;
  speedis_100 :  in std_logic;
  rmii_rx_crs_dv :  in std_logic;
  rmii_rx_er :  in std_logic;
  rmii_rxd :  in std_logic_vector(1 downto 0);
  rmii_tx_en :  out std_logic;
  rmii_txd :  out std_logic_vector(1 downto 0);
  mii_rx_clk :  out std_logic;
  mii_rx_dv :  out std_logic;
  mii_rx_er :  out std_logic;
  mii_rxd :  out std_logic_vector(3 downto 0);
  mii_tx_clk :  out std_logic;
  mii_tx_en :  in std_logic;
  mii_tx_er :  in std_logic;
  mii_txd :  in std_logic_vector(3 downto 0);
  mii_col :  out std_logic;
  mii_crs :  out std_logic);
end mii_to_rmii_gowin;
architecture beh of mii_to_rmii_gowin is
  signal u_rmii_txrx_u_clk_gen_n37_5 : std_logic ;
  signal u_rmii_txrx_u_clk_gen_n47_3 : std_logic ;
  signal u_rmii_txrx_u_clk_gen_n55_4 : std_logic ;
  signal u_rmii_txrx_u_clk_gen_clk_div_7 : std_logic ;
  signal u_rmii_txrx_u_clk_gen_n17_6 : std_logic ;
  signal u_rmii_txrx_u_clk_gen_n19_6 : std_logic ;
  signal u_rmii_txrx_u_clk_gen_n20_6 : std_logic ;
  signal u_rmii_txrx_u_clk_gen_n37_6 : std_logic ;
  signal u_rmii_txrx_u_clk_gen_n47_4 : std_logic ;
  signal u_rmii_txrx_u_clk_gen_clk_div_8 : std_logic ;
  signal u_rmii_txrx_u_clk_gen_n17_7 : std_logic ;
  signal u_rmii_txrx_u_clk_gen_n18_8 : std_logic ;
  signal u_rmii_txrx_u_clk_gen_clk_div_fall : std_logic ;
  signal u_rmii_txrx_u_clk_gen_clk_div_rise : std_logic ;
  signal u_rmii_txrx_u_clk_gen_n6_6 : std_logic ;
  signal u_rmii_txrx_u_clk_gen_n21_8 : std_logic ;
  signal u_rmii_txrx_u_rmii_tx_n32_4 : std_logic ;
  signal u_rmii_txrx_u_rmii_tx_n33_3 : std_logic ;
  signal u_rmii_txrx_u_rmii_tx_rmii_txd_1_7 : std_logic ;
  signal u_rmii_txrx_u_rmii_tx_clk_div_rise_d1 : std_logic ;
  signal u_rmii_txrx_u_rmii_tx_mii_tx_en_tmp : std_logic ;
  signal u_rmii_txrx_u_rmii_tx_clk_div_fall_d1 : std_logic ;
  signal u_rmii_txrx_u_rmii_tx_n11_6 : std_logic ;
  signal u_rmii_txrx_u_rmii_rx_n14_4 : std_logic ;
  signal u_rmii_txrx_u_rmii_rx_n79_12 : std_logic ;
  signal u_rmii_txrx_u_rmii_rx_n80_12 : std_logic ;
  signal u_rmii_txrx_u_rmii_rx_n81_12 : std_logic ;
  signal u_rmii_txrx_u_rmii_rx_n82_12 : std_logic ;
  signal u_rmii_txrx_u_rmii_rx_n100_12 : std_logic ;
  signal u_rmii_txrx_u_rmii_rx_n47_6 : std_logic ;
  signal u_rmii_txrx_u_rmii_rx_n46_5 : std_logic ;
  signal u_rmii_txrx_u_rmii_rx_di_bits_cnt_1_7 : std_logic ;
  signal u_rmii_txrx_u_rmii_rx_di_bits_first_7 : std_logic ;
  signal u_rmii_txrx_u_rmii_rx_n100_13 : std_logic ;
  signal u_rmii_txrx_u_rmii_rx_n47_7 : std_logic ;
  signal u_rmii_txrx_u_rmii_rx_di_bits_first_9 : std_logic ;
  signal u_rmii_txrx_u_rmii_rx_di_bits_cnt_1_9 : std_logic ;
  signal u_rmii_txrx_u_rmii_rx_rmii_rx_er_d1 : std_logic ;
  signal u_rmii_txrx_u_rmii_rx_rmii_rx_crs_dv_d1 : std_logic ;
  signal u_rmii_txrx_u_rmii_rx_di_bits_first : std_logic ;
  signal u_rmii_txrx_u_rmii_rx_n6_6 : std_logic ;
  signal u_rmii_txrx_u_rmii_rx_n107_5 : std_logic ;
  signal VCC_0 : std_logic ;
  signal GND_0 : std_logic ;
  signal \u_rmii_txrx/u_clk_gen/clk_cnt\ : std_logic_vector(4 downto 0);
  signal \u_rmii_txrx/u_rmii_tx/mii_txd_tmp\ : std_logic_vector(3 downto 0);
  signal \u_rmii_txrx/u_rmii_rx/rmii_rxd_d1\ : std_logic_vector(1 downto 0);
  signal \u_rmii_txrx/u_rmii_rx/di_bits_cnt\ : std_logic_vector(1 downto 0);
  signal \u_rmii_txrx/u_rmii_rx/di_bits_data\ : std_logic_vector(3 downto 0);
  signal \u_rmii_txrx/u_rmii_rx/di_bits_er\ : std_logic_vector(1 downto 0);
  signal NN : std_logic;
  signal NN_0 : std_logic;
begin
\u_rmii_txrx/u_clk_gen/n37_s1\: LUT3
generic map (
  INIT => X"E0"
)
port map (
  F => u_rmii_txrx_u_clk_gen_n37_5,
  I0 => u_rmii_txrx_u_clk_gen_n37_6,
  I1 => speedis_100,
  I2 => \u_rmii_txrx/u_clk_gen/clk_cnt\(0));
\u_rmii_txrx/u_clk_gen/n47_s0\: LUT3
generic map (
  INIT => X"C2"
)
port map (
  F => u_rmii_txrx_u_clk_gen_n47_3,
  I0 => u_rmii_txrx_u_clk_gen_n47_4,
  I1 => speedis_100,
  I2 => \u_rmii_txrx/u_clk_gen/clk_cnt\(0));
\u_rmii_txrx/u_clk_gen/n55_s1\: LUT3
generic map (
  INIT => X"0E"
)
port map (
  F => u_rmii_txrx_u_clk_gen_n55_4,
  I0 => u_rmii_txrx_u_clk_gen_n37_6,
  I1 => speedis_100,
  I2 => \u_rmii_txrx/u_clk_gen/clk_cnt\(0));
\u_rmii_txrx/u_clk_gen/clk_div_s5\: LUT3
generic map (
  INIT => X"F8"
)
port map (
  F => u_rmii_txrx_u_clk_gen_clk_div_7,
  I0 => \u_rmii_txrx/u_clk_gen/clk_cnt\(0),
  I1 => u_rmii_txrx_u_clk_gen_clk_div_8,
  I2 => speedis_100);
\u_rmii_txrx/u_clk_gen/n17_s2\: LUT4
generic map (
  INIT => X"7D80"
)
port map (
  F => u_rmii_txrx_u_clk_gen_n17_6,
  I0 => u_rmii_txrx_u_clk_gen_n17_7,
  I1 => \u_rmii_txrx/u_clk_gen/clk_cnt\(2),
  I2 => \u_rmii_txrx/u_clk_gen/clk_cnt\(3),
  I3 => \u_rmii_txrx/u_clk_gen/clk_cnt\(4));
\u_rmii_txrx/u_clk_gen/n19_s2\: LUT4
generic map (
  INIT => X"0DF0"
)
port map (
  F => u_rmii_txrx_u_clk_gen_n19_6,
  I0 => \u_rmii_txrx/u_clk_gen/clk_cnt\(4),
  I1 => \u_rmii_txrx/u_clk_gen/clk_cnt\(3),
  I2 => \u_rmii_txrx/u_clk_gen/clk_cnt\(2),
  I3 => u_rmii_txrx_u_clk_gen_n17_7);
\u_rmii_txrx/u_clk_gen/n20_s2\: LUT2
generic map (
  INIT => X"6"
)
port map (
  F => u_rmii_txrx_u_clk_gen_n20_6,
  I0 => \u_rmii_txrx/u_clk_gen/clk_cnt\(0),
  I1 => \u_rmii_txrx/u_clk_gen/clk_cnt\(1));
\u_rmii_txrx/u_clk_gen/n37_s2\: LUT4
generic map (
  INIT => X"0001"
)
port map (
  F => u_rmii_txrx_u_clk_gen_n37_6,
  I0 => \u_rmii_txrx/u_clk_gen/clk_cnt\(1),
  I1 => \u_rmii_txrx/u_clk_gen/clk_cnt\(2),
  I2 => \u_rmii_txrx/u_clk_gen/clk_cnt\(3),
  I3 => \u_rmii_txrx/u_clk_gen/clk_cnt\(4));
\u_rmii_txrx/u_clk_gen/n47_s1\: LUT4
generic map (
  INIT => X"1000"
)
port map (
  F => u_rmii_txrx_u_clk_gen_n47_4,
  I0 => \u_rmii_txrx/u_clk_gen/clk_cnt\(2),
  I1 => \u_rmii_txrx/u_clk_gen/clk_cnt\(4),
  I2 => \u_rmii_txrx/u_clk_gen/clk_cnt\(3),
  I3 => \u_rmii_txrx/u_clk_gen/clk_cnt\(1));
\u_rmii_txrx/u_clk_gen/clk_div_s6\: LUT4
generic map (
  INIT => X"1001"
)
port map (
  F => u_rmii_txrx_u_clk_gen_clk_div_8,
  I0 => \u_rmii_txrx/u_clk_gen/clk_cnt\(2),
  I1 => \u_rmii_txrx/u_clk_gen/clk_cnt\(4),
  I2 => \u_rmii_txrx/u_clk_gen/clk_cnt\(3),
  I3 => \u_rmii_txrx/u_clk_gen/clk_cnt\(1));
\u_rmii_txrx/u_clk_gen/n17_s3\: LUT2
generic map (
  INIT => X"8"
)
port map (
  F => u_rmii_txrx_u_clk_gen_n17_7,
  I0 => \u_rmii_txrx/u_clk_gen/clk_cnt\(0),
  I1 => \u_rmii_txrx/u_clk_gen/clk_cnt\(1));
\u_rmii_txrx/u_clk_gen/n18_s3\: LUT4
generic map (
  INIT => X"7F80"
)
port map (
  F => u_rmii_txrx_u_clk_gen_n18_8,
  I0 => \u_rmii_txrx/u_clk_gen/clk_cnt\(2),
  I1 => \u_rmii_txrx/u_clk_gen/clk_cnt\(0),
  I2 => \u_rmii_txrx/u_clk_gen/clk_cnt\(1),
  I3 => \u_rmii_txrx/u_clk_gen/clk_cnt\(3));
\u_rmii_txrx/u_clk_gen/clk_cnt_3_s0\: DFFCE
port map (
  Q => \u_rmii_txrx/u_clk_gen/clk_cnt\(3),
  D => u_rmii_txrx_u_clk_gen_n18_8,
  CLK => refclk,
  CE => VCC_0,
  CLEAR => u_rmii_txrx_u_clk_gen_n6_6);
\u_rmii_txrx/u_clk_gen/clk_cnt_2_s0\: DFFCE
port map (
  Q => \u_rmii_txrx/u_clk_gen/clk_cnt\(2),
  D => u_rmii_txrx_u_clk_gen_n19_6,
  CLK => refclk,
  CE => VCC_0,
  CLEAR => u_rmii_txrx_u_clk_gen_n6_6);
\u_rmii_txrx/u_clk_gen/clk_cnt_1_s0\: DFFCE
port map (
  Q => \u_rmii_txrx/u_clk_gen/clk_cnt\(1),
  D => u_rmii_txrx_u_clk_gen_n20_6,
  CLK => refclk,
  CE => VCC_0,
  CLEAR => u_rmii_txrx_u_clk_gen_n6_6);
\u_rmii_txrx/u_clk_gen/clk_cnt_0_s0\: DFFCE
port map (
  Q => \u_rmii_txrx/u_clk_gen/clk_cnt\(0),
  D => u_rmii_txrx_u_clk_gen_n21_8,
  CLK => refclk,
  CE => VCC_0,
  CLEAR => u_rmii_txrx_u_clk_gen_n6_6);
\u_rmii_txrx/u_clk_gen/clk_div_fall_s2\: DFFCE
port map (
  Q => u_rmii_txrx_u_clk_gen_clk_div_fall,
  D => u_rmii_txrx_u_clk_gen_n47_3,
  CLK => refclk,
  CE => VCC_0,
  CLEAR => u_rmii_txrx_u_clk_gen_n6_6);
\u_rmii_txrx/u_clk_gen/clk_div_rise_s2\: DFFCE
port map (
  Q => u_rmii_txrx_u_clk_gen_clk_div_rise,
  D => u_rmii_txrx_u_clk_gen_n55_4,
  CLK => refclk,
  CE => VCC_0,
  CLEAR => u_rmii_txrx_u_clk_gen_n6_6);
\u_rmii_txrx/u_clk_gen/clk_cnt_4_s0\: DFFCE
port map (
  Q => \u_rmii_txrx/u_clk_gen/clk_cnt\(4),
  D => u_rmii_txrx_u_clk_gen_n17_6,
  CLK => refclk,
  CE => VCC_0,
  CLEAR => u_rmii_txrx_u_clk_gen_n6_6);
\u_rmii_txrx/u_clk_gen/clk_div_s3\: DFFCE
generic map (
  INIT => '0'
)
port map (
  Q => NN,
  D => u_rmii_txrx_u_clk_gen_n37_5,
  CLK => refclk,
  CE => u_rmii_txrx_u_clk_gen_clk_div_7,
  CLEAR => u_rmii_txrx_u_clk_gen_n6_6);
\u_rmii_txrx/u_clk_gen/n6_s2\: INV
port map (
  O => u_rmii_txrx_u_clk_gen_n6_6,
  I => rstn);
\u_rmii_txrx/u_clk_gen/n21_s4\: INV
port map (
  O => u_rmii_txrx_u_clk_gen_n21_8,
  I => \u_rmii_txrx/u_clk_gen/clk_cnt\(0));
\u_rmii_txrx/u_rmii_tx/n32_s0\: LUT3
generic map (
  INIT => X"AC"
)
port map (
  F => u_rmii_txrx_u_rmii_tx_n32_4,
  I0 => \u_rmii_txrx/u_rmii_tx/mii_txd_tmp\(1),
  I1 => \u_rmii_txrx/u_rmii_tx/mii_txd_tmp\(3),
  I2 => u_rmii_txrx_u_rmii_tx_clk_div_fall_d1);
\u_rmii_txrx/u_rmii_tx/n33_s0\: LUT3
generic map (
  INIT => X"CA"
)
port map (
  F => u_rmii_txrx_u_rmii_tx_n33_3,
  I0 => \u_rmii_txrx/u_rmii_tx/mii_txd_tmp\(2),
  I1 => \u_rmii_txrx/u_rmii_tx/mii_txd_tmp\(0),
  I2 => u_rmii_txrx_u_rmii_tx_clk_div_fall_d1);
\u_rmii_txrx/u_rmii_tx/rmii_txd_1_s5\: LUT2
generic map (
  INIT => X"E"
)
port map (
  F => u_rmii_txrx_u_rmii_tx_rmii_txd_1_7,
  I0 => u_rmii_txrx_u_rmii_tx_clk_div_fall_d1,
  I1 => u_rmii_txrx_u_rmii_tx_clk_div_rise_d1);
\u_rmii_txrx/u_rmii_tx/clk_div_rise_d1_s0\: DFFCE
port map (
  Q => u_rmii_txrx_u_rmii_tx_clk_div_rise_d1,
  D => u_rmii_txrx_u_clk_gen_clk_div_rise,
  CLK => refclk,
  CE => VCC_0,
  CLEAR => u_rmii_txrx_u_rmii_tx_n11_6);
\u_rmii_txrx/u_rmii_tx/mii_tx_en_tmp_s0\: DFFCE
port map (
  Q => u_rmii_txrx_u_rmii_tx_mii_tx_en_tmp,
  D => mii_tx_en,
  CLK => refclk,
  CE => u_rmii_txrx_u_clk_gen_clk_div_fall,
  CLEAR => u_rmii_txrx_u_rmii_tx_n11_6);
\u_rmii_txrx/u_rmii_tx/mii_txd_tmp_3_s0\: DFFCE
port map (
  Q => \u_rmii_txrx/u_rmii_tx/mii_txd_tmp\(3),
  D => mii_txd(3),
  CLK => refclk,
  CE => u_rmii_txrx_u_clk_gen_clk_div_fall,
  CLEAR => u_rmii_txrx_u_rmii_tx_n11_6);
\u_rmii_txrx/u_rmii_tx/mii_txd_tmp_2_s0\: DFFCE
port map (
  Q => \u_rmii_txrx/u_rmii_tx/mii_txd_tmp\(2),
  D => mii_txd(2),
  CLK => refclk,
  CE => u_rmii_txrx_u_clk_gen_clk_div_fall,
  CLEAR => u_rmii_txrx_u_rmii_tx_n11_6);
\u_rmii_txrx/u_rmii_tx/mii_txd_tmp_1_s0\: DFFCE
port map (
  Q => \u_rmii_txrx/u_rmii_tx/mii_txd_tmp\(1),
  D => mii_txd(1),
  CLK => refclk,
  CE => u_rmii_txrx_u_clk_gen_clk_div_fall,
  CLEAR => u_rmii_txrx_u_rmii_tx_n11_6);
\u_rmii_txrx/u_rmii_tx/mii_txd_tmp_0_s0\: DFFCE
port map (
  Q => \u_rmii_txrx/u_rmii_tx/mii_txd_tmp\(0),
  D => mii_txd(0),
  CLK => refclk,
  CE => u_rmii_txrx_u_clk_gen_clk_div_fall,
  CLEAR => u_rmii_txrx_u_rmii_tx_n11_6);
\u_rmii_txrx/u_rmii_tx/rmii_tx_en_s2\: DFFCE
port map (
  Q => rmii_tx_en,
  D => u_rmii_txrx_u_rmii_tx_mii_tx_en_tmp,
  CLK => refclk,
  CE => u_rmii_txrx_u_rmii_tx_clk_div_fall_d1,
  CLEAR => u_rmii_txrx_u_rmii_tx_n11_6);
\u_rmii_txrx/u_rmii_tx/clk_div_fall_d1_s0\: DFFCE
port map (
  Q => u_rmii_txrx_u_rmii_tx_clk_div_fall_d1,
  D => u_rmii_txrx_u_clk_gen_clk_div_fall,
  CLK => refclk,
  CE => VCC_0,
  CLEAR => u_rmii_txrx_u_rmii_tx_n11_6);
\u_rmii_txrx/u_rmii_tx/rmii_txd_1_s3\: DFFCE
generic map (
  INIT => '0'
)
port map (
  Q => rmii_txd(1),
  D => u_rmii_txrx_u_rmii_tx_n32_4,
  CLK => refclk,
  CE => u_rmii_txrx_u_rmii_tx_rmii_txd_1_7,
  CLEAR => u_rmii_txrx_u_rmii_tx_n11_6);
\u_rmii_txrx/u_rmii_tx/rmii_txd_0_s2\: DFFCE
generic map (
  INIT => '0'
)
port map (
  Q => rmii_txd(0),
  D => u_rmii_txrx_u_rmii_tx_n33_3,
  CLK => refclk,
  CE => u_rmii_txrx_u_rmii_tx_rmii_txd_1_7,
  CLEAR => u_rmii_txrx_u_rmii_tx_n11_6);
\u_rmii_txrx/u_rmii_tx/n11_s2\: INV
port map (
  O => u_rmii_txrx_u_rmii_tx_n11_6,
  I => rstn);
\u_rmii_txrx/u_rmii_rx/n14_s0\: LUT2
generic map (
  INIT => X"E"
)
port map (
  F => u_rmii_txrx_u_rmii_rx_n14_4,
  I0 => u_rmii_txrx_u_clk_gen_clk_div_fall,
  I1 => u_rmii_txrx_u_clk_gen_clk_div_rise);
\u_rmii_txrx/u_rmii_rx/n79_s7\: LUT4
generic map (
  INIT => X"0C0A"
)
port map (
  F => u_rmii_txrx_u_rmii_rx_n79_12,
  I0 => \u_rmii_txrx/u_rmii_rx/di_bits_data\(3),
  I1 => \u_rmii_txrx/u_rmii_rx/rmii_rxd_d1\(1),
  I2 => u_rmii_txrx_u_rmii_rx_di_bits_first,
  I3 => \u_rmii_txrx/u_rmii_rx/di_bits_cnt\(0));
\u_rmii_txrx/u_rmii_rx/n80_s7\: LUT4
generic map (
  INIT => X"0C0A"
)
port map (
  F => u_rmii_txrx_u_rmii_rx_n80_12,
  I0 => \u_rmii_txrx/u_rmii_rx/di_bits_data\(2),
  I1 => \u_rmii_txrx/u_rmii_rx/rmii_rxd_d1\(0),
  I2 => u_rmii_txrx_u_rmii_rx_di_bits_first,
  I3 => \u_rmii_txrx/u_rmii_rx/di_bits_cnt\(0));
\u_rmii_txrx/u_rmii_rx/n81_s7\: LUT4
generic map (
  INIT => X"0C0A"
)
port map (
  F => u_rmii_txrx_u_rmii_rx_n81_12,
  I0 => \u_rmii_txrx/u_rmii_rx/di_bits_data\(1),
  I1 => \u_rmii_txrx/u_rmii_rx/di_bits_data\(3),
  I2 => u_rmii_txrx_u_rmii_rx_di_bits_first,
  I3 => \u_rmii_txrx/u_rmii_rx/di_bits_cnt\(0));
\u_rmii_txrx/u_rmii_rx/n82_s7\: LUT4
generic map (
  INIT => X"0C0A"
)
port map (
  F => u_rmii_txrx_u_rmii_rx_n82_12,
  I0 => \u_rmii_txrx/u_rmii_rx/di_bits_data\(0),
  I1 => \u_rmii_txrx/u_rmii_rx/di_bits_data\(2),
  I2 => u_rmii_txrx_u_rmii_rx_di_bits_first,
  I3 => \u_rmii_txrx/u_rmii_rx/di_bits_cnt\(0));
\u_rmii_txrx/u_rmii_rx/n100_s7\: LUT2
generic map (
  INIT => X"1"
)
port map (
  F => u_rmii_txrx_u_rmii_rx_n100_12,
  I0 => u_rmii_txrx_u_rmii_rx_n100_13,
  I1 => u_rmii_txrx_u_rmii_rx_di_bits_first);
\u_rmii_txrx/u_rmii_rx/n47_s2\: LUT3
generic map (
  INIT => X"0E"
)
port map (
  F => u_rmii_txrx_u_rmii_rx_n47_6,
  I0 => u_rmii_txrx_u_rmii_rx_n47_7,
  I1 => u_rmii_txrx_u_rmii_rx_di_bits_cnt_1_7,
  I2 => \u_rmii_txrx/u_rmii_rx/di_bits_cnt\(0));
\u_rmii_txrx/u_rmii_rx/n46_s1\: LUT4
generic map (
  INIT => X"0BB0"
)
port map (
  F => u_rmii_txrx_u_rmii_rx_n46_5,
  I0 => u_rmii_txrx_u_rmii_rx_di_bits_cnt_1_7,
  I1 => u_rmii_txrx_u_rmii_rx_di_bits_first,
  I2 => \u_rmii_txrx/u_rmii_rx/di_bits_cnt\(0),
  I3 => \u_rmii_txrx/u_rmii_rx/di_bits_cnt\(1));
\u_rmii_txrx/u_rmii_rx/di_bits_cnt_1_s3\: LUT4
generic map (
  INIT => X"4000"
)
port map (
  F => u_rmii_txrx_u_rmii_rx_di_bits_cnt_1_7,
  I0 => \u_rmii_txrx/u_rmii_rx/rmii_rxd_d1\(1),
  I1 => u_rmii_txrx_u_rmii_rx_di_bits_first,
  I2 => \u_rmii_txrx/u_rmii_rx/rmii_rxd_d1\(0),
  I3 => u_rmii_txrx_u_rmii_rx_rmii_rx_crs_dv_d1);
\u_rmii_txrx/u_rmii_rx/di_bits_first_s3\: LUT4
generic map (
  INIT => X"0001"
)
port map (
  F => u_rmii_txrx_u_rmii_rx_di_bits_first_7,
  I0 => u_rmii_txrx_u_rmii_rx_rmii_rx_crs_dv_d1,
  I1 => u_rmii_txrx_u_rmii_rx_di_bits_first,
  I2 => \u_rmii_txrx/u_rmii_rx/di_bits_cnt\(0),
  I3 => \u_rmii_txrx/u_rmii_rx/di_bits_cnt\(1));
\u_rmii_txrx/u_rmii_rx/n100_s8\: LUT4
generic map (
  INIT => X"0305"
)
port map (
  F => u_rmii_txrx_u_rmii_rx_n100_13,
  I0 => \u_rmii_txrx/u_rmii_rx/di_bits_er\(0),
  I1 => u_rmii_txrx_u_rmii_rx_rmii_rx_er_d1,
  I2 => \u_rmii_txrx/u_rmii_rx/di_bits_er\(1),
  I3 => \u_rmii_txrx/u_rmii_rx/di_bits_cnt\(0));
\u_rmii_txrx/u_rmii_rx/n47_s3\: LUT3
generic map (
  INIT => X"0E"
)
port map (
  F => u_rmii_txrx_u_rmii_rx_n47_7,
  I0 => \u_rmii_txrx/u_rmii_rx/di_bits_cnt\(1),
  I1 => u_rmii_txrx_u_rmii_rx_rmii_rx_crs_dv_d1,
  I2 => u_rmii_txrx_u_rmii_rx_di_bits_first);
\u_rmii_txrx/u_rmii_rx/di_bits_first_s4\: LUT4
generic map (
  INIT => X"EEE0"
)
port map (
  F => u_rmii_txrx_u_rmii_rx_di_bits_first_9,
  I0 => u_rmii_txrx_u_rmii_rx_di_bits_first_7,
  I1 => u_rmii_txrx_u_rmii_rx_di_bits_cnt_1_7,
  I2 => u_rmii_txrx_u_clk_gen_clk_div_fall,
  I3 => u_rmii_txrx_u_clk_gen_clk_div_rise);
\u_rmii_txrx/u_rmii_rx/di_bits_cnt_1_s4\: LUT4
generic map (
  INIT => X"DDD0"
)
port map (
  F => u_rmii_txrx_u_rmii_rx_di_bits_cnt_1_9,
  I0 => u_rmii_txrx_u_rmii_rx_di_bits_first,
  I1 => u_rmii_txrx_u_rmii_rx_di_bits_cnt_1_7,
  I2 => u_rmii_txrx_u_clk_gen_clk_div_fall,
  I3 => u_rmii_txrx_u_clk_gen_clk_div_rise);
\u_rmii_txrx/u_rmii_rx/rmii_rx_er_d1_s0\: DFFCE
port map (
  Q => u_rmii_txrx_u_rmii_rx_rmii_rx_er_d1,
  D => rmii_rx_er,
  CLK => refclk,
  CE => VCC_0,
  CLEAR => u_rmii_txrx_u_rmii_rx_n6_6);
\u_rmii_txrx/u_rmii_rx/rmii_rxd_d1_1_s0\: DFFCE
port map (
  Q => \u_rmii_txrx/u_rmii_rx/rmii_rxd_d1\(1),
  D => rmii_rxd(1),
  CLK => refclk,
  CE => VCC_0,
  CLEAR => u_rmii_txrx_u_rmii_rx_n6_6);
\u_rmii_txrx/u_rmii_rx/rmii_rxd_d1_0_s0\: DFFCE
port map (
  Q => \u_rmii_txrx/u_rmii_rx/rmii_rxd_d1\(0),
  D => rmii_rxd(0),
  CLK => refclk,
  CE => VCC_0,
  CLEAR => u_rmii_txrx_u_rmii_rx_n6_6);
\u_rmii_txrx/u_rmii_rx/di_bits_cnt_1_s0\: DFFCE
port map (
  Q => \u_rmii_txrx/u_rmii_rx/di_bits_cnt\(1),
  D => u_rmii_txrx_u_rmii_rx_n46_5,
  CLK => refclk,
  CE => u_rmii_txrx_u_rmii_rx_di_bits_cnt_1_9,
  CLEAR => u_rmii_txrx_u_rmii_rx_n6_6);
\u_rmii_txrx/u_rmii_rx/di_bits_cnt_0_s0\: DFFCE
port map (
  Q => \u_rmii_txrx/u_rmii_rx/di_bits_cnt\(0),
  D => u_rmii_txrx_u_rmii_rx_n47_6,
  CLK => refclk,
  CE => u_rmii_txrx_u_rmii_rx_di_bits_cnt_1_9,
  CLEAR => u_rmii_txrx_u_rmii_rx_n6_6);
\u_rmii_txrx/u_rmii_rx/di_bits_data_3_s0\: DFFCE
port map (
  Q => \u_rmii_txrx/u_rmii_rx/di_bits_data\(3),
  D => \u_rmii_txrx/u_rmii_rx/rmii_rxd_d1\(1),
  CLK => refclk,
  CE => u_rmii_txrx_u_rmii_rx_n14_4,
  CLEAR => u_rmii_txrx_u_rmii_rx_n6_6);
\u_rmii_txrx/u_rmii_rx/di_bits_data_2_s0\: DFFCE
port map (
  Q => \u_rmii_txrx/u_rmii_rx/di_bits_data\(2),
  D => \u_rmii_txrx/u_rmii_rx/rmii_rxd_d1\(0),
  CLK => refclk,
  CE => u_rmii_txrx_u_rmii_rx_n14_4,
  CLEAR => u_rmii_txrx_u_rmii_rx_n6_6);
\u_rmii_txrx/u_rmii_rx/di_bits_data_1_s0\: DFFCE
port map (
  Q => \u_rmii_txrx/u_rmii_rx/di_bits_data\(1),
  D => \u_rmii_txrx/u_rmii_rx/di_bits_data\(3),
  CLK => refclk,
  CE => u_rmii_txrx_u_rmii_rx_n14_4,
  CLEAR => u_rmii_txrx_u_rmii_rx_n6_6);
\u_rmii_txrx/u_rmii_rx/di_bits_data_0_s0\: DFFCE
port map (
  Q => \u_rmii_txrx/u_rmii_rx/di_bits_data\(0),
  D => \u_rmii_txrx/u_rmii_rx/di_bits_data\(2),
  CLK => refclk,
  CE => u_rmii_txrx_u_rmii_rx_n14_4,
  CLEAR => u_rmii_txrx_u_rmii_rx_n6_6);
\u_rmii_txrx/u_rmii_rx/di_bits_er_1_s0\: DFFCE
port map (
  Q => \u_rmii_txrx/u_rmii_rx/di_bits_er\(1),
  D => u_rmii_txrx_u_rmii_rx_rmii_rx_er_d1,
  CLK => refclk,
  CE => u_rmii_txrx_u_rmii_rx_n14_4,
  CLEAR => u_rmii_txrx_u_rmii_rx_n6_6);
\u_rmii_txrx/u_rmii_rx/di_bits_er_0_s0\: DFFCE
port map (
  Q => \u_rmii_txrx/u_rmii_rx/di_bits_er\(0),
  D => \u_rmii_txrx/u_rmii_rx/di_bits_er\(1),
  CLK => refclk,
  CE => u_rmii_txrx_u_rmii_rx_n14_4,
  CLEAR => u_rmii_txrx_u_rmii_rx_n6_6);
\u_rmii_txrx/u_rmii_rx/mii_rxd_3_s2\: DFFCE
port map (
  Q => mii_rxd(3),
  D => u_rmii_txrx_u_rmii_rx_n79_12,
  CLK => refclk,
  CE => u_rmii_txrx_u_clk_gen_clk_div_fall,
  CLEAR => u_rmii_txrx_u_rmii_rx_n6_6);
\u_rmii_txrx/u_rmii_rx/mii_rxd_2_s1\: DFFCE
port map (
  Q => mii_rxd(2),
  D => u_rmii_txrx_u_rmii_rx_n80_12,
  CLK => refclk,
  CE => u_rmii_txrx_u_clk_gen_clk_div_fall,
  CLEAR => u_rmii_txrx_u_rmii_rx_n6_6);
\u_rmii_txrx/u_rmii_rx/mii_rxd_1_s1\: DFFCE
port map (
  Q => mii_rxd(1),
  D => u_rmii_txrx_u_rmii_rx_n81_12,
  CLK => refclk,
  CE => u_rmii_txrx_u_clk_gen_clk_div_fall,
  CLEAR => u_rmii_txrx_u_rmii_rx_n6_6);
\u_rmii_txrx/u_rmii_rx/mii_rxd_0_s1\: DFFCE
port map (
  Q => mii_rxd(0),
  D => u_rmii_txrx_u_rmii_rx_n82_12,
  CLK => refclk,
  CE => u_rmii_txrx_u_clk_gen_clk_div_fall,
  CLEAR => u_rmii_txrx_u_rmii_rx_n6_6);
\u_rmii_txrx/u_rmii_rx/mii_rx_er_s2\: DFFCE
port map (
  Q => mii_rx_er,
  D => u_rmii_txrx_u_rmii_rx_n100_12,
  CLK => refclk,
  CE => u_rmii_txrx_u_clk_gen_clk_div_fall,
  CLEAR => u_rmii_txrx_u_rmii_rx_n6_6);
\u_rmii_txrx/u_rmii_rx/mii_rx_dv_s2\: DFFCE
port map (
  Q => NN_0,
  D => u_rmii_txrx_u_rmii_rx_n107_5,
  CLK => refclk,
  CE => u_rmii_txrx_u_clk_gen_clk_div_fall,
  CLEAR => u_rmii_txrx_u_rmii_rx_n6_6);
\u_rmii_txrx/u_rmii_rx/rmii_rx_crs_dv_d1_s0\: DFFCE
port map (
  Q => u_rmii_txrx_u_rmii_rx_rmii_rx_crs_dv_d1,
  D => rmii_rx_crs_dv,
  CLK => refclk,
  CE => VCC_0,
  CLEAR => u_rmii_txrx_u_rmii_rx_n6_6);
\u_rmii_txrx/u_rmii_rx/di_bits_first_s0\: DFFPE
port map (
  Q => u_rmii_txrx_u_rmii_rx_di_bits_first,
  D => u_rmii_txrx_u_rmii_rx_n107_5,
  CLK => refclk,
  CE => u_rmii_txrx_u_rmii_rx_di_bits_first_9,
  PRESET => u_rmii_txrx_u_rmii_rx_n6_6);
\u_rmii_txrx/u_rmii_rx/n6_s2\: INV
port map (
  O => u_rmii_txrx_u_rmii_rx_n6_6,
  I => rstn);
\u_rmii_txrx/u_rmii_rx/n107_s2\: INV
port map (
  O => u_rmii_txrx_u_rmii_rx_n107_5,
  I => u_rmii_txrx_u_rmii_rx_di_bits_first);
\u_rmii_txrx/mii_col_d_s\: LUT2
generic map (
  INIT => X"8"
)
port map (
  F => mii_col,
  I0 => NN_0,
  I1 => mii_tx_en);
\u_rmii_txrx/mii_crs_d_s\: LUT2
generic map (
  INIT => X"E"
)
port map (
  F => mii_crs,
  I0 => NN_0,
  I1 => mii_tx_en);
VCC_s: VCC
port map (
  V => VCC_0);
GND_s: GND
port map (
  G => GND_0);
GSR_0: GSR
port map (
  GSRI => VCC_0);
  mii_rx_clk <= NN;
  mii_rx_dv <= NN_0;
  mii_tx_clk <= NN;
end beh;
