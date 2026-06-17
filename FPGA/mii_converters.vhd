library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity mii_converters is
  generic (
    MII_WIDTH     : integer := 2;
    ETHERNET_TYPE : string  := "RMII";
    PLATFORM      : string  := "ALTERA"
  );
  port (
    rst_n_i                 : in std_logic;
    phy_refclk_i: in std_logic                     := '0';
    phy_mii_enet_rx_clk     : in std_logic                     := '0';
    phy_mii_enet_rx_dv      : in std_logic                     := '0';
    phy_mii_enet_rx_err      : in std_logic                     := '0';
    phy_mii_enet_resetn     : out std_logic                    := '0';
    phy_mii_enet_rx_d       : in std_logic_vector(MII_WIDTH - 1 downto 0)  := (others => '0');
    phy_mii_enet_tx_clk     : out std_logic                    := '0';
    phy_mii_enet_tx_en      : out std_logic                    := '0';
    phy_mii_enet_tx_d       : out std_logic_vector(MII_WIDTH - 1 downto 0) := (others => '0');
    mac_speed_i             : in std_logic_vector(1 downto 0);
    mii_rx_clock_o          : out std_logic;
    mii_tx_clock_o          : out std_logic;
    mii_rx_err_o            : out std_logic;
    mii_rx_dv_o             : out std_logic;
    mii_rxd_o               : out std_logic_vector(7 downto 0);
    mii_tx_err_i            : in std_logic;
    mii_tx_en_i             : in std_logic;
    mii_txd_i               : in std_logic_vector(7 downto 0)
  );
end entity;

architecture rtl of mii_converters is

  component rmii_phy_if
    port (
      rstn_async       : in std_logic;
      mode_speed       : in std_logic;
      mac_mii_crs      : out std_logic;
      mac_mii_rxrst    : out std_logic;
      mac_mii_rxc      : out std_logic;
      mac_mii_rxdv     : out std_logic;
      mac_mii_rxer     : out std_logic;
      mac_mii_rxd      : out std_logic_vector(3 downto 0);
      mac_mii_txrst    : out std_logic;
      mac_mii_txc      : out std_logic;
      mac_mii_txen     : in std_logic;
      mac_mii_txer     : in std_logic;
      mac_mii_txd      : in std_logic_vector(3 downto 0);
      phy_rmii_ref_clk : in std_logic;
      phy_rmii_crsdv   : in std_logic;
      phy_rmii_rxer    : in std_logic;
      phy_rmii_rxd     : in std_logic_vector(1 downto 0);
      phy_rmii_txen    : out std_logic;
      phy_rmii_txd     : out std_logic_vector(1 downto 0)
    );
  end component;
  component rgmii_phy_if
    generic (
      CLOCK_INPUT_STYLE : string;
      IODDR_STYLE       : string;
      TARGET            : string;
      USE_CLK90         : string
    );
    port (
      clk                : in std_logic;
      clk90              : in std_logic;
      rst                : in std_logic;
      mac_gmii_tx_en     : in std_logic;
      mac_gmii_tx_er     : in std_logic;
      phy_rgmii_rx_clk   : in std_logic;
      phy_rgmii_rx_ctl   : in std_logic;
      mac_gmii_txd       : in std_logic_vector(7 downto 0);
      phy_rgmii_rxd      : in std_logic_vector(3 downto 0);
      speed              : in std_logic_vector(1 downto 0);
      mac_gmii_rx_clk    : out std_logic;
      mac_gmii_rx_rst    : out std_logic;
      mac_gmii_rx_dv     : out std_logic;
      mac_gmii_rx_er     : out std_logic;
      mac_gmii_tx_clk    : out std_logic;
      mac_gmii_tx_rst    : out std_logic;
      mac_gmii_tx_clk_en : out std_logic;
      phy_rgmii_tx_clk   : out std_logic;
      phy_rgmii_tx_ctl   : out std_logic;
      mac_gmii_rxd       : out std_logic_vector(7 downto 0);
      phy_rgmii_txd      : out std_logic_vector(3 downto 0)
    );
  end component;
  signal enet_clk    : std_logic;
  signal enet_clk_90 : std_logic;

  signal gmii_rx_clk : std_logic;
  signal gmii_rx_dv  : std_logic;
  signal gmii_rx_err : std_logic;
  signal gmii_rxd    : std_logic_vector(7 downto 0);
  signal gmii_tx_clk : std_logic;
  signal gmii_tx_en  : std_logic;
  signal gmii_tx_err : std_logic;
  signal gmii_txd    : std_logic_vector(7 downto 0);
  signal mii_txd : std_logic_vector(3 downto 0);
  
begin
    phy_mii_enet_resetn <= rst_n_i;
  rgmiigen : if (ethernet_type = "RGMII") generate
    gmii_tx_en  <= mii_tx_en_i;
    gmii_txd    <= mii_txd_i;
    gmii_tx_err <= mii_tx_err_i;

    mii_rx_clock_o <= gmii_rx_clk;
    mii_rx_err_o   <= gmii_rx_err;
    mii_rx_dv_o    <= gmii_rx_dv;
    mii_rxd_o      <= gmii_rxd;

    mii_tx_clock_o <= gmii_tx_clk;
    rgmii_if_inst : rgmii_phy_if
    generic map(
      CLOCK_INPUT_STYLE => "BUFG",
      IODDR_STYLE       => "IODDR2",
      TARGET            => platform,
      USE_CLK90         => "TRUE"
    )
    port map
    (
      clk              => enet_clk,
      clk90            => enet_clk_90,
      rst              => not rst_n_i,
      phy_rgmii_rx_clk => phy_mii_enet_rx_clk,
      phy_rgmii_rx_ctl => phy_mii_enet_rx_dv,
      phy_rgmii_rxd    => phy_mii_enet_rx_d,
      phy_rgmii_tx_clk => phy_mii_enet_tx_clk,
      phy_rgmii_tx_ctl => phy_mii_enet_tx_en,
      phy_rgmii_txd    => phy_mii_enet_tx_d,

      speed           => mac_speed_i,
      mac_gmii_tx_en  => gmii_tx_en,
      mac_gmii_tx_er  => gmii_tx_err,
      mac_gmii_txd    => gmii_txd,
      mac_gmii_rx_clk => gmii_rx_clk,
      mac_gmii_rx_dv  => gmii_rx_dv,
      mac_gmii_rx_er  => gmii_rx_err,
      mac_gmii_tx_clk => gmii_tx_clk,
      mac_gmii_rxd    => gmii_rxd

    );
    rgmiiclks_inst : entity work.ethernet_clks
      port map
      (
        inclk0 => phy_refclk_i,
        c0     => enet_clk,
        c1     => enet_clk_90
      );
  end generate;
  rmiigen : if (ethernet_type = "RMII" and platform = "ALTERA") generate

    -- MAC-side MII interface mapping (identical to the RGMII path, lines above).
    -- These were missing: mii_rx_clock_o/mii_tx_clock_o were never driven, so the
    -- MAC clocks were stuck at GND and every network module was optimized away;
    -- the MAC TX nibble was likewise undriven (gmii_txd read but never assigned).
    gmii_tx_en  <= mii_tx_en_i;
    gmii_txd    <= mii_txd_i;
    gmii_tx_err <= mii_tx_err_i;

    mii_rx_clock_o <= gmii_rx_clk;
    mii_rx_err_o   <= gmii_rx_err;
    mii_rx_dv_o    <= gmii_rx_dv;
    mii_rxd_o      <= gmii_rxd;
    mii_tx_clock_o <= gmii_tx_clk;

    -- MII TX: MAC outputs 8 bits (GMII), only lower 4 bits are used for MII
    mii_txd <= gmii_txd(3 downto 0);
    -- MII RX: rmii_phy_if drives only the lower nibble; tie off upper nibble
    gmii_rxd(7 downto 4) <= (others => '0');
    -- enet_clk is only generated by the RGMII PLL; in RMII mode drive it from
    -- the 50 MHz RMII reference clock so the MAC's MIIM and TX logic have a clock.
    enet_clk    <= gmii_tx_clk;
    enet_clk_90 <= gmii_tx_clk;
    rmii_phy_if_inst : rmii_phy_if
    port map
    (
      mode_speed       => '1',
      rstn_async       => rst_n_i,
      mac_mii_rxc      => gmii_rx_clk,
      mac_mii_rxdv     => gmii_rx_dv,
      mac_mii_rxer     => gmii_rx_err,
      mac_mii_rxd      => gmii_rxd(3 downto 0),
      mac_mii_txc      => gmii_tx_clk,
      mac_mii_txen     => gmii_tx_en,
      mac_mii_txer     => gmii_tx_err,
      mac_mii_txd      => mii_txd,
      phy_rmii_ref_clk => phy_refclk_i,
      phy_rmii_crsdv   => phy_mii_enet_rx_dv,
      phy_rmii_rxer    => phy_mii_enet_rx_err,
      phy_rmii_rxd     => phy_mii_enet_rx_d,
      phy_rmii_txen    => phy_mii_enet_tx_en,
      phy_rmii_txd     => phy_mii_enet_tx_d
    );

  end generate;
end architecture;