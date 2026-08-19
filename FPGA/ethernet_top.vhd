library ieee;
use ieee.std_logic_1164.all;
use IEEE.NUMERIC_STD.all;
use work.miim_types.all;
use work.ethernet_types.all;
use work.system_cfg_pkg.all;

use work.wallclock_signals_pkg.all;
entity ethernet_top is
  generic (
    MIIM_CLOCK_DIVIDER : POSITIVE := 50;
    MIIM_PHY_ADDRESS      : t_phy_address := (others => '0');
    MII_TYPE : t_mii_types;
    PTP_IN_SOFTWARE: BOOLEAN := false;
    PHY_TYPE : t_phy_names := OTHER
  );
  port (
    sys_clk125MHz_i : IN STD_LOGIC;
    enet_clk_i      : IN STD_LOGIC;
    mac_rx_clock_o : OUT STD_LOGIC;
    mac_tx_clock_o : OUT STD_LOGIC;
    rst_n : IN STD_LOGIC;


    -- RX Signals to Logic
    received_packet_length_o : OUT unsigned(10 downto 0);
    
    is_mcu_pkt_tog_o : OUT STD_LOGIC;
    is_rtp_pkt_tog_o : OUT STD_LOGIC;
    is_ptp_frame_o : OUT STD_LOGIC;

    rtp_ram_addr_i : IN unsigned(10 downto 0);
    mcu_ram_addr_i : IN unsigned(10 downto 0);

    eth_ram_data_sys_rtp_o : OUT STD_LOGIC_VECTOR(7 downto 0);
    eth_ram_data_rx_mcu_o : OUT STD_LOGIC_VECTOR(7 downto 0);

    -- TX Signals to Logic
    eth_tx_en_mcu_i : IN STD_LOGIC;
    eth_tx_en_ptp_i : IN STD_LOGIC;
    eth_tx_en_rtp_i : IN STD_LOGIC;

    eth_tx_data_mcu_i : IN STD_LOGIC_VECTOR(7 downto 0);
    eth_tx_data_ptp_i : IN STD_LOGIC_VECTOR(7 downto 0);
    eth_tx_data_rtp_i : IN STD_LOGIC_VECTOR(7 downto 0);

    -- signals for tx arbiter
    eth_tx_req_mcu_i : IN STD_LOGIC;
    eth_tx_req_ptp_i : IN STD_LOGIC;
    eth_tx_req_rtp_i : IN STD_LOGIC;

    eth_tx_allow_mcu_o : out STD_LOGIC;
    eth_tx_allow_ptp_o : out STD_LOGIC;
    eth_tx_allow_rtp_o : out STD_LOGIC;


    -- mac control signals
    mac_reset_i : IN std_logic;
    mac_tx_reset_o : OUT std_logic;
    mac_addr_i : IN std_logic_vector(47 downto 0);
    mac_speed_o : OUT STD_LOGIC_VECTOR(1 downto 0);
    mac_linkup_o : out STD_LOGIC;
    mac_tx_byte_sent_o : out STD_LOGIC;
    mac_tx_busy_o : out STD_LOGIC;
    mac_rx_reset_o : out STD_LOGIC;
    mac_sof_sent_tog_o : out STD_LOGIC;
    mac_sof_sent_pulse_o : out STD_LOGIC;
    mac_sof_recv_tog_o : out STD_LOGIC;
    mac_tx_start_prefetch_o : out STD_LOGIC;
    mac_tx_start_prefetch_tog_o : out STD_LOGIC;

    mac_rx_data_o : out STD_LOGIC_VECTOR(7 downto 0);
    mac_rx_byte_received_o : out std_logic;
    mac_rx_byte_receive_index_o : out unsigned(10 downto 0);

    -- mii interface

    mii_rx_clock_i : IN STD_LOGIC;
    mii_tx_clock_i : IN STD_LOGIC;
    mii_rx_err_i : IN STD_LOGIC;
    mii_rx_dv_i : IN STD_LOGIC;
    mii_rxd_i : IN STD_LOGIC_VECTOR(7 downto 0);

    mii_tx_err_o : OUT STD_LOGIC;
    mii_tx_en_o : OUT STD_LOGIC;
    mii_txd_o : OUT std_logic_vector(7 downto 0);

    enet_mdio : INOUT std_logic;
    enet_mdc : OUT std_logic;
    rx_timestamp_i : in t_eth_timestamp

  );
end ethernet_top;

architecture rtl of ethernet_top is
    SIGNAL mac_speed_override : t_ethernet_speed := SPEED_UNSPECIFIED;
        SIGNAL mac_rx_frame : STD_LOGIC;
    SIGNAL mac_rx_byte_received : STD_LOGIC;
    SIGNAL eth_frame_rdy : STD_LOGIC;
    SIGNAL mac_rx_data : STD_LOGIC_VECTOR(7 downto 0);
    SIGNAL mac_tx_data : STD_LOGIC_VECTOR(7 downto 0);
    SIGNAL mac_speed      : STD_ULOGIC_VECTOR(1 downto 0);
    SIGNAL mii_txd_int    : STD_ULOGIC_VECTOR(7 downto 0);
    SIGNAL mii_rxd_int    : STD_ULOGIC_VECTOR(7 downto 0);
    SIGNAL mac_rx_data_su : STD_ULOGIC_VECTOR(7 downto 0);
    SIGNAL eth_ram_wr_data : STD_LOGIC_VECTOR (7 downto 0);
    SIGNAL eth_ram_wr_addr : UNSIGNED (10 downto 0);
    SIGNAL mac_rx_error : STD_LOGIC;
    SIGNAL mac_tx_enable : STD_LOGIC;

    SIGNAL reverse_mac_addr : STD_ULOGIC_VECTOR(47 downto 0);

    SIGNAL mac_rx_clock : STD_LOGIC;
    SIGNAL mac_tx_Clock : STD_LOGIC;
    SIGNAL is_mcu_pkt_tog_receive : STD_LOGIC;
    SIGNAL is_rtp_pkt_tog_receive : STD_LOGIC;

    SIGNAL is_mcu_pkt_tog_done : STD_LOGIC;
    SIGNAL is_rtp_pkt_tog_done : STD_LOGIC;
    
begin
  mac_speed_override <= SPEED_100MBPS WHEN MII_TYPE = RMII OR MII_TYPE = MII else SPEED_UNSPECIFIED;
  is_rtp_pkt_tog_o <= is_rtp_pkt_tog_done when mac_speed = b"01" else is_rtp_pkt_tog_receive;
  is_mcu_pkt_tog_o <= is_mcu_pkt_tog_done when mac_speed = b"01" else is_mcu_pkt_tog_receive;

  mac_rx_clock_o <= mac_rx_clock;
  mac_rx_byte_receive_index_o <= eth_ram_wr_addr;
  mac_rx_byte_received_o <= mac_rx_byte_received;
  mac_rx_data_o <= eth_ram_wr_data;
  b2v_eth_rx : entity work.ethernet_receive
    generic map(
      lastRamAddress => 1532
    )
    port map
    (
      rx_clk           => mac_rx_clock,
      rx_frame         => mac_rx_frame,
      rx_byte_received => mac_rx_byte_received,
      rx_error         => mac_rx_error,
      rx_data          => mac_rx_data,
      frame_rdy        => eth_frame_rdy,
      receive_byte_index         => eth_ram_wr_addr,
      received_byte         => eth_ram_wr_data,
      rx_byte_count    => received_packet_length_o,
      is_mcu_pkt_tog_o => is_mcu_pkt_tog_receive,
      is_ptp_frame_o => is_ptp_frame_o,
      is_rtp_pkt_tog_o => is_rtp_pkt_tog_receive,
      rx_timestamp_i => rx_timestamp_i);


b2v_eth_buf : entity work.eth_ram
    generic map(
      lastAddress => 1532,
      PTP_TO_MCU => PTP_IN_SOFTWARE
      )
    port map
    (
      rx_clk             => mac_rx_clock,
      sync_in            => eth_frame_rdy,
      data_in            => eth_ram_wr_data,
      readAddrRTP        => rtp_ram_addr_i,
      readAddrMCU        => mcu_ram_addr_i,
      writeAddr          => eth_ram_wr_addr,
      dataOut_sysclk_rtp => eth_ram_data_sys_rtp_o,

      dataOut_rxclk => eth_ram_data_rx_mcu_o,

      is_mcu_pkt_tog_o => is_mcu_pkt_tog_done,
      is_rtp_pkt_tog_o => is_rtp_pkt_tog_done,
      sys_clk_i => sys_clk125MHz_i);

      


  b2v_packetaggregator : entity work.ethernet_packet_aggregator
    port map
    (
      tx_en0_i => eth_tx_en_mcu_i,
      tx_en1_i => eth_tx_en_rtp_i,
      tx_en2_i => eth_tx_en_ptp_i,
      data0_i  => eth_tx_data_mcu_i,
      data1_i  => eth_tx_data_rtp_i,
      data2_i  => eth_tx_data_ptp_i,
      tx_en_o  => mac_tx_enable,
      data_o   => mac_tx_data);

  b2v_packetarbiter : entity work.eth_tx_arbiter
    port map
    (
      clk_i       => mac_tx_clock,
      rst_n_i     => rst_n,
      ptp_req_i   => eth_tx_req_ptp_i,
      audio_req_i => eth_tx_req_rtp_i,
      mcu_req_i   => eth_tx_req_mcu_i,
      ptp_allow_o => eth_tx_allow_ptp_o,
      mcu_allow_o => eth_tx_allow_mcu_o,
      audio_allow_o => eth_tx_allow_rtp_o);

      b2v_revmac : entity work.reverse_mac
PORT MAP(mac_address_i => mac_addr_i,
		 mac_address_o => reverse_mac_addr);



b2v_yol_mac : entity work.ethernet
GENERIC MAP(MIIM_CLOCK_DIVIDER => MIIM_CLOCK_DIVIDER,
			MIIM_DISABLE => false,
			MIIM_PHY_ADDRESS => MIIM_PHY_ADDRESS,
			MIIM_POLL_WAIT_TICKS => 10000,
			MIIM_RESET_WAIT_TICKS => 125000,
      PHY_TYPE => PHY_TYPE
			)
PORT MAP(clock_125_i => enet_clk_i,
		 reset_i => mac_reset_i,
		 mii_tx_clk_i => mii_tx_clock_i,
		 mii_rx_clk_i => mii_rx_clock_i,
		 mii_rx_er_i => mii_rx_err_i,
		 mii_rx_dv_i => mii_rx_dv_i,
        mii_txd_o => mii_txd_int,
         mii_rxd_i => mii_rxd_int,
        mii_tx_er_o => mii_tx_err_o,
		 mii_tx_en_o => mii_tx_en_o,



		 miim_clock_i => enet_clk_i,
         mdio_io => enet_mdio,
         mdc_o => enet_mdc,
         
         tx_enable_i => mac_tx_enable,
		 
		 mac_address_i => reverse_mac_addr,
		
		 tx_data_i => std_ulogic_vector(mac_tx_data),
		 
		 tx_reset_o => mac_tx_reset_o,
		 link_up_o => mac_linkup_o,
		 tx_clock_o => mac_tx_clock,
		 tx_byte_sent_o => mac_tx_byte_sent_o,
		 tx_busy_o => mac_tx_busy_o,
		 rx_clock_o => mac_rx_clock,
		 rx_reset_o => mac_rx_reset_o,
		 rx_frame_o => mac_rx_frame,
		 rx_byte_received_o => mac_rx_byte_received,
		 rx_error_o => mac_rx_error,
		 tx_sof_delim_tog_o => mac_sof_sent_tog_o,
		 rx_sof_delim_tog_o => mac_sof_recv_tog_o,
     tx_sof_delim_o => mac_sof_sent_pulse_o,
     tx_start_prefetch_o => mac_tx_start_prefetch_o,
     tx_start_prefetch_tog_o => mac_tx_start_prefetch_tog_o,
		 
		 rx_data_o => mac_rx_data_su,
		 speed_o => mac_speed,
     speed_override_i => mac_speed_override
         
         );
        mac_rx_clock_o <= mac_rx_clock;
        mac_tx_clock_o <= mac_tx_clock;

        -- std_ulogic_vector <-> std_logic_vector conversions for YOL MAC
        mii_txd_o    <= std_logic_vector(mii_txd_int);
        mii_rxd_int  <= std_ulogic_vector(mii_rxd_i);
        mac_speed_o  <= std_logic_vector(mac_speed);
        mac_rx_data  <= std_logic_vector(mac_rx_data_su);
    end rtl;