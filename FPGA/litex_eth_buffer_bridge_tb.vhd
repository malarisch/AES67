-- ####################################################################
-- # GHDL testbench: RX timestamp trailer path
-- #   ethernet_receive -> eth_ram -> litex_eth_buffer_bridge
-- #
-- # Reproduces the burst race the trailer scheme fixes: two back-to-back
-- # frames while the shared TSU value moves on to the next frame's
-- # timestamp mid-flight; every drained frame must carry ITS OWN
-- # timestamp in the 5-byte trailer, and the payload must be intact
-- # (the trailer write must not clobber the last data byte).
-- #
-- #   ghdl -a --std=08 packages/wallclock_signals_pkg.vhd \
-- #        FPGA_Ethernet/FPGA/ethernet_receive.vhd \
-- #        FPGA_Ethernet/FPGA/eth_ram.vhd \
-- #        litex_eth_buffer_bridge.vhd litex_eth_buffer_bridge_tb.vhd
-- #   ghdl --elab-run --std=08 litex_eth_buffer_bridge_tb -gADD_RX_TIMESTAMP=true
-- #   ghdl --elab-run --std=08 litex_eth_buffer_bridge_tb -gADD_RX_TIMESTAMP=false
-- ####################################################################
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.wallclock_signals_pkg.all;

entity litex_eth_buffer_bridge_tb is
  generic (
    ADD_RX_TIMESTAMP : boolean := true
  );
end entity;

architecture sim of litex_eth_buffer_bridge_tb is

  constant ZERO11 : std_logic_vector(10 downto 0) := (others => '0');
  constant ZERO8  : std_logic_vector(7 downto 0)  := (others => '0');

  signal rx_clk  : std_logic := '0';   -- GMII byte clock, 125 MHz
  signal mcu_clk : std_logic := '0';   -- SoC clock, 50 MHz
  signal done    : boolean := false;

  -- ethernet_receive inputs
  signal rx_frame         : std_logic := '0';
  signal rx_byte_received : std_logic := '0';
  signal rx_error         : std_logic := '0';
  signal rx_data          : std_logic_vector(7 downto 0) := (others => '0');
  signal rx_timestamp     : t_eth_timestamp :=
    (seconds => (others => '0'), nanoseconds => (others => '0'));

  -- ethernet_receive -> eth_ram
  signal wr_index      : unsigned(10 downto 0);
  signal wr_byte       : std_logic_vector(7 downto 0);
  signal frame_rdy     : std_logic;
  signal rx_byte_count : unsigned(10 downto 0);
  signal is_mcu_tog    : std_logic;
  signal is_rtp_tog    : std_logic;
  signal is_ptp        : std_logic;

  -- eth_ram <-> bridge
  signal mcu_rd_addr : std_logic_vector(10 downto 0);
  signal mcu_rd_data : std_logic_vector(7 downto 0);

  -- bridge <-> "SoC"
  signal buf_rx_data  : std_logic_vector(7 downto 0);
  signal buf_rx_addr  : std_logic_vector(10 downto 0);
  signal buf_rx_len   : std_logic_vector(10 downto 0);
  signal buf_rx_we    : std_logic;
  signal buf_rx_valid : std_logic;
  signal buf_rx_ack   : std_logic := '0';
  signal rx_overflow  : std_logic;
  signal mac_rx_reset : std_logic := '1';

  type t_buf is array (0 to 2047) of std_logic_vector(7 downto 0);
  signal soc_buf : t_buf := (others => (others => '0'));

  -- Deterministic payload pattern, with the header bytes ethernet_receive
  -- keys on (IPv4 ethertype, UDP proto, non-RTP dst port -> MCU path).
  function frame_byte(seed : integer; i : integer) return std_logic_vector is
  begin
    case i is
      when 12     => return x"08";
      when 13     => return x"00";
      when 23     => return x"11";
      when 36     => return x"12";  -- dst port 0x1234 (not 5004)
      when 37     => return x"34";
      when others => return std_logic_vector(to_unsigned((seed + i) mod 256, 8));
    end case;
  end function;

begin

  rx_clk  <= not rx_clk after 4 ns when not done else '0';
  mcu_clk <= not mcu_clk after 10 ns when not done else '0';

  dut_rx : entity work.ethernet_receive
    generic map (lastRamAddress => 1532)
    port map (
      rx_clk             => rx_clk,
      rx_frame           => rx_frame,
      rx_data            => rx_data,
      rx_byte_received   => rx_byte_received,
      rx_error           => rx_error,
      receive_byte_index => wr_index,
      received_byte      => wr_byte,
      frame_rdy          => frame_rdy,
      rx_byte_count      => rx_byte_count,
      is_mcu_pkt_tog_o   => is_mcu_tog,
      is_rtp_pkt_tog_o   => is_rtp_tog,
      is_ptp_frame_o     => is_ptp,
      rx_timestamp_i     => rx_timestamp);

  dut_ram : entity work.eth_ram
    generic map (lastAddress => 1532, PTP_TO_MCU => true)
    port map (
      rx_clk             => rx_clk,
      writeAddr          => wr_index,
      data_in            => wr_byte,
      sync_in            => frame_rdy,
      readAddrRTP        => to_unsigned(0, 11),
      readAddrMCU        => unsigned(mcu_rd_addr),
      dataOut_sysclk_rtp => open,
      dataOut_rxclk      => mcu_rd_data,
      sync_out           => open,
      is_mcu_pkt_o       => open,
      is_rtp_pkt_o       => open,
      is_mcu_pkt_tog_o   => open,
      is_rtp_pkt_tog_o   => open,
      sys_clk_i          => rx_clk);

  dut_bridge : entity work.litex_eth_buffer_bridge
    generic map (
      ADD_RX_TIMESTAMP => ADD_RX_TIMESTAMP,
      RX_RING_SIZE     => 2048)
    port map (
      buf_rx_data_o          => buf_rx_data,
      buf_rx_addr_o          => buf_rx_addr,
      buf_rx_we_o            => buf_rx_we,
      buf_rx_len_o           => buf_rx_len,
      buf_rx_valid_o         => buf_rx_valid,
      buf_rx_ack_i           => buf_rx_ack,
      buf_tx_addr_o          => open,
      buf_tx_len_i           => ZERO11,
      buf_tx_dat_i           => ZERO8,
      eth_tx_request_i       => '0',
      eth_tx_done_o          => open,
      eth_rx_overflow_o      => rx_overflow,
      mac_tx_clock_i         => rx_clk,
      mac_tx_reset_i         => '0',
      mac_tx_enable_o        => open,
      mac_tx_byte_sent_i     => '0',
      mac_tx_dat_o           => open,
      mac_start_prefetch_i   => '0',
      mac_speed_in           => "10",
      tx_allow_req_o         => open,
      tx_allow_i             => '0',
      mac_rx_clock_i         => rx_clk,
      mac_rx_reset_i         => mac_rx_reset,
      parse_mcu_packet_tog_i => is_mcu_tog,
      pkt_len_i              => std_logic_vector(rx_byte_count),
      eth_ram_data_i         => mcu_rd_data,
      eth_ram_addr_o         => mcu_rd_addr,
      mcu_clk_i              => mcu_clk);

  -- Capture the bridge's writes into the "LiteX" single buffer.
  p_cap : process (rx_clk)
  begin
    if rising_edge(rx_clk) then
      if buf_rx_we = '1' then
        soc_buf(to_integer(unsigned(buf_rx_addr))) <= buf_rx_data;
      end if;
    end if;
  end process;

  p_stim : process
    procedure send_frame(constant seed : in integer; constant len : in integer) is
    begin
      wait until rising_edge(rx_clk);
      rx_frame <= '1';
      for i in 0 to len - 1 loop
        rx_byte_received <= '1';
        rx_data          <= frame_byte(seed, i);
        wait until rising_edge(rx_clk);
      end loop;
      rx_byte_received <= '0';
      rx_frame         <= '0';
    end procedure;

    procedure check_frame(constant seed : in integer; constant len : in integer;
                          constant ts   : in t_eth_timestamp) is
      variable exp_len : integer;
    begin
      wait until buf_rx_valid = '1' for 100 us;
      assert buf_rx_valid = '1'
        report "timeout waiting for buf_rx_valid" severity failure;
      wait until rising_edge(mcu_clk);

      if ADD_RX_TIMESTAMP then
        exp_len := len + 5;
      else
        exp_len := len;
      end if;
      assert to_integer(unsigned(buf_rx_len)) = exp_len
        report "bad rx len: got " &
               integer'image(to_integer(unsigned(buf_rx_len))) &
               " expected " & integer'image(exp_len)
        severity failure;

      for i in 0 to len - 1 loop
        assert soc_buf(i) = frame_byte(seed, i)
          report "payload mismatch at byte " & integer'image(i)
          severity failure;
      end loop;

      if ADD_RX_TIMESTAMP then
        assert soc_buf(len) = "0000" & std_logic_vector(ts.seconds)
          report "trailer seconds mismatch" severity failure;
        assert soc_buf(len + 1) = std_logic_vector(ts.nanoseconds(7 downto 0))
          report "trailer ns[7:0] mismatch" severity failure;
        assert soc_buf(len + 2) = std_logic_vector(ts.nanoseconds(15 downto 8))
          report "trailer ns[15:8] mismatch" severity failure;
        assert soc_buf(len + 3) = std_logic_vector(ts.nanoseconds(23 downto 16))
          report "trailer ns[23:16] mismatch" severity failure;
        assert soc_buf(len + 4) = "00" & std_logic_vector(ts.nanoseconds(29 downto 24))
          report "trailer ns[29:24] mismatch" severity failure;
      end if;

      -- Ack and wait for valid to drop before the next entry.
      buf_rx_ack <= '1';
      wait until rising_edge(mcu_clk);
      wait until rising_edge(mcu_clk);
      wait until rising_edge(mcu_clk);
      buf_rx_ack <= '0';
      wait until buf_rx_valid = '0' for 10 us;
      assert buf_rx_valid = '0'
        report "buf_rx_valid did not clear after ack" severity failure;
    end procedure;

    constant TS_A : t_eth_timestamp :=
      (seconds => to_unsigned(5, 4),  nanoseconds => to_unsigned(305419896, 30));
    constant TS_B : t_eth_timestamp :=
      (seconds => to_unsigned(9, 4),  nanoseconds => to_unsigned(180149760, 30));
    constant TS_C : t_eth_timestamp :=
      (seconds => to_unsigned(15, 4), nanoseconds => to_unsigned(999999999, 30));
  begin
    mac_rx_reset <= '1';
    for i in 1 to 8 loop
      wait until rising_edge(rx_clk);
    end loop;
    mac_rx_reset <= '0';
    for i in 1 to 8 loop
      wait until rising_edge(rx_clk);
    end loop;

    -- Frame A owns TS_A ...
    rx_timestamp <= TS_A;
    send_frame(16#40#, 70);
    -- ... but the shared TSU register already moves on (burst!) while A's
    -- trailer append and ring-buffer copy are still in flight.
    rx_timestamp <= TS_B;
    for i in 1 to 12 loop                -- minimum-ish interframe gap
      wait until rising_edge(rx_clk);
    end loop;
    send_frame(16#A0#, 64);
    rx_timestamp <= TS_C;                -- and moves on again after B

    check_frame(16#40#, 70, TS_A);
    check_frame(16#A0#, 64, TS_B);

    report "PASS (ADD_RX_TIMESTAMP=" & boolean'image(ADD_RX_TIMESTAMP) & ")"
      severity note;
    done <= true;
    wait;
  end process;

end architecture;
