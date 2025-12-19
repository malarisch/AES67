-- ####################################################################
-- # FMC Parallel Ethernet Client (simplified, no PTP)
-- # Register map matches spi_ethernet_client for basic TX/RX.
-- ####################################################################
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.ethernet_types.all;

entity fmc_ethernet_client is
  generic (
    TX_ADDR_WIDTH : positive := 11;  -- 2**11 = 2048 Bytes
    RX_ADDR_WIDTH : positive := 11
  );
  port (
    clk_sys_i   : in  std_ulogic;
    rst_sys_i   : in  std_ulogic;

    -- Simple asynchronous FMC-like bus
    fmc_addr_i  : in  std_ulogic_vector(6 downto 0);
    fmc_data_io : inout std_ulogic_vector(7 downto 0);
    fmc_ne_n_i  : in  std_ulogic;
    fmc_noe_n_i : in  std_ulogic;
    fmc_nwe_n_i : in  std_ulogic;
    fmc_int_o   : out std_ulogic;

    -- MAC TX
    mac_tx_clock_i     : in  std_ulogic;
    mac_tx_reset_i     : in  std_ulogic;
    mac_tx_enable_o    : out std_ulogic;
    mac_tx_data_o      : out t_ethernet_data;
    mac_tx_byte_sent_i : in  std_ulogic;
    mac_tx_busy_i      : in  std_ulogic;

    -- MAC RX
    mac_rx_clock_i        : in  std_ulogic;
    mac_rx_reset_i        : in  std_ulogic;
    mac_rx_frame_i        : in  std_ulogic;
    mac_rx_data_i         : in  t_ethernet_data;
    mac_rx_byte_rcv_i     : in  std_ulogic;
    mac_rx_error_i        : in  std_ulogic
  );
end entity;

architecture rtl of fmc_ethernet_client is

  signal reg_tx_len      : unsigned(15 downto 0) := (others => '0');
  signal reg_tx_start    : std_ulogic := '0';

  signal reg_rx_len      : unsigned(15 downto 0) := (others => '0');
  signal reg_rx_ready    : std_ulogic := '0';
  signal reg_rx_overflow : std_ulogic := '0';

  -- RX clear handshake
  signal rx_clear_req_sys       : std_ulogic := '0';
  signal rx_clear_ack_mac       : std_ulogic := '0';
  signal rx_clear_ack_sync1_sys : std_ulogic := '0';
  signal rx_clear_ack_sync2_sys : std_ulogic := '0';
  signal rx_clear_sys_pulse     : std_ulogic := '0';
  signal rx_clear_req_sync1_mac : std_ulogic := '0';
  signal rx_clear_req_sync2_mac : std_ulogic := '0';
  signal rx_clear_req_mac_d     : std_ulogic := '0';
  signal rx_clear_mac_pulse     : std_ulogic := '0';

  signal reg_rx_len_sys      : unsigned(15 downto 0) := (others => '0');
  signal reg_rx_ready_sys    : std_ulogic := '0';
  signal reg_rx_overflow_sys : std_ulogic := '0';

  signal reg_tx_len_reset : std_ulogic := '0';

  -- TX FIFO
  signal txfifo_wr_en    : std_ulogic := '0';
  signal txfifo_wr_data  : std_ulogic_vector(7 downto 0) := (others => '0');
  signal txfifo_wr_full  : std_ulogic;
  signal txfifo_rd_en    : std_ulogic := '0';
  signal txfifo_rd_data  : std_ulogic_vector(7 downto 0);
  signal txfifo_rd_empty : std_ulogic;

  -- RX FIFO
  signal rxfifo_wr_en    : std_ulogic := '0';
  signal rxfifo_wr_data  : std_ulogic_vector(7 downto 0) := (others => '0');
  signal rxfifo_wr_full  : std_ulogic;
  signal rxfifo_rd_en    : std_ulogic := '0';
  signal rxfifo_rd_data  : std_ulogic_vector(7 downto 0);
  signal rxfifo_rd_empty : std_ulogic;
  signal fmc_rx_bytes_sent   : unsigned(15 downto 0) := (others => '0');

  -- MAC TX
  signal mac_tx_active   : std_ulogic := '0';
  signal tx_start_sync_1 : std_ulogic := '0';
  signal tx_start_sync_2 : std_ulogic := '0';
  signal tx_start_pulse  : std_ulogic := '0';
  signal tx_prefetch_valid : std_ulogic := '0';
  signal tx_byte_sent_count : unsigned(15 downto 0) := (others => '0');
  signal tx_clear_toggle_mac : std_ulogic := '0';
  signal tx_clear_sync_sys_1 : std_ulogic := '0';
  signal tx_clear_sync_sys_2 : std_ulogic := '0';
  signal tx_clear_sys_pulse  : std_ulogic := '0';
  signal tx_clear_mac_d      : std_ulogic := '0';
  signal tx_clear_mac_pulse  : std_ulogic := '0';
  signal mac_tx_preamble_bytes_sent : unsigned(3 downto 0) := (others => '0');

  type t_tx_SM is (s_Idle, s_PrimeTx, s_Transmit, s_End);
  signal sm_tx_ethernet : t_tx_SM := s_Idle;

  -- RX bookkeeping
  signal mac_rx_frame_d : std_ulogic := '0';
  signal debug_rx_byte_cnt : unsigned(15 downto 0) := (others => '0');

  -- FMC signals
  signal fmc_data_out : std_ulogic_vector(7 downto 0) := (others => '0');
  signal fmc_data_oe  : std_ulogic := '0';
  signal fmc_data_in  : std_ulogic_vector(7 downto 0);
  signal nwe_d        : std_ulogic := '1';
  signal fmc_noe_n_sync_d : std_ulogic := '1';
  signal fmc_nwe_n_sync_d : std_ulogic := '1';
  signal fmc_ne_n_sync_d  : std_ulogic := '1';
  signal fmc_ne_n_meta  : std_ulogic := '1';
  signal fmc_ne_n_sync  : std_ulogic := '1';
  signal fmc_noe_n_meta : std_ulogic := '1';
  signal fmc_noe_n_sync : std_ulogic := '1';
  signal fmc_nwe_n_meta : std_ulogic := '1';
  signal fmc_nwe_n_sync : std_ulogic := '1';
  signal fmc_addr_meta  : std_ulogic_vector(6 downto 0) := (others => '0');
  signal fmc_addr_sync  : std_ulogic_vector(6 downto 0) := (others => '0');
  signal fmc_addr_sync_d : std_ulogic_vector(6 downto 0) := (others => '0');
  signal fmc_din_meta   : std_ulogic_vector(7 downto 0) := (others => '0');
  signal fmc_din_sync   : std_ulogic_vector(7 downto 0) := (others => '0');
  signal fmc_din_sync_d : std_ulogic_vector(7 downto 0) := (others => '0');

  signal fmc_wr_addr_lat : unsigned(6 downto 0) := (others => '0');
  signal fmc_wr_data_lat : std_ulogic_vector(7 downto 0) := (others => '0');
  signal fmc_read_latched : std_ulogic := '0';
  signal fmc_read_addr_lat : unsigned(6 downto 0) := (others => '0');
  signal fmc_read_data_lat : std_ulogic_vector(7 downto 0) := (others => '0');
  signal fmc_ne_qual_low : std_ulogic := '0';

begin

  fmc_data_in <= fmc_data_io;
  -- Drive data bus only when internal OE is asserted *and* external raw strobes
  -- indicate a read cycle. This prevents stale data from a previous read from
  -- appearing at the very beginning of the next read cycle.
  fmc_data_io <= fmc_data_out
    when (fmc_data_oe = '1') and (fmc_ne_n_i = '0') and (fmc_noe_n_i = '0') and (fmc_nwe_n_i = '1')
    else (others => 'Z');

  -- Qualify NE low using both raw and synchronized versions to tolerate phase differences
  fmc_ne_qual_low <= '1' when (fmc_ne_n_i = '0') or (fmc_ne_n_sync = '0') or (fmc_ne_n_sync_d = '0') else '0';

  --------------------------------------------------------------------
  -- TX-FIFO
  --------------------------------------------------------------------
  tx_fifo_inst : entity work.async_fifo
    generic map (
      DATA_WIDTH => 8,
      ADDR_WIDTH => TX_ADDR_WIDTH
    )
    port map (
      wr_clk_i   => clk_sys_i,
      wr_rst_i   => rst_sys_i or tx_clear_sys_pulse,
      wr_en_i    => txfifo_wr_en,
      wr_data_i  => txfifo_wr_data,
      wr_full_o  => txfifo_wr_full,

      rd_clk_i   => mac_tx_clock_i,
      rd_rst_i   => mac_tx_reset_i or tx_clear_mac_pulse,
      rd_en_i    => txfifo_rd_en,
      rd_data_o  => txfifo_rd_data,
      rd_empty_o => txfifo_rd_empty
    );

  --------------------------------------------------------------------
  -- RX-FIFO
  --------------------------------------------------------------------
  rx_fifo_inst : entity work.async_fifo
    generic map (
      DATA_WIDTH => 8,
      ADDR_WIDTH => RX_ADDR_WIDTH
    )
    port map (
      wr_clk_i   => mac_rx_clock_i,
      wr_rst_i   => mac_rx_reset_i or rx_clear_mac_pulse,
      wr_en_i    => rxfifo_wr_en,
      wr_data_i  => rxfifo_wr_data,
      wr_full_o  => rxfifo_wr_full,

      rd_clk_i   => clk_sys_i,
      rd_rst_i   => rst_sys_i or rx_clear_sys_pulse,
      rd_en_i    => rxfifo_rd_en,
      rd_data_o  => rxfifo_rd_data,
      rd_empty_o => rxfifo_rd_empty
    );

  --------------------------------------------------------------------
  -- Sync clear between domains
  --------------------------------------------------------------------
  process(clk_sys_i, rst_sys_i)
  begin
    if rst_sys_i = '1' then
      rx_clear_sys_pulse <= '0';
      rx_clear_ack_sync1_sys <= '0';
      rx_clear_ack_sync2_sys <= '0';
      reg_rx_len_sys      <= (others => '0');
      reg_rx_ready_sys    <= '0';
      reg_rx_overflow_sys <= '0';

    elsif rising_edge(clk_sys_i) then
      rx_clear_ack_sync1_sys <= rx_clear_ack_mac;
      rx_clear_ack_sync2_sys <= rx_clear_ack_sync1_sys;
      rx_clear_sys_pulse <= rx_clear_ack_sync1_sys and not rx_clear_ack_sync2_sys;

      if rx_clear_sys_pulse = '1' then
        reg_rx_ready_sys    <= '0';
        reg_rx_overflow_sys <= '0';
        reg_rx_len_sys      <= (others => '0');
      else
        reg_rx_ready_sys    <= reg_rx_ready;
        reg_rx_overflow_sys <= reg_rx_overflow;
        reg_rx_len_sys      <= reg_rx_len;
      end if;

      tx_clear_sync_sys_1 <= tx_clear_toggle_mac;
      tx_clear_sync_sys_2 <= tx_clear_sync_sys_1;
      tx_clear_sys_pulse  <= tx_clear_sync_sys_1 xor tx_clear_sync_sys_2;
    end if;
  end process;

  --------------------------------------------------------------------
  -- FMC signal synchronization into clk_sys_i domain
  --------------------------------------------------------------------
  process(clk_sys_i, rst_sys_i)
  begin
    if rst_sys_i = '1' then
      fmc_ne_n_meta <= '1';
      fmc_ne_n_sync <= '1';
      fmc_ne_n_sync_d <= '1';
      fmc_noe_n_meta <= '1';
      fmc_noe_n_sync <= '1';
      fmc_noe_n_sync_d <= '1';
      fmc_nwe_n_meta <= '1';
      fmc_nwe_n_sync <= '1';
      fmc_nwe_n_sync_d <= '1';

      fmc_addr_meta <= (others => '0');
      fmc_addr_sync <= (others => '0');
      fmc_addr_sync_d <= (others => '0');
      fmc_din_meta  <= (others => '0');
      fmc_din_sync  <= (others => '0');
      fmc_din_sync_d <= (others => '0');

      nwe_d <= '1';
    elsif rising_edge(clk_sys_i) then
      fmc_ne_n_meta <= fmc_ne_n_i;
      fmc_ne_n_sync <= fmc_ne_n_meta;
      fmc_ne_n_sync_d <= fmc_ne_n_sync;

      fmc_noe_n_meta <= fmc_noe_n_i;
      fmc_noe_n_sync <= fmc_noe_n_meta;
      fmc_noe_n_sync_d <= fmc_noe_n_sync;

      fmc_nwe_n_meta <= fmc_nwe_n_i;
      fmc_nwe_n_sync <= fmc_nwe_n_meta;
      fmc_nwe_n_sync_d <= fmc_nwe_n_sync;
      nwe_d <= fmc_nwe_n_sync_d;

      fmc_addr_meta <= fmc_addr_i;
      fmc_addr_sync <= fmc_addr_meta;
      fmc_addr_sync_d <= fmc_addr_sync;

      fmc_din_meta  <= fmc_data_in;
      fmc_din_sync  <= fmc_din_meta;
      fmc_din_sync_d <= fmc_din_sync;
    end if;
  end process;

  --------------------------------------------------------------------
  -- FMC bus handling (byte accesses, lower byte used)
  --------------------------------------------------------------------
  process(clk_sys_i, rst_sys_i)
    variable addr_v  : unsigned(6 downto 0);
  begin
    if rst_sys_i = '1' then
      txfifo_wr_en    <= '0';
      rxfifo_rd_en    <= '0';
      reg_tx_len      <= (others => '0');
      reg_tx_start    <= '0';
      reg_tx_len_reset <= '0';
      rx_clear_req_sys   <= '0';
      fmc_rx_bytes_sent <= (others => '0');
      fmc_data_out    <= (others => '0');
      fmc_data_oe     <= '0';
      fmc_read_latched <= '0';
      fmc_read_addr_lat <= (others => '0');
      fmc_read_data_lat <= (others => '0');
    elsif rising_edge(clk_sys_i) then
      txfifo_wr_en <= '0';
      rxfifo_rd_en <= '0';
      fmc_data_out <= fmc_read_data_lat;
      fmc_data_oe  <= '0';
      -- Clear request once MAC side acknowledged
      if rx_clear_sys_pulse = '1' then
        rx_clear_req_sys <= '0';
      end if;

      -- Drop read latch when NOE deasserts or chip deselects (raw levels to avoid phase races)
      if (fmc_noe_n_i = '1') or (fmc_ne_n_i = '1') then
        fmc_read_latched <= '0';
      end if;

      -- Latch write address/data while write strobe is active (prevents sampling after NWE rises).
      if (fmc_ne_n_sync = '0') and (fmc_ne_n_sync_d = '1') then
        -- Capture early (1-stage synced) address/data during NWE low to avoid missing
        -- simultaneous NE/NWE assertions.
        fmc_wr_addr_lat <= unsigned(fmc_addr_meta);
        fmc_wr_data_lat <= fmc_din_meta;
      end if;

      -- Detect end of write cycle on falling edge of synchronized NWE (one update per FMC write transaction).
      if (fmc_ne_n_sync = '0') and (nwe_d = '1') and (fmc_nwe_n_sync = '0') then
        --addr_v := fmc_wr_addr_lat;
        case fmc_wr_addr_lat is
          when "0000000" =>  -- 0x00 TX_LEN low
            reg_tx_len(7 downto 0) <= unsigned(fmc_wr_data_lat);
            reg_tx_len_reset <= '1';
          when "0000001" =>  -- 0x01 TX_LEN high
            reg_tx_len(15 downto 8) <= unsigned(fmc_wr_data_lat);
            reg_tx_len_reset <= '1';
          when "0000010" =>  -- 0x02 TX_CTRL
            reg_tx_start <= fmc_wr_data_lat(0);
          when "0100010" =>  -- 0x22 RX_STATUS clear
            if fmc_wr_data_lat(0) = '1' then
              rx_clear_req_sys <= '1';
              fmc_rx_bytes_sent   <= (others => '0');
            end if;
          when others =>
            if addr_v >= 16#10# and addr_v < 16#20# then
              if txfifo_wr_full = '0' then
                txfifo_wr_data <= fmc_wr_data_lat;
                txfifo_wr_en   <= '1';
              end if;
            end if;
        end case;
      end if;

      -- Latch address and prepare read data when NOE is low and not yet latched.
      -- We still use the synchronized NOE level, but do not require a clean falling edge,
      -- so a simultaneous NE/NOE assertion will still produce data in the next clk_sys_i.
      if (fmc_read_latched = '0') and (fmc_ne_qual_low = '1') and (fmc_nwe_n_sync = '1') and (fmc_noe_n_sync = '0') then
        -- Use the most recent address (1-stage synced) to handle NE/NOE asserting together.
        addr_v := unsigned(fmc_addr_meta);
        fmc_read_addr_lat <= addr_v;
        fmc_read_data_lat <= (others => '0');
        case addr_v is
          when "0100000" => fmc_read_data_lat <= std_ulogic_vector(reg_rx_len_sys(7 downto 0)); -- 0x20
          when "0100001" => fmc_read_data_lat <= std_ulogic_vector(reg_rx_len_sys(15 downto 8)); -- 0x21
          when "0100010" => -- 0x22
            fmc_read_data_lat(0) <= reg_rx_ready_sys;
            fmc_read_data_lat(1) <= reg_rx_overflow_sys;
          when others =>
            if addr_v >= 16#30# and addr_v < 16#40# then
              if rxfifo_rd_empty = '0' then
                fmc_read_data_lat <= rxfifo_rd_data;
                rxfifo_rd_en <= '1';
                fmc_rx_bytes_sent <= fmc_rx_bytes_sent + 1;
              end if;
            end if;
        end case;
        fmc_read_latched <= '1';
      end if;

      -- Drive bus during read while latched and external strobes indicate read
      if (fmc_read_latched = '1') and (fmc_ne_qual_low = '1') and (fmc_noe_n_sync = '0') and (fmc_nwe_n_sync = '1') then
        fmc_data_out <= fmc_read_data_lat;
        fmc_data_oe  <= '1';
      end if;

      if reg_tx_len_reset = '1' then
        reg_tx_len_reset <= '0';
      end if;
    end if;
  end process;

  --------------------------------------------------------------------
  -- TX start sync to MAC clock
  --------------------------------------------------------------------
  process(mac_tx_clock_i, mac_tx_reset_i)
  begin
    if mac_tx_reset_i = '1' then
      tx_start_sync_1 <= '0';
      tx_start_sync_2 <= '0';
      tx_start_pulse  <= '0';
    elsif rising_edge(mac_tx_clock_i) then
      tx_start_sync_1 <= reg_tx_start;
      tx_start_sync_2 <= tx_start_sync_1;
      tx_start_pulse  <= tx_start_sync_1 and not tx_start_sync_2;
    end if;
  end process;

  --------------------------------------------------------------------
  -- TX MAC state machine (derived from spi_ethernet_client)
  --------------------------------------------------------------------
  process(mac_tx_clock_i, mac_tx_reset_i)
  begin
    if mac_tx_reset_i = '1' then
      tx_clear_toggle_mac <= '0';
      mac_tx_active   <= '0';
      mac_tx_enable_o <= '0';
      mac_tx_data_o   <= (others => '0');
      txfifo_rd_en    <= '0';
      tx_prefetch_valid <= '0';
      mac_tx_preamble_bytes_sent <= (others => '0');
      tx_byte_sent_count <= (others => '0');
    elsif rising_edge(mac_tx_clock_i) then
      txfifo_rd_en <= '0';
      case sm_tx_ethernet is
        when s_Idle =>
          mac_tx_data_o   <= (others => '0');
          mac_tx_enable_o <= '0';
          if (tx_start_pulse = '1') and (mac_tx_active = '0') then
            if txfifo_rd_empty = '0' then
              sm_tx_ethernet <= s_PrimeTx;
              mac_tx_data_o <= txfifo_rd_data;
              mac_tx_enable_o <= '1';
              tx_prefetch_valid <= '1';
            end if;
          end if;

        when s_PrimeTx =>
          mac_tx_preamble_bytes_sent <= mac_tx_preamble_bytes_sent + 1;
          mac_tx_active <= '1';
          tx_prefetch_valid <= '1';
          if (mac_tx_preamble_bytes_sent = 12) then
            txfifo_rd_en <= '1';
          end if;
          if mac_tx_preamble_bytes_sent = 14 then
            if txfifo_rd_en = '0' then
              sm_tx_ethernet <= s_Transmit;
              txfifo_rd_en <= '1';
              mac_tx_preamble_bytes_sent <= (others => '0');
            end if;
          end if;
          mac_tx_data_o <= txfifo_rd_data;

        when s_Transmit =>
          mac_tx_enable_o <= '1';
          if mac_tx_byte_sent_i = '1' then
            if tx_prefetch_valid = '1' then
              mac_tx_data_o <= txfifo_rd_data;
              tx_byte_sent_count <= tx_byte_sent_count + 1;
            end if;
            if txfifo_rd_empty = '0' then
              txfifo_rd_en <= '1';
              tx_prefetch_valid <= '0';
            else
              if (tx_byte_sent_count + 1) >= reg_tx_len then
                sm_tx_ethernet <= s_End;
              end if;
            end if;
          end if;
          if txfifo_rd_en = '1' then
            tx_prefetch_valid <= '1';
          end if;

        when s_End =>
          mac_tx_enable_o <= '0';
          mac_tx_data_o   <= (others => '0');
          mac_tx_active   <= '0';
          sm_tx_ethernet  <= s_Idle;
          tx_prefetch_valid <= '0';
          tx_byte_sent_count <= (others => '0');
          tx_clear_toggle_mac <= not tx_clear_toggle_mac;
      end case;
    end if;
  end process;

  --------------------------------------------------------------------
  -- TX FIFO clear sync
  --------------------------------------------------------------------
  process(mac_tx_clock_i, mac_tx_reset_i)
  begin
    if mac_tx_reset_i = '1' then
      tx_clear_mac_d     <= '0';
      tx_clear_mac_pulse <= '0';
    elsif rising_edge(mac_tx_clock_i) then
      tx_clear_mac_pulse <= tx_clear_toggle_mac xor tx_clear_mac_d;
      tx_clear_mac_d     <= tx_clear_toggle_mac;
    end if;
  end process;

  --------------------------------------------------------------------
  -- RX path
  --------------------------------------------------------------------
  process(mac_rx_clock_i, mac_rx_reset_i)
    variable len_next    : unsigned(reg_rx_len'range);
    variable ready_next  : std_ulogic;
    variable ovf_next    : std_ulogic;
  begin
    if mac_rx_reset_i = '1' then
      rxfifo_wr_en      <= '0';
      rxfifo_wr_data    <= (others => '0');
      reg_rx_len        <= (others => '0');
      reg_rx_ready      <= '0';
      reg_rx_overflow   <= '0';
      mac_rx_frame_d    <= '0';
      rx_clear_req_sync1_mac <= '0';
      rx_clear_req_sync2_mac <= '0';
      rx_clear_req_mac_d     <= '0';
      rx_clear_ack_mac       <= '0';
      rx_clear_mac_pulse     <= '0';
    elsif rising_edge(mac_rx_clock_i) then
      rxfifo_wr_en <= '0';
      rx_clear_req_sync1_mac <= rx_clear_req_sys;
      rx_clear_req_sync2_mac <= rx_clear_req_sync1_mac;
      rx_clear_req_mac_d     <= rx_clear_req_sync2_mac;
      rx_clear_mac_pulse     <= rx_clear_req_sync2_mac and not rx_clear_req_mac_d;
      rx_clear_ack_mac       <= rx_clear_req_sync2_mac;
      mac_rx_frame_d  <= mac_rx_frame_i;

      len_next   := reg_rx_len;
      ready_next := reg_rx_ready;
      ovf_next   := reg_rx_overflow;

      if rx_clear_mac_pulse = '1' then
        len_next   := (others => '0');
        ready_next := '0';
        ovf_next   := '0';
      end if;

      if (mac_rx_frame_i = '1') and (mac_rx_frame_d = '0') then
        len_next   := (others => '0');
        ready_next := '0';
        if reg_rx_ready = '1' then
          ovf_next := '1';
        else
          ovf_next := '0';
        end if;
      end if;

      if ready_next = '0' then
        if (mac_rx_frame_i = '1') and (mac_rx_byte_rcv_i = '1') then
          debug_rx_byte_cnt <= debug_rx_byte_cnt + 1;
          if rxfifo_wr_full = '0' then
            rxfifo_wr_data <= mac_rx_data_i;
            rxfifo_wr_en   <= '1';
            len_next       := len_next + 1;
          else
            ovf_next := '1';
          end if;
        end if;

        if (mac_rx_frame_i = '0') and (mac_rx_frame_d = '1') then
          if len_next /= 0 then
            ready_next := '1';
          end if;
        end if;
      end if;

      reg_rx_len      <= len_next;
      reg_rx_ready    <= ready_next;
      reg_rx_overflow <= ovf_next;
    end if;
  end process;

  fmc_int_o <= reg_rx_ready_sys or reg_rx_overflow_sys;

end architecture;
