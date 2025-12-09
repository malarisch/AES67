library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- ####################################################################
-- # 1) Generische asynchrone FIFO (Dual-Clock, Gray-Pointer)
-- ####################################################################

entity async_fifo is
  generic (
    DATA_WIDTH : positive := 8;
    ADDR_WIDTH : positive := 8     -- FIFO-Tiefe = 2**ADDR_WIDTH
  );
  port (
    -- Schreibseite
    wr_clk_i   : in  std_ulogic;
    wr_rst_i   : in  std_ulogic;
    wr_en_i    : in  std_ulogic;
    wr_data_i  : in  std_ulogic_vector(DATA_WIDTH-1 downto 0);
    wr_full_o  : out std_ulogic;

    -- Leseseite
    rd_clk_i   : in  std_ulogic;
    rd_rst_i   : in  std_ulogic;
    rd_en_i    : in  std_ulogic;
    rd_data_o  : out std_ulogic_vector(DATA_WIDTH-1 downto 0);
    rd_empty_o : out std_ulogic
  );
end entity async_fifo;

architecture rtl_async_fifo of async_fifo is

  constant DEPTH : natural := 2 ** ADDR_WIDTH;

  type ram_t is array (0 to DEPTH-1) of std_ulogic_vector(DATA_WIDTH-1 downto 0);
  signal ram : ram_t;

  -- Binäre Pointer
  signal wr_ptr_bin : unsigned(ADDR_WIDTH downto 0) := (others => '0');
  signal rd_ptr_bin : unsigned(ADDR_WIDTH downto 0) := (others => '0');

  -- Gray-Pointer
  signal wr_ptr_gray : unsigned(ADDR_WIDTH downto 0) := (others => '0');
  signal rd_ptr_gray : unsigned(ADDR_WIDTH downto 0) := (others => '0');

  -- Synchronisierte Pointer
  signal wr_ptr_gray_rdclk_1, wr_ptr_gray_rdclk_2 : unsigned(ADDR_WIDTH downto 0) := (others => '0');
  signal rd_ptr_gray_wrclk_1, rd_ptr_gray_wrclk_2 : unsigned(ADDR_WIDTH downto 0) := (others => '0');

  -- Full/Empty
  signal full_reg  : std_ulogic := '0';
  signal empty_reg : std_ulogic := '1';

  -- Gray-Konvertierung
  function bin2gray(b : unsigned) return unsigned is
  begin
    return (b srl 1) xor b;
  end function;

  function gray2bin(g : unsigned) return unsigned is
    variable b : unsigned(g'range) := (others => '0');
  begin
    b(b'high) := g(g'high);
    for i in (b'high-1) downto 0 loop
      b(i) := b(i+1) xor g(i);
    end loop;
    return b;
  end function;

begin

  --------------------------------------------------------------------
  -- Schreibseite
  --------------------------------------------------------------------
  process(wr_clk_i, wr_rst_i)
    variable next_wr_ptr_gray : unsigned(wr_ptr_gray'range);
    variable rd_ptr_gray_inv  : unsigned(rd_ptr_gray_wrclk_2'range);
  begin
    if wr_rst_i = '1' then
      wr_ptr_bin  <= (others => '0');
      wr_ptr_gray <= (others => '0');
      rd_ptr_gray_wrclk_1 <= (others => '0');
      rd_ptr_gray_wrclk_2 <= (others => '0');
      full_reg    <= '0';
    elsif rising_edge(wr_clk_i) then

      -- Synchronisierung des Read-Pointers in Schreib-Clockdomain
      rd_ptr_gray_wrclk_1 <= rd_ptr_gray;
      rd_ptr_gray_wrclk_2 <= rd_ptr_gray_wrclk_1;

      -- Full-Berechnung:
      -- FIFO voll, wenn nächster Write-Pointer gleich Read-Pointer mit invertiertem MSB ist
      -- (klassische Full/Empty-Logik für Gray-Pointer-FIFOs)
      -- Invertiere die beiden obersten Bits des synchronisierten Read-Pointers
      -- (Cliff Cummings Async FIFO full detection)
      next_wr_ptr_gray := bin2gray(wr_ptr_bin + 1);
      rd_ptr_gray_inv  := rd_ptr_gray_wrclk_2;
      rd_ptr_gray_inv(rd_ptr_gray_inv'high downto rd_ptr_gray_inv'high-1) := 
        not rd_ptr_gray_wrclk_2(rd_ptr_gray_wrclk_2'high downto rd_ptr_gray_wrclk_2'high-1);

      if next_wr_ptr_gray = rd_ptr_gray_inv then
        full_reg <= '1';
      else
        full_reg <= '0';
      end if;

      if (wr_en_i = '1') and (full_reg = '0') then
        ram(to_integer(wr_ptr_bin(ADDR_WIDTH-1 downto 0))) <= wr_data_i;
        wr_ptr_bin  <= wr_ptr_bin + 1;
        wr_ptr_gray <= bin2gray(wr_ptr_bin + 1);
      end if;

    end if;
  end process;

  wr_full_o <= full_reg;

  --------------------------------------------------------------------
  -- Leseseite
  --------------------------------------------------------------------
  process(rd_clk_i, rd_rst_i)
  begin
    if rd_rst_i = '1' then
      rd_ptr_bin  <= (others => '0');
      rd_ptr_gray <= (others => '0');
      wr_ptr_gray_rdclk_1 <= (others => '0');
      wr_ptr_gray_rdclk_2 <= (others => '0');
      empty_reg   <= '1';
      rd_data_o   <= (others => '0');
    elsif rising_edge(rd_clk_i) then

      -- Synchronisierung des Write-Pointers in Lese-Clockdomain
      wr_ptr_gray_rdclk_1 <= wr_ptr_gray;
      wr_ptr_gray_rdclk_2 <= wr_ptr_gray_rdclk_1;

      -- Show-ahead Read-Daten bereitstellen
      rd_data_o <= ram(to_integer(rd_ptr_bin(ADDR_WIDTH-1 downto 0)));

      -- Empty, wenn Gray-Read-Pointer == synchronisiertem Gray-Write-Pointer
      if rd_ptr_gray = wr_ptr_gray_rdclk_2 then
        empty_reg <= '1';
      else
        empty_reg <= '0';
      end if;

      if (rd_en_i = '1') and (empty_reg = '0') then
        rd_ptr_bin <= rd_ptr_bin + 1;
        rd_ptr_gray <= bin2gray(rd_ptr_bin + 1);
      end if;

    end if;
  end process;

  rd_empty_o <= empty_reg;

end architecture rtl_async_fifo;

-- ####################################################################
-- # 2) SPI-Ethernet-Client für den MAC von FPGA_Ethernet
-- ####################################################################
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.ethernet_types.all;

entity spi_ethernet_client is
  generic (
    TX_ADDR_WIDTH : positive := 11;  -- 2**11 = 2048 Bytes
    RX_ADDR_WIDTH : positive := 11
  );
  port (
    ------------------------------------------------------------------
    -- System-/SPI-Domain (Sampling-Clock, typ. höher als SPI-SCK)
    ------------------------------------------------------------------
    clk_sys_i   : in  std_ulogic;
    rst_sys_i   : in  std_ulogic;

    -- SPI-Slave (Mode 0, CPOL=0, CPHA=0)
    spi_sck_i   : in  std_ulogic;
    spi_cs_n_i  : in  std_ulogic;
    spi_mosi_i  : in  std_ulogic;
    spi_miso_o  : out std_ulogic;
    spi_int_n_o : out std_ulogic;  -- aktiv-low Interrupt: RX ready/overflow
    spi_transaction_done : out std_ulogic; -- für Debug/Analyse: eine SPI-Transaktion abgeschlossen

    ------------------------------------------------------------------
    -- MAC-Client TX-Schnittstelle
    ------------------------------------------------------------------
    mac_tx_clock_i     : in  std_ulogic;        -- tx_clock_o
    mac_tx_reset_i     : in  std_ulogic;        -- tx_reset_o
    mac_tx_enable_o    : out std_ulogic;        -- tx_enable_i
    mac_tx_data_o      : out t_ethernet_data;   -- tx_data_i
    mac_tx_byte_sent_i : in  std_ulogic;        -- tx_byte_sent_o
    mac_tx_busy_i      : in  std_ulogic;        -- tx_busy_o

    ------------------------------------------------------------------
    -- MAC-Client RX-Schnittstelle
    ------------------------------------------------------------------
    mac_rx_clock_i        : in  std_ulogic;      -- rx_clock_o
    mac_rx_reset_i        : in  std_ulogic;      -- rx_reset_o
    mac_rx_frame_i        : in  std_ulogic;      -- rx_frame_o
    mac_rx_data_i         : in  t_ethernet_data; -- rx_data_o
    mac_rx_byte_rcv_i     : in  std_ulogic;      -- rx_byte_received_o
    mac_rx_error_i        : in  std_ulogic       -- rx_error_o
  );
end entity spi_ethernet_client;

architecture rtl of spi_ethernet_client is

  --------------------------------------------------------------------
  -- SPI-Domain: einfache Kommando-/Register-Logik
  -- Protokoll:
  --   1. Byte: [7]=1: Write, [7]=0: Read, [6..0]=Adresse
  --   Write-Register:
  --      0x00/0x01: TX-Länge (Low/High)
  --      0x02    : TX-Start-Bit (Bit 0)
  --      0x22    : RX-Status-Clear (Bit0=ACK)
  --      0x10.. : TX-Daten, gehen in TX-FIFO
  --   Read-Register:
  --      0x20/0x21: RX-Länge (Low/High)
  --      0x22    : RX-Status (Bit0=READY, Bit1=OVF)
  --      0x30.. : RX-Daten aus RX-FIFO
  --------------------------------------------------------------------

  type t_spi_state is (SPI_IDLE, SPI_CMD, SPI_DATA);
  signal spi_state      : t_spi_state := SPI_IDLE;

  signal spi_bit_cnt    : integer range 0 to 7 := 0;
  signal spi_shift_in   : std_ulogic_vector(7 downto 0) := (others => '0');
  signal spi_shift_out  : std_ulogic_vector(7 downto 0) := (others => '0');
  signal spi_miso_lat   : std_ulogic := '0';
  signal spi_miso_drv   : std_ulogic := '0';
  signal spi_rw_n       : std_ulogic := '0';  -- 1=Write, 0=Read
  signal spi_addr       : std_ulogic_vector(6 downto 0) := (others => '0');

  signal sck_meta       : std_ulogic := '0';
  signal sck_sync       : std_ulogic := '0';
  signal sck_sync_d     : std_ulogic := '0';
  signal sck_rising     : std_ulogic := '0';
  signal sck_falling    : std_ulogic := '0';
  
  signal mosi_meta      : std_ulogic := '0';
  signal mosi_sync      : std_ulogic := '0';
  
  signal cs_meta        : std_ulogic := '1';
  signal spi_cs_sync    : std_ulogic := '1';
  signal spi_cs_sync_d  : std_ulogic := '1';

  --------------------------------------------------------------------
  -- Steuerregister
  --------------------------------------------------------------------
  signal reg_tx_len      : unsigned(15 downto 0) := (others => '0');
  signal reg_tx_start    : std_ulogic := '0';

  signal reg_rx_len      : unsigned(15 downto 0) := (others => '0');
  signal reg_rx_ready    : std_ulogic := '0';
  signal reg_rx_overflow : std_ulogic := '0';
  signal reg_rx_len_sys      : unsigned(15 downto 0) := (others => '0');
  signal reg_rx_ready_sys    : std_ulogic := '0';
  signal reg_rx_overflow_sys : std_ulogic := '0';
  signal reg_rx_len_sync_1   : unsigned(15 downto 0) := (others => '0');
  signal reg_rx_len_sync_2   : unsigned(15 downto 0) := (others => '0');
  signal reg_rx_ready_sync_1 : std_ulogic := '0';
  signal reg_rx_ready_sync_2 : std_ulogic := '0';
  signal reg_rx_overflow_sync_1 : std_ulogic := '0';
  signal reg_rx_overflow_sync_2 : std_ulogic := '0';
  signal rx_clear_toggle_sys : std_ulogic := '0';
  signal rx_clear_sync_1     : std_ulogic := '0';
  signal rx_clear_sync_2     : std_ulogic := '0';
  signal rx_clear_sys_d      : std_ulogic := '0';
  signal rx_clear_sys_pulse  : std_ulogic := '0';
  signal rx_clear_mac_pulse  : std_ulogic := '0';
  signal mac_rx_frame_d      : std_ulogic := '0';
  signal spi_int_n           : std_ulogic := '1';

  --------------------------------------------------------------------
  -- TX-Async-FIFO (SPI-Domain -> MAC-TX-Domain)
  --------------------------------------------------------------------
  signal txfifo_wr_en    : std_ulogic := '0';
  signal txfifo_wr_data  : std_ulogic_vector(7 downto 0) := (others => '0');
  signal txfifo_wr_full  : std_ulogic;

  signal txfifo_rd_en    : std_ulogic := '0';
  signal txfifo_rd_data  : std_ulogic_vector(7 downto 0);
  signal txfifo_rd_empty : std_ulogic;

  --------------------------------------------------------------------
  -- RX-Async-FIFO (MAC-RX-Domain -> SPI-Domain)
  --------------------------------------------------------------------
  signal rxfifo_wr_en    : std_ulogic := '0';
  signal rxfifo_wr_data  : std_ulogic_vector(7 downto 0) := (others => '0');
  signal rxfifo_wr_full  : std_ulogic;

  signal rxfifo_rd_en    : std_ulogic := '0';
  signal rxfifo_rd_data  : std_ulogic_vector(7 downto 0);
  signal rxfifo_rd_empty : std_ulogic;
  signal spi_rx_bytes_sent   : unsigned(15 downto 0) := (others => '0');

  --------------------------------------------------------------------
  -- MAC-TX-Steuerung
  --------------------------------------------------------------------
  signal mac_tx_active   : std_ulogic := '0';
  signal tx_start_sync_1 : std_ulogic := '0';
  signal tx_start_sync_2 : std_ulogic := '0';
  signal tx_start_pulse  : std_ulogic := '0';
  signal tx_prefetch_valid : std_ulogic := '0';
  signal tx_length       : unsigned(15 downto 0) := (others => '0');
  signal tx_byte_sent_count : unsigned(15 downto 0) := (others => '0');
  signal tx_clear_toggle_mac : std_ulogic := '0';
  signal tx_clear_sync_sys_1 : std_ulogic := '0';
  signal tx_clear_sync_sys_2 : std_ulogic := '0';
  signal tx_clear_sys_pulse  : std_ulogic := '0';
  signal tx_clear_mac_d      : std_ulogic := '0';
  signal tx_clear_mac_pulse  : std_ulogic := '0';
  signal spi_tx_data_count   : unsigned(15 downto 0) := (others => '0');
  
  signal spi_resume_data     : std_ulogic := '0';
  signal spi_cs_falling      : std_ulogic := '0';

  signal mac_tx_preamble_bytes_sent : unsigned(3 downto 0) := (others => '0');

  type t_tx_SM is (s_Idle, s_PrimeTx, s_Transmit, s_End);
  signal sm_tx_ethernet : t_tx_SM := s_Idle;

begin

  --------------------------------------------------------------------
  -- TX-FIFO-Instanz
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
  -- RX-FIFO-Instanz
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
  -- SPI-SCK und CS in clk_sys_i-Domain synchronisieren
  --------------------------------------------------------------------
  process(clk_sys_i, rst_sys_i)
  begin
    if rst_sys_i = '1' then
      rx_clear_sys_d     <= '0';
      rx_clear_sys_pulse <= '0';
      tx_clear_sync_sys_1 <= '0';
      tx_clear_sync_sys_2 <= '0';
      tx_clear_sys_pulse  <= '0';
      reg_rx_len_sync_1      <= (others => '0');
      reg_rx_len_sync_2      <= (others => '0');
      reg_rx_ready_sync_1    <= '0';
      reg_rx_ready_sync_2    <= '0';
      reg_rx_overflow_sync_1 <= '0';
      reg_rx_overflow_sync_2 <= '0';
      reg_rx_len_sys         <= (others => '0');
      reg_rx_ready_sys       <= '0';
      reg_rx_overflow_sys    <= '0';
    elsif rising_edge(clk_sys_i) then
      rx_clear_sys_d     <= rx_clear_toggle_sys;
      rx_clear_sys_pulse <= rx_clear_toggle_sys xor rx_clear_sys_d;
      tx_clear_sync_sys_1 <= tx_clear_toggle_mac;
      tx_clear_sync_sys_2 <= tx_clear_sync_sys_1;
      tx_clear_sys_pulse  <= tx_clear_sync_sys_1 xor tx_clear_sync_sys_2;

      -- Synchronisation der mac->sys Flags/Länge
      reg_rx_len_sync_1      <= reg_rx_len;
      reg_rx_len_sync_2      <= reg_rx_len_sync_1;
      reg_rx_ready_sync_1    <= reg_rx_ready;
      reg_rx_ready_sync_2    <= reg_rx_ready_sync_1;
      reg_rx_overflow_sync_1 <= reg_rx_overflow;
      reg_rx_overflow_sync_2 <= reg_rx_overflow_sync_1;

      reg_rx_overflow_sys <= reg_rx_overflow_sync_2;
      reg_rx_ready_sys    <= reg_rx_ready_sync_2;
      reg_rx_len_sys      <= reg_rx_len_sync_2;

      if rx_clear_sys_pulse = '1' then
        reg_rx_ready_sys    <= '0';
        reg_rx_overflow_sys <= '0';
        reg_rx_len_sys      <= (others => '0');
      end if;
    end if;
  end process;

  process(clk_sys_i, rst_sys_i)
  begin
    if rst_sys_i = '1' then
      cs_meta       <= '1';
      spi_cs_sync   <= '1';
      spi_cs_sync_d <= '1';
      spi_cs_falling <= '0';
      
      sck_meta      <= '0';
      sck_sync      <= '0';
      sck_sync_d    <= '0';
      sck_rising    <= '0';
      
      mosi_meta     <= '0';
      mosi_sync     <= '0';
    elsif rising_edge(clk_sys_i) then
      -- 3-Stage Synchronizer for CS
      cs_meta       <= spi_cs_n_i;
      spi_cs_sync   <= cs_meta;
      spi_cs_sync_d <= spi_cs_sync;
      
      if (spi_cs_sync_d = '1') and (spi_cs_sync = '0') then
        spi_cs_falling <= '1';
      else
        spi_cs_falling <= '0';
      end if;
      
      -- 3-Stage Synchronizer for SCK
      sck_meta      <= spi_sck_i;
      sck_sync      <= sck_meta;
      sck_sync_d    <= sck_sync;
      
      if (sck_sync = '1') and (sck_sync_d = '0') then
        sck_rising <= '1';
      else
        sck_rising <= '0';
      end if;
      
      -- 3-Stage Synchronizer for MOSI (to match SCK latency)
      mosi_meta     <= spi_mosi_i;
      mosi_sync     <= mosi_meta;
      -- We use mosi_sync (Stage 2) when sck_rising (Stage 2->3 transition) is detected
      
      -- Removed sck_falling generation as it is not used anymore
      sck_falling <= '0'; 
    end if;
  end process;

  --------------------------------------------------------------------
  -- SPI-State-Machine (Byte-weise, Mode 0)
  --------------------------------------------------------------------
  process(clk_sys_i, rst_sys_i)
    variable next_addr      : std_ulogic_vector(6 downto 0);
    variable next_addr_inc  : std_ulogic_vector(6 downto 0);
    variable rw_cmd         : std_ulogic;
    variable next_shift_out : std_ulogic_vector(7 downto 0);
    variable shift_in_var   : std_ulogic_vector(7 downto 0);
    variable shift_out_var  : std_ulogic_vector(7 downto 0);
    variable next_wr_en     : std_ulogic;
    variable is_first_byte  : std_ulogic;
    variable data_cnt_tx_var   : unsigned(15 downto 0);
  begin
    if rst_sys_i = '1' then
      spi_state     <= SPI_IDLE;
      spi_resume_data <= '0';
      spi_bit_cnt   <= 0;
      spi_shift_in  <= (others => '0');
      spi_shift_out <= (others => '0');
      spi_miso_lat  <= '0';
      spi_rw_n      <= '0';
      spi_addr      <= (others => '0');
      spi_miso_drv  <= '0';
      txfifo_wr_en  <= '0';
      rxfifo_rd_en  <= '0';
      next_wr_en    := '0';
      is_first_byte := '0';
      data_cnt_tx_var  := (others => '0');
      spi_tx_data_count <= (others => '0');
    elsif rising_edge(clk_sys_i) then
      data_cnt_tx_var := spi_tx_data_count;
      if next_wr_en = '1' then
        if is_first_byte = '0' then
          txfifo_wr_en <= '1';
        end if;
        next_wr_en   := '0';
        else
          txfifo_wr_en <= '0';
      end if;
      rxfifo_rd_en <= '0';
      shift_in_var  := spi_shift_in;
      shift_out_var := spi_shift_out;


      if spi_cs_sync = '1' then
        spi_state <= SPI_IDLE;
        if spi_resume_data = '1' then
          -- Weitermachen mit Datenübertragung
          spi_state      <= SPI_DATA;
          else
            spi_transaction_done <= '0';
        end if;
        spi_bit_cnt <= 0;
        spi_miso_lat <= '0';

      else
        if sck_rising = '1' then
          -- Bit von MOSI einlesen (using mosi_sync from Stage 2)
          shift_in_var(7 - spi_bit_cnt) := mosi_sync;
          
          if spi_bit_cnt = 7 then
            spi_bit_cnt   <= 0;
            rw_cmd        := spi_rw_n;
            next_addr     := spi_addr;
            if unsigned(next_addr) = 16#7F# then
              next_addr_inc := next_addr;  -- saturieren, kein Wrap
            else
              next_addr_inc := std_ulogic_vector(unsigned(next_addr) + 1);
            end if;
            next_shift_out := shift_out_var;

            case spi_state is
              when SPI_IDLE =>
                -- erstes Byte = CMD
                rw_cmd    := shift_in_var(7);
                next_addr := shift_in_var(6 downto 0);
                spi_rw_n  <= rw_cmd;
                spi_addr  <= next_addr;
                spi_state <= SPI_DATA;

                if rw_cmd = '0' then
                  -- Prefetch erstes Read-Byte
                  next_shift_out := (others => '0');
                  case next_addr is
                    when "0100000" =>      -- 0x20 RX_LEN low
                      next_shift_out := std_ulogic_vector(reg_rx_len_sys(7 downto 0));
                    when "0100001" =>      -- 0x21 RX_LEN high
                      next_shift_out := std_ulogic_vector(reg_rx_len_sys(15 downto 8));
                    when "0100010" =>      -- 0x22 RX_STATUS
                      next_shift_out(0) := reg_rx_ready_sys;
                      next_shift_out(1) := reg_rx_overflow_sys;
                      next_shift_out(7 downto 2) := (others => '0');
                    when others =>
                      -- 0x30.. : RX-Datenfenster aus RX-FIFO (alles >= 0x30)
                      if unsigned(next_addr) >= 16#30# then
                        if rxfifo_rd_empty = '0' then
                          is_first_byte := '1';
                          next_shift_out := rxfifo_rd_data;
                          rxfifo_rd_en   <= '1';
                        end if;
                      end if;
                  end case;
                  spi_shift_out <= next_shift_out;
                else
                  spi_shift_out <= (others => '0');
                end if;

              when SPI_DATA =>

                if rw_cmd = '1' then
                  is_first_byte := '0';
                  -- WRITE
                  -- 0x10.. : TX-Datenfenster -> TX-FIFO

                  -- if-else zum vermeiden von kollisionen bei next_addr und der hochzählenden länge bei längeren paketen!
                  if (spi_resume_data = '1') then
                    -- Weitermachen eines vorher unterbrochenen Schreibvorgangs
                      if txfifo_wr_full = '0' then
                          txfifo_wr_data <= shift_in_var;
                          next_wr_en   := '1';
                      end if;
                      
                      data_cnt_tx_var := data_cnt_tx_var + 1;
                      -- Wenn alle erwarteten Bytes geschrieben, nach Abschluss zurück in Idle
                      if data_cnt_tx_var >= reg_tx_len then
                        spi_resume_data <= '0';
                        spi_state       <= SPI_IDLE;
                        spi_transaction_done <= '1';
                      else
                        spi_resume_data <= '1';
                      end if;
                  else
                  
                  case next_addr is
                    when "0000000" =>  -- 0x00 TX_LEN low
                      reg_tx_len(7 downto 0) <= unsigned(shift_in_var);
                      data_cnt_tx_var           := (others => '0');
                      spi_resume_data        <= '0';
                    when "0000001" =>  -- 0x01 TX_LEN high
                      reg_tx_len(15 downto 8) <= unsigned(shift_in_var);
                      data_cnt_tx_var           := (others => '0');
                      spi_resume_data        <= '0';
                    when "0000010" =>  -- 0x02 TX_CTRL
                      reg_tx_start <= shift_in_var(0);
                      data_cnt_tx_var  := (others => '0');
                      spi_resume_data <= '0';
                    when "0100010" =>  -- 0x22 RX_STATUS clear request
                      if shift_in_var(0) = '1' then
                        rx_clear_toggle_sys <= not rx_clear_toggle_sys;
                      end if;
                    when others =>
                    if unsigned(next_addr) >= 16#10# then
                        if txfifo_wr_full = '0' then
                          txfifo_wr_data <= shift_in_var;
                          next_wr_en   := '1';
                        end if;
                        
                        data_cnt_tx_var := data_cnt_tx_var + 1;
                        -- Wenn alle erwarteten Bytes geschrieben, nach Abschluss zurück in Idle
                        if data_cnt_tx_var >= reg_tx_len then
                          spi_resume_data <= '0';
                          spi_state       <= SPI_IDLE;
                          spi_transaction_done <= '1';
                        else
                          spi_resume_data <= '1';
                        end if;
                      end if;
                  end case;
                  end if;
                  spi_shift_out <= (others => '0');
                else

                  if (spi_resume_data = '1') then
                    if rxfifo_rd_empty = '0' then
                          next_shift_out := rxfifo_rd_data;
                          rxfifo_rd_en   <= '1';
                    else
                          next_shift_out := (others => '0');
                    end if;
                    
                    spi_rx_bytes_sent <= spi_rx_bytes_sent + 1;
                    spi_transaction_done <= '0';
                    -- Wenn alle erwarteten Bytes geschrieben, nach Abschluss zurück in Idle
                    if spi_rx_bytes_sent + 1 >= reg_rx_len_sys then
                      spi_resume_data <= '0';
                      spi_state       <= SPI_IDLE;
                      spi_transaction_done <= '1';
                      spi_rx_bytes_sent <= (others => '0');
                    else
                      spi_resume_data <= '1';
                    end if;
                  else 
                  -- READ:
                  next_shift_out := (others => '0');
                  case next_addr_inc is
                    when "0100000" =>      -- 0x20 RX_LEN low
                      next_shift_out := std_ulogic_vector(reg_rx_len_sys(7 downto 0));
                    when "0100001" =>      -- 0x21 RX_LEN high
                      next_shift_out := std_ulogic_vector(reg_rx_len_sys(15 downto 8));
                    when "0100010" =>      -- 0x22 RX_STATUS
                      next_shift_out(0) := reg_rx_ready_sys;
                      next_shift_out(1) := reg_rx_overflow_sys;
                      next_shift_out(7 downto 2) := (others => '0');
                    when others =>
                      -- 0x30.. : RX-Datenfenster aus RX-FIFO (alles >= 0x30)
                      if unsigned(next_addr) >= 16#30# then
                        if rxfifo_rd_empty = '0' then
                          next_shift_out := rxfifo_rd_data;
                          rxfifo_rd_en   <= '1';
                        else
                          next_shift_out := (others => '0');
                        end if;
                        
                        spi_rx_bytes_sent <= spi_rx_bytes_sent + 1;
                        spi_transaction_done <= '0';
                        -- Wenn alle erwarteten Bytes geschrieben, nach Abschluss zurück in Idle
                        -- Offset 1 weil reg_rx_len ist 1 indexiert
                        if spi_rx_bytes_sent +1 >= reg_rx_len_sys then
                          spi_resume_data <= '0';
                          spi_state       <= SPI_IDLE;
                          spi_transaction_done <= '1';
                          spi_rx_bytes_sent <= (others => '0');
                        else
                          spi_resume_data <= '1';
                        end if;
                      end if;
                  end case;
                  end if;
                  spi_shift_out <= next_shift_out;
                end if;
                -- Adresse innerhalb eines Transfers automatisch hochzählen
                spi_addr <= next_addr_inc;
                spi_tx_data_count <= data_cnt_tx_var;
            when others =>
              spi_state <= SPI_IDLE;
            end case;

            -- Shift-Register updaten
            spi_shift_in  <= shift_in_var;
            spi_shift_out <= next_shift_out;
            
            -- MISO Update on Rising Edge (Delayed)
            -- This ensures data is ready well before the next Master Rising Edge
            if spi_bit_cnt = 7 then
               spi_miso_lat <= next_shift_out(7);
            else
               spi_miso_lat <= next_shift_out(7 - (spi_bit_cnt + 1));
            end if;
          else
            spi_bit_cnt <= spi_bit_cnt + 1;
            spi_shift_in  <= shift_in_var;
            -- MISO Update on Rising Edge (Delayed)
            spi_miso_lat <= next_shift_out(7 - (spi_bit_cnt + 1));
          end if;
        end if;
        
        -- Removed sck_falling update to avoid race condition at high speeds
      end if;
      -- Auch außerhalb Bitende den Zähler zurückschreiben
      spi_tx_data_count <= data_cnt_tx_var;
    end if;
  end process;

  -- Treibt MISO stets aus dem aktuellen Shiftregister heraus.
  -- Mode 0: der Master sampelt auf rising edge, daher muss das Bit bereits
  -- vor der Flanke stabil sein.
  --spi_miso_o <= spi_shift_out(7 - spi_bit_cnt);
  spi_miso_o <= spi_miso_lat;
  --------------------------------------------------------------------
  -- TX-Start in MAC-TX-Clockdomain synchronisieren
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
  -- MAC-TX-Logik: liest aus TX-FIFO und speist MAC
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
    elsif rising_edge(mac_tx_clock_i) then

      txfifo_rd_en <= '0';
      case sm_tx_ethernet is
        when s_Idle =>
          mac_tx_data_o   <= (others => '0');
          mac_tx_enable_o <= '0';

          if (tx_start_pulse = '1') and (mac_tx_active = '0') then
            if txfifo_rd_empty = '0' then
            
            -- Prefetch nächste Position für die nächste Byte-Sent-Pulse
              
              
              sm_tx_ethernet <= s_PrimeTx;
              mac_tx_data_o <= txfifo_rd_data;
              mac_tx_enable_o <= '1';  
              tx_prefetch_valid <= '1';
            
            end if;
          end if;


        when s_PrimeTx =>
          mac_tx_preamble_bytes_sent <= mac_tx_preamble_bytes_sent + 1;
          mac_tx_active <= '1';
          tx_prefetch_valid <= '1'; -- neues Wort kommt im nächsten Takt
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
          -- Jedes mal, wenn der MAC ein Byte bestätigt, das nächste liefern
          if mac_tx_byte_sent_i = '1' then
            -- Nach byte_sent_i liegt das zuvor vorgefetchte Wort an.
            if tx_prefetch_valid = '1' then
              mac_tx_data_o <= txfifo_rd_data;
              tx_byte_sent_count <= tx_byte_sent_count + 1;
            end if;
          -- Prefetch das nächste Wort
            if txfifo_rd_empty = '0' then
              txfifo_rd_en <= '1';
              tx_prefetch_valid <= '0';
            else
              if (tx_byte_sent_count + 1) >= reg_tx_len then
                -- Ende des Pakets erreicht
                sm_tx_ethernet <= s_End;
              end if;
              
            end if;
          end if;
        -- Wenn gerade ein rd_en ausgelöst wurde, ist im nächsten Takt das Wort gültig
        if txfifo_rd_en = '1' then
          tx_prefetch_valid <= '1';
          if txfifo_rd_en = '1' then
            tx_prefetch_valid <= '1';
          end if;
        end if;
            
        when s_End =>
          mac_tx_enable_o <= '0';
          mac_tx_data_o   <= (others => '0');
          mac_tx_active   <= '0';
          sm_tx_ethernet  <= s_Idle;
          mac_tx_active   <= '0';
            tx_prefetch_valid <= '0';
            tx_byte_sent_count <= (others => '0');
            tx_clear_toggle_mac <= not tx_clear_toggle_mac; -- FIFO-Pointer nach TX zurücksetzen

      end case;
      
      

    end if;
  end process;

  --------------------------------------------------------------------
  -- TX-FIFO-Clear-Synchronisation (MAC -> SYS und MAC)
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
  -- MAC-RX-Logik: schreibt empfangene Bytes in RX-FIFO
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
      rx_clear_sync_1   <= '0';
      rx_clear_sync_2   <= '0';
      mac_rx_frame_d    <= '0';
    elsif rising_edge(mac_rx_clock_i) then

      rxfifo_wr_en <= '0';
      rx_clear_sync_1 <= rx_clear_toggle_sys;
      rx_clear_sync_2 <= rx_clear_sync_1;
      mac_rx_frame_d  <= mac_rx_frame_i;

      len_next   := reg_rx_len;
      ready_next := reg_rx_ready;
      ovf_next   := reg_rx_overflow;

      -- Clear durch SPI-Seite angefordert (hat immer Priorität)
      if rx_clear_sync_1 /= rx_clear_sync_2 then
        len_next   := (others => '0');
        ready_next := '0';
        ovf_next   := '0';
      end if;

      -- Start einer neuen RX-Session: zurücksetzen, aber Overflow markieren,
      -- falls vorher noch READY anlag.
      if (mac_rx_frame_i = '1') and (mac_rx_frame_d = '0') then
        len_next   := (others => '0');
        ready_next := '0';
        if reg_rx_ready = '1' then
          ovf_next := '1';
        else
          ovf_next := '0';
        end if;
      end if;

      -- Wenn bereits ein Paket anliegt und SPI es nicht abgeholt hat,
      -- keine neuen Bytes schreiben.
      if ready_next = '0' then

      if (mac_rx_frame_i = '1') and (mac_rx_byte_rcv_i = '1') then
        if rxfifo_wr_full = '0' then
          rxfifo_wr_data <= mac_rx_data_i;
          rxfifo_wr_en   <= '1';
          len_next       := len_next + 1;
        else
          ovf_next := '1';
        end if;
      end if;

      -- Frame-Ende erkennen: Frame war aktiv, jetzt nicht mehr
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

  -- Interrupt: aktiv low, wenn Daten bereit oder Overflow
  spi_int_n <= not (reg_rx_ready_sys or reg_rx_overflow_sys);
  rx_clear_mac_pulse <= rx_clear_sync_1 xor rx_clear_sync_2;
  spi_int_n_o <= spi_int_n;

end architecture rtl;
