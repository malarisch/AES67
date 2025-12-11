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