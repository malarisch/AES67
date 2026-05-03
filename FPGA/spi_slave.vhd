--------------------------------------------------------------------------------
-- PROJECT: SPI MASTER AND SLAVE FOR FPGA
--------------------------------------------------------------------------------
-- AUTHORS: Jakub Cabal <jakubcabal@gmail.com>
-- LICENSE: The MIT License, please read LICENSE file
-- WEBSITE: https://github.com/jakubcabal/spi-fpga
--------------------------------------------------------------------------------

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

-- THE SPI SLAVE MODULE SUPPORT ONLY SPI MODE 0 (CPOL=0, CPHA=0)!!!

entity SPI_SLAVE is
    Generic (
        WORD_SIZE : natural := 8 -- size of transfer word in bits, must be power of two
    );
    Port (
        CLK      : in  std_logic; -- system clock
        RST      : in  std_logic; -- high active synchronous reset
        -- SPI SLAVE INTERFACE
        SCLK     : in  std_logic; -- SPI clock
        CS_N     : in  std_logic; -- SPI chip select, active in low
        MOSI     : in  std_logic; -- SPI serial data from master to slave
        MISO     : out std_logic; -- SPI serial data from slave to master
        -- USER INTERFACE
        DIN      : in  std_logic_vector(WORD_SIZE-1 downto 0); -- data for transmission to SPI master
        DIN_VLD  : in  std_logic; -- when DIN_VLD = 1, data for transmission are valid
        DIN_RDY  : out std_logic; -- when DIN_RDY = 1, SPI slave is ready to accept valid data for transmission
        DOUT     : out std_logic_vector(WORD_SIZE-1 downto 0); -- received data from SPI master
        DOUT_VLD : out std_logic  -- when DOUT_VLD = 1, received data are valid
    );
end entity;

architecture RTL of SPI_SLAVE is
    -- clog function because gowin somehow does not know  ieee.math_real
    function clog2(n : natural) return natural is
        variable v : natural := n - 1;
        variable r : natural := 0;
    begin
        while v > 0 loop
            v := v / 2;
            r := r + 1;
        end loop;
        return r;
    end function;

    constant BIT_CNT_WIDTH : natural := clog2(WORD_SIZE);

    --signal sclk_meta          : std_logic;
    signal cs_n_meta          : std_logic;
    --signal mosi_meta          : std_logic;
    signal sclk_reg           : std_logic;
    signal cs_n_reg           : std_logic;
    signal mosi_reg           : std_logic;
    signal spi_clk_reg        : std_logic;
    signal spi_clk_redge_en   : std_logic;
    signal spi_clk_fedge_en   : std_logic;
    signal bit_cnt            : unsigned(BIT_CNT_WIDTH-1 downto 0);
    signal bit_cnt_max        : std_logic;
    signal last_bit_en        : std_logic;
    signal load_data_en       : std_logic;
    signal tx_shreg           : std_logic_vector(WORD_SIZE-1 downto 0);
    signal rx_shreg           : std_logic_vector(WORD_SIZE-1 downto 0);
    signal slave_ready        : std_logic;
    signal shreg_busy         : std_logic;
    signal rx_data_vld        : std_logic;

begin

    -- -------------------------------------------------------------------------
    --  INPUT SYNCHRONIZATION REGISTERS
    -- -------------------------------------------------------------------------

    sync_ffs_p : process (CLK)
    begin
        if (rising_edge(CLK)) then
            cs_n_meta <= CS_N;
            cs_n_reg  <= cs_n_meta;
            if (cs_n_reg = '0') then
                sclk_reg  <= SCLK;
                mosi_reg <= MOSI;
            else
                sclk_reg  <= '0';
            end if;
        end if;
    end process;
    -- -------------------------------------------------------------------------
    --  SPI CLOCK REGISTER
    -- -------------------------------------------------------------------------

    -- The SPI clock register is necessary for clock edge detection.
    spi_clk_reg_p : process (CLK)
    begin
        if (rising_edge(CLK)) then
            if (RST = '1') then
                spi_clk_reg <= '0';
            else
                spi_clk_reg <= sclk_reg;
            end if;
        end if;
    end process;

    -- -------------------------------------------------------------------------
    --  SPI CLOCK EDGES FLAGS
    -- -------------------------------------------------------------------------

    -- Falling edge is detect when sclk_reg=0 and spi_clk_reg=1.
    spi_clk_fedge_en <= not sclk_reg and spi_clk_reg;
    -- Rising edge is detect when sclk_reg=1 and spi_clk_reg=0.
    spi_clk_redge_en <= sclk_reg and not spi_clk_reg;

    -- -------------------------------------------------------------------------
    --  RECEIVED BITS COUNTER
    -- -------------------------------------------------------------------------

    -- The counter counts received bits from the master. Counter is enabled when
    -- falling edge of SPI clock is detected and not asserted cs_n_reg.
    bit_cnt_p : process (CLK)
    begin
        if (rising_edge(CLK)) then
            if (RST = '1') then
                bit_cnt <= (others => '0');
            elsif (spi_clk_fedge_en = '1' and cs_n_reg = '0') then
                if (bit_cnt_max = '1') then
                    bit_cnt <= (others => '0');
                else
                    bit_cnt <= bit_cnt + 1;
                end if;
            elsif cs_n_reg = '1' then
                bit_cnt <= (others => '0'); -- reset bit counter when cs is deasserted

            end if;
        end if;
    end process;

    -- The flag of maximal value of the bit counter.
    bit_cnt_max <= '1' when (bit_cnt = WORD_SIZE-1) else '0';

    -- -------------------------------------------------------------------------
    --  LAST BIT FLAG REGISTER
    -- -------------------------------------------------------------------------

    -- The flag of last bit of received byte is only registered the flag of
    -- maximal value of the bit counter.
    last_bit_en_p : process (CLK)
    begin
        if (rising_edge(CLK)) then
            if (RST = '1') then
                last_bit_en <= '0';
            else
                last_bit_en <= bit_cnt_max;
            end if;
        end if;
    end process;

    -- -------------------------------------------------------------------------
    --  RECEIVED DATA VALID FLAG
    -- -------------------------------------------------------------------------

    -- Received data from master are valid when falling edge of SPI clock is
    -- detected and the last bit of received byte is detected.
    rx_data_vld <= spi_clk_fedge_en and last_bit_en;

    -- -------------------------------------------------------------------------
    --  SHIFT REGISTER BUSY FLAG REGISTER
    -- -------------------------------------------------------------------------

    -- Data shift register is busy until it sends all input data to SPI master.
    shreg_busy_p : process (CLK)
    begin
        if (rising_edge(CLK)) then
            if (RST = '1') then
                shreg_busy <= '0';
            else
                if (DIN_VLD = '1' and (cs_n_reg = '1' or rx_data_vld = '1')) then
                    shreg_busy <= '1';
                elsif (rx_data_vld = '1') then
                    shreg_busy <= '0';
                else
                    shreg_busy <= shreg_busy;
                end if;
            end if;
        end if;
    end process;

    -- The SPI slave is ready for accept new input data when cs_n_reg is assert and
    -- shift register not busy or when received data are valid.
    slave_ready <= (cs_n_reg and not shreg_busy) or rx_data_vld;
    
    -- The new input data is loaded into the shift register when the SPI slave
    -- is ready and input data are valid.
    load_data_en <= DIN_VLD;

    -- -------------------------------------------------------------------------
    --  TX SHIFT REGISTER (MISO, CPHA=0: shift on falling edge)
    -- -------------------------------------------------------------------------

    -- For SPI mode 0 the slave must present the next MISO bit on the falling
    -- edge of SCLK so the master sees it stable on the following rising edge.
    -- MISO is driven combinationally from the MSB of tx_shreg, eliminating an
    -- extra output register and reducing SCLK->MISO latency by one sys_clk.
    --
    -- Inter-byte glitch avoidance: do NOT shift on the falling edge that
    -- follows the last bit of a byte. Otherwise MISO would briefly drop
    -- to the shifted-in '0' (or to the previous bit 6) for one sys_clk
    -- before load_data_en swaps in the next byte's MSB — long enough for
    -- the master to sample the wrong value on the following rising edge.
    -- Holding the last bit on MISO until DIN is loaded keeps the line
    -- glitch-free across byte boundaries.
    tx_shreg_p : process (CLK)
    begin
        if (rising_edge(CLK)) then
            if (load_data_en = '1') then
                tx_shreg <= DIN;
            elsif (spi_clk_fedge_en = '1' and cs_n_reg = '0' and last_bit_en = '0') then
                tx_shreg <= tx_shreg(WORD_SIZE-2 downto 0) & '0';
            end if;
        end if;
    end process;

    MISO <= tx_shreg(WORD_SIZE-1);

    -- -------------------------------------------------------------------------
    --  RX SHIFT REGISTER (MOSI, CPHA=0: sample on rising edge)
    -- -------------------------------------------------------------------------

    rx_shreg_p : process (CLK)
    begin
        if (rising_edge(CLK)) then
            if (spi_clk_redge_en = '1' and cs_n_reg = '0') then
                rx_shreg <= rx_shreg(WORD_SIZE-2 downto 0) & mosi_reg;
            end if;
        end if;
    end process;

    -- -------------------------------------------------------------------------
    --  ASSIGNING OUTPUT SIGNALS
    -- -------------------------------------------------------------------------

    DIN_RDY  <= slave_ready;
    DOUT     <= rx_shreg;
    DOUT_VLD <= rx_data_vld;

end architecture;
