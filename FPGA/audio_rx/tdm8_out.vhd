---------------------------------------------------------------------------------
-- Engineer:      Based on i2s_out
--
-- Create Date:   2026
-- Design Name:   tdm8_out
--
-- Description:
--
-- This module provides a bridge between a parallel audio source (e.g. RX
-- ringbuffer, 8 channels) and a TDM-8 serial device (audio DAC).
--
-- TDM-8 format per DAC spec:
-- - SCLK = 256 * Fs (8 channels x 32 bits)
-- - LRCK = Fs, pulsed high for one SCLK period at frame start
-- - LRCK is sampled valid on the rising SCLK edge preceding the MSB
-- - Data (DOUT) is MSB-first, valid on rising SCLK edge
-- - Each slot is 32 bits wide, left-justified, zero-padded
-- - Data transitions occur on falling SCLK edge

-- - BIT_CLK Bit clock (256*Fs)

--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity tdm8_out is
  generic (width : integer := 24);

  port (
    -- TDM ports
    FSYNC     : in  std_logic; -- Frame sync (1 BCLK pulse, shared with DACs)
    BIT_CLK   : in  std_logic; -- Bit clock (256*Fs)
    DOUT      : out std_logic; -- Serial data output (valid on rising SCLK)

    -- Control ports
    RESET   : in std_logic; -- Asynchronous Reset (Active Low)


    DIN      : IN std_logic_vector(width * 8 - 1 downto 0)
  );
end tdm8_out;

architecture rtl of tdm8_out is

  signal shift_reg    : std_logic_vector(31 downto 0);
  signal bit_count    : unsigned(7 downto 0); -- 0..255 counts through entire frame
  signal running      : std_logic;
  signal z_fsync      : std_logic;
  -- Latched input data array
  
  -- Pad audio data to 32 bits (left-justified, zero-padded)
  function pad32(d : std_logic_vector(width - 1 downto 0)) return std_logic_vector is
    variable result : std_logic_vector(31 downto 0) := (others => '0');
  begin
    result(31 downto 32 - width) := d;
    return result;
  end function;

begin



  -------------------------------------------------------------------------
  -- Data shift-out on falling SCLK edge.
  -- Data transitions on falling edge so it is stable and valid
  -- when sampled on the following rising edge.
  --
  -- When FSYNC is high (sampled on falling edge), we latch channel data
  -- and output MSB of slot 0. The DAC sees LRCK=1 and MSB on the next
  -- rising edge. On subsequent falling edges we shift out the remaining
  -- bits.
  -------------------------------------------------------------------------
  tdm_proc : process (RESET, BIT_CLK)
    variable next_slot : integer range 0 to 7;
  begin
    if (RESET = '0') then

      shift_reg  <= (others => '0');
      bit_count  <= (others => '0');
      running    <= '0';
      DOUT       <= '0';


    elsif falling_edge(BIT_CLK) then
      z_fsync <= FSYNC;


      if (FSYNC = '1' and z_fsync = '0') then
        -- Frame start

        shift_reg <= pad32(DIN(width - 1 downto 0));
        DOUT      <= DIN(width - 1);
        bit_count <= to_unsigned(1, 8);
        running   <= '1';
      end if;
      
      if (running = '1') then
        -- Shift out current MSB
        DOUT      <= shift_reg(31);
        shift_reg <= shift_reg(30 downto 0) & '0';

        -- At end of slot (bit 31), pre-load next slot's data
        if (bit_count(4 downto 0) = "11111") then
          next_slot := to_integer(bit_count(7 downto 5)) + 1;
          if (next_slot <= 7) then
            shift_reg <= pad32(DIN((next_slot + 1) * width - 1 downto next_slot * width));
          end if;
        end if;

        if (bit_count = 255) then
          bit_count <= (others => '0');
          running   <= '0';
        else
          bit_count <= bit_count + 1;
        end if;

      else
        DOUT <= '0';
      end if;

    end if;
  end process tdm_proc;

end rtl;
