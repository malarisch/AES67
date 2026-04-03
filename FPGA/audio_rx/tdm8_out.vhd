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
--
-- FSYNC input is already in the correct format: 1 BCLK wide pulse,
-- synchronous to BIT_CLK, shared with the DACs. It is passed through
-- as LRCK directly.
--
-- Timing (F = falling edge, R = rising edge):
--
--  SCLK: ...F...R...F...R...F...R...F...R...F...R...
--  LRCK:        ___/‾‾‾‾‾‾‾‾‾‾‾\____________________
--                   ^sampled high    ^sampled low
--  DOUT: ---------X MSB        X b22        X b21
--                   ^valid          ^valid
--
-- Input takes:
-- - DATA_CH0..DATA_CH7 parallel audio words
-- - BIT_CLK Bit clock (256*Fs)
-- - FSYNC Frame sync (1 BCLK wide pulse, shared with DACs)
-- - RESET Asynchronous Reset (Active Low)
--
-- Output provides:
-- - DOUT TDM serial data (valid on rising SCLK)
-- - LRCK Frame sync passthrough
--
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
    LRCK      : out std_logic; -- Frame sync passthrough

    -- Control ports
    RESET   : in std_logic; -- Asynchronous Reset (Active Low)

    -- Parallel input ports
    DATA_CH0 : in std_logic_vector(width - 1 downto 0);
    DATA_CH1 : in std_logic_vector(width - 1 downto 0);
    DATA_CH2 : in std_logic_vector(width - 1 downto 0);
    DATA_CH3 : in std_logic_vector(width - 1 downto 0);
    DATA_CH4 : in std_logic_vector(width - 1 downto 0);
    DATA_CH5 : in std_logic_vector(width - 1 downto 0);
    DATA_CH6 : in std_logic_vector(width - 1 downto 0);
    DATA_CH7 : in std_logic_vector(width - 1 downto 0)
  );
end tdm8_out;

architecture rtl of tdm8_out is

  signal shift_reg    : std_logic_vector(31 downto 0);
  signal bit_count    : unsigned(7 downto 0); -- 0..255 counts through entire frame
  signal running      : std_logic;

  -- Latched input data array
  type channel_array_t is array (0 to 7) of std_logic_vector(width - 1 downto 0);
  signal ch_data_lat  : channel_array_t;

  -- Pad audio data to 32 bits (left-justified, zero-padded)
  function pad32(d : std_logic_vector(width - 1 downto 0)) return std_logic_vector is
    variable result : std_logic_vector(31 downto 0) := (others => '0');
  begin
    result(31 downto 32 - width) := d;
    return result;
  end function;

begin

  -- FSYNC is already the correct LRCK signal, pass through directly
  LRCK <= FSYNC;

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
      for i in 0 to 7 loop
        ch_data_lat(i) <= (others => '0');
      end loop;

    elsif falling_edge(BIT_CLK) then

      if (FSYNC = '1') then
        -- Frame start: FSYNC is high for exactly 1 BCLK.
        -- Latch all channel data and output MSB of slot 0.
        ch_data_lat(0) <= DATA_CH0;
        ch_data_lat(1) <= DATA_CH1;
        ch_data_lat(2) <= DATA_CH2;
        ch_data_lat(3) <= DATA_CH3;
        ch_data_lat(4) <= DATA_CH4;
        ch_data_lat(5) <= DATA_CH5;
        ch_data_lat(6) <= DATA_CH6;
        ch_data_lat(7) <= DATA_CH7;

        shift_reg <= pad32(DATA_CH0);
        DOUT      <= DATA_CH0(width - 1);
        bit_count <= to_unsigned(1, 8);
        running   <= '1';

      elsif (running = '1') then
        -- Shift out current MSB
        DOUT      <= shift_reg(31);
        shift_reg <= shift_reg(30 downto 0) & '0';

        -- At end of slot (bit 31), pre-load next slot's data
        if (bit_count(4 downto 0) = "11111") then
          next_slot := to_integer(bit_count(7 downto 5)) + 1;
          if (next_slot <= 7) then
            shift_reg <= pad32(ch_data_lat(next_slot));
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
