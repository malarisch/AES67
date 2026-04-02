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
-- TDM-8 format: 8 channels time-multiplexed on a single data line.
-- FSYNC indicates frame start (active high for one BCLK cycle).
-- Each slot is 32 bits wide, total frame = 256 BCLK cycles.
-- Data is MSB-first, left-justified within each 32-bit slot.
-- Only the first 'width' bits contain audio data, remaining bits are zero-padded.
--
-- Input takes:
-- -DATA_CH0..DATA_CH7 parallel audio words
-- -BIT_CLK Bit clock (256*fs for 8 channels x 32 bits)
-- -FSYNC Frame sync (active high for one BCLK at frame start)
-- -RESET Asynchronous Reset (Active Low)
--
-- Output provides:
-- -DOUT TDM serial data
-- -TDM_FSYNC TDM frame sync (one BCLK cycle pulse at frame start)
--
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity tdm8_out is
  generic (width : integer := 24);

  port (
    -- TDM ports
    FSYNC     : in  std_logic; -- System frame sync (48kHz, 50% duty)
    BIT_CLK   : in  std_logic; -- Bit clock (256*fs)
    DOUT      : out std_logic; -- Serial data output
    TDM_FSYNC : out std_logic; -- TDM frame sync (one BCLK pulse at frame start)

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

  -- BIT_CLK domain signals
  signal shift_reg    : std_logic_vector(width - 1 downto 0);
  signal bit_count    : integer range 0 to 31;
  signal slot_count   : integer range 0 to 7;
  signal fsync_prev   : std_logic;

  -- Latched input data array (BIT_CLK domain)
  type channel_array_t is array (0 to 7) of std_logic_vector(width - 1 downto 0);
  signal ch_data_lat  : channel_array_t;

begin

  -------------------------------------------------------------------------
  -- TDM Parallel to Serial (BIT_CLK Domain)
  -- FSYNC rising edge marks the start of slot 0.
  -- Data is shifted out on falling edge of BIT_CLK, MSB first.
  -- Each slot is 32 BCLK wide; first 'width' bits are audio data,
  -- remaining bits are zero-padded.
  -------------------------------------------------------------------------
  tdm_out_proc : process (RESET, BIT_CLK)
  begin
    if (RESET = '0') then

      shift_reg  <= (others => '0');
      bit_count  <= 0;
      slot_count <= 0;
      fsync_prev <= '0';
      DOUT       <= '0';
      TDM_FSYNC  <= '0';
      for i in 0 to 7 loop
        ch_data_lat(i) <= (others => '0');
      end loop;

    elsif falling_edge(BIT_CLK) then

      -- Detect FSYNC rising edge
      fsync_prev <= FSYNC;

      if (FSYNC = '1' and fsync_prev = '0') then
        -- Frame start: latch all input channels and load slot 0
        ch_data_lat(0) <= DATA_CH0;
        ch_data_lat(1) <= DATA_CH1;
        ch_data_lat(2) <= DATA_CH2;
        ch_data_lat(3) <= DATA_CH3;
        ch_data_lat(4) <= DATA_CH4;
        ch_data_lat(5) <= DATA_CH5;
        ch_data_lat(6) <= DATA_CH6;
        ch_data_lat(7) <= DATA_CH7;

        shift_reg  <= DATA_CH0;
        slot_count <= 0;
        bit_count  <= 0;

        -- Output MSB of first slot immediately
        DOUT      <= DATA_CH0(width - 1);
        TDM_FSYNC <= '1';

      else
        -- TDM FSYNC is only one BCLK cycle wide
        TDM_FSYNC <= '0';
        -- Shift out data
        if (bit_count < width) then
          -- Output MSB first
          DOUT <= shift_reg(width - 1);
          shift_reg(width - 1 downto 1) <= shift_reg(width - 2 downto 0);
          shift_reg(0) <= '0';
        else
          -- Zero padding after audio data
          DOUT <= '0';
        end if;

        if (bit_count = 31) then
          -- End of slot: load next channel
          bit_count <= 0;
          if (slot_count < 7) then
            slot_count <= slot_count + 1;
            shift_reg  <= ch_data_lat(slot_count + 1);
          end if;
        else
          bit_count <= bit_count + 1;
        end if;
      end if;
    end if;
  end process tdm_out_proc;

end rtl;
