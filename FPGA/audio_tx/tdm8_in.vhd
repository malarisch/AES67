---------------------------------------------------------------------------------
-- Engineer:      Based on i2s_in by Klimann Wendelin
--
-- Create Date:   2026
-- Design Name:   tdm8_in
--
-- Description:
--
-- This module provides a bridge between a TDM-8 serial device (audio ADC) and
-- a parallel device (FPGA logic, ringbuffer).
--
-- TDM-8 format: 8 channels time-multiplexed on a single data line.
-- FSYNC indicates frame start (active high for one BCLK cycle or first slot).
-- Each slot is 32 bits wide, total frame = 256 BCLK cycles.
-- Data is MSB-first, left-justified within each 32-bit slot.
-- Only the first 'width' bits of each slot are captured.
--
-- Input takes:
-- -DIN TDM serial data
-- -BIT_CLK Bit clock (256*fs for 8 channels x 32 bits)
-- -FSYNC Frame sync (rising edge marks start of channel 0)
-- -SYS_CLK System Clock for CDC
-- -RESET Asynchronous Reset (Active Low)
--
-- Output provides:
-- -DATA_CH0..DATA_CH7 parallel audio words (on SYS_CLK domain)
-- -DATA_RDY pulse when all 8 channels have been captured (on SYS_CLK domain)
--
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity tdm8_in is
  generic (width : integer := 24);

  port (
    -- TDM ports
    FSYNC   : in std_logic; -- Frame sync (rising edge = start of frame)
    BIT_CLK : in std_logic; -- Bit clock (256*fs)
    DIN     : in std_logic; -- Serial data input

    -- Control ports
    RESET   : in std_logic; -- Asynchronous Reset (Active Low)
    SYS_CLK : in std_logic; -- System Clock for CDC

    -- Parallel output ports
    data_out : out STD_LOGIC_VECTOR(width * 8 - 1 downto 0);

    -- Output status
    DATA_RDY : out std_logic -- Pulse when frame is complete
  );
end tdm8_in;

architecture rtl of tdm8_in is

  -- BIT_CLK domain signals
  signal shift_reg      : std_logic_vector(width - 1 downto 0);
  signal bit_count      : integer range 0 to 31;
  signal slot_count     : integer range 0 to 7;
  signal fsync_prev     : std_logic;
  signal frame_active   : std_logic;

  -- Channel data registers (BIT_CLK domain)
  type channel_array_t is array (0 to 7) of std_logic_vector(width - 1 downto 0);
  signal ch_data        : channel_array_t;
  signal data_rdy_int   : std_logic;

  -- CDC synchronizer signals
  signal data_rdy_sync1 : std_logic;
  signal data_rdy_sync2 : std_logic;
  signal data_rdy_prev  : std_logic;

  -- SYS_CLK domain latched data
  signal ch_data_sys    : channel_array_t;

begin

  -------------------------------------------------------------------------
  -- TDM Serial to Parallel (BIT_CLK Domain)
  -- FSYNC rising edge marks the start of slot 0.
  -- Data is sampled on rising edge of BIT_CLK, MSB first.
  -- Each slot is 32 BCLK wide; only first 'width' bits are captured.
  -------------------------------------------------------------------------
  tdm_in_proc : process (RESET, BIT_CLK)
  begin
    if (RESET = '0') then

      shift_reg    <= (others => '0');
      bit_count    <= 0;
      slot_count   <= 0;
      fsync_prev   <= '0';
      frame_active <= '0';
      data_rdy_int <= '0';
      for i in 0 to 7 loop
        ch_data(i) <= (others => '0');
      end loop;

    elsif rising_edge(BIT_CLK) then
      -- Default: clear ready pulse
      data_rdy_int <= '0';

      -- Detect FSYNC rising edge
      fsync_prev <= FSYNC;

      if (FSYNC = '1' and fsync_prev = '0') then
        -- Frame start: if previous frame was active, store last slot
        if (frame_active = '1') then
          ch_data(slot_count) <= shift_reg;
          data_rdy_int <= '1';
        end if;

        -- Start new frame: sample first bit (MSB of channel 0) on this edge
        frame_active <= '1';
        slot_count   <= 0;
        bit_count    <= 1;
        shift_reg    <= (others => '0');
        shift_reg(0) <= DIN;

      elsif (frame_active = '1') then
        -- Shift in data
        if (bit_count < width) then
          shift_reg(width - 1 downto 1) <= shift_reg(width - 2 downto 0);
          shift_reg(0)                  <= DIN;
        end if;

        if (bit_count = 31) then
          -- End of slot: store captured data and advance
          ch_data(slot_count) <= shift_reg;

          bit_count <= 0;
          shift_reg <= (others => '0');

          if (slot_count < 7) then
            slot_count <= slot_count + 1;
          end if;
          -- slot 7 completion is handled at next FSYNC edge
        else
          bit_count <= bit_count + 1;
        end if;
      end if;
    end if;
  end process tdm_in_proc;

  -------------------------------------------------------------------------
  -- 2-Flip-Flop Synchronizer: BIT_CLK -> SYS_CLK
  -------------------------------------------------------------------------
  sync_rdy : process (RESET, SYS_CLK)
  begin
    if (RESET = '0') then
      data_rdy_sync1 <= '0';
      data_rdy_sync2 <= '0';
    elsif rising_edge(SYS_CLK) then
      data_rdy_sync1 <= data_rdy_int;
      data_rdy_sync2 <= data_rdy_sync1;
    end if;
  end process sync_rdy;

  -------------------------------------------------------------------------
  -- Data Transfer to SYS_CLK Domain
  -------------------------------------------------------------------------
  data_transfer : process (RESET, SYS_CLK)
  begin
    if (RESET = '0') then
      for i in 0 to 7 loop
        ch_data_sys(i) <= (others => '0');
      end loop;
      data_rdy_prev <= '0';
      DATA_RDY      <= '0';

    elsif rising_edge(SYS_CLK) then
      data_rdy_prev <= data_rdy_sync2;

      if (data_rdy_sync2 = '1' and data_rdy_prev = '0') then
        -- Rising edge: latch all channels
        for i in 0 to 7 loop
          data_out((i+1) * width - 1 downto i*width) <= ch_data(i);
        end loop;
        DATA_RDY <= '1';
      elsif (data_rdy_sync2 = '0' and data_rdy_prev = '1') then
        DATA_RDY <= '0';
      end if;
    end if;
  end process data_transfer;

end rtl;
