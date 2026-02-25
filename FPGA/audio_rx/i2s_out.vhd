---------------------------------------------------------------------------------
-- Engineer:      Based on i2s_in by Klimann Wendelin
--
-- Create Date:   2025
-- Design Name:   i2s_out
--
-- Description:
--
-- This module provides a bridge between a parallel audio source (e.g. RX
-- ringbuffer) and an I2S serial device (audio DAC).
--
-- It's coded as a generic VHDL entity, so developer can choose the proper signal
-- width (8/16/24/32 bit)
--
-- Input takes:
-- -DATA_L / DATA_R parallel audio words
-- -I2S Bit Clock
-- -I2S LR Clock (directly generated or directly derived from BIT_CLK)
-- -SYS_CLK (System Clock for synchronization)
--
-- Output provides:
-- -DOUT I2S serial data
-- -DATA_REQ_L / DATA_REQ_R request pulses for next sample (on SYS_CLK domain)
--
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;

entity i2s_out is
  generic (width : integer := 24);

  port (
    -- I2S ports
    LR_CLK  : in  std_logic;
    BIT_CLK : in  std_logic;
    DOUT    : out std_logic;

    -- Control ports
    RESET   : in std_logic; -- Asynchronous Reset (Active Low)
    SYS_CLK : in std_logic; -- System Clock for CDC

    -- Parallel ports
    DATA_L : in std_logic_vector(width - 1 downto 0);
    DATA_R : in std_logic_vector(width - 1 downto 0)
  );
end i2s_out;

architecture rtl of i2s_out is

  -- I2S Domain signals
  signal shift_reg    : std_logic_vector(width - 1 downto 0);
  signal s_current_lr : std_logic;

  -- Bit counter: only shift out first 'width' bits (left-justified at 64*fs)
  signal bit_count    : integer range 0 to 63;

  -- Latched parallel data (BIT_CLK domain)
  signal data_l_lat   : std_logic_vector(width - 1 downto 0);
  signal data_r_lat   : std_logic_vector(width - 1 downto 0);

begin

  -------------------------------------------------------------------------
  -- I2S Parallel to Serial (BIT_CLK Domain)
  -- Standard I2S: MSB is delayed by 1 BCLK after LRCLK edge
  -- Compatible with AD1941 DSP at BCLK = 64*fs (32 clocks per channel)
  -- Only shifts out first 'width' bits, pads with 0 after that
  -------------------------------------------------------------------------
  i2s_out_proc : process (RESET, BIT_CLK)
  begin
    if (RESET = '0') then

      shift_reg   <= (others => '0');
      s_current_lr <= '0';
      bit_count    <= 0;
      data_l_lat   <= (others => '0');
      data_r_lat   <= (others => '0');
      DOUT         <= '0';

    elsif falling_edge(BIT_CLK) then

      -- Check for channel change (LRCLK edge)
      if (s_current_lr /= LR_CLK) then
        -- Channel changed - load new data for the upcoming channel
        s_current_lr <= LR_CLK;
        bit_count    <= 0;

        if (LR_CLK = '0') then
          -- Starting Left channel
          shift_reg <= DATA_L;
        else
          -- Starting Right channel
          shift_reg <= DATA_R;
        end if;

        -- I2S standard: output '0' during the 1 BCLK delay after LRCLK edge
        DOUT <= '0';

      else
        -- Same channel - shift out data bits
        if (bit_count < width) then
          -- Output MSB first
          DOUT <= shift_reg(width - 1);
          shift_reg(width - 1 downto 1) <= shift_reg(width - 2 downto 0);
          shift_reg(0) <= '0';
        else
          -- Padding bits after data
          DOUT <= '0';
        end if;

        -- Always increment counter (even during padding)
        if (bit_count < 63) then
          bit_count <= bit_count + 1;
        end if;
      end if;
    end if;
  end process i2s_out_proc;

end rtl;
