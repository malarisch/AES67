---------------------------------------------------------------------------------
-- Engineer:      Klimann Wendelin 
--
-- Create Date:   07:25:11 11/Okt/2013
-- Design Name:   i2s_in
--
-- Description:   
-- 
-- This module provides a bridge between an I2S serial device (audio ADC, S/PDIF 
-- Decoded data) and a parallel device (microcontroller, IP block).
--
-- Modified: Hinzugefügt Clock Domain Crossing für System Clock
--
-- It's coded as a generic VHDL entity, so developer can choose the proper signal
-- width (8/16/24/32 bit)
--
-- Input takes:
-- -I2S Data
-- -I2S Bit Clock
-- -I2S LR Clock (Left/Right channel indication)
-- -SYS_CLK (System Clock for synchronization)
--
-- Output provides:
-- -DATA_L / DATA_R parallel inputs
-- -DATA_RDY_L / DATA_RDY_R output ready signals.
-- 
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;

entity i2s_in is
  -- width: How many bits (from MSB) are gathered from the serial I2S input
  generic (width : integer := 24);

  port (
    --  I2S ports
    LR_CLK  : in std_logic; --Left/Right indicator clock
    BIT_CLK : in std_logic; --Bit clock
    DIN     : in std_logic; --Data Input

    -- Control ports
    RESET   : in std_logic; --Asynchronous Reset (Active Low)
    SYS_CLK : in std_logic; --System Clock für CDC

    -- Parallel ports
    DATA_L : out std_logic_vector(width - 1 downto 0);
    DATA_R : out std_logic_vector(width - 1 downto 0);

    -- Output status ports
    DATA_RDY_L : out std_logic; --Falling edge means data is ready
    DATA_RDY_R : out std_logic --Falling edge means data is ready
  );
end i2s_in;
architecture rtl of i2s_in is

  -- I2S Domain Signale
  signal shift_reg       : std_logic_vector(width - 1 downto 0);
  signal s_parallel_load : std_logic;
  signal s_current_lr    : std_logic;

  -- Interne Datenregister (BIT_CLK Domain)
  signal data_l_int     : std_logic_vector(width - 1 downto 0);
  signal data_r_int     : std_logic_vector(width - 1 downto 0);
  signal data_rdy_l_int : std_logic;
  signal data_rdy_r_int : std_logic;

  -- Synchronizer Signale (2-Flip-Flop Chain)
  signal data_rdy_l_sync1 : std_logic;
  signal data_rdy_l_sync2 : std_logic;
  signal data_rdy_r_sync1 : std_logic;
  signal data_rdy_r_sync2 : std_logic;

  -- Latched Data im SYS_CLK Domain
  signal data_l_sys : std_logic_vector(width - 1 downto 0);
  signal data_r_sys : std_logic_vector(width - 1 downto 0);

  -- Edge Detection für Ready Signale
  signal data_rdy_l_prev : std_logic;
  signal data_rdy_r_prev : std_logic;

begin

  -------------------------------------------------------------------------
  -- I2S Serial to Parallel (BIT_CLK Domain - unverändert)
  -------------------------------------------------------------------------
  i2s_in : process (RESET, BIT_CLK, LR_CLK, DIN)
  begin
    if (RESET = '0') then

      data_l_int <= (others => '0');
      data_r_int <= (others => '0');
      shift_reg  <= (others => '0');

      s_current_lr    <= '0';
      s_parallel_load <= '0';
      data_rdy_l_int  <= '0';
      data_rdy_r_int  <= '0';

    elsif (BIT_CLK'event and BIT_CLK = '1') then
      -- Clear ready signals first (they should be pulses, not level signals)
      data_rdy_l_int <= '0';
      data_rdy_r_int <= '0';

      -- Load data from shift register BEFORE shifting to avoid race condition
      -- This ensures we capture the complete word before it gets corrupted by shifting
      if (s_parallel_load = '1') then
        -- Load the complete word that was just received
        if (s_current_lr = '0') then
          --Output Right Channel
          data_r_int     <= shift_reg;
          data_rdy_r_int <= '1'; -- Pulse ready signal when data is loaded
        else
          --Output Left Channel
          data_l_int     <= shift_reg;
          data_rdy_l_int <= '1'; -- Pulse ready signal when data is loaded
        end if;
        s_parallel_load <= '0';
      end if;

      -- Check for channel change BEFORE loading/shifting
      if (s_current_lr /= LR_CLK) then
        -- Channel changed - load the PREVIOUS complete word (using old s_current_lr)
        if (s_parallel_load = '1') then
          -- Load the complete word that was just received for the previous channel
          if (s_current_lr = '0') then
            --Output Right Channel (previous word)
            data_r_int     <= shift_reg;
            data_rdy_r_int <= '1'; -- Pulse ready signal when data is loaded
          else
            --Output Left Channel (previous word)
            data_l_int     <= shift_reg;
            data_rdy_l_int <= '1'; -- Pulse ready signal when data is loaded
          end if;
        end if;

        -- Now shift and start new word
        shift_reg(width - 1 downto 1) <= shift_reg(width - 2 downto 0);
        shift_reg(0)                  <= DIN;
        s_current_lr                  <= LR_CLK;

        -- Setup for parallel register load when new word is complete (after 24 bits)
        s_parallel_load <= '1';
      else
        shift_reg(width - 1 downto 1) <= shift_reg(width - 2 downto 0);
        shift_reg(0)                  <= DIN;
        -- Load data if parallel_load is set (this should not happen during normal operation,
        -- but handle it for safety)
        if (s_parallel_load = '1') then
          if (s_current_lr = '0') then
            --Output Right Channel
            data_r_int     <= shift_reg;
            data_rdy_r_int <= '1';
          else
            --Output Left Channel
            data_l_int     <= shift_reg;
            data_rdy_l_int <= '1';
          end if;
          s_parallel_load <= '0';
        end if;
      end if;
    end if;
  end process i2s_in;

  -------------------------------------------------------------------------
  -- 2-Flip-Flop Synchronizer: BIT_CLK -> SYS_CLK
  -- Reduziert Metastabilität und handhabt Clock Jitter
  -------------------------------------------------------------------------
  sync_left : process (RESET, SYS_CLK)
  begin
    if (RESET = '0') then
      data_rdy_l_sync1 <= '0';
      data_rdy_l_sync2 <= '0';
    elsif (rising_edge(SYS_CLK)) then
      data_rdy_l_sync1 <= data_rdy_l_int; -- 1. FF
      data_rdy_l_sync2 <= data_rdy_l_sync1; -- 2. FF
    end if;
  end process sync_left;

  sync_right : process (RESET, SYS_CLK)
  begin
    if (RESET = '0') then
      data_rdy_r_sync1 <= '0';
      data_rdy_r_sync2 <= '0';
    elsif (rising_edge(SYS_CLK)) then
      data_rdy_r_sync1 <= data_rdy_r_int; -- 1. FF
      data_rdy_r_sync2 <= data_rdy_r_sync1; -- 2. FF
    end if;
  end process sync_right;

  -------------------------------------------------------------------------
  -- Data Transfer auf SYS_CLK Domain
  -- Daten werden gelatcht wenn Ready-Signal erkannt wird
  -------------------------------------------------------------------------
  data_transfer : process (RESET, SYS_CLK)
  begin
    if (RESET = '0') then
      data_l_sys      <= (others => '0');
      data_r_sys      <= (others => '0');
      data_rdy_l_prev <= '0';
      data_rdy_r_prev <= '0';
      DATA_RDY_L      <= '0';
      DATA_RDY_R      <= '0';

    elsif (rising_edge(SYS_CLK)) then
      -- Edge Detection für Left Channel
      data_rdy_l_prev <= data_rdy_l_sync2;
      if (data_rdy_l_sync2 = '1' and data_rdy_l_prev = '0') then
        -- Rising edge detected - Latch data
        data_l_sys <= data_l_int;
        DATA_RDY_L <= '1';
      elsif (data_rdy_l_sync2 = '0' and data_rdy_l_prev = '1') then
        -- Falling edge
        DATA_RDY_L <= '0';
      end if;

      -- Edge Detection für Right Channel
      data_rdy_r_prev <= data_rdy_r_sync2;
      if (data_rdy_r_sync2 = '1' and data_rdy_r_prev = '0') then
        -- Rising edge detected - Latch data
        data_r_sys <= data_r_int;
        DATA_RDY_R <= '1';
      elsif (data_rdy_r_sync2 = '0' and data_rdy_r_prev = '1') then
        -- Falling edge
        DATA_RDY_R <= '0';
      end if;
    end if;
  end process data_transfer;

  -- Output Assignment
  DATA_L <= data_l_sys;
  DATA_R <= data_r_sys;

end rtl;