library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity audio_clock_controller is
  port (
    clk_sys_i   : in  std_ulogic; -- 250 MHz System Clock
    rst_i       : in  std_ulogic;
    
    -- PTP Time Input (from ptp_timer)
    ptp_time_i  : in  unsigned(63 downto 0); -- 64-bit ns
    
    -- Audio Clock Feedback (from PLL output)
    mclk_i      : in  std_ulogic; -- 12.288 MHz
    
    -- PLL Dynamic Phase Control Interface
    pll_scanclk_o       : out std_ulogic;
    pll_phaseupdown_o   : out std_ulogic;
    pll_phasestep_o     : out std_ulogic;
    pll_cnt_sel_o       : out std_ulogic_vector(2 downto 0);
    pll_phasedone_i     : in  std_ulogic
  );
end entity audio_clock_controller;

architecture rtl of audio_clock_controller is

  -- Target Period for 12.288 MHz (48kHz * 256)
  -- 1 / 12.288 MHz = 81.380208333 ns
  -- Format: 32.32 fixed point (ns) for high precision accumulator
  constant TARGET_PERIOD : unsigned(63 downto 0) := x"00000051_614D5555"; 
  
  -- Accumulator for Ideal Time
  signal ideal_time_accum : unsigned(63 downto 0) := (others => '0');
  signal next_ideal_edge  : unsigned(63 downto 0) := (others => '0');
  
  -- MCLK Edge Detection
  signal mclk_meta : std_ulogic;
  signal mclk_sync : std_ulogic;
  signal mclk_prev : std_ulogic;
  signal mclk_rising : std_ulogic;
  
  -- Phase Error Calculation
  signal time_diff : signed(63 downto 0);
  
  -- State Machine
  type t_state is (IDLE, PULSE_STEP, WAIT_DONE, COOLDOWN);
  signal state : t_state := IDLE;
  signal timer : unsigned(15 downto 0) := (others => '0');
  
  -- Scan Clock Divider
  signal scanclk_div : unsigned(2 downto 0) := (others => '0'); -- Divide by 8 (31.25 MHz)
  signal scanclk_int : std_ulogic;

begin

  -- Generate Scan Clock (250 MHz / 8 = 31.25 MHz)
  process(clk_sys_i)
  begin
    if rising_edge(clk_sys_i) then
        scanclk_div <= scanclk_div + 1;
    end if;
  end process;
  
  scanclk_int   <= scanclk_div(2);
  pll_scanclk_o <= scanclk_int;
  
  pll_cnt_sel_o <= "000"; -- Select Counter C0 (MCLK)

  process(clk_sys_i, rst_i)
  begin
    if rst_i = '1' then
      ideal_time_accum <= (others => '0');
      next_ideal_edge  <= TARGET_PERIOD;
      mclk_meta <= '0'; mclk_sync <= '0'; mclk_prev <= '0';
      state <= IDLE;
      pll_phasestep_o <= '0';
      pll_phaseupdown_o <= '1';
    elsif rising_edge(clk_sys_i) then
      
      -- 1. Synchronize MCLK
      mclk_meta <= mclk_i;
      mclk_sync <= mclk_meta;
      mclk_prev <= mclk_sync;
      mclk_rising <= mclk_sync and not mclk_prev;
      
      -- 2. Update Ideal Time Target
      -- We don't just accumulate every cycle; we want to know "When should the NEXT edge be?"
      -- But simpler: Maintain a "Virtual Clock" that runs at exactly 12.288 MHz relative to PTP time.
      -- Actually, we can just compare the PTP time at the moment of MCLK rising edge 
      -- with the "Expected" PTP time.
      
      if mclk_rising = '1' then
        -- Calculate Error: Expected - Actual
        -- Expected comes from our internal accumulator which adds exactly 81.38ns per edge
        ideal_time_accum <= ideal_time_accum + TARGET_PERIOD;
        
        -- Current PTP Time (convert to 32.32 for comparison)
        -- ptp_time_i is 64-bit integer ns? No, usually 48.16 or similar.
        -- Let's assume ptp_time_i is pure nanoseconds (integer) for now based on previous context,
        -- but ptp_timer.vhd showed 8.24 format for increment, but output is 64-bit.
        -- Let's assume ptp_time_i is 64-bit integer ns for simplicity or 32.32.
        -- Checking ptp_timer.vhd: "counter_ns <= counter_ns + resize(inc_int, 64)..."
        -- It seems to be 64-bit integer nanoseconds (plus fractional part in accum).
        -- Wait, ptp_timer.vhd output `current_time_o` is just `counter_ns`.
        -- If `inc_int` is added, it's integer ns.
        
        -- Error = Ideal - Actual
        -- We need to align them. Let's just look at the lower bits to align phase.
        -- 12.288 MHz period is ~81ns.
        -- We want (PTP_Time % 81.38) to be constant (or 0).
        
        -- Simplified Phase Detector:
        -- Just check if we are drifting.
        -- But we need to lock to PTP.
        
        -- Let's use a simpler approach:
        -- We want the MCLK edge to happen exactly when PTP Time is a multiple of 81.38ns.
        -- Error = (PTP_Time - Start_Time) % Period
        -- If Error > Period/2 -> We are late -> Speed up
        -- If Error < Period/2 -> We are early -> Slow down
      end if;
      
      -- 3. Control Loop (Bang-Bang or Proportional)
      case state is
        when IDLE =>
          pll_phasestep_o <= '0';
          
          if mclk_rising = '1' then
             -- Check Phase Error
             -- For this example, let's assume we just want to match frequency first.
             -- But we need phase lock.
             
             -- Let's assume we want to correct if error > 10ns.
             -- Implementation detail: This requires careful math with the 64-bit PTP time.
             -- Placeholder logic:
             -- if (too_slow) then
             --   pll_phaseupdown_o <= '1'; -- Advance Phase (shorten period)
             --   pll_phasestep_o <= '1';
             --   state <= WAIT_DONE;
             -- elsif (too_fast) then
             --   pll_phaseupdown_o <= '0'; -- Retard Phase (lengthen period)
             --   pll_phasestep_o <= '1';
             --   state <= PULSE_STEP;
             -- end if;
          end if;
          
        when PULSE_STEP =>
          -- Hold phasestep high for at least 2 scanclk cycles
          -- scanclk period = 8 sysclk cycles. 2 scanclk = 16 sysclk.
          -- Let's hold for 32 cycles to be safe.
          pll_phasestep_o <= '1';
          if timer = 32 then
            pll_phasestep_o <= '0';
            state <= WAIT_DONE;
            timer <= (others => '0');
          else
            timer <= timer + 1;
          end if;
          
        when WAIT_DONE =>
          pll_phasestep_o <= '0';
          if pll_phasedone_i = '1' then
            state <= COOLDOWN;
            timer <= x"FFFF"; -- Wait a bit before next adjustment
          end if;
          
        when COOLDOWN =>
          if timer = 0 then
            state <= IDLE;
          else
            timer <= timer - 1;
          end if;
          
      end case;
      
    end if;
  end process;

end architecture rtl;
