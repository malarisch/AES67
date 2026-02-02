-- ============================================================
-- PTPv2 Servo - Clock Discipline Algorithm
-- ============================================================
-- Implements a PI (Proportional-Integral) controller to discipline
-- the local clock to the PTP master.
--
-- Simplified approach:
-- 1. First few samples: Just observe and filter, don't correct
-- 2. After warmup: Apply frequency correction based on filtered offset
-- 3. No phase jumps from servo - parser handles initial clock set
--
-- TUNING GUIDE:
-- Effective Kp = KP_GAIN / 2^GAIN_SHIFT
-- Effective Ki = KI_GAIN / 2^(GAIN_SHIFT + 2)
-- For stability with 1 Hz Sync rate, use Kp ~ 0.1-0.5, Ki ~ 0.001-0.01
-- Example: KP_GAIN=8, KI_GAIN=1, GAIN_SHIFT=6 gives Kp=0.125, Ki=0.004
-- ============================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity ptpv2_servo is
    generic(
        -- PI controller gains
        -- Effective Kp = KP_GAIN / 2^GAIN_SHIFT
        -- Effective Ki = KI_GAIN / 2^(GAIN_SHIFT + 2)
        -- WARNING: Values too high cause oscillation!
        KP_GAIN : integer := 8;    -- Proportional gain numerator
        KI_GAIN : integer := 2;    -- Integral gain numerator  
        GAIN_SHIFT : integer := 7; -- Divide gains by 128 (Kp=0.0625, Ki=0.002)
        
        -- Filter coefficient for offset (exponential moving average)
        -- alpha = 1/2^FILTER_SHIFT. Higher = more smoothing, slower response
        FILTER_SHIFT : integer := 2;  -- alpha = 1/8 (more smoothing)
        
        -- Warmup: ignore first N samples to let filter settle
        WARMUP_SAMPLES : integer := 8;
        
        -- Lock thresholds
        LOCK_THRESHOLD_NS   : integer := 1000;   -- Consider locked if offset < 1µs
        UNLOCK_THRESHOLD_NS : integer := 10000;  -- Unlock if offset > 10µs
        LOCK_COUNT_THRESHOLD : integer := 8      -- Need 8 consecutive good measurements
    );
    port(
        clk                 : in  std_logic;
        reset_n             : in  std_logic;
        
        -- Input from ptpv2_parser
        offset_from_master_i : in  signed(63 downto 0);  -- Nanoseconds
        mean_path_delay_i    : in  signed(63 downto 0);  -- Nanoseconds
        calc_valid_i         : in  std_logic;
        
        -- Outputs to wallclock
        freq_correction_o    : out signed(31 downto 0);  -- PPB correction (parts per billion)
        phase_jump_o         : out signed(31 downto 0);  -- One-time ns adjustment (not used currently)
        phase_jump_valid_o   : out std_logic;            -- Pulse to apply phase jump
        
        -- Status
        locked_o             : out std_logic;
        
        -- Debug outputs
        filtered_offset_o    : out signed(63 downto 0);
        integral_o           : out signed(63 downto 0)
    );
end entity;

architecture Behavioral of ptpv2_servo is

    -- Filtered offset (exponential moving average)
    signal filtered_offset : signed(63 downto 0) := (others => '0');
    
    -- PI controller state
    signal integral_sum    : signed(63 downto 0) := (others => '0');
    signal freq_correction : signed(31 downto 0) := (others => '0');
    
    -- Lock detection
    signal lock_counter    : integer range 0 to 255 := 0;
    signal locked          : std_logic := '0';
    
    -- Sample counter for warmup
    signal sample_count    : integer range 0 to 65535 := 0;
    
    -- Sanity check: reject obviously invalid measurements (> 1s)
    constant MAX_VALID_OFFSET : signed(63 downto 0) := to_signed(1_000_000_000, 64);  -- 1s
    
    -- Phase jump threshold: if offset > 10ms, do phase jump instead of frequency correction
    constant PHASE_JUMP_THRESHOLD : signed(63 downto 0) := to_signed(10_000_000, 64);  -- 10ms
    
    -- Phase jump output registers
    signal phase_jump_reg       : signed(31 downto 0) := (others => '0');
    signal phase_jump_valid_reg : std_logic := '0';
    
begin

    -- Output assignments
    freq_correction_o <= freq_correction;
    locked_o <= locked;
    filtered_offset_o <= filtered_offset;
    integral_o <= integral_sum;
    phase_jump_o <= phase_jump_reg;
    phase_jump_valid_o <= phase_jump_valid_reg;
    
    servo_proc: process(clk, reset_n)
        variable offset_sample   : signed(63 downto 0);
        variable offset_abs      : signed(63 downto 0);
        variable proportional    : signed(63 downto 0);
        variable pi_output       : signed(63 downto 0);
        variable filter_delta    : signed(63 downto 0);
        variable scaled_offset   : signed(31 downto 0);  -- Scaled down offset for multiplication
        variable mult_result     : signed(63 downto 0);  -- Result of 32x32 multiplication
    begin
        if reset_n = '0' then
            filtered_offset <= (others => '0');
            integral_sum    <= (others => '0');
            freq_correction <= (others => '0');
            lock_counter    <= 0;
            locked          <= '0';
            sample_count    <= 0;
            phase_jump_reg       <= (others => '0');
            phase_jump_valid_reg <= '0';
            
        elsif rising_edge(clk) then
            -- Default: no phase jump this cycle
            phase_jump_valid_reg <= '0';
            
            if calc_valid_i = '1' then
                offset_sample := offset_from_master_i;
                
                -- Calculate absolute offset for threshold comparisons
                if offset_sample < 0 then
                    offset_abs := -offset_sample;
                else
                    offset_abs := offset_sample;
                end if;
                
                -- ============================================
                -- SANITY CHECK: Reject invalid measurements
                -- ============================================
                if offset_abs < MAX_VALID_OFFSET then
                    
                    -- Count valid samples
                    if sample_count < 65535 then
                        sample_count <= sample_count + 1;
                    end if;
                    
                    -- ============================================
                    -- PHASE JUMP: If offset is too large for frequency correction
                    -- ============================================
                    if offset_abs > PHASE_JUMP_THRESHOLD then
                        -- Large offset: apply phase jump to quickly correct
                        -- Limit phase jump to ±2^30 ns (~1 second) per jump
                        if offset_sample > to_signed(2**30 - 1, 64) then
                            phase_jump_reg <= to_signed(-(2**30 - 1), 32);  -- Negative to slow down
                        elsif offset_sample < to_signed(-(2**30 - 1), 64) then
                            phase_jump_reg <= to_signed(2**30 - 1, 32);     -- Positive to speed up
                        else
                            phase_jump_reg <= -offset_sample(31 downto 0);   -- Exact correction (negated)
                        end if;
                        phase_jump_valid_reg <= '1';
                        
                        -- Reset filter and integrator after phase jump
                        filtered_offset <= (others => '0');
                        integral_sum <= (others => '0');
                        freq_correction <= (others => '0');
                        sample_count <= 0;  -- Restart warmup
                        locked <= '0';
                        lock_counter <= 0;
                        
                    else
                        -- ============================================
                        -- Normal operation: frequency correction
                        -- ============================================
                        
                        -- ============================================
                        -- Low-pass filter the offset
                        -- First sample: initialize filter directly
                        -- Subsequent: exponential moving average
                        -- ============================================
                        if sample_count = 0 then
                            -- First sample: initialize filter
                            filtered_offset <= offset_sample;
                        else
                            -- EMA: filtered = filtered + (sample - filtered) >> FILTER_SHIFT
                            filter_delta := shift_right(offset_sample - filtered_offset, FILTER_SHIFT);
                            filtered_offset <= filtered_offset + filter_delta;
                        end if;
                        
                        -- ============================================
                        -- PI Controller (only after warmup)
                        -- ============================================
                        if sample_count >= WARMUP_SAMPLES then
                            
                            -- Proportional term: Kp * filtered_offset / 2^GAIN_SHIFT
                            -- Scale down offset to 32 bits first (prevents overflow)
                            -- Clamp to 32-bit range to prevent wrap-around
                            if filtered_offset > to_signed(2**30 - 1, 64) then
                                scaled_offset := to_signed(2**30 - 1, 32);
                            elsif filtered_offset < to_signed(-(2**30), 64) then
                                scaled_offset := to_signed(-(2**30), 32);
                            else
                                scaled_offset := filtered_offset(31 downto 0);
                            end if;
                            
                            -- Proportional: Kp * scaled_offset / 2^GAIN_SHIFT
                            -- Note: 32-bit * 32-bit = 64-bit result
                            -- SIGN INVERSION: positive offset means local clock is AHEAD,
                            -- so we need NEGATIVE frequency correction to slow down.
                            -- Formula: freq_correction = -Kp * offset
                            mult_result := scaled_offset * to_signed(KP_GAIN, 32);
                            proportional := -shift_right(mult_result, GAIN_SHIFT);
                            
                            -- Integral term: Accumulate filtered offset (also inverted)
                            -- With anti-windup: limit to ±1 billion PPB (±1000 PPM)
                            if integral_sum > to_signed(-1_000_000_000, 64) and 
                               integral_sum < to_signed(1_000_000_000, 64) then
                                mult_result := scaled_offset * to_signed(KI_GAIN, 32);
                                integral_sum <= integral_sum - shift_right(mult_result, GAIN_SHIFT + 2);
                            end if;
                            
                            -- Combined PI output
                            pi_output := proportional + integral_sum;
                            
                            -- Limit frequency correction to ±500 PPM = ±500000 PPB
                            if pi_output > to_signed(500_000, 64) then
                                freq_correction <= to_signed(500_000, 32);
                            elsif pi_output < to_signed(-500_000, 64) then
                                freq_correction <= to_signed(-500_000, 32);
                            else
                                freq_correction <= pi_output(31 downto 0);
                            end if;
                            
                            -- ============================================
                            -- Lock detection
                            -- ============================================
                            if offset_abs < to_signed(LOCK_THRESHOLD_NS, 64) then
                                if lock_counter < LOCK_COUNT_THRESHOLD then
                                    lock_counter <= lock_counter + 1;
                                else
                                    locked <= '1';
                                end if;
                            else
                                if offset_abs > to_signed(UNLOCK_THRESHOLD_NS, 64) then
                                    locked <= '0';
                                end if;
                                lock_counter <= 0;
                            end if;
                            
                        end if;  -- warmup complete
                        
                    end if;  -- phase jump vs normal operation
                    
                end if;  -- sanity check passed
                
            end if;  -- calc_valid
        end if;  -- rising_edge
    end process;

end Behavioral;