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
-- MESSAGE INTERVAL AWARENESS:
-- The PI controller gains are scaled based on the sync message interval.
-- Faster sync rates (negative logMessageInterval) require reduced gains
-- to maintain stability. The gains are normalized to 1 Hz (interval=0).
--
-- TUNING GUIDE:
-- Effective Kp = KP_GAIN / 2^GAIN_SHIFT / sync_rate_multiplier
-- Effective Ki = KI_GAIN / 2^(GAIN_SHIFT + 2) / sync_rate_multiplier
-- For stability with 1 Hz Sync rate, use Kp ~ 0.1-0.5, Ki ~ 0.001-0.01
-- Example: KP_GAIN=8, KI_GAIN=1, GAIN_SHIFT=6 gives Kp=0.125, Ki=0.004
-- ============================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity ptpv2_servo is
    generic(
        -- PI controller gains (normalized to 1 Hz sync rate)
        -- Actual gains are scaled by 2^log_msg_interval for faster rates

        -- Effective Kp = KP_GAIN / 2^GAIN_SHIFT / 2^(-log_msg_interval)
        -- Effective Ki = KI_GAIN / 2^(GAIN_SHIFT + 2) / 2^(-log_msg_interval)
        -- CRITICAL: At 1 Hz sample rate, Kp must be < 0.5 for stability!
        -- freq_correction acts for full second, so Kp=1 means 100% correction = oscillation
        KP_GAIN : integer := 13;   -- Proportional gain numerator 
        KI_GAIN : integer := 5;    -- Integral gain numerator
        GAIN_SHIFT : integer := 6; -- Divide gains (base shift for 1 Hz)
        
        -- Filter coefficient for offset (exponential moving average)
        -- No filter needed if Kp is properly tuned
        FILTER_SHIFT : integer := 0;  -- alpha
        
        -- Warmup: ignore first N samples to let filter settle
        WARMUP_SAMPLES : integer := 1;  -- Quick start
        
        -- Lock thresholds
        LOCK_THRESHOLD_NS   : integer := 500;    -- Consider locked if offset < 500ns
        UNLOCK_THRESHOLD_NS : integer := 5000;   -- Unlock if offset > 5µs
        LOCK_COUNT_THRESHOLD : integer := 2;     -- Need only 2 consecutive good measurements
        
        -- Sync timeout multiplier (timeout = 3 * sync_interval)
        -- At 125 MHz clock: 125_000_000 cycles/sec
        CLOCK_FREQ_HZ : integer := 125_000_000
    );
    port(
        clk                 : in  std_logic;
        reset_n             : in  std_logic;
        
        -- Input from ptpv2_parser
        offset_from_master_i : in  signed(31 downto 0);  -- Nanoseconds
        mean_path_delay_i    : in  signed(31 downto 0);  -- Nanoseconds
        calc_valid_i         : in  std_logic;
        log_msg_interval_i   : in  signed(7 downto 0);   -- PTP logMessageInterval (signed!)
        log_msg_interval_valid_i : in std_logic;         -- Pulse when interval is updated
        
        -- Outputs to wallclock
        freq_correction_o    : out signed(31 downto 0);  -- PPB correction (parts per billion)
        phase_jump_o         : out signed(31 downto 0);  -- One-time ns adjustment (not used currently)
        phase_jump_valid_o   : out std_logic;            -- Pulse to apply phase jump
        
        -- Status
        locked_o             : out std_logic;
        sync_timeout_o       : out std_logic;  -- Pulses when no sync received for too long
        
        -- Debug outputs
        filtered_offset_o    : out signed(31 downto 0);
        integral_o           : out signed(31 downto 0);
        effective_gain_shift_o : out integer range 0 to 15;  -- Debug: actual gain shift used
        median_offset_o      : out signed(31 downto 0);      -- Debug: median-filtered offset
        median_delay_o       : out signed(31 downto 0);      -- Debug: median-filtered path delay
        median_valid_o       : out std_logic                 -- Debug: median filter has enough samples
    );
end entity;

architecture Behavioral of ptpv2_servo is

    -- Filtered offset (exponential moving average)
    signal filtered_offset : signed(31 downto 0) := (others => '0');
    
    -- PI controller state
    signal integral_sum    : signed(31 downto 0) := (others => '0');
    signal freq_correction : signed(31 downto 0) := (others => '0');
    
    -- Lock detection
    signal lock_counter    : integer range 0 to 255 := 0;
    signal locked          : std_logic := '0';
    
    -- Sample counter for warmup
    signal sample_count    : integer range 0 to 65535 := 0;
    
    -- Sync timeout counter
    signal timeout_counter : unsigned(31 downto 0) := (others => '0');
    signal timeout_limit   : unsigned(31 downto 0) := to_unsigned(CLOCK_FREQ_HZ * 4, 32);  -- Default 4s for 1 Hz
    signal sync_timeout    : std_logic := '0';
    
    -- Message interval tracking
    signal current_log_interval : signed(7 downto 0) := (others => '0');
    signal effective_gain_shift : integer range 0 to 15 := GAIN_SHIFT;
    
    -- ============================================
    -- Median filter for outlier rejection (5 samples)
    -- ============================================
    constant MEDIAN_SIZE : integer := 5;
    type sample_buffer_t is array(0 to MEDIAN_SIZE-1) of signed(31 downto 0);
    
    -- Sample buffers (circular)
    signal offset_buffer      : sample_buffer_t := (others => (others => '0'));
    signal delay_buffer       : sample_buffer_t := (others => (others => '0'));
    signal buffer_write_idx   : integer range 0 to MEDIAN_SIZE-1 := 0;
    signal buffer_fill_count  : integer range 0 to MEDIAN_SIZE := 0;
    
    -- Median-filtered values
    signal median_offset      : signed(31 downto 0) := (others => '0');
    signal median_delay       : signed(31 downto 0) := (others => '0');
    signal median_valid       : std_logic := '0';
    
    -- Sanity check: reject obviously invalid measurements (> 1s)
    constant MAX_VALID_OFFSET : signed(31 downto 0) := to_signed(1_000_000_000, 32);  -- 1s
    
    -- Phase jump threshold: if offset > this, do phase jump instead of frequency correction
    -- Set low enough to quickly correct initial offset from parser (~100µs)
    constant PHASE_JUMP_THRESHOLD : signed(31 downto 0) := to_signed(100_000, 32);  -- 10µs (was 10ms!)
    
    -- Phase jump output registers
    signal phase_jump_reg       : signed(31 downto 0) := (others => '0');
    signal phase_jump_valid_reg : std_logic := '0';
    
    -- ============================================
    -- Function: Find median of 5 signed values
    -- Uses a sorting network approach (simple for 5 elements)
    -- ============================================
    function median5(buf : sample_buffer_t) return signed is
        variable a, b, c, d, e : signed(31 downto 0);
        variable t : signed(31 downto 0);
    begin
        -- Load values
        a := buf(0); b := buf(1); c := buf(2); d := buf(3); e := buf(4);
        
        -- Sorting network for 5 elements to find median
        -- Step 1: Compare and swap pairs
        if a > b then t := a; a := b; b := t; end if;
        if c > d then t := c; c := d; d := t; end if;
        
        -- Step 2: Move smaller of (a,c) and larger of (b,d)
        if a > c then t := a; a := c; c := t; end if;
        if b > d then t := b; b := d; d := t; end if;
        
        -- Step 3: Insert e and find median
        -- Median is max(min(b,c), min(max(a,e), max(c,d)))
        if b > c then t := b; b := c; c := t; end if;
        if a > e then t := a; a := e; e := t; end if;
        
        -- Now a <= b <= c, a <= e, c <= d
        -- Median candidates: b, c, or e
        if e > c then
            return c;
        elsif e < b then
            return b;
        else
            return e;
        end if;
    end function;
    
begin

    -- Output assignments
    freq_correction_o <= freq_correction;
    locked_o <= locked;
    filtered_offset_o <= filtered_offset;
    integral_o <= integral_sum;
    phase_jump_o <= phase_jump_reg;
    phase_jump_valid_o <= phase_jump_valid_reg;
    sync_timeout_o <= sync_timeout;
    effective_gain_shift_o <= effective_gain_shift;
    median_offset_o <= median_offset;
    median_delay_o  <= median_delay;
    median_valid_o  <= median_valid;
    
    -- ============================================================
    -- Calculate effective gain shift based on message interval
    -- For faster sync rates (negative interval), increase shift to reduce gain
    -- For slower sync rates (positive interval), could decrease shift, but cap at GAIN_SHIFT
    -- 
    -- interval  | sync rate | gain multiplier | extra shift
    --    0      |  1 Hz     |   1x            |   0
    --   -1      |  2 Hz     |   0.5x          |   1
    --   -2      |  4 Hz     |   0.25x         |   2
    --   -3      |  8 Hz     |   0.125x        |   3
    --    1      |  0.5 Hz   |   2x (capped)   |  -1 (but cap at 0)
    -- ============================================================

    
    servo_proc: process(clk, reset_n)
        variable offset_sample   : signed(31 downto 0);
        variable offset_abs      : signed(31 downto 0);
        variable proportional    : signed(31 downto 0);
        variable pi_output       : signed(31 downto 0);
        variable filter_delta    : signed(31 downto 0);
        variable scaled_offset   : signed(31 downto 0);  -- Scaled down offset for multiplication
        variable mult_result     : signed(63 downto 0);  -- Result of 32x32 multiplication
        variable timeout_cycles  : unsigned(31 downto 0);  -- Calculated timeout
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
            timeout_counter <= (others => '0');
            timeout_limit   <= to_unsigned(CLOCK_FREQ_HZ * 4, 32);  -- Default 4s
            sync_timeout    <= '0';
            current_log_interval <= (others => '0');
            -- Median filter reset
            offset_buffer <= (others => (others => '0'));
            delay_buffer  <= (others => (others => '0'));
            buffer_write_idx  <= 0;
            buffer_fill_count <= 0;
            median_offset <= (others => '0');
            median_delay  <= (others => '0');
            median_valid  <= '0';
            
        elsif rising_edge(clk) then
            -- Default: no phase jump this cycle, clear timeout pulse
            phase_jump_valid_reg <= '0';
            sync_timeout <= '0';
            
            -- ============================================
            -- UPDATE MESSAGE INTERVAL (separate from calc_valid!)
            -- This allows timeout adjustment even when measurements are rejected
            -- ============================================
            if log_msg_interval_valid_i = '1' then
                current_log_interval <= log_msg_interval_i;
                
                -- Calculate timeout limit based on message interval
                -- timeout = 4 * 2^log_interval seconds (in clock cycles)
                -- For log_interval = -2: 4 * 0.25s = 1s
                -- For log_interval = 0:  4 * 1s = 4s
                -- For log_interval = 1:  4 * 2s = 8s
                if log_msg_interval_i >= 0 then
                    -- Slow sync (1 Hz or slower): shift left
                    if log_msg_interval_i > 3 then
                        -- Cap at 32s (4 * 8s) to prevent overflow
                        timeout_limit <= to_unsigned(CLOCK_FREQ_HZ * 32, 32);
                    else
                        timeout_limit <= shift_left(to_unsigned(CLOCK_FREQ_HZ * 4, 32), to_integer(log_msg_interval_i));
                    end if;
                else
                    -- Fast sync (>1 Hz): shift right
                    if log_msg_interval_i < -4 then
                        -- Cap at minimum ~250ms (4 * 1/16s)
                        timeout_limit <= to_unsigned(CLOCK_FREQ_HZ / 4, 32);
                    else
                        timeout_limit <= shift_right(to_unsigned(CLOCK_FREQ_HZ * 4, 32), to_integer(-log_msg_interval_i));
                    end if;
                end if;
            end if;
            
            -- ============================================
            -- SYNC TIMEOUT DETECTION
            -- Reset counter on valid calc, otherwise count up
            -- ============================================
            if calc_valid_i = '1' then
                timeout_counter <= (others => '0');
            else
                -- No valid calc this cycle - increment timeout counter
                if timeout_counter < timeout_limit then
                    timeout_counter <= timeout_counter + 1;
                else
                    -- TIMEOUT! Signal it and reset servo state
                    sync_timeout <= '1';
                    locked <= '0';
                    lock_counter <= 0;
                    -- Don't reset integrator completely - just decay it
                    integral_sum <= shift_right(integral_sum, 1);
                    -- Reset timeout counter to avoid continuous timeout pulses
                    timeout_counter <= (others => '0');
                end if;
            end if;
            
            if calc_valid_i = '1' then
                -- ============================================
                -- MEDIAN FILTER: Store samples and calculate median
                -- ============================================
                
                -- Store new samples in circular buffer
                offset_buffer(buffer_write_idx) <= offset_from_master_i;
                delay_buffer(buffer_write_idx)  <= mean_path_delay_i;
                
                -- Update write index (circular)
                if buffer_write_idx = MEDIAN_SIZE - 1 then
                    buffer_write_idx <= 0;
                else
                    buffer_write_idx <= buffer_write_idx + 1;
                end if;
                
                -- Track how many samples we have
                if buffer_fill_count < MEDIAN_SIZE then
                    buffer_fill_count <= buffer_fill_count + 1;
                end if;
                
                -- Calculate median when buffer is full
                if buffer_fill_count >= MEDIAN_SIZE - 1 then  -- Will be MEDIAN_SIZE after this cycle
                    median_offset <= median5(offset_buffer);
                    median_delay  <= median5(delay_buffer);
                    median_valid  <= '1';
                else
                    -- Not enough samples yet - use raw value
                    median_offset <= offset_from_master_i;
                    median_delay  <= mean_path_delay_i;
                    median_valid  <= '0';
                end if;
                
                -- Use median-filtered offset for processing
                if buffer_fill_count >= MEDIAN_SIZE then
                    offset_sample := median_offset;
                else
                    offset_sample := offset_from_master_i;
                end if;
                
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
                        if offset_sample > to_signed(2**30 - 1, 32) then
                            phase_jump_reg <= to_signed(-(2**30 - 1), 32);  -- Negative to slow down
                        elsif offset_sample < to_signed(-(2**30 - 1), 32) then
                            phase_jump_reg <= to_signed(2**30 - 1, 32);     -- Positive to speed up
                        else
                            phase_jump_reg <= -offset_sample;   -- Exact correction (negated)
                        end if;
                        phase_jump_valid_reg <= '1';
                        
                        -- Reset filter and integrator after phase jump
                        filtered_offset <= (others => '0');
                        integral_sum <= (others => '0');
                        freq_correction <= (others => '0');
                        sample_count <= 0;  -- Restart warmup
                        locked <= '0';
                        lock_counter <= 0;
                        -- Reset median filter buffers
                        offset_buffer <= (others => (others => '0'));
                        delay_buffer  <= (others => (others => '0'));
                        buffer_write_idx  <= 0;
                        buffer_fill_count <= 0;
                        median_valid <= '0';
                        
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
                            -- Clamp to 30-bit range to prevent wrap-around in multiplication
                            if filtered_offset > to_signed(2**30 - 1, 32) then
                                scaled_offset := to_signed(2**30 - 1, 32);
                            elsif filtered_offset < to_signed(-(2**30), 32) then
                                scaled_offset := to_signed(-(2**30), 32);
                            else
                                scaled_offset := filtered_offset;
                            end if;
                            
                            -- Proportional: Kp * scaled_offset / 2^effective_gain_shift
                            -- Note: 32-bit * 32-bit = 64-bit result
                            -- SIGN INVERSION: positive offset means local clock is AHEAD,
                            -- so we need NEGATIVE frequency correction to slow down.
                            -- Formula: freq_correction = -Kp * offset
                            -- 
                            -- GAIN SCALING: effective_gain_shift is larger for faster sync rates
                            -- This reduces the proportional gain to maintain stability
                            mult_result := scaled_offset * to_signed(KP_GAIN, 32);
                            proportional := -resize(shift_right(mult_result, effective_gain_shift), 32);
                            
                            -- Integral term: Accumulate filtered offset (also inverted)
                            -- Anti-windup: limit to ±500000 PPB (same as output limit!)
                            -- This prevents integrator from winding up beyond useful range
                            -- Note: integral uses effective_gain_shift + 2 (same relative Ki/Kp ratio)
                            if integral_sum > to_signed(-500_000, 32) and 
                               integral_sum < to_signed(500_000, 32) then
                                mult_result := scaled_offset * to_signed(KI_GAIN, 32);
                                integral_sum <= integral_sum - resize(shift_right(mult_result, effective_gain_shift + 2), 32);
                            elsif integral_sum >= to_signed(500_000, 32) and scaled_offset > 0 then
                                -- At positive limit but offset is positive, let it decrease
                                mult_result := scaled_offset * to_signed(KI_GAIN, 32);
                                integral_sum <= integral_sum - resize(shift_right(mult_result, effective_gain_shift + 2), 32);
                            elsif integral_sum <= to_signed(-500_000, 32) and scaled_offset < 0 then
                                -- At negative limit but offset is negative, let it increase
                                mult_result := scaled_offset * to_signed(KI_GAIN, 32);
                                integral_sum <= integral_sum - resize(shift_right(mult_result, effective_gain_shift + 2), 32);
                            end if;
                            
                            -- Combined PI output
                            pi_output := proportional + integral_sum;
                            
                            -- Limit frequency correction to ±500 PPM = ±500000 PPB
                            if pi_output > to_signed(500_000, 32) then
                                freq_correction <= to_signed(500_000, 32);
                            elsif pi_output < to_signed(-500_000, 32) then
                                freq_correction <= to_signed(-500_000, 32);
                            else
                                freq_correction <= pi_output;
                            end if;
                            
                            -- ============================================
                            -- Lock detection
                            -- ============================================
                            if offset_abs < to_signed(LOCK_THRESHOLD_NS, 32) then
                                if lock_counter < LOCK_COUNT_THRESHOLD then
                                    lock_counter <= lock_counter + 1;
                                else
                                    locked <= '1';
                                end if;
                            else
                                if offset_abs > to_signed(UNLOCK_THRESHOLD_NS, 32) then
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