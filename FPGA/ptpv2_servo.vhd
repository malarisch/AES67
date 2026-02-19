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
        sync_timeout_o       : out std_logic   -- Pulses when no sync received for too long
    );
end entity;

architecture Behavioral of ptpv2_servo is

    -- Filtered offset - full 32 bits to handle offsets up to PHASE_JUMP_THRESHOLD
    signal filtered_offset : signed(31 downto 0) := (others => '0');
    
    -- PI controller state - narrowed to 20 bits
    -- Both are clamped to ±500,000; 2^19 = 524,288 > 500,000
    signal integral_sum    : signed(19 downto 0) := (others => '0');
    signal freq_correction : signed(19 downto 0) := (others => '0');
    
    -- Lock detection - sized to actual need
    signal lock_counter    : integer range 0 to LOCK_COUNT_THRESHOLD := 0;
    signal locked          : std_logic := '0';
    
    -- Sample counter for warmup - sized to actual need
    signal sample_count    : integer range 0 to WARMUP_SAMPLES + 1 := 0;
    
    -- Sync timeout counter
    signal timeout_counter : unsigned(31 downto 0) := (others => '0');
    signal timeout_limit   : unsigned(31 downto 0) := to_unsigned(CLOCK_FREQ_HZ * 4, 32);  -- Default 4s for 1 Hz
    signal sync_timeout    : std_logic := '0';
    
    -- Message interval tracking
    signal current_log_interval : signed(7 downto 0) := (others => '0');
    
    -- Gain shift is a compile-time constant (avoids runtime barrel shifter)
    constant EFFECTIVE_GAIN_SHIFT : integer := GAIN_SHIFT;
    
    -- ============================================
    -- Median filter for outlier rejection (5 samples)
    -- ============================================
    constant MEDIAN_SIZE : integer := 5;
    type sample_buffer_t is array(0 to MEDIAN_SIZE-1) of signed(31 downto 0);
    
    -- Sample buffer (circular) - only offset needs median filtering
    signal offset_buffer      : sample_buffer_t := (others => (others => '0'));
    signal buffer_write_idx   : integer range 0 to MEDIAN_SIZE-1 := 0;
    signal buffer_fill_count  : integer range 0 to MEDIAN_SIZE := 0;
    
    -- Median-filtered values
    signal median_offset      : signed(31 downto 0) := (others => '0');
    signal median_valid       : std_logic := '0';
    
    -- Sanity check: reject obviously invalid measurements (> 1s)
    constant MAX_VALID_OFFSET : signed(31 downto 0) := to_signed(1_000_000_000, 32);  -- 1s
    
    -- Phase jump threshold: if offset > this, do phase jump instead of frequency correction
    -- Must be large enough that PI loop handles initial clock_set residual (~3ms)
    -- but small enough to quickly correct true time steps
    constant PHASE_JUMP_THRESHOLD : signed(31 downto 0) := to_signed(10_000_000, 32);  -- 10ms
    
    -- Phase jump output registers
    signal phase_jump_reg       : signed(31 downto 0) := (others => '0');
    signal phase_jump_valid_reg : std_logic := '0';
    
    -- Pipelined median computation signals
    -- The combinational sorting network was the critical timing path (~23ns).
    -- Pipeline spreads the 5-element sort over 5 clock cycles (~3ns per stage).
    -- calc_valid pulses arrive millions of cycles apart, so latency is invisible.
    type median_state_t is (M_IDLE, M_STAGE1, M_STAGE2, M_STAGE3, M_SELECT);
    signal median_state   : median_state_t := M_IDLE;
    signal sort_a, sort_b, sort_c, sort_d, sort_e : signed(31 downto 0) := (others => '0');
    signal median_trigger : std_logic := '0';
    
    -- ============================================
    -- Pipelined PI controller signals
    -- The combinational path filtered_offset -> freq_correction was ~15ns.
    -- Pipeline spreads the PI calculation over 5 clock cycles.
    -- calc_valid pulses arrive millions of cycles apart (1-8 Hz), so latency is invisible.
    -- ============================================
    type pi_state_t is (PI_IDLE, PI_MULT, PI_SHIFT, PI_CLAMP, PI_OUTPUT);
    signal pi_state       : pi_state_t := PI_IDLE;
    signal pi_trigger     : std_logic := '0';
    
    -- Pipeline registers for PI calculation
    signal pi_input       : signed(31 downto 0) := (others => '0');  -- Latched filtered_offset
    signal pi_mult_p      : signed(47 downto 0) := (others => '0');  -- KP multiplication result
    signal pi_mult_i      : signed(47 downto 0) := (others => '0');  -- KI multiplication result
    signal pi_shifted_p   : signed(47 downto 0) := (others => '0');  -- Shifted P term
    signal pi_shifted_i   : signed(47 downto 0) := (others => '0');  -- Shifted I term
    signal pi_proportional: signed(19 downto 0) := (others => '0');  -- Clamped proportional
    signal pi_int_update  : signed(19 downto 0) := (others => '0');  -- Integral update value
    
begin

    -- Output assignments (sign-extend narrowed internal signals to 32-bit ports)
    freq_correction_o <= resize(freq_correction, 32);
    locked_o <= locked;
    phase_jump_o <= phase_jump_reg;
    phase_jump_valid_o <= phase_jump_valid_reg;
    sync_timeout_o <= sync_timeout;
    
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
        variable filter_delta    : signed(31 downto 0);  -- Same width as filtered_offset
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
            buffer_write_idx  <= 0;
            buffer_fill_count <= 0;
            median_offset <= (others => '0');
            median_valid  <= '0';
            median_state  <= M_IDLE;
            sort_a <= (others => '0');
            sort_b <= (others => '0');
            sort_c <= (others => '0');
            sort_d <= (others => '0');
            sort_e <= (others => '0');
            median_trigger <= '0';
            -- PI pipeline reset
            pi_state <= PI_IDLE;
            pi_trigger <= '0';
            pi_input <= (others => '0');
            pi_mult_p <= (others => '0');
            pi_mult_i <= (others => '0');
            pi_shifted_p <= (others => '0');
            pi_shifted_i <= (others => '0');
            pi_proportional <= (others => '0');
            pi_int_update <= (others => '0');
            
        elsif rising_edge(clk) then
            -- Default: no phase jump this cycle, clear timeout pulse
            phase_jump_valid_reg <= '0';
            sync_timeout <= '0';
            median_trigger <= '0';  -- Default: clear trigger pulse each cycle
            
            -- ============================================
            -- PIPELINED MEDIAN SORT
            -- Sorting network for 5 elements spread over 5 clock cycles.
            -- Each stage does at most one 32-bit compare-swap per element,
            -- easily meeting 125 MHz timing (~3ns combinational depth).
            -- ============================================
            case median_state is
                when M_IDLE =>
                    if median_trigger = '1' then
                        -- Load sort registers from buffer (buffer was updated
                        -- on the previous cycle, so it includes the latest sample)
                        sort_a <= offset_buffer(0);
                        sort_b <= offset_buffer(1);
                        sort_c <= offset_buffer(2);
                        sort_d <= offset_buffer(3);
                        sort_e <= offset_buffer(4);
                        median_state <= M_STAGE1;
                    end if;
                    
                when M_STAGE1 =>
                    -- Compare-swap pairs: (a,b) and (c,d) in parallel
                    if sort_a > sort_b then
                        sort_a <= sort_b; sort_b <= sort_a;
                    end if;
                    if sort_c > sort_d then
                        sort_c <= sort_d; sort_d <= sort_c;
                    end if;
                    median_state <= M_STAGE2;
                    
                when M_STAGE2 =>
                    -- Cross compare-swap: (a,c) and (b,d) in parallel
                    if sort_a > sort_c then
                        sort_a <= sort_c; sort_c <= sort_a;
                    end if;
                    if sort_b > sort_d then
                        sort_b <= sort_d; sort_d <= sort_b;
                    end if;
                    median_state <= M_STAGE3;
                    
                when M_STAGE3 =>
                    -- Final swaps: (b,c) and (a,e) in parallel
                    if sort_b > sort_c then
                        sort_b <= sort_c; sort_c <= sort_b;
                    end if;
                    if sort_a > sort_e then
                        sort_a <= sort_e; sort_e <= sort_a;
                    end if;
                    median_state <= M_SELECT;
                    
                when M_SELECT =>
                    -- Median is among sort_b, sort_c, sort_e
                    -- After sorting: a <= b <= c <= d, a <= e
                    if sort_e > sort_c then
                        median_offset <= sort_c;
                    elsif sort_e < sort_b then
                        median_offset <= sort_b;
                    else
                        median_offset <= sort_e;
                    end if;
                    median_valid <= '1';
                    median_state <= M_IDLE;
            end case;
            
            -- ============================================
            -- PIPELINED PI CONTROLLER
            -- Spreads the PI calculation over 4 clock cycles:
            --   PI_MULT:   Perform 32x16 multiplications
            --   PI_SHIFT:  Shift results for gain scaling
            --   PI_CLAMP:  Clamp P term, calculate I update
            --   PI_OUTPUT: Add P+I, clamp final output
            -- ============================================
            pi_trigger <= '0';  -- Default: clear trigger each cycle
            
            case pi_state is
                when PI_IDLE =>
                    if pi_trigger = '1' then
                        -- Latch filtered_offset and start multiplication
                        pi_input <= filtered_offset;
                        pi_state <= PI_MULT;
                    end if;
                    
                when PI_MULT =>
                    -- Perform both multiplications (32-bit × 16-bit = 48-bit)
                    pi_mult_p <= pi_input * to_signed(KP_GAIN, 16);
                    pi_mult_i <= pi_input * to_signed(KI_GAIN, 16);
                    pi_state <= PI_SHIFT;
                    
                when PI_SHIFT =>
                    -- Shift for gain scaling
                    pi_shifted_p <= shift_right(pi_mult_p, EFFECTIVE_GAIN_SHIFT);
                    pi_shifted_i <= shift_right(pi_mult_i, EFFECTIVE_GAIN_SHIFT + 2);
                    pi_state <= PI_CLAMP;
                    
                when PI_CLAMP =>
                    -- Clamp proportional term to ±500,000 (and negate)
                    if pi_shifted_p > to_signed(500_000, 48) then
                        pi_proportional <= to_signed(-500_000, 20);
                    elsif pi_shifted_p < to_signed(-500_000, 48) then
                        pi_proportional <= to_signed(500_000, 20);
                    else
                        pi_proportional <= -resize(pi_shifted_p, 20);
                    end if;
                    
                    -- Calculate integral update (will clamp in next stage)
                    pi_int_update <= integral_sum - resize(pi_shifted_i, 20);
                    pi_state <= PI_OUTPUT;
                    
                when PI_OUTPUT =>
                    -- Clamp and update integral (for next iteration)
                    if pi_int_update > to_signed(500_000, 20) then
                        integral_sum <= to_signed(500_000, 20);
                    elsif pi_int_update < to_signed(-500_000, 20) then
                        integral_sum <= to_signed(-500_000, 20);
                    else
                        integral_sum <= pi_int_update;
                    end if;
                    
                    -- Calculate and clamp final PI output
                    -- Note: Uses OLD integral_sum (matching original behavior)
                    if pi_proportional + integral_sum > to_signed(500_000, 20) then
                        freq_correction <= to_signed(500_000, 20);
                    elsif pi_proportional + integral_sum < to_signed(-500_000, 20) then
                        freq_correction <= to_signed(-500_000, 20);
                    else
                        freq_correction <= pi_proportional + integral_sum;
                    end if;
                    
                    pi_state <= PI_IDLE;
            end case;
            
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
                
                -- Store new offset sample in circular buffer
                offset_buffer(buffer_write_idx) <= offset_from_master_i;
                
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
                    -- Trigger pipelined median computation (result in ~5 clock cycles)
                    median_trigger <= '1';
                else
                    -- Not enough samples yet - use raw value directly
                    median_offset <= offset_from_master_i;
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
                    
                    -- Count valid samples (only up to warmup threshold + 1)
                    if sample_count <= WARMUP_SAMPLES then
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
                        -- Reset median filter buffer and pipeline
                        offset_buffer <= (others => (others => '0'));
                        buffer_write_idx  <= 0;
                        buffer_fill_count <= 0;
                        median_valid <= '0';
                        median_state <= M_IDLE;
                        median_trigger <= '0';
                        -- Reset PI pipeline
                        pi_state <= PI_IDLE;
                        pi_trigger <= '0';
                        
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
                        -- Trigger the pipelined PI calculation.
                        -- Result will be available in ~4 clock cycles.
                        -- ============================================
                        if sample_count >= WARMUP_SAMPLES then
                            pi_trigger <= '1';
                            
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