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

entity ptpv2_servo_with_median is
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

architecture Behavioral of ptpv2_servo_with_median is

    -- Filtered offset - 25 bits handles offsets up to PHASE_JUMP_THRESHOLD (10M < 16M)
    signal filtered_offset : signed(24 downto 0) := (others => '0');
    
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
    -- OPTIMIZATION: 25-bit buffer since values > PHASE_JUMP_THRESHOLD (10M)
    -- trigger immediate phase jump. 25 bits handles ±16M with saturation.
    -- ============================================
    constant MEDIAN_SIZE : integer := 5;
    constant MEDIAN_SAT_LIMIT : signed(24 downto 0) := to_signed(10_000_000, 25);  -- Match PHASE_JUMP_THRESHOLD
    type sample_buffer_t is array(0 to MEDIAN_SIZE-1) of signed(24 downto 0);
    
    -- Sample buffer (circular) - only offset needs median filtering
    signal offset_buffer      : sample_buffer_t := (others => (others => '0'));
    signal buffer_write_idx   : integer range 0 to MEDIAN_SIZE-1 := 0;
    signal buffer_fill_count  : integer range 0 to MEDIAN_SIZE := 0;
    
    -- Median-filtered values (25 bits; values > PHASE_JUMP_THRESHOLD trigger phase jump)
    signal median_offset      : signed(24 downto 0) := (others => '0');
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
    -- OPTIMIZATION: Only 25 bits needed since values > 10M trigger phase jump anyway.
    type median_state_t is (M_IDLE, M_STAGE1, M_STAGE2, M_STAGE3, M_SELECT);
    signal median_state   : median_state_t := M_IDLE;
    signal sort_a, sort_b, sort_c, sort_d, sort_e : signed(24 downto 0) := (others => '0');
    signal median_trigger : std_logic := '0';
    
    -- ============================================
    -- Pipelined PI controller signals
    -- The combinational path filtered_offset -> freq_correction was ~15ns.
    -- Pipeline spreads the PI calculation over 4 clock cycles.
    -- calc_valid pulses arrive millions of cycles apart (1-8 Hz), so latency is invisible.
    --
    -- OPTIMIZATION: Register widths are minimized based on:
    --   - Output clamped to ±500,000 (needs 20 bits)
    --   - Max useful input: 500,000 * 64 / 13 ≈ 2,500,000 (needs 22 bits)
    --   - Multiply 22-bit × 4-bit = 26 bits (KP/KI max ~16)
    --   - After shift by 6: 20 bits
    -- ============================================
    type pi_state_t is (PI_IDLE, PI_MULT, PI_CLAMP, PI_OUTPUT);
    signal pi_state       : pi_state_t := PI_IDLE;
    signal pi_trigger     : std_logic := '0';
    
    -- Clamped input limit: values beyond this saturate the output anyway
    constant PI_INPUT_LIMIT : signed(21 downto 0) := to_signed(2_500_000, 22);
    
    -- Pipeline registers for PI calculation (optimized widths)
    signal pi_input       : signed(21 downto 0) := (others => '0');  -- Clamped input (22 bit)
    signal pi_mult_p      : signed(25 downto 0) := (others => '0');  -- 22 + 4 bit for KP
    signal pi_mult_i      : signed(25 downto 0) := (others => '0');  -- 22 + 4 bit for KI
    signal pi_proportional: signed(19 downto 0) := (others => '0');  -- Clamped proportional
    signal pi_int_update  : signed(19 downto 0) := (others => '0');  -- Integral update value
    
    -- ============================================
    -- INPUT PROCESSING PIPELINE
    -- Critical timing path was: buffer_fill_count → offset_sample mux →
    -- offset_abs calculation → phase_jump decision → offset_buffer reset
    -- This 4-stage pipeline breaks the 11ns path into ~3ns stages.
    -- ============================================
    type input_state_t is (INP_IDLE, INP_SELECT, INP_DECIDE, INP_ACTION);
    signal input_state       : input_state_t := INP_IDLE;
    
    -- Stage 1 registers: Captured and saturated input
    signal inp_offset_sat    : signed(24 downto 0) := (others => '0');  -- Saturated input
    signal inp_write_idx     : integer range 0 to MEDIAN_SIZE-1 := 0;   -- Where to write
    signal inp_fill_was_full : std_logic := '0';                        -- Was buffer full?
    
    -- Stage 2 registers: Selected offset and absolute value
    signal inp_offset_selected : signed(31 downto 0) := (others => '0');
    signal inp_offset_abs      : signed(31 downto 0) := (others => '0');
    signal inp_valid_meas      : std_logic := '0';  -- offset_abs < MAX_VALID_OFFSET
    
    -- Stage 3 registers: Decision results
    signal inp_do_phase_jump   : std_logic := '0';
    signal inp_phase_jump_val  : signed(31 downto 0) := (others => '0');
    signal inp_do_normal_op    : std_logic := '0';
    
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
        variable filter_delta    : signed(24 downto 0);  -- Same width as filtered_offset
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
            pi_proportional <= (others => '0');
            pi_int_update <= (others => '0');
            -- Input processing pipeline reset
            input_state <= INP_IDLE;
            inp_offset_sat <= (others => '0');
            inp_write_idx <= 0;
            inp_fill_was_full <= '0';
            inp_offset_selected <= (others => '0');
            inp_offset_abs <= (others => '0');
            inp_valid_meas <= '0';
            inp_do_phase_jump <= '0';
            inp_phase_jump_val <= (others => '0');
            inp_do_normal_op <= '0';
            
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
            -- Spreads the PI calculation over 3 clock cycles:
            --   PI_MULT:   Clamp input, perform multiplications
            --   PI_CLAMP:  Shift, clamp P term, calculate I update
            --   PI_OUTPUT: Add P+I, clamp final output
            --
            -- OPTIMIZATION: Input is pre-clamped to ±2.5M so multiply
            -- results fit in 26 bits. This saves ~44 LUTs per 48→26 bit.
            -- ============================================
            pi_trigger <= '0';  -- Default: clear trigger each cycle
            
            case pi_state is
                when PI_IDLE =>
                    if pi_trigger = '1' then
                        -- Clamp filtered_offset to useful range before multiplication
                        -- Values beyond PI_INPUT_LIMIT would saturate output anyway
                        if filtered_offset > resize(PI_INPUT_LIMIT, 25) then
                            pi_input <= PI_INPUT_LIMIT;
                        elsif filtered_offset < -resize(PI_INPUT_LIMIT, 25) then
                            pi_input <= -PI_INPUT_LIMIT;
                        else
                            pi_input <= resize(filtered_offset, 22);
                        end if;
                        pi_state <= PI_MULT;
                    end if;
                    
                when PI_MULT =>
                    -- Perform both multiplications (22-bit × 4-bit = 26-bit)
                    pi_mult_p <= pi_input * to_signed(KP_GAIN, 4);
                    pi_mult_i <= pi_input * to_signed(KI_GAIN, 4);
                    pi_state <= PI_CLAMP;
                    
                when PI_CLAMP =>
                    -- Shift and clamp proportional term (negate for correction direction)
                    -- After shift by 6, result fits in 20 bits due to input clamping
                    pi_proportional <= -resize(shift_right(pi_mult_p, EFFECTIVE_GAIN_SHIFT), 20);
                    
                    -- Calculate integral update (will clamp in next stage)
                    pi_int_update <= integral_sum - resize(shift_right(pi_mult_i, EFFECTIVE_GAIN_SHIFT + 2), 20);
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
            
            -- ============================================
            -- PIPELINED INPUT PROCESSING
            -- Breaks the critical path: buffer_fill_count → offset_buffer
            -- into 4 stages with ~3ns each (was ~11ns combinational).
            --
            -- Stage 1 (CAPTURE): Register input, write buffer, update indices
            -- Stage 2 (SELECT):  Choose median vs raw, compute abs value
            -- Stage 3 (DECIDE):  Determine phase_jump vs normal operation
            -- Stage 4 (ACTION):  Execute the decided action
            -- ============================================
            case input_state is
                when INP_IDLE =>
                    if calc_valid_i = '1' then
                        -- ========== STAGE 1: CAPTURE ==========
                        -- Saturate input to 25-bit and store metadata
                        if offset_from_master_i > resize(MEDIAN_SAT_LIMIT, 32) then
                            inp_offset_sat <= MEDIAN_SAT_LIMIT;
                        elsif offset_from_master_i < -resize(MEDIAN_SAT_LIMIT, 32) then
                            inp_offset_sat <= -MEDIAN_SAT_LIMIT;
                        else
                            inp_offset_sat <= resize(offset_from_master_i, 25);
                        end if;
                        
                        -- Capture current write index before updating
                        inp_write_idx <= buffer_write_idx;
                        
                        -- Remember if buffer was full (for median vs raw selection)
                        if buffer_fill_count >= MEDIAN_SIZE then
                            inp_fill_was_full <= '1';
                        else
                            inp_fill_was_full <= '0';
                        end if;
                        
                        -- Update write index (circular) - will take effect next cycle
                        if buffer_write_idx = MEDIAN_SIZE - 1 then
                            buffer_write_idx <= 0;
                        else
                            buffer_write_idx <= buffer_write_idx + 1;
                        end if;
                        
                        -- Update fill count
                        if buffer_fill_count < MEDIAN_SIZE then
                            buffer_fill_count <= buffer_fill_count + 1;
                        end if;
                        
                        input_state <= INP_SELECT;
                    end if;
                    
                when INP_SELECT =>
                    -- ========== STAGE 2: WRITE BUFFER & SELECT ==========
                    -- Write saturated sample to buffer (using captured index)
                    offset_buffer(inp_write_idx) <= inp_offset_sat;
                    
                    -- Trigger median computation if buffer is now full
                    if buffer_fill_count >= MEDIAN_SIZE then
                        median_trigger <= '1';
                    end if;
                    
                    -- Select between median-filtered or raw offset
                    -- Use inp_fill_was_full (registered) to avoid timing path
                    if inp_fill_was_full = '1' then
                        inp_offset_selected <= resize(median_offset, 32);
                    else
                        inp_offset_selected <= resize(inp_offset_sat, 32);
                    end if;
                    
                    -- Pre-calculate absolute value of the raw/saturated input
                    -- (We'll use this for decisions in next stage)
                    if inp_fill_was_full = '1' then
                        if median_offset < 0 then
                            inp_offset_abs <= resize(-median_offset, 32);
                        else
                            inp_offset_abs <= resize(median_offset, 32);
                        end if;
                    else
                        if inp_offset_sat < 0 then
                            inp_offset_abs <= resize(-inp_offset_sat, 32);
                        else
                            inp_offset_abs <= resize(inp_offset_sat, 32);
                        end if;
                    end if;
                    
                    input_state <= INP_DECIDE;
                    
                when INP_DECIDE =>
                    -- ========== STAGE 3: DECIDE ==========
                    -- Determine what action to take based on registered values
                    inp_do_phase_jump <= '0';
                    inp_do_normal_op <= '0';
                    
                    -- Sanity check: reject obviously invalid measurements
                    if inp_offset_abs < MAX_VALID_OFFSET then
                        inp_valid_meas <= '1';
                        
                        -- Phase jump decision
                        if inp_offset_abs > PHASE_JUMP_THRESHOLD then
                            inp_do_phase_jump <= '1';
                            -- Pre-calculate phase jump value
                            if inp_offset_selected > to_signed(2**30 - 1, 32) then
                                inp_phase_jump_val <= to_signed(-(2**30 - 1), 32);
                            elsif inp_offset_selected < to_signed(-(2**30 - 1), 32) then
                                inp_phase_jump_val <= to_signed(2**30 - 1, 32);
                            else
                                inp_phase_jump_val <= -inp_offset_selected;
                            end if;
                        else
                            inp_do_normal_op <= '1';
                        end if;
                    else
                        inp_valid_meas <= '0';
                    end if;
                    
                    input_state <= INP_ACTION;
                    
                when INP_ACTION =>
                    -- ========== STAGE 4: ACTION ==========
                    -- Execute the decided action using registered decision signals
                    
                    if inp_valid_meas = '1' then
                        -- Count valid samples
                        if sample_count <= WARMUP_SAMPLES then
                            sample_count <= sample_count + 1;
                        end if;
                        
                        if inp_do_phase_jump = '1' then
                            -- Apply phase jump
                            phase_jump_reg <= inp_phase_jump_val;
                            phase_jump_valid_reg <= '1';
                            
                            -- Reset filter and integrator
                            filtered_offset <= (others => '0');
                            integral_sum <= (others => '0');
                            freq_correction <= (others => '0');
                            sample_count <= 0;
                            locked <= '0';
                            lock_counter <= 0;
                            
                            -- Reset median filter
                            offset_buffer <= (others => (others => '0'));
                            buffer_write_idx <= 0;
                            buffer_fill_count <= 0;
                            median_valid <= '0';
                            median_state <= M_IDLE;
                            median_trigger <= '0';
                            
                            -- Reset PI pipeline
                            pi_state <= PI_IDLE;
                            pi_trigger <= '0';
                            
                        elsif inp_do_normal_op = '1' then
                            -- Normal frequency correction operation
                            
                            -- Low-pass filter the offset
                            if sample_count = 0 then
                                filtered_offset <= resize(inp_offset_selected, 25);
                            else
                                filter_delta := resize(shift_right(resize(inp_offset_selected, 25) - filtered_offset, FILTER_SHIFT), 25);
                                filtered_offset <= filtered_offset + filter_delta;
                            end if;
                            
                            -- PI Controller (after warmup)
                            if sample_count >= WARMUP_SAMPLES then
                                pi_trigger <= '1';
                                
                                -- Lock detection
                                if inp_offset_abs < to_signed(LOCK_THRESHOLD_NS, 32) then
                                    if lock_counter < LOCK_COUNT_THRESHOLD then
                                        lock_counter <= lock_counter + 1;
                                    else
                                        locked <= '1';
                                    end if;
                                else
                                    if inp_offset_abs > to_signed(UNLOCK_THRESHOLD_NS, 32) then
                                        locked <= '0';
                                    end if;
                                    lock_counter <= 0;
                                end if;
                            end if;
                        end if;
                    end if;
                    
                    input_state <= INP_IDLE;
            end case;
        end if;  -- rising_edge
    end process;

end Behavioral;