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
-- SIMPLIFIED VERSION: No median filter!
-- The ptpv2_parser now handles min-filtering of path delays.
-- This servo just applies PI control directly to the input.
-- ============================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity ptpv2_servo is
    generic(
        -- PI controller gains (normalized to 1 Hz sync rate)
        -- Effective Kp = KP_GAIN / 2^GAIN_SHIFT / 2^(-log_msg_interval)
        -- Effective Ki = KI_GAIN / 2^(GAIN_SHIFT + 2) / 2^(-log_msg_interval)
        -- CRITICAL: At 1 Hz sample rate, Kp must be < 0.5 for stability!
        KP_GAIN : integer := 5;   -- Proportional gain numerator 
        KI_GAIN : integer := 6;    -- Integral gain numerator
        GAIN_SHIFT : integer := 6; -- Divide gains (base shift for 1 Hz)
        GAIN_SHIFT_LOCKED: integer := 4; -- Additional gain reduction when locked
        
        -- Filter coefficient for offset (exponential moving average)
        FILTER_SHIFT : integer := 0;  -- alpha
        
        -- Warmup: ignore first N samples to let filter settle
        WARMUP_SAMPLES : integer := 8; 
        
        -- Lock thresholds
        LOCK_THRESHOLD_NS   : integer := 500;    -- Consider locked if offset < 500ns
        UNLOCK_THRESHOLD_NS : integer := 5000;   -- Unlock if offset > 5µs
        LOCK_COUNT_THRESHOLD : integer := 30;     -- Need only 2 consecutive good measurements
        
        -- Sync timeout multiplier (timeout = 3 * sync_interval)
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
        phase_jump_o         : out signed(31 downto 0);  -- One-time ns adjustment
        phase_jump_valid_o   : out std_logic;            -- Pulse to apply phase jump
        
        -- Status
        locked_o             : out std_logic;
        sync_timeout_o       : out std_logic   -- Pulses when no sync received for too long
    );
end entity;

architecture Behavioral of ptpv2_servo is

    -- Filtered offset
    signal filtered_offset : signed(31 downto 0) := (others => '0');
    
    -- PI controller state (clamped to ±500,000)
    signal integral_sum    : signed(31 downto 0) := (others => '0');
    signal freq_correction : signed(31 downto 0) := (others => '0');
    
    -- Lock detection
    signal lock_counter    : integer range 0 to LOCK_COUNT_THRESHOLD := 0;
    signal locked          : std_logic := '0';
    
    -- Sample counter for warmup
    signal sample_count    : integer range 0 to WARMUP_SAMPLES + 1 := 0;
    
    -- Sync timeout counter
    signal timeout_counter : unsigned(31 downto 0) := (others => '0');
    signal timeout_limit   : unsigned(31 downto 0) := to_unsigned(CLOCK_FREQ_HZ * 4, 32);
    signal sync_timeout    : std_logic := '0';
    
    -- Message interval tracking
    signal current_log_interval : signed(7 downto 0) := (others => '0');
    
    -- Gain shift is a compile-time constant
    signal EFFECTIVE_GAIN_SHIFT : integer := GAIN_SHIFT;
    
    -- Sanity check: reject obviously invalid measurements (> 1s)
    constant MAX_VALID_OFFSET : signed(31 downto 0) := to_signed(500_000_000, 32);
    
    -- Phase jump threshold: if offset > this, do phase jump instead of frequency correction
    constant PHASE_JUMP_THRESHOLD : signed(31 downto 0) := to_signed(500_000, 32);  -- 1ms
    
    -- Phase jump output registers
    signal phase_jump_reg       : signed(31 downto 0) := (others => '0');
    signal phase_jump_valid_reg : std_logic := '0';
    
    -- Pipelined PI controller
    type pi_state_t is (PI_IDLE, PI_MULT, PI_CLAMP, PI_SUM, PI_OUTPUT);
    signal pi_state       : pi_state_t := PI_IDLE;
    signal pi_trigger     : std_logic := '0';
    
    -- Pipeline registers for PI calculation
    signal pi_input       : signed(31 downto 0) := (others => '0');
    signal pi_mult_p      : signed(35 downto 0) := (others => '0');  -- 32+4 bits
    signal pi_mult_i      : signed(35 downto 0) := (others => '0');
    signal pi_proportional: signed(31 downto 0) := (others => '0');
    signal pi_int_update  : signed(31 downto 0) := (others => '0');
    signal pi_sum_raw     : signed(31 downto 0) := (others => '0');  -- Added: pre-clamp sum
    
    -- Input processing pipeline
    type input_state_t is (INP_IDLE, INP_DECIDE, INP_ACTION);
    signal input_state       : input_state_t := INP_IDLE;
    
    -- Stage registers
    signal inp_offset_abs    : signed(31 downto 0) := (others => '0');
    signal inp_offset        : signed(31 downto 0) := (others => '0');
    signal inp_valid_meas    : std_logic := '0';
    signal inp_do_phase_jump : std_logic := '0';
    signal inp_phase_jump_val: signed(31 downto 0) := (others => '0');
    signal inp_do_normal_op  : std_logic := '0';
    
begin

    -- Output assignments
    freq_correction_o <= freq_correction;
    locked_o <= locked;
    phase_jump_o <= phase_jump_reg;
    phase_jump_valid_o <= phase_jump_valid_reg;
    sync_timeout_o <= sync_timeout;
    
    servo_proc: process(clk, reset_n)
        variable filter_delta : signed(31 downto 0);
        variable abs_offset   : signed(31 downto 0);
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
            timeout_limit   <= to_unsigned(CLOCK_FREQ_HZ * 4, 32);
            sync_timeout    <= '0';
            current_log_interval <= (others => '0');
            -- PI pipeline reset
            pi_state <= PI_IDLE;
            pi_trigger <= '0';
            pi_input <= (others => '0');
            pi_mult_p <= (others => '0');
            pi_mult_i <= (others => '0');
            pi_proportional <= (others => '0');
            pi_int_update <= (others => '0');
            pi_sum_raw <= (others => '0');
            -- Input processing pipeline reset
            input_state <= INP_IDLE;
            inp_offset_abs <= (others => '0');
            inp_offset <= (others => '0');
            inp_valid_meas <= '0';
            inp_do_phase_jump <= '0';
            inp_phase_jump_val <= (others => '0');
            inp_do_normal_op <= '0';
            
        elsif rising_edge(clk) then
            -- Default: no phase jump this cycle, clear timeout pulse
            phase_jump_valid_reg <= '0';
            sync_timeout <= '0';
            pi_trigger <= '0';
            
            -- ============================================
            -- PIPELINED PI CONTROLLER (5 states)
            -- PI_IDLE → PI_MULT → PI_CLAMP → PI_SUM → PI_OUTPUT
            -- Each state ≤ 6ns to meet 125 MHz timing.
            -- ============================================
            case pi_state is
                when PI_IDLE =>
                    if pi_trigger = '1' then
                        pi_input <= filtered_offset;
                        pi_state <= PI_MULT;
                    end if;
                    
                when PI_MULT =>
                    -- Perform both multiplications (32×4 = 36 bits)
                    pi_mult_p <= pi_input * to_signed(KP_GAIN, 4);
                    pi_mult_i <= pi_input * to_signed(KI_GAIN, 4);
                    pi_state <= PI_CLAMP;
                    
                when PI_CLAMP =>
                    -- Shift and negate proportional term
                    pi_proportional <= -resize(shift_right(pi_mult_p, EFFECTIVE_GAIN_SHIFT), 32);
                    -- Calculate integral update term
                    pi_int_update <= integral_sum - resize(shift_right(pi_mult_i, EFFECTIVE_GAIN_SHIFT + 2), 32);
                    pi_state <= PI_SUM;
                
                when PI_SUM =>
                    -- ===== NEW STAGE: Compute sum before clamping =====
                    -- Uses pi_proportional from previous cycle, integral_sum (old value)
                    pi_sum_raw <= pi_proportional + integral_sum;
                    
                    -- Clamp and update integral (can happen in parallel)
                    if pi_int_update > to_signed(500_000, 32) then
                        integral_sum <= to_signed(500_000, 32);
                    elsif pi_int_update < to_signed(-500_000, 32) then
                        integral_sum <= to_signed(-500_000, 32);
                    else
                        integral_sum <= pi_int_update;
                    end if;
                    pi_state <= PI_OUTPUT;
                    
                when PI_OUTPUT =>
                    -- ===== Clamp final output from registered sum =====
                    -- Uses pi_sum_raw from previous cycle (already computed)
                    if pi_sum_raw > to_signed(500_000, 32) then
                        freq_correction <= to_signed(500_000, 32);
                    elsif pi_sum_raw < to_signed(-500_000, 32) then
                        freq_correction <= to_signed(-500_000, 32);
                    else
                        freq_correction <= pi_sum_raw;
                    end if;
                    
                    pi_state <= PI_IDLE;
            end case;
            
            -- ============================================
            -- UPDATE MESSAGE INTERVAL
            -- ============================================
            if log_msg_interval_valid_i = '1' then
                current_log_interval <= log_msg_interval_i;
                
                -- Calculate timeout limit based on message interval
                if log_msg_interval_i >= 0 then
                    if log_msg_interval_i > 3 then
                        timeout_limit <= to_unsigned(CLOCK_FREQ_HZ * 32, 32);
                    else
                        timeout_limit <= shift_left(to_unsigned(CLOCK_FREQ_HZ * 4, 32), to_integer(log_msg_interval_i));
                    end if;
                else
                    if log_msg_interval_i < -4 then
                        timeout_limit <= to_unsigned(CLOCK_FREQ_HZ / 4, 32);
                    else
                        timeout_limit <= shift_right(to_unsigned(CLOCK_FREQ_HZ * 4, 32), to_integer(-log_msg_interval_i));
                    end if;
                end if;
            end if;
            
            -- ============================================
            -- SYNC TIMEOUT DETECTION
            -- ============================================
            if calc_valid_i = '1' then
                timeout_counter <= (others => '0');
            else
                if timeout_counter < timeout_limit then
                    timeout_counter <= timeout_counter + 1;
                else
                    sync_timeout <= '1';
                    locked <= '0';
                    lock_counter <= 0;
                    integral_sum <= shift_right(integral_sum, 1);
                    timeout_counter <= (others => '0');
                end if;
            end if;
            
            -- ============================================
            -- INPUT PROCESSING PIPELINE (SIMPLIFIED - NO MEDIAN)
            -- The parser now handles min-filtering.
            -- We just check for validity and decide action.
            -- ============================================
            case input_state is
                when INP_IDLE =>
                    if calc_valid_i = '1' then
                        -- Capture input and compute absolute value
                        inp_offset <= offset_from_master_i;
                        if offset_from_master_i < 0 then
                            inp_offset_abs <= -offset_from_master_i;
                        else
                            inp_offset_abs <= offset_from_master_i;
                        end if;
                        input_state <= INP_DECIDE;
                    end if;
                    
                when INP_DECIDE =>
                    -- Determine action based on offset magnitude
                    inp_do_phase_jump <= '0';
                    inp_do_normal_op <= '0';

                    -- Sanity check: reject obviously invalid measurements
                    if inp_offset_abs < MAX_VALID_OFFSET then
                        inp_valid_meas <= '1';

                        -- Phase jump decision
                        if inp_offset_abs > PHASE_JUMP_THRESHOLD and sample_count >= WARMUP_SAMPLES then
                            inp_do_phase_jump <= '1';
                            -- Calculate phase jump value (negate offset)
                            if inp_offset > to_signed(2**30 - 1, 32) then
                                inp_phase_jump_val <= to_signed(-(2**30 - 1), 32);
                            elsif inp_offset < to_signed(-(2**30 - 1), 32) then
                                inp_phase_jump_val <= to_signed(2**30 - 1, 32);
                            else
                                inp_phase_jump_val <= -inp_offset;
                            end if;
                        else
                            inp_do_normal_op <= '1';
                        end if;
                    else
                        inp_valid_meas <= '0';
                    end if;
                    
                    input_state <= INP_ACTION;
                    
                when INP_ACTION =>
                    -- Execute decided action
                    if inp_valid_meas = '1' then
                        -- Count valid samples
                        if sample_count <= WARMUP_SAMPLES then
                            sample_count <= sample_count + 1;
                        end if;
                        
                        if inp_do_phase_jump = '1' then
                            -- Apply phase jump
                            phase_jump_reg <= inp_phase_jump_val;
                            phase_jump_valid_reg <= '1';
                            locked <= '0';
                            lock_counter <= 0;
                            
                        elsif inp_do_normal_op = '1' then
                            -- Normal frequency correction operation
                            
                            -- Low-pass filter the offset
                            if sample_count = 0 then
                                filtered_offset <= inp_offset;
                            else
                                filter_delta := shift_right(inp_offset - filtered_offset, FILTER_SHIFT);
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
                                        EFFECTIVE_GAIN_SHIFT <= GAIN_SHIFT_LOCKED;
                                    end if;
                                else
                                    if inp_offset_abs > to_signed(UNLOCK_THRESHOLD_NS, 32) then
                                        locked <= '0';
                                        EFFECTIVE_GAIN_SHIFT <= GAIN_SHIFT;
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
