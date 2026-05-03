-- ============================================================
-- PTPv2 Servo - Clock Discipline Algorithm
-- ============================================================
-- Implements a PI (Proportional-Integral) controller to discipline
-- the local clock to the PTP master.
--
-- All tuning values that used to be generics are now ports so they
-- can be live-tuned via SoC CSRs / SPI without re-synthesising.
-- The remaining generics only define max-ranges for register widths.
--
-- Internal PI state (filtered offset, integral, proportional, raw sum,
-- effective shift, lock counter, sample count) is exposed on dedicated
-- monitoring outputs to allow the SoC to observe loop behaviour live.
-- ============================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity ptpv2_servo is
  generic (
    -- Maximum allowed warmup count and lock-count threshold.
    -- These size the internal counters; the actual operating values
    -- are taken from the *_i input ports.
    MAX_WARMUP_SAMPLES    : integer := 255;
    MAX_LOCK_COUNT        : integer := 255;
    -- Maximum effective gain shift (sets shifter width).
    MAX_GAIN_SHIFT        : integer := 31
  );
  port (
    clk     : in std_logic;
    reset_n : in std_logic;

    -- Input from ptpv2_parser
    offset_from_master_i     : in signed(31 downto 0); -- Nanoseconds
    calc_valid_i             : in std_logic;
    log_msg_interval_i       : in signed(7 downto 0);  -- PTP logMessageInterval (signed!)
    log_msg_interval_valid_i : in std_logic;           -- Pulse when interval is updated

    -- Live-tuning inputs (replace former generics)
    -- All defaults match the previous generic defaults.
    kp_gain_i              : in signed(7 downto 0)  := to_signed(40, 8);
    ki_gain_i              : in signed(7 downto 0)  := to_signed(5, 8);
    gain_shift_i           : in unsigned(4 downto 0):= to_unsigned(3, 5);
    gain_shift_locked_i    : in unsigned(4 downto 0):= to_unsigned(0, 5);
    ki_extra_shift_i       : in unsigned(4 downto 0):= to_unsigned(3, 5);
    filter_shift_i         : in unsigned(4 downto 0):= to_unsigned(0, 5);
    warmup_samples_i       : in unsigned(7 downto 0):= to_unsigned(16, 8);
    lock_threshold_ns_i    : in unsigned(31 downto 0):= to_unsigned(500, 32);
    unlock_threshold_ns_i  : in unsigned(31 downto 0):= to_unsigned(5000, 32);
    lock_count_threshold_i : in unsigned(7 downto 0):= to_unsigned(24, 8);

    -- Outputs to wallclock
    freq_correction_o  : out signed(31 downto 0); -- PPB correction (parts per billion)
    phase_jump_o       : out signed(31 downto 0); -- One-time ns adjustment
    phase_jump_valid_o : out std_logic;           -- Pulse to apply phase jump

    -- Request a full clock reconfiguration (offset implausibly large)
    request_clock_reconfigure_o : out std_logic;

    -- Status
    locked_o       : out std_logic;

    -- Monitoring outputs (live PI internal state)
    mon_filtered_offset_o      : out signed(31 downto 0);
    mon_integral_sum_o         : out signed(31 downto 0);
    mon_pi_proportional_o      : out signed(31 downto 0);
    mon_pi_sum_raw_o           : out signed(31 downto 0);
    mon_effective_gain_shift_o : out unsigned(7 downto 0);
    mon_lock_counter_o         : out unsigned(15 downto 0);
    mon_sample_count_o         : out unsigned(15 downto 0);
    mon_first_lock_achieved_o  : out std_logic
  );
end entity;

architecture Behavioral of ptpv2_servo is

  -- Filtered offset
  signal filtered_offset : signed(31 downto 0) := (others => '0');

  -- PI controller state (clamped to ±500,000)
  signal integral_sum    : signed(31 downto 0) := (others => '0');
  signal freq_correction : signed(31 downto 0) := (others => '0');

  -- Lock detection (sized via generic)
  signal lock_counter : integer range 0 to MAX_LOCK_COUNT := 0;
  signal locked       : std_logic                          := '0';

  -- Sample counter for warmup (sized via generic)
  signal sample_count : integer range 0 to MAX_WARMUP_SAMPLES + 1 := 0;

  -- Settle phase between frequency pre-seed and PI engagement.
  -- During settle, the IIR filter keeps running but pi_trigger stays low so
  -- the proportional path doesn't fight a not-yet-converged filter.
  constant SETTLE_SAMPLES : integer := 8;
  signal settle_count : integer range 0 to SETTLE_SAMPLES := 0;
  signal freq_seeded  : std_logic := '0';

  -- One-shot phase jump after settle, before PI engages. After the jump we
  -- wait one more sample so the filter sees the post-jump offset before PI runs.
  signal post_settle_jump_done : std_logic := '0';
  signal post_jump_wait_done   : std_logic := '0';

  -- Message interval tracking
  signal current_log_interval : signed(7 downto 0) := (others => '0');

  -- Sync rate gain scaling: adds -log_msg_interval to base shift
  signal interval_shift : natural range 0 to 7 := 0;

  -- Total effective gain shift
  signal EFFECTIVE_GAIN_SHIFT : natural range 0 to MAX_GAIN_SHIFT := 0;

  -- Frequency estimation
  -- first_offset is captured at sample_count = 0; the offset at sample_count =
  -- warmup-1 is used directly (inp_offset) to compute the drift over the
  -- warmup window. The drift seeds the integral so the loop starts on-frequency.
  signal first_offset : signed(31 downto 0) := (others => '0');
  signal first_lock_achieved : std_logic := '0';

  -- floor(log2(warmup_samples_i)) approximation. We assume warmup_samples_i is a
  -- power of two; if not, the seed magnitude is off by <2x which the I-loop absorbs.
  function log2_floor(v : unsigned) return integer is
  begin
    for i in v'high downto 0 loop
      if v(i) = '1' then
        return i;
      end if;
    end loop;
    return 0;
  end function;
  signal warmup_log2 : integer range 0 to 7 := 4;  -- default: log2(16) = 4

  -- Pre-seed value for the integral, computed once at end of warmup.
  signal freq_seed_ppb : signed(31 downto 0) := (others => '0');
  signal freq_seed_pulse : std_logic := '0';

  -- Sanity check: reject obviously invalid measurements (> 0.5 s)
  constant MAX_VALID_OFFSET : signed(31 downto 0) := to_signed(500_000_000, 32);

  -- Phase jump threshold: if offset > this, do phase jump
  constant PHASE_JUMP_THRESHOLD : signed(31 downto 0) := to_signed(200_000, 32);

  -- Reconfigure threshold
  constant RECONFIGURE_THRESHOLD : signed(31 downto 0) := to_signed(500_000, 32);

  -- Phase jump output registers
  signal phase_jump_reg       : signed(31 downto 0) := (others => '0');
  signal phase_jump_valid_reg : std_logic           := '0';

  -- Reconfigure request register (pulsed)
  signal request_reconfigure_reg : std_logic := '0';

  -- Pipelined PI controller
  type pi_state_t is (PI_IDLE, PI_MULT, PI_CLAMP, PI_SUM, PI_OUTPUT);
  signal pi_state   : pi_state_t := PI_IDLE;
  signal pi_trigger : std_logic  := '0';

  -- Decision flag: this measurement requests a clock reconfiguration
  signal inp_do_reconfigure : std_logic := '0';

  -- Pipeline registers for PI calculation
  signal pi_input        : signed(31 downto 0) := (others => '0');
  signal pi_mult_p       : signed(39 downto 0) := (others => '0'); -- 32+8 bits (kp 8b)
  signal pi_mult_i       : signed(39 downto 0) := (others => '0');
  signal pi_proportional : signed(31 downto 0) := (others => '0');
  signal pi_int_update   : signed(31 downto 0) := (others => '0');
  signal pi_sum_raw      : signed(31 downto 0) := (others => '0');

  -- Input processing pipeline
  type input_state_t is (INP_IDLE, INP_DECIDE, INP_ACTION);
  signal input_state : input_state_t := INP_IDLE;

  signal inp_offset_abs     : signed(31 downto 0) := (others => '0');
  signal inp_offset         : signed(31 downto 0) := (others => '0');
  signal inp_valid_meas     : std_logic           := '0';
  signal inp_do_phase_jump  : std_logic           := '0';
  signal inp_phase_jump_val : signed(31 downto 0) := (others => '0');
  signal inp_do_normal_op   : std_logic           := '0';
  signal pi_wait_state      : unsigned(1 downto 0)          := (others => '0');
begin

  -- Concurrent gain shift calculation, now sourced entirely from input ports.
  EFFECTIVE_GAIN_SHIFT <=
    to_integer(gain_shift_i) + interval_shift + to_integer(gain_shift_locked_i)
      when locked = '1' else
    to_integer(gain_shift_i) + interval_shift
      when first_lock_achieved = '1' else
    to_integer(gain_shift_i);

  warmup_log2 <= log2_floor(warmup_samples_i);

  -- Output assignments
  freq_correction_o  <= freq_correction;
  locked_o           <= locked;
  phase_jump_o       <= phase_jump_reg;
  phase_jump_valid_o <= phase_jump_valid_reg;
  request_clock_reconfigure_o <= request_reconfigure_reg;

  -- Monitoring outputs
  mon_filtered_offset_o      <= filtered_offset;
  mon_integral_sum_o         <= integral_sum;
  mon_pi_proportional_o      <= pi_proportional;
  mon_pi_sum_raw_o           <= pi_sum_raw;
  mon_effective_gain_shift_o <= to_unsigned(EFFECTIVE_GAIN_SHIFT, 8);
  mon_lock_counter_o         <= to_unsigned(lock_counter, 16);
  mon_sample_count_o         <= to_unsigned(sample_count, 16);
  mon_first_lock_achieved_o  <= first_lock_achieved;

  gain_scaler_proc : process (clk, reset_n)
  begin
    if reset_n = '0' then
      current_log_interval <= (others => '0');
      interval_shift       <= 0;
    elsif rising_edge(clk) then
      if log_msg_interval_valid_i = '1' then
        current_log_interval <= log_msg_interval_i;
        if log_msg_interval_i < 0 then
          if log_msg_interval_i < -7 then
            interval_shift <= 7;
          else
            interval_shift <= to_integer(-log_msg_interval_i);
          end if;
        else
          interval_shift <= 0;
        end if;
      end if;
    end if;
  end process;


  pi_controller_proc : process (clk, reset_n)
  begin
    if reset_n = '0' then
      pi_state <= PI_IDLE;
      pi_input        <= (others => '0');
      pi_mult_p       <= (others => '0');
      pi_mult_i       <= (others => '0');
      pi_proportional <= (others => '0');
      pi_int_update   <= (others => '0');
      pi_sum_raw      <= (others => '0');

      integral_sum    <= (others => '0');
      freq_correction <= (others => '0');
      pi_wait_state   <= (others => '0');
    elsif rising_edge(clk) then

      -- Frequency pre-seed: when servo_proc has computed freq_seed_ppb at the
      -- end of warmup, latch it into both the integral and freq_correction so
      -- the wallclock immediately tracks the estimated drift. This races with
      -- PI_SUM's integral_sum write, but freq_seed_pulse only fires during
      -- warmup before pi_trigger ever asserts, so the PI pipeline is idle.
      if freq_seed_pulse = '1' then
        integral_sum    <= freq_seed_ppb;
        freq_correction <= freq_seed_ppb;
      end if;

      if (pi_wait_state = 3) then
        case pi_state is
          when PI_IDLE =>
            if pi_trigger = '1' then
              pi_input <= filtered_offset;
              pi_state <= PI_MULT;
            end if;

          when PI_MULT =>
            -- 32×8 = 40 bits
            pi_mult_p <= pi_input * kp_gain_i;
            pi_mult_i <= pi_input * ki_gain_i;
            pi_state  <= PI_CLAMP;

          when PI_CLAMP =>
            pi_proportional <= - resize(shift_right(pi_mult_p, EFFECTIVE_GAIN_SHIFT), 32);
            pi_int_update <= integral_sum -
              resize(shift_right(pi_mult_i,
                                 EFFECTIVE_GAIN_SHIFT + to_integer(ki_extra_shift_i)), 32);
            pi_state      <= PI_SUM;

          when PI_SUM =>
            pi_sum_raw <= pi_proportional + integral_sum;

            if pi_int_update > to_signed(500_000, 32) then
              integral_sum <= to_signed(500_000, 32);
            elsif pi_int_update < to_signed(-500_000, 32) then
              integral_sum <= to_signed(-500_000, 32);
            else
              integral_sum <= pi_int_update;
            end if;
            pi_state <= PI_OUTPUT;

          when PI_OUTPUT =>
            if pi_sum_raw > to_signed(500_000, 32) then
              freq_correction <= to_signed(500_000, 32);
            elsif pi_sum_raw < to_signed(-500_000, 32) then
              freq_correction <= to_signed(-500_000, 32);
            else
              freq_correction <= pi_sum_raw;
            end if;

            pi_state <= PI_IDLE;
        end case;
      end if;
      pi_wait_state <= pi_wait_state + 1;
      
    end if;

  end process;

  -- detects lock state
  lock_detection_process : process (clk, reset_n)
  begin
    if reset_n = '0' then
      lock_counter <= 0;
      locked       <= '0';
      first_lock_achieved <= '0';
    elsif rising_edge(clk) then
      if (inp_do_phase_jump = '1' or inp_do_reconfigure = '1') then
        locked       <= '0';
        lock_counter <= 0;
      end if;

      if (pi_trigger = '1') then
        if inp_offset_abs < signed('0' & std_logic_vector(lock_threshold_ns_i)) then
          if lock_counter < to_integer(lock_count_threshold_i) and
             lock_counter < MAX_LOCK_COUNT then
            lock_counter <= lock_counter + 1;
          else
            locked              <= '1';
            first_lock_achieved <= '1';
          end if;
        else
          if inp_offset_abs > signed('0' & std_logic_vector(unlock_threshold_ns_i)) then
            locked       <= '0';
            lock_counter <= 0;
          elsif lock_counter > 0 then
            lock_counter <= lock_counter - 1;
          end if;
        end if;
      end if;
    end if;
  end process;
  -- servo controller
  servo_proc : process (clk, reset_n)
    variable filter_delta : signed(31 downto 0);
  begin
    if reset_n = '0' then
      filtered_offset      <= (others => '0');
      sample_count         <= 0;
      phase_jump_reg       <= (others => '0');
      phase_jump_valid_reg <= '0';
      first_offset         <= (others => '0');
      freq_seed_ppb        <= (others => '0');
      freq_seed_pulse      <= '0';
      settle_count         <= 0;
      freq_seeded          <= '0';
      post_settle_jump_done <= '0';
      post_jump_wait_done   <= '0';
      pi_trigger           <= '0';
      request_reconfigure_reg <= '0';
      input_state        <= INP_IDLE;
      inp_offset_abs     <= (others => '0');
      inp_offset         <= (others => '0');
      inp_valid_meas     <= '0';
      inp_do_phase_jump  <= '0';
      inp_phase_jump_val <= (others => '0');
      inp_do_normal_op   <= '0';
      inp_do_reconfigure <= '0';

    elsif rising_edge(clk) then
      phase_jump_valid_reg <= '0';
      pi_trigger           <= '0';
      request_reconfigure_reg <= '0';
      freq_seed_pulse      <= '0';

      case input_state is
        when INP_IDLE =>
          if calc_valid_i = '1' then
            inp_offset <= offset_from_master_i;
            if offset_from_master_i < 0 then
              inp_offset_abs <= - offset_from_master_i;
            else
              inp_offset_abs <= offset_from_master_i;
            end if;
            input_state <= INP_DECIDE;
          end if;

        when INP_DECIDE =>
          inp_do_phase_jump  <= '0';
          inp_do_normal_op   <= '0';
          inp_do_reconfigure <= '0';

          if inp_offset_abs < MAX_VALID_OFFSET then
            inp_valid_meas <= '1';

            if locked = '1'
               and inp_offset_abs > signed('0' & std_logic_vector(unlock_threshold_ns_i))
               and inp_offset_abs <= RECONFIGURE_THRESHOLD then
              inp_valid_meas <= '0';
            elsif inp_offset_abs > RECONFIGURE_THRESHOLD then
              inp_do_reconfigure <= '1';
            elsif inp_offset_abs > PHASE_JUMP_THRESHOLD
                  and sample_count >= to_integer(warmup_samples_i) then
              inp_do_phase_jump <= '1';
              if inp_offset > to_signed(2 ** 30 - 1, 32) then
                inp_phase_jump_val <= to_signed( - (2 ** 30 - 1), 32);
              elsif inp_offset < to_signed( - (2 ** 30 - 1), 32) then
                inp_phase_jump_val <= to_signed(2 ** 30 - 1, 32);
              else
                inp_phase_jump_val <= - inp_offset;
              end if;
            else
              inp_do_normal_op <= '1';
            end if;
          else
            inp_valid_meas <= '0';
          end if;

          input_state <= INP_ACTION;

        when INP_ACTION =>
          if inp_valid_meas = '1' then
            if sample_count <= to_integer(warmup_samples_i) and
               sample_count < MAX_WARMUP_SAMPLES then
              sample_count    <= sample_count + 1;
            end if;

            if inp_do_reconfigure = '1' then
              request_reconfigure_reg <= '1';

            elsif inp_do_phase_jump = '1' then
              phase_jump_reg       <= inp_phase_jump_val;
              phase_jump_valid_reg <= '1';

            elsif inp_do_normal_op = '1' then

              if sample_count = 0 then
                filtered_offset <= inp_offset;
                first_offset    <= inp_offset;
              else
                filter_delta := shift_right(inp_offset - filtered_offset,
                                            to_integer(filter_shift_i));
                filtered_offset <= filtered_offset + filter_delta;
              end if;

              -- End of warmup: estimate drift over the warmup window and
              -- pre-seed the integral. PTP convention: offset > 0 means the
              -- local (slave) clock is AHEAD of master ⇒ running too fast ⇒
              -- needs NEGATIVE ppb correction. So Δoffset > 0 over the warmup
              -- window ⇒ slave drifting further ahead ⇒ negative seed.
              -- ppb = -Δoffset_ns / (N · T_sync_s) where T_sync_s = 2^logInt.
              -- Approximate /N as shift right by floor(log2(N)). Net shift
              -- is (log2(N) + logInt); for fast sync (logInt < 0) this can
              -- go negative, in which case we left-shift instead.
              if sample_count = to_integer(warmup_samples_i) - 1 and freq_seeded = '0' then
                if (warmup_log2 + to_integer(current_log_interval)) >= 0 then
                  freq_seed_ppb <= shift_right(
                    first_offset - inp_offset,
                    warmup_log2 + to_integer(current_log_interval));
                else
                  freq_seed_ppb <= shift_left(
                    first_offset - inp_offset,
                    -(warmup_log2 + to_integer(current_log_interval)));
                end if;
                freq_seed_pulse <= '1';
                freq_seeded     <= '1';
                settle_count    <= 0;
              end if;

              -- Startup sequence after warmup:
              --   1) settle_count counts SETTLE_SAMPLES — IIR converges on the
              --      pre-seeded frequency.
              --   2) one-shot phase jump on the current offset.
              --   3) wait one sample so the filter sees the post-jump offset.
              --   4) pi_trigger goes high.
              if sample_count >= to_integer(warmup_samples_i) then
                if settle_count < SETTLE_SAMPLES then
                  settle_count <= settle_count + 1;
                elsif post_settle_jump_done = '0' then
                  if inp_offset > to_signed(2 ** 30 - 1, 32) then
                    phase_jump_reg <= to_signed( - (2 ** 30 - 1), 32);
                  elsif inp_offset < to_signed( - (2 ** 30 - 1), 32) then
                    phase_jump_reg <= to_signed(2 ** 30 - 1, 32);
                  else
                    phase_jump_reg <= - inp_offset;
                  end if;
                  phase_jump_valid_reg  <= '1';
                  post_settle_jump_done <= '1';
                elsif post_jump_wait_done = '0' then
                  post_jump_wait_done <= '1';
                else
                  pi_trigger <= '1';
                end if;
              end if;
            end if;
          end if;

          input_state <= INP_IDLE;
      end case;
    end if; -- rising_edge
  end process;

end Behavioral;
