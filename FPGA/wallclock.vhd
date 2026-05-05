library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity wallclock is
    generic(
        -- nanoseconds added per clock tick (125 MHz -> 8 ns)
        -- WARNING: Ensure this matches your clock! 125MHz=8ns, 62.5MHz=16ns
        increment_interval : natural := 8;
        -- System clock frequency in Hz (for NCO computation)
        sys_clk_hz : natural := 125_000_000;
        -- Audio sample rate in Hz
        audio_fs : natural := 48_000
    );
    port(
        clk                     : in  std_logic;
        reset_n                 : in  std_logic;

        wallclock_seconds_o     : out unsigned(47 downto 0);
        wallclock_nanoseconds_o : out unsigned(31 downto 0);

        wallclock_set_i         : in  std_logic;
        wallclock_seconds_i     : in  unsigned(47 downto 0);
        wallclock_nanoseconds_i : in  unsigned(31 downto 0);

        -- PTP Servo correction inputs
        freq_correction_ppb_i   : in  signed(31 downto 0);  -- PPB correction (parts per billion)
        phase_jump_ns_i         : in  signed(31 downto 0);  -- One-time phase adjustment in ns
        phase_jump_valid_i      : in  std_logic;            -- Pulse to apply phase jump

        second_pulse_o          : out std_logic;

        -- ============================================
        -- PTP-disciplined master audio clock (NCO-direct, no PLL)
        -- Output is register-driven from sys_clk domain.
        -- Jitter: ±1 sys_clk period (±8ns @125MHz).
        -- The NCO is disciplined via freq_correction_ppb, so this
        -- clock tracks PTP time with sub-PPB accuracy.
        -- ============================================
        audio_mclk_o            : out std_logic;  -- Master clock, fs*512 = 24.576 MHz

        -- ============================================
        -- Media clock for RTP packets (32-bit, from PTP epoch)
        -- Computed from wallclock: media_clock = (sec*48000 + sample_in_sec)
        -- ============================================
        media_clock_o           : out unsigned(31 downto 0);
        -- Pulse at fs rate in sys_clk domain (rising edge of LRCK)
        sample_pulse_o          : out std_logic;
        -- 1 ms tick in sys_clk domain (derived from sample_pulse, audio_fs/1000 samples)
        ms_pulse_o              : out std_logic
    );
end wallclock;

architecture Behavioral of wallclock is
    constant NS_PER_SEC : signed(31 downto 0) := to_signed(1_000_000_000, 32);
    
    -- Main time registers (as SIGNALS, not variables)
    signal nsec_reg : signed(31 downto 0) := (others => '0');
    signal sec_reg  : unsigned(47 downto 0) := (others => '0');
    
    -- Fractional nanosecond accumulator for sub-nanosecond precision
    -- Range limited to ±2^31 to prevent runaway
    signal frac_ns_accum : signed(31 downto 0) := (others => '0');
    
    -- Fractional accumulator overflow threshold.
    --
    -- Per cycle we add (increment_interval * ppb) = 8 * ppb to frac_ns_accum.
    -- Per real second there are sys_clk_hz cycles (1.25e8 @ 125 MHz). For
    -- 1 ppb to translate into exactly 1 ns/s of correction (the definition
    -- of ppb), we need: (sys_clk_hz * 8 * 1) / FRAC_OVERFLOW = 1 ns/s
    --   → FRAC_OVERFLOW = sys_clk_hz * increment_interval = 1e9.
    --
    -- The previous threshold of 2^30 was off by 2^30/1e9 ≈ 1.0737, i.e. the
    -- wallclock interpreted 1 ppb as ~0.93 ns/s. The PI servo locked the
    -- wallclock anyway (by overshooting the ppb output by ~7.4 %), but the
    -- NCO uses NCO_PPB_SCALE = NCO_BASE_INC_48 / 1e9 (correct ppb scaling),
    -- so it received the *overshot* value at face value and ran 7.4 % of
    -- the XO drift fast/slow relative to the master — visible as a constant
    -- audio fs drift even with PTP "locked".
    constant FRAC_OVERFLOW  : signed(31 downto 0) :=
        to_signed(sys_clk_hz * increment_interval, 32);
    constant FRAC_UNDERFLOW : signed(31 downto 0) :=
        to_signed(-(sys_clk_hz * increment_interval), 32);
    
    -- Internal second pulse (usable by other processes)
    signal second_pulse_int : std_logic := '0';
    
    -- ============================================================
    -- NCO for direct master clock generation (MCLK = fs * 512)
    -- ============================================================
    -- 48-bit phase accumulator: 32 integer bits + 16 fractional bits.
    -- Bit 47 (= old MSB) toggles at NCO frequency.
    -- Increment per 125 MHz tick (in 48-bit units):
    --   24576000/125e6 * 2^48 = NCO_BASE_INC_48 ≈ 5.534e13
    --
    -- The fractional bits avoid the lossy >>16 quantisation that the
    -- previous design applied to the ppb correction. Without them, the
    -- ppb-to-increment conversion truncated towards −∞ (signed ASR),
    -- biasing the NCO frequency slightly below the wallclock-disciplined
    -- target and causing the audio fs to drift even with PTP locked.
    -- ============================================================
    constant NCO_PHASE_BITS : natural := 48;
    constant NCO_FRAC_BITS  : natural := 16;  -- 48 - 32

    -- The original 32-bit NCO base increment fits in a VHDL integer.
    -- The 48-bit increment is just (this << NCO_FRAC_BITS), so we build it
    -- without ever needing a >32-bit integer literal (VHDL's integer is
    -- typically 32-bit on synthesisers).
    constant NCO_BASE_INC_32 : signed(31 downto 0) :=
        to_signed(integer(real(audio_fs * 512) / real(sys_clk_hz) * 4294967296.0), 32);

    constant NCO_BASE_INC_48 : signed(NCO_PHASE_BITS - 1 downto 0) :=
        shift_left(resize(NCO_BASE_INC_32, NCO_PHASE_BITS), NCO_FRAC_BITS);

    -- ppb correction multiplier: NCO_BASE_INC_48 / 1e9 (rounded to integer).
    -- 1 ppb of frequency correction = NCO_PPB_SCALE units of 48-bit increment.
    -- Width holds ~55340 for fs=48k @125MHz; 18 bits leaves headroom.
    -- Note: this is the OLD constant (used to be paired with >>16); now it's
    -- consumed at full precision because the 48-bit accumulator absorbs the
    -- extra 16 fractional bits.
    constant NCO_PPB_SCALE : signed(17 downto 0) :=
        to_signed(integer(real(audio_fs * 512) / real(sys_clk_hz)
                          * 4294967296.0 / 1.0e9 * 65536.0),
                  18);

    signal nco_phase      : unsigned(NCO_PHASE_BITS - 1 downto 0) := (others => '0');
    signal nco_phase_prev : std_logic := '0';  -- Previous MSB for edge detect
    signal nco_increment  : signed(NCO_PHASE_BITS - 1 downto 0) := NCO_BASE_INC_48;

    -- Pipeline register to break freq_correction → nco_increment critical path
    -- Original: 32×16 multiply + shift + add (~10ns combined)
    -- Split: Stage 1 (multiply) → Stage 2 (add). No shift now — the result
    -- is consumed at full 48-bit precision.
    signal ppb_adj_reg    : signed(NCO_PHASE_BITS - 1 downto 0) := (others => '0');
    
    -- Sample divider: count 512 MCLK rising edges → 1 sample period.
    -- MCLK MSB rising edge = one MCLK cycle. 512 cycles = 1 fs period.
    signal mclk_cnt      : unsigned(8 downto 0) := (others => '0');
    signal sample_pulse_int : std_logic := '0';

    -- Millisecond tick: count audio_fs/1000 sample pulses per ms.
    constant MS_DIVIDER    : natural := audio_fs / 1000;
    signal ms_cnt          : unsigned(15 downto 0) := (others => '0');
    signal ms_pulse_int    : std_logic := '0';
    
    -- ============================================================
    -- Media Clock (AES67): epoch-aligned, NCO-coherent
    --
    -- media_clock increments by 1 on each sample_pulse (NCO-driven),
    -- keeping it phase-coherent with fs_clk/LRCK. The NCO is PTP-
    -- disciplined via freq_correction_ppb, so the count stays accurate.
    -- On wallclock_set_i, the absolute value is loaded from the
    -- wallclock (sec*48000 + sample_in_sec) to establish epoch alignment.
    -- ============================================================
    constant MEDIA_CLK_RECIP : unsigned(17 downto 0) :=
        to_unsigned(integer(real(audio_fs) * 4294967296.0 / 1.0e9), 18);

    signal media_clock_reg  : unsigned(31 downto 0) := (others => '0');

    -- For wallclock_set resync: pipeline to compute absolute media clock
    -- Pipeline: Stage 0 (register nsec) → Stage 1 (multiply) → Stage 2 (add)
    signal media_nsec_reg   : unsigned(31 downto 0) := (others => '0');
    signal media_mult_reg   : unsigned(49 downto 0) := (others => '0');
    signal media_base       : unsigned(31 downto 0) := (others => '0');

    -- Resync counter: 0 = idle, 1..3 = pipeline flushing after wallclock_set
    signal media_resync_cnt : unsigned(1 downto 0) := (others => '0');
    
    -- ============================================================
    -- Pipeline registers to break freq_correction → sec_reg critical path.
    -- Original single-cycle chain (~17ns):
    --   multiply → frac_add → overflow → ns_adjust → nsec_add
    --   → compare_1e9 → 48-bit sec_increment
    -- Pipelined into 5 stages (≤6ns each):
    --   Stage 1: multiply → frac_increment_reg
    --   Stage 2: frac_add + overflow → ns_adjust_pipe
    --   Stage 3a: nsec_add → new_nsec_pipe (SPLIT from original Stage 3)
    --   Stage 3b: compare_1e9 + conditional → sec_adj_pipe
    --   Stage 4: 48-bit sec_increment
    -- Total latency: 4 extra cycles (32ns). Invisible at PTP rate (≤128 Hz).
    -- ============================================================
    signal frac_increment_reg : signed(31 downto 0) := (others => '0');
    signal ns_adjust_pipe     : integer range -1 to 1 := 0;
    -- Pre-computed increment value (breaks ns_adjust_pipe → new_nsec critical path)
    signal ns_increment_reg   : signed(31 downto 0) := to_signed(increment_interval, 32);
    signal new_nsec_pipe      : signed(31 downto 0) := (others => '0');  -- Stage 3a output
    signal sec_adj_pipe       : integer range -1 to 1 := 0;
    
    -- Pre-computed rollover values to break Stage 3b timing path
    -- These track (new_nsec_pipe - NS_PER_SEC) and (new_nsec_pipe + NS_PER_SEC)
    -- Updated in parallel with new_nsec_pipe using ns_increment_reg
    signal new_nsec_minus_sec : signed(31 downto 0) := -NS_PER_SEC;  -- new_nsec_pipe - NS_PER_SEC
    signal new_nsec_plus_sec  : signed(31 downto 0) := NS_PER_SEC;   -- new_nsec_pipe + NS_PER_SEC
    
    -- Phase jump pipeline registers
    -- Original: nsec_reg + phase_jump → compare → nsec_reg (~9ns in one cycle)
    -- Pipelined: Stage A: compute sum → phase_jump_sum_reg
    --            Stage B: compare/normalize → apply to nsec_reg, new_nsec_pipe
    signal phase_jump_pending : std_logic := '0';
    signal phase_jump_sum_reg : signed(31 downto 0) := (others => '0');
    signal nco_ppb_adj_wait : std_logic := '0';
    signal wallclock_set_i_reg : std_logic;
    
begin

    -- Output assignments
    wallclock_nanoseconds_o <= unsigned(nsec_reg);
    wallclock_seconds_o     <= sec_reg;
    second_pulse_o          <= second_pulse_int;
    audio_mclk_o            <= std_logic(nco_phase(NCO_PHASE_BITS - 1));  -- NCO MSB = MCLK at ~24.576 MHz
    media_clock_o           <= media_clock_reg;
    sample_pulse_o          <= sample_pulse_int;
    ms_pulse_o              <= ms_pulse_int;

    -- ============================================================
    -- NCO Process: Generate PTP-disciplined audio clocks
    -- BCLK (fs*64 = 3.072 MHz): NCO phase accumulator MSB
    -- LRCK (fs = 48 kHz): divide BCLK by 64
    -- Both are PTP-disciplined via freq_correction_ppb.
    --
    -- Pipeline (to meet timing):
    --   Stage 1: ppb_adj_reg <= freq_correction * NCO_PPB_SCALE  (full width)
    --   Stage 2: nco_increment <= NCO_BASE_INC_48 + ppb_adj_reg
    -- ============================================================

    nco_ppb_adj_proc: process(clk, reset_n)
    begin
        if reset_n = '0' then
            nco_ppb_adj_wait <= '0';
            ppb_adj_reg      <= (others => '0');
        elsif rising_edge(clk) then
            if (nco_ppb_adj_wait = '0') then
                nco_ppb_adj_wait <= '1';
                -- Full-width product fits in 32+18 = 50 bits; resize to 48
                -- (the integer overflow margin only matters for ppb beyond
                -- ±2^31/NCO_PPB_SCALE ≈ ±38 ppm, well outside servo range).
                ppb_adj_reg <= resize(freq_correction_ppb_i * NCO_PPB_SCALE,
                                      NCO_PHASE_BITS);
            else
                nco_ppb_adj_wait <= '0';
            end if;
        end if;
    end process;
    nco_proc: process(clk, reset_n)
    begin
        if reset_n = '0' then
            nco_phase        <= (others => '0');
            nco_phase_prev   <= '0';
            nco_increment    <= NCO_BASE_INC_48;
            mclk_cnt         <= (others => '0');
            sample_pulse_int <= '0';
        elsif rising_edge(clk) then
            sample_pulse_int <= '0';

            -- latch in when calc stable
            if (nco_ppb_adj_wait = '0') then
                nco_increment <= NCO_BASE_INC_48 + ppb_adj_reg;
            end if;
            -- Advance NCO phase accumulator
            nco_phase <= unsigned(signed(nco_phase) + nco_increment);
            nco_phase_prev <= std_logic(nco_phase(NCO_PHASE_BITS - 1));

            -- Detect rising edge of MCLK (NCO MSB 0→1): count 512 per sample
            if nco_phase_prev = '0' and nco_phase(NCO_PHASE_BITS - 1) = '1' then
                if mclk_cnt = 511 then
                    mclk_cnt         <= (others => '0');
                    sample_pulse_int <= '1';  -- One sys_clk pulse at sample boundary
                else
                    mclk_cnt <= mclk_cnt + 1;
                end if;
            end if;
        end if;
    end process nco_proc;

    -- ============================================================
    -- Millisecond Pulse Process
    -- Counts audio_fs/1000 sample pulses (48 @ 48 kHz) and emits a
    -- one-cycle pulse in sys_clk domain. Phase-coherent with the
    -- PTP-disciplined NCO, so drift is bounded by PTP accuracy.
    -- ============================================================
    ms_proc: process(clk, reset_n)
    begin
        if reset_n = '0' then
            ms_cnt       <= (others => '0');
            ms_pulse_int <= '0';
        elsif rising_edge(clk) then
            ms_pulse_int <= '0';
            if sample_pulse_int = '1' then
                if ms_cnt = MS_DIVIDER - 1 then
                    ms_cnt       <= (others => '0');
                    ms_pulse_int <= '1';
                else
                    ms_cnt <= ms_cnt + 1;
                end if;
            end if;
        end if;
    end process ms_proc;

    -- ============================================================
    -- Media Clock Process (AES67)
    --
    -- media_clock_reg increments by 1 on each sample_pulse_int
    -- (NCO-driven), keeping it phase-coherent with fs_clk/LRCK.
    -- No per-second resync — the NCO is already PTP-disciplined
    -- via freq_correction_ppb, so the count stays accurate.
    --
    -- On wallclock_set_i: hard-set to the wallclock-derived
    -- absolute value (sec*48000 + sample_in_sec) via a 3-stage
    -- pipeline. This is the only time the media clock jumps.
    -- ============================================================
    media_proc: process(clk, reset_n)
        variable sample_in_sec  : unsigned(15 downto 0);  -- 0..47999
    begin
        if reset_n = '0' then
            media_clock_reg  <= (others => '0');
            media_base       <= (others => '0');
            media_nsec_reg   <= (others => '0');
            media_mult_reg   <= (others => '0');
            media_resync_cnt <= (others => '0');
            wallclock_set_i_reg <= '0';
        elsif rising_edge(clk) then
            -- ===== Pipeline: always running for wallclock_set resync =====
            -- Stage 0: register nsec. On wallclock_set the main process writes
            -- nsec_reg <= wallclock_nanoseconds_i in the same cycle, so we have
            -- to inject the input value here directly — otherwise the pipeline
            -- captures the stale nsec_reg from before the set, producing a
            -- media_clock that jumps slightly off the new epoch every reset.
            if wallclock_set_i = '1' then
                media_nsec_reg <= wallclock_nanoseconds_i;
            else
                media_nsec_reg <= unsigned(nsec_reg);
            end if;
            -- Stage 1: multiply (32×18 = 50 bits)
            media_mult_reg <= media_nsec_reg * MEDIA_CLK_RECIP;
            wallclock_set_i_reg <= wallclock_set_i;
            if wallclock_set_i_reg = '1' then
                -- Hard set: compute media_base from wallclock seconds
                media_base <= resize(
                    wallclock_seconds_i(31 downto 0)
                    * to_unsigned(audio_fs, 32), 32);
                -- Start resync pipeline (3 cycles for nsec to propagate)
                media_resync_cnt <= to_unsigned(1, 2);

            elsif media_resync_cnt = 3 then
                -- Pipeline output ready: load absolute wallclock-derived value
                sample_in_sec := media_mult_reg(47 downto 32);
                media_clock_reg <= media_base + resize(sample_in_sec, 32);
                media_resync_cnt <= (others => '0');

            elsif media_resync_cnt /= 0 then
                -- Pipeline flushing, count up
                media_resync_cnt <= media_resync_cnt + 1;

            elsif sample_pulse_int = '1' then
                -- Normal operation: increment by 1, coherent with NCO
                media_clock_reg <= media_clock_reg + 1;
            end if;
        end if;
    end process media_proc;

    -- ============================================================
    -- Main Wallclock Timekeeping Process (5-stage pipeline)
    --
    -- Pipeline stages (signal reads get PREVIOUS cycle's value):
    --   Stage 1: freq_correction → multiply → frac_increment_reg  (~5ns)
    --   Stage 2: frac_increment_reg → frac_add → overflow → ns_adjust_pipe  (~6ns)
    --   Stage 3a: ns_adjust_pipe → nsec_add → new_nsec_pipe  (~5ns)
    --   Stage 3b: new_nsec_pipe → compare_1e9 → nsec_reg, sec_adj_pipe  (~6ns)
    --            Also resets new_nsec_pipe on rollover (overwrites Stage 3a)
    --   Stage 4: sec_adj_pipe → 48-bit sec increment  (~5ns)
    --
    -- Latency: 4 extra cycles (32ns) from freq_correction change to sec_reg.
    -- This is invisible: freq_correction changes at ≤128 Hz (millions of
    -- cycles apart), and the phase/second updates being a few cycles late
    -- (32ns) are negligible for PTP accuracy requirements.
    -- ============================================================
    process(clk, reset_n)
        variable new_frac  : signed(31 downto 0);
    begin
        if reset_n = '0' then
            second_pulse_int   <= '0';
            nsec_reg           <= (others => '0');
            sec_reg            <= (others => '0');
            frac_ns_accum      <= (others => '0');
            frac_increment_reg <= (others => '0');
            ns_adjust_pipe     <= 0;
            ns_increment_reg   <= to_signed(increment_interval, 32);
            new_nsec_pipe      <= (others => '0');
            new_nsec_minus_sec <= -NS_PER_SEC;
            new_nsec_plus_sec  <= NS_PER_SEC;
            sec_adj_pipe       <= 0;
            phase_jump_pending <= '0';
            phase_jump_sum_reg <= (others => '0');

        elsif rising_edge(clk) then
            second_pulse_int <= '0';

            -- ===== STAGE 1: Pre-compute multiply (always, independent) =====
            -- freq_correction changes at PTP rate — 1 cycle delay invisible.
            -- Use resize() to preserve sign; raw slicing of a signed vector
            -- drops the sign bit and corrupts negative corrections.
            frac_increment_reg <= resize(to_signed(increment_interval, 12)
                                         * resize(freq_correction_ppb_i, 20), 32);

            -- ===== STAGE 4: Apply delayed seconds rollover =====
            -- sec_adj_pipe was written by Stage 3b in the PREVIOUS cycle.
            -- VHDL signal semantics: we read the old value here.
            if sec_adj_pipe = 1 then
                sec_reg <= sec_reg + 1;
                second_pulse_int <= '1';
            elsif sec_adj_pipe = -1 then
                sec_reg <= sec_reg - 1;
            end if;
            -- Default for this cycle (may be overwritten below)
            sec_adj_pipe <= 0;

            -- ===== PHASE JUMP PIPELINE: Stage A (capture + add) =====
            -- On phase_jump_valid_i pulse: compute nsec_reg + phase_jump, set pending flag
            -- The actual application happens in Stage B next cycle.
            if phase_jump_valid_i = '1' then
                phase_jump_sum_reg <= nsec_reg + resize(phase_jump_ns_i, 32);
                phase_jump_pending <= '1';
            end if;

            -- ===== OVERRIDE PATHS =====
            if wallclock_set_i = '1' then
                -- Hard set of time — overrides everything
                nsec_reg      <= signed(wallclock_nanoseconds_i);
                sec_reg       <= wallclock_seconds_i;
                frac_ns_accum <= (others => '0');
                ns_adjust_pipe <= 0;
                new_nsec_pipe  <= signed(wallclock_nanoseconds_i);
                new_nsec_minus_sec <= signed(wallclock_nanoseconds_i) - NS_PER_SEC;
                new_nsec_plus_sec  <= signed(wallclock_nanoseconds_i) + NS_PER_SEC;
                sec_adj_pipe   <= 0;
                phase_jump_pending <= '0';

            elsif phase_jump_pending = '1' then
                -- ===== PHASE JUMP PIPELINE: Stage B (normalize + apply) =====
                -- Uses phase_jump_sum_reg from Stage A (previous cycle)
                phase_jump_pending <= '0';
                if phase_jump_sum_reg >= NS_PER_SEC then
                    nsec_reg      <= phase_jump_sum_reg - NS_PER_SEC;
                    new_nsec_pipe <= phase_jump_sum_reg - NS_PER_SEC;
                    new_nsec_minus_sec <= phase_jump_sum_reg - NS_PER_SEC - NS_PER_SEC;
                    new_nsec_plus_sec  <= phase_jump_sum_reg - NS_PER_SEC + NS_PER_SEC;
                    sec_adj_pipe  <= 1;
                elsif phase_jump_sum_reg < 0 then
                    nsec_reg      <= phase_jump_sum_reg + NS_PER_SEC;
                    new_nsec_pipe <= phase_jump_sum_reg + NS_PER_SEC;
                    new_nsec_minus_sec <= phase_jump_sum_reg;
                    new_nsec_plus_sec  <= phase_jump_sum_reg + NS_PER_SEC + NS_PER_SEC;
                    sec_adj_pipe  <= -1;
                else
                    nsec_reg      <= phase_jump_sum_reg;
                    new_nsec_pipe <= phase_jump_sum_reg;
                    new_nsec_minus_sec <= phase_jump_sum_reg - NS_PER_SEC;
                    new_nsec_plus_sec  <= phase_jump_sum_reg + NS_PER_SEC;
                end if;
                ns_adjust_pipe <= 0;

            else
                -- ===== STAGE 2: Fractional accumulation + overflow =====
                -- Uses frac_increment_reg from Stage 1 of PREVIOUS cycle.
                new_frac := frac_ns_accum + frac_increment_reg;
                ns_adjust_pipe <= 0;  -- default

                if new_frac >= FRAC_OVERFLOW then
                    frac_ns_accum  <= new_frac - FRAC_OVERFLOW;
                    ns_adjust_pipe <= 1;
                elsif new_frac <= FRAC_UNDERFLOW then
                    frac_ns_accum  <= new_frac - FRAC_UNDERFLOW;
                    ns_adjust_pipe <= -1;
                else
                    frac_ns_accum <= new_frac;
                end if;

                -- ===== STAGE 2.5: Pre-compute increment value =====
                -- ns_adjust_pipe read here is from Stage 2 of PREVIOUS cycle.
                -- This breaks the ns_adjust_pipe → new_nsec → new_nsec_minus_sec chain.
                ns_increment_reg <= to_signed(increment_interval + ns_adjust_pipe, 32);

                -- ===== STAGE 3a: Update all three values in PARALLEL =====
                -- Uses ns_increment_reg from PREVIOUS cycle (already computed).
                -- All three adds happen in parallel - no chain!
                new_nsec_pipe      <= new_nsec_pipe + ns_increment_reg;
                new_nsec_minus_sec <= new_nsec_minus_sec + ns_increment_reg;
                new_nsec_plus_sec  <= new_nsec_plus_sec + ns_increment_reg;

                -- ===== STAGE 3b: Compare and MUX (minimal arithmetic) =====
                -- Uses new_nsec_pipe from PREVIOUS cycle (reads OLD value).
                -- Uses pre-computed minus/plus values from PREVIOUS cycle.
                -- On rollover, reset all three values to maintain invariant:
                --   new_nsec_minus_sec = new_nsec_pipe - NS_PER_SEC
                --   new_nsec_plus_sec = new_nsec_pipe + NS_PER_SEC
                -- Use constants for minus/plus since after rollover the value is near 0 or 1e9.
                if new_nsec_pipe >= NS_PER_SEC then
                    nsec_reg      <= new_nsec_minus_sec;
                    -- Corrected value is ~ 0, so reset offsets to constants
                    new_nsec_pipe      <= new_nsec_minus_sec;
                    new_nsec_minus_sec <= -NS_PER_SEC;  -- ~0 - 1e9 = -1e9
                    new_nsec_plus_sec  <= NS_PER_SEC;   -- ~0 + 1e9 = +1e9
                    sec_adj_pipe  <= 1;
                elsif new_nsec_pipe < 0 then
                    nsec_reg      <= new_nsec_plus_sec;
                    -- Corrected value is ~ 1e9, so reset offsets to constants
                    new_nsec_pipe      <= new_nsec_plus_sec;
                    new_nsec_minus_sec <= (others => '0');  -- ~1e9 - 1e9 = 0
                    new_nsec_plus_sec  <= NS_PER_SEC + NS_PER_SEC;  -- ~1e9 + 1e9 = 2e9
                    sec_adj_pipe  <= -1;
                else
                    nsec_reg <= new_nsec_pipe;
                end if;
            end if;
        end if;
    end process;

end Behavioral;
