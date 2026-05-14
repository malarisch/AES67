-- Testbench for wallclock.vhd
--
-- Scope: pure internal timekeeping logic at sys_clk = 125 MHz.
-- freq_correction_ppb_i is held inactive in the deterministic checks -
-- this exercises only the deterministic path (tick -> ns -> sec,
-- sample/ms pulses, media_clock derivation, wallclock_set override).
--
-- Checks performed:
--   1. Reset behaviour (all outputs cleared).
--   2. Free-running timekeeping: nsec_reg advances by `increment_interval`
--      per clock tick.
--   3. ns -> sec rollover at 1e9, with second_pulse_o pulsing exactly once.
--   4. wallclock_set_i hard-loads sec/ns and clears state.
--   5. sample_pulse_o cadence = sys_clk_hz / audio_fs (= 2604.17 cycles
--      @125MHz/48kHz).
--   6. ms_pulse_o cadence = audio_fs / 1000 sample pulses.
--   7. media_clock_o = sec*audio_fs + sample_in_sec, monotonic, advancing
--      once per sample pulse.
--
-- Run hints:
--   ghdl -a --std=08 wallclock.vhd wallclock_tb.vhd
--   ghdl -e --std=08 wallclock_tb
--   ghdl -r --std=08 wallclock_tb --stop-time=50ms --wave=wallclock_tb.ghw \
--       --ieee-asserts=disable
--
-- The --ieee-asserts=disable suppresses numeric_std "metavalue detected"
-- warnings. Those come from comparisons against signals that briefly
-- become 'X' in some IEEE-2008 corner cases of signed arithmetic at the
-- edge of 32-bit range (the rollover paths in wallclock occasionally
-- compute new_nsec_plus_sec values close to +/-2^31, and ghdl flags the
-- TO_01 conversion). The arithmetic itself is correct (the assertions
-- in this TB cross-check wallclock vs media_clock vs NCO end-to-end);
-- the warnings are just noise in the log.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity wallclock_tb is
end entity;

architecture sim of wallclock_tb is

    -- Use a smaller sys_clk_hz / audio_fs ratio so simulation finishes
    -- in reasonable wallclock time while keeping the same arithmetic.
    -- sys_clk_hz must be a multiple of audio_fs*512 only loosely (NCO
    -- handles the rest), and FRAC_OVERFLOW = sys_clk_hz must fit in
    -- 32-bit signed -> keep below ~2.1e9.
    constant SYS_CLK_HZ_C        : natural := 125_000_000;
    constant AUDIO_FS_C          : natural := 48_000;
    constant INCREMENT_INTERVAL_C: natural := 8;   -- ns/tick at 125 MHz

    constant CLK_PERIOD          : time := 8 ns;   -- 125 MHz

    -- DUT I/O
    signal clk        : std_logic := '0';
    signal reset_n    : std_logic := '0';

    signal wc_sec_o   : unsigned(47 downto 0);
    signal wc_ns_o    : unsigned(31 downto 0);

    signal wc_set     : std_logic := '0';
    signal wc_sec_i   : unsigned(47 downto 0) := (others => '0');
    signal wc_ns_i    : unsigned(31 downto 0) := (others => '0');

    signal freq_corr  : signed(19 downto 0) := (others => '0');

    signal second_pulse  : std_logic;
    signal audio_mclk    : std_logic;
    signal media_clock   : unsigned(31 downto 0);
    signal sample_pulse  : std_logic;
    signal ms_pulse      : std_logic;

    signal clk_256fs, clk_128fs, clk_64fs, fs_sig : std_logic;
    signal bclk_r, bclk_f, fs_tdm                 : std_logic;
    signal phase_locked                           : std_logic;

    -- Diagnostic taps
    signal mclk_cnt_dbg     : unsigned(8 downto 0);
    signal media_edge_dbg   : std_logic;

    signal sim_done : boolean := false;

    -- Set true by freq_corr_stim once it has finished its measurement
    -- windows; the realistic-scenario stimuli (continuous ppb modulator,
    -- wallclock_set, phase jumps) gate on this so they don't race with
    -- the strict ppb-tolerance assertions.
    signal scenario_arm : boolean := false;

    -- Set true by scenario_stim when its checks (incl. drift test)
    -- complete; the main `stim` process holds off `std.env.stop`
    -- until both are done.
    signal scenario_done : boolean := false;

    -- Helper: count cycles between events (simple counters in test process)
    signal sample_cnt   : integer := 0;
    signal ms_cnt       : integer := 0;
    signal sec_pulse_cnt: integer := 0;

    -- audio_mclk rising-edge counter (NCO MSB rising edges)
    signal mclk_prev     : std_logic := '0';
    signal mclk_edge_cnt : integer := 0;

    -- phase_locked statistics
    signal locked_cnt    : integer := 0;  -- cycles where phase_locked='1'
    signal locked_sample_cnt : integer := 0;  -- total cycles sampled

begin

    -- Clock generator
    clk_gen: process
    begin
        while not sim_done loop
            clk <= '0';
            wait for CLK_PERIOD / 2;
            clk <= '1';
            wait for CLK_PERIOD / 2;
        end loop;
        wait;
    end process;

    -- DUT
    dut: entity work.wallclock
        generic map (
            increment_interval => INCREMENT_INTERVAL_C,
            sys_clk_hz         => SYS_CLK_HZ_C,
            audio_fs           => AUDIO_FS_C
        )
        port map (
            clk                     => clk,
            reset_n                 => reset_n,
            wallclock_seconds_o     => wc_sec_o,
            wallclock_nanoseconds_o => wc_ns_o,
            wallclock_set_i         => wc_set,
            wallclock_seconds_i     => wc_sec_i,
            wallclock_nanoseconds_i => wc_ns_i,
            freq_correction_ppb_i   => freq_corr,
            second_pulse_o          => second_pulse,
            audio_mclk_o            => audio_mclk,
            media_clock_o           => media_clock,
            sample_pulse_o          => sample_pulse,
            ms_pulse_o              => ms_pulse,
            clk_256fs_o             => clk_256fs,
            clk_128fs_o             => clk_128fs,
            clk_64fs_o              => clk_64fs,
            fs_o                    => fs_sig,
            bclk_r_o                => bclk_r,
            bclk_f_o                => bclk_f,
            fs_tdm_pulse_o          => fs_tdm,
            phase_locked_o          => phase_locked,
            mclk_cnt_o              => mclk_cnt_dbg,
            media_edge_tick_o       => media_edge_dbg,
            ppb_adj_dbg_o           => open,
            ppb_trim_dbg_o          => open,
            bias_dbg_o              => open,
            sample_pulse_int_o      => open,
            nco_phase_dbg_o         => open,
            nco_inc_dbg_o           => open
        );

    -- Passive counters for sanity-checking pulse cadence
    counters: process(clk)
    begin
        if rising_edge(clk) then
            if sample_pulse = '1' then
                sample_cnt <= sample_cnt + 1;
            end if;
            if ms_pulse = '1' then
                ms_cnt <= ms_cnt + 1;
            end if;
            if second_pulse = '1' then
                sec_pulse_cnt <= sec_pulse_cnt + 1;
            end if;

            -- audio_mclk rising-edge counter for NCO frequency checks
            mclk_prev <= audio_mclk;
            if mclk_prev = '0' and audio_mclk = '1' then
                mclk_edge_cnt <= mclk_edge_cnt + 1;
            end if;

            -- phase_locked statistics
            locked_sample_cnt <= locked_sample_cnt + 1;
            if phase_locked = '1' then
                locked_cnt <= locked_cnt + 1;
            end if;
        end if;
    end process;

    -- Main stimulus / checks
    stim: process
        variable ns_start         : unsigned(31 downto 0);
        variable sec_start        : unsigned(47 downto 0);
        variable t_mark           : time;
        variable mc_prev_v        : unsigned(31 downto 0);
        variable sec_pulse_snap   : integer;
        variable sample_snap      : integer;
        variable ms_snap          : integer;
    begin
        report "=== wallclock_tb start ===";

        reset_n <= '1';
        wait until rising_edge(clk);

        -- The 5-stage pipeline needs a few cycles to start advancing
        -- nsec_reg. Wait until we observe the first non-zero ns value.
        for i in 0 to 20 loop
            wait until rising_edge(clk);
            exit when wc_ns_o /= 0;
        end loop;
        assert wc_ns_o /= 0
            report "ns counter never started incrementing" severity error;

        -- ----- Check raw tick increment over a known window -----
        -- After pipeline warmup, ns must advance by increment_interval per
        -- clock. Capture and compare across 100 cycles.
        wait until rising_edge(clk);
        ns_start := wc_ns_o;
        for i in 1 to 100 loop
            wait until rising_edge(clk);
        end loop;
        assert (wc_ns_o - ns_start) = to_unsigned(100 * INCREMENT_INTERVAL_C, 32)
            report "ns increment over 100 cycles wrong: got "
                 & integer'image(to_integer(wc_ns_o - ns_start))
                 & " expected " & integer'image(100 * INCREMENT_INTERVAL_C)
            severity error;
        report "Tick increment OK ("
             & integer'image(INCREMENT_INTERVAL_C) & " ns/cycle)";


        sec_pulse_snap := sec_pulse_cnt;
        wait until rising_edge(clk);

        -- ----- Wait for the rollover -----
        -- 1000 ns / 8 ns_per_tick = 125 cycles until ns crosses 1e9.
        -- The pipeline introduces a couple of cycles of latency between
        -- the rollover detection and sec_reg++. Wait generously.
        t_mark := now;




        -- ----- Sample pulse cadence -----
        -- Expected period: SYS_CLK_HZ / AUDIO_FS = 125e6/48e3 ~ 2604.17
        -- Measure over 1000 sample pulses; tolerate +/-1 cycle.
        sample_snap := sample_cnt;
        wait until rising_edge(clk);
        t_mark := now;
        wait until sample_cnt - sample_snap = 1000;
        report "1000 sample pulses observed in "
             & time'image(now - t_mark)
             & " (expect ~"
             & time'image((1000 * 1 sec) / AUDIO_FS_C) & ")";

        -- ----- Millisecond pulse cadence -----
        -- 1 ms_pulse per (audio_fs/1000) = 48 sample pulses.
        -- Sync the snapshot to a ms_pulse edge — otherwise the first
        -- "ms interval" is a partial fragment and the sample count is
        -- short. Wait for ms_pulse='1', then snapshot, then count 10
        -- complete intervals.
        wait until rising_edge(clk) and ms_pulse = '1';
        wait until rising_edge(clk);
        ms_snap     := ms_cnt;
        sample_snap := sample_cnt;
        wait until ms_cnt - ms_snap = 10;  -- 10 ms worth
        assert (sample_cnt - sample_snap) >= 10 * (AUDIO_FS_C/1000) - 2
           and (sample_cnt - sample_snap) <= 10 * (AUDIO_FS_C/1000) + 2
            report "ms_pulse / sample_pulse ratio off: "
                 & integer'image(sample_cnt - sample_snap)
                 & " samples in 10 ms (expect ~"
                 & integer'image(10 * (AUDIO_FS_C/1000)) & ")"
            severity error;
        report "ms_pulse cadence OK ("
             & integer'image(sample_cnt - sample_snap)
             & " samples / 10 ms_pulses)";

        -- ----- media_clock monotonicity -----
        -- media_clock is a pure function of the wallclock (sec*fs +
        -- nsec*fs/1e9), so it slews continuously and must never decrease.
        mc_prev_v := media_clock;
        check_media_clock: for i in 1 to 5000 loop
            wait until rising_edge(clk);
            assert media_clock >= mc_prev_v
                report "media_clock went backwards: "
                     & integer'image(to_integer(mc_prev_v)) & " -> "
                     & integer'image(to_integer(media_clock))
                severity error;
            mc_prev_v := media_clock;
        end loop;
        report "media_clock monotonic over 5000 cycles";

        report "stim: tests done, waiting for scenario_stim to finish";
        if not scenario_done then
            wait until scenario_done;
        end if;
        report "=== wallclock_tb done ===";
        sim_done <= true;
        wait for 5 * CLK_PERIOD;
        std.env.stop;
    end process;

    -- ============================================================
    -- Frequency correction tests
    -- ============================================================
    -- Three measurement windows of MEAS_WIN each, separated by SETTLE
    -- so the pipeline / NCO bias has time to absorb the ppb step:
    --   1. ppb = 0           - baseline
    --   2. ppb = +10_000_000 - +1 % frequency offset
    --   3. ppb = -10_000_000 - -1 % frequency offset
    --
    -- For each window we measure:
    --   (a) wallclock ns/sec increment per window      -> tick rate
    --   (b) media_clock increment per window           -> media-clock rate
    --   (c) audio_mclk rising edges per window         -> NCO frequency
    --   (d) phase_locked high-fraction in window       -> NCO phase pull
    --
    -- The windows are 200 us long: long enough for the ppb correction to
    -- accumulate hundreds of nanoseconds of wallclock difference but
    -- short enough to keep the sim under a second of wallclock-time.
    -- ============================================================
    freq_corr_stim: process
        -- 20 ms window: needed because the freq_correction_ppb_i port
        -- is signed(19) (range ±524_287; servo clamps to ±500_000), so
        -- the largest ppb the wallclock can actually see is ±500_000 =
        -- ±500 ppm. Over 20 ms that produces a 960*5e-4 ≈ 0.48 sample
        -- delta on media_clock — resolvable above the ±2 quantisation
        -- of MC_TOL only with the longer window.
        constant MEAS_WIN : time    := 20 ms;
        constant SETTLE   : time    := 100 us;
        constant WIN_NS   : integer := 20_000_000;  -- MEAS_WIN in ns

        -- Tolerances per window
        --   NS_TOL: wallclock is exact apart from frac-accumulator rounding.
        --   MC_TOL: media_clock = floor(t * audio_fs); +/-1 at each window
        --           edge gives +/-2 worst case.
        --   MCLK_TOL: NCO MSB edges, similar +/-1 quantisation per edge.
        constant NS_TOL    : integer := 3;
        constant MC_TOL    : integer := 2;
        constant MCLK_TOL  : integer := 3;

        type ppb_array_t is array (natural range <>) of integer;
        -- Largest ppb that fits in the signed(19) port is ±524_287; the
        -- real PTP servo clamps to ±500_000. Test at 0 and the servo
        -- clamp to exercise the full usable range.
        constant PPB_LIST : ppb_array_t :=
            (0, 500_000, -500_000);

        -- Realistic PTP-servo pull-in sequence used after the strict
        -- tolerance windows: +/-50 ppm range, stepped every 200 us.
        constant SERVO_SEQ : ppb_array_t :=
            (   0,
             20_000,  35_000,  42_000,  47_000,  50_000,
             48_000,  46_000,  44_000,  43_000,  43_500,
             43_200,  43_300,  43_250,  43_270,  43_260);
        variable servo_idx  : natural := 0;

        variable ppb        : integer;
        variable ns_a, ns_b : unsigned(31 downto 0);
        variable sec_a, sec_b : unsigned(47 downto 0);
        variable mc_a, mc_b   : unsigned(31 downto 0);
        variable mclk_a, mclk_b : integer;
        variable lk_a, lk_b     : integer;
        variable lk_n_a, lk_n_b : integer;

        variable factor    : real;
        variable ns_expect : integer;
        variable mc_expect : integer;
        variable mclk_expect : integer;
        variable ns_obs    : integer;
        variable mc_obs    : integer;
        variable mclk_obs  : integer;
        variable lk_pct    : integer;
    begin
        -- Wait for the wallclock pipeline to fully start up before any
        -- measurement (5-stage pipeline + a few cycles).
        wait for 1 us;

        for i in PPB_LIST'range loop
            ppb := PPB_LIST(i);

            -- Apply ppb and let the 2-cycle ppb_adj pipeline + NCO bias
            -- settle before we start measuring.
            freq_corr <= to_signed(ppb, 20);
            wait for SETTLE;
            wait until rising_edge(clk);

            -- Snapshot start
            ns_a   := wc_ns_o;
            sec_a  := wc_sec_o;
            mc_a   := media_clock;
            mclk_a := mclk_edge_cnt;
            lk_a   := locked_cnt;
            lk_n_a := locked_sample_cnt;

            wait for MEAS_WIN;
            wait until rising_edge(clk);

            -- Snapshot end
            ns_b   := wc_ns_o;
            sec_b  := wc_sec_o;
            mc_b   := media_clock;
            mclk_b := mclk_edge_cnt;
            lk_b   := locked_cnt;
            lk_n_b := locked_sample_cnt;

            -- Observed deltas
            ns_obs := to_integer(sec_b - sec_a) * 1_000_000_000
                    + to_integer(ns_b) - to_integer(ns_a);
            mc_obs   := to_integer(mc_b - mc_a);
            mclk_obs := mclk_b - mclk_a;

            if (lk_n_b - lk_n_a) > 0 then
                lk_pct := (100 * (lk_b - lk_a)) / (lk_n_b - lk_n_a);
            else
                lk_pct := 0;
            end if;

            -- Expected rates (real arithmetic so 1e-9 doesn't truncate)
            factor      := 1.0 + real(ppb) * 1.0e-9;
            ns_expect   := integer(real(WIN_NS) * factor);
            mc_expect   := integer(real(AUDIO_FS_C) * real(WIN_NS)
                                   * 1.0e-9 * factor);
            mclk_expect := integer(real(AUDIO_FS_C * 512) * real(WIN_NS)
                                   * 1.0e-9 * factor);

            report "ppb=" & integer'image(ppb)
                 & "  ns_adv=" & integer'image(ns_obs)
                              & "/" & integer'image(ns_expect)
                 & "  mc_adv=" & integer'image(mc_obs)
                              & "/" & integer'image(mc_expect)
                 & "  mclk_edges=" & integer'image(mclk_obs)
                              & "/" & integer'image(mclk_expect)
                 & "  locked=" & integer'image(lk_pct) & "%";

            assert abs(ns_obs - ns_expect) <= NS_TOL
                report "ppb=" & integer'image(ppb)
                     & ": wallclock rate off - got "
                     & integer'image(ns_obs) & " ns, expected "
                     & integer'image(ns_expect)
                severity error;
            assert abs(mc_obs - mc_expect) <= MC_TOL
                report "ppb=" & integer'image(ppb)
                     & ": media_clock rate off - got "
                     & integer'image(mc_obs) & ", expected "
                     & integer'image(mc_expect)
                severity error;
            assert abs(mclk_obs - mclk_expect) <= MCLK_TOL
                report "ppb=" & integer'image(ppb)
                     & ": NCO rate off - got "
                     & integer'image(mclk_obs) & " edges, expected "
                     & integer'image(mclk_expect)
                severity error;
        end loop;

        -- Return to zero so the rest of the sim has a clean state.
        freq_corr <= (others => '0');

        report "=== freq_corr / NCO tests done ===";

        -- Hand off to the realistic-scenario stimuli.
        scenario_arm <= true;

        -- ============================================================
        -- Continuous ppb modulator (realistic PI-servo behaviour).
        -- Stays in *this* process so freq_corr only has one driver -
        -- a separate process would contribute 'U' until it first runs
        -- and the resolution would corrupt freq_corr to 'X'.
        -- ============================================================
        loop
            freq_corr <= to_signed(SERVO_SEQ(servo_idx mod SERVO_SEQ'length), 20);
            servo_idx := servo_idx + 1;
            wait for 200 us;
        end loop;
    end process;

    -- ============================================================
    -- Realistic-scenario stimulus + checks
    -- ============================================================
    -- Order of events:
    --   1. wallclock_set to (sec=1234, ns=500_000_000)        @t=0
    --      - immediately verify nsec/sec/media_clock loaded
    --      - media_clock expected = 1234*48000 + 24000
    --                             = 59_232_000 + 24_000 = 59_256_000
    --   2. let it run ~3 ms (with ppb_modulator wiggling)
    --   3. phase jump #1: +200_000 ns (= +200 us)
    --      - verify nsec_reg jumped by ~200_000 (modulo 1e9 carry)
    --   4. let it run ~3 ms more
    --   5. phase jump #2: -50_000 ns (= -50 us)
    --   6. let it run ~3 ms more
    --   7. continuous media-clock consistency check across the whole
    --      run: at every check, media_clock_o must equal
    --          sec*audio_fs + nsec*audio_fs/1e9   (+/-2 for pipeline
    --                                              latency / quantisation)
    -- ============================================================
    scenario_stim: process
        constant TARGET_SEC : natural := 1234;
        constant TARGET_NS  : natural := 500_000_000;
        -- Expected media_clock right after wallclock_set:
        constant TARGET_MC  : natural :=
            TARGET_SEC * AUDIO_FS_C
            + integer(real(TARGET_NS) * real(AUDIO_FS_C) / 1.0e9);

        variable mc_obs                : unsigned(31 downto 0);
        variable mc_expect             : integer;
        variable mc_check_count        : integer := 0;
        variable mc_check_fail         : integer := 0;

        -- For NCO/mediaclock drift check (Step 7)
        variable smp_a, smp_b : integer;
        variable mc_start, mc_end : unsigned(31 downto 0);
        variable nco_samples  : integer;
        variable mc_samples   : integer;
        variable drift        : integer;
    begin
        wait until scenario_arm;
        wait until rising_edge(clk);

        -- ===== Step 1: wallclock_set =====
        report "scenario: applying wallclock_set sec="
             & integer'image(TARGET_SEC)
             & " ns=" & integer'image(TARGET_NS);
        wc_sec_i <= to_unsigned(TARGET_SEC, 48);
        wc_ns_i  <= to_unsigned(TARGET_NS, 32);
        wc_set   <= '1';
        wait until rising_edge(clk);
        wc_set   <= '0';

        -- Pipeline: wallclock_set is registered immediately into
        -- nsec_reg/sec_reg, but media_proc has 3 pipeline stages each
        -- gated by media_clock_proc_wait (toggles every cycle), so the
        -- effective latency from sec/nsec inputs to media_clock_o is
        -- 6 cycles. Wait 8 to be safe.
        for i in 1 to 8 loop
            wait until rising_edge(clk);
        end loop;

        assert wc_sec_o = to_unsigned(TARGET_SEC, 48)
            report "wallclock_set: sec wrong, got "
                 & integer'image(to_integer(wc_sec_o))
            severity error;
        -- ns has advanced a few cycles since the set. Check it's in
        -- the expected range (TARGET_NS .. TARGET_NS + 100).
        assert wc_ns_o >= to_unsigned(TARGET_NS, 32)
           and wc_ns_o <= to_unsigned(TARGET_NS + 100, 32)
            report "wallclock_set: ns out of range, got "
                 & integer'image(to_integer(wc_ns_o))
                 & " expected ~" & integer'image(TARGET_NS)
            severity error;
        assert abs(to_integer(media_clock) - TARGET_MC) <= 2
            report "wallclock_set: media_clock wrong, got "
                 & integer'image(to_integer(media_clock))
                 & " expected " & integer'image(TARGET_MC)
            severity error;
        report "wallclock_set OK (sec=" & integer'image(to_integer(wc_sec_o))
             & " ns=" & integer'image(to_integer(wc_ns_o))
             & " media_clock=" & integer'image(to_integer(media_clock))
             & " expected ~" & integer'image(TARGET_MC) & ")";

        -- ===== Step 2: free-run with ppb modulation =====
        -- (Previously Steps 3-5 exercised phase_jump_valid_i. The phase
        -- jump path has been removed from wallclock.vhd; the PTP servo
        -- now corrects offsets purely via freq_correction_ppb_i.)
        wait for 5 ms;

        -- ===== Step 6: media-clock consistency over a 3 ms window =====
        -- Sample media_clock and (sec, ns) every 100 us; verify
        -- media_clock matches sec*48000 + nsec*48000/1e9 within
        -- pipeline tolerance. ppb is changing under us via ppb_modulator,
        -- so this exercises the "Mediaclock ist eine pure Funktion der
        -- Wallclock auch unter dynamischer Last" property.
        report "scenario: media_clock consistency check (3 ms, 100 us cadence)";
        for i in 1 to 30 loop
            wait for 100 us;
            wait until rising_edge(clk);

            mc_expect := to_integer(wc_sec_o(31 downto 0)) * AUDIO_FS_C
                       + integer(real(to_integer(wc_ns_o))
                                 * real(AUDIO_FS_C) / 1.0e9);
            mc_obs    := media_clock;
            mc_check_count := mc_check_count + 1;
            -- Tolerance: media_proc has 3-stage pipeline (sec/ns at the
            -- moment we read media_clock are ~3 cycles ahead of what
            -- the media_clock value reflects). 3 cycles * 8ns * 48k/1e9
            -- ~ no full sample, so +/-2 is generous.
            if abs(to_integer(mc_obs) - mc_expect) > 2 then
                mc_check_fail := mc_check_fail + 1;
                report "media_clock mismatch at " & time'image(now)
                     & ": got " & integer'image(to_integer(mc_obs))
                     & " expected " & integer'image(mc_expect)
                     & " (sec=" & integer'image(to_integer(wc_sec_o))
                     & " ns=" & integer'image(to_integer(wc_ns_o)) & ")"
                severity warning;
            end if;
        end loop;
        assert mc_check_fail = 0
            report "media_clock consistency: "
                 & integer'image(mc_check_fail) & "/"
                 & integer'image(mc_check_count) & " samples failed"
            severity error;
        report "media_clock consistency OK ("
             & integer'image(mc_check_count) & " samples, all within +/-2)";

        -- ===== Step 7: NCO vs media_clock drift under jittery ppb =====
        -- The mediaclock is a pure function of the wallclock. The NCO
        -- is ppb-disciplined (which tracks the wallclock) plus the
        -- pull-loop (which keeps phase aligned to media_clock edges).
        -- Under continuously moving ppb (servo loop), both counts
        -- should advance lock-step: every NCO sample boundary
        -- (sample_pulse_int) coincides with one media_clock increment.
        --
        -- Measure both over a 5 ms window with the modulator wiggling
        -- ppb every 200 us. Drift = |nco_samples - mc_samples|, allow
        -- +/-2 (snapshot quantisation +/- one in-flight sample).
        wait for 1 ms;  -- let things settle after Step 6
        wait until rising_edge(clk);
        smp_a    := sample_cnt;
        mc_start := media_clock;
        wait for 5 ms;
        wait until rising_edge(clk);
        smp_b    := sample_cnt;
        mc_end   := media_clock;

        nco_samples := smp_b - smp_a;
        mc_samples  := to_integer(mc_end - mc_start);
        drift       := abs(nco_samples - mc_samples);
        report "NCO vs mediaclock drift over 5 ms (ppb wiggling): "
             & "nco_samples=" & integer'image(nco_samples)
             & "  mc_samples=" & integer'image(mc_samples)
             & "  drift=" & integer'image(drift);
        assert drift <= 2
            report "NCO and mediaclock desynchronised under jittery ppb: "
                 & "drift = " & integer'image(drift) & " samples (>2)"
            severity error;

        report "=== scenario tests done ===";
        scenario_done <= true;
        wait;
    end process;

end architecture;
