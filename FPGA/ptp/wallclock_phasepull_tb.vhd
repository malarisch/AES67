-- Phase-pull-loop testbench for wallclock.vhd
--
-- Scope: isolate the NCO phase-pull PI loop, verify it locks without a
-- limit cycle, and dump its internal state per media_edge_tick as CSV.
--
-- The synthesis loop gains (LG=10 / INT=16) give ~6 Hz loop bandwidth,
-- i.e. ~200 ms settling -- far too slow to simulate. The TB therefore
-- overrides the gain generics with LG=6 / INT=27. That scales both
-- gains by the same argument (see the loop-analysis comment in
-- wallclock.vhd): damping stays ~1, the dynamics just run ~16x faster,
-- so acquisition + settling fit in a few ms of sim time. The loop
-- STRUCTURE under test (held P bias, pipelined I clamp, no control-path
-- dead band) is identical to synthesis.
--
-- Sequence (each phase ends in a PASS/FAIL check):
--   1. Reset, short free-run, wallclock_set to a mid-second epoch
--      (hard-resets the NCO -> acquisition from a large phase error).
--   2. wallclock_set with +400_000 ns (~19.2 sample periods, lands
--      mid-sample) -> second acquisition.
--   3. Constant freq_correction_ppb (+10 ppm): feed-forward hits NCO
--      and wallclock identically, the loop must stay undisturbed.
--   4. Stepped ppb every 400 us mimicking discrete servo updates.
--   5. wallclock_set to 997.5 ms: nsec > 2^29 exercises the unsigned
--      set path (the old sign-extension loaded nsec - 2^30), and the
--      second rollover ~2.5 ms later lands inside the observation
--      window to verify the coherent (sec, nsec) media_clock snapshot
--      (the old skew glitched media_clock by +/-48000 at each rollover
--      and kicked bogus full-scale phase errors into the pull loop).
-- After each settle wait, max |phase_err| is monitored over an
-- observation window -- a limit cycle (the old INT=30 behaviour) fails
-- these checks; quantisation noise of +/-1..2 ticks passes.
--
-- Run (fast, ~13 ms sim time):
--   cd <workdir>
--   ghdl -a --std=08 packages/audioclks_pkg.vhd \
--        packages/wallclock_signals_pkg.vhd ptp/wallclock.vhd \
--        ptp/wallclock_phasepull_tb.vhd
--   ghdl -e --std=08 wallclock_phasepull_tb
--   ghdl -r --std=08 wallclock_phasepull_tb > phasepull.csv
--
-- CSV rows: time_us,kind,phase_id,mclk_cnt,phase_err,ppb_adj,ppb_trim,
--           bias,nco_phase_hi,nco_inc_hi   (kind M = media edge,
--           N = NCO sample pulse). Plot e.g. phase_err over time_us.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.audioclks_pkg.all;
use work.wallclock_signals_pkg.all;

entity wallclock_phasepull_tb is
end entity;

architecture sim of wallclock_phasepull_tb is

    constant SYS_CLK_HZ_C        : natural := 125_000_000;
    constant AUDIO_FS_C          : natural := 48_000;
    constant INCREMENT_INTERVAL_C: natural := 8;

    -- TB-scaled loop gains (see header comment).
    constant TB_LOOP_GAIN_SHIFT  : natural := 6;
    constant TB_INT_GAIN_SHIFT   : natural := 27;

    constant CLK_PERIOD : time := 8 ns;

    signal clk     : std_logic := '0';
    signal reset_n : std_logic := '0';

    signal wc : t_wallclock_signals;

    signal second_pulse : std_logic;
    signal audio_mclk   : std_logic;
    signal media_clock  : unsigned(31 downto 0);
    signal sample_pulse : std_logic;
    signal ms_pulse     : std_logic;
    signal audio_clocks : t_audio_clocks;
    signal phase_locked : std_logic;

    signal mclk_cnt_dbg     : unsigned(8 downto 0);
    signal media_edge_dbg   : std_logic;
    signal ppb_adj_dbg      : signed(31 downto 0);
    signal ppb_trim_dbg     : signed(31 downto 0);
    signal bias_dbg         : signed(31 downto 0);
    signal sample_pulse_int : std_logic;
    signal nco_phase_dbg    : unsigned(31 downto 0);
    signal nco_inc_dbg      : signed(31 downto 0);

    signal sim_done : boolean := false;

    -- Test phase indicator (mirrored into the CSV so post-processing can
    -- partition the trace: 0 = warmup, 1 = post wallclock_set,
    -- 2 = post offset-set, 3 = constant-ppb, 4 = stepped-ppb).
    signal phase_id : integer := 0;

    -- Phase-error monitor: mon_phase_err is the latest per-edge error,
    -- mon_max_abs the maximum |error| since the last mon_clear pulse.
    signal mon_phase_err : integer := 0;
    signal mon_max_abs   : integer := 0;
    signal mon_clear     : boolean := false;

begin

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

    -- Unused wallclock control input.
    wc.wallclock_do_phasejump_i <= '0';

    dut: entity work.wallclock
        generic map (
            increment_interval   => INCREMENT_INTERVAL_C,
            sys_clk_hz           => SYS_CLK_HZ_C,
            audio_fs             => AUDIO_FS_C,
            pull_loop_gain_shift => TB_LOOP_GAIN_SHIFT,
            pull_int_gain_shift  => TB_INT_GAIN_SHIFT
        )
        port map (
            clk                => clk,
            reset_n            => reset_n,
            wallclock_signals  => wc,
            second_pulse_o     => second_pulse,
            media_clock_o      => media_clock,
            audio_mclk_o       => audio_mclk,
            sample_pulse_o     => sample_pulse,
            ms_pulse_o         => ms_pulse,
            clocks_o           => audio_clocks,
            phase_locked_o     => phase_locked,
            mclk_cnt_o         => mclk_cnt_dbg,
            media_edge_tick_o  => media_edge_dbg,
            ppb_adj_dbg_o      => ppb_adj_dbg,
            ppb_trim_dbg_o     => ppb_trim_dbg,
            bias_dbg_o         => bias_dbg,
            sample_pulse_int_o => sample_pulse_int,
            nco_phase_dbg_o    => nco_phase_dbg,
            nco_inc_dbg_o      => nco_inc_dbg
        );

    -- ============================================================
    -- Phase-error monitor (same mclk_cnt -> phase_err mapping as the
    -- pull loop itself).
    -- ============================================================
    mon: process(clk)
        variable e : integer;
    begin
        if rising_edge(clk) then
            if mon_clear then
                mon_max_abs <= 0;
            elsif media_edge_dbg = '1' then
                if to_integer(mclk_cnt_dbg) <= 255 then
                    e := -to_integer(mclk_cnt_dbg);
                else
                    e := 512 - to_integer(mclk_cnt_dbg);
                end if;
                mon_phase_err <= e;
                if abs(e) > mon_max_abs then
                    mon_max_abs <= abs(e);
                end if;
            end if;
        end if;
    end process;

    -- ============================================================
    -- CSV logger: one row per media_edge_tick (M) and per
    -- sample_pulse_int (N). The DBG signals are registered outputs --
    -- they reflect what the loop computed on the previous clock edge,
    -- so each event is logged one cycle late, when the loop's response
    -- to that edge is visible.
    -- ============================================================
    logger: process
        variable phase_err : integer;
        variable t_us      : real;
        variable kind      : string(1 to 1);
        variable saw_edge  : boolean;
        variable saw_kind  : string(1 to 1);
        variable saw_mc    : integer;
    begin
        report "time_us,kind,phase_id,mclk_cnt,phase_err,ppb_adj,ppb_trim,bias,nco_phase_hi,nco_inc_hi";
        saw_edge := false;
        wait until sim_done = true or rising_edge(clk);
        while not sim_done loop
            if saw_edge then
                t_us := real(now / 1 ns) / 1000.0;
                if saw_mc <= 255 then
                    phase_err := -saw_mc;
                else
                    phase_err := 512 - saw_mc;
                end if;
                report real'image(t_us)
                     & "," & saw_kind
                     & "," & integer'image(phase_id)
                     & "," & integer'image(saw_mc)
                     & "," & integer'image(phase_err)
                     & "," & integer'image(to_integer(ppb_adj_dbg))
                     & "," & integer'image(to_integer(ppb_trim_dbg))
                     & "," & integer'image(to_integer(bias_dbg))
                     & "," & integer'image(to_integer(signed('0' & nco_phase_dbg(31 downto 1))))
                     & "," & integer'image(to_integer(nco_inc_dbg));
                saw_edge := false;
            end if;
            kind := " ";
            if media_edge_dbg = '1' then
                kind := "M";
            elsif sample_pulse_int = '1' then
                kind := "N";
            end if;
            if kind /= " " then
                saw_edge := true;
                saw_kind := kind;
                saw_mc   := to_integer(mclk_cnt_dbg);
            end if;
            wait until sim_done = true or rising_edge(clk);
        end loop;
        wait;
    end process;

    -- ============================================================
    -- Stimulus + checks
    -- ============================================================
    stim: process
        variable errors : natural := 0;

        -- Clear the max-|phase_err| monitor, observe for obs_time, then
        -- require the maximum stayed at or below limit_ticks.
        procedure observe_and_check(obs_time    : time;
                                    limit_ticks : integer;
                                    msg         : string;
                                    variable errs : inout natural) is
        begin
            mon_clear <= true;
            wait for 4 * CLK_PERIOD;
            mon_clear <= false;
            wait for obs_time;
            if mon_max_abs > limit_ticks then
                report "CHECK FAIL: " & msg
                     & " -- max |phase_err| = " & integer'image(mon_max_abs)
                     & " ticks (limit " & integer'image(limit_ticks) & ")"
                     severity error;
                errs := errs + 1;
            else
                report "CHECK PASS: " & msg
                     & " -- max |phase_err| = " & integer'image(mon_max_abs)
                     & " ticks (limit " & integer'image(limit_ticks) & ")";
            end if;
        end procedure;
    begin
        report "=== phase-pull TB start ===";

        wc.wallclock_set_i         <= '0';
        wc.wallclock_seconds_i     <= (others => '0');
        wc.wallclock_nanoseconds_i <= (others => '0');
        wc.freq_correction_ppb_i   <= (others => '0');

        reset_n <= '0';
        wait for 200 ns;
        reset_n <= '1';
        wait for 100 us;

        -- ----- Phase 1: wallclock_set to a realistic mid-second value.
        -- Hard-resets nco_phase/mclk_cnt -> acquisition from up to a
        -- half-sample phase error.
        report "--- wallclock_set (acquisition) ---";
        wc.wallclock_seconds_i     <= std_logic_vector(to_unsigned(1234, 48));
        wc.wallclock_nanoseconds_i <= std_logic_vector(to_unsigned(500_000_000, 30));
        wc.wallclock_set_i         <= '1';
        wait until rising_edge(clk);
        wc.wallclock_set_i         <= '0';
        phase_id <= 1;

        wait for 3 ms;   -- settle (~3x slowest time constant at TB gains)
        observe_and_check(1 ms, 3, "phase 1: locked after wallclock_set", errors);
        if phase_locked /= '1' then
            report "CHECK FAIL: phase 1: phase_locked_o not asserted"
                severity error;
            errors := errors + 1;
        end if;

        -- ----- Phase 2: re-set with +400_000 ns (~19.2 sample periods,
        -- deliberately not a multiple -> pull loop restarts mid-sample).
        report "--- wallclock_set with +400_000 ns offset ---";
        wc.wallclock_nanoseconds_i <= std_logic_vector(to_unsigned(500_400_000, 30));
        wc.wallclock_set_i         <= '1';
        wait until rising_edge(clk);
        wc.wallclock_set_i         <= '0';
        phase_id <= 2;

        wait for 3 ms;
        observe_and_check(1 ms, 3, "phase 2: re-locked after offset set", errors);

        -- ----- Phase 3: constant ppb (residual XO drift as the PTP
        -- servo would command it). Feed-forward reaches NCO and
        -- wallclock identically, so the loop must not be disturbed.
        report "--- constant ppb = +10000 ---";
        wc.freq_correction_ppb_i <= to_signed(10_000, 20);
        phase_id <= 3;
        wait for 1 ms;
        observe_and_check(1 ms, 3, "phase 3: undisturbed by constant ppb", errors);

        -- ----- Phase 4: step ppb every 400 us to mimic discrete servo
        -- updates (compressed version of the 0.7 s sawtooth cadence
        -- observed on real HW).
        report "--- stepped ppb (servo-like updates) ---";
        phase_id <= 4;
        for k in 0 to 5 loop
            case k mod 6 is
                when 0 => wc.freq_correction_ppb_i <= to_signed( 8_000, 20);
                when 1 => wc.freq_correction_ppb_i <= to_signed(12_000, 20);
                when 2 => wc.freq_correction_ppb_i <= to_signed( 9_500, 20);
                when 3 => wc.freq_correction_ppb_i <= to_signed(11_000, 20);
                when 4 => wc.freq_correction_ppb_i <= to_signed(10_500, 20);
                when 5 => wc.freq_correction_ppb_i <= to_signed( 9_800, 20);
                when others => null;
            end case;
            wait for 400 us;
        end loop;
        observe_and_check(1 ms, 3, "phase 4: tracks stepped ppb", errors);

        -- ----- Phase 5: set to 997.5 ms into the second.
        --  * nsec > 2^29 exercises the unsigned interpretation of the
        --    set path (the old sign-extension loaded nsec - 2^30 and
        --    the clock settled 73.7 ms + 1 s off).
        --  * The second rollover ~2.5 ms later falls inside the
        --    observation window: the coherent (sec, nsec) media_clock
        --    snapshot must not glitch (the old skew produced bogus
        --    full-scale phase errors twice per rollover).
        report "--- wallclock_set to 997.5 ms (rollover in window) ---";
        wc.wallclock_seconds_i     <= std_logic_vector(to_unsigned(1234, 48));
        wc.wallclock_nanoseconds_i <= std_logic_vector(to_unsigned(997_500_000, 30));
        wc.wallclock_set_i         <= '1';
        wait until rising_edge(clk);
        wc.wallclock_set_i         <= '0';
        phase_id <= 5;

        wait for 2.3 ms;
        observe_and_check(1.7 ms, 3, "phase 5: unsigned set + second rollover", errors);
        if wc.wallclock_seconds_o /= to_unsigned(1235, 48) then
            report "CHECK FAIL: phase 5: seconds_o /= 1235 after rollover"
                severity error;
            errors := errors + 1;
        else
            report "CHECK PASS: phase 5: seconds rolled over to 1235";
        end if;

        if errors = 0 then
            report "=== phase-pull TB PASSED ===";
        else
            report "=== phase-pull TB FAILED: "
                 & integer'image(errors) & " check(s) ===" severity error;
        end if;
        sim_done <= true;
        wait for 5 * CLK_PERIOD;
        assert errors = 0 severity failure;
        std.env.stop;
    end process;

end architecture;
