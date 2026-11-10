-- External-PLL-MCLK testbench for wallclock.vhd (MCLK_FROM_VCXO +
-- VCXO_EXT_PLL, e.g. a CS2000 as jitter cleaner)
--
-- The NCO MCLK leaves on vcxo_pump_o.pll_ref; a behavioural PLL (second
-- order, ~5 kHz bandwidth -- TB-scaled, a CS2000 runs 1..128 Hz) locks
-- to it with a static phase offset PLL_OFS_CYC and feeds its clean clock
-- back on vcxo_clk_i. The wallclock must align the MCLK counter (and
-- with DOMAIN the VCXO-clocked pin clocks) to the wallclock sample grid
-- without steering anything.
--
-- Sequence (each phase ends in a PASS/FAIL check):
--   1. Cold start: PLL starts 50 ppm off, wallclock_set, acquisition.
--   2. wallclock_set +400_000 ns: NCO restarts, PLL slews, counter
--      realigns.
--   3. PLL loses lock (free-runs +500 ppm for 400 us, ~5 MCLK of drift)
--      and re-locks with a different phase offset: counter re-snaps.
--   4. Stepped freq_correction_ppb (servo updates) while locked: lock
--      must hold and the sample grid must not jump.
-- Locked checks: lock flag held, exactly 512 PLL edges per sample, PLL
-- sample pulse within 1.5 MCLK + 1 clk of the NCO grid (the PLL follows
-- the NCO; the counter snaps to the wallclock grid, the NCO wanders
-- +-1 MCLK around it). With DOMAIN additionally: 512 edges per pin LRCK
-- period and internal LRCK 1..3 clk AHEAD of the pin LRCK.
--
-- Run (~14 ms sim time, ~10 minutes):
--   ghdl -a --std=08 packages/audioclks_pkg.vhd \
--        packages/wallclock_signals_pkg.vhd ptp/average.vhd \
--        ptp/wallclock_vcxo_pkg.vhd ptp/wallclock_time.vhd \
--        ptp/wallclock_media_clock.vhd ptp/wallclock_nco.vhd \
--        ptp/wallclock_clk_div.vhd ptp/wallclock_vcxo_pd.vhd \
--        ptp/wallclock_vcxo_ctl.vhd ptp/wallclock_vcxo_pump.vhd \
--        ptp/wallclock_extpll_mon.vhd ptp/wallclock_vcxo_domain.vhd \
--        ptp/wallclock_vcxo.vhd ptp/wallclock.vhd \
--        ptp/wallclock_extpll_tb.vhd
--   ghdl -e --std=08 wallclock_extpll_tb
--   ghdl -r --std=08 wallclock_extpll_tb [-gDOMAIN=false] \
--        [-gSYS_CLK_HZ=125000000] > extpll.csv
--
-- CSV rows (one per PD window): time_us,phase_id,err_cyc,locked,pll_ppm

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;

use work.audioclks_pkg.all;
use work.wallclock_signals_pkg.all;

entity wallclock_extpll_tb is
    generic (
        DOMAIN      : boolean := true;
        -- sys_clk; must have an integer period in ns
        SYS_CLK_HZ  : natural := 100_000_000;
        -- PLL output phase vs. its reference, MCLK cycles
        PLL_OFS_CYC : real := 0.37
    );
end entity;

architecture sim of wallclock_extpll_tb is

    constant AUDIO_FS_C : natural := 48_000;
    constant CLK_NS     : natural := 1_000_000_000 / SYS_CLK_HZ;
    constant CLK_PERIOD : time    := CLK_NS * 1 ns;
    constant CLK_PS     : integer := CLK_NS * 1000;
    constant F_NOM      : real    := real(AUDIO_FS_C * 512);
    constant T_NOM_FS   : real    := 1.0e15 / F_NOM;

    -- PLL model: per reference edge, u = KP*e + I, I += KI*e (e in cycles)
    constant PLL_BW     : real := 5.0e3;
    constant KP         : real := 2.0 * MATH_PI * PLL_BW / F_NOM;
    constant KI         : real := KP * KP / 4.0;

    -- PLL vs NCO sample pulse limit [clk]
    constant SKEW_LIMIT : integer :=
        integer(ceil(1.5 * real(SYS_CLK_HZ) / F_NOM)) + 1;

    signal clk     : std_logic := '0';
    signal reset_n : std_logic := '0';
    signal wc      : t_wallclock_signals;

    signal audio_clocks : t_audio_clocks;
    signal pin_clocks   : t_audio_clocks;
    signal sample_pulse : std_logic;   -- PLL grid
    signal nco_spulse   : std_logic;   -- NCO grid
    signal locked       : std_logic;
    signal pump         : t_vcxo_pump;
    signal err_dbg      : signed(31 downto 0);

    -- PLL model
    signal pll_clk      : std_logic := '0';
    signal pll_u        : real := 0.0;       -- relative frequency increase
    signal pll_ofs      : real := PLL_OFS_CYC;
    signal pll_freerun  : boolean := false;
    signal freerun_ppm  : real := 0.0;
    signal t_out_last   : time := 0 ns;

    signal sim_done : boolean := false;
    signal phase_id : integer := 0;

    -- monitors (cleared by mon_clear)
    signal mon_clear     : boolean := false;
    signal mon_bad_cnt   : natural := 0;
    signal mon_samples   : natural := 0;
    signal mon_max_skew  : integer := 0;
    signal mon_skew_sum  : integer := 0;
    signal mon_skew_n    : natural := 0;
    signal mon_unlocked  : boolean := false;
    signal mon_pin_bad   : natural := 0;
    signal mon_pin_n     : natural := 0;
    signal mon_lag_min   : integer := 1000;
    signal mon_lag_max   : integer := -1000;

    function err_cycles(x : signed) return real is
    begin
        return real(to_integer(x)) / 65536.0;
    end function;
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

    wc.wallclock_do_phasejump_i <= '0';
    wc.nco_ppb_adj_i            <= (others => '0');
    wc.nco_ppb_adj_valid_i      <= '0';

    dut: entity work.wallclock
        generic map (
            increment_interval   => CLK_NS,
            sys_clk_hz           => SYS_CLK_HZ,
            audio_fs             => AUDIO_FS_C,
            pull_loop_gain_shift => 6,     -- TB-scaled NCO pull loop, as
            pull_int_gain_shift  => 27,    -- in wallclock_phasepull_tb
            MCLK_FROM_VCXO       => true,
            VCXO_EXT_PLL         => true,
            VCXO_PD_AVG_LOG2     => 8,
            VCXO_DOMAIN_CLOCKS   => DOMAIN
        )
        port map (
            clk                  => clk,
            reset_n              => reset_n,
            wallclock_signals    => wc,
            second_pulse_o       => open,
            media_clock_o        => open,
            audio_mclk_o         => open,
            sample_pulse_o       => sample_pulse,
            ms_pulse_o           => open,
            clocks_o             => audio_clocks,
            phase_locked_o       => open,
            mclk_cnt_o           => open,
            media_edge_tick_o    => open,
            ppb_adj_dbg_o        => open,
            ppb_trim_dbg_o       => open,
            bias_dbg_o           => open,
            sample_pulse_int_o   => nco_spulse,
            nco_phase_dbg_o      => open,
            nco_inc_dbg_o        => open,
            vcxo_clk_i           => pll_clk,
            vcxo_pump_o          => pump,
            vcxo_locked_o        => locked,
            vcxo_phase_err_dbg_o => err_dbg,
            vcxo_pump_dbg_o      => open,
            pin_clocks_o         => pin_clocks
        );

    -- ============================================================
    -- PLL model: phase detector on the reference edges ...
    -- ============================================================
    pll_pd: process
        variable integ : real := 50.0e-6;   -- crystal 50 ppm off at start
        variable e     : real;
    begin
        wait until rising_edge(pump.pll_ref) or sim_done;
        while not sim_done loop
            if not pll_freerun then
                -- output edge vs. reference edge + offset, wrapped to
                -- +-1/2 cycle; > 0 = output late
                e := real((t_out_last - now) / 1 fs) / T_NOM_FS - pll_ofs;
                e := e - round(e);
                integ := integ + KI * e;
                pll_u <= KP * e + integ;
            else
                pll_u <= freerun_ppm * 1.0e-6;
            end if;
            wait until rising_edge(pump.pll_ref) or sim_done;
        end loop;
        wait;
    end process;

    -- ... and the output oscillator
    pll_osc: process
        variable hp : time;
    begin
        while not sim_done loop
            hp := (0.5 * T_NOM_FS * (1.0 - pll_u)) * 1 fs;
            pll_clk    <= '1';
            t_out_last <= now;
            wait for hp;
            pll_clk <= '0';
            wait for hp;
        end loop;
        wait;
    end process;

    -- ============================================================
    -- Monitors
    -- ============================================================
    -- PLL edges per sample (512 without grid jumps), skew vs. NCO grid
    edge_mon: process(pll_clk, clk)
        variable edges  : natural := 0;
        variable t_nco  : time := 0 ns;
        variable t_pll  : time := 0 ns;
        variable skew   : integer;
    begin
        if rising_edge(clk) then
            if mon_clear then
                if sample_pulse = '1' then
                    edges := 0;
                end if;
                mon_bad_cnt  <= 0;
                mon_samples  <= 0;
                mon_max_skew <= 0;
                mon_skew_sum <= 0;
                mon_skew_n   <= 0;
                mon_unlocked <= false;
            else
                if locked /= '1' then
                    mon_unlocked <= true;
                end if;
                if nco_spulse = '1' then
                    t_nco := now;
                end if;
                if sample_pulse = '1' then
                    if edges /= 512 then
                        mon_bad_cnt <= mon_bad_cnt + 1;
                        report "sample period with " & integer'image(edges) & " edges";
                    end if;
                    edges := 0;
                    t_pll := now;
                    mon_samples <= mon_samples + 1;
                end if;
                if (sample_pulse = '1' or nco_spulse = '1') and t_nco > 0 ns and t_pll > 0 ns then
                    skew := (t_pll - t_nco) / CLK_PERIOD;
                    if abs(skew) < 256 then
                        if abs(skew) > mon_max_skew then
                            mon_max_skew <= abs(skew);
                        end if;
                        mon_skew_sum <= mon_skew_sum + skew;
                        mon_skew_n   <= mon_skew_n + 1;
                    end if;
                end if;
            end if;
        end if;
        if rising_edge(pll_clk) then
            edges := edges + 1;
        end if;
    end process;

    -- Pin LRCK vs. PLL edges and the internal LRCK (DOMAIN)
    pin_mon: process(pll_clk, pin_clocks.fsclk_50, audio_clocks.fsclk_50)
        variable edges : natural := 0;
        variable t_pin : time := 0 ns;
        variable t_int : time := 0 ns;
        variable d     : integer;
    begin
        if rising_edge(pll_clk) then
            edges := edges + 1;
        end if;
        if mon_clear then
            if rising_edge(pin_clocks.fsclk_50) then
                edges := 0;     -- keep the period count aligned
            end if;
            mon_pin_bad <= 0;
            mon_pin_n   <= 0;
            mon_lag_min <= 1000;
            mon_lag_max <= -1000;
        else
            if rising_edge(pin_clocks.fsclk_50) then
                if edges /= 512 then
                    mon_pin_bad <= mon_pin_bad + 1;
                    report "pin LRCK period with " & integer'image(edges) & " edges";
                end if;
                edges := 0;
                t_pin := now;
                mon_pin_n <= mon_pin_n + 1;
            end if;
            -- lag = internal - pin LRCK edge, nearest pairing
            if rising_edge(audio_clocks.fsclk_50) then
                t_int := now;
                if t_pin > 0 ns then
                    d := ((now - t_pin) / 1 ps + CLK_PS / 2) / CLK_PS;
                    if d < 256 then
                        if d < mon_lag_min then mon_lag_min <= d; end if;
                        if d > mon_lag_max then mon_lag_max <= d; end if;
                    end if;
                end if;
            end if;
            if rising_edge(pin_clocks.fsclk_50) and t_int > 0 ns then
                d := -(((now - t_int) / 1 ps + CLK_PS / 2) / CLK_PS);
                if d > -256 then
                    if d < mon_lag_min then mon_lag_min <= d; end if;
                    if d > mon_lag_max then mon_lag_max <= d; end if;
                end if;
            end if;
        end if;
    end process;

    -- one CSV row per PD window (err_dbg changes then)
    win_mon: process
    begin
        report "time_us,phase_id,err_cyc,locked,pll_ppm";
        while not sim_done loop
            wait on err_dbg, sim_done;
            wait until rising_edge(clk);
            report real'image(real(now / 1 ns) / 1000.0)
                 & "," & integer'image(phase_id)
                 & "," & real'image(err_cycles(err_dbg))
                 & "," & std_logic'image(locked)
                 & "," & real'image(pll_u * 1.0e6);
        end loop;
        wait;
    end process;

    -- ============================================================
    -- Stimulus + checks
    -- ============================================================
    stim: process
        variable errors : natural := 0;

        procedure check_locked(obs_time : time;
                               msg      : string;
                               variable errs : inout natural) is
        begin
            mon_clear <= true;
            wait for 4 * CLK_PERIOD;
            mon_clear <= false;
            wait for obs_time;
            if DOMAIN then
                if mon_pin_bad /= 0 or mon_pin_n < 10 or mon_lag_min < -3 or mon_lag_max > -1 then
                    report "CHECK FAIL: " & msg & " [pin clocks]"
                         & " bad_periods=" & integer'image(mon_pin_bad)
                         & " periods=" & integer'image(mon_pin_n)
                         & " int_lag=" & integer'image(mon_lag_min) & ".."
                         & integer'image(mon_lag_max) & " clk"
                         severity error;
                    errs := errs + 1;
                else
                    report "CHECK PASS: " & msg & " [pin clocks]"
                         & " periods=" & integer'image(mon_pin_n)
                         & " int_lag=" & integer'image(mon_lag_min) & ".."
                         & integer'image(mon_lag_max) & " clk";
                end if;
            end if;
            if mon_unlocked or mon_bad_cnt /= 0 or mon_max_skew > SKEW_LIMIT
               or mon_samples < 10 then
                report "CHECK FAIL: " & msg
                     & " -- unlocked=" & boolean'image(mon_unlocked)
                     & " bad_samples=" & integer'image(mon_bad_cnt)
                     & " samples=" & integer'image(mon_samples)
                     & " max_skew=" & integer'image(mon_max_skew) & " clk (limit "
                     & integer'image(SKEW_LIMIT) & ")"
                     severity error;
                errs := errs + 1;
            else
                report "CHECK PASS: " & msg
                     & " -- samples=" & integer'image(mon_samples)
                     & ", max_skew=" & integer'image(mon_max_skew) & " clk"
                     & ", mean_skew=" & real'image(real(mon_skew_sum) / real(mon_skew_n)) & " clk"
                     & ", err=" & real'image(err_cycles(err_dbg)) & " cyc";
            end if;
        end procedure;

        procedure do_set(ns : natural) is
        begin
            wait until rising_edge(clk);
            wc.wallclock_seconds_i     <= std_logic_vector(to_unsigned(1234, 48));
            wc.wallclock_nanoseconds_i <= std_logic_vector(to_unsigned(ns, 30));
            wc.wallclock_set_i         <= '1';
            wait for 10 * CLK_PERIOD;
            wc.wallclock_set_i         <= '0';
        end procedure;
    begin
        report "=== external-PLL TB start (sys_clk " & integer'image(SYS_CLK_HZ)
             & " Hz, DOMAIN=" & boolean'image(DOMAIN) & ") ===";
        wc.wallclock_set_i         <= '0';
        wc.wallclock_seconds_i     <= (others => '0');
        wc.wallclock_nanoseconds_i <= (others => '0');
        wc.freq_correction_ppb_i   <= (others => '0');

        wait for 20 * CLK_PERIOD;
        reset_n <= '1';
        wait for 50 us;

        -- 1. cold start
        phase_id <= 1;
        do_set(100_000_000);
        wait for 3000 us;
        check_locked(500 us, "phase 1: cold start, PLL acquisition + grid alignment", errors);

        -- 2. wallclock set: NCO restarts, the PLL follows
        phase_id <= 2;
        do_set(100_400_000 + 7_777);
        wait for 4000 us;   -- the NCO (and with it the PLL) needs ~4 ms
        check_locked(500 us, "phase 2: realign after wallclock_set", errors);

        -- 3. PLL loses lock, drifts ~5 MCLK, re-locks at another phase
        phase_id <= 3;
        freerun_ppm <= 500.0;
        pll_freerun <= true;
        wait for 400 us;
        pll_ofs     <= -0.42;
        pll_freerun <= false;
        wait for 2000 us;
        check_locked(500 us, "phase 3: re-snap after PLL re-lock", errors);

        -- 4. servo-like ppb steps while locked: no unlock, no grid jump
        phase_id <= 4;
        mon_clear <= true;
        wait for 4 * CLK_PERIOD;
        mon_clear <= false;
        for k in 1 to 8 loop
            wc.freq_correction_ppb_i <= to_signed(((k mod 3) - 1) * 4000 + k * 500, 20);
            wait for 250 us;
        end loop;
        if mon_unlocked or mon_bad_cnt /= 0 or mon_max_skew > SKEW_LIMIT then
            report "CHECK FAIL: phase 4: stepped ppb -- unlocked=" & boolean'image(mon_unlocked)
                 & " bad_samples=" & integer'image(mon_bad_cnt)
                 & " max_skew=" & integer'image(mon_max_skew) & " clk"
                 severity error;
            errors := errors + 1;
        else
            report "CHECK PASS: phase 4: stepped ppb -- samples=" & integer'image(mon_samples)
                 & ", max_skew=" & integer'image(mon_max_skew) & " clk";
        end if;

        if errors = 0 then
            report "=== external-PLL TB PASSED ===";
        else
            report "=== external-PLL TB FAILED: " & integer'image(errors) & " check(s) ===";
        end if;
        sim_done <= true;
        wait;
    end process;

end architecture;
