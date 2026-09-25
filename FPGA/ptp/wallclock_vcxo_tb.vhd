-- VCXO-MCLK testbench for wallclock.vhd (MCLK_FROM_VCXO = true)
--
-- Models the board-side analog loop and checks that the charge-pump PI
-- loop locks the VCXO to the NCO:
--
--   pd_a --[10k]--+                       VCXO CV = V(node)
--   pd_b --[100R]-+-- node --[5R1]--[C]-- GND
--                            (constant leakage current, optional)
--
--   f_vcxo = 24.576 MHz * (1 + (OFFSET_PPM + KV_PPM_PER_V * (V - 1.65)) * 1e-6)
--   clamped to +/-PULL_PPM
--
-- Time scaling: the real loop (C = 470 uF, 1.33 ms windows, ~0.2 Hz
-- bandwidth) would take minutes of simulated time. The TB shrinks C to
-- 1.6 nF and the PD window to 2^8 edges and re-derives the gains with
-- the same formulas as the header of gen_mclk_vcxo:
--   Kf (pd_a, mid-rail) = 165 uA * 8 ns / 1.6 nF * 1229 Hz/V = 1.0 Hz
--   Kp = 2^14, Ki = 2^10, T = 10.4 us -> wn = 9.9e3 rad/s, zeta = 0.83
--   Kb = 2^2 -> leak-comp zero at wn/26
-- i.e. the same normalised dynamics ~8600x faster.
--
-- Sequence (each phase ends in a PASS/FAIL check):
--   1. Cold start: cap empty (CV = 0 V, VCXO ~ -72 ppm), wallclock_set.
--      Fast acquisition via pd_b, then PI lock via pd_a.
--   2. wallclock_set +400_000 ns: NCO restarts, VCXO counter realigns.
--   3. VCXO frequency step +20 ppm (temperature jump): PI re-locks.
--   4. 20 uA loop-filter leakage: the Kb integrator must take the static
--      phase error back to ~0.
-- Locked checks: |window phase error| small, exactly 512 VCXO edges per
-- sample, and VCXO sample pulse within +/-1 sys_clk of the NCO one
-- (i.e. same LRCK grid as an NCO-mode board; the NCO still runs here
-- because its debug outputs are connected).
--
-- With -gDOMAIN=true the pin clocks come from the VCXO domain; then also
-- checked: 512 VCXO edges per pin LRCK period, internal LRCK 1..3 clk
-- AHEAD of the pin LRCK (the TDM input samples its synchronised data on
-- the internal edge and must not see the bit launched on the pin edge),
-- and the pin LRCK vs. the pin LRCK an NCO-mode
-- board would drive (NCO sample pulse + 1 output register).
--
-- Run (~18 ms sim time, ~10 minutes):
--   ghdl -a --std=08 packages/audioclks_pkg.vhd \
--        packages/wallclock_signals_pkg.vhd ptp/average.vhd \
--        ptp/wallclock.vhd ptp/wallclock_vcxo_tb.vhd
--   ghdl -e --std=08 wallclock_vcxo_tb
--   ghdl -r --std=08 wallclock_vcxo_tb [-gDOMAIN=true] > vcxo.csv
--
-- CSV rows (one per PI window): time_us,phase_id,err_cyc,pump,locked,cv_v,f_ppm

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;

use work.audioclks_pkg.all;
use work.wallclock_signals_pkg.all;

entity wallclock_vcxo_tb is
    generic (
        -- true: also test VCXO_DOMAIN_CLOCKS (pin clocks from the VCXO)
        DOMAIN : boolean := false
    );
end entity;

architecture sim of wallclock_vcxo_tb is

    constant SYS_CLK_HZ_C : natural := 125_000_000;
    constant AUDIO_FS_C   : natural := 48_000;
    constant CLK_PERIOD   : time := 8 ns;
    constant F_NOM        : real := 24.576e6;

    -- analog model
    constant VDD          : real := 3.3;
    constant R_A          : real := 10.0e3;
    constant R_B          : real := 100.0;
    constant R_S          : real := 5.1;
    constant C_F          : real := 1.6e-9;     -- TB-scaled (board: 470 uF)
    constant KV_PPM_PER_V : real := 50.0;
    constant PULL_PPM     : real := 100.0;
    constant V_MID        : real := 1.65;
    constant DT           : real := 8.0e-9;

    -- VCXO vs NCO sample pulse, sys_clk cycles. The NCO pull loop has a
    -- +/-1 MCLK (~5 clk) quantiser dead zone, so where an NCO board sits
    -- inside it is itself uncertain by ~+/-2.5 clk.
    constant SKEW_LIMIT   : integer := 3;

    signal clk     : std_logic := '0';
    signal reset_n : std_logic := '0';
    signal wc      : t_wallclock_signals;

    signal audio_clocks  : t_audio_clocks;
    signal sample_pulse  : std_logic;   -- VCXO grid
    signal nco_spulse    : std_logic;   -- NCO grid
    signal phase_locked  : std_logic;
    signal vcxo_clk      : std_logic := '0';
    signal pump          : t_vcxo_pump;
    signal vcxo_locked   : std_logic;
    signal err_dbg       : signed(31 downto 0);
    signal pump_dbg      : signed(31 downto 0);
    signal pin_clocks    : t_audio_clocks;

    signal v_cap         : real := 0.0;    -- cap voltage
    signal v_cv          : real := 0.0;    -- node voltage = VCXO CV
    signal offset_ppm    : real := 10.0;
    signal i_leak        : real := 0.0;
    signal f_ppm_now     : real := 0.0;

    signal sim_done      : boolean := false;
    signal phase_id      : integer := 0;

    -- lock monitors (cleared by mon_clear)
    signal mon_clear     : boolean := false;
    signal mon_max_err   : real := 0.0;     -- max |window error| [MCLK cycles]
    signal mon_bad_cnt   : natural := 0;    -- samples with /= 512 VCXO edges
    signal mon_max_skew  : integer := 0;    -- max |VCXO - NCO sample pulse| [clk]
    signal mon_skew_sum  : integer := 0;
    signal mon_skew_n    : natural := 0;
    signal mon_unlocked  : boolean := false;
    signal mon_samples   : natural := 0;
    -- pin-clock monitors (DOMAIN only)
    signal mon_pin_bad   : natural := 0;    -- pin LRCK periods /= 512 edges
    signal mon_pin_n     : natural := 0;
    signal mon_lag_min   : integer := 1000; -- internal - pin LRCK [clk]
    signal mon_lag_max   : integer := -1000;
    signal mon_pskew_sum : integer := 0;    -- pin LRCK - NCO-board LRCK [clk]
    signal mon_pskew_max : integer := 0;
    signal mon_pskew_n   : natural := 0;

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
            increment_interval   => 8,
            sys_clk_hz           => SYS_CLK_HZ_C,
            audio_fs             => AUDIO_FS_C,
            pull_loop_gain_shift => 6,     -- TB-scaled NCO pull loop, as
            pull_int_gain_shift  => 27,    -- in wallclock_phasepull_tb
            MCLK_FROM_VCXO       => true,
            VCXO_PD_AVG_LOG2     => 8,
            VCXO_KP_SHIFT        => 14,
            VCXO_KI_SHIFT        => 10,
            VCXO_KB_SHIFT        => 2,
            VCXO_FAST_PUMP       => 1,
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
            phase_locked_o       => phase_locked,
            mclk_cnt_o           => open,
            media_edge_tick_o    => open,
            ppb_adj_dbg_o        => open,
            ppb_trim_dbg_o       => open,
            bias_dbg_o           => open,
            sample_pulse_int_o   => nco_spulse,
            nco_phase_dbg_o      => open,
            nco_inc_dbg_o        => open,
            vcxo_clk_i           => vcxo_clk,
            vcxo_pump_o          => pump,
            vcxo_locked_o        => vcxo_locked,
            vcxo_phase_err_dbg_o => err_dbg,
            vcxo_pump_dbg_o      => pump_dbg,
            pin_clocks_o         => pin_clocks
        );

    -- ============================================================
    -- Loop filter: integrate the pump current once per sys_clk (the
    -- pump outputs are registered, so they are constant for a cycle).
    -- ============================================================
    loop_filter: process(clk)
        variable i  : real;
        variable vc : real;
    begin
        if rising_edge(clk) then
            vc := v_cap;
            i  := 0.0;
            if pump.pd_a_oe = '1' then
                if pump.pd_a = '1' then
                    i := i + (VDD - vc) / (R_A + R_S);
                else
                    i := i - vc / (R_A + R_S);
                end if;
            end if;
            if pump.pd_b_oe = '1' then
                if pump.pd_b = '1' then
                    i := i + (VDD - vc) / (R_B + R_S);
                else
                    i := i - vc / (R_B + R_S);
                end if;
            end if;
            v_cv  <= vc + i * R_S;
            vc    := vc + (i - i_leak) * DT / C_F;
            if vc < 0.0 then
                vc := 0.0;
            elsif vc > VDD then
                vc := VDD;
            end if;
            v_cap <= vc;
        end if;
    end process;

    vcxo: process
        variable ppm : real;
        variable hp  : time;
    begin
        while not sim_done loop
            ppm := offset_ppm + KV_PPM_PER_V * (v_cv - V_MID);
            if ppm > PULL_PPM then
                ppm := PULL_PPM;
            elsif ppm < -PULL_PPM then
                ppm := -PULL_PPM;
            end if;
            f_ppm_now <= ppm;
            hp := (0.5 / (F_NOM * (1.0 + ppm * 1.0e-6))) * 1 sec;
            vcxo_clk <= '1';
            wait for hp;
            vcxo_clk <= '0';
            wait for hp;
        end loop;
        wait;
    end process;

    -- ============================================================
    -- Monitors
    -- ============================================================
    -- VCXO edges per VCXO sample period (must be 512 without slips)
    edge_mon: process(vcxo_clk, sample_pulse, clk)
        variable edges    : natural := 0;
        variable t_nco    : time := 0 ns;
        variable t_vcxo   : time := 0 ns;
        variable skew     : integer;
    begin
        -- A VCXO edge coincident with the clk edge that shows sample_pulse
        -- is already the first edge of the new period (the pulse trails the
        -- 512th edge by ~2.5 clk), so check before counting it.
        if rising_edge(clk) then
            if mon_clear then
                if sample_pulse = '1' then
                    edges := 0;     -- keep the period count aligned
                end if;
                mon_bad_cnt  <= 0;
                mon_max_skew <= 0;
                mon_samples  <= 0;
                mon_skew_sum <= 0;
                mon_skew_n   <= 0;
            else
                if nco_spulse = '1' then
                    t_nco := now;
                end if;
                if sample_pulse = '1' then
                    -- sample_pulse is registered on the clk edge after the
                    -- 512th detected edge; edges counted up to here
                    if edges /= 512 then
                        mon_bad_cnt <= mon_bad_cnt + 1;
                    end if;
                    edges := 0;
                    t_vcxo := now;
                    mon_samples <= mon_samples + 1;
                end if;
                -- pair each pulse with the other grid's nearest one
                if (sample_pulse = '1' or nco_spulse = '1') and t_nco > 0 ns and t_vcxo > 0 ns then
                    skew := (t_vcxo - t_nco) / CLK_PERIOD;
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
        if rising_edge(vcxo_clk) then
            edges := edges + 1;
        end if;
    end process;

    -- Pin LRCK (fsclk_50 rising = sample boundary) against the VCXO edges,
    -- the internal LRCK and the NCO grid.
    pin_mon: process(vcxo_clk, pin_clocks.fsclk_50, audio_clocks.fsclk_50, nco_spulse, clk)
        variable edges   : natural := 0;
        variable t_pin   : time := 0 ns;
        variable t_ncofs : time := 0 ns;
        variable t_int   : time := 0 ns;
        variable d       : integer;
    begin
        if rising_edge(vcxo_clk) then
            edges := edges + 1;
        end if;
        if mon_clear then
            mon_pin_bad   <= 0;
            mon_pin_n     <= 0;
            mon_lag_min   <= 1000;
            mon_lag_max   <= -1000;
            mon_pskew_sum <= 0;
            mon_pskew_max <= 0;
            mon_pskew_n   <= 0;
        else
            if rising_edge(pin_clocks.fsclk_50) then
                if edges /= 512 then
                    mon_pin_bad <= mon_pin_bad + 1;
                end if;
                edges := 0;
                t_pin := now;
                mon_pin_n <= mon_pin_n + 1;
            end if;
            -- lag = internal - pin LRCK edge, paired with whichever edge of
            -- the other grid is nearest (either can come first)
            if rising_edge(audio_clocks.fsclk_50) then
                t_int := now;
                if t_pin > 0 ns then
                    d := ((now - t_pin) / 1 ps + 4000) / 8000;
                    if d < 256 then
                        if d < mon_lag_min then mon_lag_min <= d; end if;
                        if d > mon_lag_max then mon_lag_max <= d; end if;
                    end if;
                end if;
            end if;
            if rising_edge(pin_clocks.fsclk_50) and t_int > 0 ns then
                d := -(((now - t_int) / 1 ps + 4000) / 8000);
                if d > -256 then
                    if d < mon_lag_min then mon_lag_min <= d; end if;
                    if d > mon_lag_max then mon_lag_max <= d; end if;
                end if;
            end if;
            -- NCO board: fsclk register follows its counter by one clk
            if rising_edge(clk) and nco_spulse = '1' then
                t_ncofs := now + CLK_PERIOD;
            end if;
            if (rising_edge(pin_clocks.fsclk_50) or (rising_edge(clk) and nco_spulse = '1'))
               and t_pin > 0 ns and t_ncofs > 0 ns then
                d := (t_pin - t_ncofs) / 1 ps;
                if d >= 0 then d := (d + 4000) / 8000; else d := -((-d + 4000) / 8000); end if;
                if abs(d) < 256 then
                    mon_pskew_sum <= mon_pskew_sum + d;
                    mon_pskew_n   <= mon_pskew_n + 1;
                    if abs(d) > mon_pskew_max then mon_pskew_max <= abs(d); end if;
                end if;
            end if;
        end if;
    end process;

    -- one CSV row + error monitor per PI window (err_dbg changes then)
    win_mon: process
        variable e : real;
    begin
        report "time_us,phase_id,err_cyc,pump,locked,cv_v,f_ppm";
        while not sim_done loop
            wait on err_dbg, pump_dbg, sim_done;
            wait until rising_edge(clk);
            e := err_cycles(err_dbg);
            report real'image(real(now / 1 ns) / 1000.0)
                 & "," & integer'image(phase_id)
                 & "," & real'image(e)
                 & "," & integer'image(to_integer(pump_dbg))
                 & "," & std_logic'image(vcxo_locked)
                 & "," & real'image(v_cv)
                 & "," & real'image(f_ppm_now);
        end loop;
        wait;
    end process;

    err_mon: process(clk)
        variable e : real;
    begin
        if rising_edge(clk) then
            if mon_clear then
                mon_max_err  <= 0.0;
                mon_unlocked <= false;
            else
                e := abs(err_cycles(err_dbg));
                if e > mon_max_err then
                    mon_max_err <= e;
                end if;
                if vcxo_locked /= '1' then
                    mon_unlocked <= true;
                end if;
            end if;
        end if;
    end process;

    -- ============================================================
    -- Stimulus + checks
    -- ============================================================
    stim: process
        variable errors : natural := 0;

        procedure check_locked(obs_time : time;
                               max_err  : real;
                               msg      : string;
                               variable errs : inout natural) is
        begin
            mon_clear <= true;
            wait for 4 * CLK_PERIOD;
            mon_clear <= false;
            wait for obs_time;
            if DOMAIN then
                if mon_pin_bad /= 0 or mon_pin_n < 10 or mon_lag_min < -3 or mon_lag_max > -1
                   or mon_pskew_max > SKEW_LIMIT then
                    report "CHECK FAIL: " & msg & " [pin clocks]"
                         & " bad_periods=" & integer'image(mon_pin_bad)
                         & " periods=" & integer'image(mon_pin_n)
                         & " int_lag=" & integer'image(mon_lag_min) & ".."
                         & integer'image(mon_lag_max) & " clk"
                         & " pin_vs_nco max=" & integer'image(mon_pskew_max)
                         & " mean=" & real'image(real(mon_pskew_sum) / real(mon_pskew_n + 1)) & " clk"
                         severity error;
                    errs := errs + 1;
                else
                    report "CHECK PASS: " & msg & " [pin clocks]"
                         & " periods=" & integer'image(mon_pin_n)
                         & " int_lag=" & integer'image(mon_lag_min) & ".."
                         & integer'image(mon_lag_max) & " clk"
                         & " pin_vs_nco max=" & integer'image(mon_pskew_max)
                         & " mean=" & real'image(real(mon_pskew_sum) / real(mon_pskew_n)) & " clk";
                end if;
            end if;
            if mon_unlocked or mon_max_err > max_err or mon_bad_cnt /= 0
               or (not DOMAIN and mon_max_skew > SKEW_LIMIT) or mon_samples < 10 then
                report "CHECK FAIL: " & msg
                     & " -- unlocked=" & boolean'image(mon_unlocked)
                     & " max|err|=" & real'image(mon_max_err) & " cyc (limit "
                     & real'image(max_err) & ")"
                     & " bad_samples=" & integer'image(mon_bad_cnt)
                     & " max_skew=" & integer'image(mon_max_skew) & " clk"
                     & " mean_skew=" & real'image(real(mon_skew_sum) / real(mon_skew_n + 1)) & " clk"
                     severity error;
                errs := errs + 1;
            else
                report "CHECK PASS: " & msg
                     & " -- max|err|=" & real'image(mon_max_err) & " cyc,"
                     & " samples=" & integer'image(mon_samples)
                     & ", max_skew=" & integer'image(mon_max_skew) & " clk"
                     & ", mean_skew=" & real'image(real(mon_skew_sum) / real(mon_skew_n)) & " clk";
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
        report "=== VCXO TB start ===";
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
        wait for 3500 us;
        check_locked(500 us, 0.1, "phase 1: cold-start acquisition + lock", errors);

        -- 2. wallclock set: NCO restarts at a new phase
        phase_id <= 2;
        do_set(100_400_000 + 7_777);
        wait for 4500 us;   -- the NCO (comparison grid) needs ~4 ms
        check_locked(500 us, 0.1, "phase 2: re-lock after wallclock_set", errors);

        -- 3. VCXO frequency step
        phase_id <= 3;
        offset_ppm <= offset_ppm + 20.0;
        wait for 1500 us;
        check_locked(500 us, 0.1, "phase 3: re-lock after +20 ppm VCXO step", errors);

        -- 4. loop-filter leakage -> Kb must remove the static error
        phase_id <= 4;
        i_leak <= 20.0e-6;
        wait for 5000 us;
        check_locked(500 us, 0.03, "phase 4: zero static error with 20 uA leakage", errors);

        if errors = 0 then
            report "=== VCXO TB PASSED ===";
        else
            report "=== VCXO TB FAILED: " & integer'image(errors) & " check(s) ==="
                severity error;
        end if;
        sim_done <= true;
        wait;
    end process;

end architecture;
