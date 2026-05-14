-- Phase-pull-loop diagnostic testbench for wallclock.vhd
--
-- Scope: isolate the NCO phase-pull loop and dump its internal state
-- per media_edge_tick. Designed to make the "sawtooth on real HW"
-- behaviour visible in seconds of sim time instead of needing
-- SignalTap windows on the real board.
--
-- Sequence:
--   1. Reset, then a few cycles of free-run.
--   2. wallclock_set to a non-zero epoch.
--   3. wallclock_set with a +400_000 ns offset (~19 mclk-ticks @ 48kHz
--      fs*512 = 24.576 MHz; well above the dead band). Drops the NCO
--      mid-sample to stress the pull loop. (Was a phase_jump in earlier
--      versions; the phase_jump path has been removed.)
--   4. Apply a constant ppb that does NOT perfectly match the XO
--      drift the loop is trying to remove. This is what happens on
--      real HW between PTP servo updates: ppb_adj_reg is close but
--      not exact, so the NCO drifts at the residual rate until the
--      pull loop catches up.
--   5. Optionally step ppb at intervals to mimic the discrete-step
--      servo updates that produce the 0.7 s sawtooth on real HW.
--   6. Log one CSV row per media_edge_tick:
--        time_us, mclk_cnt, phase_err, ppb_adj, ppb_trim, bias
--
-- Run:
--   ghdl -a --std=08 wallclock.vhd wallclock_phasepull_tb.vhd
--   ghdl -e --std=08 wallclock_phasepull_tb
--   ghdl -r --std=08 wallclock_phasepull_tb --stop-time=10ms \
--        --ieee-asserts=disable > phasepull.csv
--
-- Inspect with e.g.:
--   awk -F, 'NR>1 {print $1, $2}' phasepull.csv | gnuplot -p -e \
--     "plot '-' using 1:2 with lines"
--
-- The CSV header is the first report line; subsequent reports are
-- one row per media_edge_tick.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity wallclock_phasepull_tb is
end entity;

architecture sim of wallclock_phasepull_tb is

    constant SYS_CLK_HZ_C        : natural := 125_000_000;
    constant AUDIO_FS_C          : natural := 48_000;
    constant INCREMENT_INTERVAL_C: natural := 8;

    constant CLK_PERIOD : time := 8 ns;

    signal clk     : std_logic := '0';
    signal reset_n : std_logic := '0';

    signal wc_sec_o  : unsigned(47 downto 0);
    signal wc_ns_o   : unsigned(31 downto 0);

    signal wc_set    : std_logic := '0';
    signal wc_sec_i  : unsigned(47 downto 0) := (others => '0');
    signal wc_ns_i   : unsigned(31 downto 0) := (others => '0');

    signal freq_corr   : signed(19 downto 0) := (others => '0');
    signal second_pulse : std_logic;
    signal audio_mclk   : std_logic;
    signal media_clock  : unsigned(31 downto 0);
    signal sample_pulse : std_logic;
    signal ms_pulse     : std_logic;

    signal clk_256fs, clk_128fs, clk_64fs, fs_sig : std_logic;
    signal bclk_r, bclk_f, fs_tdm                 : std_logic;
    signal phase_locked                           : std_logic;

    signal mclk_cnt_dbg     : unsigned(8 downto 0);
    signal media_edge_dbg   : std_logic;
    signal ppb_adj_dbg      : signed(31 downto 0);
    signal ppb_trim_dbg     : signed(31 downto 0);
    signal bias_dbg         : signed(31 downto 0);
    signal sample_pulse_int : std_logic;
    signal nco_phase_dbg    : unsigned(31 downto 0);
    signal nco_inc_dbg      : signed(31 downto 0);

    signal sim_done : boolean := false;

    -- Test phase indicator (mirrored into CSV so post-processing can
    -- partition the trace by what was happening when each row was
    -- logged: 0 = warmup, 1 = post wallclock_set, 2 = post offset-set,
    -- 3 = constant-ppb drift, 4 = stepped-ppb).
    signal phase_id : integer := 0;

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
            ppb_adj_dbg_o           => ppb_adj_dbg,
            ppb_trim_dbg_o          => ppb_trim_dbg,
            bias_dbg_o              => bias_dbg,
            sample_pulse_int_o      => sample_pulse_int,
            nco_phase_dbg_o         => nco_phase_dbg,
            nco_inc_dbg_o           => nco_inc_dbg
        );

    -- Capture mclk_cnt every cycle, but only log on edges. Helps
    -- distinguish "loop is doing nothing" from "loop is doing
    -- something but mclk_cnt happens to land at the same value
    -- on each measurement".
    cycle_dump: process(clk)
        variable last_log_t : time := 0 ns;
    begin
        if rising_edge(clk) then
            -- Dump every 5 us to keep volume tractable.
            if now - last_log_t >= 5 us then
                report "TICK," & time'image(now)
                     & "," & integer'image(to_integer(mclk_cnt_dbg))
                     & "," & integer'image(to_integer(signed('0' & nco_phase_dbg(31 downto 1))))
                     & "," & integer'image(to_integer(ppb_trim_dbg));
                last_log_t := now;
            end if;
        end if;
    end process;

    -- ============================================================
    -- Logger: one CSV row per media_edge_tick AND one row per
    -- sample_pulse_int. The DBG signals for ppb_trim and bias are
    -- registered outputs from nco_bias_proc — they reflect what the
    -- loop computed on the PREVIOUS clock edge. So we sample one
    -- cycle AFTER each event to see the effect of the loop's
    -- response to that edge.
    -- ============================================================
    logger: process
        variable phase_err : integer;
        variable mc        : integer;
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
            -- If we saw an edge on the *previous* cycle, log now: the
            -- registered debug signals (ppb_trim, bias) have caught up.
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
            -- Latch any edge on *this* cycle for next-cycle reporting.
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
    -- Stimulus
    -- ============================================================
    stim: process
    begin
        report "=== phase-pull TB start ===";

        reset_n  <= '0';
        wait for 200 ns;
        reset_n  <= '1';
        wait for 1 us;

        -- ----- Step 1: wallclock_set so media_clock is at a realistic value
        wc_sec_i <= to_unsigned(1234, 48);
        wc_ns_i  <= to_unsigned(500_000_000, 32);
        wc_set   <= '1';
        wait until rising_edge(clk);
        wc_set   <= '0';
        phase_id <= 1;

        -- Let things settle for 500 us (~24 sample edges) at ppb=0.
        wait for 500 us;

        -- ----- Step 2: re-set wallclock to a deliberately off-grid
        -- nanosecond value (was a phase_jump of +400_000 ns in the old
        -- design; phase_jump has been removed, so we drive the NCO
        -- through the same kind of mid-sample offset via wallclock_set,
        -- which also hard-resets nco_phase / mclk_cnt to 0).
        -- 500_400_000 ns - 500_000_000 ns = +400_000 ns offset
        -- (~ 19.2 sample periods, not a multiple, so the pull loop
        -- starts mid-sample).
        report "--- wallclock_set with +400_000 ns offset ---";
        wc_sec_i <= to_unsigned(1234, 48);
        wc_ns_i  <= to_unsigned(500_400_000, 32);
        wc_set   <= '1';
        wait until rising_edge(clk);
        wc_set   <= '0';
        phase_id <= 2;

        -- Watch the loop pull from 0 (hard-reset) back into alignment.
        -- At LG=6 (= 1/64 per edge) and worst-case start error of
        -- 256 ticks, expect <2 ticks of error within ~6.4 ms.
        wait for 10 ms;

        -- ----- Step 3: constant ppb that simulates residual XO drift
        -- that the main PTP servo has not perfectly cancelled.
        -- 10_000 ppb = 10 ppm, ~ what a freshly-PI'd XO might still
        -- show between servo updates. If the I-term works, ppb_trim
        -- should converge so the NCO continues to track at zero phase
        -- error.
        report "--- residual drift: ppb=+10000 ---";
        freq_corr <= to_signed(10_000, 20);
        phase_id  <= 3;
        wait for 5 ms;

        -- ----- Step 4: step ppb every 700 us to mimic the discrete-
        -- step PTP servo output on real HW. The sawtooth on real
        -- hardware has a 0.7 s period; we compress it by 1000x so
        -- the same effect shows up in sim time.
        report "--- stepped ppb (servo-like updates) ---";
        phase_id <= 4;
        for k in 0 to 19 loop
            case k mod 6 is
                when 0 => freq_corr <= to_signed( 8_000, 20);
                when 1 => freq_corr <= to_signed(12_000, 20);
                when 2 => freq_corr <= to_signed( 9_500, 20);
                when 3 => freq_corr <= to_signed(11_000, 20);
                when 4 => freq_corr <= to_signed(10_500, 20);
                when 5 => freq_corr <= to_signed( 9_800, 20);
                when others => null;
            end case;
            wait for 700 us;
        end loop;

        report "=== phase-pull TB done ===";
        sim_done <= true;
        wait for 5 * CLK_PERIOD;
        std.env.stop;
    end process;

end architecture;
