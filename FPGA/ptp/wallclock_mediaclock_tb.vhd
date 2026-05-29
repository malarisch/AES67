-- Media-clock / BCLK-divider testbench for wallclock.vhd
--
-- Scope: the audio clock-tree relationships, checked over a handful of
-- fs periods (~500 us). The two existing wallclock TBs cover timekeeping
-- (wallclock_tb) and the phase-pull dynamics (wallclock_phasepull_tb);
-- this one just confirms the structural edge relationships the I2S/TDM
-- datapath depends on:
--
--   A. media_clock advances by exactly +1 per fs (LRCK) frame -- no
--      lost or doubled samples -- and sample_pulse_o lands exactly on
--      the fs rising edge (both come from the same NCO mclk_cnt wrap).
--
--   B. The BCLK divider tree holds the exact power-of-two ratios within
--      one fs frame:
--          clk_256fs : clk_128fs : clk_64fs(=BCLK) : fs = 256:128:64:1
--      i.e. between two consecutive fs rising edges there are exactly
--      256 / 128 / 64 rising edges of the respective sub-clocks.
--
--   C. fs_tdm_pulse_o is the TDM frame sync: it must pulse exactly once
--      per fs frame, start on the fs rising edge, and be exactly one
--      256fs period wide. The RTL holds it high across 2 NCO rising
--      edges (mclk_cnt 511->0 then 0->1), and 256fs toggles on every
--      NCO rising edge, so the high phase spans exactly 2 256fs half-
--      cycles = 1 full 256fs period (= 2 MCLK = 80 ns @ 24.576 MHz).
--
-- freq_correction_ppb_i is held at 0: these ratios are structural and
-- hold at the nominal NCO frequency.
--
-- Run:
--   ghdl -a --std=08 wallclock.vhd wallclock_mediaclock_tb.vhd
--   ghdl -e --std=08 wallclock_mediaclock_tb
--   ghdl -r --std=08 wallclock_mediaclock_tb --stop-time=2ms \
--        --ieee-asserts=disable
--
-- --ieee-asserts=disable suppresses numeric_std "metavalue detected"
-- noise from the wallclock rollover paths (see wallclock_tb header).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity wallclock_mediaclock_tb is
end entity;

architecture sim of wallclock_mediaclock_tb is

    constant SYS_CLK_HZ_C        : natural := 125_000_000;
    constant AUDIO_FS_C          : natural := 48_000;
    constant INCREMENT_INTERVAL_C: natural := 8;   -- ns/tick at 125 MHz

    constant CLK_PERIOD          : time := 8 ns;   -- 125 MHz

    -- How many fs frames to police. ~24 frames fit in 500 us @ 48 kHz;
    -- check a few of those (skip the first couple while the NCO spins up).
    constant N_FRAMES            : integer := 16;

    -- DUT I/O
    signal clk        : std_logic := '0';
    signal reset_n    : std_logic := '0';

    signal wc_sec_o   : unsigned(47 downto 0);
    signal wc_ns_o    : unsigned(29 downto 0);

    signal wc_set     : std_logic := '0';
    signal wc_sec_i   : unsigned(47 downto 0) := (others => '0');
    signal wc_ns_i    : unsigned(29 downto 0) := (others => '0');

    signal freq_corr  : signed(19 downto 0) := (others => '0');

    signal second_pulse  : std_logic;
    signal audio_mclk    : std_logic;
    signal media_clock   : unsigned(31 downto 0);
    signal sample_pulse  : std_logic;
    signal ms_pulse      : std_logic;

    signal clk_256fs, clk_128fs, clk_64fs, fs_sig : std_logic;
    signal bclk_r, bclk_f, fs_tdm                 : std_logic;
    signal phase_locked                           : std_logic;

    signal mclk_cnt_dbg     : unsigned(8 downto 0);
    signal media_edge_dbg   : std_logic;

    signal sim_done : boolean := false;

    -- Registered previous values for edge detection (sys_clk domain).
    signal prev_256fs : std_logic := '0';
    signal prev_128fs : std_logic := '0';
    signal prev_64fs  : std_logic := '0';
    signal prev_fs    : std_logic := '0';
    signal prev_tdm   : std_logic := '0';

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

    -- Track previous sub-clock values for rising-edge detection.
    edge_track: process(clk)
    begin
        if rising_edge(clk) then
            prev_256fs <= clk_256fs;
            prev_128fs <= clk_128fs;
            prev_64fs  <= clk_64fs;
            prev_fs    <= fs_sig;
            prev_tdm   <= fs_tdm;
        end if;
    end process;

    -- ============================================================
    -- Single stimulus + check process. Over N_FRAMES consecutive fs
    -- frames, count sub-clock rising edges and media_clock increments.
    -- ============================================================
    check_proc: process
        variable e64, e128, e256 : integer;   -- per-frame sub-clock edges
        variable mc_inc          : integer;   -- media_clock increments in frame
        variable mc_frame_start  : unsigned(31 downto 0);
        variable mc_frame_end    : unsigned(31 downto 0);
        variable saw_sp_at_fs    : boolean;   -- sample_pulse coincided with fs edge
        variable tdm_at_fs       : boolean;   -- fs_tdm high on the fs rising edge
        variable tdm_pulses      : integer;   -- fs_tdm rising edges in the frame
        variable tdm_256_halves  : integer;   -- 256fs transitions while fs_tdm='1'
        variable bad_frames      : integer := 0;
    begin
        report "=== media-clock / BCLK TB start ===";

        reset_n <= '1';

        -- Let the NCO spin up: skip the first ~4 fs frames so the divider
        -- tree and media_clock are running steadily before we measure.
        for s in 1 to 4 loop
            wait until rising_edge(clk) and fs_sig = '1' and prev_fs = '0';
        end loop;
        report "warm-up done at " & time'image(now)
             & ", checking " & integer'image(N_FRAMES) & " fs frames";

        for f in 1 to N_FRAMES loop
            -- We are sitting on an fs rising edge (start of frame).
            mc_frame_start := media_clock;
            e64  := 0;
            e128 := 0;
            e256 := 0;
            tdm_pulses     := 0;
            tdm_256_halves := 0;

            -- sample_pulse must assert on this exact fs rising edge:
            -- both are driven by the mclk_cnt 511->0 wrap in nco_proc.
            saw_sp_at_fs := (sample_pulse = '1');
            -- fs_tdm must be high on the fs edge: the RTL arms it on the
            -- same mclk_cnt 511->0 wrap that raises fs.
            tdm_at_fs    := (fs_tdm = '1');

            -- Count sub-clock edges until the NEXT fs rising edge. We also
            -- count 256fs transitions (rising OR falling) that happen while
            -- fs_tdm is high -- the TDM pulse should span exactly 2 of them
            -- (= one full 256fs period), and only fall once.
            loop
                wait until rising_edge(clk);
                if prev_64fs = '0' and clk_64fs = '1' then
                    e64 := e64 + 1;
                end if;
                if prev_128fs = '0' and clk_128fs = '1' then
                    e128 := e128 + 1;
                end if;
                if prev_256fs = '0' and clk_256fs = '1' then
                    e256 := e256 + 1;
                end if;
                -- fs_tdm rising edge (a second one in-frame would be a bug).
                if prev_tdm = '0' and fs_tdm = '1' then
                    tdm_pulses := tdm_pulses + 1;
                end if;
                -- 256fs transitions counted only while the TDM sync is high.
                -- prev_tdm (not fs_tdm) so the cycle fs_tdm falls is excluded
                -- and the cycle it rose is included -- matching the high span.
                if prev_tdm = '1'
                   and ((prev_256fs = '0' and clk_256fs = '1')
                     or (prev_256fs = '1' and clk_256fs = '0')) then
                    tdm_256_halves := tdm_256_halves + 1;
                end if;
                exit when fs_sig = '1' and prev_fs = '0';
            end loop;

            mc_frame_end := media_clock;
            mc_inc := to_integer(mc_frame_end - mc_frame_start);

            -- ----- Test B: divider ratios within this frame -----
            if e64 /= 64 or e128 /= 128 or e256 /= 256 then
                bad_frames := bad_frames + 1;
                report "frame " & integer'image(f)
                     & ": sub-clock edge tally wrong -- 64fs="
                     & integer'image(e64) & " 128fs=" & integer'image(e128)
                     & " 256fs=" & integer'image(e256)
                     & " (expected 64/128/256)"
                    severity error;
            end if;

            -- ----- Test A: media_clock + sample_pulse vs fs edge -----
            if mc_inc /= 1 then
                bad_frames := bad_frames + 1;
                report "frame " & integer'image(f)
                     & ": media_clock advanced by " & integer'image(mc_inc)
                     & " across one fs frame (expected 1)"
                    severity error;
            end if;
            if not saw_sp_at_fs then
                bad_frames := bad_frames + 1;
                report "frame " & integer'image(f)
                     & ": sample_pulse_o did NOT assert on the fs rising edge"
                    severity error;
            end if;

            -- ----- Test C: fs_tdm_pulse frame sync -----
            -- (1) exactly one TDM pulse per fs frame
            if tdm_pulses /= 1 then
                bad_frames := bad_frames + 1;
                report "frame " & integer'image(f)
                     & ": fs_tdm pulsed " & integer'image(tdm_pulses)
                     & " times (expected exactly 1 per fs frame)"
                    severity error;
            end if;
            -- (2) it starts on the fs rising edge
            if not tdm_at_fs then
                bad_frames := bad_frames + 1;
                report "frame " & integer'image(f)
                     & ": fs_tdm was NOT high on the fs rising edge "
                     & "(should start with the frame)"
                    severity error;
            end if;
            -- (3) it is exactly one 256fs period wide: its high phase spans
            -- exactly 2 256fs half-cycles (one rising + one falling 256fs
            -- transition = one full 256fs period = 2 MCLK).
            if tdm_256_halves /= 2 then
                bad_frames := bad_frames + 1;
                report "frame " & integer'image(f)
                     & ": fs_tdm high phase spanned " & integer'image(tdm_256_halves)
                     & " 256fs half-cycles (expected 2 = one full 256fs period)"
                    severity error;
            end if;
        end loop;

        assert bad_frames = 0
            report "media-clock / BCLK checks FAILED: "
                 & integer'image(bad_frames) & " bad observations over "
                 & integer'image(N_FRAMES) & " frames"
            severity error;

        report "ALL OK over " & integer'image(N_FRAMES) & " fs frames: "
             & "256:128:64:fs = 256:128:64:1, media_clock +1 per frame, "
             & "sample_pulse on every fs edge, fs_tdm = 1 pulse/frame "
             & "one 256fs period wide on the fs edge";
        report "=== media-clock / BCLK TB done ===";

        sim_done <= true;
        wait for 5 * CLK_PERIOD;
        std.env.stop;
    end process;

end architecture;
