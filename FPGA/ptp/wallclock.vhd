-- ============================================================
-- wallclock: PTP wallclock + AES67 media clock + audio clock tree
--
-- Top level only; the functions live in their own files:
--   wallclock_time.vhd        48-bit seconds + nanoseconds time base
--   wallclock_media_clock.vhd RTP media clock (pure function of the time)
--   wallclock_nco.vhd         PTP-disciplined MCLK NCO + phase pull
--   wallclock_vcxo*.vhd       optional external VCXO as MCLK (PD, PI
--                             controller, charge pump, VCXO-domain pins)
--   wallclock_clk_div.vhd     bclk / fs / TDM frame clocks from mclk_cnt
-- ============================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.audioclks_pkg.all;
use work.wallclock_signals_pkg.all;

entity wallclock is
    generic(
        -- nanoseconds added per clock tick (125 MHz -> 8 ns)
        -- WARNING: Ensure this matches your clock! 125MHz=8ns, 62.5MHz=16ns
        increment_interval : natural := 8;
        -- System clock frequency in Hz (for NCO computation)
        sys_clk_hz : natural := 125_000_000;
        -- Audio sample rate in Hz
        audio_fs : natural := 48_000;
        pull_loop_gain_shift : natural := 14;
        pull_int_gain_shift  : natural := 16;   -- unused (no I term any more)
        PTP_IN_SOFTWARE : boolean := false;

        -- ============================================================
        -- MCLK source select
        --   false : MCLK = NCO MSB (sys_clk quantised, no oscillator needed)
        --   true  : MCLK = external 24.576 MHz VCXO on vcxo_clk_i. The VCXO
        --           is phase-locked to the wallclock by a PI loop driving
        --           the tri-state charge pump vcxo_pump_o; mclk_cnt and
        --           every divided clock (bclk, fs, tdm frame) then count
        --           VCXO edges instead of NCO edges. The NCO keeps running
        --           (debug outputs), and both variants land on the same
        --           sample grid. See wallclock_vcxo.vhd.
        -- ============================================================
        MCLK_FROM_VCXO    : boolean := false;
        -- VCXO edges per PI update = 2^VCXO_PD_AVG_LOG2 (15 -> 1.33 ms).
        -- The phase detector averages over the whole window, which turns
        -- the 8 ns sampling quantisation into ~ps resolution.
        VCXO_PD_AVG_LOG2  : natural := 15;
        -- PI gains as powers of two, in charge-pump sys_clk cycles per
        -- MCLK cycle of phase error (velocity form, the loop-filter cap is
        -- the accumulator; see wallclock_vcxo_ctl.vhd).
        VCXO_KP_SHIFT     : integer := 21;
        VCXO_KI_SHIFT     : integer := 13;
        -- Second integrator that learns the pump rate needed to cancel
        -- loop-filter leakage (electrolytic C, VCXO CV input). Without it
        -- leakage shows up as a static phase offset. Negative = right shift.
        VCXO_KB_SHIFT     : integer := 1;
        -- Acquisition: pump cycles per window on the low-ohmic path while
        -- the phase detector is saturated (sustained cycle slips).
        VCXO_FAST_PUMP    : natural := 16384;
        -- Set if a higher CV lowers the VCXO frequency.
        VCXO_PUMP_INVERT  : boolean := false;
        -- Only with MCLK_FROM_VCXO: generate the pin-level BCLK/LRCK/TDM
        -- frame clocks (pin_clocks_o) in the VCXO clock domain, i.e.
        -- without sys_clk quantisation jitter. The internal clocks_o stay
        -- on the sys_clk grid (they run ~2.5 sys_clk behind the pins).
        -- Requires sys_clk >= 4.5 * MCLK (counter hand-over, see
        -- wallclock_vcxo_domain.vhd).
        VCXO_DOMAIN_CLOCKS : boolean := false
    );
    port(
        clk                     : in  std_logic;
        reset_n                 : in  std_logic;

        wallclock_signals : inout t_wallclock_signals := WALLCLOCK_SIGNALS_UNDRIVEN;


        second_pulse_o          : out std_logic;

        -- ============================================
        -- Media clock for RTP packets (32-bit, from PTP epoch)
        -- Computed from wallclock: media_clock = (sec*48000 + sample_in_sec)
        -- ============================================
        media_clock_o           : out unsigned(31 downto 0);
        audio_mclk_o : out std_logic;
        -- Pulse at fs rate in sys_clk domain (rising edge of LRCK)
        sample_pulse_o          : out std_logic;
        -- 1 ms tick in sys_clk domain (derived from sample_pulse, audio_fs/1000 samples)
        ms_pulse_o              : out std_logic;


        clocks_o : out t_audio_clocks := AUDIO_CLOCKS_RESET;
        phase_locked_o : out std_logic;

        -- Diagnostic outputs for SignalTap / TB monitoring of the
        -- phase pull loop. mclk_cnt_o = NCO sub-sample counter
        -- (0..511), media_edge_tick_o = 1-cycle pulse on each
        -- wallclock-derived sample boundary. Sampling mclk_cnt_o on
        -- media_edge_tick_o gives the phase error in mclk-ticks.
        mclk_cnt_o        : out unsigned(8 downto 0);
        media_edge_tick_o : out std_logic;
        -- Pull-loop internals (TB visibility; small enough for SignalTap)
        ppb_adj_dbg_o     : out signed(31 downto 0);  -- low 32b of ppb_adj_reg
        ppb_trim_dbg_o    : out signed(31 downto 0);  -- always 0 (no trim integrator any more)
        bias_dbg_o        : out signed(31 downto 0);  -- low 32b of nco_phase_bias
        sample_pulse_int_o : out std_logic;           -- NCO sample boundary pulse
        nco_phase_dbg_o   : out unsigned(31 downto 0);-- top 32b of nco_phase
        nco_inc_dbg_o     : out signed(31 downto 0);  -- top 32b of nco_increment

        -- VCXO path (only used with MCLK_FROM_VCXO = true)
        vcxo_clk_i        : in  std_logic := '0';     -- VCXO output, async to clk
        vcxo_pump_o       : out t_vcxo_pump := VCXO_PUMP_IDLE;
        vcxo_locked_o     : out std_logic;
        -- Window-averaged phase error NCO - VCXO, Q16 MCLK cycles
        vcxo_phase_err_dbg_o : out signed(31 downto 0);
        -- Last charge-pump command, signed sys_clk cycles per window
        vcxo_pump_dbg_o   : out signed(31 downto 0);
        -- Audio clocks for the output pins. = clocks_o, except with
        -- VCXO_DOMAIN_CLOCKS, where bclk/fs come from VCXO-clocked
        -- registers and mclk is vcxo_clk_i itself.
        pin_clocks_o      : out t_audio_clocks := AUDIO_CLOCKS_RESET
    );
end wallclock;

architecture Behavioral of wallclock is
    -- time base
    signal nsec          : unsigned(29 downto 0);
    signal sec           : unsigned(47 downto 0);
    signal sec_adj       : integer range -1 to 1;
    signal media_clock   : unsigned(31 downto 0);

    -- wallclock_set / phase-jump strobes: synchronised, rising edge
    signal wallclock_set_1       : std_logic;
    signal wallclock_set_2       : std_logic;
    signal wallclock_set_3       : std_logic;
    signal wallclock_phasejump_1 : std_logic;
    signal wallclock_phasejump_2 : std_logic;
    signal wallclock_phasejump_3 : std_logic;
    signal set_evt               : std_logic;
    signal jump_evt              : std_logic;

    -- NCO
    signal nco_mclk         : std_logic;
    signal nco_mclk_cnt     : unsigned(8 downto 0);
    signal nco_sample_pulse : std_logic;
    signal nco_phase_locked : std_logic;

    -- active MCLK source (NCO or VCXO); every divided clock follows it
    signal mclk_int         : std_logic;
    signal mclk_cnt         : unsigned(8 downto 0);
    signal sample_pulse_int : std_logic;

    signal audioclks_reg    : t_audio_clocks;

    -- Millisecond tick: count audio_fs/1000 sample pulses per ms.
    constant MS_DIVIDER     : natural := audio_fs / 1000;
    signal ms_cnt           : unsigned(15 downto 0) := (others => '0');
    signal ms_pulse_int     : std_logic := '0';
begin

    sync_proc: process (clk)
    begin
        if (rising_edge(clk)) then
            wallclock_set_1 <= wallclock_signals.wallclock_set_i;
            wallclock_set_2 <= wallclock_set_1;
            wallclock_set_3 <= wallclock_set_2;
            wallclock_phasejump_1 <= wallclock_signals.wallclock_do_phasejump_i;
            wallclock_phasejump_2 <= wallclock_phasejump_1;
            wallclock_phasejump_3 <= wallclock_phasejump_2;
        end if;
    end process;

    set_evt  <= '1' when wallclock_set_3 = '0' and wallclock_set_2 = '1' else '0';
    jump_evt <= '1' when wallclock_phasejump_3 = '0' and wallclock_phasejump_2 = '1' else '0';

    -- Output assignments
    wallclock_signals.wallclock_nanoseconds_o <= nsec;
    wallclock_signals.wallclock_seconds_o     <= sec;

    -- NCO MSB (NCO mode) or the sys_clk-synchronised VCXO (VCXO mode, where
    -- the pin-level MCLK comes straight from the oscillator)
    audio_mclk_o   <= mclk_int;
    media_clock_o  <= media_clock;
    sample_pulse_o <= sample_pulse_int;
    ms_pulse_o     <= ms_pulse_int;
    clocks_o       <= audioclks_reg;

    -- Diagnostic outputs (see port declaration)
    mclk_cnt_o         <= nco_mclk_cnt;
    sample_pulse_int_o <= nco_sample_pulse;
    ppb_trim_dbg_o     <= (others => '0');

    -- ============================================================
    -- Time base + media clock
    -- ============================================================
    u_time: entity work.wallclock_time
        generic map(
            increment_interval => increment_interval,
            sys_clk_hz         => sys_clk_hz
        )
        port map(
            clk                   => clk,
            reset_n               => reset_n,
            freq_correction_ppb_i => wallclock_signals.freq_correction_ppb_i,
            set_i                 => set_evt,
            jump_i                => jump_evt,
            seconds_i             => wallclock_signals.wallclock_seconds_i,
            nanoseconds_i         => wallclock_signals.wallclock_nanoseconds_i,
            nsec_o                => nsec,
            sec_o                 => sec,
            sec_adj_o             => sec_adj,
            second_pulse_o        => second_pulse_o
        );

    u_media: entity work.wallclock_media_clock
        generic map(
            audio_fs => audio_fs
        )
        port map(
            clk           => clk,
            reset_n       => reset_n,
            nsec_i        => nsec,
            sec_i         => sec,
            sec_adj_i     => sec_adj,
            media_clock_o => media_clock
        );

    -- ============================================================
    -- NCO (always present: MCLK source in NCO mode, debug in VCXO mode)
    -- ============================================================
    u_nco: entity work.wallclock_nco
        generic map(
            sys_clk_hz           => sys_clk_hz,
            audio_fs             => audio_fs,
            pull_loop_gain_shift => pull_loop_gain_shift,
            PTP_IN_SOFTWARE      => PTP_IN_SOFTWARE
        )
        port map(
            clk                   => clk,
            reset_n               => reset_n,
            freq_correction_ppb_i => wallclock_signals.freq_correction_ppb_i,
            nco_ppb_adj_i         => wallclock_signals.nco_ppb_adj_i,
            nco_ppb_adj_valid_i   => wallclock_signals.nco_ppb_adj_valid_i,
            set_i                 => set_evt,
            media_clock_i         => media_clock,
            mclk_o                => nco_mclk,
            mclk_cnt_o            => nco_mclk_cnt,
            sample_pulse_o        => nco_sample_pulse,
            phase_locked_o        => nco_phase_locked,
            media_edge_tick_o     => media_edge_tick_o,
            ppb_adj_dbg_o         => ppb_adj_dbg_o,
            bias_dbg_o            => bias_dbg_o,
            phase_dbg_o           => nco_phase_dbg_o,
            inc_dbg_o             => nco_inc_dbg_o
        );

    -- ============================================================
    -- MCLK source
    -- ============================================================
    gen_mclk_nco: if not MCLK_FROM_VCXO generate
        pin_clocks_o         <= audioclks_reg;
        mclk_cnt             <= nco_mclk_cnt;
        sample_pulse_int     <= nco_sample_pulse;
        mclk_int             <= nco_mclk;
        phase_locked_o       <= nco_phase_locked;
        vcxo_pump_o          <= VCXO_PUMP_IDLE;
        vcxo_locked_o        <= '0';
        vcxo_phase_err_dbg_o <= (others => '0');
        vcxo_pump_dbg_o      <= (others => '0');
    end generate gen_mclk_nco;

    gen_mclk_vcxo: if MCLK_FROM_VCXO generate
        signal realign_evt : std_logic;
        signal vcxo_locked : std_logic;
    begin
        realign_evt    <= set_evt or jump_evt;
        phase_locked_o <= vcxo_locked;
        vcxo_locked_o  <= vcxo_locked;

        u_vcxo: entity work.wallclock_vcxo
            generic map(
                sys_clk_hz    => sys_clk_hz,
                audio_fs      => audio_fs,
                PD_AVG_LOG2   => VCXO_PD_AVG_LOG2,
                KP_SHIFT      => VCXO_KP_SHIFT,
                KI_SHIFT      => VCXO_KI_SHIFT,
                KB_SHIFT      => VCXO_KB_SHIFT,
                FAST_PUMP     => VCXO_FAST_PUMP,
                PUMP_INVERT   => VCXO_PUMP_INVERT,
                DOMAIN_CLOCKS => VCXO_DOMAIN_CLOCKS
            )
            port map(
                clk             => clk,
                reset_n         => reset_n,
                vcxo_clk_i      => vcxo_clk_i,
                nsec_i          => nsec,
                realign_i       => realign_evt,
                sys_clocks_i    => audioclks_reg,
                mclk_o          => mclk_int,
                mclk_cnt_o      => mclk_cnt,
                sample_pulse_o  => sample_pulse_int,
                locked_o        => vcxo_locked,
                pump_o          => vcxo_pump_o,
                phase_err_dbg_o => vcxo_phase_err_dbg_o,
                pump_dbg_o      => vcxo_pump_dbg_o,
                pin_clocks_o    => pin_clocks_o
            );
    end generate gen_mclk_vcxo;

    -- ============================================================
    -- Audio clock tree (sys_clk domain)
    -- ============================================================
    u_clk_div: entity work.wallclock_clk_div
        port map(
            clk        => clk,
            mclk_i     => mclk_int,
            mclk_cnt_i => mclk_cnt,
            clocks_o   => audioclks_reg
        );

    -- ============================================================
    -- Millisecond pulse: audio_fs/1000 sample pulses (48 @ 48 kHz),
    -- phase-coherent with the PTP-disciplined MCLK.
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

end Behavioral;
