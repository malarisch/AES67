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
        pull_int_gain_shift  : natural := 16;
        PTP_IN_SOFTWARE : boolean := false;

        -- ============================================================
        -- MCLK source select
        --   false : MCLK = NCO MSB (sys_clk quantised, no oscillator needed)
        --   true  : MCLK = external 24.576 MHz VCXO on vcxo_clk_i. The VCXO
        --           is phase-locked to the NCO by a PI loop driving the
        --           tri-state charge pump vcxo_pump_o; mclk_cnt and every
        --           divided clock (bclk, fs, tdm frame) then count VCXO
        --           edges instead of NCO edges. The NCO keeps running as
        --           the phase reference, so both variants land on the
        --           same sample grid.
        -- ============================================================
        MCLK_FROM_VCXO    : boolean := false;
        -- VCXO edges per PI update = 2^VCXO_PD_AVG_LOG2 (15 -> 1.33 ms).
        -- The phase detector averages over the whole window, which turns
        -- the 8 ns sampling quantisation into ~ps resolution.
        VCXO_PD_AVG_LOG2  : natural := 15;
        -- PI gains as powers of two, in charge-pump sys_clk cycles per
        -- MCLK cycle of phase error (velocity form, the loop-filter cap is
        -- the accumulator; see the loop analysis at gen_mclk_vcxo).
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
        -- gen_vcxo_domain).
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
        ppb_trim_dbg_o    : out signed(31 downto 0);  -- low 32b of ppb_trim
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
    constant NS_PER_SEC : signed(31 downto 0) := to_signed(1_000_000_000, 32);
    
    -- Main time registers (as SIGNALS, not variables)
    signal nsec_reg : signed(31 downto 0) := (others => '0');
    signal sec_reg  : unsigned(47 downto 0) := (others => '0');
    
    -- Fractional nanosecond accumulator for sub-nanosecond precision.
    -- Range: bounded by ±FRAC_OVERFLOW = ±sys_clk_hz = ±125e6, which
    -- fits in 28-bit signed (max ±134M). Narrowing from 32 to 28 bits
    -- shortens the Stage-2 adder/compare/subtractor carry chain by
    -- ~4 bits worth of LUT delay -- the dominant critical path here.
    signal frac_ns_accum_minus : signed(27 downto 0) := (others => '0');
    signal frac_ns_accum_plus : signed(27 downto 0) := (others => '0');
    signal frac_ns_accum : signed(27 downto 0) := (others => '0');
    signal new_frac : signed (28 downto 0) := (others => '0');

    constant FRAC_OVERFLOW  : signed(27 downto 0) :=
        to_signed(sys_clk_hz, 28);
    constant FRAC_UNDERFLOW : signed(27 downto 0) :=
        to_signed(-sys_clk_hz, 28);
    
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
    -- Rounded to nearest, not floored. VHDL's integer() truncates toward
    -- zero. For sys_clk = 125 MHz / fs = 48 kHz, the exact value is
    -- 843_705_581.57, so floor() gives -0.57 ULP of error. With NCO and
    -- MEDIA_CLK_RECIP both floored independently, the resulting NCO vs
    -- media_clock drift was ~2 ppm -- enough to drive the PI pull loop
    -- into a slow sawtooth on a free-running leader where there is no
    -- real PTP correction. Rounding both constants symmetrises the error
    -- to +/-0.5 ULP each and removes the systematic relative drift.
    constant NCO_BASE_INC_32 : signed(31 downto 0) :=
        to_signed(integer(real(audio_fs * 512) / real(sys_clk_hz)
                          * 4294967296.0 + 0.5), 32);

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
    signal phase_err  : signed(10 downto 0) := (others => '0');
    signal phase_error_valid: std_logic := '0';
    

    -- Pipeline register to break freq_correction → nco_increment critical path
    -- Original: 32×16 multiply + shift + add (~10ns combined)
    -- Split: Stage 1 (multiply) → Stage 2 (add). No shift now — the result
    -- is consumed at full 48-bit precision.
    signal ppb_adj_reg    : signed(NCO_PHASE_BITS - 1 downto 0) := (others => '0');
    signal ppb_adj_reg_last    : signed(NCO_PHASE_BITS - 1 downto 0) := (others => '0');
    signal ppb_adj_reg_avg    : signed(NCO_PHASE_BITS - 1 downto 0) := (others => '0');
    signal ppb_adj_reg_valid : std_logic;
    
    signal mcu_nco_adj_valid_sync1 : std_logic;
    signal mcu_nco_adj_valid_sync2 : std_logic;
    signal mcu_nco_adj_valid_sync3 : std_logic;
    -- Sample divider: count 512 MCLK rising edges → 1 sample period.
    -- MCLK MSB rising edge = one MCLK cycle. 512 cycles = 1 fs period.
    -- nco_mclk_cnt / nco_sample_pulse always follow the NCO (they are the
    -- phase reference of the VCXO loop); mclk_cnt / sample_pulse_int are
    -- the active MCLK source that every divided clock is derived from
    -- (= the NCO ones, or the VCXO-driven counter with MCLK_FROM_VCXO).
    signal nco_mclk_cnt     : unsigned(8 downto 0) := (others => '0');
    signal nco_sample_pulse : std_logic := '0';
    signal mclk_cnt      : unsigned(8 downto 0) := (others => '0');
    signal sample_pulse_int : std_logic := '0';
    signal mclk_int      : std_logic;
    signal nco_phase_locked : std_logic := '0';

    signal audioclks_reg : t_audio_clocks := AUDIO_CLOCKS_RESET;
    signal fs_tdm_cnt    : unsigned(1 downto 0) := (others => '0');

    -- NCO MSB edge detection (sys_clk domain — phase already exists in
    -- nco_phase, so we just compare current MSB against nco_phase_prev).
    signal nco_rising_tick  : std_logic;
    signal nco_falling_tick : std_logic;


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

    constant MEDIA_CLK_RECIP : unsigned(25 downto 0) :=
        to_unsigned(integer(real(audio_fs) * 4294967296.0 / 1.0e9
                            * 256.0 + 0.5), 26);

    signal media_clock_reg  : unsigned(31 downto 0) := (others => '0');
    signal media_clock_nsec_latch : UNSIGNED(29 downto 0) := (others => '0');
    -- Seconds latched in the SAME cycle as the nanoseconds (plus the
    -- not-yet-applied rollover increment) so the media_clock pipeline
    -- always computes from a coherent (sec, nsec) pair.
    signal media_sec_latch  : unsigned(31 downto 0) := (others => '0');
    -- 32-bit nsec * 26-bit RECIP = 58-bit product
    signal media_mult_reg   : unsigned(55 downto 0) := (others => '0');
    signal media_base       : unsigned(31 downto 0) := (others => '0');


    signal media_clock_prev : unsigned(31 downto 0) := (others => '0');
    signal media_edge_tick  : std_logic;




    constant PHASE_ERR_TO_BIAS_SHIFT : natural :=
        NCO_PHASE_BITS - 9 - pull_loop_gain_shift;  -- 48-9-10 = 29

    constant PULL_DEAD_BAND       : natural := 1;

    signal nco_phase_bias : signed(NCO_PHASE_BITS - 1 downto 0) := (others => '0');
    
    
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
    

    signal nco_ppb_adj_wait : std_logic := '0';
    signal media_clock_proc_wait : std_logic := '0';

    signal wallclock_set_1 : std_logic;
    signal wallclock_set_2 : std_logic;
    signal wallclock_set_3 : std_logic;

    signal wallclock_phasejump_1 : std_logic;
    signal wallclock_phasejump_2 : std_logic;
    signal wallclock_phasejump_3 : std_logic;

    attribute PRESERVE : boolean;

    -- Shift by a (possibly negative) constant amount after resizing to w.
    function shift_signed(x : signed; n : integer; w : natural) return signed is
        variable r : signed(w - 1 downto 0);
    begin
        r := resize(x, w);
        if n >= 0 then
            return shift_left(r, n);
        else
            return shift_right(r, -n);
        end if;
    end function;

    function abs_s(x : signed) return signed is
    begin
        if x < 0 then
            return -x;
        end if;
        return x;
    end function;

    function min_nat(a, b : natural) return natural is
    begin
        if a < b then
            return a;
        end if;
        return b;
    end function;

    function pump_polarity return std_logic is
    begin
        if VCXO_PUMP_INVERT then
            return '1';
        end if;
        return '0';
    end function;

    -- Clamp x to [-lim, +lim] (lim >= 0, same width as x).
    function clamp_signed(x : signed; lim : signed) return signed is
    begin
        if x > lim then
            return lim;
        elsif x < -lim then
            return -lim;
        end if;
        return x;
    end function;

begin


    process (clk)

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

    -- Output assignments
    -- Take the low 30 bits of nsec_reg. resize() of an unsigned keeps the
    -- least-significant bits, so this is bit-identical to the old
    -- unsigned(nsec_reg)(29 downto 0) slice -- but slicing the result of a
    -- type conversion is illegal in strict VHDL (GHDL rejects it), whereas
    -- resize() is portable across GHDL / ModelSim / Quartus.
    wallclock_signals.wallclock_nanoseconds_o <= resize(unsigned(nsec_reg), 30);
    wallclock_signals.wallclock_seconds_o     <= sec_reg;

    second_pulse_o          <= second_pulse_int;
    -- NCO MSB (NCO mode) or the sys_clk-synchronised VCXO (VCXO mode, where
    -- the pin-level MCLK comes straight from the oscillator)
    audioclks_reg.mclk            <= mclk_int;
    audio_mclk_o <= mclk_int;
    media_clock_o           <= media_clock_reg;
    sample_pulse_o          <= sample_pulse_int;
    ms_pulse_o              <= ms_pulse_int;

    clocks_o <= audioclks_reg;

    -- Diagnostic outputs (see port declaration)
    mclk_cnt_o         <= nco_mclk_cnt;
    media_edge_tick_o  <= media_edge_tick;
    ppb_adj_dbg_o      <= ppb_adj_reg(31 downto 0);
    -- nco_phase_bias = phase_err << PHASE_ERR_TO_BIAS_SHIFT; recover
    bias_dbg_o         <= resize(shift_right(nco_phase_bias,
                                  PHASE_ERR_TO_BIAS_SHIFT), 32);
    sample_pulse_int_o <= nco_sample_pulse;
    nco_phase_dbg_o    <= nco_phase(47 downto 16);
    nco_inc_dbg_o      <= nco_increment(47 downto 16);
    

    -- NCO MSB edge detection: nco_phase_prev holds the previous MSB
    -- (registered in nco_proc), so a flank is just (prev xor current).
    nco_rising_tick  <= '1' when (nco_phase_prev = '0'
                                  and nco_phase(NCO_PHASE_BITS - 1) = '1') else '0';
    nco_falling_tick <= '1' when (nco_phase_prev = '1'
                                  and nco_phase(NCO_PHASE_BITS - 1) = '0') else '0';

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
           
            if (PTP_IN_SOFTWARE = true) then
                 mcu_nco_adj_valid_sync1 <= wallclock_signals.nco_ppb_adj_valid_i;
            mcu_nco_adj_valid_sync2 <= mcu_nco_adj_valid_sync1;
            mcu_nco_adj_valid_sync3 <= mcu_nco_adj_valid_sync2;
                if mcu_nco_adj_valid_sync2 /= mcu_nco_adj_valid_sync3 then
                    ppb_adj_reg <= wallclock_signals.nco_ppb_adj_i;
                end if;
            else
            if (nco_ppb_adj_wait = '0') then
                nco_ppb_adj_wait <= '1';
                
                
                    ppb_adj_reg_last <= ppb_adj_reg;
                    ppb_adj_reg <= resize(wallclock_signals.freq_correction_ppb_i * NCO_PPB_SCALE,
                                          NCO_PHASE_BITS);
                
            else
                nco_ppb_adj_wait <= '0';
            end if;
        end if;
        end if;
    end process;
    ppb_adj_reg_valid <= '1' when ppb_adj_reg /= ppb_adj_reg_last and nco_ppb_adj_wait = '1' else '0';
    media_edge_tick <= '1' when (media_clock_reg /= media_clock_prev)
                       else '0';


    nco_bias_proc: process (clk, reset_n)
    begin
        if reset_n = '0' then
            phase_error_valid       <='0';
        elsif rising_edge(clk) then
            phase_error_valid <= '0';
            -- ppb_trim is persistent (integrator) - no default assignment.

            if media_edge_tick = '1' then

                if nco_mclk_cnt <= to_unsigned(255, 9) then
                    -- Lower half: NCO ahead by mclk_cnt ticks.
                    -- For mclk_cnt = 0 this gives 0 (no bias).
                    phase_err <= -signed(resize(nco_mclk_cnt, 11));
                else
                    -- Upper half: NCO behind by (512 - mclk_cnt).
                    phase_err <= to_signed(512, 11)
                               - signed(resize(nco_mclk_cnt, 11));
                end if;
                phase_error_valid <= '1';

            end if;
        end if;
    end process;


    nco_p_proc: process (clk, reset_n) begin
        if (reset_n = '0') then
            nco_phase_bias <= (others => '0');
            nco_phase_locked <= '0';
        elsif rising_edge(clk) then
            if wallclock_set_3 = '0' and wallclock_set_2 = '1' then
                nco_phase_bias <= (others => '0');
                nco_phase_locked <= '0';
            elsif (phase_error_valid = '1') then
                nco_phase_bias <=
                    shift_left(resize(phase_err, NCO_PHASE_BITS),
                               PHASE_ERR_TO_BIAS_SHIFT);
                if phase_err > to_signed(PULL_DEAD_BAND, 11)
                   or phase_err < to_signed(-PULL_DEAD_BAND, 11) then
                    nco_phase_locked <= '0';
                else
                    nco_phase_locked <= '1';
                end if;
            end if;
        end if;
    end process;

    gen_nco_avg : if (PTP_IN_SOFTWARE = false) generate
    average_inst: entity work.average
     generic map(
        DATA_WIDTH => NCO_PHASE_BITS,
        DEPTH => 8
    )
     port map(
        clk_i => clk,
        rst_n_i => reset_n,
        data_i => ppb_adj_reg,
        data_valid_i => ppb_adj_reg_valid,
        data_o => ppb_adj_reg_avg
    );
     end generate;
    nco_increment_proc: process(clk, reset_n) begin

        if (reset_n = '0') then
            nco_increment <= (others => '0');
        elsif (rising_edge(clk)) then
        if (PTP_IN_SOFTWARE = true) then

                nco_increment <= NCO_BASE_INC_48 + ppb_adj_reg + nco_phase_bias;
            
        else
            nco_increment <= NCO_BASE_INC_48 + ppb_adj_reg_avg + nco_phase_bias;
        end if;

        end if;
    end process;
    

    nco_proc: process(clk, reset_n)
    begin
        if reset_n = '0' then
            nco_phase          <= (others => '0');
            nco_phase_prev     <= '0';
            nco_mclk_cnt       <= (others => '0');
            nco_sample_pulse   <= '0';
            media_clock_prev   <= (others => '0');
            fs_tdm_cnt         <= (others => '0');
        elsif rising_edge(clk) then
            nco_sample_pulse <= '0';
            media_clock_prev <= media_clock_reg;

            if wallclock_set_3 = '0' and wallclock_set_2 = '1' then
                nco_phase      <= (others => '0');
                nco_phase_prev <= '0';
                nco_mclk_cnt   <= (others => '0');
            else
                nco_phase <= unsigned(signed(nco_phase) + nco_increment);
                nco_phase_prev <= std_logic(nco_phase(NCO_PHASE_BITS - 1));
                if nco_phase_prev = '0' and nco_phase(NCO_PHASE_BITS - 1) = '1' then
                    -- NCO rising edge: advance divider counter.
                    nco_mclk_cnt <= nco_mclk_cnt + 1;
                    if nco_mclk_cnt = 511 then
                        
                        nco_sample_pulse <= '1';
                    end if;
                    
                end if;
            end if;
        end if;
    

    end process nco_proc;

    -- ============================================================
    -- MCLK source: NCO (default, unchanged behaviour)
    -- ============================================================
    gen_mclk_nco: if not MCLK_FROM_VCXO generate
        pin_clocks_o         <= audioclks_reg;
        mclk_cnt             <= nco_mclk_cnt;
        sample_pulse_int     <= nco_sample_pulse;
        mclk_int             <= std_logic(nco_phase(NCO_PHASE_BITS - 1));  -- NCO MSB = MCLK at ~24.576 MHz
        phase_locked_o       <= nco_phase_locked;
        vcxo_pump_o          <= VCXO_PUMP_IDLE;
        vcxo_locked_o        <= '0';
        vcxo_phase_err_dbg_o <= (others => '0');
        vcxo_pump_dbg_o      <= (others => '0');
    end generate gen_mclk_nco;

    gen_mclk_vcxo: if MCLK_FROM_VCXO generate
        constant MCLK_HZ  : real    := real(audio_fs * 512);
        constant Q        : natural := 16;          -- fractional bits of phase values
        constant PH_W     : natural := 9 + Q;       -- phase in MCLK cycles mod 512
        constant ACC_W    : natural := 32;          -- controller width
        constant QP       : natural := 8;           -- fractional bits of pump cycles
        constant AVG_N    : natural := VCXO_PD_AVG_LOG2;
        constant SUM_W    : natural := PH_W + AVG_N + 1;

        -- nsec -> MCLK cycles: 512*fs/1e9 * 2^36 (31 bit). Product bits
        -- [44:20] = Q16 cycles mod 512. Rounding error 1.5e-10 -> at most
        -- 0.004 MCLK cycles (0.15 ns) at the end of each second.
        constant RECIP_SHIFT : natural := 36;
        constant MCLK_RECIP  : unsigned(30 downto 0) :=
            to_unsigned(integer(MCLK_HZ / 1.0e9 * 2.0**RECIP_SHIFT + 0.5), 31);
        constant RECIP_LO    : unsigned(17 downto 0) := MCLK_RECIP(17 downto 0);
        constant RECIP_HI    : unsigned(12 downto 0) := MCLK_RECIP(30 downto 18);

        -- see phase detector: 2.5 (6.0 with VCXO_DOMAIN_CLOCKS) sys_clk
        -- periods in Q16 MCLK cycles
        function ref_ofs_clks return real is
        begin
            if VCXO_DOMAIN_CLOCKS then
                -- pins run one MCLK behind the internal count, see
                -- gen_vcxo_domain
                return 6.0 - real(sys_clk_hz) / real(audio_fs * 512);
            end if;
            return 2.5;
        end function;
        constant REF_OFS  : unsigned(PH_W - 1 downto 0) :=
            to_unsigned(integer(ref_ofs_clks * MCLK_HZ / real(sys_clk_hz)
                                * 2.0**Q + 0.5), PH_W);
        constant CLAMP    : signed(PH_W - 1 downto 0) := to_signed(32 * 2**Q, PH_W);
        constant HALF_CYC : signed(PH_W - 1 downto 0) := to_signed(2**Q / 2, PH_W);
        -- window counts as saturated above this (fast-mode trigger)
        constant SAT_THR  : signed(PH_W - 1 downto 0) := to_signed(16 * 2**Q, PH_W);
        constant ONE_CYC  : signed(PH_W - 1 downto 0) := to_signed(2**Q, PH_W);
        -- sys_clk cycles over which one window's pump pulses are spread:
        -- slightly shorter than a window, so they are always delivered
        -- before the next command even with the VCXO pulled fast.
        constant WIN_CYC  : natural :=
            integer(2.0**AVG_N * real(sys_clk_hz) / MCLK_HZ
                    * (1.0 - 1.0 / 256.0));
        constant PUMP_MAX : signed(ACC_W - 1 downto 0) :=
            shift_left(to_signed(WIN_CYC, ACC_W), QP);
        -- |e[n]-e[n-1]| beyond this saturates the pump anyway (Kp*DE_MAX
        -- = 2 windows of pumping); clamping it first keeps p_term small.
        function de_max_q return integer is
            variable r : real;
        begin
            r := 2.0 * real(WIN_CYC) / 2.0**VCXO_KP_SHIFT * 2.0**Q;
            if r < 1.0 then
                return 1;
            elsif r > 4194304.0 then   -- 2^22
                return 4194304;
            end if;
            return integer(r);
        end function;
        constant DE_MAX   : signed(PH_W - 1 downto 0) := to_signed(de_max_q, PH_W);
        constant FAST_N   : natural := min_nat(VCXO_FAST_PUMP, WIN_CYC);
        constant PUMP_POL : std_logic := pump_polarity;

        constant LOCK_THR   : signed(PH_W - 1 downto 0) := to_signed(2**Q / 4, PH_W); -- 1/4 MCLK
        constant UNLOCK_THR : signed(PH_W - 1 downto 0) := to_signed(2**Q, PH_W);     -- 1 MCLK
        constant EXIT_MARGIN: signed(PH_W - 1 downto 0) := to_signed(2**Q / 256, PH_W);

        -- synchroniser / edge detect
        signal vcxo_meta   : std_logic := '0';
        signal vcxo_sync   : std_logic := '0';
        signal vcxo_sync_d : std_logic := '0';
        attribute PRESERVE of vcxo_meta : signal is true;
        attribute PRESERVE of vcxo_sync : signal is true;
        signal vcxo_tick   : std_logic;

        -- VCXO-driven MCLK counter (drives every divided audio clock).
        -- vcxo_cnt = vcxo_raw + slip_ofs; the PD snapshots vcxo_raw and
        -- adds the *current* slip_ofs when it computes the error, so a slip
        -- decided while later edges are still in the pipeline is never
        -- counted twice.
        signal vcxo_cnt    : unsigned(8 downto 0) := (others => '0');
        signal vcxo_raw    : unsigned(8 downto 0) := (others => '0');
        signal slip_ofs    : unsigned(8 downto 0) := (others => '0');
        signal vcxo_spulse : std_logic := '0';

        -- phase detector pipeline
        signal pd_v        : std_logic_vector(1 to 8) := (others => '0');
        signal pd_nsec     : unsigned(29 downto 0) := (others => '0');
        signal pd_raw1     : unsigned(8 downto 0) := (others => '0');
        signal pd_raw2, pd_raw3, pd_raw4, pd_raw5, pd_raw6 : unsigned(8 downto 0) := (others => '0');
        signal pp_ll       : unsigned(35 downto 0) := (others => '0');
        signal pp_lh       : unsigned(30 downto 0) := (others => '0');
        signal pp_hl       : unsigned(29 downto 0) := (others => '0');
        signal pp_hh       : unsigned(24 downto 0) := (others => '0');
        signal pp_mid      : unsigned(31 downto 0) := (others => '0');
        signal pp_ll2      : unsigned(35 downto 0) := (others => '0');
        signal pp_hh2      : unsigned(8 downto 0) := (others => '0');
        signal pp_hh3      : unsigned(8 downto 0) := (others => '0');
        signal pp_sum      : unsigned(44 downto 0) := (others => '0');
        signal wc_phase    : unsigned(PH_W - 1 downto 0) := (others => '0');
        signal pd_refc     : unsigned(PH_W - 1 downto 0) := (others => '0');
        signal pd_err      : signed(PH_W - 1 downto 0) := (others => '0');
        signal pd_err_used : signed(PH_W - 1 downto 0) := (others => '0');
        signal slip_up     : std_logic := '0';
        signal slip_dn     : std_logic := '0';
        signal slip_thr    : signed(PH_W - 1 downto 0) := to_signed(2**Q / 2, PH_W);

        -- window accumulation
        signal win_cnt     : unsigned(AVG_N - 1 downto 0) := (others => '0');
        signal e_sum       : signed(SUM_W - 1 downto 0) := (others => '0');
        signal win_slip    : std_logic := '0';
        signal win_done    : std_logic := '0';
        signal win_mean    : signed(PH_W - 1 downto 0) := (others => '0');
        signal win_had_slip: std_logic := '0';
        signal realign_evt : std_logic;
        signal holdoff     : unsigned(1 downto 0) := "10";

        -- controller (one arithmetic operation per step, it has a whole
        -- window of time)
        signal ctl_step    : natural range 0 to 11 := 0;
        signal ae          : signed(PH_W - 1 downto 0) := (others => '0'); -- |e_mean|
        signal ae_prev_m   : signed(PH_W - 1 downto 0) := (others => '0'); -- |e_prev| - margin
        signal de          : signed(PH_W - 1 downto 0) := (others => '0');
        signal p_ok        : std_logic := '0';
        signal b_add       : signed(ACC_W - 1 downto 0) := (others => '0');
        signal b_next      : signed(ACC_W - 1 downto 0) := (others => '0');
        signal a_abs       : signed(ACC_W - 1 downto 0) := (others => '0');
        signal a_neg       : std_logic := '0';
        signal n_abs       : signed(ACC_W - 1 downto 0) := (others => '0');
        signal v_n         : signed(ACC_W - 1 downto 0) := (others => '0');
        signal e_mean      : signed(PH_W - 1 downto 0) := (others => '0');
        signal e_prev      : signed(PH_W - 1 downto 0) := (others => '0');
        signal prev_valid  : std_logic := '0';
        signal w_slip      : std_logic := '0';
        signal w_slip_prev : std_logic := '0';
        signal w_invalid   : std_logic := '0';
        signal sat_run     : unsigned(1 downto 0) := (others => '0');
        signal fast_mode   : std_logic := '0';
        signal fast_cmd    : std_logic := '0';
        signal run_pi      : std_logic := '0';
        signal p_term      : signed(ACC_W - 1 downto 0) := (others => '0');
        signal i_term      : signed(ACC_W - 1 downto 0) := (others => '0');
        signal b_int       : signed(ACC_W - 1 downto 0) := (others => '0');
        signal pump_acc    : signed(ACC_W - 1 downto 0) := (others => '0');
        signal pump_res    : signed(ACC_W - 1 downto 0) := (others => '0');
        signal pump_cmd    : signed(31 downto 0) := (others => '0');
        signal lock_cnt    : unsigned(4 downto 0) := (others => '0');
        signal vcxo_locked : std_logic := '0';

        -- charge-pump PDM
        signal pump_start  : std_logic := '0';
        signal pump_len    : natural range 0 to WIN_CYC := 0;   -- pulses this window
        signal pump_up     : std_logic := '0';
        signal pump_on_b   : std_logic := '0';
        signal pdm_acc     : natural range 0 to 2 * WIN_CYC := 0;
        signal pdm_left    : natural range 0 to WIN_CYC := 0;   -- cycles left in window
        signal pump_pulse  : std_logic := '0';
    begin
        assert real(sys_clk_hz) > 2.2 * MCLK_HZ
            report "wallclock: sys_clk too slow to sample the VCXO"
            severity failure;
        assert VCXO_KI_SHIFT <= 17 and VCXO_KB_SHIFT <= 17
            report "wallclock: VCXO gain shift overflows the controller width"
            severity failure;

        mclk_int             <= vcxo_sync;
        mclk_cnt             <= vcxo_cnt;
        sample_pulse_int     <= vcxo_spulse;
        phase_locked_o       <= vcxo_locked;
        vcxo_locked_o        <= vcxo_locked;
        vcxo_phase_err_dbg_o <= resize(e_mean, 32);
        vcxo_pump_dbg_o      <= pump_cmd;

        vcxo_tick   <= vcxo_sync and not vcxo_sync_d;
        realign_evt <= '1' when (wallclock_set_3 = '0' and wallclock_set_2 = '1')
                             or (wallclock_phasejump_3 = '0' and wallclock_phasejump_2 = '1')
                       else '0';

        vcxo_sync_proc: process(clk)
        begin
            if rising_edge(clk) then
                vcxo_meta   <= vcxo_clk_i;
                vcxo_sync   <= vcxo_meta;
                vcxo_sync_d <= vcxo_sync;
            end if;
        end process;

        -- VCXO MCLK counter + phase detector stage 1
        vcxo_cnt_proc: process(clk, reset_n)
            variable v_inc : integer range -1 to 2;
        begin
            if reset_n = '0' then
                vcxo_cnt    <= (others => '0');
                vcxo_raw    <= (others => '0');
                slip_ofs    <= (others => '0');
                vcxo_spulse <= '0';
                pd_nsec     <= (others => '0');
                pd_raw1     <= (others => '0');
            elsif rising_edge(clk) then
                v_inc := 0;
                if vcxo_tick = '1' then
                    v_inc := 1;
                    vcxo_raw <= vcxo_raw + 1;
                end if;
                if slip_up = '1' then
                    v_inc := v_inc + 1;
                    slip_ofs <= slip_ofs + 1;
                elsif slip_dn = '1' then
                    v_inc := v_inc - 1;
                    slip_ofs <= slip_ofs - 1;
                end if;

                vcxo_spulse <= '0';
                case v_inc is
                    when 1 =>
                        vcxo_cnt <= vcxo_cnt + 1;
                        if vcxo_cnt = 511 then
                            vcxo_spulse <= '1';
                        end if;
                    when 2 =>
                        vcxo_cnt <= vcxo_cnt + 2;
                        if vcxo_cnt >= 510 then
                            vcxo_spulse <= '1';
                        end if;
                    when -1 =>
                        vcxo_cnt <= vcxo_cnt - 1;
                    when others =>
                        null;
                end case;

                -- Stage 1: wallclock time of this edge + its raw index.
                -- nsec_reg is the time of the previous clk edge, i.e. 1.5
                -- sys_clk (mean) after the VCXO edge -> part of REF_OFS.
                if vcxo_tick = '1' then
                    pd_nsec <= unsigned(nsec_reg(29 downto 0));
                    pd_raw1 <= vcxo_raw + 1;
                end if;
            end if;
        end process;

        -- Phase detector stages 2..8: nsec -> MCLK phase (4 partial
        -- products, each <= 18x18, so every stage fits 125 MHz on small
        -- parts), offset, error vs. the VCXO count, slips.
        pd_proc: process(clk, reset_n)
        begin
            if reset_n = '0' then
                pd_v        <= (others => '0');
                slip_up     <= '0';
                slip_dn     <= '0';
                pd_err      <= (others => '0');
                pd_err_used <= (others => '0');
                slip_thr    <= HALF_CYC;
            elsif rising_edge(clk) then
                pd_v <= vcxo_tick & pd_v(1 to 7);

                -- While realigning (boot, wallclock set / phase jump) snap
                -- the counter to the nearest MCLK edge; afterwards saturate.
                if holdoff /= 0 then
                    slip_thr <= HALF_CYC;
                else
                    slip_thr <= CLAMP;
                end if;

                -- Stage 2: partial products
                pp_ll   <= pd_nsec(17 downto 0)  * RECIP_LO;
                pp_lh   <= pd_nsec(17 downto 0)  * RECIP_HI;
                pp_hl   <= pd_nsec(29 downto 18) * RECIP_LO;
                pp_hh   <= pd_nsec(29 downto 18) * RECIP_HI;
                pd_raw2 <= pd_raw1;
                -- Stage 3
                pp_mid  <= resize(pp_lh, 32) + resize(pp_hl, 32);
                pp_ll2  <= pp_ll;
                pp_hh2  <= pp_hh(8 downto 0);   -- only bits < 2^45 matter
                pd_raw3 <= pd_raw2;
                -- Stage 4: ll + mid<<18 (the low 18 bits are ll's alone)
                pp_sum(17 downto 0)  <= pp_ll2(17 downto 0);
                pp_sum(44 downto 18) <= pp_ll2(35 downto 18) + pp_mid(26 downto 0);
                pp_hh3  <= pp_hh2;
                pd_raw4 <= pd_raw3;
                -- Stage 5: + hh<<36 -> Q16 cycles mod 512
                wc_phase(Q - 1 downto 0)    <= pp_sum(35 downto 20);
                wc_phase(PH_W - 1 downto Q) <= pp_sum(44 downto 36) + pp_hh3;
                pd_raw5 <= pd_raw4;
                -- Stage 6
                pd_refc <= wc_phase - REF_OFS;
                pd_raw6 <= pd_raw5;
                -- Stage 7: only the integer part differs, mod 512 wraps
                -- into [-256, 256) cycles
                pd_err <= signed((pd_refc(PH_W - 1 downto Q) - (pd_raw6 + slip_ofs))
                                 & pd_refc(Q - 1 downto 0));
                -- Stage 8: saturate by slipping the VCXO counter
                slip_up <= '0';
                slip_dn <= '0';
                if pd_v(7) = '1' then
                    if pd_err > slip_thr then
                        slip_up     <= '1';           -- VCXO count behind
                        pd_err_used <= pd_err - ONE_CYC;
                    elsif pd_err < -slip_thr then
                        slip_dn     <= '1';           -- VCXO count ahead
                        pd_err_used <= pd_err + ONE_CYC;
                    else
                        pd_err_used <= pd_err;
                    end if;
                end if;
            end if;
        end process;

        -- Window average of the phase error
        win_proc: process(clk, reset_n)
            variable v_sum : signed(SUM_W - 1 downto 0);
        begin
            if reset_n = '0' then
                win_cnt  <= (others => '0');
                e_sum    <= (others => '0');
                win_slip <= '0';
                win_done <= '0';
                win_mean <= (others => '0');
                win_had_slip <= '0';
            elsif rising_edge(clk) then
                win_done <= '0';
                if pd_v(8) = '1' then
                    v_sum   := e_sum + resize(pd_err_used, SUM_W);
                    win_cnt <= win_cnt + 1;
                    if win_cnt = 2**AVG_N - 1 then
                        win_mean     <= resize(shift_right(v_sum, AVG_N), PH_W);
                        win_had_slip <= win_slip or slip_up or slip_dn;
                        win_done     <= '1';
                        e_sum        <= (others => '0');
                        win_slip     <= '0';
                    else
                        e_sum <= v_sum;
                        if slip_up = '1' or slip_dn = '1' then
                            win_slip <= '1';
                        end if;
                    end if;
                end if;
            end if;
        end process;

        -- PI controller + mode/lock state, one step per clock after each
        -- window. Pump quantities are Q8 sys_clk cycles (QP), phase Q16.
        ctl_proc: process(clk, reset_n)
        begin
            if reset_n = '0' then
                ctl_step    <= 0;
                e_mean      <= (others => '0');
                e_prev      <= (others => '0');
                ae          <= (others => '0');
                ae_prev_m   <= (others => '0');
                de          <= (others => '0');
                p_ok        <= '0';
                prev_valid  <= '0';
                w_slip      <= '0';
                w_slip_prev <= '0';
                w_invalid   <= '0';
                sat_run     <= (others => '0');
                fast_mode   <= '0';
                fast_cmd    <= '0';
                run_pi      <= '0';
                p_term      <= (others => '0');
                i_term      <= (others => '0');
                b_add       <= (others => '0');
                b_next      <= (others => '0');
                b_int       <= (others => '0');
                pump_acc    <= (others => '0');
                pump_res    <= (others => '0');
                a_abs       <= (others => '0');
                a_neg       <= '0';
                n_abs       <= (others => '0');
                v_n         <= (others => '0');
                pump_cmd    <= (others => '0');
                pump_start  <= '0';
                pump_len    <= 0;
                pump_up     <= '0';
                pump_on_b   <= '0';
                lock_cnt    <= (others => '0');
                vcxo_locked <= '0';
                holdoff     <= "10";
            elsif rising_edge(clk) then
                pump_start <= '0';

                -- Wallclock set / phase jump: the reference jumps, the PD
                -- realigns by slipping. Mask this and the next window.
                if realign_evt = '1' then
                    holdoff <= "10";
                end if;

                case ctl_step is
                    when 0 =>
                        if win_done = '1' then
                            e_mean      <= win_mean;
                            ae          <= abs_s(win_mean);
                            w_slip      <= win_had_slip;
                            w_slip_prev <= w_slip;
                            if holdoff /= 0 or realign_evt = '1' then
                                w_invalid <= '1';
                                if realign_evt = '0' then
                                    holdoff <= holdoff - 1;
                                end if;
                            else
                                w_invalid <= '0';
                            end if;
                            ctl_step <= 1;
                        end if;

                    when 1 =>
                        -- mode decision + lock detector
                        fast_cmd <= '0';
                        run_pi   <= '0';
                        p_ok     <= prev_valid and not w_invalid;
                        if w_invalid = '1' then
                            prev_valid  <= '0';
                            sat_run     <= (others => '0');
                            vcxo_locked <= '0';
                            lock_cnt    <= (others => '0');
                        else
                            -- saturated window: slipped, or far out of lock
                            if w_slip = '1' or ae > SAT_THR then
                                if sat_run /= 3 then
                                    sat_run <= sat_run + 1;
                                end if;
                            else
                                sat_run <= (others => '0');
                            end if;

                            if fast_mode = '1' then
                                -- leave once the (unsaturated) error shrinks:
                                -- the VCXO frequency has crossed the target
                                if w_slip = '0' and w_slip_prev = '0' and prev_valid = '1'
                                   and ae < ae_prev_m then
                                    fast_mode <= '0';
                                    pump_res  <= (others => '0');
                                else
                                    fast_cmd <= '1';
                                end if;
                            elsif (w_slip = '1' or ae > SAT_THR) and sat_run >= 2 then
                                fast_mode <= '1';
                                fast_cmd  <= '1';
                            else
                                run_pi <= '1';
                            end if;
                            prev_valid <= '1';

                            if fast_mode = '1' or w_slip = '1' or ae > UNLOCK_THR then
                                vcxo_locked <= '0';
                                lock_cnt    <= (others => '0');
                            elsif ae < LOCK_THR then
                                if lock_cnt = 31 then
                                    vcxo_locked <= '1';
                                else
                                    lock_cnt <= lock_cnt + 1;
                                end if;
                            end if;
                        end if;
                        de        <= e_mean - e_prev;
                        e_prev    <= e_mean;
                        ae_prev_m <= ae - EXIT_MARGIN;
                        ctl_step  <= 2;

                    when 2 =>
                        -- velocity-form P needs the previous error; after
                        -- an invalid window it is taken as equal (bumpless)
                        de     <= clamp_signed(de, DE_MAX);
                        i_term <= shift_signed(e_mean, VCXO_KI_SHIFT + QP - Q, ACC_W);
                        b_add  <= shift_signed(e_mean, VCXO_KB_SHIFT + QP - Q, ACC_W);
                        ctl_step <= 3;

                    when 3 =>
                        if p_ok = '1' then
                            p_term <= shift_signed(de, VCXO_KP_SHIFT + QP - Q, ACC_W);
                        else
                            p_term <= (others => '0');
                        end if;
                        b_next   <= b_int + b_add;
                        ctl_step <= 4;

                    when 4 =>
                        -- leak estimate: frozen while slipping, so a
                        -- saturated pull-in can't wind it up
                        if run_pi = '1' and w_slip = '0' then
                            b_int <= clamp_signed(b_next, shift_right(PUMP_MAX, 1));
                        end if;
                        pump_acc <= p_term + i_term;
                        ctl_step <= 5;

                    when 5 =>
                        pump_acc <= pump_acc + b_int;
                        ctl_step <= 6;

                    when 6 =>
                        pump_acc <= pump_acc + pump_res;
                        ctl_step <= 7;

                    when 7 =>
                        pump_acc <= clamp_signed(pump_acc, shift_left(PUMP_MAX, 1));
                        ctl_step <= 8;

                    when 8 =>
                        a_abs    <= abs_s(pump_acc);
                        a_neg    <= pump_acc(ACC_W - 1);
                        ctl_step <= 9;

                    when 9 =>
                        -- whole pump cycles (toward zero), at most one window
                        n_abs <= shift_right(a_abs, QP);
                        if shift_right(a_abs, QP) > to_signed(WIN_CYC, ACC_W) then
                            n_abs <= to_signed(WIN_CYC, ACC_W);
                        end if;
                        ctl_step <= 10;

                    when 10 =>
                        if a_neg = '1' then
                            v_n <= -n_abs;
                        else
                            v_n <= n_abs;
                        end if;
                        ctl_step <= 11;

                    when others =>
                        -- issue the command, keep the fraction
                        if fast_cmd = '1' then
                            pump_len  <= FAST_N;
                            pump_up   <= not e_mean(PH_W - 1);
                            pump_on_b <= '1';
                            if e_mean < 0 then
                                pump_cmd <= to_signed(-FAST_N, 32);
                            else
                                pump_cmd <= to_signed(FAST_N, 32);
                            end if;
                        elsif run_pi = '1' then
                            pump_res  <= pump_acc - shift_left(v_n, QP);
                            pump_len  <= to_integer(n_abs);
                            pump_up   <= not a_neg;
                            pump_on_b <= '0';
                            pump_cmd  <= resize(v_n, 32);
                        else
                            pump_len  <= 0;
                            pump_on_b <= '0';
                            pump_cmd  <= (others => '0');
                        end if;
                        pump_start <= '1';
                        ctl_step   <= 0;
                end case;
            end if;
        end process;

        -- Spread pump_len single-cycle pulses evenly over WIN_CYC cycles
        pdm_proc: process(clk, reset_n)
            variable v_acc : natural range 0 to 2 * WIN_CYC;
        begin
            if reset_n = '0' then
                pdm_acc    <= 0;
                pdm_left   <= 0;
                pump_pulse <= '0';
                vcxo_pump_o <= VCXO_PUMP_IDLE;
            elsif rising_edge(clk) then
                pump_pulse <= '0';
                if pump_start = '1' then
                    pdm_acc  <= 0;
                    pdm_left <= WIN_CYC;
                elsif pdm_left /= 0 then
                    pdm_left <= pdm_left - 1;
                    v_acc := pdm_acc + pump_len;
                    if v_acc >= WIN_CYC then
                        v_acc := v_acc - WIN_CYC;
                        pump_pulse <= '1';
                    end if;
                    pdm_acc <= v_acc;
                end if;

                -- registered pin drive: only the selected path is ever enabled
                vcxo_pump_o.pd_a    <= pump_up xor PUMP_POL;
                vcxo_pump_o.pd_b    <= pump_up xor PUMP_POL;
                vcxo_pump_o.pd_a_oe <= pump_pulse and not pump_on_b;
                vcxo_pump_o.pd_b_oe <= pump_pulse and pump_on_b;
            end if;
        end process;

        gen_vcxo_sysclk_pins: if not VCXO_DOMAIN_CLOCKS generate
            pin_clocks_o <= audioclks_reg;
        end generate gen_vcxo_sysclk_pins;


        gen_vcxo_domain: if VCXO_DOMAIN_CLOCKS generate
            signal vraw_v   : unsigned(8 downto 0) := (others => '0');
            signal vraw_g   : std_logic_vector(8 downto 0) := (others => '0');
            signal vofs_m   : unsigned(8 downto 0) := (others => '0');
            signal vofs_q1  : unsigned(8 downto 0) := (others => '0');
            signal vofs_q2  : unsigned(8 downto 0) := (others => '0');
            signal vofs_v   : unsigned(8 downto 0) := (others => '0');
            signal pclk     : t_audio_clocks := AUDIO_CLOCKS_RESET;
            attribute PRESERVE of vofs_m  : signal is true;
            attribute PRESERVE of vofs_q1 : signal is true;

            signal g_meta   : std_logic_vector(8 downto 0) := (others => '0');
            signal g_sync   : std_logic_vector(8 downto 0) := (others => '0');
            attribute PRESERVE of g_meta : signal is true;
            attribute PRESERVE of g_sync : signal is true;
            signal vraw_s   : unsigned(8 downto 0) := (others => '0');
            signal tick_d   : std_logic_vector(1 to 3) := (others => '0');
            signal vofs_s   : unsigned(8 downto 0) := (others => '0');
        begin
            assert real(sys_clk_hz) >= 4.5 * MCLK_HZ
                report "wallclock: VCXO_DOMAIN_CLOCKS needs sys_clk >= 4.5 * MCLK"
                severity failure;

            pin_clocks_o <= (
                mclk      => vcxo_clk_i,
                clk_256fs => pclk.clk_256fs,
                clk_128fs => pclk.clk_128fs,
                clk_64fs  => pclk.clk_64fs,
                fsclk_50  => pclk.fsclk_50
            );

            -- VCXO domain: free-running count, offset take-over, and the
            -- same divider logic as the sys_clk processes below -- but on
            -- the new count, so the pins switch on the counting edge.
            vcxo_dom_proc: process(vcxo_clk_i)
                variable c  : unsigned(8 downto 0);
                variable cs : unsigned(8 downto 0);
            begin
                if rising_edge(vcxo_clk_i) then
                    vraw_v <= vraw_v + 1;
                    vraw_g <= std_logic_vector((vraw_v + 1) xor shift_right(vraw_v + 1, 1));

                    vofs_m  <= vofs_s;
                    vofs_q1 <= vofs_m;
                    vofs_q2 <= vofs_q1;
                    if vofs_q1 = vofs_q2 then
                        vofs_v <= vofs_q2;
                    end if;

                    -- pins = internal count delayed by one MCLK (see above)
                    c := vraw_v + vofs_v;

                    pclk.clk_256fs.bclk <= not c(0);
                    pclk.clk_128fs.bclk <= not c(1);
                    pclk.clk_64fs.bclk  <= not c(2);
                    pclk.fsclk_50       <= not c(8);

                    if c = 0 then
                        pclk.clk_256fs.fsclk_tdm <= '1';
                        pclk.clk_128fs.fsclk_tdm <= '1';
                        pclk.clk_64fs.fsclk_tdm  <= '1';
                    elsif c = 2 then
                        pclk.clk_256fs.fsclk_tdm <= '0';
                    elsif c = 4 then
                        pclk.clk_128fs.fsclk_tdm <= '0';
                    elsif c = 8 then
                        pclk.clk_64fs.fsclk_tdm  <= '0';
                    end if;

                    cs := c + 2;
                    pclk.clk_256fs.fsclk_i2s_50 <= not cs(8);
                    if cs = 0 then
                        pclk.clk_256fs.fsclk_i2s_tdm <= '1';
                    elsif cs = 2 then
                        pclk.clk_256fs.fsclk_i2s_tdm <= '0';
                    end if;
                    cs := c + 3;
                    pclk.clk_256fs.fsclk_i2s_50_dac <= not cs(8);

                    cs := c + 4;
                    pclk.clk_128fs.fsclk_i2s_50 <= not cs(8);
                    if cs = 0 then
                        pclk.clk_128fs.fsclk_i2s_tdm <= '1';
                    elsif cs = 4 then
                        pclk.clk_128fs.fsclk_i2s_tdm <= '0';
                    end if;
                    cs := c + 6;
                    pclk.clk_128fs.fsclk_i2s_50_dac <= not cs(8);

                    cs := c + 8;
                    pclk.clk_64fs.fsclk_i2s_50 <= not cs(8);
                    if cs = 0 then
                        pclk.clk_64fs.fsclk_i2s_tdm <= '1';
                    elsif cs = 8 then
                        pclk.clk_64fs.fsclk_i2s_tdm <= '0';
                    end if;
                    cs := c + 12;
                    pclk.clk_64fs.fsclk_i2s_50_dac <= not cs(8);
                end if;
            end process;

            -- sys_clk domain: synchronise the Gray count, measure offset
            vofs_proc: process(clk)
                variable b : unsigned(8 downto 0);
            begin
                if rising_edge(clk) then
                    g_meta <= vraw_g;
                    g_sync <= g_meta;
                    b(8) := g_sync(8);
                    for i in 7 downto 0 loop
                        b(i) := b(i + 1) xor g_sync(i);
                    end loop;
                    vraw_s <= b;

                    tick_d <= vcxo_tick & tick_d(1 to 2);
                    if tick_d(3) = '1' then
                        vofs_s <= vcxo_cnt - vraw_s;
                    end if;
                end if;
            end process;
        end generate gen_vcxo_domain;
    end generate gen_mclk_vcxo;
    process (clk)
    begin
        if rising_edge(clk) then
                audioclks_reg.clk_256fs.bclk <= not mclk_cnt(0);
    audioclks_reg.clk_128fs.bclk <= not mclk_cnt(1);
    audioclks_reg.clk_64fs.bclk  <= not mclk_cnt(2);
    audioclks_reg.fsclk_50 <= not mclk_cnt(8);
        end if;
    end process;

    process (clk)

    begin
        if rising_edge(clk) then
            if (mclk_cnt = 0) then
                        audioclks_reg.clk_256fs.fsclk_tdm <= '1';
                        audioclks_reg.clk_128fs.fsclk_tdm <= '1';
                        audioclks_reg.clk_64fs.fsclk_tdm <= '1';
                    elsif (mclk_cnt = 2) then
                        audioclks_reg.clk_256fs.fsclk_tdm <= '0';
                    elsif (mclk_cnt = 4) then
                        audioclks_reg.clk_128fs.fsclk_tdm <= '0';
                    elsif (mclk_cnt = 8) then
                        audioclks_reg.clk_64fs.fsclk_tdm <= '0';
                    end if;
        end if;
    end process;

    
    process (clk)
        variable cntsub : unsigned(8 downto 0);
    begin
        if rising_edge(clk) then
            cntsub := mclk_cnt + 2;
            audioclks_reg.clk_256fs.fsclk_i2s_50 <= not cntsub(8);
            

            if (cntsub = 0) then
                audioclks_reg.clk_256fs.fsclk_i2s_tdm <= '1';
            elsif (cntsub = 2) then
                audioclks_reg.clk_256fs.fsclk_i2s_tdm <= '0';
            end if;
            cntsub := mclk_cnt + 3;
            audioclks_reg.clk_256fs.fsclk_i2s_50_dac <= not cntsub(8);
        end if;
    end process;
    process (clk)
        variable cntsub : unsigned(8 downto 0);
    begin
        if rising_edge(clk) then
            cntsub := mclk_cnt + 4;
            audioclks_reg.clk_128fs.fsclk_i2s_50 <= not cntsub(8);
            if (cntsub = 0) then
                audioclks_reg.clk_128fs.fsclk_i2s_tdm <= '1';
            elsif (cntsub = 4) then
                audioclks_reg.clk_128fs.fsclk_i2s_tdm <= '0';
            end if;
            cntsub := mclk_cnt + 6;
            audioclks_reg.clk_128fs.fsclk_i2s_50_dac <= not cntsub(8);
        end if;
    end process;
    process (clk)
        variable cntsub : unsigned(8 downto 0);
    begin
        if rising_edge(clk) then
            cntsub := mclk_cnt + 8;
            audioclks_reg.clk_64fs.fsclk_i2s_50 <= not cntsub(8);
            if (cntsub = 0) then
                audioclks_reg.clk_64fs.fsclk_i2s_tdm <= '1';
            elsif (cntsub = 8) then
                audioclks_reg.clk_64fs.fsclk_i2s_tdm <= '0';
            end if;
             cntsub := mclk_cnt + 12;
            audioclks_reg.clk_64fs.fsclk_i2s_50_dac <= not cntsub(8);
        end if;
    end process;
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
    -- media_clock is now a *pure function* of the wallclock -- no NCO
    -- dependency at all. Every cycle:
    --   media_base    = sec_reg(31:0) * audio_fs                  (Stage 0)
    --   media_mult    = nsec_reg * MEDIA_CLK_RECIP                (Stage 1)
    --   media_clock   = media_base_reg + media_mult(55:40)        (Stage 2)
    -- RECIP is in Q26 form (= audio_fs * 2^(32+8) / 1e9 rounded), so the
    -- top 16 bits of the product sit at [55:40], not [47:32] as in the
    -- previous Q18 version. This means media_clock_reg(0) toggles at the
    -- wallclock sample boundary -- bit-exactly synchronous across all
    -- PTP-locked boards. It is the reference the NCO is pulled towards.
    media_proc: process(clk, reset_n)
        variable sample_in_sec  : unsigned(15 downto 0);  -- 0..47999
    begin
        if reset_n = '0' then
            media_clock_reg  <= (others => '0');
            media_base       <= (others => '0');
            media_mult_reg   <= (others => '0');
            media_clock_proc_wait <= '0';
        elsif rising_edge(clk) then
            if (media_clock_proc_wait = '0') then

            media_clock_nsec_latch <= unsigned(nsec_reg(29 downto 0));
            media_sec_latch <= unsigned(signed(sec_reg(31 downto 0))
                                        + to_signed(sec_adj_pipe, 32));

            media_base <= resize(media_sec_latch
                                 * to_unsigned(audio_fs, 32), 32);
            media_mult_reg <= media_clock_nsec_latch * MEDIA_CLK_RECIP;
            -- extract sample_in_sec from bits [55:40] of the
            -- Q26 product (top 16 bits = integer part 0..47999)
            sample_in_sec := media_mult_reg(55 downto 40);
            media_clock_reg <= media_base + resize(sample_in_sec, 32);
            end if;
            media_clock_proc_wait <= not media_clock_proc_wait;
        end if;
    end process media_proc;

    -- ============================================================
    -- Main Wallclock Timekeeping Process (5-stage pipeline)
    --
    -- Pipeline stages (signal reads get PREVIOUS cycle's value):
    --   Stage 1: freq_correction → resize → frac_increment_reg  (trivial)
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

    process (clk, reset_n)
    begin
        if (reset_n = '0') then
            new_frac <= (others => '0');
        elsif rising_edge(clk) then
            new_frac <= resize(frac_ns_accum, 29)
                          + resize(wallclock_signals.freq_correction_ppb_i, 29);
        end if;
    end process;


    process (clk, reset_n)

    begin 
        if (reset_n = '0') then
            frac_ns_accum_minus <= (others => '0');
            frac_ns_accum_plus <= (others => '0');
        elsif rising_edge(clk) then
            frac_ns_accum_minus <= resize(new_frac - FRAC_UNDERFLOW, 28);
            frac_ns_accum_plus <= resize(new_frac - FRAC_OVERFLOW, 28);
        end if;
    end process;

    process (clk, reset_n)
    begin
        if reset_n = '0' then
            frac_ns_accum      <= (others => '0');
            ns_adjust_pipe     <= 0;
        elsif rising_edge(clk) then
            
                ns_adjust_pipe <= 0;  -- default

                if new_frac >= resize(FRAC_OVERFLOW, 29) then
                    frac_ns_accum  <= frac_ns_accum_plus;
                    ns_adjust_pipe <= 1;
                elsif new_frac <= resize(FRAC_UNDERFLOW, 29) then
                    frac_ns_accum  <= frac_ns_accum_minus;
                    ns_adjust_pipe <= -1;
                else
                    frac_ns_accum <= resize(new_frac, 28);
                end if;
        end if;
    end process;
    process(clk, reset_n)
    begin
        if reset_n = '0' then
            second_pulse_int   <= '0';
            nsec_reg           <= (others => '0');
            sec_reg            <= (others => '0');
            ns_increment_reg   <= to_signed(increment_interval, 32);
            new_nsec_pipe      <= (others => '0');
            new_nsec_minus_sec <= -NS_PER_SEC;
            new_nsec_plus_sec  <= NS_PER_SEC;
            sec_adj_pipe       <= 0;

        elsif rising_edge(clk) then
            second_pulse_int <= '0';

            -- ===== STAGE 1: Latch ppb correction (no multiply needed) =====
            -- freq_correction changes at PTP rate — 1 cycle delay invisible.
            -- The previous version multiplied ppb by increment_interval (= 8)
            -- here, but the FRAC_OVERFLOW threshold carried the same factor,
            -- so the two cancelled out. Dropping the multiply shrinks the
            -- operand width that the Stage-2 adder/compare/subtractor see,
            -- which is the dominant critical path on this clock.
            

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



            -- ===== OVERRIDE PATHS =====
            if wallclock_set_3 = '0' and wallclock_set_2 = '1' then
                -- Hard set of time — overrides everything.

                nsec_reg      <= signed(resize(unsigned(wallclock_signals.wallclock_nanoseconds_i), 32));
                sec_reg       <= unsigned(wallclock_signals.wallclock_seconds_i);
                new_nsec_pipe  <= signed(resize(unsigned(wallclock_signals.wallclock_nanoseconds_i), 32));
                new_nsec_minus_sec <= signed(resize(unsigned(wallclock_signals.wallclock_nanoseconds_i), 32)) - NS_PER_SEC;
                new_nsec_plus_sec  <= signed(resize(unsigned(wallclock_signals.wallclock_nanoseconds_i), 32)) + NS_PER_SEC;
                sec_adj_pipe   <= 0;

            
            elsif wallclock_phasejump_2 = '1' and wallclock_phasejump_3 = '0' then
                nsec_reg      <= nsec_reg + resize(signed(wallclock_signals.wallclock_nanoseconds_i), 32);
                sec_reg       <= unsigned(signed(sec_reg) + signed(wallclock_signals.wallclock_seconds_i));
                new_nsec_pipe  <= new_nsec_pipe + resize(signed(wallclock_signals.wallclock_nanoseconds_i), 32);
                new_nsec_minus_sec <= new_nsec_minus_sec + resize(signed(wallclock_signals.wallclock_nanoseconds_i), 32) - NS_PER_SEC;
                new_nsec_plus_sec  <= new_nsec_plus_sec + resize(signed(wallclock_signals.wallclock_nanoseconds_i), 32) + NS_PER_SEC;
                sec_adj_pipe   <= 0;

            else
                -- ===== STAGE 2: Fractional accumulation + overflow =====
                -- Uses frac_increment_reg from Stage 1 of PREVIOUS cycle.
                -- 29-bit add to avoid overflow at range limits; result is
                -- always brought back into the ±FRAC_OVERFLOW band before
                -- being stored in the 28-bit frac_ns_accum.
                

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
