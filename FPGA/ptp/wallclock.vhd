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
        ms_pulse_o              : out std_logic;


        clk_256fs_o             : out std_logic;  -- fs * 256
        clk_128fs_o             : out std_logic;  -- fs * 128
        clk_64fs_o              : out std_logic;  -- fs * 64  (= BCLK for I2S)
        fs_o                    : out std_logic;  -- fs       (= LRCK, 50 % duty)
        bclk_r_o                : out std_logic;  -- 256fs sampled on NCO rising  edge
        bclk_f_o                : out std_logic;  -- 256fs sampled on NCO falling edge
        fs_tdm_pulse_o          : out std_logic;   -- fs frame sync, 1 BCLK wide
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
        nco_inc_dbg_o     : out signed(31 downto 0)   -- top 32b of nco_increment
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

    -- Pipeline register to break freq_correction → nco_increment critical path
    -- Original: 32×16 multiply + shift + add (~10ns combined)
    -- Split: Stage 1 (multiply) → Stage 2 (add). No shift now — the result
    -- is consumed at full 48-bit precision.
    signal ppb_adj_reg    : signed(NCO_PHASE_BITS - 1 downto 0) := (others => '0');
    
    -- Sample divider: count 512 MCLK rising edges → 1 sample period.
    -- MCLK MSB rising edge = one MCLK cycle. 512 cycles = 1 fs period.
    signal mclk_cnt      : unsigned(8 downto 0) := (others => '0');
    signal sample_pulse_int : std_logic := '0';

    signal clk_256fs_r   : std_logic := '0';
    signal clk_128fs_r   : std_logic := '0';
    signal clk_64fs_r    : std_logic := '0';
    signal fs_r          : std_logic := '0';
    signal bclk_r_r      : std_logic := '0';
    signal bclk_f_r      : std_logic := '0';
    -- TDM frame sync: held high for 2 NCO rising edges (= 1 BCLK).
    signal fs_tdm_r      : std_logic := '0';
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

    -- 32-bit nsec * 26-bit RECIP = 58-bit product
    signal media_mult_reg   : unsigned(57 downto 0) := (others => '0');
    signal media_base       : unsigned(31 downto 0) := (others => '0');


    signal media_clock_prev : unsigned(31 downto 0) := (others => '0');
    signal media_edge_tick  : std_logic;



    constant PULL_LOOP_GAIN_SHIFT : natural := 10;
    constant PHASE_ERR_TO_BIAS_SHIFT : natural :=
        NCO_PHASE_BITS - 9 - PULL_LOOP_GAIN_SHIFT;  -- 48-9-10 = 29
    constant PULL_INT_GAIN_SHIFT  : natural := 30;
    constant PULL_DEAD_BAND       : natural := 4;


    constant PPB_TRIM_LIMIT_32 : integer :=
        integer(real(audio_fs * 512) / real(sys_clk_hz)
                * 4294967296.0 * 200.0e-6);
    constant PPB_TRIM_LIMIT : signed(NCO_PHASE_BITS - 1 downto 0) :=
        shift_left(to_signed(PPB_TRIM_LIMIT_32, NCO_PHASE_BITS), 16);
    signal ppb_trim : signed(NCO_PHASE_BITS - 1 downto 0) := (others => '0');

    signal nco_phase_bias : signed(NCO_PHASE_BITS - 1 downto 0) := (others => '0');
    

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

begin

    -- Output assignments
    wallclock_nanoseconds_o <= unsigned(nsec_reg);
    wallclock_seconds_o     <= sec_reg;
    second_pulse_o          <= second_pulse_int;
    audio_mclk_o            <= std_logic(nco_phase(NCO_PHASE_BITS - 1));  -- NCO MSB = MCLK at ~24.576 MHz
    media_clock_o           <= media_clock_reg;
    sample_pulse_o          <= sample_pulse_int;
    ms_pulse_o              <= ms_pulse_int;

    clk_256fs_o    <= clk_256fs_r;
    clk_128fs_o    <= clk_128fs_r;
    clk_64fs_o     <= clk_64fs_r;
    fs_o           <= fs_r;
    bclk_r_o       <= bclk_r_r;
    bclk_f_o       <= bclk_f_r;
    fs_tdm_pulse_o <= fs_tdm_r;

    -- Diagnostic outputs (see port declaration)
    mclk_cnt_o         <= mclk_cnt;
    media_edge_tick_o  <= media_edge_tick;
    -- Show ppb_trim shifted right by 16, so the visible 32-bit value
    -- maps 1:1 with ppb_adj_reg's scaling (both are in NCO-base-32 units
    -- after the shift). With +/-200 ppm clamp, ppb_trim's 48-bit value
    -- saturates at ~+/-1.1e10, which truncated to 32 bits flips sign.
    -- Shifting first keeps the displayed value within signed-32 range.
    ppb_adj_dbg_o      <= ppb_adj_reg(31 downto 0);
    ppb_trim_dbg_o     <= resize(shift_right(ppb_trim, 16), 32);
    -- nco_phase_bias = phase_err << PHASE_ERR_TO_BIAS_SHIFT; recover
    -- phase_err by shifting back. This shows the *current-cycle* P-term
    -- (= 0 except for one cycle following each media_edge_tick).
    bias_dbg_o         <= resize(shift_right(nco_phase_bias,
                                  PHASE_ERR_TO_BIAS_SHIFT), 32);
    sample_pulse_int_o <= sample_pulse_int;
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
    -- ===== Phase-pull edge detector =====
    -- media_clock_reg is a monotonically increasing sample index. A
    -- sample boundary = "value just changed". Compare against the
    -- previous-cycle value rather than just bit(0) (which toggles at
    -- fs/2 = 24 kHz, not fs — that mistake meant the pull loop only
    -- saw every second boundary and at the wrong cadence).
    media_edge_tick <= '1' when (media_clock_reg /= media_clock_prev)
                       else '0';


    nco_bias_proc: process (clk, reset_n)
        -- 11-bit signed: holds -1024..+1023, large enough for both
        -- halves of mclk_cnt 0..511 expressed as a signed phase error
        -- and for the +512 constant used in the upper-half formula
        -- (which doesn't fit in 10-bit signed).
        variable phase_err  : signed(10 downto 0);
        variable trim_step  : signed(NCO_PHASE_BITS - 1 downto 0);
        variable trim_next  : signed(NCO_PHASE_BITS - 1 downto 0);
    begin
        if reset_n = '0' then
            nco_phase_bias <= (others => '0');
            ppb_trim       <= (others => '0');
            phase_locked_o <= '0';
        elsif rising_edge(clk) then
            nco_phase_bias <= (others => '0');
            -- ppb_trim is persistent (integrator) - no default assignment.
            if wallclock_set_i = '1' or phase_jump_valid_i = '1' then
                -- Hard resync zeroes the NCO; clear the integrator too so
                -- it doesn't carry old drift compensation across the jump.
                ppb_trim <= (others => '0');
            elsif media_edge_tick = '1' then
                -- Compute signed phase error in mclk-ticks. Applied on
                -- EVERY edge (no dead band): a dead band would let
                -- residual XO drift push the NCO out of band, then snap
                -- it back when it crosses the threshold. With a
                -- continuous PI loop, the integrator absorbs the drift
                -- so the P-term sees ~0 steady-state error.
                if mclk_cnt <= to_unsigned(255, 9) then
                    -- Lower half: NCO ahead by mclk_cnt ticks.
                    -- For mclk_cnt = 0 this gives 0 (no bias).
                    phase_err := -signed(resize(mclk_cnt, 11));
                else
                    -- Upper half: NCO behind by (512 - mclk_cnt).
                    phase_err := to_signed(512, 11)
                               - signed(resize(mclk_cnt, 11));
                end if;
                -- P-term: one-shot bias on nco_increment for this cycle.

                if phase_err > to_signed(PULL_DEAD_BAND, 11)
                   or phase_err < to_signed(-PULL_DEAD_BAND, 11) then
                    nco_phase_bias <=
                        shift_left(resize(phase_err, NCO_PHASE_BITS),
                                   PHASE_ERR_TO_BIAS_SHIFT);
                end if;

                -- I-term: accumulate phase_err into ppb_trim with clamp.
                trim_step := shift_left(resize(phase_err, NCO_PHASE_BITS),
                                        PULL_INT_GAIN_SHIFT);
                trim_next := ppb_trim + trim_step;
                if trim_next > PPB_TRIM_LIMIT then
                    ppb_trim <= PPB_TRIM_LIMIT;
                elsif trim_next < -PPB_TRIM_LIMIT then
                    ppb_trim <= -PPB_TRIM_LIMIT;
                else
                    ppb_trim <= trim_next;
                end if;

                -- phase_locked_o is purely a status flag; the dead
                -- band only governs whether we declare lock, not
                -- whether we apply correction.
                if mclk_cnt <= to_unsigned(2, 9)
                   or mclk_cnt >= to_unsigned(509, 9) then
                    phase_locked_o <= '1';
                else
                    phase_locked_o <= '0';
                end if;
            end if;
        end if;
    end process;

    nco_increment <= NCO_BASE_INC_48 + ppb_adj_reg + ppb_trim + nco_phase_bias;

    nco_proc: process(clk, reset_n)
        variable v_new_cnt : unsigned(8 downto 0);
    begin
        if reset_n = '0' then
            nco_phase          <= (others => '0');
            nco_phase_prev     <= '0';
            mclk_cnt           <= (others => '0');
            sample_pulse_int   <= '0';
            media_clock_prev   <= (others => '0');
            clk_256fs_r        <= '0';
            clk_128fs_r        <= '0';
            clk_64fs_r         <= '0';
            fs_r               <= '0';
            bclk_r_r           <= '0';
            fs_tdm_r           <= '0';
            fs_tdm_cnt         <= (others => '0');
        elsif rising_edge(clk) then
            sample_pulse_int <= '0';
            media_clock_prev <= media_clock_reg;

            -- ===== Phase-pull bias (computed in nco_bias_proc) =====
            -- nco_phase_bias is a one-shot signed bias added to
            -- nco_increment for one cycle on every media_edge_tick. The
            -- magnitude is proportional to the phase error (see
            -- nco_bias_proc). Sign convention:
            --   mclk_cnt in [3..255]   -> NCO AHEAD  -> bias < 0
            --   mclk_cnt in [256..508] -> NCO BEHIND -> bias > 0
            -- The dead band {0,1,2,509,510,511} sets phase_locked='1'.

            -- nco_increment = base + ppb-correction + phase-pull-bias.
            -- All three are recomputed every cycle so the phase-pull
            -- bias is a true one-shot kick — adding it to the previous
            -- nco_increment instead would let the bias accumulate
            -- across cycles whenever ppb_adj_reg's update was held off
            -- by nco_ppb_adj_wait, producing a phase-dependent
            -- frequency offset. ppb_adj_reg itself only changes every
            -- other cycle (see nco_ppb_adj_proc) but reading the held
            -- value here is fine.
            
            -- ===== Hard resync on wallclock_set / phase_jump =====
            -- These are large-step events. A pull loop would take too
            -- long to converge from arbitrary phase, so reset the NCO
            -- to align with the wallclock fs edge that media_proc will
            -- produce ~3 cycles later. The pull loop then maintains it.
            if wallclock_set_i = '1' or phase_jump_valid_i = '1' then
                nco_phase      <= (others => '0');
                nco_phase_prev <= '0';
                mclk_cnt       <= (others => '0');
            else
                nco_phase <= unsigned(signed(nco_phase) + nco_increment);
                nco_phase_prev <= std_logic(nco_phase(NCO_PHASE_BITS - 1));
                if nco_phase_prev = '0' and nco_phase(NCO_PHASE_BITS - 1) = '1' then
                    -- NCO rising edge: advance divider counter.
                    if mclk_cnt = 511 then
                        v_new_cnt        := (others => '0');
                        sample_pulse_int <= '1';
                        -- Frame boundary: arm TDM frame sync for 1 BCLK.
                        fs_tdm_r   <= '1';
                        fs_tdm_cnt <= to_unsigned(2, 2);
                    else
                        v_new_cnt := mclk_cnt + 1;
                        if fs_tdm_cnt /= 0 then
                            fs_tdm_cnt <= fs_tdm_cnt - 1;
                            if fs_tdm_cnt = 1 then
                                fs_tdm_r <= '0';
                            end if;
                        end if;
                    end if;
                    mclk_cnt <= v_new_cnt;

                    -- Drive sub-clocks from the post-increment counter.
                    -- fs rises with sample_pulse (cnt = 0 → fs = '1'),
                    -- so fs is the inverted MSB of v_new_cnt.
                    clk_256fs_r <= v_new_cnt(0);
                    clk_128fs_r <= v_new_cnt(1);
                    clk_64fs_r  <= v_new_cnt(2);
                    fs_r        <= not v_new_cnt(8);
                    bclk_r_r    <= v_new_cnt(0);
                end if;
            end if;
        end if;
    end process nco_proc;

    -- bclk_f: like bclk_r_r but updated on NCO falling edges
    -- (half-period shift), giving a phase-shifted 256fs for DACs that
    -- want data captured on the opposite BCLK edge.
    bclk_f_proc: process(clk, reset_n)
    begin
        if reset_n = '0' then
            bclk_f_r <= '0';
        elsif rising_edge(clk) then
            if nco_falling_tick = '1' then
                bclk_f_r <= mclk_cnt(0);
            end if;
        end if;
    end process bclk_f_proc;

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
        elsif rising_edge(clk) then
            -- Stage 0: register sec*audio_fs (cheap -- sec_reg changes at 1 Hz)
            media_base <= resize(sec_reg(31 downto 0)
                                 * to_unsigned(audio_fs, 32), 32);
            -- Stage 1: multiply nsec * RECIP (32x26 = 58 bits)
            media_mult_reg <= unsigned(nsec_reg) * MEDIA_CLK_RECIP;
            -- Stage 2: extract sample_in_sec from bits [55:40] of the
            -- Q26 product (top 16 bits = integer part 0..47999)
            sample_in_sec := media_mult_reg(55 downto 40);
            media_clock_reg <= media_base + resize(sample_in_sec, 32);
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
            -- Full-width signed multiply (no narrowing of ppb): the
            -- previous resize(ppb, 20) capped ppb at +/-2^19 = +/-524k,
            -- silently truncating any larger correction. Multiplier core
            -- is signed(12) * signed(32) = signed(44); we resize to 32
            -- after the multiply since frac_ns_accum is 32-bit and
            -- 8 * 250e6 = 2e9 still fits.
            frac_increment_reg <= resize(to_signed(increment_interval, 12)
                                         * freq_correction_ppb_i, 32);

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
