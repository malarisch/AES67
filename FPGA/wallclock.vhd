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
        -- PTP-disciplined Audio Clocks (NCO-direct, no PLL)
        -- All outputs are register-driven from sys_clk domain.
        -- Jitter: ±1 sys_clk period (±8ns @125MHz).
        -- This is acceptable for I2S codecs (2.5% of BCLK period).
        -- The NCO is disciplined via freq_correction_ppb, so these
        -- clocks track PTP time with sub-PPB accuracy.
        -- ============================================
        audio_bclk_o            : out std_logic;  -- I2S bit clock, fs*64 = 3.072 MHz
        audio_lrck_o            : out std_logic;  -- I2S L/R clock, fs = 48 kHz

        -- ============================================
        -- Media clock for RTP packets (32-bit, from PTP epoch)
        -- Computed from wallclock: media_clock = (sec*48000 + sample_in_sec)
        -- ============================================
        media_clock_o           : out unsigned(31 downto 0);
        -- Pulse at fs rate in sys_clk domain (rising edge of LRCK)
        sample_pulse_o          : out std_logic
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
    
    -- Constants for overflow detection (use 2^30 as threshold)
    constant FRAC_OVERFLOW  : signed(31 downto 0) := to_signed(2**30, 32);
    constant FRAC_UNDERFLOW : signed(31 downto 0) := to_signed(-(2**30), 32);
    
    -- Internal second pulse (usable by other processes)
    signal second_pulse_int : std_logic := '0';
    
    -- ============================================================
    -- NCO for direct Audio Clock generation (BCLK = fs * 64)
    -- ============================================================
    -- Phase accumulator: 32-bit, MSB toggles at NCO frequency.
    -- NCO freq = fs * 64 = 3.072 MHz (for 48 kHz).
    -- Increment per 125 MHz tick: 3072000/125e6 * 2^32 = 105,553,116
    -- Jitter: ±8ns (1 sys_clk period). At BCLK period = 325ns → 2.5%.
    -- Standard I2S codecs (CS4272, PCM1808 etc.) tolerate >5% jitter.
    -- ============================================================
    constant NCO_BASE_INC : signed(31 downto 0) := 
        to_signed(integer(real(audio_fs * 64) / real(sys_clk_hz) * 4294967296.0), 32);
    
    -- PPB scaling: how much to adjust NCO increment per PPB of correction.
    -- adj = NCO_BASE_INC * ppb / 1e9 ≈ (ppb * NCO_PPB_SCALE) >> 16
    -- For 105553116 / 1e9 * 2^16 = 6917
    constant NCO_PPB_SCALE : signed(15 downto 0) := 
        to_signed(integer(real(audio_fs * 64) / real(sys_clk_hz) * 4294967296.0 / 1.0e9 * 65536.0), 16);
    
    signal nco_phase      : unsigned(31 downto 0) := (others => '0');
    signal nco_phase_prev : std_logic := '0';  -- Previous MSB for edge detect
    signal nco_increment  : signed(31 downto 0) := NCO_BASE_INC;
    
    -- LRCK divider: count 64 BCLK rising edges → 1 sample period
    -- BCLK MSB rising edge = one BCLK cycle. 64 cycles = 1 LRCK period.
    -- Counter counts 0..63, toggles LRCK at 0 and 32 (50% duty cycle).
    signal bclk_cnt      : unsigned(5 downto 0) := (others => '0');
    signal lrck_reg      : std_logic := '0';
    signal sample_pulse_int : std_logic := '0';
    
    -- ============================================================
    -- Media Clock (AES67): epoch-aligned, computed from wallclock
    -- media_clock = (seconds * audio_fs + sample_in_second) mod 2^32
    -- where sample_in_second = floor(nanoseconds * audio_fs / 1e9)
    -- Using reciprocal multiplication:
    --   sample_in_sec = (nanoseconds * K) >> 32  where K = audio_fs * 2^32 / 1e9
    --   For 48kHz: K = 48000 * 4294967296 / 1e9 = 206158
    -- ============================================================
    constant MEDIA_CLK_RECIP : unsigned(17 downto 0) :=
        to_unsigned(integer(real(audio_fs) * 4294967296.0 / 1.0e9), 18);

    -- Base media clock: seconds contribution (updated on second pulse)
    signal media_base       : unsigned(31 downto 0) := (others => '0');
    signal media_clock_reg  : unsigned(31 downto 0) := (others => '0');
    signal media_init_done  : std_logic := '0';
    
begin

    -- Output assignments
    wallclock_nanoseconds_o <= unsigned(nsec_reg);
    wallclock_seconds_o     <= sec_reg;
    second_pulse_o          <= second_pulse_int;
    audio_bclk_o            <= std_logic(nco_phase(31));  -- NCO MSB = BCLK at ~3.072 MHz
    audio_lrck_o            <= lrck_reg;                   -- LRCK at ~48 kHz
    media_clock_o           <= media_clock_reg;
    sample_pulse_o          <= sample_pulse_int;

    -- ============================================================
    -- NCO Process: Generate PTP-disciplined audio clocks
    -- BCLK (fs*64 = 3.072 MHz): NCO phase accumulator MSB
    -- LRCK (fs = 48 kHz): divide BCLK by 64
    -- Both are PTP-disciplined via freq_correction_ppb.
    -- ============================================================
    nco_proc: process(clk, reset_n)
        variable ppb_adj : signed(31 downto 0);
    begin
        if reset_n = '0' then
            nco_phase        <= (others => '0');
            nco_phase_prev   <= '0';
            nco_increment    <= NCO_BASE_INC;
            bclk_cnt         <= (others => '0');
            lrck_reg         <= '0';
            sample_pulse_int <= '0';
        elsif rising_edge(clk) then
            sample_pulse_int <= '0';
            
            -- Update NCO increment from PTP frequency correction
            -- adj = NCO_BASE_INC * ppb / 1e9 ≈ (ppb * NCO_PPB_SCALE) >> 16
            ppb_adj := resize(
                shift_right(freq_correction_ppb_i * NCO_PPB_SCALE, 16), 32);
            nco_increment <= NCO_BASE_INC + ppb_adj;
            
            -- Advance NCO phase accumulator
            nco_phase <= unsigned(signed(nco_phase) + nco_increment);
            nco_phase_prev <= std_logic(nco_phase(31));
            
            -- Detect rising edge of BCLK (NCO MSB 0→1)
            if nco_phase_prev = '0' and nco_phase(31) = '1' then
                -- Count 64 BCLK cycles per LRCK period
                if bclk_cnt = 63 then
                    bclk_cnt         <= (others => '0');
                    lrck_reg         <= '0';  -- Left channel starts
                    sample_pulse_int <= '1';  -- One sys_clk pulse at sample boundary
                elsif bclk_cnt = 31 then
                    bclk_cnt <= bclk_cnt + 1;
                    lrck_reg <= '1';  -- Right channel starts (50% duty cycle)
                else
                    bclk_cnt <= bclk_cnt + 1;
                end if;
            end if;
        end if;
    end process nco_proc;

    -- ============================================================
    -- Media Clock Computation Process (AES67)
    -- Epoch-aligned: media_clock = (sec * 48000 + sample_in_sec) mod 2^32
    -- ============================================================
    media_proc: process(clk, reset_n)
        variable ns_times_recip : unsigned(49 downto 0);  -- 32 × 18 = 50 bits
        variable sample_in_sec  : unsigned(15 downto 0);  -- 0..47999
    begin
        if reset_n = '0' then
            media_base      <= (others => '0');
            media_clock_reg <= (others => '0');
            media_init_done <= '0';
        elsif rising_edge(clk) then
            if wallclock_set_i = '1' then
                -- Hard set: compute base from new seconds value
                media_base <= resize(
                    wallclock_seconds_i(15 downto 0) * to_unsigned(audio_fs, 16), 32);
                media_init_done <= '1';
            elsif second_pulse_int = '1' then
                -- Every second: advance base by audio_fs samples
                media_base <= media_base + to_unsigned(audio_fs, 32);
            end if;

            -- sample_in_sec = floor(nanoseconds * MEDIA_CLK_RECIP / 2^32)
            ns_times_recip := unsigned(nsec_reg) * MEDIA_CLK_RECIP;
            sample_in_sec  := ns_times_recip(47 downto 32);

            -- Output: base + offset within second
            media_clock_reg <= media_base + resize(sample_in_sec, 32);
        end if;
    end process media_proc;

    -- ============================================================
    -- Main Wallclock Timekeeping Process
    -- ============================================================
    process(clk, reset_n)
        variable frac_increment : signed(31 downto 0);
        variable new_frac       : signed(31 downto 0);
        variable ns_adjust      : integer range -1 to 1;
        variable new_nsec       : signed(31 downto 0);
        variable sec_inc        : integer range -1 to 1;
    begin
        if reset_n = '0' then
            second_pulse_int <= '0';
            nsec_reg       <= (others => '0');
            sec_reg        <= (others => '0');
            frac_ns_accum  <= (others => '0');

        elsif rising_edge(clk) then
            second_pulse_int <= '0';
            sec_inc := 0;

            if wallclock_set_i = '1' then
                -- Hard set of time
                nsec_reg <= signed(wallclock_nanoseconds_i);
                sec_reg  <= wallclock_seconds_i;
                frac_ns_accum <= (others => '0');
                
            elsif phase_jump_valid_i = '1' then
                -- Apply one-time phase correction
                new_nsec := nsec_reg + resize(phase_jump_ns_i, 32);
                
                -- Handle wrap-around
                if new_nsec >= NS_PER_SEC then
                    nsec_reg <= new_nsec - NS_PER_SEC;
                    sec_reg  <= sec_reg + 1;
                    second_pulse_int <= '1';
                elsif new_nsec < 0 then
                    nsec_reg <= new_nsec + NS_PER_SEC;
                    sec_reg  <= sec_reg - 1;
                else
                    nsec_reg <= new_nsec;
                end if;
                
            else
                -- ============================================
                -- Normal tick with frequency correction
                -- ============================================
                -- PPB correction: frac_increment = increment_interval * ppb
                -- Scale: when frac_ns_accum reaches ±2^30, adjust by ±1 ns
                -- This gives effective resolution of ~1 PPB
                
                -- Calculate fractional increment: increment_interval * ppb
                -- For ±500,000 PPB max: 8 * 500,000 = 4M, fits in 32 bits
                -- Use 12-bit * 20-bit = 32-bit multiply (no truncation)
                -- freq(19:0) covers ±524,287 PPB — sufficient for servo's ±500,000 limit
                frac_increment := to_signed(increment_interval, 12) *
                                  freq_correction_ppb_i(19 downto 0);
                
                -- Accumulate and check overflow
                new_frac := frac_ns_accum + frac_increment;
                ns_adjust := 0;
                
                if new_frac >= FRAC_OVERFLOW then
                    new_frac := new_frac - FRAC_OVERFLOW;
                    ns_adjust := 1;
                elsif new_frac <= FRAC_UNDERFLOW then
                    new_frac := new_frac - FRAC_UNDERFLOW;  -- Subtracting negative = adding
                    ns_adjust := -1;
                end if;
                
                frac_ns_accum <= new_frac;
                
                -- Update nanoseconds
                new_nsec := nsec_reg + to_signed(increment_interval + ns_adjust, 32);
                
                -- Handle second rollover
                if new_nsec >= NS_PER_SEC then
                    nsec_reg <= new_nsec - NS_PER_SEC;
                    sec_reg  <= sec_reg + 1;
                    second_pulse_int <= '1';
                elsif new_nsec < 0 then
                    -- Can happen with negative adjustment at nsec=0
                    nsec_reg <= new_nsec + NS_PER_SEC;
                    sec_reg  <= sec_reg - 1;
                else
                    nsec_reg <= new_nsec;
                end if;
            end if;
        end if;
    end process;

end Behavioral;
