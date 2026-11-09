-- ============================================================
-- wallclock_nco: PTP-disciplined MCLK NCO (MCLK = fs * 512)
--
-- 48-bit phase accumulator: 32 integer bits + 16 fractional bits.
-- Bit 47 (MSB) toggles at MCLK frequency.
-- Increment per 125 MHz tick (in 48-bit units):
--   24576000/125e6 * 2^48 = NCO_BASE_INC_48 ~ 5.534e13
--
--
-- Frequency: hardware PTP -> freq_correction_ppb_i * NCO_PPB_SCALE,
-- averaged over 8 updates; software PTP -> nco_ppb_adj_i (48-bit
-- increment offset, taken over on each toggle of nco_ppb_adj_valid_i).
--
-- Phase: on every media_clock step (wallclock sample boundary) the
-- position of the NCO's sub-sample counter is the phase error; it is
-- applied as a held proportional bias on the increment (phase pull).
--
-- Pipeline (to meet timing):
--   Stage 1: ppb_adj_reg <= freq_correction * NCO_PPB_SCALE  (full width)
--   Stage 2: nco_increment <= NCO_BASE_INC_48 + ppb_adj + phase bias
-- ============================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity wallclock_nco is
    generic(
        sys_clk_hz           : natural;
        audio_fs             : natural;
        pull_loop_gain_shift : natural;
        PTP_IN_SOFTWARE      : boolean
    );
    port(
        clk                   : in  std_logic;
        reset_n               : in  std_logic;

        freq_correction_ppb_i : in  signed(19 downto 0);
        nco_ppb_adj_i         : in  signed(47 downto 0);
        nco_ppb_adj_valid_i   : in  std_logic;   -- toggle = new value (async)
        -- wallclock set event: restarts the NCO and the phase pull
        set_i                 : in  std_logic;
        -- wallclock-derived media clock (phase reference)
        media_clock_i         : in  unsigned(31 downto 0);

        mclk_o                : out std_logic;   -- NCO MSB
        mclk_cnt_o            : out unsigned(8 downto 0);  -- MCLK cycle in sample
        sample_pulse_o        : out std_logic;   -- mclk_cnt wrap
        phase_locked_o        : out std_logic;
        media_edge_tick_o     : out std_logic;   -- media_clock stepped

        -- debug
        ppb_adj_dbg_o         : out signed(31 downto 0);
        bias_dbg_o            : out signed(31 downto 0);
        phase_dbg_o           : out unsigned(31 downto 0);
        inc_dbg_o             : out signed(31 downto 0)
    );
end entity wallclock_nco;

architecture rtl of wallclock_nco is
    constant NCO_PHASE_BITS : natural := 48;
    constant NCO_FRAC_BITS  : natural := 16;  -- 48 - 32


    constant NCO_BASE_INC_32 : signed(31 downto 0) :=
        to_signed(integer(real(audio_fs * 512) / real(sys_clk_hz)
                          * 4294967296.0 + 0.5), 32);

    constant NCO_BASE_INC_48 : signed(NCO_PHASE_BITS - 1 downto 0) :=
        shift_left(resize(NCO_BASE_INC_32, NCO_PHASE_BITS), NCO_FRAC_BITS);


    constant NCO_PPB_SCALE : signed(17 downto 0) :=
        to_signed(integer(real(audio_fs * 512) / real(sys_clk_hz)
                          * 4294967296.0 / 1.0e9 * 65536.0),
                  18);

    constant PHASE_ERR_TO_BIAS_SHIFT : natural :=
        NCO_PHASE_BITS - 9 - pull_loop_gain_shift;
    constant PULL_DEAD_BAND : natural := 1;

    signal nco_phase         : unsigned(NCO_PHASE_BITS - 1 downto 0) := (others => '0');
    signal nco_phase_prev    : std_logic := '0';  -- previous MSB for edge detect
    signal nco_increment     : signed(NCO_PHASE_BITS - 1 downto 0) := NCO_BASE_INC_48;
    signal phase_err         : signed(10 downto 0) := (others => '0');
    signal phase_error_valid : std_logic := '0';
    signal nco_phase_bias    : signed(NCO_PHASE_BITS - 1 downto 0) := (others => '0');
    signal nco_phase_locked  : std_logic := '0';

    -- Stage-1 register: breaks freq_correction -> nco_increment path
    signal ppb_adj_reg       : signed(NCO_PHASE_BITS - 1 downto 0) := (others => '0');
    signal ppb_adj_reg_last  : signed(NCO_PHASE_BITS - 1 downto 0) := (others => '0');
    signal ppb_adj_reg_avg   : signed(NCO_PHASE_BITS - 1 downto 0) := (others => '0');
    signal ppb_adj_reg_valid : std_logic;
    signal nco_ppb_adj_wait  : std_logic := '0';

    signal mcu_nco_adj_valid_sync1 : std_logic;
    signal mcu_nco_adj_valid_sync2 : std_logic;
    signal mcu_nco_adj_valid_sync3 : std_logic;

    -- Sample divider: 512 MCLK rising edges = 1 sample period
    signal nco_mclk_cnt     : unsigned(8 downto 0) := (others => '0');
    signal nco_sample_pulse : std_logic := '0';

    signal media_clock_prev : unsigned(31 downto 0) := (others => '0');
    signal media_edge_tick  : std_logic;
begin

    mclk_o            <= std_logic(nco_phase(NCO_PHASE_BITS - 1));
    mclk_cnt_o        <= nco_mclk_cnt;
    sample_pulse_o    <= nco_sample_pulse;
    phase_locked_o    <= nco_phase_locked;
    media_edge_tick_o <= media_edge_tick;

    ppb_adj_dbg_o <= ppb_adj_reg(31 downto 0);
    -- nco_phase_bias = phase_err << PHASE_ERR_TO_BIAS_SHIFT; recover
    bias_dbg_o    <= resize(shift_right(nco_phase_bias,
                                        PHASE_ERR_TO_BIAS_SHIFT), 32);
    phase_dbg_o   <= nco_phase(47 downto 16);
    inc_dbg_o     <= nco_increment(47 downto 16);

    -- ============================================================
    -- Frequency: ppb -> increment offset
    -- ============================================================
    nco_ppb_adj_proc: process(clk, reset_n)
    begin
        if reset_n = '0' then
            nco_ppb_adj_wait <= '0';
            ppb_adj_reg      <= (others => '0');
        elsif rising_edge(clk) then
            if (PTP_IN_SOFTWARE = true) then
                mcu_nco_adj_valid_sync1 <= nco_ppb_adj_valid_i;
                mcu_nco_adj_valid_sync2 <= mcu_nco_adj_valid_sync1;
                mcu_nco_adj_valid_sync3 <= mcu_nco_adj_valid_sync2;
                if mcu_nco_adj_valid_sync2 /= mcu_nco_adj_valid_sync3 then
                    ppb_adj_reg <= nco_ppb_adj_i;
                end if;
            else
                if (nco_ppb_adj_wait = '0') then
                    nco_ppb_adj_wait <= '1';
                    ppb_adj_reg_last <= ppb_adj_reg;
                    ppb_adj_reg <= resize(freq_correction_ppb_i * NCO_PPB_SCALE,
                                          NCO_PHASE_BITS);
                else
                    nco_ppb_adj_wait <= '0';
                end if;
            end if;
        end if;
    end process;

    ppb_adj_reg_valid <= '1' when ppb_adj_reg /= ppb_adj_reg_last
                                  and nco_ppb_adj_wait = '1' else '0';

    gen_nco_avg : if (PTP_IN_SOFTWARE = false) generate
        average_inst: entity work.average
            generic map(
                DATA_WIDTH => NCO_PHASE_BITS,
                DEPTH      => 8
            )
            port map(
                clk_i        => clk,
                rst_n_i      => reset_n,
                data_i       => ppb_adj_reg,
                data_valid_i => ppb_adj_reg_valid,
                data_o       => ppb_adj_reg_avg
            );
    end generate;

    -- ============================================================
    -- Phase: NCO position at each wallclock sample boundary
    -- ============================================================
    media_edge_tick <= '1' when (media_clock_i /= media_clock_prev) else '0';

    nco_bias_proc: process (clk, reset_n)
    begin
        if reset_n = '0' then
            phase_error_valid <= '0';
        elsif rising_edge(clk) then
            phase_error_valid <= '0';

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

    nco_p_proc: process (clk, reset_n)
    begin
        if (reset_n = '0') then
            nco_phase_bias   <= (others => '0');
            nco_phase_locked <= '0';
        elsif rising_edge(clk) then
            if set_i = '1' then
                nco_phase_bias   <= (others => '0');
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

    -- ============================================================
    -- Accumulator
    -- ============================================================
    nco_increment_proc: process(clk, reset_n)
    begin
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
            nco_phase        <= (others => '0');
            nco_phase_prev   <= '0';
            nco_mclk_cnt     <= (others => '0');
            nco_sample_pulse <= '0';
            media_clock_prev <= (others => '0');
        elsif rising_edge(clk) then
            nco_sample_pulse <= '0';
            media_clock_prev <= media_clock_i;

            if set_i = '1' then
                nco_phase      <= (others => '0');
                nco_phase_prev <= '0';
                nco_mclk_cnt   <= (others => '0');
            else
                nco_phase      <= unsigned(signed(nco_phase) + nco_increment);
                nco_phase_prev <= std_logic(nco_phase(NCO_PHASE_BITS - 1));
                if nco_phase_prev = '0' and nco_phase(NCO_PHASE_BITS - 1) = '1' then
                    -- NCO rising edge: advance divider counter
                    nco_mclk_cnt <= nco_mclk_cnt + 1;
                    if nco_mclk_cnt = 511 then
                        nco_sample_pulse <= '1';
                    end if;
                end if;
            end if;
        end if;
    end process nco_proc;

end architecture rtl;
