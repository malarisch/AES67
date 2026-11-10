-- ============================================================
-- wallclock_time: PTP time base (48-bit seconds + nanoseconds)
--
-- Advances the wallclock by increment_interval ns per clk, trimmed by
-- freq_correction_ppb_i through a fractional-nanosecond accumulator.
-- set_i loads an absolute time, jump_i adds a signed offset (both are
-- single-cycle events, synchronised by the caller).
--
-- Pipeline (signal reads get the PREVIOUS cycle's value):
--   Stage 1: new_frac = frac_ns_accum + freq_correction (29-bit add)
--   Stage 2: frac overflow -> ns_adjust_pipe (+-1 ns)
--   Stage 2.5: ns_increment_reg = increment_interval + ns_adjust_pipe
--   Stage 3a: new_nsec_pipe (+ pre-computed +-1e9 variants) += increment
--   Stage 3b: compare against 1e9 / 0 -> nsec_reg, sec_adj_pipe
--   Stage 4: sec_adj_pipe -> 48-bit seconds increment
--
-- Latency: 4 extra cycles (32 ns) from freq_correction change to
-- sec_reg. This is invisible: freq_correction changes at <= 128 Hz
-- (millions of cycles apart), and the phase/second updates being a few
-- cycles late are negligible for PTP accuracy requirements.
-- ============================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity wallclock_time is
    generic(
        -- nanoseconds added per clock tick (125 MHz -> 8 ns)
        increment_interval : natural;
        sys_clk_hz         : natural
    );
    port(
        clk                   : in  std_logic;
        reset_n               : in  std_logic;

        freq_correction_ppb_i : in  signed(19 downto 0);
        set_i                 : in  std_logic;   -- load seconds_i/nanoseconds_i
        jump_i                : in  std_logic;   -- add seconds_i/nanoseconds_i (signed)
        seconds_i             : in  std_logic_vector(47 downto 0);
        nanoseconds_i         : in  std_logic_vector(29 downto 0);

        nsec_o                : out unsigned(29 downto 0);
        sec_o                 : out unsigned(47 downto 0);
        -- Pending seconds rollover (+1/-1), applied to sec_o next cycle.
        -- Lets consumers build a coherent (sec, nsec) pair.
        sec_adj_o             : out integer range -1 to 1;
        second_pulse_o        : out std_logic
    );
end entity wallclock_time;

architecture rtl of wallclock_time is
    constant NS_PER_SEC : signed(31 downto 0) := to_signed(1_000_000_000, 32);

    -- Main time registers
    signal nsec_reg : signed(31 downto 0) := (others => '0');
    signal sec_reg  : unsigned(47 downto 0) := (others => '0');

    -- Fractional nanosecond accumulator for sub-nanosecond precision.
    -- Range: bounded by +-FRAC_OVERFLOW = +-sys_clk_hz = +-125e6, which
    -- fits in 28-bit signed (max +-134M). Narrowing from 32 to 28 bits
    -- shortens the Stage-2 adder/compare/subtractor carry chain by
    -- ~4 bits worth of LUT delay -- the dominant critical path here.
    signal frac_ns_accum_minus : signed(27 downto 0) := (others => '0');
    signal frac_ns_accum_plus  : signed(27 downto 0) := (others => '0');
    signal frac_ns_accum       : signed(27 downto 0) := (others => '0');
    signal new_frac            : signed(28 downto 0) := (others => '0');

    -- The ppb correction is accumulated directly (no multiply by
    -- increment_interval): an earlier version multiplied here, but the
    -- overflow threshold carried the same factor, so the two cancelled.
    -- Dropping it shrinks the operand width of the Stage-2 critical path.
    constant FRAC_OVERFLOW  : signed(27 downto 0) := to_signed(sys_clk_hz, 28);
    constant FRAC_UNDERFLOW : signed(27 downto 0) := to_signed(-sys_clk_hz, 28);

    signal second_pulse_int : std_logic := '0';

    signal ns_adjust_pipe   : integer range -1 to 1 := 0;
    -- Pre-computed increment value (breaks ns_adjust_pipe -> new_nsec critical path)
    signal ns_increment_reg : signed(31 downto 0) := to_signed(increment_interval, 32);
    signal new_nsec_pipe    : signed(31 downto 0) := (others => '0');  -- Stage 3a output
    signal sec_adj_pipe     : integer range -1 to 1 := 0;

    -- Pre-computed rollover values to break the Stage 3b timing path.
    -- Invariant: new_nsec_minus_sec = new_nsec_pipe - NS_PER_SEC,
    --            new_nsec_plus_sec  = new_nsec_pipe + NS_PER_SEC
    signal new_nsec_minus_sec : signed(31 downto 0) := -NS_PER_SEC;
    signal new_nsec_plus_sec  : signed(31 downto 0) := NS_PER_SEC;
begin
    assert real(increment_interval) * real(sys_clk_hz) = 1.0e9
        report "wallclock: increment_interval (ns) does not match sys_clk_hz"
        severity failure;

    -- Take the low 30 bits of nsec_reg (nanoseconds are always 0..1e9-1).
    nsec_o         <= unsigned(nsec_reg(29 downto 0));
    sec_o          <= sec_reg;
    sec_adj_o      <= sec_adj_pipe;
    second_pulse_o <= second_pulse_int;

    -- ===== STAGE 1: fractional accumulation =====
    -- 29-bit add to avoid overflow at the range limits; the result is
    -- always brought back into the +-FRAC_OVERFLOW band before being
    -- stored in the 28-bit frac_ns_accum.
    frac_add_proc: process (clk, reset_n)
    begin
        if (reset_n = '0') then
            new_frac <= (others => '0');
        elsif rising_edge(clk) then
            new_frac <= resize(frac_ns_accum, 29)
                          + resize(freq_correction_ppb_i, 29);
        end if;
    end process;

    -- Both wrap-around candidates in parallel with the compare below
    frac_wrap_proc: process (clk, reset_n)
    begin
        if (reset_n = '0') then
            frac_ns_accum_minus <= (others => '0');
            frac_ns_accum_plus  <= (others => '0');
        elsif rising_edge(clk) then
            frac_ns_accum_minus <= resize(new_frac - FRAC_UNDERFLOW, 28);
            frac_ns_accum_plus  <= resize(new_frac - FRAC_OVERFLOW, 28);
        end if;
    end process;

    -- ===== STAGE 2: fractional overflow -> +-1 ns =====
    frac_ovf_proc: process (clk, reset_n)
    begin
        if reset_n = '0' then
            frac_ns_accum  <= (others => '0');
            ns_adjust_pipe <= 0;
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

    time_proc: process(clk, reset_n)
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

            -- ===== STAGE 4: Apply delayed seconds rollover =====
            -- sec_adj_pipe was written by Stage 3b in the PREVIOUS cycle.
            if sec_adj_pipe = 1 then
                sec_reg <= sec_reg + 1;
                second_pulse_int <= '1';
            elsif sec_adj_pipe = -1 then
                sec_reg <= sec_reg - 1;
            end if;
            -- Default for this cycle (may be overwritten below)
            sec_adj_pipe <= 0;

            -- ===== OVERRIDE PATHS =====
            if set_i = '1' then
                -- Hard set of time -- overrides everything.
                nsec_reg           <= signed(resize(unsigned(nanoseconds_i), 32));
                sec_reg            <= unsigned(seconds_i);
                new_nsec_pipe      <= signed(resize(unsigned(nanoseconds_i), 32));
                new_nsec_minus_sec <= signed(resize(unsigned(nanoseconds_i), 32)) - NS_PER_SEC;
                new_nsec_plus_sec  <= signed(resize(unsigned(nanoseconds_i), 32)) + NS_PER_SEC;
                sec_adj_pipe       <= 0;

            elsif jump_i = '1' then
                nsec_reg           <= nsec_reg + resize(signed(nanoseconds_i), 32);
                sec_reg            <= unsigned(signed(sec_reg) + signed(seconds_i));
                new_nsec_pipe      <= new_nsec_pipe + resize(signed(nanoseconds_i), 32);
                new_nsec_minus_sec <= new_nsec_minus_sec + resize(signed(nanoseconds_i), 32) - NS_PER_SEC;
                new_nsec_plus_sec  <= new_nsec_plus_sec + resize(signed(nanoseconds_i), 32) + NS_PER_SEC;
                sec_adj_pipe       <= 0;

            else
                -- ===== STAGE 2.5: Pre-compute increment value =====
                -- ns_adjust_pipe is from Stage 2 of the PREVIOUS cycle.
                -- Breaks the ns_adjust_pipe -> new_nsec -> new_nsec_minus_sec chain.
                ns_increment_reg <= to_signed(increment_interval + ns_adjust_pipe, 32);

                -- ===== STAGE 3a: Update all three values in PARALLEL =====
                -- Uses ns_increment_reg from the PREVIOUS cycle; no chain.
                new_nsec_pipe      <= new_nsec_pipe + ns_increment_reg;
                new_nsec_minus_sec <= new_nsec_minus_sec + ns_increment_reg;
                new_nsec_plus_sec  <= new_nsec_plus_sec + ns_increment_reg;

                -- ===== STAGE 3b: Compare and MUX (minimal arithmetic) =====
                -- On rollover, reset all three values to keep the
                -- invariant. Constants are fine for minus/plus since after
                -- rollover the value is near 0 or 1e9.
                if new_nsec_pipe >= NS_PER_SEC then
                    nsec_reg           <= new_nsec_minus_sec;
                    new_nsec_pipe      <= new_nsec_minus_sec;
                    new_nsec_minus_sec <= -NS_PER_SEC;             -- ~0 - 1e9
                    new_nsec_plus_sec  <= NS_PER_SEC;              -- ~0 + 1e9
                    sec_adj_pipe       <= 1;
                elsif new_nsec_pipe < 0 then
                    nsec_reg           <= new_nsec_plus_sec;
                    new_nsec_pipe      <= new_nsec_plus_sec;
                    new_nsec_minus_sec <= (others => '0');         -- ~1e9 - 1e9
                    new_nsec_plus_sec  <= NS_PER_SEC + NS_PER_SEC; -- ~1e9 + 1e9
                    sec_adj_pipe       <= -1;
                else
                    nsec_reg <= new_nsec_pipe;
                end if;
            end if;
        end if;
    end process;

end architecture rtl;
