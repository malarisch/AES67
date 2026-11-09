-- ============================================================
-- wallclock_vcxo_pd: VCXO MCLK counter + phase detector
--
-- Synchronises the VCXO into clk, counts its edges (0..511 per sample;
-- this count drives every divided audio clock in VCXO mode) and
-- compares each edge against the wallclock: nsec at the edge, converted
-- to MCLK cycles mod 512, minus the VCXO count = phase error (Q16).
-- Errors beyond the slip threshold slip the counter by one MCLK. The
-- error is averaged over 2^AVG_LOG2 edges, which turns the 8 ns
-- sampling quantisation into ~ps resolution.
-- ============================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.wallclock_vcxo_pkg.all;

entity wallclock_vcxo_pd is
    generic(
        sys_clk_hz    : natural;
        audio_fs      : natural;
        AVG_LOG2      : natural;
        -- pins run on the VCXO-domain clocks (shifts the reference offset)
        DOMAIN_CLOCKS : boolean
    );
    port(
        clk            : in  std_logic;
        reset_n        : in  std_logic;
        vcxo_clk_i     : in  std_logic;                -- async to clk
        nsec_i         : in  unsigned(29 downto 0);    -- wallclock nanoseconds
        -- realigning (boot, wallclock set / phase jump): snap the counter
        -- to the nearest MCLK edge instead of saturating
        coarse_i       : in  std_logic;

        mclk_o         : out std_logic;                -- synchronised VCXO
        tick_o         : out std_logic;                -- VCXO rising edge
        cnt_o          : out unsigned(8 downto 0);     -- MCLK cycle in sample
        sample_pulse_o : out std_logic;                -- cnt wrap

        win_done_o     : out std_logic;                -- 1-cycle, window complete
        win_mean_o     : out signed(VCXO_PH_W - 1 downto 0);  -- mean error, Q16
        win_slip_o     : out std_logic                 -- window contained a slip
    );
end entity wallclock_vcxo_pd;

architecture rtl of wallclock_vcxo_pd is
    constant MCLK_HZ : real    := real(audio_fs * 512);
    constant Q       : natural := VCXO_Q;
    constant PH_W    : natural := VCXO_PH_W;
    constant SUM_W   : natural := PH_W + AVG_LOG2 + 1;

    -- nsec -> MCLK cycles: 512*fs/1e9 * 2^36 (31 bit). Product bits
    -- [44:20] = Q16 cycles mod 512. Rounding error 1.5e-10 -> at most
    -- 0.004 MCLK cycles (0.15 ns) at the end of each second.
    constant RECIP_SHIFT : natural := 36;
    constant MCLK_RECIP  : unsigned(30 downto 0) :=
        to_unsigned(integer(MCLK_HZ / 1.0e9 * 2.0**RECIP_SHIFT + 0.5), 31);
    constant RECIP_LO    : unsigned(17 downto 0) := MCLK_RECIP(17 downto 0);
    constant RECIP_HI    : unsigned(12 downto 0) := MCLK_RECIP(30 downto 18);

    -- Latency of the measured edge vs. the reference: 2.5 sys_clk
    -- periods (6.0 with DOMAIN_CLOCKS), in Q16 MCLK cycles
    function ref_ofs_clks return real is
    begin
        if DOMAIN_CLOCKS then
            -- pins run one MCLK behind the internal count, see
            -- wallclock_vcxo_domain
            return 6.0 - real(sys_clk_hz) / real(audio_fs * 512);
        end if;
        return 2.5;
    end function;
    constant REF_OFS  : unsigned(PH_W - 1 downto 0) :=
        to_unsigned(integer(ref_ofs_clks * MCLK_HZ / real(sys_clk_hz)
                            * 2.0**Q + 0.5), PH_W);
    constant CLAMP    : signed(PH_W - 1 downto 0) := to_signed(32 * 2**Q, PH_W);
    constant HALF_CYC : signed(PH_W - 1 downto 0) := to_signed(2**Q / 2, PH_W);
    constant ONE_CYC  : signed(PH_W - 1 downto 0) := to_signed(2**Q, PH_W);

    -- synchroniser / edge detect
    signal vcxo_meta   : std_logic := '0';
    signal vcxo_sync   : std_logic := '0';
    signal vcxo_sync_d : std_logic := '0';
    attribute PRESERVE : boolean;
    attribute PRESERVE of vcxo_meta : signal is true;
    attribute PRESERVE of vcxo_sync : signal is true;
    signal vcxo_tick   : std_logic;

    -- VCXO-driven MCLK counter. vcxo_cnt = vcxo_raw + slip_ofs; the PD
    -- snapshots vcxo_raw and adds the *current* slip_ofs when it
    -- computes the error, so a slip decided while later edges are still
    -- in the pipeline is never counted twice.
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
    signal win_cnt      : unsigned(AVG_LOG2 - 1 downto 0) := (others => '0');
    signal e_sum        : signed(SUM_W - 1 downto 0) := (others => '0');
    signal win_slip     : std_logic := '0';
    signal win_done     : std_logic := '0';
    signal win_mean     : signed(PH_W - 1 downto 0) := (others => '0');
    signal win_had_slip : std_logic := '0';
begin
    assert real(sys_clk_hz) > 2.2 * MCLK_HZ
        report "wallclock: sys_clk too slow to sample the VCXO"
        severity failure;

    mclk_o         <= vcxo_sync;
    tick_o         <= vcxo_tick;
    cnt_o          <= vcxo_cnt;
    sample_pulse_o <= vcxo_spulse;
    win_done_o     <= win_done;
    win_mean_o     <= win_mean;
    win_slip_o     <= win_had_slip;

    vcxo_tick <= vcxo_sync and not vcxo_sync_d;

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
            -- nsec_i is the time of the previous clk edge, i.e. 1.5
            -- sys_clk (mean) after the VCXO edge -> part of REF_OFS.
            if vcxo_tick = '1' then
                pd_nsec <= nsec_i;
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

            -- While realigning snap the counter to the nearest MCLK
            -- edge; afterwards saturate.
            if coarse_i = '1' then
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
            win_cnt      <= (others => '0');
            e_sum        <= (others => '0');
            win_slip     <= '0';
            win_done     <= '0';
            win_mean     <= (others => '0');
            win_had_slip <= '0';
        elsif rising_edge(clk) then
            win_done <= '0';
            if pd_v(8) = '1' then
                v_sum   := e_sum + resize(pd_err_used, SUM_W);
                win_cnt <= win_cnt + 1;
                if win_cnt = 2**AVG_LOG2 - 1 then
                    win_mean     <= resize(shift_right(v_sum, AVG_LOG2), PH_W);
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

end architecture rtl;
