-- ============================================================
-- wallclock_vcxo_ctl: PI controller + mode / lock state of the VCXO loop
--
-- Runs once per phase-detector window, one arithmetic operation per
-- clock (it has a whole window of time). Output is a charge-pump
-- command: pump_len pulses on the fine (pd_a) or coarse (pd_b) path,
-- spread over the next window by wallclock_vcxo_pump.
--
--   PI    : velocity form, the loop-filter capacitor is the
--           accumulator. pump = Kp*(e[n]-e[n-1]) + Ki*e[n] + b
--   b     : second integrator (Kb) that learns the pump rate needed to
--           cancel loop-filter leakage; frozen while slipping.
--   fast  : after >= 2 saturated windows the coarse path pumps FAST_PUMP
--           cycles per window until the error starts shrinking.
--   lock  : 32 consecutive windows below 1/4 MCLK; lost above 1 MCLK.
--
-- Pump quantities are Q8 sys_clk cycles (VCXO_QP), phase Q16 (VCXO_Q).
-- ============================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.wallclock_vcxo_pkg.all;

entity wallclock_vcxo_ctl is
    generic(
        WIN_CYC   : natural;   -- sys_clk cycles per pump window
        KP_SHIFT  : integer;
        KI_SHIFT  : integer;
        KB_SHIFT  : integer;
        FAST_PUMP : natural
    );
    port(
        clk          : in  std_logic;
        reset_n      : in  std_logic;
        -- wallclock set / phase jump: mask this and the next window
        realign_i    : in  std_logic;
        win_done_i   : in  std_logic;
        win_mean_i   : in  signed(VCXO_PH_W - 1 downto 0);
        win_slip_i   : in  std_logic;

        -- still in the realign hold-off (phase detector: coarse slips)
        holdoff_o    : out std_logic;
        pump_start_o : out std_logic;                    -- new command
        pump_len_o   : out natural range 0 to WIN_CYC;   -- pulses this window
        pump_up_o    : out std_logic;                    -- direction
        pump_fast_o  : out std_logic;                    -- coarse path (pd_b)
        locked_o     : out std_logic;

        err_dbg_o    : out signed(VCXO_PH_W - 1 downto 0);  -- window mean error
        pump_dbg_o   : out signed(31 downto 0)              -- signed pump cycles
    );
end entity wallclock_vcxo_ctl;

architecture rtl of wallclock_vcxo_ctl is
    constant Q     : natural := VCXO_Q;
    constant QP    : natural := VCXO_QP;
    constant PH_W  : natural := VCXO_PH_W;
    constant ACC_W : natural := VCXO_ACC_W;

    constant PUMP_MAX : signed(ACC_W - 1 downto 0) :=
        shift_left(to_signed(WIN_CYC, ACC_W), QP);
    -- window counts as saturated above this (fast-mode trigger)
    constant SAT_THR  : signed(PH_W - 1 downto 0) := to_signed(16 * 2**Q, PH_W);
    -- |e[n]-e[n-1]| beyond this saturates the pump anyway (Kp*DE_MAX
    -- = 2 windows of pumping); clamping it first keeps p_term small.
    function de_max_q return integer is
        variable r : real;
    begin
        r := 2.0 * real(WIN_CYC) / 2.0**KP_SHIFT * 2.0**Q;
        if r < 1.0 then
            return 1;
        elsif r > 4194304.0 then   -- 2^22
            return 4194304;
        end if;
        return integer(r);
    end function;
    constant DE_MAX   : signed(PH_W - 1 downto 0) := to_signed(de_max_q, PH_W);
    constant FAST_N   : natural := min_nat(FAST_PUMP, WIN_CYC);

    constant LOCK_THR    : signed(PH_W - 1 downto 0) := to_signed(2**Q / 4, PH_W); -- 1/4 MCLK
    constant UNLOCK_THR  : signed(PH_W - 1 downto 0) := to_signed(2**Q, PH_W);     -- 1 MCLK
    constant EXIT_MARGIN : signed(PH_W - 1 downto 0) := to_signed(2**Q / 256, PH_W);

    signal holdoff     : unsigned(1 downto 0) := "10";
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

    signal pump_start  : std_logic := '0';
    signal pump_len    : natural range 0 to WIN_CYC := 0;
    signal pump_up     : std_logic := '0';
    signal pump_on_b   : std_logic := '0';
begin
    assert KI_SHIFT <= 17 and KB_SHIFT <= 17
        report "wallclock: VCXO gain shift overflows the controller width"
        severity failure;

    holdoff_o    <= '1' when holdoff /= 0 else '0';
    pump_start_o <= pump_start;
    pump_len_o   <= pump_len;
    pump_up_o    <= pump_up;
    pump_fast_o  <= pump_on_b;
    locked_o     <= vcxo_locked;
    err_dbg_o    <= e_mean;
    pump_dbg_o   <= pump_cmd;

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
            if realign_i = '1' then
                holdoff <= "10";
            end if;

            case ctl_step is
                when 0 =>
                    if win_done_i = '1' then
                        e_mean      <= win_mean_i;
                        ae          <= abs_s(win_mean_i);
                        w_slip      <= win_slip_i;
                        w_slip_prev <= w_slip;
                        if holdoff /= 0 or realign_i = '1' then
                            w_invalid <= '1';
                            if realign_i = '0' then
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
                    i_term <= shift_signed(e_mean, KI_SHIFT + QP - Q, ACC_W);
                    b_add  <= shift_signed(e_mean, KB_SHIFT + QP - Q, ACC_W);
                    ctl_step <= 3;

                when 3 =>
                    if p_ok = '1' then
                        p_term <= shift_signed(de, KP_SHIFT + QP - Q, ACC_W);
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

end architecture rtl;
