library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- IEEE 1588 Best Master Clock Algorithm (BMA)
--
-- Tracks a single best-external-candidate from received Announce messages,
-- compares it against this node's own dataset, and drives the leader/follower
-- outputs. An external candidate is considered lost after 3 x announceInterval
-- seconds without a refreshing Announce; at that point the node falls back
-- to becoming leader if its own dataset is better than no candidate (trivially
-- true) or remains follower of nothing (is_leader=1).
--
-- Dataset comparison order per IEEE 1588 (lower value wins):
--   priority1 -> clockClass -> clockAccuracy -> offsetScaledLogVariance
--   -> priority2 -> clockIdentity (tie breaker)
--
-- Timing note:
-- The two dataset comparisons live in their own always-running processes
-- (P_CMP_ANN_EXT, P_CMP_EXT_SELF). Their results are registered into
-- ann_better_r / ext_beats_self_r before being consumed by the main FSM,
-- which now adds one cycle of wait state at the end of an Announce update.
-- The compare-to-register paths are declared multi-cycle in the SDC, which
-- lets the synthesis tool relax timing on those long comparators.
-- See FPGA SDC: set_multicycle_path on *|ann_better_r and *|ext_beats_self_r.

entity ptpv2_bmc is
    port (
        clk                : in  std_logic;
        reset_n            : in  std_logic;
        second_pulse_i     : in  std_logic;   -- 1 Hz tick (sys clk domain)

        -- Own dataset (local clock)
        my_clock_identity_i        : in std_logic_vector(63 downto 0);
        my_priority1_i             : in std_logic_vector(7 downto 0);
        my_clock_class_i           : in std_logic_vector(7 downto 0);
        my_clock_accuracy_i        : in std_logic_vector(7 downto 0);
        my_offset_scaled_log_var_i : in std_logic_vector(15 downto 0);
        my_priority2_i             : in std_logic_vector(7 downto 0);

        -- Announce message input (from parser)
        announce_valid_i                 : in std_logic;
        announce_clock_identity_i        : in std_logic_vector(63 downto 0);
        announce_priority1_i             : in std_logic_vector(7 downto 0);
        announce_clock_class_i           : in std_logic_vector(7 downto 0);
        announce_clock_accuracy_i        : in std_logic_vector(7 downto 0);
        announce_offset_scaled_log_var_i : in std_logic_vector(15 downto 0);
        announce_priority2_i             : in std_logic_vector(7 downto 0);
        announce_log_msg_interval_i      : in std_logic_vector(7 downto 0);

        -- BMC outputs
        is_leader_o          : out std_logic;
        is_follower_o        : out std_logic;
        current_leader_id_o  : out std_logic_vector(63 downto 0)
    );
end entity;

architecture rtl of ptpv2_bmc is

    -- Best external candidate slot
    signal ext_valid        : std_logic := '0';
    signal ext_clock_id     : std_logic_vector(63 downto 0) := (others => '0');
    signal ext_priority1    : std_logic_vector(7 downto 0) := (others => '1');
    signal ext_clock_class  : std_logic_vector(7 downto 0) := (others => '1');
    signal ext_clock_acc    : std_logic_vector(7 downto 0) := (others => '1');
    signal ext_oslv         : std_logic_vector(15 downto 0) := (others => '1');
    signal ext_priority2    : std_logic_vector(7 downto 0) := (others => '1');

    -- Timeout counter (seconds remaining before external candidate expires)
    signal ext_timeout_sec  : unsigned(7 downto 0) := (others => '0');

    -- Result registers
    signal is_leader_r       : std_logic := '1';
    signal is_follower_r     : std_logic := '0';
    signal current_leader_r  : std_logic_vector(63 downto 0) := (others => '0');

    -- Registered comparator outputs. The two processes P_CMP_ANN_EXT and
    -- P_CMP_EXT_SELF run the long dataset comparisons continuously and park
    -- the result here. The main FSM consumes them one cycle later.
    signal ann_better_r      : std_logic := '0';  -- announce strictly better than ext
    signal ext_beats_self_r  : std_logic := '0';  -- ext strictly better than self

    -- One-cycle wait state after updating ext from an Announce: gives the
    -- ext-vs-self comparator time to settle before the main FSM samples it.
    signal update_wait_r     : std_logic := '0';

    -- Decode signed log2(interval seconds) -> seconds (clamped to >=1),
    -- then multiply by 3 for the announceReceiptTimeout.
    function decode_timeout_sec(log_msg_interval : std_logic_vector(7 downto 0))
        return unsigned is
        variable s : signed(7 downto 0);
        variable secs : unsigned(7 downto 0);
    begin
        s := signed(log_msg_interval);
        if s <= 0 then
            secs := to_unsigned(1, 8);   -- sub-second or 1s -> treat as 1s
        elsif s = 1 then
            secs := to_unsigned(2, 8);
        elsif s = 2 then
            secs := to_unsigned(4, 8);
        elsif s = 3 then
            secs := to_unsigned(8, 8);
        elsif s = 4 then
            secs := to_unsigned(16, 8);
        else
            secs := to_unsigned(16, 8);  -- clamp large values
        end if;
        -- 3 x interval
        return resize(secs + secs + secs, 8);
    end function;

    -- Return '1' if dataset A is strictly better than dataset B
    -- (lower priority1, then clockClass, then clockAccuracy, then OSLV,
    --  then priority2, then clockIdentity).
    function dataset_better(
        a_prio1, a_class, a_acc, a_prio2 : std_logic_vector(7 downto 0);
        a_oslv                           : std_logic_vector(15 downto 0);
        a_id                             : std_logic_vector(63 downto 0);
        b_prio1, b_class, b_acc, b_prio2 : std_logic_vector(7 downto 0);
        b_oslv                           : std_logic_vector(15 downto 0);
        b_id                             : std_logic_vector(63 downto 0)
    ) return std_logic is
    begin
        if    unsigned(a_prio1) < unsigned(b_prio1) then return '1';
        elsif unsigned(a_prio1) > unsigned(b_prio1) then return '0';
        elsif unsigned(a_class) < unsigned(b_class) then return '1';
        elsif unsigned(a_class) > unsigned(b_class) then return '0';
        elsif unsigned(a_acc)   < unsigned(b_acc)   then return '1';
        elsif unsigned(a_acc)   > unsigned(b_acc)   then return '0';
        elsif unsigned(a_oslv)  < unsigned(b_oslv)  then return '1';
        elsif unsigned(a_oslv)  > unsigned(b_oslv)  then return '0';
        elsif unsigned(a_prio2) < unsigned(b_prio2) then return '1';
        elsif unsigned(a_prio2) > unsigned(b_prio2) then return '0';
        elsif unsigned(a_id)    < unsigned(b_id)    then return '1';
        else return '0';
        end if;
    end function;

begin

    is_leader_o         <= is_leader_r;
    is_follower_o       <= is_follower_r;
    current_leader_id_o <= current_leader_r;

    ------------------------------------------------------------------
    -- P_CMP_ANN_EXT
    -- Continuously compares the incoming Announce fields against the
    -- stored ext slot. Result is registered; downstream logic uses the
    -- register, not the combinational function output.
    -- This path is declared multi-cycle in the SDC.
    ------------------------------------------------------------------
    P_CMP_ANN_EXT : process (clk, reset_n)
    begin
        if reset_n = '0' then
            ann_better_r <= '0';
        elsif rising_edge(clk) then
            ann_better_r <= dataset_better(
                announce_priority1_i, announce_clock_class_i,
                announce_clock_accuracy_i, announce_priority2_i,
                announce_offset_scaled_log_var_i,
                announce_clock_identity_i,
                ext_priority1, ext_clock_class,
                ext_clock_acc, ext_priority2,
                ext_oslv, ext_clock_id);
        end if;
    end process;

    ------------------------------------------------------------------
    -- P_CMP_EXT_SELF
    -- Continuously compares the ext slot against the local dataset.
    -- Result is registered.
    -- This path is declared multi-cycle in the SDC.
    ------------------------------------------------------------------
    P_CMP_EXT_SELF : process (clk, reset_n)
    begin
        if reset_n = '0' then
            ext_beats_self_r <= '0';
        elsif rising_edge(clk) then
            ext_beats_self_r <= dataset_better(
                ext_priority1, ext_clock_class,
                ext_clock_acc, ext_priority2,
                ext_oslv, ext_clock_id,
                my_priority1_i, my_clock_class_i,
                my_clock_accuracy_i, my_priority2_i,
                my_offset_scaled_log_var_i, my_clock_identity_i);
        end if;
    end process;

    ------------------------------------------------------------------
    -- P_MAIN
    -- Uses the registered comparator outputs. After updating the ext
    -- slot from an Announce, a one-cycle wait state (update_wait_r)
    -- gives the ext-vs-self comparator time to re-evaluate before the
    -- leader/follower outputs are recomputed.
    ------------------------------------------------------------------
    P_MAIN : process (clk, reset_n)
        variable accept_announce : std_logic;
    begin
        if reset_n = '0' then
            ext_valid        <= '0';
            ext_clock_id     <= (others => '0');
            ext_priority1    <= (others => '1');
            ext_clock_class  <= (others => '1');
            ext_clock_acc    <= (others => '1');
            ext_oslv         <= (others => '1');
            ext_priority2    <= (others => '1');
            ext_timeout_sec  <= (others => '0');
            is_leader_r      <= '1';
            is_follower_r    <= '0';
            current_leader_r <= (others => '0');
            update_wait_r    <= '0';

        elsif rising_edge(clk) then

            -- default: wait state clears after one cycle
            update_wait_r <= '0';

            -- 1 Hz timeout countdown for external candidate
            if second_pulse_i = '1' and ext_valid = '1' then
                if ext_timeout_sec <= 1 then
                    ext_valid       <= '0';
                    ext_timeout_sec <= (others => '0');
                else
                    ext_timeout_sec <= ext_timeout_sec - 1;
                end if;
            end if;

            -- Incoming Announce: ignore self-announce, otherwise compare
            -- against stored best candidate (or install if slot empty).
            -- accept condition uses the *registered* ann_better_r flag.
            if announce_valid_i = '1' and
               announce_clock_identity_i /= my_clock_identity_i then

                if ext_valid = '0' then
                    accept_announce := '1';
                elsif announce_clock_identity_i = ext_clock_id then
                    -- same candidate refreshing itself
                    accept_announce := '1';
                else
                    accept_announce := ann_better_r;
                end if;

                if accept_announce = '1' then
                    ext_valid       <= '1';
                    ext_clock_id    <= announce_clock_identity_i;
                    ext_priority1   <= announce_priority1_i;
                    ext_clock_class <= announce_clock_class_i;
                    ext_clock_acc   <= announce_clock_accuracy_i;
                    ext_oslv        <= announce_offset_scaled_log_var_i;
                    ext_priority2   <= announce_priority2_i;
                    ext_timeout_sec <= decode_timeout_sec(announce_log_msg_interval_i);
                    update_wait_r   <= '1';  -- stall output update one cycle
                end if;
            end if;

            -- Recompute leader/follower from registered state. Skip during
            -- the wait state so ext_beats_self_r reflects the *new* ext.
            if update_wait_r = '0' then
                if ext_valid = '1' then
                    if ext_beats_self_r = '1' then
                        is_leader_r      <= '0';
                        is_follower_r    <= '1';
                        current_leader_r <= ext_clock_id;
                    else
                        is_leader_r      <= '1';
                        is_follower_r    <= '0';
                        current_leader_r <= my_clock_identity_i;
                    end if;
                else
                    is_leader_r      <= '1';
                    is_follower_r    <= '0';
                    current_leader_r <= my_clock_identity_i;
                end if;
            end if;

        end if;
    end process;

end architecture;
