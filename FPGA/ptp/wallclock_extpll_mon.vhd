-- ============================================================
-- wallclock_extpll_mon: lock / realign state for an external PLL MCLK
--
-- (MCLK_SRC_EXT_PLL) The external PLL (e.g. CS2000) tracks the NCO MCLK
-- by itself; nothing is steered from here. Its output has an unknown,
-- slowly wandering phase vs. the wallclock sample grid (PLL offset,
-- re-lock, NCO phase-pull wander of ~+-1 MCLK). wallclock_vcxo_pd
-- measures that phase per window; this block only decides when the
-- MCLK counter may snap to the nearest MCLK edge:
--
--   realign  : after reset and wallclock set / phase jump; after 2
--              consecutive windows with |error| > RESNAP_CYC or a slip
--              (lost lock); and after RECENTER_WINDOWS consecutive
--              windows with |error| > 1.5 MCLK (PLL re-locked at
--              another phase, slow drift). The PD then slips the counter
--              to the nearest edge for this and the next window.
--   otherwise: the counter is frozen (PD slip threshold at its clamp),
--              so the sample grid never jumps while the PLL is fine.
--   lock     : LOCK_WINDOWS consecutive valid windows within RESNAP_CYC.
-- ============================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.wallclock_vcxo_pkg.all;

entity wallclock_extpll_mon is
    port(
        clk        : in  std_logic;
        reset_n    : in  std_logic;
        -- wallclock set / phase jump
        realign_i  : in  std_logic;
        win_done_i : in  std_logic;
        win_mean_i : in  signed(VCXO_PH_W - 1 downto 0);
        win_slip_i : in  std_logic;

        -- realigning (phase detector: snap to the nearest MCLK edge)
        holdoff_o  : out std_logic;
        locked_o   : out std_logic;
        err_dbg_o  : out signed(VCXO_PH_W - 1 downto 0)   -- last window mean
    );
end entity wallclock_extpll_mon;

architecture rtl of wallclock_extpll_mon is
    constant Q    : natural := VCXO_Q;
    constant PH_W : natural := VCXO_PH_W;

    -- Grid offset (MCLK cycles) beyond which the lock counts as lost and
    -- the counter re-snaps after 2 windows. A re-snap shifts the sample
    -- grid by whole MCLK cycles, so the normal offset (PLL's static phase
    -- <= 0.5 after a snap, plus NCO phase-pull wander of ~+-1) must stay
    -- well below it.
    constant RESNAP_CYC   : natural := 4;
    constant RESNAP_THR   : signed(PH_W - 1 downto 0) := to_signed(RESNAP_CYC * 2**Q, PH_W);
    -- Persistent offset beyond the normal wander: re-center the grid,
    -- but only after RECENTER_WINDOWS in a row (~43 ms at the default
    -- 2^15-edge windows), so wander never triggers it.
    constant RECENTER_THR     : signed(PH_W - 1 downto 0) := to_signed(3 * 2**Q / 2, PH_W);
    constant RECENTER_WINDOWS : natural := 32;
    constant LOCK_WINDOWS     : natural := 16;

    signal holdoff  : unsigned(1 downto 0) := "10";
    signal bad_run  : std_logic := '0';
    signal far_cnt  : natural range 0 to RECENTER_WINDOWS - 1 := 0;
    signal lock_cnt : natural range 0 to LOCK_WINDOWS - 1 := 0;
    signal locked   : std_logic := '0';
    signal e_mean   : signed(PH_W - 1 downto 0) := (others => '0');
begin

    holdoff_o <= '1' when holdoff /= 0 else '0';
    locked_o  <= locked;
    err_dbg_o <= e_mean;

    mon_proc: process(clk, reset_n)
    begin
        if reset_n = '0' then
            holdoff  <= "10";
            bad_run  <= '0';
            far_cnt  <= 0;
            lock_cnt <= 0;
            locked   <= '0';
            e_mean   <= (others => '0');
        elsif rising_edge(clk) then
            if realign_i = '1' then
                holdoff  <= "10";
                bad_run  <= '0';
                far_cnt  <= 0;
                lock_cnt <= 0;
                locked   <= '0';
            end if;

            if win_done_i = '1' then
                e_mean <= win_mean_i;
                if holdoff /= 0 or realign_i = '1' then
                    -- window spans a realign: not valid
                    if realign_i = '0' then
                        holdoff <= holdoff - 1;
                    end if;
                    bad_run  <= '0';
                    far_cnt  <= 0;
                    lock_cnt <= 0;
                    locked   <= '0';
                elsif win_slip_i = '1' or abs_s(win_mean_i) > RESNAP_THR then
                    far_cnt  <= 0;
                    lock_cnt <= 0;
                    locked   <= '0';
                    if bad_run = '1' then
                        holdoff <= "10";    -- re-snap the counter
                        bad_run <= '0';
                    else
                        bad_run <= '1';
                    end if;
                elsif abs_s(win_mean_i) > RECENTER_THR and far_cnt = RECENTER_WINDOWS - 1 then
                    holdoff  <= "10";       -- re-center the grid
                    bad_run  <= '0';
                    far_cnt  <= 0;
                    lock_cnt <= 0;
                    locked   <= '0';
                else
                    bad_run <= '0';
                    if abs_s(win_mean_i) > RECENTER_THR then
                        far_cnt <= far_cnt + 1;
                    else
                        far_cnt <= 0;
                    end if;
                    if lock_cnt = LOCK_WINDOWS - 1 then
                        locked <= '1';
                    else
                        lock_cnt <= lock_cnt + 1;
                    end if;
                end if;
            end if;
        end if;
    end process;

end architecture rtl;
