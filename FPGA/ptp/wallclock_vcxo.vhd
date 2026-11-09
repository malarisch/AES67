-- ============================================================
-- wallclock_vcxo: external VCXO as MCLK, phase-locked to the wallclock
--
-- (wallclock MCLK_FROM_VCXO) The 24.576 MHz VCXO is counted in the
-- sys_clk domain and compared against the wallclock; a PI loop drives
-- the tri-state charge pump on its CV loop filter:
--
--   vcxo_clk_i -> wallclock_vcxo_pd   (counter, phase detector, window avg)
--              -> wallclock_vcxo_ctl  (PI + leak integrator, fast mode, lock)
--              -> wallclock_vcxo_pump (PDM pulses on pd_a / pd_b)
--
-- The VCXO count replaces the NCO count as mclk_cnt, so every divided
-- clock (bclk, fs, tdm frame) follows the VCXO but lands on the same
-- sample grid as an NCO board. With DOMAIN_CLOCKS the pin clocks come
-- from wallclock_vcxo_domain instead of the sys_clk clock tree.
-- ============================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.audioclks_pkg.all;
use work.wallclock_vcxo_pkg.all;

entity wallclock_vcxo is
    generic(
        sys_clk_hz    : natural;
        audio_fs      : natural;
        PD_AVG_LOG2   : natural;
        KP_SHIFT      : integer;
        KI_SHIFT      : integer;
        KB_SHIFT      : integer;
        FAST_PUMP     : natural;
        PUMP_INVERT   : boolean;
        DOMAIN_CLOCKS : boolean
    );
    port(
        clk             : in  std_logic;
        reset_n         : in  std_logic;
        vcxo_clk_i      : in  std_logic;
        nsec_i          : in  unsigned(29 downto 0);
        -- wallclock set / phase jump event
        realign_i       : in  std_logic;
        -- sys_clk-domain clock tree, used for the pins without DOMAIN_CLOCKS
        sys_clocks_i    : in  t_audio_clocks;

        mclk_o          : out std_logic;              -- synchronised VCXO
        mclk_cnt_o      : out unsigned(8 downto 0);
        sample_pulse_o  : out std_logic;
        locked_o        : out std_logic;
        pump_o          : out t_vcxo_pump;
        phase_err_dbg_o : out signed(31 downto 0);    -- window mean, Q16 MCLK
        pump_dbg_o      : out signed(31 downto 0);
        pin_clocks_o    : out t_audio_clocks
    );
end entity wallclock_vcxo;

architecture rtl of wallclock_vcxo is
    constant WIN_CYC : natural := vcxo_win_cyc(sys_clk_hz, audio_fs, PD_AVG_LOG2);

    signal holdoff    : std_logic;
    signal vcxo_tick  : std_logic;
    signal vcxo_cnt   : unsigned(8 downto 0);
    signal win_done   : std_logic;
    signal win_mean   : signed(VCXO_PH_W - 1 downto 0);
    signal win_slip   : std_logic;
    signal e_mean     : signed(VCXO_PH_W - 1 downto 0);
    signal pump_start : std_logic;
    signal pump_len   : natural range 0 to WIN_CYC;
    signal pump_up    : std_logic;
    signal pump_fast  : std_logic;
begin

    mclk_cnt_o      <= vcxo_cnt;
    phase_err_dbg_o <= resize(e_mean, 32);

    u_pd: entity work.wallclock_vcxo_pd
        generic map(
            sys_clk_hz    => sys_clk_hz,
            audio_fs      => audio_fs,
            AVG_LOG2      => PD_AVG_LOG2,
            DOMAIN_CLOCKS => DOMAIN_CLOCKS
        )
        port map(
            clk            => clk,
            reset_n        => reset_n,
            vcxo_clk_i     => vcxo_clk_i,
            nsec_i         => nsec_i,
            coarse_i       => holdoff,
            mclk_o         => mclk_o,
            tick_o         => vcxo_tick,
            cnt_o          => vcxo_cnt,
            sample_pulse_o => sample_pulse_o,
            win_done_o     => win_done,
            win_mean_o     => win_mean,
            win_slip_o     => win_slip
        );

    u_ctl: entity work.wallclock_vcxo_ctl
        generic map(
            WIN_CYC   => WIN_CYC,
            KP_SHIFT  => KP_SHIFT,
            KI_SHIFT  => KI_SHIFT,
            KB_SHIFT  => KB_SHIFT,
            FAST_PUMP => FAST_PUMP
        )
        port map(
            clk          => clk,
            reset_n      => reset_n,
            realign_i    => realign_i,
            win_done_i   => win_done,
            win_mean_i   => win_mean,
            win_slip_i   => win_slip,
            holdoff_o    => holdoff,
            pump_start_o => pump_start,
            pump_len_o   => pump_len,
            pump_up_o    => pump_up,
            pump_fast_o  => pump_fast,
            locked_o     => locked_o,
            err_dbg_o    => e_mean,
            pump_dbg_o   => pump_dbg_o
        );

    u_pump: entity work.wallclock_vcxo_pump
        generic map(
            WIN_CYC     => WIN_CYC,
            PUMP_INVERT => PUMP_INVERT
        )
        port map(
            clk     => clk,
            reset_n => reset_n,
            start_i => pump_start,
            len_i   => pump_len,
            up_i    => pump_up,
            fast_i  => pump_fast,
            pump_o  => pump_o
        );

    gen_sysclk_pins: if not DOMAIN_CLOCKS generate
        pin_clocks_o <= sys_clocks_i;
    end generate gen_sysclk_pins;

    gen_domain_pins: if DOMAIN_CLOCKS generate
        u_domain: entity work.wallclock_vcxo_domain
            generic map(
                sys_clk_hz => sys_clk_hz,
                audio_fs   => audio_fs
            )
            port map(
                clk          => clk,
                vcxo_clk_i   => vcxo_clk_i,
                tick_i       => vcxo_tick,
                cnt_i        => vcxo_cnt,
                pin_clocks_o => pin_clocks_o
            );
    end generate gen_domain_pins;

end architecture rtl;
