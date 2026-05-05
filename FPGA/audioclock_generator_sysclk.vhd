library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- ============================================================
-- Audio Clock Generator (sys_clk variant)
--
-- Functionally equivalent outputs to audioclock_generator.vhd, but
-- runs entirely on sys_clk. The 512fs reference (NCO MSB from
-- wallclock.audio_mclk_o) is sampled in this domain and its rising
-- and falling edges are used as enables to advance the divider
-- counter — no asynchronous logic-derived clock is used.
--
-- This avoids routing the NCO MSB on a non-global net and the
-- ±1 sys_clk jitter that comes with using it as a clock source.
-- All outputs are clean sys_clk-synchronous signals.
--
-- Use this variant when no external audio PLL (Si5351A) is fitted
-- and the NCO is the only audio time base. For setups with an
-- external PLL, instantiate audioclock_generator.vhd on the PLL's
-- clock instead.
-- ============================================================

entity audioclock_generator_sysclk is
    port (
        sys_clk   : in  std_logic;   -- system clock (e.g. 125 MHz)
        rst_n     : in  std_logic;   -- active-low reset

        -- 512fs reference (NCO MSB). May come from wallclock.audio_mclk_o.
        -- Sampled and edge-detected in sys_clk domain.
        mclk_ref  : in  std_logic;

        -- Divided clocks (sys_clk-synchronous, ~50% duty toggling on
        -- rising edges of mclk_ref)
        clk_256fs : out std_logic;
        clk_128fs : out std_logic;
        clk_64fs  : out std_logic;
        fs        : out std_logic;

        -- DAC outputs
        bclk_r    : out std_logic;   -- 256fs, toggles on rising  mclk_ref edge
        bclk_f    : out std_logic;   -- 256fs, toggles on falling mclk_ref edge
        fs_pulse  : out std_logic;   -- 1 sys_clk pulse at fs frame boundary
        fs_tdm_pulse : out std_logic -- fs frame sync, held high for 1 BCLK (2 mclk_ref cycles)
    );
end entity audioclock_generator_sysclk;

architecture rtl of audioclock_generator_sysclk is
    -- 2-FF sampler for mclk_ref (avoids glitches if the source is
    -- on routing without a global buffer)
    signal mclk_ref_s1 : std_logic := '0';
    signal mclk_ref_s2 : std_logic := '0';
    signal mclk_ref_s3 : std_logic := '0';

    signal mclk_rising_tick  : std_logic;
    signal mclk_falling_tick : std_logic;

    -- Divider counter, advanced on every rising edge of mclk_ref
    signal cnt : unsigned(8 downto 0) := (others => '0');

    -- Registered outputs
    signal clk_256fs_r : std_logic := '0';
    signal clk_128fs_r : std_logic := '0';
    signal clk_64fs_r  : std_logic := '0';
    signal fs_r        : std_logic := '0';
    signal bclk_r_r    : std_logic := '0';
    signal bclk_f_r    : std_logic := '0';
    signal fs_pulse_r  : std_logic := '0';

    -- TDM frame sync: held high across 2 rising mclk_ref edges (= 1 BCLK).
    -- Counts down on each rising mclk_ref tick after frame boundary.
    signal fs_tdm_r       : std_logic := '0';
    signal fs_tdm_cnt     : unsigned(1 downto 0) := (others => '0');
begin

    ----------------------------------------------------------------
    -- Sample mclk_ref into sys_clk domain and detect both edges
    ----------------------------------------------------------------
    p_sample : process(sys_clk, rst_n)
    begin
        if rst_n = '0' then
            mclk_ref_s1 <= '0';
            mclk_ref_s2 <= '0';
            mclk_ref_s3 <= '0';
        elsif rising_edge(sys_clk) then
            mclk_ref_s1 <= mclk_ref;
            mclk_ref_s2 <= mclk_ref_s1;
            mclk_ref_s3 <= mclk_ref_s2;
        end if;
    end process;

    mclk_rising_tick  <= '1' when (mclk_ref_s2 = '1' and mclk_ref_s3 = '0') else '0';
    mclk_falling_tick <= '1' when (mclk_ref_s2 = '0' and mclk_ref_s3 = '1') else '0';

    ----------------------------------------------------------------
    -- Counter advances on each rising mclk_ref edge.
    -- Outputs are taken from registered counter bits so they stay
    -- glitch-free in sys_clk domain.
    ----------------------------------------------------------------
    p_cnt : process(sys_clk, rst_n)
    begin
        if rst_n = '0' then
            cnt         <= (others => '0');
            clk_256fs_r <= '0';
            clk_128fs_r <= '0';
            clk_64fs_r  <= '0';
            fs_r        <= '0';
            bclk_r_r    <= '0';
            fs_pulse_r  <= '0';
            fs_tdm_r    <= '0';
            fs_tdm_cnt  <= (others => '0');
        elsif rising_edge(sys_clk) then
            fs_pulse_r <= '0';

            if mclk_rising_tick = '1' then
                cnt <= cnt + 1;

                -- Frame boundary: arm fs_pulse and start TDM hold window.
                -- fs_tdm_r goes high for 2 rising mclk_ref ticks = 1 BCLK.
                if cnt = to_unsigned(511, 9) then
                    fs_pulse_r <= '1';
                    fs_tdm_r   <= '1';
                    fs_tdm_cnt <= to_unsigned(2, 2);
                elsif fs_tdm_cnt /= 0 then
                    fs_tdm_cnt <= fs_tdm_cnt - 1;
                    if fs_tdm_cnt = 1 then
                        fs_tdm_r <= '0';
                    end if;
                end if;
            end if;

            -- Drive divided clocks from the (post-increment) counter bits.
            -- Reading cnt here gets the current value; the new value
            -- written above takes effect next cycle, which is fine.
            clk_256fs_r <= cnt(0);
            clk_128fs_r <= cnt(1);
            clk_64fs_r  <= cnt(2);
            fs_r        <= cnt(8);
            bclk_r_r    <= cnt(0);
        end if;
    end process;

    ----------------------------------------------------------------
    -- bclk_f : like bclk_r but updated on the falling edge of
    -- mclk_ref (half-period shift). Sampled in sys_clk domain.
    ----------------------------------------------------------------
    p_bclk_f : process(sys_clk, rst_n)
    begin
        if rst_n = '0' then
            bclk_f_r <= '0';
        elsif rising_edge(sys_clk) then
            if mclk_falling_tick = '1' then
                bclk_f_r <= cnt(0);
            end if;
        end if;
    end process;

    ----------------------------------------------------------------
    -- Output assignments
    ----------------------------------------------------------------
    clk_256fs <= clk_256fs_r;
    clk_128fs <= clk_128fs_r;
    clk_64fs  <= clk_64fs_r;
    fs        <= fs_r;
    bclk_r    <= bclk_r_r;
    bclk_f    <= bclk_f_r;
    fs_pulse  <= fs_pulse_r;
    fs_tdm_pulse <= fs_tdm_r;

end architecture rtl;
