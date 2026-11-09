-- ============================================================
-- wallclock_vcxo_domain: pin audio clocks in the VCXO clock domain
--
-- (VCXO_DOMAIN_CLOCKS) Generates BCLK/LRCK/TDM frame clocks for the
-- pins from VCXO-clocked registers, i.e. without sys_clk quantisation
-- jitter. A free-running VCXO-domain counter (vraw_v) is aligned to the
-- sys_clk-domain count (cnt_i, which includes the PD's slips) by an
-- offset measured in sys_clk and handed back over a stable-value
-- synchroniser. The pins then run the same divider as the sys_clk
-- clocks, one MCLK behind the internal count. Requires
-- sys_clk >= 4.5 * MCLK for the counter hand-over.
-- ============================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.audioclks_pkg.all;

entity wallclock_vcxo_domain is
    generic(
        sys_clk_hz : natural;
        audio_fs   : natural
    );
    port(
        clk          : in  std_logic;
        vcxo_clk_i   : in  std_logic;
        -- from wallclock_vcxo_pd (sys_clk domain)
        tick_i       : in  std_logic;
        cnt_i        : in  unsigned(8 downto 0);
        pin_clocks_o : out t_audio_clocks := AUDIO_CLOCKS_RESET
    );
end entity wallclock_vcxo_domain;

architecture rtl of wallclock_vcxo_domain is
    constant MCLK_HZ : real := real(audio_fs * 512);

    attribute PRESERVE : boolean;

    -- VCXO domain
    signal vraw_v  : unsigned(8 downto 0) := (others => '0');
    signal vraw_g  : std_logic_vector(8 downto 0) := (others => '0');  -- Gray
    signal vofs_m  : unsigned(8 downto 0) := (others => '0');
    signal vofs_q1 : unsigned(8 downto 0) := (others => '0');
    signal vofs_q2 : unsigned(8 downto 0) := (others => '0');
    signal vofs_v  : unsigned(8 downto 0) := (others => '0');
    signal vcnt    : unsigned(8 downto 0);
    attribute PRESERVE of vofs_m  : signal is true;
    attribute PRESERVE of vofs_q1 : signal is true;

    -- sys_clk domain
    signal g_meta  : std_logic_vector(8 downto 0) := (others => '0');
    signal g_sync  : std_logic_vector(8 downto 0) := (others => '0');
    attribute PRESERVE of g_meta : signal is true;
    attribute PRESERVE of g_sync : signal is true;
    signal vraw_s  : unsigned(8 downto 0) := (others => '0');
    signal tick_d  : std_logic_vector(1 to 3) := (others => '0');
    signal vofs_s  : unsigned(8 downto 0) := (others => '0');
begin
    assert real(sys_clk_hz) >= 4.5 * MCLK_HZ
        report "wallclock: VCXO_DOMAIN_CLOCKS needs sys_clk >= 4.5 * MCLK"
        severity failure;

    -- VCXO domain: free-running count, offset take-over (a value is
    -- used once two consecutive samples agree)
    vcxo_cnt_proc: process(vcxo_clk_i)
    begin
        if rising_edge(vcxo_clk_i) then
            vraw_v <= vraw_v + 1;
            vraw_g <= std_logic_vector((vraw_v + 1) xor shift_right(vraw_v + 1, 1));

            vofs_m  <= vofs_s;
            vofs_q1 <= vofs_m;
            vofs_q2 <= vofs_q1;
            if vofs_q1 = vofs_q2 then
                vofs_v <= vofs_q2;
            end if;
        end if;
    end process;

    -- pins = internal count delayed by one MCLK; same divider as the
    -- sys_clk clocks, so the pins switch on the counting edge
    vcnt <= vraw_v + vofs_v;

    u_div: entity work.wallclock_clk_div
        port map(
            clk        => vcxo_clk_i,
            mclk_i     => vcxo_clk_i,
            mclk_cnt_i => vcnt,
            clocks_o   => pin_clocks_o
        );

    -- sys_clk domain: synchronise the Gray count, measure the offset
    vofs_proc: process(clk)
        variable b : unsigned(8 downto 0);
    begin
        if rising_edge(clk) then
            g_meta <= vraw_g;
            g_sync <= g_meta;
            b(8) := g_sync(8);
            for i in 7 downto 0 loop
                b(i) := b(i + 1) xor g_sync(i);
            end loop;
            vraw_s <= b;

            tick_d <= tick_i & tick_d(1 to 2);
            if tick_d(3) = '1' then
                vofs_s <= cnt_i - vraw_s;
            end if;
        end if;
    end process;

end architecture rtl;
