-- ============================================================
-- wallclock_media_clock: AES67 RTP media clock
--
-- media_clock is a *pure function* of the wallclock -- no NCO
-- dependency at all. Every other cycle:
--   media_base    = sec(31:0) * audio_fs                      (Stage 0)
--   media_mult    = nsec * MEDIA_CLK_RECIP                    (Stage 1)
--   media_clock   = media_base + media_mult(55:40)            (Stage 2)
-- RECIP is in Q26 form (= audio_fs * 2^(32+8) / 1e9 rounded), so the
-- top 16 bits of the product sit at [55:40]. media_clock(0) therefore
-- toggles at the wallclock sample boundary -- bit-exactly synchronous
-- across all PTP-locked boards. It is the reference the NCO is pulled
-- towards (see wallclock_nco).
-- ============================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity wallclock_media_clock is
    generic(
        audio_fs : natural
    );
    port(
        clk           : in  std_logic;
        reset_n       : in  std_logic;
        nsec_i        : in  unsigned(29 downto 0);
        sec_i         : in  unsigned(47 downto 0);
        -- pending seconds rollover from wallclock_time
        sec_adj_i     : in  integer range -1 to 1;
        media_clock_o : out unsigned(31 downto 0)
    );
end entity wallclock_media_clock;

architecture rtl of wallclock_media_clock is
    -- Rounded to nearest, like NCO_BASE_INC_32 in wallclock_nco: flooring
    -- both constants independently caused a ~2 ppm relative drift.
    constant MEDIA_CLK_RECIP : unsigned(25 downto 0) :=
        to_unsigned(integer(real(audio_fs) * 4294967296.0 / 1.0e9
                            * 256.0 + 0.5), 26);

    signal media_clock_reg        : unsigned(31 downto 0) := (others => '0');
    signal media_clock_nsec_latch : unsigned(29 downto 0) := (others => '0');
    -- Seconds latched in the SAME cycle as the nanoseconds (plus the
    -- not-yet-applied rollover increment) so the pipeline always
    -- computes from a coherent (sec, nsec) pair.
    signal media_sec_latch        : unsigned(31 downto 0) := (others => '0');
    -- 30-bit nsec * 26-bit RECIP = 56-bit product
    signal media_mult_reg         : unsigned(55 downto 0) := (others => '0');
    signal media_base             : unsigned(31 downto 0) := (others => '0');
    -- The pipeline advances every other cycle (multicycle timing)
    signal media_clock_proc_wait  : std_logic := '0';
begin

    media_clock_o <= media_clock_reg;

    media_proc: process(clk, reset_n)
        variable sample_in_sec : unsigned(15 downto 0);  -- 0..audio_fs-1
    begin
        if reset_n = '0' then
            media_clock_reg       <= (others => '0');
            media_base            <= (others => '0');
            media_mult_reg        <= (others => '0');
            media_clock_proc_wait <= '0';
        elsif rising_edge(clk) then
            if (media_clock_proc_wait = '0') then
                media_clock_nsec_latch <= nsec_i;
                media_sec_latch <= unsigned(signed(sec_i(31 downto 0))
                                            + to_signed(sec_adj_i, 32));

                media_base <= resize(media_sec_latch
                                     * to_unsigned(audio_fs, 32), 32);
                media_mult_reg <= media_clock_nsec_latch * MEDIA_CLK_RECIP;
                -- sample_in_sec = bits [55:40] of the Q26 product
                -- (top 16 bits = integer part 0..audio_fs-1)
                sample_in_sec := media_mult_reg(55 downto 40);
                media_clock_reg <= media_base + resize(sample_in_sec, 32);
            end if;
            media_clock_proc_wait <= not media_clock_proc_wait;
        end if;
    end process media_proc;

end architecture rtl;
