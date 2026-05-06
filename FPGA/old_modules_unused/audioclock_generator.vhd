library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity audioclock_generator is
    port (
        mclk      : in  std_logic;   -- 512fs = 24.576 MHz master clock input
        rst_n     : in  std_logic;   -- active-low reset

        -- Simple divided clocks (counter bits)
        clk_256fs : out std_logic;   -- mclk / 2   = 12.288 MHz
        clk_128fs : out std_logic;   -- mclk / 4   =  6.144 MHz
        clk_64fs  : out std_logic;   -- mclk / 8   =  3.072 MHz
        fs        : out std_logic;   -- mclk / 512 = 48 kHz

        -- DAC outputs
        bclk_r    : out std_logic;   -- 256fs, goes high on rising  mclk edge
        bclk_f    : out std_logic;   -- 256fs, goes high on falling mclk edge (TDM)
        fs_pulse  : out std_logic    -- 48 kHz, high for 2 mclk periods, rising-edge aligned
    );
end entity audioclock_generator;

architecture rtl of audioclock_generator is
    signal cnt      : unsigned(8 downto 0);   -- 0..511  (512fs / 512 = fs)
    signal bclk_f_r  : std_logic;
    signal fs_pulse_r : std_logic;
begin

    ----------------------------------------------------------------
    -- 9-bit free-running counter on rising mclk
    ----------------------------------------------------------------
    p_cnt : process(mclk, rst_n)
    begin
        if rst_n = '0' then
            cnt <= (others => '0');
        elsif rising_edge(mclk) then
            cnt <= cnt + 1;
        end if;
    end process;

    ----------------------------------------------------------------
    -- Simple divider outputs (directly from counter bits)
    ----------------------------------------------------------------
    clk_256fs <= cnt(0);   -- 12.288 MHz
    clk_128fs <= cnt(1);   --  6.144 MHz
    clk_64fs  <= cnt(2);   --  3.072 MHz
    fs        <= cnt(8);   -- 48 kHz

    ----------------------------------------------------------------
    -- bclk_r : 256fs – rising edge aligned to rising mclk
    ----------------------------------------------------------------
    bclk_r <= cnt(0);

    ----------------------------------------------------------------
    -- bclk_f : 256fs – rising edge aligned to falling mclk
    --          (sample cnt(0) on falling edge → half-cycle shift)
    ----------------------------------------------------------------
    p_bclk_f : process(mclk, rst_n)
    begin
        if rst_n = '0' then
            bclk_f_r <= '0';
        elsif falling_edge(mclk) then
            bclk_f_r <= cnt(0);
        end if;
    end process;

    bclk_f <= bclk_f_r;

    ----------------------------------------------------------------
    -- fs_pulse : high for exactly 2 mclk periods at frame boundary
    --            registered on rising mclk
    ----------------------------------------------------------------
    p_fs_pulse : process(mclk, rst_n)
    begin
        if rst_n = '0' then
            fs_pulse_r <= '0';
        elsif rising_edge(mclk) then
            if cnt = to_unsigned(511, 9) or cnt = to_unsigned(0, 9) then
                fs_pulse_r <= '1';
            else
                fs_pulse_r <= '0';
            end if;
        end if;
    end process;

    fs_pulse <= fs_pulse_r;

end architecture rtl;
