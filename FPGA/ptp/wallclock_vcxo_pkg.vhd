-- ============================================================
-- wallclock_vcxo_pkg: shared number formats and helpers of the VCXO
-- MCLK loop (wallclock_vcxo_*).
-- ============================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package wallclock_vcxo_pkg is
    -- Phase values: Q16 MCLK cycles, mod 512 (one sample period)
    constant VCXO_Q     : natural := 16;
    constant VCXO_PH_W  : natural := 9 + VCXO_Q;
    -- Controller: width and fractional bits of pump quantities (Q8 sys_clk cycles)
    constant VCXO_ACC_W : natural := 32;
    constant VCXO_QP    : natural := 8;

    -- sys_clk cycles over which one window's pump pulses are spread:
    -- slightly shorter than a window, so they are always delivered
    -- before the next command even with the VCXO pulled fast.
    function vcxo_win_cyc(sys_clk_hz, audio_fs, avg_log2 : natural) return natural;

    -- Shift by a (possibly negative) constant amount after resizing to w.
    function shift_signed(x : signed; n : integer; w : natural) return signed;
    function abs_s(x : signed) return signed;
    function min_nat(a, b : natural) return natural;
    -- Clamp x to [-lim, +lim] (lim >= 0, same width as x).
    function clamp_signed(x : signed; lim : signed) return signed;
end package wallclock_vcxo_pkg;

package body wallclock_vcxo_pkg is
    function vcxo_win_cyc(sys_clk_hz, audio_fs, avg_log2 : natural) return natural is
    begin
        return integer(2.0**avg_log2 * real(sys_clk_hz) / real(audio_fs * 512)
                       * (1.0 - 1.0 / 256.0));
    end function;

    function shift_signed(x : signed; n : integer; w : natural) return signed is
        variable r : signed(w - 1 downto 0);
    begin
        r := resize(x, w);
        if n >= 0 then
            return shift_left(r, n);
        else
            return shift_right(r, -n);
        end if;
    end function;

    function abs_s(x : signed) return signed is
    begin
        if x < 0 then
            return -x;
        end if;
        return x;
    end function;

    function min_nat(a, b : natural) return natural is
    begin
        if a < b then
            return a;
        end if;
        return b;
    end function;

    function clamp_signed(x : signed; lim : signed) return signed is
    begin
        if x > lim then
            return lim;
        elsif x < -lim then
            return -lim;
        end if;
        return x;
    end function;
end package body wallclock_vcxo_pkg;
