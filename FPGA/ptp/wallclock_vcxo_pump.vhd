-- ============================================================
-- wallclock_vcxo_pump: tri-state charge-pump driver
--
-- Spreads len_i single-sys_clk pulses evenly (first-order PDM) over the
-- WIN_CYC cycles after each start_i, on the fine (pd_a, high-ohmic) or
-- coarse (pd_b, low-ohmic) path. Between pulses both paths float, so the
-- loop-filter capacitor holds the VCXO control voltage.
-- ============================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.audioclks_pkg.all;

entity wallclock_vcxo_pump is
    generic(
        WIN_CYC     : natural;
        -- set if a higher CV lowers the VCXO frequency
        PUMP_INVERT : boolean
    );
    port(
        clk     : in  std_logic;
        reset_n : in  std_logic;
        start_i : in  std_logic;
        len_i   : in  natural range 0 to WIN_CYC;
        up_i    : in  std_logic;
        fast_i  : in  std_logic;   -- '1' = coarse path pd_b
        pump_o  : out t_vcxo_pump := VCXO_PUMP_IDLE
    );
end entity wallclock_vcxo_pump;

architecture rtl of wallclock_vcxo_pump is
    function pump_polarity return std_logic is
    begin
        if PUMP_INVERT then
            return '1';
        end if;
        return '0';
    end function;
    constant PUMP_POL : std_logic := pump_polarity;

    signal pdm_acc    : natural range 0 to 2 * WIN_CYC := 0;
    signal pdm_left   : natural range 0 to WIN_CYC := 0;   -- cycles left in window
    signal pump_pulse : std_logic := '0';
begin

    pdm_proc: process(clk, reset_n)
        variable v_acc : natural range 0 to 2 * WIN_CYC;
    begin
        if reset_n = '0' then
            pdm_acc    <= 0;
            pdm_left   <= 0;
            pump_pulse <= '0';
            pump_o     <= VCXO_PUMP_IDLE;
        elsif rising_edge(clk) then
            pump_pulse <= '0';
            if start_i = '1' then
                pdm_acc  <= 0;
                pdm_left <= WIN_CYC;
            elsif pdm_left /= 0 then
                pdm_left <= pdm_left - 1;
                v_acc := pdm_acc + len_i;
                if v_acc >= WIN_CYC then
                    v_acc := v_acc - WIN_CYC;
                    pump_pulse <= '1';
                end if;
                pdm_acc <= v_acc;
            end if;

            -- registered pin drive: only the selected path is ever enabled
            pump_o.pd_a    <= up_i xor PUMP_POL;
            pump_o.pd_b    <= up_i xor PUMP_POL;
            pump_o.pd_a_oe <= pump_pulse and not fast_i;
            pump_o.pd_b_oe <= pump_pulse and fast_i;
        end if;
    end process;

end architecture rtl;
