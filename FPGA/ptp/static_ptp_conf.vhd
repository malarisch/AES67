library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use IEEE.MATH_REAL.ALL;
entity static_ptp_conf is
    port(
        servo_kp_gain_o              : OUT STD_LOGIC_VECTOR(7 downto 0)  := std_logic_vector(to_signed(40, 8));
        servo_ki_gain_o              : OUT STD_LOGIC_VECTOR(7 downto 0)  := std_logic_vector(to_signed(5, 8));
        servo_gain_shift_o           : OUT STD_LOGIC_VECTOR(4 downto 0)  := std_logic_vector(to_unsigned(3, 5));
        servo_gain_shift_locked_o    : OUT STD_LOGIC_VECTOR(4 downto 0)  := (others => '0');
        servo_ki_extra_shift_o       : OUT STD_LOGIC_VECTOR(4 downto 0)  := std_logic_vector(to_unsigned(3, 5));
        servo_filter_shift_o         : OUT STD_LOGIC_VECTOR(4 downto 0)  := (others => '0');
        servo_warmup_samples_o       : OUT STD_LOGIC_VECTOR(7 downto 0)  := std_logic_vector(to_unsigned(16, 8));
        servo_lock_threshold_ns_o    : OUT STD_LOGIC_VECTOR(31 downto 0) := std_logic_vector(to_unsigned(500, 32));
        servo_unlock_threshold_ns_o  : OUT STD_LOGIC_VECTOR(31 downto 0) := std_logic_vector(to_unsigned(5000, 32));
        servo_lock_count_threshold_o : OUT STD_LOGIC_VECTOR(7 downto 0)  := std_logic_vector(to_unsigned(24, 8));
        parser_min_filter_enable_o        : OUT STD_LOGIC := '0';
        parser_min_filter_active_depth_o  : OUT STD_LOGIC_VECTOR(7 downto 0) := std_logic_vector(to_unsigned(2, 8))

    );

end entity;

architecture rtl of static_ptp_conf is
 
begin

    

end architecture;