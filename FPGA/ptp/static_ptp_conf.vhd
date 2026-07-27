library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use IEEE.MATH_REAL.ALL;
use work.system_cfg_pkg.all;
entity static_ptp_conf is
    generic(
        
        MII_TYPE : t_mii_types;

        -- Board/PHY-specific PTP delayAsymmetry compensation (ns, signed).
        -- Positive = downstream (Master->Slave) path is longer. See
        -- IEEE 1588-2008 §7.4.2.
        -- 100M default (Cyc1000 / LAN8720A): measured +6200 ns on the
        -- LAN8720A — RX path is much longer than TX.
        DELAY_ASYMMETRY_NS_100M : integer := 0;
        DELAY_ASYMMETRY_NS_1G   : integer := 0
    );
    port(
        servo_kp_gain_o              : OUT signed(7 downto 0);
        servo_ki_gain_o              : OUT signed(7 downto 0);
        servo_gain_shift_o           : OUT UNSIGNED(4 downto 0);
        servo_gain_shift_locked_o    : OUT unsigned(4 downto 0);
        servo_ki_extra_shift_o       : OUT unsigned(4 downto 0);
        servo_filter_shift_o         : OUT unsigned(4 downto 0);
        servo_warmup_samples_o       : OUT unsigned(7 downto 0);
        servo_lock_threshold_ns_o    : OUT unsigned(31 downto 0);
        servo_unlock_threshold_ns_o  : OUT unsigned(31 downto 0);
        servo_lock_count_threshold_o : OUT unsigned(7 downto 0);
        parser_delay_asymmetry_ns_o       : OUT signed(31 downto 0)

    );

end entity;
 
architecture rtl of static_ptp_conf is

begin
        servo_kp_gain_o              <= (to_signed(11, 8));
        servo_ki_gain_o              <= (to_signed(7, 8));
        servo_gain_shift_o           <= (to_unsigned(2, 5));
        servo_gain_shift_locked_o    <= (others => '0');
        servo_ki_extra_shift_o       <= (to_unsigned(5, 5));
        servo_filter_shift_o         <= (others => '0');
        servo_warmup_samples_o       <= (to_unsigned(16, 8));
        servo_lock_threshold_ns_o    <= (to_unsigned(500, 32));
        servo_unlock_threshold_ns_o  <= (to_unsigned(5000, 32));
        servo_lock_count_threshold_o <= (to_unsigned(24, 8));
        parser_delay_asymmetry_ns_o      <=
            (to_signed(DELAY_ASYMMETRY_NS_1G,   32)) when MII_TYPE = RGMII
       else (to_signed(DELAY_ASYMMETRY_NS_100M, 32));


end architecture;
