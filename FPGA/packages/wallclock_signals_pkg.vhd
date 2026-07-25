library ieee;
use ieee.std_logic_1164.all;
USE ieee.numeric_std.all;

package wallclock_signals_pkg is
    type t_wallclock_signals is record
        wallclock_seconds_o     : unsigned(47 downto 0);
        wallclock_nanoseconds_o : unsigned(29 downto 0);

        wallclock_set_i         : std_logic;
        wallclock_do_phasejump_i : std_logic;
        wallclock_seconds_i     : std_logic_vector(47 downto 0);
        wallclock_nanoseconds_i : std_logic_vector(29 downto 0);
        freq_correction_ppb_i   : signed(19 downto 0);
        nco_ppb_adj_i           : signed(47 downto 0);

        nco_ppb_adj_valid_i     : std_logic;

    end record;


    constant WALLCLOCK_SIGNALS_UNDRIVEN : t_wallclock_signals := (
        wallclock_seconds_o      => (others => 'Z'),
        wallclock_nanoseconds_o  => (others => 'Z'),
        wallclock_set_i          => 'Z',
        wallclock_do_phasejump_i => 'Z',
        wallclock_seconds_i      => (others => 'Z'),
        wallclock_nanoseconds_i  => (others => 'Z'),
        freq_correction_ppb_i    => (others => 'Z'),
        nco_ppb_adj_i            => (others => 'Z'),
        nco_ppb_adj_valid_i      => 'Z'
    );

    type t_eth_timestamp is record
        seconds : unsigned(3 downto 0);
        nanoseconds : unsigned(29 downto 0);
    end record;

    type t_eth_timestamps is record
        rx : t_eth_timestamp;
        tx : t_eth_timestamp;
    end record;
end package wallclock_signals_pkg;