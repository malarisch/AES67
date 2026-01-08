library ieee;
use ieee.std_logic_1164.all;
use IEEE.NUMERIC_STD.ALL;

entity cdc_mac_sys is
    port(
        mac_clk_i     : in std_logic;
        reset_n_i : in std_logic;
        
        sys_clk_i      : in std_logic;
        
        sys_pulse_i    : in std_logic;
        mac_pulse_o     : out std_logic
    );
end entity;
architecture Behavioral of cdc_mac_sys is

    signal sys_toggle      : std_logic := '0'; -- toggles in sys_clk domain on incoming pulse
    signal mac_sync_meta   : std_logic := '0'; -- first sync stage in mac_clk domain
    signal mac_sync        : std_logic := '0'; -- second sync stage in mac_clk domain
    signal mac_sync_dly    : std_logic := '0'; -- delayed copy for edge detect
    
begin 

    -- Toggle on the source clock when a pulse arrives
    process(sys_clk_i, reset_n_i)
    begin
        if reset_n_i = '0' then
            sys_toggle <= '0';
        elsif rising_edge(sys_clk_i) then
            if sys_pulse_i = '1' then
                sys_toggle <= not sys_toggle;
            end if;
        end if;
    end process;

    -- Synchronize the toggle into mac_clk domain and edge-detect
    process(mac_clk_i, reset_n_i)
    begin
        if reset_n_i = '0' then
            mac_sync_meta <= '0';
            mac_sync      <= '0';
            mac_sync_dly  <= '0';
        elsif rising_edge(mac_clk_i) then
            mac_sync_meta <= sys_toggle;
            mac_sync      <= mac_sync_meta;
            mac_sync_dly  <= mac_sync;
        end if;
    end process;

    mac_pulse_o <= mac_sync xor mac_sync_dly; -- 1-cycle pulse in mac domain on toggle change

end Behavioral;