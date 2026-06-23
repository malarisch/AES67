library ieee;
use ieee.std_logic_1164.all;

package audioclks_pkg is
    type t_audio_clocks_selected is record
        bclk : std_logic;
        
        fsclk_i2s_50 : std_logic;
        fsclk_tdm : std_logic;
        fsclk_i2s_tdm : std_logic;
        
    end record;
    type t_audio_clocks is record
        mclk : std_logic;
        clk_256fs : t_audio_clocks_selected;
        clk_128fs : t_audio_clocks_selected;
        clk_64fs  : t_audio_clocks_selected;
        fsclk_50 : std_logic;
    end record;

    constant AUDIO_CLOCKS_RESET_SELECTED : t_audio_clocks_selected := (
        bclk => '0',
        
        fsclk_i2s_50 => '0',
        fsclk_tdm => '0',
        fsclk_i2s_tdm => '0'
    );
    constant AUDIO_CLOCKS_RESET : t_audio_clocks := (
        mclk => '0',
        clk_256fs => AUDIO_CLOCKS_RESET_SELECTED,
        clk_128fs => AUDIO_CLOCKS_RESET_SELECTED,
        clk_64fs  => AUDIO_CLOCKS_RESET_SELECTED,
        fsclk_50 => '0'
    );

end package audioclks_pkg;