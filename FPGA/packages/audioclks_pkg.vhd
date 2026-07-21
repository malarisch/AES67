library ieee;
use ieee.std_logic_1164.all;

package audioclks_pkg is
    type t_audio_clocks_selected is record
        bclk : std_logic;
        
        fsclk_i2s_50 : std_logic;
        fsclk_i2s_50_dac : std_logic;
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
        fsclk_i2s_tdm => '0',
        fsclk_i2s_50_dac => '0'
    );
    constant AUDIO_CLOCKS_RESET : t_audio_clocks := (
        mclk => '0',
        clk_256fs => AUDIO_CLOCKS_RESET_SELECTED,
        clk_128fs => AUDIO_CLOCKS_RESET_SELECTED,
        clk_64fs  => AUDIO_CLOCKS_RESET_SELECTED,
        fsclk_50 => '0'
    );
    subtype audio_clock_speed is std_logic_vector(2 downto 0);
    constant audio_clock_24_57 : audio_clock_speed := "000";
    constant audio_clock_12_28 : audio_clock_speed := "001";
    constant audio_clock_06_14 : audio_clock_speed := "010";
    constant audio_clock_03_07 : audio_clock_speed := "011";
    type t_audio_clock_io_cfg is record
        data_is_valid_on_rising_bclk_edge : BOOLEAN;
        bits_are_right_shifted_to_fs : BOOLEAN;
        fs_is_one_bclk_high : BOOLEAN;
        fs_high_is_channel_2 : BOOLEAN;
        tdm_channels : integer;
    end record;
    type t_audio_clock_cfg is record
        bclk_speed : audio_clock_speed;
        mclk_speed : audio_clock_speed;
        dac_cfg : t_audio_clock_io_cfg;
        adc_cfg : t_audio_clock_io_cfg;
    end record;
    constant i2s_dac_config : t_audio_clock_io_cfg := (
        data_is_valid_on_rising_bclk_edge => true,
        bits_are_right_shifted_to_fs => true,
        fs_is_one_bclk_high => false,
        fs_high_is_channel_2 => true,
        tdm_channels => 2
    );
    constant i2s_adc_config : t_audio_clock_io_cfg := (
        data_is_valid_on_rising_bclk_edge => false,
        bits_are_right_shifted_to_fs => true,
        fs_is_one_bclk_high => false,
        fs_high_is_channel_2 => true,
        tdm_channels => 2
    );
    constant i2s_lj_dac_config : t_audio_clock_io_cfg := (
        data_is_valid_on_rising_bclk_edge => true,
        bits_are_right_shifted_to_fs => false,
        fs_is_one_bclk_high => false,
        fs_high_is_channel_2 => true,
        tdm_channels => 2
    );
    constant i2s_lj_adc_config : t_audio_clock_io_cfg := (
        data_is_valid_on_rising_bclk_edge => false,
        bits_are_right_shifted_to_fs => false,
        fs_is_one_bclk_high => false,
        fs_high_is_channel_2 => true,
        tdm_channels => 2
    );
    constant tdm8_adc_config : t_audio_clock_io_cfg := (
        data_is_valid_on_rising_bclk_edge => true,
        bits_are_right_shifted_to_fs => true,
        fs_is_one_bclk_high => true,
        fs_high_is_channel_2 => false,
        tdm_channels => 8
    );
    constant tdm8_dac_config : t_audio_clock_io_cfg := (
        data_is_valid_on_rising_bclk_edge => false,
        bits_are_right_shifted_to_fs => true,
        fs_is_one_bclk_high => true,
        fs_high_is_channel_2 => false,
        tdm_channels => 8
    );

end package audioclks_pkg;