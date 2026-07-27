library ieee;
use ieee.std_logic_1164.all;
use IEEE.NUMERIC_STD.ALL;
use work.audioclks_pkg.all;
use work.miim_types.all;

package system_cfg_pkg is
    constant SYS_CLK_NS_PER_TICK : natural := 8;
    constant USE_EXTERNAL_PLL : boolean := false;
    type t_mii_types is (MII, RMII, GMII, RGMII);
    type t_phy_names is (LAN8720A, LXT973, CORTINA, OTHER);
    type t_soc_types is (LITEX_SPIBONE, LITEX_VEXRISCV_HRAM, LITEX_VEXRISCV_SDRAM, LITEX_UARTBONE);
    type t_platforms is (ALTERA, GOWIN, LATTICE, XILINX);
    function platform_to_string (platform : in t_platforms) return string;
    
    type t_network_config is record
        MII_TYPE : t_mii_types;
        MII_WIDTH : integer;
        MII_CLK_NS_PER_TICK : integer;
        MIIM_CLOCK_DIVIDER : positive;
    end record;

    type t_phy_config is record
        MIIM_PHY_ADDRESS : t_phy_address;
        PHY_TYPE : t_phy_names;
        NETWORK_CONFIG : t_network_config;
    end record;

    type t_audio_cfg is record
        MAX_STREAMS : natural;
        BUFFER_DEPTH : integer;
        CHANNELS : natural;
        TDM_PINS : integer;
        ADDA_CFG : t_audio_clock_io_cfg;
        
    end record;
    type t_global_audio_cfg is record
        USE_PARALLEL_INTERFACE : boolean;
        PARALLEL_BYTE_DEPTH : integer;
        MCLK_SPEED : audio_clock_speed;
        BCLK_SPEED : audio_clock_speed;
        RX_DA_CFG : t_audio_cfg;
        TX_AD_CFG : t_audio_cfg;
    end record;
    type t_global_system_cfg is record
        CLK_IN_SPEED : natural;
        SOC_TYPE : t_soc_types;
        PLATFORM : t_platforms;
        PHY_CONFIG : t_phy_config;

        AUDIO_CONFIG : t_global_audio_cfg;
        STATIC_PTP_CONFIG : boolean;
        PTP_IN_SOFTWARE : boolean;
        PTP_MOVING_AVERAGE_DEPTH : natural;

        ENABLE_METERING : boolean;
    end record;
    function system_cfg_to_vector (cfg : in t_global_system_cfg) return std_logic_vector;    
    constant std_mii_cfg : t_network_config := (
        MII_TYPE => MII,
        MII_WIDTH => 4,
        MII_CLK_NS_PER_TICK => 40,
        MIIM_CLOCK_DIVIDER => 25
    );
    constant std_rmii_cfg : t_network_config := (
        MII_TYPE => RMII,
        MII_WIDTH => 2,
        MII_CLK_NS_PER_TICK => 20,
        MIIM_CLOCK_DIVIDER => 50
    );
    constant std_rgmii_cfg : t_network_config := (
        MII_TYPE => RGMII,
        MII_WIDTH => 4,
        MII_CLK_NS_PER_TICK => 8,
        MIIM_CLOCK_DIVIDER => 125
    );
    constant std_lxt_cfg : t_phy_config := (
        PHY_TYPE => LXT973,
        MIIM_PHY_ADDRESS => "00010",
        NETWORK_CONFIG => std_mii_cfg
    );
    constant std_lan8720a_cfg : t_phy_config := (
        PHY_TYPE => LAN8720A,
        MIIM_PHY_ADDRESS => "00001",
        NETWORK_CONFIG => std_rmii_cfg
    );
    constant std_cortina_cfg : t_phy_config := (
        PHY_TYPE => CORTINA,
        MIIM_PHY_ADDRESS => "00000",
        NETWORK_CONFIG => std_rgmii_cfg
    );


    constant disable_audio_path : t_audio_cfg := (
        MAX_STREAMS => 1,
        BUFFER_DEPTH => 4,
        CHANNELS => 0,
        TDM_PINS => 4,
        ADDA_CFG => i2s_dac_config
    
    );
    constant four_i2s_outputs : t_audio_cfg := (
        MAX_STREAMS => 8,
        BUFFER_DEPTH => 256,
        CHANNELS => 8,
        TDM_PINS => 4,
        ADDA_CFG => i2s_dac_config
    );
    constant four_i2s_inputs : t_audio_cfg := (
        MAX_STREAMS => 4,
        BUFFER_DEPTH => 256,
        CHANNELS => 8,
        TDM_PINS => 4,
        ADDA_CFG => i2s_dac_config
    );
    constant single_lj_output : t_audio_cfg := (
        MAX_STREAMS => 2,
        BUFFER_DEPTH => 256,
        CHANNELS => 2,
        TDM_PINS => 1,
        ADDA_CFG => i2s_lj_dac_config
    );

    constant audio_config_lo : t_global_audio_cfg := (
        MCLK_SPEED => audio_clock_24_57,
        BCLK_SPEED => audio_clock_03_07,
        USE_PARALLEL_INTERFACE => false,
        PARALLEL_BYTE_DEPTH => 3,
        RX_DA_CFG => four_i2s_outputs,
        TX_AD_CFG => disable_audio_path
    );
    constant audio_config_mi : t_global_audio_cfg := (
        MCLK_SPEED => audio_clock_24_57,
        BCLK_SPEED => audio_clock_03_07,
        USE_PARALLEL_INTERFACE => false,
        PARALLEL_BYTE_DEPTH => 3,
        RX_DA_CFG => disable_audio_path,
        TX_AD_CFG => four_i2s_inputs
    );
    constant audio_config_cyc : t_global_audio_cfg := (
        MCLK_SPEED => audio_clock_24_57,
        BCLK_SPEED => audio_clock_03_07,
        USE_PARALLEL_INTERFACE => false,
        PARALLEL_BYTE_DEPTH => 3,
        RX_DA_CFG => single_lj_output,
        TX_AD_CFG => disable_audio_path
    );

    constant global_system_cfg_mi : t_global_system_cfg := (
        CLK_IN_SPEED => 25,
        SOC_TYPE => LITEX_SPIBONE,
        PLATFORM => ALTERA,
        PHY_CONFIG => std_lxt_cfg,
        AUDIO_CONFIG => audio_config_mi,
        STATIC_PTP_CONFIG => true,
        PTP_IN_SOFTWARE => true,
        ENABLE_METERING => false,
        PTP_MOVING_AVERAGE_DEPTH => 4
    );
    constant global_system_cfg_lo : t_global_system_cfg := (
        CLK_IN_SPEED => 25,
        SOC_TYPE => LITEX_SPIBONE,
        PLATFORM => ALTERA,
        PHY_CONFIG => std_lxt_cfg,
        AUDIO_CONFIG => audio_config_lo,
        STATIC_PTP_CONFIG => true,
        PTP_IN_SOFTWARE => true,
        ENABLE_METERING => false,
        PTP_MOVING_AVERAGE_DEPTH => 4
    );
    constant global_system_cfg_cyc : t_global_system_cfg := (
        CLK_IN_SPEED => 12,
        SOC_TYPE => LITEX_VEXRISCV_SDRAM,
        PLATFORM => ALTERA,
        PHY_CONFIG => std_lan8720a_cfg,
        AUDIO_CONFIG => audio_config_cyc,
        STATIC_PTP_CONFIG => true,
        PTP_IN_SOFTWARE => true,
        ENABLE_METERING => false,
        PTP_MOVING_AVERAGE_DEPTH => 4
    );

end package;

package body system_cfg_pkg is
    -- A variable of unconstrained type string is not allowed; returning
    -- string literals directly sizes the (unconstrained) return type per call.
    function platform_to_string (platform : in t_platforms) return string is
        begin
            if (platform = ALTERA) then return "ALTERA";
            elsif platform = GOWIN then return "GOWIN";
            elsif platform = LATTICE then return "LATTICE";
            elsif platform = XILINX then return "XILINX";
            end if;
        return "UNKNOWN";
        end;
    
    function To_Std_Logic(L : boolean) return std_logic is
        begin
        if L then
            return('1');
        else
            return('0');
        end if;
    end function To_Std_Logic;
    
    function system_cfg_to_vector (cfg : in t_global_system_cfg) return std_logic_vector is    
        variable vector : std_logic_vector(71 downto 0);
    begin
        vector := to_std_logic(cfg.PTP_IN_SOFTWARE) 
        & to_std_logic(cfg.STATIC_PTP_CONFIG) 
        & to_std_logic(cfg.ENABLE_METERING)
        & "00000"
        & std_logic_vector(to_unsigned(cfg.AUDIO_CONFIG.RX_DA_CFG.MAX_STREAMS, 8))
        & std_logic_vector(to_unsigned(cfg.AUDIO_CONFIG.RX_DA_CFG.CHANNELS, 8))
        & std_logic_vector(to_unsigned(cfg.AUDIO_CONFIG.RX_DA_CFG.BUFFER_DEPTH, 16))
        & std_logic_vector(to_unsigned(cfg.AUDIO_CONFIG.TX_AD_CFG.MAX_STREAMS, 8))
        & std_logic_vector(to_unsigned(cfg.AUDIO_CONFIG.TX_AD_CFG.CHANNELS, 8))
        & std_logic_vector(to_unsigned(cfg.AUDIO_CONFIG.TX_AD_CFG.BUFFER_DEPTH, 16));
        return vector;
    end;
end;