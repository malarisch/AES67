## SDC Constraints for AES67 FPGA
## Device: 10CL025YU256I7G (Cyclone 10 LP)
##
## Clock Structure:
##   c10_clk50m (50 MHz) -> PLL -> sys_clk (125 MHz), fmc_clk (250 MHz)
##   gpio23 (50 MHz RMII) -> clkdiv2 -> mac_clk (25 MHz)
##   gpio34 (24.576 MHz = 48kHz * 512 Audio MCLK)

#**************************************************************
# Time Information
#**************************************************************

set_time_format -unit ns -decimal_places 3


#**************************************************************
# Create Clock
#**************************************************************

# Primary 50 MHz oscillator input to PLL
create_clock -name {c10_clk50m} -period 20.000 -waveform { 0.000 10.000 } [get_ports {c10_clk50m}]

# RMII PHY reference clock input (50 MHz)
create_clock -name {rmii_refclk} -period 20.000 -waveform { 0.000 10.000 } [get_ports {gpio23}]

# Audio master clock from external PLL (48kHz * 512 = 24.576 MHz)
create_clock -name {audio_mclk} -period 40.690 -waveform { 0.000 20.345 } [get_ports {gpio34}]

create_clock -period 8.000  -name {enet_clk_125m} [get_ports {enet_clk_125m}]
set_clock_groups -asynchronous -group [get_clocks {enet_clk_125m}]

# MIIM (MDC/MDIO) is a slow async management interface (~2.5 MHz max).
# The MIIM logic runs internally with clock dividers. False path these
# as they are not timing critical and have no synchronous relationship.
set_false_path -from * -to [get_ports {enet_mdc}]
set_false_path -from * -to [get_ports {enet_mdio}]
set_false_path -from [get_ports {enet_mdio}] -to *

set_false_path -from [get_ports enet_intn] -to *
set_false_path -from * -to [get_ports enet_resetn]

#**************************************************************
# Create Generated Clock
#**************************************************************

# PLL output clk[0]: 125 MHz system clock (50 * 5 / 2)
create_generated_clock -name {sys_clk_125m} \
    -source [get_pins {inst6|altpll_component|auto_generated|pll1|inclk[0]}] \
    -duty_cycle 50/1 -multiply_by 5 -divide_by 2 \
    -master_clock {c10_clk50m} \
    [get_pins {inst6|altpll_component|auto_generated|pll1|clk[0]}]

# PLL output clk[1]: 250 MHz FMC clock (50 * 5)
create_generated_clock -name {fmc_clk_250m} \
    -source [get_pins {inst6|altpll_component|auto_generated|pll1|inclk[0]}] \
    -duty_cycle 50/1 -multiply_by 5 \
    -master_clock {c10_clk50m} \
    [get_pins {inst6|altpll_component|auto_generated|pll1|clk[1]}]

# MII2RMII clock divider: 25 MHz MAC clock (50 / 2)
create_generated_clock -name {mac_clk_25m} \
    -source [get_ports {gpio23}] \
    -divide_by 2 \
    -master_clock {rmii_refclk} \
    [get_registers {miirmii:inst43|intel_fpga_mii2rmii:fpga_mii2rmii_0|clkdiv2:u0_clkdiv|clkby2}]

# LiteX SoC PLL output: 70 MHz sys_clk (50 * 7 / 5)
# Note: Quartus auto-derives this from ALTPLL, but we define it explicitly for clarity
create_generated_clock -name {litex_sys_clk} \
    -source [get_pins {inst|ALTPLL|auto_generated|pll1|inclk[0]}] \
    -multiply_by 7 -divide_by 5 \
    -master_clock {c10_clk50m} \
    [get_pins {inst|ALTPLL|auto_generated|pll1|clk[0]}]


#**************************************************************
# Set Clock Latency
#**************************************************************



#**************************************************************
# Set Clock Uncertainty
#**************************************************************

# Let Quartus derive clock uncertainties based on PLL characteristics

#**************************************************************
# Set Input Delay
#**************************************************************



#**************************************************************
# Set Output Delay
#**************************************************************



#**************************************************************
# Set Clock Groups
#**************************************************************

# Clock domains:
# 1. System domain: c10_clk50m -> PLL -> sys_clk_125m, fmc_clk_250m
# 2. MAC domain: rmii_refclk -> mac_clk_25m
# 3. Audio domain: audio_mclk
# 4. LiteX SoC domain: c10_clk50m -> litex_sys_clk

set_clock_groups -asynchronous \
    -group [get_clocks {c10_clk50m sys_clk_125m fmc_clk_250m}] \
    -group [get_clocks {rmii_refclk mac_clk_25m}] \
    -group [get_clocks {audio_mclk}] \
    -group [get_clocks {litex_sys_clk}]


#**************************************************************
# Set False Path
#**************************************************************

# CDC synchronizers in PTP logic (inside ptp_module inst24)
set_false_path -from [get_registers {*inst24|inst35|tx_enable}] -to [get_registers {*inst24|inst34|tx_en_i_meta}]
set_false_path -to [get_registers {*inst24|inst14|parse_ptp_packet_meta}]

# PTP is_leader and is_follower signals cross from fmc_clk_250m to sys_clk_125m domain
# These are slow-changing configuration signals - false path to all PTP modules
set_false_path -from [get_registers {*fmc_ethernet_client*|ptp_is_leader_o}] -to [get_registers {*inst24|inst34|*}]
set_false_path -from [get_registers {*fmc_ethernet_client*|ptp_is_leader_o}] -to [get_registers {*inst24|inst14|*}]
set_false_path -from [get_registers {*fmc_ethernet_client*|ptp_is_follower_o}] -to [get_registers {*inst24|inst34|*}]
set_false_path -from [get_registers {*fmc_ethernet_client*|ptp_is_follower_o}] -to [get_registers {*inst24|inst14|*}]

# tx_router shadow register CDC (config_wr_clk -> sys_clk) - inside audio_tx_module inst13
# These are slow-changing config values updated only during stream setup
set_false_path -from [get_registers {*inst13|inst37|samples_per_packet_shadow[*][*]}] -to [get_registers {*inst13|inst37|samples_per_packet_sync[*][*]}]
set_false_path -from [get_registers {*inst13|inst37|threshold_shadow[*][*]}] -to [get_registers {*inst13|inst37|threshold_sync[*][*]}]

# tx_router tx_en CDC (tx_transmitter domain -> sys_clk)
set_false_path -to [get_registers {*inst13|inst37|tx_en_meta}]

# LiteX SoC reset signals - static during normal operation
set_false_path -from [get_registers {*aes67soc_reset_storage*}] -to *
set_false_path -from [get_registers {*aes67soc_reset_re*}] -to *
set_false_path -from [get_registers {*crg_rst*}] -to *


#**************************************************************
# Set Multicycle Path
#**************************************************************



#**************************************************************
# Set Maximum Delay
#**************************************************************



#**************************************************************
# Set Minimum Delay
#**************************************************************



#**************************************************************
# Set Input Transition
#**************************************************************

derive_pll_clocks
derive_clock_uncertainty


source ./rgmii_phy_if.sdc
source ./rgmii_io.sdc

# RGMII interface
constrain_rgmii_input_pins "enet" "enet_rx_clk" "enet_rx_dv enet_rx_d*"
constrain_rgmii_output_pins "enet" "inst9|altpll_component|auto_generated|pll1|clk[0]" "enet_tx_clk" "enet_tx_en enet_tx_d*"

