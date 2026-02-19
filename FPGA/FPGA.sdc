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


#**************************************************************
# Set Clock Latency
#**************************************************************



#**************************************************************
# Set Clock Uncertainty
#**************************************************************

# Let Quartus derive clock uncertainties based on PLL characteristics
derive_clock_uncertainty


#**************************************************************
# Set Input Delay
#**************************************************************



#**************************************************************
# Set Output Delay
#**************************************************************



#**************************************************************
# Set Clock Groups
#**************************************************************

# Three asynchronous clock domains:
# 1. System domain: c10_clk50m -> PLL -> sys_clk_125m, fmc_clk_250m
# 2. MAC domain: rmii_refclk -> mac_clk_25m
# 3. Audio domain: audio_mclk

set_clock_groups -asynchronous \
    -group [get_clocks {c10_clk50m sys_clk_125m fmc_clk_250m}] \
    -group [get_clocks {rmii_refclk mac_clk_25m}] \
    -group [get_clocks {audio_mclk}]


#**************************************************************
# Set False Path
#**************************************************************

# CDC synchronizers in PTP logic
set_false_path -from [get_registers {*ptpv2_sender*tx_en*}] -to [get_registers {*ptpv2_controller*tx_en_i_meta*}]
set_false_path -to [get_registers {*ptpv2_parser*parse_ptp_packet_meta*}]


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
