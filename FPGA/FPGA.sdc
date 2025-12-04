## Generated SDC file "FPGA.sdc"

## Copyright (C) 2025  Altera Corporation. All rights reserved.
## Your use of Altera Corporation's design tools, logic functions 
## and other software and tools, and any partner logic 
## functions, and any output files from any of the foregoing 
## (including device programming or simulation files), and any 
## associated documentation or information are expressly subject 
## to the terms and conditions of the Altera Program License 
## Subscription Agreement, the Altera Quartus Prime License Agreement,
## the Altera IP License Agreement, or other applicable license
## agreement, including, without limitation, that your use is for
## the sole purpose of programming logic devices manufactured by
## Altera and sold by Altera or its authorized distributors.  Please
## refer to the Altera Software License Subscription Agreements 
## on the Quartus Prime software download page.


## VENDOR  "Altera"
## PROGRAM "Quartus Prime"
## VERSION "Version 25.1std.0 Build 1129 10/21/2025 SC Lite Edition"

## DATE    "Mon Nov 10 16:53:07 2025"

##
## DEVICE  "10CL025YU256I7G"
##


#**************************************************************
# Time Information
#**************************************************************

set_time_format -unit ns -decimal_places 3



#**************************************************************
# Create Clock
#**************************************************************

create_clock -name {altera_reserved_tck} -period 100.000 -waveform { 0.000 50.000 } [get_ports {altera_reserved_tck}]
create_clock -name {c10_clk50m} -period 20.000 -waveform { 0.000 10.000 } [get_ports {c10_clk50m}]
create_clock -name {gpio32} -period 325.520 -waveform { 0.000 162.760 } [get_ports {gpio32}]


#**************************************************************
# Create Generated Clock
#**************************************************************

create_generated_clock -name {altpll_noeding:inst31|altpll:altpll_component|altpll_noeding_altpll:auto_generated|wire_pll1_clk[0]} -source [get_pins {inst31|altpll_component|auto_generated|pll1|inclk[0]}] -duty_cycle 50/1 -multiply_by 5 -divide_by 2 -master_clock {c10_clk50m} [get_pins {inst31|altpll_component|auto_generated|pll1|clk[0]}] 
create_generated_clock -name {altpll_noeding:inst31|altpll:altpll_component|altpll_noeding_altpll:auto_generated|wire_pll1_clk[1]} -source [get_pins {inst31|altpll_component|auto_generated|pll1|inclk[0]}] -duty_cycle 50/1 -multiply_by 2 -master_clock {c10_clk50m} [get_pins {inst31|altpll_component|auto_generated|pll1|clk[1]}] 
create_generated_clock -name {altpll_noeding:inst31|altpll:altpll_component|altpll_noeding_altpll:auto_generated|wire_pll1_clk[2]} -source [get_pins {inst31|altpll_component|auto_generated|pll1|inclk[0]}] -duty_cycle 50/1 -multiply_by 1 -divide_by 2 -master_clock {c10_clk50m} [get_pins {inst31|altpll_component|auto_generated|pll1|clk[2]}] 
create_generated_clock -name {altpll_noeding:inst31|altpll:altpll_component|altpll_noeding_altpll:auto_generated|wire_pll1_clk[3]} -source [get_pins {inst31|altpll_component|auto_generated|pll1|inclk[0]}] -duty_cycle 50/1 -multiply_by 1 -divide_by 5 -master_clock {c10_clk50m} [get_pins {inst31|altpll_component|auto_generated|pll1|clk[3]}] 
create_generated_clock -name {altpll_noeding:inst31|altpll:altpll_component|altpll_noeding_altpll:auto_generated|wire_pll1_clk[4]} -source [get_pins {inst31|altpll_component|auto_generated|pll1|inclk[0]}] -duty_cycle 50/1 -multiply_by 1 -divide_by 50 -master_clock {c10_clk50m} [get_pins {inst31|altpll_component|auto_generated|pll1|clk[4]}] 
create_generated_clock -name {audioclks:inst2|altpll:altpll_component|audioclks_altpll:auto_generated|wire_pll1_clk[0]} -source [get_pins {inst2|altpll_component|auto_generated|pll1|inclk[0]}] -duty_cycle 50/1 -multiply_by 6 -divide_by 3125 -master_clock {c10_clk50m} [get_pins {inst2|altpll_component|auto_generated|pll1|clk[0]}] 
create_generated_clock -name {audioclks:inst2|altpll:altpll_component|audioclks_altpll:auto_generated|wire_pll1_clk[1]} -source [get_pins {inst2|altpll_component|auto_generated|pll1|inclk[0]}] -duty_cycle 50/1 -multiply_by 3 -divide_by 3125 -master_clock {c10_clk50m} [get_pins {inst2|altpll_component|auto_generated|pll1|clk[1]}] 


#**************************************************************
# Set Clock Latency
#**************************************************************



#**************************************************************
# Set Clock Uncertainty
#**************************************************************



#**************************************************************
# Set Input Delay
#**************************************************************



#**************************************************************
# Set Output Delay
#**************************************************************



#**************************************************************
# Set Clock Groups
#**************************************************************

set_clock_groups -asynchronous -group [get_clocks {altera_reserved_tck}] 
set_clock_groups -asynchronous -group [get_clocks {altera_reserved_tck}] 
set_clock_groups -asynchronous -group [get_clocks {altera_reserved_tck}] 


#**************************************************************
# Set False Path
#**************************************************************



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

