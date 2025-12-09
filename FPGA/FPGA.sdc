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

## DATE    "Mon Dec  8 00:08:25 2025"

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
create_clock -name {gpio0_sck} -period 40.000 -waveform { 0.000 20.000 } [get_ports {gpio0}]


#**************************************************************
# Create Generated Clock
#**************************************************************

create_generated_clock -name {audioclks:inst2|altpll:altpll_component|audioclks_altpll:auto_generated|wire_pll1_clk[1]} -source [get_pins {inst2|altpll_component|auto_generated|pll1|inclk[0]}] -duty_cycle 50/1 -multiply_by 3 -divide_by 3125 -master_clock {c10_clk50m} [get_pins {inst2|altpll_component|auto_generated|pll1|clk[1]}] 


#**************************************************************
# Set Clock Latency
#**************************************************************



#**************************************************************
# Set Clock Uncertainty
#**************************************************************

set_clock_uncertainty -rise_from [get_clocks {gpio0_sck}] -rise_to [get_clocks {c10_clk50m}] -setup 0.070  
set_clock_uncertainty -rise_from [get_clocks {gpio0_sck}] -rise_to [get_clocks {c10_clk50m}] -hold 0.090  
set_clock_uncertainty -rise_from [get_clocks {gpio0_sck}] -fall_to [get_clocks {c10_clk50m}] -setup 0.070  
set_clock_uncertainty -rise_from [get_clocks {gpio0_sck}] -fall_to [get_clocks {c10_clk50m}] -hold 0.090  
set_clock_uncertainty -fall_from [get_clocks {gpio0_sck}] -rise_to [get_clocks {c10_clk50m}] -setup 0.070  
set_clock_uncertainty -fall_from [get_clocks {gpio0_sck}] -rise_to [get_clocks {c10_clk50m}] -hold 0.090  
set_clock_uncertainty -fall_from [get_clocks {gpio0_sck}] -fall_to [get_clocks {c10_clk50m}] -setup 0.070  
set_clock_uncertainty -fall_from [get_clocks {gpio0_sck}] -fall_to [get_clocks {c10_clk50m}] -hold 0.090  
set_clock_uncertainty -rise_from [get_clocks {audioclks:inst2|altpll:altpll_component|audioclks_altpll:auto_generated|wire_pll1_clk[1]}] -rise_to [get_clocks {audioclks:inst2|altpll:altpll_component|audioclks_altpll:auto_generated|wire_pll1_clk[1]}]  0.030  
set_clock_uncertainty -rise_from [get_clocks {audioclks:inst2|altpll:altpll_component|audioclks_altpll:auto_generated|wire_pll1_clk[1]}] -fall_to [get_clocks {audioclks:inst2|altpll:altpll_component|audioclks_altpll:auto_generated|wire_pll1_clk[1]}]  0.030  
set_clock_uncertainty -fall_from [get_clocks {audioclks:inst2|altpll:altpll_component|audioclks_altpll:auto_generated|wire_pll1_clk[1]}] -rise_to [get_clocks {audioclks:inst2|altpll:altpll_component|audioclks_altpll:auto_generated|wire_pll1_clk[1]}]  0.030  
set_clock_uncertainty -fall_from [get_clocks {audioclks:inst2|altpll:altpll_component|audioclks_altpll:auto_generated|wire_pll1_clk[1]}] -fall_to [get_clocks {audioclks:inst2|altpll:altpll_component|audioclks_altpll:auto_generated|wire_pll1_clk[1]}]  0.030  
set_clock_uncertainty -rise_from [get_clocks {gpio32}] -rise_to [get_clocks {c10_clk50m}] -setup 0.080  
set_clock_uncertainty -rise_from [get_clocks {gpio32}] -rise_to [get_clocks {c10_clk50m}] -hold 0.110  
set_clock_uncertainty -rise_from [get_clocks {gpio32}] -fall_to [get_clocks {c10_clk50m}] -setup 0.080  
set_clock_uncertainty -rise_from [get_clocks {gpio32}] -fall_to [get_clocks {c10_clk50m}] -hold 0.110  
set_clock_uncertainty -fall_from [get_clocks {gpio32}] -rise_to [get_clocks {c10_clk50m}] -setup 0.080  
set_clock_uncertainty -fall_from [get_clocks {gpio32}] -rise_to [get_clocks {c10_clk50m}] -hold 0.110  
set_clock_uncertainty -fall_from [get_clocks {gpio32}] -fall_to [get_clocks {c10_clk50m}] -setup 0.080  
set_clock_uncertainty -fall_from [get_clocks {gpio32}] -fall_to [get_clocks {c10_clk50m}] -hold 0.110  
set_clock_uncertainty -rise_from [get_clocks {c10_clk50m}] -rise_to [get_clocks {c10_clk50m}]  0.100  
set_clock_uncertainty -rise_from [get_clocks {c10_clk50m}] -fall_to [get_clocks {c10_clk50m}]  0.100  
set_clock_uncertainty -fall_from [get_clocks {c10_clk50m}] -rise_to [get_clocks {c10_clk50m}]  0.100  
set_clock_uncertainty -fall_from [get_clocks {c10_clk50m}] -fall_to [get_clocks {c10_clk50m}]  0.100  
set_clock_uncertainty -rise_from [get_clocks {altera_reserved_tck}] -rise_to [get_clocks {altera_reserved_tck}]  0.020  
set_clock_uncertainty -rise_from [get_clocks {altera_reserved_tck}] -fall_to [get_clocks {altera_reserved_tck}]  0.020  
set_clock_uncertainty -fall_from [get_clocks {altera_reserved_tck}] -rise_to [get_clocks {altera_reserved_tck}]  0.020  
set_clock_uncertainty -fall_from [get_clocks {altera_reserved_tck}] -fall_to [get_clocks {altera_reserved_tck}]  0.020  


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

