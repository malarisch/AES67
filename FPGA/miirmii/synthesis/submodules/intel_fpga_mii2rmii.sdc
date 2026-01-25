# (C) 2001-2025 Altera Corporation. All rights reserved.
# Your use of Altera Corporation's design tools, logic functions and other 
# software and tools, and its AMPP partner logic functions, and any output 
# files from any of the foregoing (including device programming or simulation 
# files), and any associated documentation or information are expressly subject 
# to the terms and conditions of the Altera Program License Subscription 
# Agreement, Altera IP License Agreement, or other applicable 
# license agreement, including, without limitation, that your use is for the 
# sole purpose of programming logic devices manufactured by Altera and sold by 
# Altera or its authorized distributors.  Please refer to the applicable 
# agreement for further details.


 
 
#**************************************************************
# Time Information
#**************************************************************
 
set_time_format -unit ns -decimal_places 3
 
 
#**************************************************************
# Create Clock
#**************************************************************
 
#Please inlcude appropriate clock constraint for the path that supplies the MII2RMII IP with 50MHz clock within your system level SDC
 
#**************************************************************
# Create Generated Clock
#**************************************************************
 
create_generated_clock -name {u0_5M_clkdiv10} -source [get_pins -compatibility_mode {*|u0_clkddiv10|temp_clk|clk}] -divide_by 10 [get_registers {*|u0_clkddiv10|temp_clk}]
create_generated_clock -name {u1_2_5M_clkdiv2} -source [get_nets {*|u0_clkddiv10|temp_clk}] -divide_by 2  [get_registers {*|u1_clkdiv|clkby2}] 
create_generated_clock -name {u0_25M_clkdiv2} -source [get_pins -compatibility_mode {*|u0_clkdiv|clkby2|clk}] -divide_by 2 [get_registers {*|u0_clkdiv|clkby2}]
 
#For more insight as to where the source and target paths comes from, please compile design and look into the RTL schematic for the MII2RMII IP
