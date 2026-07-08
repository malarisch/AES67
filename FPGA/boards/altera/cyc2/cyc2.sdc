## Generated SDC file "cyc2.sdc"

## Copyright (C) 1991-2013 Altera Corporation
## Your use of Altera Corporation's design tools, logic functions 
## and other software and tools, and its AMPP partner logic 
## functions, and any output files from any of the foregoing 
## (including device programming or simulation files), and any 
## associated documentation or information are expressly subject 
## to the terms and conditions of the Altera Program License 
## Subscription Agreement, Altera MegaCore Function License 
## Agreement, or other applicable license agreement, including, 
## without limitation, that your use is for the sole purpose of 
## programming logic devices manufactured by Altera and sold by 
## Altera or its authorized distributors.  Please refer to the 
## applicable agreement for further details.


## VENDOR  "Altera"
## PROGRAM "Quartus II"
## VERSION "Version 13.0.1 Build 232 06/12/2013 Service Pack 1 SJ Web Edition"

## DATE    "Tue Jul  7 17:51:07 2026"

##
## DEVICE  "EP2C8Q208C7"
##


#**************************************************************
# Time Information
#**************************************************************

set_time_format -unit ns -decimal_places 3



#**************************************************************
# Create Clock
#**************************************************************

create_clock -name {txclk_i} -period 40.000 -waveform { 0.000 20.000 } [get_ports {txclk_i}]
create_clock -name {rxclk_i} -period 40.000 -waveform { 0.000 20.000 } [get_ports { rxclk_i }]
create_clock -name {soc_top:soc_top_inst|wb_bridge_top:wb_bridge_top_inst|aes67_wb_bridge:aes67_wb_bridge_inst|aes67_top:aes67_top_inst|ptp_module:ptp_inst|wallclock:b2v_wallclock|audioclks_reg.clk_64fs.bclk} -period 81.779 -waveform { 0.000 40.889 } [get_registers {soc_top:soc_top_inst|wb_bridge_top:wb_bridge_top_inst|aes67_wb_bridge:aes67_wb_bridge_inst|aes67_top:aes67_top_inst|ptp_module:ptp_inst|wallclock:b2v_wallclock|audioclks_reg.clk_64fs.bclk}]


#**************************************************************
# Create Generated Clock
#**************************************************************

create_generated_clock -name {soc_top_inst|wb_bridge_top_inst|sysclk_pll_gen_inst|\sysclkgen25:sysclks_altpll_25m_in_inst|altpll_component|pll|clk[0]} -source [get_pins {soc_top_inst|wb_bridge_top_inst|sysclk_pll_gen_inst|\sysclkgen25:sysclks_altpll_25m_in_inst|altpll_component|pll|inclk[0]}] -duty_cycle 50.000 -multiply_by 5 -master_clock {txclk_i} [get_pins {soc_top_inst|wb_bridge_top_inst|sysclk_pll_gen_inst|\sysclkgen25:sysclks_altpll_25m_in_inst|altpll_component|pll|clk[0]}] 
create_generated_clock -name {soc_top_inst|wb_bridge_top_inst|sysclk_pll_gen_inst|\sysclkgen25:sysclks_altpll_25m_in_inst|altpll_component|pll|clk[1]} -source [get_pins {soc_top_inst|wb_bridge_top_inst|sysclk_pll_gen_inst|\sysclkgen25:sysclks_altpll_25m_in_inst|altpll_component|pll|inclk[0]}] -duty_cycle 50.000 -multiply_by 3 -master_clock {txclk_i} [get_pins {soc_top_inst|wb_bridge_top_inst|sysclk_pll_gen_inst|\sysclkgen25:sysclks_altpll_25m_in_inst|altpll_component|pll|clk[1]}] 


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

set_clock_groups -asynchronous -group [get_clocks {soc_top_inst|wb_bridge_top_inst|sysclk_pll_gen_inst|\sysclkgen25:sysclks_altpll_25m_in_inst|altpll_component|pll|clk[0]}] -group [get_clocks {soc_top:soc_top_inst|wb_bridge_top:wb_bridge_top_inst|aes67_wb_bridge:aes67_wb_bridge_inst|aes67_top:aes67_top_inst|ptp_module:ptp_inst|wallclock:b2v_wallclock|audioclks_reg.clk_64fs.bclk}] 
set_clock_groups -asynchronous -group [get_clocks {soc_top_inst|wb_bridge_top_inst|sysclk_pll_gen_inst|\sysclkgen25:sysclks_altpll_25m_in_inst|altpll_component|pll|clk[0]}] -group [get_clocks {soc_top:soc_top_inst|wb_bridge_top:wb_bridge_top_inst|aes67_wb_bridge:aes67_wb_bridge_inst|aes67_top:aes67_top_inst|ptp_module:ptp_inst|wallclock:b2v_wallclock|audioclks_reg.clk_64fs.bclk}] 


#**************************************************************
# Set False Path
#**************************************************************

set_false_path  -from  [get_clocks {soc_top_inst|wb_bridge_top_inst|sysclk_pll_gen_inst|\sysclkgen25:sysclks_altpll_25m_in_inst|altpll_component|pll|clk[0]}]  -to  [get_clocks {soc_top_inst|wb_bridge_top_inst|sysclk_pll_gen_inst|\sysclkgen25:sysclks_altpll_25m_in_inst|altpll_component|pll|clk[1]}]
set_false_path  -from  [get_clocks {soc_top_inst|wb_bridge_top_inst|sysclk_pll_gen_inst|\sysclkgen25:sysclks_altpll_25m_in_inst|altpll_component|pll|clk[1]}]  -to  [get_clocks {soc_top_inst|wb_bridge_top_inst|sysclk_pll_gen_inst|\sysclkgen25:sysclks_altpll_25m_in_inst|altpll_component|pll|clk[0]}]
set_false_path  -from  [get_clocks {soc_top_inst|wb_bridge_top_inst|sysclk_pll_gen_inst|\sysclkgen25:sysclks_altpll_25m_in_inst|altpll_component|pll|clk[0]}]  -to  [get_clocks {soc_top_inst|wb_bridge_top_inst|sysclk_pll_gen_inst|\sysclkgen25:sysclks_altpll_25m_in_inst|altpll_component|pll|clk[1]}]
set_false_path  -from  [get_clocks {soc_top_inst|wb_bridge_top_inst|sysclk_pll_gen_inst|\sysclkgen25:sysclks_altpll_25m_in_inst|altpll_component|pll|clk[1]}]  -to  [get_clocks {soc_top_inst|wb_bridge_top_inst|sysclk_pll_gen_inst|\sysclkgen25:sysclks_altpll_25m_in_inst|altpll_component|pll|clk[0]}]
set_false_path  -from  [get_clocks {soc_top_inst|wb_bridge_top_inst|sysclk_pll_gen_inst|\sysclkgen25:sysclks_altpll_25m_in_inst|altpll_component|pll|clk[0]}]  -to  [get_clocks {soc_top_inst|wb_bridge_top_inst|sysclk_pll_gen_inst|\sysclkgen25:sysclks_altpll_25m_in_inst|altpll_component|pll|clk[1]}]
set_false_path  -from  [get_clocks {soc_top_inst|wb_bridge_top_inst|sysclk_pll_gen_inst|\sysclkgen25:sysclks_altpll_25m_in_inst|altpll_component|pll|clk[1]}]  -to  [get_clocks {soc_top_inst|wb_bridge_top_inst|sysclk_pll_gen_inst|\sysclkgen25:sysclks_altpll_25m_in_inst|altpll_component|pll|clk[0]}]
set_false_path  -from  [get_clocks {txclk_i}]  -to  [get_clocks {rxclk_i}]
set_false_path  -from  [get_clocks {soc_top_inst|wb_bridge_top_inst|sysclk_pll_gen_inst|\sysclkgen25:sysclks_altpll_25m_in_inst|altpll_component|pll|clk[0]}]  -to  [get_clocks {rxclk_i}]
set_false_path -from [get_registers {*speed_o*}] 
set_false_path -from [get_registers {*aes67_csr*storage*}] 
set_false_path -from [get_registers {*aes67soc_reset_storage*}] 
set_false_path -from [get_registers {*litex_eth_buffer_bridge_inst|rx_overflow_reg*}] 
set_false_path -from [get_registers {*litex_eth_buffer_bridge_inst|rx_valid_reg*}] 
set_false_path -to [get_registers {*eth_buf_rx_valid_meta*}]
set_false_path -to [get_registers {*eth_buf_rx_len_latched*}]
set_false_path -from [get_registers {*eth_buf_tx_len*storage*}] 
set_false_path -from [get_registers {*eth_buf_rx_ack*storage*}] 


#**************************************************************
# Set Multicycle Path
#**************************************************************

set_multicycle_path -setup -end -to [get_registers {*b2v_wallclock|ppb_adj_reg[*]}] 2
set_multicycle_path -hold -end -to [get_registers {*b2v_wallclock|ppb_adj_reg[*]}] 1
set_multicycle_path -setup -end -from [get_registers {*media_clock_nsec_latch*}] -to [get_registers {*media_mult_reg*}] 2
set_multicycle_path -hold -end -from [get_registers {*media_clock_nsec_latch*}] -to [get_registers {*media_mult_reg*}] 1


#**************************************************************
# Set Maximum Delay
#**************************************************************



#**************************************************************
# Set Minimum Delay
#**************************************************************



#**************************************************************
# Set Input Transition
#**************************************************************

