//Copyright (C)2014-2026 GOWIN Semiconductor Corporation.
//All rights reserved.
//File Title: Timing Constraints file
//Tool Version: V1.9.12 
//Created Time: 2026-05-01 16:26:03
create_clock -name rmii_refclk -period 20 -waveform {0 10} [get_ports {rmii_ref_clk}] -add
create_clock -name clk_50Mhz -period 20 -waveform {0 10} [get_ports {clk_50m}] -add
create_generated_clock -name nco_mclk -source [get_ports {clk_50m}] -master_clock clk_50Mhz -divide_by 40 -multiply_by 36 [get_nets {top_inst/aes67_top_inst/wc_mclk}]
create_generated_clock -name tdm_bclk_f_d -source [get_ports {clk_50m}] -master_clock clk_50Mhz -divide_by 80 -multiply_by 36 -invert [get_pins {top_inst/aes67_top_inst/audioclocks_inst/bclk_f_r_s0/Q}]
create_generated_clock -name mac_clk -source [get_ports {rmii_ref_clk}] -master_clock rmii_refclk -divide_by 2 -add [get_pins {top_inst/rmiigen_gowin.rmii_phy_if_inst_gowin/u_rmii_txrx/u_clk_gen/clk_div_s3/Q}]
set_clock_uncertainty 4 -setup -from [get_clocks {nco_mclk}] -to [get_clocks {clk_50Mhz}] 
set_false_path -from [get_clocks {rmii_refclk}] -to [get_clocks {clk_50Mhz}] 
set_false_path -from [get_clocks {clk_50Mhz}] -to [get_clocks {rmii_refclk}] 
set_false_path -from [get_clocks {rmii_refclk}] -to [get_clocks {mac_clk}] 
set_false_path -from [get_clocks {mac_clk}] -to [get_clocks {rmii_refclk}] 
set_false_path -from [get_clocks {mac_clk}] -to [get_clocks {clk_50Mhz}] 
set_false_path -from [get_clocks {clk_50Mhz}] -to [get_clocks {mac_clk}] 
set_clock_groups -asynchronous -group [get_clocks {clk_50Mhz nco_mclk tdm_bclk_f_d}] -group [get_clocks {rmii_refclk mac_clk}]
set_false_path -from [get_regs {top_inst/spigen.spictrl_inst/mac_address_o*}] 
set_false_path -from [get_regs {top_inst/spigen.spictrl_inst/ip_address_o*}] 
set_false_path -from [get_regs {top_inst/spigen.spictrl_inst/servo_*}] 
set_false_path -to [get_regs {top_inst/spigen.spictrl_inst/servo_*}] 
set_multicycle_path -to [get_regs {*clock_configure_timestamp_seconds_o*}]  -setup -end 8
set_multicycle_path -to [get_regs {*clock_configure_timestamp_seconds_o*}]  -hold -end 7
set_multicycle_path -to [get_regs {*ns_sum*}]  -setup -end 8
set_multicycle_path -to [get_regs {*ns_sum*}]  -hold -end 7
set_multicycle_path -to [get_regs {*clock_configure_timestamp_nanoseconds_o*}]  -setup -end 8
set_multicycle_path -to [get_regs {*clock_configure_timestamp_nanoseconds_o*}]  -hold -end 7
set_multicycle_path -to [get_regs {*pi_input*}]  -setup -end 4
set_multicycle_path -to [get_regs {*pi_input*}]  -hold -end 3
set_multicycle_path -to [get_regs {*pi_proportional*}]  -setup -end 4
set_multicycle_path -to [get_regs {*pi_proportional*}]  -hold -end 3
set_multicycle_path -to [get_regs {*pi_int_update*}]  -setup -end 4
set_multicycle_path -to [get_regs {*pi_int_update*}]  -hold -end 3
set_multicycle_path -to [get_regs {*pi_sum_raw*}]  -setup -end 4
set_multicycle_path -to [get_regs {*pi_sum_raw*}]  -hold -end 3
set_multicycle_path -to [get_regs {*integral_sum*}]  -setup -end 4
set_multicycle_path -to [get_regs {*integral_sum*}]  -hold -end 3
set_multicycle_path -to [get_regs {*freq_correction*}]  -setup -end 4
set_multicycle_path -to [get_regs {*freq_correction*}]  -hold -end 3
set_multicycle_path -to [get_regs {*ann_better_r*}]  -setup -end 2
set_multicycle_path -to [get_regs {*ann_better_r*}]  -hold -end 1
set_multicycle_path -to [get_regs {*ext_beats_self_r*}]  -setup -end 2
set_multicycle_path -to [get_regs {*ext_beats_self_r*}]  -hold -end 1
