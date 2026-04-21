

# HyperRAM (LiteX SDR PHY, 4:1 ratio, all logic in litex_sys_clk domain)
# Internal register-to-register timing is covered by litex_sys_clk (12.5ns).
# I/O paths need max_delay constraints to ensure registers are placed in/near
# I/O cells, preventing excessive routing delay that causes read data errors.
#
# Control signals (active before/after transfers, not timing-critical)
set_false_path -from * -to [get_ports {hbus_rstn hbus_cs2n}]
#
# Clock output: must have minimal and matched skew between P/N
set_max_delay -from [get_clocks {litex_sys_clk}] -to [get_ports {hbus_clk0_p}] 6.0
set_max_delay -from [get_clocks {litex_sys_clk}] -to [get_ports {hbus_clk0_n}] 6.0
#
# DQ output: data must be stable before HyperRAM samples on hbus_clk edge
set_max_delay -from [get_clocks {litex_sys_clk}] -to [get_ports {hbus_dq[*]}] 6.0
#
# DQ input: data from HyperRAM must meet setup to litex_sys_clk register
set_max_delay -from [get_ports {hbus_dq[*]}] -to [get_clocks {litex_sys_clk}] 6.0
#
# RWDS output: strobe/mask timing during writes
set_max_delay -from [get_clocks {litex_sys_clk}] -to [get_ports {hbus_rwds}] 6.0
#
# RWDS input: strobe edge detection must meet setup to litex_sys_clk
set_max_delay -from [get_ports {hbus_rwds}] -to [get_clocks {litex_sys_clk}] 6.0
