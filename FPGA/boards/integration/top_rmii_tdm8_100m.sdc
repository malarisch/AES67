# ----------------------------------------------------------------------------
# Timing constraints for top_rmii_tdm8_100m (AES67 core, 100 MHz sys_clk).
#
# Template for the integrating project: the port names below are the ports
# of top_rmii_tdm8_100m. When the core sits below another top level,
# constrain the clocks on the integrating design's own ports / PLL outputs
# instead and keep the rest. Set aes67_sdc_dir to the AES67 FPGA/sdc folder.
#
# Checked with Quartus 25.1 on a 10CL025 C8 (slowest speed grade): all
# corners met at 100 MHz.
# ----------------------------------------------------------------------------
set_time_format -unit ns -decimal_places 3

if {![info exists aes67_sdc_dir]} {
    set aes67_sdc_dir ../../../sdc
}

create_clock -name {sys_clk}          -period 10.000 [get_ports {sys_clk_i}]
create_clock -name {phy_rmii_ref_clk} -period 20.000 [get_ports {rmii_ref_clk_i}]
# external PLL MCLK (CS2000 output), 24.576 MHz
create_clock -name {ext_mclk}         -period 40.690 [get_ports {ext_mclk_i}]
create_clock -name {spi_clk}          -period 40.000 [get_ports {spi_clk_i}]

# RMII -> MII: 25 MHz MAC clocks divided from the 50 MHz reference
create_generated_clock -name {mac_mii_rxc} -source [get_ports {rmii_ref_clk_i}] \
    -divide_by 2 [get_registers {*mac_mii_rxc*}]
create_generated_clock -name {mac_mii_txc} -source [get_ports {rmii_ref_clk_i}] \
    -divide_by 2 [get_registers {*mac_mii_txc*}]

derive_clock_uncertainty

set sys_pll_clk0 {sys_clk}
source $aes67_sdc_dir/audioclocks.sdc
source $aes67_sdc_dir/ptp.sdc
source $aes67_sdc_dir/litex_csr.sdc

# All clock domains are crossed with synchronisers / handshakes.
set_clock_groups -asynchronous \
    -group [get_clocks {sys_clk nco_mclk clk_256fs}] \
    -group [get_clocks {phy_rmii_ref_clk mac_mii_rxc mac_mii_txc}] \
    -group [get_clocks {ext_mclk}] \
    -group [get_clocks {spi_clk}]

# Slow / asynchronous pins
set_false_path -from [get_ports {rst_n_i}] -to *
set_false_path -from * -to [get_ports {irq_o}]
