## SDC Constraints for AES67 FPGA
## Device: 10CL025YU256I7G (Cyclone 10 LP)
##
## Clock structure (this build: soctype = "spi", USE_EXTERNAL_PLL = "false"):
##   clock_i (50 MHz)        -> sysclks_altpll_50m_in_inst -> 125 MHz sys clock
##                                                         -> 80 MHz mcu_clk
##   phy_rgmii_enet_clk_125m -> rgmiiclks_inst -> phy_rgmii_enet_tx_clk (0deg)
##                                             -> internal TX clock     (90deg)
##   phy_rgmii_enet_rx_clk   (125 MHz, 90deg shifted by PHY)
##   pll_512fs_i             -> audio_mclk    (only USE_EXTERNAL_PLL = "true")
##   wallclock NCO           -> nco_mclk      (only USE_EXTERNAL_PLL = "false")
##
## SDC ordering note:
##   PLL atom pins (altpll_component|auto_generated|pll1|*) only appear after
##   derive_pll_clocks runs. So this file is structured:
##     1. board-input clocks (real ports)
##     2. derive_pll_clocks   -> creates all PLL outputs under auto names
##     3. sub-SDCs (audioclocks/ptp/hyperram/...) -- now that all clocks exist
##     4. RGMII timing procs  -> create derived TX clocks
##     5. false paths and final clock groups (every clock now exists)

set_time_format -unit ns -decimal_places 3

#**************************************************************
# 1. Board-input clocks
#**************************************************************

create_clock -name {clock_i}        -period 20.000 -waveform { 0.000 10.000 } [get_ports {clock_i}]
create_clock -name {enet_clk_125m}  -period 8.000  -waveform { 0.000 4.000  } [get_ports {phy_rgmii_enet_clk_125m}]

#**************************************************************
# 2. Auto-derive every PLL-internal clock
#**************************************************************
# Generates clocks for sysclks_altpll_50m_in_inst (125 MHz, 80 MHz, ...) and
# rgmiiclks_inst (TX 0deg, TX 90deg) under their atom names, e.g.
#   top_inst|\sysclkgen50:sysclks_altpll_50m_in_inst|altpll_component|auto_generated|pll1|clk[0]
#   top_inst|\rgmiigen:rgmiiclks_inst|altpll_component|auto_generated|pll1|clk[0]

derive_pll_clocks
derive_clock_uncertainty

# Hierarchy paths to PLL outputs we reference below. Quartus encodes the
# generate-statement label as "\<label>:" in the netlist; the backslash is
# part of the identifier, not a Tcl escape, so it stays literal in {braces}.
set sys_pll_clk0      {soc_top_inst|sysclk_pll_gen_inst|\sysclkgen50:sysclks_altpll_50m_in_inst|altpll_component|auto_generated|pll1|clk[1]}
set sys_pll_clk1      {soc_top_inst|sysclk_pll_gen_inst|\sysclkgen50:sysclks_altpll_50m_in_inst|altpll_component|auto_generated|pll1|clk[1]}
set rgmii_pll_clk0    {soc_top_inst|mii_converters_inst|\rgmiigen:rgmiiclks_inst|altpll_component|auto_generated|pll1|clk[0]}

#**************************************************************
# 3. Per-feature sub-SDCs (after derive_pll_clocks)
#**************************************************************

#source ../../../sdc/audioclocks.sdc
source ../../../sdc/hyperram.sdc
source ../../../sdc/ptp.sdc
source ../../../sdc/litex_csr.sdc
#source ../../../sdc/spictrl.sdc

#**************************************************************
# 4. RGMII timing
#**************************************************************
# rgmii_io.sdc procs need:
#   - clock source as a *pin path* (the rgmii TX PLL clk[0] atom output)
#   - external clock pin as a *port*

source ../../../sdc/rgmii_phy_if.sdc
source ../../../sdc/rgmii_io.sdc

constrain_rgmii_input_pins  "enet" "phy_rgmii_enet_rx_clk" "phy_rgmii_enet_rx_dv phy_rgmii_enet_rx_d*"
constrain_rgmii_output_pins "enet" "$rgmii_pll_clk0"       "phy_rgmii_enet_tx_clk" "phy_rgmii_enet_tx_en phy_rgmii_enet_tx_d*"

#**************************************************************
# 5a. SPI slave (control interface from MCU)
#**************************************************************
# spictrl is a SPI slave clocked by spictrl_clk_i (driven by the external
# MCU). It is internally double-synchronised into sys_clk (see spi_slave.vhd
# CDC chain), so we declare spictrl_clk_i as its own async clock and treat
# all paths between it and sys_clk as false.
#
# Conservative SPI rate: 25 MHz max (40 ns period). Bump if the MCU drives
# faster — but the spi_slave does CDC, not source-synchronous capture, so
# the input-delay numbers are mostly informational.

create_clock -name {spictrl_clk} -period 40.000 [get_ports {spictrl_clk_i}]

set_input_delay  -clock {spictrl_clk} -max 5 [get_ports {spictrl_mosi_i spictrl_cs_n_i}]
set_input_delay  -clock {spictrl_clk} -min 0 [get_ports {spictrl_mosi_i spictrl_cs_n_i}]
set_output_delay -clock {spictrl_clk} -max 5 [get_ports {spictrl_miso_o}]
set_output_delay -clock {spictrl_clk} -min 0 [get_ports {spictrl_miso_o}]

# IRQ to MCU is a level/edge with no critical timing.
set_false_path -from * -to [get_ports {spictrl_irq_n_o}]

#**************************************************************
# 5b. Audio I/O — synchronous to the audio clock domain
#**************************************************************
# All audio I/O is external to the FPGA but stays inside one source-
# synchronous domain (the FPGA generates both the audio clocks and the
# data/lrclk). The downstream ADC/DAC sees the same clock edges; FPGA
# internal launch/capture timing is what matters and is already covered
# by the audio clock declarations in audioclocks.sdc. We mark the pins
# false_path here so unconstrained-port reports stop flagging them.
#
# pll_256fs_rising / pll_256fs_falling : ADC/DAC bit clocks
# pll_512fs_o                           : MCLK passthrough (port to port)
# lrclk_o / lrclk_tdm_o                 : frame syncs
# tdm8out_0_o / tdm8out_1_o             : serial audio out
# tdm8in_0_i  / tdm8in_1_i              : serial audio in

#set_false_path -from * -to [get_ports {pll_256fs_rising pll_256fs_falling pll_512fs_o lrclk_o lrclk_tdm_o tdm8out_0_o tdm8out_1_o}]
#set_false_path -from [get_ports {tdm8in_0_i tdm8in_1_i}] -to *

#**************************************************************
# 5c. False paths — slow / async pins
#**************************************************************

# MIIM Management Interface (slow async, ~2.5 MHz max)
set_false_path -from * -to [get_ports {enet_mdc}]
set_false_path -from * -to [get_ports {enet_mdio}]
set_false_path -from [get_ports {enet_mdio}] -to *

# PHY reset (async output)
set_false_path -from * -to [get_ports {phy_rgmii_enet_resetn}]

# User LED (async, visual only)
set_false_path -from * -to [get_ports {user_led[*]}]

# ADDA board reset (async output to external board)
set_false_path -from * -to [get_ports {adda_nRST}]

# Async reset input
set_false_path -from [get_ports {rst_n_i}] -to *

#**************************************************************
# 6. Asynchronous clock groups — final
#**************************************************************
# Domains:
#   - sys clock: clock_i + 125 MHz PLL output
#   - audio: audio_mclk + nco_mclk + derived (audio_mclk vs nco_mclk handled
#            -exclusive inside audioclocks.sdc)
#   - RGMII TX: PHY-tx port + the rgmii TX PLL clk[0]
#   - RGMII RX: PHY rx clock + virtual rx clock + derived rx clock
#
# Reference PLL outputs by their atom paths (same set ... above) — this is
# the only name they are guaranteed to have post-derive_pll_clocks.

set_clock_groups -asynchronous \
    -group [get_clocks [list clock_i $sys_pll_clk0]] \
	 -group [get_clocks [list $sys_pll_clk1]] \
    -group [get_clocks [list phy_rgmii_enet_tx_clk $rgmii_pll_clk0 enet_tx_clk_125m]] \
    -group [get_clocks {enet_clk_125m enet_rx_clk_125m virt_enet_rx_clk_125m phy_rgmii_enet_rx_clk}] \
    -group [get_clocks {spictrl_clk}]
