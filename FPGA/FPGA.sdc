## SDC Constraints for AES67 FPGA
## Device: 10CL025YU256I7G (Cyclone 10 LP)
##
## Clock Structure:
##   c10_clk50m (50 MHz) -> PLL inst6 -> sys_clk_125m (125 MHz)
##   c10_clk50m (50 MHz) -> PLL inst  -> litex_sys_clk (80 MHz)
##   enet_clk_125m (125 MHz) -> PLL inst9 -> RGMII TX clocks
##   enet_rx_clk (125 MHz, 90° phase shifted by PHY)
##   gpio34 (24.576 MHz = 48kHz * 512 Audio MCLK)
##   Audio clock divider chain: pll_fs512 -> pll_fs256 -> pll_fs128 -> pll_fs64 -> fs -> clk_1kHz

#**************************************************************
# Time Information
#**************************************************************

set_time_format -unit ns -decimal_places 3


#**************************************************************
# Create Clock
#**************************************************************

# Primary 50 MHz oscillator input to PLLs
create_clock -name {c10_clk50m} -period 20.000 -waveform { 0.000 10.000 } [get_ports {c10_clk50m}]

# Audio master clock from external PLL (48kHz * 512 = 24.576 MHz)
create_clock -name {audio_mclk} -period 40.690 -waveform { 0.000 20.345 } [get_ports {gpio34}]

# RGMII 125 MHz reference clock from PHY
create_clock -name {enet_clk_125m} -period 8.000 -waveform { 0.000 4.000 } [get_ports {enet_clk_125m}]


#**************************************************************
# Create Generated Clock
#**************************************************************

# PLL inst6 output clk[0]: 125 MHz system clock (50 * 5 / 2)
create_generated_clock -name {sys_clk_125m} \
    -source [get_pins {inst6|altpll_component|auto_generated|pll1|inclk[0]}] \
    -duty_cycle 50.00 -multiply_by 5 -divide_by 2 \
    -master_clock {c10_clk50m} \
    [get_pins {inst6|altpll_component|auto_generated|pll1|clk[0]}]

# LiteX SoC PLL (inst) output: 80 MHz sys_clk (50 * 8 / 5)
create_generated_clock -name {litex_sys_clk} \
    -source [get_pins {inst|ALTPLL|auto_generated|pll1|inclk[0]}] \
    -multiply_by 8 -divide_by 5 \
    -master_clock {c10_clk50m} \
    [get_pins {inst|ALTPLL|auto_generated|pll1|clk[0]}]

# RGMII TX PLL (inst9) output clk[0]: 125 MHz TX internal clock
create_generated_clock -name {inst9|altpll_component|auto_generated|pll1|clk[0]} \
    -source [get_pins {inst9|altpll_component|auto_generated|pll1|inclk[0]}] \
    -duty_cycle 50.00 -multiply_by 1 -divide_by 1 \
    -master_clock {enet_clk_125m} \
    [get_pins {inst9|altpll_component|auto_generated|pll1|clk[0]}]

# RGMII TX PLL (inst9) output clk[1]: 125 MHz with 90° phase shift (for DDR TX timing)
create_generated_clock -name {inst9|altpll_component|auto_generated|pll1|clk[1]} \
    -source [get_pins {inst9|altpll_component|auto_generated|pll1|inclk[0]}] \
    -duty_cycle 50.00 -multiply_by 1 -divide_by 1 \
    -phase 90.0 \
    -master_clock {enet_clk_125m} \
    [get_pins {inst9|altpll_component|auto_generated|pll1|clk[1]}]

# Audio clock divider chain (from audio_mclk = pll_fs512)
# inst33: pll_fs512 / 2 = pll_fs256
create_generated_clock -name {pll_fs256} \
    -source [get_ports {gpio34}] \
    -divide_by 2 \
    -master_clock {audio_mclk} \
    [get_registers {clk_by_x:inst33|b}]

# inst16: pll_fs256 / 2 = pll_fs128
create_generated_clock -name {pll_fs128} \
    -source [get_registers {clk_by_x:inst33|b}] \
    -divide_by 2 \
    -master_clock {pll_fs256} \
    [get_registers {clk_by_x:inst16|b}]

# inst27: pll_fs128 / 2 = pll_fs64
create_generated_clock -name {pll_fs64} \
    -source [get_registers {clk_by_x:inst16|b}] \
    -divide_by 2 \
    -master_clock {pll_fs128} \
    [get_registers {clk_by_x:inst27|b}]

# inst29: pll_fs64 / 64 = fs (48 kHz)
create_generated_clock -name {fs} \
    -source [get_registers {clk_by_x:inst27|b}] \
    -divide_by 64 \
    -master_clock {pll_fs64} \
    [get_registers {clk_by_x:inst29|b}]

# inst5: fs / 48 = clk_1kHz
create_generated_clock -name {clk_1kHz} \
    -source [get_registers {clk_by_x:inst29|b}] \
    -divide_by 48 \
    -master_clock {fs} \
    [get_registers {clk_by_x:inst5|b}]


#**************************************************************
# Set Clock Groups
#**************************************************************

# Clock domains:
# 1. System 125 MHz domain: c10_clk50m -> PLL inst6 -> sys_clk_125m
# 2. LiteX SoC domain: c10_clk50m -> PLL inst -> litex_sys_clk (80 MHz)
# 3. Audio domain: audio_mclk and derived divider clocks (async external oscillator)
# 4. RGMII TX domain: enet_clk_125m -> PLL inst9 -> TX clocks
# 5. RGMII RX domain: enet_rx_clk (from PHY, created in rgmii_io.sdc)
#
# Note: sys_clk_125m and litex_sys_clk share the same source (c10_clk50m)
# but have unrelated PLL outputs and no synchronous phase relationship,
# so they are treated as asynchronous.

set_clock_groups -asynchronous \
    -group [get_clocks {c10_clk50m sys_clk_125m}] \
    -group [get_clocks {litex_sys_clk}] \
    -group [get_clocks {audio_mclk pll_fs256 pll_fs128 pll_fs64 fs clk_1kHz}] \
    -group [get_clocks {enet_clk_125m inst9|altpll_component|auto_generated|pll1|clk[0] inst9|altpll_component|auto_generated|pll1|clk[1]}]


#**************************************************************
# Set False Path
#**************************************************************

# --- MIIM Management Interface (slow async, ~2.5 MHz max) ---
set_false_path -from * -to [get_ports {enet_mdc}]
set_false_path -from * -to [get_ports {enet_mdio}]
set_false_path -from [get_ports {enet_mdio}] -to *
set_false_path -from * -to [get_ports {enet_resetn}]

# --- CDC synchronizers in PTP logic (inside ptp_module inst24) ---
set_false_path -from [get_registers {*inst24|inst35|tx_enable}] -to [get_registers {*inst24|inst34|tx_en_i_meta}]
set_false_path -to [get_registers {*inst24|inst14|parse_ptp_packet_meta}]

# --- PTP is_leader/is_follower: litex_sys_clk -> sys_clk_125m ---
# Slow-changing configuration signals from LiteX SoC CSR to PTP modules
set_false_path -from [get_registers {*aes67_csr_ctrl_storage*}] -to [get_registers {*inst24|inst34|*}]
set_false_path -from [get_registers {*aes67_csr_ctrl_storage*}] -to [get_registers {*inst24|inst14|*}]

# --- tx_router shadow register CDC (litex_sys_clk -> sys_clk_125m) ---
# Slow-changing config values updated only during stream setup
set_false_path -from [get_registers {*inst13|inst37|samples_per_packet_shadow[*][*]}] -to [get_registers {*inst13|inst37|samples_per_packet_sync[*][*]}]
set_false_path -from [get_registers {*inst13|inst37|threshold_shadow[*][*]}] -to [get_registers {*inst13|inst37|threshold_sync[*][*]}]

# --- tx_router tx_en CDC (tx_transmitter domain -> sys_clk) ---
set_false_path -to [get_registers {*inst13|inst37|tx_en_meta}]

# --- LiteX SoC reset signals - static during normal operation ---
set_false_path -from [get_registers {*aes67soc_reset_storage*}] -to *
set_false_path -from [get_registers {*aes67soc_reset_re*}] -to *

# --- LiteX SoC CSR registers crossing to FPGA logic domains ---
# All aes67_csr_* storage registers live in litex_sys_clk (80 MHz) and
# drive signals consumed in sys_clk_125m, enet_rx_clk, or audio_mclk domains.
# CDC is handled by 2-FF synchronizers in the FPGA logic.

# Control register (pll_ppb_start, ptp_is_leader, ptp_is_follower, eth_tx_request)
set_false_path -from [get_registers {*aes67_csr_ctrl_storage*}]

# Static configuration registers
set_false_path -from [get_registers {*aes67_csr_mac_addr*storage*}]
set_false_path -from [get_registers {*aes67_csr_ip_addr*storage*}]
set_false_path -from [get_registers {*aes67_csr_ptp_leader_id*storage*}]
set_false_path -from [get_registers {*aes67_csr_ptp_announce*storage*}]
set_false_path -from [get_registers {*aes67_csr_ptp_log*storage*}]
set_false_path -from [get_registers {*aes67_csr_ptp_time_source*storage*}]

# Stream config RAM write signals: litex_sys_clk -> FPGA logic
# These are directly driven from LiteX Wishbone bus outputs (no intermediate registers
# at the port boundary), so we match the SoC-internal source registers
set_false_path -from [get_registers {*stream_cfg*storage*}]

# --- Ethernet packet buffer CDC (mac_rx/mac_tx <-> litex_sys_clk) ---
# CDC handled by dual-port SRAM and synchronizers in LiteX EthPacketBuffer

# litex_eth_buffer_bridge (inst8): enet_rx_clk_125m -> litex_sys_clk
set_false_path -from [get_registers {*inst8|rx_overflow_reg*}]
set_false_path -from [get_registers {*inst8|rx_valid_reg*}]
set_false_path -from [get_registers {*inst8|buf_rx_len_o[*]}]

# LiteX SoC internal RX buffer CDC synchronizers
set_false_path -to [get_registers {*eth_buf_rx_valid_meta*}]
set_false_path -to [get_registers {*eth_buf_rx_len_latched*}]

# LiteX SoC TX buffer length and ack crossing litex_sys_clk -> mac_tx
set_false_path -from [get_registers {*eth_buf_tx_len*storage*}]
set_false_path -from [get_registers {*eth_buf_rx_ack*storage*}]

# --- I/O port false paths (directly driven by LiteX SoC, no FPGA-side timing) ---
# Serial UARTs
set_false_path -from * -to [get_ports {arduino_io2}]
set_false_path -from [get_ports {arduino_io1}] -to *
set_false_path -from * -to [get_ports {gpio4}]
set_false_path -from [get_ports {arduino_io3}] -to *

# I2C (directly connected to LiteX SoC, open-drain, slow)
set_false_path -from * -to [get_ports {gpio0 gpio1 gpio2 gpio3}]
set_false_path -from [get_ports {gpio0 gpio1 gpio2 gpio3}] -to *

# SPI (directly connected to LiteX SoC, max ~400 kHz for SD card init)
set_false_path -from * -to [get_ports {gpio5 gpio6 gpio7}]
set_false_path -from [get_ports {gpio8}] -to *

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

# User LED
set_false_path -from * -to [get_ports {user_led[*]}]

# Reset input
set_false_path -from [get_ports {c10_resetn}] -to *


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
