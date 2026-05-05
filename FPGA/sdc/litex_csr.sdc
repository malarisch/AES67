# --- PTP is_leader/is_follower: litex_sys_clk -> sys_clk_125m ---
# Slow-changing configuration signals from LiteX SoC CSR to PTP modules
set_false_path -from [get_registers {*aes67_csr_ctrl_storage*}] -to [get_registers {*ptp_inst|b2v_controller|*}]
set_false_path -from [get_registers {*aes67_csr_ctrl_storage*}] -to [get_registers {*ptp_inst|b2v_ptpparser|*}]

# --- tx_router threshold_shadow CDC (config_wr_clk -> sys_clk) ---
# threshold_shadow is written once at boot during stream setup from
# config_wr_clk domain and read in sys_clk sample counting logic.
# Single-write, static thereafter — no bus-consistency requirement.
set_false_path -from [get_registers {*audiotx_inst|b2v_tx_router|threshold_shadow[*][*]}]

# --- tx_router tx_en CDC (tx_transmitter domain -> sys_clk) ---
set_false_path -to [get_registers {*audiotx_inst|b2v_tx_router|tx_en_meta}]

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
# The StreamConfigRAM is a Wishbone slave (not a CSR), so its write signals are
# driven directly from the SoC bus. The tx_router config_wr_clk_i is litex_sys_clk
# and writes are slow (stream setup only). The shadow registers in tx_router are
# covered by the async clock group separation (litex_sys_clk vs sys_clk_125m).

# --- Ethernet packet buffer CDC (mac_rx/mac_tx <-> litex_sys_clk) ---
# CDC handled by dual-port SRAM and synchronizers in LiteX EthPacketBuffer

# litex_eth_buffer_bridge (litex_eth_inst): enet_rx_clk_125m -> litex_sys_clk
set_false_path -from [get_registers {*litex_eth_inst|rx_overflow_reg*}]
set_false_path -from [get_registers {*litex_eth_inst|rx_valid_reg*}]
set_false_path -from [get_registers {*litex_eth_inst|buf_rx_len_o[*]}]

# LiteX SoC internal RX buffer CDC synchronizers
set_false_path -to [get_registers {*eth_buf_rx_valid_meta*}]
set_false_path -to [get_registers {*eth_buf_rx_len_latched*}]

# LiteX SoC TX buffer length and ack crossing litex_sys_clk -> mac_tx
set_false_path -from [get_registers {*eth_buf_tx_len*storage*}]
set_false_path -from [get_registers {*eth_buf_rx_ack*storage*}]

# --- I/O port false paths (directly driven by LiteX SoC, no FPGA-side timing) ---
# Serial UARTs
set_false_path -from * -to [get_ports {uart0_tx}]
set_false_path -from [get_ports {uart0_rx}] -to *
#set_false_path -from * -to [get_ports {gpio4}]
#set_false_path -from [get_ports {arduino_io3}] -to *

# I2C (directly connected to LiteX SoC, open-drain, slow)
set_false_path -from * -to [get_ports {i2c0_scl i2c0_sda}]
set_false_path -from [get_ports {i2c0_scl i2c0_sda}] -to *
set_false_path -from * -to [get_ports {i2c1_scl i2c1_sda}]
set_false_path -from [get_ports {i2c1_scl i2c1_sda}] -to *


# --- PTP is_leader/is_follower: litex_sys_clk -> sys_clk_125m ---
# Slow-changing configuration signals from LiteX SoC CSR to PTP modules
set_false_path -from [get_registers {*aes67_csr_ctrl_storage*}] -to [get_registers {*ptp_inst|b2v_controller|*}]
set_false_path -from [get_registers {*aes67_csr_ctrl_storage*}] -to [get_registers {*ptp_inst|b2v_ptpparser|*}]

# --- tx_router shadow register CDC ---
# samples_per_packet_shadow and threshold_shadow no longer have separate _sync
# registers (CDC removed from tx_router). Shadow registers are written from
# config_wr_clk domain and read in sys_clk — covered by stream_cfg false_path below.

#- LiteX SoC reset signals - static during normal operation ---
set_false_path -from [get_registers {*aes67soc_reset_storage*}] -to *
set_false_path -from [get_registers {*aes67soc_reset_re*}] -to *

# --- LiteX SoC CSR registers crossing to FPGA logic domains ---
# All aes67_csr_* storage registers live in litex_sys_clk (80 MHz) and
# drive signals consumed in sys_clk_125m, enet_rx_clk, or audio_mclk domains.
# CDC is handled by 2-FF synchronizers in the FPGA logic.

# Control register (pll_ppb_start, ptp_is_leader, ptp_is_follower, eth_tx_request)
set_false_path -from [get_registers {*aes67_csr_ctrl_storage*}]

# Static configuration registers
set_false_path -from [get_registers {*aes67_csr*}]
set_false_path -from [get_registers {*aes67_csr_mac_addr*storage*}]
set_false_path -from [get_registers {*aes67_csr_ip_addr*storage*}]
set_false_path -from [get_registers {*aes67_csr_ptp_leader_id*storage*}]
set_false_path -from [get_registers {*aes67_csr_ptp_announce*storage*}]
set_false_path -from [get_registers {*aes67_csr_ptp_log*storage*}]
set_false_path -from [get_registers {*aes67_csr_ptp_time_source*storage*}]

# ADDA board reset (async output to external board)
set_false_path -from * -to [get_ports {adda_nRST}]
