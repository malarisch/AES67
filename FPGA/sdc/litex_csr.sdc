# litex_csr.sdc — false paths for LiteX-SoC CSR / packet-buffer crossings into
# the FPGA data plane.
#
# Hierarchy updated for the Wishbone-bridge refactor (2026-06):
#   - the AES67 register surface (aes67_csr_*) lives in the aes67_bridge SoC,
#   - the eth buffer bridge instance is now litex_eth_buffer_bridge_inst,
#   - ptp_leader_id_hi/lo became CSRStatus (no *_storage),
#   - the SoC reset CSR has no *_re net (only *_storage / *_wr_stb),
#   - the adda_nRST output port is currently disabled in topevalkit.vhd.
# Leading '*' absorbs the instance prefix (soc_top_inst|aes67_wb_bridge_inst|...).

set_false_path -from [get_registers {*speed_o*}] -to *
# --- AES67 control register -> PTP modules (litex_sys_clk -> sys_clk_125m) ---
# Slow-changing config; CDC handled by 2-FF synchronizers in FPGA logic.
set_false_path -from [get_registers {*aes67_csr_ctrl_storage*}] -to [get_registers {*ptp_inst|b2v_controller|*}]
set_false_path -from [get_registers {*aes67_csr_ctrl_storage*}] -to [get_registers {*ptp_inst|b2v_ptpparser|*}]

# --- All AES67 CSRStorage config registers crossing into FPGA logic domains ---
# (ctrl, mac_addr, ip_addr, ptp_*, servo_*, gm_*, meter_clear, ...). Static/slow
# config written once from litex_sys_clk and resynchronised in the consumer.
set_false_path -from [get_registers {*aes67_csr*storage*}]
# Note: ptp_leader_id_hi/lo are CSRStatus (FPGA BMA -> CPU read), so there is no
# *_storage to constrain; that read path is cut by the async clock groups.

# --- LiteX SoC reset (static during normal operation) ---
set_false_path -from [get_registers {*aes67soc_reset_storage*}] -to *

# --- tx_router shadow / tx_en CDC ---
# threshold_shadow written once at stream setup (config_wr_clk); tx_en_meta is
# the first sync FF of the tx_transmitter->sys_clk crossing.
set_false_path -from [get_registers {*audiotx_inst|b2v_tx_router|threshold_shadow[*][*]}]
set_false_path -to   [get_registers {*audiotx_inst|b2v_tx_router|tx_en_meta}]

# --- Ethernet packet buffer bridge (enet_rx_clk_125m -> litex_sys_clk) ---
set_false_path -from [get_registers {*litex_eth_buffer_bridge_inst|rx_overflow_reg*}]
set_false_path -from [get_registers {*litex_eth_buffer_bridge_inst|rx_valid_reg*}]
set_false_path -from [get_registers {*litex_eth_buffer_bridge_inst|buf_rx_len_o[*]}]

# LiteX EthPacketBuffer internal RX CDC synchronizers
set_false_path -to [get_registers {*eth_buf_rx_valid_meta*}]
set_false_path -to [get_registers {*eth_buf_rx_len_latched*}]

# TX buffer length / RX ack crossing litex_sys_clk -> mac_tx
set_false_path -from [get_registers {*eth_buf_tx_len*storage*}]
set_false_path -from [get_registers {*eth_buf_rx_ack*storage*}]

# --- I/O ports directly driven by LiteX SoC (slow / async, no FPGA-side timing) ---
set_false_path -from * -to [get_ports {uart0_tx}]
set_false_path -from [get_ports {uart0_rx}] -to *
set_false_path -from * -to [get_ports {i2c0_scl i2c0_sda}]
set_false_path -from [get_ports {i2c0_scl i2c0_sda}] -to *

# Note: the adda_nRST output port is currently commented out in topevalkit.vhd
# (driven internally as aes67_ctrl_adda_nrst but not routed to a pin). Re-add a
# "set_false_path -from * -to [get_ports {adda_nRST}]" here once it is re-enabled.
