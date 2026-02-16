# Registers: Ethernet

Write Register 0x00 Eth TX Length LSB Impl
Write Register 0x01 Eth TX Length MSB Impl
Write Register 0x02 Eth TX CTRL Impl
Content Bit [0] => Reg TX Start

Write Register 0x22 Eth RX Status Clear Impl
Content Bit [0] => Reg RX Clear

Write Register 0x10 - 0x20 ETH TX FRAME - Length From 0x00 0x01 Impl

Write Register 0x40 - 6 Bytes MAC ADDR Impl
0x000 Mac Addr 0
0x001 Mac Addr 1
0x002 Mac Addr 2
0x003 Mac Addr 3
0x004 Mac Addr 4
0x005 Mac Addr 5


Write Register 0x41 - 4 Bytes IP ADDR Impl
0x006 IP Addr 0
0x007 IP Addr 1
0x008 IP Addr 2
0x009 IP Addr 4



Read Register 0x20 ETH RX Lengh LSB Impl
Read Register 0x21 ETH RX Length MSB Impl
Read Register 0x22 ETH RX Status Impl
Bit [0] Reg RX Ready
Bit [1] Reg RX Overflow

Read Register 0x30 - 0x40: ETH RX FRAME - Length MSB + LSB Impl

# Write Registers - PTP Configuration

Write Register 0x55 -- PTP Configuration - 11 Byte Write
FPGA RAM 0x010..0x17 Byte 1..8: Current Leader Clock Identity
FPGA RAM 0x018 Byte 9 PTP Time Source
FPGA RAM 0x019 Byte 10 ptp LogMessageInterval_Sync
FPGA RAM 0x020 Byte 11 ptp LogMessageInterval_Announce

# Write Registers - Audio Stream Destination

Write Register 0x57 -- Audio Destination IP + Port - 6 Byte Write
Byte 0: Destination IP Addr 0 (MSB, network byte order)
Byte 1: Destination IP Addr 1
Byte 2: Destination IP Addr 2
Byte 3: Destination IP Addr 3 (LSB)
Byte 4: Destination UDP Port MSB
Byte 5: Destination UDP Port LSB
Output: audio_dst_ip_o(31..0), audio_dst_port_o(15..0)

# Write Registers - TX Stream Config

Write Register 0x58 -- TX Stream Config - 20 Byte Write (per stream, base = stream_id * 32)
Byte 0:     stream_id (0..7)
Bytes 1-4:  Destination IP address (network byte order, MSB first)
Byte 5:     Channel count (1..8)
Byte 6:     Samples per packet per channel
Bytes 7-14: Channel IDs (up to 8, one byte each)
Byte 15:    Reserved
Bytes 16-19: SSRC (32-bit, big-endian, for RTP header)
Output: tx_stream_config_wr_en_o, tx_stream_config_wr_addr_o, tx_stream_config_wr_data_o

# Status Registers

Write Register 0x50 - Flag Bitmask
[0] Start PLL PPB Measurement; FPGA Implementation: sets output PLL_PBB_Measurment_start_o high until acknowledged by Input PLL_PBB_Measurement_valid_i going low. Then the output must be set to low as well. Note: The valid I is for the first measurement low, since no measurement was taken yet
[1] Reset Wallclock -- Implementation: Output pin reset_wallclock_o
[2] Reset PTP -- Implementation: Output pin reset_ptp_o
[3] Reset Ethernet -- Implementation: Output pin reset_ethernet_o
[4] PTP: Is Leader -- Implementattion: Output pin ptp_is_leader
[5] PTP: Is Follower -- Output pin
[6]
[7]

Read Register 0x50 - Flag Bitmask -- Clocking

[0] PLL PPB Measurement Valid -- simple input pin, needs cdc
[1] Wallclock Locked -- simple input pin, needs CDC
[2] Wallclock Did Phasejump -- simple input pin, needs CDC
[3] Wallclock Configured -- simple input pin, needs cdc
[4] PTP Leader lost -- simple input pin, needs cdc
[5]
[6]
[7]

Read Register 0x51 - Flag Bitmask -- Ethernet
[0] Ethernet Link up -- input pin, needs cdc
[1] Ethernet Link Speed 0 Bitmask: 00 10 Mbps - 01 100 Mbps - 10 1000 Mbps -- input, needs cdc
[2] Ethernet Link Speed 1 -- input pin, needs cdc
[3] 
[4]
[5]
[6]
[7]

Read Register 0x52 -- Path Delay -- Four Byte Read: Input path_delay_i (31..0)
Read Register 0x53 -- Leader Offset -- Four Byte Read: Input leader_offset_i (31..0)
Read Regiter  0x54 -- pll_ptp_ppb offset -- Four Byte Read: Input clock_ppb_meter_i (31..0) - only if pll_ppb_measurement_valid_i is high, if not output just zeros

Read Register 0x60 .. 0x7F
-- Direct Access to RAM (system_config_reg.vhd) - Address minus 0x60