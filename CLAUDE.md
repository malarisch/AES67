# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

AES67 professional audio-over-IP implementation using an FPGA (Cyclone 10LP) for the data plane and an MCU (STM32H753ZI running Zephyr RTOS) for the control plane. The FPGA handles Ethernet MAC, PTPv2, RTP audio packet processing, and media clock generation. The MCU handles DHCP, PTP Best Master Clock algorithm, clock generator control, and UI.

## Build Commands

### Firmware (Zephyr)
```bash
cd soc_firmware/app
west build -b nucleo_h753zi -p    # clean build
west build                         # incremental build
west flash                         # flash to board
```
Zephyr v4.2.0, west manifest at `soc_firmware/app/west-manifest/west.yml`.

### FPGA
Intel Quartus Prime 13.1.4, project file `FPGA/FPGA.qpf`, device 10CL025YU256I7G.

## Architecture

### Two-Domain Split
- **FPGA (`FPGA/`)**: ~28 VHDL/Verilog modules. Ethernet MAC (forked from YOL), PTPv2 leader+follower, wallclock discipline, media clock derivation, I2S audio input, RTP packet aggregation.
- **MCU (`soc_firmware/app/`)**: Zephyr C application. DHCP, PTP BMC algorithm, Si5351A clock generator driver (I2C), SSD1306 OLED display, network management.

### MCU-FPGA Bridge (FMC)
The FMC peripheral at `0x60000000` provides register-mapped access to the FPGA. Register map documented in `config_ram_address_map.md`. Key regions:
- `0x00-0x02`: ETH TX control/length
- `0x10-0x20`: ETH TX frame data
- `0x20-0x22`: ETH RX status/length
- `0x30-0x40`: ETH RX frame data
- `0x40`: MAC address (6-byte auto-increment write)
- `0x41`: IP address (4-byte auto-increment write)
- `0x50`: Status/control flags (PTP, wallclock, resets)
- `0x51`: Ethernet link status
- `0x52-0x54`: PTP metrics (path delay, offset, PPB)
- `0x55`: PTP config (leader identity, time source, logMessageInterval)

### Critical Hardware Constraints
1. **MPU region required**: FMC memory at `0x60000000` must be marked `ATTR_MPU_IO` in device tree to prevent Cortex-M7 speculative reads causing bus faults. Defined in `app.overlay`.
2. **FPGA ready gating**: MCU boots before FPGA. GPIO PC13 (active high) gates all FMC access — driver checks `is_fpga_ready()` before any bus operation.
3. **RX thread stack**: Must be 4096 bytes (`CONFIG_ETH_FMC_BASIC_RX_STACK_SIZE`) due to on-stack frame buffer allocation.
4. **Clock domain crossing**: FPGA PTP controller uses CDC synchronizers with PRESERVE attributes — do not remove.

## Key Source Files

### Firmware
- `soc_firmware/app/src/main.c` — Entry point, FPGA register config, DHCP, network setup
- `soc_firmware/app/src/ptp_bmc.c` — IEEE 1588 Best Master Clock algorithm (multicast 224.0.1.129:320)
- `soc_firmware/app/src/ui_display.c` — SSD1306 OLED debug display
- `soc_firmware/app/drivers/eth_fmc_basic/` — FMC Ethernet bridge driver (primary)
- `soc_firmware/app/drivers/eth_spi_basic/` — SPI Ethernet bridge driver (legacy, deprecated)
- `soc_firmware/app/drivers/si5351a/` — Si5351A I2C clock generator with PPB correction
- `soc_firmware/app/prj.conf` — Zephyr kernel/subsystem config
- `soc_firmware/app/app.overlay` — Device tree (FMC timing, MPU, I2C devices, GPIO)

### FPGA
- `FPGA/fmc_ethernet_client.vhd` — FMC bridge (FPGA side)
- `FPGA/ptpv2_controller.vhd` — PTP state machine (Sync, Follow_Up, Announce, Delay_Resp)
- `FPGA/ptpv2_servo.vhd` — Wallclock discipline algorithm
- `FPGA/wallclock.vhd` — 48-bit seconds + 32-bit nanoseconds PTP clock
- `FPGA/ethernet_packet_aggregator.vhd` — RTP audio packet construction
- `FPGA/system_config_reg.vhd` — Register interface for MCU-accessible config
- `FPGA/audio_clock_controller.vhd` — Media clock from PTP wallclock
- `FPGA/clock_ppb_meter.vhd` — PPB correction measurement for external PLL

## Conventions
- **FPGA**: VHDL preferred for new logic. Verilog used for some audio clock modules.
- **Firmware**: Follow Zephyr coding style and device tree conventions.
- **Bridge changes**: Updating the FMC protocol requires changes on BOTH sides — VHDL in `FPGA/fmc_ethernet_client.vhd` and C in `soc_firmware/app/drivers/eth_fmc_basic/`.
- **Verification**: When unsure about Zephyr APIs, STM32 pinmux, or interrupt semantics, read the source or check upstream docs rather than guessing.
- **Debugging**: Zephyr shell (`CONFIG_SHELL=y`) and logging (`LOG_INF`, `LOG_ERR`) are enabled. FPGA has debug UART output (`urt_dbg.vhd`).
