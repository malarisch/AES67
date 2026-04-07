# AES67

A full AES67 Audio-over-IP implementation on a single FPGA. A Cyclone 10LP handles both the data plane (Ethernet MAC, PTPv2, RTP audio) and the control plane (LiteX RISC-V SoC running Zephyr RTOS). Some code was LLM generated, but human-checked and debugged.

For transparency, this is primarily a learning project. I had no FPGA experience before and only basic embedded experience (ESP32 + temperature sensor level).

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           FPGA (Cyclone 10LP)                              │
│                                                                            │
│  ┌─────────────┐   ┌─────────────┐   ┌─────────────┐   ┌───────────────┐  │
│  │ Ethernet    │   │   PTPv2     │   │  Wallclock  │   │  Audio TX/RX  │  │
│  │ MAC (YOL)   │◄──┤ Controller  │──►│  48b:32b    │──►│  RTP Packets  │  │
│  │ + Timestamp │   │ + Servo PI  │   │ + Media Clk │   │  + I2S I/O    │  │
│  └──────┬──────┘   └─────────────┘   └──────┬──────┘   └───────────────┘  │
│         │                                   │                              │
│         │     ┌─────────────────────────────┴──────────────────┐           │
│         │     │       Wishbone Bus / LiteX CSR Interface       │           │
│         │     │  - Register-mapped config                      │           │
│         │     │  - ETH TX/RX packet buffers (dual-port RAM)    │           │
│         └─────┴────────────────────────────────────────────────┘           │
│                                   ▲                                        │
│                                   │                                        │
│  ┌────────────────────────────────┼───────────────────────────────────┐    │
│  │                  LiteX VexRiscv SoC (80 MHz)                       │    │
│  │                                                                    │    │
│  │  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐             │    │
│  │  │  Zephyr RTOS │  │  HyperRAM    │  │  SPI Flash   │             │    │
│  │  │  (Firmware)  │  │  (16 MB RAM) │  │  (Boot+FW)   │             │    │
│  │  └──────────────┘  └──────────────┘  └──────────────┘             │    │
│  │                                                                    │    │
│  │  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐             │    │
│  │  │   PTP BMC    │  │   SAP/SDP    │  │   Webserver  │             │    │
│  │  │   Algorithm  │  │  Announce    │  │   Config UI  │             │    │
│  │  └──────────────┘  └──────────────┘  └──────────────┘             │    │
│  │                                                                    │    │
│  │  ┌──────────────┐  ┌──────────────┐                               │    │
│  │  │   Si5351A    │  │   SSD1306    │                               │    │
│  │  │   PLL Ctrl   │  │   Display    │                               │    │
│  │  └──────────────┘  └──────────────┘                               │    │
│  └────────────────────────────────────────────────────────────────────┘    │
│                                                                            │
└────────────────────────────────────────────────────────────────────────────┘
```

The external STM32H753ZI MCU has been replaced by a LiteX-generated RISC-V softcore (VexRiscv) running directly on the FPGA. The SoC boots from external SPI flash via a boot stub that copies the BIOS into HyperRAM, then loads Zephyr firmware. Communication with the FPGA data plane uses LiteX CSR registers over the Wishbone bus instead of the previous FMC memory-mapped interface.

## FPGA Architecture (Data Plane)

The FPGA handles all time-critical audio processing. Key modules in `FPGA/`:

### Ethernet
| Module | File | Description |
|--------|------|-------------|
| Ethernet MAC | `FPGA_Ethernet/` | Fork of YOL MAC with SOF timestamp output |
| Timestamp | `ethernet_timestamp.vhd` | Latches 48b:32b wallclock at SOF delimiter |

### PTP (IEEE 1588)
| Module | File | Description |
|--------|------|-------------|
| Controller | `ptp/ptpv2_controller.vhd` | State machine for Sync, Follow_Up, Announce, Delay_Resp |
| Parser | `ptp/ptpv2_parser.vhd` | Extracts timestamps & computes offset/path delay |
| Servo | `ptp/ptpv2_servo.vhd` | PI controller for clock discipline (PPB correction) |
| Servo (Median) | `ptp/ptpv2_servo_median.vhd` | Variant with median filtering |
| Sender | `ptp/ptpv2_sender.vhd` | Constructs PTP packets |

### Clock & Timing
| Module | File | Description |
|--------|------|-------------|
| Wallclock | `wallclock.vhd` | PTP-disciplined 48b seconds + 32b nanoseconds |
| NCO | `wallclock.vhd` | Direct audio clock synthesis (BCLK, LRCK) |
| Media Clock | `wallclock.vhd` | RTP timestamp counter derived from wallclock |
| PPB Meter | `clock_ppb_meter.vhd` | Measures PPB offset for external PLL correction |

### Audio
| Module | File | Description |
|--------|------|-------------|
| TX Router | `audio_tx/tx_router.vhd` | Multi-stream config RAM, sample aggregation |
| TX Transmitter | `audio_tx/tx_transmitter.vhd` | RTP packet construction with SSRC |
| TX Sample Buffer | `audio_tx/tx_sample_buffer.vhd` | Ring buffer for outgoing samples |
| TDM8 Input | `audio_tx/tdm8_in.vhd` | 8-channel TDM input mux |
| RX Ringbuffer | `audio_rx/rx_ringbuffer.vhd` | Stream demux, playout buffer |
| I2S Input | `I2S_IN.vhd` | 48kHz/24bit I2S deserializer |
| I2S Output | `audio_rx/i2s_out.vhd` | I2S serializer to DAC |
| TDM8 Output | `audio_rx/tdm8_out.vhd` | 8-channel TDM output mux |

### Data Flow
```
           ┌───────────────────────────────────────────────────────────┐
  I2S IN   │    TX Path                                                │
    ──────►│  I2S_IN → tx_sample_buffer → tx_router → tx_transmitter   │───► RTP out
           │                                    ↑                      │
           │                              config from SoC              │
           └───────────────────────────────────────────────────────────┘

           ┌───────────────────────────────────────────────────────────┐
  RTP IN   │    RX Path                                                │
    ──────►│  UDP parser → rx_ringbuffer (stream demux) → i2s_out      │───► I2S OUT
           │                      ↑                                    │
           │                 stream_ram config                         │
           └───────────────────────────────────────────────────────────┘
```

## LiteX SoC

The control plane runs on a LiteX-generated VexRiscv RISC-V softcore, generated by `litex_soc/generate.py`.

### SoC Resources
| Resource | Details |
|----------|---------|
| CPU | VexRiscv RISC-V @ 80 MHz (PLL from 50 MHz input) |
| RAM | IS66WVH16M8ALL HyperRAM, 16 MB @ 0x20000000 |
| Flash | W25Q64 SPI flash @ 0x30000000 (BIOS + firmware) |
| CSR | Peripheral registers @ 0xf0000000 |
| I2C0 | SSD1306 display + Si5351A PLL |
| I2C1 | AD/DA card control |
| SPI0 | SD card |
| UART0 | Console |

### Boot Flow
1. FPGA configures from internal flash, bringing up both data plane and SoC
2. Boot stub at SPI flash reset vector (0x30000000) copies LiteX BIOS to HyperRAM (0x207F0000)
3. Sets HyperRAM controller latency (6 CK for power-on default)
4. BIOS loads Zephyr firmware image (`.fbi` format with CRC-32 header) from flash
5. Zephyr boots, initializes drivers, starts DHCP and application threads

## Firmware Architecture (Control Plane)

Zephyr RTOS runs on the LiteX VexRiscv SoC and handles all non-realtime tasks. Source in `soc_firmware/app/`:

### Core Modules
| Module | File | Description |
|--------|------|-------------|
| Main | `src/main.c` | Init, DHCP, network setup |
| PTP BMC | `src/ptp_bmc.c` | IEEE 1588 Best Leader Clock algorithm on 224.0.1.129:320 |
| SAP/SDP | `src/sap_sdp.c` | Session announcement (239.255.255.255:9875), foreign stream discovery |
| SDP Utils | `src/aes67_sdp_utils.c` | SDP parsing/formatting, PTP clock ID formatting |
| RTSP | `src/rtsp.c` | RAVENNA RTSP server/client (stream subscription, session control) |
| mDNS/DNS-SD | `src/mdns_sd.c` | mDNS responder + DNS-SD service advertisement (RFC 6762/6763) |
| Webserver | `src/webserver.c` | REST API + gzipped static web UI |
| Config | `src/aes67_config.c` | Centralized runtime configuration with defaults |
| Config JSON | `src/config_json.c` | JSON serialization/parsing (shared by SD & flash storage) |
| SD Config | `src/sd_config.c` | SD card config persistence (FAT, crash-safe A/B slots) |
| Flash Config | `src/flash_config.c` | SPI flash config storage (8KB slots, CRC-32 header) |
| FW Update | `src/fw_update.c` | HTTP + shell firmware update, FBI format verification |
| Card Manager | `src/card_manager.c` | I2C board detection & runtime I/O card selection |
| UI Display | `src/ui_display.c` | SSD1306 OLED status rendering |
| FPGA Regs | `src/fpga_regs.c` | High-level register write helpers (via FPGA HAL) |
| FPGA Poll | `src/fpga_poll.c` | Status polling (PTP lock, link state) |
| PLL Control | `src/pll_ctrl.c` | Si5351A PPB correction from FPGA measurements |

### Drivers
| Driver | Path | Description |
|--------|------|-------------|
| FPGA HAL | `drivers/fpga_hal/` | Backend-agnostic hardware abstraction (LiteX CSR or FMC) |
| LiteX Ethernet | `drivers/eth_litex/` | Zephyr network interface via LiteX CSR + Wishbone packet buffers |
| Si5351A | `drivers/si5351a/` | I2C clock generator with PPB correction |
| SPI Flash | `drivers/spi_flash/` | LiteSPI master for firmware updates & config storage |
| Display | `drivers/display_ctrl/` | LED/button/7-segment UART control |
| MI Card | `drivers/mi_card/` | 8-channel ADC preamp control (I2C) |
| LO Card | `drivers/lo_card/` | Line output DA/ADC control (I2C) |
| IO Card | `drivers/io_card/` | Flexible I/O card control (I2C) |

### FPGA HAL

Application code accesses FPGA registers through `fpga_hal.h`, which abstracts the underlying transport:

- **LiteX backend** (`fpga_hal_litex.c`): Uses LiteX CSR registers. FPGA is always ready (integrated SoC).
- **FMC backend** (`fpga_hal_fmc.c`): Legacy STM32H7 FMC memory-mapped access. Retained for backward compatibility.

Backend selection via Kconfig: `CONFIG_FPGA_HAL_LITEX=y` (default) or `CONFIG_FPGA_HAL_FMC=y`.

### CSR Register Map (SoC ↔ FPGA)

| Address | R/W | Description |
|---------|-----|-------------|
| `0x00-0x02` | W | ETH TX length + control |
| `0x10-0x20` | W | ETH TX frame data |
| `0x20-0x22` | R | ETH RX length + status |
| `0x30-0x40` | R | ETH RX frame data |
| `0x40` | W | MAC address (6 bytes) |
| `0x41` | W | IP address (4 bytes) |
| `0x50` | R/W | Flags: PLL, reset, PTP mode |
| `0x51` | R | Ethernet link status |
| `0x52-0x54` | R | PTP metrics (path delay, offset, PPB) |
| `0x55` | W | PTP config (leader identity, intervals) |
| `0x57` | W | Audio destination IP:port |
| `0x58` | W | TX stream config (20 bytes/stream) |
| `0x59` | W | RX stream config (18 bytes/stream) |

Full register map: see [config_ram_address_map.md](config_ram_address_map.md)

## Build Instructions

### LiteX SoC Generation
```bash
cd litex_soc
make    # Generates SoC HDL + device tree + CSR headers
```

### Firmware (Zephyr)
```bash
cd soc_firmware/app
source ../.venv/bin/activate         # Activate Python venv for west
west build -b litex_vexriscv -p      # Clean build
```
The build produces a `.fbi` flash image (Zephyr binary with length + CRC-32 header) for loading via LiteX BIOS.

### FPGA
Open `FPGA/FPGA.qpf` in Intel Quartus Prime 25.1. Target device: 10CL025YU256I7G.

## Current Status

### Working
- Ethernet RX + TX via LiteX CSR interface
- Network config (MAC, DHCP IP)
- PTPv2 Leader and Follower mode with BMC
- Wallclock discipline and media clock derivation
- Si5351A driver with PPB correction
- Audio TX/RX paths (48kHz/24bit I2S, TDM8)
- RTP packet generation and parsing
- SAP/SDP announcements with foreign stream discovery

- Webserver with REST API + gzipped web UI
- Persistent configuration (SD card FAT filesystem with crash-safe A/B slots, SPI flash fallback)
- HTTP + shell-based firmware updates (FBI format with CRC-32 verification)
- Internal audio routing matrix
- LiteX SoC boot from SPI flash via HyperRAM
- Runtime detection of multiple I/O card types (MI, LO, IO)
- SSD1306 OLED status display

### Todo
- Further tune PI controller (currently ±30ns jitter when locked)
- FPGA resource optimization (PTP servo uses ~1600 LUTs)
- Phase jump handling
 - Fix RAVENNA RTSP server/client for stream subscription
 - Fix mDNS responder + DNS-SD service advertisement
## Technical Details

### PTP Clock Discipline
The FPGA implements a PI controller in `ptpv2_servo.vhd`:
- Filters offset measurements
- Outputs frequency correction in PPB
- Lock detection with hysteresis (500ns lock / 5µs unlock threshold)
- Message interval awareness (scales gains for different sync rates)

### Media Clock Generation
`wallclock.vhd` generates reference clocks using an NCO for PLL discipline:
- NCO outputs (BCLK, LRCK) are used to measure phase error against external PLL
- `clock_ppb_meter.vhd` compares NCO edges vs Si5351A edges → PPB correction
- Si5351A (external I2C PLL) provides the actual low-jitter audio clocks
- Media clock counter: (seconds × 48000 + sample_in_second) for RTP timestamps

The NCO has ±8ns jitter (1 sys_clk period), which is fine for measurement but not for direct I2S use.

## License
See [LICENSE.md](LICENSE.md)
