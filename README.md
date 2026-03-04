# AES67

A full AES67 Audio-over-IP implementation using FPGA and MCU. Currently targeting Cyclone 10LP (FPGA) and STM32H753ZI (MCU with Zephyr RTOS). Some code was LLM generated, but human-checked and debugged. 

For transparency, this is primarily a learning project. I had no FPGA experience before and only basic embedded experience (ESP32 + temperature sensor level).

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              FPGA (Cyclone 10LP)                            │
│  ┌─────────────┐   ┌─────────────┐   ┌─────────────┐   ┌─────────────────┐  │
│  │ Ethernet    │   │   PTPv2     │   │  Wallclock  │   │   Audio TX/RX   │  │
│  │ MAC (YOL)   │◄──┤ Controller  │──►│  48b:32b    │──►│   RTP Packets   │  │
│  │ + Timestamp │   │ + Servo PI  │   │ + Media Clk │   │   + I2S I/O     │  │
│  └──────┬──────┘   └─────────────┘   └──────┬──────┘   └─────────────────┘  │
│         │                                   │                               │
│         │         ┌─────────────────────────┴───────────────┐               │
│         │         │            FMC Bridge                   │               │
│         │         │  (fmc_ethernet_client.vhd)              │               │
│         │         │  - Register-mapped config               │               │
│         │         │  - ETH TX/RX packet buffers             │               │
│         └─────────┴─────────────────────────────────────────┘               │
│                                    ▲                                        │
│                                    │ FMC Bus @ 0x60000000                   │
└────────────────────────────────────┼────────────────────────────────────────┘
                                     │
┌────────────────────────────────────┼────────────────────────────────────────┐
│                                    ▼                                        │
│  ┌─────────────────────────────────────────────────────────────────┐        │
│  │            FMC Ethernet Driver (eth_fmc_basic)                  │        │
│  │  - Zephyr network interface (eth0)                              │        │
│  │  - FPGA state detection & recovery                              │        │
│  └─────────────────────────────────────────────────────────────────┘        │
│                                                                             │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐    │
│  │   PTP BMC    │  │   SAP/SDP    │  │   Webserver  │  │   Si5351A    │    │
│  │   Algorithm  │  │  Announce    │  │   Config UI  │  │   PLL Ctrl   │    │
│  └──────────────┘  └──────────────┘  └──────────────┘  └──────────────┘    │
│                                                                             │
│                            MCU (STM32H753ZI + Zephyr)                       │
└─────────────────────────────────────────────────────────────────────────────┘
```

## FPGA Architecture (Data Plane)

The FPGA handles all time-critical audio processing. Key modules in `FPGA/`:

### Ethernet
| Module | File | Description |
|--------|------|-------------|
| Ethernet MAC | `FPGA_Ethernet/` | Fork of YOL MAC with SOF timestamp output |
| Timestamp | `ethernet_timestamp.vhd` | Latches 48b:32b wallclock at SOF delimiter |
| FMC Bridge | `fmc_ethernet_client.vhd` | Register-mapped interface to MCU |

### PTP (IEEE 1588)
| Module | File | Description |
|--------|------|-------------|
| Controller | `ptp/ptpv2_controller.vhd` | State machine for Sync, Follow_Up, Announce, Delay_Resp |
| Parser | `ptp/ptpv2_parser.vhd` | Extracts timestamps & computes offset/path delay |
| Servo | `ptp/ptpv2_servo.vhd` | PI controller for clock discipline (PPB correction) |
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
| RX Ringbuffer | `audio_rx/rx_ringbuffer.vhd` | Stream demux, playout buffer |
| I2S Input | `I2S_IN.vhd` | 48kHz/24bit I2S deserializer |
| I2S Output | `audio_rx/i2s_out.vhd` | I2S serializer to DAC |

### Data Flow
```
           ┌───────────────────────────────────────────────────────────┐
  I2S IN   │    TX Path                                                │
    ──────►│  I2S_IN → tx_sample_buffer → tx_router → tx_transmitter   │───► RTP out
           │                                    ↑                      │
           │                              config from MCU              │
           └───────────────────────────────────────────────────────────┘
           
           ┌───────────────────────────────────────────────────────────┐
  RTP IN   │    RX Path                                                │
    ──────►│  UDP parser → rx_ringbuffer (stream demux) → i2s_out      │───► I2S OUT
           │                      ↑                                    │
           │                 stream_ram config                         │
           └───────────────────────────────────────────────────────────┘
```

## Firmware Architecture (Control Plane)

The STM32H753ZI runs Zephyr RTOS and handles all non-realtime tasks. Source in `soc_firmware/app/`:

### Core Modules
| Module | File | Description |
|--------|------|-------------|
| Main | `src/main.c` | Init, DHCP, FPGA recovery callback |
| PTP BMC | `src/ptp_bmc.c` | IEEE 1588 Best Master Clock algorithm on 224.0.1.129:320 |
| SAP/SDP | `src/sap_sdp.c` | Session announcement (239.255.255.255:9875), stream config |
| Webserver | `src/webserver.c` | HTTP config UI |
| FPGA Regs | `src/fpga_regs.c` | High-level register write helpers |
| FPGA Poll | `src/fpga_poll.c` | Status polling (PTP lock, link state) |
| PLL Control | `src/pll_ctrl.c` | Si5351A PPB correction from FPGA measurements |

### Drivers
| Driver | Path | Description |
|--------|------|-------------|
| FMC Ethernet | `drivers/eth_fmc_basic/` | Zephyr network interface via FMC bus |
| Si5351A | `drivers/si5351a/` | I2C clock generator with PPB correction |
| Display | `drivers/display_ctrl/` | SSD1306 OLED status display |
| MI Card | `drivers/mi_card/` | 8-channel ADC preamp control |

### FMC Register Map (MCU ↔ FPGA)

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

### Firmware (Zephyr)
```bash
cd soc_firmware/app
source ../.venv/bin/activate  # Activate Python venv for west
west build -b nucleo_h753zi -p  # Clean build
west flash                       # Flash to board
```

### FPGA
Open `FPGA/FPGA.qpf` in Intel Quartus Prime 25.1. Target device: 10CL025YU256I7G.

## Current Status

### Working
- Ethernet RX + TX on FPGA and MCU (via FMC bridge)
- Network config via MCU (MAC, DHCP IP)
- PTPv2 Leader and Follower mode with BMC
- Wallclock discipline and media clock derivation
- Si5351A driver with PPB correction
- Audio TX/RX paths (48kHz/24bit I2S)
- RTP packet generation and parsing
- SAP/SDP announcements
- Webserver configuration UI
- FPGA/MCU reset recovery
- Internal audio routing matrix

### Todo
- Further tune PI controller (currently ±30ns jitter when locked)
- FPGA resource optimization (PTP servo uses ~1600 LUTs)
- Phase jump handling
- RGMII support (currently RMII via LAN8720)
- Replace Altera RMII→MII converter with custom logic
- FPGA bitstream upload from MCU

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

### FMC Bridge Considerations
The FMC bridge requires special handling:
1. **MPU Region**: FMC memory must be marked `ATTR_MPU_IO` to prevent Cortex-M7 speculative reads
2. **Stack Size**: RX thread needs 4096 bytes due to on-stack frame buffer
3. **FPGA Ready**: GPIO PC13 gates FMC access until FPGA is configured

## License
See [LICENSE.md](LICENSE.md)


