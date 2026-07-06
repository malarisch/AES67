# aes67_eth — FPGA Ethernet + PHC kernel driver

Out-of-tree Linux driver that turns the AES67 FPGA (CPU-less `aes67_bridge`
target; FPGA top built with `PTP_IN_SOFTWARE = true`) into a first-class network device with
**hardware PTP timestamping**, so stock **`ptp4l`** can discipline the FPGA
wallclock. This is Phase 5 ("PTP offload to the SoC") of
`config_tool/docs/control-plane-plan.md`.

## What it provides

- **`net_device`** carrying the FPGA `eth_buf` control-plane datapath (the same
  RX-drain / TX-inject protocol the userspace daemon used, now in-kernel).
- **PHC** (`/dev/ptpN`, clock name `aes67_wallclock`) mapping the wallclock CSRs:
  `gettime`/`settime`/`adjtime` (phase jump) / `adjfine` (ppb).
- **HW timestamps**: TX from the `tx_timestamp_*` CSRs, RX from the 5-byte
  trailer the FPGA appends after the payload. The captured 4-bit seconds are
  extended to full time by reading the live wallclock seconds.
- **`/dev/aes67ctl`**: a peek/poke char device so the userspace daemon
  (`aes67d`) and `aes67cfg` keep reaching FPGA registers now that the **kernel
  owns the SPI bus** (single Wishbone master).

## Build

```sh
make                      # against the running kernel
make KDIR=<target-headers> ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu-   # cross
make regs CSV=../../litex_soc/build/aes67_bridge/csr.csv   # regenerate aes67_regs.h
```

`aes67_regs.h` is generated from the LiteX `csr.csv` so register addresses track
`litex_soc/generate.py` (they shift when CSRs are added/removed). Regenerate it
whenever the gateware CSR map changes.

## Wiring

Bind via a device-tree overlay (`dts/aes67-overlay.dts`): an SPI child node with
`compatible = "aes67,spibone"`. Wire `eth_buf_irq` to a GPIO and list it under
`interrupts` for interrupt-driven RX; otherwise the driver polls (`poll_ms`).

## Use with ptp4l

```sh
insmod aes67_eth_drv.ko
ethtool -T eth0          # shows HW TX/RX + a PHC index
ptp4l -H -i eth0 -m      # hardware timestamping, disciplines the FPGA wallclock
```

## Module parameters

- `poll_ms` (default 1): RX poll interval when no IRQ is wired.
- `rx_ts` (default 1): parse the FPGA RX hardware-timestamp trailer. Set to 0
  until the FPGA RX FSM appends the trailer (see the FPGA dependency below).

## FPGA dependency

The driver expects the RX buffer layout `payload | FCS(4) | seconds(1) |
nanoseconds_LE(4)`, with `eth_buf_rx_len` counting all of it. The trailer is
produced by the `RX_WRITE_SECONDS`/`RX_WRITE_NANOSECONDS` states in
`FPGA/litex_eth_buffer_bridge.vhd`; confirm those are reachable and that
`buf_rx_len` includes the 5 trailer bytes before relying on RX timestamps
(`rx_ts=1`).
