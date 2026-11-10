# aes67_eth — FPGA Ethernet + PHC kernel driver

Out-of-tree Linux driver that turns the AES67 FPGA (CPU-less `aes67_bridge`
target) into a first-class network device. On gateware built with
`PTP_IN_SOFTWARE = true` it adds **hardware PTP timestamping**, so stock
**`ptp4l`** can discipline the FPGA wallclock. This is Phase 5 ("PTP offload to
the SoC") of `config_tool/docs/control-plane-plan.md`.

The driver reads the gateware's build configuration (`system_cfg_*` CSRs) at
probe and adapts:

| Gateware        | RX timestamp trailer | PHC                         | HW timestamping |
|-----------------|----------------------|-----------------------------|-----------------|
| software PTP    | parsed               | read/write (ptp4l steers it) | yes             |
| hardware PTP    | absent (FPGA strips) | read-only (`phc2sys` can follow FPGA time) | no — the FPGA runs PTP itself |

## What it provides

- **`net_device`** carrying the FPGA `eth_buf` control-plane datapath (the same
  RX-drain / TX-inject protocol the userspace daemon used, now in-kernel).
- **PHC** (`/dev/ptpN`, clock name `aes67_wallclock`) mapping the wallclock CSRs:
  `gettime`/`settime`/`adjtime` (phase jump) / `adjfine` (ppb).
- **Media-clock rate loop** (software PTP): in that gateware the audio NCO
  takes its frequency only from the `nco_ppb_adj` CSRs, not from
  `wallclock_ppb`. `adjfine` therefore also feeds a smoothed copy of the
  applied ppb (Q16 EWMA, snap on steps > 2000 ppb) into `nco_ppb_adj` — the
  same loop the Zephyr firmware runs in `ptp_clock_aes67.c`. Both rate
  registers are reset to 0 at probe.
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
`litex_soc/generate.py` (they shift when CSRs are added/removed, and between
builds with/without `--no-servo-csr` / `--no-metering-csr`). Regenerate it
whenever the gateware CSR map changes. At probe the driver does a scratch
register read-back at the header's address and refuses to bind on a mismatch
(`CSR map mismatch ... make regs`), instead of writing to the wrong registers.

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
- `rx_ts` (default -1 = auto): RX timestamp trailer present (1) / absent (0).
  Auto follows `system_cfg_flags.ptp_in_software`; only override for gateware
  without the `system_cfg` CSRs.
- `map_check` (default 1): scratch read-back check of `aes67_regs.h` at probe.
- `nco_units_per_ppb` (default 55340 = 48 kHz at a 125 MHz wallclock sys_clk):
  conversion for the media-clock rate loop; 0 disables the loop.
- `nco_smooth_shift` (default 7): EWMA time constant of that loop, in powers of
  two of `adjfine` calls.

## RX buffer layout

Software-PTP gateware: `payload | FCS(4) | seconds(1) | nanoseconds_LE(4)`,
with `eth_buf_rx_len` counting all of it. `ethernet_receive` latches the
timestamp at byte 23 and writes the trailer into `eth_ram`; the bridge
(`FPGA/litex_eth_buffer_bridge.vhd`) passes it on in software-PTP mode and
strips it in hardware-PTP mode.
