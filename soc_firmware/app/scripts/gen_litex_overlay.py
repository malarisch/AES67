#!/usr/bin/env python3
"""
Generate a Zephyr device-tree include (.dtsi) from a LiteX csr.json.

Each AES67 LiteX board variant (cyclone10, cyc1000, …) has a checked-in
`litex_vexriscv_<target>.dts` file that `#include`s the generated
`<target>_csr.dtsi`. The dtsi contains all target-specific base addresses
(peripherals, RAM, CPU clock) so that adding or removing a CSR in the SoC
simply regenerates the file without touching the hand-written DTS.

Outputs:
  <out-dir>/<target>_csr.dtsi

Usage:
  gen_litex_overlay.py --csr-json <path> --target <name> --out-dir <path>
"""

import argparse
import json
import os
import sys


PERIPHS = {
    # csr name -> (DT label used in *.dts, reg-names list)
    "ctrl":   ("ctrl0", ["reset", "scratch", "bus_errors"]),
    "timer0": ("timer0", ["load", "reload", "en", "update_value", "value",
                          "ev_status", "ev_pending", "ev_enable",
                          "uptime_latch", "uptime_cycles"]),
    "uart":   ("uart0", ["rxtx", "txfull", "rxempty", "ev_status", "ev_pending",
                         "ev_enable", "txempty", "rxfull"]),
    "uart1":  ("uart1", ["rxtx", "txfull", "rxempty", "ev_status", "ev_pending",
                         "ev_enable", "txempty", "rxfull"]),
    # Single shared I2C bus (LiteI2C): display + PLL + AD/DA card controller.
    "i2c0":   ("i2c_card", ["phy_speed_mode", "master_active", "master_settings",
                            "master_addr", "master_rxtx", "master_status"]),
    "spi0":   ("spi0", ["control", "status", "mosi", "miso", "cs", "loopback"]),
}


def emit_periph(lines, csr_name, base, reg_names):
    label = PERIPHS[csr_name][0]
    off = 0
    regs = []
    for n in reg_names:
        size = 8 if n == "uptime_cycles" else 4
        regs.append((base + off, size))
        off += size

    lines.append(f"&{label} {{")
    reg_entries = [f"<0x{a:08x} 0x{s:x}>" for a, s in regs]
    lines.append("\treg = " + ",\n\t      ".join(reg_entries) + ";")
    lines.append("};")
    lines.append("")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csr-json", required=True)
    ap.add_argument("--target",   required=True)
    ap.add_argument("--out-dir",  required=True)
    args = ap.parse_args()

    with open(args.csr_json) as f:
        csr = json.load(f)

    bases     = csr["csr_bases"]
    memories  = csr["memories"]
    constants = csr.get("constants", {})

    sys_clk_freq = int(constants.get("config_clock_frequency", 0))
    if sys_clk_freq == 0:
        print(f"warning: config_clock_frequency missing from {args.csr_json}",
              file=sys.stderr)

    main_ram = memories["main_ram"]
    ram_base = main_ram["base"]
    ram_size = main_ram["size"]

    lines = [
        "/*",
        " * AUTO-GENERATED — DO NOT EDIT.",
        f" * Target:  {args.target}",
        f" * Source:  {os.path.abspath(args.csr_json)}",
        " * Regenerate by re-running `make` in litex_soc/ and rebuilding the app.",
        " */",
        "",
        "&cpu0 {",
        f"\tclock-frequency = <{sys_clk_freq}>;",
        "};",
        "",
        "&ram0 {",
        f"\treg = <0x{ram_base:08x} 0x{ram_size:08x}>;",
        "};",
        "",
    ]

    for csr_name, (_label, reg_names) in PERIPHS.items():
        if csr_name not in bases:
            # uart1 is optional on some targets — skip silently.
            if csr_name == "uart1":
                continue
            print(f"warning: {csr_name} missing from {args.csr_json}",
                  file=sys.stderr)
            continue
        emit_periph(lines, csr_name, bases[csr_name], reg_names)

    os.makedirs(args.out_dir, exist_ok=True)
    out_path = os.path.join(args.out_dir, f"{args.target}_csr.dtsi")
    new = "\n".join(lines) + "\n"
    if os.path.exists(out_path):
        with open(out_path) as f:
            if f.read() == new:
                return
    with open(out_path, "w") as f:
        f.write(new)
    print(f"[gen_litex_overlay] wrote {out_path}")


if __name__ == "__main__":
    main()
