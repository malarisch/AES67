#!/usr/bin/env python3
"""
Generate a Zephyr device-tree overlay fragment from a LiteX csr.json file.

The static overlay `litex_vexriscv.overlay` hardcodes base addresses that
change between SoC targets (cyclone10 / cyc1000 / gowin).  Rather than
duplicating the overlay per target, this script derives all target-specific
addresses from the LiteX-generated `csr.json` and emits a small overlay
fragment that is included before the static overlay.

Outputs:
  - <out>/litex_soc_generated.overlay  (DT fragment with all base addresses)

Usage:
  gen_litex_overlay.py --csr-json <path> --out-dir <path>
"""

import argparse
import json
import os
import sys


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csr-json", required=True)
    ap.add_argument("--out-dir",  required=True)
    args = ap.parse_args()

    with open(args.csr_json) as f:
        csr = json.load(f)

    bases     = csr["csr_bases"]
    memories  = csr["memories"]
    constants = csr.get("constants", {})

    sys_clk_freq = int(constants.get("config_clock_frequency", 0))
    if sys_clk_freq == 0:
        print("warning: config_clock_frequency missing from csr.json", file=sys.stderr)

    main_ram = memories["main_ram"]
    ram_base = main_ram["base"]
    ram_size = main_ram["size"]

    # -- Build overlay text ---------------------------------------------------
    lines = [
        "/*",
        " * AUTO-GENERATED — DO NOT EDIT.",
        f" * Generated from {os.path.abspath(args.csr_json)}",
        " * by soc_firmware/app/scripts/gen_litex_overlay.py",
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

    # Peripheral base address overrides.  Every peripheral the static overlay
    # touches gets its `reg` list regenerated here with the correct base.
    reg_names = {
        "ctrl":   ["reset", "scratch", "bus_errors"],
        "timer0": ["load", "reload", "en", "update_value", "value",
                   "ev_status", "ev_pending", "ev_enable",
                   "uptime_latch", "uptime_cycles"],
        "uart":   ["rxtx", "txfull", "rxempty", "ev_status", "ev_pending",
                   "ev_enable", "txempty", "rxfull"],
        "uart1":  ["rxtx", "txfull", "rxempty", "ev_status", "ev_pending",
                   "ev_enable", "txempty", "rxfull"],
        "i2c0":   ["write", "read"],
        "i2c1":   ["phy_speed_mode", "master_active", "master_settings",
                   "master_addr", "master_rxtx", "master_status"],
        "spi0":   ["control", "status", "mosi", "miso", "cs", "loopback"],
    }

    # Mapping csr name -> DT node label used in overlay
    node_label = {
        "ctrl":   "ctrl0",
        "timer0": "timer0",
        "uart":   "uart0",     # LiteX calls console uart "uart"; DT label is uart0
        "uart1":  "uart1",
        "i2c0":   "i2c0",
        "i2c1":   "i2c_card",  # static overlay defines this as i2c@… node
        "spi0":   "spi0",
    }

    for csr_name, label in node_label.items():
        if csr_name not in bases:
            print(f"warning: {csr_name} missing from csr.json", file=sys.stderr)
            continue
        base = bases[csr_name]
        names = reg_names[csr_name]

        regs = []
        off = 0
        for n in names:
            size = 8 if n == "uptime_cycles" else 4
            regs.append(f"\t      <0x{base + off:08x} 0x{size:x}>")
            off += size

        lines.append(f"&{label} {{")
        lines.append("\treg = " + regs[0].lstrip() + ("," if len(regs) > 1 else ";"))
        for i, r in enumerate(regs[1:]):
            sep = "," if i < len(regs) - 2 else ";"
            lines.append(r + sep)
        lines.append("};")
        lines.append("")

    # Note: eth_buf is a dummy DT node used only for IRQ dispatch; its `reg`
    # is arbitrary (the real buffer is accessed via the memories.eth_buf
    # region directly in C).  No override needed here.

    # -- Write overlay --------------------------------------------------------
    os.makedirs(args.out_dir, exist_ok=True)
    out_path = os.path.join(args.out_dir, "litex_soc_generated.overlay")
    new = "\n".join(lines) + "\n"
    if os.path.exists(out_path):
        with open(out_path) as f:
            if f.read() == new:
                return  # unchanged → don't retrigger cmake
    with open(out_path, "w") as f:
        f.write(new)
    print(f"[gen_litex_overlay] wrote {out_path}")


if __name__ == "__main__":
    main()
