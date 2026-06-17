#!/usr/bin/env python3
"""Merge CPU-SoC and AES67-bridge LiteX generated headers into one set.

Background
----------
The Wishbone-compliance refactor split the design into two *separate* LiteX
SoCs, each emitting its own ``generated/{csr,mem,soc}.h``:

  * the CPU SoC (``cyclone10`` / ``cyc1000``) — owns CPU-local peripherals
    (SPIFLASH, timer, uart, ...);
  * the standalone AES67 bridge (``aes67_bridge``) — owns the AES67 register
    surface (``aes67_csr``, ``eth_buf``, the stream-config RAMs).

The firmware needs registers from BOTH. The bridge is mapped into the CPU's
address space at the SAME absolute addresses its own headers use (AES67_WB
window @ 0x90000000, CSR sub-bridge @ 0x90010000), so the bridge's addresses
are directly usable by the CPU once their ``CSR_BASE``-relative ``#define``\\s
are resolved to absolute literals.

This script copies the CPU build's ``generated/`` dir and extends ``csr.h`` and
``mem.h`` with the bridge's definitions, producing one include tree the
firmware can consume unchanged (via ``<generated/csr.h>`` etc.).

Run from the firmware CMake; re-runs when either build's csr.json changes.
"""

import argparse
import os
import re
import shutil

CSR_GUARD_END = "#endif /* ! __GENERATED_CSR_H */"


def _read(path):
    with open(path) as f:
        return f.read()


def _merge_csr(cpu, bridge):
    """Append the bridge's unique register #defines to the CPU csr.h.

    The bridge is also a SoCCore, so it carries generic peripherals (``ctrl``,
    ``identifier_mem``) and inline accessor functions that collide with the CPU
    build.  The firmware only uses the AES67-specific register *addresses* via
    raw ``litex_csr_read/write(ADDR)`` (never the generated inline accessors),
    so we keep only the bridge ``#define``\\s whose macro name is NOT already
    defined by the CPU build, dropping every function body and the colliding
    generic peripherals.

    Bridge registers are ``(CSR_BASE + offset)`` with the bridge's own
    ``CSR_BASE``; we substitute that base with its absolute literal so the
    addresses are correct from the CPU's point of view (the bridge CSR window is
    mapped there).  Word-boundary match leaves ``CSR_<periph>_BASE`` untouched.
    """
    m = re.search(r"#define\s+CSR_BASE\s+(\S+)", bridge)
    if not m:
        raise SystemExit("merge_litex_csr: no CSR_BASE in bridge csr.h")
    bridge_base = m.group(1)

    cpu_macros = set(re.findall(r"^#define\s+(\w+)", cpu, re.M))

    kept = []
    for line in bridge.splitlines():
        md = re.match(r"#define\s+(\w+)", line)
        if not md:
            continue
        name = md.group(1)
        if name == "CSR_BASE" or name in cpu_macros:
            continue
        kept.append(re.sub(r"\bCSR_BASE\b", bridge_base, line))

    head = cpu.rsplit(CSR_GUARD_END, 1)[0]
    banner = (f"\n//----------------------------------------------------------\n"
              f"// Merged in: AES67 bridge register addresses (separate SoC,\n"
              f"// base {bridge_base}; generic peripherals + accessors omitted)\n"
              f"//----------------------------------------------------------\n")
    return head + banner + "\n".join(kept) + "\n" + CSR_GUARD_END + "\n"


def _merge_mem(cpu, bridge):
    """Append the bridge's memory regions (ETH_BUF, *_STREAM_CFG) to cpu mem.h.

    Skips regions the CPU build already defines and the bridge's own ``CSR``
    region (the CPU has its own CSR space at 0xf0000000).
    """
    cpu_regions = set(re.findall(r"#define\s+(\w+)_BASE\b", cpu))

    block_re = re.compile(r"#ifndef\s+(\w+)_BASE\n(?:#define[^\n]*\n)+#endif\n")
    extra = []
    for mo in block_re.finditer(bridge):
        name = mo.group(1)
        if name == "CSR" or name in cpu_regions:
            continue
        extra.append(mo.group(0))

    if not extra:
        return cpu

    head, _, _ = cpu.rpartition("#endif")
    banner = "\n/* Merged in: AES67 bridge memory regions */\n"
    return head + banner + "".join(extra) + "#endif\n"


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cpu-inc", required=True,
                    help="CPU build software/include dir (contains generated/)")
    ap.add_argument("--bridge-inc", required=True,
                    help="aes67_bridge build software/include dir")
    ap.add_argument("--out-inc", required=True,
                    help="output software/include dir to (re)create")
    args = ap.parse_args()

    cpu_gen    = os.path.join(args.cpu_inc, "generated")
    bridge_gen = os.path.join(args.bridge_inc, "generated")
    out_gen    = os.path.join(args.out_inc, "generated")

    for d, what in ((cpu_gen, "CPU"), (bridge_gen, "aes67_bridge")):
        if not os.path.isdir(d):
            raise SystemExit(f"merge_litex_csr: {what} generated dir not found: {d}\n"
                             f"  Run: cd litex_soc && python generate.py")

    # Start from a verbatim copy of the CPU generated headers (soc.h, git.h,
    # sdram_phy.h, ... all carried over), then extend csr.h / mem.h.
    shutil.rmtree(out_gen, ignore_errors=True)
    shutil.copytree(cpu_gen, out_gen)

    merged_csr = _merge_csr(_read(os.path.join(cpu_gen, "csr.h")),
                            _read(os.path.join(bridge_gen, "csr.h")))
    merged_mem = _merge_mem(_read(os.path.join(cpu_gen, "mem.h")),
                            _read(os.path.join(bridge_gen, "mem.h")))

    with open(os.path.join(out_gen, "csr.h"), "w") as f:
        f.write(merged_csr)
    with open(os.path.join(out_gen, "mem.h"), "w") as f:
        f.write(merged_mem)

    print(f"merge_litex_csr: wrote {out_gen} (CPU + aes67_bridge)")


if __name__ == "__main__":
    main()
