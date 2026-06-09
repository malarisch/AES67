#!/usr/bin/env python3
"""
Generate VexRiscV SoC HDL for manual integration into FPGA top-level.

Outputs Verilog to litex_soc/build/. Does NOT invoke synthesis or P&R.

Usage:
    source soc_firmware/.venv/bin/activate
    python litex_soc/generate.py [--sys-clk-freq 75e6] [--output-dir litex_soc/build]

The generated SoC has no internal PLL.  The top-level FPGA design must feed
the SoC its clocks via these ports:
    cyclone10:  clk_sys                (sys_clk_freq, typ. 75 MHz)
    cyc1000:    clk_sys, clk_sys_ps    (90deg phase-shifted, for SDRAM)
    gowin:      clk_sys, clk_sys2x     (2x, for DDR3)

The SoC itself is defined in the :mod:`aes67_soc` package (one module per
concern: platform IO, CRG, peripherals, SoC assembly).  This file is just the
command-line front-end that selects a target and drives the LiteX Builder.
"""

import argparse
import os
import shutil
import subprocess
import sys

# Make the aes67_soc package importable regardless of the current working
# directory (this script is run as `python litex_soc/generate.py`).
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from litex.soc.integration.builder import Builder
from litex.soc.doc import generate_docs, generate_svd

from aes67_soc import (
    AES67SoC,
    Cyclone10StubPlatform,
    Cyc1000StubPlatform,
    GowinStubPlatform,
)


# -- Main ---------------------------------------------------------------------

ALL_TARGETS = ["cyclone10", "cyc1000", "gowin"]
# Default sys clock frequency — must match what the top-level PLL feeds into clk_sys.
# 75 MHz chosen because the Cyclone 10LP ALTPLL cannot cleanly divide common
# crystal inputs to 80 MHz; 75 MHz gives clean ratios for all targets.
DEFAULT_SYS_CLK_FREQ = {"cyclone10": 75e6, "cyc1000": 75e6, "gowin": 75e6}


def _generate_docs_svd(soc, target, output_dir):
    """Emit register-level Sphinx docs + an SVD file for a finalized SoC.

    The SoC is already finalized by ``builder.build()``, so the LiteX doc/SVD
    generators can read its CSR/memory map directly.  Docs land in
    ``<output_dir>/documentation`` (and are compiled to HTML if sphinx-build is
    available); the SVD goes next to the generated software headers.
    """
    doc_dir = os.path.join(output_dir, "documentation")
    sw_dir  = os.path.join(output_dir, "software")

    generate_docs(
        soc, doc_dir,
        project_name = f"AES67 LiteX SoC ({target})",
        author       = "AES67 project",
        from_scratch = True,
    )
    generate_svd(soc, sw_dir, name=f"aes67_{target}")

    svd_path = os.path.join(sw_dir, f"aes67_{target}.svd")

    # Compile the Sphinx sources to HTML when the toolchain is present.  This is
    # best-effort: a missing sphinx-build must not fail the HDL/SVD generation.
    html_dir = os.path.join(doc_dir, "_build", "html")
    sphinx = shutil.which("sphinx-build")
    if sphinx is not None:
        result = subprocess.run(
            [sphinx, "-M", "html", doc_dir, os.path.join(doc_dir, "_build"), "-q"],
            capture_output=True, text=True,
        )
        if result.returncode != 0:
            print(f"WARNING: sphinx-build failed for {target}; "
                  f"RST sources are still in {doc_dir}.\n{result.stderr.strip()}")
            html_dir = None
    else:
        print(f"NOTE: sphinx-build not found; skipping HTML for {target}. "
              f"RST sources are in {doc_dir} (pip install sphinx sphinxcontrib-wavedrom).")
        html_dir = None

    return svd_path, doc_dir, html_dir


def _build_target(target, args):
    sys_clk_freq = args.sys_clk_freq if args.sys_clk_freq is not None else DEFAULT_SYS_CLK_FREQ[target]
    output_dir   = args.output_dir or os.path.join(os.path.dirname(__file__), "build", target)

    if target == "gowin":
        platform = GowinStubPlatform()
        soc = AES67SoC(
            platform,
            sys_clk_freq         = int(sys_clk_freq),
            target               = "gowin",
            integrated_sram_size = args.sram_size * 1024,
        )
    elif target == "cyc1000":
        platform = Cyc1000StubPlatform()
        soc = AES67SoC(
            platform,
            sys_clk_freq         = int(sys_clk_freq),
            target               = "cyc1000",
            integrated_sram_size = args.sram_size * 1024,
        )
    else:
        platform = Cyclone10StubPlatform()
        soc = AES67SoC(
            platform,
            sys_clk_freq         = int(sys_clk_freq),
            target               = "cyclone10",
            with_hyperram        = args.with_hyperram,
            integrated_sram_size = args.sram_size * 1024,
        )

    builder = Builder(soc,
        output_dir       = output_dir,
        compile_software = True,
        compile_gateware = False,
        csr_json         = os.path.join(output_dir, "csr.json"),
        csr_csv          = os.path.join(output_dir, "csr.csv"),
        bios_console     = args.bios_console,
    )
    builder.build(build_name="litex_soc_" + target, run=False)

    # Register-level documentation + SVD (SoC is finalized after build()).
    svd_path, doc_dir, html_dir = _generate_docs_svd(soc, target, output_dir)

    print(f"\nTarget:           {target}")
    print(f"HDL generated in: {output_dir}/gateware/")
    print(f"CSR map:          {output_dir}/csr.json")
    print(f"CSR CSV:          {output_dir}/csr.csv")
    print(f"SVD:              {svd_path}")
    print(f"Docs (RST):       {doc_dir}/")
    if html_dir is not None:
        print(f"Docs (HTML):      {html_dir}/index.html")


def main():
    parser = argparse.ArgumentParser(description="Generate AES67 VexRiscV SoC HDL")
    parser.add_argument("--target",             default=None, choices=ALL_TARGETS, help="Target FPGA platform (default: build all)")
    parser.add_argument("--sys-clk-freq",       default=None,  type=float, help="System clock frequency (Hz). Default: 75 MHz for all targets (must match top-level PLL output).")
    parser.add_argument("--with-hyperram",      action="store_true", default=True,       help="Enable HyperRAM support (cyclone10 only)")
    parser.add_argument("--sram-size",          default=4,     type=int,   help="SRAM size in KB (default: 4)")
    parser.add_argument("--bios-console",       default="full", choices=["full", "lite", "disable"], help="BIOS console mode (disable saves most ROM)")
    parser.add_argument("--output-dir",         default=None,              help="Output directory")
    args = parser.parse_args()

    targets = [args.target] if args.target is not None else ALL_TARGETS
    if len(targets) > 1 and args.output_dir is not None:
        parser.error("--output-dir cannot be combined with multi-target build; specify --target.")

    for target in targets:
        _build_target(target, args)

    print(f"\nIntegrate the top-level Verilog into your FPGA design.")


if __name__ == "__main__":
    main()
