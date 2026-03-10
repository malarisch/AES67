#!/usr/bin/env python3
"""
Generate VexRiscV SoC HDL for manual integration into FPGA top-level.

Outputs Verilog to litex_soc/build/. Does NOT invoke synthesis or P&R.

Usage:
    source soc_firmware/.venv/bin/activate
    python litex_soc/generate.py [--sys-clk-freq 50e6] [--output-dir litex_soc/build]
"""

import argparse
import os
import shutil

from migen import *

from litex.gen import *

from litex.soc.cores.clock import Cyclone10LPPLL
from litex.soc.integration.soc_core import SoCCore
from litex.soc.integration.soc import SoCRegion
from litex.soc.integration.builder import Builder
from litex.soc.cores.hyperbus import HyperRAM

# -- Minimal platform stub (no toolchain, just pin definitions) ---------------

from litex.build.generic_platform import Pins, Subsignal, IOStandard
from litex.build.altera import AlteraPlatform

_io = [
    # Clock (50 MHz input)
    ("clk50", 0, Pins(1)),

    # Serial / UART
    ("serial", 0,
        Subsignal("tx", Pins(1)),
        Subsignal("rx", Pins(1)),
    ),

    # HyperRAM (directly connected to top-level ports)
    ("hyperram", 0,
        Subsignal("clk",   Pins(1)),
        Subsignal("rst_n", Pins(1)),
        Subsignal("dq",    Pins(8)),
        Subsignal("cs_n",  Pins(1)),
        Subsignal("rwds",  Pins(1)),
    ),
]


class StubPlatform(AlteraPlatform):
    """Altera-based platform stub for HDL-only generation (no synthesis)."""
    default_clk_name   = "clk50"
    default_clk_period = 1e9 / 50e6

    def __init__(self):
        # 10CL025YU256I7G = project's Cyclone 10LP device
        AlteraPlatform.__init__(self, "10CL025YU256I7G", _io, toolchain="quartus")

    def create_programmer(self):
        raise NotImplementedError

    def build(self, fragment, build_dir="build", build_name="top", run=True, **kwargs):
        os.makedirs(build_dir, exist_ok=True)
        cwd = os.getcwd()
        os.chdir(build_dir)

        from migen.fhdl.structure import _Fragment
        if not isinstance(fragment, _Fragment):
            fragment = fragment.get_fragment()
        self.finalize(fragment)

        v_output = self.get_verilog(fragment, name=build_name)
        v_output.write(f"{build_name}.v")

        # Copy all registered source files (e.g. VexRiscv CPU) into build dir
        for src_path, language, library, *_ in self.sources:
            dst = os.path.join(build_dir, os.path.basename(src_path))
            if os.path.abspath(src_path) != os.path.abspath(dst):
                shutil.copy2(src_path, dst)

        os.chdir(cwd)
        return v_output.ns


# -- CRG (Clock Reset Generator) ----------------------------------------------

class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq, with_sys2x=False):
        self.rst    = Signal()
        self.cd_sys = ClockDomain()

        # Clk / Rst
        clk50 = platform.request("clk50")

        # PLL
        # Note: 10CL025YU256I7G is Industrial temp with speed grade 7 (-I7)
        self.pll = pll = Cyclone10LPPLL(speedgrade="-I7")
        self.comb += pll.reset.eq(self.rst)
        pll.register_clkin(clk50, 50e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq)

        # HyperRAM 2:1 ratio requires sys2x clock domain
        if with_sys2x:
            self.cd_sys2x = ClockDomain()
            pll.create_clkout(self.cd_sys2x, 2 * sys_clk_freq)


# -- SoC definition ----------------------------------------------------------

class AES67SoC(SoCCore):
    mem_map = {
        "hyperram": 0x20000000,
    }
    mem_map.update(SoCCore.mem_map)

    def __init__(self, platform, sys_clk_freq, with_hyperram=False, hyperram_clk_ratio="4:1", **kwargs):

        # SoCCore - must be initialized before CRG
        SoCCore.__init__(self, platform, sys_clk_freq,
            cpu_type             = "vexriscv",
            cpu_variant          = "minimal",
            integrated_rom_size  = 32 * 1024,  # 32 KB boot ROM
            integrated_sram_size = 8 * 1024,   # 8 KB SRAM
            integrated_main_ram_size = 0,
            ident                = "AES67-LiteX-SoC",
            ident_version        = True,
            with_uart            = True,
            uart_name            = "serial",
            uart_baudrate        = 115200,
            with_timer           = True,
            **kwargs,
        )

        # Determine if sys2x clock is needed (HyperRAM 2:1 mode)
        need_sys2x = with_hyperram and (hyperram_clk_ratio == "2:1")

        # CRG - Clock and Reset Generator (must be after SoCCore.__init__)
        self.crg = _CRG(platform, sys_clk_freq, with_sys2x=need_sys2x)

        # HyperRAM (16 MB IS66WVH16M8ALL on C10LP board)
        if with_hyperram:
            self.hyperram = HyperRAM(
                pads         = platform.request("hyperram"),
                sys_clk_freq = sys_clk_freq,
                clk_ratio    = hyperram_clk_ratio,  # "4:1" = safe, "2:1" = 2x faster
            )
            self.bus.add_slave("hyperram", slave=self.hyperram.bus,
                region=SoCRegion(origin=0x20000000, size=16*1024*1024, mode="rwx"))


# -- Main ---------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Generate AES67 VexRiscV SoC HDL")
    parser.add_argument("--sys-clk-freq",       default=50e6,  type=float, help="System clock frequency (Hz)")
    parser.add_argument("--with-hyperram",      action="store_true",       help="Enable HyperRAM support")
    parser.add_argument("--hyperram-clk-ratio", default="4:1", choices=["4:1", "2:1"], help="HyperRAM clock ratio (2:1 = 2x faster)")
    parser.add_argument("--output-dir",          default=None,              help="Output directory")
    args = parser.parse_args()

    output_dir = args.output_dir or os.path.join(os.path.dirname(__file__), "build")

    platform = StubPlatform()
    soc = AES67SoC(
        platform,
        sys_clk_freq       = int(args.sys_clk_freq),
        with_hyperram      = args.with_hyperram,
        hyperram_clk_ratio = args.hyperram_clk_ratio,
    )

    builder = Builder(soc,
        output_dir       = output_dir,
        compile_software = True,
        compile_gateware = False,
        csr_json         = os.path.join(output_dir, "csr.json"),
        csr_csv          = os.path.join(output_dir, "csr.csv"),
    )
    builder.build(build_name="top", run=False)

    print(f"\nHDL generated in: {output_dir}/gateware/")
    print(f"CSR map:          {output_dir}/csr.json")
    print(f"CSR CSV:          {output_dir}/csr.csv")
    print(f"\nIntegrate the top-level Verilog into your FPGA design.")


if __name__ == "__main__":
    main()
