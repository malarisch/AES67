"""SPIBone Wishbone master (4-wire SPI -> Wishbone bridge).

In the ``spibone`` target an external host replaces the VexRiscv softcore as the
*sole* Wishbone master, reaching every AES67 CSR + memory-mapped peripheral over
a 4-wire SPI link.  The bridge core runs at 1/4 the sys clock, so the host polls
for completion (see the upstream ``litex.soc.cores.spi.spi_bone`` docs).

We deliberately use the **repo-local** ``litex_soc/spi_bone.py`` (a fork of the
upstream core) instead of ``litex.soc.cores.spi.spi_bone``: it adds the
auto-incrementing *burst* read/write commands that stock spibone lacks, so the
Linux daemon can stream whole Ethernet frames in one SPI transfer instead of one
round-trip per word.  The single-word protocol is unchanged, so the Rust config
tool keeps working.  It is loaded by absolute path so the import works regardless
of the current working directory.

The ``SPIBone`` instance is constructed in ``AES67SoC.__init__`` (so Migen names
its nets after the ``spibone`` SoC attribute, keeping the naming discipline used
by the other peripherals); this helper only registers it as a bus master.
"""

import importlib.util
import os

# Load the burst-capable fork from litex_soc/spi_bone.py (two dirs up from this
# file: aes67_soc/peripherals/ -> aes67_soc/ -> litex_soc/).
_SPI_BONE_PATH = os.path.normpath(
    os.path.join(os.path.dirname(__file__), os.pardir, os.pardir, "spi_bone.py"))
_spec = importlib.util.spec_from_file_location("aes67_spi_bone", _SPI_BONE_PATH)
_spi_bone = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_spi_bone)
SPIBone = _spi_bone.SPIBone


def add_spibone(soc, platform):
    """Register ``soc.spibone``'s Wishbone interface as a bus master.

    ``soc.spibone`` must already be instantiated in the SoC's ``__init__``.
    """
    soc.bus.add_master(name="spibone", master=soc.spibone.bus)
    return soc.spibone
