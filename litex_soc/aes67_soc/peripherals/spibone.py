"""SPIBone Wishbone master (4-wire SPI -> Wishbone bridge).

In the ``spibone`` target an external host replaces the VexRiscv softcore as the
*sole* Wishbone master, reaching every AES67 CSR + memory-mapped peripheral over
a 4-wire SPI link.  The bridge core runs at 1/4 the sys clock, so the host polls
for completion (see the upstream ``litex.soc.cores.spi.spi_bone`` docs).

The ``SPIBone`` instance is constructed in ``AES67SoC.__init__`` (so Migen names
its nets after the ``spibone`` SoC attribute, keeping the naming discipline used
by the other peripherals); this helper only registers it as a bus master.
"""

from litex.soc.cores.spi.spi_bone import SPIBone


def add_spibone(soc, platform):
    """Register ``soc.spibone``'s Wishbone interface as a bus master.

    ``soc.spibone`` must already be instantiated in the SoC's ``__init__``.
    """
    soc.bus.add_master(name="spibone", master=soc.spibone.bus)
    return soc.spibone
