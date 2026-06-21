"""UARTBone Wishbone master (serial UART -> Wishbone bridge).

In the ``uartbone`` target an external host replaces the VexRiscv softcore as the
*sole* Wishbone master, reaching every AES67 CSR + memory-mapped peripheral over
a 2-wire async serial link.  It is the serial counterpart of the ``spibone``
bridge (see :mod:`aes67_soc.peripherals.spibone`); the host talks the
``litex_server``/``litex.tools.litex_term`` wire protocol over the UART.

The ``UARTBone`` instance is constructed in ``AES67SoC.__init__`` (so Migen names
its nets after the ``uartbone`` SoC attribute, keeping the naming discipline used
by the other peripherals); this helper only registers it as a bus master.
"""

from litex.soc.cores.uart import UARTBone, UARTPHY


def add_uartbone(soc, platform):
    """Register ``soc.uartbone``'s Wishbone interface as a bus master.

    ``soc.uartbone`` must already be instantiated in the SoC's ``__init__``.
    """
    soc.bus.add_master(name="uartbone", master=soc.uartbone.wishbone)
    return soc.uartbone
