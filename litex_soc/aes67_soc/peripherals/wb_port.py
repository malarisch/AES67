"""External Wishbone port helpers — split the SoC across a module boundary.

These expose a Wishbone bus at the top-level pads so the AES67 register surface
(``aes67_csr`` + ``eth_buf`` + ``tx/rx_stream_cfg``) can live in a **separate**
generated HDL module from the VexRiscv SoC, wired together in ``top.vhd`` by a
single bus bundle instead of dozens of CSR signals.

Two symmetric helpers:

- :func:`add_external_wb_slave` — used on the **SoC (master) side**.  Adds a bare
  ``wishbone.Interface`` as a *slave region* on the SoC bus and routes it to pads.
  The SoC crossbar drives it as a master toward that address window; the signals
  leave the chip and reach the bridge.

- :func:`add_external_wb_master` — used on the **bridge (slave) side**.  Adds a bare
  ``wishbone.Interface`` as an *incoming master* on the bridge's SoCMini bus and
  routes it to pads.  From the bridge's view this master drives its CSR/peripheral
  slaves.

Wishbone signal directions (from the Interface layout):
  M->S: adr, dat_w, sel, cyc, stb, we, cti, bte
  S->M: dat_r, ack, err
"""

from migen import *

from litex.soc.interconnect import wishbone


# Signals grouped by direction relative to the *master*.
_WB_M_TO_S = ["adr", "dat_w", "sel", "cyc", "stb", "we", "cti", "bte"]
_WB_S_TO_M = ["dat_r", "ack", "err"]


def _wire_master_to_pads(soc, iface, pads):
    """Route a WB interface whose master is *inside* this module to pads.

    Master drives M->S signals out; reads S->M signals in.
    """
    soc.comb += [getattr(pads, n).eq(getattr(iface, n)) for n in _WB_M_TO_S]
    soc.comb += [getattr(iface, n).eq(getattr(pads, n)) for n in _WB_S_TO_M]


def _wire_slave_to_pads(soc, iface, pads):
    """Route a WB interface whose master is *outside* this module to pads.

    The external master drives M->S signals in; this module's slaves drive S->M out.
    """
    soc.comb += [getattr(iface, n).eq(getattr(pads, n)) for n in _WB_M_TO_S]
    soc.comb += [getattr(pads, n).eq(getattr(iface, n)) for n in _WB_S_TO_M]


def add_external_wb_slave(soc, platform, pads_name, region):
    """SoC (master) side: expose an external WB slave region at ``pads_name``.

    ``region`` is the SoCRegion the external bridge answers for.  The SoC bus
    treats this as a slave; its crossbar masters drive the interface out to pads.
    """
    iface = wishbone.Interface(data_width=32, adr_width=30)
    soc.bus.add_slave(name=pads_name, slave=iface, region=region)
    pads = platform.request(pads_name)
    _wire_master_to_pads(soc, iface, pads)
    return iface


def add_external_wb_master(soc, platform, pads_name):
    """Bridge (slave) side: expose an incoming external WB master at ``pads_name``.

    The external SoC drives this master in; the bridge's slaves answer.
    """
    iface = wishbone.Interface(data_width=32, adr_width=30)
    pads = platform.request(pads_name)
    _wire_slave_to_pads(soc, iface, pads)
    soc.bus.add_master(name=pads_name, master=iface)
    return iface


# -- eth_buf RX-ready interrupt across the module boundary --------------------

class _ExtIRQ(Module):
    """Trivial holder so the SoC's IRQ finalize finds ``self.<name>.irq``.

    The SoC IRQ machinery wires ``self.<name>.irq`` (or ``.ev.irq``) to the CPU
    interrupt array.  This module's ``irq`` is driven from an input pad — the
    bridge's eth_buf interrupt, brought in across top.vhd.
    """
    def __init__(self, irq_in):
        self.irq = Signal()
        self.comb += self.irq.eq(irq_in)


def add_eth_irq_output(soc, platform, pads_name="eth_buf_irq"):
    """Bridge side: route ``soc.eth_buf``'s RX-ready IRQ out to a pad."""
    pad = platform.request(pads_name)
    soc.comb += pad.eq(soc.eth_buf.ev.irq)


def add_eth_irq_input(soc, platform, pads_name="eth_buf_irq", name="eth_buf"):
    """SoC side: take the bridge's eth_buf IRQ in and map it to a CPU interrupt.

    Registers an interrupt named ``name`` (default "eth_buf") so the generated
    ``ETH_BUF_INTERRUPT`` constant matches what the firmware expects.
    """
    pad = platform.request(pads_name)
    setattr(soc, name, _ExtIRQ(pad))
    soc.irq.add(name)
