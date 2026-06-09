"""Stream configuration RAM (Wishbone slave, CPU writes, FPGA reads)."""

from litex.gen import *

from litex.soc.integration.soc import SoCRegion
from litex.soc.interconnect.csr import AutoCSR
from litex.soc.interconnect import wishbone


# -- Stream Configuration RAM (256 bytes, CPU writes, FPGA reads) ------------

class StreamConfigRAM(LiteXModule, AutoCSR):
    """
    256-byte configuration RAM for audio stream parameters.

    CPU writes via Wishbone (sys clock domain).  The FPGA side gets a
    registered copy of every write as wr_en/wr_addr/wr_data output signals,
    which feed directly into the tx_router or rx_ringbuffer config RAM.

    The FPGA-side config RAMs (tx_router, rx_ringbuffer) are the actual
    storage — this module just forwards CPU writes to them.  No read-back
    from the FPGA side is needed (write-only from SoC perspective).

    Wishbone address map: 256 byte addresses (1 byte per 32-bit word).
    """
    def __init__(self):
        # -- FPGA-side output signals (directly connected to config RAM write port) --
        self.o_wr_en   = Signal()
        self.o_wr_addr = Signal(8)
        self.o_wr_data = Signal(8)

        # -- Wishbone slave (write-only, 256 addresses) --
        self.bus = wishbone.Interface(data_width=32, adr_width=10)
        wb = self.bus

        # Register the write and generate FPGA-side signals
        self.sync += [
            self.o_wr_en.eq(0),
            If(wb.cyc & wb.stb & wb.we,
                self.o_wr_en.eq(1),
                self.o_wr_addr.eq(wb.adr[:8]),
                self.o_wr_data.eq(wb.dat_w[:8]),
            ),
        ]

        # Always ack in 1 cycle
        wb_ack = Signal()
        self.sync += [
            wb_ack.eq(0),
            If(wb.cyc & wb.stb & ~wb_ack,
                wb_ack.eq(1),
            ),
        ]
        self.comb += [
            wb.ack.eq(wb_ack),
            wb.dat_r.eq(0),  # write-only, reads return 0
        ]


def add_stream_cfg(soc, platform, name, pads_name, origin):
    """Register ``soc.<name>``'s Wishbone slave and wire its pads.

    The StreamConfigRAM must already be attached as ``soc.<name>`` in the SoC's
    ``__init__`` so Migen names its nets ``<name>_*`` (e.g. ``tx_stream_cfg_*``),
    matched by FPGA/sdc/litex_csr.sdc.  ``pads_name`` is the platform resource
    name (e.g. "tx_stream_cfg").
    """
    cfg = getattr(soc, name)
    soc.bus.add_slave(name, slave=cfg.bus,
        region=SoCRegion(origin=origin, size=1024, cached=False))

    cfg_pads = platform.request(pads_name)
    soc.comb += [
        cfg_pads.wr_en.eq(cfg.o_wr_en),
        cfg_pads.wr_addr.eq(cfg.o_wr_addr),
        cfg_pads.wr_data.eq(cfg.o_wr_data),
    ]
    return cfg
