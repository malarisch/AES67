"""Ethernet packet buffers (memory-mapped Wishbone slave, 1518 bytes each)."""

from litex.gen import *

from migen import Memory
from litex.soc.integration.soc import SoCRegion
from litex.soc.interconnect.csr import AutoCSR, CSRStorage, CSRStatus
from litex.soc.interconnect import wishbone
from litex.soc.interconnect.csr_eventmanager import EventManager, EventSourceProcess


# -- Ethernet Packet Buffers (memory-mapped, 1518 bytes each) -----------------

class EthPacketBuffer(LiteXModule, AutoCSR):
    """
    RX and TX packet buffers (1518 bytes each) with CSR-accessible length/control
    and an IRQ for RX packet received.

    The FPGA side writes the RX buffer and reads the TX buffer via dedicated signals.
    The CPU side reads the RX buffer and writes the TX buffer via a Wishbone slave.

    Wishbone address map (word-addressed, 1 byte per 32-bit word):
      - 0x000..0x5ED: RX buffer (1518 bytes, read-only from CPU)
      - 0x800..0xDED: TX buffer (1518 bytes, write-only from CPU)
    Byte addresses (CPU view): RX at +0x0000, TX at +0x2000.
    """
    def __init__(self):
        # -- External FPGA-side signals --
        # RX: FPGA writes into buffer, asserts rx_valid when packet is complete
        self.i_rx_data   = Signal(8)    # data byte from FPGA
        self.i_rx_addr   = Signal(11)   # write address from FPGA (0..1517)
        self.i_rx_we     = Signal()     # write enable from FPGA
        self.i_rx_len    = Signal(11)   # received packet length
        self.i_rx_valid  = Signal()     # packet ready to read (level)
        self.o_rx_ack    = Signal()     # SoC done reading (active high)
        # TX: SoC writes buffer, FPGA reads via signals
        self.o_tx_data   = Signal(8)    # data byte to FPGA
        self.i_tx_addr   = Signal(11)   # read address from FPGA (0..1517)
        self.o_tx_len    = Signal(11)   # packet length for FPGA

        # -- IRQ (EventManager required by LiteX irq.add()) --
        self.submodules.ev = EventManager()
        self.ev.rx_ready = EventSourceProcess(description="RX packet received")
        self.ev.finalize()

        # -- CSRs --
        self.rx_len    = CSRStatus(16, description="RX packet length (bytes)")
        self.rx_ready  = CSRStatus(1,  description="RX packet ready to read")
        self.rx_ack    = CSRStorage(1,  description="Write 1 to acknowledge RX packet")
        self.tx_len    = CSRStorage(16, description="TX packet length (bytes)")

        # -- Internal memories (true dual-port, independent clocks) --
        #
        # Each memory has two ports on different clock domains:
        #   - "sys"    port: CPU side (Wishbone, sys_clk_freq SoC clock)
        #   - "mac_rx" port: RX buffer FPGA write port (MAC RX clock)
        #   - "mac_tx" port: TX buffer FPGA read port (MAC TX clock)
        #
        # Cyclone 10LP block RAMs natively support dual-clock operation.

        # RX buffer: MAC writes (port A, mac_rx domain), CPU reads (port B, sys domain)
        rx_mem = Memory(8, 1518, name="eth_rx_buf")
        self.specials += rx_mem
        rx_wr_port = rx_mem.get_port(write_capable=True, clock_domain="mac_rx")
        self.specials += rx_wr_port
        rx_rd_port = rx_mem.get_port(has_re=True, clock_domain="sys")
        self.specials += rx_rd_port

        # TX buffer: CPU writes (port A, sys domain), MAC reads (port B, mac_tx domain)
        tx_mem = Memory(8, 1518, name="eth_tx_buf")
        self.specials += tx_mem
        tx_wr_port = tx_mem.get_port(write_capable=True, clock_domain="sys")
        self.specials += tx_wr_port
        tx_rd_port = tx_mem.get_port(has_re=True, clock_domain="mac_tx")
        self.specials += tx_rd_port

        # -- FPGA-side wiring (directly on MAC clock domains) --
        # RX: MAC writes (mac_rx clock)
        self.comb += [
            rx_wr_port.adr.eq(self.i_rx_addr),
            rx_wr_port.dat_w.eq(self.i_rx_data),
            rx_wr_port.we.eq(self.i_rx_we),
        ]
        # TX: MAC reads (mac_tx clock)
        self.comb += [
            tx_rd_port.adr.eq(self.i_tx_addr),
            tx_rd_port.re.eq(1),
            self.o_tx_data.eq(tx_rd_port.dat_r),
        ]

        # -- Wishbone slave interface (CPU side, sys clock domain) --
        # Need 12-bit address to cover 0x000..0x5ED (RX) and 0x800..0xDED (TX)
        self.bus = wishbone.Interface(data_width=32, adr_width=12)
        wb = self.bus

        # bit 11 of word address selects TX (0x800+) vs RX (0x000+)
        is_tx = wb.adr[11]
        cpu_addr = Signal(11)
        self.comb += cpu_addr.eq(wb.adr[:11])

        # CPU reads RX buffer (sys domain port)
        self.comb += [
            rx_rd_port.adr.eq(cpu_addr),
            rx_rd_port.re.eq(wb.cyc & wb.stb & ~is_tx & ~wb.we),
        ]

        # CPU writes TX buffer (sys domain port)
        # cpu_addr is bits [10:0] of wb.adr — already the offset within the TX
        # region (bit 11 selects TX vs RX), so no subtraction needed.
        self.comb += [
            tx_wr_port.adr.eq(cpu_addr),
            tx_wr_port.dat_w.eq(wb.dat_w[:8]),
            tx_wr_port.we.eq(wb.cyc & wb.stb & is_tx & wb.we),
        ]

        # Wishbone read data: only RX buffer is readable, TX is write-only
        self.comb += wb.dat_r.eq(rx_rd_port.dat_r)

        # Ack: 1-cycle latency for memory read
        wb_ack = Signal()
        self.sync += [
            wb_ack.eq(0),
            If(wb.cyc & wb.stb & ~wb_ack,
                wb_ack.eq(1),
            ),
        ]
        self.comb += wb.ack.eq(wb_ack)

        # -- CDC: synchronise mac_rx domain signals into sys domain --
        # i_rx_valid is a level signal from mac_rx_clock; needs 2-FF sync
        # to avoid metastability on the IRQ trigger and status reads.
        rx_valid_meta = Signal(reset=0)
        rx_valid_sync = Signal(reset=0)
        self.sync += [
            rx_valid_meta.eq(self.i_rx_valid),
            rx_valid_sync.eq(rx_valid_meta),
        ]

        # i_rx_len is multi-bit but only valid while rx_valid is stable-high,
        # so we latch it on the rising edge of the synchronised rx_valid.
        rx_valid_sync_d = Signal()
        rx_len_latched  = Signal(11)
        self.sync += [
            rx_valid_sync_d.eq(rx_valid_sync),
            If(rx_valid_sync & ~rx_valid_sync_d,
                rx_len_latched.eq(self.i_rx_len),
            ),
        ]

        # -- RX status, ACK, TX length --
        self.comb += [
            self.rx_len.status.eq(rx_len_latched),
            self.rx_ready.status.eq(rx_valid_sync),
            self.o_rx_ack.eq(self.rx_ack.storage),
            self.o_tx_len.eq(self.tx_len.storage[:11]),
        ]

        # IRQ: EventSourceProcess triggers on low->high of its trigger input
        # (it is active-low internally, so we invert: trigger=~rx_valid means
        #  IRQ fires when rx_valid goes high)
        self.comb += self.ev.rx_ready.trigger.eq(~rx_valid_sync)


def add_eth_buffer(soc, platform, origin):
    """Register ``soc.eth_buf``'s Wishbone slave + IRQ and wire its pads.

    ``soc.eth_buf`` must already be instantiated in the SoC's ``__init__`` so
    Migen names its nets ``eth_buf_*`` (matched by FPGA/sdc/litex_csr.sdc), not
    after this helper.  See the note in soc.py.
    """
    # Register packet buffer memory on the Wishbone bus
    soc.bus.add_slave("eth_buf", slave=soc.eth_buf.bus,
        region=SoCRegion(origin=origin, size=0x4000, cached=False))

    # Wire buffer signals to top-level pads
    eth_buf_pads = platform.request("eth_buf")
    soc.comb += [
        # RX: FPGA writes -> SoC reads
        soc.eth_buf.i_rx_data.eq(eth_buf_pads.rx_data),
        soc.eth_buf.i_rx_addr.eq(eth_buf_pads.rx_addr),
        soc.eth_buf.i_rx_we.eq(eth_buf_pads.rx_we),
        soc.eth_buf.i_rx_len.eq(eth_buf_pads.rx_len),
        soc.eth_buf.i_rx_valid.eq(eth_buf_pads.rx_valid),
        eth_buf_pads.rx_ack.eq(soc.eth_buf.o_rx_ack),
        # TX: SoC writes -> FPGA reads
        eth_buf_pads.tx_data.eq(soc.eth_buf.o_tx_data),
        soc.eth_buf.i_tx_addr.eq(eth_buf_pads.tx_addr),
        eth_buf_pads.tx_len.eq(soc.eth_buf.o_tx_len),
    ]

    # Add IRQ for RX packet received — only on SoCs that have an IRQ controller.
    # The CPU-less spibone bridge has no IRQs; the external host polls the
    # rx_ready CSR instead.
    if soc.irq.enabled:
        soc.irq.add("eth_buf")
    return soc.eth_buf
