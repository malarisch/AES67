"""Ethernet packet buffers (memory-mapped Wishbone slave).

RX is a window onto the RX ring buffer that lives in the FPGA-side
litex_eth_buffer_bridge (VHDL); TX is a 1518-byte buffer in here."""

from litex.gen import *

from migen import Memory
from litex.soc.integration.soc import SoCRegion
from litex.soc.interconnect.csr import AutoCSR, CSRStorage, CSRStatus
from litex.soc.interconnect import wishbone
from litex.soc.interconnect.csr_eventmanager import EventManager, EventSourceProcess


# -- Ethernet Packet Buffers --------------------------------------------------

class EthPacketBuffer(LiteXModule, AutoCSR):
    """
    RX/TX packet buffers with CSR-accessible length/control and an IRQ for
    RX packet received.

    RX has no memory in here: the FPGA-side bridge keeps the received frames
    in its dual-clock ring buffer (8-bit MAC write port, 32-bit read port on
    this SoC's sys clock) and publishes the head frame. CPU word reads of the
    RX window go out on ``o_rx_rd_addr`` (word offset into the frame) and the
    bridge answers on ``i_rx_rd_data`` one sys cycle later; ``i_rx_len`` /
    ``i_rx_valid`` / ``o_rx_ack`` are sys-domain signals (no CDC here).

    TX: the CPU writes the buffer via Wishbone, the FPGA reads it byte-wide on
    the MAC TX clock.

    Wishbone address map (word-addressed, 4 payload bytes per 32-bit word,
    little-endian lanes: frame byte 4i+k sits in word i bits [8k+7:8k]):
      - word 0x000..0x7FF: RX frame (read-only from CPU, frame at word 0)
      - word 0x800..0x97B: TX buffer (1518 bytes, write-only from CPU)
    Byte addresses (CPU view): RX at +0x0000, TX at +0x2000.

    The read-only ``bytes_per_word`` CSR reports the packing (4). Gateware
    generations before the packed layout carried one payload byte per word
    and did not have this CSR — hosts resolve it by name from csr.csv and
    fall back to byte packing when it is absent.
    """
    def __init__(self):
        # -- External FPGA-side signals --
        # RX (all sys domain): the bridge's ring buffer read port
        self.o_rx_rd_addr = Signal(11)  # word offset into the published frame
        self.i_rx_rd_data = Signal(32)  # frame bytes 4i..4i+3, 1 cycle later
        self.i_rx_len    = Signal(11)   # published frame length
        self.i_rx_valid  = Signal()     # frame published (level)
        self.o_rx_ack    = Signal()     # SoC done reading (rising edge pops)
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
        self.bytes_per_word = CSRStatus(8, reset=4,
            description="Payload bytes per 32-bit buffer word (absent on "
                        "legacy 1-byte-per-word gateware)")

        # -- TX memory (simple dual port, independent clocks) --
        #   - "sys"    port: CPU writes (Wishbone, SoC clock)
        #   - "mac_tx" port: FPGA reads (MAC TX clock)
        # 4 payload bytes per 32-bit word, little-endian lanes, implemented as
        # four independent byte-wide lane memories instead of one 32-bit RAM
        # with byte enables: plain simple-dual-port, dual-clock templates keep
        # BRAM inference trivially safe. 1518 bytes -> 380 words per lane.
        buf_words = (1518 + 3) // 4

        # TX buffer: CPU writes (sys domain), MAC reads (mac_tx domain)
        tx_wr_ports = []
        tx_rd_ports = []
        for k in range(4):
            mem = Memory(8, buf_words, name=f"eth_tx_buf{k}")
            self.specials += mem
            wr = mem.get_port(write_capable=True, clock_domain="sys")
            rd = mem.get_port(has_re=True, clock_domain="mac_tx")
            self.specials += wr, rd
            tx_wr_ports.append(wr)
            tx_rd_ports.append(rd)

        # -- FPGA-side wiring --
        # TX: MAC reads one byte per cycle. Lane reads keep the old port's
        # 1-cycle latency, so the lane select is the byte address delayed by
        # one mac_tx cycle.
        tx_lane = Signal(2)
        self.sync.mac_tx += tx_lane.eq(self.i_tx_addr[:2])
        for k in range(4):
            self.comb += [
                tx_rd_ports[k].adr.eq(self.i_tx_addr[2:]),
                tx_rd_ports[k].re.eq(1),
            ]
        self.comb += self.o_tx_data.eq(
            Array([p.dat_r for p in tx_rd_ports])[tx_lane])

        # -- Wishbone slave interface (CPU side, sys clock domain) --
        # Need 12-bit address to cover 0x000..0x5ED (RX) and 0x800..0xDED (TX)
        self.bus = wishbone.Interface(data_width=32, adr_width=12)
        wb = self.bus

        # bit 11 of word address selects TX (0x800+) vs RX (0x000+)
        is_tx = wb.adr[11]
        cpu_addr = Signal(11)
        self.comb += cpu_addr.eq(wb.adr[:11])

        # CPU reads the RX frame straight from the bridge's ring buffer: the
        # word offset goes out, the bridge's registered read port answers one
        # cycle later (same latency as an internal block RAM).
        self.comb += self.o_rx_rd_addr.eq(cpu_addr)

        # CPU writes TX buffer (sys domain ports, wb.sel gates the lanes so
        # sub-word stores from the softcore stay correct)
        # cpu_addr is bits [10:0] of wb.adr — already the offset within the TX
        # region (bit 11 selects TX vs RX), so no subtraction needed.
        for k in range(4):
            self.comb += [
                tx_wr_ports[k].adr.eq(cpu_addr),
                tx_wr_ports[k].dat_w.eq(wb.dat_w[8 * k:8 * k + 8]),
                tx_wr_ports[k].we.eq(wb.cyc & wb.stb & is_tx & wb.we &
                                     wb.sel[k]),
            ]

        # Wishbone read data: only RX is readable, TX is write-only
        self.comb += wb.dat_r.eq(self.i_rx_rd_data)

        # Ack: 1-cycle latency for memory read
        wb_ack = Signal()
        self.sync += [
            wb_ack.eq(0),
            If(wb.cyc & wb.stb & ~wb_ack,
                wb_ack.eq(1),
            ),
        ]
        self.comb += wb.ack.eq(wb_ack)

        # -- RX status, ACK, TX length --
        # i_rx_valid / i_rx_len come from the bridge's sys-domain reader (no
        # CDC); i_rx_len is stable while i_rx_valid is high.
        self.comb += [
            self.rx_len.status.eq(self.i_rx_len),
            self.rx_ready.status.eq(self.i_rx_valid),
            self.o_rx_ack.eq(self.rx_ack.storage),
            self.o_tx_len.eq(self.tx_len.storage[:11]),
        ]

        # IRQ: EventSourceProcess triggers on low->high of its trigger input
        # (it is active-low internally, so we invert: trigger=~rx_valid means
        #  IRQ fires when rx_valid goes high)
        self.comb += self.ev.rx_ready.trigger.eq(~self.i_rx_valid)


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
        # RX: SoC reads the bridge's ring buffer
        eth_buf_pads.rx_rd_addr.eq(soc.eth_buf.o_rx_rd_addr),
        soc.eth_buf.i_rx_rd_data.eq(eth_buf_pads.rx_rd_data),
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
