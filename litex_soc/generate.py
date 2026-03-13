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
from litex.soc.cores.bitbang import I2CMaster
from litex.soc.cores.spi import SPIMaster
from litex.soc.cores.uart import RS232PHY, UART
from litex.soc.interconnect.csr import AutoCSR, CSRStorage, CSRStatus, CSRField
from litex.soc.interconnect import wishbone
from litex.soc.interconnect.csr_eventmanager import EventManager, EventSourceProcess
from migen.genlib.fifo import SyncFIFOBuffered

# -- Minimal platform stub (no toolchain, just pin definitions) ---------------

from litex.build.generic_platform import Pins, Subsignal, IOStandard
from litex.build.altera import AlteraPlatform

_io = [
    # Clock (50 MHz input)
    ("clk50", 0, Pins(1)),

    # MAC clock inputs (directly from FPGA Ethernet MAC)
    ("clk_mac_rx", 0, Pins(1)),
    ("clk_mac_tx", 0, Pins(1)),

    # SoC sys_clk output (80 MHz PLL output, directly active to FPGA for config RAM write clocks)
    ("sys_clk_out", 0, Pins(1)),

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

    # I2C 0: Display + PLL (SSD1306 + Si5351A)
    ("i2c", 0,
        Subsignal("scl", Pins(1)),
        Subsignal("sda", Pins(1)),
    ),

    # I2C 1: AD/DA Card Controller
    ("i2c", 1,
        Subsignal("scl", Pins(1)),
        Subsignal("sda", Pins(1)),
    ),

    # SPI: SD Card
    ("spi", 0,
        Subsignal("clk",  Pins(1)),
        Subsignal("mosi", Pins(1)),
        Subsignal("miso", Pins(1)),
        Subsignal("cs_n", Pins(1)),
    ),

    # UART 1: AD/DA Card LEDs (115200 baud)
    ("serial", 1,
        Subsignal("tx", Pins(1)),
        Subsignal("rx", Pins(1)),
    ),

    # AES67 CSR I/O (directly active to/from external FPGA logic)
    ("aes67_ctrl", 0,
        # Inputs (from external FPGA logic -> SoC)
        Subsignal("pll_ppb_valid",      Pins(1)),
        Subsignal("pll_ppb_wc_count",   Pins(22)),
        Subsignal("pll_ppb_pll_count",  Pins(22)),
        Subsignal("wallclock_locked",   Pins(1)),
        Subsignal("wallclock_phasejump", Pins(1)),
        Subsignal("wallclock_configured", Pins(1)),
        Subsignal("ptp_sync_lost",      Pins(1)),
        Subsignal("eth_link_up",        Pins(1)),
        Subsignal("eth_speed",          Pins(2)),
        Subsignal("ptp_path_delay",     Pins(32)),
        Subsignal("ptp_offset",         Pins(32)),
        Subsignal("eth_tx_done",        Pins(1)),
        Subsignal("eth_rx_overflow",    Pins(1)),
        # Outputs (SoC -> external FPGA logic)
        Subsignal("pll_ppb_start",      Pins(1)),
        Subsignal("ptp_is_leader",      Pins(1)),
        Subsignal("ptp_is_follower",    Pins(1)),
        Subsignal("mac_addr",           Pins(48)),
        Subsignal("ip_addr",            Pins(32)),
        Subsignal("ptp_leader_id",      Pins(64)),
        Subsignal("ptp_time_source",    Pins(8)),
        Subsignal("ptp_log_msg_interval", Pins(8)),
        Subsignal("ptp_announce_msg_interval", Pins(8)),
        Subsignal("eth_tx_request",     Pins(1)),
    ),

    # TX stream config RAM (SoC writes, tx_router reads via FPGA-side port)
    ("tx_stream_cfg", 0,
        Subsignal("wr_en",   Pins(1)),   # output: write enable
        Subsignal("wr_addr", Pins(8)),   # output: write address (0..255)
        Subsignal("wr_data", Pins(8)),   # output: write data
    ),

    # RX stream config RAM (SoC writes, rx_ringbuffer reads via FPGA-side port)
    ("rx_stream_cfg", 0,
        Subsignal("wr_en",   Pins(1)),   # output: write enable
        Subsignal("wr_addr", Pins(8)),   # output: write address (0..255)
        Subsignal("wr_data", Pins(8)),   # output: write data
    ),

    # Ethernet packet buffer I/O (directly active to/from external FPGA logic)
    ("eth_buf", 0,
        # RX buffer: FPGA writes, SoC reads
        Subsignal("rx_data",    Pins(8)),   # input: data byte from FPGA
        Subsignal("rx_addr",    Pins(11)),  # input: write address from FPGA (0..1499)
        Subsignal("rx_we",      Pins(1)),   # input: write enable from FPGA
        Subsignal("rx_len",     Pins(11)),  # input: received packet length
        Subsignal("rx_valid",   Pins(1)),   # input: packet ready (level)
        Subsignal("rx_ack",     Pins(1)),   # output: SoC done reading
        # TX buffer: SoC writes, FPGA reads
        Subsignal("tx_data",    Pins(8)),   # output: data byte to FPGA
        Subsignal("tx_addr",    Pins(11)),  # input: read address from FPGA (0..1499)
        Subsignal("tx_len",     Pins(11)),  # output: packet length
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

    def build(self, fragment, build_dir="build", build_name="litex_soc", run=True, **kwargs):
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

        # MAC clock domains (directly fed from external FPGA Ethernet MAC).
        # Used for the FPGA-side ports of dual-port RAMs in eth_buf.
        self.cd_mac_rx = ClockDomain(reset_less=True)
        self.cd_mac_tx = ClockDomain(reset_less=True)
        self.comb += [
            self.cd_mac_rx.clk.eq(platform.request("clk_mac_rx")),
            self.cd_mac_tx.clk.eq(platform.request("clk_mac_tx")),
        ]


# -- Custom CSRs -------------------------------------------------------------

class AES67CSRs(LiteXModule, AutoCSR):
    """Application-specific control/status registers for AES67 FPGA logic."""
    def __init__(self):
        # =====================================================================
        # Input signals (directly active from external FPGA logic)
        # =====================================================================
        self.i_pll_ppb_valid       = Signal()
        self.i_pll_ppb_wc_count    = Signal(22)
        self.i_pll_ppb_pll_count   = Signal(22)
        self.i_wallclock_locked    = Signal()
        self.i_wallclock_phasejump = Signal()
        self.i_wallclock_configured = Signal()
        self.i_ptp_sync_lost       = Signal()
        self.i_eth_link_up         = Signal()
        self.i_eth_speed           = Signal(2)
        self.i_ptp_path_delay      = Signal(32)
        self.i_ptp_offset          = Signal(32)
        self.i_eth_tx_done         = Signal()
        self.i_eth_rx_overflow     = Signal()

        # =====================================================================
        # Output signals (directly active to external FPGA logic)
        # =====================================================================
        self.o_pll_ppb_start       = Signal()
        self.o_ptp_is_leader       = Signal()
        self.o_ptp_is_follower     = Signal()
        self.o_mac_addr            = Signal(48)
        self.o_ip_addr             = Signal(32)
        self.o_ptp_leader_id       = Signal(64)
        self.o_ptp_time_source     = Signal(8)
        self.o_ptp_log_msg_interval = Signal(8)
        self.o_ptp_announce_msg_interval = Signal(8)
        self.o_eth_tx_request      = Signal()

        # =====================================================================
        # CSR: Status registers (RO) — directly sampled from input signals
        # =====================================================================
        self.pll_ppb_status = CSRStatus(32, fields=[
            CSRField("valid",     size=1,  offset=0,  description="PPB measurement valid"),
        ])
        self.pll_ppb_wc_count = CSRStatus(32, description="PPB measurement wallclock count [21:0]")
        self.pll_ppb_pll_count = CSRStatus(32, description="PPB measurement PLL count [21:0]")

        self.status = CSRStatus(32, fields=[
            CSRField("wallclock_locked",    size=1, offset=0, description="Wallclock locked"),
            CSRField("wallclock_phasejump", size=1, offset=1, description="Wallclock phase jump"),
            CSRField("wallclock_configured", size=1, offset=2, description="Wallclock configured"),
            CSRField("ptp_sync_lost",       size=1, offset=3, description="PTP sync lost"),
            CSRField("eth_link_up",         size=1, offset=4, description="Ethernet link up"),
            CSRField("eth_speed",           size=2, offset=5, description="Ethernet speed (00=10M,01=100M,10=1G)"),
            CSRField("eth_tx_done",         size=1, offset=7, description="Ethernet TX done"),
            CSRField("eth_rx_overflow",     size=1, offset=8, description="Ethernet RX overflow"),
        ])

        self.ptp_path_delay = CSRStatus(32, description="PTP path delay (ns)")
        self.ptp_offset     = CSRStatus(32, description="PTP offset from master (ns)")

        # =====================================================================
        # CSR: Control registers (RW) — directly drive output signals
        # =====================================================================
        self.ctrl = CSRStorage(32, fields=[
            CSRField("pll_ppb_start",   size=1, offset=0, description="Start PPB measurement"),
            CSRField("ptp_is_leader",   size=1, offset=1, description="PTP leader mode"),
            CSRField("ptp_is_follower", size=1, offset=2, description="PTP follower mode"),
            CSRField("eth_tx_request",  size=1, offset=3, description="Request Ethernet TX"),
        ])

        self.mac_addr_lo = CSRStorage(32, description="MAC address [31:0]")
        self.mac_addr_hi = CSRStorage(32, description="MAC address [47:32] (bits [15:0] used)")
        self.ip_addr     = CSRStorage(32, description="IP address [31:0]")

        self.ptp_leader_id_lo = CSRStorage(32, description="PTP leader identity [31:0]")
        self.ptp_leader_id_hi = CSRStorage(32, description="PTP leader identity [63:32]")

        self.ptp_time_source = CSRStorage(8, description="PTP time source")
        self.ptp_log_msg_interval = CSRStorage(8, description="PTP logMessageInterval")
        self.ptp_announce_msg_interval = CSRStorage(8, description="PTP announce logMessageInterval")

        # -- Scratch register (RW, no HW connection) --
        self.scratch = CSRStorage(32, description="Scratch register (read/write, no HW effect)")

        # =====================================================================
        # Wiring: input signals -> CSR status fields
        # =====================================================================
        self.comb += [
            self.pll_ppb_status.fields.valid.eq(self.i_pll_ppb_valid),
            self.pll_ppb_wc_count.status.eq(self.i_pll_ppb_wc_count),
            self.pll_ppb_pll_count.status.eq(self.i_pll_ppb_pll_count),

            self.status.fields.wallclock_locked.eq(self.i_wallclock_locked),
            self.status.fields.wallclock_phasejump.eq(self.i_wallclock_phasejump),
            self.status.fields.wallclock_configured.eq(self.i_wallclock_configured),
            self.status.fields.ptp_sync_lost.eq(self.i_ptp_sync_lost),
            self.status.fields.eth_link_up.eq(self.i_eth_link_up),
            self.status.fields.eth_speed.eq(self.i_eth_speed),
            self.status.fields.eth_tx_done.eq(self.i_eth_tx_done),
            self.status.fields.eth_rx_overflow.eq(self.i_eth_rx_overflow),

            self.ptp_path_delay.status.eq(self.i_ptp_path_delay),
            self.ptp_offset.status.eq(self.i_ptp_offset),
        ]

        # =====================================================================
        # Wiring: CSR storage fields -> output signals
        # =====================================================================
        self.comb += [
            self.o_pll_ppb_start.eq(self.ctrl.fields.pll_ppb_start),
            self.o_ptp_is_leader.eq(self.ctrl.fields.ptp_is_leader),
            self.o_ptp_is_follower.eq(self.ctrl.fields.ptp_is_follower),
            self.o_eth_tx_request.eq(self.ctrl.fields.eth_tx_request),

            self.o_mac_addr.eq(Cat(self.mac_addr_lo.storage, self.mac_addr_hi.storage[:16])),
            self.o_ip_addr.eq(self.ip_addr.storage),

            self.o_ptp_leader_id.eq(Cat(self.ptp_leader_id_lo.storage, self.ptp_leader_id_hi.storage)),
            self.o_ptp_time_source.eq(self.ptp_time_source.storage),
            self.o_ptp_log_msg_interval.eq(self.ptp_log_msg_interval.storage),
            self.o_ptp_announce_msg_interval.eq(self.ptp_announce_msg_interval.storage),
        ]


# -- Ethernet Packet Buffers (memory-mapped, 1500 bytes each) -----------------

class EthPacketBuffer(LiteXModule, AutoCSR):
    """
    RX and TX packet buffers (1500 bytes each) with CSR-accessible length/control
    and an IRQ for RX packet received.

    The FPGA side writes the RX buffer and reads the TX buffer via dedicated signals.
    The CPU side reads the RX buffer and writes the TX buffer via a Wishbone slave.

    Wishbone address map (byte-addressed, 1 byte per 32-bit word):
      - 0x000..0x5DB: RX buffer (1500 bytes, read-only from CPU)
      - 0x800..0xDDB: TX buffer (1500 bytes, read/write from CPU)
    """
    def __init__(self):
        # -- External FPGA-side signals --
        # RX: FPGA writes into buffer, asserts rx_valid when packet is complete
        self.i_rx_data   = Signal(8)    # data byte from FPGA
        self.i_rx_addr   = Signal(11)   # write address from FPGA (0..1499)
        self.i_rx_we     = Signal()     # write enable from FPGA
        self.i_rx_len    = Signal(11)   # received packet length
        self.i_rx_valid  = Signal()     # packet ready to read (level)
        self.o_rx_ack    = Signal()     # SoC done reading (active high)
        # TX: SoC writes buffer, FPGA reads via signals
        self.o_tx_data   = Signal(8)    # data byte to FPGA
        self.i_tx_addr   = Signal(11)   # read address from FPGA (0..1499)
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
        #   - "sys"    port: CPU side (Wishbone, 80 MHz SoC clock)
        #   - "mac_rx" port: RX buffer FPGA write port (MAC RX clock)
        #   - "mac_tx" port: TX buffer FPGA read port (MAC TX clock)
        #
        # Cyclone 10LP block RAMs natively support dual-clock operation.

        # RX buffer: MAC writes (port A, mac_rx domain), CPU reads (port B, sys domain)
        rx_mem = Memory(8, 1500, name="eth_rx_buf")
        self.specials += rx_mem
        rx_wr_port = rx_mem.get_port(write_capable=True, clock_domain="mac_rx")
        self.specials += rx_wr_port
        rx_rd_port = rx_mem.get_port(has_re=True, clock_domain="sys")
        self.specials += rx_rd_port

        # TX buffer: CPU writes (port A, sys domain), MAC reads (port B, mac_tx domain)
        tx_mem = Memory(8, 1500, name="eth_tx_buf")
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
        self.bus = wishbone.Interface(data_width=32, adr_width=12)
        wb = self.bus

        # bit 9 of word address selects TX (0x800+) vs RX (0x000+)
        is_tx = wb.adr[9]
        cpu_addr = Signal(11)
        self.comb += cpu_addr.eq(wb.adr[:11])

        # CPU reads RX buffer (sys domain port)
        self.comb += [
            rx_rd_port.adr.eq(cpu_addr),
            rx_rd_port.re.eq(wb.cyc & wb.stb & ~is_tx & ~wb.we),
        ]

        # CPU writes TX buffer (sys domain port)
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

        # -- RX status, ACK, TX length --
        rx_valid_r = Signal()
        self.sync += rx_valid_r.eq(self.i_rx_valid)

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


# -- SoC definition ----------------------------------------------------------

class AES67SoC(SoCCore):
    mem_map = dict(SoCCore.mem_map)
    mem_map.update({
        "main_ram":       0x20000000,  # HyperRAM serves as main RAM
        "eth_buf":        0x30000000,
        "tx_stream_cfg":  0x30001000,
        "rx_stream_cfg":  0x30002000,
    })

    def __init__(self, platform, sys_clk_freq, with_hyperram=False, hyperram_clk_ratio="4:1", integrated_rom_size=24*1024, integrated_sram_size=4*1024, **kwargs):

        # SoCCore - must be initialized before CRG
        SoCCore.__init__(self, platform, sys_clk_freq,
            cpu_type             = "vexriscv",
            cpu_variant          = "standard",
            integrated_rom_size  = integrated_rom_size,
            integrated_sram_size = integrated_sram_size,
            integrated_main_ram_size = 0,
            ident                = "AES67-LiteX-SoC",
            ident_version        = True,
            with_uart            = True,
            uart_name            = "serial",
            uart_baudrate        = 115200,
            with_timer           = True,
            timer_uptime         = True,
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
            self.bus.add_slave("main_ram", slave=self.hyperram.bus,
                region=SoCRegion(origin=0x20000000, size=16*1024*1024, mode="rwx"))

        # -- I2C 0: Display (SSD1306) + PLL (Si5351A) -------------------------
        self.i2c0 = I2CMaster(platform.request("i2c", 0))

        # -- I2C 1: AD/DA Card Controller --------------------------------------
        self.i2c1 = I2CMaster(platform.request("i2c", 1))

        # -- SPI: SD Card ------------------------------------------------------
        self.spi0 = SPIMaster(
            pads         = platform.request("spi"),
            data_width   = 8,
            sys_clk_freq = sys_clk_freq,
            spi_clk_freq = 400e3,  # SD card init requires ≤400 kHz
        )

        # -- UART 1: AD/DA Card LEDs (115200) ----------------------------------
        self.uart1_phy = RS232PHY(platform.request("serial", 1), clk_freq=sys_clk_freq, baudrate=1e6)
        self.uart1 = UART(self.uart1_phy, tx_fifo_depth=16, rx_fifo_depth=16)
        self.irq.add("uart1")

        # -- sys_clk output (for FPGA-side config RAM write clocks) -------------
        self.comb += platform.request("sys_clk_out").eq(ClockSignal("sys"))

        # -- Application CSRs --------------------------------------------------
        self.aes67_csr = AES67CSRs()

        # Wire CSR signals to top-level pads
        aes67_pads = platform.request("aes67_ctrl")
        self.comb += [
            # Inputs: pads -> CSR
            self.aes67_csr.i_pll_ppb_valid.eq(aes67_pads.pll_ppb_valid),
            self.aes67_csr.i_pll_ppb_wc_count.eq(aes67_pads.pll_ppb_wc_count),
            self.aes67_csr.i_pll_ppb_pll_count.eq(aes67_pads.pll_ppb_pll_count),
            self.aes67_csr.i_wallclock_locked.eq(aes67_pads.wallclock_locked),
            self.aes67_csr.i_wallclock_phasejump.eq(aes67_pads.wallclock_phasejump),
            self.aes67_csr.i_wallclock_configured.eq(aes67_pads.wallclock_configured),
            self.aes67_csr.i_ptp_sync_lost.eq(aes67_pads.ptp_sync_lost),
            self.aes67_csr.i_eth_link_up.eq(aes67_pads.eth_link_up),
            self.aes67_csr.i_eth_speed.eq(aes67_pads.eth_speed),
            self.aes67_csr.i_ptp_path_delay.eq(aes67_pads.ptp_path_delay),
            self.aes67_csr.i_ptp_offset.eq(aes67_pads.ptp_offset),
            self.aes67_csr.i_eth_tx_done.eq(aes67_pads.eth_tx_done),
            self.aes67_csr.i_eth_rx_overflow.eq(aes67_pads.eth_rx_overflow),
            # Outputs: CSR -> pads
            aes67_pads.pll_ppb_start.eq(self.aes67_csr.o_pll_ppb_start),
            aes67_pads.ptp_is_leader.eq(self.aes67_csr.o_ptp_is_leader),
            aes67_pads.ptp_is_follower.eq(self.aes67_csr.o_ptp_is_follower),
            aes67_pads.mac_addr.eq(self.aes67_csr.o_mac_addr),
            aes67_pads.ip_addr.eq(self.aes67_csr.o_ip_addr),
            aes67_pads.ptp_leader_id.eq(self.aes67_csr.o_ptp_leader_id),
            aes67_pads.ptp_time_source.eq(self.aes67_csr.o_ptp_time_source),
            aes67_pads.ptp_log_msg_interval.eq(self.aes67_csr.o_ptp_log_msg_interval),
            aes67_pads.ptp_announce_msg_interval.eq(self.aes67_csr.o_ptp_announce_msg_interval),
            aes67_pads.eth_tx_request.eq(self.aes67_csr.o_eth_tx_request),
        ]

        # -- Ethernet Packet Buffers -------------------------------------------
        self.eth_buf = EthPacketBuffer()

        # Register packet buffer memory on the Wishbone bus
        self.bus.add_slave("eth_buf", slave=self.eth_buf.bus,
            region=SoCRegion(origin=0x30000000, size=4096))

        # Wire buffer signals to top-level pads
        eth_buf_pads = platform.request("eth_buf")
        self.comb += [
            # RX: FPGA writes -> SoC reads
            self.eth_buf.i_rx_data.eq(eth_buf_pads.rx_data),
            self.eth_buf.i_rx_addr.eq(eth_buf_pads.rx_addr),
            self.eth_buf.i_rx_we.eq(eth_buf_pads.rx_we),
            self.eth_buf.i_rx_len.eq(eth_buf_pads.rx_len),
            self.eth_buf.i_rx_valid.eq(eth_buf_pads.rx_valid),
            eth_buf_pads.rx_ack.eq(self.eth_buf.o_rx_ack),
            # TX: SoC writes -> FPGA reads
            eth_buf_pads.tx_data.eq(self.eth_buf.o_tx_data),
            self.eth_buf.i_tx_addr.eq(eth_buf_pads.tx_addr),
            eth_buf_pads.tx_len.eq(self.eth_buf.o_tx_len),
        ]

        # Add IRQ for RX packet received
        self.irq.add("eth_buf")

        # -- TX Stream Config RAM (256 bytes, write-only from SoC) -------------
        self.tx_stream_cfg = StreamConfigRAM()
        self.bus.add_slave("tx_stream_cfg", slave=self.tx_stream_cfg.bus,
            region=SoCRegion(origin=0x30001000, size=1024))

        tx_cfg_pads = platform.request("tx_stream_cfg")
        self.comb += [
            tx_cfg_pads.wr_en.eq(self.tx_stream_cfg.o_wr_en),
            tx_cfg_pads.wr_addr.eq(self.tx_stream_cfg.o_wr_addr),
            tx_cfg_pads.wr_data.eq(self.tx_stream_cfg.o_wr_data),
        ]

        # -- RX Stream Config RAM (256 bytes, write-only from SoC) -------------
        self.rx_stream_cfg = StreamConfigRAM()
        self.bus.add_slave("rx_stream_cfg", slave=self.rx_stream_cfg.bus,
            region=SoCRegion(origin=0x30002000, size=1024))

        rx_cfg_pads = platform.request("rx_stream_cfg")
        self.comb += [
            rx_cfg_pads.wr_en.eq(self.rx_stream_cfg.o_wr_en),
            rx_cfg_pads.wr_addr.eq(self.rx_stream_cfg.o_wr_addr),
            rx_cfg_pads.wr_data.eq(self.rx_stream_cfg.o_wr_data),
        ]


# -- Main ---------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Generate AES67 VexRiscV SoC HDL")
    parser.add_argument("--sys-clk-freq",       default=80e6,  type=float, help="System clock frequency (Hz)")
    parser.add_argument("--with-hyperram",      action="store_true", default=True,       help="Enable HyperRAM support")
    parser.add_argument("--hyperram-clk-ratio", default="4:1", choices=["4:1", "2:1"], help="HyperRAM clock ratio (2:1 = 2x faster)")
    parser.add_argument("--rom-size",           default=24,    type=int,   help="ROM size in KB (default: 24)")
    parser.add_argument("--sram-size",          default=4,     type=int,   help="SRAM size in KB (default: 8)")
    parser.add_argument("--bios-console",       default="disable", choices=["full", "lite", "disable"], help="BIOS console mode (disable saves most ROM)")
    parser.add_argument("--output-dir",         default=None,              help="Output directory")
    args = parser.parse_args()

    output_dir = args.output_dir or os.path.join(os.path.dirname(__file__), "build")

    platform = StubPlatform()
    soc = AES67SoC(
        platform,
        sys_clk_freq         = int(args.sys_clk_freq),
        with_hyperram        = args.with_hyperram,
        hyperram_clk_ratio   = args.hyperram_clk_ratio,
        integrated_rom_size  = args.rom_size * 1024,
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
    builder.build(build_name="litex_soc", run=False)

    print(f"\nHDL generated in: {output_dir}/gateware/")
    print(f"CSR map:          {output_dir}/csr.json")
    print(f"CSR CSV:          {output_dir}/csr.csv")
    print(f"\nIntegrate the top-level Verilog into your FPGA design.")


if __name__ == "__main__":
    main()
