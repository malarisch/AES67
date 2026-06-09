"""HyperRAM peripheral with warm-reset recovery (Cyclone 10LP)."""

from litex.gen import *

from litex.soc.cores.hyperbus import HyperRAM
from litex.soc.interconnect.csr import CSRStorage, CSRStatus, CSRField


# -- HyperRAM subclass with warm-reset recovery -------------------------------

class AES67HyperRAM(HyperRAM):
    """HyperRAM with automatic startup reset for warm-reset recovery.

    Problem: after an FPGA-only reconfiguration (no power cycle) the HyperRAM
    *chip* retains whatever CR0 the BIOS previously programmed (e.g. 3 CK
    latency), while the LiteX controller CSR resets to its default (7 CK).
    The resulting latency mismatch corrupts every access.

    The upstream CSR "rst" field uses pulse=True which asserts RST# for only
    1 sys_clk cycle (~13.3 ns at 75 MHz).  The IS66WVH16M8ALL/8M8ALL datasheet
    requires tRPH >= 200 ns.  The chip ignores the sub-spec pulse.

    Fix: this subclass overrides add_csr() to drive core.rst with:
      1) An automatic startup counter that holds RST# for 4096 sys_clk cycles
         (~55 µs at 75 MHz) after every FPGA configuration.
      2) A CSR-pulse stretcher that extends any software-triggered RST# to
         32 sys_clk cycles (~427 ns).

    After either reset, the chip returns to power-on defaults (6 CK latency,
    128 B wrapped burst, fixed latency mode).
    """

    def add_csr(self, default_latency=7, latency_mode="fixed"):
        # -- Config / Status CSRs (identical to upstream) ----------------------
        self.config = CSRStorage(fields=[
            CSRField("rst",     offset=0, size=1, pulse=True, description="HyperRAM Rst."),
            CSRField("latency", offset=8, size=8,             description="HyperRAM Latency (X1).", reset=default_latency),
        ])

        # -- Startup reset counter (fires on every FPGA configuration) ---------
        startup_cnt  = Signal(13, reset=0)
        startup_done = Signal(reset=0)
        self.sync += [
            If(~startup_done,
                If(startup_cnt < 4096,
                    startup_cnt.eq(startup_cnt + 1),
                ).Else(
                    startup_done.eq(1),
                ),
            ),
        ]

        # -- CSR-pulse stretcher (extends 1-cycle pulse to 32 cycles) ----------
        csr_rst_cnt = Signal(6, reset=0)
        self.sync += [
            If(self.config.fields.rst,
                csr_rst_cnt.eq(32),
            ).Elif(csr_rst_cnt != 0,
                csr_rst_cnt.eq(csr_rst_cnt - 1),
            ),
        ]

        # -- Combined rst → core.rst ------------------------------------------
        rst_combined = Signal()
        self.comb += rst_combined.eq(
            ~startup_done                  # automatic startup period
            | self.config.fields.rst       # CSR pulse (1 cycle)
            | (csr_rst_cnt != 0)           # CSR pulse stretched
        )
        self.comb += [
            self.core.rst.eq(rst_combined),
            self.core.latency.eq(self.config.fields.latency),
        ]

        # -- Status CSR --------------------------------------------------------
        self.status = CSRStatus(fields=[
            CSRField("latency_mode", offset=0, size=1, values=[
                ("``0b0``", "Fixed Latency."),
                ("``0b1``", "Variable Latency."),
            ], reset={"fixed": 0b0, "variable": 0b1}[latency_mode]),
            CSRField("clk_ratio", offset=1, size=4, values=[
                ("``4``", "HyperRAM Clk = Sys Clk/4."),
                ("``2``", "HyperRAM Clk = Sys Clk/2."),
            ], reset={"4:1": 4, "2:1": 2}[self.clk_ratio]),
        ])

        # -- Register access interface (identical to upstream) -----------------
        self.reg_control = CSRStorage(fields=[
            CSRField("write", offset=0, size=1, pulse=True, description="Issue Register Write."),
            CSRField("read",  offset=1, size=1, pulse=True, description="Issue Register Read."),
            CSRField("addr",  offset=8, size=2, values=[
                ("``0b00``", "Identification Register 0 (Read Only)."),
                ("``0b01``", "Identification Register 1 (Read Only)."),
                ("``0b10``", "Configuration Register 0."),
                ("``0b11``", "Configuration Register 1."),
            ]),
        ])
        self.reg_status = CSRStatus(fields=[
            CSRField("done", offset=0, size=1, description="Register Access Done."),
        ])
        self.reg_wdata = CSRStorage(16, description="Register Write Data.")
        self.reg_rdata = CSRStatus( 16, description="Register Read Data.")

        self.reg_fsm = reg_fsm = FSM(reset_state="IDLE")
        reg_fsm.act("IDLE",
            self.reg_status.fields.done.eq(1),
            If(self.reg_control.fields.write,
                NextState("WRITE"),
            ).Elif(self.reg_control.fields.read,
                NextState("READ"),
            )
        )
        reg_fsm.act("WRITE",
            self.core.reg.stb.eq(1),
            self.core.reg.we.eq(1),
            self.core.reg.adr.eq(self.reg_control.fields.addr),
            self.core.reg.dat_w.eq(self.reg_wdata.storage),
            If(self.core.reg.ack,
                NextState("IDLE"),
            )
        )
        reg_fsm.act("READ",
            self.core.reg.stb.eq(1),
            self.core.reg.we.eq(0),
            self.core.reg.adr.eq(self.reg_control.fields.addr),
            If(self.core.reg.ack,
                NextValue(self.reg_rdata.status, self.core.reg.dat_r),
                NextState("IDLE"),
            )
        )
