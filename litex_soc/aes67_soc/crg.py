"""Clock-Reset Generators (CRG) for each target.

All three CRGs consume their clocks directly from external pads.  The top-level
FPGA design instantiates one PLL and feeds the SoC the required clock(s):
  - cyclone10: clk_sys (sys_clk_freq, typ. 80 MHz)
  - cyc1000:   clk_sys + clk_sys_ps (sys_clk_freq, 90deg shifted, for SDRAM)
  - gowin:     clk_sys + clk_sys2x (sys_clk_freq and 2x, for DDR3)

This keeps the SoC portable: no target-specific PLL primitives are baked into
the generated SoC HDL, and a single top-level PLL can drive everything
(including non-SoC FPGA logic on the same clock).
"""

from migen import *

from litex.gen import *

from migen.genlib.resetsync import AsyncResetSynchronizer


def _bind_mac_clocks(crg, platform):
    """Wire mac_rx/mac_tx clock domains directly from the Ethernet MAC pads."""
    crg.cd_mac_rx = ClockDomain(reset_less=True)
    crg.cd_mac_tx = ClockDomain(reset_less=True)
    crg.comb += [
        crg.cd_mac_rx.clk.eq(platform.request("clk_mac_rx")),
        crg.cd_mac_tx.clk.eq(platform.request("clk_mac_tx")),
    ]


# -- CRG (Clock Reset Generator) — Cyclone 10LP --------------------------------

class _CRG_Cyclone10(LiteXModule):
    def __init__(self, platform, sys_clk_freq):
        self.rst    = Signal()
        self.cd_sys = ClockDomain()

        # System clock fed from external (top-level) PLL.
        self.comb += self.cd_sys.clk.eq(platform.request("clk_sys"))
        self.specials += AsyncResetSynchronizer(self.cd_sys, self.rst)

        _bind_mac_clocks(self, platform)


# -- CRG (Clock Reset Generator) — spibone bridge (no main RAM) ---------------

class _CRG_Spibone(LiteXModule):
    """CRG for the CPU-less spibone bridge: sys clock + MAC clocks, no RAM clocks.

    Identical to _CRG_Cyclone10 minus any main-RAM concern — eth_buf still needs
    the mac_rx/mac_tx domains, so those are bound here too.
    """
    def __init__(self, platform, sys_clk_freq):
        self.rst    = Signal()
        self.cd_sys = ClockDomain()

        # System clock fed from external (top-level) PLL.
        self.comb += self.cd_sys.clk.eq(platform.request("clk_sys"))
        self.specials += AsyncResetSynchronizer(self.cd_sys, self.rst)

        _bind_mac_clocks(self, platform)


# -- CRG (Clock Reset Generator) — Cyclone 10LP CYC1000 (SDRAM) ---------------

class _CRG_Cyc1000(LiteXModule):
    def __init__(self, platform, sys_clk_freq):
        self.rst        = Signal()
        self.cd_sys     = ClockDomain()
        self.cd_sys_ps  = ClockDomain()  # 90deg phase-shifted for SDRAM

        # Both clocks fed from external (top-level) PLL.
        self.comb += self.cd_sys.clk.eq(platform.request("clk_sys"))
        self.comb += self.cd_sys_ps.clk.eq(platform.request("clk_sys_ps"))
        self.specials += AsyncResetSynchronizer(self.cd_sys, self.rst)

        # SDRAM clock output pad (phase-shifted)
        self.comb += platform.request("sdram_clock").eq(self.cd_sys_ps.clk)

        _bind_mac_clocks(self, platform)


# -- CRG (Clock Reset Generator) — Gowin GW2A (Tang Primer 20k) ---------------

class _CRG_Gowin(LiteXModule):
    def __init__(self, platform, sys_clk_freq):
        self.rst    = Signal()
        self.cd_sys = ClockDomain()
        self.cd_por = ClockDomain()

        # DDR3 requires 2:1 clock ratio
        self.cd_init    = ClockDomain()
        self.cd_sys2x   = ClockDomain()
        self.cd_sys2x_i = ClockDomain()

        # # #

        self.stop  = Signal()
        self.reset = Signal()

        # Both clocks fed from external (top-level) PLL.
        clk_sys   = platform.request("clk_sys")
        clk_sys2x = platform.request("clk_sys2x")

        # Power on reset (driven from sys clock — onboard POR is not aware
        # of reprogramming).
        por_count = Signal(16, reset=2**16-1)
        por_done  = Signal()
        self.comb += self.cd_por.clk.eq(clk_sys)
        self.comb += por_done.eq(por_count == 0)
        self.sync.por += If(~por_done, por_count.eq(por_count - 1))

        # 2:1 clock for DDR3: clk_sys2x → DHCEN (gated) → sys2x → CLKDIV/2 → sys
        self.comb += self.cd_sys2x_i.clk.eq(clk_sys2x)
        self.specials += [
            Instance("DHCEN",
                i_CLKIN  = self.cd_sys2x_i.clk,
                i_CE     = self.stop,
                o_CLKOUT = self.cd_sys2x.clk),
            Instance("CLKDIV",
                p_DIV_MODE = "2",
                i_CALIB    = 0,
                i_HCLKIN   = self.cd_sys2x.clk,
                i_RESETN   = ~self.reset,
                o_CLKOUT   = self.cd_sys.clk),
        ]

        # Init clock domain (raw sys clock, used by DDR3 init FSM)
        self.comb += self.cd_init.clk.eq(clk_sys)
        self.comb += self.cd_init.rst.eq(~por_done)

        self.specials += AsyncResetSynchronizer(self.cd_sys, ~por_done | self.rst | self.reset)

        _bind_mac_clocks(self, platform)
