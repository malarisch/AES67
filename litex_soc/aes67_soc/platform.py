"""Platform / IO definitions and HDL-only build stubs (no synthesis).

The SoC is generated HDL-only: these platforms define just the pin map and a
``build()`` that emits Verilog without invoking the vendor toolchain.
"""

import os
import shutil

from litex.build.generic_platform import Pins, Subsignal, IOStandard
from litex.build.altera import AlteraPlatform
from litex.build.gowin.platform import GowinPlatform


# -- Shared I/O: everything except clock and main RAM (identical across targets) --

_io_common = [
    # MAC clock inputs (directly from FPGA Ethernet MAC)
    ("clk_mac_rx", 0, Pins(1)),
    ("clk_mac_tx", 0, Pins(1)),

    # SoC sys_clk output (for FPGA-side config RAM write clocks)
    ("sys_clk_out", 0, Pins(1)),

    # Serial / UART
    ("serial", 0,
        Subsignal("tx", Pins(1)),
        Subsignal("rx", Pins(1)),
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

    # SPI Flash: W25Q64 (8 MB, memory-mapped)
    ("spiflash", 0,
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
        Subsignal("pll_ppb_wc_count",   Pins(25)),
        Subsignal("pll_ppb_pll_count",  Pins(25)),
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
        # PTP BMA results (FPGA -> SoC, read-only status)
        Subsignal("ptp_is_leader",      Pins(1)),
        Subsignal("ptp_is_follower",    Pins(1)),
        Subsignal("ptp_leader_id",      Pins(64)),
        # Outputs (SoC -> external FPGA logic)
        Subsignal("pll_ppb_start",      Pins(1)),
        Subsignal("mac_addr",           Pins(48)),
        Subsignal("ip_addr",            Pins(32)),
        Subsignal("ptp_time_source",    Pins(8)),
        Subsignal("ptp_log_msg_interval", Pins(8)),
        Subsignal("ptp_announce_msg_interval", Pins(8)),
        Subsignal("ptp_gm_priority1",  Pins(8)),
        Subsignal("ptp_gm_priority2",  Pins(8)),
        Subsignal("ptp_gm_clock_class", Pins(8)),
        Subsignal("ptp_gm_clock_accuracy", Pins(8)),
        Subsignal("eth_tx_request",     Pins(1)),
        Subsignal("adda_nrst",          Pins(1)),   # AD/DA card nRST (output, active-high release)
        Subsignal("ptp_reset",          Pins(1)),   # SoC -> FPGA: reset PTP module + wallclock
        # PTP servo / parser tuning (SoC -> FPGA)
        Subsignal("servo_kp_gain",              Pins(8)),
        Subsignal("servo_ki_gain",              Pins(8)),
        Subsignal("servo_gain_shift",           Pins(5)),
        Subsignal("servo_gain_shift_locked",    Pins(5)),
        Subsignal("servo_ki_extra_shift",       Pins(5)),
        Subsignal("servo_filter_shift",         Pins(5)),
        Subsignal("servo_warmup_samples",       Pins(8)),
        Subsignal("servo_lock_threshold_ns",    Pins(32)),
        Subsignal("servo_unlock_threshold_ns",  Pins(32)),
        Subsignal("servo_lock_count_threshold", Pins(8)),
        Subsignal("parser_min_filter_enable",       Pins(1)),
        Subsignal("parser_min_filter_active_depth", Pins(8)),
        Subsignal("parser_delay_asymmetry_ns",      Pins(32)),
        # PTP servo monitoring (FPGA -> SoC)
        Subsignal("servo_mon_filtered_offset",      Pins(32)),
        Subsignal("servo_mon_integral_sum",         Pins(32)),
        Subsignal("servo_mon_pi_proportional",      Pins(32)),
        Subsignal("servo_mon_pi_sum_raw",           Pins(32)),
        Subsignal("servo_mon_effective_gain_shift", Pins(8)),
        Subsignal("servo_mon_lock_counter",         Pins(16)),
        Subsignal("servo_mon_sample_count",         Pins(16)),
        Subsignal("servo_mon_first_lock_achieved",  Pins(1)),
        # Audio metering (from external FPGA logic -> SoC)
        Subsignal("rx_meter_signal",    Pins(16)),  # RX signal detect (1 bit per channel)
        Subsignal("rx_meter_clip",      Pins(16)),  # RX clip detect (1 bit per channel)
        Subsignal("tx_meter_signal",    Pins(16)),  # TX signal detect (1 bit per channel)
        Subsignal("tx_meter_clip",      Pins(16)),  # TX clip detect (1 bit per channel)
        Subsignal("meter_clear",        Pins(1)),   # Metering clear toggle (SoC -> FPGA)
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

    # eth_buf RX-ready interrupt line, crossing the SoC<->bridge boundary.
    # The bridge (where eth_buf lives) drives this OUT; the SoC takes it IN and
    # feeds it to the VexRiscv external interrupt array (so the firmware keeps
    # its RX IRQ instead of polling rx_ready over the slow Wishbone bus).
    ("eth_buf_irq", 0, Pins(1)),

    # AES67 external Wishbone window — the bus boundary between the SoC (master)
    # and the standalone AES67 bridge (slave).  One contiguous 64 KiB window at
    # 0x90000000 carries the whole AES67 surface (eth_buf buffer, stream RAMs,
    # and the bridge's WB->CSR sub-bridge for aes67_csr).  Only requested by the
    # SoC built with aes67_external=True and by the aes67_bridge target.
    # 30-bit word address (4 GiB / 32-bit words).  Signal directions per the
    # Wishbone layout: adr/dat_w/sel/cyc/stb/we/cti/bte are M->S, dat_r/ack/err S->M.
    ("aes67_wb", 0,
        Subsignal("adr",   Pins(30)),
        Subsignal("dat_w", Pins(32)),
        Subsignal("dat_r", Pins(32)),
        Subsignal("sel",   Pins(4)),
        Subsignal("cyc",   Pins(1)),
        Subsignal("stb",   Pins(1)),
        Subsignal("we",    Pins(1)),
        Subsignal("cti",   Pins(3)),
        Subsignal("bte",   Pins(2)),
        Subsignal("ack",   Pins(1)),
        Subsignal("err",   Pins(1)),
    ),

    # SPIBone: 4-wire SPI -> Wishbone bridge (only requested by the spibone
    # target, where an external host replaces the VexRiscv softcore as the
    # sole Wishbone master).  Declared here so it is available to any target;
    # LiteX only emits requested resources as ports, so the 3 CPU targets that
    # never request it are unaffected.
    ("spibone", 0,
        Subsignal("clk",  Pins(1)),
        Subsignal("cs_n", Pins(1)),
        Subsignal("mosi", Pins(1)),
        Subsignal("miso", Pins(1)),
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

# Target-specific I/O: clock + main RAM
_io_cyclone10 = [
    ("clk_sys", 0, Pins(1)),
    ("hyperram", 0,
        Subsignal("clk",   Pins(1)),
        Subsignal("rst_n", Pins(1)),
        Subsignal("dq",    Pins(8)),
        Subsignal("cs_n",  Pins(1)),
        Subsignal("rwds",  Pins(1)),
    ),
] + _io_common

_io_cyc1000 = [
    ("clk_sys",    0, Pins(1)),
    ("clk_sys_ps", 0, Pins(1)),  # 90deg phase-shifted sys clock for SDRAM (from top-level PLL)
    ("sdram_clock", 0, Pins(1)),
    ("sdram", 0,
        Subsignal("a",     Pins(14)),
        Subsignal("ba",    Pins(2)),
        Subsignal("cs_n",  Pins(1)),
        Subsignal("cke",   Pins(1)),
        Subsignal("ras_n", Pins(1)),
        Subsignal("cas_n", Pins(1)),
        Subsignal("we_n",  Pins(1)),
        Subsignal("dq",    Pins(16)),
        Subsignal("dm",    Pins(2)),
    ),
] + _io_common

_io_gowin = [
    ("clk_sys",   0, Pins(1)),
    ("clk_sys2x", 0, Pins(1)),  # 2x sys clock for DDR3 (from top-level PLL)
    ("ddram", 0,
        Subsignal("a",       Pins(14)),
        Subsignal("ba",      Pins(3)),
        Subsignal("ras_n",   Pins(1)),
        Subsignal("cas_n",   Pins(1)),
        Subsignal("we_n",    Pins(1)),
        Subsignal("cs_n",    Pins(1)),
        Subsignal("dm",      Pins(2)),
        Subsignal("dq",      Pins(16)),
        Subsignal("dqs_p",   Pins(2)),
        Subsignal("dqs_n",   Pins(2)),
        Subsignal("clk_p",   Pins(1)),
        Subsignal("clk_n",   Pins(1)),
        Subsignal("cke",     Pins(1)),
        Subsignal("odt",     Pins(1)),
        Subsignal("reset_n", Pins(1)),
    ),
] + _io_common

# spibone target: CPU-less register bridge.  No main-RAM pins; the MAC clocks
# (clk_mac_rx/tx, used by eth_buf) already live in _io_common, and clk_sys feeds
# the sys domain.  The external host drives the bus over the "spibone" SPI pads.
_io_spibone = [
    ("clk_sys", 0, Pins(1)),
] + _io_common

# aes67_bridge target: the AES67 register surface split off as a standalone
# Wishbone-slave module.  Same IO superset as spibone (clk_sys + MAC clocks +
# the AES67 direct signals + the aes67_wb_* bus pads); LiteX only emits the
# resources actually requested by the build, so the unused pads (spiflash, i2c,
# spi, serial, …) never appear as ports.
_io_bridge = [
    ("clk_sys", 0, Pins(1)),
] + _io_common


# -- Shared build helper (HDL-only, no synthesis) -----------------------------

def _stub_build(platform, fragment, build_dir="build", build_name="litex_soc", run=True, **kwargs):
    os.makedirs(build_dir, exist_ok=True)
    cwd = os.getcwd()
    os.chdir(build_dir)

    from migen.fhdl.structure import _Fragment
    if not isinstance(fragment, _Fragment):
        fragment = fragment.get_fragment()
    platform.finalize(fragment)

    v_output = platform.get_verilog(fragment, name=build_name)
    v_output.write(f"{build_name}.v")

    # Copy all registered source files (e.g. VexRiscv CPU) into build dir
    for src_path, language, library, *_ in platform.sources:
        dst = os.path.join(build_dir, os.path.basename(src_path))
        if os.path.abspath(src_path) != os.path.abspath(dst):
            shutil.copy2(src_path, dst)

    os.chdir(cwd)
    return v_output.ns


class Cyclone10StubPlatform(AlteraPlatform):
    """Altera-based platform stub for HDL-only generation (no synthesis)."""
    default_clk_name   = "clk_sys"
    default_clk_period = 1e9 / 75e6

    def __init__(self):
        AlteraPlatform.__init__(self, "10CL025YU256I7G", _io_cyclone10, toolchain="quartus")

    def create_programmer(self):
        raise NotImplementedError

    def build(self, fragment, **kwargs):
        return _stub_build(self, fragment, **kwargs)


class SpiboneStubPlatform(AlteraPlatform):
    """Platform stub for the CPU-less spibone bridge (HDL-only generation).

    The FPGA device is irrelevant for HDL-only generation (no synthesis), so we
    reuse the Cyclone 10LP part.  The generated bridge is device-agnostic.
    """
    default_clk_name   = "clk_sys"
    default_clk_period = 1e9 / 75e6

    def __init__(self):
        AlteraPlatform.__init__(self, "10CL025YU256I7G", _io_spibone, toolchain="quartus")

    def create_programmer(self):
        raise NotImplementedError

    def build(self, fragment, **kwargs):
        return _stub_build(self, fragment, **kwargs)


class BridgeStubPlatform(AlteraPlatform):
    """Platform stub for the standalone AES67 Wishbone bridge (HDL-only).

    Device is irrelevant for HDL-only generation; reuse the Cyclone 10LP part.
    """
    default_clk_name   = "clk_sys"
    default_clk_period = 1e9 / 75e6

    def __init__(self):
        AlteraPlatform.__init__(self, "10CL025YU256I7G", _io_bridge, toolchain="quartus")

    def create_programmer(self):
        raise NotImplementedError

    def build(self, fragment, **kwargs):
        return _stub_build(self, fragment, **kwargs)


class Cyc1000StubPlatform(AlteraPlatform):
    """Altera-based platform stub for Trenz CYC1000 (SDRAM)."""
    default_clk_name   = "clk_sys"
    default_clk_period = 1e9 / 75e6

    def __init__(self):
        # CYC1000 uses -C8 speed grade (commercial).
        AlteraPlatform.__init__(self, "10CL025YU256C8G", _io_cyc1000, toolchain="quartus")

    def create_programmer(self):
        raise NotImplementedError

    def build(self, fragment, **kwargs):
        return _stub_build(self, fragment, **kwargs)


class GowinStubPlatform(GowinPlatform):
    """Gowin-based platform stub for HDL-only generation (no synthesis)."""
    default_clk_name   = "clk_sys"
    default_clk_period = 1e9 / 75e6

    def __init__(self):
        GowinPlatform.__init__(self, "GW2A-LV18PG256C8/I7", _io_gowin,
                               toolchain="gowin", devicename="GW2A-18C")

    def create_programmer(self):
        raise NotImplementedError

    def build(self, fragment, **kwargs):
        return _stub_build(self, fragment, **kwargs)
