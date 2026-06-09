"""AES67 SoC top-level assembly.

Composes the VexRiscv SoC, target-specific CRG + main RAM, and the AES67
peripherals.  Each Wishbone/CSR peripheral is pulled in via its ``add_*()``
helper from :mod:`aes67_soc.peripherals`, so adding or removing a peripheral is
a single line here.
"""

from migen import *

from litex.gen import *

from litex.soc.integration.soc_core import SoCCore
from litex.soc.integration.soc import SoCRegion
from litex.soc.cores.bitbang import I2CMaster
from litex.soc.cores.spi import SPIMaster
from litex.soc.cores.uart import RS232PHY, UART
from litex.soc.interconnect.csr import CSRStorage

from .crg import _CRG_Cyclone10, _CRG_Cyc1000, _CRG_Gowin
from .peripherals import (
    AES67HyperRAM,
    AES67CSRs, add_aes67_csr,
    EthPacketBuffer, add_eth_buffer,
    StreamConfigRAM, add_stream_cfg,
)


# -- SoC definition ----------------------------------------------------------

class AES67SoC(SoCCore):
    mem_map = dict(SoCCore.mem_map)
    mem_map.update({
        "main_ram":       0x20000000,  # HyperRAM serves as main RAM
        "spiflash":       0x30000000,  # W25Q64 SPI Flash (8 MB, memory-mapped)
        # Place packet buffers and stream config in IO region (>= 0x80000000)
        # so VexRiscV data cache treats them as uncacheable.
        # Bit 31 of the physical address controls isIoAccess in VexRiscv.v.
        "eth_buf":        0x90000000,
        "tx_stream_cfg":  0x90004000,
        "rx_stream_cfg":  0x90005000,
    })

    def __init__(self, platform, sys_clk_freq, target="cyclone10",
                 with_hyperram=False,
                 integrated_sram_size=4*1024, **kwargs):

        # Boot flow: CPU resets to SPI flash where a tiny boot stub copies
        # the BIOS into main RAM and jumps there.  The BIOS therefore runs
        # at full RAM speed instead of slow SPI read speed (~1.5 MiB/s).
        #
        # SPI Flash layout (see boot_stub/):
        #   0x30000000: boot_stub  (512 bytes)
        #   0x30000200: bios.bin   (up to ~63.5 KB)
        #   0x30010000: firmware   (Zephyr app, optional)
        #
        # Per-target BIOS run address:
        #   Cyclone10: copy to HyperRAM (pre-initialized by stub via CSR) →
        #              linked at top of 8 MB HyperRAM = 0x207F0000.
        #   Gowin:     copy to DDR3 top = 0x27FF0000.
        #   Cyc1000:   SDR SDRAM needs an init sequence that only the BIOS
        #              knows how to run, so we execute-in-place (XIP) the
        #              BIOS directly from SPI flash at 0x30000200.  The
        #              BIOS then initializes SDRAM itself, loads firmware
        #              into SDRAM, and jumps there.
        cpu_reset_address = self.mem_map["spiflash"]  # 0x30000000
        bios_size         = 0x10000                    # 64 KB

        if target == "gowin":
            main_ram_size    = 128*1024*1024  # 128 MB DDR3
            bios_run_address = 0x20000000 + main_ram_size - bios_size
        elif target == "cyc1000":
            main_ram_size    = 4*1024*1024    # 4 MB SDR SDRAM (M12L64322A, 64 Mbit)
            bios_run_address = cpu_reset_address + 0x200  # XIP from flash
        else:
            main_ram_size    = 8*1024*1024    # 8 MB HyperRAM
            bios_run_address = 0x20000000 + main_ram_size - bios_size

        # SoCCore - must be initialized before CRG
        SoCCore.__init__(self, platform, sys_clk_freq,
            cpu_type             = "vexriscv",
            cpu_variant          = "standard",
            bus_interconnect     = "crossbar",
            integrated_rom_size  = 0,
            cpu_reset_address    = cpu_reset_address,
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

        # VexRiscv's CPU-level mem_map hardcodes main_ram at 0x40000000 and
        # overwrites our subclass setting during SoCCore.__init__.  Restore
        # our value so add_sdram() / bus.add_slave() place main_ram at the
        # same address the BIOS linker region expects (0x20000000).
        self.mem_map["main_ram"] = 0x20000000

        # ROM_DISABLE: no block-RAM ROM, BIOS is external.
        # Linker region "rom" points to top of main RAM so BIOS gets linked
        # at the address where it will actually execute (after boot stub
        # copies it there).
        self.add_constant("ROM_DISABLE", 1)
        self.cpu.use_rom = True
        self.bus.add_region("rom", SoCRegion(
            origin = bios_run_address,
            size   = bios_size,
            mode   = "rx",
            cached = True,
            linker = True))

        # -- CRG and main RAM (target-specific) --------------------------------
        if target == "gowin":
            # Gowin GW2A: external clk_sys + clk_sys2x + DDR3
            self.crg = _CRG_Gowin(platform, sys_clk_freq)

            from litedram.phy import GW2DDRPHY
            from litedram.modules import MT41K64M16
            self.ddrphy = GW2DDRPHY(
                pads         = platform.request("ddram"),
                sys_clk_freq = sys_clk_freq,
            )
            self.ddrphy.settings.rtt_nom = "disabled"
            self.comb += self.crg.stop.eq(self.ddrphy.init.stop)
            self.comb += self.crg.reset.eq(self.ddrphy.init.reset)
            self.add_sdram("sdram",
                phy           = self.ddrphy,
                module        = MT41K64M16(sys_clk_freq, "1:2"),
                origin        = self.mem_map["main_ram"],
                size          = main_ram_size,
                l2_cache_size = 8192,
            )
        elif target == "cyc1000":
            # Trenz CYC1000: external clk_sys + clk_sys_ps + SDR SDRAM
            self.crg = _CRG_Cyc1000(platform, sys_clk_freq)

            from litedram.phy import GENSDRPHY
            from litedram.modules import M12L64322A
            self.sdrphy = GENSDRPHY(platform.request("sdram"), sys_clk_freq)
            self.add_sdram("sdram",
                phy           = self.sdrphy,
                module        = M12L64322A(sys_clk_freq, "1:1"),
                origin        = self.mem_map["main_ram"],
                size          = main_ram_size,
                l2_cache_size = 8192,
            )
        else:
            # Cyclone 10LP: external clk_sys + HyperRAM (4:1 ratio).
            self.crg = _CRG_Cyclone10(platform, sys_clk_freq)

            if with_hyperram:
                self.hyperram = AES67HyperRAM(
                    pads         = platform.request("hyperram"),
                    sys_clk_freq = sys_clk_freq,
                    clk_ratio    = "4:1",
                )
                self.bus.add_slave("main_ram", slave=self.hyperram.bus,
                    region=SoCRegion(origin=0x20000000, size=main_ram_size, mode="rwx"))

        # -- I2C 0: Display (SSD1306) + PLL (Si5351A) -------------------------
        self.i2c0 = I2CMaster(platform.request("i2c", 0))

        # -- I2C 1: AD/DA Card Controller (hardware I2C via LiteI2C) -----------
        self.add_i2c_master(
            name  = "i2c1",
            pads  = platform.request("i2c", 1),
        )

        # -- SPI: SD Card ------------------------------------------------------
        self.spi0 = SPIMaster(
            pads         = platform.request("spi"),
            data_width   = 8,
            sys_clk_freq = sys_clk_freq,
            spi_clk_freq = 400e3,  # SD card init requires ≤400 kHz
        )

        # -- SPI Flash: W25Q64 (8 MB, memory-mapped, BIOS executes from here) --
        # with_master=True: enables the SPI master port alongside memory-mapped
        # reads.  The master port allows the firmware to issue raw SPI commands
        # (write-enable, page-program, sector-erase) for in-system firmware
        # updates.  The BIOS still skips frequency calibration (see
        # SPIFLASH_SKIP_FREQ_INIT below) so it never activates the master
        # during boot — avoiding the XIP deadlock.  By the time Zephyr runs,
        # all code executes from HyperRAM, so master access is safe.
        # XIP targets (cyc1000) run the BIOS directly from the memory-mapped
        # SPI flash, so any master-port access during BIOS init would deadlock
        # the CPU (the BIOS is fetching instructions from MMAP while the
        # master tries to take over the same port).  Disable the master port
        # entirely on XIP builds.  Targets that copy the BIOS into RAM first
        # (cyclone10, gowin) can keep the master enabled for firmware updates.
        xip_boot = (target == "cyc1000")

        from litespi.modules import W25Q64
        from litespi.opcodes import SpiNorFlashOpCodes
        self.add_spi_flash(name="spiflash", mode="1x",
            module=W25Q64(SpiNorFlashOpCodes.READ_1_1_1),
            clk_freq=20e6,
            with_master=not xip_boot)

        if not xip_boot:
            # Gate the MMAP port's crossbar request with a CSR so firmware can
            # disable memory-mapped reads while using the SPI master port.
            # Without this, the MMAP port's round-robin arbitration interferes
            # with master transactions, corrupting SPI flash writes/reads.
            self.spiflash_mmap_en = CSRStorage(1, reset=1,
                description="Set to 0 to disable MMAP flash access (allows clean master access).")
            original_mmap_req = self.spiflash.crossbar.user_request[0]
            gated_req = Signal()
            self.comb += gated_req.eq(original_mmap_req & self.spiflash_mmap_en.storage)
            self.spiflash.crossbar.user_request[0] = gated_req

        # Skip SPI Flash frequency auto-calibration — on RAM-copy targets the
        # master port would interfere with the MMAP arbitration; on XIP the
        # BIOS itself is running from the flash it would be calibrating.
        self.add_constant("SPIFLASH_SKIP_FREQ_INIT", 1)

        # Firmware image sits after stub + BIOS region in SPI flash.
        self.add_constant("FLASH_BOOT_ADDRESS",
            self.mem_map["spiflash"] + bios_size)

        # -- UART 1: AD/DA Card LEDs (115200) ----------------------------------
        self.uart1_phy = RS232PHY(platform.request("serial", 1), clk_freq=sys_clk_freq, baudrate=115200)
        self.uart1 = UART(self.uart1_phy, tx_fifo_depth=16, rx_fifo_depth=16)
        self.irq.add("uart1")

        # -- sys_clk output (for FPGA-side config RAM write clocks) -------------
        self.comb += platform.request("sys_clk_out").eq(ClockSignal("sys"))

        # -- AES67 peripherals -------------------------------------------------
        # Each peripheral is instantiated *here* (so Migen names its nets after
        # the SoC attribute, e.g. ``aes67_csr_*`` — the names that the Quartus
        # timing constraints in FPGA/sdc/litex_csr.sdc match on), then handed to
        # its module's wiring helper for the bus-slave / pad / IRQ hook-up.
        self.aes67_csr = AES67CSRs()
        add_aes67_csr(self, platform)

        self.eth_buf = EthPacketBuffer()
        add_eth_buffer(self, platform, origin=self.mem_map["eth_buf"])

        self.tx_stream_cfg = StreamConfigRAM()
        add_stream_cfg(self, platform, name="tx_stream_cfg",
                       pads_name="tx_stream_cfg", origin=self.mem_map["tx_stream_cfg"])
        self.rx_stream_cfg = StreamConfigRAM()
        add_stream_cfg(self, platform, name="rx_stream_cfg",
                       pads_name="rx_stream_cfg", origin=self.mem_map["rx_stream_cfg"])
