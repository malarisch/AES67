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

from .crg import _CRG_Cyclone10, _CRG_Cyc1000, _CRG_Gowin, _CRG_Spibone
from .peripherals import (
    AES67HyperRAM,
    AES67CSRs, add_aes67_csr,
    EthPacketBuffer, add_eth_buffer,
    StreamConfigRAM, add_stream_cfg,
    add_spibone,
    add_uartbone,
    add_external_wb_master, add_external_wb_slave,
    add_eth_irq_output, add_eth_irq_input,
)
from litex.soc.cores.spi.spi_bone import SPIBone
from litex.soc.cores.uart import UARTBone, UARTPHY


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
                 integrated_sram_size=4*1024,
                 with_servo=True, with_metering=True, with_ptp_sw=False, **kwargs):

        # Architecture: the AES67 register surface (CSRs + eth_buf + stream RAMs)
        # lives ONLY in the standalone "aes67_bridge" module.  Every other target
        # is a pure bus *master* that drives the bridge across top.vhd:
        #   - cyclone10/cyc1000/gowin: VexRiscv SoC, exposes a Wishbone master to
        #                              the bridge (no AES67 peripherals locally).
        #   - "spibone":               CPU-less SPI->Wishbone master to the bridge.
        #   - "uartbone":              CPU-less UART->Wishbone master to the bridge.
        #   - "aes67_bridge":          the AES67 peripherals as a Wishbone-slave
        #                              module (the only target that builds them).
        #
        # All CPU-less targets drop everything CPU-adjacent (VexRiscv, BIOS/boot,
        # main RAM, SPI-flash boot, I2C/SPI/UART).
        is_spibone      = (target == "spibone")
        is_uartbone     = (target == "uartbone")
        is_aes67_bridge = (target == "aes67_bridge")
        cpu_less        = is_spibone or is_uartbone or is_aes67_bridge

        if is_aes67_bridge:
            # Land the bridge's auto Wishbone->CSR bridge (which carries
            # aes67_csr) inside the single AES67 window, above the eth_buf buffer
            # (0x90000000) and stream RAMs (0x90004/5000).  The CSR region is
            # 64 KiB and LiteX requires a region origin aligned to its size, so
            # 0x90010000 (64 KiB-aligned) — the window spans 0x90000000+0x20000.
            self.mem_map = dict(self.mem_map)
            self.mem_map["csr"] = 0x90010000

        if cpu_less:
            # SoCMini-equivalent: no CPU, no integrated ROM/SRAM/main-RAM, no
            # UART/timer.  The bus master is added after the peripherals so it
            # sees every slave (SPIBone for spibone; external WB for aes67_bridge).
            ident = {
                "spibone":      "AES67-LiteX-SoC-spibone",
                "uartbone":     "AES67-LiteX-SoC-uartbone",
                "aes67_bridge": "AES67-LiteX-Bridge",
            }[target]
            SoCCore.__init__(self, platform, sys_clk_freq,
                cpu_type                 = "None",
                bus_interconnect         = "crossbar",
                integrated_rom_size      = 0,
                integrated_sram_size     = 0,
                integrated_main_ram_size = 0,
                ident                    = ident,
                ident_version            = True,
                with_uart                = False,
                with_timer               = False,
                **kwargs,
            )
        else:
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
        if cpu_less:
            # CPU-less bridge (spibone / uartbone / aes67_bridge): sys clock
            # only, no main RAM.  Only aes67_bridge instantiates eth_buf, so only
            # it needs the mac_rx/tx domains; spibone/uartbone leave them unbound.
            self.crg = _CRG_Spibone(platform, sys_clk_freq,
                                    with_mac_clocks=is_aes67_bridge)
        elif target == "gowin":
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

        # -- CPU-side peripherals (I2C / SPI / flash / UART) -------------------
        # Skipped entirely on the CPU-less bridges (spibone / aes67_bridge):
        # there is no CPU to drive them, and a pure register bridge keeps only
        # the AES67 CSRs + memory-mapped peripherals below.
        if not cpu_less:
            # -- I2C 0: Display (SSD1306) + PLL (Si5351A) ---------------------
            self.i2c0 = I2CMaster(platform.request("i2c", 0))

            # -- I2C 1: AD/DA Card Controller (hardware I2C via LiteI2C) ------
            self.add_i2c_master(
                name  = "i2c1",
                pads  = platform.request("i2c", 1),
            )

            # -- SPI: SD Card --------------------------------------------------
            self.spi0 = SPIMaster(
                pads         = platform.request("spi"),
                data_width   = 8,
                sys_clk_freq = sys_clk_freq,
                spi_clk_freq = 400e3,  # SD card init requires ≤400 kHz
            )

            # -- SPI Flash: W25Q64 (8 MB, memory-mapped, BIOS executes here) --
            # with_master=True: enables the SPI master port alongside memory-
            # mapped reads.  The master port allows the firmware to issue raw
            # SPI commands (write-enable, page-program, sector-erase) for
            # in-system firmware updates.  The BIOS still skips frequency
            # calibration (see SPIFLASH_SKIP_FREQ_INIT below) so it never
            # activates the master during boot — avoiding the XIP deadlock.  By
            # the time Zephyr runs, all code executes from HyperRAM, so master
            # access is safe.
            # XIP targets (cyc1000) run the BIOS directly from the memory-mapped
            # SPI flash, so any master-port access during BIOS init would
            # deadlock the CPU (the BIOS is fetching instructions from MMAP
            # while the master tries to take over the same port).  Disable the
            # master port entirely on XIP builds.  Targets that copy the BIOS
            # into RAM first (cyclone10, gowin) can keep the master enabled for
            # firmware updates.
            xip_boot = (target == "cyc1000")

            from litespi.modules import W25Q64
            from litespi.opcodes import SpiNorFlashOpCodes
            self.add_spi_flash(name="spiflash", mode="1x",
                module=W25Q64(SpiNorFlashOpCodes.READ_1_1_1),
                clk_freq=20e6,
                with_master=not xip_boot)

            if not xip_boot:
                # Gate the MMAP port's crossbar request with a CSR so firmware
                # can disable memory-mapped reads while using the SPI master
                # port.  Without this, the MMAP port's round-robin arbitration
                # interferes with master transactions, corrupting SPI flash
                # writes/reads.
                self.spiflash_mmap_en = CSRStorage(1, reset=1,
                    description="Set to 0 to disable MMAP flash access (allows clean master access).")
                original_mmap_req = self.spiflash.crossbar.user_request[0]
                gated_req = Signal()
                self.comb += gated_req.eq(original_mmap_req & self.spiflash_mmap_en.storage)
                self.spiflash.crossbar.user_request[0] = gated_req

            # Skip SPI Flash frequency auto-calibration — on RAM-copy targets
            # the master port would interfere with the MMAP arbitration; on XIP
            # the BIOS itself is running from the flash it would be calibrating.
            self.add_constant("SPIFLASH_SKIP_FREQ_INIT", 1)

            # Firmware image sits after stub + BIOS region in SPI flash.
            self.add_constant("FLASH_BOOT_ADDRESS",
                self.mem_map["spiflash"] + bios_size)

            # -- UART 1: AD/DA Card LEDs (115200) -----------------------------
            self.uart1_phy = RS232PHY(platform.request("serial", 1), clk_freq=sys_clk_freq, baudrate=115200)
            self.uart1 = UART(self.uart1_phy, tx_fifo_depth=16, rx_fifo_depth=16)
            self.irq.add("uart1")

        # -- AES67 register surface --------------------------------------------
        # The AES67 surface (aes67_csr + eth_buf + stream RAMs) lives in ONE
        # contiguous Wishbone window (0x90000000 + 128 KiB) so it can be a
        # standalone module.  The CSRs can't stay in the SoC's shared CSR space
        # (interleaved with the CPU's own banks), hence a dedicated window with
        # the bridge's own WB->CSR sub-bridge (see mem_map["csr"] = 0x90010000).
        #
        #   - aes67_bridge: builds the peripherals + exposes them via an INCOMING
        #     external WB master port (the SoC/SPI master drives it in top.vhd).
        #   - every other target: builds NO peripherals; exposes a master toward
        #     this window, routed to the bridge across top.vhd.
        AES67_WIN_ORIGIN = 0x90000000
        AES67_WIN_SIZE   = 0x20000     # 128 KiB (IO slaves + CSR sub-bridge @ 0x90010000)

        if is_aes67_bridge:
            # The only target that instantiates the AES67 peripherals.  Each is
            # created *in this scope* so Migen names its nets after the SoC
            # attribute (e.g. ``aes67_csr_*`` — the names the Quartus timing
            # constraints in FPGA/sdc/litex_csr.sdc match on), then wired up.
            self.aes67_csr = AES67CSRs(with_servo=with_servo, with_metering=with_metering,
                                       with_ptp_sw=with_ptp_sw)
            add_aes67_csr(self, platform)

            self.eth_buf = EthPacketBuffer()
            add_eth_buffer(self, platform, origin=self.mem_map["eth_buf"])

            self.tx_stream_cfg = StreamConfigRAM()
            add_stream_cfg(self, platform, name="tx_stream_cfg",
                           pads_name="tx_stream_cfg", origin=self.mem_map["tx_stream_cfg"])
            self.rx_stream_cfg = StreamConfigRAM()
            add_stream_cfg(self, platform, name="rx_stream_cfg",
                           pads_name="rx_stream_cfg", origin=self.mem_map["rx_stream_cfg"])

            # Incoming external WB master port (driven by the SoC/SPI master).
            add_external_wb_master(self, platform, "aes67_wb")

            # Route eth_buf's RX-ready IRQ out to a pad so the CPU SoC (a
            # separate module) can take it as a real interrupt.
            add_eth_irq_output(self, platform, "eth_buf_irq")
        else:
            # Master-only target: route the AES67 window out to the bridge over a
            # single external WB slave region.  On the CPU-less *bone masters the
            # bridge core is added first so the crossbar has a master driving that
            # region.
            if is_spibone:
                # SPI->Wishbone master.  Constructed here (not in the helper) so
                # Migen names its nets ``spibone_*`` after this SoC attribute.
                self.spibone = SPIBone(platform.request("spibone"))
                add_spibone(self, platform)
            elif is_uartbone:
                # UART->Wishbone master.  Constructed here (not in the helper) so
                # Migen names its nets ``uartbone_*`` after this SoC attribute.
                self.uartbone = UARTBone(
                    phy           = UARTPHY(platform.request("uartbone"),
                                            clk_freq = sys_clk_freq,
                                            baudrate = 115200),
                    clk_freq      = sys_clk_freq,
                    address_width = self.bus.address_width)
                add_uartbone(self, platform)
            add_external_wb_slave(self, platform, "aes67_wb",
                region=SoCRegion(origin=AES67_WIN_ORIGIN, size=AES67_WIN_SIZE, cached=False))

            # CPU targets: take the bridge's eth_buf IRQ in and map it to a
            # VexRiscv external interrupt (so the firmware keeps its RX IRQ
            # instead of polling).  The CPU-less *bone masters have no IRQ
            # controller.
            if not cpu_less:
                add_eth_irq_input(self, platform, "eth_buf_irq", name="eth_buf")
