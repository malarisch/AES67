# Agent Documentation: AES67 Project

## Subsystem: FMC Ethernet Driver (`eth_fmc_basic`)

### Overview
The `eth_fmc_basic` driver enables the STM32H7 MCU to communicate with the FPGA-based Ethernet MAC via the Flexible Memory Controller (FMC). This replaces the previous SPI-based approach for higher bandwidth.

### Critical Implementation Details

#### 1. Memory Protection Unit (MPU)
*   **Issue:** The Cortex-M7 core performs speculative reads on the FMC memory region (`0x60000000`). If the FPGA is not configured or the bus is unstable, these speculative reads cause a bus fault/hard fault immediately upon boot.
*   **Fix:** A specific MPU region must be defined in the Device Tree to mark the FMC bank as "Device" memory (`ATTR_MPU_IO`). This disables cache and speculative access.
*   **Location:** `soc_firmware/app/app.overlay`
    ```dts
    fmc_memory: memory@60000000 {
        compatible = "zephyr,memory-region", "mmio-sram";
        reg = <0x60000000 0x10000>;
        zephyr,memory-region = "FMC_ETH_MEM";
        zephyr,memory-attr = <( DT_MEM_ARM(ATTR_MPU_IO) )>;
    };
    ```

#### 2. Thread Stack Size
*   **Issue:** The RX thread allocates a full Ethernet frame buffer (`ETH_FMC_MAX_PKT_SIZE` ~1522 bytes) on the stack. The default stack size (1024 bytes) caused a stack overflow and immediate crash when the thread started.
*   **Fix:** Increased stack size to 4096 bytes.
*   **Configuration:** `soc_firmware/app/prj.conf`
    ```properties
    CONFIG_ETH_FMC_BASIC_RX_STACK_SIZE=4096
    ```

#### 3. FPGA Ready Signal
*   **Issue:** The MCU boots faster than the FPGA. Accessing the FMC bus before the FPGA is configured results in reading garbage (0xFFFF) or bus errors.
*   **Implementation:** A GPIO input is used to gate FMC access.
*   **Pin:** `PC13` (User Button on Nucleo-H753ZI).
*   **Logic:** Active High.
    *   `0`: Driver drops TX packets and skips RX polling.
    *   `1`: Driver operates normally.
*   **Code:** `soc_firmware/app/drivers/eth_fmc_basic/eth_fmc_basic.c` checks `is_fpga_ready()` in both RX and TX loops.

### Driver Files
*   **Source:** `soc_firmware/app/drivers/eth_fmc_basic/eth_fmc_basic.c`
*   **Kconfig:** `soc_firmware/app/drivers/eth_fmc_basic/Kconfig.fmc_basic`
*   **Overlay:** `soc_firmware/app/app.overlay`

### Build Instructions
```bash
cd soc_firmware/app
west build -p
west flash
```
