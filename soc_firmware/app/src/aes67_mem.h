/*
 * Memory-placement helpers shared by the application modules.
 *
 * AES67_BIG_BSS: put a large zero-initialised static buffer into external
 * PSRAM on the ESP32-S3 (.ext_ram.bss, zeroed by esp_psram init before
 * main()). dram0 on that part is ~390 KB and shared with IRAM; PSRAM is 8 MB.
 * Everywhere else the macro is a no-op.
 *
 * Rules for what may carry it:
 *  - plain data only: no thread stacks (Xtensa threads on PSRAM stacks
 *    crash), no k_work/k_sem/k_mutex/k_thread objects;
 *  - not touched from the spibone/eth_spi hot copy path (keep those
 *    scratch buffers in internal DRAM);
 *  - not accessed while the flash cache is disabled (only the esp_flash
 *    code itself runs then, with interrupts masked, so ordinary buffers
 *    are fine; flash_area_read() into PSRAM goes through a k_malloc'd
 *    bounce buffer, writes are chunked by esp_flash).
 */
#ifndef AES67_MEM_H
#define AES67_MEM_H

#if defined(CONFIG_ESP_SPIRAM)
#define AES67_BIG_BSS __attribute__((section(".ext_ram.bss")))
#else
#define AES67_BIG_BSS
#endif

#endif /* AES67_MEM_H */
