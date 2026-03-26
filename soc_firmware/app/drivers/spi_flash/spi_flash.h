/*
 * SPI NOR Flash driver for LiteSPI master (W25Q64).
 *
 * Provides sector erase, page program, read, and JEDEC-ID queries
 * via the LiteSPI master CSRs.  All code runs from HyperRAM, so
 * using the master port does not conflict with XIP boot.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SPI_FLASH_H_
#define SPI_FLASH_H_

#include <stdint.h>
#include <stddef.h>

/* W25Q64 geometry */
#define SPI_FLASH_PAGE_SIZE    256
#define SPI_FLASH_SECTOR_SIZE  4096
#define SPI_FLASH_TOTAL_SIZE   (8 * 1024 * 1024)  /* 8 MB */

/* Firmware image sits at this offset in flash (after boot stub + BIOS). */
#define SPI_FLASH_FW_OFFSET    0x10000

/**
 * @brief Initialise the SPI flash master interface.
 *
 * Configures the LiteSPI PHY for single-bit (1x) SPI at the current
 * clock divider setting.  Must be called once before any other function.
 */
void spi_flash_init(void);

/**
 * @brief Read the JEDEC manufacturer / device ID.
 * @return 24-bit ID (manufacturer << 16 | type << 8 | capacity).
 */
uint32_t spi_flash_read_jedec_id(void);

/**
 * @brief Read data from flash.
 *
 * @param addr  Byte address in flash (0-based).
 * @param buf   Destination buffer.
 * @param len   Number of bytes to read.
 */
void spi_flash_read(uint32_t addr, uint8_t *buf, size_t len);

/**
 * @brief Erase a 4 KB sector.
 *
 * @param addr  Any address within the sector to erase (aligned down).
 *              Blocks until the erase completes.
 */
void spi_flash_sector_erase(uint32_t addr);

/**
 * @brief Program up to 256 bytes (one page).
 *
 * @param addr  Start address (must not cross a page boundary).
 * @param data  Source data.
 * @param len   Number of bytes (1..256).
 *              Blocks until the program completes.
 */
void spi_flash_page_program(uint32_t addr, const uint8_t *data, size_t len);

/**
 * @brief Erase + program an arbitrary region.
 *
 * Erases the required sectors, then programs the data page-by-page.
 * Verifies each page after programming.
 *
 * @param addr  Start address (should be sector-aligned for best results).
 * @param data  Source data.
 * @param len   Number of bytes.
 * @return 0 on success, negative errno on failure.
 */
int spi_flash_write_verified(uint32_t addr, const uint8_t *data, size_t len);

#endif /* SPI_FLASH_H_ */
