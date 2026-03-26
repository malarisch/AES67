/*
 * SPI NOR Flash driver for LiteSPI master (W25Q64).
 *
 * Matches the byte-at-a-time protocol used by the LiteX BIOS
 * (liblitespi/spiflash.c).  Always uses 8-bit transfers via the
 * LiteSPI master RXTX CSR.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include "spi_flash.h"
#include "../eth_litex/litex_csr_compat.h"

LOG_MODULE_REGISTER(spi_flash, LOG_LEVEL_DBG);

/* Mutex protecting all SPI master port access. */
static K_MUTEX_DEFINE(spi_mutex);

/* ── W25Q64 SPI commands ─────────────────────────────────────────── */
#define CMD_WRITE_ENABLE   0x06
#define CMD_READ_STATUS1   0x05
#define CMD_READ_DATA      0x03
#define CMD_PAGE_PROGRAM   0x02
#define CMD_SECTOR_ERASE   0x20  /* 4 KB */
#define CMD_READ_JEDEC_ID  0x9F
#define CMD_ENABLE_RESET   0x66
#define CMD_RESET_DEVICE   0x99

/* Status register bit 0 = WIP (write in progress) */
#define SR_WIP  0x01

/* ── MMAP port gating ────────────────────────────────────────────── */

/**
 * Disable the MMAP (memory-mapped read) port on the LiteSPI crossbar.
 * Must be called before any SPI master operation to prevent the MMAP
 * port from arbitrating on the SPI bus and corrupting master transfers.
 */
static void spi_mmap_disable(void)
{
	csr_write_simple(0, CSR_MAIN_SPIFLASH_MMAP_EN_ADDR);
}

/**
 * Re-enable the MMAP port.  Call after master operations are complete.
 */
static void spi_mmap_enable(void)
{
	csr_write_simple(1, CSR_MAIN_SPIFLASH_MMAP_EN_ADDR);
}

/* ── Low-level helpers (match LiteX BIOS protocol) ───────────────── */

static inline bool spi_tx_ready(void)
{
	return (csr_read_simple(CSR_SPIFLASH_MASTER_STATUS_ADDR)
		>> CSR_SPIFLASH_MASTER_STATUS_TX_READY_OFFSET) & 1;
}

static inline bool spi_rx_ready(void)
{
	return (csr_read_simple(CSR_SPIFLASH_MASTER_STATUS_ADDR)
		>> CSR_SPIFLASH_MASTER_STATUS_RX_READY_OFFSET) & 1;
}

static void spi_drain_rx(void)
{
	while (spi_rx_ready()) {
		(void)csr_read_simple(CSR_SPIFLASH_MASTER_RXTX_ADDR);
	}
}

/**
 * Wait until the PHY is idle (tx_ready asserted, no pending RX data).
 * This ensures any in-flight MMAP or master transaction has completed.
 */
static void spi_wait_phy_idle(void)
{
	int timeout = 10000;

	/* Wait for TX ready — indicates PHY is not busy */
	while (!spi_tx_ready() && --timeout > 0) {
		k_busy_wait(1);
	}
	spi_drain_rx();
}

/**
 * Set PHY config for 8-bit, single-SPI, MOSI output enabled.
 * len=8, width=1, mask=1.
 */
static void spi_set_8bit_config(void)
{
	uint32_t word = (8u << CSR_SPIFLASH_MASTER_PHYCONFIG_LEN_OFFSET)
		      | (1u << CSR_SPIFLASH_MASTER_PHYCONFIG_WIDTH_OFFSET)
		      | (1u << CSR_SPIFLASH_MASTER_PHYCONFIG_MASK_OFFSET);
	csr_write_simple(word, CSR_SPIFLASH_MASTER_PHYCONFIG_ADDR);
}

/**
 * Transfer one byte: send `b`, return received byte.
 */
static uint8_t spi_transfer_byte(uint8_t b)
{
	while (!spi_tx_ready()) {
		/* spin */
	}
	csr_write_simple((uint32_t)b, CSR_SPIFLASH_MASTER_RXTX_ADDR);
	while (!spi_rx_ready()) {
		/* spin */
	}
	return (uint8_t)csr_read_simple(CSR_SPIFLASH_MASTER_RXTX_ADDR);
}

/**
 * Execute a SPI command: assert CS, transfer `len` bytes, deassert CS.
 * `tx` = bytes to send, `rx` = bytes received (may be same buffer or NULL).
 */
static void spi_cmd(const uint8_t *tx, uint8_t *rx, size_t len)
{
	spi_drain_rx();
	spi_set_8bit_config();
	csr_write_simple(1, CSR_SPIFLASH_MASTER_CS_ADDR);

	for (size_t i = 0; i < len; i++) {
		uint8_t r = spi_transfer_byte(tx[i]);
		if (rx) {
			rx[i] = r;
		}
	}

	csr_write_simple(0, CSR_SPIFLASH_MASTER_CS_ADDR);
}

/**
 * Execute a SPI command with a separate read phase.
 * Sends `tx_len` bytes from `tx`, then clocks out `rx_len` dummy bytes
 * while reading into `rx`.
 */
static void spi_cmd_read(const uint8_t *tx, size_t tx_len,
			  uint8_t *rx, size_t rx_len)
{
	spi_drain_rx();
	spi_set_8bit_config();
	csr_write_simple(1, CSR_SPIFLASH_MASTER_CS_ADDR);

	for (size_t i = 0; i < tx_len; i++) {
		(void)spi_transfer_byte(tx[i]);
	}
	for (size_t i = 0; i < rx_len; i++) {
		rx[i] = spi_transfer_byte(0x00);
	}

	csr_write_simple(0, CSR_SPIFLASH_MASTER_CS_ADDR);
}

/**
 * Execute a SPI command with a separate write phase.
 * Sends `cmd_len` command bytes, then `data_len` data bytes.
 */
static void spi_cmd_write(const uint8_t *cmd, size_t cmd_len,
			   const uint8_t *data, size_t data_len)
{
	spi_drain_rx();
	spi_set_8bit_config();
	csr_write_simple(1, CSR_SPIFLASH_MASTER_CS_ADDR);

	for (size_t i = 0; i < cmd_len; i++) {
		(void)spi_transfer_byte(cmd[i]);
	}
	for (size_t i = 0; i < data_len; i++) {
		(void)spi_transfer_byte(data[i]);
	}

	csr_write_simple(0, CSR_SPIFLASH_MASTER_CS_ADDR);
}

/* ── Flash operations ────────────────────────────────────────────── */

/**
 * Set SPI clock divisor.  SPI clock = sys_clk / (2 * (1 + div)).
 * At 80 MHz sys_clk: div=1 → 20 MHz, div=3 → 10 MHz, div=7 → 5 MHz.
 */
static void spi_set_clk_divisor(uint8_t div)
{
	csr_write_simple((uint32_t)div, CSR_SPIFLASH_PHY_CLK_DIVISOR_ADDR);
}

void spi_flash_init(void)
{
	/* Disable MMAP port — prevents crossbar arbitration interference.
	 * After boot everything runs from HyperRAM, MMAP reads not needed. */
	spi_mmap_disable();

	/* Deassert CS first, then wait for any in-flight PHY transaction
	 * (e.g. a stale MMAP read) to drain completely. */
	csr_write_simple(0, CSR_SPIFLASH_MASTER_CS_ADDR);
	k_msleep(1);
	spi_wait_phy_idle();

	/* Set a safe, slow SPI clock: div=7 → 5 MHz (sys=80MHz) */
	spi_set_clk_divisor(7);

	/* Toggle CS a few times with no data to flush any partial
	 * bit-state left in the flash's internal shift register. */
	for (int i = 0; i < 4; i++) {
		csr_write_simple(1, CSR_SPIFLASH_MASTER_CS_ADDR);
		k_busy_wait(10);
		csr_write_simple(0, CSR_SPIFLASH_MASTER_CS_ADDR);
		k_busy_wait(10);
	}

	/* Issue SPI flash reset sequence (0x66 + 0x99) */
	uint8_t cmd_rst;

	cmd_rst = CMD_ENABLE_RESET;
	spi_cmd(&cmd_rst, NULL, 1);
	k_usleep(50);

	cmd_rst = CMD_RESET_DEVICE;
	spi_cmd(&cmd_rst, NULL, 1);
	/* W25Q needs 30 µs after reset; wait longer to be safe */
	k_msleep(1);

	/* Drain again after reset — PHY may have buffered stale data */
	spi_wait_phy_idle();

	/* Read and log JEDEC ID (retry up to 5 times) */
	uint32_t id = 0;
	for (int attempt = 0; attempt < 5; attempt++) {
		id = spi_flash_read_jedec_id();
		LOG_INF("JEDEC ID attempt %d: 0x%06x", attempt, id);
		if ((id >> 16) == 0xEF) {
			break;  /* Manufacturer matches Winbond */
		}
		k_msleep(10);
	}

	LOG_INF("SPI flash master initialised (clk_div=7, JEDEC=0x%06x)", id);
}

uint32_t spi_flash_read_jedec_id(void)
{
	uint8_t tx[4] = { CMD_READ_JEDEC_ID, 0x00, 0x00, 0x00 };
	uint8_t rx[4] = { 0 };

	k_mutex_lock(&spi_mutex, K_FOREVER);

	spi_drain_rx();
	spi_set_8bit_config();
	csr_write_simple(1, CSR_SPIFLASH_MASTER_CS_ADDR);

	for (int i = 0; i < 4; i++) {
		rx[i] = spi_transfer_byte(tx[i]);
	}

	csr_write_simple(0, CSR_SPIFLASH_MASTER_CS_ADDR);

	k_mutex_unlock(&spi_mutex);

	LOG_DBG("JEDEC raw: %02x %02x %02x %02x", rx[0], rx[1], rx[2], rx[3]);

	return ((uint32_t)rx[1] << 16) | ((uint32_t)rx[2] << 8) | rx[3];
}

static uint8_t flash_read_status(void)
{
	uint8_t cmd = CMD_READ_STATUS1;
	uint8_t status;

	spi_cmd_read(&cmd, 1, &status, 1);
	return status;
}

static void flash_wait_busy(void)
{
	while (flash_read_status() & SR_WIP) {
		k_usleep(100);
	}
}

static void flash_write_enable(void)
{
	uint8_t cmd = CMD_WRITE_ENABLE;

	spi_cmd(&cmd, NULL, 1);
}

/* ── Internal (unlocked) flash operations ────────────────────────── */

static void flash_read_unlocked(uint32_t addr, uint8_t *buf, size_t len)
{
	uint8_t cmd[4] = {
		CMD_READ_DATA,
		(addr >> 16) & 0xFF,
		(addr >>  8) & 0xFF,
		 addr        & 0xFF,
	};

	spi_cmd_read(cmd, 4, buf, len);
}

static void flash_sector_erase_unlocked(uint32_t addr)
{
	uint8_t cmd[4] = {
		CMD_SECTOR_ERASE,
		(addr >> 16) & 0xFF,
		(addr >>  8) & 0xFF,
		 addr        & 0xFF,
	};

	flash_write_enable();
	spi_cmd(cmd, NULL, 4);
	flash_wait_busy();
}

static void flash_page_program_unlocked(uint32_t addr, const uint8_t *data,
					size_t len)
{
	uint8_t cmd[4] = {
		CMD_PAGE_PROGRAM,
		(addr >> 16) & 0xFF,
		(addr >>  8) & 0xFF,
		 addr        & 0xFF,
	};

	flash_write_enable();
	spi_cmd_write(cmd, 4, data, len);
	flash_wait_busy();
}

/* ── Public (locked) flash operations ────────────────────────────── */

void spi_flash_read(uint32_t addr, uint8_t *buf, size_t len)
{
	k_mutex_lock(&spi_mutex, K_FOREVER);
	flash_read_unlocked(addr, buf, len);
	k_mutex_unlock(&spi_mutex);
}

void spi_flash_sector_erase(uint32_t addr)
{
	k_mutex_lock(&spi_mutex, K_FOREVER);
	flash_sector_erase_unlocked(addr);
	k_mutex_unlock(&spi_mutex);
}

void spi_flash_page_program(uint32_t addr, const uint8_t *data, size_t len)
{
	if (len == 0 || len > SPI_FLASH_PAGE_SIZE) {
		return;
	}

	k_mutex_lock(&spi_mutex, K_FOREVER);
	flash_page_program_unlocked(addr, data, len);
	k_mutex_unlock(&spi_mutex);
}

int spi_flash_write_verified(uint32_t addr, const uint8_t *data, size_t len)
{
	uint8_t verify_buf[SPI_FLASH_PAGE_SIZE];

	k_mutex_lock(&spi_mutex, K_FOREVER);

	/* Erase required sectors */
	uint32_t erase_start = addr & ~(SPI_FLASH_SECTOR_SIZE - 1);
	uint32_t erase_end = (addr + len + SPI_FLASH_SECTOR_SIZE - 1) &
			     ~(SPI_FLASH_SECTOR_SIZE - 1);

	LOG_INF("Erasing 0x%08x..0x%08x (%u sectors)",
		erase_start, erase_end,
		(erase_end - erase_start) / SPI_FLASH_SECTOR_SIZE);

	for (uint32_t sa = erase_start; sa < erase_end; sa += SPI_FLASH_SECTOR_SIZE) {
		flash_sector_erase_unlocked(sa);
	}

	/* Program page by page */
	size_t offset = 0;

	while (offset < len) {
		size_t chunk = SPI_FLASH_PAGE_SIZE - ((addr + offset) % SPI_FLASH_PAGE_SIZE);
		if (chunk > len - offset) {
			chunk = len - offset;
		}

		flash_page_program_unlocked(addr + offset, data + offset, chunk);

		/* Verify */
		flash_read_unlocked(addr + offset, verify_buf, chunk);
		if (memcmp(verify_buf, data + offset, chunk) != 0) {
			LOG_ERR("Verify failed at 0x%08x", addr + (uint32_t)offset);
			k_mutex_unlock(&spi_mutex);
			return -EIO;
		}

		offset += chunk;
	}

	k_mutex_unlock(&spi_mutex);

	LOG_INF("Programmed and verified %u bytes at 0x%08x", (unsigned)len, addr);
	return 0;
}
