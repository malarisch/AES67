/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: Apache-2.0
 *
 * SPI Flash configuration storage implementation.
 *
 * Uses the custom spi_flash driver (LiteSPI master) for low-level
 * flash I/O and config_json for serialization / parsing.
 *
 * Two 8 KB slots (A/B) at the top of flash provide crash-safe updates:
 *  - Each slot has a 16-byte header: magic + sequence + length + CRC-32.
 *  - On load, the slot with the highest valid sequence is used.
 *  - On save, the older slot is overwritten and the sequence incremented.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/crc.h>
#include <string.h>

#include "flash_config.h"
#include "config_json.h"
#include "../drivers/spi_flash/spi_flash.h"

LOG_MODULE_REGISTER(flash_config, LOG_LEVEL_INF);

/* ---- Layout constants ---- */

/* Offset within flash (0-based).  Must not overlap firmware area.
 * Default: 0x7F0000 = 8 MB - 64 KB, leaving 64 KB at the very top. */
#ifndef CONFIG_FLASH_CONFIG_OFFSET
#define CONFIG_FLASH_CONFIG_OFFSET  0x7F0000
#endif

#define SLOT_SIZE       (2 * SPI_FLASH_SECTOR_SIZE)   /* 8 KB per slot */
#define SLOT_A_OFFSET   (CONFIG_FLASH_CONFIG_OFFSET)
#define SLOT_B_OFFSET   (CONFIG_FLASH_CONFIG_OFFSET + SLOT_SIZE)

/* Maximum JSON payload per slot (slot size minus header). */
#define MAX_PAYLOAD     (SLOT_SIZE - sizeof(struct slot_header))

#define FLASH_CONFIG_MAGIC  0xCF67AE01u

/* ---- Slot header (16 bytes, stored little-endian) ---- */
struct slot_header {
	uint32_t magic;     /* FLASH_CONFIG_MAGIC */
	uint32_t seq;       /* Monotonically increasing write counter */
	uint32_t length;    /* JSON payload length in bytes (excl NUL) */
	uint32_t crc32;     /* CRC-32 of the JSON payload */
};

/* ---- Module state ---- */
static struct k_mutex flash_mutex;
static bool flash_ready;
static bool config_dirty;
static enum flash_config_load_status load_status = FLASH_CONFIG_NOT_LOADED;

/* Sequence numbers of the two slots (0 = invalid / empty). */
static uint32_t seq_a;
static uint32_t seq_b;

/* ---- JSON buffer (shared with serialization) ---- */
#define JSON_BUF_SIZE  8192
static char json_buf[JSON_BUF_SIZE];

/* ================================================================
 * Internal: read and validate a slot header
 * ================================================================ */
static bool read_slot(uint32_t offset, struct slot_header *hdr)
{
	spi_flash_read(offset, (uint8_t *)hdr, sizeof(*hdr));

	if (hdr->magic != FLASH_CONFIG_MAGIC) {
		return false;
	}
	if (hdr->length == 0 || hdr->length > MAX_PAYLOAD) {
		return false;
	}
	return true;
}

/* Validate the payload CRC of a slot whose header was already read. */
static bool validate_slot_crc(uint32_t offset, const struct slot_header *hdr)
{
	/* Read payload into json_buf (caller must hold mutex). */
	if (hdr->length >= JSON_BUF_SIZE) {
		return false;
	}

	spi_flash_read(offset + sizeof(struct slot_header),
		       (uint8_t *)json_buf, hdr->length);
	json_buf[hdr->length] = '\0';

	uint32_t crc = crc32_ieee((const uint8_t *)json_buf, hdr->length);
	return (crc == hdr->crc32);
}

/* ================================================================
 * Internal: erase and write a slot
 * ================================================================ */
static int write_slot(uint32_t offset, uint32_t new_seq,
		      const char *payload, uint32_t payload_len)
{
	struct slot_header hdr = {
		.magic  = FLASH_CONFIG_MAGIC,
		.seq    = new_seq,
		.length = payload_len,
		.crc32  = crc32_ieee((const uint8_t *)payload, payload_len),
	};

	/* Erase the two sectors that make up this slot. */
	spi_flash_sector_erase(offset);
	spi_flash_sector_erase(offset + SPI_FLASH_SECTOR_SIZE);

	/* Write header. */
	spi_flash_page_program(offset, (const uint8_t *)&hdr, sizeof(hdr));

	/* Write payload page-by-page. */
	uint32_t addr = offset + sizeof(hdr);
	const uint8_t *src = (const uint8_t *)payload;
	uint32_t remaining = payload_len;

	while (remaining > 0) {
		/* Respect page boundary: bytes left in current page. */
		uint32_t page_space = SPI_FLASH_PAGE_SIZE -
				      (addr % SPI_FLASH_PAGE_SIZE);
		uint32_t chunk = (remaining < page_space) ? remaining
							  : page_space;

		spi_flash_page_program(addr, src, chunk);
		addr      += chunk;
		src       += chunk;
		remaining -= chunk;
	}

	/* Read-back verification. */
	struct slot_header verify_hdr;
	spi_flash_read(offset, (uint8_t *)&verify_hdr, sizeof(verify_hdr));

	if (verify_hdr.magic != FLASH_CONFIG_MAGIC ||
	    verify_hdr.seq   != new_seq ||
	    verify_hdr.length != payload_len ||
	    verify_hdr.crc32 != hdr.crc32) {
		LOG_ERR("Flash config: header verify failed");
		return -EIO;
	}

	/* Verify payload CRC */
	if (!validate_slot_crc(offset, &verify_hdr)) {
		LOG_ERR("Flash config: payload verify failed");
		return -EIO;
	}

	return 0;
}

/* ================================================================
 * Public API
 * ================================================================ */

int flash_config_init(void)
{
	k_mutex_init(&flash_mutex);

	/* Verify flash is accessible. */
	uint32_t jedec = spi_flash_read_jedec_id();
	if (jedec == 0 || jedec == 0xFFFFFF) {
		LOG_ERR("Flash config: SPI flash not detected (JEDEC=0x%06x)",
			jedec);
		return -ENODEV;
	}

	/* Sanity: make sure our config area doesn't exceed flash. */
	if (SLOT_B_OFFSET + SLOT_SIZE > SPI_FLASH_TOTAL_SIZE) {
		LOG_ERR("Flash config: config area exceeds flash size!");
		return -ENOMEM;
	}

	flash_ready = true;
	LOG_INF("Flash config: init OK (JEDEC 0x%06x, config @ 0x%06x)",
		jedec, CONFIG_FLASH_CONFIG_OFFSET);
	return 0;
}

int flash_config_load(void)
{
	struct slot_header hdr_a, hdr_b;
	bool valid_a, valid_b;

	if (!flash_ready) {
		load_status = FLASH_CONFIG_LOAD_ERROR;
		return -ENODEV;
	}

	k_mutex_lock(&flash_mutex, K_FOREVER);

	/* Read both slot headers. */
	valid_a = read_slot(SLOT_A_OFFSET, &hdr_a);
	valid_b = read_slot(SLOT_B_OFFSET, &hdr_b);

	/* Validate CRCs for valid headers. */
	if (valid_a) {
		valid_a = validate_slot_crc(SLOT_A_OFFSET, &hdr_a);
		if (!valid_a) {
			LOG_WRN("Slot A: CRC mismatch");
		}
	}
	if (valid_b) {
		valid_b = validate_slot_crc(SLOT_B_OFFSET, &hdr_b);
		if (!valid_b) {
			LOG_WRN("Slot B: CRC mismatch");
		}
	}

	seq_a = valid_a ? hdr_a.seq : 0;
	seq_b = valid_b ? hdr_b.seq : 0;

	if (!valid_a && !valid_b) {
		LOG_INF("Flash config: no valid config found, using defaults");
		load_status = FLASH_CONFIG_LOAD_EMPTY;
		k_mutex_unlock(&flash_mutex);
		return -ENOENT;
	}

	/* Pick the slot with the higher sequence number. */
	uint32_t use_offset;
	const struct slot_header *use_hdr;

	if (!valid_b || (valid_a && hdr_a.seq >= hdr_b.seq)) {
		use_offset = SLOT_A_OFFSET;
		use_hdr = &hdr_a;
		LOG_INF("Flash config: using slot A (seq %u)", hdr_a.seq);
	} else {
		use_offset = SLOT_B_OFFSET;
		use_hdr = &hdr_b;
		LOG_INF("Flash config: using slot B (seq %u)", hdr_b.seq);
	}

	/* Read payload (validate_slot_crc already loaded it, but re-read
	 * in case the other slot's CRC check overwrote json_buf). */
	spi_flash_read(use_offset + sizeof(struct slot_header),
		       (uint8_t *)json_buf, use_hdr->length);
	json_buf[use_hdr->length] = '\0';

	/* Apply configuration. */
	config_json_parse_and_apply(json_buf);

	load_status = FLASH_CONFIG_LOAD_OK;
	k_mutex_unlock(&flash_mutex);
	return 0;
}

int flash_config_save(void)
{
	if (!flash_ready) {
		return -ENODEV;
	}

	k_mutex_lock(&flash_mutex, K_FOREVER);

	/* Serialize current configuration. */
	int len = config_json_serialize(json_buf, JSON_BUF_SIZE);
	if (len <= 0 || len >= JSON_BUF_SIZE) {
		LOG_ERR("Flash config: serialize failed");
		k_mutex_unlock(&flash_mutex);
		return -ENOMEM;
	}

	if ((uint32_t)len > MAX_PAYLOAD) {
		LOG_ERR("Flash config: payload too large (%d > %u)",
			len, (unsigned)MAX_PAYLOAD);
		k_mutex_unlock(&flash_mutex);
		return -ENOMEM;
	}

	/* Determine which slot to write (the older one). */
	uint32_t new_seq = ((seq_a > seq_b) ? seq_a : seq_b) + 1;
	uint32_t target_offset;

	if (seq_a <= seq_b) {
		target_offset = SLOT_A_OFFSET;
		LOG_INF("Flash config: writing slot A (seq %u)", new_seq);
	} else {
		target_offset = SLOT_B_OFFSET;
		LOG_INF("Flash config: writing slot B (seq %u)", new_seq);
	}

	int ret = write_slot(target_offset, new_seq, json_buf, (uint32_t)len);
	if (ret == 0) {
		/* Update cached sequence numbers. */
		if (target_offset == SLOT_A_OFFSET) {
			seq_a = new_seq;
		} else {
			seq_b = new_seq;
		}
		config_dirty = false;
		LOG_INF("Flash config: saved (%d bytes, seq %u)", len, new_seq);
	}

	k_mutex_unlock(&flash_mutex);
	return ret;
}

void flash_config_mark_dirty(void)
{
	config_dirty = true;
}

int flash_config_flush(void)
{
	if (!config_dirty) {
		return 0;
	}
	return flash_config_save();
}

int flash_config_erase(void)
{
	if (!flash_ready) {
		return -ENODEV;
	}

	k_mutex_lock(&flash_mutex, K_FOREVER);

	LOG_WRN("Flash config: erasing both config slots");

	spi_flash_sector_erase(SLOT_A_OFFSET);
	spi_flash_sector_erase(SLOT_A_OFFSET + SPI_FLASH_SECTOR_SIZE);
	spi_flash_sector_erase(SLOT_B_OFFSET);
	spi_flash_sector_erase(SLOT_B_OFFSET + SPI_FLASH_SECTOR_SIZE);

	seq_a = 0;
	seq_b = 0;
	load_status = FLASH_CONFIG_LOAD_EMPTY;

	k_mutex_unlock(&flash_mutex);
	LOG_INF("Flash config: both slots erased");
	return 0;
}

enum flash_config_load_status flash_config_get_load_status(void)
{
	return load_status;
}

const char *flash_config_status_str(enum flash_config_load_status status)
{
	switch (status) {
	case FLASH_CONFIG_NOT_LOADED:
		return "not_loaded";
	case FLASH_CONFIG_LOAD_OK:
		return "ok";
	case FLASH_CONFIG_LOAD_EMPTY:
		return "empty";
	case FLASH_CONFIG_LOAD_ERROR:
		return "error";
	default:
		return "unknown";
	}
}
