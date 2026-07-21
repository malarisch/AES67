/*

 *
 * Flash configuration storage implementation.
 *
 * Uses config_json for serialization / parsing on top of one of two
 * build-time-selected flash backends:
 *  - LITESPI (integrated softcore): the FPGA board's SPI NOR flash via
 *    the custom spi_flash driver, slots at CONFIG_FLASH_CONFIG_OFFSET.
 *  - FLASH_AREA (external MCU, e.g. ESP32-S3): the MCU's own program
 *    flash via Zephyr's flash map, slots at the start of the fixed
 *    "storage" partition.
 *
 * Two 8 KB slots (A/B) provide crash-safe updates:
 *  - Each slot has a 16-byte header: magic + sequence + length + CRC-32.
 *  - On load, the slot with the highest valid sequence is used.
 *  - On save, the older slot is overwritten and the sequence incremented.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/crc.h>
#include <zephyr/init.h>
#include <string.h>

#include "flash_config.h"
#include "config_json.h"

LOG_MODULE_REGISTER(flash_config, LOG_LEVEL_INF);

/* ---- Layout constants ---- */

#define CFG_SECTOR_SIZE 4096
#define SLOT_SIZE       (2 * CFG_SECTOR_SIZE)         /* 8 KB per slot */

#if defined(CONFIG_FLASH_CONFIG_BACKEND_FLASH_AREA)
/* Slots live at the start of the "storage" fixed partition. */
#define SLOT_A_OFFSET   0
#else
/* Offset within flash (0-based).  Must not overlap firmware area.
 * Default: 0x7F0000 = 8 MB - 64 KB, leaving 64 KB at the very top. */
#ifndef CONFIG_FLASH_CONFIG_OFFSET
#define CONFIG_FLASH_CONFIG_OFFSET  0x7F0000
#endif
#define SLOT_A_OFFSET   (CONFIG_FLASH_CONFIG_OFFSET)
#endif

#define SLOT_B_OFFSET   (SLOT_A_OFFSET + SLOT_SIZE)

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
 * Flash backend primitives
 * ================================================================ */

#if defined(CONFIG_FLASH_CONFIG_BACKEND_FLASH_AREA)

#include <zephyr/storage/flash_map.h>

static const struct flash_area *cfg_area;

static int cfg_backend_init(void)
{
	int ret = flash_area_open(FIXED_PARTITION_ID(storage_partition), &cfg_area);

	if (ret < 0) {
		LOG_ERR("Flash config: storage partition unavailable (err %d)", ret);
		return ret;
	}

	if (!device_is_ready(flash_area_get_device(cfg_area))) {
		LOG_ERR("Flash config: flash device not ready");
		return -ENODEV;
	}

	if (cfg_area->fa_size < SLOT_B_OFFSET + SLOT_SIZE) {
		LOG_ERR("Flash config: storage partition too small (%u < %u)",
			(unsigned int)cfg_area->fa_size,
			(unsigned int)(SLOT_B_OFFSET + SLOT_SIZE));
		return -ENOMEM;
	}

	/* The slot header (16 bytes) and the tail-padding logic in
	 * cfg_flash_write() assume the write-block size divides 16. */
	if (flash_area_align(cfg_area) > 16U ||
	    16U % flash_area_align(cfg_area) != 0U) {
		LOG_ERR("Flash config: unsupported write alignment %u",
			(unsigned int)flash_area_align(cfg_area));
		return -ENOTSUP;
	}

	LOG_INF("Flash config: using storage partition @ 0x%lx (%u KiB)",
		(unsigned long)cfg_area->fa_off,
		(unsigned int)(cfg_area->fa_size / 1024U));
	return 0;
}

static int cfg_flash_read(uint32_t offset, void *buf, size_t len)
{
	return flash_area_read(cfg_area, offset, buf, len);
}

/* Write @len bytes, padding the unaligned tail with 0xFF up to the
 * device's write-block size (padding stays within the slot: payloads are
 * capped at MAX_PAYLOAD and the header is a multiple of the alignment). */
static int cfg_flash_write(uint32_t offset, const void *buf, size_t len)
{
	uint8_t align = flash_area_align(cfg_area);
	size_t main_len = len & ~((size_t)align - 1U);
	int ret = 0;

	if (main_len > 0) {
		ret = flash_area_write(cfg_area, offset, buf, main_len);
	}

	if (ret == 0 && len > main_len) {
		uint8_t tail[16];

		memset(tail, 0xFF, sizeof(tail));
		memcpy(tail, (const uint8_t *)buf + main_len, len - main_len);
		ret = flash_area_write(cfg_area, offset + main_len, tail, align);
	}

	return ret;
}

static int cfg_flash_erase_slot(uint32_t offset)
{
	return flash_area_erase(cfg_area, offset, SLOT_SIZE);
}

#else /* CONFIG_FLASH_CONFIG_BACKEND_LITESPI */

#include "../drivers/spi_flash/spi_flash.h"

BUILD_ASSERT(CFG_SECTOR_SIZE == SPI_FLASH_SECTOR_SIZE,
	     "slot layout assumes 4 KiB erase sectors");

static int cfg_backend_init(void)
{
	uint32_t jedec = spi_flash_read_jedec_id();

	if (jedec == 0 || jedec == 0xFFFFFF) {
		LOG_ERR("Flash config: SPI flash not detected (JEDEC=0x%06x)",
			jedec);
		return -ENODEV;
	}

	if (SLOT_B_OFFSET + SLOT_SIZE > SPI_FLASH_TOTAL_SIZE) {
		LOG_ERR("Flash config: config area exceeds flash size!");
		return -ENOMEM;
	}

	LOG_INF("Flash config: init OK (JEDEC 0x%06x, config @ 0x%06x)",
		jedec, CONFIG_FLASH_CONFIG_OFFSET);
	return 0;
}

static int cfg_flash_read(uint32_t offset, void *buf, size_t len)
{
	spi_flash_read(offset, buf, len);
	return 0;
}

static int cfg_flash_write(uint32_t offset, const void *buf, size_t len)
{
	const uint8_t *src = buf;

	while (len > 0) {
		/* Respect page boundary: bytes left in current page. */
		size_t page_space = SPI_FLASH_PAGE_SIZE -
				    (offset % SPI_FLASH_PAGE_SIZE);
		size_t chunk = MIN(len, page_space);

		spi_flash_page_program(offset, src, chunk);
		offset += chunk;
		src    += chunk;
		len    -= chunk;
	}

	return 0;
}

static int cfg_flash_erase_slot(uint32_t offset)
{
	spi_flash_sector_erase(offset);
	spi_flash_sector_erase(offset + SPI_FLASH_SECTOR_SIZE);
	return 0;
}

#endif /* backend selection */

/* ================================================================
 * Internal: read and validate a slot header
 * ================================================================ */
static bool read_slot(uint32_t offset, struct slot_header *hdr)
{
	if (cfg_flash_read(offset, hdr, sizeof(*hdr)) != 0) {
		return false;
	}

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

	if (cfg_flash_read(offset + sizeof(struct slot_header),
			   json_buf, hdr->length) != 0) {
		return false;
	}
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

	/* Erase the slot. */
	if (cfg_flash_erase_slot(offset) != 0) {
		LOG_ERR("Flash config: slot erase failed");
		return -EIO;
	}

	/* Write header, then payload. */
	if (cfg_flash_write(offset, &hdr, sizeof(hdr)) != 0 ||
	    cfg_flash_write(offset + sizeof(hdr), payload, payload_len) != 0) {
		LOG_ERR("Flash config: slot write failed");
		return -EIO;
	}

	/* Read-back verification. */
	struct slot_header verify_hdr;

	if (cfg_flash_read(offset, &verify_hdr, sizeof(verify_hdr)) != 0) {
		LOG_ERR("Flash config: verify read failed");
		return -EIO;
	}

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
	int ret;

	/* Idempotent: the early SYS_INIT below already brings the backend up,
	 * and main() calls this again during its normal bring-up. */
	if (flash_ready) {
		return 0;
	}

	k_mutex_init(&flash_mutex);

	ret = cfg_backend_init();
	if (ret < 0) {
		return ret;
	}

	flash_ready = true;
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
	if (cfg_flash_read(use_offset + sizeof(struct slot_header),
			   json_buf, use_hdr->length) != 0) {
		LOG_ERR("Flash config: payload read failed");
		load_status = FLASH_CONFIG_LOAD_ERROR;
		k_mutex_unlock(&flash_mutex);
		return -EIO;
	}
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

	if (cfg_flash_erase_slot(SLOT_A_OFFSET) != 0 ||
	    cfg_flash_erase_slot(SLOT_B_OFFSET) != 0) {
		LOG_ERR("Flash config: erase failed");
		k_mutex_unlock(&flash_mutex);
		return -EIO;
	}

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

/* ================================================================
 * Early load
 * ================================================================ */

/*
 * Load the stored configuration before the network stack starts.
 *
 * The device MAC is derived from the configured product serial number, and
 * Zephyr's PTP stack derives its clock identity (EUI-64) from that MAC when
 * it initialises at APPLICATION/CONFIG_PTP_INIT_PRIO. Loading the config
 * here — after the flash driver (CONFIG_FLASH_INIT_PRIORITY) and before the
 * network stack (POST_KERNEL/CONFIG_NET_INIT_PRIO) — is what makes the
 * serial known early enough for both to use the final address.
 *
 * Failures are non-fatal: the compiled-in defaults simply stay in place and
 * main() retries the load during its normal bring-up.
 */
#define FLASH_CONFIG_EARLY_INIT_PRIO 70

BUILD_ASSERT(FLASH_CONFIG_EARLY_INIT_PRIO > CONFIG_FLASH_INIT_PRIORITY,
	     "config must load after the flash driver is ready");
#ifdef CONFIG_NET_INIT_PRIO
BUILD_ASSERT(FLASH_CONFIG_EARLY_INIT_PRIO < CONFIG_NET_INIT_PRIO,
	     "config must load before the network stack derives the MAC");
#endif

static int flash_config_early_load(void)
{
	int ret = flash_config_init();

	if (ret < 0) {
		LOG_WRN("Early flash config init failed (%d); using defaults", ret);
		return 0;
	}

	ret = flash_config_load();
	if (ret == -ENOENT) {
		LOG_INF("No stored configuration in flash yet, using defaults");
	} else if (ret < 0) {
		LOG_WRN("Early flash config load failed (%d); using defaults", ret);
	} else {
		LOG_INF("Configuration loaded from flash (pre-network)");
	}

	return 0;
}

SYS_INIT(flash_config_early_load, POST_KERNEL, FLASH_CONFIG_EARLY_INIT_PRIO);
