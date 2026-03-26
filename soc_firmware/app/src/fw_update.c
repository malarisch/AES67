/*
 * Firmware update module.
 *
 * Shell commands:
 *   fw_update info       – Show JEDEC ID and current firmware header
 *   fw_update erase      – Erase firmware region
 *   fw_update verify     – Verify CRC of current firmware image
 *
 * HTTP endpoint:
 *   POST /api/fw_update  – Upload .fbi image (raw binary body),
 *                           streams to flash, then responds with status.
 *   GET  /api/fw_update  – Returns current firmware info as JSON.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/http/server.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/sys/crc.h>
#include <string.h>
#include <stdio.h>

#include "fw_update.h"
#include "../drivers/spi_flash/spi_flash.h"

LOG_MODULE_REGISTER(fw_update, LOG_LEVEL_INF);

/* FBI header: 4 bytes length (LE) + 4 bytes CRC-32 (LE) */
#define FBI_HEADER_SIZE   8
#define FW_FLASH_ADDR     SPI_FLASH_FW_OFFSET   /* 0x10000 */
/* Max firmware size: flash total minus firmware offset */
#define FW_MAX_SIZE       (SPI_FLASH_TOTAL_SIZE - FW_FLASH_ADDR)

/* ── Streaming flash writer state ────────────────────────────────── */

struct fw_writer {
	uint32_t flash_addr;     /* next write address in flash */
	uint32_t total_received; /* bytes received so far */
	uint32_t total_expected; /* from Content-Length, or 0 if unknown */
	uint32_t erased_up_to;   /* sectors erased up to this address */
	uint8_t  page_buf[SPI_FLASH_PAGE_SIZE];
	size_t   page_buf_len;   /* bytes buffered in page_buf */
	bool     active;         /* update in progress */
	int      error;          /* non-zero on failure */
};

static struct fw_writer fw_state;

static void fw_writer_reset(void)
{
	memset(&fw_state, 0, sizeof(fw_state));
	fw_state.flash_addr = FW_FLASH_ADDR;
	fw_state.erased_up_to = FW_FLASH_ADDR;
}

/**
 * Ensure sectors are erased up to (but not including) `end_addr`.
 */
static void fw_ensure_erased(uint32_t end_addr)
{
	while (fw_state.erased_up_to < end_addr) {
		spi_flash_sector_erase(fw_state.erased_up_to);
		fw_state.erased_up_to += SPI_FLASH_SECTOR_SIZE;
	}
}

/**
 * Flush any remaining data in the page buffer to flash.
 */
static int fw_flush_page(void)
{
	if (fw_state.page_buf_len == 0) {
		return 0;
	}

	fw_ensure_erased(fw_state.flash_addr + fw_state.page_buf_len);
	spi_flash_page_program(fw_state.flash_addr, fw_state.page_buf,
			       fw_state.page_buf_len);

	/* Verify */
	uint8_t verify[SPI_FLASH_PAGE_SIZE];

	spi_flash_read(fw_state.flash_addr, verify, fw_state.page_buf_len);
	if (memcmp(verify, fw_state.page_buf, fw_state.page_buf_len) != 0) {
		LOG_ERR("FW: verify failed at 0x%08x", fw_state.flash_addr);
		return -EIO;
	}

	fw_state.flash_addr += fw_state.page_buf_len;
	fw_state.page_buf_len = 0;
	return 0;
}

/**
 * Feed data into the streaming flash writer.
 */
static int fw_writer_feed(const uint8_t *data, size_t len)
{
	while (len > 0) {
		size_t space = SPI_FLASH_PAGE_SIZE - fw_state.page_buf_len;
		size_t copy = (len < space) ? len : space;

		memcpy(fw_state.page_buf + fw_state.page_buf_len, data, copy);
		fw_state.page_buf_len += copy;
		fw_state.total_received += copy;
		data += copy;
		len -= copy;

		if (fw_state.page_buf_len == SPI_FLASH_PAGE_SIZE) {
			int ret = fw_flush_page();

			if (ret < 0) {
				return ret;
			}
		}
	}
	return 0;
}

/**
 * Finalise the write — flush remaining data.
 */
static int fw_writer_finish(void)
{
	return fw_flush_page();
}

/* ── FBI header helpers ──────────────────────────────────────────── */

struct fbi_header {
	uint32_t length;
	uint32_t crc32;
};

static bool read_fbi_header(struct fbi_header *hdr)
{
	uint8_t buf[FBI_HEADER_SIZE];

	spi_flash_read(FW_FLASH_ADDR, buf, FBI_HEADER_SIZE);

	hdr->length = (uint32_t)buf[0] | ((uint32_t)buf[1] << 8) |
		      ((uint32_t)buf[2] << 16) | ((uint32_t)buf[3] << 24);
	hdr->crc32 = (uint32_t)buf[4] | ((uint32_t)buf[5] << 8) |
		     ((uint32_t)buf[6] << 16) | ((uint32_t)buf[7] << 24);

	/* Sanity check */
	if (hdr->length == 0 || hdr->length > FW_MAX_SIZE ||
	    hdr->length == 0xFFFFFFFF) {
		return false;
	}
	return true;
}

static bool verify_fbi_crc(const struct fbi_header *hdr)
{
	/* Read payload in chunks and compute CRC-32 */
	uint32_t crc = 0;
	uint32_t remaining = hdr->length;
	uint32_t addr = FW_FLASH_ADDR + FBI_HEADER_SIZE;
	uint8_t chunk[256];

	while (remaining > 0) {
		size_t n = (remaining < sizeof(chunk)) ? remaining : sizeof(chunk);

		spi_flash_read(addr, chunk, n);
		crc = crc32_ieee_update(crc, chunk, n);
		addr += n;
		remaining -= n;
	}

	return crc == hdr->crc32;
}

/* ── Shell commands ──────────────────────────────────────────────── */

static int cmd_fw_info(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	uint32_t jedec = spi_flash_read_jedec_id();

	shell_print(sh, "JEDEC ID: 0x%06X (mfr=0x%02X type=0x%02X cap=0x%02X)",
		    jedec, (jedec >> 16) & 0xFF, (jedec >> 8) & 0xFF, jedec & 0xFF);

	struct fbi_header hdr;

	if (read_fbi_header(&hdr)) {
		shell_print(sh, "Firmware image at 0x%05X:", FW_FLASH_ADDR);
		shell_print(sh, "  Payload size: %u bytes", hdr.length);
		shell_print(sh, "  CRC-32:       0x%08X", hdr.crc32);
		shell_print(sh, "  Verifying...");
		if (verify_fbi_crc(&hdr)) {
			shell_print(sh, "  CRC OK");
		} else {
			shell_print(sh, "  CRC MISMATCH!");
		}
	} else {
		shell_print(sh, "No valid firmware image found at 0x%05X", FW_FLASH_ADDR);
	}
	return 0;
}

static int cmd_fw_erase(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "Erasing firmware region (0x%05X..0x%05X)...",
		    FW_FLASH_ADDR, SPI_FLASH_TOTAL_SIZE);

	for (uint32_t addr = FW_FLASH_ADDR; addr < SPI_FLASH_TOTAL_SIZE;
	     addr += SPI_FLASH_SECTOR_SIZE) {
		spi_flash_sector_erase(addr);
		if ((addr & 0xFFFF) == 0) {
			shell_print(sh, "  0x%06X...", addr);
		}
	}

	shell_print(sh, "Erase complete.");
	return 0;
}

static int cmd_fw_verify(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	struct fbi_header hdr;

	if (!read_fbi_header(&hdr)) {
		shell_print(sh, "No valid firmware image found.");
		return -ENOENT;
	}

	shell_print(sh, "Verifying %u bytes (CRC-32: 0x%08X)...",
		    hdr.length, hdr.crc32);

	if (verify_fbi_crc(&hdr)) {
		shell_print(sh, "CRC OK");
		return 0;
	}
	shell_print(sh, "CRC MISMATCH!");
	return -EIO;
}

static int cmd_fw_reboot(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "Rebooting...");
	k_msleep(100);
	sys_reboot(SYS_REBOOT_COLD);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(fw_update_cmds,
	SHELL_CMD(info, NULL, "Show JEDEC ID and firmware image header", cmd_fw_info),
	SHELL_CMD(erase, NULL, "Erase firmware region in SPI flash", cmd_fw_erase),
	SHELL_CMD(verify, NULL, "Verify CRC of current firmware image", cmd_fw_verify),
	SHELL_CMD(reboot, NULL, "Reboot the system", cmd_fw_reboot),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(fw_update, &fw_update_cmds,
		    "Firmware update commands", NULL);

/* ── HTTP handler (called from webserver api_handler) ────────────── */

int fw_update_http_handler(struct http_client_ctx *client,
			   enum http_data_status status,
			   const struct http_request_ctx *request_ctx,
			   struct http_response_ctx *response_ctx)
{
	static char resp_buf[256];

	static const struct http_header json_hdrs[] = {
		{.name = "Content-Type", .value = "application/json"},
		{.name = "Access-Control-Allow-Origin", .value = "*"},
	};
	static const struct http_header cors_hdrs[] = {
		{.name = "Access-Control-Allow-Origin", .value = "*"},
		{.name = "Access-Control-Allow-Methods", .value = "GET, POST, OPTIONS"},
		{.name = "Access-Control-Allow-Headers", .value = "Content-Type"},
	};

	enum http_method method = client->method;

	/* ── OPTIONS (CORS preflight) ── */
	if (method == HTTP_OPTIONS) {
		response_ctx->headers = cors_hdrs;
		response_ctx->header_count = ARRAY_SIZE(cors_hdrs);
		response_ctx->status = HTTP_204_NO_CONTENT;
		response_ctx->final_chunk = true;
		return 0;
	}

	/* ── GET /api/fw_update → return firmware info ── */
	if (method == HTTP_GET) {
		struct fbi_header hdr;
		int len;

		if (read_fbi_header(&hdr)) {
			bool crc_ok = verify_fbi_crc(&hdr);

			len = snprintf(resp_buf, sizeof(resp_buf),
				"{\"jedec_id\":\"0x%06X\",\"fw_size\":%u,"
				"\"fw_crc\":\"0x%08X\",\"crc_ok\":%s}",
				spi_flash_read_jedec_id(),
				hdr.length, hdr.crc32,
				crc_ok ? "true" : "false");
		} else {
			len = snprintf(resp_buf, sizeof(resp_buf),
				"{\"jedec_id\":\"0x%06X\",\"fw_size\":0,"
				"\"fw_crc\":null,\"crc_ok\":false}",
				spi_flash_read_jedec_id());
		}

		response_ctx->body = (const uint8_t *)resp_buf;
		response_ctx->body_len = len;
		response_ctx->headers = json_hdrs;
		response_ctx->header_count = ARRAY_SIZE(json_hdrs);
		response_ctx->status = HTTP_200_OK;
		response_ctx->final_chunk = true;
		return 0;
	}

	/* ── POST /api/fw_update → stream firmware to flash ── */
	if (method != HTTP_POST) {
		response_ctx->status = HTTP_405_METHOD_NOT_ALLOWED;
		response_ctx->final_chunk = true;
		return 0;
	}

	/* Aborted? */
	if (status == HTTP_SERVER_DATA_ABORTED) {
		if (fw_state.active) {
			LOG_WRN("FW: upload aborted after %u bytes",
				fw_state.total_received);
			fw_state.active = false;
		}
		return 0;
	}

	/* First chunk — initialise writer */
	if (!fw_state.active) {
		fw_writer_reset();
		fw_state.active = true;
		LOG_INF("FW: upload started");
	}

	/* Feed incoming data */
	if (request_ctx->data_len > 0 && fw_state.error == 0) {
		/* Bounds check */
		if (fw_state.total_received + request_ctx->data_len > FW_MAX_SIZE) {
			LOG_ERR("FW: image too large (>%u bytes)", FW_MAX_SIZE);
			fw_state.error = -ENOMEM;
		} else {
			int ret = fw_writer_feed(request_ctx->data,
						 request_ctx->data_len);
			if (ret < 0) {
				fw_state.error = ret;
			}
		}
	}

	/* Not final yet — keep receiving */
	if (status != HTTP_SERVER_DATA_FINAL) {
		response_ctx->final_chunk = false;
		return 0;
	}

	/* ── Final chunk — flush and respond ── */
	fw_state.active = false;
	int len;

	if (fw_state.error == 0) {
		fw_state.error = fw_writer_finish();
	}

	if (fw_state.error != 0) {
		len = snprintf(resp_buf, sizeof(resp_buf),
			"{\"status\":\"error\",\"error\":%d,\"bytes\":%u}",
			fw_state.error, fw_state.total_received);
		response_ctx->status = HTTP_500_INTERNAL_SERVER_ERROR;
	} else {
		/* Verify the written image */
		struct fbi_header hdr;
		bool valid = read_fbi_header(&hdr) && verify_fbi_crc(&hdr);

		len = snprintf(resp_buf, sizeof(resp_buf),
			"{\"status\":\"ok\",\"bytes\":%u,\"crc_ok\":%s}",
			fw_state.total_received,
			valid ? "true" : "false");
		response_ctx->status = HTTP_200_OK;
		LOG_INF("FW: upload complete: %u bytes, CRC %s",
			fw_state.total_received, valid ? "OK" : "FAIL");
	}

	response_ctx->body = (const uint8_t *)resp_buf;
	response_ctx->body_len = len;
	response_ctx->headers = json_hdrs;
	response_ctx->header_count = ARRAY_SIZE(json_hdrs);
	response_ctx->final_chunk = true;
	return 0;
}

/* ── Init ────────────────────────────────────────────────────────── */

void fw_update_init(void)
{
	spi_flash_init();

	uint32_t jedec = spi_flash_read_jedec_id();

	LOG_INF("SPI flash JEDEC ID: 0x%06X", jedec);

	struct fbi_header hdr;

	if (read_fbi_header(&hdr)) {
		LOG_INF("Firmware image: %u bytes, CRC-32: 0x%08X", hdr.length, hdr.crc32);
	}
}
