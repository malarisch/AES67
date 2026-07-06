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

/* Forward declarations for FBI helpers (used by writer thread) */
struct fbi_header {
	uint32_t length;
	uint32_t crc32;
};
static bool read_fbi_header(struct fbi_header *hdr);
static bool verify_fbi_crc(const struct fbi_header *hdr);

/* ── Async flash writer ──────────────────────────────────────────
 *
 * The HTTP handler copies incoming chunks into a ring buffer; a
 * separate low-priority thread drains the ring buffer to SPI flash.
 * This keeps flash sector-erase latency (~150 ms) off the HTTP thread
 * so the TCP connection stays alive.
 * ─────────────────────────────────────────────────────────────────── */

/* Ring buffer: must be a power-of-two and large enough to absorb
 * at least one HTTP receive window while a sector erase runs. */
#define FW_RING_SIZE  8192
#define FW_RING_MASK  (FW_RING_SIZE - 1)

BUILD_ASSERT((FW_RING_SIZE & FW_RING_MASK) == 0,
	     "FW_RING_SIZE must be power of two");

static uint8_t  fw_ring[FW_RING_SIZE];
static volatile uint32_t fw_ring_head;    /* written by HTTP thread */
static volatile uint32_t fw_ring_tail;    /* read by writer thread */

/* Writer thread */
#define FW_WRITER_STACK_SIZE 2048
static K_THREAD_STACK_DEFINE(fw_writer_stack, FW_WRITER_STACK_SIZE);
static struct k_thread fw_writer_thread;
static struct k_sem    fw_data_sem;       /* signalled when data available */

enum fw_phase {
	FW_IDLE,
	FW_RECEIVING,     /* HTTP handler is feeding data */
	FW_RX_COMPLETE,   /* all HTTP data received, writer still draining */
	FW_DONE,          /* writer finished (success or error) */
};

struct fw_writer {
	uint32_t flash_addr;     /* next write address in flash */
	uint32_t total_received; /* bytes fed by HTTP handler */
	uint32_t total_written;  /* bytes flushed to flash */
	uint32_t erased_up_to;   /* sectors erased up to this address */
	uint8_t  page_buf[SPI_FLASH_PAGE_SIZE];
	size_t   page_buf_len;   /* bytes buffered in page_buf */
	volatile enum fw_phase phase;
	int      error;          /* non-zero on failure */
	bool     crc_ok;         /* set by writer thread after verification */
};

static struct fw_writer fw_state;

static void fw_writer_reset(void)
{
	LOG_DBG("Reset fw writer");
	memset(&fw_state, 0, sizeof(fw_state));
	fw_state.flash_addr = FW_FLASH_ADDR;
	fw_state.erased_up_to = FW_FLASH_ADDR;
	fw_state.phase = FW_IDLE;
	fw_ring_head = 0;
	fw_ring_tail = 0;
}

/**
 * Ensure sectors are erased up to (but not including) `end_addr`.
 */
static void fw_ensure_erased(uint32_t end_addr)
{
	LOG_DBG("Started Erase");
	while (fw_state.erased_up_to < end_addr) {
		spi_flash_sector_erase(fw_state.erased_up_to);
		fw_state.erased_up_to += SPI_FLASH_SECTOR_SIZE;
	}
	LOG_DBG("Erase don");
}

/**
 * Flush any remaining data in the page buffer to flash.
 */
static int fw_flush_page(void)
{
	LOG_DBG("Started flush page");
	if (fw_state.page_buf_len == 0) {
		LOG_WRN("Page buf leng not 0");
		return 0;
	}

	fw_ensure_erased(fw_state.flash_addr + fw_state.page_buf_len);
	LOG_DBG("Ensure Erase Done");
	spi_flash_page_program(fw_state.flash_addr, fw_state.page_buf,
			       fw_state.page_buf_len);
	LOG_DBG("Flash Page Program done");
	/* Verify */
	uint8_t verify[SPI_FLASH_PAGE_SIZE];

	spi_flash_read(fw_state.flash_addr, verify, fw_state.page_buf_len);
	LOG_DBG("Started Verify Read");
	if (memcmp(verify, fw_state.page_buf, fw_state.page_buf_len) != 0) {
		LOG_ERR("FW: verify failed at 0x%08x", fw_state.flash_addr);
		return -EIO;
	}
	LOG_DBG("Verify Read Done");

	fw_state.flash_addr += fw_state.page_buf_len;
	fw_state.total_written += fw_state.page_buf_len;
	fw_state.page_buf_len = 0;
	return 0;
}

/**
 * Ring buffer: number of bytes available to read.
 */
static inline uint32_t fw_ring_avail(void)
{
	return fw_ring_head - fw_ring_tail;
}

/**
 * Ring buffer: free space for writing.
 */
static inline uint32_t fw_ring_free(void)
{
	return FW_RING_SIZE - fw_ring_avail();
}

/**
 * HTTP handler calls this to push data into the ring buffer.
 * Returns 0 on success, -ENOMEM if the ring buffer overflows.
 */
static int fw_ring_push(const uint8_t *data, size_t len)
{
	LOG_DBG("FW ring push, length: %d", len);
	if (len > fw_ring_free()) {
		return -ENOMEM;
	}

	for (size_t i = 0; i < len; i++) {
		fw_ring[fw_ring_head & FW_RING_MASK] = data[i];
		fw_ring_head++;
	}

	/* Wake writer thread */
	k_sem_give(&fw_data_sem);
	return 0;
}

/**
 * Writer thread: drain ring buffer into the page buffer and flash.
 */
static void fw_writer_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		/* Wait for data or completion signal */
		k_sem_take(&fw_data_sem, K_FOREVER);

		if (fw_state.phase == FW_IDLE) {
			continue;
		}

		/* Drain all available data from ring buffer */
		while (fw_ring_avail() > 0 && fw_state.error == 0) {
			size_t space = SPI_FLASH_PAGE_SIZE - fw_state.page_buf_len;
			size_t avail = fw_ring_avail();
			size_t n = (avail < space) ? avail : space;

			for (size_t i = 0; i < n; i++) {
				fw_state.page_buf[fw_state.page_buf_len++] =
					fw_ring[fw_ring_tail & FW_RING_MASK];
				fw_ring_tail++;
			}

			if (fw_state.page_buf_len == SPI_FLASH_PAGE_SIZE) {
				int ret = fw_flush_page();
				if (ret < 0) {
					fw_state.error = ret;
				}
			}
		}

		/* If HTTP reception is complete and ring is drained, finalize */
		if (fw_state.phase == FW_RX_COMPLETE && fw_ring_avail() == 0 &&
		    fw_state.error == 0) {
			fw_state.error = fw_flush_page();
			if (fw_state.error == 0) {
				/* Verify the written image */
				struct fbi_header hdr;

				fw_state.crc_ok = read_fbi_header(&hdr) &&
						  verify_fbi_crc(&hdr);
				LOG_INF("FW: writer done, %u bytes, CRC %s",
					fw_state.total_written,
					fw_state.crc_ok ? "OK" : "FAIL");
			} else {
				LOG_ERR("FW: final flush failed (%d)",
					fw_state.error);
			}
			fw_state.phase = FW_DONE;
		} else if (fw_state.error != 0 &&
			   fw_state.phase != FW_DONE) {
			fw_state.phase = FW_DONE;
		}
	}
}

/* ── FBI header helpers ──────────────────────────────────────────── */

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
			   enum http_transaction_status status,
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

	LOG_DBG("cb enter: method=%d status=%d data_len=%zu phase=%d",
		method, status, request_ctx->data_len, fw_state.phase);

	if (status == HTTP_SERVER_TRANSACTION_COMPLETE) {
		/* Response fully sent — must not restart the writer. */
		return 0;
	}

	/* ── OPTIONS (CORS preflight) ── */
	if (method == HTTP_OPTIONS) {
		LOG_DBG("OPTIONS preflight");
		response_ctx->headers = cors_hdrs;
		response_ctx->header_count = ARRAY_SIZE(cors_hdrs);
		response_ctx->status = HTTP_204_NO_CONTENT;
		response_ctx->final_chunk = true;
		return 0;
	}

	/* ── GET /api/fw_update → return firmware info + writer status ── */
	if (method == HTTP_GET) {
		int len;

		if (fw_state.phase == FW_RECEIVING ||
		    fw_state.phase == FW_RX_COMPLETE) {
			len = snprintf(resp_buf, sizeof(resp_buf),
				"{\"phase\":\"writing\","
				"\"received\":%u,\"written\":%u}",
				fw_state.total_received,
				fw_state.total_written);
		} else if (fw_state.phase == FW_DONE) {
			if (fw_state.error != 0) {
				len = snprintf(resp_buf, sizeof(resp_buf),
					"{\"phase\":\"error\","
					"\"error\":%d,\"written\":%u}",
					fw_state.error, fw_state.total_written);
			} else {
				len = snprintf(resp_buf, sizeof(resp_buf),
					"{\"phase\":\"done\","
					"\"bytes\":%u,\"crc_ok\":%s}",
					fw_state.total_written,
					fw_state.crc_ok ? "true" : "false");
			}
		} else {
			/* FW_IDLE — show stored firmware info */
			struct fbi_header hdr;

			if (read_fbi_header(&hdr)) {
				len = snprintf(resp_buf, sizeof(resp_buf),
					"{\"phase\":\"idle\","
					"\"jedec_id\":\"0x%06X\","
					"\"fw_size\":%u,"
					"\"fw_crc\":\"0x%08X\"}",
					spi_flash_read_jedec_id(),
					hdr.length, hdr.crc32);
			} else {
				len = snprintf(resp_buf, sizeof(resp_buf),
					"{\"phase\":\"idle\","
					"\"jedec_id\":\"0x%06X\","
					"\"fw_size\":0}",
					spi_flash_read_jedec_id());
			}
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
		LOG_DBG("not POST, method=%d -> 405", method);
		response_ctx->status = HTTP_405_METHOD_NOT_ALLOWED;
		response_ctx->final_chunk = true;
		return 0;
	}

	LOG_DBG("POST: status=%d data_len=%zu data_ptr=%p phase=%d total_rx=%u",
		status, request_ctx->data_len, (void *)request_ctx->data,
		fw_state.phase, fw_state.total_received);

	/* Aborted? */
	if (status == HTTP_SERVER_TRANSACTION_ABORTED) {
		LOG_WRN("FW: ABORTED status=%d phase=%d total_rx=%u",
			status, fw_state.phase, fw_state.total_received);
		if (fw_state.phase == FW_RECEIVING ||
		    fw_state.phase == FW_RX_COMPLETE) {
			LOG_WRN("FW: upload aborted after %u bytes",
				fw_state.total_received);
			fw_state.error = -ECANCELED;
			fw_state.phase = FW_IDLE;
		}
		return 0;
	}

	/* First chunk — initialise writer */
	if (fw_state.phase == FW_IDLE || fw_state.phase == FW_DONE) {
		LOG_DBG("init writer: phase was %d", fw_state.phase);
		fw_writer_reset();
		fw_state.phase = FW_RECEIVING;
		LOG_INF("FW: upload started");
	}

	/* Push incoming data into ring buffer (non-blocking) */
	if (request_ctx->data_len > 0 && fw_state.error == 0) {
		LOG_DBG("push %zu bytes, total_rx_before=%u, ring_free=%u",
			request_ctx->data_len, fw_state.total_received,
			fw_ring_free());
		/* Bounds check */
		if (fw_state.total_received + request_ctx->data_len > FW_MAX_SIZE) {
			LOG_ERR("FW: image too large (>%u bytes)", FW_MAX_SIZE);
			fw_state.error = -ENOMEM;
		} else {
			/* If ring buffer is full, yield briefly to let
			 * writer thread drain it. */
			int retries = 200;

			while (fw_ring_free() < request_ctx->data_len &&
			       --retries > 0) {
				LOG_DBG("ring full, retry %d, free=%u need=%zu",
					200 - retries, fw_ring_free(),
					request_ctx->data_len);
				k_msleep(5);
			}

			int ret = fw_ring_push(request_ctx->data,
					       request_ctx->data_len);
			if (ret < 0) {
				LOG_ERR("FW: ring buffer overflow (free=%u need=%zu)",
					fw_ring_free(), request_ctx->data_len);
				fw_state.error = ret;
			} else {
				fw_state.total_received += request_ctx->data_len;
				LOG_DBG("pushed OK, total_rx=%u",
					fw_state.total_received);
			}
		}
	} else {
		LOG_DBG("no data to push: data_len=%zu error=%d",
			request_ctx->data_len, fw_state.error);
	}

	/* Not final yet — keep receiving */
	if (status != HTTP_SERVER_REQUEST_DATA_FINAL) {
		LOG_DBG("not final, returning final_chunk=false");
		response_ctx->final_chunk = false;
		return 0;
	}

	LOG_DBG("FINAL chunk, total_rx=%u, signalling writer",
		fw_state.total_received);

	/* ── Final chunk received — signal writer, respond immediately ── */
	fw_state.phase = FW_RX_COMPLETE;
	k_sem_give(&fw_data_sem);

	int len;

	if (fw_state.error != 0) {
		len = snprintf(resp_buf, sizeof(resp_buf),
			"{\"status\":\"error\",\"error\":%d,\"bytes\":%u}",
			fw_state.error, fw_state.total_received);
		response_ctx->status = HTTP_500_INTERNAL_SERVER_ERROR;
	} else {
		/* Writer still draining ring buffer → tell client to poll */
		len = snprintf(resp_buf, sizeof(resp_buf),
			"{\"status\":\"writing\",\"bytes\":%u}",
			fw_state.total_received);
		response_ctx->status = HTTP_200_OK;
		LOG_INF("FW: upload received: %u bytes, writing to flash",
			fw_state.total_received);
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

	k_sem_init(&fw_data_sem, 0, 1);

	k_thread_create(&fw_writer_thread, fw_writer_stack,
			 K_THREAD_STACK_SIZEOF(fw_writer_stack),
			 fw_writer_thread_fn, NULL, NULL, NULL,
			 K_PRIO_PREEMPT(12), 0, K_NO_WAIT);
	k_thread_name_set(&fw_writer_thread, "fw_writer");
}
