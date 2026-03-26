/*
 * Firmware update module.
 *
 * Provides:
 *  - Shell commands for flash inspection and firmware upload.
 *  - HTTP handler for /api/fw_update (called from webserver api_handler).
 *
 * The firmware image is the raw .fbi file (8-byte header + payload)
 * written to SPI flash at SPI_FLASH_FW_OFFSET (0x10000).
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef FW_UPDATE_H_
#define FW_UPDATE_H_

#include <zephyr/net/http/server.h>

/**
 * @brief Initialise the firmware update subsystem (SPI flash + shell).
 */
void fw_update_init(void);

/**
 * @brief HTTP handler for /api/fw_update (GET + POST).
 *
 * Must be called directly from the api_handler for streaming POST support
 * (the generic api_handler accumulates the body in a small buffer, which
 * is not suitable for firmware images).
 */
int fw_update_http_handler(struct http_client_ctx *client,
			   enum http_data_status status,
			   const struct http_request_ctx *request_ctx,
			   struct http_response_ctx *response_ctx);

#endif /* FW_UPDATE_H_ */
