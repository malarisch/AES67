/*
 * Shared JSON serialization / parsing for device configuration.
 *
 * Used by both sd_config (SD card) and flash_config (SPI flash) to
 * convert the runtime aes67_device_config + stream tables to/from
 * a JSON text representation.
 */

#ifndef CONFIG_JSON_H_
#define CONFIG_JSON_H_

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Serialize the current device config + streams to JSON.
 *
 * @param buf   Destination buffer.
 * @param sz    Size of buffer in bytes.
 * @return Number of bytes written (excluding NUL), or a negative errno
 *         (-ENOMEM if the buffer is too small; buf is left empty).
 */
int config_json_serialize(char *buf, size_t sz);

/**
 * @brief Parse a JSON document and apply device + stream configuration.
 *
 * Keys the document does not carry keep their current value, so a
 * partial document is a valid patch. Calls aes67_config,
 * aes67_conn_configure_tx_stream and aes67_conn_configure_rx_stream as
 * appropriate.
 *
 * @param json  NUL-terminated JSON document. Must be WRITABLE: the
 *              parser NUL-terminates tokens in place while decoding
 *              them (it restores the bytes afterwards), so string
 *              literals and read-only buffers are not accepted.
 * @return 0 on success, or a negative errno if the document is
 *         malformed — in which case nothing at all is applied.
 */
int config_json_parse_and_apply(char *json);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_JSON_H_ */
