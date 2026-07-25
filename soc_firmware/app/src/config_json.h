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
 * @return Number of bytes written (excluding NUL), or negative on overflow.
 */
int config_json_serialize(char *buf, size_t sz);

/**
 * @brief Parse a JSON string and apply device + stream configuration.
 *
 * Calls aes67_config, aes67_conn_configure_tx_stream and
 * aes67_conn_configure_rx_stream as appropriate.
 *
 * @param json  NUL-terminated JSON string.
 * @return 0 on success, negative errno on error.
 */
int config_json_parse_and_apply(const char *json);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_JSON_H_ */
