/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: Apache-2.0
 *
 * SD Card configuration storage module.
 *
 * Stores device configuration and stream settings on an SD card
 * as JSON file. Supports loading at boot and saving on changes.
 */

#ifndef SD_CONFIG_H_
#define SD_CONFIG_H_

#include <zephyr/kernel.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Configuration file path on SD card */
#define SD_CONFIG_FILE_PATH  "/SD:/config.json"
#define SD_MOUNT_POINT       "/SD:"

/**
 * @brief Configuration load status.
 */
enum sd_config_load_status {
	SD_CONFIG_NOT_LOADED,      /* Config not yet loaded */
	SD_CONFIG_LOAD_OK,         /* Config loaded successfully */
	SD_CONFIG_LOAD_NO_FILE,    /* No config file found (defaults used) */
	SD_CONFIG_LOAD_NO_CARD,    /* SD card not mounted */
	SD_CONFIG_LOAD_ERROR,      /* Error reading/parsing config */
};

/**
 * @brief Initialize the SD card subsystem and mount the filesystem.
 *
 * This should be called early in main() to mount the SD card.
 * It does NOT load the config - call sd_config_load() separately
 * after the FPGA is ready.
 *
 * @return 0 on success, negative errno on error
 */
int sd_config_init(void);

/**
 * @brief Check if the SD card is mounted and ready.
 *
 * @return true if SD card is mounted
 */
bool sd_config_is_ready(void);

/**
 * @brief Load configuration from SD card.
 *
 * Reads the JSON config file and applies settings to:
 * - Device configuration (aes67_config)
 * - TX streams (via sap_sdp_configure_tx_stream)
 * - RX streams (via sap_sdp_configure_rx_stream)
 *
 * Call this after FPGA is ready so stream configs can be written.
 *
 * @return 0 on success, negative errno on error
 */
int sd_config_load(void);

/**
 * @brief Save current configuration to SD card.
 *
 * Serializes the current device config and all active TX/RX streams
 * to JSON and writes to the config file.
 *
 * @return 0 on success, negative errno on error
 */
int sd_config_save(void);

/**
 * @brief Mark configuration as dirty (needs saving).
 *
 * Call this when any configuration changes. The module may
 * defer the actual save to avoid excessive writes.
 */
void sd_config_mark_dirty(void);

/**
 * @brief Flush any pending configuration changes to SD card.
 *
 * Forces an immediate save if the config is dirty.
 *
 * @return 0 on success, negative errno on error
 */
int sd_config_flush(void);

/**
 * @brief Format the SD card (create fresh FAT filesystem).
 *
 * WARNING: This will erase all data on the SD card!
 *
 * @return 0 on success, negative errno on error
 */
int sd_config_format(void);

/**
 * @brief Get the status of the last config load attempt.
 *
 * @return Load status enum value
 */
enum sd_config_load_status sd_config_get_load_status(void);

/**
 * @brief Get a human-readable string for the load status.
 *
 * @param status  Load status enum value
 * @return String description
 */
const char *sd_config_status_str(enum sd_config_load_status status);

#ifdef __cplusplus
}
#endif

#endif /* SD_CONFIG_H_ */
