/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: Apache-2.0
 *
 * SPI Flash configuration storage module.
 *
 * Stores device configuration in the upper area of the W25Q64 SPI
 * flash using a simple A/B slot scheme with sequence numbers for
 * crash-safe updates and basic wear leveling.
 *
 * Flash layout (at CONFIG_FLASH_CONFIG_OFFSET, default 0x7F0000):
 *   Slot A: offset + 0x0000  (2 sectors = 8 KB)
 *   Slot B: offset + 0x2000  (2 sectors = 8 KB)
 *
 * Each slot contains a 16-byte header followed by JSON payload:
 *   [4] magic  [4] sequence  [4] length  [4] CRC-32  [payload...]
 */

#ifndef FLASH_CONFIG_H_
#define FLASH_CONFIG_H_

#include <zephyr/kernel.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Flash configuration load status.
 */
enum flash_config_load_status {
	FLASH_CONFIG_NOT_LOADED,    /* Config not yet loaded */
	FLASH_CONFIG_LOAD_OK,       /* Config loaded successfully */
	FLASH_CONFIG_LOAD_EMPTY,    /* No valid config found (defaults used) */
	FLASH_CONFIG_LOAD_ERROR,    /* Error reading/parsing config */
};

/**
 * @brief Initialize the flash config subsystem.
 *
 * Verifies flash connectivity (JEDEC ID check).
 *
 * @return 0 on success, negative errno on error
 */
int flash_config_init(void);

/**
 * @brief Load configuration from SPI flash.
 *
 * Reads both A/B slots, picks the one with the highest valid
 * sequence number, parses its JSON payload, and applies it.
 *
 * @return 0 on success, negative errno on error
 */
int flash_config_load(void);

/**
 * @brief Save current configuration to SPI flash.
 *
 * Serializes config to JSON and writes to the slot with the
 * lower sequence number (the older one), then verifies the write.
 *
 * @return 0 on success, negative errno on error
 */
int flash_config_save(void);

/**
 * @brief Mark configuration as dirty (needs saving).
 */
void flash_config_mark_dirty(void);

/**
 * @brief Flush any pending configuration changes to flash.
 *
 * @return 0 on success, negative errno on error
 */
int flash_config_flush(void);

/**
 * @brief Erase both config slots on flash.
 *
 * @return 0 on success, negative errno on error
 */
int flash_config_erase(void);

/**
 * @brief Get the status of the last config load attempt.
 */
enum flash_config_load_status flash_config_get_load_status(void);

/**
 * @brief Get a human-readable string for the load status.
 */
const char *flash_config_status_str(enum flash_config_load_status status);

#ifdef __cplusplus
}
#endif

#endif /* FLASH_CONFIG_H_ */
