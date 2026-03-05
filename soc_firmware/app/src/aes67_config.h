/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: Apache-2.0
 *
 * Global runtime configuration for the AES67 device.
 *
 * Replaces scattered compile-time constants with a single mutable
 * configuration object.  Default values match the previous #defines
 * so behaviour is identical until changed via REST API / shell.
 */

#ifndef AES67_CONFIG_H_
#define AES67_CONFIG_H_

#include <zephyr/kernel.h>
#include <zephyr/net/net_ip.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Device identification ---- */

#define AES67_DEVICE_NAME_MAX   32
#define AES67_VENDOR_MAX        16
#define AES67_PRODUCT_MAX       24
#define AES67_SERIAL_MAX        16
#define AES67_NODE_ID_MAX       64  /* "vendor product serial" */

/* ---- Configuration structure ---- */

struct aes67_device_config {
	/* -- Device identification (RAVENNA format) -- */
	char vendor[AES67_VENDOR_MAX];     /* Vendor name */
	char product[AES67_PRODUCT_MAX];   /* Product name */
	char serial[AES67_SERIAL_MAX];     /* Serial number (unique) */
	char device_name[AES67_DEVICE_NAME_MAX];  /* User-defined friendly name */

	/* -- AES67 audio defaults -- */
	char     default_mcast_addr[16];   /* e.g. "239.69.0.1" */
	uint16_t default_port;             /* RTP port, default 5004 */
	uint8_t  default_channels;
	uint8_t  default_bit_depth;
	uint32_t default_sample_rate;
	uint16_t default_samples_per_pkt;
	uint8_t  default_payload_type;

	/* -- PTP -- */
	uint8_t  ptp_domain;
	uint8_t  ptp_priority1;
	uint8_t  ptp_priority2;
	int8_t   ptp_log_sync_interval;       /* AES67: -3  (125 ms) */
	int8_t   ptp_log_announce_interval;    /* AES67:  0  (1 s)   */

	/* -- PLL / PI controller -- */
	int32_t  pi_kp_num;
	int32_t  pi_kp_den;
	int32_t  pi_ki_num;
	int32_t  pi_ki_den;
	int32_t  pi_imax;          /* Anti-windup limit (ppb) */
	int32_t  pi_outlier_ppb;   /* Outlier rejection threshold */
	uint32_t pi_warmup_cycles;

	/* -- SAP -- */
	uint32_t sap_announce_interval_s;
	bool     sap_announce_enabled;
};

/**
 * @brief Get a pointer to the global (mutable) configuration.
 *
 * The returned pointer is valid for the lifetime of the process.
 * Callers MUST hold the config mutex while reading multi-byte
 * fields if they could race with a writer.
 */
struct aes67_device_config *aes67_config_get(void);

/**
 * @brief Lock the configuration mutex before reading / writing.
 */
void aes67_config_lock(void);

/**
 * @brief Unlock the configuration mutex after reading / writing.
 */
void aes67_config_unlock(void);

/**
 * @brief Reset all configuration values to compiled-in defaults.
 */
void aes67_config_reset_defaults(void);

/**
 * @brief Build the RAVENNA-format node ID string.
 *
 * Generates "<vendor> <product> <serial>" from the current config.
 * Caller must provide a buffer of at least AES67_NODE_ID_MAX bytes.
 *
 * @param buf    Output buffer
 * @param buflen Buffer size
 * @return Pointer to buf (for chaining)
 */
char *aes67_config_build_node_id(char *buf, size_t buflen);

/**
 * @brief Get the hostname (vendor-product-serial, sanitized for DNS).
 *
 * Replaces spaces with dashes and lowercases for DNS compatibility.
 * Caller must provide a buffer of at least AES67_NODE_ID_MAX bytes.
 *
 * @param buf    Output buffer
 * @param buflen Buffer size
 * @return Pointer to buf (for chaining)
 */
char *aes67_config_build_hostname(char *buf, size_t buflen);

#ifdef __cplusplus
}
#endif

#endif /* AES67_CONFIG_H_ */
