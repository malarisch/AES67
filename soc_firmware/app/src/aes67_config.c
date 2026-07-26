/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: Apache-2.0
 *
 * Global runtime configuration – implementation.
 */

#include "aes67_config.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <errno.h>
#include <zephyr/logging/log.h>
#ifdef CONFIG_SD_CONFIG
#include "sd_config.h"
#endif
#ifdef CONFIG_FLASH_CONFIG
#include "flash_config.h"
#endif

LOG_MODULE_REGISTER(aes67_config, LOG_LEVEL_INF);

/* ---- Mutex protecting the config struct ---- */
static struct k_mutex cfg_mutex;
static bool cfg_mutex_inited;

/* ---- The single global config instance ---- */
static struct aes67_device_config g_config;

/* ---- Compiled-in defaults (match previous #defines) ---- */

void aes67_config_reset_defaults(void)
{
	memset(&g_config, 0, sizeof(g_config));

	/* Device identification (RAVENNA format) */
	strncpy(g_config.vendor, "AES67", AES67_VENDOR_MAX - 1);
	strncpy(g_config.product, "AudioNode", AES67_PRODUCT_MAX - 1);
	strncpy(g_config.serial, "0001", AES67_SERIAL_MAX - 1);
	strncpy(g_config.device_name, "AES67 Node",
		AES67_DEVICE_NAME_MAX - 1);

	/* AES67 audio */
	strncpy(g_config.default_mcast_addr, "239.69.0.1",
		sizeof(g_config.default_mcast_addr) - 1);
	g_config.default_port            = 5004;
	g_config.default_channels        = 2;
	g_config.default_bit_depth       = 24;
	g_config.default_sample_rate     = 48000;
	g_config.default_samples_per_pkt = 48;
	g_config.default_payload_type    = 97;

	/* PTP */
	g_config.ptp_domain                = 0;
	g_config.ptp_priority1             = 128;
	g_config.ptp_priority2             = 128;
	g_config.ptp_clock_class           = 248;
	g_config.ptp_clock_accuracy        = 0xFE;
	g_config.ptp_log_sync_interval     = -3;
	g_config.ptp_log_announce_interval = 0;
	g_config.ptp_delay_asymmetry_ns    = 0;

	/* PI controller */
	g_config.pi_kp_num        = 1;
	g_config.pi_kp_den        = 4;
	g_config.pi_ki_num        = 1;
	g_config.pi_ki_den        = 32;
	g_config.pi_imax           = 500000;
	g_config.pi_outlier_ppb    = 50000000;
	g_config.pi_warmup_cycles  = 3;

	/* SAP */
	g_config.sap_announce_interval_s = 30;
	g_config.sap_announce_enabled    = true;

	LOG_INF("Configuration reset to defaults");
}

struct aes67_device_config *aes67_config_get(void)
{
	if (!cfg_mutex_inited) {
		k_mutex_init(&cfg_mutex);
		aes67_config_reset_defaults();
		cfg_mutex_inited = true;
	}
	return &g_config;
}

void aes67_config_lock(void)
{
	if (!cfg_mutex_inited) {
		k_mutex_init(&cfg_mutex);
		aes67_config_reset_defaults();
		cfg_mutex_inited = true;
	}
	k_mutex_lock(&cfg_mutex, K_FOREVER);
}

void aes67_config_unlock(void)
{
	k_mutex_unlock(&cfg_mutex);
}

char *aes67_config_build_node_id(char *buf, size_t buflen)
{
	if (!buf || buflen == 0) {
		return buf;
	}

	snprintf(buf, buflen, "%s %s %s",
		 g_config.vendor, g_config.product, g_config.serial);
	return buf;
}

char *aes67_config_build_hostname(char *buf, size_t buflen)
{
	if (!buf || buflen == 0) {
		return buf;
	}

	if (g_config.device_name[0] != '\0') {
		/* Use friendly name with spaces removed */
		char *dst = buf;
		const char *src = g_config.device_name;
		size_t remaining = buflen - 1;

		while (*src && remaining > 0) {
			if (*src != ' ') {
				*dst++ = tolower((unsigned char)*src);
				remaining--;
			}
			src++;
		}
		*dst = '\0';
	} else {
		/* No friendly name: use "vendor_product_serial" */
		snprintf(buf, buflen, "%s_%s_%s",
			 g_config.vendor, g_config.product, g_config.serial);

		/* Sanitize for DNS: lowercase */
		for (char *p = buf; *p; p++) {
			*p = tolower((unsigned char)*p);
		}
	}
	return buf;
}

void aes67_config_build_mac(uint8_t mac[6])
{
	/* Locally-administered unicast prefix (bit 1 of byte 0 set, bit 0
	 * clear), so no real OUI is needed. This is the same prefix the
	 * Ethernet drivers use as their built-in default, which keeps the
	 * default serial "0001" mapping to the historic 02:AA:E6:70:00:01. */
	static const uint8_t prefix[4] = { 0x02, 0xAA, 0xE6, 0x70 };
	const char *serial = g_config.serial;
	uint32_t id;
	char *end;

	memcpy(mac, prefix, sizeof(prefix));

	/* A plain decimal serial is carried verbatim in the low two bytes so
	 * it stays readable in the MAC. Anything else (empty, non-numeric or
	 * out of range) is folded with FNV-1a, which still gives a stable and
	 * well-distributed value for the same serial. */
	id = (uint32_t)strtoul(serial, &end, 10);
	if (serial[0] == '\0' || *end != '\0' || id > 0xFFFFu) {
		uint32_t h = 2166136261u;

		for (const char *p = serial; *p != '\0'; p++) {
			h = (h ^ (uint8_t)*p) * 16777619u;
		}
		id = (h ^ (h >> 16)) & 0xFFFFu;
	}

	mac[4] = (uint8_t)(id >> 8);
	mac[5] = (uint8_t)id;
}

int aes67_config_persist(void)
{
	int ret = -ENODEV;

#ifdef CONFIG_SD_CONFIG
	{
		int r = sd_config_save();

		if (r == 0) {
			ret = 0;
		} else if (r != -ENODEV) {
			/* -ENODEV just means "no card here"; anything else is
			 * a real write failure worth reporting. */
			LOG_WRN("Config save to SD failed: %d", r);
		}
	}
#endif

#ifdef CONFIG_FLASH_CONFIG
	{
		int r = flash_config_save();

		if (r == 0) {
			ret = 0;
		} else {
			LOG_WRN("Config save to flash failed: %d", r);
		}
	}
#endif

	if (ret != 0) {
		LOG_ERR("Configuration not persisted — no storage backend "
			"accepted it; changes will be lost on reboot");
	}

	return ret;
}
