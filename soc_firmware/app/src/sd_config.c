/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: Apache-2.0
 *
 * SD Card configuration storage implementation.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/fs/fs.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/socket.h>
#include <ff.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "sd_config.h"
#include "aes67_config.h"
#include "sap_sdp.h"

LOG_MODULE_REGISTER(sd_config, LOG_LEVEL_INF);

/* ---- Filesystem state ---- */
static FATFS fat_fs;
static bool sd_mounted;
static bool config_dirty;
static struct k_mutex sd_mutex;
static enum sd_config_load_status load_status = SD_CONFIG_NOT_LOADED;

static struct fs_mount_t sd_mount = {
	.type = FS_FATFS,
	.fs_data = &fat_fs,
	.mnt_point = SD_MOUNT_POINT,
};

/* ---- JSON buffer for serialization ---- */
#define JSON_BUF_SIZE  8192
static char json_buf[JSON_BUF_SIZE];

/* ================================================================
 * Simple JSON serialization helpers
 * ================================================================ */

static int json_start(char *buf, size_t sz)
{
	return snprintf(buf, sz, "{\n");
}

static int json_end(char *buf, size_t sz, int pos)
{
	/* Remove trailing comma+newline */
	if (pos > 2 && buf[pos - 2] == ',') {
		pos -= 2;
		buf[pos++] = '\n';
	}
	pos += snprintf(buf + pos, sz - pos, "}\n");
	return pos;
}

static int json_str(char *buf, size_t sz, int pos, const char *key,
		    const char *val, int indent)
{
	for (int i = 0; i < indent; i++) {
		pos += snprintf(buf + pos, sz - pos, "  ");
	}
	pos += snprintf(buf + pos, sz - pos, "\"%s\": \"%s\",\n", key, val);
	return pos;
}

static int json_int(char *buf, size_t sz, int pos, const char *key,
		    int32_t val, int indent)
{
	for (int i = 0; i < indent; i++) {
		pos += snprintf(buf + pos, sz - pos, "  ");
	}
	pos += snprintf(buf + pos, sz - pos, "\"%s\": %d,\n", key, val);
	return pos;
}

static int json_uint(char *buf, size_t sz, int pos, const char *key,
		     uint32_t val, int indent)
{
	for (int i = 0; i < indent; i++) {
		pos += snprintf(buf + pos, sz - pos, "  ");
	}
	pos += snprintf(buf + pos, sz - pos, "\"%s\": %u,\n", key, val);
	return pos;
}

static int json_bool(char *buf, size_t sz, int pos, const char *key,
		     bool val, int indent)
{
	for (int i = 0; i < indent; i++) {
		pos += snprintf(buf + pos, sz - pos, "  ");
	}
	pos += snprintf(buf + pos, sz - pos, "\"%s\": %s,\n",
			key, val ? "true" : "false");
	return pos;
}

static int json_array_start(char *buf, size_t sz, int pos,
			    const char *key, int indent)
{
	for (int i = 0; i < indent; i++) {
		pos += snprintf(buf + pos, sz - pos, "  ");
	}
	pos += snprintf(buf + pos, sz - pos, "\"%s\": [\n", key);
	return pos;
}

static int json_array_end(char *buf, size_t sz, int pos, int indent)
{
	/* Remove trailing comma+newline from last element */
	if (pos > 2 && buf[pos - 2] == ',') {
		pos -= 2;
		buf[pos++] = '\n';
	}
	for (int i = 0; i < indent; i++) {
		pos += snprintf(buf + pos, sz - pos, "  ");
	}
	pos += snprintf(buf + pos, sz - pos, "],\n");
	return pos;
}

static int json_obj_start(char *buf, size_t sz, int pos, int indent)
{
	for (int i = 0; i < indent; i++) {
		pos += snprintf(buf + pos, sz - pos, "  ");
	}
	pos += snprintf(buf + pos, sz - pos, "{\n");
	return pos;
}

static int json_obj_end(char *buf, size_t sz, int pos, int indent)
{
	/* Remove trailing comma+newline */
	if (pos > 2 && buf[pos - 2] == ',') {
		pos -= 2;
		buf[pos++] = '\n';
	}
	for (int i = 0; i < indent; i++) {
		pos += snprintf(buf + pos, sz - pos, "  ");
	}
	pos += snprintf(buf + pos, sz - pos, "},\n");
	return pos;
}

/* ================================================================
 * Serialize device configuration to JSON
 * ================================================================ */
static int serialize_device_config(char *buf, size_t sz, int pos)
{
	struct aes67_device_config *cfg = aes67_config_get();

	aes67_config_lock();

	pos = json_str(buf, sz, pos, "device_name", cfg->device_name, 1);
	pos = json_str(buf, sz, pos, "default_mcast_addr",
		       cfg->default_mcast_addr, 1);
	pos = json_uint(buf, sz, pos, "default_port", cfg->default_port, 1);
	pos = json_uint(buf, sz, pos, "default_channels",
			cfg->default_channels, 1);
	pos = json_uint(buf, sz, pos, "default_bit_depth",
			cfg->default_bit_depth, 1);
	pos = json_uint(buf, sz, pos, "default_sample_rate",
			cfg->default_sample_rate, 1);
	pos = json_uint(buf, sz, pos, "default_samples_per_pkt",
			cfg->default_samples_per_pkt, 1);
	pos = json_uint(buf, sz, pos, "default_payload_type",
			cfg->default_payload_type, 1);
	pos = json_uint(buf, sz, pos, "ptp_domain", cfg->ptp_domain, 1);
	pos = json_uint(buf, sz, pos, "ptp_priority1", cfg->ptp_priority1, 1);
	pos = json_uint(buf, sz, pos, "ptp_priority2", cfg->ptp_priority2, 1);
	pos = json_int(buf, sz, pos, "ptp_log_sync_interval",
		       cfg->ptp_log_sync_interval, 1);
	pos = json_int(buf, sz, pos, "ptp_log_announce_interval",
		       cfg->ptp_log_announce_interval, 1);
	pos = json_int(buf, sz, pos, "pi_kp_num", cfg->pi_kp_num, 1);
	pos = json_int(buf, sz, pos, "pi_kp_den", cfg->pi_kp_den, 1);
	pos = json_int(buf, sz, pos, "pi_ki_num", cfg->pi_ki_num, 1);
	pos = json_int(buf, sz, pos, "pi_ki_den", cfg->pi_ki_den, 1);
	pos = json_int(buf, sz, pos, "pi_imax", cfg->pi_imax, 1);
	pos = json_int(buf, sz, pos, "pi_outlier_ppb", cfg->pi_outlier_ppb, 1);
	pos = json_uint(buf, sz, pos, "pi_warmup_cycles",
			cfg->pi_warmup_cycles, 1);
	pos = json_uint(buf, sz, pos, "sap_announce_interval_s",
			cfg->sap_announce_interval_s, 1);
	pos = json_bool(buf, sz, pos, "sap_announce_enabled",
			cfg->sap_announce_enabled, 1);

	aes67_config_unlock();
	return pos;
}

/* ================================================================
 * Serialize TX streams to JSON
 * ================================================================ */
static int serialize_tx_streams(char *buf, size_t sz, int pos)
{
	const struct aes67_tx_stream *streams = sap_sdp_get_tx_streams();
	char ip_str[INET_ADDRSTRLEN];

	pos = json_array_start(buf, sz, pos, "tx_streams", 1);

	for (int i = 0; i < AES67_MAX_TX_STREAMS; i++) {
		if (!streams[i].active) {
			continue;
		}

		pos = json_obj_start(buf, sz, pos, 2);
		pos = json_uint(buf, sz, pos, "stream_id",
				streams[i].stream_id, 3);

		zsock_inet_ntop(AF_INET, &streams[i].dst_ip,
				ip_str, sizeof(ip_str));
		pos = json_str(buf, sz, pos, "dst_ip", ip_str, 3);
		pos = json_uint(buf, sz, pos, "channel_count",
				streams[i].channel_count, 3);
		pos = json_uint(buf, sz, pos, "samples_per_packet",
				streams[i].samples_per_packet, 3);
		pos = json_uint(buf, sz, pos, "ssrc", streams[i].ssrc, 3);

		/* Channel IDs array */
		pos = json_array_start(buf, sz, pos, "ch_ids", 3);
		for (int j = 0; j < streams[i].channel_count &&
				j < AES67_MAX_CH_PER_STREAM; j++) {
			for (int k = 0; k < 4; k++) {
				pos += snprintf(buf + pos, sz - pos, "  ");
			}
			pos += snprintf(buf + pos, sz - pos, "%u,\n",
					streams[i].ch_ids[j]);
		}
		pos = json_array_end(buf, sz, pos, 3);

		pos = json_obj_end(buf, sz, pos, 2);
	}

	pos = json_array_end(buf, sz, pos, 1);
	return pos;
}

/* ================================================================
 * Serialize RX streams to JSON
 * ================================================================ */
static int serialize_rx_streams(char *buf, size_t sz, int pos)
{
	const struct aes67_rx_stream *streams = sap_sdp_get_rx_streams();
	char ip_str[INET_ADDRSTRLEN];

	pos = json_array_start(buf, sz, pos, "rx_streams", 1);

	for (int i = 0; i < AES67_MAX_RX_STREAMS; i++) {
		if (!streams[i].active) {
			continue;
		}

		pos = json_obj_start(buf, sz, pos, 2);
		pos = json_uint(buf, sz, pos, "stream_id",
				streams[i].stream_id, 3);

		zsock_inet_ntop(AF_INET, &streams[i].dst_ip,
				ip_str, sizeof(ip_str));
		pos = json_str(buf, sz, pos, "dst_ip", ip_str, 3);
		pos = json_uint(buf, sz, pos, "dst_port",
				streams[i].dst_port, 3);
		pos = json_uint(buf, sz, pos, "channel_count",
				streams[i].channel_count, 3);
		pos = json_uint(buf, sz, pos, "output_delay",
				streams[i].output_delay, 3);
		pos = json_uint(buf, sz, pos, "samples_per_channel",
				streams[i].samples_per_channel, 3);

		/* Channel map array */
		pos = json_array_start(buf, sz, pos, "ch_map", 3);
		for (int j = 0; j < streams[i].channel_count &&
				j < AES67_MAX_CH_PER_STREAM; j++) {
			for (int k = 0; k < 4; k++) {
				pos += snprintf(buf + pos, sz - pos, "  ");
			}
			pos += snprintf(buf + pos, sz - pos, "%u,\n",
					streams[i].ch_map[j]);
		}
		pos = json_array_end(buf, sz, pos, 3);

		pos = json_obj_end(buf, sz, pos, 2);
	}

	pos = json_array_end(buf, sz, pos, 1);
	return pos;
}

/* ================================================================
 * Full config serialization
 * ================================================================ */
static int serialize_config(char *buf, size_t sz)
{
	int pos = 0;

	pos = json_start(buf, sz);
	pos = serialize_device_config(buf, sz, pos);
	pos = serialize_tx_streams(buf, sz, pos);
	pos = serialize_rx_streams(buf, sz, pos);
	pos = json_end(buf, sz, pos);

	return pos;
}

/* ================================================================
 * Simple JSON parser helpers
 * ================================================================ */

/* Skip whitespace */
static const char *skip_ws(const char *p)
{
	while (*p && (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r')) {
		p++;
	}
	return p;
}

/* Find value for key in JSON object (very simple, not recursive) */
static const char *json_find_key(const char *json, const char *key)
{
	char pattern[64];
	snprintf(pattern, sizeof(pattern), "\"%s\"", key);

	const char *p = strstr(json, pattern);
	if (!p) {
		return NULL;
	}
	p += strlen(pattern);
	p = skip_ws(p);
	if (*p != ':') {
		return NULL;
	}
	p++;
	return skip_ws(p);
}

/* Parse string value (returns pointer to static buffer) */
static char *json_parse_str(const char *p)
{
	static char str_buf[128];
	if (!p || *p != '"') {
		return NULL;
	}
	p++;
	int i = 0;
	while (*p && *p != '"' && i < (int)sizeof(str_buf) - 1) {
		str_buf[i++] = *p++;
	}
	str_buf[i] = '\0';
	return str_buf;
}

/* Parse integer value */
static int32_t json_parse_int(const char *p)
{
	if (!p) {
		return 0;
	}
	return (int32_t)strtol(p, NULL, 10);
}

/* Parse unsigned integer value */
static uint32_t json_parse_uint(const char *p)
{
	if (!p) {
		return 0;
	}
	return (uint32_t)strtoul(p, NULL, 10);
}

/* Parse boolean value */
static bool json_parse_bool(const char *p)
{
	if (!p) {
		return false;
	}
	return (strncmp(p, "true", 4) == 0);
}

/* Find array by key, returns pointer to '[' */
static const char *json_find_array(const char *json, const char *key)
{
	const char *p = json_find_key(json, key);
	if (!p || *p != '[') {
		return NULL;
	}
	return p;
}

/* Find next object in array */
static const char *json_next_obj(const char *p)
{
	if (!p) {
		return NULL;
	}
	p = skip_ws(p);
	if (*p == '[') {
		p++;
	}
	if (*p == ',') {
		p++;
	}
	p = skip_ws(p);
	if (*p != '{') {
		return NULL;
	}
	return p;
}

/* Find end of current object */
static const char *json_obj_end_ptr(const char *p)
{
	if (!p || *p != '{') {
		return NULL;
	}
	int depth = 1;
	p++;
	while (*p && depth > 0) {
		if (*p == '{') depth++;
		else if (*p == '}') depth--;
		p++;
	}
	return p;
}

/* ================================================================
 * Parse and apply device configuration
 * ================================================================ */
static int parse_device_config(const char *json)
{
	struct aes67_device_config *cfg = aes67_config_get();
	const char *v;
	char *s;

	aes67_config_lock();

	if ((v = json_find_key(json, "device_name")) != NULL) {
		s = json_parse_str(v);
		if (s) {
			strncpy(cfg->device_name, s,
				AES67_DEVICE_NAME_MAX - 1);
		}
	}

	if ((v = json_find_key(json, "default_mcast_addr")) != NULL) {
		s = json_parse_str(v);
		if (s) {
			strncpy(cfg->default_mcast_addr, s,
				sizeof(cfg->default_mcast_addr) - 1);
		}
	}

	if ((v = json_find_key(json, "default_port")) != NULL) {
		cfg->default_port = (uint16_t)json_parse_uint(v);
	}
	if ((v = json_find_key(json, "default_channels")) != NULL) {
		cfg->default_channels = (uint8_t)json_parse_uint(v);
	}
	if ((v = json_find_key(json, "default_bit_depth")) != NULL) {
		cfg->default_bit_depth = (uint8_t)json_parse_uint(v);
	}
	if ((v = json_find_key(json, "default_sample_rate")) != NULL) {
		cfg->default_sample_rate = json_parse_uint(v);
	}
	if ((v = json_find_key(json, "default_samples_per_pkt")) != NULL) {
		cfg->default_samples_per_pkt = (uint16_t)json_parse_uint(v);
	}
	if ((v = json_find_key(json, "default_payload_type")) != NULL) {
		cfg->default_payload_type = (uint8_t)json_parse_uint(v);
	}
	if ((v = json_find_key(json, "ptp_domain")) != NULL) {
		cfg->ptp_domain = (uint8_t)json_parse_uint(v);
	}
	if ((v = json_find_key(json, "ptp_priority1")) != NULL) {
		cfg->ptp_priority1 = (uint8_t)json_parse_uint(v);
	}
	if ((v = json_find_key(json, "ptp_priority2")) != NULL) {
		cfg->ptp_priority2 = (uint8_t)json_parse_uint(v);
	}
	if ((v = json_find_key(json, "ptp_log_sync_interval")) != NULL) {
		cfg->ptp_log_sync_interval = (int8_t)json_parse_int(v);
	}
	if ((v = json_find_key(json, "ptp_log_announce_interval")) != NULL) {
		cfg->ptp_log_announce_interval = (int8_t)json_parse_int(v);
	}
	if ((v = json_find_key(json, "pi_kp_num")) != NULL) {
		cfg->pi_kp_num = json_parse_int(v);
	}
	if ((v = json_find_key(json, "pi_kp_den")) != NULL) {
		cfg->pi_kp_den = json_parse_int(v);
	}
	if ((v = json_find_key(json, "pi_ki_num")) != NULL) {
		cfg->pi_ki_num = json_parse_int(v);
	}
	if ((v = json_find_key(json, "pi_ki_den")) != NULL) {
		cfg->pi_ki_den = json_parse_int(v);
	}
	if ((v = json_find_key(json, "pi_imax")) != NULL) {
		cfg->pi_imax = json_parse_int(v);
	}
	if ((v = json_find_key(json, "pi_outlier_ppb")) != NULL) {
		cfg->pi_outlier_ppb = json_parse_int(v);
	}
	if ((v = json_find_key(json, "pi_warmup_cycles")) != NULL) {
		cfg->pi_warmup_cycles = json_parse_uint(v);
	}
	if ((v = json_find_key(json, "sap_announce_interval_s")) != NULL) {
		cfg->sap_announce_interval_s = json_parse_uint(v);
	}
	if ((v = json_find_key(json, "sap_announce_enabled")) != NULL) {
		cfg->sap_announce_enabled = json_parse_bool(v);
	}

	aes67_config_unlock();
	LOG_INF("Device configuration loaded");
	return 0;
}

/* ================================================================
 * Parse integer array (for channel IDs/map)
 * ================================================================ */
static int parse_int_array(const char *arr, uint8_t *out, int max_count)
{
	if (!arr || *arr != '[') {
		return 0;
	}
	arr++;
	int count = 0;
	while (*arr && count < max_count) {
		arr = skip_ws(arr);
		if (*arr == ']') {
			break;
		}
		if (*arr >= '0' && *arr <= '9') {
			out[count++] = (uint8_t)strtoul(arr, NULL, 10);
			while (*arr && *arr != ',' && *arr != ']') {
				arr++;
			}
		}
		if (*arr == ',') {
			arr++;
		}
	}
	return count;
}

/* ================================================================
 * Parse and configure TX streams
 * ================================================================ */
static int parse_tx_streams(const char *json)
{
	const char *arr = json_find_array(json, "tx_streams");
	if (!arr) {
		LOG_DBG("No tx_streams found in config");
		return 0;
	}

	const char *obj = arr;
	int count = 0;

	while ((obj = json_next_obj(obj)) != NULL) {
		const char *obj_end = json_obj_end_ptr(obj);
		if (!obj_end) {
			break;
		}

		/* Extract values from this stream object */
		uint8_t stream_id = 0;
		struct in_addr dst_ip = {0};
		uint8_t channel_count = 0;
		uint8_t samples_per_packet = 48;
		uint32_t ssrc = 0;
		uint8_t ch_ids[AES67_MAX_CH_PER_STREAM] = {0};

		const char *v;

		if ((v = json_find_key(obj, "stream_id")) != NULL) {
			stream_id = (uint8_t)json_parse_uint(v);
		}
		if ((v = json_find_key(obj, "dst_ip")) != NULL) {
			char *ip_str = json_parse_str(v);
			if (ip_str) {
				zsock_inet_pton(AF_INET, ip_str, &dst_ip);
			}
		}
		if ((v = json_find_key(obj, "channel_count")) != NULL) {
			channel_count = (uint8_t)json_parse_uint(v);
		}
		if ((v = json_find_key(obj, "samples_per_packet")) != NULL) {
			samples_per_packet = (uint8_t)json_parse_uint(v);
		}
		if ((v = json_find_key(obj, "ssrc")) != NULL) {
			ssrc = json_parse_uint(v);
		}

		const char *ch_arr = json_find_array(obj, "ch_ids");
		int num_ch = parse_int_array(ch_arr, ch_ids, AES67_MAX_CH_PER_STREAM);
		if (num_ch == 0) {
			/* Default channel IDs */
			for (int j = 0; j < channel_count; j++) {
				ch_ids[j] = j;
			}
			num_ch = channel_count;
		}

		/* Configure the stream */
		int ret = sap_sdp_configure_tx_stream(stream_id, &dst_ip,
						      channel_count,
						      samples_per_packet,
						      ch_ids, num_ch, ssrc);
		if (ret == 0) {
			LOG_INF("TX stream %u configured from SD card", stream_id);
			count++;
		} else {
			LOG_ERR("Failed to configure TX stream %u: %d",
				stream_id, ret);
		}

		obj = obj_end;
	}

	LOG_INF("Loaded %d TX streams from config", count);
	return count;
}

/* ================================================================
 * Parse and configure RX streams
 * ================================================================ */
static int parse_rx_streams(const char *json)
{
	const char *arr = json_find_array(json, "rx_streams");
	if (!arr) {
		LOG_DBG("No rx_streams found in config");
		return 0;
	}

	const char *obj = arr;
	int count = 0;

	while ((obj = json_next_obj(obj)) != NULL) {
		const char *obj_end = json_obj_end_ptr(obj);
		if (!obj_end) {
			break;
		}

		/* Extract values from this stream object */
		uint8_t stream_id = 0;
		struct in_addr dst_ip = {0};
		uint16_t dst_port = 5004;
		uint8_t channel_count = 0;
		uint8_t output_delay = 48;
		uint8_t samples_per_channel = 48;
		uint8_t ch_map[AES67_MAX_CH_PER_STREAM] = {0};

		const char *v;

		if ((v = json_find_key(obj, "stream_id")) != NULL) {
			stream_id = (uint8_t)json_parse_uint(v);
		}
		if ((v = json_find_key(obj, "dst_ip")) != NULL) {
			char *ip_str = json_parse_str(v);
			if (ip_str) {
				zsock_inet_pton(AF_INET, ip_str, &dst_ip);
			}
		}
		if ((v = json_find_key(obj, "dst_port")) != NULL) {
			dst_port = (uint16_t)json_parse_uint(v);
		}
		if ((v = json_find_key(obj, "channel_count")) != NULL) {
			channel_count = (uint8_t)json_parse_uint(v);
		}
		if ((v = json_find_key(obj, "output_delay")) != NULL) {
			output_delay = (uint8_t)json_parse_uint(v);
		}
		if ((v = json_find_key(obj, "samples_per_channel")) != NULL) {
			samples_per_channel = (uint8_t)json_parse_uint(v);
		}

		const char *ch_arr = json_find_array(obj, "ch_map");
		int num_ch = parse_int_array(ch_arr, ch_map, AES67_MAX_CH_PER_STREAM);
		if (num_ch == 0) {
			/* Default channel map: 1:1 */
			for (int j = 0; j < channel_count; j++) {
				ch_map[j] = j;
			}
		}

		/* Configure the stream */
		int ret = sap_sdp_configure_rx_stream(stream_id, &dst_ip,
						      dst_port, ch_map,
						      channel_count,
						      output_delay,
						      samples_per_channel);
		if (ret == 0) {
			LOG_INF("RX stream %u configured from SD card", stream_id);
			count++;
		} else {
			LOG_ERR("Failed to configure RX stream %u: %d",
				stream_id, ret);
		}

		obj = obj_end;
	}

	LOG_INF("Loaded %d RX streams from config", count);
	return count;
}

/* ================================================================
 * Public API: Initialize SD card
 * ================================================================ */
int sd_config_init(void)
{
	int ret;

	k_mutex_init(&sd_mutex);

	/* Check if disk is ready */
	ret = disk_access_init("SD");
	if (ret != 0) {
		LOG_WRN("SD card not detected (ret=%d)", ret);
		return -ENODEV;
	}

	/* Mount FAT filesystem */
	ret = fs_mount(&sd_mount);
	if (ret != 0) {
		LOG_ERR("Failed to mount SD card: %d", ret);
		return ret;
	}

	sd_mounted = true;
	LOG_INF("SD card mounted at %s", SD_MOUNT_POINT);
	return 0;
}

/* ================================================================
 * Public API: Check if SD is ready
 * ================================================================ */
bool sd_config_is_ready(void)
{
	return sd_mounted;
}

/* ================================================================
 * Public API: Load configuration from SD card
 * ================================================================ */
int sd_config_load(void)
{
	struct fs_file_t file;
	struct fs_dirent entry;
	int ret;

	if (!sd_mounted) {
		LOG_WRN("SD card not mounted, cannot load config");
		load_status = SD_CONFIG_LOAD_NO_CARD;
		return -ENODEV;
	}

	k_mutex_lock(&sd_mutex, K_FOREVER);

	/* Check if config file exists */
	ret = fs_stat(SD_CONFIG_FILE_PATH, &entry);
	if (ret != 0) {
		LOG_INF("No config file found at %s, using defaults",
			SD_CONFIG_FILE_PATH);
		load_status = SD_CONFIG_LOAD_NO_FILE;
		k_mutex_unlock(&sd_mutex);
		return -ENOENT;
	}

	if (entry.size >= JSON_BUF_SIZE) {
		LOG_ERR("Config file too large: %zu bytes", entry.size);
		load_status = SD_CONFIG_LOAD_ERROR;
		k_mutex_unlock(&sd_mutex);
		return -ENOMEM;
	}

	/* Open and read file */
	fs_file_t_init(&file);
	ret = fs_open(&file, SD_CONFIG_FILE_PATH, FS_O_READ);
	if (ret != 0) {
		LOG_ERR("Failed to open config file: %d", ret);
		load_status = SD_CONFIG_LOAD_ERROR;
		k_mutex_unlock(&sd_mutex);
		return ret;
	}

	ssize_t bytes = fs_read(&file, json_buf, entry.size);
	fs_close(&file);

	if (bytes < 0) {
		LOG_ERR("Failed to read config file: %zd", bytes);
		load_status = SD_CONFIG_LOAD_ERROR;
		k_mutex_unlock(&sd_mutex);
		return (int)bytes;
	}

	json_buf[bytes] = '\0';
	LOG_INF("Read %zd bytes from config file", bytes);

	/* Parse and apply configuration */
	parse_device_config(json_buf);
	parse_tx_streams(json_buf);
	parse_rx_streams(json_buf);

	load_status = SD_CONFIG_LOAD_OK;
	k_mutex_unlock(&sd_mutex);
	return 0;
}

/* ================================================================
 * Public API: Save configuration to SD card
 * ================================================================ */
int sd_config_save(void)
{
	struct fs_file_t file;
	int ret;

	if (!sd_mounted) {
		LOG_WRN("SD card not mounted, cannot save config");
		return -ENODEV;
	}

	k_mutex_lock(&sd_mutex, K_FOREVER);

	/* Serialize current config to JSON */
	int len = serialize_config(json_buf, JSON_BUF_SIZE);
	if (len < 0 || len >= JSON_BUF_SIZE) {
		LOG_ERR("Failed to serialize config");
		k_mutex_unlock(&sd_mutex);
		return -ENOMEM;
	}

	/* Open file for writing (truncate if exists) */
	fs_file_t_init(&file);
	ret = fs_open(&file, SD_CONFIG_FILE_PATH,
		      FS_O_CREATE | FS_O_WRITE | FS_O_TRUNC);
	if (ret != 0) {
		LOG_ERR("Failed to open config file for writing: %d", ret);
		k_mutex_unlock(&sd_mutex);
		return ret;
	}

	/* Write JSON data */
	ssize_t written = fs_write(&file, json_buf, len);
	ret = fs_close(&file);

	if (written != len) {
		LOG_ERR("Failed to write config file: %zd", written);
		k_mutex_unlock(&sd_mutex);
		return -EIO;
	}

	config_dirty = false;
	LOG_INF("Configuration saved to SD card (%d bytes)", len);

	k_mutex_unlock(&sd_mutex);
	return 0;
}

/* ================================================================
 * Public API: Mark config as dirty
 * ================================================================ */
void sd_config_mark_dirty(void)
{
	config_dirty = true;
}

/* ================================================================
 * Public API: Flush pending changes
 * ================================================================ */
int sd_config_flush(void)
{
	if (!config_dirty) {
		return 0;
	}
	return sd_config_save();
}

/* ================================================================
 * Public API: Format SD card
 * ================================================================ */
int sd_config_format(void)
{
	int ret;

	if (!sd_mounted) {
		LOG_WRN("SD card not mounted, cannot format");
		return -ENODEV;
	}

	k_mutex_lock(&sd_mutex, K_FOREVER);

	LOG_WRN("Formatting SD card...");

	/* Unmount first */
	ret = fs_unmount(&sd_mount);
	if (ret != 0 && ret != -EINVAL) {
		LOG_ERR("Failed to unmount SD card: %d", ret);
		k_mutex_unlock(&sd_mutex);
		return ret;
	}

	/* Format using FatFS f_mkfs with MKFS_PARM structure */
	MKFS_PARM mkfs_opt = {
		.fmt = FM_FAT32,
		.n_fat = 0,    /* Use default */
		.align = 0,    /* Use default */
		.n_root = 0,   /* Use default */
		.au_size = 0   /* Use default cluster size */
	};
	FRESULT fret = f_mkfs("SD:", &mkfs_opt, json_buf, JSON_BUF_SIZE);
	if (fret != FR_OK) {
		LOG_ERR("Failed to format SD card: %d", fret);
		/* Try to remount */
		fs_mount(&sd_mount);
		k_mutex_unlock(&sd_mutex);
		return -EIO;
	}

	/* Remount */
	ret = fs_mount(&sd_mount);
	if (ret != 0) {
		LOG_ERR("Failed to remount SD card after format: %d", ret);
		sd_mounted = false;
		k_mutex_unlock(&sd_mutex);
		return ret;
	}

	load_status = SD_CONFIG_LOAD_NO_FILE;
	LOG_INF("SD card formatted successfully");

	k_mutex_unlock(&sd_mutex);
	return 0;
}

/* ================================================================
 * Public API: Get load status
 * ================================================================ */
enum sd_config_load_status sd_config_get_load_status(void)
{
	return load_status;
}

/* ================================================================
 * Public API: Get status string
 * ================================================================ */
const char *sd_config_status_str(enum sd_config_load_status status)
{
	switch (status) {
	case SD_CONFIG_NOT_LOADED:
		return "not_loaded";
	case SD_CONFIG_LOAD_OK:
		return "ok";
	case SD_CONFIG_LOAD_NO_FILE:
		return "no_file";
	case SD_CONFIG_LOAD_NO_CARD:
		return "no_card";
	case SD_CONFIG_LOAD_ERROR:
		return "error";
	default:
		return "unknown";
	}
}
