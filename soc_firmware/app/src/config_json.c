/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: Apache-2.0
 *
 * Shared JSON serialization / parsing for device configuration.
 * Extracted from sd_config.c so both SD card and SPI flash storage
 * can share the same format.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/socket.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "config_json.h"
#include "aes67_config.h"
#include "sap_sdp.h"

LOG_MODULE_REGISTER(config_json, LOG_LEVEL_INF);

/* ================================================================
 * Simple JSON serialization helpers
 * ================================================================ */

static int json_start(char *buf, size_t sz)
{
	return snprintf(buf, sz, "{\n");
}

static int json_end(char *buf, size_t sz, int pos)
{
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

	pos = json_str(buf, sz, pos, "vendor", cfg->vendor, 1);
	pos = json_str(buf, sz, pos, "product", cfg->product, 1);
	pos = json_str(buf, sz, pos, "serial", cfg->serial, 1);
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
		pos = json_str(buf, sz, pos, "name", streams[i].name, 3);

		zsock_inet_ntop(AF_INET, &streams[i].dst_ip,
				ip_str, sizeof(ip_str));
		pos = json_str(buf, sz, pos, "dst_ip", ip_str, 3);
		pos = json_uint(buf, sz, pos, "channel_count",
				streams[i].channel_count, 3);
		pos = json_uint(buf, sz, pos, "samples_per_packet",
				streams[i].samples_per_packet, 3);
		pos = json_uint(buf, sz, pos, "ssrc", streams[i].ssrc, 3);

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
 * Simple JSON parser helpers
 * ================================================================ */

static const char *skip_ws(const char *p)
{
	while (*p && (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r')) {
		p++;
	}
	return p;
}

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

static int32_t json_parse_int(const char *p)
{
	if (!p) {
		return 0;
	}
	return (int32_t)strtol(p, NULL, 10);
}

static uint32_t json_parse_uint(const char *p)
{
	if (!p) {
		return 0;
	}
	return (uint32_t)strtoul(p, NULL, 10);
}

static bool json_parse_bool(const char *p)
{
	if (!p) {
		return false;
	}
	return (strncmp(p, "true", 4) == 0);
}

static const char *json_find_array(const char *json, const char *key)
{
	const char *p = json_find_key(json, key);
	if (!p || *p != '[') {
		return NULL;
	}
	return p;
}

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

static const char *json_obj_end_ptr(const char *p)
{
	if (!p || *p != '{') {
		return NULL;
	}
	int depth = 1;
	p++;
	while (*p && depth > 0) {
		if (*p == '{') {
			depth++;
		} else if (*p == '}') {
			depth--;
		}
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

	if ((v = json_find_key(json, "vendor")) != NULL) {
		s = json_parse_str(v);
		if (s) {
			strncpy(cfg->vendor, s, AES67_VENDOR_MAX - 1);
		}
	}
	if ((v = json_find_key(json, "product")) != NULL) {
		s = json_parse_str(v);
		if (s) {
			strncpy(cfg->product, s, AES67_PRODUCT_MAX - 1);
		}
	}
	if ((v = json_find_key(json, "serial")) != NULL) {
		s = json_parse_str(v);
		if (s) {
			strncpy(cfg->serial, s, AES67_SERIAL_MAX - 1);
		}
	}
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

		uint8_t stream_id = 0;
		struct in_addr dst_ip = {0};
		uint8_t channel_count = 0;
		uint8_t samples_per_packet = 48;
		uint32_t ssrc = 0;
		uint8_t ch_ids[AES67_MAX_CH_PER_STREAM] = {0};
		char stream_name[AES67_STREAM_NAME_MAX] = {0};

		const char *v;

		if ((v = json_find_key(obj, "stream_id")) != NULL) {
			stream_id = (uint8_t)json_parse_uint(v);
		}
		if ((v = json_find_key(obj, "name")) != NULL) {
			char *name_str = json_parse_str(v);
			if (name_str) {
				strncpy(stream_name, name_str,
					AES67_STREAM_NAME_MAX - 1);
			}
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
		int num_ch = parse_int_array(ch_arr, ch_ids,
					     AES67_MAX_CH_PER_STREAM);
		if (num_ch == 0) {
			for (int j = 0; j < channel_count; j++) {
				ch_ids[j] = j;
			}
			num_ch = channel_count;
		}

		int ret = sap_sdp_configure_tx_stream(
			stream_id, &dst_ip, channel_count,
			samples_per_packet, ch_ids, num_ch, ssrc,
			stream_name[0] ? stream_name : NULL);
		if (ret == 0) {
			LOG_INF("TX stream %u configured", stream_id);
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
		int num_ch = parse_int_array(ch_arr, ch_map,
					     AES67_MAX_CH_PER_STREAM);
		if (num_ch == 0) {
			for (int j = 0; j < channel_count; j++) {
				ch_map[j] = j;
			}
		}

		int ret = sap_sdp_configure_rx_stream(
			stream_id, &dst_ip, dst_port, ch_map,
			channel_count, output_delay, samples_per_channel);
		if (ret == 0) {
			LOG_INF("RX stream %u configured", stream_id);
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
 * Public API
 * ================================================================ */

int config_json_serialize(char *buf, size_t sz)
{
	int pos = 0;

	pos = json_start(buf, sz);
	pos = serialize_device_config(buf, sz, pos);
	pos = serialize_tx_streams(buf, sz, pos);
	pos = serialize_rx_streams(buf, sz, pos);
	pos = json_end(buf, sz, pos);

	return pos;
}

int config_json_parse_and_apply(const char *json)
{
	parse_device_config(json);
	parse_tx_streams(json);
	parse_rx_streams(json);
	return 0;
}
