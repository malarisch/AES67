/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: Apache-2.0
 *
 * REST API + static-file web server for the AES67 device.
 *
 * Uses the Zephyr HTTP server subsystem.
 *
 * Static web UI is served from "/" as a gzipped blob included at build time.
 * Dynamic REST endpoints live under "/api/...".
 */

#include <zephyr/kernel.h>
#include <zephyr/net/http/server.h>
#include <zephyr/net/http/service.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/dhcpv4.h>
#include <zephyr/net/socket.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <stdio.h>

#include "webserver.h"
#include "aes67_config.h"
#include "ieee1588_utils.h"
#include "ptp_bmc.h"
#include "sap_sdp.h"
#include "ui_display.h"
#ifdef CONFIG_SD_CONFIG
#include "sd_config.h"
#endif
#include <zephyr/sys/reboot.h>
#include "../drivers/fpga_hal/fpga_hal.h"
#include "fw_update.h"
#ifdef CONFIG_MI_CARD
#include "../drivers/mi_card/mi_card.h"
#endif
#ifdef CONFIG_LO_CARD
#include "../drivers/lo_card/lo_card.h"
#endif
#ifdef CONFIG_IO_CARD
#include "../drivers/io_card/io_card.h"
#endif
#ifdef CONFIG_DISPLAY_CTRL
#include "../drivers/display_ctrl/display_ctrl.h"
#endif
#include "card_manager.h"

/* local_memmem is a GNU extension not available in picolibc / Zephyr */
static void *local_memmem(const void *haystack, size_t haystacklen,
			  const void *needle, size_t needlelen)
{
	if (needlelen == 0) {
		return (void *)haystack;
	}
	if (haystacklen < needlelen) {
		return NULL;
	}
	const uint8_t *h = haystack;
	const uint8_t *n = needle;

	for (size_t i = 0; i <= haystacklen - needlelen; i++) {
		if (h[i] == n[0] && memcmp(&h[i], n, needlelen) == 0) {
			return (void *)&h[i];
		}
	}
	return NULL;
}

LOG_MODULE_REGISTER(webserver, LOG_LEVEL_INF);

/* ================================================================
 * HTTP service definition — port 80, max 2 concurrent clients
 * ================================================================ */

static uint16_t http_port = 80;

HTTP_SERVICE_DEFINE(aes67_http, "0.0.0.0", &http_port, 4, 10, NULL, NULL);

/* ================================================================
 * JSON response buffer (shared, one request at a time since
 * dynamic resources in Zephyr are already serialised per-holder)
 * ================================================================ */

#define JSON_BUF_SIZE 4096
static char json_buf[JSON_BUF_SIZE];

/* ================================================================
 * Tiny JSON helpers (no library needed)
 * ================================================================ */

static int json_start_object(char *buf, size_t sz)
{
	return snprintf(buf, sz, "{");
}

static int json_end_object(char *buf, size_t sz, int pos)
{
	/* Replace trailing comma with nothing */
	if (pos > 1 && buf[pos - 1] == ',') {
		pos--;
	}
	pos += snprintf(buf + pos, sz - pos, "}");
	return pos;
}

static int json_add_str(char *buf, size_t sz, int pos,
			 const char *key, const char *val)
{
	pos += snprintf(buf + pos, sz - pos, "\"%s\":\"%s\",", key, val);
	return pos;
}

static int json_add_int(char *buf, size_t sz, int pos,
			 const char *key, int32_t val)
{
	pos += snprintf(buf + pos, sz - pos, "\"%s\":%d,", key, val);
	return pos;
}

static int json_add_uint(char *buf, size_t sz, int pos,
			  const char *key, uint32_t val)
{
	pos += snprintf(buf + pos, sz - pos, "\"%s\":%u,", key, val);
	return pos;
}

static int json_add_bool(char *buf, size_t sz, int pos,
			  const char *key, bool val)
{
	pos += snprintf(buf + pos, sz - pos, "\"%s\":%s,",
			key, val ? "true" : "false");
	return pos;
}

static int json_add_key(char *buf, size_t sz, int pos, const char *key)
{
	pos += snprintf(buf + pos, sz - pos, "\"%s\":", key);
	return pos;
}

static int json_start_array(char *buf, size_t sz, int pos)
{
	pos += snprintf(buf + pos, sz - pos, "[");
	return pos;
}

static int json_end_array(char *buf, size_t sz, int pos)
{
	if (pos > 1 && buf[pos - 1] == ',') {
		pos--;
	}
	pos += snprintf(buf + pos, sz - pos, "],");
	return pos;
}

static void format_clock_id(char *out, size_t sz, const uint8_t id[8])
{
	snprintf(out, sz,
		 "%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X",
		 id[0], id[1], id[2], id[3],
		 id[4], id[5], id[6], id[7]);
}

static void format_ip(char *out, size_t sz, const struct in_addr *addr)
{
	zsock_inet_ntop(AF_INET, addr, out, sz);
}

/* ================================================================
 * FPGA metrics cache (updated by main polling thread via
 * ui_display_set_metrics)
 * ================================================================ */

/* We read from the same metrics that ui_display uses.
 * To avoid coupling we just read FPGA regs directly here. */
static int read_fpga_status(struct ui_fpga_metrics *m)
{
	uint32_t status = fpga_hal_read_status();

	m->ppb_valid        = !!(status & FPGA_HAL_CLK_PPB_VALID);
	m->wc_locked        = !!(status & FPGA_HAL_CLK_WC_LOCKED);
	m->wc_phasejump     = !!(status & FPGA_HAL_CLK_WC_PHASEJUMP);
	m->wc_configured    = !!(status & FPGA_HAL_CLK_WC_CONFIGURED);
	m->ptp_leader_lost  = !!(status & FPGA_HAL_CLK_PTP_LEADER_LOST);

	m->link_up    = !!(status & FPGA_HAL_ETH_LINK_UP);
	m->speed_code = (status & FPGA_HAL_ETH_SPEED_MASK) >>
			FPGA_HAL_ETH_SPEED_SHIFT;

	m->path_delay_ns = fpga_hal_read_path_delay();
	m->leader_offset_ns = fpga_hal_read_ptp_offset();

	/* Read raw counters and calculate PPB */
	uint32_t count_wc = 0, count_pll = 0;

	fpga_hal_read_ppb_counts(&count_wc, &count_pll);

	if (count_wc > 0) {
		int32_t diff = (int32_t)count_pll - (int32_t)count_wc;
		m->ppb_offset = (int32_t)(((int64_t)diff * 1000000000LL) /
					  (int64_t)count_wc);
	} else {
		m->ppb_offset = 0;
	}

	return 0;
}

/* ================================================================
 * JSON builders for each endpoint
 * ================================================================ */

static int build_status_network(char *buf, size_t sz)
{
	struct net_if *iface = net_if_get_default();
	int p = json_start_object(buf, sz);
	char tmp[INET_ADDRSTRLEN];

	if (iface) {
		struct net_linkaddr *ll = net_if_get_link_addr(iface);

		if (ll && ll->len >= 6) {
			char mac[18];

			snprintf(mac, sizeof(mac),
				 "%02x:%02x:%02x:%02x:%02x:%02x",
				 ll->addr[0], ll->addr[1], ll->addr[2],
				 ll->addr[3], ll->addr[4], ll->addr[5]);
			p = json_add_str(buf, sz, p, "mac", mac);
		}

		struct net_if_ipv4 *ipv4 = iface->config.ip.ipv4;

		if (ipv4 && ipv4->unicast[0].ipv4.is_used) {
			format_ip(tmp, sizeof(tmp),
				  &ipv4->unicast[0].ipv4.address.in_addr);
			p = json_add_str(buf, sz, p, "ip", tmp);

			format_ip(tmp, sizeof(tmp),
				  &ipv4->unicast[0].netmask);
			p = json_add_str(buf, sz, p, "netmask", tmp);

			format_ip(tmp, sizeof(tmp), &ipv4->gw);
			p = json_add_str(buf, sz, p, "gateway", tmp);
		} else {
			p = json_add_str(buf, sz, p, "ip", "0.0.0.0");
		}

		p = json_add_bool(buf, sz, p, "link_up", net_if_is_up(iface));
	}

	/* FPGA-reported link info */
	struct ui_fpga_metrics m = {0};

	if (read_fpga_status(&m) == 0) {
		p = json_add_str(buf, sz, p, "phy_speed", eth_speed_to_text(m.speed_code));
		p = json_add_bool(buf, sz, p, "phy_link_up", m.link_up);
	}

	p = json_end_object(buf, sz, p);
	return p;
}

static int build_status_ptp(char *buf, size_t sz)
{
	int p = json_start_object(buf, sz);
	char id_str[32];

	/* Role */
	enum ptp_bmc_role role = ptp_bmc_get_role();
	const char *role_str =
		(role == PTP_ROLE_LEADER)   ? "leader" :
		(role == PTP_ROLE_FOLLOWER) ? "follower" : "listening";
	p = json_add_str(buf, sz, p, "role", role_str);

	/* Our identity */
	uint8_t my_id[8];

	ptp_bmc_get_clock_identity(my_id);
	format_clock_id(id_str, sizeof(id_str), my_id);
	p = json_add_str(buf, sz, p, "clock_identity", id_str);

	/* Best master */
	uint8_t best_id[8];

	if (ptp_bmc_get_best_master_id(best_id) == 0) {
		format_clock_id(id_str, sizeof(id_str), best_id);
		p = json_add_str(buf, sz, p, "best_master", id_str);
	} else {
		p = json_add_str(buf, sz, p, "best_master", "none");
	}

	/* Own dataset */
	const struct ptp_announce_dataset *own = ptp_bmc_get_own_dataset();
	if (own) {
		p = json_add_uint(buf, sz, p, "own_priority1",
				   own->gm_priority1);
		p = json_add_uint(buf, sz, p, "own_priority2",
				   own->gm_priority2);
		p = json_add_uint(buf, sz, p, "own_clock_class",
				   own->gm_clock_class);
		p = json_add_uint(buf, sz, p, "own_clock_accuracy",
				   own->gm_clock_accuracy);
	}

	/* Foreign masters */
	int fm_count = 0;
	const struct ptp_announce_dataset *fms =
		ptp_bmc_get_foreign_masters(&fm_count);

	p = json_add_key(buf, sz, p, "foreign_masters");
	p = json_start_array(buf, sz, p);

	for (int i = 0; i < fm_count; i++) {
		p += snprintf(buf + p, sz - p, "{");
		format_clock_id(id_str, sizeof(id_str),
				fms[i].sender_clock_id);
		p = json_add_str(buf, sz, p, "sender_id", id_str);
		format_clock_id(id_str, sizeof(id_str),
				fms[i].gm_identity);
		p = json_add_str(buf, sz, p, "gm_identity", id_str);
		p = json_add_uint(buf, sz, p, "priority1",
				   fms[i].gm_priority1);
		p = json_add_uint(buf, sz, p, "clock_class",
				   fms[i].gm_clock_class);
		p = json_add_uint(buf, sz, p, "clock_accuracy",
				   fms[i].gm_clock_accuracy);
		p = json_add_uint(buf, sz, p, "priority2",
				   fms[i].gm_priority2);
		p = json_add_uint(buf, sz, p, "steps_removed",
				   fms[i].steps_removed);
		p = json_add_uint(buf, sz, p, "time_source",
				   fms[i].time_source);
		p = json_add_uint(buf, sz, p, "announce_count",
				   fms[i].announce_count);
		/* Remove trailing comma and close */
		if (p > 1 && buf[p - 1] == ',') {
			p--;
		}
		p += snprintf(buf + p, sz - p, "},");
	}

	p = json_end_array(buf, sz, p);
	p = json_end_object(buf, sz, p);
	return p;
}

static int build_status_fpga(char *buf, size_t sz)
{
	struct ui_fpga_metrics m = {0};
	int p = json_start_object(buf, sz);

	if (read_fpga_status(&m) == 0) {
		p = json_add_bool(buf, sz, p, "ppb_valid", m.ppb_valid);
		p = json_add_bool(buf, sz, p, "wc_locked", m.wc_locked);
		p = json_add_bool(buf, sz, p, "wc_phasejump", m.wc_phasejump);
		p = json_add_bool(buf, sz, p, "wc_configured", m.wc_configured);
		p = json_add_bool(buf, sz, p, "ptp_leader_lost",
				   m.ptp_leader_lost);
		p = json_add_bool(buf, sz, p, "link_up", m.link_up);
		p = json_add_int(buf, sz, p, "path_delay_ns",
				  m.path_delay_ns);
		p = json_add_int(buf, sz, p, "leader_offset_ns",
				  m.leader_offset_ns);
		p = json_add_int(buf, sz, p, "ppb_offset", m.ppb_offset);

		p = json_add_str(buf, sz, p, "phy_speed", eth_speed_to_text(m.speed_code));
	} else {
		p = json_add_str(buf, sz, p, "error",
				  "FPGA not available");
	}

	p = json_end_object(buf, sz, p);
	return p;
}

static int build_status_streams(char *buf, size_t sz)
{
	int p = json_start_object(buf, sz);
	char tmp[INET_ADDRSTRLEN];

	/* TX streams */
	const struct aes67_tx_stream *txs = sap_sdp_get_tx_streams();

	p = json_add_key(buf, sz, p, "tx_streams");
	p = json_start_array(buf, sz, p);

	for (int i = 0; i < AES67_MAX_TX_STREAMS; i++) {
		if (!txs[i].active) {
			continue;
		}
		p += snprintf(buf + p, sz - p, "{");
		p = json_add_uint(buf, sz, p, "stream_id", txs[i].stream_id);
		p = json_add_str(buf, sz, p, "name", txs[i].name);
		format_ip(tmp, sizeof(tmp), &txs[i].dst_ip);
		p = json_add_str(buf, sz, p, "dst_ip", tmp);
		p = json_add_uint(buf, sz, p, "channel_count",
				   txs[i].channel_count);
		p = json_add_uint(buf, sz, p, "samples_per_packet",
				   txs[i].samples_per_packet);

		p = json_add_key(buf, sz, p, "ch_ids");
		p = json_start_array(buf, sz, p);
		for (int j = 0; j < txs[i].channel_count; j++) {
			p += snprintf(buf + p, sz - p, "%u,",
				      txs[i].ch_ids[j]);
		}
		p = json_end_array(buf, sz, p);

		if (p > 1 && buf[p - 1] == ',') {
			p--;
		}
		p += snprintf(buf + p, sz - p, "},");
	}

	p = json_end_array(buf, sz, p);

	/* RX streams */
	const struct aes67_rx_stream *rxs = sap_sdp_get_rx_streams();

	p = json_add_key(buf, sz, p, "rx_streams");
	p = json_start_array(buf, sz, p);

	for (int i = 0; i < AES67_MAX_RX_STREAMS; i++) {
		if (!rxs[i].active) {
			continue;
		}
		p += snprintf(buf + p, sz - p, "{");
		p = json_add_uint(buf, sz, p, "stream_id", rxs[i].stream_id);
		format_ip(tmp, sizeof(tmp), &rxs[i].dst_ip);
		p = json_add_str(buf, sz, p, "dst_ip", tmp);
		p = json_add_uint(buf, sz, p, "dst_port", rxs[i].dst_port);
		p = json_add_uint(buf, sz, p, "channel_count",
				   rxs[i].channel_count);
		p = json_add_uint(buf, sz, p, "output_delay",
				   rxs[i].output_delay);
		p = json_add_uint(buf, sz, p, "samples_per_channel",
				   rxs[i].samples_per_channel);

		p = json_add_key(buf, sz, p, "ch_map");
		p = json_start_array(buf, sz, p);
		for (int j = 0; j < rxs[i].channel_count; j++) {
			p += snprintf(buf + p, sz - p, "%u,",
				      rxs[i].ch_map[j]);
		}
		p = json_end_array(buf, sz, p);

		if (p > 1 && buf[p - 1] == ',') {
			p--;
		}
		p += snprintf(buf + p, sz - p, "},");
	}

	p = json_end_array(buf, sz, p);

	/* Discovered (foreign) streams */
	int sap_count = 0;
	const struct sap_foreign_stream *foreign =
		sap_sdp_get_foreign_streams(&sap_count);

	p = json_add_key(buf, sz, p, "discovered_streams");
	p = json_start_array(buf, sz, p);

	for (int i = 0; i < SAP_MAX_FOREIGN_STREAMS; i++) {
		if (!foreign[i].valid) {
			continue;
		}
		p += snprintf(buf + p, sz - p, "{");
		p = json_add_str(buf, sz, p, "name", foreign[i].name);
		format_ip(tmp, sizeof(tmp), &foreign[i].mcast_addr);
		p = json_add_str(buf, sz, p, "mcast_addr", tmp);
		p = json_add_uint(buf, sz, p, "port", foreign[i].port);
		p = json_add_uint(buf, sz, p, "channels",
				   foreign[i].channels);
		p = json_add_uint(buf, sz, p, "bit_depth",
				   foreign[i].bit_depth);
		p = json_add_uint(buf, sz, p, "sample_rate",
				   foreign[i].sample_rate);
		p = json_add_uint(buf, sz, p, "samples_per_packet",
				   foreign[i].samples_per_packet);
		if (foreign[i].ssrc != 0) {
			p += snprintf(buf + p, sz - p, "\"ssrc\":\"%08X\",",
				      foreign[i].ssrc);
		} else {
			p += snprintf(buf + p, sz - p, "\"ssrc\":\"\",");
		}
		format_ip(tmp, sizeof(tmp), &foreign[i].origin_addr);
		p = json_add_str(buf, sz, p, "origin_addr", tmp);
		if (p > 1 && buf[p - 1] == ',') {
			p--;
		}
		p += snprintf(buf + p, sz - p, "},");
	}

	p = json_end_array(buf, sz, p);
	p = json_end_object(buf, sz, p);
	return p;
}

static int build_config_json(char *buf, size_t sz)
{
	aes67_config_lock();
	const struct aes67_device_config *cfg = aes67_config_get();
	int p = json_start_object(buf, sz);

	p = json_add_str(buf, sz, p, "device_name", cfg->device_name);
	p = json_add_str(buf, sz, p, "default_mcast_addr",
			  cfg->default_mcast_addr);
	p = json_add_uint(buf, sz, p, "default_port", cfg->default_port);
	p = json_add_uint(buf, sz, p, "default_channels",
			   cfg->default_channels);
	p = json_add_uint(buf, sz, p, "default_bit_depth",
			   cfg->default_bit_depth);
	p = json_add_uint(buf, sz, p, "default_sample_rate",
			   cfg->default_sample_rate);
	p = json_add_uint(buf, sz, p, "default_samples_per_pkt",
			   cfg->default_samples_per_pkt);
	p = json_add_uint(buf, sz, p, "default_payload_type",
			   cfg->default_payload_type);
	p = json_add_uint(buf, sz, p, "ptp_domain", cfg->ptp_domain);
	p = json_add_uint(buf, sz, p, "ptp_priority1", cfg->ptp_priority1);
	p = json_add_uint(buf, sz, p, "ptp_priority2", cfg->ptp_priority2);
	p = json_add_uint(buf, sz, p, "ptp_clock_class", cfg->ptp_clock_class);
	p = json_add_uint(buf, sz, p, "ptp_clock_accuracy", cfg->ptp_clock_accuracy);
	p = json_add_int(buf, sz, p, "ptp_log_sync_interval",
			  cfg->ptp_log_sync_interval);
	p = json_add_int(buf, sz, p, "ptp_log_announce_interval",
			  cfg->ptp_log_announce_interval);
	p = json_add_int(buf, sz, p, "pi_kp_num", cfg->pi_kp_num);
	p = json_add_int(buf, sz, p, "pi_kp_den", cfg->pi_kp_den);
	p = json_add_int(buf, sz, p, "pi_ki_num", cfg->pi_ki_num);
	p = json_add_int(buf, sz, p, "pi_ki_den", cfg->pi_ki_den);
	p = json_add_int(buf, sz, p, "pi_imax", cfg->pi_imax);
	p = json_add_int(buf, sz, p, "pi_outlier_ppb", cfg->pi_outlier_ppb);
	p = json_add_uint(buf, sz, p, "pi_warmup_cycles",
			   cfg->pi_warmup_cycles);
	p = json_add_uint(buf, sz, p, "sap_announce_interval_s",
			   cfg->sap_announce_interval_s);
	p = json_add_bool(buf, sz, p, "sap_announce_enabled",
			   cfg->sap_announce_enabled);

	aes67_config_unlock();
	p = json_end_object(buf, sz, p);
	return p;
}

/* ================================================================
 * Minimal JSON value extractor (no library needed)
 *
 * Finds "key": <value> in a JSON string and copies value into out.
 * Returns the number of characters copied, or 0 if not found.
 * ================================================================ */

static int json_find_str(const char *json, size_t json_len,
			  const char *key, char *out, size_t out_sz)
{
	/* Search for "key":" */
	char needle[64];
	int nlen = snprintf(needle, sizeof(needle), "\"%s\":\"", key);

	if (nlen <= 0) {
		return 0;
	}

	const char *start = local_memmem(json, json_len, needle, nlen);

	if (!start) {
		return 0;
	}
	start += nlen;

	const char *end = memchr(start, '"', json_len - (start - json));

	if (!end) {
		return 0;
	}
	size_t vlen = end - start;

	if (vlen >= out_sz) {
		vlen = out_sz - 1;
	}
	memcpy(out, start, vlen);
	out[vlen] = '\0';
	return (int)vlen;
}

static bool json_find_int(const char *json, size_t json_len,
			   const char *key, int32_t *out)
{
	char needle[64];
	int nlen = snprintf(needle, sizeof(needle), "\"%s\":", key);

	if (nlen <= 0) {
		return false;
	}

	const char *start = local_memmem(json, json_len, needle, nlen);

	if (!start) {
		return false;
	}
	start += nlen;

	/* Skip whitespace */
	while (start < json + json_len && *start == ' ') {
		start++;
	}

	char *endptr;
	long val = strtol(start, &endptr, 10);

	if (endptr == start) {
		return false;
	}
	*out = (int32_t)val;
	return true;
}

static bool json_find_bool(const char *json, size_t json_len,
			    const char *key, bool *out)
{
	char needle[64];
	int nlen = snprintf(needle, sizeof(needle), "\"%s\":", key);

	if (nlen <= 0) {
		return false;
	}

	const char *start = local_memmem(json, json_len, needle, nlen);

	if (!start) {
		return false;
	}
	start += nlen;

	while (start < json + json_len && *start == ' ') {
		start++;
	}

	if (start + 4 <= json + json_len && memcmp(start, "true", 4) == 0) {
		*out = true;
		return true;
	}
	if (start + 5 <= json + json_len && memcmp(start, "false", 5) == 0) {
		*out = false;
		return true;
	}
	return false;
}

/* ================================================================
 * Apply configuration POST body
 * ================================================================ */

static int apply_config_json(const char *json, size_t len)
{
	aes67_config_lock();
	struct aes67_device_config *cfg = aes67_config_get();
	char str_tmp[AES67_DEVICE_NAME_MAX];
	int32_t i_tmp;
	bool b_tmp;

	if (json_find_str(json, len, "device_name",
			  str_tmp, sizeof(str_tmp)) > 0) {
		strncpy(cfg->device_name, str_tmp,
			AES67_DEVICE_NAME_MAX - 1);
	}

	if (json_find_str(json, len, "default_mcast_addr",
			  str_tmp, sizeof(str_tmp)) > 0) {
		strncpy(cfg->default_mcast_addr, str_tmp,
			sizeof(cfg->default_mcast_addr) - 1);
	}

	if (json_find_int(json, len, "default_port", &i_tmp)) {
		cfg->default_port = (uint16_t)i_tmp;
	}
	if (json_find_int(json, len, "default_channels", &i_tmp)) {
		cfg->default_channels = (uint8_t)i_tmp;
	}
	if (json_find_int(json, len, "default_bit_depth", &i_tmp)) {
		cfg->default_bit_depth = (uint8_t)i_tmp;
	}
	if (json_find_int(json, len, "default_sample_rate", &i_tmp)) {
		cfg->default_sample_rate = (uint32_t)i_tmp;
	}
	if (json_find_int(json, len, "default_samples_per_pkt", &i_tmp)) {
		cfg->default_samples_per_pkt = (uint16_t)i_tmp;
	}
	if (json_find_int(json, len, "default_payload_type", &i_tmp)) {
		cfg->default_payload_type = (uint8_t)i_tmp;
	}
	if (json_find_int(json, len, "ptp_domain", &i_tmp)) {
		cfg->ptp_domain = (uint8_t)i_tmp;
	}
	if (json_find_int(json, len, "ptp_priority1", &i_tmp)) {
		cfg->ptp_priority1 = (uint8_t)i_tmp;
	}
	if (json_find_int(json, len, "ptp_priority2", &i_tmp)) {
		cfg->ptp_priority2 = (uint8_t)i_tmp;
	}
	if (json_find_int(json, len, "ptp_clock_class", &i_tmp)) {
		cfg->ptp_clock_class = (uint8_t)i_tmp;
	}
	if (json_find_int(json, len, "ptp_clock_accuracy", &i_tmp)) {
		cfg->ptp_clock_accuracy = (uint8_t)i_tmp;
	}
	if (json_find_int(json, len, "ptp_log_sync_interval", &i_tmp)) {
		cfg->ptp_log_sync_interval = (int8_t)i_tmp;
	}
	if (json_find_int(json, len, "ptp_log_announce_interval", &i_tmp)) {
		cfg->ptp_log_announce_interval = (int8_t)i_tmp;
	}
	if (json_find_int(json, len, "pi_kp_num", &i_tmp)) {
		cfg->pi_kp_num = i_tmp;
	}
	if (json_find_int(json, len, "pi_kp_den", &i_tmp)) {
		cfg->pi_kp_den = i_tmp;
	}
	if (json_find_int(json, len, "pi_ki_num", &i_tmp)) {
		cfg->pi_ki_num = i_tmp;
	}
	if (json_find_int(json, len, "pi_ki_den", &i_tmp)) {
		cfg->pi_ki_den = i_tmp;
	}
	if (json_find_int(json, len, "pi_imax", &i_tmp)) {
		cfg->pi_imax = i_tmp;
	}
	if (json_find_int(json, len, "pi_outlier_ppb", &i_tmp)) {
		cfg->pi_outlier_ppb = i_tmp;
	}
	if (json_find_int(json, len, "pi_warmup_cycles", &i_tmp)) {
		cfg->pi_warmup_cycles = (uint32_t)i_tmp;
	}
	if (json_find_int(json, len, "sap_announce_interval_s", &i_tmp)) {
		cfg->sap_announce_interval_s = (uint32_t)i_tmp;
	}
	if (json_find_bool(json, len, "sap_announce_enabled", &b_tmp)) {
		cfg->sap_announce_enabled = b_tmp;
		sap_sdp_set_announce(b_tmp);
	}

	aes67_config_unlock();

	/* Propagate PTP config changes to BMC's own dataset */
	ptp_bmc_update_own_dataset();

	LOG_INF("WEB: Configuration updated via REST API");
	return 0;
}

/* ================================================================
 * Apply TX stream POST body
 * ================================================================ */

static int apply_tx_stream_json(const char *json, size_t len)
{
	int32_t stream_id = -1;
	int32_t channel_count = 0;
	int32_t samples_per_pkt = 48;
	char dst_ip_str[INET_ADDRSTRLEN] = {0};
	char stream_name[AES67_STREAM_NAME_MAX] = {0};

	if (!json_find_int(json, len, "stream_id", &stream_id) ||
	    stream_id < 0 || stream_id > 7) {
		return -EINVAL;
	}

	if (json_find_str(json, len, "dst_ip", dst_ip_str,
			  sizeof(dst_ip_str)) <= 0) {
		return -EINVAL;
	}

	/* Parse optional stream name */
	json_find_str(json, len, "name", stream_name, sizeof(stream_name));

	struct in_addr dst;

	if (zsock_inet_pton(AF_INET, dst_ip_str, &dst) != 1) {
		return -EINVAL;
	}

	json_find_int(json, len, "channel_count", &channel_count);
	json_find_int(json, len, "samples_per_pkt", &samples_per_pkt);

	if (channel_count < 1 || channel_count > 8) {
		channel_count = 2;
	}

	/* Parse channel IDs from "ch_ids":[0,1,...] */
	uint8_t ch_ids[8] = {0};
	char ch_needle[] = "\"ch_ids\":[";
	const char *arr = local_memmem(json, len, ch_needle, strlen(ch_needle));

	if (arr) {
		arr += strlen(ch_needle);
		for (int i = 0; i < channel_count && i < 8; i++) {
			while (*arr == ' ') {
				arr++;
			}
			ch_ids[i] = (uint8_t)strtol(arr, (char **)&arr, 10);
			if (*arr == ',') {
				arr++;
			}
		}
	} else {
		/* Default: sequential channel IDs */
		for (int i = 0; i < channel_count; i++) {
			ch_ids[i] = (uint8_t)i;
		}
	}

	return sap_sdp_configure_tx_stream((uint8_t)stream_id, &dst,
					   (uint8_t)channel_count,
					   (uint8_t)samples_per_pkt,
					   ch_ids, (uint8_t)channel_count,
					   0, stream_name[0] ? stream_name : NULL);
}

/* ================================================================
 * Apply RX stream POST body
 * ================================================================ */

static int apply_rx_stream_json(const char *json, size_t len)
{
	int32_t stream_id = -1;
	int32_t channel_count = 0;
	int32_t output_delay = 0;
	int32_t samples_per_channel = 48;
	int32_t dst_port_val = 5004;

	if (!json_find_int(json, len, "stream_id", &stream_id) ||
	    stream_id < 0 || stream_id >= AES67_MAX_RX_STREAMS) {
		return -EINVAL;
	}

	/* Destination IP address / multicast group (required) */
	char ip_str[INET_ADDRSTRLEN] = {0};
	struct in_addr dst_ip = {0};

	if (json_find_str(json, len, "dst_ip", ip_str, sizeof(ip_str)) <= 0) {
		return -EINVAL;
	}
	if (zsock_inet_pton(AF_INET, ip_str, &dst_ip) != 1) {
		return -EINVAL;
	}

	/* Destination port */
	json_find_int(json, len, "dst_port", &dst_port_val);

	json_find_int(json, len, "channel_count", &channel_count);
	json_find_int(json, len, "output_delay", &output_delay);
	json_find_int(json, len, "samples_per_channel", &samples_per_channel);

	if (channel_count < 1 || channel_count > AES67_MAX_CH_PER_STREAM) {
		channel_count = 2;
	}
	if (samples_per_channel < 1 || samples_per_channel > 255) {
		samples_per_channel = 48;
	}

	/* Parse ch_map from "ch_map":[0,1,...] */
	uint8_t ch_map[AES67_MAX_CH_PER_STREAM] = {0};
	char ch_needle[] = "\"ch_map\":[";
	const char *arr = local_memmem(json, len, ch_needle, strlen(ch_needle));

	if (arr) {
		arr += strlen(ch_needle);
		for (int i = 0; i < channel_count && i < AES67_MAX_CH_PER_STREAM; i++) {
			while (*arr == ' ') {
				arr++;
			}
			ch_map[i] = (uint8_t)strtol(arr, (char **)&arr, 10);
			if (*arr == ',') {
				arr++;
			}
		}
	} else {
		/* Default: identity mapping */
		for (int i = 0; i < channel_count; i++) {
			ch_map[i] = (uint8_t)i;
		}
	}

	return sap_sdp_configure_rx_stream((uint8_t)stream_id,
					   &dst_ip,
					   (uint16_t)dst_port_val,
					   ch_map,
					   (uint8_t)channel_count,
					   (uint8_t)output_delay,
					   (uint8_t)samples_per_channel);
}

/* ================================================================
 * DELETE TX stream
 * ================================================================ */

static int delete_tx_stream(int stream_id)
{
	if (stream_id < 0 || stream_id >= AES67_MAX_TX_STREAMS) {
		return -EINVAL;
	}

	/* Deactivate stream: write zero config to FPGA and clear local table */
	struct in_addr zero_ip = {.s_addr = 0};
	uint8_t zero_ch[8] = {0};

	return sap_sdp_configure_tx_stream((uint8_t)stream_id, &zero_ip,
					   0, 0, zero_ch, 0, 0, NULL);
}

/* ================================================================
 * DELETE RX stream
 * ================================================================ */

static int delete_rx_stream(int stream_id)
{
	if (stream_id < 0 || stream_id >= AES67_MAX_RX_STREAMS) {
		return -EINVAL;
	}

	/* Zero IP+port config effectively disables the stream in the FPGA */
	struct in_addr zero_ip = {.s_addr = 0};
	uint8_t zero_map[AES67_MAX_CH_PER_STREAM] = {0};

	return sap_sdp_configure_rx_stream((uint8_t)stream_id, &zero_ip, 0,
					   zero_map, 1, 0, 0);
}

/* ================================================================
 * Build a combined status response (/api/status)
 * ================================================================ */

static int build_full_status(char *buf, size_t sz)
{
	int p = json_start_object(buf, sz);

	p = json_add_key(buf, sz, p, "network");
	p += build_status_network(buf + p, sz - p);
	p += snprintf(buf + p, sz - p, ",");

	p = json_add_key(buf, sz, p, "ptp");
	p += build_status_ptp(buf + p, sz - p);
	p += snprintf(buf + p, sz - p, ",");

	p = json_add_key(buf, sz, p, "fpga");
	p += build_status_fpga(buf + p, sz - p);
	p += snprintf(buf + p, sz - p, ",");

	p = json_add_key(buf, sz, p, "streams");
	p += build_status_streams(buf + p, sz - p);

	/* Don't add trailing comma for last element */
	p = json_end_object(buf, sz, p);
	return p;
}

#ifdef CONFIG_MI_CARD
/* ================================================================
 * MI Card (8-channel preamp) status/control
 * ================================================================ */

static int build_mi_status(char *buf, size_t sz)
{
	int p = json_start_object(buf, sz);

	/* Global settings */
	int hpf = mi_card_get_hpf();
	int f96 = mi_card_get_96khz();

	p = json_add_bool(buf, sz, p, "hpf", (hpf > 0));
	p = json_add_bool(buf, sz, p, "f96khz", (f96 > 0));

	/* Per-channel settings */
	p = json_add_key(buf, sz, p, "channels");
	p = json_start_array(buf, sz, p);

	for (int ch = 0; ch < MI_NUM_CHANNELS; ch++) {
		p += snprintf(buf + p, sz - p, "{");
		p = json_add_uint(buf, sz, p, "id", ch);

		int gain = mi_card_get_gain(ch);
		int phantom = mi_card_get_phantom(ch);
		int muted = mi_card_get_mute(ch);

		p = json_add_int(buf, sz, p, "gain", gain);
		p = json_add_bool(buf, sz, p, "phantom", (phantom > 0));
		p = json_add_bool(buf, sz, p, "muted", (muted > 0));

		/* Remove trailing comma and close */
		if (p > 1 && buf[p - 1] == ',') {
			p--;
		}
		p += snprintf(buf + p, sz - p, "},");
	}

	p = json_end_array(buf, sz, p);
	p = json_end_object(buf, sz, p);
	return p;
}

static int apply_mi_channel_json(int channel, const char *json, size_t len)
{
	int32_t val;
	bool bval;
	int ret = 0;

	if (channel < 0 || channel >= MI_NUM_CHANNELS) {
		return -EINVAL;
	}

	if (json_find_int(json, len, "gain", &val)) {
		ret = mi_card_set_gain((uint8_t)channel, (int8_t)val);
		if (ret < 0) {
			return ret;
		}
	}

	if (json_find_bool(json, len, "phantom", &bval)) {
		ret = mi_card_set_phantom((uint8_t)channel, bval);
		if (ret < 0) {
			return ret;
		}
	}

	if (json_find_bool(json, len, "muted", &bval)) {
		ret = mi_card_set_mute((uint8_t)channel, bval);
		if (ret < 0) {
			return ret;
		}
	}

	return 0;
}

static int apply_mi_global_json(const char *json, size_t len)
{
	bool bval;
	int ret = 0;

	if (json_find_bool(json, len, "hpf", &bval)) {
		ret = mi_card_set_hpf(bval);
		if (ret < 0) {
			return ret;
		}
	}

	if (json_find_bool(json, len, "f96khz", &bval)) {
		ret = mi_card_set_96khz(bval);
		if (ret < 0) {
			return ret;
		}
	}

	return 0;
}
#endif /* CONFIG_MI_CARD */

#ifdef CONFIG_LO_CARD
/* ================================================================
 * LO Card (8-channel line output) status/control
 * ================================================================ */

static int build_lo_status(char *buf, size_t sz)
{
	int p = json_start_object(buf, sz);

	/* Global settings */
	int f96 = lo_card_get_96khz();
	int oe = lo_card_get_output_enable();

	p = json_add_bool(buf, sz, p, "f96khz", (f96 > 0));
	p = json_add_bool(buf, sz, p, "output_enable", (oe > 0));

	/* Per-channel settings */
	p = json_add_key(buf, sz, p, "channels");
	p = json_start_array(buf, sz, p);

	for (int ch = 0; ch < LO_NUM_CHANNELS; ch++) {
		p += snprintf(buf + p, sz - p, "{");
		p = json_add_uint(buf, sz, p, "id", ch);

		int clip = lo_card_get_clip(ch);
		int muted = lo_card_get_mute(ch);

		p = json_add_int(buf, sz, p, "clip", clip);
		p = json_add_bool(buf, sz, p, "muted", (muted > 0));

		if (p > 1 && buf[p - 1] == ',') {
			p--;
		}
		p += snprintf(buf + p, sz - p, "},");
	}

	p = json_end_array(buf, sz, p);
	p = json_end_object(buf, sz, p);
	return p;
}

static int apply_lo_channel_json(int channel, const char *json, size_t len)
{
	int32_t val;
	bool bval;
	int ret = 0;

	if (channel < 0 || channel >= LO_NUM_CHANNELS) {
		return -EINVAL;
	}

	if (json_find_int(json, len, "clip", &val)) {
		ret = lo_card_set_clip((uint8_t)channel, (int8_t)val);
		if (ret < 0) {
			return ret;
		}
	}

	if (json_find_bool(json, len, "muted", &bval)) {
		ret = lo_card_set_mute((uint8_t)channel, bval);
		if (ret < 0) {
			return ret;
		}
	}

	return 0;
}

static int apply_lo_global_json(const char *json, size_t len)
{
	bool bval;
	int ret = 0;

	if (json_find_bool(json, len, "f96khz", &bval)) {
		ret = lo_card_set_96khz(bval);
		if (ret < 0) {
			return ret;
		}
	}

	if (json_find_bool(json, len, "output_enable", &bval)) {
		ret = lo_card_enable_outputs(bval);
		if (ret < 0) {
			return ret;
		}
	}

	return 0;
}
#endif /* CONFIG_LO_CARD */

/* ================================================================
 * Card Manager — /api/cards/* builders and apply helpers
 * ================================================================ */

/**
 * Build a JSON object for one card slot:
 * {
 *   "slot": 0,
 *   "type": "MI (8-ch ADC preamp)",
 *   "type_id": 1,
 *   "present": true,
 *   "initialized": true,
 *   "board_id": 1,
 *   "soft_id": 0,
 *   "hard_rev": 0
 * }
 */
static int build_card_slot_json(char *buf, size_t sz, int slot)
{
	const struct card_info *info = card_manager_get_info(slot);
	int p = json_start_object(buf, sz);

	p = json_add_int(buf, sz, p, "slot", slot);

	if (info == NULL) {
		p = json_add_bool(buf, sz, p, "present", false);
		p = json_add_str(buf, sz, p, "type", "none");
		p = json_add_int(buf, sz, p, "type_id", 0);
		p = json_end_object(buf, sz, p);
		return p;
	}

	p = json_add_bool(buf, sz, p, "present",     info->present);
	p = json_add_bool(buf, sz, p, "initialized", info->initialized);
	p = json_add_str(buf, sz, p, "type", card_type_name(info->type));
	p = json_add_int(buf, sz, p, "type_id", (int)info->type);

	if (info->present) {
		p = json_add_int(buf, sz, p, "board_id", info->ident.board_id);
		p = json_add_int(buf, sz, p, "soft_id",  info->ident.soft_id);
		p = json_add_int(buf, sz, p, "hard_rev", info->ident.hard_rev);
	}

	p = json_end_object(buf, sz, p);
	return p;
}

/** GET /api/cards — summary of all slots + last I2C scan */
static int build_cards_overview(char *buf, size_t sz)
{
	int p = json_start_object(buf, sz);

	/* Per-slot info */
	p = json_add_key(buf, sz, p, "slots");
	p = json_start_array(buf, sz, p);
	for (int s = 0; s < CARD_MAX_SLOTS; s++) {
		p += build_card_slot_json(buf + p, sz - p, s);
		p += snprintf(buf + p, sz - p, ",");
	}
	p = json_end_array(buf, sz, p);

	/* Last I2C bus scan result */
	struct card_i2c_scan_result scan;
	card_manager_get_scan_result(&scan);

	p = json_add_key(buf, sz, p, "i2c_devices");
	p = json_start_array(buf, sz, p);
	for (int i = 0; i < scan.count; i++) {
		p += snprintf(buf + p, sz - p, "%d,", scan.addr[i]);
	}
	p = json_end_array(buf, sz, p);

	p = json_end_object(buf, sz, p);
	return p;
}

#ifdef CONFIG_IO_CARD
/** GET /api/cards/io — full IO card status (inputs + outputs) */
static int build_io_card_status(char *buf, size_t sz)
{
	int p = json_start_object(buf, sz);

	/* Global */
	p = json_add_bool(buf, sz, p, "output_enable",
			  io_card_get_output_enable() > 0);

	/* Input channels */
	uint16_t overflow_mask = io_card_get_in_overflow();

	p = json_add_key(buf, sz, p, "inputs");
	p = json_start_array(buf, sz, p);
	for (int ch = 0; ch < IO_NUM_IN_CHANNELS; ch++) {
		p += snprintf(buf + p, sz - p, "{");
		p = json_add_uint(buf, sz, p, "id", (uint32_t)ch);
		p = json_add_int(buf, sz, p,  "gain",    io_card_get_in_gain(ch));
		p = json_add_bool(buf, sz, p, "phantom",  io_card_get_in_phantom(ch) > 0);
		p = json_add_bool(buf, sz, p, "muted",    io_card_get_in_mute(ch) > 0);
		p = json_add_bool(buf, sz, p, "clip",
				   (overflow_mask & (1U << ch)) != 0);
		if (buf[p - 1] == ',') { p--; }
		p += snprintf(buf + p, sz - p, "},");
	}
	p = json_end_array(buf, sz, p);

	/* Output channels */
	p = json_add_key(buf, sz, p, "outputs");
	p = json_start_array(buf, sz, p);
	for (int ch = 0; ch < IO_NUM_OUT_CHANNELS; ch++) {
		p += snprintf(buf + p, sz - p, "{");
		p = json_add_uint(buf, sz, p, "id", (uint32_t)ch);
		p = json_add_int(buf, sz, p,  "clip",  io_card_get_out_clip(ch));
		p = json_add_bool(buf, sz, p, "muted", io_card_get_out_mute(ch) > 0);
		if (buf[p - 1] == ',') { p--; }
		p += snprintf(buf + p, sz - p, "},");
	}
	p = json_end_array(buf, sz, p);

	p = json_end_object(buf, sz, p);
	return p;
}

static int apply_io_in_channel_json(int ch, const char *json, size_t len)
{
	int32_t val;
	bool bval;

	if (ch < 0 || ch >= IO_NUM_IN_CHANNELS) {
		return -EINVAL;
	}

	if (json_find_int(json, len, "gain", &val)) {
		int ret = io_card_set_in_gain((uint8_t)ch, (int8_t)val);
		if (ret < 0) { return ret; }
	}
	if (json_find_bool(json, len, "phantom", &bval)) {
		int ret = io_card_set_in_phantom((uint8_t)ch, bval);
		if (ret < 0) { return ret; }
	}
	if (json_find_bool(json, len, "muted", &bval)) {
		int ret = io_card_set_in_mute((uint8_t)ch, bval);
		if (ret < 0) { return ret; }
	}
	return 0;
}

static int apply_io_out_channel_json(int ch, const char *json, size_t len)
{
	int32_t val;
	bool bval;

	if (ch < 0 || ch >= IO_NUM_OUT_CHANNELS) {
		return -EINVAL;
	}

	if (json_find_int(json, len, "clip", &val)) {
		int ret = io_card_set_out_clip((uint8_t)ch, (int8_t)val);
		if (ret < 0) { return ret; }
	}
	if (json_find_bool(json, len, "muted", &bval)) {
		int ret = io_card_set_out_mute((uint8_t)ch, bval);
		if (ret < 0) { return ret; }
	}
	return 0;
}

static int apply_io_global_json(const char *json, size_t len)
{
	bool bval;

	if (json_find_bool(json, len, "output_enable", &bval)) {
		int ret = io_card_enable_outputs(bval);
		if (ret < 0) { return ret; }
	}
	if (json_find_bool(json, len, "f96khz", &bval)) {
		int ret = io_card_set_96khz(bval);
		if (ret < 0) { return ret; }
	}
	return 0;
}
#endif /* CONFIG_IO_CARD */

/* ================================================================
 * POST body accumulator
 * ================================================================ */

#define POST_BODY_MAX 4096
static char post_body[POST_BODY_MAX];
static size_t post_body_len;

/* ================================================================
 * Display debug helpers
 * ================================================================ */

#ifdef CONFIG_DISPLAY_CTRL

static int build_display_debug_status(char *buf, size_t sz)
{
	int p = json_start_object(buf, sz);

	p = json_add_bool(buf, sz, p, "ready", display_ctrl_ready());

	/* 7-segment state (character names) */
	static const char *vcc_names[] = {
		"0","1","2","3","4","5","6","7","8","9",
		"A","b","C","d","E","F","G","h","I","J",
		"L","M","n","O","P","q","r","S","t","U",
		"X","y"," ","-"
	};

	p = json_add_key(buf, sz, p, "segments");
	p += snprintf(buf + p, sz - p, "{");
	for (int d = 0; d < DC_NUM_DISPLAYS; d++) {
		const char *dname = d == 0 ? "left" : d == 1 ? "mid" : "right";
		p = json_add_key(buf, sz, p, dname);
		p += snprintf(buf + p, sz - p, "{");

		int l = display_ctrl_get_segment_left(d);
		int r = display_ctrl_get_segment_right(d);

		if (l >= 0 && l < VCC_LAST) {
			p = json_add_str(buf, sz, p, "left", vcc_names[l]);
		}
		if (r >= 0 && r < VCC_LAST) {
			p = json_add_str(buf, sz, p, "right", vcc_names[r]);
		}
		if (p > 1 && buf[p - 1] == ',') p--;
		p += snprintf(buf + p, sz - p, "},");
	}
	if (p > 1 && buf[p - 1] == ',') p--;
	p += snprintf(buf + p, sz - p, "},");

	/* Channel LEDs */
	p = json_add_key(buf, sz, p, "channel_leds");
	p += snprintf(buf + p, sz - p, "{");
	p = json_add_uint(buf, sz, p, "mute",
			  display_ctrl_get_channel_led_pattern(DC_CHNLED_MUTE));
	p = json_add_uint(buf, sz, p, "signal",
			  display_ctrl_get_channel_led_pattern(DC_CHNLED_SIGNAL));
	p = json_add_uint(buf, sz, p, "clip",
			  display_ctrl_get_channel_led_pattern(DC_CHNLED_CLIP));
	p = json_add_uint(buf, sz, p, "phantom",
			  display_ctrl_get_channel_led_pattern(DC_CHNLED_PHANTOM));
	if (p > 1 && buf[p - 1] == ',') p--;
	p += snprintf(buf + p, sz - p, "},");

	/* System LEDs */
	p = json_add_key(buf, sz, p, "sys_leds");
	p += snprintf(buf + p, sz - p, "{");
	static const char *sled_names[] = {
		"psu_b","lop","ext","96k","master","48k","lip","psu_a",
		"usb","eth","rem_lip","rem_lop","pwr"
	};
	for (int i = 0; i < DC_SYSLED_LAST && i < 13; i++) {
		p = json_add_int(buf, sz, p, sled_names[i],
				 display_ctrl_get_sys_led(i));
	}
	if (p > 1 && buf[p - 1] == ',') p--;
	p += snprintf(buf + p, sz - p, "},");

	p = json_add_int(buf, sz, p, "max_channels", DC_MAX_CHANNELS);
	p = json_end_object(buf, sz, p);
	return p;
}

/* ASCII to VCC helper for web API */
static uint8_t ascii_to_vcc_web(char c)
{
	if (c >= '0' && c <= '9') return VCC_NUM0 + (c - '0');
	switch (c) {
	case 'A': case 'a': return VCC_A;
	case 'B': case 'b': return VCC_B;
	case 'C': case 'c': return VCC_C;
	case 'D': case 'd': return VCC_D;
	case 'E': case 'e': return VCC_E;
	case 'F': case 'f': return VCC_F;
	case 'G': case 'g': return VCC_G;
	case 'H': case 'h': return VCC_H;
	case 'I': case 'i': return VCC_I;
	case 'J': case 'j': return VCC_J;
	case 'L': case 'l': return VCC_L;
	case 'M': case 'm': return VCC_M;
	case 'N': case 'n': return VCC_N;
	case 'O': case 'o': return VCC_O;
	case 'P': case 'p': return VCC_P;
	case 'Q': case 'q': return VCC_Q;
	case 'R': case 'r': return VCC_R;
	case 'S': case 's': return VCC_S;
	case 'T': case 't': return VCC_T;
	case 'U': case 'u': return VCC_U;
	case 'X': case 'x': return VCC_X;
	case 'Y': case 'y': return VCC_Y;
	case '-': return VCC_MINUS;
	case ' ': default: return VCC_SPACE;
	}
}

/* POST /api/debug/display/segment
 * Body: {"display":0, "left":"A", "right":"3"} */
static int apply_display_segment_json(const char *json, size_t len)
{
	int32_t disp = 0;
	char left_str[4] = {0}, right_str[4] = {0};

	if (!display_ctrl_ready()) {
		return -ENODEV;
	}

	if (!json_find_int(json, len, "display", &disp)) {
		return -EINVAL;
	}
	if (disp < 0 || disp >= DC_NUM_DISPLAYS) {
		return -EINVAL;
	}

	json_find_str(json, len, "left", left_str, sizeof(left_str));
	json_find_str(json, len, "right", right_str, sizeof(right_str));

	/* Convert single characters to VCC codes */
	uint8_t lc = VCC_SPACE, rc = VCC_SPACE;

	if (left_str[0]) {
		lc = ascii_to_vcc_web(left_str[0]);
	}
	if (right_str[0]) {
		rc = ascii_to_vcc_web(right_str[0]);
	}

	return display_ctrl_set_segment(disp, lc, rc);
}

/* POST /api/debug/display/text
 * Body: {"text":"Ab Cd"} — up to 6 chars, spread across 3 displays */
static int apply_display_text_json(const char *json, size_t len)
{
	char text[8] = {0};

	if (!display_ctrl_ready()) {
		return -ENODEV;
	}

	if (json_find_str(json, len, "text", text, sizeof(text)) <= 0) {
		return -EINVAL;
	}

	return display_ctrl_show_status(text);
}

/* POST /api/debug/display/sysled
 * Body: {"led":0, "state":3}   (state: 0=off,1=blink1,2=blink2,3=on) */
static int apply_display_sysled_json(const char *json, size_t len)
{
	int32_t led = -1, state = -1;

	if (!display_ctrl_ready()) {
		return -ENODEV;
	}

	if (!json_find_int(json, len, "led", &led) ||
	    !json_find_int(json, len, "state", &state)) {
		return -EINVAL;
	}
	if (led < 0 || led >= DC_SYSLED_LAST || state < 0 || state > 4) {
		return -EINVAL;
	}

	return display_ctrl_set_sys_led(led, state);
}

/* POST /api/debug/display/chnled
 * Body: {"channel":0, "type":0, "on":true}
 *   type: 0=mute, 1=signal, 2=clip, 3=phantom
 * OR set all at once:
 * Body: {"type":0, "pattern":255}  — bitmask of channels */
static int apply_display_chnled_json(const char *json, size_t len)
{
	int32_t channel = -1, type = -1, pattern = -1;
	bool on = false;

	if (!display_ctrl_ready()) {
		return -ENODEV;
	}

	if (!json_find_int(json, len, "type", &type)) {
		return -EINVAL;
	}
	if (type < 0 || type >= DC_CHNLED_LAST) {
		return -EINVAL;
	}

	/* Bulk mode: set entire pattern */
	if (json_find_int(json, len, "pattern", &pattern)) {
		return display_ctrl_set_channel_leds_by_type(type,
							     (uint32_t)pattern);
	}

	/* Single channel mode */
	if (!json_find_int(json, len, "channel", &channel)) {
		return -EINVAL;
	}
	if (channel < 0 || channel >= DC_MAX_CHANNELS) {
		return -EINVAL;
	}
	json_find_bool(json, len, "on", &on);

	return display_ctrl_set_channel_led(channel, type, on);
}

#endif /* CONFIG_DISPLAY_CTRL */

/* ================================================================
 * Dynamic handler for /api/ wildcard
 *
 * Dispatches based on client->url_buffer and client->method.
 * ================================================================ */

static int api_handler(struct http_client_ctx *client,
		       enum http_data_status status,
		       const struct http_request_ctx *request_ctx,
		       struct http_response_ctx *response_ctx,
		       void *user_data)
{
	const char *url = (const char *)client->url_buffer;
	enum http_method method = client->method;

	/* Firmware update needs streaming (body too large to buffer).
	 * Delegate immediately, including abort notifications. */
	if (strcmp(url, "/api/fw_update") == 0) {
		return fw_update_http_handler(client, status,
					      request_ctx, response_ctx);
	}

	if (status == HTTP_SERVER_DATA_ABORTED) {
		post_body_len = 0;
		return 0;
	}

	/* ----- Accumulate POST body ----- */
	if ((method == HTTP_POST || method == HTTP_DELETE) &&
	    status != HTTP_SERVER_DATA_FINAL &&
	    request_ctx->data_len > 0) {
		size_t space = POST_BODY_MAX - post_body_len;
		size_t copy = (request_ctx->data_len < space) ?
			       request_ctx->data_len : space;
		memcpy(post_body + post_body_len, request_ctx->data, copy);
		post_body_len += copy;

		/* Not final yet, don't respond */
		response_ctx->final_chunk = false;
		return 0;
	}

	/* Accumulate final chunk too */
	if ((method == HTTP_POST || method == HTTP_DELETE) &&
	    request_ctx->data_len > 0) {
		size_t space = POST_BODY_MAX - post_body_len;
		size_t copy = (request_ctx->data_len < space) ?
			       request_ctx->data_len : space;
		memcpy(post_body + post_body_len, request_ctx->data, copy);
		post_body_len += copy;
	}

	/* ----- GET endpoints ----- */
	int json_len = 0;

	static const struct http_header json_hdrs[] = {
		{.name = "Content-Type", .value = "application/json"},
		{.name = "Access-Control-Allow-Origin", .value = "*"},
	};

	if (method == HTTP_GET) {
		if (strcmp(url, "/api/status") == 0) {
			json_len = build_full_status(json_buf,
						     JSON_BUF_SIZE);
		} else if (strcmp(url, "/api/status/network") == 0) {
			json_len = build_status_network(json_buf,
							JSON_BUF_SIZE);
		} else if (strcmp(url, "/api/status/ptp") == 0) {
			json_len = build_status_ptp(json_buf, JSON_BUF_SIZE);
		} else if (strcmp(url, "/api/status/fpga") == 0) {
			json_len = build_status_fpga(json_buf, JSON_BUF_SIZE);
		} else if (strcmp(url, "/api/status/streams") == 0) {
			json_len = build_status_streams(json_buf,
							JSON_BUF_SIZE);
		} else if (strcmp(url, "/api/config") == 0) {
			json_len = build_config_json(json_buf, JSON_BUF_SIZE);
#ifdef CONFIG_MI_CARD
		} else if (strcmp(url, "/api/mi") == 0) {
			json_len = build_mi_status(json_buf, JSON_BUF_SIZE);
		} else if (strcmp(url, "/api/mi/reset") == 0) {
			/* GET /api/mi/reset returns board info */
			struct mi_board_info info;
			if (mi_card_detect(&info)) {
				json_len = snprintf(json_buf, JSON_BUF_SIZE,
					"{\"soft_id\":%d,\"board_id\":%d,\"rev\":%d}",
					info.soft_id, info.board_id, info.hard_rev);
			} else {
				json_len = snprintf(json_buf, JSON_BUF_SIZE,
					"{\"error\":\"board not detected\"}");
				response_ctx->status = HTTP_503_SERVICE_UNAVAILABLE;
			}
#endif
#ifdef CONFIG_LO_CARD
		} else if (strcmp(url, "/api/lo") == 0) {
			json_len = build_lo_status(json_buf, JSON_BUF_SIZE);
		} else if (strcmp(url, "/api/lo/reset") == 0) {
			/* GET /api/lo/reset returns board info */
			struct lo_board_info info;
			if (lo_card_detect(&info)) {
				json_len = snprintf(json_buf, JSON_BUF_SIZE,
					"{\"soft_id\":%d,\"board_id\":%d,\"rev\":%d}",
					info.soft_id, info.board_id, info.hard_rev);
			} else {
				json_len = snprintf(json_buf, JSON_BUF_SIZE,
					"{\"error\":\"board not detected\"}");
				response_ctx->status = HTTP_503_SERVICE_UNAVAILABLE;
			}
#endif
		/* ---- Card Manager ---- */
		} else if (strcmp(url, "/api/cards") == 0) {
			json_len = build_cards_overview(json_buf, JSON_BUF_SIZE);
		} else if (strcmp(url, "/api/cards/scan") == 0) {
			/* GET triggers a fresh scan AND returns result */
			card_manager_rescan();
			json_len = build_cards_overview(json_buf, JSON_BUF_SIZE);
#ifdef CONFIG_IO_CARD
		} else if (strcmp(url, "/api/cards/io") == 0) {
			const struct card_info *ci = card_manager_get_info(CARD_SLOT_MAIN);
			if (ci && ci->type == CARD_TYPE_IO && ci->initialized) {
				json_len = build_io_card_status(json_buf, JSON_BUF_SIZE);
			} else {
				json_len = snprintf(json_buf, JSON_BUF_SIZE,
					"{\"error\":\"IO card not present\"}");
				response_ctx->status = HTTP_503_SERVICE_UNAVAILABLE;
			}
#endif
#ifdef CONFIG_DISPLAY_CTRL
		/* ---- Display Debug ---- */
		} else if (strcmp(url, "/api/debug/display") == 0) {
			json_len = build_display_debug_status(json_buf,
							      JSON_BUF_SIZE);
#endif
		} else if (strcmp(url, "/api/system") == 0) {
			/* GET /api/system - returns system status including SD */
			int p = json_start_object(json_buf, JSON_BUF_SIZE);
#ifdef CONFIG_SD_CONFIG
			p = json_add_bool(json_buf, JSON_BUF_SIZE, p, "sd_mounted",
					   sd_config_is_ready());
			p = json_add_str(json_buf, JSON_BUF_SIZE, p, "sd_config_status",
					  sd_config_status_str(sd_config_get_load_status()));
#else
			p = json_add_bool(json_buf, JSON_BUF_SIZE, p, "sd_mounted", false);
			p = json_add_str(json_buf, JSON_BUF_SIZE, p, "sd_config_status", "disabled");
#endif
			p = json_add_str(json_buf, JSON_BUF_SIZE, p, "version", "1.0.0");
			p = json_end_object(json_buf, JSON_BUF_SIZE, p);
			json_len = p;
		} else {
			/* 404 */
			json_len = snprintf(json_buf, JSON_BUF_SIZE,
					    "{\"error\":\"not found\"}");
			response_ctx->status = HTTP_404_NOT_FOUND;
		}

		if (response_ctx->status == 0) {
			response_ctx->status = HTTP_200_OK;
		}
		response_ctx->body = (const uint8_t *)json_buf;
		response_ctx->body_len = json_len;
		response_ctx->final_chunk = true;
		response_ctx->headers = json_hdrs;
		response_ctx->header_count = ARRAY_SIZE(json_hdrs);
		return 0;
	}

	/* ----- POST endpoints ----- */
	if (method == HTTP_POST && status == HTTP_SERVER_DATA_FINAL) {
		int ret = 0;

		if (strcmp(url, "/api/config") == 0) {
			ret = apply_config_json(post_body, post_body_len);
			if (ret == 0) {
				json_len = build_config_json(json_buf,
							    JSON_BUF_SIZE);
#ifdef CONFIG_SD_CONFIG
				sd_config_save();
#endif
			}
		} else if (strcmp(url, "/api/streams/tx") == 0) {
			ret = apply_tx_stream_json(post_body, post_body_len);
			if (ret == 0) {
				json_len = snprintf(json_buf, JSON_BUF_SIZE,
						    "{\"ok\":true}");
#ifdef CONFIG_SD_CONFIG
				sd_config_save();
#endif
			}
		} else if (strcmp(url, "/api/streams/rx") == 0) {
			ret = apply_rx_stream_json(post_body, post_body_len);
			if (ret == 0) {
				json_len = snprintf(json_buf, JSON_BUF_SIZE,
						    "{\"ok\":true}");
#ifdef CONFIG_SD_CONFIG
				sd_config_save();
#endif
			}
#ifdef CONFIG_MI_CARD
		} else if (strcmp(url, "/api/mi") == 0) {
			/* Global MI settings (HPF, 96kHz) */
			ret = apply_mi_global_json(post_body, post_body_len);
			if (ret == 0) {
				json_len = build_mi_status(json_buf,
							   JSON_BUF_SIZE);
			}
		} else if (strncmp(url, "/api/mi/channel/", 16) == 0) {
			/* Per-channel settings: /api/mi/channel/0 */
			int ch = atoi(url + 16);
			ret = apply_mi_channel_json(ch, post_body,
						    post_body_len);
			if (ret == 0) {
				json_len = build_mi_status(json_buf,
							   JSON_BUF_SIZE);
			}
		} else if (strcmp(url, "/api/mi/reset") == 0) {
			/* POST /api/mi/reset triggers hardware reset */
#ifdef CONFIG_MI_CARD_NRST_GPIO
			ret = mi_card_hw_reset();
#else
			ret = mi_card_reset();
#endif
			if (ret == 0) {
				json_len = snprintf(json_buf, JSON_BUF_SIZE,
					"{\"ok\":true,\"message\":\"Board reset complete\"}");
			}
#endif
#ifdef CONFIG_LO_CARD
		} else if (strcmp(url, "/api/lo") == 0) {
			/* Global LO settings (96kHz, output enable) */
			ret = apply_lo_global_json(post_body, post_body_len);
			if (ret == 0) {
				json_len = build_lo_status(json_buf,
							   JSON_BUF_SIZE);
			}
		} else if (strncmp(url, "/api/lo/channel/", 16) == 0) {
			/* Per-channel settings: /api/lo/channel/0 */
			int ch = atoi(url + 16);
			ret = apply_lo_channel_json(ch, post_body,
						    post_body_len);
			if (ret == 0) {
				json_len = build_lo_status(json_buf,
							   JSON_BUF_SIZE);
			}
		} else if (strcmp(url, "/api/lo/reset") == 0) {
			/* POST /api/lo/reset triggers reset */
#ifdef CONFIG_LO_CARD_NRST_GPIO
			ret = lo_card_hw_reset();
#else
			ret = lo_card_reset();
#endif
			if (ret == 0) {
				json_len = snprintf(json_buf, JSON_BUF_SIZE,
					"{\"ok\":true,\"message\":\"Board reset complete\"}");
			}
#endif
		/* ---- Card Manager POST ---- */
		} else if (strcmp(url, "/api/cards/scan") == 0) {
			/* POST /api/cards/scan — trigger rescan, return new state */
			card_manager_rescan();
			json_len = build_cards_overview(json_buf, JSON_BUF_SIZE);
#ifdef CONFIG_IO_CARD
		} else if (strcmp(url, "/api/cards/io") == 0) {
			/* POST /api/cards/io — global IO card settings */
			const struct card_info *ci = card_manager_get_info(CARD_SLOT_MAIN);
			if (ci && ci->type == CARD_TYPE_IO && ci->initialized) {
				ret = apply_io_global_json(post_body, post_body_len);
				if (ret == 0) {
					json_len = build_io_card_status(json_buf, JSON_BUF_SIZE);
				}
			} else {
				ret = -ENODEV;
			}
		} else if (strncmp(url, "/api/cards/io/in/", 17) == 0) {
			/* POST /api/cards/io/in/0 — per input-channel settings */
			int ch = atoi(url + 17);
			const struct card_info *ci = card_manager_get_info(CARD_SLOT_MAIN);
			if (ci && ci->type == CARD_TYPE_IO && ci->initialized) {
				ret = apply_io_in_channel_json(ch, post_body, post_body_len);
				if (ret == 0) {
					json_len = build_io_card_status(json_buf, JSON_BUF_SIZE);
				}
			} else {
				ret = -ENODEV;
			}
		} else if (strncmp(url, "/api/cards/io/out/", 18) == 0) {
			/* POST /api/cards/io/out/0 — per output-channel settings */
			int ch = atoi(url + 18);
			const struct card_info *ci = card_manager_get_info(CARD_SLOT_MAIN);
			if (ci && ci->type == CARD_TYPE_IO && ci->initialized) {
				ret = apply_io_out_channel_json(ch, post_body, post_body_len);
				if (ret == 0) {
					json_len = build_io_card_status(json_buf, JSON_BUF_SIZE);
				}
			} else {
				ret = -ENODEV;
			}
		} else if (strcmp(url, "/api/cards/io/reset") == 0) {
			/* POST /api/cards/io/reset — reinit IO card */
			const struct card_info *ci = card_manager_get_info(CARD_SLOT_MAIN);
			if (ci && ci->type == CARD_TYPE_IO) {
				ret = io_card_reset();
				if (ret == 0) {
					json_len = snprintf(json_buf, JSON_BUF_SIZE,
						"{\"ok\":true}");
				}
			} else {
				ret = -ENODEV;
			}
#endif /* CONFIG_IO_CARD */
#ifdef CONFIG_DISPLAY_CTRL
		/* ---- Display Debug POST ---- */
		} else if (strcmp(url, "/api/debug/display/segment") == 0) {
			ret = apply_display_segment_json(post_body,
							 post_body_len);
			if (ret == 0) {
				json_len = snprintf(json_buf, JSON_BUF_SIZE,
					"{\"ok\":true}");
			}
		} else if (strcmp(url, "/api/debug/display/sysled") == 0) {
			ret = apply_display_sysled_json(post_body,
							post_body_len);
			if (ret == 0) {
				json_len = snprintf(json_buf, JSON_BUF_SIZE,
					"{\"ok\":true}");
			}
		} else if (strcmp(url, "/api/debug/display/chnled") == 0) {
			ret = apply_display_chnled_json(post_body,
							post_body_len);
			if (ret == 0) {
				json_len = snprintf(json_buf, JSON_BUF_SIZE,
					"{\"ok\":true}");
			}
		} else if (strcmp(url, "/api/debug/display/test") == 0) {
			ret = display_ctrl_full_test();
			if (ret == 0) {
				json_len = snprintf(json_buf, JSON_BUF_SIZE,
					"{\"ok\":true,\"message\":\"All LEDs on, 888888\"}");
			}
		} else if (strcmp(url, "/api/debug/display/clear") == 0) {
			ret = display_ctrl_all_off();
			if (ret == 0) {
				json_len = snprintf(json_buf, JSON_BUF_SIZE,
					"{\"ok\":true,\"message\":\"All off\"}");
			}
		} else if (strcmp(url, "/api/debug/display/reset") == 0) {
#if defined(CONFIG_DISPLAY_CTRL_NRST_GPIO) || defined(CONFIG_DISPLAY_CTRL_NRST_HAL)
			ret = display_ctrl_hw_reset();
			if (ret == 0) {
				/* Re-init display UART after hw reset */
				display_ctrl_init(display_ctrl_get_uart());
				json_len = snprintf(json_buf, JSON_BUF_SIZE,
					"{\"ok\":true,\"message\":"
					"\"Shared nRST reset complete\"}");
			}
#else
			ret = -ENOTSUP;
#endif
		} else if (strcmp(url, "/api/debug/display/text") == 0) {
			ret = apply_display_text_json(post_body,
						      post_body_len);
			if (ret == 0) {
				json_len = snprintf(json_buf, JSON_BUF_SIZE,
					"{\"ok\":true}");
			}
#endif /* CONFIG_DISPLAY_CTRL */
		} else if (strcmp(url, "/api/system/reboot") == 0) {
			/* POST /api/system/reboot - reboot MCU */
			LOG_WRN("Reboot requested via REST API");
			json_len = snprintf(json_buf, JSON_BUF_SIZE,
				"{\"ok\":true,\"message\":\"Rebooting...\"}");
			/* Send response before rebooting */
			response_ctx->status = HTTP_200_OK;
			response_ctx->body = (const uint8_t *)json_buf;
			response_ctx->body_len = json_len;
			response_ctx->final_chunk = true;
			response_ctx->headers = json_hdrs;
			response_ctx->header_count = ARRAY_SIZE(json_hdrs);
			/* Schedule reboot after short delay to allow response */
			k_msleep(100);
			sys_reboot(SYS_REBOOT_COLD);
			return 0;
#ifdef CONFIG_SD_CONFIG
		} else if (strcmp(url, "/api/sd/format") == 0) {
			/* POST /api/sd/format - format SD card */
			ret = sd_config_format();
			if (ret == 0) {
				json_len = snprintf(json_buf, JSON_BUF_SIZE,
					"{\"ok\":true,\"message\":\"SD card formatted\"}");
			}
#endif
		} else {
			json_len = snprintf(json_buf, JSON_BUF_SIZE,
					    "{\"error\":\"not found\"}");
			response_ctx->status = HTTP_404_NOT_FOUND;
		}

		post_body_len = 0;

		if (ret < 0) {
			json_len = snprintf(json_buf, JSON_BUF_SIZE,
					    "{\"error\":\"invalid data\","
					    "\"code\":%d}", ret);
			response_ctx->status = HTTP_400_BAD_REQUEST;
		}

		if (response_ctx->status == 0) {
			response_ctx->status = HTTP_200_OK;
		}
		response_ctx->body = (const uint8_t *)json_buf;
		response_ctx->body_len = json_len;
		response_ctx->final_chunk = true;
		response_ctx->headers = json_hdrs;
		response_ctx->header_count = ARRAY_SIZE(json_hdrs);
		return 0;
	}

	/* ----- DELETE endpoints ----- */
	if (method == HTTP_DELETE && status == HTTP_SERVER_DATA_FINAL) {
		/* URL like /api/streams/tx/3 */
		int ret = -EINVAL;

		if (strncmp(url, "/api/streams/tx/", 16) == 0) {
			int sid = atoi(url + 16);

			ret = delete_tx_stream(sid);
#ifdef CONFIG_SD_CONFIG
			if (ret == 0) {
				sd_config_save();
			}
#endif
		} else if (strncmp(url, "/api/streams/rx/", 16) == 0) {
			int sid = atoi(url + 16);

			ret = delete_rx_stream(sid);
#ifdef CONFIG_SD_CONFIG
			if (ret == 0) {
				sd_config_save();
			}
#endif
		}

		post_body_len = 0;

		if (ret == 0) {
			json_len = snprintf(json_buf, JSON_BUF_SIZE,
					    "{\"ok\":true}");
			response_ctx->status = HTTP_200_OK;
		} else {
			json_len = snprintf(json_buf, JSON_BUF_SIZE,
					    "{\"error\":\"invalid\","
					    "\"code\":%d}", ret);
			response_ctx->status = HTTP_400_BAD_REQUEST;
		}

		response_ctx->body = (const uint8_t *)json_buf;
		response_ctx->body_len = json_len;
		response_ctx->final_chunk = true;
		response_ctx->headers = json_hdrs;
		response_ctx->header_count = ARRAY_SIZE(json_hdrs);
		return 0;
	}

	/* ----- OPTIONS (CORS preflight) ----- */
	if (method == HTTP_OPTIONS) {
		static const struct http_header cors_hdrs[] = {
			{.name = "Access-Control-Allow-Origin", .value = "*"},
			{.name = "Access-Control-Allow-Methods",
			 .value = "GET,POST,DELETE,OPTIONS"},
			{.name = "Access-Control-Allow-Headers",
			 .value = "Content-Type"},
		};
		response_ctx->status = HTTP_200_OK;
		response_ctx->body = (const uint8_t *)"";
		response_ctx->body_len = 0;
		response_ctx->final_chunk = true;
		response_ctx->headers = cors_hdrs;
		response_ctx->header_count = ARRAY_SIZE(cors_hdrs);
		return 0;
	}

	return 0;
}

/* ================================================================
 * API resource registration (wildcard matches /api/...)
 * ================================================================ */

static struct http_resource_detail_dynamic api_resource_detail = {
	.common = {
		.type = HTTP_RESOURCE_TYPE_DYNAMIC,
		.bitmask_of_supported_http_methods =
			BIT(HTTP_GET) | BIT(HTTP_POST) |
			BIT(HTTP_DELETE) | BIT(HTTP_OPTIONS),
		.content_type = "application/json",
	},
	.cb = api_handler,
	.user_data = NULL,
};

HTTP_RESOURCE_DEFINE(api_resource, aes67_http, "/api/*",
		     &api_resource_detail);

/* ================================================================
 * Static web UI — served as gzipped blob from /
 * The .gz.inc file is generated at build time by CMake.
 * ================================================================ */

static const uint8_t index_html_gz[] = {
#include "index.html.gz.inc"
};

static struct http_resource_detail_static index_resource_detail = {
	.common = {
		.type = HTTP_RESOURCE_TYPE_STATIC,
		.bitmask_of_supported_http_methods = BIT(HTTP_GET),
		.content_encoding = "gzip",
		.content_type = "text/html",
	},
	.static_data = index_html_gz,
	.static_data_len = sizeof(index_html_gz),
};

HTTP_RESOURCE_DEFINE(index_resource, aes67_http, "/",
		     &index_resource_detail);

/* ================================================================
 * Debug page — display & LED tester served from /debug
 * ================================================================ */

#ifdef CONFIG_DISPLAY_CTRL
static const uint8_t debug_html_gz[] = {
#include "debug.html.gz.inc"
};

static struct http_resource_detail_static debug_resource_detail = {
	.common = {
		.type = HTTP_RESOURCE_TYPE_STATIC,
		.bitmask_of_supported_http_methods = BIT(HTTP_GET),
		.content_encoding = "gzip",
		.content_type = "text/html",
	},
	.static_data = debug_html_gz,
	.static_data_len = sizeof(debug_html_gz),
};

HTTP_RESOURCE_DEFINE(debug_resource, aes67_http, "/debug",
		     &debug_resource_detail);
#endif /* CONFIG_DISPLAY_CTRL */

/* ================================================================
 * Public API
 * ================================================================ */

int webserver_start(void)
{
	/* Initialize config defaults before first use */
	aes67_config_get();

	int ret = http_server_start();

	if (ret < 0) {
		LOG_ERR("WEB: Failed to start HTTP server: %d", ret);
		return ret;
	}

	LOG_INF("WEB: HTTP server started on port %u", http_port);
	return 0;
}
