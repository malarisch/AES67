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
#include "ptp_bmc.h"
#include "sap_sdp.h"
#include "ui_display.h"
#include "../drivers/eth_fmc_basic/eth_fmc_basic.h"
#ifdef CONFIG_MI_CARD
#include "../drivers/mi_card/mi_card.h"
#endif
#ifdef CONFIG_LO_CARD
#include "../drivers/lo_card/lo_card.h"
#endif

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
	const struct device *fmc = device_get_binding("eth_fmc0");
	uint8_t status, eth_status;
	int ret;

	if (!fmc) {
		return -ENODEV;
	}

	ret = eth_fmc_reg_read(fmc, ETH_FMC_REG_STATUS_CLK, &status);
	if (ret < 0) {
		return ret;
	}

	m->ppb_valid        = !!(status & ETH_FMC_CLK_PPB_VALID);
	m->wc_locked        = !!(status & ETH_FMC_CLK_WC_LOCKED);
	m->wc_phasejump     = !!(status & ETH_FMC_CLK_WC_PHASEJUMP);
	m->wc_configured    = !!(status & ETH_FMC_CLK_WC_CONFIGURED);
	m->ptp_leader_lost  = !!(status & ETH_FMC_CLK_PTP_LEADER_LOST);

	ret = eth_fmc_reg_read(fmc, ETH_FMC_REG_STATUS_ETH, &eth_status);
	if (ret == 0) {
		m->link_up    = !!(eth_status & ETH_FMC_ETH_LINK_UP);
		m->speed_code = (eth_status & ETH_FMC_ETH_SPEED_MASK) >>
				ETH_FMC_ETH_SPEED_SHIFT;
	}

	/* 32-bit reads */
	uint8_t buf[4];

	ret = eth_fmc_reg_read_block(fmc, ETH_FMC_REG_PATH_DELAY, buf, 4);
	if (ret == 0) {
		m->path_delay_ns = (int32_t)((uint32_t)buf[0] |
				   ((uint32_t)buf[1] << 8) |
				   ((uint32_t)buf[2] << 16) |
				   ((uint32_t)buf[3] << 24));
	}

	ret = eth_fmc_reg_read_block(fmc, ETH_FMC_REG_LEADER_OFFSET, buf, 4);
	if (ret == 0) {
		m->leader_offset_ns = (int32_t)((uint32_t)buf[0] |
					((uint32_t)buf[1] << 8) |
					((uint32_t)buf[2] << 16) |
					((uint32_t)buf[3] << 24));
	}

	/* Read raw counters and calculate PPB (0x54 = count_wc, 0x55 = count_pll) */
	uint32_t count_wc = 0, count_pll = 0;

	ret = eth_fmc_reg_read_block(fmc, ETH_FMC_REG_COUNT_WC, buf, 4);
	if (ret == 0) {
		count_wc = ((uint32_t)buf[0] |
			    ((uint32_t)buf[1] << 8) |
			    ((uint32_t)buf[2] << 16)) & 0x3FFFFF;
	}
	ret = eth_fmc_reg_read_block(fmc, ETH_FMC_REG_COUNT_PLL, buf, 4);
	if (ret == 0) {
		count_pll = ((uint32_t)buf[0] |
			     ((uint32_t)buf[1] << 8) |
			     ((uint32_t)buf[2] << 16)) & 0x3FFFFF;
	}

	/* Calculate PPB: (count_pll - count_wc) * 1e9 / count_wc */
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
		const char *speed =
			(m.speed_code == 0) ? "10M" :
			(m.speed_code == 1) ? "100M" :
			(m.speed_code == 2) ? "1G" : "unknown";
		p = json_add_str(buf, sz, p, "phy_speed", speed);
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

		const char *speed =
			(m.speed_code == 0) ? "10M" :
			(m.speed_code == 1) ? "100M" :
			(m.speed_code == 2) ? "1G" : "unknown";
		p = json_add_str(buf, sz, p, "phy_speed", speed);
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

	if (!json_find_int(json, len, "stream_id", &stream_id) ||
	    stream_id < 0 || stream_id > 7) {
		return -EINVAL;
	}

	if (json_find_str(json, len, "dst_ip", dst_ip_str,
			  sizeof(dst_ip_str)) <= 0) {
		return -EINVAL;
	}

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
					   0); /* SSRC: auto-generate */
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
					   0, 0, zero_ch, 0, 0);
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
 * POST body accumulator
 * ================================================================ */

#define POST_BODY_MAX 1024
static char post_body[POST_BODY_MAX];
static size_t post_body_len;

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
	if (status == HTTP_SERVER_DATA_ABORTED) {
		post_body_len = 0;
		return 0;
	}

	const char *url = (const char *)client->url_buffer;
	enum http_method method = client->method;

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
			}
		} else if (strcmp(url, "/api/streams/tx") == 0) {
			ret = apply_tx_stream_json(post_body, post_body_len);
			if (ret == 0) {
				json_len = snprintf(json_buf, JSON_BUF_SIZE,
						    "{\"ok\":true}");
			}
		} else if (strcmp(url, "/api/streams/rx") == 0) {
			ret = apply_rx_stream_json(post_body, post_body_len);
			if (ret == 0) {
				json_len = snprintf(json_buf, JSON_BUF_SIZE,
						    "{\"ok\":true}");
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
		} else if (strncmp(url, "/api/streams/rx/", 16) == 0) {
			int sid = atoi(url + 16);

			ret = delete_rx_stream(sid);
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
