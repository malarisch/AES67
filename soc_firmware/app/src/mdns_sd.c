/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: Apache-2.0
 *
 * RAVENNA mDNS / DNS-SD service advertisement.
 *
 * Implements RAVENNA Operating Principles §3.5.1:
 *   - Vendor node ID services (_http._tcp, _rtsp._tcp)
 *   - User-defined device name services (_http._tcp, _rtsp._tcp)
 *
 * Uses Zephyr's built-in mDNS responder (RFC 6762) and DNS-SD
 * (RFC 6763) support.  Static services are registered at build time
 * via DNS_SD_REGISTER_SERVICE; dynamic (runtime) records use
 * mdns_responder_set_ext_records().
 *
 * Note: RAVENNA _ravenna._sub._http._tcp sub-types are not yet
 * supported by Zephyr's DNS-SD implementation (sub-type queries
 * require upstream changes).  The main service types are sufficient
 * for standard DNS-SD browsing of RAVENNA nodes.
 *
 * Streaming session advertisement (§3.5.2) registers additional
 * _rtsp._tcp services per session name.
 */

#include <zephyr/kernel.h>
#include <zephyr/net/dns_sd.h>
#include <zephyr/net/mdns_responder.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <string.h>
#include <stdio.h>

#include "mdns_sd.h"
#include "aes67_config.h"

LOG_MODULE_REGISTER(mdns_sd, LOG_LEVEL_INF);

/*
 * Vendor node ID: "<vendor> <product> <serial>"
 * Generated from runtime config at startup.
 */
#define VENDOR_NODE_ID_MAX  64

static char vendor_node_id[VENDOR_NODE_ID_MAX];

/* ---- Port numbers (network byte order, as required by dns_sd_rec) ---- */

static const uint16_t http_port_be = sys_cpu_to_be16(80);

#ifdef CONFIG_RTSP
static const uint16_t rtsp_port_be = sys_cpu_to_be16(CONFIG_RTSP_PORT);
#endif

/* ---- DNS TXT records ---- */

/*
 * RAVENNA §3.5.1.1: Service text MAY provide node type info.
 * Format per RFC 6763 §6: length-prefixed key=value pairs.
 */
static const char ravenna_txt[] =
	"\x0c" "type=ravenna";

/* ---- Static DNS-SD services (vendor node ID, known at boot) ---- */

/*
 * We use DNS_SD_REGISTER_SERVICE for the vendor node ID services
 * since the vendor ID is a compile-time constant.  The instance
 * name points to vendor_node_id[] which is filled in mdns_sd_start().
 */

/* ---- Dynamic DNS-SD records (runtime) ---- */

/*
 * For user-defined device name + streaming sessions, we use
 * mdns_responder_set_ext_records() since these can change at runtime.
 */

#define MAX_SESSION_RECORDS  8
#define SESSION_NAME_MAX     32

/* Record slots: [0] = user HTTP, [1] = user RTSP, [2..N] = sessions */
#define EXT_REC_USER_HTTP    0
#define EXT_REC_USER_RTSP    1
#define EXT_REC_SESSION_BASE 2
#define MAX_EXT_RECORDS      (2 + MAX_SESSION_RECORDS)

static struct dns_sd_rec ext_records[MAX_EXT_RECORDS];
static size_t ext_record_count;

/* Buffers for user-defined device name (mutable) */
static char user_device_name[AES67_DEVICE_NAME_MAX];

/* Buffers for session names */
static char session_names[MAX_SESSION_RECORDS][SESSION_NAME_MAX];
static uint16_t session_ports_be[MAX_SESSION_RECORDS];

/* ---- Internal helpers ---- */

static void build_vendor_node_id(void)
{
	aes67_config_build_node_id(vendor_node_id, sizeof(vendor_node_id));
}

static void update_ext_records(void)
{
	/* Slot 0: user device name -> _http._tcp */
	ext_records[EXT_REC_USER_HTTP] = (struct dns_sd_rec){
		.instance = user_device_name,
		.service = "_http",
		.proto = "_tcp",
		.domain = "local",
		.text = ravenna_txt,
		.text_size = sizeof(ravenna_txt) - 1,
		.port = &http_port_be,
	};

#ifdef CONFIG_RTSP
	/* Slot 1: user device name -> _rtsp._tcp */
	ext_records[EXT_REC_USER_RTSP] = (struct dns_sd_rec){
		.instance = user_device_name,
		.service = "_rtsp",
		.proto = "_tcp",
		.domain = "local",
		.text = ravenna_txt,
		.text_size = sizeof(ravenna_txt) - 1,
		.port = &rtsp_port_be,
	};
	ext_record_count = 2;
#else
	ext_record_count = 1;
#endif

	/* Re-add any active session records */
	for (int i = 0; i < MAX_SESSION_RECORDS; i++) {
		if (session_names[i][0] != '\0') {
			ext_records[ext_record_count] = (struct dns_sd_rec){
				.instance = session_names[i],
				.service = "_rtsp",
				.proto = "_tcp",
				.domain = "local",
				.text = ravenna_txt,
				.text_size = sizeof(ravenna_txt) - 1,
				.port = &session_ports_be[i],
			};
			ext_record_count++;
		}
	}

	int ret = mdns_responder_set_ext_records(ext_records, ext_record_count);

	if (ret < 0) {
		LOG_ERR("Failed to set ext DNS-SD records: %d", ret);
	}
}

/* ---- Public API ---- */

int mdns_sd_start(void)
{
	build_vendor_node_id();

	/* Copy current device name for DNS-SD instance */
	struct aes67_device_config *cfg = aes67_config_get();

	strncpy(user_device_name, cfg->device_name,
		sizeof(user_device_name) - 1);
	user_device_name[sizeof(user_device_name) - 1] = '\0';

	/* Clear session slots */
	memset(session_names, 0, sizeof(session_names));
	memset(session_ports_be, 0, sizeof(session_ports_be));

	/* Register dynamic records (user device name + sessions) */
	update_ext_records();

	LOG_INF("mDNS/DNS-SD started: vendor=\"%s\" user=\"%s\"",
		vendor_node_id, user_device_name);
	LOG_INF("  HTTP on port 80, RTSP on port %u",
		(unsigned)CONFIG_RTSP_PORT);

	return 0;
}

void mdns_sd_update_device_name(void)
{
	struct aes67_device_config *cfg = aes67_config_get();

	strncpy(user_device_name, cfg->device_name,
		sizeof(user_device_name) - 1);
	user_device_name[sizeof(user_device_name) - 1] = '\0';

	update_ext_records();

	LOG_INF("mDNS/DNS-SD: device name updated to \"%s\"",
		user_device_name);
}

int mdns_sd_advertise_session(const char *session_name, uint16_t rtsp_port)
{
	if (!session_name || session_name[0] == '\0') {
		return -EINVAL;
	}

	/* Find a free slot */
	int slot = -1;

	for (int i = 0; i < MAX_SESSION_RECORDS; i++) {
		if (session_names[i][0] == '\0') {
			slot = i;
			break;
		}
	}

	if (slot < 0) {
		LOG_WRN("mDNS: No free session slots");
		return -ENOMEM;
	}

	strncpy(session_names[slot], session_name, SESSION_NAME_MAX - 1);
	session_names[slot][SESSION_NAME_MAX - 1] = '\0';
	session_ports_be[slot] = sys_cpu_to_be16(rtsp_port);

	update_ext_records();

	LOG_INF("mDNS: Advertised session \"%s\" on port %u",
		session_name, rtsp_port);
	return 0;
}

int mdns_sd_remove_session(const char *session_name)
{
	if (!session_name) {
		return -EINVAL;
	}

	for (int i = 0; i < MAX_SESSION_RECORDS; i++) {
		if (strcmp(session_names[i], session_name) == 0) {
			session_names[i][0] = '\0';
			session_ports_be[i] = 0;
			update_ext_records();
			LOG_INF("mDNS: Removed session \"%s\"", session_name);
			return 0;
		}
	}

	return -ENOENT;
}

/*
 * Static DNS-SD registrations for vendor node ID.
 *
 * These are picked up by the linker and automatically served by
 * Zephyr's mDNS responder.  The instance name points to the
 * vendor_node_id[] buffer which is filled at startup.
 */

DNS_SD_REGISTER_SERVICE(ravenna_vendor_http,
			vendor_node_id,
			"_http", "_tcp", "local",
			ravenna_txt,
			&http_port_be);

#ifdef CONFIG_RTSP
DNS_SD_REGISTER_SERVICE(ravenna_vendor_rtsp,
			vendor_node_id,
			"_rtsp", "_tcp", "local",
			ravenna_txt,
			&rtsp_port_be);
#endif
