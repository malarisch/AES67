/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: Apache-2.0
 *
 * RAVENNA mDNS / DNS-SD service advertisement.
 *
 * Registers the device's HTTP and RTSP services with DNS-SD
 * per RAVENNA Operating Principles §3.5.1, including:
 *   - <vendor node id>._http._tcp.local
 *   - <vendor node id>._rtsp._tcp.local
 *   - <user defined node name>._http._tcp.local
 *   - <user defined node name>._rtsp._tcp.local
 *   - <vendor node id>._ravenna._sub._http._tcp.local  (sub-type)
 *   - <vendor node id>._ravenna._sub._rtsp._tcp.local  (sub-type)
 *
 * Streaming sessions may be advertised individually.
 */

#ifndef MDNS_SD_H_
#define MDNS_SD_H_

#include <zephyr/kernel.h>
#include <zephyr/net/net_ip.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Start the mDNS/DNS-SD subsystem.
 *
 * Registers the basic RAVENNA service advertisements (HTTP + RTSP)
 * using both the vendor node ID and the user-defined device name.
 *
 * Must be called after the network interface is up.
 *
 * @return 0 on success, negative errno on error
 */
int mdns_sd_start(void);

/**
 * @brief Update DNS-SD records after device name change.
 *
 * Call when the user-defined device name changes at runtime
 * to re-register services with the new name.
 */
void mdns_sd_update_device_name(void);

/**
 * @brief Advertise a streaming session via DNS-SD.
 *
 * Per RAVENNA §3.5.2, advertises a named session as:
 *   <session_name>._rtsp._tcp.local
 * with the "ravenna_session" sub-type.
 *
 * @param session_name  Unique session name
 * @param rtsp_port     RTSP port for this session
 * @return 0 on success, negative errno on error
 */
int mdns_sd_advertise_session(const char *session_name, uint16_t rtsp_port);

/**
 * @brief Advertise/withdraw a session keyed by TX stream slot.
 *
 * Slot-based variant used by the TX stream configuration path: a rename
 * withdraws the old instance (goodbye) and announces the new one.
 *
 * @param slot          TX stream id (0..AES67_MAX_TX_STREAMS-1)
 * @param session_name  Session name, or NULL/"" to withdraw
 * @return 0 on success, negative errno on error
 */
int mdns_sd_set_session(uint8_t slot, const char *session_name);

/**
 * @brief Remove a streaming session advertisement.
 *
 * @param session_name  Session name to remove
 * @return 0 on success, negative errno on error
 */
int mdns_sd_remove_session(const char *session_name);

#ifdef CONFIG_NMOS_REGISTRATION
/* One usable NMOS Registration API discovered via mDNS ("usable" =
 * SRV+A+TXT complete, api_proto=http, api_ver contains v1.3). */
struct mdns_nmos_registry {
	struct in_addr ip;
	uint16_t port;
	uint8_t pri;   /* TXT pri — lower wins */
};

/**
 * @brief Snapshot the usable NMOS registries, sorted by priority.
 *
 * @return Number of entries written to out (0 = peer-to-peer operation)
 */
int mdns_sd_get_nmos_registries(struct mdns_nmos_registry *out, int max);
#endif

#ifdef __cplusplus
}
#endif

#endif /* MDNS_SD_H_ */
