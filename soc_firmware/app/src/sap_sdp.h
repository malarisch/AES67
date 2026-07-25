/*
 * Session Announcement Protocol (SAP, RFC 2974) transport.
 *
 * TX: Periodically announces the active TX streams (from aes67_conn)
 *     via SAP multicast, one announcement per stream, with an
 *     AES67-compliant SDP body (built by aes67_sdp_utils).
 * RX: Listens for foreign SAP announcements and reports them into the
 *     aes67_conn foreign-stream registry.
 *
 * Stream state, FPGA configuration and IGMP proxy joins live in
 * aes67_conn — this module is transport only.
 */

#ifndef SAP_SDP_H_
#define SAP_SDP_H_

#include <zephyr/net/net_if.h>
#include <zephyr/net/net_ip.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- SAP constants (RFC 2974) ---- */

#define SAP_MULTICAST_ADDR  "239.255.255.255"
#define SAP_PORT            9875
#define SAP_HEADER_SIZE     8   /* V1 IPv4 header without auth */

/**
 * @brief Start the SAP announcer/listener thread.
 *
 * Announce interval and enable flag are read live from the device
 * config (sap_announce_interval_s / sap_announce_enabled).
 *
 * @param iface  Network interface to use
 * @return 0 on success, negative errno on error
 */
int sap_sdp_start(struct net_if *iface);

/**
 * @brief Notify the SAP module that a valid IP has been assigned.
 *
 * Must be called after DHCP binds so the SDP origin address and
 * SAP originating source are correct.
 */
void sap_sdp_notify_ip_ready(const struct in_addr *addr);

/**
 * @brief Notify the SAP module that Ethernet link has come up.
 *        Re-joins the SAP multicast group to re-establish IGMP membership.
 */
void sap_sdp_notify_link_up(void);

/**
 * @brief Enable or disable SAP announcements (persisted device config).
 */
void sap_sdp_set_announce(bool enable);

/**
 * @brief Trigger an immediate announcement round (e.g. after a stream
 *        configuration change).
 */
void sap_sdp_trigger_announce(void);

#ifdef __cplusplus
}
#endif

#endif /* SAP_SDP_H_ */
