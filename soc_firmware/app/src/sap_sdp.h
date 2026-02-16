/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: Apache-2.0
 *
 * AES67 Session Announcement Protocol (SAP, RFC 2974) and
 * Session Description Protocol (SDP, RFC 4566) implementation.
 *
 * TX: Periodically announces our audio stream via SAP multicast.
 * RX: Listens for foreign SAP announcements and maintains a
 *     table of discovered AES67 streams.
 */

#ifndef SAP_SDP_H_
#define SAP_SDP_H_

#include <zephyr/kernel.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_ip.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- SAP constants (RFC 2974) ---- */

#define SAP_MULTICAST_ADDR  "239.255.255.255"
#define SAP_PORT            9875
#define SAP_HEADER_SIZE     8   /* V1 IPv4 header without auth */
#define SAP_ANNOUNCE_INTERVAL_S  30

/* ---- AES67 audio stream defaults ---- */

#define AES67_DEFAULT_MCAST_ADDR   "239.69.0.1"
#define AES67_DEFAULT_PORT         5004
#define AES67_DEFAULT_CHANNELS     2
#define AES67_DEFAULT_BIT_DEPTH    24
#define AES67_DEFAULT_SAMPLE_RATE  48000
#define AES67_DEFAULT_SAMPLES_PER_PKT  48
#define AES67_DEFAULT_PAYLOAD_TYPE 97

/* ---- Stream configuration ---- */

struct aes67_stream_config {
	struct in_addr mcast_addr;
	uint16_t port;
	uint8_t  channels;
	uint8_t  bit_depth;
	uint32_t sample_rate;
	uint16_t samples_per_packet;
	uint8_t  payload_type;
};

/* ---- TX stream table (up to 8 streams, matching tx_router) ---- */

#define AES67_MAX_TX_STREAMS  8
#define AES67_MAX_CH_PER_STREAM  8

struct aes67_tx_stream {
	bool     active;
	uint8_t  stream_id;
	struct in_addr dst_ip;
	uint8_t  channel_count;
	uint8_t  samples_per_packet;
	uint8_t  ch_ids[AES67_MAX_CH_PER_STREAM];
	uint32_t ssrc;  /* SSRC for RTP header, must match SDP announcement */
};

/* ---- Discovered foreign streams (SAP RX) ---- */

#define SAP_MAX_FOREIGN_STREAMS  8
#define SAP_SDP_NAME_MAX         32

struct sap_foreign_stream {
	bool     valid;
	uint16_t msg_id_hash;
	struct in_addr origin_addr;
	struct in_addr mcast_addr;
	uint16_t port;
	uint8_t  channels;
	uint8_t  bit_depth;
	uint32_t sample_rate;
	char     name[SAP_SDP_NAME_MAX];
	int64_t  last_seen_ms;
};

/* ---- Public API ---- */

/**
 * @brief Start the SAP/SDP subsystem.
 *
 * Spawns a background thread that sends SAP announcements for our
 * stream and listens for foreign SAP announcements.
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
 * @brief Get the current local stream configuration.
 */
const struct aes67_stream_config *sap_sdp_get_config(void);

/**
 * @brief Set the audio multicast destination address and port.
 *
 * @param addr  Multicast IPv4 address
 * @param port  UDP port (0 = keep current)
 * @return 0 on success, negative errno on error
 */
int sap_sdp_set_mcast(const struct in_addr *addr, uint16_t port);

/**
 * @brief Enable or disable SAP announcements.
 */
void sap_sdp_set_announce(bool enable);

/**
 * @brief Configure a TX audio stream.
 *
 * Writes the stream config to the FPGA via FMC and registers it
 * for SAP/SDP announcement. Triggers an immediate SAP announce.
 *
 * The same SSRC is written to the FPGA (for RTP packets) and included
 * in the SDP announcement to ensure consistency.
 *
 * @param stream_id        Stream index (0..7)
 * @param dst_ip           Destination multicast IP
 * @param channel_count    Number of channels (1..8)
 * @param samples_per_pkt  Samples per packet per channel
 * @param ch_ids           Array of channel IDs
 * @param num_ch_ids       Number of entries in ch_ids
 * @param ssrc             SSRC for RTP (0 = auto-generate from IP+stream_id)
 * @return 0 on success, negative errno on error
 */
int sap_sdp_configure_tx_stream(uint8_t stream_id,
				const struct in_addr *dst_ip,
				uint8_t channel_count,
				uint8_t samples_per_pkt,
				const uint8_t *ch_ids,
				uint8_t num_ch_ids,
				uint32_t ssrc);

/**
 * @brief Get the TX stream table.
 */
const struct aes67_tx_stream *sap_sdp_get_tx_streams(void);

/**
 * @brief Get the table of discovered foreign streams.
 *
 * @param count  Output: number of valid entries
 * @return Pointer to the foreign stream table (SAP_MAX_FOREIGN_STREAMS entries)
 */
const struct sap_foreign_stream *sap_sdp_get_foreign_streams(int *count);

#ifdef __cplusplus
}
#endif

#endif /* SAP_SDP_H_ */
