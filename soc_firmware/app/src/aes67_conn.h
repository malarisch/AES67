/*
 * AES67 connection management.
 *
 * Owns the device's stream state and its projection into the FPGA data
 * plane:
 *  - the local TX stream table (tx_router) and RX stream table
 *    (rx_ringbuffer), written through the FPGA HAL,
 *  - IGMP proxy joins for FPGA-terminated multicast groups (the FPGA
 *    consumes the RTP packets, but IGMP-snooping switches only forward
 *    them if the CPU emits membership reports),
 *  - the registry of foreign streams discovered by SAP, mDNS/DNS-SD or
 *    RTSP, including expiry of stale entries.
 *
 * Announcement transports (SAP, mDNS) sit on top of this module: they
 * read the tables and subscribe to TX-stream changes via the observer
 * hook. This module never calls up into them.
 */

#ifndef AES67_CONN_H_
#define AES67_CONN_H_

#include <zephyr/net/net_if.h>
#include <zephyr/net/net_ip.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- AES67 audio stream defaults ---- */

#define AES67_DEFAULT_MCAST_ADDR   "239.69.0.1"
#define AES67_DEFAULT_PORT         5004
#define AES67_DEFAULT_CHANNELS     2
#define AES67_DEFAULT_BIT_DEPTH    24
#define AES67_DEFAULT_SAMPLE_RATE  48000
#define AES67_DEFAULT_SAMPLES_PER_PKT  48
#define AES67_DEFAULT_PAYLOAD_TYPE 97

/* ---- Stream table dimensions (matching tx_router / rx_ringbuffer) ---- */

#define AES67_MAX_TX_STREAMS     8
#define AES67_MAX_RX_STREAMS     8
#define AES67_MAX_CH_PER_STREAM  8
#define AES67_STREAM_NAME_MAX    32

struct aes67_tx_stream {
	bool     active;
	uint8_t  stream_id;
	char     name[AES67_STREAM_NAME_MAX];  /* User-defined stream name */
	struct in_addr dst_ip;
	uint8_t  channel_count;
	uint8_t  samples_per_packet;
	uint8_t  ch_ids[AES67_MAX_CH_PER_STREAM];
	uint32_t ssrc;  /* SSRC for RTP header, must match SDP announcement */
};

struct aes67_rx_stream {
	bool    active;
	uint8_t stream_id;
	char    name[AES67_STREAM_NAME_MAX];        /* Remote session name */
	char    sender_name[AES67_STREAM_NAME_MAX]; /* Human-readable source */
	struct in_addr sender_ip;
	struct in_addr dst_ip;
	uint16_t dst_port;
	uint8_t  ch_map[AES67_MAX_CH_PER_STREAM]; /* output channel per input ch */
	uint8_t  channel_count;
	uint8_t  output_delay;
	uint8_t  samples_per_channel;
};

/* ---- Discovered foreign streams ---- */

/* Sized generously: SAP and mDNS sightings of the SAME RTP flow are kept
 * as separate entries (merged only in the web UI), so every real stream
 * can occupy two slots — a single RAVENNA daemon with a handful of
 * sessions plus a few of our own boards overflowed the previous 8-slot
 * table, and overflowing means discoveries are silently dropped (roughly
 * 100 B per slot, so 32 costs about 3 KB). */
#define AES67_MAX_FOREIGN_STREAMS  32
#define AES67_FOREIGN_NAME_MAX     32

/* Discovery transport a foreign-stream sighting came from (bitmask —
 * the web UI merges sightings of the same RTP flow and ORs these). */
#define AES67_VIA_SAP   0x01
#define AES67_VIA_MDNS  0x02

struct aes67_foreign_stream {
	bool     valid;
	uint8_t  via;                /* AES67_VIA_* transport bit */
	uint16_t id_hash;            /* SAP msg-id hash / mDNS instance hash */
	struct in_addr origin_addr;
	struct in_addr mcast_addr;
	uint16_t port;
	uint8_t  channels;
	uint8_t  bit_depth;
	uint32_t sample_rate;
	uint32_t ssrc;               /* from a=ssrc: line, 0 if not present */
	uint16_t samples_per_packet; /* derived from a=ptime: line, 0 if unknown */
	char     name[AES67_FOREIGN_NAME_MAX];
	char     sender_name[AES67_FOREIGN_NAME_MAX];
	int64_t  last_seen_ms;
	/* RAVENNA extensions */
	uint8_t  ptp_domain;         /* from a=clock-domain:PTPv2 <domain> */
	bool     has_clock_domain;   /* true if a=clock-domain was present */
	uint32_t sync_time;          /* from a=sync-time:<timestamp> */
	bool     has_sync_time;      /* true if a=sync-time was present */
};

/* ---- Lifecycle ---- */

/**
 * @brief Initialise connection management.
 *
 * Stores the interface for IGMP proxy joins and starts the periodic
 * expiry of stale foreign-stream entries. Stream configuration calls
 * are accepted before this (config restored from flash/SD is applied
 * early); their IGMP joins are caught up on the first IP-ready event.
 *
 * @param iface  Network interface used for IGMP membership reports
 * @return 0 on success, negative errno on error
 */
int aes67_conn_init(struct net_if *iface);

/**
 * @brief Notify that a valid IP has been assigned (e.g. DHCP bound).
 *
 * Records the local address (used for SSRC auto-generation) and joins
 * the multicast group of every active RX stream configured so far.
 */
void aes67_conn_notify_ip_ready(const struct in_addr *addr);

/**
 * @brief Notify that the Ethernet link has come (back) up.
 *
 * Re-joins the multicast groups of all active RX streams leave-first,
 * so a fresh IGMP report is emitted even though the membership still
 * exists — after a link bounce the switch's snooping table is stale.
 */
void aes67_conn_notify_link_up(void);

/* ---- TX / RX stream configuration ---- */

/**
 * @brief Configure a TX audio stream.
 *
 * Writes the stream config to the FPGA tx_router and updates the local
 * table. A zero destination IP or zero channel count deactivates the
 * stream (the zero config is still written to the FPGA to disable it in
 * hardware). Registered TX observers are notified on every change.
 *
 * @param stream_id        Stream index (0..AES67_MAX_TX_STREAMS-1)
 * @param dst_ip           Destination multicast IP (0.0.0.0 = deactivate)
 * @param channel_count    Number of channels (1..8, 0 = deactivate)
 * @param samples_per_pkt  Samples per packet per channel
 * @param ch_ids           Array of channel IDs
 * @param num_ch_ids       Number of entries in ch_ids
 * @param ssrc             SSRC for RTP (0 = auto-generate from IP+stream_id)
 * @param name             Stream name (NULL = auto-generate)
 * @return 0 on success, negative errno on error
 */
int aes67_conn_configure_tx_stream(uint8_t stream_id,
				   const struct in_addr *dst_ip,
				   uint8_t channel_count,
				   uint8_t samples_per_pkt,
				   const uint8_t *ch_ids,
				   uint8_t num_ch_ids,
				   uint32_t ssrc,
				   const char *name);

/**
 * @brief Configure an RX audio stream.
 *
 * Writes the stream config to the FPGA rx_ringbuffer stream_ram and
 * updates the local table. Joins the destination multicast group via
 * IGMP (proxy join for the FPGA). A zero destination IP deactivates
 * the stream.
 *
 * @param stream_id      Stream index (0..AES67_MAX_RX_STREAMS-1)
 * @param dst_ip         Destination IP to match (multicast group; 0 = deactivate)
 * @param dst_port       Destination UDP port to match in incoming packets
 * @param ch_map         Output channel map (ch_map[i] = output channel for input ch i)
 * @param channel_count  Number of channels (1..8)
 * @param output_delay        Output delay in samples
 * @param samples_per_channel Samples per channel per packet
 * @param name           Remote session name (NULL = auto-generate)
 * @param sender_name    Human-readable source name (NULL = unknown)
 * @param sender_ip      Source address (NULL = unknown)
 * @return 0 on success, negative errno on error
 */
int aes67_conn_configure_rx_stream(uint8_t stream_id,
				   const struct in_addr *dst_ip,
				   uint16_t dst_port,
				   const uint8_t *ch_map,
				   uint8_t channel_count,
				   uint8_t output_delay,
				   uint8_t samples_per_channel,
				   const char *name,
				   const char *sender_name,
				   const struct in_addr *sender_ip);

/**
 * @brief Get the TX stream table (AES67_MAX_TX_STREAMS entries).
 */
const struct aes67_tx_stream *aes67_conn_get_tx_streams(void);

/**
 * @brief Get the RX stream table (AES67_MAX_RX_STREAMS entries).
 */
const struct aes67_rx_stream *aes67_conn_get_rx_streams(void);

/**
 * @brief Copy one TX stream entry under the table lock.
 *
 * @return true if the entry is active (out filled in), false otherwise
 */
bool aes67_conn_copy_tx_stream(uint8_t stream_id,
			       struct aes67_tx_stream *out);

/* ---- TX-stream change observers ---- */

/**
 * Called after a TX stream changed (configured or deactivated), with the
 * table already updated. Announcement transports use this to trigger an
 * immediate re-announce / service update. Called from the configuring
 * thread; keep it short and do not configure streams from within.
 */
typedef void (*aes67_conn_tx_observer_t)(uint8_t stream_id);

/**
 * @brief Register a TX-stream change observer (max 4, no unregister).
 *
 * @return 0 on success, -ENOMEM if the observer slots are exhausted
 */
int aes67_conn_register_tx_observer(aes67_conn_tx_observer_t cb);

/* ---- Foreign stream registry ---- */

/**
 * @brief Get the table of discovered foreign streams.
 *
 * @param count  Output: number of valid entries
 * @return Pointer to the table (AES67_MAX_FOREIGN_STREAMS entries)
 */
const struct aes67_foreign_stream *aes67_conn_get_foreign_streams(int *count);

/**
 * @brief Report a discovered stream into the foreign-stream registry.
 *
 * Used by all discovery transports (SAP, mDNS, RTSP DESCRIBE): the
 * caller fills the entry (including a unique id_hash) and it is
 * upserted. fs->valid == false deletes the entry by hash.
 */
void aes67_conn_report_foreign_stream(const struct aes67_foreign_stream *fs);

/**
 * @brief Refresh the last-seen timestamp of a foreign stream by hash,
 *        preventing expiry without re-submitting the full entry.
 */
void aes67_conn_touch_foreign_stream(uint16_t id_hash);

#ifdef __cplusplus
}
#endif

#endif /* AES67_CONN_H_ */
