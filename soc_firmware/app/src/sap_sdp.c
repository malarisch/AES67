/*
 * SAP (RFC 2974) announcement transport.
 *
 * TX: one SAP announcement per active TX stream (tables owned by
 *     aes67_conn), SDP bodies built by aes67_sdp_utils.
 * RX: foreign SAP announcements on 239.255.255.255:9875 are parsed and
 *     reported into the aes67_conn foreign-stream registry.
 */

#include <zephyr/kernel.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/igmp.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <string.h>

#include "sap_sdp.h"
#include "aes67_conn.h"
#include "aes67_sdp_utils.h"
#include "aes67_config.h"
#include "ptp_bmc.h"
#include "ptp_ctrl.h"

LOG_MODULE_REGISTER(sap_sdp, LOG_LEVEL_INF);

/* ---- Thread resources ---- */
/* The RX path keeps a SAP_TX_BUF_SIZE receive buffer on the stack and runs
 * SDP parsing + the aes67_conn foreign registry (which logs) at its deepest
 * point — with immediate-mode logging that is ~2 KiB on top. 2048 overflowed
 * the moment the first real SAP announcements arrived. */
#define SAP_STACK_SIZE   6144
#define SAP_PRIORITY     K_PRIO_PREEMPT(12)

K_THREAD_STACK_DEFINE(sap_stack, SAP_STACK_SIZE);
static struct k_thread sap_thread_data;

/* ---- SAP packet constants ---- */
#define SAP_CONTENT_TYPE  "application/sdp"
#define SAP_TX_BUF_SIZE   512

/* Fallback if the configured interval is 0 (unset) */
#define SAP_DEFAULT_INTERVAL_S  30
#define SAP_MIN_INTERVAL_S      5

/* ---- State ---- */
static struct net_if *sap_iface;
static struct in_addr my_ip_addr;
static bool ip_ready;
static volatile bool force_announce;
static struct k_sem sap_ip_sem;

/* ---- SAP message ID (hash of originating source) ---- */
static uint16_t sap_msg_id_hash;

static uint32_t announce_interval_s(void)
{
	uint32_t s = aes67_config_get()->sap_announce_interval_s;

	if (s == 0) {
		return SAP_DEFAULT_INTERVAL_S;
	}
	return MAX(s, SAP_MIN_INTERVAL_S);
}

/* ================================================================
 * ts-refclk clock identity: the *elected* PTP grandmaster, or our own
 * clock identity while we ARE the grandmaster (as GM the stream is
 * traceable to us).  Returns NULL while neither is the case; the SDP
 * builder then omits the a=ts-refclk line entirely.
 * ================================================================ */
static const uint8_t *refclk_clock_id(uint8_t out[8])
{
	struct ptp_ctrl_status st;

	ptp_ctrl_get_status(&st);
	if (st.gm_valid) {
		memcpy(out, st.gm_id, 8);
		return out;
	}
	if (st.role == PTP_CTRL_ROLE_LEADER) {
		memcpy(out, st.clock_id, 8);
		return out;
	}
	return NULL;
}

/* ================================================================
 * Build SDP body for a TX stream
 * ================================================================ */
static int build_sdp_for_tx_stream(char *buf, size_t buf_size,
				   const struct aes67_tx_stream *stream)
{
	uint8_t gmid[8];
	struct aes67_device_config *cfg = aes67_config_get();
	struct aes67_sdp_params params = {
		.origin_addr = my_ip_addr,
		.connection_addr = stream->dst_ip,
		.stream_id = stream->stream_id,
		.channel_count = stream->channel_count,
		.bit_depth = AES67_DEFAULT_BIT_DEPTH,
		.sample_rate = AES67_DEFAULT_SAMPLE_RATE,
		.samples_per_packet = stream->samples_per_packet,
		.port = AES67_DEFAULT_PORT,
		.payload_type = AES67_DEFAULT_PAYLOAD_TYPE,
		.ssrc = stream->ssrc,
		.clock_id = refclk_clock_id(gmid),
		.stream_name = stream->name,
		.ptp_domain = cfg->ptp_domain,
		.sync_time = 0,  /* epoch-aligned RTP timestamp */
	};
	return aes67_sdp_build(buf, buf_size, &params);
}

/* ================================================================
 * Send a SAP announcement for one TX stream (RFC 2974)
 *
 * SAP header (8 bytes for IPv4, no auth):
 *   Byte 0: V=1 (bits 7-5), A=0 (bit 4), R=0 (bit 3),
 *           T=0 (bit 2, announce), E=0 (bit 1), C=0 (bit 0)
 *   Byte 1: Auth length = 0
 *   Bytes 2-3: Message ID hash (per-stream)
 *   Bytes 4-7: Originating source (IPv4)
 * Then: "application/sdp\0" content type
 * Then: SDP body
 * ================================================================ */
static int send_sap_announce_stream(int sock, const struct sockaddr_in *dst,
				    const struct aes67_tx_stream *stream)
{
	static uint8_t tx_buf[SAP_TX_BUF_SIZE];
	int offset = 0;

	/* Use stream_id to create unique msg_id per stream */
	uint16_t stream_msg_id = sap_msg_id_hash + stream->stream_id;

	/* SAP header */
	tx_buf[0] = 0x20;
	tx_buf[1] = 0x00;
	tx_buf[2] = (uint8_t)(stream_msg_id >> 8);
	tx_buf[3] = (uint8_t)(stream_msg_id & 0xFF);
	memcpy(&tx_buf[4], &my_ip_addr.s_addr, 4);
	offset = SAP_HEADER_SIZE;

	/* Content type (NUL-terminated) */
	size_t ct_len = strlen(SAP_CONTENT_TYPE) + 1;

	if (offset + ct_len >= SAP_TX_BUF_SIZE) {
		return -ENOMEM;
	}
	memcpy(&tx_buf[offset], SAP_CONTENT_TYPE, ct_len);
	offset += ct_len;

	/* SDP body */
	int sdp_len = build_sdp_for_tx_stream((char *)&tx_buf[offset],
					      SAP_TX_BUF_SIZE - offset,
					      stream);
	if (sdp_len < 0) {
		return sdp_len;
	}
	offset += sdp_len;

	ssize_t sent = zsock_sendto(sock, tx_buf, offset, 0,
				     (const struct sockaddr *)dst,
				     sizeof(*dst));
	if (sent < 0) {
		LOG_WRN("sendto stream %u failed: %d",
			stream->stream_id, errno);
		return -errno;
	}

	LOG_DBG("Sent stream %u announcement (%d bytes)",
		stream->stream_id, offset);
	return 0;
}

/* Announce every active TX stream once. */
static void announce_all_streams(int sock, const struct sockaddr_in *dst)
{
	struct aes67_tx_stream stream;

	for (uint8_t i = 0; i < AES67_MAX_TX_STREAMS; i++) {
		if (aes67_conn_copy_tx_stream(i, &stream)) {
			send_sap_announce_stream(sock, dst, &stream);
		}
	}
}

/* ================================================================
 * Parse a received SAP packet into a foreign-stream entry
 *
 * Returns 0 for an announcement, 1 for a deletion, negative errno on
 * parse failure.
 * ================================================================ */
static int parse_sap_sdp(const uint8_t *buf, size_t len,
			  struct aes67_foreign_stream *out)
{
	if (len < SAP_HEADER_SIZE + 2) {
		return -EINVAL;
	}

	uint8_t version = (buf[0] >> 5) & 0x07;

	if (version != 1) {
		return -ENOTSUP;
	}

	bool is_delete = !!(buf[0] & 0x04);

	if (is_delete) {
		/* Deletion — mark stream as invalid by msg_id_hash */
		out->valid = false;
		out->id_hash = sys_get_be16(&buf[2]);
		return 1; /* Signal deletion */
	}

	uint8_t addr_type = (buf[0] >> 4) & 0x01; /* 0=IPv4, 1=IPv6 */
	uint8_t auth_len = buf[1];
	size_t hdr_size = 4 + (addr_type ? 16 : 4) + auth_len * 4;

	if (len < hdr_size) {
		return -EINVAL;
	}

	out->id_hash = sys_get_be16(&buf[2]);

	if (!addr_type) {
		memcpy(&out->origin_addr.s_addr, &buf[4], 4);
	}

	/* Skip content type string (find NUL terminator) */
	const char *payload = (const char *)&buf[hdr_size];
	size_t remaining = len - hdr_size;
	const char *sdp_start = memchr(payload, '\0', remaining);

	if (!sdp_start) {
		return -EINVAL;
	}
	sdp_start++; /* Skip past NUL */
	remaining -= (sdp_start - payload);

	/* Parse SDP body using shared helper */
	struct aes67_sdp_parsed parsed;
	int ret = aes67_sdp_parse(sdp_start, remaining, &parsed);
	if (ret < 0) {
		return ret;
	}

	/* Copy parsed fields */
	out->valid = true;
	memcpy(out->name, parsed.name, sizeof(out->name));
	memcpy(out->sender_name, parsed.origin_name,
	       sizeof(out->sender_name));
	out->mcast_addr = parsed.connection_addr;
	out->port = parsed.port;
	out->channels = parsed.channels;
	out->bit_depth = parsed.bit_depth;
	out->sample_rate = parsed.sample_rate;
	out->ssrc = parsed.ssrc;
	out->samples_per_packet = parsed.samples_per_packet;
	/* RAVENNA extensions */
	out->ptp_domain = parsed.ptp_domain;
	out->has_clock_domain = parsed.has_clock_domain;
	out->sync_time = parsed.sync_time;
	out->has_sync_time = parsed.has_sync_time;

	return 0;
}

/* ================================================================
 * SAP thread: TX announcements + RX foreign announcements
 * ================================================================ */
static void sap_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	int sock;
	int ret;

	LOG_INF("Thread starting");

	/* Wait for network interface to be up */
	while (!net_if_is_up(sap_iface)) {
		k_msleep(500);
	}

	/* Wait until we have a valid IP */
	LOG_INF("Waiting for valid IP address...");
	k_sem_take(&sap_ip_sem, K_FOREVER);
	LOG_INF("IP ready, opening socket");

	/* Create UDP socket */
	sock = zsock_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (sock < 0) {
		LOG_ERR("Failed to create socket: %d", errno);
		return;
	}

	/* Bind to SAP port for receiving */
	struct sockaddr_in bind_addr;

	memset(&bind_addr, 0, sizeof(bind_addr));
	bind_addr.sin_family = AF_INET;
	bind_addr.sin_port = htons(SAP_PORT);
	bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);

	ret = zsock_bind(sock, (struct sockaddr *)&bind_addr,
			 sizeof(bind_addr));
	if (ret < 0) {
		LOG_ERR("Failed to bind socket: %d", errno);
		zsock_close(sock);
		return;
	}

	/* Join SAP multicast group 239.255.255.255 */
	struct in_addr sap_group;

	zsock_inet_pton(AF_INET, SAP_MULTICAST_ADDR, &sap_group);
	ret = net_ipv4_igmp_join(sap_iface, &sap_group, NULL);
	if (ret < 0) {
		LOG_WRN("Failed to join multicast %s: %d (continuing)",
			SAP_MULTICAST_ADDR, ret);
	} else {
		LOG_INF("Joined multicast group %s", SAP_MULTICAST_ADDR);
	}

	/* SAP TX destination */
	struct sockaddr_in sap_dst;

	memset(&sap_dst, 0, sizeof(sap_dst));
	sap_dst.sin_family = AF_INET;
	sap_dst.sin_port = htons(SAP_PORT);
	zsock_inet_pton(AF_INET, SAP_MULTICAST_ADDR, &sap_dst.sin_addr);

	/* ---- Main loop ---- */
	int64_t last_announce_ms = 0;
	uint8_t rx_buf[SAP_TX_BUF_SIZE];

	while (1) {
		int64_t now = k_uptime_get();
		int64_t interval_ms = (int64_t)announce_interval_s() * 1000;

		/* Send announcements if interval elapsed or forced */
		if (aes67_config_get()->sap_announce_enabled &&
		    (force_announce ||
		     (now - last_announce_ms) >= interval_ms)) {
			force_announce = false;
			announce_all_streams(sock, &sap_dst);
			last_announce_ms = now;
		}

		/* Compute remaining time until next announcement */
		int64_t next_tx_ms = last_announce_ms + interval_ms;
		int poll_timeout_ms = (int)(next_tx_ms - now);

		poll_timeout_ms = CLAMP(poll_timeout_ms, 100, 2000);

		/* Poll for incoming SAP packets */
		struct zsock_pollfd pfd = {
			.fd = sock,
			.events = ZSOCK_POLLIN,
		};
		int pret = zsock_poll(&pfd, 1, poll_timeout_ms);

		if (pret <= 0) {
			continue;
		}

		struct sockaddr_in src_addr;
		socklen_t src_len = sizeof(src_addr);

		ssize_t n = zsock_recvfrom(sock, rx_buf, sizeof(rx_buf),
					    ZSOCK_MSG_DONTWAIT,
					    (struct sockaddr *)&src_addr,
					    &src_len);
		if (n <= 0) {
			continue;
		}

		/* Don't process our own announcements */
		if (src_addr.sin_addr.s_addr == my_ip_addr.s_addr) {
			continue;
		}

		struct aes67_foreign_stream parsed;

		ret = parse_sap_sdp(rx_buf, n, &parsed);
		if (ret >= 0) {
			parsed.via = AES67_VIA_SAP;
			aes67_conn_report_foreign_stream(&parsed);
		}
	}
}

/* ================================================================
 * Public API
 * ================================================================ */

/* TX-stream change observer: announce the new state right away. */
static void on_tx_stream_change(uint8_t stream_id)
{
	ARG_UNUSED(stream_id);
	force_announce = true;
}

int sap_sdp_start(struct net_if *iface)
{
	if (!iface) {
		return -EINVAL;
	}

	sap_iface = iface;
	force_announce = false;

	k_sem_init(&sap_ip_sem, 0, 1);
	if (ip_ready) {
		/* IP arrived before the thread existed */
		k_sem_give(&sap_ip_sem);
	}

	aes67_conn_register_tx_observer(on_tx_stream_change);

	k_thread_create(&sap_thread_data, sap_stack, SAP_STACK_SIZE,
			sap_thread_fn, NULL, NULL, NULL,
			SAP_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&sap_thread_data, "sap_sdp");

	LOG_INF("Started");
	return 0;
}

void sap_sdp_notify_ip_ready(const struct in_addr *addr)
{
	my_ip_addr = *addr;

	/* Generate message ID hash from IP */
	uint32_t ip_val = sys_be32_to_cpu(addr->s_addr);

	sap_msg_id_hash = (uint16_t)(ip_val ^ (ip_val >> 16));

	ip_ready = true;
	k_sem_give(&sap_ip_sem);
	LOG_INF("IP-ready notification received");
}

void sap_sdp_notify_link_up(void)
{
	if (!ip_ready || !sap_iface) {
		LOG_DBG("link_up notify ignored (no IP yet)");
		return;
	}

	struct in_addr sap_group;

	zsock_inet_pton(AF_INET, SAP_MULTICAST_ADDR, &sap_group);

	/* Leave first: a plain re-join of an existing membership returns
	 * without emitting a fresh IGMP report — but that report is the
	 * whole point after a link bounce, because the switch's snooping
	 * table is stale. */
	(void)net_ipv4_igmp_leave(sap_iface, &sap_group);

	int ret = net_ipv4_igmp_join(sap_iface, &sap_group, NULL);

	if (ret < 0) {
		LOG_WRN("Failed to rejoin multicast %s: %d",
			SAP_MULTICAST_ADDR, ret);
	} else {
		LOG_INF("Rejoined SAP multicast group %s", SAP_MULTICAST_ADDR);
	}
}

void sap_sdp_set_announce(bool enable)
{
	struct aes67_device_config *cfg = aes67_config_get();

	aes67_config_lock();
	cfg->sap_announce_enabled = enable;
	aes67_config_unlock();

	if (enable) {
		force_announce = true;
	}
	LOG_INF("Announcements %s", enable ? "enabled" : "disabled");
}

void sap_sdp_trigger_announce(void)
{
	force_announce = true;
}
