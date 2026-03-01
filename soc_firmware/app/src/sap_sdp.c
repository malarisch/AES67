/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: Apache-2.0
 *
 * AES67 SAP/SDP implementation.
 *
 * TX: Sends periodic SAP (RFC 2974) announcements carrying an
 *     AES67-compliant SDP (RFC 4566) session description for our
 *     audio stream.
 *
 * RX: Listens for foreign SAP announcements on 239.255.255.255:9875
 *     and maintains a table of discovered AES67 streams.
 *
 * Shell: Provides "aes67" commands for runtime configuration.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/igmp.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <string.h>
#include <stdio.h>

#include "sap_sdp.h"
#include "../drivers/eth_fmc_basic/eth_fmc_basic.h"

LOG_MODULE_REGISTER(sap_sdp, LOG_LEVEL_INF);

/* ---- Thread resources ---- */
#define SAP_STACK_SIZE   2048
#define SAP_PRIORITY     K_PRIO_PREEMPT(12)

K_THREAD_STACK_DEFINE(sap_stack, SAP_STACK_SIZE);
static struct k_thread sap_thread_data;

/* ---- State ---- */
static struct net_if *sap_iface;
static int sap_sock = -1;  /* Socket for multicast rejoin */
static struct in_addr my_ip_addr;
static uint8_t my_clock_id[8];
static bool ip_ready;
static bool announce_enabled = true;
static struct k_sem sap_ip_sem;
static struct k_mutex sap_mutex;

/* ---- SAP packet constants ---- */
#define SAP_CONTENT_TYPE  "application/sdp"
#define SAP_TX_BUF_SIZE   512

/* ---- Local stream configuration (legacy single-stream) ---- */
static struct aes67_stream_config local_config;

/* ---- TX stream table ---- */
static struct aes67_tx_stream tx_streams[AES67_MAX_TX_STREAMS];
static volatile bool force_announce;

/* ---- RX stream table ---- */
static struct aes67_rx_stream rx_streams[AES67_MAX_RX_STREAMS];

/* ---- Foreign stream table ---- */
static struct sap_foreign_stream foreign_streams[SAP_MAX_FOREIGN_STREAMS];

/* ---- SAP message ID (hash of originating source) ---- */
static uint16_t sap_msg_id_hash;

/* ================================================================
 * Helper: Build EUI-64 clock identity from 48-bit MAC address
 * (same logic as ptp_bmc.c)
 * ================================================================ */
static void mac_to_clock_identity(const uint8_t mac[6], uint8_t clock_id[8])
{
	clock_id[0] = mac[0] ^ 0x02;
	clock_id[1] = mac[1];
	clock_id[2] = mac[2];
	clock_id[3] = 0xFF;
	clock_id[4] = 0xFE;
	clock_id[5] = mac[3];
	clock_id[6] = mac[4];
	clock_id[7] = mac[5];
}

/* ================================================================
 * Helper: Format clock identity as PTP=IEEE1588-2008 string
 * Output: "XX-XX-XX-FF-FE-XX-XX-XX"
 * ================================================================ */
static int format_ptp_clock_id(char *buf, size_t len, const uint8_t id[8])
{
	return snprintf(buf, len,
			"%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X",
			id[0], id[1], id[2], id[3],
			id[4], id[5], id[6], id[7]);
}

/* ================================================================
 * Build SDP body for our AES67 stream
 *
 * Returns the number of bytes written to buf (not including NUL),
 * or negative errno if the buffer is too small.
 * ================================================================ */
static int build_sdp(char *buf, size_t buf_size)
{
	char ip_str[INET_ADDRSTRLEN];
	char mcast_str[INET_ADDRSTRLEN];
	char clock_id_str[32];

	zsock_inet_ntop(AF_INET, &my_ip_addr, ip_str, sizeof(ip_str));
	zsock_inet_ntop(AF_INET, &local_config.mcast_addr,
			mcast_str, sizeof(mcast_str));
	format_ptp_clock_id(clock_id_str, sizeof(clock_id_str), my_clock_id);

	/* Compute ptime in microseconds, then format as fractional ms.
	 * 16 samples at 48000 Hz = 333.333 µs = 0.333 ms */
	uint32_t ptime_us = (uint32_t)local_config.samples_per_packet *
			    1000000U / local_config.sample_rate;

	/* Session ID: use the IP address as a simple unique-ish number */
	uint32_t session_id = sys_be32_to_cpu(my_ip_addr.s_addr);

	int n = snprintf(buf, buf_size,
		"v=0\r\n"
		"o=- %u 1 IN IP4 %s\r\n"
		"s=Der geile Hecht\r\n"
		"i=%uch %ubit %uHz\r\n"
		"c=IN IP4 %s/32\r\n"
		"t=0 0\r\n"
		"m=audio %u RTP/AVP %u\r\n"
		"a=rtpmap:%u L%u/%u/%u\r\n"
		"a=ptime:%u.%03u\r\n"
		"a=ts-refclk:ptp=IEEE1588-2008:%s\r\n"
		"a=mediaclk:direct=0\r\n",
		session_id, ip_str,
		local_config.channels, local_config.bit_depth,
		local_config.sample_rate,
		mcast_str,
		local_config.port, local_config.payload_type,
		local_config.payload_type, local_config.bit_depth,
		local_config.sample_rate, local_config.channels,
		ptime_us / 1000, ptime_us % 1000,
		clock_id_str);

	if (n < 0 || (size_t)n >= buf_size) {
		return -ENOMEM;
	}
	return n;
}

/* ================================================================
 * Build SDP body for a TX stream from the tx_streams table
 * ================================================================ */
static int build_sdp_for_tx_stream(char *buf, size_t buf_size,
				   const struct aes67_tx_stream *stream)
{
	char ip_str[INET_ADDRSTRLEN];
	char mcast_str[INET_ADDRSTRLEN];
	char clock_id_str[32];

	zsock_inet_ntop(AF_INET, &my_ip_addr, ip_str, sizeof(ip_str));
	zsock_inet_ntop(AF_INET, &stream->dst_ip, mcast_str, sizeof(mcast_str));
	format_ptp_clock_id(clock_id_str, sizeof(clock_id_str), my_clock_id);

	uint32_t ptime_us = (uint32_t)stream->samples_per_packet *
			    1000000U / AES67_DEFAULT_SAMPLE_RATE;

	/* Use stream_id to create distinct session IDs */
	uint32_t session_id = sys_be32_to_cpu(my_ip_addr.s_addr) +
			      stream->stream_id;

	int n = snprintf(buf, buf_size,
		"v=0\r\n"
		"o=- %u %u IN IP4 %s\r\n"
		"s=AES67 Stream %u\r\n"
		"i=%uch %ubit %uHz\r\n"
		"c=IN IP4 %s/32\r\n"
		"t=0 0\r\n"
		"m=audio %u RTP/AVP %u\r\n"
		"a=rtpmap:%u L%u/%u/%u\r\n"
		"a=ptime:%u.%03u\r\n"
		"a=ts-refclk:ptp=IEEE1588-2008:%s\r\n"
		"a=mediaclk:direct=0\r\n"
		"a=ssrc:%u cname:aes67@%s\r\n",
		session_id, stream->stream_id, ip_str,
		stream->stream_id,
		stream->channel_count, AES67_DEFAULT_BIT_DEPTH,
		AES67_DEFAULT_SAMPLE_RATE,
		mcast_str,
		AES67_DEFAULT_PORT, AES67_DEFAULT_PAYLOAD_TYPE,
		AES67_DEFAULT_PAYLOAD_TYPE, AES67_DEFAULT_BIT_DEPTH,
		AES67_DEFAULT_SAMPLE_RATE, stream->channel_count,
		ptime_us / 1000, ptime_us % 1000,
		clock_id_str,
		stream->ssrc, ip_str);

	if (n < 0 || (size_t)n >= buf_size) {
		return -ENOMEM;
	}
	return n;
}

/* ================================================================
 * Send a SAP announcement for a specific TX stream
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

	/* Content type */
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
		LOG_WRN("SAP: sendto stream %u failed: %d",
			stream->stream_id, errno);
		return -errno;
	}

	LOG_DBG("SAP: Sent stream %u announcement (%d bytes)",
		stream->stream_id, offset);
	return 0;
}

/* ================================================================
 * Build and send a SAP announcement packet (RFC 2974)
 *
 * SAP header (8 bytes for IPv4, no auth):
 *   Byte 0: V=1 (bits 7-5), A=0 (bit 4), R=0 (bit 3),
 *           T=0 (bit 2, announce), E=0 (bit 1), C=0 (bit 0)
 *   Byte 1: Auth length = 0
 *   Bytes 2-3: Message ID hash
 *   Bytes 4-7: Originating source (IPv4)
 *
 * Then: "application/sdp\0" content type
 * Then: SDP body
 * ================================================================ */
static int send_sap_announce(int sock, const struct sockaddr_in *dst)
{
	static uint8_t tx_buf[SAP_TX_BUF_SIZE];
	int offset = 0;

	/* SAP header */
	tx_buf[0] = 0x20; /* V=1, A=0, R=0, T=0, E=0, C=0 */
	tx_buf[1] = 0x00; /* Auth length = 0 */
	tx_buf[2] = (uint8_t)(sap_msg_id_hash >> 8);
	tx_buf[3] = (uint8_t)(sap_msg_id_hash & 0xFF);
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
	int sdp_len = build_sdp((char *)&tx_buf[offset],
				SAP_TX_BUF_SIZE - offset);
	if (sdp_len < 0) {
		return sdp_len;
	}
	offset += sdp_len;

	ssize_t sent = zsock_sendto(sock, tx_buf, offset, 0,
				     (const struct sockaddr *)dst,
				     sizeof(*dst));
	if (sent < 0) {
		LOG_WRN("SAP: sendto failed: %d", errno);
		return -errno;
	}

	LOG_DBG("SAP: Sent announcement (%d bytes)", offset);
	return 0;
}

/* ================================================================
 * Parse a received SAP packet and extract SDP fields
 * ================================================================ */
static int parse_sap_sdp(const uint8_t *buf, size_t len,
			  struct sap_foreign_stream *out)
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
		out->msg_id_hash = sys_get_be16(&buf[2]);
		return 1; /* Signal deletion */
	}

	uint8_t addr_type = (buf[0] >> 4) & 0x01; /* 0=IPv4, 1=IPv6 */
	uint8_t auth_len = buf[1];
	size_t hdr_size = 4 + (addr_type ? 16 : 4) + auth_len * 4;

	if (len < hdr_size) {
		return -EINVAL;
	}

	out->msg_id_hash = sys_get_be16(&buf[2]);

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

	/* Parse SDP fields line by line */
	memset(out->name, 0, sizeof(out->name));
	out->valid = true;
	out->port = 0;
	out->channels = 0;
	out->bit_depth = 0;
	out->sample_rate = 0;
	out->ssrc = 0;
	out->samples_per_packet = 0;
	out->mcast_addr.s_addr = 0;

	const char *line = sdp_start;
	const char *end = sdp_start + remaining;

	while (line < end) {
		const char *eol = memchr(line, '\n', end - line);
		size_t line_len = eol ? (size_t)(eol - line) : (size_t)(end - line);

		/* Strip \r */
		size_t clean_len = line_len;

		if (clean_len > 0 && line[clean_len - 1] == '\r') {
			clean_len--;
		}

		if (clean_len > 2 && line[0] == 's' && line[1] == '=') {
			/* Session name */
			size_t copy_len = clean_len - 2;

			if (copy_len >= SAP_SDP_NAME_MAX) {
				copy_len = SAP_SDP_NAME_MAX - 1;
			}
			memcpy(out->name, line + 2, copy_len);
			out->name[copy_len] = '\0';
		} else if (clean_len > 2 && line[0] == 'c' && line[1] == '=') {
			/* Connection: c=IN IP4 <addr>/TTL */
			const char *ip_start = NULL;
			const char *p = line + 2;
			const char *p_end = line + clean_len;

			/* Find the IP address after "IN IP4 " */
			const char *marker = "IN IP4 ";
			size_t marker_len = 7;

			for (; p + marker_len <= p_end; p++) {
				if (memcmp(p, marker, marker_len) == 0) {
					ip_start = p + marker_len;
					break;
				}
			}
			if (ip_start) {
				char addr_buf[INET_ADDRSTRLEN];
				const char *slash = memchr(ip_start, '/',
							   p_end - ip_start);
				size_t addr_len = slash ?
					(size_t)(slash - ip_start) :
					(size_t)(p_end - ip_start);

				if (addr_len < sizeof(addr_buf)) {
					memcpy(addr_buf, ip_start, addr_len);
					addr_buf[addr_len] = '\0';
					zsock_inet_pton(AF_INET, addr_buf,
							&out->mcast_addr);
				}
			}
		} else if (clean_len > 2 && line[0] == 'm' && line[1] == '=') {
			/* Media: m=audio <port> RTP/AVP <pt> */
			unsigned int port_val = 0;

			/* Find port number after "audio " */
			const char *p = line + 2;

			if (clean_len > 8 && memcmp(p, "audio ", 6) == 0) {
				p += 6;
				while (p < line + clean_len && *p >= '0' &&
				       *p <= '9') {
					port_val = port_val * 10 +
						   (*p - '0');
					p++;
				}
				out->port = (uint16_t)port_val;
			}
		} else if (clean_len > 10 && memcmp(line, "a=rtpmap:", 9) == 0) {
			/* a=rtpmap:<pt> L<depth>/<rate>/<ch> */
			const char *p = line + 9;
			const char *p_end = line + clean_len;

			/* Skip payload type number */
			while (p < p_end && *p >= '0' && *p <= '9') {
				p++;
			}
			if (p < p_end && *p == ' ') {
				p++;
			}
			/* Expect L<depth>/<rate>/<ch> */
			if (p < p_end && *p == 'L') {
				p++;
				unsigned int depth = 0, rate = 0, ch = 0;

				while (p < p_end && *p >= '0' && *p <= '9') {
					depth = depth * 10 + (*p - '0');
					p++;
				}
				if (p < p_end && *p == '/') {
					p++;
					while (p < p_end && *p >= '0' &&
					       *p <= '9') {
						rate = rate * 10 + (*p - '0');
						p++;
					}
				}
				if (p < p_end && *p == '/') {
					p++;
					while (p < p_end && *p >= '0' &&
					       *p <= '9') {
						ch = ch * 10 + (*p - '0');
						p++;
					}
				}
				out->bit_depth = (uint8_t)depth;
				out->sample_rate = rate;
				out->channels = (uint8_t)ch;
			}
		} else if (clean_len > 7 && memcmp(line, "a=ssrc:", 7) == 0) {
			/* a=ssrc:<ssrc> cname:... -- extract 32-bit SSRC */
			const char *p = line + 7;
			const char *p_end = line + clean_len;
			uint32_t ssrc_val = 0;

			while (p < p_end && *p >= '0' && *p <= '9') {
				ssrc_val = ssrc_val * 10 + (uint32_t)(*p - '0');
				p++;
			}
			out->ssrc = ssrc_val;
		} else if (clean_len > 8 && memcmp(line, "a=ptime:", 8) == 0) {
			/* a=ptime:<ms>[.<frac>] -- store raw us, resolve after loop */
			const char *p = line + 8;
			const char *p_end = line + clean_len;
			uint32_t ptime_ms = 0;
			uint32_t ptime_frac_us = 0;

			while (p < p_end && *p >= '0' && *p <= '9') {
				ptime_ms = ptime_ms * 10 + (uint32_t)(*p - '0');
				p++;
			}
			if (p < p_end && *p == '.') {
				p++;
				uint32_t scale = 100000U;

				while (p < p_end && *p >= '0' &&
				       *p <= '9' && scale > 0) {
					ptime_frac_us += (*p - '0') * scale;
					scale /= 10;
					p++;
				}
			}
			/* Store raw us as placeholder; resolved to samples below */
			out->samples_per_packet = (uint16_t)(
				ptime_ms * 1000U + ptime_frac_us);
		}

		if (!eol) {
			break;
		}
		line = eol + 1;
	}

	/* Resolve samples_per_packet: stored above as raw ptime us,
	 * compute samples = ptime_us * sample_rate / 1000000 */
	if (out->samples_per_packet > 0 && out->sample_rate > 0) {
		uint32_t ptime_us = out->samples_per_packet;

		out->samples_per_packet = (uint16_t)(
			ptime_us * out->sample_rate / 1000000U);
	}

	return 0;
}

/* ================================================================
 * Foreign stream table management
 * ================================================================ */

static void foreign_stream_expire(void)
{
	int64_t now = k_uptime_get();
	/* Timeout: 3× SAP announce interval */
	int64_t timeout_ms = 3 * SAP_ANNOUNCE_INTERVAL_S * 1000;

	for (int i = 0; i < SAP_MAX_FOREIGN_STREAMS; i++) {
		if (foreign_streams[i].valid &&
		    (now - foreign_streams[i].last_seen_ms) > timeout_ms) {
			LOG_INF("SAP: Stream expired: %s",
				foreign_streams[i].name);
			foreign_streams[i].valid = false;
		}
	}
}

static void foreign_stream_update(const struct sap_foreign_stream *parsed)
{
	k_mutex_lock(&sap_mutex, K_FOREVER);

	/* Handle deletion */
	if (!parsed->valid) {
		for (int i = 0; i < SAP_MAX_FOREIGN_STREAMS; i++) {
			if (foreign_streams[i].valid &&
			    foreign_streams[i].msg_id_hash ==
				    parsed->msg_id_hash) {
				LOG_INF("SAP: Stream deleted: %s",
					foreign_streams[i].name);
				foreign_streams[i].valid = false;
			}
		}
		k_mutex_unlock(&sap_mutex);
		return;
	}

	/* Find existing entry or free slot */
	int free_idx = -1;

	for (int i = 0; i < SAP_MAX_FOREIGN_STREAMS; i++) {
		if (foreign_streams[i].valid &&
		    foreign_streams[i].msg_id_hash == parsed->msg_id_hash &&
		    foreign_streams[i].origin_addr.s_addr ==
			    parsed->origin_addr.s_addr) {
			/* Update existing */
			foreign_streams[i] = *parsed;
			foreign_streams[i].last_seen_ms = k_uptime_get();
			k_mutex_unlock(&sap_mutex);
			return;
		}
		if (!foreign_streams[i].valid && free_idx < 0) {
			free_idx = i;
		}
	}

	/* New entry */
	if (free_idx >= 0) {
		foreign_streams[free_idx] = *parsed;
		foreign_streams[free_idx].last_seen_ms = k_uptime_get();

		char addr_str[INET_ADDRSTRLEN];

		zsock_inet_ntop(AF_INET, &parsed->mcast_addr,
				addr_str, sizeof(addr_str));
		LOG_INF("SAP: Discovered stream: %s @ %s:%u "
			"(%uch %ubit %uHz)",
			parsed->name, addr_str, parsed->port,
			parsed->channels, parsed->bit_depth,
			parsed->sample_rate);
	} else {
		LOG_WRN("SAP: Foreign stream table full");
	}

	k_mutex_unlock(&sap_mutex);
}

/* ================================================================
 * SAP thread: TX announcements + RX foreign announcements
 * ================================================================ */
static void sap_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	int sock;
	int ret;

	LOG_INF("SAP: Thread starting");

	/* Wait for network interface to be up */
	while (!net_if_is_up(sap_iface)) {
		k_msleep(500);
	}

	/* Wait until we have a valid IP */
	LOG_INF("SAP: Waiting for valid IP address...");
	k_sem_take(&sap_ip_sem, K_FOREVER);
	LOG_INF("SAP: IP ready, opening socket");

	/* Create UDP socket */
	sock = zsock_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (sock < 0) {
		LOG_ERR("SAP: Failed to create socket: %d", errno);
		return;
	}
	sap_sock = sock;  /* Store for multicast rejoin */

	/* Bind to SAP port for receiving */
	struct sockaddr_in bind_addr;

	memset(&bind_addr, 0, sizeof(bind_addr));
	bind_addr.sin_family = AF_INET;
	bind_addr.sin_port = htons(SAP_PORT);
	bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);

	ret = zsock_bind(sock, (struct sockaddr *)&bind_addr,
			 sizeof(bind_addr));
	if (ret < 0) {
		LOG_ERR("SAP: Failed to bind socket: %d", errno);
		zsock_close(sock);
		return;
	}

	/* Join SAP multicast group 239.255.255.255 */
	struct ip_mreqn mreq;

	memset(&mreq, 0, sizeof(mreq));
	zsock_inet_pton(AF_INET, SAP_MULTICAST_ADDR, &mreq.imr_multiaddr);
	mreq.imr_ifindex = net_if_get_by_iface(sap_iface);

	ret = zsock_setsockopt(sock, IPPROTO_IP, IP_ADD_MEMBERSHIP,
			       &mreq, sizeof(mreq));
	if (ret < 0) {
		LOG_WRN("SAP: Failed to join multicast %s: %d (continuing)",
			SAP_MULTICAST_ADDR, errno);
	} else {
		LOG_INF("SAP: Joined multicast group %s", SAP_MULTICAST_ADDR);
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

		/* Send announcements if interval elapsed or forced */
		if (announce_enabled &&
		    (force_announce ||
		     (now - last_announce_ms) >= SAP_ANNOUNCE_INTERVAL_S * 1000)) {
			force_announce = false;

			/* Announce TX streams */
			k_mutex_lock(&sap_mutex, K_FOREVER);
			for (int i = 0; i < AES67_MAX_TX_STREAMS; i++) {
				if (tx_streams[i].active) {
					send_sap_announce_stream(sock,
								 &sap_dst,
								 &tx_streams[i]);
				}
			}
			k_mutex_unlock(&sap_mutex);

			last_announce_ms = now;
		}

		/* Compute remaining time until next announcement */
		int64_t next_tx_ms = last_announce_ms +
				     SAP_ANNOUNCE_INTERVAL_S * 1000;
		int poll_timeout_ms = (int)(next_tx_ms - now);

		if (poll_timeout_ms < 100) {
			poll_timeout_ms = 100;
		}
		if (poll_timeout_ms > 2000) {
			poll_timeout_ms = 2000;
		}

		/* Poll for incoming SAP packets */
		struct zsock_pollfd pfd = {
			.fd = sock,
			.events = ZSOCK_POLLIN,
		};
		int pret = zsock_poll(&pfd, 1, poll_timeout_ms);

		if (pret > 0) {
			struct sockaddr_in src_addr;
			socklen_t src_len = sizeof(src_addr);

			ssize_t n = zsock_recvfrom(sock, rx_buf, sizeof(rx_buf),
						    ZSOCK_MSG_DONTWAIT,
						    (struct sockaddr *)&src_addr,
						    &src_len);
			if (n > 0) {
				/* Don't process our own announcements */
				if (src_addr.sin_addr.s_addr !=
				    my_ip_addr.s_addr) {
					struct sap_foreign_stream parsed;

					ret = parse_sap_sdp(rx_buf, n,
							     &parsed);
					if (ret >= 0) {
						foreign_stream_update(&parsed);
					}
				}
			}
		}

		/* Periodically expire old entries */
		k_mutex_lock(&sap_mutex, K_FOREVER);
		foreign_stream_expire();
		k_mutex_unlock(&sap_mutex);
	}
}

/* ================================================================
 * Shell commands: "aes67" subcommands
 * ================================================================ */

static int cmd_aes67_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	char mcast_str[INET_ADDRSTRLEN];
	char ip_str[INET_ADDRSTRLEN];

	zsock_inet_ntop(AF_INET, &local_config.mcast_addr,
			mcast_str, sizeof(mcast_str));
	zsock_inet_ntop(AF_INET, &my_ip_addr, ip_str, sizeof(ip_str));

	shell_print(sh, "=== AES67 Stream Configuration ===");
	shell_print(sh, "Source IP:    %s", ip_ready ? ip_str : "(no IP)");
	shell_print(sh, "Multicast:    %s:%u", mcast_str, local_config.port);
	shell_print(sh, "Audio:        %uch %ubit %uHz",
		    local_config.channels, local_config.bit_depth,
		    local_config.sample_rate);
	shell_print(sh, "Pkt samples:  %u", local_config.samples_per_packet);
	shell_print(sh, "Payload type: %u", local_config.payload_type);
	shell_print(sh, "SAP announce: %s", announce_enabled ? "ON" : "OFF");

	/* TX streams */
	shell_print(sh, "\n=== TX Streams ===");
	k_mutex_lock(&sap_mutex, K_FOREVER);
	{
		int tx_count = 0;

		for (int i = 0; i < AES67_MAX_TX_STREAMS; i++) {
			if (tx_streams[i].active) {
				char dst_str[INET_ADDRSTRLEN];

				zsock_inet_ntop(AF_INET, &tx_streams[i].dst_ip,
						dst_str, sizeof(dst_str));
				shell_print(sh, "  [%u] dst=%s ch=%u spp=%u ids=[",
					    tx_streams[i].stream_id, dst_str,
					    tx_streams[i].channel_count,
					    tx_streams[i].samples_per_packet);
				for (int j = 0; j < tx_streams[i].channel_count; j++) {
					shell_print(sh, "    %u%s",
						    tx_streams[i].ch_ids[j],
						    j < tx_streams[i].channel_count - 1
						    ? "," : "");
				}
				shell_print(sh, "  ]");
				tx_count++;
			}
		}
		if (tx_count == 0) {
			shell_print(sh, "  (none)");
		}
	}
	k_mutex_unlock(&sap_mutex);

	/* Discovered streams */
	shell_print(sh, "\n=== Discovered Streams ===");
	int count = 0;

	k_mutex_lock(&sap_mutex, K_FOREVER);
	for (int i = 0; i < SAP_MAX_FOREIGN_STREAMS; i++) {
		if (foreign_streams[i].valid) {
			char addr_str[INET_ADDRSTRLEN];

			zsock_inet_ntop(AF_INET,
					&foreign_streams[i].mcast_addr,
					addr_str, sizeof(addr_str));
			shell_print(sh, "  [%d] %s @ %s:%u  %uch %ubit %uHz",
				    count, foreign_streams[i].name,
				    addr_str, foreign_streams[i].port,
				    foreign_streams[i].channels,
				    foreign_streams[i].bit_depth,
				    foreign_streams[i].sample_rate);
			count++;
		}
	}
	k_mutex_unlock(&sap_mutex);

	if (count == 0) {
		shell_print(sh, "  (none)");
	}

	return 0;
}

static int cmd_aes67_mcast(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 2) {
		shell_error(sh, "Usage: aes67 mcast <ip> [port]");
		return -EINVAL;
	}

	struct in_addr new_addr;

	if (zsock_inet_pton(AF_INET, argv[1], &new_addr) != 1) {
		shell_error(sh, "Invalid IP address: %s", argv[1]);
		return -EINVAL;
	}

	/* Verify it's a multicast address (224.0.0.0 - 239.255.255.255) */
	uint8_t first_octet = ((uint8_t *)&new_addr.s_addr)[0];

	if (first_octet < 224 || first_octet > 239) {
		shell_error(sh, "Not a multicast address (must be 224-239.x.x.x)");
		return -EINVAL;
	}

	uint16_t new_port = local_config.port;

	if (argc >= 3) {
		unsigned long p = strtoul(argv[2], NULL, 10);

		if (p == 0 || p > 65535) {
			shell_error(sh, "Invalid port: %s", argv[2]);
			return -EINVAL;
		}
		new_port = (uint16_t)p;
	}

	int ret = sap_sdp_set_mcast(&new_addr, new_port);

	if (ret == 0) {
		char addr_str[INET_ADDRSTRLEN];

		zsock_inet_ntop(AF_INET, &new_addr, addr_str,
				sizeof(addr_str));
		shell_print(sh, "Multicast set to %s:%u", addr_str, new_port);
	}

	return ret;
}

static int cmd_aes67_announce(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 2) {
		shell_print(sh, "SAP announce: %s",
			    announce_enabled ? "ON" : "OFF");
		return 0;
	}

	if (strcmp(argv[1], "on") == 0) {
		sap_sdp_set_announce(true);
		shell_print(sh, "SAP announce enabled");
	} else if (strcmp(argv[1], "off") == 0) {
		sap_sdp_set_announce(false);
		shell_print(sh, "SAP announce disabled");
	} else {
		shell_error(sh, "Usage: aes67 announce [on|off]");
		return -EINVAL;
	}

	return 0;
}

static int cmd_aes67_txstream(const struct shell *sh, size_t argc, char **argv)
{
	/* aes67 txstream <stream_id> <dst_ip> <ch_count> <samples_per_pkt> <ch0> [ch1] ... */
	if (argc < 6) {
		shell_error(sh, "Usage: aes67 txstream <id 0-7> <dst_ip> "
			    "<ch_count 1-8> <samples_per_pkt> <ch0> [ch1..ch7]");
		return -EINVAL;
	}

	unsigned long id = strtoul(argv[1], NULL, 10);

	if (id > 7) {
		shell_error(sh, "stream_id must be 0..7");
		return -EINVAL;
	}

	struct in_addr dst;

	if (zsock_inet_pton(AF_INET, argv[2], &dst) != 1) {
		shell_error(sh, "Invalid IP: %s", argv[2]);
		return -EINVAL;
	}

	unsigned long ch_count = strtoul(argv[3], NULL, 10);

	if (ch_count < 1 || ch_count > 8) {
		shell_error(sh, "ch_count must be 1..8");
		return -EINVAL;
	}

	unsigned long spp = strtoul(argv[4], NULL, 10);

	if (spp < 1 || spp > 255) {
		shell_error(sh, "samples_per_pkt must be 1..255");
		return -EINVAL;
	}

	/* Remaining args are channel IDs */
	int num_ch_args = argc - 5;

	if (num_ch_args < (int)ch_count) {
		shell_error(sh, "Expected %lu channel IDs, got %d", ch_count, num_ch_args);
		return -EINVAL;
	}

	uint8_t ch_ids[8] = {0};

	for (int i = 0; i < (int)ch_count && i < 8; i++) {
		unsigned long cid = strtoul(argv[5 + i], NULL, 10);

		if (cid > 255) {
			shell_error(sh, "Channel ID must be 0..255");
			return -EINVAL;
		}
		ch_ids[i] = (uint8_t)cid;
	}

	int ret = sap_sdp_configure_tx_stream((uint8_t)id, &dst,
					      (uint8_t)ch_count,
					      (uint8_t)spp,
					      ch_ids, (uint8_t)ch_count,
					      0); /* SSRC: auto-generate */
	if (ret < 0) {
		shell_error(sh, "Failed to configure stream: %d", ret);
		return ret;
	}

	char addr_str[INET_ADDRSTRLEN];

	zsock_inet_ntop(AF_INET, &dst, addr_str, sizeof(addr_str));
	shell_print(sh, "TX stream %lu: dst=%s ch=%lu spp=%lu channels=[",
		    id, addr_str, ch_count, spp);
	for (int i = 0; i < (int)ch_count; i++) {
		shell_print(sh, "  %u%s", ch_ids[i],
			    i < (int)ch_count - 1 ? "," : "");
	}
	shell_print(sh, "]");

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(aes67_cmds,
	SHELL_CMD(status, NULL, "Show AES67 stream config and discovered streams",
		  cmd_aes67_status),
	SHELL_CMD(mcast, NULL, "Set audio multicast address: aes67 mcast <ip> [port]",
		  cmd_aes67_mcast),
	SHELL_CMD(announce, NULL, "Enable/disable SAP: aes67 announce [on|off]",
		  cmd_aes67_announce),
	SHELL_CMD(txstream, NULL,
		  "Configure TX stream: aes67 txstream <id> <ip> <ch_count> <spp> <ch0> [ch1..7]",
		  cmd_aes67_txstream),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(aes67, &aes67_cmds, "AES67 stream configuration", NULL);

/* ================================================================
 * Public API
 * ================================================================ */

int sap_sdp_start(struct net_if *iface)
{
	if (!iface) {
		return -EINVAL;
	}

	sap_iface = iface;

	/* Derive clock identity from MAC */
	struct net_linkaddr *ll = net_if_get_link_addr(iface);

	if (!ll || ll->len < 6) {
		LOG_ERR("SAP: No valid MAC address on interface");
		return -EINVAL;
	}

	mac_to_clock_identity(ll->addr, my_clock_id);

	/* Initialize default stream config */
	zsock_inet_pton(AF_INET, AES67_DEFAULT_MCAST_ADDR,
			&local_config.mcast_addr);
	local_config.port = AES67_DEFAULT_PORT;
	local_config.channels = AES67_DEFAULT_CHANNELS;
	local_config.bit_depth = AES67_DEFAULT_BIT_DEPTH;
	local_config.sample_rate = AES67_DEFAULT_SAMPLE_RATE;
	local_config.samples_per_packet = AES67_DEFAULT_SAMPLES_PER_PKT;
	local_config.payload_type = AES67_DEFAULT_PAYLOAD_TYPE;

	/* Compute SAP message ID hash from our IP (will be updated on DHCP) */
	sap_msg_id_hash = 0;

	/* Init sync primitives */
	k_sem_init(&sap_ip_sem, 0, 1);
	k_mutex_init(&sap_mutex);
	memset(foreign_streams, 0, sizeof(foreign_streams));
	memset(tx_streams, 0, sizeof(tx_streams));
	memset(rx_streams, 0, sizeof(rx_streams));
	force_announce = false;
	ip_ready = false;

	/* Start SAP thread */
	k_thread_create(&sap_thread_data, sap_stack, SAP_STACK_SIZE,
			sap_thread_fn, NULL, NULL, NULL,
			SAP_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&sap_thread_data, "sap_sdp");

	LOG_INF("SAP/SDP: Started");
	return 0;
}

void sap_sdp_notify_ip_ready(const struct in_addr *addr)
{
	memcpy(&my_ip_addr, addr, sizeof(my_ip_addr));

	/* Generate message ID hash from IP */
	uint32_t ip_val = sys_be32_to_cpu(addr->s_addr);

	sap_msg_id_hash = (uint16_t)(ip_val ^ (ip_val >> 16));

	ip_ready = true;
	k_sem_give(&sap_ip_sem);
	LOG_INF("SAP: IP-ready notification received");
}

const struct aes67_stream_config *sap_sdp_get_config(void)
{
	return &local_config;
}

int sap_sdp_set_mcast(const struct in_addr *addr, uint16_t port)
{
	k_mutex_lock(&sap_mutex, K_FOREVER);
	memcpy(&local_config.mcast_addr, addr, sizeof(local_config.mcast_addr));
	if (port > 0) {
		local_config.port = port;
	}
	uint16_t effective_port = local_config.port;

	k_mutex_unlock(&sap_mutex);

	char addr_str[INET_ADDRSTRLEN];

	zsock_inet_ntop(AF_INET, addr, addr_str, sizeof(addr_str));
	LOG_INF("SAP: Multicast config updated to %s:%u",
		addr_str, effective_port);

	return 0;
}

void sap_sdp_set_announce(bool enable)
{
	announce_enabled = enable;
	LOG_INF("SAP: Announcements %s", enable ? "enabled" : "disabled");
}

const struct sap_foreign_stream *sap_sdp_get_foreign_streams(int *count)
{
	int n = 0;

	for (int i = 0; i < SAP_MAX_FOREIGN_STREAMS; i++) {
		if (foreign_streams[i].valid) {
			n++;
		}
	}
	if (count) {
		*count = n;
	}
	return foreign_streams;
}

int sap_sdp_configure_tx_stream(uint8_t stream_id,
				const struct in_addr *dst_ip,
				uint8_t channel_count,
				uint8_t samples_per_pkt,
				const uint8_t *ch_ids,
				uint8_t num_ch_ids,
				uint32_t ssrc)
{
	if (stream_id >= AES67_MAX_TX_STREAMS || !dst_ip) {
		return -EINVAL;
	}

	/* Auto-generate SSRC if not provided (use IP + stream_id as seed) */
	uint32_t effective_ssrc = ssrc;
	if (effective_ssrc == 0) {
		effective_ssrc = sys_be32_to_cpu(dst_ip->s_addr) ^ 
				 (stream_id << 24) ^
				 sys_be32_to_cpu(my_ip_addr.s_addr);
	}

	/* Write to FPGA */
	const struct device *fmc = device_get_binding("eth_fmc0");

	if (!fmc) {
		LOG_ERR("SAP: FMC device not found");
		return -ENODEV;
	}

	int ret = eth_fmc_write_tx_stream_config(fmc, stream_id, dst_ip,
						 channel_count,
						 samples_per_pkt,
						 ch_ids, num_ch_ids,
						 effective_ssrc);
	if (ret < 0) {
		LOG_ERR("SAP: Failed to write stream %u to FPGA: %d",
			stream_id, ret);
		return ret;
	}

	/* Update local table */
	k_mutex_lock(&sap_mutex, K_FOREVER);
	tx_streams[stream_id].active = true;
	tx_streams[stream_id].stream_id = stream_id;
	memcpy(&tx_streams[stream_id].dst_ip, dst_ip, sizeof(*dst_ip));
	tx_streams[stream_id].channel_count = channel_count;
	tx_streams[stream_id].samples_per_packet = samples_per_pkt;
	memset(tx_streams[stream_id].ch_ids, 0,
	       sizeof(tx_streams[stream_id].ch_ids));
	for (uint8_t i = 0; i < num_ch_ids && i < AES67_MAX_CH_PER_STREAM; i++) {
		tx_streams[stream_id].ch_ids[i] = ch_ids[i];
	}
	tx_streams[stream_id].ssrc = effective_ssrc;
	k_mutex_unlock(&sap_mutex);

	/* Force immediate SAP announcement */
	force_announce = true;

	char addr_str[INET_ADDRSTRLEN];

	zsock_inet_ntop(AF_INET, dst_ip, addr_str, sizeof(addr_str));
	LOG_INF("SAP: TX stream %u configured: dst=%s ch=%u spp=%u ssrc=0x%08x",
		stream_id, addr_str, channel_count, samples_per_pkt, effective_ssrc);

	return 0;
}

const struct aes67_tx_stream *sap_sdp_get_tx_streams(void)
{
	return tx_streams;
}

int sap_sdp_configure_rx_stream(uint8_t stream_id,
				uint32_t ssrc,
				const uint8_t *ch_map,
				uint8_t channel_count,
				uint8_t output_delay,
				uint8_t samples_per_channel)
{
	if (stream_id >= AES67_MAX_RX_STREAMS || !ch_map ||
	    channel_count == 0 || channel_count > AES67_MAX_CH_PER_STREAM) {
		return -EINVAL;
	}

	const struct device *fmc = device_get_binding("eth_fmc0");

	if (!fmc) {
		LOG_ERR("SAP: FMC device not found");
		return -ENODEV;
	}

	int ret = eth_fmc_write_rx_stream_config(fmc, stream_id, ssrc,
						 ch_map, channel_count,
						 output_delay,
						 samples_per_channel);
	if (ret < 0) {
		LOG_ERR("SAP: Failed to write RX stream %u to FPGA: %d",
			stream_id, ret);
		return ret;
	}

	k_mutex_lock(&sap_mutex, K_FOREVER);
	rx_streams[stream_id].active = true;
	rx_streams[stream_id].stream_id = stream_id;
	rx_streams[stream_id].ssrc = ssrc;
	rx_streams[stream_id].channel_count = channel_count;
	rx_streams[stream_id].output_delay = output_delay;
	rx_streams[stream_id].samples_per_channel = samples_per_channel;
	memset(rx_streams[stream_id].ch_map, 0,
	       sizeof(rx_streams[stream_id].ch_map));
	for (uint8_t i = 0; i < channel_count; i++) {
		rx_streams[stream_id].ch_map[i] = ch_map[i];
	}
	k_mutex_unlock(&sap_mutex);

	LOG_INF("SAP: RX stream %u configured: ssrc=0x%08x ch=%u spc=%u delay=%u",
		stream_id, ssrc, channel_count, samples_per_channel, output_delay);
	return 0;
}

const struct aes67_rx_stream *sap_sdp_get_rx_streams(void)
{
	return rx_streams;
}

/* ================================================================
 * Rejoin SAP multicast group after link up
 *
 * Called when Ethernet link transitions from down to up to ensure
 * IGMP membership is re-established.
 * ================================================================ */
void sap_sdp_notify_link_up(void)
{
	if (sap_sock < 0 || !sap_iface) {
		LOG_DBG("SAP: link_up notify ignored (no socket yet)");
		return;
	}

	struct ip_mreqn mreq;

	memset(&mreq, 0, sizeof(mreq));
	zsock_inet_pton(AF_INET, SAP_MULTICAST_ADDR, &mreq.imr_multiaddr);
	mreq.imr_ifindex = net_if_get_by_iface(sap_iface);

	int ret = zsock_setsockopt(sap_sock, IPPROTO_IP, IP_ADD_MEMBERSHIP,
				   &mreq, sizeof(mreq));
	if (ret < 0 && errno != EADDRINUSE) {
		LOG_WRN("SAP: Failed to rejoin multicast %s: %d",
			SAP_MULTICAST_ADDR, errno);
	} else {
		LOG_INF("SAP: Rejoined SAP multicast group %s", SAP_MULTICAST_ADDR);
	}
}
