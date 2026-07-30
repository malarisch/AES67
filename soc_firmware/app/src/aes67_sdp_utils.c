/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: Apache-2.0
 *
 * AES67 SDP utility functions implementation.
 */

#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>
#include <zephyr/sys/byteorder.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>

#include "aes67_sdp_utils.h"
#include "sap_sdp.h"  /* For AES67 defaults */

/* ================================================================
 * String Helpers
 * ================================================================ */

int aes67_strncasecmp(const char *s1, const char *s2, size_t n)
{
	for (size_t i = 0; i < n; i++) {
		int c1 = tolower((unsigned char)s1[i]);
		int c2 = tolower((unsigned char)s2[i]);
		if (c1 != c2) {
			return c1 - c2;
		}
		if (c1 == '\0') {
			return 0;
		}
	}
	return 0;
}

const char *aes67_parse_uint(const char *p, const char *end, uint32_t *out)
{
	uint32_t val = 0;
	while (p < end && *p >= '0' && *p <= '9') {
		val = val * 10 + (uint32_t)(*p - '0');
		p++;
	}
	*out = val;
	return p;
}

/* ================================================================
 * PTP Clock ID Formatting
 * ================================================================ */

int aes67_format_clock_id(char *buf, size_t buf_len, const uint8_t id[8])
{
	return snprintf(buf, buf_len,
			"%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X",
			id[0], id[1], id[2], id[3],
			id[4], id[5], id[6], id[7]);
}

/* ================================================================
 * SDP Generation
 *
 * Generates RAVENNA-compliant SDP with required extensions:
 *   - a=clock-domain:PTPv2 <domain>  (session-level, REQUIRED)
 *   - a=sync-time:<timestamp>        (stream-level, for phase-accurate playback)
 * ================================================================ */

int aes67_sdp_build(char *buf, size_t buf_size,
		    const struct aes67_sdp_params *params)
{
	char origin_str[INET_ADDRSTRLEN];
	char conn_str[INET_ADDRSTRLEN];
	char clock_id_str[32];

	zsock_inet_ntop(AF_INET, &params->origin_addr, origin_str, sizeof(origin_str));
	zsock_inet_ntop(AF_INET, &params->connection_addr, conn_str, sizeof(conn_str));

	if (params->clock_id) {
		aes67_format_clock_id(clock_id_str, sizeof(clock_id_str),
				      params->clock_id);
	}

	/* Compute ptime: samples_per_packet * 1000000 / sample_rate */
	uint32_t ptime_us = aes67_ptime_us(params->samples_per_packet,
					   params->sample_rate);

	/* Session ID: use origin IP + stream_id for uniqueness */
	uint32_t session_id = sys_be32_to_cpu(params->origin_addr.s_addr) +
			      params->stream_id;

	const char *stream_name = params->stream_name;
	if (!stream_name || stream_name[0] == '\0') {
		stream_name = "AES67 Stream";
	}

	size_t pos = 0;

	/* snprintf() reports the length it WOULD have written, so a single
	 * truncated line already pushes pos past buf_size. Appending after
	 * that would compute `buf_size - pos` as a size_t underflow and
	 * write beyond the buffer — bail out on the first overflow instead
	 * and let the caller see -ENOMEM. */
#define SDP_APPEND(...)                                                     \
	do {                                                                \
		int _n;                                                     \
		if (pos >= buf_size) {                                      \
			return -ENOMEM;                                     \
		}                                                           \
		_n = snprintf(buf + pos, buf_size - pos, __VA_ARGS__);      \
		if (_n < 0) {                                               \
			return -ENOMEM;                                     \
		}                                                           \
		pos += (size_t)_n;                                          \
	} while (0)

	/* === Session-level description === */

	/* v= (protocol version) */
	SDP_APPEND("v=0\r\n");

	/* o= (origin) */
	SDP_APPEND("o=- %u %u IN IP4 %s\r\n",
		   session_id, params->stream_id, origin_str);

	/* s= (session name) */
	SDP_APPEND("s=%s\r\n", stream_name);

	/* i= (session info) */
	SDP_APPEND("i=%uch %ubit %uHz\r\n",
		   params->channel_count, params->bit_depth,
		   params->sample_rate);

	/* t= (timing) */
	SDP_APPEND("t=0 0\r\n");

	/* a=clock-domain (RAVENNA REQUIRED - session-level sync source) */
	SDP_APPEND("a=clock-domain:PTPv2 %u\r\n", params->ptp_domain);

	/* === Media-level description === */

	/* m= (media) */
	SDP_APPEND("m=audio %u RTP/AVP %u\r\n",
		   params->port, params->payload_type);

	/* c= (connection data). Media-level rather than session-level:
	 * RFC 4566 allows either, but NMOS controllers (and the
	 * nmos-testing SDP checks) look for it inside the media section. */
	SDP_APPEND("c=IN IP4 %s/32\r\n", conn_str);

	/* a=source-filter (RFC 4570): ST 2110-10 requires multicast
	 * sessions to declare the expected source address (SSM/IGMPv3). */
	if ((sys_be32_to_cpu(params->connection_addr.s_addr) >> 28) == 0xe &&
	    params->origin_addr.s_addr != 0) {
		SDP_APPEND("a=source-filter: incl IN IP4 %s %s\r\n",
			   conn_str, origin_str);
	}

	/* a=rtpmap */
	SDP_APPEND("a=rtpmap:%u L%u/%u/%u\r\n",
		   params->payload_type, params->bit_depth,
		   params->sample_rate, params->channel_count);

	/* a=ptime */
	SDP_APPEND("a=ptime:%u.%03u\r\n", ptime_us / 1000, ptime_us % 1000);

	/* a=sync-time (RAVENNA - stream-level timestamp association) */
	SDP_APPEND("a=sync-time:%u\r\n", params->sync_time);

	/* a=ts-refclk (AES67 reference clock): the elected PTP grandmaster
	 * incl. domain.  Omitted while no grandmaster is known (clock_id ==
	 * NULL) — a zero placeholder would claim traceability to a clock
	 * that does not exist.  Matches the Linux daemon's behaviour. */
	if (params->clock_id) {
		SDP_APPEND("a=ts-refclk:ptp=IEEE1588-2008:%s:%u\r\n",
			   clock_id_str, params->ptp_domain);
	}

	/* a=mediaclk */
	SDP_APPEND("a=mediaclk:direct=0\r\n");

	/* a=ssrc (optional) */
	if (params->ssrc != 0) {
		SDP_APPEND("a=ssrc:%u cname:aes67@%s\r\n",
			   params->ssrc, origin_str);
	}

#undef SDP_APPEND

	if (pos >= buf_size) {
		return -ENOMEM;
	}
	return (int)pos;
}

/* ================================================================
 * SDP Parsing
 * ================================================================ */

int aes67_sdp_parse(const char *sdp, size_t sdp_len,
		    struct aes67_sdp_parsed *out)
{
	memset(out, 0, sizeof(*out));

	const char *line = sdp;
	const char *end = sdp + sdp_len;

	while (line < end) {
		const char *eol = memchr(line, '\n', end - line);
		size_t line_len = eol ? (size_t)(eol - line) : (size_t)(end - line);

		/* Strip trailing \r */
		size_t clean_len = line_len;
		if (clean_len > 0 && line[clean_len - 1] == '\r') {
			clean_len--;
		}

		if (clean_len < 2) {
			goto next_line;
		}

		char type = line[0];
		char eq = line[1];
		if (eq != '=') {
			goto next_line;
		}

		const char *val = line + 2;
		size_t val_len = clean_len - 2;

		switch (type) {
		case 's':
			/* Session name */
			if (val_len >= AES67_SDP_NAME_MAX) {
				val_len = AES67_SDP_NAME_MAX - 1;
			}
			memcpy(out->name, val, val_len);
			out->name[val_len] = '\0';
			break;

		case 'o':
			/* Origin: o=<username> <sess_id> <version> IN IP4 <addr> */
			{
				const char *first_space = memchr(val, ' ', val_len);
				size_t name_len = first_space ?
					(size_t)(first_space - val) : val_len;

				if (!(name_len == 1 && val[0] == '-')) {
					name_len = MIN(name_len,
						       sizeof(out->origin_name) - 1);
					memcpy(out->origin_name, val, name_len);
					out->origin_name[name_len] = '\0';
				}

				/* Find last token which is IP address */
				const char *p = val + val_len;
				while (p > val && *(p - 1) != ' ') {
					p--;
				}
				if (p < val + val_len) {
					char addr_buf[INET_ADDRSTRLEN];
					size_t addr_len = (val + val_len) - p;
					if (addr_len < sizeof(addr_buf)) {
						memcpy(addr_buf, p, addr_len);
						addr_buf[addr_len] = '\0';
						zsock_inet_pton(AF_INET, addr_buf,
								&out->origin_addr);
					}
				}
			}
			break;

		case 'c':
			/* Connection: c=IN IP4 <addr>[/TTL] */
			{
				const char *marker = "IN IP4 ";
				size_t marker_len = 7;
				const char *p = val;
				const char *p_end = val + val_len;

				/* Find "IN IP4 " */
				for (; p + marker_len <= p_end; p++) {
					if (memcmp(p, marker, marker_len) == 0) {
						p += marker_len;
						break;
					}
				}
				if (p < p_end) {
					char addr_buf[INET_ADDRSTRLEN];
					const char *slash = memchr(p, '/', p_end - p);
					size_t addr_len = slash ?
						(size_t)(slash - p) :
						(size_t)(p_end - p);
					if (addr_len < sizeof(addr_buf)) {
						memcpy(addr_buf, p, addr_len);
						addr_buf[addr_len] = '\0';
						zsock_inet_pton(AF_INET, addr_buf,
								&out->connection_addr);
					}
				}
			}
			break;

		case 'm':
			/* Media: m=audio <port> RTP/AVP <pt> */
			if (val_len > 6 && memcmp(val, "audio ", 6) == 0) {
				uint32_t port_val = 0;
				aes67_parse_uint(val + 6, val + val_len, &port_val);
				out->port = (uint16_t)port_val;
			}
			break;

		case 'a':
			/* Attributes */
			if (val_len > 7 && memcmp(val, "rtpmap:", 7) == 0) {
				/* a=rtpmap:<pt> L<depth>/<rate>/<ch> */
				const char *p = val + 7;
				const char *p_end = val + val_len;

				/* Skip payload type */
				while (p < p_end && *p >= '0' && *p <= '9') p++;
				if (p < p_end && *p == ' ') p++;

				/* Expect L<depth>/<rate>/<ch> */
				if (p < p_end && *p == 'L') {
					p++;
					uint32_t depth = 0, rate = 0, ch = 0;
					p = aes67_parse_uint(p, p_end, &depth);
					if (p < p_end && *p == '/') {
						p++;
						p = aes67_parse_uint(p, p_end, &rate);
					}
					if (p < p_end && *p == '/') {
						p++;
						aes67_parse_uint(p, p_end, &ch);
					}
					out->bit_depth = (uint8_t)depth;
					out->sample_rate = rate;
					out->channels = (uint8_t)ch;
				}
			} else if (val_len > 5 && memcmp(val, "ssrc:", 5) == 0) {
				/* a=ssrc:<ssrc> cname:... */
				uint32_t ssrc_val = 0;
				aes67_parse_uint(val + 5, val + val_len, &ssrc_val);
				out->ssrc = ssrc_val;
			} else if (val_len > 6 && memcmp(val, "ptime:", 6) == 0) {
				/* a=ptime:<ms>[.<frac>] */
				const char *p = val + 6;
				const char *p_end = val + val_len;
				uint32_t ptime_ms = 0;
				uint32_t ptime_frac_us = 0;

				p = aes67_parse_uint(p, p_end, &ptime_ms);
				if (p < p_end && *p == '.') {
					p++;
					/* ptime is in milliseconds, so the first
					 * fractional digit is worth 100 us.
					 * Digits past microsecond resolution
					 * are dropped (scale reaches 0). */
					uint32_t scale = 100U;
					while (p < p_end && *p >= '0' && *p <= '9' && scale > 0) {
						ptime_frac_us += (uint32_t)(*p - '0') * scale;
						scale /= 10;
						p++;
					}
				}
				/* Store raw ptime in us temporarily */
				out->samples_per_packet = (uint16_t)(ptime_ms * 1000U + ptime_frac_us);
			} else if (val_len > 13 && memcmp(val, "clock-domain:", 13) == 0) {
				/* a=clock-domain:PTPv2 <domain> (RAVENNA session-level) */
				const char *p = val + 13;
				const char *p_end = val + val_len;
				/* Skip "PTPv2 " prefix if present */
				if (p + 6 <= p_end && memcmp(p, "PTPv2 ", 6) == 0) {
					p += 6;
					uint32_t domain = 0;
					aes67_parse_uint(p, p_end, &domain);
					out->ptp_domain = (uint8_t)domain;
					out->has_clock_domain = true;
				}
			} else if (val_len > 10 && memcmp(val, "sync-time:", 10) == 0) {
				/* a=sync-time:<rtp_timestamp> (RAVENNA stream-level) */
				uint32_t timestamp = 0;
				aes67_parse_uint(val + 10, val + val_len, &timestamp);
				out->sync_time = timestamp;
				out->has_sync_time = true;
			}
			break;
		}

next_line:
		if (!eol) break;
		line = eol + 1;
	}

	/* Convert ptime to samples if we have sample_rate */
	if (out->samples_per_packet > 0 && out->sample_rate > 0) {
		uint32_t ptime_us = out->samples_per_packet;
		out->samples_per_packet = (uint16_t)(
			ptime_us * out->sample_rate / 1000000U);
	}

	return 0;
}
