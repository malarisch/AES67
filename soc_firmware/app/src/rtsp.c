/*
 * RAVENNA-compliant RTSP server and client for AES67 stream control.
 *
 * Implements RFC 2326 RTSP protocol for:
 *   - Server: Exposes TX streams for remote subscription
 *   - Client: Subscribes to remote streams for RX
 *
 * Stream URLs follow RAVENNA conventions:
 *   rtsp://<ip>:554/by-name/<stream_name>
 *   rtsp://<ip>:554/by-id/<stream_id>
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/socket.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/random/random.h>
#include <string.h>
#include <strings.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

#include "rtsp.h"
#include "aes67_conn.h"
#include "aes67_config.h"
#include "ptp_bmc.h"
#include "aes67_sdp_utils.h"

LOG_MODULE_REGISTER(rtsp, LOG_LEVEL_INF);

/* ---- Thread resources ---- */
#define RTSP_SERVER_STACK_SIZE  4096
#define RTSP_SERVER_PRIORITY    K_PRIO_PREEMPT(11)

K_THREAD_STACK_DEFINE(rtsp_server_stack, RTSP_SERVER_STACK_SIZE);
static struct k_thread rtsp_server_thread_data;

/* ---- Server state ---- */
static int rtsp_server_sock = -1;
static bool rtsp_server_running;
static struct k_mutex rtsp_mutex;
static struct rtsp_session rtsp_sessions[RTSP_MAX_SESSIONS];

/* ---- Client connections state ---- */
static struct rtsp_client_connection rtsp_clients[RTSP_MAX_CLIENTS];

/* ---- Local IP address (set after DHCP) ---- */
static struct in_addr local_ip_addr;

/* ---- Receive/send buffers (per-connection in thread) ---- */
static char rtsp_recv_buf[RTSP_RECV_BUF_SIZE];
static char rtsp_send_buf[RTSP_SEND_BUF_SIZE];

/* ================================================================
 * Method name lookup
 * ================================================================ */

static const char *method_names[] = {
	[RTSP_METHOD_OPTIONS]       = "OPTIONS",
	[RTSP_METHOD_DESCRIBE]      = "DESCRIBE",
	[RTSP_METHOD_ANNOUNCE]      = "ANNOUNCE",
	[RTSP_METHOD_SETUP]         = "SETUP",
	[RTSP_METHOD_PLAY]          = "PLAY",
	[RTSP_METHOD_PAUSE]         = "PAUSE",
	[RTSP_METHOD_TEARDOWN]      = "TEARDOWN",
	[RTSP_METHOD_GET_PARAMETER] = "GET_PARAMETER",
	[RTSP_METHOD_SET_PARAMETER] = "SET_PARAMETER",
};

/* Case-insensitive string comparison - use shared helper */
#define local_strncasecmp aes67_strncasecmp

static const char *get_status_phrase(rtsp_status_t status)
{
	switch (status) {
	case RTSP_STATUS_OK:                  return "OK";
	case RTSP_STATUS_CREATED:             return "Created";
	case RTSP_STATUS_BAD_REQUEST:         return "Bad Request";
	case RTSP_STATUS_UNAUTHORIZED:        return "Unauthorized";
	case RTSP_STATUS_NOT_FOUND:           return "Not Found";
	case RTSP_STATUS_METHOD_NOT_ALLOWED:  return "Method Not Allowed";
	case RTSP_STATUS_NOT_ACCEPTABLE:      return "Not Acceptable";
	case RTSP_STATUS_SESSION_NOT_FOUND:   return "Session Not Found";
	case RTSP_STATUS_METHOD_INVALID:      return "Method Not Valid in This State";
	case RTSP_STATUS_UNSUPPORTED_TRANSPORT: return "Unsupported Transport";
	case RTSP_STATUS_INTERNAL_ERROR:      return "Internal Server Error";
	case RTSP_STATUS_NOT_IMPLEMENTED:     return "Not Implemented";
	case RTSP_STATUS_SERVICE_UNAVAILABLE: return "Service Unavailable";
	default:                              return "Unknown";
	}
}

/* ================================================================
 * Parse RTSP method from string
 * ================================================================ */

static rtsp_method_t parse_method(const char *method_str, size_t len)
{
	for (int i = 0; i < ARRAY_SIZE(method_names); i++) {
		if (method_names[i] &&
		    strlen(method_names[i]) == len &&
		    strncmp(method_str, method_names[i], len) == 0) {
			return (rtsp_method_t)i;
		}
	}
	return RTSP_METHOD_UNKNOWN;
}

/* ================================================================
 * Parse RTSP request line and headers
 *
 * Format:
 *   METHOD rtsp://host/path RTSP/1.0\r\n
 *   CSeq: <number>\r\n
 *   Transport: RTP/AVP;unicast;client_port=5004-5005\r\n
 *   Session: <session_id>\r\n
 *   \r\n
 * ================================================================ */

static int parse_rtsp_request(const char *buf, size_t len,
			      struct rtsp_request *req)
{
	const char *p = buf;
	const char *end = buf + len;
	const char *line_end;

	memset(req, 0, sizeof(*req));
	req->method = RTSP_METHOD_UNKNOWN;
	req->transport_type = RTSP_TRANSPORT_UNKNOWN;

	/* Find first line end */
	line_end = strstr(p, "\r\n");
	if (!line_end) {
		return -EINVAL;
	}

	/* Parse request line: METHOD SP URL SP VERSION */
	const char *method_end = strchr(p, ' ');
	if (!method_end || method_end >= line_end) {
		return -EINVAL;
	}

	req->method = parse_method(p, method_end - p);
	if (req->method == RTSP_METHOD_UNKNOWN) {
		LOG_WRN("RTSP: Unknown method");
		return -ENOTSUP;
	}

	/* Parse URL */
	p = method_end + 1;
	const char *url_end = strchr(p, ' ');
	if (!url_end || url_end >= line_end) {
		return -EINVAL;
	}

	size_t url_len = MIN(url_end - p, RTSP_MAX_URL_LENGTH - 1);
	memcpy(req->url, p, url_len);
	req->url[url_len] = '\0';

	/* Skip past request line */
	p = line_end + 2;

	/* Parse headers */
	while (p < end) {
		line_end = strstr(p, "\r\n");
		if (!line_end) {
			break;
		}

		/* Empty line = end of headers */
		if (line_end == p) {
			break;
		}

		size_t line_len = line_end - p;

		/* CSeq: <number> */
		if (line_len > 6 && local_strncasecmp(p, "CSeq:", 5) == 0) {
			const char *val = p + 5;
			while (*val == ' ' && val < line_end) val++;
			req->cseq = (uint32_t)strtoul(val, NULL, 10);
		}
		/* Session: <id> */
		else if (line_len > 8 && local_strncasecmp(p, "Session:", 8) == 0) {
			const char *val = p + 8;
			while (*val == ' ' && val < line_end) val++;
			size_t id_len = MIN(line_end - val, sizeof(req->session_id) - 1);
			/* Session ID may have ;timeout=... suffix */
			const char *semi = memchr(val, ';', id_len);
			if (semi) {
				id_len = semi - val;
			}
			memcpy(req->session_id, val, id_len);
			req->session_id[id_len] = '\0';
			req->has_session = true;
		}
		/* Transport: RTP/AVP;... */
		else if (line_len > 10 && local_strncasecmp(p, "Transport:", 10) == 0) {
			const char *val = p + 10;
			while (*val == ' ' && val < line_end) val++;

			req->has_transport = true;

			/* Check transport type */
			if (strstr(val, "multicast")) {
				req->transport_type = RTSP_TRANSPORT_RTP_AVP_UDP_MULTICAST;
			} else {
				req->transport_type = RTSP_TRANSPORT_RTP_AVP_UDP_UNICAST;
			}

			/* Parse client_port=XXXX-YYYY */
			const char *cp = strstr(val, "client_port=");
			if (cp && cp < line_end) {
				cp += 12;
				req->client_port_rtp = (uint16_t)strtoul(cp, NULL, 10);
				const char *dash = strchr(cp, '-');
				if (dash && dash < line_end) {
					req->client_port_rtcp = (uint16_t)strtoul(dash + 1, NULL, 10);
				} else {
					req->client_port_rtcp = req->client_port_rtp + 1;
				}
			}

			/* Parse destination=<ip> for multicast */
			const char *dest = strstr(val, "destination=");
			if (dest && dest < line_end) {
				dest += 12;
				char ip_buf[INET_ADDRSTRLEN];
				size_t ip_len = 0;
				while (dest + ip_len < line_end &&
				       ip_len < sizeof(ip_buf) - 1 &&
				       dest[ip_len] != ';' &&
				       dest[ip_len] != '\r') {
					ip_buf[ip_len] = dest[ip_len];
					ip_len++;
				}
				ip_buf[ip_len] = '\0';
				zsock_inet_pton(AF_INET, ip_buf, &req->destination);
			}
		}

		p = line_end + 2;
	}

	return 0;
}

/* ================================================================
 * Extract stream ID from RTSP URL
 *
 * Supported formats:
 *   rtsp://host:port/by-id/<stream_id>
 *   rtsp://host:port/by-name/<name>
 *   rtsp://host:port/stream/<stream_id>
 *   rtsp://host:port/<stream_id>
 * ================================================================ */

static int extract_stream_id_from_url(const char *url, uint8_t *stream_id_out)
{
	const char *path;

	/* Skip rtsp:// prefix */
	if (strncmp(url, "rtsp://", 7) == 0) {
		path = strchr(url + 7, '/');
		if (!path) {
			return -EINVAL;
		}
		path++; /* Skip the '/' */
	} else if (url[0] == '/') {
		path = url + 1;
	} else {
		path = url;
	}

	/* Check for /by-id/<id> */
	if (strncmp(path, "by-id/", 6) == 0) {
		*stream_id_out = (uint8_t)strtoul(path + 6, NULL, 10);
		return 0;
	}

	/* Check for /stream/<id> */
	if (strncmp(path, "stream/", 7) == 0) {
		*stream_id_out = (uint8_t)strtoul(path + 7, NULL, 10);
		return 0;
	}

	/* Check for /by-name/<name> - lookup in TX streams table */
	if (strncmp(path, "by-name/", 8) == 0) {
		char name[AES67_STREAM_NAME_MAX];
		size_t n = 0;
		const char *p = path + 8;

		/* Percent-decode the path segment */
		while (*p && *p != ' ' && *p != '?' && *p != '\r' &&
		       n < sizeof(name) - 1) {
			if (p[0] == '%' && isxdigit((int)p[1]) &&
			    isxdigit((int)p[2])) {
				char hex[3] = { p[1], p[2], '\0' };

				name[n++] = (char)strtoul(hex, NULL, 16);
				p += 3;
			} else {
				name[n++] = *p++;
			}
		}
		name[n] = '\0';

		const struct aes67_tx_stream *tx = aes67_conn_get_tx_streams();

		for (int i = 0; i < AES67_MAX_TX_STREAMS; i++) {
			if (tx[i].active &&
			    strcasecmp(tx[i].name, name) == 0) {
				*stream_id_out = tx[i].stream_id;
				return 0;
			}
		}

		/* mDNS announces device-unique instance names in the form
		 * "<device name> <stream name>" (bare stream names collide
		 * between boards on the link) — accept that prefixed form by
		 * matching the stream name as a space-separated suffix. */
		size_t nlen = strlen(name);

		for (int i = 0; i < AES67_MAX_TX_STREAMS; i++) {
			size_t sl;

			if (!tx[i].active) {
				continue;
			}
			sl = strlen(tx[i].name);
			if (sl > 0 && sl < nlen &&
			    name[nlen - sl - 1] == ' ' &&
			    strcasecmp(name + (nlen - sl), tx[i].name) == 0) {
				*stream_id_out = tx[i].stream_id;
				return 0;
			}
		}
		LOG_WRN("RTSP: by-name: no TX stream named \"%s\"", name);
		return -ENOENT;
	}

	/* Try direct numeric ID */
	if (path[0] >= '0' && path[0] <= '9') {
		*stream_id_out = (uint8_t)strtoul(path, NULL, 10);
		return 0;
	}

	return -EINVAL;
}

/* ================================================================
 * Generate session ID
 * ================================================================ */

static void generate_session_id(char *buf, size_t len)
{
	uint32_t rnd = sys_rand32_get();
	snprintf(buf, len, "%08X", rnd);
}

/* ================================================================
 * Find or create session
 * ================================================================ */

static struct rtsp_session *find_session(const char *session_id)
{
	for (int i = 0; i < RTSP_MAX_SESSIONS; i++) {
		if (rtsp_sessions[i].active &&
		    strcmp(rtsp_sessions[i].session_id, session_id) == 0) {
			return &rtsp_sessions[i];
		}
	}
	return NULL;
}

static struct rtsp_session *create_session(int client_sock,
					   const struct sockaddr_in *client_addr,
					   uint8_t stream_id)
{
	k_mutex_lock(&rtsp_mutex, K_FOREVER);

	for (int i = 0; i < RTSP_MAX_SESSIONS; i++) {
		if (!rtsp_sessions[i].active) {
			struct rtsp_session *sess = &rtsp_sessions[i];
			memset(sess, 0, sizeof(*sess));
			sess->active = true;
			sess->client_sock = client_sock;
			sess->client_addr = client_addr->sin_addr;
			sess->stream_id = stream_id;
			sess->last_activity_ms = k_uptime_get();
			generate_session_id(sess->session_id, sizeof(sess->session_id));
			k_mutex_unlock(&rtsp_mutex);
			return sess;
		}
	}

	k_mutex_unlock(&rtsp_mutex);
	return NULL;
}

static void destroy_session(struct rtsp_session *sess)
{
	if (sess) {
		k_mutex_lock(&rtsp_mutex, K_FOREVER);
		sess->active = false;
		k_mutex_unlock(&rtsp_mutex);
	}
}

/* ================================================================
 * Build SDP for a TX stream using shared helper
 * ================================================================ */

static int build_sdp_for_stream(char *buf, size_t buf_size, uint8_t stream_id)
{
	const struct aes67_tx_stream *streams = aes67_conn_get_tx_streams();
	const struct aes67_tx_stream *stream = &streams[stream_id];

	if (!stream->active) {
		return -ENOENT;
	}

	/* ts-refclk: the elected PTP grandmaster (like the SAP announcer and
	 * the Linux daemon).  NULL while no grandmaster is known — the SDP
	 * then omits the a=ts-refclk line. */
	uint8_t gmid[8];
	const uint8_t *clock_id =
		(ptp_bmc_get_best_master_id(gmid) == 0) ? gmid : NULL;

	struct aes67_device_config *cfg = aes67_config_get();
	struct aes67_sdp_params params = {
		.origin_addr = local_ip_addr,
		.connection_addr = stream->dst_ip,
		.stream_id = stream->stream_id,
		.channel_count = stream->channel_count,
		.bit_depth = AES67_DEFAULT_BIT_DEPTH,
		.sample_rate = AES67_DEFAULT_SAMPLE_RATE,
		.samples_per_packet = stream->samples_per_packet,
		.port = AES67_DEFAULT_PORT,
		.payload_type = AES67_DEFAULT_PAYLOAD_TYPE,
		.ssrc = stream->ssrc,
		.clock_id = clock_id,
		.stream_name = stream->name,
		.ptp_domain = cfg->ptp_domain,
		.sync_time = 0,  /* epoch-aligned RTP timestamp */
	};
	return aes67_sdp_build(buf, buf_size, &params);
}

/* ================================================================
 * Build RTSP response
 * ================================================================ */

static int build_response_status(char *buf, size_t sz, rtsp_status_t status,
				 uint32_t cseq)
{
	return snprintf(buf, sz,
		"%s %d %s\r\n"
		"CSeq: %u\r\n",
		RTSP_VERSION, status, get_status_phrase(status), cseq);
}

/* ================================================================
 * Handle OPTIONS request
 * ================================================================ */

static int handle_options(int sock, const struct rtsp_request *req)
{
	int pos = build_response_status(rtsp_send_buf, sizeof(rtsp_send_buf),
					RTSP_STATUS_OK, req->cseq);

	pos += snprintf(rtsp_send_buf + pos, sizeof(rtsp_send_buf) - pos,
		"Public: OPTIONS, DESCRIBE, SETUP, PLAY, PAUSE, TEARDOWN\r\n"
		"\r\n");

	return zsock_send(sock, rtsp_send_buf, pos, 0);
}

/* ================================================================
 * Handle DESCRIBE request - returns SDP
 * ================================================================ */

static int handle_describe(int sock, const struct rtsp_request *req)
{
	uint8_t stream_id;
	int ret;

	ret = extract_stream_id_from_url(req->url, &stream_id);
	if (ret < 0 || stream_id >= AES67_MAX_TX_STREAMS) {
		int pos = build_response_status(rtsp_send_buf, sizeof(rtsp_send_buf),
						RTSP_STATUS_NOT_FOUND, req->cseq);
		pos += snprintf(rtsp_send_buf + pos, sizeof(rtsp_send_buf) - pos, "\r\n");
		return zsock_send(sock, rtsp_send_buf, pos, 0);
	}

	/* Build SDP body */
	char sdp_buf[512];
	int sdp_len = build_sdp_for_stream(sdp_buf, sizeof(sdp_buf), stream_id);
	if (sdp_len < 0) {
		int pos = build_response_status(rtsp_send_buf, sizeof(rtsp_send_buf),
						RTSP_STATUS_NOT_FOUND, req->cseq);
		pos += snprintf(rtsp_send_buf + pos, sizeof(rtsp_send_buf) - pos, "\r\n");
		return zsock_send(sock, rtsp_send_buf, pos, 0);
	}

	/* Build response with SDP */
	int pos = build_response_status(rtsp_send_buf, sizeof(rtsp_send_buf),
					RTSP_STATUS_OK, req->cseq);

	pos += snprintf(rtsp_send_buf + pos, sizeof(rtsp_send_buf) - pos,
		"Content-Type: application/sdp\r\n"
		"Content-Length: %d\r\n"
		"\r\n%s",
		sdp_len, sdp_buf);

	LOG_INF("RTSP: DESCRIBE stream %u, SDP len=%d", stream_id, sdp_len);
	return zsock_send(sock, rtsp_send_buf, pos, 0);
}

/* ================================================================
 * Handle SETUP request - creates session
 * ================================================================ */

static int handle_setup(int sock, const struct sockaddr_in *client_addr,
			const struct rtsp_request *req)
{
	uint8_t stream_id;
	int ret;

	ret = extract_stream_id_from_url(req->url, &stream_id);
	if (ret < 0 || stream_id >= AES67_MAX_TX_STREAMS) {
		int pos = build_response_status(rtsp_send_buf, sizeof(rtsp_send_buf),
						RTSP_STATUS_NOT_FOUND, req->cseq);
		pos += snprintf(rtsp_send_buf + pos, sizeof(rtsp_send_buf) - pos, "\r\n");
		return zsock_send(sock, rtsp_send_buf, pos, 0);
	}

	/* Verify stream is active */
	const struct aes67_tx_stream *streams = aes67_conn_get_tx_streams();
	if (!streams[stream_id].active) {
		int pos = build_response_status(rtsp_send_buf, sizeof(rtsp_send_buf),
						RTSP_STATUS_NOT_FOUND, req->cseq);
		pos += snprintf(rtsp_send_buf + pos, sizeof(rtsp_send_buf) - pos, "\r\n");
		return zsock_send(sock, rtsp_send_buf, pos, 0);
	}

	/* Check for existing session or create new */
	struct rtsp_session *sess = NULL;
	if (req->has_session) {
		sess = find_session(req->session_id);
	}
	if (!sess) {
		sess = create_session(sock, client_addr, stream_id);
		if (!sess) {
			int pos = build_response_status(rtsp_send_buf, sizeof(rtsp_send_buf),
							RTSP_STATUS_SERVICE_UNAVAILABLE, req->cseq);
			pos += snprintf(rtsp_send_buf + pos, sizeof(rtsp_send_buf) - pos, "\r\n");
			return zsock_send(sock, rtsp_send_buf, pos, 0);
		}
	}

	/* Store transport info */
	sess->transport_type = req->transport_type;
	sess->client_port_rtp = req->client_port_rtp;
	sess->client_port_rtcp = req->client_port_rtcp;

	/* Build response */
	int pos = build_response_status(rtsp_send_buf, sizeof(rtsp_send_buf),
					RTSP_STATUS_OK, req->cseq);

	/* Get stream multicast address */
	char mcast_str[INET_ADDRSTRLEN];
	zsock_inet_ntop(AF_INET, &streams[stream_id].dst_ip, mcast_str, sizeof(mcast_str));

	if (req->transport_type == RTSP_TRANSPORT_RTP_AVP_UDP_MULTICAST) {
		pos += snprintf(rtsp_send_buf + pos, sizeof(rtsp_send_buf) - pos,
			"Transport: RTP/AVP;multicast;destination=%s;port=%u-%u;ttl=32\r\n",
			mcast_str, AES67_DEFAULT_PORT, AES67_DEFAULT_PORT + 1);
	} else {
		/* Unicast - client specifies ports */
		pos += snprintf(rtsp_send_buf + pos, sizeof(rtsp_send_buf) - pos,
			"Transport: RTP/AVP;unicast;client_port=%u-%u;server_port=%u-%u\r\n",
			req->client_port_rtp, req->client_port_rtcp,
			AES67_DEFAULT_PORT, AES67_DEFAULT_PORT + 1);
	}

	pos += snprintf(rtsp_send_buf + pos, sizeof(rtsp_send_buf) - pos,
		"Session: %s;timeout=%d\r\n"
		"\r\n",
		sess->session_id, RTSP_SESSION_TIMEOUT_S);

	LOG_INF("RTSP: SETUP stream %u, session=%s", stream_id, sess->session_id);
	return zsock_send(sock, rtsp_send_buf, pos, 0);
}

/* ================================================================
 * Handle PLAY request
 * ================================================================ */

static int handle_play(int sock, const struct rtsp_request *req)
{
	if (!req->has_session) {
		int pos = build_response_status(rtsp_send_buf, sizeof(rtsp_send_buf),
						RTSP_STATUS_SESSION_NOT_FOUND, req->cseq);
		pos += snprintf(rtsp_send_buf + pos, sizeof(rtsp_send_buf) - pos, "\r\n");
		return zsock_send(sock, rtsp_send_buf, pos, 0);
	}

	struct rtsp_session *sess = find_session(req->session_id);
	if (!sess) {
		int pos = build_response_status(rtsp_send_buf, sizeof(rtsp_send_buf),
						RTSP_STATUS_SESSION_NOT_FOUND, req->cseq);
		pos += snprintf(rtsp_send_buf + pos, sizeof(rtsp_send_buf) - pos, "\r\n");
		return zsock_send(sock, rtsp_send_buf, pos, 0);
	}

	sess->is_playing = true;
	sess->last_activity_ms = k_uptime_get();

	int pos = build_response_status(rtsp_send_buf, sizeof(rtsp_send_buf),
					RTSP_STATUS_OK, req->cseq);

	pos += snprintf(rtsp_send_buf + pos, sizeof(rtsp_send_buf) - pos,
		"Session: %s\r\n"
		"RTP-Info: url=%s;seq=0;rtptime=0\r\n"
		"\r\n",
		sess->session_id, req->url);

	LOG_INF("RTSP: PLAY session=%s", sess->session_id);
	return zsock_send(sock, rtsp_send_buf, pos, 0);
}

/* ================================================================
 * Handle PAUSE request
 * ================================================================ */

static int handle_pause(int sock, const struct rtsp_request *req)
{
	if (!req->has_session) {
		int pos = build_response_status(rtsp_send_buf, sizeof(rtsp_send_buf),
						RTSP_STATUS_SESSION_NOT_FOUND, req->cseq);
		pos += snprintf(rtsp_send_buf + pos, sizeof(rtsp_send_buf) - pos, "\r\n");
		return zsock_send(sock, rtsp_send_buf, pos, 0);
	}

	struct rtsp_session *sess = find_session(req->session_id);
	if (!sess) {
		int pos = build_response_status(rtsp_send_buf, sizeof(rtsp_send_buf),
						RTSP_STATUS_SESSION_NOT_FOUND, req->cseq);
		pos += snprintf(rtsp_send_buf + pos, sizeof(rtsp_send_buf) - pos, "\r\n");
		return zsock_send(sock, rtsp_send_buf, pos, 0);
	}

	sess->is_playing = false;
	sess->last_activity_ms = k_uptime_get();

	int pos = build_response_status(rtsp_send_buf, sizeof(rtsp_send_buf),
					RTSP_STATUS_OK, req->cseq);

	pos += snprintf(rtsp_send_buf + pos, sizeof(rtsp_send_buf) - pos,
		"Session: %s\r\n"
		"\r\n",
		sess->session_id);

	LOG_INF("RTSP: PAUSE session=%s", sess->session_id);
	return zsock_send(sock, rtsp_send_buf, pos, 0);
}

/* ================================================================
 * Handle TEARDOWN request
 * ================================================================ */

static int handle_teardown(int sock, const struct rtsp_request *req)
{
	if (!req->has_session) {
		int pos = build_response_status(rtsp_send_buf, sizeof(rtsp_send_buf),
						RTSP_STATUS_SESSION_NOT_FOUND, req->cseq);
		pos += snprintf(rtsp_send_buf + pos, sizeof(rtsp_send_buf) - pos, "\r\n");
		return zsock_send(sock, rtsp_send_buf, pos, 0);
	}

	struct rtsp_session *sess = find_session(req->session_id);
	if (!sess) {
		int pos = build_response_status(rtsp_send_buf, sizeof(rtsp_send_buf),
						RTSP_STATUS_SESSION_NOT_FOUND, req->cseq);
		pos += snprintf(rtsp_send_buf + pos, sizeof(rtsp_send_buf) - pos, "\r\n");
		return zsock_send(sock, rtsp_send_buf, pos, 0);
	}

	int pos = build_response_status(rtsp_send_buf, sizeof(rtsp_send_buf),
					RTSP_STATUS_OK, req->cseq);

	pos += snprintf(rtsp_send_buf + pos, sizeof(rtsp_send_buf) - pos,
		"Session: %s\r\n"
		"\r\n",
		sess->session_id);

	int ret = zsock_send(sock, rtsp_send_buf, pos, 0);

	LOG_INF("RTSP: TEARDOWN session=%s", sess->session_id);
	destroy_session(sess);

	return ret;
}

/* ================================================================
 * Handle a single RTSP request
 * ================================================================ */

static int handle_request(int sock, const struct sockaddr_in *client_addr,
			  const char *buf, size_t len)
{
	struct rtsp_request req;
	int ret;

	ret = parse_rtsp_request(buf, len, &req);
	if (ret < 0) {
		int pos = build_response_status(rtsp_send_buf, sizeof(rtsp_send_buf),
						RTSP_STATUS_BAD_REQUEST, 0);
		pos += snprintf(rtsp_send_buf + pos, sizeof(rtsp_send_buf) - pos, "\r\n");
		return zsock_send(sock, rtsp_send_buf, pos, 0);
	}

	LOG_DBG("RTSP: %s %s CSeq=%u",
		method_names[req.method] ? method_names[req.method] : "?",
		req.url, req.cseq);

	switch (req.method) {
	case RTSP_METHOD_OPTIONS:
		return handle_options(sock, &req);
	case RTSP_METHOD_DESCRIBE:
		return handle_describe(sock, &req);
	case RTSP_METHOD_SETUP:
		return handle_setup(sock, client_addr, &req);
	case RTSP_METHOD_PLAY:
		return handle_play(sock, &req);
	case RTSP_METHOD_PAUSE:
		return handle_pause(sock, &req);
	case RTSP_METHOD_TEARDOWN:
		return handle_teardown(sock, &req);
	default:
		/* Not implemented */
		{
			int pos = build_response_status(rtsp_send_buf, sizeof(rtsp_send_buf),
							RTSP_STATUS_NOT_IMPLEMENTED, req.cseq);
			pos += snprintf(rtsp_send_buf + pos, sizeof(rtsp_send_buf) - pos, "\r\n");
			return zsock_send(sock, rtsp_send_buf, pos, 0);
		}
	}
}

/* ================================================================
 * Handle a client connection
 * ================================================================ */

static void handle_client(int client_sock, const struct sockaddr_in *client_addr)
{
	char client_ip[INET_ADDRSTRLEN];
	zsock_inet_ntop(AF_INET, &client_addr->sin_addr, client_ip, sizeof(client_ip));
	LOG_INF("RTSP: Client connected from %s:%u",
		client_ip, ntohs(client_addr->sin_port));

	/* Set socket timeout */
	struct timeval tv = {
		.tv_sec = RTSP_SESSION_TIMEOUT_S,
		.tv_usec = 0
	};
	zsock_setsockopt(client_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

	/* Requests can arrive fragmented across several TCP segments (e.g.
	 * the Rust rtsp_types client writes the request line and headers
	 * piecewise) or pipelined back-to-back.  Collect until a complete
	 * request (headers + optional Content-Length body) is buffered,
	 * handle it, then shift any remainder down. */
	size_t used = 0;

	while (rtsp_server_running) {
		if (used >= sizeof(rtsp_recv_buf) - 1) {
			LOG_WRN("RTSP: request from %s too large, closing",
				client_ip);
			break;
		}

		int len = zsock_recv(client_sock, rtsp_recv_buf + used,
				     sizeof(rtsp_recv_buf) - 1 - used, 0);
		if (len <= 0) {
			if (len == 0) {
				LOG_INF("RTSP: Client %s disconnected", client_ip);
			} else if (errno != EAGAIN && errno != EWOULDBLOCK) {
				LOG_ERR("RTSP: recv error: %d", errno);
			}
			break;
		}

		used += len;
		rtsp_recv_buf[used] = '\0';

		bool conn_error = false;

		while (!conn_error) {
			char *hdr_end = strstr(rtsp_recv_buf, "\r\n\r\n");

			if (!hdr_end) {
				break;  /* headers incomplete, keep reading */
			}

			size_t req_len = (hdr_end + 4) - rtsp_recv_buf;

			/* Body present? (ANNOUNCE carries SDP) */
			const char *cl = strstr(rtsp_recv_buf, "Content-Length:");

			if (cl && cl < hdr_end) {
				req_len += strtoul(cl + 15, NULL, 10);
				if (req_len > sizeof(rtsp_recv_buf) - 1) {
					LOG_WRN("RTSP: oversized body from %s",
						client_ip);
					conn_error = true;
					break;
				}
				if (used < req_len) {
					break;  /* body incomplete */
				}
			}

			/* NUL-terminate this request for the string-based
			 * parser, keeping any pipelined follow-up intact. */
			char saved = rtsp_recv_buf[req_len];

			rtsp_recv_buf[req_len] = '\0';
			int ret = handle_request(client_sock, client_addr,
						 rtsp_recv_buf, req_len);
			rtsp_recv_buf[req_len] = saved;

			if (ret < 0) {
				LOG_ERR("RTSP: send error: %d", ret);
				conn_error = true;
				break;
			}

			memmove(rtsp_recv_buf, rtsp_recv_buf + req_len,
				used - req_len);
			used -= req_len;
			rtsp_recv_buf[used] = '\0';
		}

		if (conn_error) {
			break;
		}
	}

	zsock_close(client_sock);
}

/* ================================================================
 * Session expiry check
 * ================================================================ */

static void expire_sessions(void)
{
	int64_t now = k_uptime_get();
	int64_t timeout_ms = RTSP_SESSION_TIMEOUT_S * 1000;

	k_mutex_lock(&rtsp_mutex, K_FOREVER);
	for (int i = 0; i < RTSP_MAX_SESSIONS; i++) {
		if (rtsp_sessions[i].active &&
		    (now - rtsp_sessions[i].last_activity_ms) > timeout_ms) {
			LOG_INF("RTSP: Session %s expired",
				rtsp_sessions[i].session_id);
			rtsp_sessions[i].active = false;
		}
	}
	k_mutex_unlock(&rtsp_mutex);
}

/* ================================================================
 * RTSP server thread
 * ================================================================ */

static void rtsp_server_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	struct sockaddr_in server_addr = {
		.sin_family = AF_INET,
		.sin_port = htons(RTSP_DEFAULT_PORT),
		.sin_addr.s_addr = htonl(INADDR_ANY),
	};

	/* Create TCP socket */
	rtsp_server_sock = zsock_socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (rtsp_server_sock < 0) {
		LOG_ERR("RTSP: Failed to create socket: %d", errno);
		return;
	}

	/* Allow address reuse */
	int reuse = 1;
	zsock_setsockopt(rtsp_server_sock, SOL_SOCKET, SO_REUSEADDR,
			 &reuse, sizeof(reuse));

	/* Bind */
	int ret = zsock_bind(rtsp_server_sock, (struct sockaddr *)&server_addr,
			     sizeof(server_addr));
	if (ret < 0) {
		LOG_ERR("RTSP: Failed to bind: %d", errno);
		zsock_close(rtsp_server_sock);
		rtsp_server_sock = -1;
		return;
	}

	/* Listen */
	ret = zsock_listen(rtsp_server_sock, RTSP_MAX_CLIENTS);
	if (ret < 0) {
		LOG_ERR("RTSP: Failed to listen: %d", errno);
		zsock_close(rtsp_server_sock);
		rtsp_server_sock = -1;
		return;
	}

	LOG_INF("RTSP: Server listening on port %u", RTSP_DEFAULT_PORT);
	rtsp_server_running = true;

	/* Set non-blocking with timeout for accept */
	struct timeval tv = { .tv_sec = 5, .tv_usec = 0 };
	zsock_setsockopt(rtsp_server_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

	while (rtsp_server_running) {
		struct sockaddr_in client_addr;
		socklen_t client_len = sizeof(client_addr);

		int client_sock = zsock_accept(rtsp_server_sock,
					       (struct sockaddr *)&client_addr,
					       &client_len);
		if (client_sock < 0) {
			if (errno == EAGAIN || errno == EWOULDBLOCK) {
				/* Timeout - check for session expiry */
				expire_sessions();
				continue;
			}
			LOG_ERR("RTSP: accept error: %d", errno);
			continue;
		}

		/* Handle client in same thread (simple single-client model)
		 * For multi-client, we'd spawn threads or use poll() */
		handle_client(client_sock, &client_addr);
	}

	zsock_close(rtsp_server_sock);
	rtsp_server_sock = -1;
	LOG_INF("RTSP: Server stopped");
}

/* ================================================================
 * Public API: Server
 * ================================================================ */

int rtsp_server_start(void)
{
	if (rtsp_server_running) {
		return -EALREADY;
	}

	k_mutex_init(&rtsp_mutex);
	memset(rtsp_sessions, 0, sizeof(rtsp_sessions));
	memset(rtsp_clients, 0, sizeof(rtsp_clients));

	/* Get local IP from network interface */
	struct net_if *iface = net_if_get_default();
	if (iface) {
		/* Just get any IPv4 address from the interface */
		struct net_if_ipv4 *ipv4 = iface->config.ip.ipv4;
		if (ipv4 && ipv4->unicast[0].ipv4.is_used) {
			local_ip_addr = ipv4->unicast[0].ipv4.address.in_addr;
		}
	}

	k_thread_create(&rtsp_server_thread_data,
			rtsp_server_stack,
			K_THREAD_STACK_SIZEOF(rtsp_server_stack),
			rtsp_server_thread,
			NULL, NULL, NULL,
			RTSP_SERVER_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&rtsp_server_thread_data, "rtsp_server");

	return 0;
}

void rtsp_server_stop(void)
{
	rtsp_server_running = false;
	/* Thread will exit on next accept timeout */
}

bool rtsp_server_is_running(void)
{
	return rtsp_server_running;
}

const struct rtsp_session *rtsp_get_sessions(int *count)
{
	int n = 0;
	for (int i = 0; i < RTSP_MAX_SESSIONS; i++) {
		if (rtsp_sessions[i].active) {
			n++;
		}
	}
	if (count) {
		*count = n;
	}
	return rtsp_sessions;
}

/* ================================================================
 * RTSP ANNOUNCE - push SDP update to connected clients
 * ================================================================ */

int rtsp_announce_stream_update(uint8_t stream_id)
{
	if (stream_id >= AES67_MAX_TX_STREAMS) {
		return -EINVAL;
	}

	/* Build SDP for this stream */
	char sdp_buf[512];
	int sdp_len = build_sdp_for_stream(sdp_buf, sizeof(sdp_buf), stream_id);
	if (sdp_len < 0) {
		return sdp_len;
	}

	k_mutex_lock(&rtsp_mutex, K_FOREVER);

	int sent_count = 0;
	for (int i = 0; i < RTSP_MAX_SESSIONS; i++) {
		if (!rtsp_sessions[i].active ||
		    rtsp_sessions[i].stream_id != stream_id ||
		    rtsp_sessions[i].client_sock < 0) {
			continue;
		}

		struct rtsp_session *sess = &rtsp_sessions[i];

		/* Format RTSP URL for stream */
		char url[RTSP_MAX_URL_LENGTH];
		char ip_str[INET_ADDRSTRLEN];
		zsock_inet_ntop(AF_INET, &local_ip_addr, ip_str, sizeof(ip_str));
		snprintf(url, sizeof(url), "rtsp://%s:%u/by-id/%u",
			 ip_str, RTSP_DEFAULT_PORT, stream_id);

		/* Build ANNOUNCE request */
		int pos = snprintf(rtsp_send_buf, sizeof(rtsp_send_buf),
			"ANNOUNCE %s %s\r\n"
			"CSeq: 0\r\n"
			"Session: %s\r\n"
			"Content-Type: application/sdp\r\n"
			"Content-Length: %d\r\n"
			"\r\n%s",
			url, RTSP_VERSION,
			sess->session_id,
			sdp_len, sdp_buf);

		int ret = zsock_send(sess->client_sock, rtsp_send_buf, pos, 0);
		if (ret > 0) {
			sent_count++;
			LOG_INF("RTSP: ANNOUNCE sent to session %s", sess->session_id);
		}
	}

	k_mutex_unlock(&rtsp_mutex);

	return sent_count;
}

/* ================================================================
 * Public API: Client
 * ================================================================ */

/**
 * @brief Find a free client connection slot
 */
static struct rtsp_client_connection *find_free_client_slot(void)
{
	for (int i = 0; i < RTSP_MAX_CLIENTS; i++) {
		if (!rtsp_clients[i].active) {
			return &rtsp_clients[i];
		}
	}
	return NULL;
}

/**
 * @brief Find client connection by RX stream ID
 */
static struct rtsp_client_connection *find_client_by_rx_stream(uint8_t rx_stream_id)
{
	for (int i = 0; i < RTSP_MAX_CLIENTS; i++) {
		if (rtsp_clients[i].active &&
		    rtsp_clients[i].rx_stream_id == rx_stream_id) {
			return &rtsp_clients[i];
		}
	}
	return NULL;
}

/**
 * @brief Send RTSP request and receive response
 */
static int rtsp_client_transact(struct rtsp_client_connection *conn,
				const char *request, size_t req_len,
				char *response, size_t resp_size)
{
	int ret = zsock_send(conn->sock, request, req_len, 0);
	if (ret < 0) {
		LOG_ERR("RTSP client: send failed: %d", errno);
		return -errno;
	}

	/* Receive response */
	ret = zsock_recv(conn->sock, response, resp_size - 1, 0);
	if (ret <= 0) {
		LOG_ERR("RTSP client: recv failed: %d", errno);
		return ret == 0 ? -ECONNRESET : -errno;
	}

	response[ret] = '\0';
	return ret;
}

/**
 * @brief Parse SDP from DESCRIBE response and extract stream info
 *
 * Uses the shared aes67_sdp_parse helper.
 */
static int parse_describe_response(const char *response, size_t len,
				   struct in_addr *mcast_addr,
				   uint16_t *port,
				   uint8_t *channels,
				   uint32_t *ssrc)
{
	/* Find Content-Length and body */
	const char *body = strstr(response, "\r\n\r\n");
	if (!body) {
		return -EINVAL;
	}
	body += 4;

	/* Calculate body length */
	size_t body_len = len - (body - response);

	/* Parse SDP using shared helper */
	struct aes67_sdp_parsed parsed;
	int ret = aes67_sdp_parse(body, body_len, &parsed);
	if (ret < 0) {
		return ret;
	}

	/* Copy relevant fields to output parameters */
	*mcast_addr = parsed.connection_addr;
	*port = parsed.port;
	*channels = parsed.channels;
	*ssrc = parsed.ssrc;

	return 0;
}

/* ---- One-shot DESCRIBE (mDNS discovery) ---- */

static K_MUTEX_DEFINE(describe_lock);
static char describe_buf[2048];

/* Percent-encode a URL path segment (session names may contain spaces). */
static size_t url_encode_segment(const char *in, char *out, size_t out_size)
{
	static const char hex[] = "0123456789ABCDEF";
	size_t n = 0;

	for (const uint8_t *p = (const uint8_t *)in; *p; p++) {
		bool plain = (*p >= 'A' && *p <= 'Z') ||
			     (*p >= 'a' && *p <= 'z') ||
			     (*p >= '0' && *p <= '9') ||
			     *p == '-' || *p == '_' || *p == '.' || *p == '~';

		if (plain) {
			if (n + 1 >= out_size) {
				break;
			}
			out[n++] = *p;
		} else {
			if (n + 3 >= out_size) {
				break;
			}
			out[n++] = '%';
			out[n++] = hex[*p >> 4];
			out[n++] = hex[*p & 0xF];
		}
	}
	out[n] = '\0';
	return n;
}

int rtsp_client_describe(const struct in_addr *server_addr,
			 uint16_t server_port,
			 const char *session_name,
			 struct aes67_sdp_parsed *out)
{
	char enc_name[2 * AES67_STREAM_NAME_MAX];
	char ip_str[INET_ADDRSTRLEN];
	int sock;
	int ret;

	if (!server_addr || !session_name || !out) {
		return -EINVAL;
	}

	sock = zsock_socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (sock < 0) {
		return -errno;
	}

	struct timeval tv = { .tv_sec = 5, .tv_usec = 0 };

	zsock_setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
	zsock_setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

	struct sockaddr_in srv = {
		.sin_family = AF_INET,
		.sin_port = htons(server_port),
		.sin_addr = *server_addr,
	};

	ret = zsock_connect(sock, (struct sockaddr *)&srv, sizeof(srv));
	if (ret < 0) {
		ret = -errno;
		goto out_close;
	}

	zsock_inet_ntop(AF_INET, server_addr, ip_str, sizeof(ip_str));
	url_encode_segment(session_name, enc_name, sizeof(enc_name));

	k_mutex_lock(&describe_lock, K_FOREVER);

	int req_len = snprintf(describe_buf, sizeof(describe_buf),
		"DESCRIBE rtsp://%s:%u/by-name/%s %s\r\n"
		"CSeq: 1\r\n"
		"Accept: application/sdp\r\n"
		"\r\n",
		ip_str, server_port, enc_name, RTSP_VERSION);

	ret = zsock_send(sock, describe_buf, req_len, 0);
	if (ret < 0) {
		ret = -errno;
		goto out_unlock;
	}

	/* Collect the response until headers + body are complete (the SDP
	 * may arrive in a separate TCP segment from the headers). */
	size_t total = 0;
	const char *body = NULL;

	while (total < sizeof(describe_buf) - 1) {
		ret = zsock_recv(sock, describe_buf + total,
				 sizeof(describe_buf) - 1 - total, 0);
		if (ret <= 0) {
			break;
		}
		total += ret;
		describe_buf[total] = '\0';

		body = strstr(describe_buf, "\r\n\r\n");
		if (!body) {
			continue;
		}
		body += 4;

		const char *cl = strstr(describe_buf, "Content-Length:");

		if (!cl || cl > body) {
			break;  /* no length header: take what we have */
		}
		size_t content_len = strtoul(cl + 15, NULL, 10);

		if (total - (body - describe_buf) >= content_len) {
			break;  /* body complete */
		}
	}

	if (total == 0 || !body) {
		ret = -EIO;
		goto out_unlock;
	}

	if (strstr(describe_buf, "200 ") == NULL) {
		LOG_DBG("RTSP describe: non-200 for \"%s\"", session_name);
		ret = -ENOENT;
		goto out_unlock;
	}

	ret = aes67_sdp_parse(body, total - (body - describe_buf), out);

out_unlock:
	k_mutex_unlock(&describe_lock);
out_close:
	zsock_close(sock);
	return ret;
}

int rtsp_client_subscribe(const struct in_addr *server_addr,
			  uint16_t server_port,
			  const char *stream_name,
			  uint8_t rx_stream_id)
{
	struct rtsp_client_connection *conn;
	int ret;

	if (rx_stream_id >= AES67_MAX_RX_STREAMS) {
		return -EINVAL;
	}

	/* Check if already subscribed */
	conn = find_client_by_rx_stream(rx_stream_id);
	if (conn) {
		LOG_WRN("RTSP client: RX stream %u already subscribed", rx_stream_id);
		return -EALREADY;
	}

	/* Get free slot */
	conn = find_free_client_slot();
	if (!conn) {
		LOG_ERR("RTSP client: No free connection slots");
		return -ENOMEM;
	}

	/* Create TCP connection */
	conn->sock = zsock_socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (conn->sock < 0) {
		LOG_ERR("RTSP client: socket failed: %d", errno);
		return -errno;
	}

	struct sockaddr_in srv_addr = {
		.sin_family = AF_INET,
		.sin_port = htons(server_port),
		.sin_addr = *server_addr,
	};

	/* Set timeout */
	struct timeval tv = { .tv_sec = 10, .tv_usec = 0 };
	zsock_setsockopt(conn->sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
	zsock_setsockopt(conn->sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

	ret = zsock_connect(conn->sock, (struct sockaddr *)&srv_addr, sizeof(srv_addr));
	if (ret < 0) {
		LOG_ERR("RTSP client: connect failed: %d", errno);
		zsock_close(conn->sock);
		return -errno;
	}

	conn->server_addr = *server_addr;
	conn->server_port = server_port;
	conn->rx_stream_id = rx_stream_id;
	conn->cseq = 1;

	char ip_str[INET_ADDRSTRLEN];
	zsock_inet_ntop(AF_INET, server_addr, ip_str, sizeof(ip_str));
	LOG_INF("RTSP client: Connected to %s:%u", ip_str, server_port);

	/* Build stream URL */
	char url[RTSP_MAX_URL_LENGTH];
	snprintf(url, sizeof(url), "rtsp://%s:%u/%s",
		 ip_str, server_port, stream_name);

	/* ---- Step 1: DESCRIBE ---- */
	int req_len = snprintf(rtsp_send_buf, sizeof(rtsp_send_buf),
		"DESCRIBE %s %s\r\n"
		"CSeq: %u\r\n"
		"Accept: application/sdp\r\n"
		"\r\n",
		url, RTSP_VERSION, conn->cseq++);

	ret = rtsp_client_transact(conn, rtsp_send_buf, req_len,
				   rtsp_recv_buf, sizeof(rtsp_recv_buf));
	if (ret < 0) {
		goto error;
	}

	/* Check for 200 OK */
	if (strstr(rtsp_recv_buf, "200 ") == NULL) {
		LOG_ERR("RTSP client: DESCRIBE failed: %s", rtsp_recv_buf);
		ret = -ENOENT;
		goto error;
	}

	/* Parse SDP */
	struct in_addr mcast_addr = { 0 };
	uint16_t port = AES67_DEFAULT_PORT;
	uint8_t channels = 2;
	uint32_t ssrc = 0;

	parse_describe_response(rtsp_recv_buf, ret, &mcast_addr, &port, &channels, &ssrc);

	/* ---- Step 2: SETUP ---- */
	req_len = snprintf(rtsp_send_buf, sizeof(rtsp_send_buf),
		"SETUP %s %s\r\n"
		"CSeq: %u\r\n"
		"Transport: RTP/AVP;multicast\r\n"
		"\r\n",
		url, RTSP_VERSION, conn->cseq++);

	ret = rtsp_client_transact(conn, rtsp_send_buf, req_len,
				   rtsp_recv_buf, sizeof(rtsp_recv_buf));
	if (ret < 0) {
		goto error;
	}

	/* Extract Session ID from response */
	const char *sess_hdr = strstr(rtsp_recv_buf, "Session:");
	if (sess_hdr) {
		sess_hdr += 8;
		while (*sess_hdr == ' ') sess_hdr++;
		size_t id_len = 0;
		while (sess_hdr[id_len] && sess_hdr[id_len] != ';' &&
		       sess_hdr[id_len] != '\r' &&
		       id_len < sizeof(conn->session_id) - 1) {
			conn->session_id[id_len] = sess_hdr[id_len];
			id_len++;
		}
		conn->session_id[id_len] = '\0';
	}

	/* ---- Step 3: PLAY ---- */
	req_len = snprintf(rtsp_send_buf, sizeof(rtsp_send_buf),
		"PLAY %s %s\r\n"
		"CSeq: %u\r\n"
		"Session: %s\r\n"
		"\r\n",
		url, RTSP_VERSION, conn->cseq++, conn->session_id);

	ret = rtsp_client_transact(conn, rtsp_send_buf, req_len,
				   rtsp_recv_buf, sizeof(rtsp_recv_buf));
	if (ret < 0) {
		goto error;
	}

	/* Configure local RX stream with received SDP info */
	uint8_t ch_map[AES67_MAX_CH_PER_STREAM] = {0, 1, 2, 3, 4, 5, 6, 7};
	ret = aes67_conn_configure_rx_stream(rx_stream_id,
					  &mcast_addr,
					  port,
					  ch_map,
					  channels,
					  16,  /* output delay */
					  AES67_DEFAULT_SAMPLES_PER_PKT);
	if (ret < 0) {
		LOG_ERR("RTSP client: Failed to configure RX stream: %d", ret);
		goto error;
	}

	conn->active = true;
	LOG_INF("RTSP client: Subscribed to stream, RX %u, session=%s",
		rx_stream_id, conn->session_id);

	return 0;

error:
	zsock_close(conn->sock);
	conn->sock = -1;
	memset(conn, 0, sizeof(*conn));
	return ret;
}

int rtsp_client_unsubscribe(uint8_t rx_stream_id)
{
	struct rtsp_client_connection *conn = find_client_by_rx_stream(rx_stream_id);
	if (!conn) {
		return -ENOENT;
	}

	/* Send TEARDOWN */
	char ip_str[INET_ADDRSTRLEN];
	zsock_inet_ntop(AF_INET, &conn->server_addr, ip_str, sizeof(ip_str));

	char url[RTSP_MAX_URL_LENGTH];
	snprintf(url, sizeof(url), "rtsp://%s:%u/by-id/%u",
		 ip_str, conn->server_port, rx_stream_id);

	int req_len = snprintf(rtsp_send_buf, sizeof(rtsp_send_buf),
		"TEARDOWN %s %s\r\n"
		"CSeq: %u\r\n"
		"Session: %s\r\n"
		"\r\n",
		url, RTSP_VERSION, conn->cseq++, conn->session_id);

	zsock_send(conn->sock, rtsp_send_buf, req_len, 0);

	/* Close connection */
	zsock_close(conn->sock);
	conn->sock = -1;
	conn->active = false;
	memset(conn, 0, sizeof(*conn));

	LOG_INF("RTSP client: Unsubscribed RX stream %u", rx_stream_id);
	return 0;
}

const struct rtsp_client_connection *rtsp_get_client_connections(int *count)
{
	int n = 0;
	for (int i = 0; i < RTSP_MAX_CLIENTS; i++) {
		if (rtsp_clients[i].active) {
			n++;
		}
	}
	if (count) {
		*count = n;
	}
	return rtsp_clients;
}

/* ================================================================
 * URL helper
 * ================================================================ */

int rtsp_format_stream_url(char *buf, size_t buf_size, uint8_t stream_id)
{
	char ip_str[INET_ADDRSTRLEN];
	zsock_inet_ntop(AF_INET, &local_ip_addr, ip_str, sizeof(ip_str));

	return snprintf(buf, buf_size, "rtsp://%s:%u/by-id/%u",
			ip_str, RTSP_DEFAULT_PORT, stream_id);
}

/* ================================================================
 * Notify RTSP of IP address (called from main after DHCP)
 * ================================================================ */

void rtsp_notify_ip_ready(const struct in_addr *addr)
{
	local_ip_addr = *addr;
	LOG_INF("RTSP: IP address set to %d.%d.%d.%d",
		addr->s4_addr[0], addr->s4_addr[1],
		addr->s4_addr[2], addr->s4_addr[3]);
}

/* ================================================================
 * Shell commands
 * ================================================================ */

#ifdef CONFIG_SHELL

#include <zephyr/shell/shell.h>

static int cmd_rtsp_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "RTSP Server: %s",
		    rtsp_server_running ? "Running" : "Stopped");

	if (rtsp_server_running) {
		char ip_str[INET_ADDRSTRLEN];
		zsock_inet_ntop(AF_INET, &local_ip_addr, ip_str, sizeof(ip_str));
		shell_print(sh, "  Address: %s:%u", ip_str, RTSP_DEFAULT_PORT);
	}

	/* List sessions */
	int sess_count = 0;
	for (int i = 0; i < RTSP_MAX_SESSIONS; i++) {
		if (rtsp_sessions[i].active) {
			sess_count++;
			char client_ip[INET_ADDRSTRLEN];
			zsock_inet_ntop(AF_INET, &rtsp_sessions[i].client_addr,
					client_ip, sizeof(client_ip));
			shell_print(sh, "  Session %s: stream=%u client=%s %s",
				    rtsp_sessions[i].session_id,
				    rtsp_sessions[i].stream_id,
				    client_ip,
				    rtsp_sessions[i].is_playing ? "PLAYING" : "PAUSED");
		}
	}
	shell_print(sh, "  Active sessions: %d", sess_count);

	/* List client connections */
	int client_count = 0;
	for (int i = 0; i < RTSP_MAX_CLIENTS; i++) {
		if (rtsp_clients[i].active) {
			client_count++;
			char srv_ip[INET_ADDRSTRLEN];
			zsock_inet_ntop(AF_INET, &rtsp_clients[i].server_addr,
					srv_ip, sizeof(srv_ip));
			shell_print(sh, "  Client %d: server=%s:%u rx_stream=%u session=%s",
				    i, srv_ip, rtsp_clients[i].server_port,
				    rtsp_clients[i].rx_stream_id,
				    rtsp_clients[i].session_id);
		}
	}
	shell_print(sh, "  Active client connections: %d", client_count);

	return 0;
}

static int cmd_rtsp_start(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	int ret = rtsp_server_start();
	if (ret == -EALREADY) {
		shell_print(sh, "RTSP server already running");
	} else if (ret < 0) {
		shell_error(sh, "Failed to start RTSP server: %d", ret);
	} else {
		shell_print(sh, "RTSP server started on port %u", RTSP_DEFAULT_PORT);
	}
	return ret < 0 ? ret : 0;
}

static int cmd_rtsp_stop(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	rtsp_server_stop();
	shell_print(sh, "RTSP server stop requested");
	return 0;
}

static int cmd_rtsp_subscribe(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 4) {
		shell_error(sh, "Usage: rtsp subscribe <server_ip> <stream_name> <rx_stream_id>");
		return -EINVAL;
	}

	struct in_addr server_addr;
	if (zsock_inet_pton(AF_INET, argv[1], &server_addr) != 1) {
		shell_error(sh, "Invalid IP address: %s", argv[1]);
		return -EINVAL;
	}

	uint8_t rx_stream_id = (uint8_t)strtoul(argv[3], NULL, 10);

	int ret = rtsp_client_subscribe(&server_addr, RTSP_DEFAULT_PORT,
					argv[2], rx_stream_id);
	if (ret < 0) {
		shell_error(sh, "Subscribe failed: %d", ret);
	} else {
		shell_print(sh, "Subscribed to %s stream '%s' -> RX %u",
			    argv[1], argv[2], rx_stream_id);
	}
	return ret;
}

static int cmd_rtsp_unsubscribe(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 2) {
		shell_error(sh, "Usage: rtsp unsubscribe <rx_stream_id>");
		return -EINVAL;
	}

	uint8_t rx_stream_id = (uint8_t)strtoul(argv[1], NULL, 10);

	int ret = rtsp_client_unsubscribe(rx_stream_id);
	if (ret < 0) {
		shell_error(sh, "Unsubscribe failed: %d", ret);
	} else {
		shell_print(sh, "Unsubscribed RX stream %u", rx_stream_id);
	}
	return ret;
}

static int cmd_rtsp_url(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 2) {
		shell_error(sh, "Usage: rtsp url <stream_id>");
		return -EINVAL;
	}

	uint8_t stream_id = (uint8_t)strtoul(argv[1], NULL, 10);

	char url[RTSP_MAX_URL_LENGTH];
	int ret = rtsp_format_stream_url(url, sizeof(url), stream_id);
	if (ret > 0) {
		shell_print(sh, "%s", url);
	}
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(rtsp_cmds,
	SHELL_CMD(status, NULL, "Show RTSP server status", cmd_rtsp_status),
	SHELL_CMD(start, NULL, "Start RTSP server", cmd_rtsp_start),
	SHELL_CMD(stop, NULL, "Stop RTSP server", cmd_rtsp_stop),
	SHELL_CMD(subscribe, NULL, "Subscribe to remote stream: <ip> <name> <rx_id>",
		  cmd_rtsp_subscribe),
	SHELL_CMD(unsubscribe, NULL, "Unsubscribe from stream: <rx_id>",
		  cmd_rtsp_unsubscribe),
	SHELL_CMD(url, NULL, "Show RTSP URL for TX stream: <stream_id>", cmd_rtsp_url),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(rtsp, &rtsp_cmds, "RTSP server/client commands", NULL);

#endif /* CONFIG_SHELL */
