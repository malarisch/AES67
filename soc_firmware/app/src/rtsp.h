/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: Apache-2.0
 *
 * RAVENNA-compliant RTSP server and client for AES67 stream control.
 *
 * Server: Handles DESCRIBE, SETUP, PLAY, TEARDOWN requests for TX streams.
 * Client: Connects to remote RTSP servers to subscribe to RX streams.
 *
 * Based on:
 *   - RFC 2326 (Real Time Streaming Protocol)
 *   - RAVENNA Operating Principles (ALC NetworX)
 */

#ifndef RTSP_H_
#define RTSP_H_

#include <zephyr/kernel.h>
#include <zephyr/net/net_ip.h>
#include <stdint.h>
#include <stdbool.h>

struct aes67_sdp_parsed;

#ifdef __cplusplus
extern "C" {
#endif

/* ---- RTSP constants ---- */

#define RTSP_DEFAULT_PORT       554
#define RTSP_VERSION            "RTSP/1.0"
#define RTSP_MAX_CLIENTS        4
#define RTSP_MAX_SESSIONS       8
#define RTSP_MAX_URL_LENGTH     128
#define RTSP_MAX_HEADER_LENGTH  256
#define RTSP_RECV_BUF_SIZE      1024
#define RTSP_SEND_BUF_SIZE      1536

/* Session timeout in seconds (60s as per RFC 2326) */
#define RTSP_SESSION_TIMEOUT_S  60

/* ---- RTSP methods (RFC 2326, Section 6) ---- */

typedef enum {
	RTSP_METHOD_OPTIONS,
	RTSP_METHOD_DESCRIBE,
	RTSP_METHOD_ANNOUNCE,
	RTSP_METHOD_SETUP,
	RTSP_METHOD_PLAY,
	RTSP_METHOD_PAUSE,
	RTSP_METHOD_TEARDOWN,
	RTSP_METHOD_GET_PARAMETER,
	RTSP_METHOD_SET_PARAMETER,
	RTSP_METHOD_UNKNOWN
} rtsp_method_t;

/* ---- RTSP status codes (RFC 2326, Section 7.1.1) ---- */

typedef enum {
	RTSP_STATUS_OK                  = 200,
	RTSP_STATUS_CREATED             = 201,
	RTSP_STATUS_BAD_REQUEST         = 400,
	RTSP_STATUS_UNAUTHORIZED        = 401,
	RTSP_STATUS_NOT_FOUND           = 404,
	RTSP_STATUS_METHOD_NOT_ALLOWED  = 405,
	RTSP_STATUS_NOT_ACCEPTABLE      = 406,
	RTSP_STATUS_SESSION_NOT_FOUND   = 454,
	RTSP_STATUS_METHOD_INVALID      = 455,
	RTSP_STATUS_UNSUPPORTED_TRANSPORT = 461,
	RTSP_STATUS_INTERNAL_ERROR      = 500,
	RTSP_STATUS_NOT_IMPLEMENTED     = 501,
	RTSP_STATUS_SERVICE_UNAVAILABLE = 503
} rtsp_status_t;

/* ---- RTSP transport types ---- */

typedef enum {
	RTSP_TRANSPORT_RTP_AVP_UDP_UNICAST,
	RTSP_TRANSPORT_RTP_AVP_UDP_MULTICAST,
	RTSP_TRANSPORT_UNKNOWN
} rtsp_transport_type_t;

/* ---- Parsed RTSP request ---- */

struct rtsp_request {
	rtsp_method_t method;
	char url[RTSP_MAX_URL_LENGTH];
	uint32_t cseq;
	char session_id[32];
	rtsp_transport_type_t transport_type;
	uint16_t client_port_rtp;
	uint16_t client_port_rtcp;
	struct in_addr destination;
	bool has_session;
	bool has_transport;
};

/* ---- RTSP session ---- */

struct rtsp_session {
	bool active;
	char session_id[32];
	int client_sock;
	struct in_addr client_addr;
	uint8_t stream_id;  /* Associated TX/RX stream index */
	bool is_playing;
	int64_t last_activity_ms;
	rtsp_transport_type_t transport_type;
	uint16_t client_port_rtp;
	uint16_t client_port_rtcp;
};

/* ---- RTSP client connection (for RX stream subscription) ---- */

struct rtsp_client_connection {
	bool active;
	int sock;
	struct in_addr server_addr;
	uint16_t server_port;
	char session_id[32];
	uint8_t rx_stream_id;  /* Local RX stream to configure */
	uint32_t cseq;
};

/* ---- Public API: Server ---- */

/**
 * @brief Start the RTSP server.
 *
 * Spawns a background thread that listens on RTSP_DEFAULT_PORT (554)
 * for incoming RTSP requests.
 *
 * @return 0 on success, negative errno on error
 */
int rtsp_server_start(void);

/**
 * @brief Stop the RTSP server.
 */
void rtsp_server_stop(void);

/**
 * @brief Check if the RTSP server is running.
 */
bool rtsp_server_is_running(void);

/**
 * @brief Get the list of active RTSP sessions.
 *
 * @param count  Output: number of active sessions
 * @return Pointer to session array
 */
const struct rtsp_session *rtsp_get_sessions(int *count);

/**
 * @brief Push an SDP update (ANNOUNCE) to all connected clients
 *        subscribed to a specific stream.
 *
 * Per RAVENNA, this notifies clients of stream configuration changes.
 *
 * @param stream_id  TX stream that was updated
 * @return 0 on success, negative errno on error
 */
int rtsp_announce_stream_update(uint8_t stream_id);

/* ---- Public API: Client ---- */

/**
 * @brief Connect to a remote RTSP server and subscribe to a stream.
 *
 * This performs the RTSP client sequence: DESCRIBE -> SETUP -> PLAY
 * and configures the local RX stream with the received SDP information.
 *
 * @param server_addr   IP address of the RTSP server
 * @param server_port   RTSP port (554)
 * @param stream_name   Stream name/path (e.g., "stream1" or full URL)
 * @param rx_stream_id  Local RX stream index to configure
 * @return 0 on success, negative errno on error
 */
int rtsp_client_subscribe(const struct in_addr *server_addr,
			  uint16_t server_port,
			  const char *stream_name,
			  uint8_t rx_stream_id);

/**
 * @brief One-shot DESCRIBE: fetch and parse a remote session's SDP.
 *
 * Connects, requests rtsp://<addr>:<port>/by-name/<session_name>
 * (percent-encoded) and parses the returned SDP.  No SETUP/PLAY, the
 * connection is closed afterwards.  Used by mDNS session discovery.
 *
 * @param server_addr   IP address of the RTSP server
 * @param server_port   RTSP port
 * @param session_name  RAVENNA session name (raw, will be URL-encoded)
 * @param out           Parsed SDP output
 * @return 0 on success, negative errno on error
 */
int rtsp_client_describe(const struct in_addr *server_addr,
			 uint16_t server_port,
			 const char *session_name,
			 struct aes67_sdp_parsed *out);

/**
 * @brief Disconnect from a remote RTSP server (TEARDOWN).
 *
 * @param rx_stream_id  Local RX stream index
 * @return 0 on success, negative errno on error
 */
int rtsp_client_unsubscribe(uint8_t rx_stream_id);

/**
 * @brief Get active client connections.
 */
const struct rtsp_client_connection *rtsp_get_client_connections(int *count);

/* ---- URL helpers ---- */

/**
 * @brief Generate RTSP URL for a TX stream.
 *
 * Format: rtsp://<ip>:<port>/by-name/<stream_name>
 *         or rtsp://<ip>:<port>/by-id/<stream_id>
 *
 * @param buf         Output buffer
 * @param buf_size    Buffer size
 * @param stream_id   TX stream index
 * @return Number of bytes written (excluding NUL), or negative on error
 */
int rtsp_format_stream_url(char *buf, size_t buf_size, uint8_t stream_id);

/**
 * @brief Notify RTSP module that IP address is ready (after DHCP).
 *
 * @param addr  Local IP address
 */
void rtsp_notify_ip_ready(const struct in_addr *addr);

#ifdef __cplusplus
}
#endif

#endif /* RTSP_H_ */
