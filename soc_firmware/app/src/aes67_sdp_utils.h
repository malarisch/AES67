/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: Apache-2.0
 *
 * AES67 SDP utility functions - shared helpers for SAP/SDP and RTSP modules.
 *
 * This module provides common functions to avoid code duplication:
 *   - SDP body generation for TX streams
 *   - SDP field parsing
 *   - PTP clock ID formatting
 *   - String helpers
 */

#ifndef AES67_SDP_UTILS_H_
#define AES67_SDP_UTILS_H_

#include <zephyr/kernel.h>
#include <zephyr/net/net_ip.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- SDP generation parameters ---- */

struct aes67_sdp_params {
	struct in_addr origin_addr;      /* o= line: origin IP */
	struct in_addr connection_addr;  /* c= line: multicast/unicast dest */
	uint8_t        stream_id;        /* For session ID uniqueness */
	uint8_t        channel_count;
	uint8_t        bit_depth;
	uint32_t       sample_rate;
	uint16_t       samples_per_packet;
	uint16_t       port;             /* RTP port */
	uint8_t        payload_type;
	uint32_t       ssrc;             /* 0 = omit a=ssrc line */
	const uint8_t *clock_id;         /* 8-byte PTP clock ID, NULL = omit */
	const char    *stream_name;      /* s= line, NULL = use default */
	/* RAVENNA extensions */
	uint8_t        ptp_domain;       /* a=clock-domain:PTPv2 <domain> */
	uint32_t       sync_time;        /* a=sync-time:<rtp_timestamp>, 0 = use default */
};

/* ---- Parsed SDP fields ---- */

#define AES67_SDP_NAME_MAX  32

struct aes67_sdp_parsed {
	char           name[AES67_SDP_NAME_MAX];
	char           origin_name[AES67_SDP_NAME_MAX];
	struct in_addr origin_addr;
	struct in_addr connection_addr;
	uint16_t       port;
	uint8_t        channels;
	uint8_t        bit_depth;
	uint32_t       sample_rate;
	uint32_t       ssrc;
	uint16_t       samples_per_packet;
	/* RAVENNA extensions */
	uint8_t        ptp_domain;       /* from a=clock-domain:PTPv2 <domain> */
	bool           has_clock_domain; /* true if a=clock-domain was present */
	uint32_t       sync_time;        /* from a=sync-time:<timestamp> */
	bool           has_sync_time;    /* true if a=sync-time was present */
};

/* ================================================================
 * SDP Generation
 * ================================================================ */

/**
 * @brief Build an AES67-compliant SDP body.
 *
 * Generates a complete SDP session description including:
 *   v=, o=, s=, i=, c=, t=, m=, a=rtpmap, a=ptime, a=ts-refclk, a=mediaclk
 *   and optionally a=ssrc.
 *
 * @param buf       Output buffer
 * @param buf_size  Size of output buffer
 * @param params    SDP parameters
 * @return Number of bytes written (excluding NUL), or negative errno
 */
int aes67_sdp_build(char *buf, size_t buf_size,
		    const struct aes67_sdp_params *params);

/**
 * @brief Compute ptime in microseconds from samples_per_packet and sample_rate.
 *
 * @param samples_per_packet  Samples per packet per channel
 * @param sample_rate         Sample rate in Hz
 * @return ptime in microseconds
 */
static inline uint32_t aes67_ptime_us(uint16_t samples_per_packet,
				      uint32_t sample_rate)
{
	return (uint32_t)samples_per_packet * 1000000U / sample_rate;
}

/* ================================================================
 * SDP Parsing
 * ================================================================ */

/**
 * @brief Parse SDP fields from an SDP body string.
 *
 * Extracts common fields: session name, connection address, port,
 * audio parameters (channels, bit depth, sample rate), SSRC, and ptime.
 *
 * @param sdp       Pointer to SDP body (after headers)
 * @param sdp_len   Length of SDP body
 * @param out       Output structure for parsed fields
 * @return 0 on success, negative errno on error
 */
int aes67_sdp_parse(const char *sdp, size_t sdp_len,
		    struct aes67_sdp_parsed *out);

/* ================================================================
 * PTP Clock ID Formatting
 * ================================================================ */

/**
 * @brief Format an 8-byte PTP clock identity as "XX-XX-XX-XX-XX-XX-XX-XX".
 *
 * @param buf      Output buffer (at least 24 bytes)
 * @param buf_len  Size of output buffer
 * @param id       8-byte clock identity
 * @return Number of bytes written (excluding NUL)
 */
int aes67_format_clock_id(char *buf, size_t buf_len, const uint8_t id[8]);

/* ================================================================
 * String Helpers
 * ================================================================ */

/**
 * @brief Case-insensitive string comparison (strncasecmp replacement).
 *
 * picolibc does not provide strncasecmp, so we provide our own.
 *
 * @param s1  First string
 * @param s2  Second string
 * @param n   Maximum characters to compare
 * @return 0 if equal, <0 if s1<s2, >0 if s1>s2
 */
int aes67_strncasecmp(const char *s1, const char *s2, size_t n);

/**
 * @brief Skip whitespace in a string.
 *
 * @param p    Pointer to string
 * @param end  End of string (exclusive)
 * @return Pointer to first non-whitespace character, or end if all whitespace
 */
static inline const char *aes67_skip_ws(const char *p, const char *end)
{
	while (p < end && (*p == ' ' || *p == '\t')) {
		p++;
	}
	return p;
}

/**
 * @brief Parse an unsigned integer from a string.
 *
 * Stops at first non-digit character.
 *
 * @param p      Pointer to string
 * @param end    End of string (exclusive)
 * @param out    Output value
 * @return Pointer past the last digit parsed
 */
const char *aes67_parse_uint(const char *p, const char *end, uint32_t *out);

#ifdef __cplusplus
}
#endif

#endif /* AES67_SDP_UTILS_H_ */
