/*
 * Unit tests for the shared SDP builder/parser (src/aes67_sdp_utils.c).
 *
 * The generated body is what SAP announcements, RTSP DESCRIBE and the
 * NMOS sender manifests all ship, so the canonical case is asserted
 * verbatim — a silent change here breaks interop, not just a unit test.
 */

#include <zephyr/ztest.h>
#include <zephyr/net/net_ip.h>
#include <errno.h>
#include <string.h>

#include "aes67_sdp_utils.h"

static const uint8_t clock_id[8] = {
	0x02, 0xAA, 0xE6, 0x70, 0x00, 0x01, 0x02, 0x03
};

static struct in_addr addr(const char *s)
{
	struct in_addr a;

	zassert_equal(net_addr_pton(AF_INET, s, &a), 0, "bad address %s", s);
	return a;
}

static struct aes67_sdp_params default_params(void)
{
	struct aes67_sdp_params p = {
		.origin_addr        = addr("192.168.1.50"),
		.connection_addr    = addr("239.69.0.1"),
		.stream_id          = 0,
		.channel_count      = 2,
		.bit_depth          = 24,
		.sample_rate        = 48000,
		.samples_per_packet = 48,
		.port               = 5004,
		.payload_type       = 97,
		.ssrc               = 0x12345678,
		.clock_id           = clock_id,
		.stream_name        = "Test Stream",
		.ptp_domain         = 0,
		.sync_time          = 0,
	};

	return p;
}

/* 192.168.1.50 as a host-order integer — the session id the builder
 * derives from the origin address plus the stream index. */
#define SESSION_ID "3232235826"

static const char canonical_sdp[] =
	"v=0\r\n"
	"o=- " SESSION_ID " 0 IN IP4 192.168.1.50\r\n"
	"s=Test Stream\r\n"
	"i=2ch 24bit 48000Hz\r\n"
	"t=0 0\r\n"
	"a=clock-domain:PTPv2 0\r\n"
	"m=audio 5004 RTP/AVP 97\r\n"
	"c=IN IP4 239.69.0.1/32\r\n"
	"a=source-filter: incl IN IP4 239.69.0.1 192.168.1.50\r\n"
	"a=rtpmap:97 L24/48000/2\r\n"
	"a=ptime:1.000\r\n"
	"a=sync-time:0\r\n"
	"a=ts-refclk:ptp=IEEE1588-2008:02-AA-E6-70-00-01-02-03:0\r\n"
	"a=mediaclk:direct=0\r\n"
	"a=ssrc:305419896 cname:aes67@192.168.1.50\r\n";

ZTEST(sdp_utils, test_build_canonical)
{
	struct aes67_sdp_params p = default_params();
	char buf[1024];
	int n = aes67_sdp_build(buf, sizeof(buf), &p);

	zassert_equal(n, (int)strlen(canonical_sdp), "length mismatch");
	zassert_str_equal(buf, canonical_sdp);
}

ZTEST(sdp_utils, test_build_optional_lines)
{
	struct aes67_sdp_params p = default_params();
	char buf[1024];

	/* No grandmaster known: ts-refclk must be omitted rather than
	 * claiming traceability to an all-zero clock. */
	p.clock_id = NULL;
	zassert_true(aes67_sdp_build(buf, sizeof(buf), &p) > 0);
	zassert_is_null(strstr(buf, "ts-refclk"));

	/* SSRC 0 means "not assigned yet" -> no a=ssrc line. */
	p = default_params();
	p.ssrc = 0;
	zassert_true(aes67_sdp_build(buf, sizeof(buf), &p) > 0);
	zassert_is_null(strstr(buf, "a=ssrc:"));

	/* source-filter is a multicast-only (SSM) attribute. */
	p = default_params();
	p.connection_addr = addr("192.168.1.99");
	zassert_true(aes67_sdp_build(buf, sizeof(buf), &p) > 0);
	zassert_is_null(strstr(buf, "source-filter"));
	zassert_not_null(strstr(buf, "c=IN IP4 192.168.1.99/32\r\n"));

	/* ...and it needs a known origin to name as the source. */
	p = default_params();
	memset(&p.origin_addr, 0, sizeof(p.origin_addr));
	zassert_true(aes67_sdp_build(buf, sizeof(buf), &p) > 0);
	zassert_is_null(strstr(buf, "source-filter"));

	/* Empty / NULL stream name falls back to the generic session name. */
	p = default_params();
	p.stream_name = NULL;
	zassert_true(aes67_sdp_build(buf, sizeof(buf), &p) > 0);
	zassert_not_null(strstr(buf, "s=AES67 Stream\r\n"));
	p.stream_name = "";
	zassert_true(aes67_sdp_build(buf, sizeof(buf), &p) > 0);
	zassert_not_null(strstr(buf, "s=AES67 Stream\r\n"));
}

ZTEST(sdp_utils, test_build_ptime_and_domain)
{
	struct aes67_sdp_params p = default_params();
	char buf[1024];

	/* 12 samples @ 48 kHz = 250 us -> "0.250". */
	p.samples_per_packet = 12;
	p.ptp_domain = 127;
	p.sync_time = 4242;
	zassert_true(aes67_sdp_build(buf, sizeof(buf), &p) > 0);
	zassert_not_null(strstr(buf, "a=ptime:0.250\r\n"));
	zassert_not_null(strstr(buf, "a=clock-domain:PTPv2 127\r\n"));
	zassert_not_null(strstr(buf, "a=sync-time:4242\r\n"));
	zassert_not_null(strstr(buf,
		"a=ts-refclk:ptp=IEEE1588-2008:02-AA-E6-70-00-01-02-03:127\r\n"));

	/* 96 kHz / 96 samples = 1 ms again, and the rtpmap follows the rate. */
	p = default_params();
	p.sample_rate = 96000;
	p.samples_per_packet = 96;
	p.channel_count = 8;
	zassert_true(aes67_sdp_build(buf, sizeof(buf), &p) > 0);
	zassert_not_null(strstr(buf, "a=rtpmap:97 L24/96000/8\r\n"));
	zassert_not_null(strstr(buf, "a=ptime:1.000\r\n"));
}

ZTEST(sdp_utils, test_build_truncation_is_bounded)
{
	struct aes67_sdp_params p = default_params();
	/* Guard bytes on both sides: snprintf() returns the length it WOULD
	 * have written, so an unguarded builder walks off the end after the
	 * first truncated line. */
	struct {
		uint8_t front[16];
		char body[48];
		uint8_t back[16];
	} g;

	memset(&g, 0xA5, sizeof(g));

	for (size_t sz = 1; sz <= sizeof(g.body); sz++) {
		int n = aes67_sdp_build(g.body, sz, &p);

		zassert_equal(n, -ENOMEM,
			      "buffer of %zu bytes should not fit an SDP body",
			      sz);
		for (size_t i = 0; i < sizeof(g.front); i++) {
			zassert_equal(g.front[i], 0xA5, "underrun at %zu", i);
			zassert_equal(g.back[i], 0xA5, "overrun at %zu", i);
		}
	}
}

ZTEST(sdp_utils, test_parse_roundtrip)
{
	struct aes67_sdp_parsed out;

	zassert_equal(aes67_sdp_parse(canonical_sdp, strlen(canonical_sdp),
				      &out), 0);

	zassert_str_equal(out.name, "Test Stream");
	/* "o=-" is the anonymous origin: no user name to report. */
	zassert_str_equal(out.origin_name, "");
	zassert_equal(out.origin_addr.s_addr, addr("192.168.1.50").s_addr);
	zassert_equal(out.connection_addr.s_addr, addr("239.69.0.1").s_addr);
	zassert_equal(out.port, 5004);
	zassert_equal(out.channels, 2);
	zassert_equal(out.bit_depth, 24);
	zassert_equal(out.sample_rate, 48000);
	zassert_equal(out.ssrc, 0x12345678);
	/* a=ptime:1.000 @ 48 kHz -> 48 samples per packet. */
	zassert_equal(out.samples_per_packet, 48);
	zassert_true(out.has_clock_domain);
	zassert_equal(out.ptp_domain, 0);
	zassert_true(out.has_sync_time);
	zassert_equal(out.sync_time, 0);
}

ZTEST(sdp_utils, test_parse_foreign_variants)
{
	struct aes67_sdp_parsed out;
	/* A RAVENNA-style announcement: named origin, LF-only line ends,
	 * session-level c=, fractional ptime, non-zero domain. */
	static const char sdp[] =
		"v=0\n"
		"o=ravenna 1234 1 IN IP4 10.0.0.7\n"
		"s=Horus Out 1\n"
		"c=IN IP4 239.1.2.3/32\n"
		"t=0 0\n"
		"a=clock-domain:PTPv2 42\n"
		"m=audio 5004 RTP/AVP 98\n"
		"a=rtpmap:98 L16/44100/1\n"
		"a=ptime:0.125\n"
		"a=sync-time:987654\n"
		"a=ssrc:42 cname:x@y\n";

	zassert_equal(aes67_sdp_parse(sdp, strlen(sdp), &out), 0);
	zassert_str_equal(out.name, "Horus Out 1");
	zassert_str_equal(out.origin_name, "ravenna");
	zassert_equal(out.origin_addr.s_addr, addr("10.0.0.7").s_addr);
	zassert_equal(out.connection_addr.s_addr, addr("239.1.2.3").s_addr);
	zassert_equal(out.port, 5004);
	zassert_equal(out.channels, 1);
	zassert_equal(out.bit_depth, 16);
	zassert_equal(out.sample_rate, 44100);
	zassert_equal(out.ssrc, 42);
	zassert_equal(out.ptp_domain, 42);
	zassert_true(out.has_clock_domain);
	zassert_equal(out.sync_time, 987654);
	/* 125 us @ 44.1 kHz = 5.5 -> 5 samples (integer truncation). */
	zassert_equal(out.samples_per_packet, 5);
}

ZTEST(sdp_utils, test_parse_defensive)
{
	struct aes67_sdp_parsed out;

	/* Empty and junk input must not fault; everything stays zeroed. */
	zassert_equal(aes67_sdp_parse("", 0, &out), 0);
	zassert_equal(out.port, 0);
	zassert_equal(out.sample_rate, 0);
	zassert_false(out.has_clock_domain);
	zassert_false(out.has_sync_time);

	static const char junk[] = "not sdp at all\r\nx\r\n=\r\nv\r\n";

	zassert_equal(aes67_sdp_parse(junk, strlen(junk), &out), 0);
	zassert_equal(out.port, 0);

	/* An over-long session name is truncated into the fixed field. */
	static const char long_name[] =
		"s=0123456789012345678901234567890123456789\r\n";

	zassert_equal(aes67_sdp_parse(long_name, strlen(long_name), &out), 0);
	zassert_equal(strlen(out.name), AES67_SDP_NAME_MAX - 1);

	/* ptime without a rtpmap: no sample rate, so no conversion happens
	 * and the raw microseconds are left in place. */
	static const char ptime_only[] = "a=ptime:1.000\r\n";

	zassert_equal(aes67_sdp_parse(ptime_only, strlen(ptime_only), &out), 0);
	zassert_equal(out.sample_rate, 0);
	zassert_equal(out.samples_per_packet, 1000);

	/* Non-PTPv2 clock-domain is ignored rather than mis-parsed. */
	static const char other_clock[] = "a=clock-domain:PTPv1 3\r\n";

	zassert_equal(aes67_sdp_parse(other_clock, strlen(other_clock), &out),
		      0);
	zassert_false(out.has_clock_domain);
}

ZTEST(sdp_utils, test_format_clock_id)
{
	char buf[32];

	zassert_equal(aes67_format_clock_id(buf, sizeof(buf), clock_id), 23);
	zassert_str_equal(buf, "02-AA-E6-70-00-01-02-03");
}

ZTEST(sdp_utils, test_ptime_us)
{
	zassert_equal(aes67_ptime_us(48, 48000), 1000);
	zassert_equal(aes67_ptime_us(6, 48000), 125);
	zassert_equal(aes67_ptime_us(96, 96000), 1000);
	zassert_equal(aes67_ptime_us(64, 48000), 1333);
}

ZTEST(sdp_utils, test_strncasecmp)
{
	zassert_equal(aes67_strncasecmp("abc", "ABC", 3), 0);
	zassert_equal(aes67_strncasecmp("abc", "abc", 100), 0,
		      "must stop at the NUL, not read past it");
	zassert_equal(aes67_strncasecmp("abcdef", "abcXYZ", 3), 0);
	zassert_true(aes67_strncasecmp("abc", "abd", 3) < 0);
	zassert_true(aes67_strncasecmp("abd", "abc", 3) > 0);
	zassert_equal(aes67_strncasecmp("anything", "else", 0), 0);
	zassert_true(aes67_strncasecmp("ab", "abc", 3) < 0);
}

ZTEST(sdp_utils, test_parse_uint)
{
	uint32_t v;
	const char *s = "1234abc";
	const char *p = aes67_parse_uint(s, s + strlen(s), &v);

	zassert_equal(v, 1234);
	zassert_equal(p, s + 4, "must stop at the first non-digit");

	/* No digits at all: zero, and the cursor does not move. */
	s = "abc";
	p = aes67_parse_uint(s, s + 3, &v);
	zassert_equal(v, 0);
	zassert_equal(p, s);

	/* The end pointer is honoured even mid-number. */
	s = "9999";
	p = aes67_parse_uint(s, s + 2, &v);
	zassert_equal(v, 99);
	zassert_equal(p, s + 2);
}

ZTEST_SUITE(sdp_utils, NULL, NULL, NULL, NULL, NULL);
