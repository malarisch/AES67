/*
 * OCP.1 marshaling: big-endian primitives, strings, blobs, PDU framing.
 */

#include <zephyr/ztest.h>
#include <string.h>

#include "ocp1.h"

ZTEST_SUITE(ocp1, NULL, NULL, NULL, NULL, NULL);

ZTEST(ocp1, test_primitives_roundtrip)
{
	uint8_t buf[64];
	struct ocp1_writer w;
	struct ocp1_reader r;
	char str[16];

	ocp1_w_init(&w, buf, sizeof(buf));
	ocp1_w_u8(&w, 0xAB);
	ocp1_w_u16(&w, 0x1234);
	ocp1_w_u32(&w, 0xDEADBEEF);
	ocp1_w_u64(&w, 0x0102030405060708ULL);
	ocp1_w_i32(&w, -22);
	ocp1_w_f32(&w, -22.0f);
	ocp1_w_bool(&w, true);
	ocp1_w_str(&w, "Errors");
	zassert_false(w.overflow);

	/* Network byte order on the wire */
	zassert_equal(buf[0], 0xAB);
	zassert_equal(buf[1], 0x12);
	zassert_equal(buf[2], 0x34);
	zassert_equal(buf[3], 0xDE);
	zassert_equal(buf[6], 0xEF);
	/* -22.0f = 0xC1B00000 */
	zassert_equal(buf[19], 0xC1);
	zassert_equal(buf[20], 0xB0);
	/* OcaString: u16 count + bytes */
	zassert_equal(buf[24], 0);
	zassert_equal(buf[25], 6);
	zassert_mem_equal(&buf[26], "Errors", 6);

	ocp1_r_init(&r, buf, w.len);
	zassert_equal(ocp1_r_u8(&r), 0xAB);
	zassert_equal(ocp1_r_u16(&r), 0x1234);
	zassert_equal(ocp1_r_u32(&r), 0xDEADBEEF);
	zassert_equal(ocp1_r_u64(&r), 0x0102030405060708ULL);
	zassert_equal(ocp1_r_i32(&r), -22);
	zassert_equal(ocp1_r_f32(&r), -22.0f);
	zassert_true(ocp1_r_bool(&r));
	zassert_equal(ocp1_r_str(&r, str, sizeof(str)), 6);
	zassert_str_equal(str, "Errors");
	zassert_false(r.err);
	zassert_equal(ocp1_r_remaining(&r), 0);
}

/* AES70-3 6.3.2.4: the OcaCounter example byte sequence. */
ZTEST(ocp1, test_counter_example)
{
	static const uint8_t expect[] = {
		0, 3,                       /* ID */
		0, 0, 0, 0, 0, 0, 0, 100,   /* Value */
		0, 0, 0, 0, 0, 0, 0, 0,     /* InitialValue */
		0, 6, 'E', 'r', 'r', 'o', 'r', 's',
		0, 0,                       /* Notifiers: empty list */
	};
	uint8_t buf[64];
	struct ocp1_writer w;

	ocp1_w_init(&w, buf, sizeof(buf));
	ocp1_w_u16(&w, 3);
	ocp1_w_u64(&w, 100);
	ocp1_w_u64(&w, 0);
	ocp1_w_str(&w, "Errors");
	ocp1_w_u16(&w, 0);
	zassert_equal(w.len, sizeof(expect));
	zassert_mem_equal(buf, expect, sizeof(expect));
}

ZTEST(ocp1, test_writer_overflow_is_sticky)
{
	uint8_t buf[3];
	struct ocp1_writer w;

	ocp1_w_init(&w, buf, sizeof(buf));
	ocp1_w_u16(&w, 1);
	zassert_false(w.overflow);
	ocp1_w_u16(&w, 2);       /* does not fit */
	zassert_true(w.overflow);
	zassert_equal(w.len, 2);
	ocp1_w_u8(&w, 3);        /* would fit, but overflow is sticky */
	zassert_equal(w.len, 2);
}

ZTEST(ocp1, test_reader_short_input)
{
	uint8_t buf[3] = {0, 5, 'a'};
	struct ocp1_reader r;
	char str[8];

	ocp1_r_init(&r, buf, sizeof(buf));
	zassert_equal(ocp1_r_str(&r, str, sizeof(str)), 0);
	zassert_true(r.err);
	zassert_equal(ocp1_r_u32(&r), 0);
	zassert_equal(ocp1_r_remaining(&r), 0);
}

ZTEST(ocp1, test_string_truncation)
{
	uint8_t buf[16];
	struct ocp1_writer w;
	struct ocp1_reader r;
	char str[4];

	ocp1_w_init(&w, buf, sizeof(buf));
	ocp1_w_str(&w, "abcdef");
	ocp1_w_u8(&w, 0x42);
	ocp1_r_init(&r, buf, w.len);
	zassert_equal(ocp1_r_str(&r, str, sizeof(str)), 6);
	zassert_str_equal(str, "abc");
	/* The whole string was consumed regardless of the truncation. */
	zassert_equal(ocp1_r_u8(&r), 0x42);
}

ZTEST(ocp1, test_blob_and_class_id)
{
	static const uint8_t data[] = {1, 2, 3};
	static const uint16_t cid[] = {1, 1, 1, 5};
	uint8_t buf[32];
	struct ocp1_writer w;
	struct ocp1_reader r;
	size_t n;
	const uint8_t *p;

	ocp1_w_init(&w, buf, sizeof(buf));
	ocp1_w_blob(&w, data, sizeof(data));
	ocp1_w_class_id(&w, cid, 4);
	zassert_equal(w.len, 2 + 3 + 2 + 8);

	ocp1_r_init(&r, buf, w.len);
	p = ocp1_r_blob(&r, &n);
	zassert_not_null(p);
	zassert_equal(n, 3);
	zassert_mem_equal(p, data, 3);
	zassert_equal(ocp1_r_u16(&r), 4);
	zassert_equal(ocp1_r_u16(&r), 1);
	zassert_equal(ocp1_r_u16(&r), 1);
	zassert_equal(ocp1_r_u16(&r), 1);
	zassert_equal(ocp1_r_u16(&r), 5);
}

ZTEST(ocp1, test_pdu_header)
{
	uint8_t buf[32];
	struct ocp1_writer w;
	struct ocp1_header h;
	size_t off;

	ocp1_w_init(&w, buf, sizeof(buf));
	off = ocp1_begin_pdu(&w, OCP1_PDU_KEEPALIVE, 1);
	ocp1_w_u16(&w, 5);
	ocp1_end_pdu(&w, off);

	/* sync + 9 header bytes + 2 payload */
	zassert_equal(w.len, 12);
	zassert_equal(buf[0], OCP1_SYNC_VAL);
	zassert_equal(buf[1], 0);
	zassert_equal(buf[2], 1);                /* protocol version */
	zassert_equal(buf[6], 11);               /* PduSize excludes sync */
	zassert_equal(buf[7], OCP1_PDU_KEEPALIVE);
	zassert_equal(buf[9], 1);                /* MessageCount */

	zassert_equal(ocp1_parse_header(buf, w.len, &h), 0);
	zassert_equal(h.version, 1);
	zassert_equal(h.pdu_size, 11);
	zassert_equal(h.type, OCP1_PDU_KEEPALIVE);
	zassert_equal(h.count, 1);

	zassert_equal(ocp1_parse_header(buf, 5, &h), -EAGAIN);
	buf[0] = 0x00;
	zassert_equal(ocp1_parse_header(buf, w.len, &h), -EBADMSG);
	buf[0] = OCP1_SYNC_VAL;
	/* AES70.js / AES70 Explorer send their edition number (2..4) as the
	 * protocol version: must be accepted. */
	buf[2] = 4;
	zassert_equal(ocp1_parse_header(buf, w.len, &h), 0);
	zassert_equal(h.version, 4);
	buf[2] = 0;
	zassert_equal(ocp1_parse_header(buf, w.len, &h), -EBADMSG);
}
