/*
 * Unit tests for the REST API's JSON writer and its request-body entry
 * point (src/webapi/json_util.c).
 *
 * The writer's contract is that it never emits truncated (invalid)
 * JSON: overflow is sticky and the caller turns it into a 500 instead
 * of shipping a half-written body.
 *
 * Reading goes through Zephyr's JSON library: what is asserted here is
 * the contract the handlers rely on — absent keys are left untouched
 * (PATCH semantics), presence is reported through the bitmap, and
 * anything malformed or out of range is rejected outright.
 */

#include <zephyr/ztest.h>
#include <errno.h>
#include <string.h>

#include "webapi_priv.h"

static char buf[256];
static struct json_out jo;

static const char *built(void)
{
	jo_finish(&jo);
	return buf;
}

ZTEST(json_util, test_flat_object)
{
	jo_init(&jo, buf, sizeof(buf));
	jo_obj_begin(&jo);
	jo_str(&jo, "name", "node");
	jo_int(&jo, "offset", -42);
	jo_uint(&jo, "ssrc", 3000000000u);
	jo_bool(&jo, "locked", true);
	jo_bool(&jo, "muted", false);
	jo_obj_end(&jo);

	zassert_false(jo.overflow);
	zassert_str_equal(built(),
			  "{\"name\":\"node\",\"offset\":-42,"
			  "\"ssrc\":3000000000,\"locked\":true,"
			  "\"muted\":false}");
}

ZTEST(json_util, test_nested_containers)
{
	jo_init(&jo, buf, sizeof(buf));
	jo_obj_begin(&jo);
	jo_key(&jo, "streams");
	jo_arr_begin(&jo);
	for (int i = 0; i < 2; i++) {
		jo_obj_begin(&jo);
		jo_int(&jo, "id", i);
		jo_obj_end(&jo);
	}
	jo_arr_end(&jo);
	jo_key(&jo, "empty");
	jo_arr_begin(&jo);
	jo_arr_end(&jo);
	jo_obj_end(&jo);

	zassert_false(jo.overflow);
	zassert_str_equal(built(),
			  "{\"streams\":[{\"id\":0},{\"id\":1}],\"empty\":[]}");
}

ZTEST(json_util, test_string_escaping)
{
	jo_init(&jo, buf, sizeof(buf));
	jo_obj_begin(&jo);
	jo_str(&jo, "s", "a\"b\\c\nd\re\tf");
	jo_obj_end(&jo);

	zassert_false(jo.overflow);
	zassert_str_equal(built(),
			  "{\"s\":\"a\\\"b\\\\c\\nd\\re\\tf\"}");

	/* Other control characters go out as \u00XX so the body stays
	 * valid JSON even when a card reports a garbage name. */
	char raw[] = { 'x', 0x01, 0x1f, '\0' };

	jo_init(&jo, buf, sizeof(buf));
	jo_obj_begin(&jo);
	jo_str(&jo, "s", raw);
	jo_obj_end(&jo);
	zassert_str_equal(built(), "{\"s\":\"x\\u0001\\u001f\"}");
}

ZTEST(json_util, test_overflow_is_sticky)
{
	char small[16];

	jo_init(&jo, small, sizeof(small));
	jo_obj_begin(&jo);
	jo_str(&jo, "key", "a value far too long for this buffer");
	zassert_true(jo.overflow);

	size_t pos_at_overflow = jo.pos;

	/* Anything written after the overflow is dropped, and what is in
	 * the buffer is still a NUL-terminated C string. */
	jo_int(&jo, "more", 1);
	jo_obj_end(&jo);
	zassert_true(jo.overflow);
	zassert_equal(jo.pos, pos_at_overflow);
	zassert_true(strlen(small) < sizeof(small));

	/* A zero-sized buffer overflows immediately instead of writing. */
	jo_init(&jo, small, 0);
	zassert_true(jo.overflow);
	jo_obj_begin(&jo);
	zassert_equal(jo.pos, 0);
}

/* ================================================================
 * Request body parsing (Zephyr JSON library via webapi_parse_body)
 * ================================================================ */

struct probe {
	char     name[16];
	int32_t  port;
	uint8_t  channels;
	int8_t   trim;
	bool     enabled;
	uint8_t  ch_ids[8];
	size_t   ch_ids_len;
};

enum { P_NAME, P_PORT, P_CHANNELS, P_TRIM, P_ENABLED, P_CH_IDS };

static const struct json_obj_descr probe_descr[] = {
	[P_NAME] = JSON_OBJ_DESCR_PRIM(struct probe, name,
				       JSON_TOK_STRING_BUF),
	[P_PORT] = JSON_OBJ_DESCR_PRIM(struct probe, port, JSON_TOK_NUMBER),
	[P_CHANNELS] = JSON_OBJ_DESCR_PRIM(struct probe, channels,
					   JSON_TOK_UINT),
	[P_TRIM] = JSON_OBJ_DESCR_PRIM(struct probe, trim, JSON_TOK_INT),
	[P_ENABLED] = JSON_OBJ_DESCR_PRIM(struct probe, enabled,
					  JSON_TOK_TRUE),
	[P_CH_IDS] = JSON_OBJ_DESCR_ARRAY(struct probe, ch_ids, 8, ch_ids_len,
					  JSON_TOK_UINT),
};

/* The parser terminates tokens in place, so bodies must be writable. */
static int64_t parse(char *body, struct probe *out)
{
	struct webapi_request req = {
		.body = body,
		.body_len = strlen(body),
	};

	return webapi_parse_body(&req, probe_descr, ARRAY_SIZE(probe_descr),
				 out);
}

ZTEST(json_util, test_body_decodes_every_type)
{
	char body[] = "{\"name\":\"studio-1\",\"port\":5004,"
		      "\"channels\":8,\"trim\":-12,\"enabled\":true,"
		      "\"ch_ids\":[0,1,2,3]}";
	struct probe p = { 0 };
	int64_t present = parse(body, &p);

	zassert_true(present > 0, "parse failed: %d", (int)present);
	zassert_str_equal(p.name, "studio-1");
	zassert_equal(p.port, 5004);
	zassert_equal(p.channels, 8);
	zassert_equal(p.trim, -12);
	zassert_true(p.enabled);
	zassert_equal(p.ch_ids_len, 4);
	for (int i = 0; i < 4; i++) {
		zassert_equal(p.ch_ids[i], i);
	}

	/* Every descriptor reports present. */
	for (int i = 0; i <= P_CH_IDS; i++) {
		zassert_true(WEBAPI_HAS(present, i), "field %d not reported",
			     i);
	}
}

ZTEST(json_util, test_absent_keys_are_left_alone)
{
	/* PATCH semantics: pre-fill, parse, and everything the body does
	 * not mention keeps the value it had. */
	char body[] = "{\"port\":9000}";
	struct probe p = {
		.port = 5004,
		.channels = 2,
		.trim = 3,
		.enabled = true,
		.ch_ids_len = 2,
	};

	strcpy(p.name, "keep-me");

	int64_t present = parse(body, &p);

	zassert_true(present > 0);
	zassert_equal(p.port, 9000);
	zassert_str_equal(p.name, "keep-me");
	zassert_equal(p.channels, 2);
	zassert_equal(p.trim, 3);
	zassert_true(p.enabled);
	zassert_equal(p.ch_ids_len, 2);

	zassert_true(WEBAPI_HAS(present, P_PORT));
	zassert_false(WEBAPI_HAS(present, P_NAME));
	zassert_false(WEBAPI_HAS(present, P_ENABLED));
}

ZTEST(json_util, test_whitespace_and_unknown_keys)
{
	/* The old substring matcher missed a value behind a space after
	 * the colon, and could match a key nested in another object. */
	char spaced[] = "{ \"name\" : \"spaced\" , \"port\" : 42 }";
	char nested[] = "{\"other\":{\"port\":1},\"port\":7}";
	struct probe p = { 0 };

	zassert_true(parse(spaced, &p) > 0);
	zassert_str_equal(p.name, "spaced");
	zassert_equal(p.port, 42);

	memset(&p, 0, sizeof(p));
	zassert_true(parse(nested, &p) > 0);
	zassert_equal(p.port, 7, "a nested key must not be picked up");
}

ZTEST(json_util, test_escapes_are_decoded)
{
	char body[] = "{\"name\":\"a\\\"b\\\\c\"}";
	struct probe p = { 0 };

	zassert_true(parse(body, &p) > 0);
	zassert_str_equal(p.name, "a\"b\\c");
}

ZTEST(json_util, test_malformed_bodies_are_rejected)
{
	static const char *const bad[] = {
		"{",
		"{\"port\":}",
		"{\"port\" 5004}",
		"not json",
	};
	struct probe p = { 0 };

	ARRAY_FOR_EACH(bad, i) {
		char body[64];

		strncpy(body, bad[i], sizeof(body) - 1);
		body[sizeof(body) - 1] = '\0';
		zassert_true(parse(body, &p) < 0,
			     "\"%s\" should not parse", bad[i]);
	}

	/* Anything after the closing brace is ignored rather than
	 * rejected — the object itself is what the endpoint contracts on,
	 * and an HTTP body never carries a second document. */
	char trailing[] = "{\"port\":5004} and then some";

	zassert_true(parse(trailing, &p) > 0);
	zassert_equal(p.port, 5004);

	/* Out-of-range for the destination type is rejected, not wrapped. */
	char overflow[] = "{\"channels\":300}";

	zassert_true(parse(overflow, &p) < 0);

	/* An empty body is reported as such rather than parsed. */
	char empty[] = "";

	zassert_equal(parse(empty, &p), -EINVAL);
}

ZTEST(json_util, test_oversized_values_are_rejected)
{
	/* name[] is 16 bytes; a longer value must fail rather than being
	 * silently truncated into the fixed field. */
	char body[] = "{\"name\":\"0123456789012345678\"}";
	struct probe p = { 0 };

	zassert_true(parse(body, &p) < 0);

	/* More array elements than the destination holds. */
	char too_many[] = "{\"ch_ids\":[0,1,2,3,4,5,6,7,8,9]}";

	zassert_true(parse(too_many, &p) < 0);
}

ZTEST_SUITE(json_util, NULL, NULL, NULL, NULL, NULL);
