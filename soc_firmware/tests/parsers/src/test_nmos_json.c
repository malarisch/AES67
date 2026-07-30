/*
 * Unit tests for the minimal JSON DOM parser behind the NMOS
 * Connection API (src/nmos/nmos_json.c).
 *
 * The IS-05/IS-08 PATCH bodies come straight off the wire, so the
 * malformed-input paths matter as much as the happy path.
 */

#include <zephyr/ztest.h>
#include <errno.h>
#include <stdint.h>
#include <string.h>

#include "nmos_json.h"

#define POOL_SIZE 64

static struct nj_node pool[POOL_SIZE];

static const struct nj_node *parse_ok(const char *json)
{
	int root = nj_parse(json, strlen(json), pool, POOL_SIZE);

	zassert_true(root >= 0, "parse failed (%d) for: %s", root, json);
	return &pool[root];
}

ZTEST(nmos_json, test_scalars)
{
	const struct nj_node *root = parse_ok(
		"{\"s\":\"text\",\"i\":42,\"neg\":-7,\"t\":true,"
		"\"f\":false,\"n\":null}");

	zassert_equal(root->type, NJ_OBJ);
	zassert_equal(nj_count(pool, root), 6);

	const struct nj_node *s = nj_get(pool, root, "s");

	zassert_not_null(s);
	zassert_equal(s->type, NJ_STR);
	zassert_true(nj_streq(s, "text"));
	zassert_false(nj_streq(s, "other"));

	zassert_equal(nj_get(pool, root, "i")->num, 42);
	zassert_equal(nj_get(pool, root, "neg")->num, -7);

	zassert_equal(nj_get(pool, root, "t")->type, NJ_BOOL);
	zassert_true(nj_get(pool, root, "t")->bval);
	zassert_false(nj_get(pool, root, "f")->bval);
	zassert_equal(nj_get(pool, root, "n")->type, NJ_NULL);

	/* Absent key, and lookups on non-objects. */
	zassert_is_null(nj_get(pool, root, "missing"));
	zassert_is_null(nj_get(pool, s, "s"));
	zassert_is_null(nj_get(pool, NULL, "s"));
}

ZTEST(nmos_json, test_nested)
{
	/* Shaped like an IS-05 sender PATCH. */
	const struct nj_node *root = parse_ok(
		"{\"transport_params\":["
		"{\"destination_ip\":\"239.69.0.1\",\"destination_port\":5004},"
		"{\"destination_ip\":\"auto\",\"destination_port\":\"auto\"}],"
		"\"activation\":{\"mode\":\"activate_immediate\"}}");

	const struct nj_node *tp = nj_get(pool, root, "transport_params");

	zassert_not_null(tp);
	zassert_equal(tp->type, NJ_ARR);
	zassert_equal(nj_count(pool, tp), 2);

	const struct nj_node *leg0 = nj_item(pool, tp, 0);
	const struct nj_node *leg1 = nj_item(pool, tp, 1);

	zassert_not_null(leg0);
	zassert_true(nj_streq(nj_get(pool, leg0, "destination_ip"),
			      "239.69.0.1"));
	zassert_equal(nj_get(pool, leg0, "destination_port")->num, 5004);
	zassert_true(nj_streq(nj_get(pool, leg1, "destination_port"), "auto"));

	/* Out-of-range index. */
	zassert_is_null(nj_item(pool, tp, 2));
	zassert_is_null(nj_item(pool, tp, -1));

	const struct nj_node *act = nj_get(pool, root, "activation");

	zassert_true(nj_streq(nj_get(pool, act, "mode"), "activate_immediate"));

	/* nj_item() is array-only by contract: an object's members are not
	 * reachable by index, only by key. */
	zassert_is_null(nj_item(pool, act, 0));
	zassert_is_null(nj_item(pool, NULL, 0));
}

ZTEST(nmos_json, test_empty_containers)
{
	const struct nj_node *root = parse_ok("{\"a\":[],\"o\":{}}");

	const struct nj_node *a = nj_get(pool, root, "a");
	const struct nj_node *o = nj_get(pool, root, "o");

	zassert_equal(a->type, NJ_ARR);
	zassert_equal(o->type, NJ_OBJ);
	zassert_equal(nj_count(pool, a), 0);
	zassert_equal(nj_count(pool, o), 0);
	zassert_equal(a->first_child, -1);
	zassert_is_null(nj_item(pool, a, 0));

	/* nj_count() on scalars and NULL is defined as 0. */
	zassert_equal(nj_count(pool, NULL), 0);
	zassert_equal(nj_count(pool, parse_ok("7")), 0);
}

ZTEST(nmos_json, test_top_level_scalar_and_whitespace)
{
	const struct nj_node *n = parse_ok("  \r\n\t 42 \r\n ");

	zassert_equal(n->type, NJ_NUM);
	zassert_equal(n->num, 42);

	n = parse_ok("\t{ \"a\" : [ 1 , 2 ] }\n");
	zassert_equal(nj_count(pool, nj_get(pool, n, "a")), 2);
}

ZTEST(nmos_json, test_number_edges)
{
	/* Non-integer numbers are accepted syntactically, integer part kept. */
	zassert_equal(parse_ok("1.75")->num, 1);
	zassert_equal(parse_ok("-2.5")->num, -2);
	zassert_equal(parse_ok("1e3")->num, 1);

	/* Values beyond int32 saturate rather than wrap. */
	zassert_equal(parse_ok("99999999999")->num, INT32_MAX);
	zassert_equal(parse_ok("0")->num, 0);
}

ZTEST(nmos_json, test_strcpy_unescapes)
{
	const struct nj_node *root = parse_ok(
		"{\"a\":\"line\\nnext\\ttab\","
		"\"b\":\"quote\\\"slash\\\\fwd\\/\","
		"\"c\":\"\\u0041\\u007a\","
		"\"d\":\"\\u1234\"}");
	char out[32];

	zassert_equal(nj_strcpy(nj_get(pool, root, "a"), out, sizeof(out)), 13);
	zassert_mem_equal(out, "line\nnext\ttab", 14);

	zassert_true(nj_strcpy(nj_get(pool, root, "b"), out, sizeof(out)) > 0);
	zassert_str_equal(out, "quote\"slash\\fwd/");

	/* Basic-Latin \u escapes are mapped, anything else becomes '?'. */
	zassert_equal(nj_strcpy(nj_get(pool, root, "c"), out, sizeof(out)), 2);
	zassert_str_equal(out, "Az");
	zassert_equal(nj_strcpy(nj_get(pool, root, "d"), out, sizeof(out)), 1);
	zassert_str_equal(out, "?");
}

ZTEST(nmos_json, test_strcpy_errors)
{
	const struct nj_node *root = parse_ok("{\"s\":\"abcdef\",\"i\":1}");
	char out[4];

	/* Too small for value + NUL. */
	zassert_equal(nj_strcpy(nj_get(pool, root, "s"), out, sizeof(out)),
		      -ENOMEM);
	/* Not a string / no node / zero-size buffer. */
	zassert_equal(nj_strcpy(nj_get(pool, root, "i"), out, sizeof(out)),
		      -EINVAL);
	zassert_equal(nj_strcpy(NULL, out, sizeof(out)), -EINVAL);
	zassert_equal(nj_strcpy(nj_get(pool, root, "s"), out, 0), -EINVAL);

	/* nj_streq() never matches a non-string node. */
	zassert_false(nj_streq(nj_get(pool, root, "i"), "1"));
	zassert_false(nj_streq(NULL, ""));
}

ZTEST(nmos_json, test_malformed_input)
{
	static const char *const bad[] = {
		"",                       /* empty */
		"{",                      /* unterminated object */
		"[1,2",                   /* unterminated array */
		"{\"a\":}",               /* missing value */
		"{\"a\" 1}",              /* missing colon */
		"{a:1}",                  /* unquoted key */
		"{\"a\":1,}",             /* trailing comma */
		"{\"a\":\"unterminated}", /* unterminated string */
		"tru",                    /* truncated literal */
		"nul",
		"{\"a\":1} trailing",     /* garbage after the root value */
		"[1,2][3]",
		"-",                      /* sign without digits */
		"@",                      /* not a JSON value at all */
	};

	ARRAY_FOR_EACH(bad, i) {
		int ret = nj_parse(bad[i], strlen(bad[i]), pool, POOL_SIZE);

		zassert_equal(ret, -EINVAL, "expected -EINVAL for \"%s\", got %d",
			      bad[i], ret);
	}
}

ZTEST(nmos_json, test_pool_exhaustion)
{
	const char *json = "[1,2,3,4,5,6,7,8]";
	struct nj_node small[4];

	zassert_equal(nj_parse(json, strlen(json), small, ARRAY_SIZE(small)),
		      -ENOMEM);

	/* The same body fits in a pool sized for array + elements. */
	struct nj_node exact[9];

	zassert_true(nj_parse(json, strlen(json), exact, ARRAY_SIZE(exact)) >= 0);
}

ZTEST(nmos_json, test_depth_limit)
{
	/* Guard against stack exhaustion from hostile nesting. */
	const char *deep = "[[[[[[[[[[[[1]]]]]]]]]]]]";

	zassert_equal(nj_parse(deep, strlen(deep), pool, POOL_SIZE), -EINVAL);

	/* Nesting the API actually uses stays well inside the limit. */
	const char *ok = "{\"a\":{\"b\":[{\"c\":1}]}}";

	zassert_true(nj_parse(ok, strlen(ok), pool, POOL_SIZE) >= 0);
}

ZTEST_SUITE(nmos_json, NULL, NULL, NULL, NULL, NULL);
