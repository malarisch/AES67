/*
 * Minimal JSON DOM parser for the NMOS Connection API.
 *
 * The IS-05 PATCH/POST bodies are small but genuinely nested (activation
 * objects, transport_params arrays, bulk arrays), which the flat
 * json_find_* helpers in json_util.c cannot represent. This parser
 * builds a node tree over the request buffer without copying: string
 * values stay as escaped spans into the body and are unescaped on
 * extraction (nj_strcpy).
 *
 * Only what the API needs: objects, arrays, strings, integers, bools,
 * null. Non-integer numbers are accepted syntactically but truncated.
 */

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "nmos_json.h"

struct nj_ctx {
	const char *p;
	const char *end;
	struct nj_node *pool;
	int pool_size;
	int used;
};

static void skip_ws(struct nj_ctx *c)
{
	while (c->p < c->end &&
	       (*c->p == ' ' || *c->p == '\t' || *c->p == '\r' || *c->p == '\n')) {
		c->p++;
	}
}

static int alloc_node(struct nj_ctx *c)
{
	if (c->used >= c->pool_size) {
		return -ENOMEM;
	}

	struct nj_node *n = &c->pool[c->used];

	memset(n, 0, sizeof(*n));
	n->first_child = -1;
	n->next = -1;
	return c->used++;
}

/* Scan a string body (after the opening quote); returns the span between
 * the quotes without unescaping, and leaves c->p after the closing quote. */
static int scan_string(struct nj_ctx *c, const char **out, uint32_t *out_len)
{
	const char *start = c->p;

	while (c->p < c->end) {
		if (*c->p == '\\') {
			c->p++;
			if (c->p >= c->end) {
				return -EINVAL;
			}
		} else if (*c->p == '"') {
			*out = start;
			*out_len = (uint32_t)(c->p - start);
			c->p++;
			return 0;
		}
		c->p++;
	}
	return -EINVAL;
}

static int parse_value(struct nj_ctx *c, int depth);

static int parse_members(struct nj_ctx *c, int parent, bool is_obj, int depth)
{
	char close = is_obj ? '}' : ']';
	int prev = -1;

	skip_ws(c);
	if (c->p < c->end && *c->p == close) {
		c->p++;
		return 0;
	}

	for (;;) {
		const char *key = NULL;
		uint32_t key_len = 0;

		skip_ws(c);
		if (is_obj) {
			if (c->p >= c->end || *c->p != '"') {
				return -EINVAL;
			}
			c->p++;
			if (scan_string(c, &key, &key_len) < 0) {
				return -EINVAL;
			}
			skip_ws(c);
			if (c->p >= c->end || *c->p != ':') {
				return -EINVAL;
			}
			c->p++;
		}

		int idx = parse_value(c, depth);

		if (idx < 0) {
			return idx;
		}
		c->pool[idx].key = key;
		c->pool[idx].key_len = (uint16_t)key_len;
		if (prev < 0) {
			c->pool[parent].first_child = (int16_t)idx;
		} else {
			c->pool[prev].next = (int16_t)idx;
		}
		prev = idx;

		skip_ws(c);
		if (c->p >= c->end) {
			return -EINVAL;
		}
		if (*c->p == ',') {
			c->p++;
			continue;
		}
		if (*c->p == close) {
			c->p++;
			return 0;
		}
		return -EINVAL;
	}
}

static int parse_value(struct nj_ctx *c, int depth)
{
	if (depth > 8) {
		return -EINVAL;
	}
	skip_ws(c);
	if (c->p >= c->end) {
		return -EINVAL;
	}

	int idx = alloc_node(c);

	if (idx < 0) {
		return idx;
	}

	struct nj_node *n = &c->pool[idx];
	char ch = *c->p;

	if (ch == '{' || ch == '[') {
		n->type = (ch == '{') ? NJ_OBJ : NJ_ARR;
		c->p++;
		int ret = parse_members(c, idx, ch == '{', depth + 1);

		return ret < 0 ? ret : idx;
	}
	if (ch == '"') {
		c->p++;
		n->type = NJ_STR;
		return scan_string(c, &n->val, &n->val_len) < 0 ? -EINVAL : idx;
	}
	if (ch == 't' || ch == 'f') {
		size_t len = (ch == 't') ? 4 : 5;

		if (c->end - c->p < (ptrdiff_t)len ||
		    memcmp(c->p, ch == 't' ? "true" : "false", len) != 0) {
			return -EINVAL;
		}
		n->type = NJ_BOOL;
		n->bval = (ch == 't');
		c->p += len;
		return idx;
	}
	if (ch == 'n') {
		if (c->end - c->p < 4 || memcmp(c->p, "null", 4) != 0) {
			return -EINVAL;
		}
		n->type = NJ_NULL;
		c->p += 4;
		return idx;
	}
	if (ch == '-' || (ch >= '0' && ch <= '9')) {
		bool neg = (ch == '-');
		int64_t v = 0;
		const char *start = c->p;

		if (neg) {
			c->p++;
		}
		while (c->p < c->end && *c->p >= '0' && *c->p <= '9') {
			if (v < INT32_MAX) {
				v = v * 10 + (*c->p - '0');
			}
			c->p++;
		}
		/* Accept (and truncate) fraction/exponent syntactically. */
		while (c->p < c->end &&
		       (*c->p == '.' || *c->p == 'e' || *c->p == 'E' ||
			*c->p == '+' || *c->p == '-' ||
			(*c->p >= '0' && *c->p <= '9'))) {
			c->p++;
		}
		if (c->p == start + (neg ? 1 : 0)) {
			return -EINVAL;
		}
		n->type = NJ_NUM;
		if (v > INT32_MAX) {
			v = INT32_MAX;
		}
		n->num = neg ? -(int32_t)v : (int32_t)v;
		n->val = start;
		n->val_len = (uint32_t)(c->p - start);
		return idx;
	}
	return -EINVAL;
}

int nj_parse(const char *buf, size_t len, struct nj_node *pool, int pool_size)
{
	struct nj_ctx c = {
		.p = buf,
		.end = buf + len,
		.pool = pool,
		.pool_size = pool_size,
	};

	int root = parse_value(&c, 0);

	if (root < 0) {
		return root;
	}
	skip_ws(&c);
	if (c.p != c.end) {
		return -EINVAL;
	}
	return root;
}

const struct nj_node *nj_get(const struct nj_node *pool,
			     const struct nj_node *obj, const char *key)
{
	size_t klen = strlen(key);

	if (obj == NULL || obj->type != NJ_OBJ) {
		return NULL;
	}
	for (int i = obj->first_child; i >= 0; i = pool[i].next) {
		if (pool[i].key_len == klen &&
		    memcmp(pool[i].key, key, klen) == 0) {
			return &pool[i];
		}
	}
	return NULL;
}

const struct nj_node *nj_item(const struct nj_node *pool,
			      const struct nj_node *arr, int idx)
{
	if (arr == NULL || arr->type != NJ_ARR) {
		return NULL;
	}
	for (int i = arr->first_child; i >= 0; i = pool[i].next) {
		if (idx-- == 0) {
			return &pool[i];
		}
	}
	return NULL;
}

int nj_count(const struct nj_node *pool, const struct nj_node *n)
{
	int cnt = 0;

	if (n == NULL || (n->type != NJ_ARR && n->type != NJ_OBJ)) {
		return 0;
	}
	for (int i = n->first_child; i >= 0; i = pool[i].next) {
		cnt++;
	}
	return cnt;
}

int nj_strcpy(const struct nj_node *n, char *out, size_t sz)
{
	static const char hex[] = "0123456789abcdef";
	size_t o = 0;

	if (n == NULL || n->type != NJ_STR || sz == 0) {
		return -EINVAL;
	}
	for (uint32_t i = 0; i < n->val_len; i++) {
		char ch = n->val[i];

		if (ch == '\\' && i + 1 < n->val_len) {
			char esc = n->val[++i];

			switch (esc) {
			case 'n':
				ch = '\n';
				break;
			case 'r':
				ch = '\r';
				break;
			case 't':
				ch = '\t';
				break;
			case 'b':
				ch = '\b';
				break;
			case 'f':
				ch = '\f';
				break;
			case 'u':
				/* Only Basic-Latin escapes are mapped; anything
				 * else becomes '?' (never appears in SDP). */
				if (i + 4 < n->val_len) {
					const char *h1 = memchr(hex, n->val[i + 3] | 0x20, 16);
					const char *h2 = memchr(hex, n->val[i + 4] | 0x20, 16);

					if (n->val[i + 1] == '0' && n->val[i + 2] == '0' &&
					    h1 != NULL && h2 != NULL) {
						ch = (char)(((h1 - hex) << 4) | (h2 - hex));
					} else {
						ch = '?';
					}
					i += 4;
				} else {
					return -EINVAL;
				}
				break;
			default:
				ch = esc; /* '"', '\\', '/' */
				break;
			}
		}
		if (o + 1 >= sz) {
			return -ENOMEM;
		}
		out[o++] = ch;
	}
	out[o] = '\0';
	return (int)o;
}

bool nj_streq(const struct nj_node *n, const char *s)
{
	size_t len = strlen(s);

	return n != NULL && n->type == NJ_STR && n->val_len == len &&
	       memcmp(n->val, s, len) == 0;
}
