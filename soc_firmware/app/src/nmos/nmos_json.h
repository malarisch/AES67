/*
 * Minimal JSON DOM parser for the NMOS Connection API (see nmos_json.c).
 */

#ifndef NMOS_JSON_H_
#define NMOS_JSON_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum nj_type {
	NJ_NULL = 0,
	NJ_BOOL,
	NJ_NUM,
	NJ_STR,
	NJ_OBJ,
	NJ_ARR,
};

struct nj_node {
	uint8_t type;         /* enum nj_type */
	bool bval;            /* NJ_BOOL */
	int32_t num;          /* NJ_NUM (integer part, saturated) */
	const char *key;      /* member key span (raw, unescaped), NULL if none */
	uint16_t key_len;
	const char *val;      /* NJ_STR: span between quotes (still escaped) */
	uint32_t val_len;
	int16_t first_child;  /* NJ_OBJ/NJ_ARR: pool index, -1 if empty */
	int16_t next;         /* next sibling pool index, -1 at end */
};

/* Parse buf into pool; returns the root node's pool index (>= 0) or
 * -EINVAL on malformed input / -ENOMEM if the pool is too small.
 * The tree references buf — keep it alive while the tree is used. */
int nj_parse(const char *buf, size_t len, struct nj_node *pool, int pool_size);

/* Object member by key (NULL if absent or obj is not an object). */
const struct nj_node *nj_get(const struct nj_node *pool,
			     const struct nj_node *obj, const char *key);

/* idx-th array element (NULL if out of range or not an array). */
const struct nj_node *nj_item(const struct nj_node *pool,
			      const struct nj_node *arr, int idx);

/* Number of children of an array/object node. */
int nj_count(const struct nj_node *pool, const struct nj_node *n);

/* Copy + unescape a string value; returns length or negative errno. */
int nj_strcpy(const struct nj_node *n, char *out, size_t sz);

/* String value equals s (compares the raw span — fine for enum-like
 * values that never contain escapes, e.g. "auto", mode names). */
bool nj_streq(const struct nj_node *n, const char *s);

#ifdef __cplusplus
}
#endif

#endif /* NMOS_JSON_H_ */
