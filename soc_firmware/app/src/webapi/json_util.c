/*
 * JSON output writer + minimal body parser for the REST API.
 * No heap, no library: writes into a caller-provided buffer, extracts
 * scalar values by key from flat request bodies.
 */

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "webapi_priv.h"

/* memmem is a GNU extension not available in picolibc / Zephyr */
static const void *local_memmem(const void *haystack, size_t haystacklen,
				const void *needle, size_t needlelen)
{
	const uint8_t *h = haystack;
	const uint8_t *n = needle;

	if (needlelen == 0) {
		return haystack;
	}
	if (haystacklen < needlelen) {
		return NULL;
	}
	for (size_t i = 0; i <= haystacklen - needlelen; i++) {
		if (h[i] == n[0] && memcmp(&h[i], n, needlelen) == 0) {
			return &h[i];
		}
	}
	return NULL;
}

/* ================================================================
 * Writer
 * ================================================================ */

void jo_init(struct json_out *jo, char *buf, size_t size)
{
	jo->buf = buf;
	jo->size = size;
	jo->pos = 0;
	jo->overflow = false;
	if (size > 0) {
		buf[0] = '\0';
	} else {
		jo->overflow = true;
	}
}

static void jo_putc(struct json_out *jo, char c)
{
	if (jo->overflow || jo->pos + 1 >= jo->size) {
		jo->overflow = true;
		return;
	}
	jo->buf[jo->pos++] = c;
	jo->buf[jo->pos] = '\0';
}

void jo_raw(struct json_out *jo, const char *s)
{
	size_t len = strlen(s);

	if (jo->overflow || jo->pos + len >= jo->size) {
		jo->overflow = true;
		return;
	}
	memcpy(jo->buf + jo->pos, s, len + 1);
	jo->pos += len;
}

void jo_fmt(struct json_out *jo, const char *fmt, ...)
{
	va_list ap;
	int n;

	if (jo->overflow) {
		return;
	}

	va_start(ap, fmt);
	n = vsnprintf(jo->buf + jo->pos, jo->size - jo->pos, fmt, ap);
	va_end(ap);

	if (n < 0 || (size_t)n >= jo->size - jo->pos) {
		jo->overflow = true;
		/* keep the buffer a valid C string at the old position */
		jo->buf[jo->pos] = '\0';
		return;
	}
	jo->pos += n;
}

static void jo_strip_comma(struct json_out *jo)
{
	if (!jo->overflow && jo->pos > 0 && jo->buf[jo->pos - 1] == ',') {
		jo->buf[--jo->pos] = '\0';
	}
}

void jo_obj_begin(struct json_out *jo)
{
	jo_putc(jo, '{');
}

void jo_obj_end(struct json_out *jo)
{
	jo_strip_comma(jo);
	jo_raw(jo, "},");
}

void jo_arr_begin(struct json_out *jo)
{
	jo_putc(jo, '[');
}

void jo_arr_end(struct json_out *jo)
{
	jo_strip_comma(jo);
	jo_raw(jo, "],");
}

void jo_key(struct json_out *jo, const char *key)
{
	jo_fmt(jo, "\"%s\":", key);
}

static void jo_quoted(struct json_out *jo, const char *val)
{
	jo_putc(jo, '"');
	for (const char *c = val; *c != '\0'; c++) {
		switch (*c) {
		case '"':
			jo_raw(jo, "\\\"");
			break;
		case '\\':
			jo_raw(jo, "\\\\");
			break;
		case '\n':
			jo_raw(jo, "\\n");
			break;
		case '\r':
			jo_raw(jo, "\\r");
			break;
		case '\t':
			jo_raw(jo, "\\t");
			break;
		default:
			if ((uint8_t)*c < 0x20) {
				jo_fmt(jo, "\\u%04x", (uint8_t)*c);
			} else {
				jo_putc(jo, *c);
			}
		}
	}
	jo_raw(jo, "\",");
}

void jo_str(struct json_out *jo, const char *key, const char *val)
{
	jo_key(jo, key);
	jo_quoted(jo, val);
}

void jo_int(struct json_out *jo, const char *key, int32_t val)
{
	jo_fmt(jo, "\"%s\":%d,", key, val);
}

void jo_uint(struct json_out *jo, const char *key, uint32_t val)
{
	jo_fmt(jo, "\"%s\":%u,", key, val);
}

void jo_bool(struct json_out *jo, const char *key, bool val)
{
	jo_fmt(jo, "\"%s\":%s,", key, val ? "true" : "false");
}

size_t jo_finish(struct json_out *jo)
{
	jo_strip_comma(jo);
	return jo->pos;
}

/* ================================================================
 * Parser
 * ================================================================ */

static const char *find_value(const char *json, size_t json_len,
			      const char *key, const char *suffix)
{
	char needle[64];
	int nlen = snprintf(needle, sizeof(needle), "\"%s\":%s", key, suffix);

	if (nlen <= 0 || (size_t)nlen >= sizeof(needle)) {
		return NULL;
	}

	const char *start = local_memmem(json, json_len, needle, nlen);

	return start != NULL ? start + nlen : NULL;
}

int json_find_str(const char *json, size_t json_len, const char *key,
		  char *out, size_t out_sz)
{
	const char *start = find_value(json, json_len, key, "\"");

	if (start == NULL || out_sz == 0) {
		return 0;
	}

	const char *end = memchr(start, '"', json_len - (start - json));

	if (end == NULL) {
		return 0;
	}

	size_t vlen = end - start;

	if (vlen >= out_sz) {
		vlen = out_sz - 1;
	}
	memcpy(out, start, vlen);
	out[vlen] = '\0';
	return (int)vlen;
}

bool json_find_int(const char *json, size_t json_len, const char *key,
		   int32_t *out)
{
	const char *start = find_value(json, json_len, key, "");

	if (start == NULL) {
		return false;
	}

	while (start < json + json_len && *start == ' ') {
		start++;
	}

	char *endptr;
	long val = strtol(start, &endptr, 10);

	if (endptr == start) {
		return false;
	}
	*out = (int32_t)val;
	return true;
}

bool json_find_bool(const char *json, size_t json_len, const char *key,
		    bool *out)
{
	const char *start = find_value(json, json_len, key, "");

	if (start == NULL) {
		return false;
	}

	while (start < json + json_len && *start == ' ') {
		start++;
	}

	if (start + 4 <= json + json_len && memcmp(start, "true", 4) == 0) {
		*out = true;
		return true;
	}
	if (start + 5 <= json + json_len && memcmp(start, "false", 5) == 0) {
		*out = false;
		return true;
	}
	return false;
}

int json_find_u8_array(const char *json, size_t json_len, const char *key,
		       uint8_t *out, int max)
{
	const char *p = find_value(json, json_len, key, "[");
	const char *end = json + json_len;
	int count = 0;

	if (p == NULL) {
		return 0;
	}

	while (count < max && p < end && *p != ']') {
		while (p < end && (*p == ' ' || *p == ',')) {
			p++;
		}
		if (p >= end || *p == ']') {
			break;
		}

		char *endptr;
		long val = strtol(p, &endptr, 10);

		if (endptr == p) {
			break;
		}
		out[count++] = (uint8_t)val;
		p = endptr;
	}
	return count;
}
