/*
 * JSON output writer + request-body entry point for the REST API.
 *
 * The writer streams into a caller-provided buffer without a heap: the
 * REST and NMOS documents are assembled at runtime (variable numbers of
 * streams, cards and resources), which a descriptor-driven encoder
 * cannot express. Reading goes the other way — request bodies have a
 * fixed shape per endpoint, so they are decoded by Zephyr's JSON
 * library through webapi_parse_body().
 */

#include <errno.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "webapi_priv.h"

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
 * Request body parsing
 *
 * One entry point for every resource: Zephyr's JSON library decodes the
 * body into the caller's struct according to its descriptor table, and
 * reports which keys were actually present.
 * ================================================================ */

int64_t webapi_parse_body(struct webapi_request *req,
			  const struct json_obj_descr *descr, size_t descr_len,
			  void *val)
{
	if (req->body == NULL || req->body_len == 0) {
		return -EINVAL;
	}

	return json_obj_parse(req->body, req->body_len, descr, descr_len, val);
}
