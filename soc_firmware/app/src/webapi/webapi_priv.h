/*
 *
 * Internal interfaces of the modular REST API.
 *
 * Each resource lives in its own webapi_<resource>.c and exports a route
 * table; webapi_core.c owns the HTTP service, matches URL + method against
 * the registered tables and turns handler results into HTTP responses.
 */

#ifndef WEBAPI_PRIV_H_
#define WEBAPI_PRIV_H_

#include <zephyr/net/http/server.h>
#include <zephyr/net/http/method.h>
#include <zephyr/data/json.h>
#include <zephyr/sys/util.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
 * JSON output writer
 *
 * Appends into a fixed buffer; every value helper leaves a trailing
 * comma which jo_obj_end()/jo_arr_end()/jo_finish() strip, so builders
 * nest without offset arithmetic. Overflow is sticky: once the buffer
 * is full nothing more is written and the request fails with 500
 * instead of emitting truncated (invalid) JSON.
 * ================================================================ */

struct json_out {
	char *buf;
	size_t size;
	size_t pos;
	bool overflow;
};

void jo_init(struct json_out *jo, char *buf, size_t size);
void jo_raw(struct json_out *jo, const char *s);
void jo_fmt(struct json_out *jo, const char *fmt, ...);
void jo_obj_begin(struct json_out *jo);
void jo_obj_end(struct json_out *jo);
void jo_arr_begin(struct json_out *jo);
void jo_arr_end(struct json_out *jo);
void jo_key(struct json_out *jo, const char *key);
/* String values are escaped (quotes, backslash, control chars). */
void jo_str(struct json_out *jo, const char *key, const char *val);
void jo_int(struct json_out *jo, const char *key, int32_t val);
void jo_uint(struct json_out *jo, const char *key, uint32_t val);
void jo_bool(struct json_out *jo, const char *key, bool val);
/* Strip the top-level trailing comma; returns body length. */
size_t jo_finish(struct json_out *jo);

/* ================================================================
 * Routing
 * ================================================================ */

struct webapi_request {
	/* Request body (POST/PUT/PATCH/DELETE), fully accumulated and
	 * NUL-terminated. Writable: Zephyr's JSON parser terminates
	 * tokens in place while decoding them. */
	char *body;
	size_t body_len;
	/* Value of the "{id}" path segment, -1 if the route has none. */
	int id;
	/* Response body writer. A handler that returns 0 without writing
	 * anything gets a default {"ok":true} body. */
	struct json_out out;
	/* Optional HTTP status override; 0 = derive from handler result. */
	uint16_t status;
};

/* Returns 0 on success or a negative errno (mapped to 4xx/5xx + error JSON
 * by the core; -ENOENT/-EINVAL -> 400, -ENODEV -> 503, -ENOTSUP -> 501). */
typedef int (*webapi_handler_t)(struct webapi_request *req);

struct webapi_route {
	uint8_t method; /* enum http_method */
	/* Literal path, or one "{id}" segment matching a decimal integer,
	 * e.g. "/api/streams/tx/{id}". */
	const char *path;
	webapi_handler_t handler;
};

#define WEBAPI_ROUTE(_m, _p, _h) { .method = (_m), .path = (_p), .handler = (_h) }

/* ================================================================
 * Request body parsing
 *
 * Bodies are described as a struct plus a json_obj_descr table and
 * decoded by Zephyr's JSON library. Pre-fill the struct with the
 * current values before calling: the parser only writes the keys the
 * body actually carries, so anything omitted keeps its current value —
 * which is exactly PATCH semantics, with no per-field presence check.
 *
 * Where presence itself is the decision (a required field, or "absent
 * means do something else"), use the returned bitmap: bit i is set when
 * descr[i] was present. Index it through an enum declared alongside the
 * descriptor table so the two cannot drift apart.
 * ================================================================ */

/** Returns the bitmap of decoded fields, or a negative errno if the
 *  body is missing or not valid JSON. */
int64_t webapi_parse_body(struct webapi_request *req,
			  const struct json_obj_descr *descr, size_t descr_len,
			  void *val);

/** True if field @p idx_ (an index into the descriptor table) was
 *  present in the body. */
#define WEBAPI_HAS(bitmap_, idx_) (((bitmap_) & BIT64(idx_)) != 0)

/* Route tables exported by the resource modules. Terminated by count. */
struct webapi_module {
	const struct webapi_route *routes;
	size_t count;
};

extern const struct webapi_module webapi_network_module;
extern const struct webapi_module webapi_fpga_module;
extern const struct webapi_module webapi_ptp_module;
extern const struct webapi_module webapi_streams_module;
extern const struct webapi_module webapi_config_module;
extern const struct webapi_module webapi_system_module;
extern const struct webapi_module webapi_cards_module;
#ifdef CONFIG_DISPLAY_CTRL
extern const struct webapi_module webapi_display_module;
#endif

/* ================================================================
 * Cross-module helpers
 * ================================================================ */

/* Object builders reused by /api/summary (each emits one {...} object,
 * usable top-level or nested behind jo_key()). */
void webapi_build_network(struct json_out *jo);
void webapi_build_fpga(struct json_out *jo);
void webapi_build_ptp(struct json_out *jo);

/* FPGA metric snapshot incl. the SW-PTP substitution of the servo
 * readouts (webapi_fpga.c). */
struct ui_fpga_metrics;
int webapi_read_fpga_metrics(struct ui_fpga_metrics *m);

/* Persist the device configuration to whatever backend exists
 * (SD card and/or flash). No-op when neither is configured. */
void webapi_persist_config(void);

#ifdef __cplusplus
}
#endif

#endif /* WEBAPI_PRIV_H_ */
