/*
 * NMOS IS-04 Node API — HTTP surface.
 *
 * One dynamic resource next to the REST API's "/api" wildcard (own
 * holder, so NMOS traffic does not contend with dashboard polling): the
 * whole /x-nmos/ subtree including the sender SDP manifests. The IS-04
 * interop rules the nmos-testing tool enforces live here:
 *  - CORS headers on every response, including errors,
 *  - error bodies {"code","error","debug"} for every status >= 400,
 *  - Content-Type exactly application/json (application/sdp for
 *    manifests), no parameters,
 *  - both trailing-slash and bare forms accepted for GET,
 *  - base-path listings with trailing-slash entries.
 *
 * /receivers/{id}/target answers 501: connection management is IS-05's
 * job (allowed from v1.3).
 */

#include <zephyr/kernel.h>
#include <zephyr/net/http/server.h>
#include <zephyr/net/http/service.h>
#include <zephyr/logging/log.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include "nmos.h"
#include "nmos_priv.h"

LOG_MODULE_REGISTER(nmos_api, LOG_LEVEL_INF);

/* Own response buffer: full resource lists (8 senders/receivers) exceed
 * the REST API's shared 4 KB buffer. */
#define NMOS_BUF_SIZE 8192
static char nmos_buf[NMOS_BUF_SIZE];

/* ================================================================
 * Collections
 * ================================================================ */

enum nmos_coll {
	COLL_DEVICES,
	COLL_SOURCES,
	COLL_FLOWS,
	COLL_SENDERS,
	COLL_RECEIVERS,
};

static bool coll_from_seg(const char *seg, enum nmos_coll *coll)
{
	static const char *const names[] = {
		[COLL_DEVICES] = "devices",
		[COLL_SOURCES] = "sources",
		[COLL_FLOWS] = "flows",
		[COLL_SENDERS] = "senders",
		[COLL_RECEIVERS] = "receivers",
	};

	for (size_t i = 0; i < ARRAY_SIZE(names); i++) {
		if (strcmp(seg, names[i]) == 0) {
			*coll = (enum nmos_coll)i;
			return true;
		}
	}
	return false;
}

static enum nmos_res_kind coll_kind(enum nmos_coll coll)
{
	switch (coll) {
	case COLL_DEVICES:
		return NMOS_RES_DEVICE;
	case COLL_SOURCES:
		return NMOS_RES_SOURCE;
	case COLL_FLOWS:
		return NMOS_RES_FLOW;
	case COLL_SENDERS:
		return NMOS_RES_SENDER;
	case COLL_RECEIVERS:
	default:
		return NMOS_RES_RECEIVER;
	}
}

static int coll_max(enum nmos_coll coll)
{
	switch (coll) {
	case COLL_DEVICES:
		return 1;
	case COLL_RECEIVERS:
		return nmos_rx_count();
	default:
		return nmos_tx_count();
	}
}

/* Emits the resource object, or nothing if the slot is inactive. */
static bool coll_build_one(struct json_out *jo, enum nmos_coll coll, int idx)
{
	switch (coll) {
	case COLL_DEVICES:
		nmos_build_device(jo);
		return true;
	case COLL_SOURCES:
		return nmos_build_source(jo, idx);
	case COLL_FLOWS:
		return nmos_build_flow(jo, idx);
	case COLL_SENDERS:
		return nmos_build_sender(jo, idx);
	case COLL_RECEIVERS:
	default:
		return nmos_build_receiver(jo, idx);
	}
}

/* Find the slot whose UUID matches; -1 if none. */
static int coll_find_uuid(enum nmos_coll coll, const char *want)
{
	char uuid[NMOS_UUID_STR_LEN];

	for (int i = 0; i < coll_max(coll); i++) {
		nmos_uuid(coll_kind(coll), i, uuid);
		if (strcasecmp(uuid, want) == 0) {
			return i;
		}
	}
	return -1;
}

/* ================================================================
 * Response helpers
 * ================================================================ */

static const struct http_header json_hdrs[] = {
	{.name = "Content-Type", .value = "application/json"},
	{.name = "Access-Control-Allow-Origin", .value = "*"},
};

static const struct http_header sdp_hdrs[] = {
	{.name = "Content-Type", .value = "application/sdp"},
	{.name = "Access-Control-Allow-Origin", .value = "*"},
};

static void finish_rsp(struct http_response_ctx *rsp, uint16_t status,
		       size_t body_len, bool sdp)
{
	rsp->status = status;
	rsp->body = (const uint8_t *)nmos_buf;
	rsp->body_len = body_len;
	rsp->final_chunk = true;
	rsp->headers = sdp ? sdp_hdrs : json_hdrs;
	rsp->header_count = sdp ? ARRAY_SIZE(sdp_hdrs) : ARRAY_SIZE(json_hdrs);
}

/* IS-04 error body: all three keys required, code matches the status. */
static void error_rsp(struct http_response_ctx *rsp, uint16_t status,
		      const char *msg)
{
	struct json_out jo;

	jo_init(&jo, nmos_buf, sizeof(nmos_buf));
	jo_obj_begin(&jo);
	jo_uint(&jo, "code", status);
	jo_str(&jo, "error", msg);
	jo_key(&jo, "debug");
	jo_raw(&jo, "null,");
	jo_obj_end(&jo);
	finish_rsp(rsp, status, jo_finish(&jo), false);
}

static void listing_rsp(struct http_response_ctx *rsp,
			const char *const *items, size_t n)
{
	struct json_out jo;

	jo_init(&jo, nmos_buf, sizeof(nmos_buf));
	jo_arr_begin(&jo);
	for (size_t i = 0; i < n; i++) {
		jo_fmt(&jo, "\"%s\",", items[i]);
	}
	jo_arr_end(&jo);
	finish_rsp(rsp, HTTP_200_OK, jo_finish(&jo), false);
}

/* ================================================================
 * Dispatch
 * ================================================================ */

static void nmos_dispatch(enum http_method method, const char *url,
			  struct http_response_ctx *rsp)
{
	char path[160];
	char *seg[7];
	int nseg = 0;
	struct json_out jo;
	enum nmos_coll coll;
	bool is_get = (method == HTTP_GET || method == HTTP_HEAD);

	/* Strip query string, then tokenize; a trailing slash simply
	 * yields no extra segment (GET must accept both forms). */
	size_t plen = strcspn(url, "?");

	if (plen >= sizeof(path)) {
		error_rsp(rsp, HTTP_404_NOT_FOUND, "path too long");
		return;
	}
	memcpy(path, url, plen);
	path[plen] = '\0';

	char *p = path;

	while (*p == '/') {
		p++;
	}
	while (*p != '\0' && nseg < (int)ARRAY_SIZE(seg)) {
		seg[nseg++] = p;
		char *slash = strchr(p, '/');

		if (slash == NULL) {
			break;
		}
		*slash = '\0';
		p = slash + 1;
	}

	if (nseg < 1 || nseg >= (int)ARRAY_SIZE(seg) ||
	    strcmp(seg[0], "x-nmos") != 0) {
		error_rsp(rsp, HTTP_404_NOT_FOUND, "no such resource");
		return;
	}

	/* ---- /x-nmos ---- */
	if (nseg == 1) {
		static const char *const apis[] = {"manifest/", "node/"};

		if (!is_get) {
			goto method_not_allowed;
		}
		listing_rsp(rsp, apis, ARRAY_SIZE(apis));
		return;
	}

	/* ---- /x-nmos/manifest/{senderId} ---- */
	if (strcmp(seg[1], "manifest") == 0) {
		if (nseg != 3) {
			goto not_found;
		}
		if (!is_get) {
			goto method_not_allowed;
		}

		int idx = coll_find_uuid(COLL_SENDERS, seg[2]);

		if (idx < 0) {
			goto not_found;
		}

		int len = nmos_build_manifest(nmos_buf, sizeof(nmos_buf), idx);

		if (len < 0) {
			goto not_found;
		}
		finish_rsp(rsp, HTTP_200_OK, (size_t)len, true);
		return;
	}

	if (strcmp(seg[1], "node") != 0) {
		goto not_found;
	}

	/* ---- /x-nmos/node ---- */
	if (nseg == 2) {
		static const char *const versions[] = {NMOS_API_VERSION "/"};

		if (!is_get) {
			goto method_not_allowed;
		}
		listing_rsp(rsp, versions, ARRAY_SIZE(versions));
		return;
	}

	if (strcmp(seg[2], NMOS_API_VERSION) != 0) {
		goto not_found;
	}

	/* ---- /x-nmos/node/v1.3 ---- */
	if (nseg == 3) {
		static const char *const base[] = {
			"self/", "sources/", "flows/",
			"devices/", "senders/", "receivers/",
		};

		if (!is_get) {
			goto method_not_allowed;
		}
		listing_rsp(rsp, base, ARRAY_SIZE(base));
		return;
	}

	/* ---- /x-nmos/node/v1.3/self ---- */
	if (nseg == 4 && strcmp(seg[3], "self") == 0) {
		if (!is_get) {
			goto method_not_allowed;
		}
		jo_init(&jo, nmos_buf, sizeof(nmos_buf));
		nmos_build_self(&jo);
		goto finish_json;
	}

	if (!coll_from_seg(seg[3], &coll)) {
		goto not_found;
	}

	/* ---- /x-nmos/node/v1.3/{collection} ---- */
	if (nseg == 4) {
		if (!is_get) {
			goto method_not_allowed;
		}
		jo_init(&jo, nmos_buf, sizeof(nmos_buf));
		jo_arr_begin(&jo);
		for (int i = 0; i < coll_max(coll); i++) {
			(void)coll_build_one(&jo, coll, i);
		}
		jo_arr_end(&jo);
		goto finish_json;
	}

	/* ---- /x-nmos/node/v1.3/{collection}/{id} ---- */
	if (nseg == 5) {
		if (!is_get) {
			goto method_not_allowed;
		}

		int idx = coll_find_uuid(coll, seg[4]);

		if (idx < 0) {
			goto not_found;
		}
		jo_init(&jo, nmos_buf, sizeof(nmos_buf));
		if (!coll_build_one(&jo, coll, idx)) {
			goto not_found;
		}
		goto finish_json;
	}

	/* ---- /x-nmos/node/v1.3/receivers/{id}/target ---- */
	if (nseg == 6 && coll == COLL_RECEIVERS &&
	    strcmp(seg[5], "target") == 0) {
		if (coll_find_uuid(COLL_RECEIVERS, seg[4]) < 0) {
			goto not_found;
		}
		if (method != HTTP_PUT) {
			goto method_not_allowed;
		}
		error_rsp(rsp, HTTP_501_NOT_IMPLEMENTED,
			  "use the IS-05 Connection API");
		return;
	}

not_found:
	error_rsp(rsp, HTTP_404_NOT_FOUND, "no such resource");
	return;

method_not_allowed:
	error_rsp(rsp, HTTP_405_METHOD_NOT_ALLOWED, "method not allowed");
	return;

finish_json:
	if (jo.overflow) {
		error_rsp(rsp, HTTP_500_INTERNAL_SERVER_ERROR,
			  "response too large");
		return;
	}
	finish_rsp(rsp, HTTP_200_OK, jo_finish(&jo), false);

	if (method == HTTP_HEAD) {
		rsp->body_len = 0;
	}
}

/* ================================================================
 * Dynamic resource handler
 * ================================================================ */

static int nmos_handler(struct http_client_ctx *client,
			enum http_transaction_status status,
			const struct http_request_ctx *request_ctx,
			struct http_response_ctx *response_ctx,
			void *user_data)
{
	const char *url = (const char *)client->url_buffer;
	enum http_method method = client->method;

	ARG_UNUSED(user_data);

	if (status == HTTP_SERVER_TRANSACTION_ABORTED ||
	    status == HTTP_SERVER_TRANSACTION_COMPLETE) {
		return 0;
	}

	if (method == HTTP_OPTIONS) {
		/* CORS preflight. Allow-Methods must cover every method the
		 * spec declares for the path (only /target has PUT). */
		static const struct http_header get_hdrs[] = {
			{.name = "Access-Control-Allow-Origin", .value = "*"},
			{.name = "Access-Control-Allow-Methods",
			 .value = "GET, HEAD, OPTIONS"},
			{.name = "Access-Control-Allow-Headers",
			 .value = "Content-Type"},
		};
		static const struct http_header put_hdrs[] = {
			{.name = "Access-Control-Allow-Origin", .value = "*"},
			{.name = "Access-Control-Allow-Methods",
			 .value = "PUT, OPTIONS"},
			{.name = "Access-Control-Allow-Headers",
			 .value = "Content-Type"},
		};
		size_t plen = strcspn(url, "?");

		while (plen > 1 && url[plen - 1] == '/') {
			plen--;
		}

		bool is_target = plen >= 7 &&
				 strncmp(url + plen - 7, "/target", 7) == 0;

		response_ctx->status = HTTP_200_OK;
		response_ctx->body = (const uint8_t *)"";
		response_ctx->body_len = 0;
		response_ctx->final_chunk = true;
		response_ctx->headers = is_target ? put_hdrs : get_hdrs;
		response_ctx->header_count =
			is_target ? ARRAY_SIZE(put_hdrs) : ARRAY_SIZE(get_hdrs);
		return 0;
	}

	/* Request bodies (the /target PUT) are consumed but not parsed —
	 * the endpoint answers 501 regardless of payload. */
	bool has_body = method == HTTP_POST || method == HTTP_PUT ||
			method == HTTP_PATCH || method == HTTP_DELETE;

	if (has_body && status != HTTP_SERVER_REQUEST_DATA_FINAL) {
		response_ctx->final_chunk = false;
		return 0;
	}

	ARG_UNUSED(request_ctx);

	nmos_dispatch(method, url, response_ctx);
	return 0;
}

static struct http_resource_detail_dynamic nmos_resource_detail = {
	.common = {
		.type = HTTP_RESOURCE_TYPE_DYNAMIC,
		.bitmask_of_supported_http_methods =
			BIT(HTTP_GET) | BIT(HTTP_HEAD) | BIT(HTTP_POST) |
			BIT(HTTP_PUT) | BIT(HTTP_PATCH) | BIT(HTTP_DELETE) |
			BIT(HTTP_OPTIONS),
		.content_type = "application/json",
	},
	.cb = nmos_handler,
	.user_data = NULL,
};

/* The wildcard does not match the bare base path, and GET /x-nmos must
 * serve the API listing — hence the extra exact-match resource. */
HTTP_RESOURCE_DEFINE(nmos_base_resource, aes67_http, "/x-nmos",
		     &nmos_resource_detail);
HTTP_RESOURCE_DEFINE(nmos_resource, aes67_http, "/x-nmos/*",
		     &nmos_resource_detail);
