/*
 *
 * REST API core: HTTP service, request routing and response handling.
 *
 * All /api/... requests share one Zephyr dynamic resource; the core
 * accumulates the request body, matches URL + method against the route
 * tables the resource modules export (webapi_priv.h) and renders handler
 * results as JSON responses. The static web UI and the firmware-update
 * endpoint keep their own resources.
 */

#include <zephyr/kernel.h>
#include <zephyr/net/http/server.h>
#include <zephyr/net/http/service.h>
#include <zephyr/logging/log.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include "../webserver.h"
#include "../aes67_config.h"
#include "webapi_priv.h"

#ifdef CONFIG_SD_CONFIG
#include "../sd_config.h"
#endif
#ifdef CONFIG_FLASH_CONFIG
#include "../flash_config.h"
#endif
#ifdef CONFIG_SPI_FLASH_LITESPI
#include "../fw_update.h"
#endif

LOG_MODULE_REGISTER(webapi, LOG_LEVEL_INF);

/* ================================================================
 * HTTP service definition — port 80.
 *
 * Concurrency follows CONFIG_HTTP_SERVER_MAX_CLIENTS (the size of the
 * server's client table). Browsers park up to 6 idle keep-alive
 * connections, and each one occupies a client slot until the 30 s
 * inactivity timeout; once all slots are taken the server stops polling
 * the listen socket entirely — new connections then hang in the backlog
 * or get reset. With the old hard-coded 4 that showed up as multi-second
 * unresponsiveness and connection resets during normal dashboard use.
 * ================================================================ */

static uint16_t http_port = 80;

HTTP_SERVICE_DEFINE(aes67_http, "0.0.0.0", &http_port,
		    CONFIG_HTTP_SERVER_MAX_CLIENTS, 10, NULL, NULL, NULL);

/* Shared response/body buffers: dynamic resources are serialised per
 * resource holder, so one request at a time uses them. */
#define JSON_BUF_SIZE 4096
static char json_buf[JSON_BUF_SIZE];

#define BODY_BUF_SIZE 4096
static char body_buf[BODY_BUF_SIZE];
static size_t body_len;

/* ================================================================
 * Route matching
 * ================================================================ */

static const struct webapi_module *const modules[] = {
	&webapi_system_module,
	&webapi_network_module,
	&webapi_fpga_module,
	&webapi_ptp_module,
	&webapi_streams_module,
	&webapi_config_module,
	&webapi_cards_module,
#ifdef CONFIG_DISPLAY_CTRL
	&webapi_display_module,
#endif
};

/* Match a route pattern segment-wise; "{id}" consumes one decimal-integer
 * segment into *id. Returns true on full match. */
static bool route_match(const char *pattern, const char *path, int *id)
{
	*id = -1;

	while (*pattern != '\0' && *path != '\0') {
		if (strncmp(pattern, "{id}", 4) == 0) {
			char *end;
			long val = strtol(path, &end, 10);

			if (end == path || val < 0 ||
			    (*end != '\0' && *end != '/')) {
				return false;
			}
			*id = (int)val;
			pattern += 4;
			path = end;
			continue;
		}
		if (*pattern != *path) {
			return false;
		}
		pattern++;
		path++;
	}
	return *pattern == '\0' && *path == '\0';
}

static const struct webapi_route *find_route(enum http_method method,
					     const char *path, int *id,
					     bool *path_exists)
{
	*path_exists = false;

	for (size_t m = 0; m < ARRAY_SIZE(modules); m++) {
		const struct webapi_module *mod = modules[m];

		for (size_t r = 0; r < mod->count; r++) {
			const struct webapi_route *route = &mod->routes[r];

			if (!route_match(route->path, path, id)) {
				continue;
			}
			*path_exists = true;
			if (route->method == method) {
				return route;
			}
		}
	}
	return NULL;
}

/* ================================================================
 * Response rendering
 * ================================================================ */

static const char *errno_message(int err)
{
	switch (err) {
	case -EINVAL:
		return "invalid request";
	case -ENOENT:
		return "not found";
	case -ENODEV:
		return "device not available";
	case -ENOTSUP:
		return "not supported";
	case -ENOMEM:
		return "out of resources";
	case -EBUSY:
		return "busy";
	default:
		return "internal error";
	}
}

static uint16_t errno_status(int err)
{
	switch (err) {
	case -EINVAL:
	case -ENOENT:
		return HTTP_400_BAD_REQUEST;
	case -ENODEV:
		return HTTP_503_SERVICE_UNAVAILABLE;
	case -ENOTSUP:
	case -ENOMEM:
	case -EBUSY:
	default:
		return HTTP_500_INTERNAL_SERVER_ERROR;
	}
}

static size_t render_error(struct json_out *jo, int err, const char *msg)
{
	jo_init(jo, json_buf, JSON_BUF_SIZE);
	jo_obj_begin(jo);
	jo_str(jo, "error", msg != NULL ? msg : errno_message(err));
	jo_int(jo, "code", err);
	jo_obj_end(jo);
	return jo_finish(jo);
}

static int dispatch(enum http_method method, const char *url,
		    struct http_response_ctx *rsp)
{
	static const struct http_header json_hdrs[] = {
		{.name = "Content-Type", .value = "application/json"},
		{.name = "Access-Control-Allow-Origin", .value = "*"},
	};
	struct webapi_request req = {
		.body = body_buf,
		.body_len = body_len,
		.id = -1,
		.status = 0,
	};
	char path[128];
	size_t body_out_len;
	bool path_exists;

	/* Strip a query string; route patterns cover the path only. */
	size_t plen = strcspn(url, "?");

	if (plen >= sizeof(path)) {
		plen = sizeof(path) - 1;
	}
	memcpy(path, url, plen);
	path[plen] = '\0';

	const struct webapi_route *route =
		find_route(method, path, &req.id, &path_exists);

	jo_init(&req.out, json_buf, JSON_BUF_SIZE);

	if (route == NULL) {
		int err = path_exists ? -ENOTSUP : -ENOENT;

		body_out_len = render_error(&req.out, err,
					    path_exists ? "method not allowed"
							: "no such endpoint");
		rsp->status = path_exists ? HTTP_405_METHOD_NOT_ALLOWED
					  : HTTP_404_NOT_FOUND;
	} else {
		int ret = route->handler(&req);

		if (ret < 0) {
			body_out_len = render_error(&req.out, ret, NULL);
			rsp->status = req.status != 0 ? req.status
						      : errno_status(ret);
		} else {
			if (req.out.pos == 0) {
				jo_obj_begin(&req.out);
				jo_bool(&req.out, "ok", true);
				jo_obj_end(&req.out);
			}
			body_out_len = jo_finish(&req.out);

			if (req.out.overflow) {
				body_out_len = render_error(
					&req.out, -ENOMEM,
					"response too large");
				rsp->status = HTTP_500_INTERNAL_SERVER_ERROR;
			} else {
				rsp->status = req.status != 0 ? req.status
							      : HTTP_200_OK;
			}
		}
	}

	rsp->body = (const uint8_t *)json_buf;
	rsp->body_len = body_out_len;
	rsp->final_chunk = true;
	rsp->headers = json_hdrs;
	rsp->header_count = ARRAY_SIZE(json_hdrs);
	return 0;
}

/* ================================================================
 * Dynamic handler for the "/api/*" wildcard resource
 * ================================================================ */

static int api_handler(struct http_client_ctx *client,
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
		/* Terminal notifications — no response may be produced, and
		 * falling through would re-run the endpoint dispatch. */
		body_len = 0;
		return 0;
	}

	if (method == HTTP_OPTIONS) {
		/* CORS preflight */
		static const struct http_header cors_hdrs[] = {
			{.name = "Access-Control-Allow-Origin", .value = "*"},
			{.name = "Access-Control-Allow-Methods",
			 .value = "GET,POST,PUT,PATCH,DELETE,OPTIONS"},
			{.name = "Access-Control-Allow-Headers",
			 .value = "Content-Type"},
		};

		response_ctx->status = HTTP_200_OK;
		response_ctx->body = (const uint8_t *)"";
		response_ctx->body_len = 0;
		response_ctx->final_chunk = true;
		response_ctx->headers = cors_hdrs;
		response_ctx->header_count = ARRAY_SIZE(cors_hdrs);
		return 0;
	}

	bool has_body = method == HTTP_POST || method == HTTP_PUT ||
			method == HTTP_PATCH || method == HTTP_DELETE;

	if (has_body && request_ctx->data_len > 0) {
		size_t space = BODY_BUF_SIZE - body_len;
		size_t copy = MIN(request_ctx->data_len, space);

		memcpy(body_buf + body_len, request_ctx->data, copy);
		body_len += copy;
	}

	if (has_body && status != HTTP_SERVER_REQUEST_DATA_FINAL) {
		/* More body chunks to come — don't respond yet. */
		response_ctx->final_chunk = false;
		return 0;
	}

	int ret = dispatch(method, url, response_ctx);

	body_len = 0;
	return ret;
}

static struct http_resource_detail_dynamic api_resource_detail = {
	.common = {
		.type = HTTP_RESOURCE_TYPE_DYNAMIC,
		.bitmask_of_supported_http_methods =
			BIT(HTTP_GET) | BIT(HTTP_POST) | BIT(HTTP_PUT) |
			BIT(HTTP_PATCH) | BIT(HTTP_DELETE) | BIT(HTTP_OPTIONS),
		.content_type = "application/json",
	},
	.cb = api_handler,
	.user_data = NULL,
};

HTTP_RESOURCE_DEFINE(api_resource, aes67_http, "/api/*",
		     &api_resource_detail);

/* ================================================================
 * Config persistence helper shared by the resource modules
 * ================================================================ */

void webapi_persist_config(void)
{
#ifdef CONFIG_SD_CONFIG
	sd_config_save();
#endif
#ifdef CONFIG_FLASH_CONFIG
	flash_config_save();
#endif
}

#ifdef CONFIG_SPI_FLASH_LITESPI
/* ================================================================
 * Firmware update — its OWN resource, deliberately not part of the
 * "/api" wildcard above. Zephyr serialises each dynamic resource via a
 * per-resource "holder": while the (minutes-long, streaming) firmware
 * POST is in flight it holds its resource, and every other request on
 * the SAME resource is answered 409 Conflict. With fw_update inside
 * the API wildcard, the dashboard auto-refresh (or a retry after a
 * dropped upload, until the dead client passes the 30 s inactivity
 * reap) collided with the upload — observed as spurious 409s on
 * /api/fw_update. Exact-match resources take precedence over
 * wildcards, so this carve-out wins regardless of definition order.
 * ================================================================ */
static int fw_update_resource_handler(struct http_client_ctx *client,
				      enum http_transaction_status status,
				      const struct http_request_ctx *request_ctx,
				      struct http_response_ctx *response_ctx,
				      void *user_data)
{
	ARG_UNUSED(user_data);

	/* Streaming handler (body far too large to buffer), including
	 * abort notifications. */
	return fw_update_http_handler(client, status, request_ctx,
				      response_ctx);
}

static struct http_resource_detail_dynamic fw_update_resource_detail = {
	.common = {
		.type = HTTP_RESOURCE_TYPE_DYNAMIC,
		.bitmask_of_supported_http_methods =
			BIT(HTTP_GET) | BIT(HTTP_POST) | BIT(HTTP_OPTIONS),
		.content_type = "application/json",
	},
	.cb = fw_update_resource_handler,
	.user_data = NULL,
};

HTTP_RESOURCE_DEFINE(fw_update_resource, aes67_http, "/api/fw_update",
		     &fw_update_resource_detail);
#endif /* CONFIG_SPI_FLASH_LITESPI */

/* ================================================================
 * Static web UI — served as gzipped blobs; the .gz.inc files are
 * generated at build time by CMake.
 * ================================================================ */

static const uint8_t index_html_gz[] = {
#include "../index.html.gz.inc"
};

static struct http_resource_detail_static index_resource_detail = {
	.common = {
		.type = HTTP_RESOURCE_TYPE_STATIC,
		.bitmask_of_supported_http_methods = BIT(HTTP_GET),
		.content_encoding = "gzip",
		.content_type = "text/html",
	},
	.static_data = index_html_gz,
	.static_data_len = sizeof(index_html_gz),
};

HTTP_RESOURCE_DEFINE(index_resource, aes67_http, "/",
		     &index_resource_detail);

#ifdef CONFIG_DISPLAY_CTRL
static const uint8_t debug_html_gz[] = {
#include "../debug.html.gz.inc"
};

static struct http_resource_detail_static debug_resource_detail = {
	.common = {
		.type = HTTP_RESOURCE_TYPE_STATIC,
		.bitmask_of_supported_http_methods = BIT(HTTP_GET),
		.content_encoding = "gzip",
		.content_type = "text/html",
	},
	.static_data = debug_html_gz,
	.static_data_len = sizeof(debug_html_gz),
};

HTTP_RESOURCE_DEFINE(debug_resource, aes67_http, "/debug",
		     &debug_resource_detail);
#endif /* CONFIG_DISPLAY_CTRL */

/* ================================================================
 * Public API
 * ================================================================ */

int webserver_start(void)
{
	/* Initialize config defaults before first use */
	aes67_config_get();

	int ret = http_server_start();

	if (ret < 0) {
		LOG_ERR("WEB: Failed to start HTTP server: %d", ret);
		return ret;
	}

	LOG_INF("WEB: HTTP server started on port %u", http_port);
	return 0;
}
