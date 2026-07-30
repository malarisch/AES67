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

#if defined(CONFIG_NMOS_IS05) || defined(CONFIG_NMOS_IS08)
#define NMOS_HAVE_BODY 1
#include "nmos_json.h"
#endif

LOG_MODULE_REGISTER(nmos_api, LOG_LEVEL_INF);

/* Own response buffer: full resource lists (8 senders/receivers incl.
 * BCP-004 caps) and the IS-08 /io view exceed the REST API's shared
 * 4 KB buffer. */
#define NMOS_BUF_SIZE 16384
static NMOS_BIG_BSS char nmos_buf[NMOS_BUF_SIZE];

#ifdef NMOS_HAVE_BODY
/* Request-body accumulator (PATCH /staged, POST /bulk, POST
 * /map/activations) plus the node pool for the JSON DOM. Sized for a
 * staged SDP transport file. */
#define NMOS_BODY_SIZE 4096
static NMOS_BIG_BSS char nmos_body[NMOS_BODY_SIZE];
static size_t nmos_body_len;
static NMOS_BIG_BSS struct nj_node nmos_nodes[256];
#endif

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
	/* Transport files change on every activation. */
	{.name = "Cache-Control", .value = "no-cache"},
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

#ifdef CONFIG_NMOS_IS05

/* ================================================================
 * IS-05 Connection API (/x-nmos/connection/...)
 * ================================================================ */

static int is05_find(bool sender, const char *uuid)
{
	return coll_find_uuid(sender ? COLL_SENDERS : COLL_RECEIVERS, uuid);
}

static void is05_uuid_listing(struct http_response_ctx *rsp, bool sender)
{
	struct json_out jo;
	char uuid[NMOS_UUID_STR_LEN];
	int max = sender ? nmos_tx_count() : nmos_rx_count();

	jo_init(&jo, nmos_buf, sizeof(nmos_buf));
	jo_arr_begin(&jo);
	for (int i = 0; i < max; i++) {
		nmos_uuid(sender ? NMOS_RES_SENDER : NMOS_RES_RECEIVER, i,
			  uuid);
		/* Trailing slash is load-bearing: clients strip the last
		 * character to recover the id. */
		jo_fmt(&jo, "\"%s/\",", uuid);
	}
	jo_arr_end(&jo);
	finish_rsp(rsp, HTTP_200_OK, jo_finish(&jo), false);
}

static void is05_patch(struct http_response_ctx *rsp, bool sender, int idx,
		       const char *body, size_t blen)
{
	const char *errmsg;
	struct is05_act_echo echo;
	int root = nj_parse(body, blen, nmos_nodes, ARRAY_SIZE(nmos_nodes));

	if (root < 0) {
		error_rsp(rsp, HTTP_400_BAD_REQUEST,
			  "request body is not valid JSON");
		return;
	}

	int st = nmos_is05_stage(sender, idx, nmos_nodes, &nmos_nodes[root],
				 &echo, &errmsg);

	if (st != 200 && st != 202) {
		error_rsp(rsp, (uint16_t)st, errmsg);
		return;
	}

	struct json_out jo;

	jo_init(&jo, nmos_buf, sizeof(nmos_buf));
	nmos_is05_build_staged(&jo, sender, idx, &echo);
	if (jo.overflow) {
		error_rsp(rsp, HTTP_500_INTERNAL_SERVER_ERROR,
			  "response too large");
		return;
	}
	finish_rsp(rsp, st == 202 ? HTTP_202_ACCEPTED : HTTP_200_OK,
		   jo_finish(&jo), false);
}

static void is05_bulk(struct http_response_ctx *rsp, bool sender,
		      const char *body, size_t blen)
{
	int root = nj_parse(body, blen, nmos_nodes, ARRAY_SIZE(nmos_nodes));

	if (root < 0 || nmos_nodes[root].type != NJ_ARR) {
		error_rsp(rsp, HTTP_400_BAD_REQUEST,
			  "request body must be a JSON array");
		return;
	}

	/* The outer structure must validate as a whole (400); errors in
	 * the per-entry params are reported per entry with a 200. */
	for (int i = nmos_nodes[root].first_child; i >= 0;
	     i = nmos_nodes[i].next) {
		const struct nj_node *id = nj_get(nmos_nodes, &nmos_nodes[i],
						  "id");

		if (nmos_nodes[i].type != NJ_OBJ || id == NULL ||
		    id->type != NJ_STR) {
			error_rsp(rsp, HTTP_400_BAD_REQUEST,
				  "every entry requires an id");
			return;
		}
	}

	struct json_out jo;

	jo_init(&jo, nmos_buf, sizeof(nmos_buf));
	jo_arr_begin(&jo);
	for (int i = nmos_nodes[root].first_child; i >= 0;
	     i = nmos_nodes[i].next) {
		const struct nj_node *item = &nmos_nodes[i];
		const struct nj_node *id = nj_get(nmos_nodes, item, "id");
		const struct nj_node *params = nj_get(nmos_nodes, item,
						      "params");
		char uuid[NMOS_UUID_STR_LEN];
		int code;

		if (nj_strcpy(id, uuid, sizeof(uuid)) < 0) {
			uuid[0] = '\0';
		}

		int idx = uuid[0] != '\0' ? is05_find(sender, uuid) : -1;

		if (idx < 0) {
			code = 404;
		} else {
			struct is05_act_echo echo;
			const char *msg;

			code = nmos_is05_stage(sender, idx, nmos_nodes,
					       params, &echo, &msg);
		}
		jo_obj_begin(&jo);
		jo_str(&jo, "id", uuid);
		jo_int(&jo, "code", code);
		jo_obj_end(&jo);
	}
	jo_arr_end(&jo);
	finish_rsp(rsp, HTTP_200_OK, jo_finish(&jo), false);
}

#endif /* CONFIG_NMOS_IS05 */

/* ================================================================
 * Dispatch
 * ================================================================ */

static void nmos_dispatch(enum http_method method, const char *url,
			  struct http_response_ctx *rsp,
			  const char *body, size_t body_len)
{
	char path[160];
	char *seg[8];
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

	ARG_UNUSED(body);
	ARG_UNUSED(body_len);

	if (nseg < 1 || nseg >= (int)ARRAY_SIZE(seg) ||
	    strcmp(seg[0], "x-nmos") != 0) {
		error_rsp(rsp, HTTP_404_NOT_FOUND, "no such resource");
		return;
	}

	/* ---- /x-nmos ---- */
	if (nseg == 1) {
		static const char *const apis[] = {
#ifdef CONFIG_NMOS_IS08
			"channelmapping/",
#endif
#ifdef CONFIG_NMOS_IS05
			"connection/",
#endif
			"manifest/", "node/",
		};

		if (!is_get) {
			goto method_not_allowed;
		}
		listing_rsp(rsp, apis, ARRAY_SIZE(apis));
		return;
	}

#ifdef CONFIG_NMOS_IS05
	/* ---- /x-nmos/connection ---- */
	if (strcmp(seg[1], "connection") == 0) {
		if (nseg == 2) {
			static const char *const vers[] = {
				NMOS_IS05_VERSION "/",
			};

			if (!is_get) {
				goto method_not_allowed;
			}
			listing_rsp(rsp, vers, ARRAY_SIZE(vers));
			return;
		}
		if (strcmp(seg[2], NMOS_IS05_VERSION) != 0) {
			goto not_found;
		}
		if (nseg == 3) {
			static const char *const modes[] = {"bulk/", "single/"};

			if (!is_get) {
				goto method_not_allowed;
			}
			listing_rsp(rsp, modes, ARRAY_SIZE(modes));
			return;
		}

		bool bulk = strcmp(seg[3], "bulk") == 0;

		if (!bulk && strcmp(seg[3], "single") != 0) {
			goto not_found;
		}
		if (nseg == 4) {
			static const char *const kinds[] = {
				"senders/", "receivers/",
			};

			if (!is_get) {
				goto method_not_allowed;
			}
			listing_rsp(rsp, kinds, ARRAY_SIZE(kinds));
			return;
		}

		bool sender = strcmp(seg[4], "senders") == 0;

		if (!sender && strcmp(seg[4], "receivers") != 0) {
			goto not_found;
		}

		if (bulk) {
			/* GET answers an explicit 405 with error body. */
			if (nseg != 5) {
				goto not_found;
			}
			if (method == HTTP_POST) {
				is05_bulk(rsp, sender, body, body_len);
				return;
			}
			goto method_not_allowed;
		}

		if (nseg == 5) {
			if (!is_get) {
				goto method_not_allowed;
			}
			is05_uuid_listing(rsp, sender);
			return;
		}

		int cidx = is05_find(sender, seg[5]);

		if (cidx < 0) {
			goto not_found;
		}
		if (nseg == 6) {
			static const char *const snd_eps[] = {
				"constraints/", "staged/", "active/",
				"transportfile/", "transporttype/",
			};
			static const char *const rcv_eps[] = {
				"constraints/", "staged/", "active/",
				"transporttype/",
			};

			if (!is_get) {
				goto method_not_allowed;
			}
			if (sender) {
				listing_rsp(rsp, snd_eps, ARRAY_SIZE(snd_eps));
			} else {
				listing_rsp(rsp, rcv_eps, ARRAY_SIZE(rcv_eps));
			}
			return;
		}

		/* nseg == 7: the per-resource endpoints */
		if (strcmp(seg[6], "staged") == 0) {
			if (method == HTTP_PATCH) {
				is05_patch(rsp, sender, cidx, body, body_len);
				return;
			}
			if (!is_get) {
				goto method_not_allowed;
			}
			jo_init(&jo, nmos_buf, sizeof(nmos_buf));
			nmos_is05_build_staged(&jo, sender, cidx, NULL);
			goto finish_json;
		}
		if (strcmp(seg[6], "active") == 0) {
			if (!is_get) {
				goto method_not_allowed;
			}
			jo_init(&jo, nmos_buf, sizeof(nmos_buf));
			nmos_is05_build_active(&jo, sender, cidx);
			goto finish_json;
		}
		if (strcmp(seg[6], "constraints") == 0) {
			if (!is_get) {
				goto method_not_allowed;
			}
			jo_init(&jo, nmos_buf, sizeof(nmos_buf));
			nmos_is05_build_constraints(&jo, sender, cidx);
			goto finish_json;
		}
		if (strcmp(seg[6], "transporttype") == 0) {
			if (!is_get) {
				goto method_not_allowed;
			}

			int len = snprintf(nmos_buf, sizeof(nmos_buf),
					   "\"urn:x-nmos:transport:rtp\"");

			finish_rsp(rsp, HTTP_200_OK, (size_t)len, false);
			return;
		}
		if (sender && strcmp(seg[6], "transportfile") == 0) {
			if (!is_get) {
				goto method_not_allowed;
			}

			int len = nmos_build_manifest(nmos_buf,
						      sizeof(nmos_buf), cidx);

			if (len < 0) {
				goto not_found;
			}
			finish_rsp(rsp, HTTP_200_OK, (size_t)len, true);
			return;
		}
		goto not_found;
	}
#endif /* CONFIG_NMOS_IS05 */

#ifdef CONFIG_NMOS_IS08
	/* ---- /x-nmos/channelmapping ---- */
	if (strcmp(seg[1], "channelmapping") == 0) {
		if (nseg == 2) {
			static const char *const vers[] = {
				NMOS_IS08_VERSION "/",
			};

			if (!is_get) {
				goto method_not_allowed;
			}
			listing_rsp(rsp, vers, ARRAY_SIZE(vers));
			return;
		}
		if (strcmp(seg[2], NMOS_IS08_VERSION) != 0) {
			goto not_found;
		}
		if (nseg == 3) {
			static const char *const base[] = {
				"inputs/", "outputs/", "map/", "io/",
			};

			if (!is_get) {
				goto method_not_allowed;
			}
			listing_rsp(rsp, base, ARRAY_SIZE(base));
			return;
		}

		if (strcmp(seg[3], "io") == 0) {
			if (nseg != 4) {
				goto not_found;
			}
			if (!is_get) {
				goto method_not_allowed;
			}
			jo_init(&jo, nmos_buf, sizeof(nmos_buf));
			nmos_is08_build_io(&jo);
			goto finish_json;
		}

		bool inputs = strcmp(seg[3], "inputs") == 0;

		if (inputs || strcmp(seg[3], "outputs") == 0) {
			if (nseg == 4) {
				if (!is_get) {
					goto method_not_allowed;
				}
				jo_init(&jo, nmos_buf, sizeof(nmos_buf));
				nmos_is08_build_list(&jo, inputs);
				goto finish_json;
			}
			if (!nmos_is08_known_id(inputs, seg[4])) {
				goto not_found;
			}
			if (nseg == 5) {
				static const char *const in_eps[] = {
					"properties/", "parent/",
					"channels/", "caps/",
				};
				static const char *const out_eps[] = {
					"properties/", "sourceid/",
					"channels/", "caps/",
				};

				if (!is_get) {
					goto method_not_allowed;
				}
				if (inputs) {
					listing_rsp(rsp, in_eps,
						    ARRAY_SIZE(in_eps));
				} else {
					listing_rsp(rsp, out_eps,
						    ARRAY_SIZE(out_eps));
				}
				return;
			}
			if (nseg != 6) {
				goto not_found;
			}
			if (!is_get) {
				goto method_not_allowed;
			}

			int kind;

			if (strcmp(seg[5], "properties") == 0) {
				kind = 0;
			} else if (inputs &&
				   strcmp(seg[5], "parent") == 0) {
				kind = 1;
			} else if (!inputs &&
				   strcmp(seg[5], "sourceid") == 0) {
				kind = 1;
			} else if (strcmp(seg[5], "channels") == 0) {
				kind = 2;
			} else if (strcmp(seg[5], "caps") == 0) {
				kind = 3;
			} else {
				goto not_found;
			}
			jo_init(&jo, nmos_buf, sizeof(nmos_buf));
			if (!nmos_is08_build_child(&jo, inputs, seg[4],
						   kind)) {
				goto not_found;
			}
			goto finish_json;
		}

		if (strcmp(seg[3], "map") != 0) {
			goto not_found;
		}
		if (nseg == 4) {
			static const char *const map_eps[] = {
				"activations/", "active/",
			};

			if (!is_get) {
				goto method_not_allowed;
			}
			listing_rsp(rsp, map_eps, ARRAY_SIZE(map_eps));
			return;
		}
		if (strcmp(seg[4], "active") == 0) {
			if (!is_get) {
				goto method_not_allowed;
			}
			if (nseg == 6 &&
			    !nmos_is08_known_id(false, seg[5])) {
				goto not_found;
			}
			if (nseg > 6) {
				goto not_found;
			}
			jo_init(&jo, nmos_buf, sizeof(nmos_buf));
			nmos_is08_build_active(&jo, nseg == 6 ? seg[5]
							      : NULL);
			goto finish_json;
		}
		if (strcmp(seg[4], "activations") != 0) {
			goto not_found;
		}
		if (nseg == 5) {
			if (method == HTTP_POST) {
				const char *errmsg = "invalid request";
				int root = nj_parse(body, body_len,
						    nmos_nodes,
						    ARRAY_SIZE(nmos_nodes));

				if (root < 0 ||
				    nmos_nodes[root].type != NJ_OBJ) {
					error_rsp(rsp, HTTP_400_BAD_REQUEST,
						  "request body is not a "
						  "JSON object");
					return;
				}
				jo_init(&jo, nmos_buf, sizeof(nmos_buf));

				int st = nmos_is08_post_activation(
					&jo, nmos_nodes, &nmos_nodes[root],
					&errmsg);

				if (st != 200 && st != 202) {
					error_rsp(rsp, (uint16_t)st, errmsg);
					return;
				}
				if (jo.overflow) {
					error_rsp(rsp,
						  HTTP_500_INTERNAL_SERVER_ERROR,
						  "response too large");
					return;
				}
				finish_rsp(rsp,
					   st == 202 ? HTTP_202_ACCEPTED
						     : HTTP_200_OK,
					   jo_finish(&jo), false);
				return;
			}
			if (!is_get) {
				goto method_not_allowed;
			}
			jo_init(&jo, nmos_buf, sizeof(nmos_buf));
			nmos_is08_build_activations(&jo);
			goto finish_json;
		}
		if (nseg != 6) {
			goto not_found;
		}
		if (method == HTTP_DELETE) {
			if (!nmos_is08_delete_activation(seg[5])) {
				goto not_found;
			}
			finish_rsp(rsp, HTTP_204_NO_CONTENT, 0, false);
			return;
		}
		if (!is_get) {
			goto method_not_allowed;
		}
		jo_init(&jo, nmos_buf, sizeof(nmos_buf));
		if (!nmos_is08_build_activation(&jo, seg[5])) {
			goto not_found;
		}
		goto finish_json;
	}
#endif /* CONFIG_NMOS_IS08 */

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
#ifdef NMOS_HAVE_BODY
		nmos_body_len = 0;
#endif
		return 0;
	}

	if (method == HTTP_OPTIONS) {
		/* CORS preflight. Allow-Methods must cover every method the
		 * spec declares for the path. */
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
		static const struct http_header patch_hdrs[] = {
			{.name = "Access-Control-Allow-Origin", .value = "*"},
			{.name = "Access-Control-Allow-Methods",
			 .value = "GET, HEAD, PATCH, OPTIONS"},
			{.name = "Access-Control-Allow-Headers",
			 .value = "Content-Type"},
		};
		static const struct http_header post_hdrs[] = {
			{.name = "Access-Control-Allow-Origin", .value = "*"},
			{.name = "Access-Control-Allow-Methods",
			 .value = "POST, OPTIONS"},
			{.name = "Access-Control-Allow-Headers",
			 .value = "Content-Type"},
		};
		static const struct http_header getpost_hdrs[] = {
			{.name = "Access-Control-Allow-Origin", .value = "*"},
			{.name = "Access-Control-Allow-Methods",
			 .value = "GET, HEAD, POST, OPTIONS"},
			{.name = "Access-Control-Allow-Headers",
			 .value = "Content-Type"},
		};
		static const struct http_header getdel_hdrs[] = {
			{.name = "Access-Control-Allow-Origin", .value = "*"},
			{.name = "Access-Control-Allow-Methods",
			 .value = "GET, HEAD, DELETE, OPTIONS"},
			{.name = "Access-Control-Allow-Headers",
			 .value = "Content-Type"},
		};
		size_t plen = strcspn(url, "?");

		while (plen > 1 && url[plen - 1] == '/') {
			plen--;
		}

		const struct http_header *hdrs = get_hdrs;
		size_t nhdrs = ARRAY_SIZE(get_hdrs);

		if (plen >= 7 && strncmp(url + plen - 7, "/target", 7) == 0) {
			hdrs = put_hdrs;
			nhdrs = ARRAY_SIZE(put_hdrs);
		} else if (plen >= 7 &&
			   strncmp(url + plen - 7, "/staged", 7) == 0) {
			hdrs = patch_hdrs;
			nhdrs = ARRAY_SIZE(patch_hdrs);
		} else if (strstr(url, "/bulk/") != NULL) {
			hdrs = post_hdrs;
			nhdrs = ARRAY_SIZE(post_hdrs);
		} else if (strstr(url, "/channelmapping/") != NULL &&
			   plen >= 12 &&
			   strncmp(url + plen - 12, "/activations", 12) == 0) {
			/* IS-08 POST /map/activations */
			hdrs = getpost_hdrs;
			nhdrs = ARRAY_SIZE(getpost_hdrs);
		} else if (strstr(url, "/activations/") != NULL) {
			/* IS-08 /map/activations/{activationId} */
			hdrs = getdel_hdrs;
			nhdrs = ARRAY_SIZE(getdel_hdrs);
		}

		response_ctx->status = HTTP_200_OK;
		response_ctx->body = (const uint8_t *)"";
		response_ctx->body_len = 0;
		response_ctx->final_chunk = true;
		response_ctx->headers = hdrs;
		response_ctx->header_count = nhdrs;
		return 0;
	}

	bool has_body = method == HTTP_POST || method == HTTP_PUT ||
			method == HTTP_PATCH || method == HTTP_DELETE;

#ifdef NMOS_HAVE_BODY
	/* Accumulate PATCH/POST bodies for the Connection / Channel
	 * Mapping APIs; an overlong body simply fails JSON parsing
	 * later (400). */
	if (has_body && request_ctx->data_len > 0) {
		size_t space = sizeof(nmos_body) - nmos_body_len;
		size_t copy = MIN(request_ctx->data_len, space);

		memcpy(nmos_body + nmos_body_len, request_ctx->data, copy);
		nmos_body_len += copy;
	}
#endif

	if (has_body && status != HTTP_SERVER_REQUEST_DATA_FINAL) {
		response_ctx->final_chunk = false;
		return 0;
	}

	ARG_UNUSED(request_ctx);

#ifdef NMOS_HAVE_BODY
	nmos_dispatch(method, url, response_ctx, nmos_body, nmos_body_len);
	nmos_body_len = 0;
#else
	nmos_dispatch(method, url, response_ctx, NULL, 0);
#endif
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
