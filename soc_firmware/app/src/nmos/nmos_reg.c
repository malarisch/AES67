/*
 * NMOS IS-04 registration client.
 *
 * Registered operation per IS-04 "Behaviour - Registration": discover
 * Registration APIs through mdns_sd's _nmos-register._tcp cache, POST
 * the resources in referential-integrity order (Node -> Device ->
 * Sources -> Flows -> Senders -> Receivers), then heartbeat every 5 s.
 * Error handling follows the spec's table:
 *  - 200 on the FIRST node POST: a stale record exists -> DELETE the
 *    node and re-register (expect 201),
 *  - 404 on heartbeat: garbage-collected -> full re-registration,
 *  - 5xx / connection failure / timeout: fail over to the next
 *    discovered registry; the first interaction there is a heartbeat
 *    (200 = still registered, 404 = re-register),
 *  - all registries failing: exponential backoff, and the node falls
 *    back to peer-to-peer operation (advertisement re-appears) until a
 *    registration succeeds again.
 *
 * The HTTP client is a minimal hand-rolled HTTP/1.1 over BSD sockets
 * (same shape as rtsp_client_describe): one request per connection,
 * "Connection: close", only the status code is consumed.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/socket.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "nmos.h"
#include "nmos_priv.h"
#include "../aes67_conn.h"
#include "../mdns_sd.h"

LOG_MODULE_REGISTER(nmos_reg, LOG_LEVEL_INF);

#define REG_BASE_PATH   "/x-nmos/registration/" NMOS_API_VERSION
#define HEARTBEAT_MS    5000
#define DISCOVER_POLL_MS 2000
#define HTTP_TIMEOUT_S  5
#define BACKOFF_MAX_S   64

static char reg_body[4096];

/* Counters at the last successful push — a later mismatch is what marks
 * resources dirty. (Resources never disappear: all TX/RX slots exist
 * persistently as sources/flows/senders/receivers.) */
static struct nmos_mdns_info pushed;

/* ================================================================
 * Minimal HTTP client
 * ================================================================ */

/* Returns the HTTP status code, or negative errno on transport failure. */
static int http_request(const struct in_addr *ip, uint16_t port,
			const char *method, const char *path,
			const char *body, size_t body_len)
{
	char head[320];
	char ipstr[INET_ADDRSTRLEN];
	char rx[192];
	int sock, ret, status = -EIO;

	sock = zsock_socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (sock < 0) {
		return -errno;
	}

	struct timeval tv = { .tv_sec = HTTP_TIMEOUT_S };

	(void)zsock_setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
	(void)zsock_setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

	struct sockaddr_in sa = {
		.sin_family = AF_INET,
		.sin_port = htons(port),
		.sin_addr = *ip,
	};

	/* Non-blocking connect with an explicit deadline: SO_SNDTIMEO does
	 * not bound connect(), and a black-holed registry (the testing
	 * tool simulates one) must not stall failover beyond a heartbeat
	 * period. */
	int fl = zsock_fcntl(sock, F_GETFL, 0);

	(void)zsock_fcntl(sock, F_SETFL, fl | O_NONBLOCK);
	ret = zsock_connect(sock, (struct sockaddr *)&sa, sizeof(sa));
	if (ret < 0 && errno == EINPROGRESS) {
		struct zsock_pollfd pfd = {
			.fd = sock,
			.events = ZSOCK_POLLOUT,
		};
		int soerr = 0;
		socklen_t slen = sizeof(soerr);

		if (zsock_poll(&pfd, 1, HTTP_TIMEOUT_S * 1000) <= 0) {
			zsock_close(sock);
			return -ETIMEDOUT;
		}
		(void)zsock_getsockopt(sock, SOL_SOCKET, SO_ERROR, &soerr,
				       &slen);
		if (soerr != 0) {
			zsock_close(sock);
			return -soerr;
		}
	} else if (ret < 0) {
		ret = -errno;
		zsock_close(sock);
		return ret;
	}
	(void)zsock_fcntl(sock, F_SETFL, fl);

	zsock_inet_ntop(AF_INET, ip, ipstr, sizeof(ipstr));

	/* Exactly one of Content-Length/Transfer-Encoding, Content-Type
	 * only alongside a body (the health POST has neither body nor
	 * Content-Type — IS-04 client implementation notes). */
	int hl = snprintf(head, sizeof(head),
			  "%s %s HTTP/1.1\r\n"
			  "Host: %s:%u\r\n"
			  "%s"
			  "Content-Length: %u\r\n"
			  "Connection: close\r\n"
			  "\r\n",
			  method, path, ipstr, port,
			  body_len > 0 ? "Content-Type: application/json\r\n"
				       : "",
			  (unsigned int)body_len);

	if (hl < 0 || hl >= (int)sizeof(head) ||
	    zsock_send(sock, head, hl, 0) != hl) {
		zsock_close(sock);
		return -EIO;
	}
	if (body_len > 0 &&
	    zsock_send(sock, body, body_len, 0) != (ssize_t)body_len) {
		zsock_close(sock);
		return -EIO;
	}

	/* Only the status line matters. */
	ret = zsock_recv(sock, rx, sizeof(rx) - 1, 0);
	if (ret > 0) {
		rx[ret] = '\0';
		const char *sp = strchr(rx, ' ');

		if (sp != NULL) {
			int code = atoi(sp + 1);

			if (code >= 100 && code <= 599) {
				status = code;
			}
		}
	} else {
		status = ret == 0 ? -ECONNRESET : -errno;
	}

	zsock_close(sock);
	return status;
}

/* ================================================================
 * Request builders
 * ================================================================ */

/* {"type":"<type>","data":{...}} into reg_body; 0 = resource inactive
 * or overflow. */
static size_t wrap_body(const char *type,
			void (*build_plain)(struct json_out *),
			bool (*build_idx)(struct json_out *, int), int idx)
{
	struct json_out jo;

	jo_init(&jo, reg_body, sizeof(reg_body));
	jo_obj_begin(&jo);
	jo_str(&jo, "type", type);
	jo_key(&jo, "data");
	if (build_plain != NULL) {
		build_plain(&jo);
	} else if (!build_idx(&jo, idx)) {
		return 0;
	}
	jo_obj_end(&jo);

	size_t len = jo_finish(&jo);

	return jo.overflow ? 0 : len;
}

static int post_body(const struct mdns_nmos_registry *r, size_t len)
{
	return http_request(&r->ip, r->port, "POST",
			    REG_BASE_PATH "/resource", reg_body, len);
}

static int delete_resource(const struct mdns_nmos_registry *r,
			   const char *coll, enum nmos_res_kind kind, int idx)
{
	char path[96];
	char uuid[NMOS_UUID_STR_LEN];

	nmos_uuid(kind, idx, uuid);
	snprintf(path, sizeof(path), REG_BASE_PATH "/resource/%s/%s",
		 coll, uuid);
	return http_request(&r->ip, r->port, "DELETE", path, NULL, 0);
}

/* Timestamp of the last heartbeat attempt — the 5 s cadence is anchored
 * here so that request duration and resource pushes do not stretch the
 * interval (the registry GCs after ~2 missed beats). */
static int64_t last_hb_ms;

static int heartbeat(const struct mdns_nmos_registry *r)
{
	char path[96];
	char uuid[NMOS_UUID_STR_LEN];

	nmos_uuid(NMOS_RES_NODE, 0, uuid);
	snprintf(path, sizeof(path), REG_BASE_PATH "/health/nodes/%s", uuid);
	last_hb_ms = k_uptime_get();
	return http_request(&r->ip, r->port, "POST", path, NULL, 0);
}

static bool post_ok(int status)
{
	return status == 200 || status == 201;
}

/* ================================================================
 * Registration / sync
 * ================================================================ */

/* POST source+flow+sender of one TX slot (parent-first). */
static int post_tx_slot(const struct mdns_nmos_registry *r, int i)
{
	size_t len;

	len = wrap_body("source", NULL, nmos_build_source, i);
	if (len == 0 || !post_ok(post_body(r, len))) {
		return -EIO;
	}
	len = wrap_body("flow", NULL, nmos_build_flow, i);
	if (len == 0 || !post_ok(post_body(r, len))) {
		return -EIO;
	}
	len = wrap_body("sender", NULL, nmos_build_sender, i);
	if (len == 0 || !post_ok(post_body(r, len))) {
		return -EIO;
	}
	return 0;
}

static int full_register(const struct mdns_nmos_registry *r)
{
	size_t len;
	int status;

	/* Node first. A 200 means the registry still holds a record from a
	 * previous run: explicitly DELETE and register afresh. */
	len = wrap_body("node", nmos_build_self, NULL, 0);
	if (len == 0) {
		return -ENOMEM;
	}
	status = post_body(r, len);
	if (status == 200) {
		LOG_INF("NMOS: stale registration found - re-registering");
		(void)delete_resource(r, "nodes", NMOS_RES_NODE, 0);
		len = wrap_body("node", nmos_build_self, NULL, 0);
		status = post_body(r, len);
	}
	if (!post_ok(status)) {
		LOG_WRN("NMOS: node registration failed (%d)", status);
		return -EIO;
	}

	/* Heartbeating starts right after the Node resource (spec startup
	 * sequence step 5) — the remaining resources follow. */
	(void)heartbeat(r);

	len = wrap_body("device", nmos_build_device, NULL, 0);
	if (len == 0 || !post_ok(post_body(r, len))) {
		return -EIO;
	}

	for (int i = 0; i < nmos_tx_count(); i++) {
		if (post_tx_slot(r, i) < 0) {
			return -EIO;
		}
	}
	for (int i = 0; i < nmos_rx_count(); i++) {
		len = wrap_body("receiver", NULL, nmos_build_receiver, i);
		if (len == 0 || !post_ok(post_body(r, len))) {
			return -EIO;
		}
	}

	nmos_get_mdns_info(&pushed);
	return 0;
}

/* Re-push what changed since the last successful push. Any failure
 * escalates to a full re-registration by the caller. */
static int sync_changes(const struct mdns_nmos_registry *r)
{
	struct nmos_mdns_info cur;
	size_t len;

	nmos_get_mdns_info(&cur);

	if (cur.ver_slf != pushed.ver_slf) {
		len = wrap_body("node", nmos_build_self, NULL, 0);
		if (len == 0 || !post_ok(post_body(r, len))) {
			return -EIO;
		}
	}

	bool tx_dirty = cur.ver_src != pushed.ver_src ||
			cur.ver_flw != pushed.ver_flw ||
			cur.ver_snd != pushed.ver_snd ||
			cur.ver_dvc != pushed.ver_dvc;

	if (tx_dirty) {
		len = wrap_body("device", nmos_build_device, NULL, 0);
		if (len == 0 || !post_ok(post_body(r, len))) {
			return -EIO;
		}
		for (int i = 0; i < nmos_tx_count(); i++) {
			if (post_tx_slot(r, i) < 0) {
				return -EIO;
			}
		}
	}

	if (cur.ver_rcv != pushed.ver_rcv) {
		for (int i = 0; i < nmos_rx_count(); i++) {
			len = wrap_body("receiver", NULL,
					nmos_build_receiver, i);
			if (len == 0 || !post_ok(post_body(r, len))) {
				return -EIO;
			}
		}
	}

	pushed = cur;
	return 0;
}

/* First interaction with a (possibly new) registry: heartbeat, and on
 * 404 a full registration. Returns 0 when the node is registered. */
static int try_adopt(const struct mdns_nmos_registry *r)
{
	int status = heartbeat(r);

	if (status == 200) {
		return 0;
	}
	if (status == 404) {
		return full_register(r);
	}
	return -EIO;
}

/* ================================================================
 * Client thread
 * ================================================================ */

K_THREAD_STACK_DEFINE(nmos_reg_stack, 6144);
static struct k_thread nmos_reg_thread_data;

static void reg_thread_fn(void *a, void *b, void *c)
{
	struct mdns_nmos_registry regs[10];
	struct mdns_nmos_registry active;
	bool registered = false;
	uint32_t backoff_s = 1;
	char ipstr[INET_ADDRSTRLEN];

	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	while (true) {
		if (!nmos_have_ip()) {
			k_msleep(1000);
			continue;
		}

		if (!registered) {
			int n = mdns_sd_get_nmos_registries(regs,
							    ARRAY_SIZE(regs));

			nmos_set_registry_present(n > 0);
			if (n == 0) {
				/* No registry on the network: P2P mode. */
				nmos_set_registered(false);
				backoff_s = 1;
				k_msleep(DISCOVER_POLL_MS);
				continue;
			}

			int i;

			for (i = 0; i < n; i++) {
				if (try_adopt(&regs[i]) == 0) {
					break;
				}
			}
			if (i == n) {
				nmos_set_registered(false);
				LOG_WRN("NMOS: all %d registries failed - "
					"backoff %u s", n, backoff_s);
				k_msleep(backoff_s * 1000);
				backoff_s = MIN(backoff_s * 2, BACKOFF_MAX_S);
				continue;
			}

			active = regs[i];
			registered = true;
			backoff_s = 1;
			nmos_set_registered(true);
			zsock_inet_ntop(AF_INET, &active.ip, ipstr,
					sizeof(ipstr));
			LOG_INF("NMOS: registered with %s:%u (pri %u)",
				ipstr, active.port, active.pri);
			continue;
		}

		int64_t wait_ms = last_hb_ms + HEARTBEAT_MS - k_uptime_get();

		if (wait_ms > 0) {
			k_msleep((int32_t)wait_ms);
		}

		int status = heartbeat(&active);

		if (status == 200) {
			if (sync_changes(&active) < 0 &&
			    full_register(&active) < 0) {
				LOG_WRN("NMOS: resource push failed - "
					"dropping registration");
				registered = false;
			}
		} else if (status == 404) {
			LOG_INF("NMOS: garbage-collected - re-registering");
			if (full_register(&active) < 0) {
				registered = false;
			}
		} else {
			LOG_WRN("NMOS: heartbeat failed (%d) - failing over",
				status);
			registered = false;
		}

		if (!registered) {
			nmos_set_registered(false);
		}
	}
}

void nmos_reg_start(void)
{
	k_thread_create(&nmos_reg_thread_data, nmos_reg_stack,
			K_THREAD_STACK_SIZEOF(nmos_reg_stack),
			reg_thread_fn, NULL, NULL, NULL,
			K_PRIO_PREEMPT(10), 0, K_NO_WAIT);
	k_thread_name_set(&nmos_reg_thread_data, "nmos_reg");
}
