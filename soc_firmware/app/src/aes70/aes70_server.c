/*
 * OCP.1 over TCP (AES70-3 clause 8.4.3.1): listener + session thread.
 *
 * One thread owns the listen socket and every controller connection
 * (zsock_poll), feeds received bytes into the session core, and runs
 * the core's periodic work (keep-alive supervision, PropertyChanged
 * detection) between polls. All device-model callbacks therefore run on
 * this thread, like the web API handlers run on the HTTP server's.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/socket.h>
#include <zephyr/shell/shell.h>
#include <errno.h>
#include <string.h>

#include "aes70.h"
#include "aes70_device.h"
#include "aes70_model.h"

LOG_MODULE_REGISTER(aes70, LOG_LEVEL_INF);

#define AES70_PORT        CONFIG_AES70_PORT
#define POLL_TIMEOUT_MS   250
#define SEND_TIMEOUT_MS   2000

struct conn {
	int sock;                      /* -1 = free */
	struct oca_session *session;
	struct sockaddr_in peer;
};

static struct conn conns[AES70_MAX_SESSIONS];
static int listen_sock = -1;
static bool running;

K_THREAD_STACK_DEFINE(aes70_stack, CONFIG_AES70_STACK_SIZE);
static struct k_thread aes70_thread_data;

static int conn_send(struct oca_session *s, const uint8_t *buf, size_t len)
{
	struct conn *c = s->transport;
	size_t off = 0;

	while (off < len) {
		int n = zsock_send(c->sock, buf + off, len - off, 0);

		if (n < 0) {
			return -errno;
		}
		off += (size_t)n;
	}
	return 0;
}

static void conn_close(struct conn *c, const char *why)
{
	char ip[NET_IPV4_ADDR_LEN];

	zsock_inet_ntop(AF_INET, &c->peer.sin_addr, ip, sizeof(ip));
	LOG_INF("controller %s:%u disconnected (%s)", ip,
		ntohs(c->peer.sin_port), why);
	aes70_core_session_close(c->session);
	zsock_close(c->sock);
	c->sock = -1;
	c->session = NULL;
}

static void accept_conn(int64_t now)
{
	struct sockaddr_in peer;
	socklen_t plen = sizeof(peer);
	int sock = zsock_accept(listen_sock, (struct sockaddr *)&peer, &plen);
	struct conn *c = NULL;
	struct timeval tv = { .tv_sec = SEND_TIMEOUT_MS / 1000,
			      .tv_usec = (SEND_TIMEOUT_MS % 1000) * 1000 };
	char ip[NET_IPV4_ADDR_LEN];

	if (sock < 0) {
		return;
	}
	for (int i = 0; i < AES70_MAX_SESSIONS; i++) {
		if (conns[i].sock < 0) {
			c = &conns[i];
			break;
		}
	}
	zsock_inet_ntop(AF_INET, &peer.sin_addr, ip, sizeof(ip));
	if (c == NULL) {
		LOG_WRN("controller %s refused: all %d sessions in use", ip,
			AES70_MAX_SESSIONS);
		zsock_close(sock);
		return;
	}
	c->session = aes70_core_session_open(conn_send, c, now);
	if (c->session == NULL) {
		zsock_close(sock);
		return;
	}
	zsock_setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
	c->sock = sock;
	c->peer = peer;
	LOG_INF("controller %s:%u connected", ip, ntohs(peer.sin_port));
}

static void server_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

	struct sockaddr_in addr = {
		.sin_family = AF_INET,
		.sin_port = htons(AES70_PORT),
		.sin_addr.s_addr = INADDR_ANY,
	};
	int one = 1;
	static uint8_t rx[512];

	listen_sock = zsock_socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (listen_sock < 0) {
		LOG_ERR("socket: %d", -errno);
		return;
	}
	zsock_setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
	if (zsock_bind(listen_sock, (struct sockaddr *)&addr, sizeof(addr)) < 0 ||
	    zsock_listen(listen_sock, 2) < 0) {
		LOG_ERR("bind/listen on %u: %d", AES70_PORT, -errno);
		zsock_close(listen_sock);
		listen_sock = -1;
		return;
	}
	running = true;
	LOG_INF("OCP.1 server listening on TCP %u", AES70_PORT);

	for (;;) {
		struct zsock_pollfd fds[1 + AES70_MAX_SESSIONS];
		int nfds = 1;
		int64_t now;

		fds[0].fd = listen_sock;
		fds[0].events = ZSOCK_POLLIN;
		for (int i = 0; i < AES70_MAX_SESSIONS; i++) {
			if (conns[i].sock >= 0) {
				fds[nfds].fd = conns[i].sock;
				fds[nfds].events = ZSOCK_POLLIN;
				nfds++;
			}
		}

		int ret = zsock_poll(fds, nfds, POLL_TIMEOUT_MS);

		now = k_uptime_get();
		if (ret > 0) {
			if (fds[0].revents & ZSOCK_POLLIN) {
				accept_conn(now);
			}
			for (int f = 1; f < nfds; f++) {
				struct conn *c = NULL;

				if (fds[f].revents == 0) {
					continue;
				}
				for (int i = 0; i < AES70_MAX_SESSIONS; i++) {
					if (conns[i].sock == fds[f].fd) {
						c = &conns[i];
						break;
					}
				}
				if (c == NULL) {
					continue;
				}
				if (fds[f].revents & (ZSOCK_POLLERR | ZSOCK_POLLHUP |
						      ZSOCK_POLLNVAL)) {
					conn_close(c, "socket error");
					continue;
				}
				int n = zsock_recv(c->sock, rx, sizeof(rx),
						   ZSOCK_MSG_DONTWAIT);

				if (n == 0) {
					conn_close(c, "closed by peer");
					continue;
				}
				if (n < 0) {
					if (errno != EAGAIN && errno != EWOULDBLOCK) {
						conn_close(c, "recv error");
					}
					continue;
				}
				int err = aes70_core_session_input(c->session, rx,
								   n, now);

				if (err < 0) {
					LOG_WRN("protocol error %d, %d bytes "
						"received", err, n);
					LOG_HEXDUMP_WRN(rx, MIN(n, 32),
							"first bytes");
					conn_close(c, "protocol error");
				}
			}
		}

		aes70_core_tick(now);
		for (int i = 0; i < AES70_MAX_SESSIONS; i++) {
			if (conns[i].sock >= 0 && conns[i].session->close_pending) {
				conn_close(&conns[i], "keep-alive timeout");
			}
		}
	}
}

int aes70_start(struct net_if *iface)
{
	int ret = aes70_device_build(iface);

	if (ret < 0) {
		return ret;
	}
	for (int i = 0; i < AES70_MAX_SESSIONS; i++) {
		conns[i].sock = -1;
	}
	k_thread_create(&aes70_thread_data, aes70_stack,
			K_THREAD_STACK_SIZEOF(aes70_stack), server_thread,
			NULL, NULL, NULL, K_PRIO_PREEMPT(8), 0, K_NO_WAIT);
	k_thread_name_set(&aes70_thread_data, "aes70");
	return 0;
}

void aes70_notify_ip_ready(const struct in_addr *addr)
{
	aes70_device_set_ip(addr);
}

void aes70_get_status(struct aes70_status *st)
{
	memset(st, 0, sizeof(*st));
	st->running = running;
	st->port = AES70_PORT;
	st->objects = aes70_core_object_count();
	for (int i = 0; i < AES70_MAX_SESSIONS; i++) {
		st->sessions += conns[i].sock >= 0;
	}
	st->subscriptions = aes70_core_subscription_count();
}

/* ================================================================
 * Shell
 * ================================================================ */

#ifdef CONFIG_SHELL
static int cmd_aes70_status(const struct shell *sh, size_t argc, char **argv)
{
	struct aes70_status st;

	ARG_UNUSED(argc); ARG_UNUSED(argv);
	aes70_get_status(&st);
	shell_print(sh, "OCP.1 server: %s, TCP port %u, %d objects",
		    st.running ? "running" : "stopped", st.port, st.objects);
	shell_print(sh, "Controllers: %d, subscriptions: %d", st.sessions,
		    st.subscriptions);
	for (int i = 0; i < AES70_MAX_SESSIONS; i++) {
		const struct conn *c = &conns[i];
		char ip[NET_IPV4_ADDR_LEN];

		if (c->sock < 0) {
			continue;
		}
		zsock_inet_ntop(AF_INET, &c->peer.sin_addr, ip, sizeof(ip));
		shell_print(sh, "  %s:%u  cmds %u  ntfs %u  heartbeat %u ms",
			    ip, ntohs(c->peer.sin_port), c->session->commands,
			    c->session->notifications, c->session->heartbeat_ms);
	}
	return 0;
}

static void trace_cmd(uint32_t ono, uint16_t level, uint16_t index, int status,
		      uint8_t out_count)
{
	const struct oca_object *o = aes70_core_find(ono);

	LOG_INF("cmd ono %u (%s) %u.%u -> status %d, %u out", ono,
		o ? o->role : "?", level, index, status, out_count);
}

static int cmd_aes70_trace(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 2 || (strcmp(argv[1], "on") && strcmp(argv[1], "off"))) {
		shell_print(sh, "usage: aes70 trace on|off");
		return -EINVAL;
	}
	aes70_core_set_trace(strcmp(argv[1], "on") == 0 ? trace_cmd : NULL);
	shell_print(sh, "command trace %s", argv[1]);
	return 0;
}

static int cmd_aes70_objects(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc); ARG_UNUSED(argv);
	for (int i = 0; ; i++) {
		const struct oca_object *o = aes70_core_object_at(i);

		if (o == NULL) {
			break;
		}
		shell_print(sh, "%5u  owner %5u  %-22s %s", o->ono, o->owner,
			    o->kind->name, o->role);
	}
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(aes70_cmds,
	SHELL_CMD(status, NULL, "Server state and connected controllers",
		  cmd_aes70_status),
	SHELL_CMD(objects, NULL, "List the AES70 object tree",
		  cmd_aes70_objects),
	SHELL_CMD_ARG(trace, NULL, "Log every command: aes70 trace on|off",
		      cmd_aes70_trace, 2, 0),
	SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(aes70, &aes70_cmds, "AES70 (OCA) control", NULL);
#endif /* CONFIG_SHELL */
