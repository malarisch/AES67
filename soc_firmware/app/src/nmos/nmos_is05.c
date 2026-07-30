/*
 * NMOS IS-05 v1.1 Connection API — connection state and activation.
 *
 * Owns the staged/active parameter sets for every Sender (TX slot) and
 * Receiver (RX slot) and projects activations into aes67_conn, which
 * writes the FPGA tables and handles IGMP. RTP core parameter set only
 * (no FEC/RTCP, single leg — no SMPTE 2022-7).
 *
 * Threading: staging and immediate activations run on the HTTP server
 * thread (same as the REST API's FPGA accesses); scheduled activations
 * fire from a small worker thread that polls the staged store. All
 * state lives behind is05_lock.
 *
 * IS-04 coupling: applying an activation always goes through
 * aes67_conn_configure_*, whose observers bump the IS-04 version
 * timestamps and DNS-SD ver_* counters — including re-activations with
 * unchanged parameters (the spec requires a version bump even then).
 * nmos_node.c pulls subscription state (sender_id/receiver_id +
 * master_enable) from here via nmos_is05_sub().
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/socket.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "nmos.h"
#include "nmos_priv.h"
#include "nmos_json.h"
#include "../aes67_config.h"
#include "../aes67_conn.h"
#include "../aes67_sdp_utils.h"

LOG_MODULE_REGISTER(nmos_is05, LOG_LEVEL_INF);

#define IP_STR_MAX     16
#define TAI_STR_MAX    28
#define IS05_SDP_MAX   1200

enum act_mode {
	ACT_NONE = 0,
	ACT_IMMEDIATE,
	ACT_ABS,
	ACT_REL,
};

/* Pending staged activation. req_str keeps the client's exact
 * requested_time spelling so GET /staged echoes it verbatim. */
struct is05_act {
	uint8_t mode;                /* ACT_ABS / ACT_REL while pending */
	char req_str[TAI_STR_MAX];   /* "" = null */
	struct nmos_tai when;        /* absolute activation time */
};

/* Last completed activation, echoed on /active. */
struct is05_last_act {
	uint8_t mode;                /* ACT_NONE = never activated */
	char req_str[TAI_STR_MAX];
	struct nmos_tai when;
};

struct tx_params {
	char src_ip[IP_STR_MAX];     /* concrete or "" while auto */
	bool src_auto;
	char dst_ip[IP_STR_MAX];
	bool dst_auto;
	uint16_t src_port;
	bool src_port_auto;
	uint16_t dst_port;
	bool dst_port_auto;
	bool rtp_enabled;
};

struct rx_params {
	char src_ip[IP_STR_MAX];     /* "" = null */
	char mcast_ip[IP_STR_MAX];   /* "" = null */
	char iface_ip[IP_STR_MAX];
	bool iface_auto;
	uint16_t dst_port;
	bool dst_port_auto;
	bool rtp_enabled;
};

struct is05_sender {
	/* staged */
	char receiver_id[NMOS_UUID_STR_LEN]; /* "" = null */
	bool master_enable;
	struct is05_act act;
	struct tx_params p;
	/* active */
	char a_receiver_id[NMOS_UUID_STR_LEN];
	bool a_master_enable;
	struct is05_last_act a_act;
	struct tx_params a_p;
};

struct is05_receiver {
	/* staged */
	char sender_id[NMOS_UUID_STR_LEN];
	bool master_enable;
	struct is05_act act;
	struct rx_params p;
	char tf_data[IS05_SDP_MAX];  /* "" = null */
	char tf_type[24];
	struct aes67_sdp_parsed sdp; /* media info from the transport file */
	bool have_sdp;
	/* active */
	char a_sender_id[NMOS_UUID_STR_LEN];
	bool a_master_enable;
	struct is05_last_act a_act;
	struct rx_params a_p;
	char a_tf_data[IS05_SDP_MAX];
	char a_tf_type[24];
	struct aes67_sdp_parsed a_sdp;
	bool a_have_sdp;
};

static NMOS_BIG_BSS struct is05_sender senders[AES67_MAX_TX_STREAMS];
static NMOS_BIG_BSS struct is05_receiver receivers[AES67_MAX_RX_STREAMS];

static K_MUTEX_DEFINE(is05_lock);
static K_SEM_DEFINE(act_sem, 0, 1);

/* Set while we are the ones writing aes67_conn, so the change observers
 * in nmos_node.c still fire (version bumps) but our own state refresh
 * does not overwrite the freshly applied parameters. */
static bool applying;

/* ================================================================
 * Small helpers
 * ================================================================ */

static void tai_str(char *buf, size_t sz, const struct nmos_tai *t)
{
	snprintf(buf, sz, "%llu:%09u", (unsigned long long)t->sec, t->nsec);
}

static bool tai_parse(const char *s, struct nmos_tai *out)
{
	char *end;
	unsigned long long sec = strtoull(s, &end, 10);

	if (end == s || *end != ':') {
		return false;
	}

	const char *ns_start = end + 1;
	unsigned long nsec = strtoul(ns_start, &end, 10);

	if (end == ns_start || *end != '\0' || nsec >= 1000000000UL) {
		return false;
	}
	out->sec = sec;
	out->nsec = (uint32_t)nsec;
	return true;
}

static bool tai_due(const struct nmos_tai *when, const struct nmos_tai *now)
{
	return now->sec > when->sec ||
	       (now->sec == when->sec && now->nsec >= when->nsec);
}

static bool valid_ipv4(const char *s)
{
	struct in_addr a;

	return zsock_inet_pton(AF_INET, s, &a) == 1;
}

static bool is_mcast(const struct in_addr *a)
{
	return (ntohl(a->s_addr) >> 28) == 0xe;
}

static uint16_t cfg_default_port(void)
{
	uint16_t port;

	aes67_config_lock();
	port = aes67_config_get()->default_port;
	aes67_config_unlock();
	return port != 0 ? port : AES67_DEFAULT_PORT;
}

/* Default TX destination for slot idx: the configured base multicast
 * address with the last octet offset by the slot index. */
static void default_tx_dst(int idx, char *buf, size_t sz)
{
	struct in_addr a;
	char base[IP_STR_MAX];

	aes67_config_lock();
	strncpy(base, aes67_config_get()->default_mcast_addr, sizeof(base) - 1);
	base[sizeof(base) - 1] = '\0';
	aes67_config_unlock();

	if (zsock_inet_pton(AF_INET, base, &a) != 1) {
		(void)zsock_inet_pton(AF_INET, AES67_DEFAULT_MCAST_ADDR, &a);
	}

	uint32_t host = ntohl(a.s_addr);

	host = (host & 0xffffff00U) | (((host & 0xffU) + (uint32_t)idx) & 0xffU);
	a.s_addr = htonl(host);
	zsock_inet_ntop(AF_INET, &a, buf, sz);
}

/* ================================================================
 * Boot state / refresh from device state
 * ================================================================ */

static void sender_defaults(struct is05_sender *s)
{
	memset(s, 0, sizeof(*s));
	s->p.src_auto = true;
	s->p.dst_auto = true;
	s->p.src_port_auto = true;
	s->p.dst_port_auto = true;
	s->p.rtp_enabled = true;
	s->a_p = s->p;
}

static void receiver_defaults(struct is05_receiver *r)
{
	memset(r, 0, sizeof(*r));
	r->p.iface_auto = true;
	strncpy(r->p.iface_ip, "auto", sizeof(r->p.iface_ip) - 1);
	r->p.dst_port_auto = true;
	r->p.rtp_enabled = true;
	r->a_p = r->p;
}

/* Mirror externally applied device state (web UI, shell, config restore)
 * into the ACTIVE side, so /active never lies about what the hardware
 * is doing. Staged parameters are left alone unless the slot state was
 * never touched via IS-05. */
static void resolve_tx(const struct tx_params *in, struct tx_params *out,
		       int idx);
static void resolve_rx(const struct rx_params *in, struct rx_params *out);
static const char *mode_name(uint8_t mode);

static void refresh_tx_from_conn(int idx)
{
	struct aes67_tx_stream tx;
	struct is05_sender *s = &senders[idx];
	bool active = aes67_conn_copy_tx_stream((uint8_t)idx, &tx);

	s->a_master_enable = active;
	s->a_p.rtp_enabled = active;
	if (active) {
		zsock_inet_ntop(AF_INET, &tx.dst_ip, s->a_p.dst_ip,
				sizeof(s->a_p.dst_ip));
		s->a_p.dst_auto = false;
		nmos_ip_str(s->a_p.src_ip, sizeof(s->a_p.src_ip));
		s->a_p.src_auto = false;
		s->a_p.src_port = cfg_default_port();
		s->a_p.src_port_auto = false;
		s->a_p.dst_port = cfg_default_port();
		s->a_p.dst_port_auto = false;
	} else {
		/* /active must never show "auto" (IS-05-01 test_11_01) —
		 * resolve any leftover defaults. */
		struct tx_params tmp;

		resolve_tx(&s->a_p, &tmp, idx);
		s->a_p = tmp;
		s->a_p.rtp_enabled = false;
	}
	/* An externally created connection has no NMOS receiver. */
	s->a_receiver_id[0] = '\0';
}

static void refresh_rx_from_conn(int idx)
{
	const struct aes67_rx_stream *rx = &aes67_conn_get_rx_streams()[idx];
	struct is05_receiver *r = &receivers[idx];
	bool active = rx->active;

	r->a_master_enable = active;
	r->a_p.rtp_enabled = active;
	if (active) {
		char dst[IP_STR_MAX];
		struct in_addr dst_ip = rx->dst_ip;

		zsock_inet_ntop(AF_INET, &dst_ip, dst, sizeof(dst));
		if (is_mcast(&dst_ip)) {
			strncpy(r->a_p.mcast_ip, dst, sizeof(r->a_p.mcast_ip) - 1);
			nmos_ip_str(r->a_p.iface_ip, sizeof(r->a_p.iface_ip));
		} else {
			r->a_p.mcast_ip[0] = '\0';
			strncpy(r->a_p.iface_ip, dst, sizeof(r->a_p.iface_ip) - 1);
		}
		r->a_p.iface_auto = false;
		r->a_p.dst_port = rx->dst_port;
		r->a_p.dst_port_auto = false;
		if (rx->sender_ip.s_addr != 0) {
			struct in_addr sip = rx->sender_ip;

			zsock_inet_ntop(AF_INET, &sip, r->a_p.src_ip,
					sizeof(r->a_p.src_ip));
		} else {
			r->a_p.src_ip[0] = '\0';
		}
	} else {
		struct rx_params tmp;

		resolve_rx(&r->a_p, &tmp);
		r->a_p = tmp;
		r->a_p.rtp_enabled = false;
	}
	r->a_sender_id[0] = '\0';
}

/* Zephyr mutexes are recursive: when the notification comes from our
 * own apply path the lock is already held by this thread and `applying`
 * suppresses the refresh; external configuration (web UI, shell, SAP)
 * waits for the lock and mirrors the new device state. */
static void tx_observer(uint8_t stream_id)
{
	if (stream_id >= AES67_MAX_TX_STREAMS) {
		return;
	}
	k_mutex_lock(&is05_lock, K_FOREVER);
	if (!applying) {
		refresh_tx_from_conn(stream_id);
	}
	k_mutex_unlock(&is05_lock);
}

static void rx_observer(uint8_t stream_id)
{
	if (stream_id >= AES67_MAX_RX_STREAMS) {
		return;
	}
	k_mutex_lock(&is05_lock, K_FOREVER);
	if (!applying) {
		refresh_rx_from_conn(stream_id);
	}
	k_mutex_unlock(&is05_lock);
}

/* ================================================================
 * Auto resolution + applying to the device
 * ================================================================ */

static void resolve_tx(const struct tx_params *in, struct tx_params *out,
		       int idx)
{
	*out = *in;
	if (in->src_auto) {
		nmos_ip_str(out->src_ip, sizeof(out->src_ip));
		out->src_auto = false;
	}
	if (in->dst_auto) {
		struct aes67_tx_stream tx;

		if (aes67_conn_copy_tx_stream((uint8_t)idx, &tx)) {
			struct in_addr d = tx.dst_ip;

			zsock_inet_ntop(AF_INET, &d, out->dst_ip,
					sizeof(out->dst_ip));
		} else {
			default_tx_dst(idx, out->dst_ip, sizeof(out->dst_ip));
		}
		out->dst_auto = false;
	}
	if (in->src_port_auto) {
		out->src_port = cfg_default_port();
		out->src_port_auto = false;
	}
	if (in->dst_port_auto) {
		out->dst_port = cfg_default_port();
		out->dst_port_auto = false;
	}
}

static void resolve_rx(const struct rx_params *in, struct rx_params *out)
{
	*out = *in;
	if (in->iface_auto) {
		nmos_ip_str(out->iface_ip, sizeof(out->iface_ip));
		out->iface_auto = false;
	}
	if (in->dst_port_auto) {
		out->dst_port = AES67_DEFAULT_PORT;
		out->dst_port_auto = false;
	}
}

/* Apply the staged sender state; returns 0 or negative errno. Caller
 * holds is05_lock. */
static int apply_sender(int idx)
{
	struct is05_sender *s = &senders[idx];
	struct tx_params rp;
	struct aes67_tx_stream tx;
	int ret;

	resolve_tx(&s->p, &rp, idx);

	bool on = s->master_enable && s->p.rtp_enabled;

	/* Channel layout / packet timing are IS-04 flow domain: keep the
	 * slot's existing configuration (or the device defaults). */
	bool was_active = nmos_tx_snapshot(idx, &tx);

	applying = true;
	if (on) {
		struct in_addr dst;

		if (zsock_inet_pton(AF_INET, rp.dst_ip, &dst) != 1) {
			applying = false;
			return -EINVAL;
		}

		uint16_t spp = tx.samples_per_packet;

		if (spp == 0) {
			aes67_config_lock();
			spp = aes67_config_get()->default_samples_per_pkt;
			aes67_config_unlock();
			if (spp == 0) {
				spp = AES67_DEFAULT_SAMPLES_PER_PKT;
			}
		}
		ret = aes67_conn_configure_tx_stream((uint8_t)idx, &dst,
						     tx.channel_count,
						     (uint8_t)spp,
						     tx.ch_ids,
						     tx.channel_count,
						     was_active ? tx.ssrc : 0,
						     tx.name);
	} else {
		struct in_addr zero = {0};
		uint8_t ch_ids[AES67_MAX_CH_PER_STREAM] = {0};

		ret = aes67_conn_configure_tx_stream((uint8_t)idx, &zero, 0,
						     tx.samples_per_packet,
						     ch_ids, 0, 0, NULL);
	}
	applying = false;

	if (ret < 0) {
		return ret;
	}

	s->a_p = rp;
	s->a_p.rtp_enabled = s->p.rtp_enabled;
	s->a_master_enable = s->master_enable;
	strcpy(s->a_receiver_id, s->receiver_id);
	return 0;
}

/* Apply the staged receiver state; caller holds is05_lock. */
static int apply_receiver(int idx)
{
	struct is05_receiver *r = &receivers[idx];
	struct rx_params rp;
	int ret;

	resolve_rx(&r->p, &rp);

	bool on = r->master_enable && r->p.rtp_enabled;

	applying = true;
	if (on) {
		struct in_addr dst, sender_ip = {0};
		uint8_t ch_map[AES67_MAX_CH_PER_STREAM];
		uint8_t channels = r->have_sdp ? r->sdp.channels : 0;
		uint16_t spc = r->have_sdp ? r->sdp.samples_per_packet : 0;
		const char *name = (r->have_sdp && r->sdp.name[0] != '\0')
					   ? r->sdp.name : NULL;

		/* Multicast group if given, otherwise unicast to our own
		 * interface address. */
		const char *dst_str = rp.mcast_ip[0] != '\0' ? rp.mcast_ip
							     : rp.iface_ip;

		if (zsock_inet_pton(AF_INET, dst_str, &dst) != 1 ||
		    dst.s_addr == 0) {
			applying = false;
			return -EINVAL;
		}
		if (rp.src_ip[0] != '\0') {
			(void)zsock_inet_pton(AF_INET, rp.src_ip, &sender_ip);
		}

		if (channels == 0 || channels > AES67_MAX_CH_PER_STREAM) {
			aes67_config_lock();
			channels = aes67_config_get()->default_channels;
			aes67_config_unlock();
			if (channels == 0 || channels > AES67_MAX_CH_PER_STREAM) {
				channels = AES67_DEFAULT_CHANNELS;
			}
		}
		if (spc == 0 || spc > 255) {
			spc = AES67_DEFAULT_SAMPLES_PER_PKT;
		}
		for (uint8_t i = 0; i < AES67_MAX_CH_PER_STREAM; i++) {
			ch_map[i] = i;
		}

		ret = aes67_conn_configure_rx_stream((uint8_t)idx, &dst,
						     rp.dst_port, ch_map,
						     channels, 0, (uint8_t)spc,
						     name,
						     r->have_sdp ?
							r->sdp.origin_name : NULL,
						     sender_ip.s_addr != 0 ?
							&sender_ip : NULL);
	} else {
		struct in_addr zero = {0};
		uint8_t ch_map[AES67_MAX_CH_PER_STREAM] = {0};

		ret = aes67_conn_configure_rx_stream((uint8_t)idx, &zero, 0,
						     ch_map, 1, 0,
						     AES67_DEFAULT_SAMPLES_PER_PKT,
						     NULL, NULL, NULL);
	}
	applying = false;

	if (ret < 0) {
		return ret;
	}

	r->a_p = rp;
	r->a_p.rtp_enabled = r->p.rtp_enabled;
	r->a_master_enable = r->master_enable;
	strcpy(r->a_sender_id, r->sender_id);
	memcpy(r->a_tf_data, r->tf_data, sizeof(r->a_tf_data));
	memcpy(r->a_tf_type, r->tf_type, sizeof(r->a_tf_type));
	r->a_sdp = r->sdp;
	r->a_have_sdp = r->have_sdp;
	return 0;
}

/* Complete an activation: apply, move the pending record to last-act,
 * clear the staged activation. Caller holds is05_lock. */
static int fire_activation(bool sender, int idx, uint8_t mode,
			   const char *req_str, const struct nmos_tai *when)
{
	int ret = sender ? apply_sender(idx) : apply_receiver(idx);
	struct is05_last_act *last = sender ? &senders[idx].a_act
					    : &receivers[idx].a_act;
	struct is05_act *act = sender ? &senders[idx].act
				      : &receivers[idx].act;

	if (ret == 0) {
		last->mode = mode;
		strncpy(last->req_str, req_str, sizeof(last->req_str) - 1);
		last->req_str[sizeof(last->req_str) - 1] = '\0';
		last->when = *when;
	}
	memset(act, 0, sizeof(*act));
	return ret;
}

/* ================================================================
 * Scheduled-activation worker
 * ================================================================ */

static void act_worker(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	for (;;) {
		bool pending = false;

		k_mutex_lock(&is05_lock, K_FOREVER);

		struct nmos_tai now;

		nmos_tai_now(&now);
		for (int i = 0; i < nmos_tx_count(); i++) {
			struct is05_act *act = &senders[i].act;

			if (act->mode != ACT_ABS && act->mode != ACT_REL) {
				continue;
			}
			if (tai_due(&act->when, &now)) {
				(void)fire_activation(true, i, act->mode,
						      act->req_str, &act->when);
			} else {
				pending = true;
			}
		}
		for (int i = 0; i < nmos_rx_count(); i++) {
			struct is05_act *act = &receivers[i].act;

			if (act->mode != ACT_ABS && act->mode != ACT_REL) {
				continue;
			}
			if (tai_due(&act->when, &now)) {
				(void)fire_activation(false, i, act->mode,
						      act->req_str, &act->when);
			} else {
				pending = true;
			}
		}
		k_mutex_unlock(&is05_lock);

		/* Poll while anything is pending — robust against wallclock
		 * steps from PTP resyncs, and 100 ms is well inside the
		 * tolerance controllers apply to scheduled activations. */
		k_sem_take(&act_sem, pending ? K_MSEC(100) : K_FOREVER);
	}
}

K_THREAD_STACK_DEFINE(is05_stack, 4096);
static struct k_thread is05_thread;

/* ================================================================
 * Staging (PATCH /staged, bulk items)
 * ================================================================ */

#define HTTP_OK        200
#define HTTP_ACCEPTED  202
#define HTTP_BAD       400
#define HTTP_LOCKED    423
#define HTTP_ERROR     500

struct stage_ctx {
	const struct nj_node *pool;
	const char **errmsg;
};

static int fail(struct stage_ctx *c, const char *msg)
{
	*c->errmsg = msg;
	return HTTP_BAD;
}

/* "auto" | integer 1..65535 */
static bool parse_port(const struct nj_node *n, uint16_t *port, bool *is_auto)
{
	if (nj_streq(n, "auto")) {
		*is_auto = true;
		return true;
	}
	if (n->type == NJ_NUM && n->num >= 1 && n->num <= 65535) {
		*port = (uint16_t)n->num;
		*is_auto = false;
		return true;
	}
	return false;
}

/* string IPv4 into a fixed buffer; allow_auto/allow_null widen the set. */
static bool parse_ip(const struct nj_node *n, char *buf, size_t sz,
		     bool *is_auto, bool allow_auto, bool allow_null)
{
	if (allow_null && n->type == NJ_NULL) {
		buf[0] = '\0';
		if (is_auto) {
			*is_auto = false;
		}
		return true;
	}
	if (allow_auto && nj_streq(n, "auto")) {
		if (is_auto) {
			*is_auto = true;
		}
		strncpy(buf, "auto", sz - 1);
		buf[sz - 1] = '\0';
		return true;
	}
	if (n->type == NJ_STR && n->val_len < sz) {
		char tmp[IP_STR_MAX];

		if (nj_strcpy(n, tmp, sizeof(tmp)) < 0 || !valid_ipv4(tmp)) {
			return false;
		}
		strcpy(buf, tmp);
		if (is_auto) {
			*is_auto = false;
		}
		return true;
	}
	return false;
}

static int stage_tx_params(struct stage_ctx *c, const struct nj_node *obj,
			   struct tx_params *p)
{
	uint16_t defp = cfg_default_port();

	for (int i = obj->first_child; i >= 0; i = c->pool[i].next) {
		const struct nj_node *n = &c->pool[i];

		if (n->key_len == 9 && memcmp(n->key, "source_ip", 9) == 0) {
			char node_ip[IP_STR_MAX];

			if (!parse_ip(n, p->src_ip, sizeof(p->src_ip),
				      &p->src_auto, true, false)) {
				return fail(c, "invalid source_ip");
			}
			nmos_ip_str(node_ip, sizeof(node_ip));
			if (!p->src_auto && strcmp(p->src_ip, node_ip) != 0) {
				return fail(c, "source_ip not offered by this sender");
			}
		} else if (n->key_len == 14 &&
			   memcmp(n->key, "destination_ip", 14) == 0) {
			if (!parse_ip(n, p->dst_ip, sizeof(p->dst_ip),
				      &p->dst_auto, true, false)) {
				return fail(c, "invalid destination_ip");
			}
		} else if (n->key_len == 11 &&
			   memcmp(n->key, "source_port", 11) == 0) {
			if (!parse_port(n, &p->src_port, &p->src_port_auto)) {
				return fail(c, "invalid source_port");
			}
			if (!p->src_port_auto && p->src_port != defp) {
				return fail(c, "source_port outside constraints");
			}
		} else if (n->key_len == 16 &&
			   memcmp(n->key, "destination_port", 16) == 0) {
			if (!parse_port(n, &p->dst_port, &p->dst_port_auto)) {
				return fail(c, "invalid destination_port");
			}
			if (!p->dst_port_auto && p->dst_port != defp) {
				return fail(c, "destination_port outside constraints");
			}
		} else if (n->key_len == 11 &&
			   memcmp(n->key, "rtp_enabled", 11) == 0) {
			if (n->type != NJ_BOOL) {
				return fail(c, "invalid rtp_enabled");
			}
			p->rtp_enabled = n->bval;
		} else {
			return fail(c, "unsupported transport parameter");
		}
	}
	return HTTP_OK;
}

static int stage_rx_params(struct stage_ctx *c, const struct nj_node *obj,
			   struct rx_params *p)
{
	for (int i = obj->first_child; i >= 0; i = c->pool[i].next) {
		const struct nj_node *n = &c->pool[i];

		if (n->key_len == 9 && memcmp(n->key, "source_ip", 9) == 0) {
			if (!parse_ip(n, p->src_ip, sizeof(p->src_ip),
				      NULL, false, true)) {
				return fail(c, "invalid source_ip");
			}
		} else if (n->key_len == 12 &&
			   memcmp(n->key, "multicast_ip", 12) == 0) {
			if (!parse_ip(n, p->mcast_ip, sizeof(p->mcast_ip),
				      NULL, false, true)) {
				return fail(c, "invalid multicast_ip");
			}
		} else if (n->key_len == 12 &&
			   memcmp(n->key, "interface_ip", 12) == 0) {
			char node_ip[IP_STR_MAX];

			if (!parse_ip(n, p->iface_ip, sizeof(p->iface_ip),
				      &p->iface_auto, true, false)) {
				return fail(c, "invalid interface_ip");
			}
			nmos_ip_str(node_ip, sizeof(node_ip));
			if (!p->iface_auto &&
			    strcmp(p->iface_ip, node_ip) != 0) {
				return fail(c, "interface_ip is not a local interface");
			}
		} else if (n->key_len == 16 &&
			   memcmp(n->key, "destination_port", 16) == 0) {
			if (!parse_port(n, &p->dst_port, &p->dst_port_auto)) {
				return fail(c, "invalid destination_port");
			}
		} else if (n->key_len == 11 &&
			   memcmp(n->key, "rtp_enabled", 11) == 0) {
			if (n->type != NJ_BOOL) {
				return fail(c, "invalid rtp_enabled");
			}
			p->rtp_enabled = n->bval;
		} else {
			return fail(c, "unsupported transport parameter");
		}
	}
	return HTTP_OK;
}

/* Populate staged receiver params from a staged SDP transport file.
 * transport_params in the same request take priority (they are applied
 * after this). */
static int stage_transport_file(struct stage_ctx *c, struct is05_receiver *r,
				const struct nj_node *tf)
{
	const struct nj_node *data = nj_get(c->pool, tf, "data");
	const struct nj_node *type = nj_get(c->pool, tf, "type");

	if (data == NULL && type == NULL) {
		return HTTP_OK;
	}

	if ((data != NULL && data->type == NJ_NULL) ||
	    (type != NULL && type->type == NJ_NULL)) {
		/* Clearing the file leaves transport_params untouched. */
		r->tf_data[0] = '\0';
		r->tf_type[0] = '\0';
		r->have_sdp = false;
		return HTTP_OK;
	}
	if (data == NULL || type == NULL) {
		return fail(c, "transport_file requires both data and type");
	}
	if (!nj_streq(type, "application/sdp")) {
		return fail(c, "unsupported transport file type");
	}

	int len = nj_strcpy(data, r->tf_data, sizeof(r->tf_data));

	if (len < 0) {
		return fail(c, "transport file too large");
	}
	strncpy(r->tf_type, "application/sdp", sizeof(r->tf_type) - 1);

	struct aes67_sdp_parsed sdp;

	if (aes67_sdp_parse(r->tf_data, (size_t)len, &sdp) < 0) {
		*c->errmsg = "transport file could not be parsed";
		return HTTP_ERROR;
	}
	r->sdp = sdp;
	r->have_sdp = true;

	/* Project the file's connection info into the transport params. */
	if (sdp.connection_addr.s_addr != 0) {
		struct in_addr conn = sdp.connection_addr;
		char buf[IP_STR_MAX];

		zsock_inet_ntop(AF_INET, &conn, buf, sizeof(buf));
		if (is_mcast(&conn)) {
			strcpy(r->p.mcast_ip, buf);
			strncpy(r->p.iface_ip, "auto",
				sizeof(r->p.iface_ip) - 1);
			r->p.iface_auto = true;
		} else {
			r->p.mcast_ip[0] = '\0';
			strcpy(r->p.iface_ip, buf);
			r->p.iface_auto = false;
		}
	}
	if (sdp.origin_addr.s_addr != 0) {
		struct in_addr orig = sdp.origin_addr;

		zsock_inet_ntop(AF_INET, &orig, r->p.src_ip,
				sizeof(r->p.src_ip));
	}
	if (sdp.port != 0) {
		r->p.dst_port = sdp.port;
		r->p.dst_port_auto = false;
	}
	r->p.rtp_enabled = true;
	return HTTP_OK;
}

static bool parse_res_id(const struct nj_node *n, char *out, size_t sz)
{
	if (n->type == NJ_NULL) {
		out[0] = '\0';
		return true;
	}
	if (n->type == NJ_STR && n->val_len == 36 && n->val_len < sz) {
		return nj_strcpy(n, out, sz) == 36;
	}
	return false;
}

int nmos_is05_stage(bool sender, int idx, const struct nj_node *pool,
		    const struct nj_node *req, struct is05_act_echo *echo,
		    const char **errmsg)
{
	struct stage_ctx c = {.pool = pool, .errmsg = errmsg};
	int status = HTTP_OK;

	*errmsg = "";
	memset(echo, 0, sizeof(*echo));

	if (req == NULL || req->type != NJ_OBJ) {
		*errmsg = "request body must be a JSON object";
		return HTTP_BAD;
	}

	/* Validate top-level keys before touching state. */
	for (int i = req->first_child; i >= 0; i = pool[i].next) {
		const struct nj_node *n = &pool[i];
		bool known =
			(n->key_len == 13 &&
			 memcmp(n->key, "master_enable", 13) == 0) ||
			(n->key_len == 10 &&
			 memcmp(n->key, "activation", 10) == 0) ||
			(n->key_len == 16 &&
			 memcmp(n->key, "transport_params", 16) == 0) ||
			(sender && n->key_len == 11 &&
			 memcmp(n->key, "receiver_id", 11) == 0) ||
			(!sender && n->key_len == 9 &&
			 memcmp(n->key, "sender_id", 9) == 0) ||
			(!sender && n->key_len == 14 &&
			 memcmp(n->key, "transport_file", 14) == 0);

		if (!known) {
			*errmsg = "unknown property in request";
			return HTTP_BAD;
		}
	}

	const struct nj_node *act = nj_get(pool, req, "activation");
	const struct nj_node *mode = act ? nj_get(pool, act, "mode") : NULL;
	const struct nj_node *rtime = act ? nj_get(pool, act, "requested_time")
					  : NULL;

	if (act != NULL && (act->type != NJ_OBJ || mode == NULL)) {
		*errmsg = "activation requires a mode";
		return HTTP_BAD;
	}

	uint8_t new_mode = ACT_NONE;
	bool cancel = false;

	if (mode != NULL) {
		if (mode->type == NJ_NULL) {
			cancel = true;
		} else if (nj_streq(mode, "activate_immediate")) {
			new_mode = ACT_IMMEDIATE;
		} else if (nj_streq(mode, "activate_scheduled_absolute")) {
			new_mode = ACT_ABS;
		} else if (nj_streq(mode, "activate_scheduled_relative")) {
			new_mode = ACT_REL;
		} else {
			*errmsg = "invalid activation mode";
			return HTTP_BAD;
		}
	}

	char req_str[TAI_STR_MAX] = "";
	struct nmos_tai req_time = {0};

	if (new_mode == ACT_ABS || new_mode == ACT_REL) {
		if (rtime == NULL || rtime->type != NJ_STR ||
		    nj_strcpy(rtime, req_str, sizeof(req_str)) < 0 ||
		    !tai_parse(req_str, &req_time)) {
			*errmsg = "scheduled activation requires a valid requested_time";
			return HTTP_BAD;
		}
	} else if (rtime != NULL && rtime->type != NJ_NULL) {
		*errmsg = "requested_time must be null for this mode";
		return HTTP_BAD;
	}

	k_mutex_lock(&is05_lock, K_FOREVER);

	struct is05_act *pending = sender ? &senders[idx].act
					  : &receivers[idx].act;

	/* A scheduled activation locks the resource; only an explicit
	 * mode:null PATCH unlocks (and may carry other changes). */
	if (pending->mode != ACT_NONE && !cancel) {
		struct nmos_tai now;

		nmos_tai_now(&now);

		int64_t due_s = (int64_t)pending->when.sec - (int64_t)now.sec;

		/* A large positive delta usually means a stale activation
		 * scheduled against a mismatched controller clock — it will
		 * hold the 423 lock until it fires or a mode:null cancels. */
		LOG_WRN("IS-05: %s %d PATCH rejected 423 - %s activation "
			"pending, due in %lld s (cancel with activation mode null)",
			sender ? "sender" : "receiver", idx,
			mode_name(pending->mode), (long long)due_s);
		k_mutex_unlock(&is05_lock);
		*errmsg = "resource locked by a pending activation";
		return HTTP_LOCKED;
	}

	/* Stage into copies first so a 400 leaves the store untouched. */
	if (sender) {
		struct is05_sender *s = &senders[idx];
		struct tx_params p = s->p;
		char rid[NMOS_UUID_STR_LEN];
		bool me = s->master_enable;

		strcpy(rid, s->receiver_id);

		const struct nj_node *n;

		if ((n = nj_get(pool, req, "receiver_id")) != NULL &&
		    !parse_res_id(n, rid, sizeof(rid))) {
			status = fail(&c, "invalid receiver_id");
		}
		if (status == HTTP_OK &&
		    (n = nj_get(pool, req, "master_enable")) != NULL) {
			if (n->type != NJ_BOOL) {
				status = fail(&c, "invalid master_enable");
			} else {
				me = n->bval;
			}
		}
		if (status == HTTP_OK &&
		    (n = nj_get(pool, req, "transport_params")) != NULL) {
			const struct nj_node *leg = nj_item(pool, n, 0);

			if (n->type != NJ_ARR || nj_count(pool, n) != 1 ||
			    leg == NULL || leg->type != NJ_OBJ) {
				status = fail(&c, "transport_params must hold one leg");
			} else {
				status = stage_tx_params(&c, leg, &p);
			}
		}
		if (status == HTTP_OK) {
			s->p = p;
			s->master_enable = me;
			strcpy(s->receiver_id, rid);
		}
	} else {
		struct is05_receiver *r = &receivers[idx];
		char sid[NMOS_UUID_STR_LEN];
		bool me = r->master_enable;
		const struct nj_node *tp_leg = NULL;

		strcpy(sid, r->sender_id);

		const struct nj_node *n;

		if ((n = nj_get(pool, req, "sender_id")) != NULL &&
		    !parse_res_id(n, sid, sizeof(sid))) {
			status = fail(&c, "invalid sender_id");
		}
		if (status == HTTP_OK &&
		    (n = nj_get(pool, req, "master_enable")) != NULL) {
			if (n->type != NJ_BOOL) {
				status = fail(&c, "invalid master_enable");
			} else {
				me = n->bval;
			}
		}
		/* Dry-run the transport params against a scratch copy first,
		 * so a 400 leaves both params and transport file untouched. */
		if (status == HTTP_OK &&
		    (n = nj_get(pool, req, "transport_params")) != NULL) {
			struct rx_params scratch = r->p;

			tp_leg = nj_item(pool, n, 0);
			if (n->type != NJ_ARR || nj_count(pool, n) != 1 ||
			    tp_leg == NULL || tp_leg->type != NJ_OBJ) {
				status = fail(&c, "transport_params must hold one leg");
				tp_leg = NULL;
			} else {
				status = stage_rx_params(&c, tp_leg, &scratch);
			}
		}
		if (status == HTTP_OK &&
		    (n = nj_get(pool, req, "transport_file")) != NULL) {
			if (n->type != NJ_OBJ) {
				status = fail(&c, "invalid transport_file");
			} else {
				status = stage_transport_file(&c, r, n);
			}
		}
		if (status == HTTP_OK && tp_leg != NULL) {
			/* Params in the request override the transport file
			 * (already validated above — cannot fail). */
			(void)stage_rx_params(&c, tp_leg, &r->p);
		}
		if (status == HTTP_OK) {
			r->master_enable = me;
			strcpy(r->sender_id, sid);
		}
	}

	if (status != HTTP_OK) {
		k_mutex_unlock(&is05_lock);
		return status;
	}

	/* Activation handling. */
	if (cancel) {
		memset(pending, 0, sizeof(*pending));
	} else if (new_mode == ACT_IMMEDIATE) {
		struct nmos_tai now;

		nmos_tai_now(&now);
		if (fire_activation(sender, idx, ACT_IMMEDIATE, "", &now) < 0) {
			k_mutex_unlock(&is05_lock);
			*errmsg = "activation failed";
			return HTTP_ERROR;
		}
		echo->mode = ACT_IMMEDIATE;
		echo->has_time = true;
		echo->when = now;
	} else if (new_mode == ACT_ABS || new_mode == ACT_REL) {
		struct nmos_tai now, when = req_time;

		nmos_tai_now(&now);
		if (new_mode == ACT_REL) {
			when.sec += now.sec;
			when.nsec += now.nsec;
			if (when.nsec >= 1000000000U) {
				when.nsec -= 1000000000U;
				when.sec++;
			}
		}
		int64_t delta_ms = ((int64_t)when.sec - (int64_t)now.sec) * 1000 +
				   ((int64_t)when.nsec - (int64_t)now.nsec) /
					   1000000;

		LOG_INF("IS-05: %s %d scheduled activation in %lld ms "
			"(wallclock %llu s)",
			sender ? "sender" : "receiver", idx,
			(long long)delta_ms, (unsigned long long)now.sec);
		/* Controllers schedule salvos at most a few seconds out. A
		 * large absolute delta almost always means the wallclock and
		 * the controller disagree on the TAI epoch (e.g. a grand-
		 * master distributing UTC: exactly 37 s behind). */
		if (new_mode == ACT_ABS &&
		    (delta_ms < -2000 || delta_ms > 30000)) {
			LOG_WRN("IS-05: absolute activation %lld s away - "
				"device TAI and controller clock disagree?",
				(long long)(delta_ms / 1000));
		}

		pending->mode = new_mode;
		strcpy(pending->req_str, req_str);
		pending->when = when;
		k_sem_give(&act_sem);

		echo->mode = new_mode;
		strcpy(echo->req_str, req_str);
		echo->has_req = true;
		echo->has_time = true;
		echo->when = when;
		status = HTTP_ACCEPTED;
	}

	k_mutex_unlock(&is05_lock);
	return status;
}

/* ================================================================
 * JSON builders (staged / active / constraints)
 * ================================================================ */

static void put_tai_or_null(struct json_out *jo, const char *key, bool present,
			    const struct nmos_tai *t)
{
	if (!present) {
		jo_key(jo, key);
		jo_raw(jo, "null,");
	} else {
		char buf[TAI_STR_MAX];

		tai_str(buf, sizeof(buf), t);
		jo_str(jo, key, buf);
	}
}

static void put_str_or_null(struct json_out *jo, const char *key,
			    const char *s)
{
	if (s[0] == '\0') {
		jo_key(jo, key);
		jo_raw(jo, "null,");
	} else {
		jo_str(jo, key, s);
	}
}

static const char *mode_name(uint8_t mode)
{
	switch (mode) {
	case ACT_IMMEDIATE:
		return "activate_immediate";
	case ACT_ABS:
		return "activate_scheduled_absolute";
	case ACT_REL:
		return "activate_scheduled_relative";
	default:
		return NULL;
	}
}

static void put_activation(struct json_out *jo, uint8_t mode,
			   const char *req_str, bool has_time,
			   const struct nmos_tai *when)
{
	const char *name = mode_name(mode);

	jo_key(jo, "activation");
	jo_obj_begin(jo);
	put_str_or_null(jo, "mode", name != NULL ? name : "");
	put_str_or_null(jo, "requested_time", req_str);
	put_tai_or_null(jo, "activation_time", has_time, when);
	jo_obj_end(jo);
}

static void put_port(struct json_out *jo, const char *key, bool is_auto,
		     uint16_t port)
{
	if (is_auto) {
		jo_str(jo, key, "auto");
	} else {
		jo_uint(jo, key, port);
	}
}

static void put_tx_params(struct json_out *jo, const struct tx_params *p)
{
	jo_key(jo, "transport_params");
	jo_arr_begin(jo);
	jo_obj_begin(jo);
	jo_str(jo, "source_ip", p->src_auto ? "auto" : p->src_ip);
	jo_str(jo, "destination_ip", p->dst_auto ? "auto" : p->dst_ip);
	put_port(jo, "source_port", p->src_port_auto, p->src_port);
	put_port(jo, "destination_port", p->dst_port_auto, p->dst_port);
	jo_bool(jo, "rtp_enabled", p->rtp_enabled);
	jo_obj_end(jo);
	jo_arr_end(jo);
}

static void put_rx_params(struct json_out *jo, const struct rx_params *p)
{
	jo_key(jo, "transport_params");
	jo_arr_begin(jo);
	jo_obj_begin(jo);
	put_str_or_null(jo, "source_ip", p->src_ip);
	put_str_or_null(jo, "multicast_ip", p->mcast_ip);
	jo_str(jo, "interface_ip", p->iface_auto ? "auto" : p->iface_ip);
	put_port(jo, "destination_port", p->dst_port_auto, p->dst_port);
	jo_bool(jo, "rtp_enabled", p->rtp_enabled);
	jo_obj_end(jo);
	jo_arr_end(jo);
}

static void put_transport_file(struct json_out *jo, const char *data,
			       const char *type)
{
	jo_key(jo, "transport_file");
	jo_obj_begin(jo);
	put_str_or_null(jo, "data", data);
	put_str_or_null(jo, "type", type);
	jo_obj_end(jo);
}

void nmos_is05_build_staged(struct json_out *jo, bool sender, int idx,
			    const struct is05_act_echo *echo)
{
	k_mutex_lock(&is05_lock, K_FOREVER);
	jo_obj_begin(jo);
	if (sender) {
		struct is05_sender *s = &senders[idx];

		put_str_or_null(jo, "receiver_id", s->receiver_id);
		jo_bool(jo, "master_enable", s->master_enable);
		if (echo != NULL) {
			put_activation(jo, echo->mode,
				       echo->has_req ? echo->req_str : "",
				       echo->has_time, &echo->when);
		} else {
			put_activation(jo, s->act.mode, s->act.req_str,
				       s->act.mode != ACT_NONE, &s->act.when);
		}
		put_tx_params(jo, &s->p);
	} else {
		struct is05_receiver *r = &receivers[idx];

		put_str_or_null(jo, "sender_id", r->sender_id);
		jo_bool(jo, "master_enable", r->master_enable);
		if (echo != NULL) {
			put_activation(jo, echo->mode,
				       echo->has_req ? echo->req_str : "",
				       echo->has_time, &echo->when);
		} else {
			put_activation(jo, r->act.mode, r->act.req_str,
				       r->act.mode != ACT_NONE, &r->act.when);
		}
		put_transport_file(jo, r->tf_data, r->tf_type);
		put_rx_params(jo, &r->p);
	}
	jo_obj_end(jo);
	k_mutex_unlock(&is05_lock);
}

void nmos_is05_build_active(struct json_out *jo, bool sender, int idx)
{
	k_mutex_lock(&is05_lock, K_FOREVER);
	jo_obj_begin(jo);
	if (sender) {
		struct is05_sender *s = &senders[idx];

		put_str_or_null(jo, "receiver_id", s->a_receiver_id);
		jo_bool(jo, "master_enable", s->a_master_enable);
		put_activation(jo, s->a_act.mode, s->a_act.req_str,
			       s->a_act.mode != ACT_NONE, &s->a_act.when);
		put_tx_params(jo, &s->a_p);
	} else {
		struct is05_receiver *r = &receivers[idx];

		put_str_or_null(jo, "sender_id", r->a_sender_id);
		jo_bool(jo, "master_enable", r->a_master_enable);
		put_activation(jo, r->a_act.mode, r->a_act.req_str,
			       r->a_act.mode != ACT_NONE, &r->a_act.when);
		put_transport_file(jo, r->a_tf_data, r->a_tf_type);
		put_rx_params(jo, &r->a_p);
	}
	jo_obj_end(jo);
	k_mutex_unlock(&is05_lock);
}

void nmos_is05_build_constraints(struct json_out *jo, bool sender, int idx)
{
	char ip[IP_STR_MAX];
	uint16_t port = cfg_default_port();

	ARG_UNUSED(idx);
	nmos_ip_str(ip, sizeof(ip));

	jo_arr_begin(jo);
	jo_obj_begin(jo);
	if (sender) {
		jo_key(jo, "source_ip");
		jo_obj_begin(jo);
		jo_key(jo, "enum");
		jo_arr_begin(jo);
		jo_fmt(jo, "\"%s\",\"auto\",", ip);
		jo_arr_end(jo);
		jo_obj_end(jo);

		jo_key(jo, "destination_ip");
		jo_obj_begin(jo);
		jo_obj_end(jo);

		jo_key(jo, "source_port");
		jo_obj_begin(jo);
		jo_key(jo, "enum");
		jo_arr_begin(jo);
		jo_fmt(jo, "%u,\"auto\",", port);
		jo_arr_end(jo);
		jo_obj_end(jo);

		jo_key(jo, "destination_port");
		jo_obj_begin(jo);
		jo_key(jo, "enum");
		jo_arr_begin(jo);
		jo_fmt(jo, "%u,\"auto\",", port);
		jo_arr_end(jo);
		jo_obj_end(jo);
	} else {
		jo_key(jo, "source_ip");
		jo_obj_begin(jo);
		jo_obj_end(jo);

		jo_key(jo, "multicast_ip");
		jo_obj_begin(jo);
		jo_obj_end(jo);

		jo_key(jo, "interface_ip");
		jo_obj_begin(jo);
		jo_key(jo, "enum");
		jo_arr_begin(jo);
		jo_fmt(jo, "\"%s\",\"auto\",", ip);
		jo_arr_end(jo);
		jo_obj_end(jo);

		jo_key(jo, "destination_port");
		jo_obj_begin(jo);
		jo_obj_end(jo);
	}
	jo_key(jo, "rtp_enabled");
	jo_obj_begin(jo);
	jo_obj_end(jo);
	jo_obj_end(jo);
	jo_arr_end(jo);
}

/* ================================================================
 * IS-04 coupling + lifecycle
 * ================================================================ */

bool nmos_is05_sub(bool sender, int idx, char id_out[NMOS_UUID_STR_LEN],
		   bool *active)
{
	k_mutex_lock(&is05_lock, K_FOREVER);
	if (sender) {
		strcpy(id_out, senders[idx].a_receiver_id);
		*active = senders[idx].a_master_enable;
	} else {
		strcpy(id_out, receivers[idx].a_sender_id);
		*active = receivers[idx].a_master_enable;
	}
	k_mutex_unlock(&is05_lock);
	/* IS-04 subscription: the peer id is only set while active
	 * (nmos-testing "parked" check expects null when disabled). */
	if (!*active) {
		id_out[0] = '\0';
	}
	return id_out[0] != '\0';
}

bool nmos_is05_locked(bool sender, int idx)
{
	bool locked;

	k_mutex_lock(&is05_lock, K_FOREVER);
	locked = (sender ? senders[idx].act.mode
			 : receivers[idx].act.mode) != ACT_NONE;
	k_mutex_unlock(&is05_lock);
	return locked;
}

void nmos_is05_init(void)
{
	for (int i = 0; i < AES67_MAX_TX_STREAMS; i++) {
		sender_defaults(&senders[i]);
		refresh_tx_from_conn(i);
		/* Active boot state (config restore) mirrors into staged;
		 * untouched slots keep the "auto" defaults there. */
		if (senders[i].a_master_enable) {
			senders[i].master_enable = true;
			senders[i].p = senders[i].a_p;
		}
	}
	for (int i = 0; i < AES67_MAX_RX_STREAMS; i++) {
		receiver_defaults(&receivers[i]);
		refresh_rx_from_conn(i);
		if (receivers[i].a_master_enable) {
			receivers[i].master_enable = true;
			receivers[i].p = receivers[i].a_p;
		}
	}

	aes67_conn_register_tx_observer(tx_observer);
	aes67_conn_register_rx_observer(rx_observer);

	k_thread_create(&is05_thread, is05_stack,
			K_THREAD_STACK_SIZEOF(is05_stack), act_worker,
			NULL, NULL, NULL, K_PRIO_PREEMPT(10), 0, K_NO_WAIT);
	k_thread_name_set(&is05_thread, "nmos_is05");
}
