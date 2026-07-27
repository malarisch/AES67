/*
 * RAVENNA mDNS / DNS-SD — self-contained responder + browser.
 *
 * Replaces Zephyr's CONFIG_MDNS_RESPONDER for this application.  RAVENNA
 * discovery (Operating Principles §3.5) requires answering DNS-SD *subtype*
 * PTR queries (_ravenna_session._sub._rtsp._tcp.local, RFC 6763 §7.1) —
 * Zephyr's responder drops 5-label queries ("unsupported number of labels")
 * — and browsing the same subtype for remote sessions.  Zephyr's responder
 * also binds 0.0.0.0:5353 without SO_REUSE*, so a second socket cannot
 * coexist.  This module therefore owns the single 5353 socket and does both
 * directions itself:
 *
 *  Responder (§3.5.1 node services, §3.5.2 session advertisement):
 *    - A    <hostname>.local
 *    - PTR/SRV/TXT for <node>._http._tcp.local and <node>._rtsp._tcp.local
 *      (both the vendor node ID and the user-defined device name)
 *    - PTR  _ravenna._sub._{http,rtsp}._tcp.local -> vendor node ID
 *      (node-level RAVENNA subtype browsing)
 *    - PTR/SRV/TXT for <session>._rtsp._tcp.local per active TX stream
 *    - PTR  _ravenna_session._sub._rtsp._tcp.local -> session instances
 *    - PTR  _services._dns-sd._udp.local (service type enumeration)
 *    - unsolicited announcements on record changes (RFC 6762 §8.3) and
 *      goodbye (TTL 0) on withdrawal
 *
 *  Browser (receiver side):
 *    - periodic PTR query for _ravenna_session._sub._rtsp._tcp.local
 *    - assembles PTR+SRV+A answers into a session cache; sends follow-up
 *      SRV/A queries when a responder omits the additionals
 *    - RTSP-DESCRIBEs each remote session (rtsp_client_describe) and
 *      reports the parsed SDP into the SAP foreign-stream table, so the
 *      web UI shows SAP- and mDNS-discovered streams alike
 */

#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/hostname.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <string.h>
#include <strings.h>
#include <stdio.h>
#include <ctype.h>

#include "mdns_sd.h"
#include "aes67_config.h"
#include "aes67_sdp_utils.h"
#include "aes67_conn.h"
#include "rtsp.h"

LOG_MODULE_REGISTER(mdns_sd, LOG_LEVEL_INF);

/* ---- Wire constants ---- */

#define MDNS_PORT        5353
#define MDNS_GROUP_ADDR  "224.0.0.251"

#define DNS_TYPE_A     1
#define DNS_TYPE_PTR   12
#define DNS_TYPE_TXT   16
#define DNS_TYPE_SRV   33
#define DNS_TYPE_ANY   255

#define DNS_CLASS_IN      0x0001
#define DNS_CLASS_FLUSH   0x8000  /* cache-flush bit (responses)   */
#define DNS_CLASS_QU      0x8000  /* unicast-response bit (queries) */

#define TTL_HOST     120   /* A + SRV (host-specific)      */
#define TTL_SERVICE  4500  /* PTR + TXT (RFC 6762 §10)     */

#define SVC_RTSP         "_rtsp._tcp.local"
#define SVC_HTTP         "_http._tcp.local"
#define SUBTYPE_RAVENNA  "_ravenna_session._sub._rtsp._tcp.local"
/* Node-level "ravenna" subtypes (RAVENNA §3.5.1): registered with the
 * vendor node ID only, so browsers can discover RAVENNA nodes as such. */
#define SUBTYPE_RAV_RTSP "_ravenna._sub._rtsp._tcp.local"
#define SUBTYPE_RAV_HTTP "_ravenna._sub._http._tcp.local"
#define SVC_ENUM         "_services._dns-sd._udp.local"

static const char txt_ravenna[] = "type=ravenna";

/* ---- Local advertisement state ---- */

#define MDNS_NAME_MAX          64
#define FQDN_MAX          96
#define MDNS_MAX_SESSIONS AES67_MAX_TX_STREAMS

static char vendor_node_id[MDNS_NAME_MAX];
static char user_device_name[MDNS_NAME_MAX];
static char host_fqdn[FQDN_MAX];         /* "<hostname>.local" */

static struct {
	char name[MDNS_NAME_MAX];             /* "" = slot unused */
} sessions[MDNS_MAX_SESSIONS];

/* ---- Discovered remote sessions (browser cache) ---- */

#define MDNS_MAX_DISCOVERED  24
#define BROWSE_INTERVAL_MS   (60 * 1000)
#define CACHE_EXPIRY_MS      (3 * BROWSE_INTERVAL_MS + 15000)
#define REPORT_REFRESH_MS    (20 * 1000)
#define DESCRIBE_MAX_FAILS   3
/* Minimum spacing between DESCRIBE attempts for one session. A changed
 * SRV port / A record legitimately re-arms a DESCRIBE — but when two
 * sources keep answering DIFFERENT records for the same name (mDNS
 * reflector, stale cache, duplicate hostname), "changed" fires on every
 * packet and the client hammers the server in an endless
 * connect/DESCRIBE/close loop at round-trip rate (~2/s observed). The
 * backoff turns any such fight into a slow trickle. */
#define DESCRIBE_MIN_INTERVAL_MS (10 * 1000)

struct discovered_session {
	bool     used;
	char     instance[MDNS_NAME_MAX];     /* session name (first label)  */
	char     srv_target[FQDN_MAX];   /* SRV target host fqdn        */
	struct in_addr host;             /* A record of srv_target      */
	uint16_t port;                   /* SRV port                    */
	bool     have_srv;
	bool     have_a;
	bool     described;              /* SDP fetched + reported      */
	uint8_t  fail_count;             /* consecutive DESCRIBE fails  */
	int64_t  last_seen_ms;           /* last PTR/SRV/A refresh      */
	int64_t  last_report_ms;         /* last foreign-table report   */
	int64_t  next_followup_ms;       /* next SRV/A follow-up query  */
	int64_t  next_describe_ms;       /* DESCRIBE rate limit         */
};

static struct discovered_session discovered[MDNS_MAX_DISCOVERED];

/* ---- Module state ---- */

static int mdns_sock = -1;
static struct k_mutex mdns_lock;
static struct k_sem describe_sem;
static bool mdns_started;

/* Pending unsolicited announcements (RFC 6762 §8.3: send at least twice) */
static int announce_pending;
static int64_t announce_next_ms;

/* Steady-state re-announcement: our SRV/A records carry TTL 120 s and are
 * normally kept alive by answering the peers' cache-refresh queries.  If
 * those multicast queries never reach us (e.g. IGMP snooping aged out our
 * 224.0.0.251 membership — group membership interval is ~260 s), peer
 * caches expire and browsers drop the service.  Re-announcing twice per
 * TTL window keeps caches fresh purely over our TX path. */
#define REANNOUNCE_INTERVAL_MS  (55 * 1000)
static int64_t reannounce_next_ms;

/* Browse query schedule */
static int browse_burst;                 /* startup burst counter */
static int64_t browse_next_ms;

K_THREAD_STACK_DEFINE(mdns_stack, 4096);
static struct k_thread mdns_thread_data;

K_THREAD_STACK_DEFINE(describe_stack, 4096);
static struct k_thread describe_thread_data;

/* ================================================================
 * DNS wire helpers
 * ================================================================ */

/* Append a dotted name as DNS labels (no compression). Returns new offset
 * or -1 on overflow. */
static int name_put(uint8_t *buf, size_t size, int off, const char *dotted)
{
	const char *p = dotted;

	if (off < 0) {
		return -1;
	}

	while (*p) {
		const char *dot = strchr(p, '.');
		size_t label_len = dot ? (size_t)(dot - p) : strlen(p);

		if (label_len == 0 || label_len > 63 ||
		    off + 1 + label_len >= size) {
			return -1;
		}
		buf[off++] = (uint8_t)label_len;
		memcpy(&buf[off], p, label_len);
		off += label_len;
		p += label_len;
		if (*p == '.') {
			p++;
		}
	}

	if ((size_t)off + 1 > size) {
		return -1;
	}
	buf[off++] = 0;
	return off;
}

/* Parse a (possibly compressed) name at `off` into a dotted string,
 * preserving case (DNS names compare case-insensitively — all matching
 * here goes through name_eq() — but the RAVENNA session name must keep
 * its original case for the RTSP by-name DESCRIBE).  Sets *next to the
 * offset just past the name at its original position.  Returns 0 or -1. */
static int name_parse(const uint8_t *msg, size_t len, size_t off,
		      char *out, size_t out_size, size_t *next)
{
	size_t out_len = 0;
	int hops = 0;
	bool jumped = false;

	if (next) {
		*next = 0;
	}

	while (true) {
		if (off >= len) {
			return -1;
		}
		uint8_t c = msg[off];

		if ((c & 0xC0) == 0xC0) {
			/* compression pointer */
			if (off + 1 >= len || ++hops > 8) {
				return -1;
			}
			uint16_t ptr = ((c & 0x3F) << 8) | msg[off + 1];

			if (!jumped && next) {
				*next = off + 2;
			}
			jumped = true;
			if (ptr >= off) {
				return -1;  /* must point backwards */
			}
			off = ptr;
			continue;
		}

		if (c == 0) {
			if (!jumped && next) {
				*next = off + 1;
			}
			break;
		}

		if ((c & 0xC0) != 0 || off + 1 + c > len) {
			return -1;
		}
		if (out_len + c + 2 > out_size) {
			return -1;
		}
		if (out_len > 0) {
			out[out_len++] = '.';
		}
		memcpy(&out[out_len], &msg[off + 1], c);
		out_len += c;
		off += 1 + c;
	}

	out[out_len] = '\0';
	return 0;
}

/* ---- Record writers (all records go into the answer section) ---- */

static int rr_header(uint8_t *buf, size_t size, int off, const char *name,
		     uint16_t type, uint16_t class_ttl_flush, uint32_t ttl)
{
	off = name_put(buf, size, off, name);
	if (off < 0 || (size_t)off + 10 > size) {
		return -1;
	}
	sys_put_be16(type, &buf[off]);
	sys_put_be16(class_ttl_flush, &buf[off + 2]);
	sys_put_be32(ttl, &buf[off + 4]);
	/* rdlength written by caller via rr_rdlen() */
	off += 10;
	return off;
}

static void rr_rdlen(uint8_t *buf, int rdata_start, int rdata_end)
{
	sys_put_be16((uint16_t)(rdata_end - rdata_start),
		     &buf[rdata_start - 2]);
}

static int add_ptr(uint8_t *buf, size_t size, int off, const char *name,
		   const char *target, uint32_t ttl)
{
	int rd;

	off = rr_header(buf, size, off, name, DNS_TYPE_PTR, DNS_CLASS_IN, ttl);
	if (off < 0) {
		return -1;
	}
	rd = off;
	off = name_put(buf, size, off, target);
	if (off < 0) {
		return -1;
	}
	rr_rdlen(buf, rd, off);
	return off;
}

static int add_srv(uint8_t *buf, size_t size, int off, const char *instance_fqdn,
		   uint16_t port)
{
	int rd;

	off = rr_header(buf, size, off, instance_fqdn, DNS_TYPE_SRV,
			DNS_CLASS_IN | DNS_CLASS_FLUSH, TTL_HOST);
	if (off < 0 || (size_t)off + 6 > size) {
		return -1;
	}
	rd = off;
	sys_put_be16(0, &buf[off]);      /* priority */
	sys_put_be16(0, &buf[off + 2]);  /* weight   */
	sys_put_be16(port, &buf[off + 4]);
	off += 6;
	off = name_put(buf, size, off, host_fqdn);
	if (off < 0) {
		return -1;
	}
	rr_rdlen(buf, rd, off);
	return off;
}

static int add_txt(uint8_t *buf, size_t size, int off, const char *instance_fqdn,
		   uint32_t ttl)
{
	size_t tl = strlen(txt_ravenna);
	int rd;

	off = rr_header(buf, size, off, instance_fqdn, DNS_TYPE_TXT,
			DNS_CLASS_IN | DNS_CLASS_FLUSH, ttl);
	if (off < 0 || (size_t)off + 1 + tl > size) {
		return -1;
	}
	rd = off;
	buf[off++] = (uint8_t)tl;
	memcpy(&buf[off], txt_ravenna, tl);
	off += tl;
	rr_rdlen(buf, rd, off);
	return off;
}

static int add_a(uint8_t *buf, size_t size, int off, const struct in_addr *ip)
{
	int rd;

	off = rr_header(buf, size, off, host_fqdn, DNS_TYPE_A,
			DNS_CLASS_IN | DNS_CLASS_FLUSH, TTL_HOST);
	if (off < 0 || (size_t)off + 4 > size) {
		return -1;
	}
	rd = off;
	memcpy(&buf[off], &ip->s_addr, 4);
	off += 4;
	rr_rdlen(buf, rd, off);
	return off;
}

/* ================================================================
 * Small utilities
 * ================================================================ */

static struct in_addr my_ipv4(void)
{
	struct in_addr zero = { 0 };
	struct net_if *iface = net_if_get_default();
	struct in_addr *addr;

	if (!iface) {
		return zero;
	}
	addr = net_if_ipv4_get_global_addr(iface, NET_ADDR_PREFERRED);
	if (addr == NULL) {
		/* No lease: the node runs on its Zeroconf address, which is
		 * exactly the case where mDNS is how anyone finds it. */
		addr = net_if_ipv4_get_ll(iface, NET_ADDR_PREFERRED);
	}
	return addr ? *addr : zero;
}

/* "<instance>._rtsp._tcp.local" */
static void instance_fqdn(char *out, size_t size, const char *instance,
			  const char *svc)
{
	snprintf(out, size, "%s.%s", instance, svc);
}

/* Case-insensitive dotted-name compare (queries arrive lowercased). */
static bool name_eq(const char *a, const char *b)
{
	return strcasecmp(a, b) == 0;
}

/* First label of a dotted name (the DNS-SD instance). */
static void first_label(const char *fqdn, char *out, size_t size)
{
	size_t i = 0;

	while (fqdn[i] && fqdn[i] != '.' && i < size - 1) {
		out[i] = fqdn[i];
		i++;
	}
	out[i] = '\0';
}

static bool is_own_session(const char *instance)
{
	for (int i = 0; i < MDNS_MAX_SESSIONS; i++) {
		if (sessions[i].name[0] != '\0' &&
		    name_eq(sessions[i].name, instance)) {
			return true;
		}
	}
	if (name_eq(instance, vendor_node_id) ||
	    name_eq(instance, user_device_name)) {
		return true;
	}
	return false;
}

static void send_packet(const uint8_t *buf, size_t len,
			const struct sockaddr_in *dst)
{
	struct sockaddr_in mcast;
	const struct sockaddr_in *to = dst;

	if (!to) {
		memset(&mcast, 0, sizeof(mcast));
		mcast.sin_family = AF_INET;
		mcast.sin_port = htons(MDNS_PORT);
		zsock_inet_pton(AF_INET, MDNS_GROUP_ADDR, &mcast.sin_addr);
		to = &mcast;
	}

	int ret = zsock_sendto(mdns_sock, buf, len, 0,
			       (const struct sockaddr *)to, sizeof(*to));
	if (ret < 0) {
		LOG_DBG("mDNS send failed: %d", errno);
	}
}

/* ================================================================
 * Responder
 * ================================================================ */

static uint8_t tx_buf[1400];

/* Start a response packet; answer count patched in finish_response(). */
static int begin_response(uint16_t query_id)
{
	memset(tx_buf, 0, 12);
	sys_put_be16(query_id, &tx_buf[0]);
	sys_put_be16(0x8400, &tx_buf[2]);  /* QR=1, AA=1 */
	return 12;
}

static void finish_response(int off, int answers,
			    const struct sockaddr_in *unicast_dst,
			    uint16_t query_id)
{
	if (answers <= 0 || off <= 12) {
		return;
	}
	sys_put_be16(query_id, &tx_buf[0]);
	sys_put_be16((uint16_t)answers, &tx_buf[6]);
	send_packet(tx_buf, off, unicast_dst);
}

/* Add PTR + SRV + TXT + A for one service instance.  `subtype` (NULL for
 * none) selects the PTR record's owner name.  Additionals that don't fit
 * are dropped (receivers re-query). */
static int add_instance(int off, int *answers, const char *instance,
			const char *svc, uint16_t port, const char *subtype,
			bool with_details, uint32_t ptr_ttl)
{
	char fqdn[FQDN_MAX + MDNS_NAME_MAX];
	struct in_addr ip = my_ipv4();
	int n;

	instance_fqdn(fqdn, sizeof(fqdn), instance, svc);

	n = add_ptr(tx_buf, sizeof(tx_buf), off,
		    subtype ? subtype : svc, fqdn, ptr_ttl);
	if (n < 0) {
		return off;
	}
	off = n;
	(*answers)++;

	if (!with_details) {
		return off;
	}

	n = add_srv(tx_buf, sizeof(tx_buf), off, fqdn, port);
	if (n > 0) {
		off = n;
		(*answers)++;
	}
	n = add_txt(tx_buf, sizeof(tx_buf), off, fqdn, TTL_SERVICE);
	if (n > 0) {
		off = n;
		(*answers)++;
	}
	if (ip.s_addr != 0) {
		n = add_a(tx_buf, sizeof(tx_buf), off, &ip);
		if (n > 0) {
			off = n;
			(*answers)++;
		}
	}
	return off;
}

/* Answer one parsed question.  Returns updated offset. */
static int answer_question(int off, int *answers, const char *qname,
			   uint16_t qtype)
{
	bool t_ptr = (qtype == DNS_TYPE_PTR || qtype == DNS_TYPE_ANY);
	bool t_srv = (qtype == DNS_TYPE_SRV || qtype == DNS_TYPE_ANY);
	bool t_txt = (qtype == DNS_TYPE_TXT || qtype == DNS_TYPE_ANY);
	bool t_a   = (qtype == DNS_TYPE_A || qtype == DNS_TYPE_ANY);
	uint16_t rtsp_port = CONFIG_RTSP_PORT;
	int n;

	if (t_ptr && name_eq(qname, SVC_ENUM)) {
		n = add_ptr(tx_buf, sizeof(tx_buf), off, SVC_ENUM, SVC_RTSP,
			    TTL_SERVICE);
		if (n > 0) {
			off = n;
			(*answers)++;
		}
		n = add_ptr(tx_buf, sizeof(tx_buf), off, SVC_ENUM, SVC_HTTP,
			    TTL_SERVICE);
		if (n > 0) {
			off = n;
			(*answers)++;
		}
		return off;
	}

	if (t_ptr && name_eq(qname, SUBTYPE_RAVENNA)) {
		for (int i = 0; i < MDNS_MAX_SESSIONS; i++) {
			if (sessions[i].name[0] == '\0') {
				continue;
			}
			off = add_instance(off, answers, sessions[i].name,
					   SVC_RTSP, rtsp_port,
					   SUBTYPE_RAVENNA, true,
					   TTL_SERVICE);
		}
		return off;
	}

	/* Node-level RAVENNA subtype browsing (§3.5.1): vendor node ID only */
	if (t_ptr && name_eq(qname, SUBTYPE_RAV_RTSP)) {
		off = add_instance(off, answers, vendor_node_id, SVC_RTSP,
				   rtsp_port, SUBTYPE_RAV_RTSP, true,
				   TTL_SERVICE);
		return off;
	}

	if (t_ptr && name_eq(qname, SUBTYPE_RAV_HTTP)) {
		off = add_instance(off, answers, vendor_node_id, SVC_HTTP,
				   80, SUBTYPE_RAV_HTTP, true, TTL_SERVICE);
		return off;
	}

	if (t_ptr && name_eq(qname, SVC_RTSP)) {
		off = add_instance(off, answers, vendor_node_id, SVC_RTSP,
				   rtsp_port, NULL, true, TTL_SERVICE);
		off = add_instance(off, answers, user_device_name, SVC_RTSP,
				   rtsp_port, NULL, true, TTL_SERVICE);
		for (int i = 0; i < MDNS_MAX_SESSIONS; i++) {
			if (sessions[i].name[0] == '\0') {
				continue;
			}
			off = add_instance(off, answers, sessions[i].name,
					   SVC_RTSP, rtsp_port, NULL, false,
					   TTL_SERVICE);
		}
		return off;
	}

	if (t_ptr && name_eq(qname, SVC_HTTP)) {
		off = add_instance(off, answers, vendor_node_id, SVC_HTTP,
				   80, NULL, true, TTL_SERVICE);
		off = add_instance(off, answers, user_device_name, SVC_HTTP,
				   80, NULL, true, TTL_SERVICE);
		return off;
	}

	if (t_a && name_eq(qname, host_fqdn)) {
		struct in_addr ip = my_ipv4();

		if (ip.s_addr != 0) {
			n = add_a(tx_buf, sizeof(tx_buf), off, &ip);
			if (n > 0) {
				off = n;
				(*answers)++;
			}
		}
		return off;
	}

	/* Instance-specific SRV/TXT queries: "<inst>._rtsp/_http._tcp.local" */
	if (t_srv || t_txt) {
		char inst[MDNS_NAME_MAX];
		bool is_rtsp;
		size_t qlen = strlen(qname);
		size_t svclen = strlen(SVC_RTSP);

		if (qlen > svclen + 1 &&
		    name_eq(qname + (qlen - svclen), SVC_RTSP)) {
			is_rtsp = true;
		} else if (qlen > svclen + 1 &&
			   name_eq(qname + (qlen - svclen), SVC_HTTP)) {
			is_rtsp = false;
		} else {
			return off;
		}

		first_label(qname, inst, sizeof(inst));
		if (!is_own_session(inst)) {
			return off;
		}

		uint16_t port = is_rtsp ? rtsp_port : 80;
		char fqdn[FQDN_MAX + MDNS_NAME_MAX];
		struct in_addr ip = my_ipv4();

		instance_fqdn(fqdn, sizeof(fqdn), inst,
			      is_rtsp ? SVC_RTSP : SVC_HTTP);

		if (t_srv) {
			n = add_srv(tx_buf, sizeof(tx_buf), off, fqdn, port);
			if (n > 0) {
				off = n;
				(*answers)++;
			}
		}
		if (t_txt) {
			n = add_txt(tx_buf, sizeof(tx_buf), off, fqdn,
				    TTL_SERVICE);
			if (n > 0) {
				off = n;
				(*answers)++;
			}
		}
		if (ip.s_addr != 0) {
			n = add_a(tx_buf, sizeof(tx_buf), off, &ip);
			if (n > 0) {
				off = n;
				(*answers)++;
			}
		}
	}

	return off;
}

static void handle_query(const uint8_t *msg, size_t len,
			 const struct sockaddr_in *src)
{
	uint16_t id = sys_get_be16(&msg[0]);
	uint16_t qdcount = sys_get_be16(&msg[4]);
	size_t off = 12;
	bool unicast_reply = (ntohs(src->sin_port) != MDNS_PORT);
	int out;
	int answers = 0;

	if (qdcount == 0 || qdcount > 8) {
		return;
	}

	out = begin_response(unicast_reply ? id : 0);

	for (int q = 0; q < qdcount; q++) {
		char qname[FQDN_MAX + MDNS_NAME_MAX];
		size_t next;

		if (name_parse(msg, len, off, qname, sizeof(qname), &next) < 0 ||
		    next + 4 > len) {
			return;
		}
		uint16_t qtype = sys_get_be16(&msg[next]);
		uint16_t qclass = sys_get_be16(&msg[next + 2]);

		off = next + 4;
		if (qclass & DNS_CLASS_QU) {
			unicast_reply = true;
		}

		out = answer_question(out, &answers, qname, qtype);
	}

	finish_response(out, answers, unicast_reply ? src : NULL,
			unicast_reply ? id : 0);
}

/* Unsolicited announcement (or goodbye with ttl=0) for everything we
 * advertise.  One packet per session keeps each well under one MTU. */
static void announce_all(uint32_t ptr_ttl)
{
	int off, answers;

	/* Node services + hostname; the vendor node ID additionally carries
	 * the node-level "ravenna" subtypes (§3.5.1). */
	answers = 0;
	off = begin_response(0);
	off = add_instance(off, &answers, vendor_node_id, SVC_RTSP,
			   CONFIG_RTSP_PORT, NULL, true, ptr_ttl);
	off = add_instance(off, &answers, vendor_node_id, SVC_RTSP,
			   CONFIG_RTSP_PORT, SUBTYPE_RAV_RTSP, false, ptr_ttl);
	off = add_instance(off, &answers, user_device_name, SVC_RTSP,
			   CONFIG_RTSP_PORT, NULL, true, ptr_ttl);
	off = add_instance(off, &answers, vendor_node_id, SVC_HTTP, 80,
			   NULL, false, ptr_ttl);
	off = add_instance(off, &answers, vendor_node_id, SVC_HTTP, 80,
			   SUBTYPE_RAV_HTTP, false, ptr_ttl);
	off = add_instance(off, &answers, user_device_name, SVC_HTTP, 80,
			   NULL, false, ptr_ttl);
	finish_response(off, answers, NULL, 0);

	/* Sessions: base PTR + subtype PTR + SRV/TXT/A each */
	for (int i = 0; i < MDNS_MAX_SESSIONS; i++) {
		if (sessions[i].name[0] == '\0') {
			continue;
		}
		answers = 0;
		off = begin_response(0);
		off = add_instance(off, &answers, sessions[i].name, SVC_RTSP,
				   CONFIG_RTSP_PORT, NULL, true, ptr_ttl);
		off = add_instance(off, &answers, sessions[i].name, SVC_RTSP,
				   CONFIG_RTSP_PORT, SUBTYPE_RAVENNA, false,
				   ptr_ttl);
		finish_response(off, answers, NULL, 0);
	}
}

/* Goodbye for a single withdrawn session. */
static void goodbye_session(const char *name)
{
	int off, answers = 0;

	off = begin_response(0);
	off = add_instance(off, &answers, name, SVC_RTSP, CONFIG_RTSP_PORT,
			   NULL, false, 0);
	off = add_instance(off, &answers, name, SVC_RTSP, CONFIG_RTSP_PORT,
			   SUBTYPE_RAVENNA, false, 0);
	finish_response(off, answers, NULL, 0);
}

/* ================================================================
 * Browser
 * ================================================================ */

static void send_browse_query(void)
{
	uint8_t buf[96];
	int off;

	memset(buf, 0, 12);
	sys_put_be16(1, &buf[4]);  /* QDCOUNT */
	off = name_put(buf, sizeof(buf), 12, SUBTYPE_RAVENNA);
	if (off < 0 || (size_t)off + 4 > sizeof(buf)) {
		return;
	}
	sys_put_be16(DNS_TYPE_PTR, &buf[off]);
	sys_put_be16(DNS_CLASS_IN, &buf[off + 2]);
	send_packet(buf, off + 4, NULL);
}

/* Follow-up SRV (+ implicit A via additionals) query for an incomplete
 * cache entry. */
static void send_followup_query(const struct discovered_session *d)
{
	uint8_t buf[192];
	char fqdn[FQDN_MAX + MDNS_NAME_MAX];
	int off;
	int qd = 0;

	memset(buf, 0, 12);
	off = 12;

	if (!d->have_srv) {
		instance_fqdn(fqdn, sizeof(fqdn), d->instance, SVC_RTSP);
		off = name_put(buf, sizeof(buf), off, fqdn);
		if (off < 0 || (size_t)off + 4 > sizeof(buf)) {
			return;
		}
		sys_put_be16(DNS_TYPE_SRV, &buf[off]);
		sys_put_be16(DNS_CLASS_IN, &buf[off + 2]);
		off += 4;
		qd++;
	} else if (!d->have_a) {
		off = name_put(buf, sizeof(buf), off, d->srv_target);
		if (off < 0 || (size_t)off + 4 > sizeof(buf)) {
			return;
		}
		sys_put_be16(DNS_TYPE_A, &buf[off]);
		sys_put_be16(DNS_CLASS_IN, &buf[off + 2]);
		off += 4;
		qd++;
	}

	if (qd > 0) {
		sys_put_be16(qd, &buf[4]);
		send_packet(buf, off, NULL);
	}
}

/* 16-bit hash of the instance name — stands in for the SAP msg-id hash so
 * the foreign-stream table can dedup/delete mDNS entries. */
static uint16_t instance_hash(const char *name)
{
	uint32_t h = 5381;

	for (const char *p = name; *p; p++) {
		h = ((h << 5) + h) ^ (uint8_t)tolower((int)*p);
	}
	return (uint16_t)(h ^ (h >> 16)) | 0x8000; /* avoid clashing with SAP ids */
}

static void report_foreign_invalid(const struct discovered_session *d)
{
	struct aes67_foreign_stream fs;

	memset(&fs, 0, sizeof(fs));
	fs.valid = false;
	fs.id_hash = instance_hash(d->instance);
	aes67_conn_report_foreign_stream(&fs);
}

static struct discovered_session *cache_find(const char *instance)
{
	for (int i = 0; i < MDNS_MAX_DISCOVERED; i++) {
		if (discovered[i].used &&
		    name_eq(discovered[i].instance, instance)) {
			return &discovered[i];
		}
	}
	return NULL;
}

static struct discovered_session *cache_get(const char *instance)
{
	struct discovered_session *d = cache_find(instance);

	if (d) {
		return d;
	}
	for (int i = 0; i < MDNS_MAX_DISCOVERED; i++) {
		if (!discovered[i].used) {
			d = &discovered[i];
			memset(d, 0, sizeof(*d));
			d->used = true;
			strncpy(d->instance, instance, sizeof(d->instance) - 1);
			return d;
		}
	}
	return NULL;
}

static void cache_remove(struct discovered_session *d)
{
	LOG_INF("mDNS: session '%s' gone", d->instance);
	if (d->described) {
		report_foreign_invalid(d);
	}
	d->used = false;
}

/* Parse a response packet: collect subtype PTRs, SRVs and As that belong to
 * (potential) RAVENNA sessions. */
static void handle_response(const uint8_t *msg, size_t len)
{
	uint16_t counts = 0;
	size_t off = 12;
	int64_t now = k_uptime_get();
	struct in_addr own = my_ipv4();

	/* answers + authority + additionals */
	counts = sys_get_be16(&msg[6]) + sys_get_be16(&msg[8]) +
		 sys_get_be16(&msg[10]);
	if (counts == 0 || counts > 64) {
		return;
	}

	/* Skip any echoed questions */
	uint16_t qd = sys_get_be16(&msg[4]);

	for (int q = 0; q < qd; q++) {
		char scratch[FQDN_MAX + MDNS_NAME_MAX];
		size_t next;

		if (name_parse(msg, len, off, scratch, sizeof(scratch),
			       &next) < 0 || next + 4 > len) {
			return;
		}
		off = next + 4;
	}

	for (int r = 0; r < counts; r++) {
		char rname[FQDN_MAX + MDNS_NAME_MAX];
		size_t next;

		if (name_parse(msg, len, off, rname, sizeof(rname), &next) < 0 ||
		    next + 10 > len) {
			return;
		}
		uint16_t type = sys_get_be16(&msg[next]);
		uint32_t ttl = sys_get_be32(&msg[next + 4]);
		uint16_t rdlen = sys_get_be16(&msg[next + 8]);
		size_t rdata = next + 10;

		if (rdata + rdlen > len) {
			return;
		}
		off = rdata + rdlen;

		if (type == DNS_TYPE_PTR && name_eq(rname, SUBTYPE_RAVENNA)) {
			char target[FQDN_MAX + MDNS_NAME_MAX];
			char inst[MDNS_NAME_MAX];

			if (name_parse(msg, len, rdata, target, sizeof(target),
				       NULL) < 0) {
				continue;
			}
			first_label(target, inst, sizeof(inst));
			if (inst[0] == '\0' || is_own_session(inst)) {
				continue;
			}

			struct discovered_session *d;

			if (ttl == 0) {
				d = cache_find(inst);
				if (d) {
					cache_remove(d);
				}
				continue;
			}
			d = cache_get(inst);
			if (d) {
				d->last_seen_ms = now;
			}
		} else if (type == DNS_TYPE_SRV) {
			/* "<inst>._rtsp._tcp.local" SRV */
			size_t rl = strlen(rname);
			size_t sl = strlen(SVC_RTSP);

			if (rl <= sl + 1 ||
			    !name_eq(rname + (rl - sl), SVC_RTSP) ||
			    rdlen < 7) {
				continue;
			}

			char inst[MDNS_NAME_MAX];

			first_label(rname, inst, sizeof(inst));

			struct discovered_session *d = cache_find(inst);

			if (!d) {
				continue;
			}
			uint16_t port = sys_get_be16(&msg[rdata + 4]);

			if (name_parse(msg, len, rdata + 6, d->srv_target,
				       sizeof(d->srv_target), NULL) < 0) {
				continue;
			}
			if (d->have_srv && d->port != port) {
				/* Loud on purpose: constant port "changes"
				 * mean two sources answer different records
				 * for this name (see DESCRIBE_MIN_INTERVAL). */
				LOG_INF("mDNS: session '%s' SRV port %u -> %u — re-DESCRIBE",
					inst, d->port, port);
				d->described = false;
			}
			d->port = port;
			d->have_srv = true;
			d->last_seen_ms = now;
		} else if (type == DNS_TYPE_A && rdlen == 4) {
			struct in_addr a;

			memcpy(&a.s_addr, &msg[rdata], 4);
			for (int i = 0; i < MDNS_MAX_DISCOVERED; i++) {
				struct discovered_session *d = &discovered[i];

				if (!d->used || !d->have_srv ||
				    !name_eq(d->srv_target, rname)) {
					continue;
				}
				if (own.s_addr != 0 && a.s_addr == own.s_addr) {
					/* our own advertisement */
					d->used = false;
					continue;
				}
				if (d->have_a && d->host.s_addr != a.s_addr) {
					char o[INET_ADDRSTRLEN], n[INET_ADDRSTRLEN];

					zsock_inet_ntop(AF_INET, &d->host, o, sizeof(o));
					zsock_inet_ntop(AF_INET, &a, n, sizeof(n));
					/* Loud on purpose: a host that keeps
					 * "changing" means two sources answer
					 * different A records for the same
					 * name — the classic driver of an
					 * endless re-DESCRIBE loop. */
					LOG_INF("mDNS: session '%s' host %s -> %s — re-DESCRIBE",
						d->instance, o, n);
					d->described = false;
				}
				d->host = a;
				d->have_a = true;
				d->last_seen_ms = now;
			}
		}
	}

	/* Wake the describe worker if anything is ready */
	for (int i = 0; i < MDNS_MAX_DISCOVERED; i++) {
		if (discovered[i].used && discovered[i].have_srv &&
		    discovered[i].have_a && !discovered[i].described &&
		    discovered[i].fail_count < DESCRIBE_MAX_FAILS &&
		    now >= discovered[i].next_describe_ms) {
			k_sem_give(&describe_sem);
			break;
		}
	}
}

/* ================================================================
 * DESCRIBE worker: fetch SDP for resolved sessions, report to SAP table
 * ================================================================ */

static void report_foreign(const struct discovered_session *d,
			   const struct aes67_sdp_parsed *sdp)
{
	struct aes67_foreign_stream fs;

	memset(&fs, 0, sizeof(fs));
	fs.valid = true;
	fs.via = AES67_VIA_MDNS;
	fs.id_hash = instance_hash(d->instance);
	fs.origin_addr = d->host;
	fs.mcast_addr = sdp->connection_addr;
	fs.port = sdp->port;
	fs.channels = sdp->channels;
	fs.bit_depth = sdp->bit_depth;
	fs.sample_rate = sdp->sample_rate;
	fs.ssrc = sdp->ssrc;
	fs.samples_per_packet = sdp->samples_per_packet;
	fs.ptp_domain = sdp->ptp_domain;
	fs.has_clock_domain = sdp->has_clock_domain;
	fs.sync_time = sdp->sync_time;
	fs.has_sync_time = sdp->has_sync_time;
	/* Prefer the SDP session name, fall back to the mDNS instance */
	if (sdp->name[0] != '\0') {
		strncpy(fs.name, sdp->name, sizeof(fs.name) - 1);
	} else {
		strncpy(fs.name, d->instance, sizeof(fs.name) - 1);
	}
	aes67_conn_report_foreign_stream(&fs);
}

static void describe_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	while (1) {
		k_sem_take(&describe_sem, K_FOREVER);

		for (int i = 0; i < MDNS_MAX_DISCOVERED; i++) {
			struct discovered_session snap;

			k_mutex_lock(&mdns_lock, K_FOREVER);
			if (!discovered[i].used || !discovered[i].have_srv ||
			    !discovered[i].have_a || discovered[i].described ||
			    discovered[i].fail_count >= DESCRIBE_MAX_FAILS ||
			    k_uptime_get() < discovered[i].next_describe_ms) {
				k_mutex_unlock(&mdns_lock);
				continue;
			}
			/* Arm the rate limit BEFORE the (slow, blocking)
			 * DESCRIBE so parallel wakes cannot double-fire. */
			discovered[i].next_describe_ms =
				k_uptime_get() + DESCRIBE_MIN_INTERVAL_MS;
			snap = discovered[i];
			k_mutex_unlock(&mdns_lock);

			struct aes67_sdp_parsed sdp;
			int ret = rtsp_client_describe(&snap.host, snap.port,
						       snap.instance, &sdp);

			k_mutex_lock(&mdns_lock, K_FOREVER);
			if (discovered[i].used &&
			    name_eq(discovered[i].instance, snap.instance)) {
				if (ret == 0) {
					discovered[i].described = true;
					discovered[i].fail_count = 0;
					discovered[i].last_report_ms =
						k_uptime_get();
				} else {
					discovered[i].fail_count++;
				}
			}
			k_mutex_unlock(&mdns_lock);

			if (ret == 0) {
				char addr_str[INET_ADDRSTRLEN];

				zsock_inet_ntop(AF_INET, &sdp.connection_addr,
						addr_str, sizeof(addr_str));
				LOG_INF("mDNS: discovered session '%s' @ %s:%u",
					snap.instance, addr_str, sdp.port);
				report_foreign(&snap, &sdp);
			} else {
				LOG_WRN("mDNS: DESCRIBE '%s' failed: %d",
					snap.instance, ret);
			}
		}
	}
}

/* ================================================================
 * Periodic work (called from the socket thread on its poll timeout)
 * ================================================================ */

static void periodic_tick(void)
{
	int64_t now = k_uptime_get();

	k_mutex_lock(&mdns_lock, K_FOREVER);

	if (announce_pending > 0 && now >= announce_next_ms) {
		announce_all(TTL_SERVICE);
		announce_pending--;
		announce_next_ms = now + 1000;
		reannounce_next_ms = now + REANNOUNCE_INTERVAL_MS;
	} else if (now >= reannounce_next_ms) {
		announce_all(TTL_SERVICE);
		reannounce_next_ms = now + REANNOUNCE_INTERVAL_MS;
	}

	if (now >= browse_next_ms) {
		send_browse_query();
		if (browse_burst < 3) {
			browse_burst++;
			browse_next_ms = now + (1000 << browse_burst);
		} else {
			browse_next_ms = now + BROWSE_INTERVAL_MS;
		}
	}

	for (int i = 0; i < MDNS_MAX_DISCOVERED; i++) {
		struct discovered_session *d = &discovered[i];

		if (!d->used) {
			continue;
		}
		if (now - d->last_seen_ms > CACHE_EXPIRY_MS) {
			cache_remove(d);
			continue;
		}
		if ((!d->have_srv || !d->have_a) &&
		    now >= d->next_followup_ms) {
			send_followup_query(d);
			d->next_followup_ms = now + 3000;
		}
		/* A DESCRIBE deferred by the rate limit has no packet to wake
		 * the worker when the backoff expires — re-arm it from here. */
		if (d->have_srv && d->have_a && !d->described &&
		    d->fail_count < DESCRIBE_MAX_FAILS &&
		    now >= d->next_describe_ms) {
			k_sem_give(&describe_sem);
		}
		if (d->described &&
		    now - d->last_report_ms > REPORT_REFRESH_MS) {
			/* Keep the foreign-stream entry from expiring while
			 * the session is still being announced — no need to
			 * re-DESCRIBE, just refresh its timestamp. */
			d->last_report_ms = now;
			aes67_conn_touch_foreign_stream(instance_hash(d->instance));
		}
	}

	k_mutex_unlock(&mdns_lock);
}

/* ================================================================
 * Socket thread
 * ================================================================ */

static uint8_t rx_buf[1500];

static void mdns_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	/* Wait for the interface + IPv4 address */
	while (1) {
		struct net_if *iface = net_if_get_default();

		if (iface && net_if_is_up(iface) && my_ipv4().s_addr != 0) {
			break;
		}
		k_msleep(500);
	}

	mdns_sock = zsock_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (mdns_sock < 0) {
		LOG_ERR("mDNS: socket failed: %d", errno);
		return;
	}

	struct sockaddr_in bind_addr;

	memset(&bind_addr, 0, sizeof(bind_addr));
	bind_addr.sin_family = AF_INET;
	bind_addr.sin_port = htons(MDNS_PORT);
	bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);

	if (zsock_bind(mdns_sock, (struct sockaddr *)&bind_addr,
		       sizeof(bind_addr)) < 0) {
		LOG_ERR("mDNS: bind 5353 failed: %d (is CONFIG_MDNS_RESPONDER "
			"still enabled?)", errno);
		zsock_close(mdns_sock);
		mdns_sock = -1;
		return;
	}

	struct ip_mreqn mreq;

	memset(&mreq, 0, sizeof(mreq));
	zsock_inet_pton(AF_INET, MDNS_GROUP_ADDR, &mreq.imr_multiaddr);
	mreq.imr_ifindex = net_if_get_by_iface(net_if_get_default());
	if (zsock_setsockopt(mdns_sock, IPPROTO_IP, IP_ADD_MEMBERSHIP,
			     &mreq, sizeof(mreq)) < 0) {
		LOG_WRN("mDNS: join %s failed: %d", MDNS_GROUP_ADDR, errno);
	}

	/* RFC 6762 §11: ALL responses — multicast and the unicast replies to
	 * QU/legacy queries — must carry IP TTL 255, or strict receivers
	 * (e.g. Avahi with check-response-ttl) drop them. */
	int ttl = 255;

	(void)zsock_setsockopt(mdns_sock, IPPROTO_IP, IP_MULTICAST_TTL,
			       &ttl, sizeof(ttl));
	(void)zsock_setsockopt(mdns_sock, IPPROTO_IP, IP_TTL,
			       &ttl, sizeof(ttl));

	LOG_INF("mDNS: responder+browser on :%d (host %s)", MDNS_PORT,
		host_fqdn);

	/* First announcements + browse burst */
	k_mutex_lock(&mdns_lock, K_FOREVER);
	announce_pending = 2;
	announce_next_ms = k_uptime_get() + 200;
	reannounce_next_ms = k_uptime_get() + REANNOUNCE_INTERVAL_MS;
	browse_burst = 0;
	browse_next_ms = k_uptime_get() + 500;
	k_mutex_unlock(&mdns_lock);

	while (1) {
		struct zsock_pollfd fds = {
			.fd = mdns_sock,
			.events = ZSOCK_POLLIN,
		};
		int ret = zsock_poll(&fds, 1, 500);

		if (ret > 0 && (fds.revents & ZSOCK_POLLIN)) {
			struct sockaddr_in src;
			socklen_t srclen = sizeof(src);
			ssize_t n = zsock_recvfrom(mdns_sock, rx_buf,
						   sizeof(rx_buf), 0,
						   (struct sockaddr *)&src,
						   &srclen);

			if (n >= 12) {
				uint16_t flags = sys_get_be16(&rx_buf[2]);

				k_mutex_lock(&mdns_lock, K_FOREVER);
				if ((flags & 0x8000) == 0) {
					handle_query(rx_buf, n, &src);
				} else {
					handle_response(rx_buf, n);
				}
				k_mutex_unlock(&mdns_lock);
			}
		}

		periodic_tick();
	}
}

/* ================================================================
 * Public API
 * ================================================================ */

/* Session names become DNS labels: no dots, max 63 bytes. */
static void sanitize_label(char *s)
{
	for (; *s; s++) {
		if (*s == '.') {
			*s = '_';
		}
	}
}

/* DNS-SD instance names MUST be unique per device on the link. Announcing
 * the bare TX-stream name meant two boards with the default config both
 * advertised "TX Stream 0" — literally identical PTR records that every
 * browser on the net (ours included) collapses into ONE cache entry whose
 * SRV/A then ping-pongs between the two hosts (observed as an endless
 * host-A<->B re-DESCRIBE loop). RAVENNA convention: "<device> <session>".
 * The server's by-name DESCRIBE resolver accepts the prefixed form via a
 * suffix match (rtsp.c), so interop is keeps working in both directions. */
static void make_session_instance(char *out, size_t sz, const char *tx_name)
{
	int n;

	if (user_device_name[0] != '\0') {
		n = snprintf(out, sz, "%s %s", user_device_name, tx_name);
	} else {
		n = snprintf(out, sz, "%s", tx_name);
	}
	if (n < 0 || (size_t)n >= sz) {
		/* Truncated to the DNS label limit — acceptable, the prefix
		 * keeps the name unique. */
		out[sz - 1] = '\0';
	}
	sanitize_label(out);
}

/* TX-stream change observer (registered with aes67_conn): advertise or
 * withdraw the session record for the changed stream slot. */
static void mdns_tx_stream_observer(uint8_t stream_id)
{
	struct aes67_tx_stream tx;

	if (stream_id >= MDNS_MAX_SESSIONS) {
		return;
	}

	if (aes67_conn_copy_tx_stream(stream_id, &tx) &&
	    tx.dst_ip.s_addr != 0 && tx.name[0] != '\0') {
		mdns_sd_set_session(stream_id, tx.name);
	} else {
		mdns_sd_set_session(stream_id, NULL);
	}
}

int mdns_sd_start(void)
{
	struct aes67_device_config *cfg = aes67_config_get();

	k_mutex_init(&mdns_lock);
	k_sem_init(&describe_sem, 0, 1);

	aes67_config_build_node_id(vendor_node_id, sizeof(vendor_node_id));
	sanitize_label(vendor_node_id);

	strncpy(user_device_name, cfg->device_name,
		sizeof(user_device_name) - 1);
	user_device_name[sizeof(user_device_name) - 1] = '\0';
	sanitize_label(user_device_name);

	snprintf(host_fqdn, sizeof(host_fqdn), "%s.local",
		 net_hostname_get());

	memset(sessions, 0, sizeof(sessions));
	memset(discovered, 0, sizeof(discovered));

	/* TX streams restored from flash/SD config were configured before
	 * this module started — pick them up now; the initial announcement
	 * covers them once the socket is up. */
	const struct aes67_tx_stream *tx = aes67_conn_get_tx_streams();

	for (int i = 0; i < MDNS_MAX_SESSIONS; i++) {
		if (tx[i].active && tx[i].dst_ip.s_addr != 0 &&
		    tx[i].name[0] != '\0') {
			make_session_instance(sessions[i].name,
					      sizeof(sessions[i].name),
					      tx[i].name);
		}
	}

	mdns_started = true;

	/* Follow TX-stream changes (RAVENNA §3.5.2): advertise/withdraw the
	 * per-session record when a stream is configured or deactivated. */
	aes67_conn_register_tx_observer(mdns_tx_stream_observer);

	k_thread_create(&mdns_thread_data, mdns_stack,
			K_THREAD_STACK_SIZEOF(mdns_stack),
			mdns_thread, NULL, NULL, NULL,
			K_PRIO_PREEMPT(9), 0, K_NO_WAIT);
	k_thread_name_set(&mdns_thread_data, "mdns");

	k_thread_create(&describe_thread_data, describe_stack,
			K_THREAD_STACK_SIZEOF(describe_stack),
			describe_thread, NULL, NULL, NULL,
			K_PRIO_PREEMPT(10), 0, K_NO_WAIT);
	k_thread_name_set(&describe_thread_data, "mdns_describe");

	LOG_INF("mDNS/DNS-SD started: vendor=\"%s\" user=\"%s\"",
		vendor_node_id, user_device_name);
	return 0;
}

void mdns_sd_update_device_name(void)
{
	struct aes67_device_config *cfg = aes67_config_get();

	if (!mdns_started) {
		return;
	}

	k_mutex_lock(&mdns_lock, K_FOREVER);
	strncpy(user_device_name, cfg->device_name,
		sizeof(user_device_name) - 1);
	user_device_name[sizeof(user_device_name) - 1] = '\0';
	sanitize_label(user_device_name);

	/* Session instances carry the device name as their unique prefix —
	 * withdraw the old names and rebuild them from the TX table. */
	const struct aes67_tx_stream *tx = aes67_conn_get_tx_streams();

	for (int i = 0; i < MDNS_MAX_SESSIONS; i++) {
		if (sessions[i].name[0] != '\0' && mdns_sock >= 0) {
			goodbye_session(sessions[i].name);
		}
		sessions[i].name[0] = '\0';
		if (tx[i].active && tx[i].dst_ip.s_addr != 0 &&
		    tx[i].name[0] != '\0') {
			make_session_instance(sessions[i].name,
					      sizeof(sessions[i].name),
					      tx[i].name);
		}
	}

	announce_pending = 2;
	announce_next_ms = k_uptime_get();
	k_mutex_unlock(&mdns_lock);

	LOG_INF("mDNS: device name updated to \"%s\"", user_device_name);
}

int mdns_sd_set_session(uint8_t slot, const char *session_name)
{
	if (slot >= MDNS_MAX_SESSIONS || !mdns_started) {
		return -EINVAL;
	}

	k_mutex_lock(&mdns_lock, K_FOREVER);

	if (!session_name || session_name[0] == '\0') {
		if (sessions[slot].name[0] != '\0') {
			char old[MDNS_NAME_MAX];

			strncpy(old, sessions[slot].name, sizeof(old));
			sessions[slot].name[0] = '\0';
			if (mdns_sock >= 0) {
				goodbye_session(old);
			}
			LOG_INF("mDNS: withdrew session \"%s\"", old);
		}
		k_mutex_unlock(&mdns_lock);
		return 0;
	}

	char inst[MDNS_NAME_MAX];

	make_session_instance(inst, sizeof(inst), session_name);

	if (strcmp(sessions[slot].name, inst) != 0) {
		if (sessions[slot].name[0] != '\0' && mdns_sock >= 0) {
			goodbye_session(sessions[slot].name);
		}
		strncpy(sessions[slot].name, inst,
			sizeof(sessions[slot].name) - 1);
		sessions[slot].name[sizeof(sessions[slot].name) - 1] = '\0';
		announce_pending = 2;
		announce_next_ms = k_uptime_get();
		LOG_INF("mDNS: advertising session \"%s\"",
			sessions[slot].name);
	}

	k_mutex_unlock(&mdns_lock);
	return 0;
}

int mdns_sd_advertise_session(const char *session_name, uint16_t rtsp_port)
{
	ARG_UNUSED(rtsp_port);

	if (!session_name || session_name[0] == '\0') {
		return -EINVAL;
	}

	for (int i = 0; i < MDNS_MAX_SESSIONS; i++) {
		if (sessions[i].name[0] == '\0') {
			return mdns_sd_set_session(i, session_name);
		}
	}
	return -ENOMEM;
}

int mdns_sd_remove_session(const char *session_name)
{
	if (!session_name) {
		return -EINVAL;
	}

	for (int i = 0; i < MDNS_MAX_SESSIONS; i++) {
		if (name_eq(sessions[i].name, session_name)) {
			return mdns_sd_set_session(i, NULL);
		}
	}
	return -ENOENT;
}
