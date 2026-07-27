/*
 * NMOS IS-04 node — internal interfaces.
 *
 * nmos_node.c owns the resource model (UUIDs, version timestamps, JSON
 * builders over the live device state); nmos_api.c is the HTTP surface
 * and the later registration client reuses the same builders for its
 * POST bodies.
 */

#ifndef NMOS_PRIV_H_
#define NMOS_PRIV_H_

#include <zephyr/net/net_ip.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "../webapi/webapi_priv.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NMOS_API_VERSION  "v1.3"
#define NMOS_HTTP_PORT    80
/* Node API interface token, referenced by sender/receiver
 * interface_bindings. There is exactly one network interface. */
#define NMOS_IFACE_NAME   "eth0"

#define NMOS_UUID_STR_LEN 37 /* 36 chars + NUL */

/* Resource kinds; the (kind, index) pair determines the deterministic
 * per-device UUID, so the values must never be reordered. */
enum nmos_res_kind {
	NMOS_RES_NODE = 0,
	NMOS_RES_DEVICE = 1,
	NMOS_RES_SOURCE = 2,
	NMOS_RES_FLOW = 3,
	NMOS_RES_SENDER = 4,
	NMOS_RES_RECEIVER = 5,
};

/* TAI timestamp for resource "version" fields (<seconds>:<nanoseconds>). */
struct nmos_tai {
	uint64_t sec;
	uint32_t nsec;
};

/* Strictly monotonic TAI from the FPGA wallclock (uptime fallback). */
void nmos_tai_now(struct nmos_tai *t);

/* Deterministic per-device UUID (lowercase, RFC 4122 v4 format). */
void nmos_uuid(enum nmos_res_kind kind, int index,
	       char out[NMOS_UUID_STR_LEN]);

/* Current node IP as dotted quad ("0.0.0.0" until an address exists). */
void nmos_ip_str(char *buf, size_t sz);

/* Number of TX/RX stream slots the loaded gateware was built with
 * (system_cfg CSRs), bounded by the firmware table sizes. */
int nmos_tx_count(void);
int nmos_rx_count(void);

/* JSON resource builders (each emits one complete {...} object).
 * The per-index builders return false when the slot does not currently
 * exist as a resource (inactive TX slot); nothing is written then. */
void nmos_build_self(struct json_out *jo);
void nmos_build_device(struct json_out *jo);
bool nmos_build_source(struct json_out *jo, int idx);
bool nmos_build_flow(struct json_out *jo, int idx);
bool nmos_build_sender(struct json_out *jo, int idx);
bool nmos_build_receiver(struct json_out *jo, int idx);

/* SDP manifest for sender idx; returns body length or negative errno
 * (-ENOENT if the TX slot is inactive). */
int nmos_build_manifest(char *buf, size_t sz, int idx);

/* True once an IPv4 address has been applied. */
bool nmos_have_ip(void);

/* Registration state drives the DNS-SD advertisement: _nmos-node._tcp
 * is withdrawn while a Registration API is present on the network or
 * the node is registered (IS-04 v1.3); otherwise the node advertises
 * in P2P mode with ver_* records. */
void nmos_set_registered(bool registered);
void nmos_set_registry_present(bool present);

#ifdef CONFIG_NMOS_REGISTRATION
void nmos_reg_start(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* NMOS_PRIV_H_ */
