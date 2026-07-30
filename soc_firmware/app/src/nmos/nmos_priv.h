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
#include "../aes67_conn.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NMOS_API_VERSION  "v1.3"
#define NMOS_HTTP_PORT    80
/* Node API interface token, referenced by sender/receiver
 * interface_bindings. There is exactly one network interface. */
#define NMOS_IFACE_NAME   "eth0"

#define NMOS_UUID_STR_LEN 37 /* 36 chars + NUL */
/* "<hostname>/<kind>/a<idx>" resource labels */
#define NMOS_LABEL_MAX    64

/* Large NMOS buffers/stores go to PSRAM on the ESP32 (dram0 is tight);
 * everywhere else this is a no-op. */
#if defined(CONFIG_ESP_SPIRAM)
#define NMOS_BIG_BSS __attribute__((section(".ext_ram.bss")))
#else
#define NMOS_BIG_BSS
#endif

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

/* Snapshot of a TX slot; inactive slots yield the persistent
 * default-configured placeholder. Returns true if the slot is active. */
struct aes67_tx_stream;
bool nmos_tx_snapshot(int idx, struct aes67_tx_stream *out);

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

/* ================================================================
 * IS-05 Connection API (nmos_is05.c)
 * ================================================================ */

#define NMOS_TAI_STR_MAX   28

struct nj_node;

#ifdef CONFIG_NMOS_IS05

#define NMOS_IS05_VERSION  "v1.1"

/* Activation echo for the PATCH response: unlike a subsequent GET
 * /staged, the response to an immediate activation still reports the
 * mode and the actual activation time. */
struct is05_act_echo {
	uint8_t mode;              /* enum act_mode (0 = none) */
	bool has_req;
	char req_str[NMOS_TAI_STR_MAX];
	bool has_time;
	struct nmos_tai when;
};

void nmos_is05_init(void);

/* Stage a PATCH body (or one bulk item); performs/schedules any
 * requested activation. Returns the HTTP status (200/202/400/423/500);
 * on failure *errmsg carries the error text. */
int nmos_is05_stage(bool sender, int idx, const struct nj_node *pool,
		    const struct nj_node *req, struct is05_act_echo *echo,
		    const char **errmsg);

/* echo == NULL renders the plain GET /staged view. */
void nmos_is05_build_staged(struct json_out *jo, bool sender, int idx,
			    const struct is05_act_echo *echo);
void nmos_is05_build_active(struct json_out *jo, bool sender, int idx);
void nmos_is05_build_constraints(struct json_out *jo, bool sender, int idx);

/* IS-04 subscription state: fills the active peer id (empty = null,
 * masked while disabled) and master_enable; returns true if an id is
 * set. */
bool nmos_is05_sub(bool sender, int idx, char id_out[NMOS_UUID_STR_LEN],
		   bool *active);

/* True while a scheduled activation is pending on the resource. */
bool nmos_is05_locked(bool sender, int idx);

#endif /* CONFIG_NMOS_IS05 */

/* TAI helpers ("<sec>:<nsec>", nsec zero-padded on output). */
void nmos_tai_str(char *buf, size_t sz, const struct nmos_tai *t);
bool nmos_tai_parse(const char *s, struct nmos_tai *out);

/* Bump the Device version + registration counter (IS-08 map changes). */
void nmos_device_touch(void);

/* ================================================================
 * IS-08 Channel Mapping API (nmos_is08.c)
 * ================================================================ */

#ifdef CONFIG_NMOS_IS08

#define NMOS_IS08_VERSION  "v1.0"

void nmos_is08_init(void);

/* GET builders. The *_child builders return false for an unknown id
 * (caller answers 404). kind: 0=properties 1=parent/sourceid
 * 2=channels 3=caps. */
void nmos_is08_build_io(struct json_out *jo);
void nmos_is08_build_list(struct json_out *jo, bool inputs);
bool nmos_is08_build_child(struct json_out *jo, bool input,
			   const char *id, int kind);
bool nmos_is08_known_id(bool input, const char *id);
void nmos_is08_build_active(struct json_out *jo, const char *output_or_null);
void nmos_is08_build_activations(struct json_out *jo);
bool nmos_is08_build_activation(struct json_out *jo, const char *act_id);
bool nmos_is08_delete_activation(const char *act_id);

/* POST /map/activations. Returns the HTTP status (200/202/400/423/500);
 * on success the response body (keyed by the new activation id) has
 * been written to jo, on failure *errmsg carries the error text. */
int nmos_is08_post_activation(struct json_out *jo,
			      const struct nj_node *pool,
			      const struct nj_node *req,
			      const char **errmsg);

/* IS-05 integration: desired TX ch_ids for a slot (always filled), and
 * the desired RX ch_map for an explicit IS-05 activation — un-mutes the
 * slot and adopts an identity block when no routing is stored yet. */
void nmos_is08_tx_chids(int idx, uint8_t ch_ids[AES67_MAX_CH_PER_STREAM]);
bool nmos_is08_rx_chmap(int idx, uint8_t channels,
			uint8_t ch_map[AES67_MAX_CH_PER_STREAM]);

/* True while IS-08 itself is writing aes67_conn (observers of other
 * modules should not treat that as an external state change). */
bool nmos_is08_applying(void);

#endif /* CONFIG_NMOS_IS08 */

#ifdef __cplusplus
}
#endif

#endif /* NMOS_PRIV_H_ */
