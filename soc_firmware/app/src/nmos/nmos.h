/*
 * NMOS (AMWA IS-04) node — public interface.
 *
 * The Node API itself (/x-nmos/node/...) is served by the shared HTTP
 * server and needs no explicit start; nmos_start() arms the version
 * bookkeeping (TAI timestamps bumped on stream-table changes via the
 * aes67_conn observers).
 */

#ifndef NMOS_H_
#define NMOS_H_

#include <zephyr/net/net_ip.h>

#ifdef __cplusplus
extern "C" {
#endif

int nmos_start(void);

/* Called for every address transition (link-local adoption, DHCP bind,
 * lease change) — the Node resource's href/api endpoints follow the IP. */
void nmos_notify_ip_ready(const struct in_addr *addr);

/* ---- DNS-SD advertisement state (consumed by mdns_sd) ----
 *
 * IS-04 v1.3: the _nmos-node._tcp advertisement is for peer-to-peer
 * operation and SHOULD be withdrawn while the node is registered with a
 * registry; the ver_* TXT records (8-bit counters, incremented on every
 * change of the corresponding Node API collection) exist only in P2P
 * mode. mdns_sd polls this and re-announces on change. */
struct nmos_mdns_info {
	bool advertise;   /* advertise _nmos-node._tcp at all */
	bool p2p;         /* include the ver_* TXT records */
	uint8_t ver_slf;
	uint8_t ver_src;
	uint8_t ver_flw;
	uint8_t ver_dvc;
	uint8_t ver_snd;
	uint8_t ver_rcv;
};

void nmos_get_mdns_info(struct nmos_mdns_info *out);

#ifdef __cplusplus
}
#endif

#endif /* NMOS_H_ */
