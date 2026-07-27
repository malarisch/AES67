/*
 *
 * /api/network — interface addressing + PHY link state.
 */

#include <zephyr/kernel.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/socket.h>

#include "webapi_priv.h"
#include "../ui_display.h"
#include "../ieee1588_utils.h"

static void format_ip(char *out, size_t sz, const struct in_addr *addr)
{
	zsock_inet_ntop(AF_INET, addr, out, sz);
}

void webapi_build_network(struct json_out *jo)
{
	struct net_if *iface = net_if_get_default();
	char tmp[INET_ADDRSTRLEN];

	jo_obj_begin(jo);

	if (iface) {
		struct net_linkaddr *ll = net_if_get_link_addr(iface);

		if (ll && ll->len >= 6) {
			char mac[18];

			snprintf(mac, sizeof(mac),
				 "%02x:%02x:%02x:%02x:%02x:%02x",
				 ll->addr[0], ll->addr[1], ll->addr[2],
				 ll->addr[3], ll->addr[4], ll->addr[5]);
			jo_str(jo, "mac", mac);
		}

		struct net_if_ipv4 *ipv4 = iface->config.ip.ipv4;
		const struct net_if_addr_ipv4 *best = NULL;

		/* The interface carries up to NET_IF_UNICAST_IPV4_ADDR_COUNT
		 * (2) addresses, and slot order is arbitrary: during the
		 * link-local -> DHCP switchover both exist, and after the LL
		 * address is removed the lease may well live in slot 1 while
		 * slot 0 is empty. Reading unicast[0] blindly therefore
		 * reported 0.0.0.0 / the stale LL address depending on slot
		 * layout. Scan all slots and prefer a non-link-local address —
		 * that is the node identity main.c announces everywhere else. */
		if (ipv4) {
			ARRAY_FOR_EACH(ipv4->unicast, slot) {
				const struct net_if_addr_ipv4 *a =
					&ipv4->unicast[slot];

				if (!a->ipv4.is_used) {
					continue;
				}
				if (best == NULL ||
				    net_ipv4_is_ll_addr(&best->ipv4.address.in_addr)) {
					best = a;
				}
			}
		}

		if (best != NULL) {
			format_ip(tmp, sizeof(tmp),
				  &best->ipv4.address.in_addr);
			jo_str(jo, "ip", tmp);

			format_ip(tmp, sizeof(tmp), &best->netmask);
			jo_str(jo, "netmask", tmp);

			format_ip(tmp, sizeof(tmp), &ipv4->gw);
			jo_str(jo, "gateway", tmp);
		} else {
			jo_str(jo, "ip", "0.0.0.0");
		}

		jo_bool(jo, "link_up", net_if_is_up(iface));
	}

	/* FPGA-reported link info */
	struct ui_fpga_metrics m = {0};

	if (webapi_read_fpga_metrics(&m) == 0) {
		jo_str(jo, "phy_speed", eth_speed_to_text(m.speed_code));
		jo_bool(jo, "phy_link_up", m.link_up);
	}

	jo_obj_end(jo);
}

static int get_network(struct webapi_request *req)
{
	webapi_build_network(&req->out);
	return 0;
}

static const struct webapi_route routes[] = {
	WEBAPI_ROUTE(HTTP_GET, "/api/network", get_network),
};

const struct webapi_module webapi_network_module = {
	.routes = routes,
	.count = ARRAY_SIZE(routes),
};
