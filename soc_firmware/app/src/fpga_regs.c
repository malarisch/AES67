#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/net/net_if.h>
#include <zephyr/logging/log.h>

#include "../drivers/fpga_hal/fpga_hal.h"
#include "fpga_regs.h"

LOG_MODULE_REGISTER(fpga_regs, LOG_LEVEL_INF);

int fpga_write_mac_address(struct net_if *iface)
{
	struct net_linkaddr *ll;

	ll = net_if_get_link_addr(iface);
	if (!ll || ll->len < 6) {
		LOG_ERR("No valid MAC address on interface");
		return -EINVAL;
	}

	int ret = fpga_hal_write_mac(ll->addr);
	if (ret < 0) {
		LOG_ERR("Failed to write MAC to FPGA: %d", ret);
		return ret;
	}

	LOG_INF("FPGA MAC set to %02x:%02x:%02x:%02x:%02x:%02x",
		ll->addr[0], ll->addr[1], ll->addr[2],
		ll->addr[3], ll->addr[4], ll->addr[5]);

	return 0;
}

int fpga_write_ip_address(const struct in_addr *addr)
{
	int ret = fpga_hal_write_ip(addr);
	if (ret < 0) {
		LOG_ERR("Failed to write IP to FPGA: %d", ret);
		return ret;
	}

	LOG_INF("FPGA IP set to %u.%u.%u.%u",
		((const uint8_t *)&addr->s_addr)[0],
		((const uint8_t *)&addr->s_addr)[1],
		((const uint8_t *)&addr->s_addr)[2],
		((const uint8_t *)&addr->s_addr)[3]);

	return 0;
}

void fpga_wait_for_link_up(void)
{
	uint32_t elapsed = 0;

	LOG_INF("Waiting for Ethernet link...");

	while (!(fpga_hal_read_status() & FPGA_HAL_ETH_LINK_UP)) {
		k_msleep(100);
		elapsed += 100;

		/* Deliberately without a timeout: everything the boot does from
		 * here on (DHCP, PTP, discovery, stream setup) needs the wire.
		 * Giving up after N seconds only produced a node that looked
		 * booted but had none of that — so keep waiting and keep
		 * saying why. */
		if (elapsed % 5000 == 0) {
			LOG_WRN("Still no Ethernet link after %u s — check the "
				"cable / link partner", elapsed / 1000);
		}
	}

	LOG_INF("Link up after %u ms", elapsed);
}

int32_t fpga_calculate_ppb(uint32_t count_wc, uint32_t count_pll)
{
	if (count_wc == 0) {
		return 0;
	}

	int32_t diff = (int32_t)count_pll - (int32_t)count_wc;
	int64_t ppb = ((int64_t)diff * 1000000000LL) / (int64_t)count_wc;

	if (ppb > INT32_MAX) {
		return INT32_MAX;
	}
	if (ppb < INT32_MIN) {
		return INT32_MIN;
	}
	return (int32_t)ppb;
}

int fpga_read_ppb(int32_t *ppb_out,
		  uint32_t *count_wc_out, uint32_t *count_pll_out)
{
	uint32_t count_wc, count_pll;

	if (!fpga_hal_read_ppb_counts(&count_wc, &count_pll)) {
		return -EIO;
	}

	*ppb_out = fpga_calculate_ppb(count_wc, count_pll);

	if (count_wc_out) {
		*count_wc_out = count_wc;
	}
	if (count_pll_out) {
		*count_pll_out = count_pll;
	}

	return 0;
}
