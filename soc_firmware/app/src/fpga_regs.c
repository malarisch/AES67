#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/net/net_if.h>
#include <zephyr/logging/log.h>

#include "../drivers/eth_fmc_basic/eth_fmc_basic.h"
#include "fpga_regs.h"

LOG_MODULE_REGISTER(fpga_regs, LOG_LEVEL_INF);

int fpga_write_mac_address(struct net_if *iface)
{
	const struct device *fmc = device_get_binding("eth_fmc0");
	struct net_linkaddr *ll;

	if (!fmc) {
		LOG_ERR("FMC device not found");
		return -ENODEV;
	}

	ll = net_if_get_link_addr(iface);
	if (!ll || ll->len < 6) {
		LOG_ERR("No valid MAC address on interface");
		return -EINVAL;
	}

	int ret = eth_fmc_reg_write(fmc, ETH_FMC_REG_MAC_ADDR, ll->addr, 6);
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
	const struct device *fmc = device_get_binding("eth_fmc0");

	if (!fmc) {
		LOG_ERR("FMC device not found");
		return -ENODEV;
	}

	int ret = eth_fmc_reg_write(fmc, ETH_FMC_REG_IP_ADDR,
				    (const uint8_t *)&addr->s_addr, 4);
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

int fpga_wait_for_link_up(uint32_t timeout_ms)
{
	const struct device *fmc = device_get_binding("eth_fmc0");
	uint32_t elapsed = 0;
	uint8_t eth_status;

	if (!fmc) {
		return -ENODEV;
	}

	LOG_INF("Waiting for Ethernet link...");

	while (1) {
		if (eth_fmc_reg_read(fmc, ETH_FMC_REG_STATUS_ETH, &eth_status) == 0) {
			if (eth_status & ETH_FMC_ETH_LINK_UP) {
				LOG_INF("Link up after %u ms", elapsed);
				return 0;
			}
		}

		if (timeout_ms > 0 && elapsed >= timeout_ms) {
			LOG_WRN("Link-up timeout (%u ms)", timeout_ms);
			return -ETIMEDOUT;
		}

		k_msleep(100);
		elapsed += 100;
	}
}

int fpga_read_32(const struct device *fmc, uint8_t reg, int32_t *val)
{
	uint8_t buf[4];
	int ret;

	ret = eth_fmc_reg_read_block(fmc, reg, buf, 4);
	if (ret < 0) {
		return ret;
	}

	*val = (int32_t)((uint32_t)buf[0] |
			 ((uint32_t)buf[1] << 8) |
			 ((uint32_t)buf[2] << 16) |
			 ((uint32_t)buf[3] << 24));
	return 0;
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

int fpga_read_ppb(const struct device *fmc, int32_t *ppb_out,
		  uint32_t *count_wc_out, uint32_t *count_pll_out)
{
	uint32_t count_wc, count_pll;
	int ret;

	ret = fpga_read_32(fmc, ETH_FMC_REG_COUNT_WC, (int32_t *)&count_wc);
	if (ret < 0) {
		return ret;
	}
	ret = fpga_read_32(fmc, ETH_FMC_REG_COUNT_PLL, (int32_t *)&count_pll);
	if (ret < 0) {
		return ret;
	}

	count_wc &= 0x3FFFFF;
	count_pll &= 0x3FFFFF;

	*ppb_out = fpga_calculate_ppb(count_wc, count_pll);

	if (count_wc_out) {
		*count_wc_out = count_wc;
	}
	if (count_pll_out) {
		*count_pll_out = count_pll;
	}

	return 0;
}
