/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: Apache-2.0
 *
 * FPGA HAL backend — FMC (STM32H7) implementation.
 *
 * Maps fpga_hal_*() calls to eth_fmc_*() functions.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include "fpga_hal.h"
#include "../eth_fmc_basic/eth_fmc_basic.h"

LOG_MODULE_REGISTER(fpga_hal, LOG_LEVEL_INF);

/* Cached recover callback (HAL uses a simpler signature than FMC) */
static fpga_hal_recover_cb_t g_recover_cb;
static void *g_recover_user_data;

static void fmc_recover_trampoline(const struct device *dev, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);
	if (g_recover_cb) {
		g_recover_cb(g_recover_user_data);
	}
}

/* ---- Device accessor ---- */

const struct device *fpga_hal_get_dev(void)
{
	return device_get_binding("eth_fmc0");
}

/* ---- FPGA ready / recovery ---- */

bool fpga_hal_is_ready(void)
{
	return eth_fmc_is_fpga_ready();
}

int fpga_hal_wait_ready(uint32_t timeout_ms)
{
	return eth_fmc_wait_for_fpga_ready(timeout_ms);
}

void fpga_hal_register_recover_cb(fpga_hal_recover_cb_t cb, void *user_data)
{
	g_recover_cb = cb;
	g_recover_user_data = user_data;

	if (cb) {
		eth_fmc_register_fpga_recover_cb(fmc_recover_trampoline, NULL);
	} else {
		eth_fmc_register_fpga_recover_cb(NULL, NULL);
	}
}

/* ---- Configuration writes ---- */

int fpga_hal_write_mac(const uint8_t mac[6])
{
	const struct device *dev = fpga_hal_get_dev();

	if (!dev) {
		return -ENODEV;
	}
	return eth_fmc_reg_write(dev, ETH_FMC_REG_MAC_ADDR, mac, 6);
}

int fpga_hal_write_ip(const struct in_addr *ip)
{
	const struct device *dev = fpga_hal_get_dev();

	if (!dev) {
		return -ENODEV;
	}
	return eth_fmc_reg_write(dev, ETH_FMC_REG_IP_ADDR,
				 (const uint8_t *)&ip->s_addr, 4);
}

int fpga_hal_write_ptp_config(uint8_t time_source,
			      int8_t log_msg_interval,
			      int8_t log_announce_interval)
{
	const struct device *dev = fpga_hal_get_dev();
	static const uint8_t zero_id[8] = {0};

	if (!dev) {
		return -ENODEV;
	}
	/* Legacy FMC backend signature still takes a leader_clock_id; pass zeros
	 * since the FPGA BMA now owns leader selection. */
	return eth_fmc_write_ptp_config(dev, zero_id, time_source,
					log_msg_interval, log_announce_interval);
}

bool fpga_hal_read_ptp_leader_id(uint8_t leader_clock_id[8])
{
	/* FMC backend has no readback for FPGA-selected leader ID. */
	memset(leader_clock_id, 0, 8);
	return false;
}

int fpga_hal_write_ptp_gm_quality(uint8_t priority1, uint8_t priority2,
				  uint8_t clock_class, uint8_t clock_accuracy)
{
	/* FMC backend does not support GM quality CSRs */
	ARG_UNUSED(priority1);
	ARG_UNUSED(priority2);
	ARG_UNUSED(clock_class);
	ARG_UNUSED(clock_accuracy);
	return 0;
}

int fpga_hal_write_tx_stream_config(uint8_t stream_id,
				    const struct in_addr *dst_ip,
				    uint8_t channel_count,
				    uint8_t samples_per_pkt,
				    const uint8_t *ch_ids,
				    uint8_t num_ch_ids,
				    uint32_t ssrc)
{
	const struct device *dev = fpga_hal_get_dev();

	if (!dev) {
		return -ENODEV;
	}
	return eth_fmc_write_tx_stream_config(dev, stream_id, dst_ip,
					      channel_count, samples_per_pkt,
					      ch_ids, num_ch_ids, ssrc);
}

int fpga_hal_write_rx_stream_config(uint8_t stream_id,
				    const struct in_addr *dst_ip,
				    uint16_t dst_port,
				    const uint8_t *ch_map,
				    uint8_t channel_count,
				    uint8_t output_delay,
				    uint8_t samples_per_channel)
{
	const struct device *dev = fpga_hal_get_dev();

	if (!dev) {
		return -ENODEV;
	}
	return eth_fmc_write_rx_stream_config(dev, stream_id, dst_ip,
					      dst_port, ch_map, channel_count,
					      output_delay, samples_per_channel);
}

/* ---- Control register ---- */

int fpga_hal_ctrl_set_bits(uint32_t bits)
{
	const struct device *dev = fpga_hal_get_dev();

	if (!dev) {
		return -ENODEV;
	}

	/* HAL control flags map 1:1 to FMC flag bits */
	uint8_t fmc_bits = 0;

	if (bits & FPGA_HAL_CTRL_PPB_START) {
		fmc_bits |= ETH_FMC_FLAG_PPB_START;
	}
	if (bits & FPGA_HAL_CTRL_RESET_WALLCLOCK) {
		fmc_bits |= ETH_FMC_FLAG_RESET_WALLCLOCK;
	}
	if (bits & FPGA_HAL_CTRL_RESET_PTP) {
		fmc_bits |= ETH_FMC_FLAG_RESET_PTP;
	}
	if (bits & FPGA_HAL_CTRL_RESET_ETHERNET) {
		fmc_bits |= ETH_FMC_FLAG_RESET_ETHERNET;
	}
	return eth_fmc_status_set_bits(dev, fmc_bits);
}

int fpga_hal_ctrl_clear_bits(uint32_t bits)
{
	const struct device *dev = fpga_hal_get_dev();

	if (!dev) {
		return -ENODEV;
	}

	uint8_t fmc_bits = 0;

	if (bits & FPGA_HAL_CTRL_PPB_START) {
		fmc_bits |= ETH_FMC_FLAG_PPB_START;
	}
	if (bits & FPGA_HAL_CTRL_RESET_WALLCLOCK) {
		fmc_bits |= ETH_FMC_FLAG_RESET_WALLCLOCK;
	}
	if (bits & FPGA_HAL_CTRL_RESET_PTP) {
		fmc_bits |= ETH_FMC_FLAG_RESET_PTP;
	}
	if (bits & FPGA_HAL_CTRL_RESET_ETHERNET) {
		fmc_bits |= ETH_FMC_FLAG_RESET_ETHERNET;
	}
	return eth_fmc_status_clear_bits(dev, fmc_bits);
}

/* ---- Status reads ---- */

uint32_t fpga_hal_read_status(void)
{
	const struct device *dev = fpga_hal_get_dev();
	uint32_t hal = 0;
	uint8_t val;

	if (!dev) {
		return 0;
	}

	/* Clocking status (register 0x50 read) */
	if (eth_fmc_reg_read(dev, ETH_FMC_REG_STATUS_CLK, &val) == 0) {
		if (val & ETH_FMC_CLK_PPB_VALID) {
			hal |= FPGA_HAL_CLK_PPB_VALID;
		}
		if (val & ETH_FMC_CLK_WC_LOCKED) {
			hal |= FPGA_HAL_CLK_WC_LOCKED;
		}
		if (val & ETH_FMC_CLK_WC_PHASEJUMP) {
			hal |= FPGA_HAL_CLK_WC_PHASEJUMP;
		}
		if (val & ETH_FMC_CLK_WC_CONFIGURED) {
			hal |= FPGA_HAL_CLK_WC_CONFIGURED;
		}
		if (val & ETH_FMC_CLK_PTP_LEADER_LOST) {
			hal |= FPGA_HAL_CLK_PTP_LEADER_LOST;
		}
	}

	/* Ethernet status (register 0x51 read) */
	if (eth_fmc_reg_read(dev, ETH_FMC_REG_STATUS_ETH, &val) == 0) {
		if (val & ETH_FMC_ETH_LINK_UP) {
			hal |= FPGA_HAL_ETH_LINK_UP;
		}
		uint32_t speed = (val & ETH_FMC_ETH_SPEED_MASK) >> ETH_FMC_ETH_SPEED_SHIFT;
		hal |= (speed << FPGA_HAL_ETH_SPEED_SHIFT);
	}

	return hal;
}

int32_t fpga_hal_read_path_delay(void)
{
	const struct device *dev = fpga_hal_get_dev();
	uint8_t buf[4];

	if (!dev) {
		return 0;
	}
	if (eth_fmc_reg_read_block(dev, ETH_FMC_REG_PATH_DELAY, buf, 4) < 0) {
		return 0;
	}

	return (int32_t)((uint32_t)buf[0] |
			 ((uint32_t)buf[1] << 8) |
			 ((uint32_t)buf[2] << 16) |
			 ((uint32_t)buf[3] << 24));
}

int32_t fpga_hal_read_ptp_offset(void)
{
	const struct device *dev = fpga_hal_get_dev();
	uint8_t buf[4];

	if (!dev) {
		return 0;
	}
	if (eth_fmc_reg_read_block(dev, ETH_FMC_REG_LEADER_OFFSET, buf, 4) < 0) {
		return 0;
	}

	return (int32_t)((uint32_t)buf[0] |
			 ((uint32_t)buf[1] << 8) |
			 ((uint32_t)buf[2] << 16) |
			 ((uint32_t)buf[3] << 24));
}

bool fpga_hal_read_ppb_counts(uint32_t *wc_count, uint32_t *pll_count)
{
	const struct device *dev = fpga_hal_get_dev();
	uint8_t buf[4];

	if (!dev) {
		*wc_count = 0;
		*pll_count = 0;
		return false;
	}

	if (eth_fmc_reg_read_block(dev, ETH_FMC_REG_COUNT_WC, buf, 4) < 0) {
		*wc_count = 0;
		*pll_count = 0;
		return false;
	}
	*wc_count = ((uint32_t)buf[0] |
		     ((uint32_t)buf[1] << 8) |
		     ((uint32_t)buf[2] << 16)) & 0x3FFFFF;

	if (eth_fmc_reg_read_block(dev, ETH_FMC_REG_COUNT_PLL, buf, 4) < 0) {
		*pll_count = 0;
		return false;
	}
	*pll_count = ((uint32_t)buf[0] |
		      ((uint32_t)buf[1] << 8) |
		      ((uint32_t)buf[2] << 16)) & 0x3FFFFF;

	/* Check PPB_VALID flag */
	uint8_t status;
	if (eth_fmc_reg_read(dev, ETH_FMC_REG_STATUS_CLK, &status) < 0) {
		return false;
	}
	return (status & ETH_FMC_CLK_PPB_VALID) != 0;
}
