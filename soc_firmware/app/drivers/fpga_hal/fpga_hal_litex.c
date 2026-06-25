/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: Apache-2.0
 *
 * FPGA HAL backend — LiteX CSR implementation.
 *
 * Maps fpga_hal_*() calls to eth_litex_*() functions and direct
 * CSR register access.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include "fpga_hal.h"
#include "../eth_litex/eth_litex.h"

LOG_MODULE_REGISTER(fpga_hal, LOG_LEVEL_INF);

/* ---- Device accessor ---- */

const struct device *fpga_hal_get_dev(void)
{
	return device_get_binding("eth_litex0");
}

/* ---- FPGA ready / recovery ---- */

bool fpga_hal_is_ready(void)
{
	/* LiteX SoC: FPGA logic is part of the same bitstream,
	 * so it is always ready when the CPU is running. */
	return true;
}

int fpga_hal_wait_ready(uint32_t timeout_ms)
{
	ARG_UNUSED(timeout_ms);
	return 0;
}

void fpga_hal_register_recover_cb(fpga_hal_recover_cb_t cb, void *user_data)
{
	ARG_UNUSED(cb);
	ARG_UNUSED(user_data);
	/* No-op: LiteX FPGA is always ready. */
}

/* ---- Configuration writes ---- */

int fpga_hal_write_mac(const uint8_t mac[6])
{
	const struct device *dev = fpga_hal_get_dev();

	if (!dev) {
		return -ENODEV;
	}
	return eth_litex_write_mac(dev, mac);
}

int fpga_hal_write_ip(const struct in_addr *ip)
{
	const struct device *dev = fpga_hal_get_dev();

	if (!dev) {
		return -ENODEV;
	}
	return eth_litex_write_ip(dev, ip);
}

int fpga_hal_write_ptp_config(uint8_t time_source,
			      int8_t log_msg_interval,
			      int8_t log_announce_interval)
{
	const struct device *dev = fpga_hal_get_dev();

	if (!dev) {
		return -ENODEV;
	}
	return eth_litex_write_ptp_config(dev, time_source,
					  log_msg_interval, log_announce_interval);
}

bool fpga_hal_read_ptp_leader_id(uint8_t leader_clock_id[8])
{
	const struct device *dev = fpga_hal_get_dev();

	if (!dev) {
		memset(leader_clock_id, 0, 8);
		return false;
	}
	return eth_litex_read_ptp_leader_id(dev, leader_clock_id);
}

int fpga_hal_write_ptp_gm_quality(uint8_t priority1, uint8_t priority2,
				  uint8_t clock_class, uint8_t clock_accuracy)
{
	const struct device *dev = fpga_hal_get_dev();

	if (!dev) {
		return -ENODEV;
	}
	return eth_litex_write_ptp_gm_quality(dev, priority1, priority2,
					       clock_class, clock_accuracy);
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
	return eth_litex_write_tx_stream_config(dev, stream_id, dst_ip,
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
	return eth_litex_write_rx_stream_config(dev, stream_id, dst_ip,
						dst_port, ch_map,
						channel_count, output_delay,
						samples_per_channel);
}

/* ---- Control register ---- */

int fpga_hal_ctrl_set_bits(uint32_t bits)
{
	const struct device *dev = fpga_hal_get_dev();

	if (!dev) {
		return -ENODEV;
	}

	/* Map HAL control flags to LiteX control register bits. */
	uint32_t litex_bits = 0;

	if (bits & FPGA_HAL_CTRL_PPB_START) {
		litex_bits |= AES67_CTRL_PPB_START;
	}

	return eth_litex_ctrl_set_bits(dev, litex_bits);
}

int fpga_hal_ctrl_clear_bits(uint32_t bits)
{
	const struct device *dev = fpga_hal_get_dev();

	if (!dev) {
		return -ENODEV;
	}

	uint32_t litex_bits = 0;

	if (bits & FPGA_HAL_CTRL_PPB_START) {
		litex_bits |= AES67_CTRL_PPB_START;
	}

	return eth_litex_ctrl_clear_bits(dev, litex_bits);
}

int fpga_hal_set_adda_nrst(bool released)
{
	const struct device *dev = fpga_hal_get_dev();

	if (!dev) {
		return -ENODEV;
	}

	if (released) {
		return eth_litex_ctrl_set_bits(dev, AES67_CTRL_ADDA_NRST);
	} else {
		return eth_litex_ctrl_clear_bits(dev, AES67_CTRL_ADDA_NRST);
	}
}

/* ---- Status reads ---- */

uint32_t fpga_hal_read_status(void)
{
	const struct device *dev = fpga_hal_get_dev();

	if (!dev) {
		return 0;
	}

	uint32_t raw = eth_litex_read_status(dev);
	uint32_t hal = 0;

	/* Map LiteX status bits → HAL status bits.
	 * wallclock_phasejump and ptp_sync_lost were removed from the bridge, so
	 * the corresponding HAL bits are never set by this backend. */
	if (raw & AES67_STATUS_WC_LOCKED) {
		hal |= FPGA_HAL_CLK_WC_LOCKED;
	}
	if (raw & AES67_STATUS_WC_CONFIGURED) {
		hal |= FPGA_HAL_CLK_WC_CONFIGURED;
	}
	if (raw & AES67_STATUS_ETH_LINK_UP) {
		hal |= FPGA_HAL_ETH_LINK_UP;
	}
	if (raw & AES67_STATUS_PTP_IS_LEADER) {
		hal |= FPGA_HAL_PTP_IS_LEADER;
	}
	if (raw & AES67_STATUS_PTP_IS_FOLLOWER) {
		hal |= FPGA_HAL_PTP_IS_FOLLOWER;
	}

	/* Map speed field */
	uint32_t speed = (raw & AES67_STATUS_ETH_SPEED_MASK) >> AES67_STATUS_ETH_SPEED_SHIFT;
	hal |= (speed << FPGA_HAL_ETH_SPEED_SHIFT);

	/* PPB valid is in a separate register on LiteX */
	uint32_t ppb_wc, ppb_pll;
	if (eth_litex_read_ppb_counts(dev, &ppb_wc, &ppb_pll)) {
		hal |= FPGA_HAL_CLK_PPB_VALID;
	}

	return hal;
}

int32_t fpga_hal_read_path_delay(void)
{
	const struct device *dev = fpga_hal_get_dev();

	if (!dev) {
		return 0;
	}
	return (int32_t)eth_litex_read_path_delay(dev);
}

int32_t fpga_hal_read_ptp_offset(void)
{
	const struct device *dev = fpga_hal_get_dev();

	if (!dev) {
		return 0;
	}
	return (int32_t)eth_litex_read_ptp_offset(dev);
}

bool fpga_hal_read_ppb_counts(uint32_t *wc_count, uint32_t *pll_count)
{
	const struct device *dev = fpga_hal_get_dev();

	if (!dev) {
		*wc_count = 0;
		*pll_count = 0;
		return false;
	}
	return eth_litex_read_ppb_counts(dev, wc_count, pll_count);
}

/* ---- PTP servo tuning + monitoring ---- */

int fpga_hal_write_ptp_tuning(const struct fpga_hal_ptp_tuning *t)
{
	const struct device *dev = fpga_hal_get_dev();

	if (!dev) {
		return -ENODEV;
	}

	struct eth_litex_ptp_tuning lt = {
		.kp_gain                 = t->kp_gain,
		.ki_gain                 = t->ki_gain,
		.gain_shift              = t->gain_shift,
		.gain_shift_locked       = t->gain_shift_locked,
		.ki_extra_shift          = t->ki_extra_shift,
		.filter_shift            = t->filter_shift,
		.warmup_samples          = t->warmup_samples,
		.lock_threshold_ns       = t->lock_threshold_ns,
		.unlock_threshold_ns     = t->unlock_threshold_ns,
		.lock_count_threshold    = t->lock_count_threshold,
		.min_filter_enable       = t->min_filter_enable,
		.min_filter_active_depth = t->min_filter_active_depth,
		.delay_asymmetry_ns      = t->delay_asymmetry_ns,
	};
	return eth_litex_write_ptp_tuning(dev, &lt);
}

void fpga_hal_read_ptp_tuning(struct fpga_hal_ptp_tuning *t)
{
	const struct device *dev = fpga_hal_get_dev();

	if (!dev || !t) {
		if (t) {
			memset(t, 0, sizeof(*t));
		}
		return;
	}

	struct eth_litex_ptp_tuning lt;

	eth_litex_read_ptp_tuning(dev, &lt);
	t->kp_gain                 = lt.kp_gain;
	t->ki_gain                 = lt.ki_gain;
	t->gain_shift              = lt.gain_shift;
	t->gain_shift_locked       = lt.gain_shift_locked;
	t->ki_extra_shift          = lt.ki_extra_shift;
	t->filter_shift            = lt.filter_shift;
	t->warmup_samples          = lt.warmup_samples;
	t->lock_threshold_ns       = lt.lock_threshold_ns;
	t->unlock_threshold_ns     = lt.unlock_threshold_ns;
	t->lock_count_threshold    = lt.lock_count_threshold;
	t->min_filter_enable       = lt.min_filter_enable;
	t->min_filter_active_depth = lt.min_filter_active_depth;
	t->delay_asymmetry_ns      = lt.delay_asymmetry_ns;
}

void fpga_hal_read_ptp_monitor(struct fpga_hal_ptp_monitor *m)
{
	const struct device *dev = fpga_hal_get_dev();

	if (!dev || !m) {
		if (m) {
			memset(m, 0, sizeof(*m));
		}
		return;
	}

	struct eth_litex_ptp_monitor lm;

	eth_litex_read_ptp_monitor(dev, &lm);
	m->filtered_offset      = lm.filtered_offset;
	m->integral_sum         = lm.integral_sum;
	m->pi_proportional      = lm.pi_proportional;
	m->pi_sum_raw           = lm.pi_sum_raw;
	m->effective_gain_shift = lm.effective_gain_shift;
	m->lock_counter         = lm.lock_counter;
	m->sample_count         = lm.sample_count;
	m->first_lock_achieved  = lm.first_lock_achieved;
}

int fpga_hal_set_ptp_reset(bool held_in_reset)
{
	const struct device *dev = fpga_hal_get_dev();

	if (!dev) {
		return -ENODEV;
	}
	return eth_litex_set_ptp_reset(dev, held_in_reset);
}

int fpga_hal_set_resets(uint32_t domains, bool held)
{
	/* Map the backend-independent domain bits onto the reset CSR layout, then
	 * read-modify-write so untouched domains keep their state. */
	uint32_t csr_bits = 0;

	if (domains & FPGA_HAL_RESET_PTP) {
		csr_bits |= BIT(CSR_AES67_CSR_RESET_PTP_OFFSET);
	}
	if (domains & FPGA_HAL_RESET_TX) {
		csr_bits |= BIT(CSR_AES67_CSR_RESET_TX_OFFSET);
	}
	if (domains & FPGA_HAL_RESET_RX) {
		csr_bits |= BIT(CSR_AES67_CSR_RESET_RX_OFFSET);
	}
	if (domains & FPGA_HAL_RESET_ETH) {
		csr_bits |= BIT(CSR_AES67_CSR_RESET_ETH_OFFSET);
	}

	uint32_t reg = litex_csr_read(CSR_AES67_CSR_RESET_ADDR);

	if (held) {
		reg |= csr_bits;
	} else {
		reg &= ~csr_bits;
	}
	litex_csr_write(CSR_AES67_CSR_RESET_ADDR, reg);
	return 0;
}

void fpga_hal_read_metering(uint16_t *rx_signal, uint16_t *rx_clip,
			    uint16_t *tx_signal, uint16_t *tx_clip)
{
	/* Metering CSRs exist only when the aes67_bridge is built with metering
	 * (dropped via --no-metering-csr when the FPGA has metering disabled).
	 * Without them, report zeros so callers see a defined value. */
#ifdef CSR_AES67_CSR_RX_METER_SIGNAL_ADDR
	/* Read all four metering status registers */
	*rx_signal = (uint16_t)litex_csr_read(CSR_AES67_CSR_RX_METER_SIGNAL_ADDR);
	*rx_clip   = (uint16_t)litex_csr_read(CSR_AES67_CSR_RX_METER_CLIP_ADDR);
	*tx_signal = (uint16_t)litex_csr_read(CSR_AES67_CSR_TX_METER_SIGNAL_ADDR);
	*tx_clip   = (uint16_t)litex_csr_read(CSR_AES67_CSR_TX_METER_CLIP_ADDR);

	/* Toggle the clear bit so FPGA resets its sticky detectors */
	uint32_t cur = litex_csr_read(CSR_AES67_CSR_METER_CLEAR_ADDR);
	litex_csr_write(CSR_AES67_CSR_METER_CLEAR_ADDR, cur ^ 1);
#else
	*rx_signal = 0;
	*rx_clip   = 0;
	*tx_signal = 0;
	*tx_clip   = 0;
#endif
}
