/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * FPGA HAL backend — SPI implementation.
 *
 * Uses the standalone fpga_spi driver (drivers/fpga_spi/) to talk to the
 * on-FPGA spictrl block. Intended for boards that host only the control
 * plane (Zephyr) on an external MCU while the FPGA owns the data plane.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include "fpga_hal.h"
#include "../fpga_spi/fpga_spi.h"

LOG_MODULE_REGISTER(fpga_hal, LOG_LEVEL_INF);

/* Shadow of the flag register (0x50). The FPGA commits the full byte on
 * every write, so we track bits locally to support set/clear semantics.
 * PPB_START is auto-cleared by the FPGA once the measurement completes;
 * we clear it from the shadow lazily when the host next writes the byte.
 */
static struct {
	struct k_spinlock lock;
	uint8_t bits;
} g_flags;

static int flags_write_locked(uint8_t new_bits)
{
	const struct device *dev = fpga_spi_get_dev();

	if (!dev) {
		return -ENODEV;
	}
	return fpga_spi_write(dev, FPGA_SPI_REG_FLAGS, &new_bits, 1);
}

static int flags_update(uint8_t set_mask, uint8_t clear_mask)
{
	k_spinlock_key_t key = k_spin_lock(&g_flags.lock);
	uint8_t bits = g_flags.bits;

	bits &= ~clear_mask;
	bits |= set_mask;
	g_flags.bits = bits;
	k_spin_unlock(&g_flags.lock, key);

	return flags_write_locked(bits);
}

/* ---- Device accessor ---- */

const struct device *fpga_hal_get_dev(void)
{
	return fpga_spi_get_dev();
}

/* ---- FPGA ready / recovery ---- */

bool fpga_hal_is_ready(void)
{
	const struct device *dev = fpga_spi_get_dev();

	return dev != NULL && device_is_ready(dev);
}

int fpga_hal_wait_ready(uint32_t timeout_ms)
{
	/* The FPGA runs independently. If the SPI bus is up we assume the
	 * FPGA is reachable; there is no ready line on this transport. */
	ARG_UNUSED(timeout_ms);
	return fpga_hal_is_ready() ? 0 : -ETIMEDOUT;
}

void fpga_hal_register_recover_cb(fpga_hal_recover_cb_t cb, void *user_data)
{
	ARG_UNUSED(cb);
	ARG_UNUSED(user_data);
	/* No recovery signalling on the SPI transport. */
}

/* ---- Configuration writes ---- */

int fpga_hal_write_mac(const uint8_t mac[6])
{
	const struct device *dev = fpga_spi_get_dev();

	if (!dev) {
		return -ENODEV;
	}
	/* Wire order: byte 0 = MAC[47:40] (MSB first) — matches input layout. */
	return fpga_spi_write(dev, FPGA_SPI_REG_MAC, mac, 6);
}

int fpga_hal_write_ip(const struct in_addr *ip)
{
	const struct device *dev = fpga_spi_get_dev();

	if (!dev) {
		return -ENODEV;
	}
	/* s_addr is stored in network byte order, same as the FPGA expects
	 * (byte 0 = IP[31:24]). */
	const uint8_t *b = (const uint8_t *)&ip->s_addr;
	return fpga_spi_write(dev, FPGA_SPI_REG_IP, b, 4);
}

/* Cache for PTP-config / GM-quality so we can write the full 7-byte block
 * atomically when only part of it is updated by the caller. */
K_MUTEX_DEFINE(g_ptp_cfg_lock);

static struct {
	uint8_t time_source;
	int8_t  log_sync;
	int8_t  log_announce;
	uint8_t prio1;
	uint8_t prio2;
	uint8_t clock_class;
	uint8_t clock_accuracy;
} g_ptp_cfg = {
	/* Sensible IEEE 1588 defaults until the first explicit write. */
	.clock_class    = 248,
	.clock_accuracy = 0xFE,
};

static int ptp_cfg_flush(void)
{
	const struct device *dev = fpga_spi_get_dev();

	if (!dev) {
		return -ENODEV;
	}
	uint8_t buf[7] = {
		g_ptp_cfg.time_source,
		(uint8_t)g_ptp_cfg.log_sync,
		(uint8_t)g_ptp_cfg.log_announce,
		g_ptp_cfg.prio1,
		g_ptp_cfg.prio2,
		g_ptp_cfg.clock_class,
		g_ptp_cfg.clock_accuracy,
	};
	return fpga_spi_write(dev, FPGA_SPI_REG_PTP_CONFIG, buf, sizeof(buf));
}

int fpga_hal_write_ptp_config(uint8_t time_source,
			      int8_t log_msg_interval,
			      int8_t log_announce_interval)
{
	k_mutex_lock(&g_ptp_cfg_lock, K_FOREVER);
	g_ptp_cfg.time_source  = time_source;
	g_ptp_cfg.log_sync     = log_msg_interval;
	g_ptp_cfg.log_announce = log_announce_interval;
	int ret = ptp_cfg_flush();
	k_mutex_unlock(&g_ptp_cfg_lock);
	return ret;
}

int fpga_hal_write_ptp_gm_quality(uint8_t priority1, uint8_t priority2,
				  uint8_t clock_class, uint8_t clock_accuracy)
{
	k_mutex_lock(&g_ptp_cfg_lock, K_FOREVER);
	g_ptp_cfg.prio1          = priority1;
	g_ptp_cfg.prio2          = priority2;
	g_ptp_cfg.clock_class    = clock_class;
	g_ptp_cfg.clock_accuracy = clock_accuracy;
	int ret = ptp_cfg_flush();
	k_mutex_unlock(&g_ptp_cfg_lock);
	return ret;
}

bool fpga_hal_read_ptp_leader_id(uint8_t leader_clock_id[8])
{
	const struct device *dev = fpga_spi_get_dev();
	uint8_t le[8];

	if (!dev || fpga_spi_read(dev, FPGA_SPI_REG_GM_ID, le, 8) < 0) {
		memset(leader_clock_id, 0, 8);
		return false;
	}
	/* Wire order is little-endian (byte 0 = bits 7..0 of gmid). The public
	 * API returns the clock identity in big-endian / network order, so
	 * reverse here. */
	for (int i = 0; i < 8; i++) {
		leader_clock_id[i] = le[7 - i];
	}

	for (int i = 0; i < 8; i++) {
		if (leader_clock_id[i] != 0) {
			return true;
		}
	}
	return false;
}

int fpga_hal_write_tx_stream_config(uint8_t stream_id,
				    const struct in_addr *dst_ip,
				    uint8_t channel_count,
				    uint8_t samples_per_pkt,
				    const uint8_t *ch_ids,
				    uint8_t num_ch_ids,
				    uint32_t ssrc)
{
	const struct device *dev = fpga_spi_get_dev();
	uint8_t buf[20];

	if (!dev) {
		return -ENODEV;
	}
	if (num_ch_ids > 8) {
		return -EINVAL;
	}

	memset(buf, 0, sizeof(buf));
	buf[0] = stream_id & 0x07;
	/* Bytes 1..4: destination IP in network byte order (big-endian). */
	memcpy(&buf[1], &dst_ip->s_addr, 4);
	buf[5] = channel_count;
	buf[6] = samples_per_pkt;
	if (ch_ids && num_ch_ids) {
		memcpy(&buf[7], ch_ids, num_ch_ids);
	}
	/* buf[15] reserved */
	/* Bytes 16..19: SSRC big-endian */
	buf[16] = (uint8_t)(ssrc >> 24);
	buf[17] = (uint8_t)(ssrc >> 16);
	buf[18] = (uint8_t)(ssrc >> 8);
	buf[19] = (uint8_t)(ssrc);

	return fpga_spi_write(dev, FPGA_SPI_REG_TX_STREAM, buf, sizeof(buf));
}

int fpga_hal_write_rx_stream_config(uint8_t stream_id,
				    const struct in_addr *dst_ip,
				    uint16_t dst_port,
				    const uint8_t *ch_map,
				    uint8_t channel_count,
				    uint8_t output_delay,
				    uint8_t samples_per_channel)
{
	const struct device *dev = fpga_spi_get_dev();
	uint8_t buf[18];

	if (!dev) {
		return -ENODEV;
	}

	memset(buf, 0, sizeof(buf));
	buf[0] = stream_id & 0x07;
	/* Bytes 1..4: destination IP big-endian (network order). */
	memcpy(&buf[1], &dst_ip->s_addr, 4);
	/* Bytes 5..6: destination UDP port big-endian. */
	buf[5] = (uint8_t)(dst_port >> 8);
	buf[6] = (uint8_t)(dst_port);
	if (ch_map && channel_count) {
		uint8_t n = channel_count > 8 ? 8 : channel_count;
		memcpy(&buf[7], ch_map, n);
	}
	buf[15] = channel_count;
	buf[16] = output_delay;
	buf[17] = samples_per_channel;

	return fpga_spi_write(dev, FPGA_SPI_REG_RX_STREAM, buf, sizeof(buf));
}

/* ---- Control register ---- */

static uint8_t hal_to_flag_bits(uint32_t bits)
{
	uint8_t out = 0;

	if (bits & FPGA_HAL_CTRL_PPB_START) {
		out |= FPGA_SPI_FLAG_PPB_START;
	}
	if (bits & FPGA_HAL_CTRL_RESET_WALLCLOCK) {
		out |= FPGA_SPI_FLAG_RESET_WALLCLOCK;
	}
	if (bits & FPGA_HAL_CTRL_RESET_PTP) {
		out |= FPGA_SPI_FLAG_RESET_PTP;
	}
	if (bits & FPGA_HAL_CTRL_RESET_ETHERNET) {
		out |= FPGA_SPI_FLAG_RESET_ETHERNET;
	}
	return out;
}

int fpga_hal_ctrl_set_bits(uint32_t bits)
{
	return flags_update(hal_to_flag_bits(bits), 0);
}

int fpga_hal_ctrl_clear_bits(uint32_t bits)
{
	return flags_update(0, hal_to_flag_bits(bits));
}

int fpga_hal_set_adda_nrst(bool released)
{
	if (released) {
		return flags_update(FPGA_SPI_FLAG_ADDA_NRST, 0);
	}
	return flags_update(0, FPGA_SPI_FLAG_ADDA_NRST);
}

/* ---- Status reads ---- */

static int32_t le_to_s32(const uint8_t b[4])
{
	return (int32_t)((uint32_t)b[0] |
			 ((uint32_t)b[1] << 8) |
			 ((uint32_t)b[2] << 16) |
			 ((uint32_t)b[3] << 24));
}

uint32_t fpga_hal_read_status(void)
{
	const struct device *dev = fpga_spi_get_dev();
	uint32_t hal = 0;
	uint8_t v;

	if (!dev) {
		return 0;
	}

	/* Clocking status (0x50 read). */
	if (fpga_spi_read(dev, FPGA_SPI_REG_STATUS_CLK, &v, 1) == 0) {
		if (v & FPGA_SPI_CLK_PPB_VALID) {
			hal |= FPGA_HAL_CLK_PPB_VALID;
		}
		if (v & FPGA_SPI_CLK_WC_LOCKED) {
			hal |= FPGA_HAL_CLK_WC_LOCKED;
		}
		if (v & FPGA_SPI_CLK_WC_CONFIGURED) {
			hal |= FPGA_HAL_CLK_WC_CONFIGURED;
		}
		if (v & FPGA_SPI_CLK_PTP_LEADER) {
			hal |= FPGA_HAL_PTP_IS_LEADER;
		}
		if (v & FPGA_SPI_CLK_PTP_FOLLOWER) {
			hal |= FPGA_HAL_PTP_IS_FOLLOWER;
		}
		/* SPI register map has no PHASEJUMP / LEADER_LOST bits. */
	}

	/* Ethernet status (0x51 read). */
	if (fpga_spi_read(dev, FPGA_SPI_REG_STATUS_ETH, &v, 1) == 0) {
		if (v & FPGA_SPI_ETH_LINK_UP) {
			hal |= FPGA_HAL_ETH_LINK_UP;
		}
		uint32_t speed = (v & FPGA_SPI_ETH_SPEED_MASK) >> FPGA_SPI_ETH_SPEED_SHIFT;
		hal |= (speed << FPGA_HAL_ETH_SPEED_SHIFT);
	}

	return hal;
}

int32_t fpga_hal_read_path_delay(void)
{
	const struct device *dev = fpga_spi_get_dev();
	uint8_t buf[4];

	if (!dev || fpga_spi_read(dev, FPGA_SPI_REG_PATH_DELAY, buf, 4) < 0) {
		return 0;
	}
	return le_to_s32(buf);
}

int32_t fpga_hal_read_ptp_offset(void)
{
	const struct device *dev = fpga_spi_get_dev();
	uint8_t buf[4];

	if (!dev || fpga_spi_read(dev, FPGA_SPI_REG_PTP_OFFSET, buf, 4) < 0) {
		return 0;
	}
	return le_to_s32(buf);
}

bool fpga_hal_read_ppb_counts(uint32_t *wc_count, uint32_t *pll_count)
{
	const struct device *dev = fpga_spi_get_dev();
	uint8_t buf[8];
	uint8_t status;

	if (!dev || fpga_spi_read(dev, FPGA_SPI_REG_PPB_COUNTERS, buf, 8) < 0) {
		*wc_count = 0;
		*pll_count = 0;
		return false;
	}
	/* Bytes 0..3 PLL, bytes 4..7 wallclock, each little-endian.
	 * Counters are 22-bit on the FPGA, mask to be safe. */
	*pll_count = ((uint32_t)buf[0] |
		      ((uint32_t)buf[1] << 8) |
		      ((uint32_t)buf[2] << 16) |
		      ((uint32_t)buf[3] << 24)) & 0x3FFFFFu;
	*wc_count  = ((uint32_t)buf[4] |
		      ((uint32_t)buf[5] << 8) |
		      ((uint32_t)buf[6] << 16) |
		      ((uint32_t)buf[7] << 24)) & 0x3FFFFFu;

	if (fpga_spi_read(dev, FPGA_SPI_REG_STATUS_CLK, &status, 1) < 0) {
		return false;
	}
	return (status & FPGA_SPI_CLK_PPB_VALID) != 0;
}

/* ---- Audio metering ----
 *
 * Register 0x30 returns `metering_bytes` bytes. With N_RX/N_TX channels
 * each contributing a 1-bit signal and a 1-bit clip flag, the FPGA shadow
 * is concatenated as `rx_sig & rx_clip & tx_sig & tx_clip` (MSB-first in
 * the shadow, i.e. rx_sig is at the top). Bytes are shifted out starting
 * from the LSB, so the wire order is tx_clip, tx_sig, rx_clip, rx_sig.
 *
 * Byte count is derived from the FPGA info register (0x00): byte 4 =
 * TXCHANNELS, byte 5 = RXCHANNELS. We cache that on the first call so
 * subsequent reads stay hot.
 *
 * The FPGA asserts its internal `meter_clear` pulse as soon as the last
 * byte of the 0x30 read transaction has been clocked out — no flag write
 * needed from the host. */

static struct {
	uint8_t tx_channels;
	uint8_t rx_channels;
	bool    known;
} g_meter_cfg;

static int meter_info_fetch(const struct device *dev)
{
	uint8_t info[8];
	int ret = fpga_spi_read(dev, FPGA_SPI_REG_INFO, info, sizeof(info));

	if (ret < 0) {
		return ret;
	}
	g_meter_cfg.tx_channels = info[4];
	g_meter_cfg.rx_channels = info[5];
	g_meter_cfg.known = true;
	return 0;
}

static int meter_ch_to_bytes(uint8_t ch)
{
	/* Ceil(ch / 8), capped at 2 bytes since the HAL API returns uint16_t. */
	int b = (ch + 7) / 8;

	if (b > 2) {
		b = 2;
	}
	return b;
}

static uint16_t pack_bytes_le(const uint8_t *buf, int nbytes)
{
	uint16_t v = 0;

	for (int i = 0; i < nbytes; i++) {
		v |= (uint16_t)buf[i] << (8 * i);
	}
	return v;
}

void fpga_hal_read_metering(uint16_t *rx_signal, uint16_t *rx_clip,
			    uint16_t *tx_signal, uint16_t *tx_clip)
{
	if (rx_signal) *rx_signal = 0;
	if (rx_clip)   *rx_clip   = 0;
	if (tx_signal) *tx_signal = 0;
	if (tx_clip)   *tx_clip   = 0;

	const struct device *dev = fpga_spi_get_dev();

	if (!dev) {
		return;
	}
	if (!g_meter_cfg.known && meter_info_fetch(dev) < 0) {
		return;
	}

	int tx_bytes = meter_ch_to_bytes(g_meter_cfg.tx_channels);
	int rx_bytes = meter_ch_to_bytes(g_meter_cfg.rx_channels);
	int total    = 2 * tx_bytes + 2 * rx_bytes;

	if (total == 0 || total > 8) {
		return;
	}

	uint8_t buf[8];

	if (fpga_spi_read(dev, FPGA_SPI_REG_METERING, buf, total) < 0) {
		return;
	}

	/* Wire order (low-to-high byte index): tx_clip, tx_sig, rx_clip, rx_sig. */
	int off = 0;
	uint16_t vtc = pack_bytes_le(&buf[off], tx_bytes); off += tx_bytes;
	uint16_t vts = pack_bytes_le(&buf[off], tx_bytes); off += tx_bytes;
	uint16_t vrc = pack_bytes_le(&buf[off], rx_bytes); off += rx_bytes;
	uint16_t vrs = pack_bytes_le(&buf[off], rx_bytes);

	if (tx_clip)   *tx_clip   = vtc;
	if (tx_signal) *tx_signal = vts;
	if (rx_clip)   *rx_clip   = vrc;
	if (rx_signal) *rx_signal = vrs;
}

/* ---- PTP servo tuning + monitoring (not implemented on SPI backend) ---- */

int fpga_hal_write_ptp_tuning(const struct fpga_hal_ptp_tuning *t)
{
	ARG_UNUSED(t);
	return -ENOTSUP;
}

void fpga_hal_read_ptp_tuning(struct fpga_hal_ptp_tuning *t)
{
	if (t) {
		memset(t, 0, sizeof(*t));
	}
}

void fpga_hal_read_ptp_monitor(struct fpga_hal_ptp_monitor *m)
{
	if (m) {
		memset(m, 0, sizeof(*m));
	}
}

int fpga_hal_set_ptp_reset(bool held_in_reset)
{
	ARG_UNUSED(held_in_reset);
	return -ENOTSUP;
}
