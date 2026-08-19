/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * Wishbone-over-SPI transport (LiteX spibone) — see spibone.h for framing.
 * Wire protocol and constants mirror driver/aes67_eth/aes67_bus.c.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <string.h>

#include "spibone.h"

LOG_MODULE_REGISTER(spibone, CONFIG_SPIBONE_LOG_LEVEL);

#define SPIBONE_NODE DT_CHOSEN(zephyr_fpga_spi)

BUILD_ASSERT(DT_NODE_EXISTS(SPIBONE_NODE),
	     "chosen `zephyr,fpga-spi` must point to the spibone slave node");

#define CMD_WRITE 0x00
#define CMD_READ  0x01
/* Burst variants (repo-local spibone fork): auto-incrementing multi-word
 * transfers framed as [cmd][addr BE][count BE16][...]. */
#define CMD_BURST_WRITE 0x02
#define CMD_BURST_READ  0x03

/* Padding bytes clocked out to capture the device response; spibone answers
 * within a couple of bytes at any sane clock, so this is generous. */
#define RESPONSE_SLACK 24
/* Burst response slack: a few extra 0xff bytes to clock out the write ack /
 * the last read word's tail. */
#define BURST_WR_SLACK 8
#define BURST_RD_SLACK 16

/* Burst chunking so the transfers fit the scratch buffers.
 *
 * Legacy byte-per-word packing (stream-cfg RAMs, pre-packing eth_buf):
 *   write: 7 + 4*256 + 8  = 1039 B per transfer
 *   read:  7 + 7*240 + 16 = 1703 B per transfer
 * Packed 4-bytes-per-word (eth_buf on packed gateware), chunk in words:
 *   write: 7 + 4*384 + 8  = 1551 B per transfer (full frame = 380 words)
 *   read:  7 + 7*288 + 16 = 2039 B per transfer (full frame = 2 transfers) */
#define BURST_WR_CHUNK 256
#define BURST_RD_CHUNK 240
#define PACKED_WR_CHUNK_WORDS 384
#define PACKED_RD_CHUNK_WORDS 288
#define BURST_BUF      2048

struct spibone_data {
	struct k_mutex lock;
};

struct spibone_config {
	struct spi_dt_spec bus;
};

static struct spibone_data spibone_data_inst;

static const struct spibone_config spibone_config_inst = {
	/* spibone gateware speaks SPI mode 0 (CPOL=0, CPHA=0), MSB first. */
	.bus = SPI_DT_SPEC_GET(SPIBONE_NODE,
			       SPI_OP_MODE_MASTER | SPI_WORD_SET(8) |
			       SPI_TRANSFER_MSB, 0),
};

/* Contiguous scratch buffers for burst transfers. Some SPI controllers
 * (ESP32 included) don't reliably emit multi-segment spi_buf_sets as one
 * continuous CS-low frame, so everything goes through one buffer pair.
 * Protected by the bus lock. */
static uint8_t burst_tx[BURST_BUF];
static uint8_t burst_rx[BURST_BUF];

static int spibone_init(const struct device *dev)
{
	struct spibone_data *data = dev->data;
	const struct spibone_config *cfg = dev->config;

	k_mutex_init(&data->lock);

	if (!spi_is_ready_dt(&cfg->bus)) {
		LOG_ERR("SPI bus %s not ready", cfg->bus.bus->name);
		return -ENODEV;
	}

	LOG_INF("spibone bridge on %s @ %u Hz",
		cfg->bus.bus->name, cfg->bus.config.frequency);
	return 0;
}

DEVICE_DEFINE(spibone, "spibone0", spibone_init, NULL,
	      &spibone_data_inst, &spibone_config_inst,
	      POST_KERNEL, CONFIG_SPIBONE_INIT_PRIORITY, NULL);

const struct device *spibone_get_dev(void)
{
	return DEVICE_GET(spibone);
}

#ifdef CONFIG_SPIBONE_STATS
/* All fields are written with the bus mutex held (the wait accounting runs
 * after the lock is acquired), so no extra synchronization is needed. */
static struct spibone_stats stats;
static uint32_t stats_hold_start;
static uint32_t stats_lock_depth;
#endif

void spibone_bus_lock(void)
{
#ifdef CONFIG_SPIBONE_STATS
	if (k_mutex_lock(&spibone_data_inst.lock, K_NO_WAIT) != 0) {
		uint32_t t0 = k_cycle_get_32();
		uint32_t wait;

		k_mutex_lock(&spibone_data_inst.lock, K_FOREVER);
		wait = k_cycle_get_32() - t0;

		stats.lock_wait_cyc += wait;
		stats.lock_contended++;
		if (wait > stats.lock_wait_max_cyc) {
			stats.lock_wait_max_cyc = wait;
		}
	}
	stats.lock_count++;
	if (stats_lock_depth++ == 0) {
		stats_hold_start = k_cycle_get_32();
	}
#else
	k_mutex_lock(&spibone_data_inst.lock, K_FOREVER);
#endif
}

void spibone_bus_unlock(void)
{
#ifdef CONFIG_SPIBONE_STATS
	if (--stats_lock_depth == 0) {
		stats.lock_hold_cyc += k_cycle_get_32() - stats_hold_start;
	}
#endif
	k_mutex_unlock(&spibone_data_inst.lock);
}

void spibone_stats_get(struct spibone_stats *out)
{
#ifdef CONFIG_SPIBONE_STATS
	k_mutex_lock(&spibone_data_inst.lock, K_FOREVER);
	*out = stats;
	stats.lock_wait_max_cyc = 0;
	k_mutex_unlock(&spibone_data_inst.lock);
#else
	memset(out, 0, sizeof(*out));
#endif
}

/* One full-duplex transfer, CS asserted across the whole buffer. */
static int spibone_xfer(const uint8_t *tx, uint8_t *rx, size_t len)
{
	const struct spibone_config *cfg = spibone_get_dev()->config;

	const struct spi_buf tx_buf = { .buf = (void *)tx, .len = len };
	const struct spi_buf rx_buf = { .buf = rx, .len = len };
	const struct spi_buf_set tx_set = { .buffers = &tx_buf, .count = 1 };
	const struct spi_buf_set rx_set = { .buffers = &rx_buf, .count = 1 };

#ifdef CONFIG_SPIBONE_STATS
	/* Callers hold the bus mutex (see spibone.h contract). */
	uint32_t t0 = k_cycle_get_32();
	int ret = spi_transceive_dt(&cfg->bus, &tx_set, &rx_set);

	stats.xfer_cyc += k_cycle_get_32() - t0;
	stats.xfer_bytes += len;
	stats.xfer_count++;
	if (ret < 0) {
		stats.xfer_errors++;
	}
	return ret;
#else
	return spi_transceive_dt(&cfg->bus, &tx_set, &rx_set);
#endif
}

int spibone_read_locked(uint32_t addr, uint32_t *val)
{
	uint8_t tx[5 + RESPONSE_SLACK];
	uint8_t rx[5 + RESPONSE_SLACK];
	int ret;

	tx[0] = CMD_READ;
	sys_put_be32(addr, &tx[1]);
	memset(&tx[5], 0xff, RESPONSE_SLACK);

	ret = spibone_xfer(tx, rx, sizeof(tx));
	if (ret < 0) {
		return ret;
	}

	/* Scan past the address echo for the sync byte, skipping the 0xff the
	 * device drives while the read is in flight. */
	for (int i = 5; i + 4 < (int)sizeof(rx); i++) {
		if (rx[i] == CMD_READ) {
			*val = sys_get_be32(&rx[i + 1]);
			return 0;
		}
		if (rx[i] != 0xff) {
			return -EIO;
		}
	}
	return -ETIMEDOUT;
}

int spibone_write_locked(uint32_t addr, uint32_t val)
{
	uint8_t tx[9 + RESPONSE_SLACK];
	uint8_t rx[9 + RESPONSE_SLACK];
	int ret;

	tx[0] = CMD_WRITE;
	sys_put_be32(addr, &tx[1]);
	sys_put_be32(val, &tx[5]);
	memset(&tx[9], 0xff, RESPONSE_SLACK);

	ret = spibone_xfer(tx, rx, sizeof(tx));
	if (ret < 0) {
		return ret;
	}

	for (int i = 9; i < (int)sizeof(rx); i++) {
		if (rx[i] == CMD_WRITE) {
			return 0;
		}
		if (rx[i] != 0xff) {
			return -EIO;
		}
	}
	return -ETIMEDOUT;
}

int spibone_read(uint32_t addr, uint32_t *val)
{
	int ret;

	spibone_bus_lock();
	ret = spibone_read_locked(addr, val);
	spibone_bus_unlock();
	return ret;
}

int spibone_write(uint32_t addr, uint32_t val)
{
	int ret;

	spibone_bus_lock();
	ret = spibone_write_locked(addr, val);
	spibone_bus_unlock();
	return ret;
}

int spibone_write_burst_locked(uint32_t addr, const uint8_t *bytes, size_t n)
{
	if (!IS_ENABLED(CONFIG_SPIBONE_BURST)) {
		/* Fallback for gateware without burst support: one word per byte. */
		for (size_t i = 0; i < n; i++) {
			int ret = spibone_write_locked(addr + 4 * i, bytes[i]);

			if (ret < 0) {
				return ret;
			}
		}
		return 0;
	}

	while (n) {
		size_t chunk = MIN(n, (size_t)BURST_WR_CHUNK);
		size_t len, i;
		int ret;

		burst_tx[0] = CMD_BURST_WRITE;
		sys_put_be32(addr, &burst_tx[1]);
		sys_put_be16((uint16_t)chunk, &burst_tx[5]);
		for (i = 0; i < chunk; i++) {
			burst_tx[7 + 4 * i + 0] = 0;
			burst_tx[7 + 4 * i + 1] = 0;
			burst_tx[7 + 4 * i + 2] = 0;
			burst_tx[7 + 4 * i + 3] = bytes[i];
		}
		len = 7 + 4 * chunk;
		memset(&burst_tx[len], 0xff, BURST_WR_SLACK);
		len += BURST_WR_SLACK;

		ret = spibone_xfer(burst_tx, burst_rx, len);
		if (ret < 0) {
			return ret;
		}

		/* Confirm the device clocked out its 0x00 completion ack. */
		for (i = 7 + 4 * chunk; i < len; i++) {
			if (burst_rx[i] == CMD_WRITE) {   /* 0x00 ack */
				break;
			}
			if (burst_rx[i] != 0xff) {
				return -EIO;
			}
		}
		if (i == len) {
			return -ETIMEDOUT;
		}

		addr  += 4 * chunk;
		bytes += chunk;
		n     -= chunk;
	}
	return 0;
}

int spibone_read_burst_locked(uint32_t addr, uint8_t *bytes, size_t n)
{
	if (!IS_ENABLED(CONFIG_SPIBONE_BURST)) {
		for (size_t i = 0; i < n; i++) {
			uint32_t word;
			int ret = spibone_read_locked(addr + 4 * i, &word);

			if (ret < 0) {
				return ret;
			}
			bytes[i] = (uint8_t)word;
		}
		return 0;
	}

	while (n) {
		size_t chunk = MIN(n, (size_t)BURST_RD_CHUNK);
		size_t len, i, w;
		int ret;

		burst_tx[0] = CMD_BURST_READ;
		sys_put_be32(addr, &burst_tx[1]);
		sys_put_be16((uint16_t)chunk, &burst_tx[5]);
		/* 7-byte header echo + up to 7 bytes/word ([pad][sync][4 data]) +
		 * slack; clock 0xff across the whole response window. */
		len = 7 + 7 * chunk + BURST_RD_SLACK;
		memset(&burst_tx[7], 0xff, len - 7);

		ret = spibone_xfer(burst_tx, burst_rx, len);
		if (ret < 0) {
			return ret;
		}

		i = 7;
		for (w = 0; w < chunk; w++) {
			/* Skip the 0xff the device drives during read latency. */
			while (i < len && burst_rx[i] == 0xff) {
				i++;
			}
			if (i + 5 > len || burst_rx[i] != CMD_READ) {
				return -ETIMEDOUT;
			}
			bytes[w] = burst_rx[i + 4];   /* low byte of the BE32 word */
			i += 5;
		}

		addr  += 4 * chunk;
		bytes += chunk;
		n     -= chunk;
	}
	return 0;
}

int spibone_write_burst_packed_locked(uint32_t addr, const uint8_t *bytes,
				      size_t n)
{
	if (!IS_ENABLED(CONFIG_SPIBONE_BURST)) {
		while (n) {
			size_t take = MIN(n, (size_t)4);
			uint32_t word = 0;
			int ret;

			for (size_t k = 0; k < take; k++) {
				word |= (uint32_t)bytes[k] << (8 * k);
			}
			ret = spibone_write_locked(addr, word);
			if (ret < 0) {
				return ret;
			}
			addr  += 4;
			bytes += take;
			n     -= take;
		}
		return 0;
	}

	while (n) {
		size_t words = MIN(DIV_ROUND_UP(n, 4),
				   (size_t)PACKED_WR_CHUNK_WORDS);
		size_t take_total = MIN(n, words * 4);
		const uint8_t *src = bytes;
		size_t rem = take_total;
		size_t len, w, i;
		int ret;

		burst_tx[0] = CMD_BURST_WRITE;
		sys_put_be32(addr, &burst_tx[1]);
		sys_put_be16((uint16_t)words, &burst_tx[5]);
		for (w = 0; w < words; w++) {
			size_t take = MIN(rem, (size_t)4);
			uint32_t word = 0;

			/* Little-endian lanes: frame byte 4w+k in bits
			 * [8k+7:8k]; the final partial word is zero-padded
			 * (tx_len bounds the frame, the pad is never sent). */
			for (size_t k = 0; k < take; k++) {
				word |= (uint32_t)src[k] << (8 * k);
			}
			sys_put_be32(word, &burst_tx[7 + 4 * w]);
			src += take;
			rem -= take;
		}
		len = 7 + 4 * words;
		memset(&burst_tx[len], 0xff, BURST_WR_SLACK);
		len += BURST_WR_SLACK;

		ret = spibone_xfer(burst_tx, burst_rx, len);
		if (ret < 0) {
			return ret;
		}

		/* Confirm the device clocked out its 0x00 completion ack. */
		for (i = 7 + 4 * words; i < len; i++) {
			if (burst_rx[i] == CMD_WRITE) {   /* 0x00 ack */
				break;
			}
			if (burst_rx[i] != 0xff) {
				return -EIO;
			}
		}
		if (i == len) {
			return -ETIMEDOUT;
		}

		addr  += 4 * words;
		bytes += take_total;
		n     -= take_total;
	}
	return 0;
}

int spibone_read_burst_packed_locked(uint32_t addr, uint8_t *bytes, size_t n)
{
	if (!IS_ENABLED(CONFIG_SPIBONE_BURST)) {
		while (n) {
			size_t take = MIN(n, (size_t)4);
			uint32_t word;
			int ret = spibone_read_locked(addr, &word);

			if (ret < 0) {
				return ret;
			}
			for (size_t k = 0; k < take; k++) {
				bytes[k] = (uint8_t)(word >> (8 * k));
			}
			addr  += 4;
			bytes += take;
			n     -= take;
		}
		return 0;
	}

	while (n) {
		size_t words = MIN(DIV_ROUND_UP(n, 4),
				   (size_t)PACKED_RD_CHUNK_WORDS);
		size_t take_total = MIN(n, words * 4);
		size_t rem = take_total;
		size_t len, i, w;
		int ret;

		burst_tx[0] = CMD_BURST_READ;
		sys_put_be32(addr, &burst_tx[1]);
		sys_put_be16((uint16_t)words, &burst_tx[5]);
		len = 7 + 7 * words + BURST_RD_SLACK;
		memset(&burst_tx[7], 0xff, len - 7);

		ret = spibone_xfer(burst_tx, burst_rx, len);
		if (ret < 0) {
			return ret;
		}

		i = 7;
		for (w = 0; w < words; w++) {
			size_t take = MIN(rem, (size_t)4);
			uint32_t word;

			while (i < len && burst_rx[i] == 0xff) {
				i++;
			}
			if (i + 5 > len || burst_rx[i] != CMD_READ) {
				return -ETIMEDOUT;
			}
			/* BE32 on the wire, little-endian lanes in the word;
			 * never write past the caller's n bytes. */
			word = sys_get_be32(&burst_rx[i + 1]);
			for (size_t k = 0; k < take; k++) {
				bytes[k] = (uint8_t)(word >> (8 * k));
			}
			bytes += take;
			rem   -= take;
			i     += 5;
		}

		addr += 4 * words;
		n    -= take_total;
	}
	return 0;
}

int spibone_write_burst(uint32_t addr, const uint8_t *bytes, size_t n)
{
	int ret;

	spibone_bus_lock();
	ret = spibone_write_burst_locked(addr, bytes, n);
	spibone_bus_unlock();
	return ret;
}

int spibone_read_burst(uint32_t addr, uint8_t *bytes, size_t n)
{
	int ret;

	spibone_bus_lock();
	ret = spibone_read_burst_locked(addr, bytes, n);
	spibone_bus_unlock();
	return ret;
}

#if defined(CONFIG_SPIBONE_STATS) && defined(CONFIG_SHELL)

#include <zephyr/shell/shell.h>

static int cmd_spibus(const struct shell *sh, size_t argc, char **argv)
{
	static struct spibone_stats prev;
	static int64_t prev_ms;
	static bool have_prev;

	struct spibone_stats now;
	int64_t now_ms = k_uptime_get();
	uint32_t cps = sys_clock_hw_cycles_per_sec();
	uint32_t cpus = cps / 1000000U;

	spibone_stats_get(&now);

	int64_t elapsed_ms = now_ms - prev_ms;
	uint64_t d_hold  = now.lock_hold_cyc - prev.lock_hold_cyc;
	uint64_t d_wait  = now.lock_wait_cyc - prev.lock_wait_cyc;
	uint64_t d_wire  = now.xfer_cyc      - prev.xfer_cyc;
	uint64_t d_bytes = now.xfer_bytes    - prev.xfer_bytes;
	uint32_t d_xfers = now.xfer_count    - prev.xfer_count;
	uint32_t d_locks = now.lock_count    - prev.lock_count;
	uint32_t d_cont  = now.lock_contended - prev.lock_contended;
	uint32_t d_errs  = now.xfer_errors   - prev.xfer_errors;

	if (!have_prev) {
		shell_print(sh, "(window = since boot; call again for a load window)");
		have_prev = true;
	}
	prev = now;
	prev_ms = now_ms;

	if (elapsed_ms <= 0) {
		return 0;
	}

	uint64_t wall_cyc = (uint64_t)elapsed_ms * cps / 1000U;
	uint32_t held_pm = (uint32_t)(d_hold * 1000U / wall_cyc);
	uint32_t wire_pm = (uint32_t)(d_wire * 1000U / wall_cyc);
	uint32_t cont_pm = d_locks ? (uint32_t)((uint64_t)d_cont * 1000U / d_locks) : 0;
	uint32_t wait_avg_us = d_cont ? (uint32_t)(d_wait / d_cont / cpus) : 0;

	shell_print(sh, "window:    %u.%02u s",
		    (uint32_t)(elapsed_ms / 1000),
		    (uint32_t)((elapsed_ms % 1000) / 10));
	shell_print(sh, "bus held:  %u.%u %%   wire busy: %u.%u %%",
		    held_pm / 10, held_pm % 10, wire_pm / 10, wire_pm % 10);
	shell_print(sh, "transfers: %u/s   %u B/s   errors: %u",
		    (uint32_t)((uint64_t)d_xfers * 1000U / elapsed_ms),
		    (uint32_t)(d_bytes * 1000U / elapsed_ms), d_errs);
	shell_print(sh, "locks:     %u/s   contended: %u (%u.%u %%)",
		    (uint32_t)((uint64_t)d_locks * 1000U / elapsed_ms),
		    d_cont, cont_pm / 10, cont_pm % 10);
	shell_print(sh, "wait:      avg %u us   max %u us",
		    wait_avg_us, now.lock_wait_max_cyc / cpus);
	return 0;
}

SHELL_CMD_REGISTER(spibus, NULL,
		   "spibone bus utilization since the previous call",
		   cmd_spibus);

#endif /* CONFIG_SPIBONE_STATS && CONFIG_SHELL */
