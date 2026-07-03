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

/* Burst chunking so a full 1518-byte frame fits the scratch buffers:
 *   write: 7 + 4*256 + 8  = 1039 B per transfer
 *   read:  7 + 7*240 + 16 = 1703 B per transfer */
#define BURST_WR_CHUNK 256
#define BURST_RD_CHUNK 240
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

void spibone_bus_lock(void)
{
	k_mutex_lock(&spibone_data_inst.lock, K_FOREVER);
}

void spibone_bus_unlock(void)
{
	k_mutex_unlock(&spibone_data_inst.lock);
}

/* One full-duplex transfer, CS asserted across the whole buffer. */
static int spibone_xfer(const uint8_t *tx, uint8_t *rx, size_t len)
{
	const struct spibone_config *cfg = spibone_get_dev()->config;

	const struct spi_buf tx_buf = { .buf = (void *)tx, .len = len };
	const struct spi_buf rx_buf = { .buf = rx, .len = len };
	const struct spi_buf_set tx_set = { .buffers = &tx_buf, .count = 1 };
	const struct spi_buf_set rx_set = { .buffers = &rx_buf, .count = 1 };

	return spi_transceive_dt(&cfg->bus, &tx_set, &rx_set);
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
