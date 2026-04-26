/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * FPGA SPI register driver — see fpga_spi.h for framing details.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include "fpga_spi.h"

LOG_MODULE_REGISTER(fpga_spi, CONFIG_FPGA_SPI_LOG_LEVEL);

#define FPGA_SPI_NODE DT_CHOSEN(zephyr_fpga_spi)

BUILD_ASSERT(DT_NODE_EXISTS(FPGA_SPI_NODE),
	     "chosen `zephyr,fpga-spi` must point to the SPI slave node on the FPGA");

struct fpga_spi_data {
	struct k_mutex lock;
};

struct fpga_spi_config {
	struct spi_dt_spec bus;
};

static struct fpga_spi_data fpga_spi_data_inst;

static const struct fpga_spi_config fpga_spi_config_inst = {
	/* FPGA SPI_SLAVE supports SPI mode 0 only (CPOL=0, CPHA=0). */
	.bus = SPI_DT_SPEC_GET(FPGA_SPI_NODE,
			       SPI_OP_MODE_MASTER | SPI_WORD_SET(8) |
			       SPI_TRANSFER_MSB, 0),
};

static int fpga_spi_init(const struct device *dev)
{
	struct fpga_spi_data *data = dev->data;
	const struct fpga_spi_config *cfg = dev->config;

	k_mutex_init(&data->lock);

	if (!spi_is_ready_dt(&cfg->bus)) {
		LOG_ERR("SPI bus %s not ready", cfg->bus.bus->name);
		return -ENODEV;
	}

	LOG_INF("FPGA SPI ready on %s @ %u Hz",
		cfg->bus.bus->name, cfg->bus.config.frequency);
	return 0;
}

DEVICE_DEFINE(fpga_spi, "fpga_spi0", fpga_spi_init, NULL,
	      &fpga_spi_data_inst, &fpga_spi_config_inst,
	      POST_KERNEL, CONFIG_FPGA_SPI_INIT_PRIORITY, NULL);

const struct device *fpga_spi_get_dev(void)
{
	return DEVICE_GET(fpga_spi);
}

int fpga_spi_read(const struct device *dev, uint8_t reg,
		  uint8_t *buf, size_t len)
{
	if (dev == NULL || buf == NULL || len == 0) {
		return -EINVAL;
	}

	const struct fpga_spi_config *cfg = dev->config;
	struct fpga_spi_data *data = dev->data;

	/* bit7 = 0 for read */
	uint8_t cmd = reg & 0x7F;

	/* TX: command byte followed by dummy bytes so the SPI engine keeps
	 * clocking while the FPGA shifts out data. A single buffer makes this
	 * a single CS-low transaction (required by the FPGA framing).
	 */
	uint8_t tx[1 + 32];
	uint8_t rx[1 + 32];

	if (len > sizeof(tx) - 1) {
		return -EMSGSIZE;
	}

	tx[0] = cmd;
	memset(&tx[1], 0, len);

	const struct spi_buf tx_buf = { .buf = tx, .len = 1 + len };
	const struct spi_buf rx_buf = { .buf = rx, .len = 1 + len };
	const struct spi_buf_set tx_set = { .buffers = &tx_buf, .count = 1 };
	const struct spi_buf_set rx_set = { .buffers = &rx_buf, .count = 1 };

	k_mutex_lock(&data->lock, K_FOREVER);
	int ret = spi_transceive_dt(&cfg->bus, &tx_set, &rx_set);
	k_mutex_unlock(&data->lock);

	if (ret < 0) {
		LOG_ERR("read reg 0x%02x failed: %d", reg, ret);
		return ret;
	}

	memcpy(buf, &rx[1], len);
	return 0;
}

int fpga_spi_write(const struct device *dev, uint8_t reg,
		   const uint8_t *buf, size_t len)
{
	if (dev == NULL || (len > 0 && buf == NULL)) {
		return -EINVAL;
	}

	const struct fpga_spi_config *cfg = dev->config;
	struct fpga_spi_data *data = dev->data;

	uint8_t tx[1 + 32];

	if (len > sizeof(tx) - 1) {
		return -EMSGSIZE;
	}

	tx[0] = 0x80 | (reg & 0x7F);
	if (len > 0) {
		memcpy(&tx[1], buf, len);
	}

	const struct spi_buf tx_buf = { .buf = tx, .len = 1 + len };
	const struct spi_buf_set tx_set = { .buffers = &tx_buf, .count = 1 };

	k_mutex_lock(&data->lock, K_FOREVER);
	int ret = spi_write_dt(&cfg->bus, &tx_set);
	k_mutex_unlock(&data->lock);

	if (ret < 0) {
		LOG_ERR("write reg 0x%02x failed: %d", reg, ret);
	}
	return ret;
}

/* =====================================================================
 * Ethernet packet framing
 * =====================================================================
 *
 * RX length (0x21): 2 bytes, big-endian on the wire. Byte 0 = length[10:8],
 * byte 1 = length[7:0]. 0 means no frame pending.
 *
 * RX data  (0x22): exactly `length` bytes. Clocking the last byte out tells
 * the FPGA the host has consumed the frame (packet_available drops).
 *
 * TX data  (0x20): [len_hi, len_lo, data[0..len-1]] where `len` is the full
 * frame length. The FPGA re-programs its own write_bytes watermark once the
 * first two bytes have been clocked in, so the transaction keeps running
 * until the whole frame has been shifted.
 */

int fpga_spi_eth_rx_len(const struct device *dev)
{
	uint8_t hdr[2];
	int ret = fpga_spi_read(dev, FPGA_SPI_REG_ETH_RX_LEN, hdr, sizeof(hdr));

	if (ret < 0) {
		return ret;
	}
	return ((int)(hdr[0] & 0x07) << 8) | hdr[1];
}

/* Shared RX scratch, same reasoning as eth_tx_scratch above: ESP32 SPI
 * controllers split multi-segment spi_buf_sets in a way the FPGA does
 * not tolerate, so we use a single contiguous TX/RX buffer pair. */
static uint8_t eth_rx_tx_scratch[1 + FPGA_SPI_ETH_MAX_PKT];
static uint8_t eth_rx_rx_scratch[1 + FPGA_SPI_ETH_MAX_PKT];

int fpga_spi_eth_rx_read(const struct device *dev,
			 uint8_t *buf, size_t len)
{
	if (dev == NULL || buf == NULL || len == 0 ||
	    len > FPGA_SPI_ETH_MAX_PKT) {
		return -EINVAL;
	}

	const struct fpga_spi_config *cfg = dev->config;
	struct fpga_spi_data *data = dev->data;

	k_mutex_lock(&data->lock, K_FOREVER);

	eth_rx_tx_scratch[0] = FPGA_SPI_REG_ETH_RX_DATA & 0x7F;
	memset(&eth_rx_tx_scratch[1], 0, len);

	const struct spi_buf tx_buf = {
		.buf = eth_rx_tx_scratch, .len = 1 + len,
	};
	const struct spi_buf rx_buf = {
		.buf = eth_rx_rx_scratch, .len = 1 + len,
	};
	const struct spi_buf_set tx_set = { .buffers = &tx_buf, .count = 1 };
	const struct spi_buf_set rx_set = { .buffers = &rx_buf, .count = 1 };

	int ret = spi_transceive_dt(&cfg->bus, &tx_set, &rx_set);
	if (ret >= 0) {
		memcpy(buf, &eth_rx_rx_scratch[1], len);
	}

	k_mutex_unlock(&data->lock);

	if (ret < 0) {
		LOG_ERR("eth rx read (%zu) failed: %d", len, ret);
	}
	return ret;
}

/* Scratch buffer shared by fpga_spi_eth_tx_write. Some SPI controllers
 * (ESP32 included) don't reliably emit multi-segment spi_buf_set bursts
 * as one continuous frame, so we concatenate the command byte, the
 * length header and the payload into a single contiguous buffer. Mutex-
 * protected by the existing bus lock. */
static uint8_t eth_tx_scratch[3 + FPGA_SPI_ETH_MAX_PKT];

int fpga_spi_eth_tx_write(const struct device *dev,
			  const uint8_t *buf, size_t len)
{
	if (dev == NULL || buf == NULL || len == 0 ||
	    len > FPGA_SPI_ETH_MAX_PKT) {
		return -EINVAL;
	}

	const struct fpga_spi_config *cfg = dev->config;
	struct fpga_spi_data *data = dev->data;

	k_mutex_lock(&data->lock, K_FOREVER);

	/* [cmd, len_hi, len_lo, payload…] in one contiguous write. */
	eth_tx_scratch[0] = (uint8_t)(0x80 | (FPGA_SPI_REG_ETH_TX & 0x7F));
	eth_tx_scratch[1] = (uint8_t)((len >> 8) & 0xFF);
	eth_tx_scratch[2] = (uint8_t)(len & 0xFF);
	memcpy(&eth_tx_scratch[3], buf, len);

	const struct spi_buf tx_buf = {
		.buf = eth_tx_scratch, .len = 3 + len,
	};
	const struct spi_buf_set tx_set = { .buffers = &tx_buf, .count = 1 };

	int ret = spi_write_dt(&cfg->bus, &tx_set);

	k_mutex_unlock(&data->lock);

	if (ret < 0) {
		LOG_ERR("eth tx write (%zu) failed: %d", len, ret);
	}
	return ret;
}
