/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * Zephyr Ethernet driver that carries frames over the on-FPGA spictrl
 * block via the `fpga_spi` bus driver. See drivers/fpga_spi/fpga_spi.h
 * and FPGA/spictrl.vhd.
 *
 * The public surface is intentionally small — the driver registers as
 * a normal Zephyr Ethernet device (ETH_NET_DEVICE_INIT) and everything
 * else goes through the standard net_if API.
 */

#ifndef ETH_SPI_H_
#define ETH_SPI_H_

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Retrieve the Ethernet-over-SPI device pointer.
 *
 * @return Device pointer, or NULL if the driver is not configured.
 */
const struct device *eth_spi_get_dev(void);

#ifdef __cplusplus
}
#endif

#endif /* ETH_SPI_H_ */
