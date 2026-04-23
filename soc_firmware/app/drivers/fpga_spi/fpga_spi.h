/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * Public API for the FPGA SPI register driver.
 *
 * This driver talks to the on-FPGA `spictrl` block (see FPGA/spictrl.vhd and
 * config_ram_address_map.md) over a standard Zephyr SPI bus. The command
 * framing is:
 *
 *     byte 0 : [R/W][addr6..addr0]   (R/W=1 -> write, 0 -> read)
 *     byte 1..N : payload
 *
 * Transaction length is implicit in the register address. CS_N is held low
 * for the whole command; the FPGA returns to idle after the declared payload
 * length. Multi-byte scalar writes (MAC, IP, PTP config, flags) are atomic:
 * the FPGA shadows bytes and commits on the last byte. Stream-config writes
 * (0x58/0x59) pass through byte-wise into the FPGA stream RAM.
 */

#ifndef FPGA_SPI_H_
#define FPGA_SPI_H_

#include <zephyr/device.h>
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Register addresses (7-bit) ---- */
#define FPGA_SPI_REG_INFO         0x00 /* R 8  */
#define FPGA_SPI_REG_MAC          0x40 /* W 6  */
#define FPGA_SPI_REG_IP           0x41 /* W 4  */
#define FPGA_SPI_REG_STATUS_CLK   0x50 /* R 1  */
#define FPGA_SPI_REG_FLAGS        0x50 /* W 1  */
#define FPGA_SPI_REG_STATUS_ETH   0x51 /* R 1  */
#define FPGA_SPI_REG_PATH_DELAY   0x52 /* R 4  */
#define FPGA_SPI_REG_PTP_OFFSET   0x53 /* R 4  */
#define FPGA_SPI_REG_PPB_COUNTERS 0x54 /* R 8  */
#define FPGA_SPI_REG_GM_ID        0x55 /* R 8  */
#define FPGA_SPI_REG_PTP_CONFIG   0x55 /* W 7  */
#define FPGA_SPI_REG_TX_STREAM    0x58 /* W 20 */
#define FPGA_SPI_REG_RX_STREAM    0x59 /* W 18 */

/* ---- Clocking status bits (read 0x50) ---- */
#define FPGA_SPI_CLK_PPB_VALID     BIT(7)
#define FPGA_SPI_CLK_WC_LOCKED     BIT(6)
#define FPGA_SPI_CLK_WC_CONFIGURED BIT(5)
#define FPGA_SPI_CLK_PTP_LEADER    BIT(4)
#define FPGA_SPI_CLK_PTP_FOLLOWER  BIT(3)

/* ---- Ethernet status bits (read 0x51) ---- */
#define FPGA_SPI_ETH_LINK_UP       BIT(7)
#define FPGA_SPI_ETH_SPEED_SHIFT   5
#define FPGA_SPI_ETH_SPEED_MASK    (BIT(6) | BIT(5))

/* ---- Flag register bits (write 0x50) ---- */
#define FPGA_SPI_FLAG_PPB_START        BIT(0)
#define FPGA_SPI_FLAG_RESET_WALLCLOCK  BIT(1)
#define FPGA_SPI_FLAG_RESET_PTP        BIT(2)
#define FPGA_SPI_FLAG_RESET_ETHERNET   BIT(3)
#define FPGA_SPI_FLAG_METER_CLEAR      BIT(4)
#define FPGA_SPI_FLAG_ADDA_NRST        BIT(5)

/**
 * @brief Get the FPGA SPI driver device pointer.
 *
 * @return Device pointer, or NULL if the driver is not ready.
 */
const struct device *fpga_spi_get_dev(void);

/**
 * @brief Raw register read.
 *
 * Sends a command byte with bit7=0 and receives @p len bytes into @p buf.
 * Byte ordering on the wire follows the FPGA register map (little-endian
 * for numeric reads, see config_ram_address_map.md).
 */
int fpga_spi_read(const struct device *dev, uint8_t reg,
		  uint8_t *buf, size_t len);

/**
 * @brief Raw register write.
 *
 * Sends a command byte with bit7=1 followed by @p len bytes from @p buf.
 * The FPGA commits scalar writes atomically on the last byte.
 */
int fpga_spi_write(const struct device *dev, uint8_t reg,
		   const uint8_t *buf, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* FPGA_SPI_H_ */
