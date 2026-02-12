/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: Apache-2.0
 *
 * Public API for the FMC Ethernet bridge driver.
 * Exposes register-level access to the FPGA configuration registers
 * mapped through the FMC bus.
 */

#ifndef ETH_FMC_BASIC_H_
#define ETH_FMC_BASIC_H_

#include <zephyr/device.h>
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- FPGA configuration register addresses ---- */
#define ETH_FMC_REG_MAC_ADDR   0x40  /* Write 6 bytes (auto-increment) */
#define ETH_FMC_REG_IP_ADDR    0x41  /* Write 4 bytes (auto-increment) */
#define ETH_FMC_REG_AUDIO_STAT 0x42  /* Audio status (reserved) */

/* ---- Status / control registers (0x50 region) ---- */

/* Write register 0x50 - flag bitmask */
#define ETH_FMC_REG_STATUS_WR  0x50
#define ETH_FMC_FLAG_PPB_START       BIT(0)  /* Start PLL PPB measurement */
#define ETH_FMC_FLAG_RESET_WALLCLOCK BIT(1)  /* Pulse reset wallclock */
#define ETH_FMC_FLAG_RESET_PTP       BIT(2)  /* Pulse reset PTP */
#define ETH_FMC_FLAG_RESET_ETHERNET  BIT(3)  /* Pulse reset Ethernet */

/* Read register 0x50 - clocking flags */
#define ETH_FMC_REG_STATUS_CLK 0x50
#define ETH_FMC_CLK_PPB_VALID        BIT(0)  /* PLL PPB measurement valid */
#define ETH_FMC_CLK_WC_LOCKED        BIT(1)  /* Wallclock locked */
#define ETH_FMC_CLK_WC_PHASEJUMP     BIT(2)  /* Wallclock did phasejump */
#define ETH_FMC_CLK_WC_CONFIGURED    BIT(3)  /* Wallclock configured */
#define ETH_FMC_CLK_PTP_LEADER_LOST  BIT(4)  /* PTP leader lost */

/* Read register 0x51 - ethernet flags */
#define ETH_FMC_REG_STATUS_ETH 0x51
#define ETH_FMC_ETH_LINK_UP          BIT(0)
#define ETH_FMC_ETH_SPEED_0          BIT(1)  /* 00=10M, 01=100M, 10=1G */
#define ETH_FMC_ETH_SPEED_1          BIT(2)
#define ETH_FMC_ETH_SPEED_MASK       (BIT(1) | BIT(2))
#define ETH_FMC_ETH_SPEED_SHIFT      1

/* Read register 0x52 - path delay, 4-byte sequential read (LSB first) */
#define ETH_FMC_REG_PATH_DELAY     0x52
/* Read register 0x53 - leader offset, 4-byte sequential read (LSB first) */
#define ETH_FMC_REG_LEADER_OFFSET  0x53
/* Read register 0x54 - PLL PPB offset, 4-byte sequential read (LSB first) */
#define ETH_FMC_REG_PPB_OFFSET     0x54

/* Read register 0x60..0x7F - direct config RAM access (addr - 0x60) */
#define ETH_FMC_REG_CONFIG_RAM     0x60

/**
 * @brief Write a block of bytes to an FPGA register address.
 *
 * The FPGA uses auto-incrementing byte writes: each successive byte
 * written to the same address is stored at the next internal offset.
 * The FPGA-side byte counter resets after the expected number of bytes
 * has been received (e.g. 6 for MAC, 4 for IP).
 *
 * @param dev   The FMC Ethernet bridge device (eth_fmc0)
 * @param reg   FPGA register address (e.g. ETH_FMC_REG_MAC_ADDR)
 * @param data  Pointer to the data buffer to write
 * @param len   Number of bytes to write
 * @return 0 on success, negative errno on error
 */
int eth_fmc_reg_write(const struct device *dev, uint8_t reg,
		      const uint8_t *data, size_t len);

/**
 * @brief Read a single byte from an FPGA register address.
 *
 * @param dev   The FMC Ethernet bridge device (eth_fmc0)
 * @param reg   FPGA register address
 * @param val   Pointer to store the read value
 * @return 0 on success, negative errno on error
 */
int eth_fmc_reg_read(const struct device *dev, uint8_t reg, uint8_t *val);

/**
 * @brief Read a block of bytes from an FPGA register address.
 *
 * Performs multiple single-byte reads from the same address.
 * Useful for auto-incrementing read windows.
 *
 * @param dev   The FMC Ethernet bridge device (eth_fmc0)
 * @param reg   FPGA register address
 * @param data  Pointer to buffer for read data
 * @param len   Number of bytes to read
 * @return 0 on success, negative errno on error
 */
int eth_fmc_reg_read_block(const struct device *dev, uint8_t reg,
			   uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* ETH_FMC_BASIC_H_ */
