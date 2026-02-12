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
