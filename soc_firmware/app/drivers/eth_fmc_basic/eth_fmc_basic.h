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
#include <zephyr/net/net_ip.h>
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
#define ETH_FMC_FLAG_PTP_IS_LEADER   BIT(4)  /* PTP: this node is leader */
#define ETH_FMC_FLAG_PTP_IS_FOLLOWER BIT(5)  /* PTP: this node is follower */

/* Write register 0x55 - PTP configuration (11-byte auto-increment write)
 * Bytes 1..8:  Current leader clock identity (EUI-64)
 * Byte  9:     PTP time source
 * Byte 10:     PTP logMessageInterval (sync)
 * Byte 11:     PTP logAnnounceInterval
 */
#define ETH_FMC_REG_PTP_CONFIG 0x55
#define ETH_FMC_PTP_CONFIG_LEN 12  /* 11 data bytes + 1 dummy to trigger FPGA latch */

/* Write register 0x58 - TX stream config (16-byte auto-increment write)
 * Byte  0:     stream_id (0..7) — FPGA computes base addr = stream_id * 16
 * Bytes 1..4:  Destination IP address (network byte order, MSB first)
 * Byte  5:     Channel count (1..8)
 * Byte  6:     Samples per packet per channel
 * Bytes 7..14: Channel IDs (up to 8, one byte each)
 * Byte  15:    Reserved
 */
#define ETH_FMC_REG_TX_STREAM_CFG  0x58
#define ETH_FMC_TX_STREAM_CFG_LEN  16

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

/* ---- Convenience helpers ---- */

/**
 * @brief Set one or more bits in the FPGA status register 0x50
 *        and write the full shadow value to the hardware.
 *
 * Thread-safe: uses a spinlock to protect the shadow byte.
 *
 * @param dev   The FMC Ethernet bridge device
 * @param bits  Bitmask of ETH_FMC_FLAG_* bits to set
 * @return 0 on success, negative errno on error
 */
int eth_fmc_status_set_bits(const struct device *dev, uint8_t bits);

/**
 * @brief Clear one or more bits in the FPGA status register 0x50
 *        and write the full shadow value to the hardware.
 *
 * Thread-safe: uses a spinlock to protect the shadow byte.
 *
 * @param dev   The FMC Ethernet bridge device
 * @param bits  Bitmask of ETH_FMC_FLAG_* bits to clear
 * @return 0 on success, negative errno on error
 */
int eth_fmc_status_clear_bits(const struct device *dev, uint8_t bits);

/**
 * @brief Write PTP configuration to FPGA register 0x55.
 *
 * Sends an 11-byte auto-increment write containing:
 *   - 8 bytes: leader clock identity (EUI-64, big-endian)
 *   - 1 byte:  PTP time source
 *   - 1 byte:  logMessageInterval for Sync (signed)
 *   - 1 byte:  logAnnounceInterval (signed)
 *
 * @param dev                    The FMC Ethernet bridge device
 * @param leader_clock_id        8-byte leader clock identity (big-endian)
 * @param time_source            PTP time source (IEEE 1588 enum)
 * @param log_msg_interval       logMessageInterval for Sync (signed, e.g. -3 for 125ms)
 * @param log_announce_interval  logAnnounceInterval (signed, e.g. 0 for 1s, 1 for 2s)
 * @return 0 on success, negative errno on error
 */
int eth_fmc_write_ptp_config(const struct device *dev,
			    const uint8_t leader_clock_id[8],
			    uint8_t time_source,
			    int8_t log_msg_interval,
			    int8_t log_announce_interval);

/**
 * @brief Write TX stream configuration to FPGA register 0x58.
 *
 * Sends a 16-byte auto-increment write to configure one TX audio stream
 * in the tx_router's config RAM.
 *
 * @param dev              The FMC Ethernet bridge device
 * @param stream_id        Stream index (0..7)
 * @param dst_ip           Destination IP (network byte order)
 * @param channel_count    Number of channels (1..8)
 * @param samples_per_pkt  Samples per packet per channel
 * @param ch_ids           Array of channel IDs (up to 8)
 * @param num_ch_ids       Number of entries in ch_ids
 * @return 0 on success, negative errno on error
 */
int eth_fmc_write_tx_stream_config(const struct device *dev,
				   uint8_t stream_id,
				   const struct in_addr *dst_ip,
				   uint8_t channel_count,
				   uint8_t samples_per_pkt,
				   const uint8_t *ch_ids,
				   uint8_t num_ch_ids);

#ifdef __cplusplus
}
#endif

#endif /* ETH_FMC_BASIC_H_ */
