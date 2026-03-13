/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: Apache-2.0
 *
 * Public API for the LiteX SoC Ethernet driver.
 * Accesses the FPGA via LiteX CSR registers and a Wishbone-mapped
 * packet buffer (EthPacketBuffer) instead of the old FMC register window.
 *
 * All register addresses are imported from the LiteX-generated headers
 * (csr.h, mem.h, soc.h) so they stay in sync when the SoC is regenerated.
 */

#ifndef ETH_LITEX_H_
#define ETH_LITEX_H_

#include <zephyr/device.h>
#include <zephyr/net/net_ip.h>
#include <stdint.h>
#include <stddef.h>

/* Import LiteX-generated register definitions.
 * This pulls in CSR_BASE, CSR_AES67_CSR_*, CSR_ETH_BUF_*, ETH_BUF_BASE,
 * ETH_BUF_INTERRUPT, etc. via the compatibility shim. */
#include "litex_csr_compat.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * Derived addresses from LiteX-generated headers
 * ======================================================================== */

/* Packet buffer memory regions (from generated/mem.h: ETH_BUF_BASE) */
#define ETH_BUF_RX_MEM   ((uintptr_t)ETH_BUF_BASE + 0x000UL) /* RX: byte 0..1499 */
#define ETH_BUF_TX_MEM   ((uintptr_t)ETH_BUF_BASE + 0x800UL) /* TX: byte 0..1499 */

/* ---- Status register bit fields (CSR_AES67_CSR_STATUS) ---- */
#define AES67_STATUS_WC_LOCKED      BIT(CSR_AES67_CSR_STATUS_WALLCLOCK_LOCKED_OFFSET)
#define AES67_STATUS_WC_PHASEJUMP   BIT(CSR_AES67_CSR_STATUS_WALLCLOCK_PHASEJUMP_OFFSET)
#define AES67_STATUS_WC_CONFIGURED  BIT(CSR_AES67_CSR_STATUS_WALLCLOCK_CONFIGURED_OFFSET)
#define AES67_STATUS_PTP_SYNC_LOST  BIT(CSR_AES67_CSR_STATUS_PTP_SYNC_LOST_OFFSET)
#define AES67_STATUS_ETH_LINK_UP    BIT(CSR_AES67_CSR_STATUS_ETH_LINK_UP_OFFSET)
#define AES67_STATUS_ETH_SPEED_SHIFT CSR_AES67_CSR_STATUS_ETH_SPEED_OFFSET
#define AES67_STATUS_ETH_SPEED_MASK  (((1U << CSR_AES67_CSR_STATUS_ETH_SPEED_SIZE) - 1) \
                                      << AES67_STATUS_ETH_SPEED_SHIFT)
#define AES67_STATUS_ETH_TX_DONE    BIT(CSR_AES67_CSR_STATUS_ETH_TX_DONE_OFFSET)
#define AES67_STATUS_ETH_RX_OVF     BIT(CSR_AES67_CSR_STATUS_ETH_RX_OVERFLOW_OFFSET)

/* ---- Control register bit fields (CSR_AES67_CSR_CTRL) ---- */
#define AES67_CTRL_PPB_START        BIT(CSR_AES67_CSR_CTRL_PLL_PPB_START_OFFSET)
#define AES67_CTRL_PTP_IS_LEADER    BIT(CSR_AES67_CSR_CTRL_PTP_IS_LEADER_OFFSET)
#define AES67_CTRL_PTP_IS_FOLLOWER  BIT(CSR_AES67_CSR_CTRL_PTP_IS_FOLLOWER_OFFSET)
#define AES67_CTRL_ETH_TX_REQUEST   BIT(CSR_AES67_CSR_CTRL_ETH_TX_REQUEST_OFFSET)

/* ---- PPB status bit fields ---- */
#define AES67_PPB_STATUS_VALID      BIT(CSR_AES67_CSR_PLL_PPB_STATUS_VALID_OFFSET)

/* ---- EventManager bit fields ---- */
#define ETH_BUF_EV_RX_READY        BIT(CSR_ETH_BUF_EV_PENDING_RX_READY_OFFSET)

/* Maximum Ethernet frame size (without FCS) */
#define ETH_LITEX_MAX_PKT_SIZE 1500

/* ========================================================================
 * CSR access primitives
 * ======================================================================== */

static inline uint32_t litex_csr_read(uintptr_t addr)
{
	return *(volatile uint32_t *)addr;
}

static inline void litex_csr_write(uintptr_t addr, uint32_t val)
{
	*(volatile uint32_t *)addr = val;
}

/* ========================================================================
 * Public API
 * ======================================================================== */

/**
 * @brief Set bits in the AES67 control register (read-modify-write).
 * Thread-safe: uses a spinlock internally.
 */
int eth_litex_ctrl_set_bits(const struct device *dev, uint32_t bits);

/**
 * @brief Clear bits in the AES67 control register (read-modify-write).
 * Thread-safe: uses a spinlock internally.
 */
int eth_litex_ctrl_clear_bits(const struct device *dev, uint32_t bits);

/**
 * @brief Write the MAC address to the FPGA CSR registers.
 */
int eth_litex_write_mac(const struct device *dev, const uint8_t mac[6]);

/**
 * @brief Write the IP address to the FPGA CSR register.
 */
int eth_litex_write_ip(const struct device *dev, const struct in_addr *ip);

/**
 * @brief Write PTP configuration to FPGA CSR registers.
 */
int eth_litex_write_ptp_config(const struct device *dev,
			       const uint8_t leader_clock_id[8],
			       uint8_t time_source,
			       int8_t log_msg_interval,
			       int8_t log_announce_interval);

/**
 * @brief Read the PPB measurement counters.
 * @return true if measurement is valid, false otherwise
 */
bool eth_litex_read_ppb_counts(const struct device *dev,
			       uint32_t *wc_count, uint32_t *pll_count);

/**
 * @brief Read PTP path delay from FPGA.
 */
uint32_t eth_litex_read_path_delay(const struct device *dev);

/**
 * @brief Read PTP offset from master from FPGA.
 */
uint32_t eth_litex_read_ptp_offset(const struct device *dev);

/**
 * @brief Read the FPGA status register.
 */
uint32_t eth_litex_read_status(const struct device *dev);

#ifdef __cplusplus
}
#endif

#endif /* ETH_LITEX_H_ */
