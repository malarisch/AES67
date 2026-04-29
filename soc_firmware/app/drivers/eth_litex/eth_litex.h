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

/* Packet buffer memory regions (from generated/mem.h: ETH_BUF_BASE)
 * Each byte occupies one 32-bit word, so byte N is at word address N.
 * TX region starts at word address 0x800 (bit 11), i.e. byte address 0x2000. */
#define ETH_BUF_RX_MEM   ((uintptr_t)ETH_BUF_BASE + 0x0000UL) /* RX: byte 0..1517 */
#define ETH_BUF_TX_MEM   ((uintptr_t)ETH_BUF_BASE + 0x2000UL) /* TX: byte 0..1517 */

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
#define AES67_STATUS_PTP_IS_LEADER  BIT(CSR_AES67_CSR_STATUS_PTP_IS_LEADER_OFFSET)
#define AES67_STATUS_PTP_IS_FOLLOWER BIT(CSR_AES67_CSR_STATUS_PTP_IS_FOLLOWER_OFFSET)

/* ---- Control register bit fields (CSR_AES67_CSR_CTRL) ---- */
#define AES67_CTRL_PPB_START        BIT(CSR_AES67_CSR_CTRL_PLL_PPB_START_OFFSET)
#define AES67_CTRL_ETH_TX_REQUEST   BIT(CSR_AES67_CSR_CTRL_ETH_TX_REQUEST_OFFSET)
#define AES67_CTRL_ADDA_NRST        BIT(CSR_AES67_CSR_CTRL_ADDA_NRST_OFFSET)

/* ---- PPB status bit fields ---- */
#define AES67_PPB_STATUS_VALID      BIT(CSR_AES67_CSR_PLL_PPB_STATUS_VALID_OFFSET)

/* ---- EventManager bit fields ---- */
#define ETH_BUF_EV_RX_READY        BIT(CSR_ETH_BUF_EV_PENDING_RX_READY_OFFSET)

/* Maximum Ethernet frame size (without FCS): 6 dst + 6 src + 4 VLAN + 2 EtherType + 1500 payload */
#define ETH_LITEX_MAX_PKT_SIZE 1518

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
			       uint8_t time_source,
			       int8_t log_msg_interval,
			       int8_t log_announce_interval);

/**
 * @brief Read the PTP leader clock identity selected by the FPGA BMA.
 *
 * @param dev              Device pointer.
 * @param leader_clock_id  Output: 8-byte clock identity (big-endian).
 * @return true if a leader is selected (non-zero ID), false otherwise.
 */
bool eth_litex_read_ptp_leader_id(const struct device *dev,
				  uint8_t leader_clock_id[8]);

/**
 * @brief Write PTP grandmaster quality fields to FPGA CSR registers.
 */
int eth_litex_write_ptp_gm_quality(const struct device *dev,
				   uint8_t priority1, uint8_t priority2,
				   uint8_t clock_class, uint8_t clock_accuracy);

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

/**
 * @brief Write TX stream configuration to the stream config RAM.
 *
 * @param dev              Device pointer.
 * @param stream_id        Stream index (0..7).
 * @param dst_ip           Destination IP (network byte order).
 * @param channel_count    Number of channels (1..8).
 * @param samples_per_pkt  Samples per packet per channel.
 * @param ch_ids           Array of channel IDs (up to 8).
 * @param num_ch_ids       Number of entries in ch_ids.
 * @param ssrc             32-bit SSRC for the RTP header (host byte order).
 * @return 0 on success, negative errno on error.
 */
int eth_litex_write_tx_stream_config(const struct device *dev,
				     uint8_t stream_id,
				     const struct in_addr *dst_ip,
				     uint8_t channel_count,
				     uint8_t samples_per_pkt,
				     const uint8_t *ch_ids,
				     uint8_t num_ch_ids,
				     uint32_t ssrc);

/* ========================================================================
 * PTP servo / parser tuning + monitoring
 * ======================================================================== */

struct eth_litex_ptp_tuning {
	int8_t   kp_gain;
	int8_t   ki_gain;
	uint8_t  gain_shift;
	uint8_t  gain_shift_locked;
	uint8_t  ki_extra_shift;
	uint8_t  filter_shift;
	uint8_t  warmup_samples;
	uint32_t lock_threshold_ns;
	uint32_t unlock_threshold_ns;
	uint8_t  lock_count_threshold;
	bool     min_filter_enable;
	uint8_t  min_filter_active_depth;
};

struct eth_litex_ptp_monitor {
	int32_t  filtered_offset;
	int32_t  integral_sum;
	int32_t  pi_proportional;
	int32_t  pi_sum_raw;
	uint8_t  effective_gain_shift;
	uint16_t lock_counter;
	uint16_t sample_count;
	bool     first_lock_achieved;
};

int  eth_litex_write_ptp_tuning(const struct device *dev,
				const struct eth_litex_ptp_tuning *t);
void eth_litex_read_ptp_tuning(const struct device *dev,
				struct eth_litex_ptp_tuning *t);
void eth_litex_read_ptp_monitor(const struct device *dev,
				struct eth_litex_ptp_monitor *m);
int  eth_litex_set_ptp_reset(const struct device *dev, bool held_in_reset);

/**
 * @brief Write RX stream configuration to the stream config RAM.
 *
 * @param dev                  Device pointer.
 * @param stream_id            Stream index (0..N).
 * @param dst_ip               Destination IP to match (multicast group).
 * @param dst_port             Destination UDP port to match (host byte order).
 * @param ch_map               Output channel map.
 * @param channel_count        Number of channels (1..8).
 * @param output_delay         Output delay in samples.
 * @param samples_per_channel  Samples per channel per packet.
 * @return 0 on success, negative errno on error.
 */
int eth_litex_write_rx_stream_config(const struct device *dev,
				     uint8_t stream_id,
				     const struct in_addr *dst_ip,
				     uint16_t dst_port,
				     const uint8_t *ch_map,
				     uint8_t channel_count,
				     uint8_t output_delay,
				     uint8_t samples_per_channel);

#ifdef __cplusplus
}
#endif

#endif /* ETH_LITEX_H_ */
