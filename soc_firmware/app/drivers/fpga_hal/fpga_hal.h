/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: Apache-2.0
 *
 * Hardware Abstraction Layer for FPGA register access.
 *
 * Provides a uniform API for application code to access FPGA configuration
 * registers, regardless of the underlying transport:
 *   - FMC bus (STM32H7 → eth_fmc_basic driver)
 *   - LiteX CSR (VexRiscv → eth_litex driver)
 *
 * Application code should include only this header and call fpga_hal_*()
 * functions.  The active backend is selected at build time via Kconfig.
 */

#ifndef FPGA_HAL_H_
#define FPGA_HAL_H_

#include <zephyr/device.h>
#include <zephyr/net/net_ip.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Status flags (backend-independent) ---- */
#define FPGA_HAL_CLK_PPB_VALID       BIT(0)
#define FPGA_HAL_CLK_WC_LOCKED       BIT(1)
#define FPGA_HAL_CLK_WC_PHASEJUMP    BIT(2)
#define FPGA_HAL_CLK_WC_CONFIGURED   BIT(3)
#define FPGA_HAL_CLK_PTP_LEADER_LOST BIT(4)

#define FPGA_HAL_ETH_LINK_UP         BIT(8)
#define FPGA_HAL_ETH_SPEED_MASK      (BIT(9) | BIT(10))
#define FPGA_HAL_ETH_SPEED_SHIFT     9

/* ---- Control flags (backend-independent) ---- */
#define FPGA_HAL_CTRL_PPB_START       BIT(0)
#define FPGA_HAL_CTRL_RESET_WALLCLOCK BIT(1)
#define FPGA_HAL_CTRL_RESET_PTP       BIT(2)
#define FPGA_HAL_CTRL_RESET_ETHERNET  BIT(3)
#define FPGA_HAL_CTRL_PTP_IS_LEADER   BIT(4)
#define FPGA_HAL_CTRL_PTP_IS_FOLLOWER BIT(5)

/**
 * @brief Callback type for FPGA recovery events.
 *
 * Called when the FPGA transitions from not-ready to ready.
 * Used to re-write configuration registers after FPGA reprogramming.
 */
typedef void (*fpga_hal_recover_cb_t)(void *user_data);

/* ========================================================================
 * Device accessor
 * ======================================================================== */

/**
 * @brief Get the FPGA HAL device pointer.
 *
 * Returns the underlying driver device ("eth_fmc0" or "eth_litex0")
 * depending on which backend is active.
 *
 * @return Pointer to device, or NULL if not found.
 */
const struct device *fpga_hal_get_dev(void);

/* ========================================================================
 * FPGA ready / recovery
 * ======================================================================== */

/**
 * @brief Check if the FPGA is ready for register access.
 *
 * On FMC: polls GPIO / FMC bus for valid responses.
 * On LiteX: always returns true (SoC contains the FPGA logic).
 */
bool fpga_hal_is_ready(void);

/**
 * @brief Block until the FPGA is ready.
 *
 * @param timeout_ms  Maximum wait time in milliseconds (0 = forever).
 * @return 0 on success, -ETIMEDOUT on timeout.
 */
int fpga_hal_wait_ready(uint32_t timeout_ms);

/**
 * @brief Register a callback invoked when the FPGA becomes ready
 *        after a reset or reprogramming event.
 *
 * On LiteX this is a no-op (FPGA is always ready).
 *
 * @param cb         Callback function (NULL to unregister).
 * @param user_data  Context pointer passed to the callback.
 */
void fpga_hal_register_recover_cb(fpga_hal_recover_cb_t cb, void *user_data);

/* ========================================================================
 * Configuration writes
 * ======================================================================== */

/**
 * @brief Write the MAC address to the FPGA.
 */
int fpga_hal_write_mac(const uint8_t mac[6]);

/**
 * @brief Write the IPv4 address to the FPGA.
 */
int fpga_hal_write_ip(const struct in_addr *ip);

/**
 * @brief Write PTP configuration to the FPGA.
 *
 * @param leader_clock_id        8-byte leader clock identity (big-endian).
 * @param time_source            PTP time source (IEEE 1588 enum).
 * @param log_msg_interval       logMessageInterval for Sync (signed).
 * @param log_announce_interval  logAnnounceInterval (signed).
 */
int fpga_hal_write_ptp_config(const uint8_t leader_clock_id[8],
			      uint8_t time_source,
			      int8_t log_msg_interval,
			      int8_t log_announce_interval);

/**
 * @brief Write PTP grandmaster quality fields to the FPGA.
 *
 * @param priority1       GM priority1 (0–255).
 * @param priority2       GM priority2 (0–255).
 * @param clock_class     GM clock class (e.g. 6=GPS, 248=default).
 * @param clock_accuracy  GM clock accuracy (IEEE 1588 enum).
 */
int fpga_hal_write_ptp_gm_quality(uint8_t priority1, uint8_t priority2,
				  uint8_t clock_class, uint8_t clock_accuracy);

/**
 * @brief Write TX stream configuration to the FPGA.
 *
 * @param stream_id        Stream index (0..7).
 * @param dst_ip           Destination IP (network byte order).
 * @param channel_count    Number of channels (1..8).
 * @param samples_per_pkt  Samples per packet per channel.
 * @param ch_ids           Array of channel IDs (up to 8).
 * @param num_ch_ids       Number of entries in ch_ids.
 * @param ssrc             32-bit SSRC for the RTP header (host byte order).
 * @return 0 on success, negative errno on error.
 */
int fpga_hal_write_tx_stream_config(uint8_t stream_id,
				    const struct in_addr *dst_ip,
				    uint8_t channel_count,
				    uint8_t samples_per_pkt,
				    const uint8_t *ch_ids,
				    uint8_t num_ch_ids,
				    uint32_t ssrc);

/**
 * @brief Write RX stream configuration to the FPGA.
 *
 * @param stream_id            Stream index (0..N).
 * @param dst_ip               Destination IP to match (multicast group).
 * @param dst_port             Destination UDP port to match (host byte order).
 * @param ch_map               Output channel map.
 * @param channel_count        Number of channels (1..8).
 * @param output_delay         Output delay in samples.
 * @param samples_per_channel  Samples per channel per packet.
 * @return 0 on success, negative errno on error.
 */
int fpga_hal_write_rx_stream_config(uint8_t stream_id,
				    const struct in_addr *dst_ip,
				    uint16_t dst_port,
				    const uint8_t *ch_map,
				    uint8_t channel_count,
				    uint8_t output_delay,
				    uint8_t samples_per_channel);

/* ========================================================================
 * Control register (set/clear bits)
 * ======================================================================== */

/**
 * @brief Set bits in the FPGA control register.
 * Thread-safe (uses spinlock internally).
 */
int fpga_hal_ctrl_set_bits(uint32_t bits);

/**
 * @brief Clear bits in the FPGA control register.
 * Thread-safe (uses spinlock internally).
 */
int fpga_hal_ctrl_clear_bits(uint32_t bits);

/**
 * @brief Assert or release the AD/DA card hardware reset line.
 *
 * @param released  true  = release reset (nRST high, board runs normally).
 *                  false = assert reset  (nRST low,  board held in reset).
 */
int fpga_hal_set_adda_nrst(bool released);

/* ========================================================================
 * Status reads
 * ======================================================================== */

/**
 * @brief Read combined FPGA status word.
 *
 * Returns a bitmask using the FPGA_HAL_CLK_* and FPGA_HAL_ETH_* flags
 * defined above.  Both clocking and ethernet status are merged into a
 * single 32-bit value.
 */
uint32_t fpga_hal_read_status(void);

/**
 * @brief Read PTP path delay from FPGA (nanoseconds).
 */
int32_t fpga_hal_read_path_delay(void);

/**
 * @brief Read PTP leader offset from FPGA (nanoseconds).
 */
int32_t fpga_hal_read_ptp_offset(void);

/**
 * @brief Read PPB measurement counters.
 *
 * @param wc_count   Output: wallclock edge count (22-bit).
 * @param pll_count  Output: PLL edge count (22-bit).
 * @return true if measurement is valid, false otherwise.
 */
bool fpga_hal_read_ppb_counts(uint32_t *wc_count, uint32_t *pll_count);

#ifdef __cplusplus
}
#endif

#endif /* FPGA_HAL_H_ */
