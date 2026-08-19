/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: Apache-2.0
 *
 * Hardware Abstraction Layer for FPGA register access.
 *
 * Provides a uniform API for application code to access FPGA configuration
 * registers, regardless of the underlying transport:
 *   - LiteX CSR (VexRiscv → eth_litex driver)
 *   - SPI (external MCU → spibone bridge)
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
#define FPGA_HAL_PTP_IS_LEADER       BIT(11)
#define FPGA_HAL_PTP_IS_FOLLOWER     BIT(12)

/* RX stream-underrun diagnostics: per-stream underrun flags (streams 3..0)
 * and the resulting per-channel underrun-mute mask (channels 7..0). Layout
 * mirrors the FPGA status CSR; 0 on backends without the fields. */
#define FPGA_HAL_RX_UNDERRUN_SHIFT   16
#define FPGA_HAL_RX_UNDERRUN_MASK    (0xFUL << FPGA_HAL_RX_UNDERRUN_SHIFT)
#define FPGA_HAL_RX_MUTE_SHIFT       20
#define FPGA_HAL_RX_MUTE_MASK        (0xFFUL << FPGA_HAL_RX_MUTE_SHIFT)

/* ---- Control flags (backend-independent) ---- */
#define FPGA_HAL_CTRL_PPB_START       BIT(0)
#define FPGA_HAL_CTRL_RESET_WALLCLOCK BIT(1)
#define FPGA_HAL_CTRL_RESET_PTP       BIT(2)
#define FPGA_HAL_CTRL_RESET_ETHERNET  BIT(3)

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
 * Returns the underlying driver device depending on which backend
 * is active.
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
 * On LiteX: always returns true (SoC contains the FPGA logic).
 * On SPI: true once the spibone bus answers.
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
 * The leader clock identity is selected by the on-FPGA BMA and is
 * read back via fpga_hal_read_ptp_leader_id().
 *
 * @param time_source            PTP time source (IEEE 1588 enum).
 * @param log_msg_interval       logMessageInterval for Sync (signed).
 * @param log_announce_interval  logAnnounceInterval (signed).
 */
int fpga_hal_write_ptp_config(uint8_t time_source,
			      int8_t log_msg_interval,
			      int8_t log_announce_interval);

/**
 * @brief Read the PTP leader clock identity selected by the FPGA BMA.
 *
 * @param leader_clock_id  Output: 8-byte clock identity (big-endian).
 * @return true if a leader is selected (non-zero ID), false otherwise.
 */
bool fpga_hal_read_ptp_leader_id(uint8_t leader_clock_id[8]);

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
 * Raw CSR access (bring-up / debug)
 * ======================================================================== */

/**
 * @brief Read one CSR word at an absolute byte address.
 *
 * Addresses come from the generated map (litex_soc/build/aes67_bridge/
 * csr.csv) and are identical on the LiteX and SPI backends.
 * Debug/bring-up tool ("fpga peek") — production code resolves
 * registers by name through the generated headers instead.
 *
 * @return 0 on success, negative errno.
 */
int fpga_hal_csr_read(uint32_t addr, uint32_t *val);

/**
 * @brief Write one CSR word at an absolute byte address ("fpga poke").
 *
 * @return 0 on success, negative errno.
 */
int fpga_hal_csr_write(uint32_t addr, uint32_t val);

/* ========================================================================
 * Static FPGA build configuration (system_cfg CSRs)
 * ======================================================================== */

/**
 * Static configuration the gateware was built with (the syscfg generic,
 * see FPGA/packages/system_cfg_pkg.vhd). Constant per bitstream; exposed
 * read-only through the aes67_bridge system_cfg_{flags,rx,tx} CSRs.
 *
 * The firmware reads this once at boot (after the FPGA is up) and starts
 * the matching services — most importantly hardware vs. software PTP —
 * instead of baking the choice in at compile time.
 */
struct fpga_hal_system_cfg {
	/* flags */
	bool ptp_in_software;   /* host PTP stack disciplines the wallclock;
				 * RX frames carry the timestamp trailer */
	bool static_ptp_config; /* servo tuning fixed via VHDL generic */
	bool metering;          /* audio metering CSRs are live */
	/* RX / DA path */
	uint8_t  rx_max_streams;
	uint8_t  rx_channels;
	uint16_t rx_buffer_depth;
	/* TX / AD path */
	uint8_t  tx_max_streams;
	uint8_t  tx_channels;
	uint16_t tx_buffer_depth;
	/* eth_buf packet-buffer layout: payload bytes per 32-bit Wishbone
	 * word (4 on packed gateware; 1 on legacy builds, whose eth_buf CSR
	 * block has no bytes_per_word register). */
	uint8_t  eth_buf_bytes_per_word;
};

/**
 * @brief Read the system_cfg CSRs from the FPGA (uncached).
 *
 * @return 0 on success, negative errno.
 */
int fpga_hal_read_system_cfg(struct fpga_hal_system_cfg *cfg);

/**
 * @brief Read the system_cfg CSRs and latch them into the process-wide
 *        cache consulted by fpga_hal_syscfg() / fpga_hal_ptp_in_software().
 *
 * Call once at boot after the FPGA answers on the bus (and again from the
 * FPGA-recovery path — the value is per bitstream). Until the first
 * successful load the cache reads as all-zero (hardware PTP, no metering).
 *
 * @return 0 on success, negative errno on bus error.
 */
int fpga_hal_syscfg_load(void);

/** @brief Cached static configuration (all-zero before the first load). */
const struct fpga_hal_system_cfg *fpga_hal_syscfg(void);

/** @brief True once fpga_hal_syscfg_load() has succeeded. */
bool fpga_hal_syscfg_valid(void);

/** @brief Convenience: cached ptp_in_software flag (false before load). */
bool fpga_hal_ptp_in_software(void);

/**
 * @brief Convenience: cached eth_buf payload bytes per 32-bit word.
 *
 * 1 (legacy byte packing) before the first syscfg load and on gateware
 * without the eth_buf bytes_per_word CSR; 4 on packed gateware.
 */
uint8_t fpga_hal_eth_buf_bytes_per_word(void);

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

/* ========================================================================
 * PTP servo / parser tuning + monitoring
 * ======================================================================== */

struct fpga_hal_ptp_tuning {
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
	int32_t  delay_asymmetry_ns;  /* IEEE 1588 delayAsymmetry, signed ns;
	                                  positive = M2S path longer than S2M */
};

struct fpga_hal_ptp_monitor {
	int32_t  filtered_offset;
	int32_t  integral_sum;
	int32_t  pi_proportional;
	int32_t  pi_sum_raw;
	uint8_t  effective_gain_shift;
	uint16_t lock_counter;
	uint16_t sample_count;
	bool     first_lock_achieved;
};

int  fpga_hal_write_ptp_tuning(const struct fpga_hal_ptp_tuning *t);
void fpga_hal_read_ptp_tuning(struct fpga_hal_ptp_tuning *t);
void fpga_hal_read_ptp_monitor(struct fpga_hal_ptp_monitor *m);
int  fpga_hal_set_ptp_reset(bool held_in_reset);

/* Reset domains of the unified "reset" CSR. The FPGA powers up with every
 * domain held (CSR resets to all-ones), so the firmware must release them in a
 * staged order at boot (and after an FPGA-reset recovery). */
#define FPGA_HAL_RESET_PTP  BIT(0)
#define FPGA_HAL_RESET_TX   BIT(1)
#define FPGA_HAL_RESET_RX   BIT(2)
#define FPGA_HAL_RESET_ETH  BIT(3)
#define FPGA_HAL_RESET_AUDIO (FPGA_HAL_RESET_TX | FPGA_HAL_RESET_RX)
#define FPGA_HAL_RESET_ALL \
	(FPGA_HAL_RESET_PTP | FPGA_HAL_RESET_AUDIO | FPGA_HAL_RESET_ETH)

/** Release (held=false) or assert (held=true) the given reset domains. */
int  fpga_hal_set_resets(uint32_t domains, bool held);

/* ========================================================================
 * Audio metering
 * ======================================================================== */

/**
 * @brief Read audio metering registers and clear sticky bits.
 *
 * Reads all four metering registers (RX/TX signal/clip), then toggles
 * the clear bit so the FPGA resets its sticky detectors.
 *
 * @param rx_signal  Output: RX signal detect bitmask (1 bit per channel).
 * @param rx_clip    Output: RX clip detect bitmask (1 bit per channel).
 * @param tx_signal  Output: TX signal detect bitmask (1 bit per channel).
 * @param tx_clip    Output: TX clip detect bitmask (1 bit per channel).
 */
void fpga_hal_read_metering(uint16_t *rx_signal, uint16_t *rx_clip,
			    uint16_t *tx_signal, uint16_t *tx_clip);

#if defined(CONFIG_FPGA_HAL_SPI) && defined(CONFIG_AES67_SPIBONE_PROBE)
/**
 * @brief Verify the FPGA answers over the spibone Wishbone bus.
 *
 * Reads the constant ctrl_scratch register and round-trips test patterns
 * through the AES67 scratch register. Call during boot bring-up after the
 * FPGA has been configured and before relying on the bus.
 *
 * @return 0 if the bus round-trips cleanly, -EIO on any fault.
 */
int fpga_hal_spibone_probe(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* FPGA_HAL_H_ */
