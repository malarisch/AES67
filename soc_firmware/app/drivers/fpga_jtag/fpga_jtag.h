/*
 * Cyclone FPGA JTAG (IEEE 1149.1) bit-bang, over four GPIOs.
 *
 * Scans the chain (reads IDCODEs) to prove the wiring and identify the
 * device, and loads a bitstream into the FPGA's SRAM over JTAG. The
 * bring-up is orchestrated explicitly from main() (fpga_jtag_boot_load)
 * so it can run after the display is up and report upload progress.
 * See dts/bindings/aes67,cyclone-jtag.yaml.
 */

#ifndef FPGA_JTAG_H_
#define FPGA_JTAG_H_

#include <stdint.h>
#include <stddef.h>

/* Maximum devices reported from a single chain scan. */
#define FPGA_JTAG_MAX_DEVICES 8

/**
 * @brief Bitstream-upload progress callback.
 *
 * Invoked during fpga_jtag_boot_load() as the configuration data is
 * shifted in, with @p percent running 0..100. Kept lightweight (no
 * blocking) — the JTAG clock is stalled while it runs.
 *
 * @param percent Fraction of the bitstream shifted so far (0..100).
 * @param ctx     Opaque pointer passed through from fpga_jtag_boot_load().
 */
typedef void (*fpga_jtag_progress_cb)(uint8_t percent, void *ctx);

/**
 * @brief Scan the JTAG chain and return the IDCODEs found.
 *
 * Resets the TAP, then shifts DR reading one 32-bit IDCODE per device
 * until the end-of-chain marker (all-ones = TDI fed back) is seen.
 *
 * @param idcodes Output array, at least @p max entries.
 * @param max     Capacity of @p idcodes.
 *
 * @return Number of devices found (>= 0), or negative errno on a wiring
 *         fault (TDO stuck high or low).
 */
int fpga_jtag_scan_chain(uint32_t *idcodes, size_t max);

/**
 * @brief Human-readable name for an Altera IDCODE, or NULL if unknown.
 */
const char *fpga_jtag_idcode_name(uint32_t idcode);

/**
 * @brief Read the FPGA's 32-bit JTAG USERCODE register.
 *
 * After a good configuration this reads the bitstream's checksum (Quartus
 * Auto-Usercode); an unconfigured device reads its default (typically
 * all-ones). Used to decide whether configuration is needed and to verify
 * it afterwards.
 *
 * @return The 32-bit USERCODE.
 */
uint32_t fpga_jtag_read_usercode(void);

#ifdef CONFIG_AES67_FPGA_JTAG_BOOT_LOAD
/**
 * @brief Scan, check, and (if needed) load the embedded bitstream over JTAG.
 *
 * Scans the chain and verifies a single known device, then reads the
 * USERCODE: if it already matches the embedded bitstream the upload is
 * skipped. Otherwise the bitstream is shifted in (with @p cb reporting
 * 0..100 %), retried until the USERCODE reads back correct or the retry
 * budget (CONFIG_AES67_FPGA_JTAG_CONFIG_RETRIES) is exhausted.
 *
 * @param cb   Optional progress callback (may be NULL).
 * @param ctx  Opaque pointer passed to @p cb.
 *
 * @return 0 if configured now, 1 if already configured (upload skipped),
 *         or a negative errno (scan/verify/upload failure).
 */
int fpga_jtag_boot_load(fpga_jtag_progress_cb cb, void *ctx);

/**
 * @brief Force-reload the embedded bitstream (no USERCODE skip).
 *
 * Recovery path: reconfigures the device unconditionally with the same
 * retried, USERCODE-verified sequence the boot load uses. All FPGA state
 * (CSRs, resets, stream tables) is lost.
 *
 * @return 0 on verified success, negative errno otherwise.
 */
int fpga_jtag_reload(fpga_jtag_progress_cb cb, void *ctx);
#endif

/**
 * @brief Result of one JTAG health check, worst condition wins.
 */
enum fpga_jtag_health {
	FPGA_JTAG_HEALTH_OK = 0,
	/** Chain unreadable (no/unknown IDCODE): wiring or FPGA power, NOT
	 *  proof of a configuration fault — nothing can be assessed. */
	FPGA_JTAG_HEALTH_LINK_DOWN,
	/** IDCODE reads fine but the USERCODE is not the embedded
	 *  bitstream's: the configuration is gone (or foreign). */
	FPGA_JTAG_HEALTH_UNCONFIGURED,
	/** The device's background CRC check found corrupted configuration
	 *  SRAM (SEU): the CRC_ERROR pin is latched high. */
	FPGA_JTAG_HEALTH_CRC_ERROR,
};

/**
 * @brief Check "how the FPGA is doing" over JTAG. Non-invasive.
 *
 * IDCODE proves the chain, USERCODE (when an embedded bitstream exists)
 * proves the configuration, and a SAMPLE boundary scan reads the
 * dual-purpose CRC_ERROR pin (requires CRC_ERROR_CHECKING ON in the
 * Quartus .qsf). Safe to run in user mode while the device operates.
 */
enum fpga_jtag_health fpga_jtag_health_check(void);

/**
 * @brief Human-readable name for a health state.
 */
const char *fpga_jtag_health_str(enum fpga_jtag_health health);

/**
 * @brief Raw read of the CRC_ERROR pin via a SAMPLE boundary scan.
 *
 * @return true if the pin samples high (error latched — or, on a board
 *         where the readout is unusable, permanently). Diagnostic aid;
 *         the monitor decides on its own whether to trust this signal.
 */
bool fpga_jtag_crc_error(void);

/**
 * @brief SAMPLE the full boundary-scan register (pin snapshot).
 *
 * Diagnostic aid for validating the CRC_ERROR cell index against known
 * pin states. Non-invasive.
 *
 * @param buf     Output, LSB-first (bit i = boundary cell i).
 * @param buf_len Capacity in bytes; must fit the register.
 *
 * @return Number of valid bits in @p buf, or negative errno.
 */
int fpga_jtag_sample_boundary(uint8_t *buf, size_t buf_len);

#ifdef CONFIG_AES67_FPGA_JTAG_MONITOR
/**
 * @brief Events reported by the health-monitor thread.
 */
enum fpga_jtag_event {
	/** Confirmed fault. Fired BEFORE the bitstream reload starts —
	 *  mute the analog outputs here. */
	FPGA_JTAG_EVT_FAULT,
	/** Bitstream reloaded and USERCODE-verified. The FPGA is blank
	 *  (resets held, no MAC/streams/PTP state) — the handler is
	 *  expected to reboot for a clean bring-up. */
	FPGA_JTAG_EVT_RECOVERED,
	/** Reload attempts exhausted; the monitor retries after a
	 *  hold-off. Outputs should stay muted. */
	FPGA_JTAG_EVT_RELOAD_FAILED,
};

/**
 * @brief Monitor event callback. Runs in the monitor thread.
 *
 * @param evt     What happened.
 * @param health  The health state that triggered the recovery.
 * @param ctx     Opaque pointer from fpga_jtag_monitor_start().
 */
typedef void (*fpga_jtag_event_cb)(enum fpga_jtag_event evt,
				   enum fpga_jtag_health health, void *ctx);

/**
 * @brief Start the periodic FPGA health monitor.
 *
 * Checks every CONFIG_AES67_FPGA_JTAG_MONITOR_INTERVAL_MS. A fault is
 * confirmed by a second check before recovery (both fault states are
 * persistent, so a genuine fault reads identically twice while a single
 * corrupted shift on marginal wiring does not). Recovery: @p evt_cb
 * (FAULT) → fpga_jtag_reload() with @p progress_cb → @p evt_cb
 * (RECOVERED / RELOAD_FAILED).
 *
 * @return 0 on success, -EALREADY if already started.
 */
int fpga_jtag_monitor_start(fpga_jtag_event_cb evt_cb,
			    fpga_jtag_progress_cb progress_cb, void *ctx);
#endif

#endif /* FPGA_JTAG_H_ */
