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
#endif

#endif /* FPGA_JTAG_H_ */
