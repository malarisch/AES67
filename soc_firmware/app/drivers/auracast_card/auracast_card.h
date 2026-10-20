/*
 * Auracast Card Driver - ESP32-S31 Bluetooth LE Audio broadcast transmitter
 *
 * The card is an I2C slave (CONFIG_AURACAST_CARD_I2C_ADDR, default 0x3A)
 * speaking the register protocol in auracast_i2c_regs.h. It boots into
 * WAIT_CONFIG and broadcasts nothing until the host writes a
 * configuration and START. Changing the configuration means RESET and a
 * fresh START; the configuration lives in the S31's RAM only.
 *
 * Life cycle on the host side:
 *   init      - identity read (MAGIC/PROTO/FW), status poll started,
 *               nothing is sent yet
 *   activate  - called by the card manager on the first PTP lock: from
 *               now on the stored configuration is applied. A broadcast
 *               the S31 is already running with the same configuration
 *               is adopted, not restarted.
 *   poll      - every second: STATE/ERROR and, from STARTING on, the
 *               status block into a cache the REST API reads. An
 *               unexpected WAIT_CONFIG (S31 restarted) triggers a new
 *               configuration cycle with backoff.
 *
 * Bus rules (see the protocol header): reads always go through
 * i2c_write_read() with the register address first and take EXACTLY
 * AURA_READ_MAX (31) bytes, the amount the S31 queues: the ESP32 slave
 * pre-loads the next FIFO byte into its shift register when a read stops
 * early, and that byte then prefixes every later read (the FIFO reset
 * on the S31 cannot remove it). NAME and CODE are always written in
 * full. The S31 stretches SCL at the start of a read; the ESP32-S3 I2C
 * master tolerates this (i2c_esp32.c programs the hardware-maximum SCL
 * timeout when the DTS has no scl-timeout-us). Only the ESP32-S3 host
 * is a runtime target for this card; the LiteX targets only build it.
 */

#ifndef AURACAST_CARD_H
#define AURACAST_CARD_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <stdbool.h>
#include <stdint.h>

#include "auracast_card_proto.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Cached view of the S31, refreshed by the driver's poll. */
struct auracast_status {
	bool     present;          /* the S31 answered the last poll */
	bool     stats_valid;      /* state >= STARTING: status block meaningful */
	uint8_t  fw_major;
	uint8_t  fw_minor;
	uint8_t  fw_patch;
	uint8_t  state;            /* AURA_STATE_* */
	uint8_t  error;            /* AURA_ERR_* of the last rejection */
	uint8_t  error_reg;        /* register the error refers to */
	uint8_t  reset_reason;     /* esp_reset_reason_t of the S31 */
	struct auracast_status_block stats;
	int      last_errno;       /* result of the last configuration cycle */
	uint32_t age_ms;           /* time since the last successful poll */
};

/** Bytes of the identity block: MAGIC(4) PROTO(1) FW(3). */
#define AURA_IDENT_LEN 8

/**
 * @brief Read the identity block from an S31 at the card address.
 *
 * Usable before init() (card manager detection). Recovers once from the
 * one-stale-byte condition (see the file comment) by draining the slave.
 *
 * @param dev    I2C bus.
 * @param ident  Receives MAGIC, PROTO_VERSION, FW_VERSION; on -ENODEV the
 *               raw bytes read, for logging.
 * @return 0 if MAGIC matches, -ENODEV otherwise or on a bus error.
 */
int auracast_card_probe(const struct device *dev, uint8_t ident[AURA_IDENT_LEN]);

/**
 * @brief Initialise the driver for a detected card.
 *
 * Reads the identity registers and starts the status poll. Does not
 * start a broadcast. Idempotent: a rescan calls it again; the stored
 * configuration is kept.
 *
 * @return 0, -ENODEV if the S31 does not answer, -ENOTSUP on a foreign
 *         protocol version.
 */
int auracast_card_init(const struct device *i2c_dev);

/** @brief True once init() succeeded. */
bool auracast_card_ready(void);

/**
 * @brief Allow broadcasting (first PTP/wallclock lock).
 *
 * Schedules the configuration cycle on the driver's work item; the
 * caller does not block. Remembered across rescans.
 */
int auracast_card_activate(void);

/** @brief Copy of the stored configuration. */
int auracast_card_get_config(struct auracast_config *out);

/**
 * @brief Replace the stored configuration and apply it.
 *
 * Validates locally (-EINVAL, error/error_reg in the status cache),
 * stores, and if activate() has happened runs the configuration cycle
 * synchronously: RESET if needed, write, START (or RESET only when
 * cfg->enabled is false). A broadcast already running with exactly this
 * configuration is kept (no restart). Before activate() the
 * configuration is only stored and 0 is returned.
 *
 * @return 0, -EINVAL (local validation or S31 CONFIG_ERROR), -EIO (S31
 *         FAULT), -ENODEV (no answer), -ETIMEDOUT (no answer after RESET),
 *         -ENOTSUP (S31 autostarts after RESET: built with
 *         AURACAST_AUTOSTART, not configurable over I2C).
 */
int auracast_card_set_config(const struct auracast_config *cfg);

/** @brief enabled = true and run the cycle. */
int auracast_card_start(void);

/** @brief enabled = false and RESET the S31 (stops the broadcast). */
int auracast_card_stop(void);

/** @brief Force RESET + START with the unchanged configuration. */
int auracast_card_restart(void);

/** @brief Copy of the status cache. No I2C traffic. */
int auracast_card_get_status(struct auracast_status *out);

#ifdef __cplusplus
}
#endif

#endif /* AURACAST_CARD_H */
