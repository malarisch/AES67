/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: Apache-2.0
 *
 * PTPv2 Best Master Clock (BMC) algorithm for AES67.
 *
 * Listens for PTP Announce messages on the standard PTP multicast
 * address (224.0.1.129, port 320), maintains an announce table of
 * foreign masters, and runs the IEEE 1588 data-set comparison
 * algorithm to elect the best master.
 *
 * Once a decision is made the result is written to the FPGA through
 * the FMC register interface:
 *   - Register 0x55: leader clock identity, time source, logMsgInterval
 *   - Register 0x50 bit[4]: PTP Is Leader flag
 */

#ifndef PTP_BMC_H_
#define PTP_BMC_H_

#include <zephyr/kernel.h>
#include <zephyr/net/net_if.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- IEEE 1588 constants ---- */

/* PTP message types (lower nibble of byte 0) */
#define PTP_MSG_SYNC          0x0
#define PTP_MSG_DELAY_REQ     0x1
#define PTP_MSG_FOLLOW_UP     0x8
#define PTP_MSG_DELAY_RESP    0x9
#define PTP_MSG_ANNOUNCE      0xB

/* PTP Ethernet / UDP parameters */
#define PTP_MULTICAST_ADDR    "224.0.1.129"
#define PTP_GENERAL_PORT      320
#define PTP_EVENT_PORT        319

/* PTP time source enumerations (IEEE 1588 Table 7) */
#define PTP_TIME_SRC_ATOMIC_CLOCK     0x10
#define PTP_TIME_SRC_GPS              0x20
#define PTP_TIME_SRC_TERRESTRIAL      0x30
#define PTP_TIME_SRC_PTP              0x40
#define PTP_TIME_SRC_NTP              0x50
#define PTP_TIME_SRC_HAND_SET         0x60
#define PTP_TIME_SRC_OTHER            0x90
#define PTP_TIME_SRC_INTERNAL_OSC     0xA0

/* AES67 mandates logMessageInterval = -3 (125 ms) for Sync,
 * and typically 0 (1 s) for Announce */
#define AES67_LOG_MSG_INTERVAL_SYNC     (-3)
#define AES67_LOG_MSG_INTERVAL_ANNOUNCE  0

/* Clock class values */
#define PTP_CLOCK_CLASS_PRIMARY           6
#define PTP_CLOCK_CLASS_APP_SPECIFIC      13
#define PTP_CLOCK_CLASS_DEFAULT           248
#define PTP_CLOCK_CLASS_FOLLOWER_ONLY     255

/* Clock accuracy */
#define PTP_CLOCK_ACCURACY_UNKNOWN        0xFE

/* Maximum number of foreign masters tracked */
#define PTP_MAX_FOREIGN_MASTERS  8

/* Announce receipt timeout: number of announce intervals before
 * a foreign master is considered lost (IEEE 1588 default: 3) */
#define PTP_ANNOUNCE_RECEIPT_TIMEOUT 3

/* ---- Announce message dataset (from received Announce) ---- */

struct ptp_announce_dataset {
	/* Fields from Announce message body */
	uint8_t  gm_priority1;
	uint8_t  gm_clock_class;
	uint8_t  gm_clock_accuracy;
	uint16_t gm_offset_scaled_log_variance;
	uint8_t  gm_priority2;
	uint8_t  gm_identity[8];        /* grandmasterIdentity (EUI-64) */
	uint16_t steps_removed;
	uint8_t  time_source;

	/* Source port identity from common header */
	uint8_t  sender_clock_id[8];
	uint16_t sender_port_number;

	/* Timing */
	int64_t  last_received_uptime_ms;  /* k_uptime_get() when last received */
	uint16_t announce_count;           /* Announce messages received from this master */
};

/* ---- BMC state ---- */

enum ptp_bmc_role {
	PTP_ROLE_LISTENING,  /* No decision yet */
	PTP_ROLE_FOLLOWER,   /* Following another master */
	PTP_ROLE_LEADER,     /* We are the best master */
};

/* ---- Public API ---- */

/**
 * @brief Initialise and start the PTP BMC subsystem.
 *
 * Spawns a background thread that listens for PTP Announce messages
 * and periodically runs the BMC algorithm.
 *
 * @param iface  Network interface to listen on
 * @return 0 on success, negative errno on error
 */
int ptp_bmc_start(struct net_if *iface);

/**
 * @brief Callback type invoked whenever the BMC decision changes
 *        (role change, leader change, etc.).
 *
 * @param new_role  The new BMC role after the change
 */
typedef void (*ptp_bmc_change_cb_t)(enum ptp_bmc_role new_role);

/**
 * @brief Register a callback that fires on every BMC state change.
 *
 * Only one callback is supported.  Calling again replaces the
 * previous one.
 *
 * @param cb  Callback function, or NULL to unregister
 */
void ptp_bmc_register_change_cb(ptp_bmc_change_cb_t cb);

/**
 * @brief Notify the BMC that a valid IP address has been assigned
 *        and written to the FPGA.  The BMC thread blocks until this
 *        is called at least once.
 */
void ptp_bmc_notify_ip_ready(void);

/**
 * @brief Notify the BMC that the FPGA has recovered (after reset/reprogram).
 *        Re-sends leader configuration to FPGA if we are currently leader.
 */
void ptp_bmc_notify_fpga_ready(void);

/**
 * @brief Notify the BMC that Ethernet link has come up.
 *        Re-joins the PTP multicast group to re-establish IGMP membership.
 */
void ptp_bmc_notify_link_up(void);

/**
 * @brief Get the current BMC role.
 */
enum ptp_bmc_role ptp_bmc_get_role(void);

/**
 * @brief Get the current best master's clock identity.
 *
 * @param out  Buffer to receive 8-byte clock identity
 * @return 0 if a best master is known, -ENOENT otherwise
 */
int ptp_bmc_get_best_master_id(uint8_t out[8]);

/**
 * @brief Get the foreign master table.
 *
 * @param count  Output: number of valid entries
 * @return Pointer to the foreign master table (PTP_MAX_FOREIGN_MASTERS entries)
 */
const struct ptp_announce_dataset *ptp_bmc_get_foreign_masters(int *count);

/**
 * @brief Get our own clock identity.
 *
 * @param out  Buffer to receive 8-byte clock identity
 */
void ptp_bmc_get_clock_identity(uint8_t out[8]);

/**
 * @brief Update the BMC's own dataset from the runtime config.
 *
 * Call after changing ptp_priority1, ptp_priority2, ptp_clock_class,
 * or ptp_clock_accuracy in aes67_device_config.  Re-runs BMC.
 */
void ptp_bmc_update_own_dataset(void);

/**
 * @brief Get the BMC's own announce dataset (what we advertise as leader).
 */
const struct ptp_announce_dataset *ptp_bmc_get_own_dataset(void);

#ifdef __cplusplus
}
#endif

#endif /* PTP_BMC_H_ */
