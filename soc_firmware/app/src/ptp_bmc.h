/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: Apache-2.0
 *
 * PTPv2 BMC SoC shim.
 *
 * The IEEE 1588 BMC runs inside the FPGA. This module pushes
 * configuration (priorities, clock class/accuracy, log intervals,
 * time source) into the FPGA CSRs and exposes the FPGA-selected
 * role / leader identity to the rest of the firmware.
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

/* ---- IEEE 1588 enums / AES67 defaults ---- */

#define PTP_TIME_SRC_ATOMIC_CLOCK     0x10
#define PTP_TIME_SRC_GPS              0x20
#define PTP_TIME_SRC_TERRESTRIAL      0x30
#define PTP_TIME_SRC_PTP              0x40
#define PTP_TIME_SRC_NTP              0x50
#define PTP_TIME_SRC_HAND_SET         0x60
#define PTP_TIME_SRC_OTHER            0x90
#define PTP_TIME_SRC_INTERNAL_OSC     0xA0

#define AES67_LOG_MSG_INTERVAL_SYNC     (-3)
#define AES67_LOG_MSG_INTERVAL_ANNOUNCE  0

#define PTP_CLOCK_CLASS_PRIMARY           6
#define PTP_CLOCK_CLASS_APP_SPECIFIC      13
#define PTP_CLOCK_CLASS_DEFAULT           248
#define PTP_CLOCK_CLASS_FOLLOWER_ONLY     255

#define PTP_CLOCK_ACCURACY_UNKNOWN        0xFE

/* ---- Announce dataset shape (used by webserver for status JSON) ---- */
struct ptp_announce_dataset {
	uint8_t  gm_priority1;
	uint8_t  gm_clock_class;
	uint8_t  gm_clock_accuracy;
	uint16_t gm_offset_scaled_log_variance;
	uint8_t  gm_priority2;
	uint8_t  gm_identity[8];
	uint16_t steps_removed;
	uint8_t  time_source;
	uint8_t  sender_clock_id[8];
	uint16_t sender_port_number;
	int64_t  last_received_uptime_ms;
	uint16_t announce_count;
};

enum ptp_bmc_role {
	PTP_ROLE_LISTENING,
	PTP_ROLE_FOLLOWER,
	PTP_ROLE_LEADER,
};

/* ---- Public API ---- */

int  ptp_bmc_start(struct net_if *iface);

typedef void (*ptp_bmc_change_cb_t)(enum ptp_bmc_role new_role);
void ptp_bmc_register_change_cb(ptp_bmc_change_cb_t cb);

void ptp_bmc_notify_ip_ready(void);
void ptp_bmc_notify_fpga_ready(void);
void ptp_bmc_notify_link_up(void);

enum ptp_bmc_role ptp_bmc_get_role(void);
int  ptp_bmc_get_best_master_id(uint8_t out[8]);

/** FPGA BMA does not expose a foreign-master list; always returns NULL/0. */
const struct ptp_announce_dataset *ptp_bmc_get_foreign_masters(int *count);

void ptp_bmc_get_clock_identity(uint8_t out[8]);
void ptp_bmc_update_own_dataset(void);

const struct ptp_announce_dataset *ptp_bmc_get_own_dataset(void);

#ifdef __cplusplus
}
#endif

#endif /* PTP_BMC_H_ */
