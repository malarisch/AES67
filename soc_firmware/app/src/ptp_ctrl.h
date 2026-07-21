/*

 * PTP control/monitoring HAL — one API for both PTP modes.
 *
 * The endpoint runs PTP either in the FPGA (hardware BMC + servo, monitored
 * and configured through ptp_bmc.c / fpga_hal) or in software (Zephyr's
 * IEEE 1588 stack, CONFIG_AES67_PTP_SOFTWARE, disciplining the FPGA wallclock
 * through the aes67 PHC). The web UI and REST API must not care which one is
 * active: this module maps status reads and configuration writes onto
 * whichever stack is running.
 *
 * Backend selection is a compile-time decision (CONFIG_AES67_PTP_SOFTWARE),
 * matching the rest of the firmware.
 */

#ifndef PTP_CTRL_H_
#define PTP_CTRL_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Deliberately not ptp_bmc.h's enum: the software backend must include the
 * Zephyr PTP stack's internal headers, whose enum ptp_time_src members
 * collide with ptp_bmc.h's PTP_TIME_SRC_* macros. */
enum ptp_ctrl_role {
	PTP_CTRL_ROLE_LISTENING,
	PTP_CTRL_ROLE_FOLLOWER,
	PTP_CTRL_ROLE_LEADER,
};

struct ptp_ctrl_status {
	const char *mode;              /* "hardware" | "software" */
	enum ptp_ctrl_role role;
	uint8_t  clock_id[8];          /* our clock identity */

	bool     gm_valid;             /* a (foreign) grandmaster is selected */
	uint8_t  gm_id[8];
	uint8_t  gm_priority1;         /* GM announce quality — software mode */
	uint8_t  gm_priority2;         /* only (the FPGA BMA does not expose */
	uint8_t  gm_clock_class;       /* the winner's announce fields) */
	uint8_t  gm_clock_accuracy;
	uint16_t steps_removed;

	int32_t  offset_ns;            /* offset from grandmaster */
	int32_t  path_delay_ns;        /* mean path delay */
	bool     locked;               /* HW: wallclock servo lock;
					* SW: |offset| below lock threshold */
};

struct ptp_ctrl_foreign {
	uint8_t  sender_id[8];
	uint16_t sender_port;
	uint8_t  gm_id[8];
	uint8_t  priority1;
	uint8_t  priority2;
	uint8_t  clock_class;
	uint8_t  clock_accuracy;
	uint16_t steps_removed;
	uint16_t announce_count;
};

/** Snapshot the current PTP state from the active stack. */
void ptp_ctrl_get_status(struct ptp_ctrl_status *st);

/** Mode-invariant "wallclock is usable" flag, mirroring the gateware's
 *  wallclock_locked (= is_leader OR servo_locked). In hardware mode this
 *  is the FPGA status bit; in software mode the SW-PTP gateware never
 *  builds the HW servo (the bit is stuck at 0), so it is derived from
 *  the Zephyr stack: leader role, or follower with the offset inside the
 *  lock window. Drives the display state and the audio output enable. */
bool ptp_ctrl_wallclock_locked(void);

/** Fill up to @p max foreign-master records; returns the count.
 *  Hardware mode returns 0 (the FPGA BMA keeps no exposed list). */
int ptp_ctrl_get_foreign_masters(struct ptp_ctrl_foreign *out, int max);

/** Push the PTP fields of the stored device config (priority1/2, clock
 *  class/accuracy, domain, log intervals) into the active PTP stack.
 *  Call at boot and after every configuration change. */
void ptp_ctrl_apply_config(void);

#ifdef __cplusplus
}
#endif

#endif /* PTP_CTRL_H_ */
