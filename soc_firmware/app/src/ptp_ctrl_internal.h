/*
 * PTP control/monitoring HAL — backend interface.
 *
 * Both backends are always compiled (hardware unconditionally, software
 * when CONFIG_AES67_PTP_SOFTWARE support is built); ptp_ctrl.c dispatches
 * between them at runtime based on the FPGA's static build configuration
 * (system_cfg CSRs → fpga_hal_ptp_in_software()).
 *
 * Split into two translation units on purpose: the software backend must
 * include the Zephyr PTP stack's internal headers, whose enum ptp_time_src
 * members collide with ptp_bmc.h's PTP_TIME_SRC_* macros used by the
 * hardware backend.
 */

#ifndef PTP_CTRL_INTERNAL_H_
#define PTP_CTRL_INTERNAL_H_

#include "ptp_ctrl.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Hardware backend — FPGA BMC + servo via ptp_bmc.c / fpga_hal */
void ptp_ctrl_hw_get_status(struct ptp_ctrl_status *st);
bool ptp_ctrl_hw_wallclock_locked(void);
int  ptp_ctrl_hw_get_foreign_masters(struct ptp_ctrl_foreign *out, int max);
void ptp_ctrl_hw_apply_config(void);

#ifdef CONFIG_AES67_PTP_SOFTWARE
/* Software backend — Zephyr IEEE 1588 stack (CONFIG_PTP) */
void ptp_ctrl_sw_get_status(struct ptp_ctrl_status *st);
bool ptp_ctrl_sw_wallclock_locked(void);
int  ptp_ctrl_sw_get_foreign_masters(struct ptp_ctrl_foreign *out, int max);
void ptp_ctrl_sw_apply_config(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* PTP_CTRL_INTERNAL_H_ */
