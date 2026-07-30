/*
 * PTP control/monitoring HAL — runtime dispatcher, see ptp_ctrl.h.
 *
 * Both backends are always built (the software one when the board can run
 * it at all, CONFIG_AES67_PTP_SOFTWARE); which one is *active* is decided
 * at boot from the FPGA's static build configuration (system_cfg CSRs):
 * gateware built with PTP_IN_SOFTWARE has no hardware BMC/servo, so the
 * Zephyr stack owns the clock — and vice versa. Before the system_cfg has
 * been loaded (or on gateware without the register) the hardware backend
 * answers, matching the FPGA power-up default.
 */

#include <zephyr/kernel.h>

#include "ptp_ctrl_internal.h"
#include "../drivers/fpga_hal/fpga_hal.h"

/* Only referenced from the CONFIG_AES67_PTP_SOFTWARE branches below —
 * builds without the software-PTP stack (e.g. the QEMU/mock target)
 * would otherwise trip -Werror=unused-function under twister. */
static __maybe_unused bool ptp_sw_active(void)
{
	return IS_ENABLED(CONFIG_AES67_PTP_SOFTWARE) &&
	       fpga_hal_ptp_in_software();
}

void ptp_ctrl_get_status(struct ptp_ctrl_status *st)
{
#ifdef CONFIG_AES67_PTP_SOFTWARE
	if (ptp_sw_active()) {
		ptp_ctrl_sw_get_status(st);
		return;
	}
#endif
	ptp_ctrl_hw_get_status(st);
}

bool ptp_ctrl_wallclock_locked(void)
{
#ifdef CONFIG_AES67_PTP_SOFTWARE
	if (ptp_sw_active()) {
		return ptp_ctrl_sw_wallclock_locked();
	}
#endif
	return ptp_ctrl_hw_wallclock_locked();
}

int ptp_ctrl_get_foreign_masters(struct ptp_ctrl_foreign *out, int max)
{
#ifdef CONFIG_AES67_PTP_SOFTWARE
	if (ptp_sw_active()) {
		return ptp_ctrl_sw_get_foreign_masters(out, max);
	}
#endif
	return ptp_ctrl_hw_get_foreign_masters(out, max);
}

void ptp_ctrl_apply_config(void)
{
#ifdef CONFIG_AES67_PTP_SOFTWARE
	if (ptp_sw_active()) {
		ptp_ctrl_sw_apply_config();
		return;
	}
#endif
	ptp_ctrl_hw_apply_config();
}
