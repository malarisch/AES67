/*
 * PTP control/monitoring HAL — hardware backend.
 *
 * FPGA BMC + servo via ptp_bmc.c / fpga_hal. Always compiled; dispatched
 * to when the gateware reports hardware PTP (see ptp_ctrl.c).
 */

#include <zephyr/kernel.h>
#include <string.h>

#include "ptp_ctrl_internal.h"
#include "ptp_bmc.h"
#include "../drivers/fpga_hal/fpga_hal.h"

void ptp_ctrl_hw_get_status(struct ptp_ctrl_status *st)
{
	memset(st, 0, sizeof(*st));
	st->mode = "hardware";

	switch (ptp_bmc_get_role()) {
	case PTP_ROLE_LEADER:
		st->role = PTP_CTRL_ROLE_LEADER;
		break;
	case PTP_ROLE_FOLLOWER:
		st->role = PTP_CTRL_ROLE_FOLLOWER;
		break;
	default:
		st->role = PTP_CTRL_ROLE_LISTENING;
		break;
	}

	ptp_bmc_get_clock_identity(st->clock_id);
	st->gm_valid = (ptp_bmc_get_best_master_id(st->gm_id) == 0);
	/* The FPGA BMA exposes only the winner's identity, not its announce
	 * quality fields — gm_priority/class/accuracy stay 0. */

	st->offset_ns     = fpga_hal_read_ptp_offset();
	st->path_delay_ns = fpga_hal_read_path_delay();
	st->locked = (fpga_hal_read_status() & FPGA_HAL_CLK_WC_LOCKED) != 0;
}

bool ptp_ctrl_hw_wallclock_locked(void)
{
	return (fpga_hal_read_status() & FPGA_HAL_CLK_WC_LOCKED) != 0;
}

int ptp_ctrl_hw_get_foreign_masters(struct ptp_ctrl_foreign *out, int max)
{
	ARG_UNUSED(out);
	ARG_UNUSED(max);
	return 0;
}

void ptp_ctrl_hw_apply_config(void)
{
	/* ptp_bmc reads the same stored config and writes the FPGA CSRs. */
	ptp_bmc_update_own_dataset();
}
