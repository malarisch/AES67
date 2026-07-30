/*
 * Unit tests for the PTP status facade (src/ptp_ctrl.c,
 * src/ptp_ctrl_hw.c) and the FPGA-BMC glue it sits on
 * (src/ptp_bmc.c).
 *
 * The web UI, the REST API, the front-panel display and the audio
 * output enable all read the device's PTP state through this one
 * facade, so the mapping from FPGA status bits to role / lock must not
 * drift. The mock gateware reports hardware PTP, which is the path
 * exercised here.
 */

#include <zephyr/ztest.h>
#include <errno.h>
#include <string.h>

#include "ptp_ctrl.h"
#include "ptp_bmc.h"
#include "aes67_config.h"
#include "fpga_hal/fpga_hal.h"

static void reset_config(void *fixture)
{
	ARG_UNUSED(fixture);
	(void)aes67_config_get();
	aes67_config_reset_defaults();
}

ZTEST(ptp_ctrl, test_dispatches_to_hardware_backend)
{
	struct ptp_ctrl_status st;

	/* The mock's system_cfg says hardware PTP, so ptp_ctrl must not
	 * route to the Zephyr stack. */
	zassert_equal(fpga_hal_syscfg_load(), 0);
	zassert_false(fpga_hal_ptp_in_software());

	memset(&st, 0xFF, sizeof(st));
	ptp_ctrl_get_status(&st);
	zassert_str_equal(st.mode, "hardware");
}

ZTEST(ptp_ctrl, test_status_maps_fpga_bits)
{
	struct ptp_ctrl_status st;

	ptp_ctrl_get_status(&st);

	/* The mock reports PTP_IS_LEADER and no foreign grandmaster. */
	zassert_equal(ptp_bmc_get_role(), PTP_ROLE_LEADER);
	zassert_equal(st.role, PTP_CTRL_ROLE_LEADER);
	zassert_false(st.gm_valid, "no foreign GM exists in the mock");

	/* WC_LOCKED is set, and both lock reads must agree. */
	zassert_true(st.locked);
	zassert_true(ptp_ctrl_wallclock_locked());
	zassert_equal(st.locked, ptp_ctrl_wallclock_locked());

	zassert_equal(st.offset_ns, 0);
	zassert_equal(st.path_delay_ns, 0);

	/* Hardware mode cannot report the winner's announce quality. */
	zassert_equal(st.gm_priority1, 0);
	zassert_equal(st.gm_priority2, 0);
	zassert_equal(st.gm_clock_class, 0);
	zassert_equal(st.gm_clock_accuracy, 0);
	zassert_equal(st.steps_removed, 0);
}

ZTEST(ptp_ctrl, test_best_master_absent)
{
	uint8_t id[8];

	memset(id, 0xAA, sizeof(id));
	zassert_equal(ptp_bmc_get_best_master_id(id), -ENOENT);
}

ZTEST(ptp_ctrl, test_no_foreign_master_list_in_hardware_mode)
{
	struct ptp_ctrl_foreign fm[4];
	int count = -1;

	memset(fm, 0xAA, sizeof(fm));
	zassert_equal(ptp_ctrl_get_foreign_masters(fm, ARRAY_SIZE(fm)), 0,
		      "the FPGA BMA exposes no foreign-master list");
	zassert_equal(ptp_ctrl_get_foreign_masters(fm, 0), 0);

	zassert_is_null(ptp_bmc_get_foreign_masters(&count));
	zassert_equal(count, 0);
}

ZTEST(ptp_ctrl, test_apply_config_updates_own_dataset)
{
	struct aes67_device_config *cfg = aes67_config_get();

	cfg->ptp_priority1 = 42;
	cfg->ptp_priority2 = 43;
	cfg->ptp_clock_class = 6;        /* GPS-traceable */
	cfg->ptp_clock_accuracy = 0x21;  /* within 100 ns */

	ptp_ctrl_apply_config();

	const struct ptp_announce_dataset *ds = ptp_bmc_get_own_dataset();

	zassert_not_null(ds);
	zassert_equal(ds->gm_priority1, 42);
	zassert_equal(ds->gm_priority2, 43);
	zassert_equal(ds->gm_clock_class, 6);
	zassert_equal(ds->gm_clock_accuracy, 0x21);

	/* The announced grandmaster identity is our own clock identity. */
	uint8_t own[8];

	ptp_bmc_get_clock_identity(own);
	zassert_mem_equal(ds->gm_identity, own, 8);
	zassert_mem_equal(ds->sender_clock_id, own, 8);

	/* Defaults must come back through the same path. */
	aes67_config_reset_defaults();
	ptp_ctrl_apply_config();
	zassert_equal(ds->gm_priority1, 128);
	zassert_equal(ds->gm_priority2, 128);
	zassert_equal(ds->gm_clock_class, 248);
	zassert_equal(ds->gm_clock_accuracy, 0xFE);
}

ZTEST(ptp_ctrl, test_apply_config_pushes_delay_asymmetry)
{
	struct aes67_device_config *cfg = aes67_config_get();
	struct fpga_hal_ptp_tuning t;

	/* delayAsymmetry is persisted config but lives in the servo tuning
	 * register: applying it must not clobber the other live-tunable
	 * fields (read-modify-write). */
	fpga_hal_read_ptp_tuning(&t);
	t.lock_threshold_ns = 1234;
	t.kp_gain = 9;
	zassert_equal(fpga_hal_write_ptp_tuning(&t), 0);

	cfg->ptp_delay_asymmetry_ns = -750;
	ptp_ctrl_apply_config();

	fpga_hal_read_ptp_tuning(&t);
	zassert_equal(t.delay_asymmetry_ns, -750);
	zassert_equal(t.lock_threshold_ns, 1234);
	zassert_equal(t.kp_gain, 9);
}

ZTEST(ptp_ctrl, test_notifications_before_start_are_inert)
{
	/* main() can fire these before ptp_bmc_start() (link/IP events race
	 * the FPGA bring-up); they must be no-ops, not crashes. */
	ptp_bmc_notify_fpga_ready();
	ptp_bmc_notify_link_up();
	ptp_bmc_notify_ip_ready();
	ptp_bmc_register_change_cb(NULL);

	zassert_equal(ptp_bmc_get_role(), PTP_ROLE_LEADER);
}

ZTEST(ptp_ctrl, test_start_requires_a_mac)
{
	/* No interface, or one without a link-layer address, must be
	 * rejected rather than deriving a clock identity from garbage. */
	zassert_equal(ptp_bmc_start(NULL), -EINVAL);
}

ZTEST_SUITE(ptp_ctrl, NULL, NULL, reset_config, NULL, NULL);
