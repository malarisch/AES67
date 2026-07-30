#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>

#include "../drivers/fpga_hal/fpga_hal.h"
#ifdef CONFIG_DISPLAY_CTRL
#include "../drivers/display_ctrl/display_ctrl.h"
#endif
#include "card_manager.h"
#include "fpga_regs.h"
#include "pll_ctrl.h"
#include "ui_display.h"
#include "ptp_bmc.h"
#include "ptp_ctrl.h"
#ifdef CONFIG_AES67_PTP_SOFTWARE
#include "../drivers/eth_litex/eth_litex.h"
#endif
#include "sap_sdp.h"
#include "aes67_conn.h"
#include "aes67_config.h"
#include "fpga_poll.h"

LOG_MODULE_REGISTER(fpga_poll, LOG_LEVEL_INF);

/* Polling interval for FPGA status registers */
#define FPGA_POLL_INTERVAL_MS 100

/* This thread does far more than PPB polling since the bring-up staging:
 * on the first wallclock lock it runs the whole card activation chain
 * (card_manager -> lo_card_activate -> card_settings_apply -> I2C driver)
 * plus logging — 2048 bytes overflowed silently on Xtensa (windowed ABI)
 * and froze the system mid-activation. */
#define PPB_POLL_STACK_SIZE 4096
#define PPB_POLL_PRIORITY   K_PRIO_PREEMPT(10)

K_THREAD_STACK_DEFINE(ppb_poll_stack, PPB_POLL_STACK_SIZE);
static struct k_thread ppb_poll_thread_data;

static fpga_poll_dhcp_restart_fn g_dhcp_restart_cb;
static ptp_bmc_change_cb_t g_role_change_cb;
static bool g_ip_valid;
static struct in_addr g_poll_ip;

static struct ui_fpga_metrics disp_metrics = {
	.speed_code = 0xFF,
};

/* Link state for display gating: reads as "up" until the first status
 * sample so early-boot display updates are never suppressed. */
static bool link_state_valid;

bool fpga_poll_link_is_up(void)
{
	return !link_state_valid || disp_metrics.link_up;
}

void fpga_poll_notify_ip_valid(const struct in_addr *ip)
{
	g_ip_valid = true;
	if (ip) {
		g_poll_ip = *ip;
	}
}

void fpga_poll_register_role_change_cb(ptp_bmc_change_cb_t cb)
{
	g_role_change_cb = cb;
}

#ifdef CONFIG_AES67_PTP_SOFTWARE
/* ---- Leader rate relax (software-PTP gateware only) ----
 * When this board wins the BMCA after the grandmaster vanished, the
 * wallclock still runs at the follower-era rate correction — worst case
 * pinned at ±524287 ppb, which downstream followers (same ±524 ppm
 * range) can never track. Ramp the correction gently to 0 whenever the
 * Zephyr servo is not the one writing it (i.e. we are not a follower);
 * ~2.5 ppm/s keeps the frequency change trackable.
 *
 * The status struct is static deliberately: it is only touched by the
 * poll thread, and keeping it off the thread stack matters — the same
 * thread also runs the deep card-activation call chain. */
static void leader_ppb_relax_tick(void)
{
	static struct ptp_ctrl_status relax_st;

	if (!fpga_hal_ptp_in_software()) {
		/* Hardware-PTP gateware: the FPGA servo owns the rate. */
		return;
	}

	ptp_ctrl_get_status(&relax_st);
	if (relax_st.role != PTP_CTRL_ROLE_FOLLOWER) {
		aes67_ptp_rate_relax_step(250);
	}
}
#endif

static enum ptp_bmc_role role_from_ctrl(enum ptp_ctrl_role role)
{
	switch (role) {
	case PTP_CTRL_ROLE_LEADER:
		return PTP_ROLE_LEADER;
	case PTP_CTRL_ROLE_FOLLOWER:
		return PTP_ROLE_FOLLOWER;
	default:
		return PTP_ROLE_LISTENING;
	}
}

/**
 * @brief Background thread that continuously measures the PPB offset
 *        between the PLL and the PTP wallclock, and applies correction
 *        to the Si5351A clock generator.
 */
static void fpga_status_poll_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	bool measurement_running = false;
	uint32_t poll_count = 0;

	LOG_INF("PPB poll thread started");

	while (1) {
		k_msleep(FPGA_POLL_INTERVAL_MS);
		poll_count++;

		/* Read combined status word */
		uint32_t status = fpga_hal_read_status();

#ifdef CONFIG_AES67_PTP_SOFTWARE
		/* Leader rate relax (every 100 ms) — see helper above. */
		leader_ppb_relax_tick();
#endif

		/* ---- Update display metrics every 1 second ---- */
		if (poll_count >= 10) {
			poll_count = 0;

			disp_metrics.ppb_valid = !!(status & FPGA_HAL_CLK_PPB_VALID);

			/* Mode-invariant PTP snapshot: ptp_ctrl dispatches to the
			 * FPGA servo CSRs (hardware PTP) or the Zephyr stack
			 * (software PTP) based on the gateware's system_cfg. The
			 * status word's wc_locked bit is only driven by the
			 * hardware servo, so it cannot be used directly; the
			 * hardware backend folds it into .locked instead. */
			struct ptp_ctrl_status ptp_st;

			ptp_ctrl_get_status(&ptp_st);
			disp_metrics.wc_locked = ptp_st.role == PTP_CTRL_ROLE_LEADER ||
						 ptp_st.locked;
			disp_metrics.wc_phasejump = !!(status & FPGA_HAL_CLK_WC_PHASEJUMP);
			disp_metrics.wc_configured = !!(status & FPGA_HAL_CLK_WC_CONFIGURED);
			disp_metrics.ptp_leader_lost = !!(status & FPGA_HAL_CLK_PTP_LEADER_LOST);

			disp_metrics.link_up = !!(status & FPGA_HAL_ETH_LINK_UP);
			disp_metrics.speed_code = (status & FPGA_HAL_ETH_SPEED_MASK) >>
						  FPGA_HAL_ETH_SPEED_SHIFT;

			/* ---- Link state tracking with DHCP restart ---- */
			static bool link_prev_up;
			static int64_t link_down_since;
			static bool link_down_handled;

			link_state_valid = true;

			if (disp_metrics.link_up && !link_prev_up) {
				LOG_INF("Ethernet link up");

#ifdef CONFIG_DISPLAY_CTRL
				/* Restore the panel from "  LINK": long
				 * outage goes through DHCP re-acquisition,
				 * a short flap back to the PTP/role view. */
				if (display_ctrl_ready()) {
					if (link_down_handled) {
						display_ctrl_show_status("  DHCP");
					} else if (disp_metrics.wc_locked) {
						enum ptp_bmc_role role =
							role_from_ctrl(ptp_st.role);
						display_ctrl_start_status_cycle(
							role == PTP_ROLE_LEADER ?
							"LEADER" : "FOLLOW",
							g_ip_valid ? &g_poll_ip : NULL);
					} else {
						display_ctrl_show_status("L  PTP");
					}
				}
#endif

				if (link_down_handled) {
					k_msleep(500);
					if (g_dhcp_restart_cb) {
						g_dhcp_restart_cb();
					}
					link_down_handled = false;
				} else if (g_ip_valid) {
					LOG_INF("Short link flap - rejoining multicast groups");
					ptp_bmc_notify_link_up();
					sap_sdp_notify_link_up();
					aes67_conn_notify_link_up();
				}
			} else if (!disp_metrics.link_up && link_prev_up) {
				link_down_since = k_uptime_get();
				link_down_handled = false;
				LOG_INF("Ethernet link down");
#ifdef CONFIG_DISPLAY_CTRL
				/* Link loss trumps the PTP state on the
				 * panel; role-change repaints are suppressed
				 * while the link is down (on_bmc_change
				 * checks fpga_poll_link_is_up()). */
				if (display_ctrl_ready()) {
					display_ctrl_stop_status_cycle();
					display_ctrl_show_status("  LINK");
				}
#endif
			} else if (!disp_metrics.link_up && !link_prev_up
				   && !link_down_handled) {
				if ((k_uptime_get() - link_down_since) > 1000) {
					LOG_INF("Link down > 1s - will restart DHCP on link-up");
					link_down_handled = true;
				}
			}
			link_prev_up = disp_metrics.link_up;

#ifdef CONFIG_DISPLAY_CTRL
			/* Update status LEDs based on FPGA status */
			if (display_ctrl_ready()) {
				static bool prev_wc_locked;
				static bool prev_link_up;

				if (disp_metrics.wc_locked != prev_wc_locked) {
					display_ctrl_set_sys_led(DC_SYSLED_48K,
						disp_metrics.wc_locked ? DC_SYSLED_ON : DC_SYSLED_OFF);
					prev_wc_locked = disp_metrics.wc_locked;

					/* Clock locked: stop loading, start metering + status cycle */
					if (disp_metrics.wc_locked) {
						display_ctrl_loading_animation_stop();
						display_ctrl_start_metering();
						display_ctrl_stop_status_cycle();

						enum ptp_bmc_role role =
							role_from_ctrl(ptp_st.role);
						const char *role_str =
							(role == PTP_ROLE_LEADER) ? "LEADER" : "FOLLOW";
						display_ctrl_start_status_cycle(
							role_str,
							g_ip_valid ? &g_poll_ip : NULL);
					}
				}

				if (disp_metrics.link_up != prev_link_up) {
					display_ctrl_set_sys_led(DC_SYSLED_LIP,
						disp_metrics.link_up ? DC_SYSLED_ON : DC_SYSLED_OFF);
					prev_link_up = disp_metrics.link_up;
				}
			}
#endif

			/* ---- Activate outputs on first wallclock lock ----
			 * Until here every output card sits muted with its
			 * converters in reset; the PTP/wallclock lock is the
			 * signal that the media clock is real. */
			{
				static bool outputs_enabled;

				if (disp_metrics.wc_locked && !outputs_enabled) {
					LOG_INF("Wallclock locked - activating audio outputs");
					card_manager_activate_outputs();
					outputs_enabled = true;
				}
			}

			/* ptp_bmc is not running with software PTP: detect role
			 * changes here and feed them to the same handler main.c
			 * registers with ptp_bmc in hardware mode (role LEDs,
			 * status cycle). */
			if (fpga_hal_ptp_in_software()) {
				static enum ptp_bmc_role prev_role = PTP_ROLE_LISTENING;
				enum ptp_bmc_role cur_role = role_from_ctrl(ptp_st.role);

				if (g_role_change_cb && cur_role != prev_role) {
					g_role_change_cb(cur_role);
					prev_role = cur_role;
				}
			}

			/* Path delay / leader offset — mode-invariant via ptp_ctrl
			 * (FPGA CSRs in hardware mode, Zephyr stack in software
			 * mode; the HW CSRs read 0 in SW-PTP gateware). */
			disp_metrics.path_delay_ns = ptp_st.path_delay_ns;
			disp_metrics.leader_offset_ns = ptp_st.offset_ns;

			/* PPB offset */
			int32_t ppb_current;

			if (fpga_read_ppb(&ppb_current, NULL, NULL) == 0) {
				disp_metrics.ppb_offset = ppb_current;
			}

			struct pll_ctrl_state pll = pll_ctrl_get_state();

			disp_metrics.correction_ppb = pll.output;
			disp_metrics.cycle = pll.cycle;
			disp_metrics.outliers = pll.outliers;
			ui_display_set_metrics(&disp_metrics);
		}

		/* If no measurement is running, start one */
		if (!measurement_running) {
			if (fpga_hal_ctrl_set_bits(FPGA_HAL_CTRL_PPB_START) < 0) {
				LOG_WRN("PPB poll: failed to start measurement");
				continue;
			}
			measurement_running = true;
			LOG_DBG("PPB measurement started");
			continue;
		}

		/* Check if measurement is complete */
		if (!(status & FPGA_HAL_CLK_PPB_VALID)) {
			continue;
		}

		/* Measurement complete — clear the start bit and read the PPB value */
		fpga_hal_ctrl_clear_bits(FPGA_HAL_CTRL_PPB_START);

		int32_t ppb_measured;
		uint32_t count_wc, count_pll;

		if (fpga_read_ppb(&ppb_measured, &count_wc, &count_pll) < 0) {
			LOG_WRN("PPB poll: failed to read PPB");
			measurement_running = false;
			continue;
		}

		/* Run PI controller iteration */
		pll_ctrl_update(ppb_measured, count_wc, count_pll);

		/* Start next measurement immediately */
		measurement_running = false;
	}
}

void fpga_poll_start(fpga_poll_dhcp_restart_fn dhcp_restart_cb)
{
	g_dhcp_restart_cb = dhcp_restart_cb;

	k_thread_create(&ppb_poll_thread_data, ppb_poll_stack,
			PPB_POLL_STACK_SIZE,
			fpga_status_poll_thread, NULL, NULL, NULL,
			PPB_POLL_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&ppb_poll_thread_data, "ppb_poll");
}
