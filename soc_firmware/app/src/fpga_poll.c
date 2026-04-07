#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>

#include "../drivers/fpga_hal/fpga_hal.h"
#ifdef CONFIG_IO_CARD
#include "../drivers/io_card/io_card.h"
#endif
#ifdef CONFIG_LO_CARD
#include "../drivers/lo_card/lo_card.h"
#endif
#ifdef CONFIG_DISPLAY_CTRL
#include "../drivers/display_ctrl/display_ctrl.h"
#endif
#include "fpga_regs.h"
#include "pll_ctrl.h"
#include "ui_display.h"
#include "ptp_bmc.h"
#include "sap_sdp.h"
#include "aes67_config.h"
#include "fpga_poll.h"

LOG_MODULE_REGISTER(fpga_poll, LOG_LEVEL_INF);

/* Polling interval for FPGA status registers */
#define FPGA_POLL_INTERVAL_MS 100

#define PPB_POLL_STACK_SIZE 2048
#define PPB_POLL_PRIORITY   K_PRIO_PREEMPT(10)

K_THREAD_STACK_DEFINE(ppb_poll_stack, PPB_POLL_STACK_SIZE);
static struct k_thread ppb_poll_thread_data;

static fpga_poll_dhcp_restart_fn g_dhcp_restart_cb;
static bool g_ip_valid;
static struct in_addr g_poll_ip;

static struct ui_fpga_metrics disp_metrics = {
	.speed_code = 0xFF,
};

void fpga_poll_notify_ip_valid(const struct in_addr *ip)
{
	g_ip_valid = true;
	if (ip) {
		g_poll_ip = *ip;
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

		/* ---- Update display metrics every 1 second ---- */
		if (poll_count >= 10) {
			poll_count = 0;

			disp_metrics.ppb_valid = !!(status & FPGA_HAL_CLK_PPB_VALID);
			disp_metrics.wc_locked = !!(status & FPGA_HAL_CLK_WC_LOCKED);
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

			if (disp_metrics.link_up && !link_prev_up) {
				LOG_INF("Ethernet link up");

				if (link_down_handled) {
					k_msleep(500);
					if (g_dhcp_restart_cb) {
						g_dhcp_restart_cb();
					}
					link_down_handled = false;
				} else if (g_ip_valid) {
					LOG_INF("Short link flap — rejoining multicast groups");
					ptp_bmc_notify_link_up();
					sap_sdp_notify_link_up();
				}
			} else if (!disp_metrics.link_up && link_prev_up) {
				link_down_since = k_uptime_get();
				link_down_handled = false;
				LOG_INF("Ethernet link down");
			} else if (!disp_metrics.link_up && !link_prev_up
				   && !link_down_handled) {
				if ((k_uptime_get() - link_down_since) > 1000) {
					LOG_INF("Link down > 1s — will restart DHCP on link-up");
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

						enum ptp_bmc_role role = ptp_bmc_get_role();
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

			/* ---- Enable outputs on first wallclock lock ---- */
			{
				static bool outputs_enabled;

				if (disp_metrics.wc_locked && !outputs_enabled) {
					LOG_INF("Wallclock locked — enabling outputs");
#ifdef CONFIG_IO_CARD
					io_card_enable_outputs(true);
#endif
#ifdef CONFIG_LO_CARD
					lo_card_enable_outputs(true);
#endif
					outputs_enabled = true;
				}
			}

			/* Path delay */
			disp_metrics.path_delay_ns = fpga_hal_read_path_delay();

			/* Leader offset */
			disp_metrics.leader_offset_ns = fpga_hal_read_ptp_offset();

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
