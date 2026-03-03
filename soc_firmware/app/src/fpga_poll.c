#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>

#include "../drivers/eth_fmc_basic/eth_fmc_basic.h"
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

static struct ui_fpga_metrics disp_metrics = {
	.speed_code = 0xFF,
};

void fpga_poll_notify_ip_valid(void)
{
	g_ip_valid = true;
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

	const struct device *fmc = device_get_binding("eth_fmc0");
	bool measurement_running = false;
	uint8_t status;
	int ret;
	uint32_t poll_count = 0;

	if (!fmc) {
		LOG_ERR("PPB poll: FMC device not found");
		return;
	}

	LOG_INF("PPB poll thread started");

	while (1) {
		k_msleep(FPGA_POLL_INTERVAL_MS);
		poll_count++;

		/* Read clocking status flags (register 0x50 read) */
		ret = eth_fmc_reg_read(fmc, ETH_FMC_REG_STATUS_CLK, &status);
		if (ret < 0) {
			LOG_WRN("PPB poll: failed to read status: %d", ret);
			continue;
		}

		/* ---- Update display metrics every 1 second ---- */
		if (poll_count >= 10) {
			poll_count = 0;

			disp_metrics.ppb_valid = !!(status & ETH_FMC_CLK_PPB_VALID);
			disp_metrics.wc_locked = !!(status & ETH_FMC_CLK_WC_LOCKED);
			disp_metrics.wc_phasejump = !!(status & ETH_FMC_CLK_WC_PHASEJUMP);
			disp_metrics.wc_configured = !!(status & ETH_FMC_CLK_WC_CONFIGURED);
			disp_metrics.ptp_leader_lost = !!(status & ETH_FMC_CLK_PTP_LEADER_LOST);

			/* Ethernet flags (register 0x51) */
			uint8_t eth_status;

			ret = eth_fmc_reg_read(fmc, ETH_FMC_REG_STATUS_ETH,
					       &eth_status);
			if (ret == 0) {
				unsigned speed_code = (eth_status &
						       ETH_FMC_ETH_SPEED_MASK) >>
						      ETH_FMC_ETH_SPEED_SHIFT;

				disp_metrics.link_up =
					!!(eth_status & ETH_FMC_ETH_LINK_UP);
				disp_metrics.speed_code = speed_code;

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
			}

#ifdef CONFIG_DISPLAY_CTRL
			/* Update status LEDs based on FPGA status */
			if (display_ctrl_ready()) {
				static bool prev_wc_locked;
				static bool prev_link_up;

				if (disp_metrics.wc_locked != prev_wc_locked) {
					display_ctrl_set_sys_led(DC_SYSLED_48K,
						disp_metrics.wc_locked ? DC_SYSLED_ON : DC_SYSLED_OFF);
					prev_wc_locked = disp_metrics.wc_locked;
				}

				if (disp_metrics.link_up != prev_link_up) {
					display_ctrl_set_sys_led(DC_SYSLED_LIP,
						disp_metrics.link_up ? DC_SYSLED_ON : DC_SYSLED_OFF);
					prev_link_up = disp_metrics.link_up;
				}
			}
#endif

			/* Path delay (register 0x52) */
			int32_t path_delay;

			ret = fpga_read_32(fmc, ETH_FMC_REG_PATH_DELAY,
					   &path_delay);
			if (ret == 0) {
				disp_metrics.path_delay_ns = path_delay;
			}

			/* Leader offset (register 0x53) */
			int32_t leader_offset;

			ret = fpga_read_32(fmc, ETH_FMC_REG_LEADER_OFFSET,
					   &leader_offset);
			if (ret == 0) {
				disp_metrics.leader_offset_ns = leader_offset;
			}

			/* PPB offset (registers 0x54/0x55) */
			int32_t ppb_current;

			ret = fpga_read_ppb(fmc, &ppb_current, NULL, NULL);
			if (ret == 0) {
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
			ret = eth_fmc_status_set_bits(fmc, ETH_FMC_FLAG_PPB_START);
			if (ret < 0) {
				LOG_WRN("PPB poll: failed to start measurement: %d", ret);
				continue;
			}
			measurement_running = true;
			LOG_DBG("PPB measurement started");
			continue;
		}

		/* Check if measurement is complete */
		if (!(status & ETH_FMC_CLK_PPB_VALID)) {
			continue;
		}

		/* Measurement complete — clear the start bit and read the PPB value */
		eth_fmc_status_clear_bits(fmc, ETH_FMC_FLAG_PPB_START);

		int32_t ppb_measured;
		uint32_t count_wc, count_pll;

		ret = fpga_read_ppb(fmc, &ppb_measured, &count_wc, &count_pll);
		if (ret < 0) {
			LOG_WRN("PPB poll: failed to read PPB: %d", ret);
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
