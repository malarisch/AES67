#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/net_event.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/dhcpv4.h>
#include <zephyr/sys/byteorder.h>
#include <string.h>
#include <zephyr/logging/log.h>

#include "../drivers/si5351a/si5351a.h"
#include "../drivers/eth_fmc_basic/eth_fmc_basic.h"
#ifdef CONFIG_MI_CARD
#include "../drivers/mi_card/mi_card.h"
#endif
#ifdef CONFIG_LO_CARD
#include "../drivers/lo_card/lo_card.h"
#endif
#ifdef CONFIG_DISPLAY_CTRL
#include "../drivers/display_ctrl/display_ctrl.h"
#endif
#include "ui_display.h"
#include "ptp_bmc.h"
#include "sap_sdp.h"
#include "webserver.h"
#include "aes67_config.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* Forward declarations for FPGA recovery */
static void fpga_reconfigure(const struct device *dev, void *user_data);
static struct net_if *g_iface; /* Cached for recovery callback */
static struct in_addr g_my_ip; /* Cached IP address for recovery */
static bool g_ip_valid;        /* true after DHCP bound */
static bool g_dhcp_running;    /* true while DHCP is active */

/* ---- FPGA configuration helpers ---- */

/**
 * @brief Write the interface MAC address to the FPGA via FMC register 0x40.
 *
 * The FPGA expects 6 consecutive byte-writes to address 0x40.
 * Each write auto-increments the internal byte counter.
 *
 * @param iface  Network interface whose MAC to send
 * @return 0 on success, negative errno on error
 */
static int fpga_write_mac_address(struct net_if *iface)
{
	const struct device *fmc = device_get_binding("eth_fmc0");
	struct net_linkaddr *ll;

	if (!fmc) {
		LOG_ERR("FMC device not found");
		return -ENODEV;
	}

	ll = net_if_get_link_addr(iface);
	if (!ll || ll->len < 6) {
		LOG_ERR("No valid MAC address on interface");
		return -EINVAL;
	}

	int ret = eth_fmc_reg_write(fmc, ETH_FMC_REG_MAC_ADDR, ll->addr, 6);
	if (ret < 0) {
		LOG_ERR("Failed to write MAC to FPGA: %d", ret);
		return ret;
	}

	LOG_INF("FPGA MAC set to %02x:%02x:%02x:%02x:%02x:%02x",
		ll->addr[0], ll->addr[1], ll->addr[2],
		ll->addr[3], ll->addr[4], ll->addr[5]);

	return 0;
}

/**
 * @brief Write an IPv4 address to the FPGA via FMC register 0x41.
 *
 * The FPGA expects 4 consecutive byte-writes to address 0x41.
 * Each write auto-increments the internal byte counter.
 *
 * @param addr  IPv4 address in network byte order (4 bytes)
 * @return 0 on success, negative errno on error
 */
static int fpga_write_ip_address(const struct in_addr *addr)
{
	const struct device *fmc = device_get_binding("eth_fmc0");

	if (!fmc) {
		LOG_ERR("FMC device not found");
		return -ENODEV;
	}

	int ret = eth_fmc_reg_write(fmc, ETH_FMC_REG_IP_ADDR,
				    (const uint8_t *)&addr->s_addr, 4);
	if (ret < 0) {
		LOG_ERR("Failed to write IP to FPGA: %d", ret);
		return ret;
	}

	LOG_INF("FPGA IP set to %u.%u.%u.%u",
		((const uint8_t *)&addr->s_addr)[0],
		((const uint8_t *)&addr->s_addr)[1],
		((const uint8_t *)&addr->s_addr)[2],
		((const uint8_t *)&addr->s_addr)[3]);

	return 0;
}

/* ---- Link-Up detection ---- */

/**
 * @brief Wait for Ethernet link-up by polling FPGA register 0x51.
 *
 * @param timeout_ms  Maximum time to wait (0 = no timeout)
 * @return 0 on link-up, -ETIMEDOUT on timeout
 */
static int wait_for_link_up(uint32_t timeout_ms)
{
	const struct device *fmc = device_get_binding("eth_fmc0");
	uint32_t elapsed = 0;
	uint8_t eth_status;

	if (!fmc) {
		return -ENODEV;
	}

	LOG_INF("Waiting for Ethernet link...");

	while (1) {
		if (eth_fmc_reg_read(fmc, ETH_FMC_REG_STATUS_ETH, &eth_status) == 0) {
			if (eth_status & ETH_FMC_ETH_LINK_UP) {
				LOG_INF("Link up after %u ms", elapsed);
				return 0;
			}
		}

		if (timeout_ms > 0 && elapsed >= timeout_ms) {
			LOG_WRN("Link-up timeout (%u ms)", timeout_ms);
			return -ETIMEDOUT;
		}

		k_msleep(100);
		elapsed += 100;
	}
}

/**
 * @brief Restart DHCP on the cached interface.
 *
 * Stops any running DHCP session, invalidates the cached IP, and
 * starts a fresh DHCP discovery.
 */
static void dhcp_restart(void)
{
	if (!g_iface) {
		return;
	}

	LOG_INF("Restarting DHCP...");

	if (g_dhcp_running) {
		net_dhcpv4_stop(g_iface);
		g_dhcp_running = false;
	}

	g_ip_valid = false;
	net_dhcpv4_start(g_iface);
	g_dhcp_running = true;
}

/* ---- DHCP event handling ---- */

static struct net_mgmt_event_callback dhcp_cb;

static void on_dhcp_bound(struct net_mgmt_event_callback *cb,
			  uint64_t mgmt_event,
			  struct net_if *iface)
{
	if (mgmt_event != NET_EVENT_IPV4_DHCP_BOUND) {
		return;
	}

	/* The event info payload is a struct net_if_dhcpv4 containing
	 * the assigned IP in 'requested_ip'.
	 */
	const struct net_if_dhcpv4 *dhcpv4 =
		(const struct net_if_dhcpv4 *)cb->info;

	if (!dhcpv4) {
		LOG_WRN("DHCP bound but no info payload");
		return;
	}

	const uint8_t *ip = (const uint8_t *)&dhcpv4->requested_ip.s_addr;

	LOG_INF("DHCP bound: %u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);

	/* Cache IP for FPGA recovery */
	g_my_ip = dhcpv4->requested_ip;
	g_ip_valid = true;

	/* Push the new IP address to the FPGA */
	fpga_write_ip_address(&dhcpv4->requested_ip);
	ui_display_set_ip(&dhcpv4->requested_ip);

	/* Unblock the BMC thread now that we have a valid IP on the FPGA */
	ptp_bmc_notify_ip_ready();

#ifdef CONFIG_DISPLAY_CTRL
	/* Update display to show PTP listening status */
	if (display_ctrl_ready()) {
		display_ctrl_show_status("L  PTP");
	}
#endif

	/* Notify SAP/SDP module of the assigned IP */
	sap_sdp_notify_ip_ready(&dhcpv4->requested_ip);
}

/* ---- FPGA Recovery Callback ---- */

/**
 * @brief Re-write all FPGA configuration registers after FPGA reset.
 *
 * Called by the FMC driver when it detects the FPGA transitioning from
 * NOT_PROGRAMMED or RESETTING to READY state.
 */
static void fpga_reconfigure(const struct device *dev, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);

	LOG_INF("FPGA recovery: re-writing configuration registers");

	if (g_iface) {
		/* Re-write MAC address */
		fpga_write_mac_address(g_iface);

		/* Re-write IP address if we have one */
		if (g_ip_valid) {
			fpga_write_ip_address(&g_my_ip);
		}
	}

	/* Re-notify PTP BMC to re-send leader config if applicable */
	ptp_bmc_notify_fpga_ready();

	LOG_INF("FPGA recovery complete");
}

/* ---- Legacy test code (unused) ---- */

#define RAW_PAYLOAD_START_LEN 30
#define RAW_PAYLOAD_MAX_LEN 1500
#define SEND_INTERVAL_MS 1000

/* Allocate worst-case payload so we never overflow when length ramps up. */
static uint8_t payload[RAW_PAYLOAD_MAX_LEN];
size_t frame_len = RAW_PAYLOAD_START_LEN;

static int send_raw_frame(struct net_if *iface, uint8_t seq)
{
    

	payload[0] = 0xAC;
	payload[1] = 0xAB;
	payload[2] = 0x16;
	payload[3] = 0x10;
	payload[4] = 0x00;
	payload[5] = 0xBE;
	payload[6] = 0xEF;


	payload[7] = 0x01;
	payload[8] = 0x02;
	payload[9] = 0x03;
	payload[10] = 0x04;
	payload[11] = 0x05;
	payload[12] = 0x06;

	payload[13] = 0x07;
	payload[14] = 0x08;
	

	frame_len++;
	if (frame_len >= RAW_PAYLOAD_MAX_LEN) {
		frame_len = RAW_PAYLOAD_START_LEN;
	}
	for (size_t i = 15; i < frame_len && i < RAW_PAYLOAD_MAX_LEN; i++) {
		payload[i] = seq;
	}
	struct net_pkt *pkt = net_pkt_alloc_with_buffer(iface, frame_len, AF_UNSPEC, 0, K_MSEC(100));
	if (!pkt) {
		LOG_ERR("pkt alloc failed");
		return -ENOMEM;
	}

	net_pkt_set_ll_proto_type(pkt, 0xDEAD);

		 // ||	    net_pkt_write(pkt, payload, RAW_PAYLOAD_LEN)
	if (net_pkt_write(pkt, payload, frame_len) < 0) {
		LOG_ERR("pkt write failed");
		net_pkt_unref(pkt);
		return -EIO;
	}

	int ret = net_send_data(pkt);
	if (ret < 0) {
		LOG_ERR("net_send_data failed (%d)", ret);
		net_pkt_unref(pkt);
		return ret;
	}

	return 0;
}

/* ---- FPGA status polling & PLL correction ---- */

/**
 * @brief Read a 4-byte (32-bit) value from a sequential FPGA register.
 *
 * The FPGA auto-increments the internal byte pointer on each read
 * to the same address.  Reads LSB first.
 *
 * @param fmc   The FMC device
 * @param reg   FPGA register address (0x52, 0x53, 0x54, 0x55)
 * @param val   Pointer to store the 32-bit result
 * @return 0 on success, negative errno on error
 */
static int fpga_read_32(const struct device *fmc, uint8_t reg, int32_t *val)
{
	uint8_t buf[4];
	int ret;

	ret = eth_fmc_reg_read_block(fmc, reg, buf, 4);
	if (ret < 0) {
		return ret;
	}

	*val = (int32_t)((uint32_t)buf[0] |
			 ((uint32_t)buf[1] << 8) |
			 ((uint32_t)buf[2] << 16) |
			 ((uint32_t)buf[3] << 24));
	return 0;
}

/* Base frequency for CLK0 – must match initial si5351a_set_frequency() call */
#define SI_CLK0_BASE_FREQ_HZ 24576000U

/* Polling interval for FPGA status registers */
#define FPGA_POLL_INTERVAL_MS 100

/* =====================================================================
 * PI controller for PLL frequency discipline
 *
 * The FPGA measures the PPB error between the PLL clock and the PTP
 * wallclock once per second.  We use a PI controller to smoothly
 * converge on zero error.
 *
 * Controller output = Kp * error + Ki * integral(error)
 *
 * Gains are now read from the global runtime configuration object
 * (aes67_config) so they can be changed at runtime via the REST API.
 *
 * Anti-windup: integrator clamped to ±pi_imax ppb
 * Outlier rejection: samples > pi_outlier_ppb rejected after warm-up
 * ===================================================================== */

/* PI controller state */
static struct {
	int64_t integrator;    /* Accumulated integral term (ppb, scaled by KI_DEN) */
	int32_t output;        /* Current total correction applied (ppb) */
	uint32_t cycle;        /* Total accepted measurement count */
	uint32_t outliers;     /* Rejected outlier count */
} pi_state;

/**
 * @brief Called by the BMC whenever the PTP state changes (role or leader).
 *        Resets the PI controller warm-up so outlier rejection re-arms.
 *        Updates status LEDs to reflect current PTP role.
 */
static void on_bmc_change(enum ptp_bmc_role new_role)
{
	LOG_INF("PLL: BMC change (role=%d) — resetting PI controller", new_role);
	pi_state.integrator = 0;
	pi_state.output     = 0;
	pi_state.cycle      = 0;
	pi_state.outliers   = 0;

#ifdef CONFIG_DISPLAY_CTRL
	/* Update status LEDs and display based on PTP role */
	if (display_ctrl_ready()) {
		switch (new_role) {
		case PTP_ROLE_LEADER:
			/* Master mode: Master LED ON, Ext LED OFF */
			display_ctrl_set_sys_led(DC_SYSLED_MSTR, DC_SYSLED_ON);
			display_ctrl_set_sys_led(DC_SYSLED_EXT, DC_SYSLED_OFF);
			display_ctrl_show_status("LEADDR");
			LOG_INF("LED: Master ON, Ext OFF");
			break;
		case PTP_ROLE_FOLLOWER:
			/* Follower mode: Master LED OFF, Ext LED ON */
			display_ctrl_set_sys_led(DC_SYSLED_MSTR, DC_SYSLED_OFF);
			display_ctrl_set_sys_led(DC_SYSLED_EXT, DC_SYSLED_ON);
			display_ctrl_show_status("SYNCNG");
			LOG_INF("LED: Master OFF, Ext ON");
			break;
		case PTP_ROLE_LISTENING:
		default:
			/* Listening: Both LEDs blink */
			display_ctrl_set_sys_led(DC_SYSLED_MSTR, DC_SYSLED_BLINK1);
			display_ctrl_set_sys_led(DC_SYSLED_EXT, DC_SYSLED_OFF);
			display_ctrl_show_status("L  PTP");
			LOG_INF("LED: Master BLINK, Ext OFF (listening)");
			break;
		}
	}
#endif
}

static struct ui_fpga_metrics disp_metrics = {
	.speed_code = 0xFF,
};

/**
 * @brief Calculate PPB from raw edge counter values.
 *
 * PPB formula:  ppb = (count_pll - count_wc) * 1e9 / count_wc
 *   Positive -> PLL is running fast relative to wallclock.
 *   Negative -> PLL is running slow relative to wallclock.
 *
 * @param count_wc   Wallclock edge count (22-bit, ~3.072M for 64·48kHz)
 * @param count_pll  PLL edge count (22-bit)
 * @return PPB difference, or 0 if count_wc is zero
 */
static int32_t calculate_ppb(uint32_t count_wc, uint32_t count_pll)
{
	if (count_wc == 0) {
		return 0;
	}

	/* Calculate diff = count_pll - count_wc (signed) */
	int32_t diff = (int32_t)count_pll - (int32_t)count_wc;

	/* ppb = diff * 1e9 / count_wc
	 * Use 64-bit arithmetic to avoid overflow (diff can be ~±4M, *1e9 overflows 32-bit)
	 */
	int64_t ppb = ((int64_t)diff * 1000000000LL) / (int64_t)count_wc;

	/* Clamp to int32_t range (should never exceed in practice) */
	if (ppb > INT32_MAX) {
		return INT32_MAX;
	}
	if (ppb < INT32_MIN) {
		return INT32_MIN;
	}
	return (int32_t)ppb;
}

/**
 * @brief Read PPB meter values and calculate PPB.
 *
 * Reads the raw counter values from FPGA registers 0x54/0x55 and
 * calculates the PPB difference.
 *
 * @param fmc         FMC device
 * @param ppb_out     Output: calculated PPB value
 * @param count_wc_out  Optional output: raw wallclock count (can be NULL)
 * @param count_pll_out Optional output: raw PLL count (can be NULL)
 * @return 0 on success, negative errno on error
 */
static int fpga_read_ppb(const struct device *fmc, int32_t *ppb_out,
			 uint32_t *count_wc_out, uint32_t *count_pll_out)
{
	uint32_t count_wc, count_pll;
	int ret;

	ret = fpga_read_32(fmc, ETH_FMC_REG_COUNT_WC, (int32_t *)&count_wc);
	if (ret < 0) {
		return ret;
	}
	ret = fpga_read_32(fmc, ETH_FMC_REG_COUNT_PLL, (int32_t *)&count_pll);
	if (ret < 0) {
		return ret;
	}

	/* Mask to 22 bits (upper byte is always 0, but be safe) */
	count_wc &= 0x3FFFFF;
	count_pll &= 0x3FFFFF;

	*ppb_out = calculate_ppb(count_wc, count_pll);

	if (count_wc_out) {
		*count_wc_out = count_wc;
	}
	if (count_pll_out) {
		*count_pll_out = count_pll;
	}

	return 0;
}

/**
 * @brief Background thread that continuously measures the PPB offset
 *        between the PLL and the PTP wallclock, and applies correction
 *        to the Si5351A clock generator.
 *
 * Flow:
 *   1. Write bit[0] of register 0x50 to start measurement.
 *   2. Poll register 0x50 every 100 ms until PPB-valid flag appears.
 *   3. Read counter values from registers 0x54/0x55 and calculate PPB.
 *   4. Apply the negated PPB as a correction to the Si5351A PLL.
 *   5. Start the next measurement immediately and repeat.
 */
static void fpga_status_poll_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	const struct device *fmc = device_get_binding("eth_fmc0");
	const struct device *clkgen = DEVICE_DT_GET(DT_NODELABEL(si5351a));
	bool measurement_running = false;
	uint8_t status;
	int ret;
	uint32_t poll_count = 0;   /* Counts 100ms ticks; print at 10 = 1s */

	if (!fmc) {
		LOG_ERR("PPB poll: FMC device not found");
		return;
	}
	if (!device_is_ready(clkgen)) {
		LOG_ERR("PPB poll: Si5351A device not ready");
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

		/* ---- Print all status registers every 1 second ---- */
		if (poll_count >= 10) {
			poll_count = 0;

			disp_metrics.ppb_valid = !!(status & ETH_FMC_CLK_PPB_VALID);
			disp_metrics.wc_locked = !!(status & ETH_FMC_CLK_WC_LOCKED);
			disp_metrics.wc_phasejump = !!(status & ETH_FMC_CLK_WC_PHASEJUMP);
			disp_metrics.wc_configured = !!(status & ETH_FMC_CLK_WC_CONFIGURED);
			disp_metrics.ptp_leader_lost = !!(status & ETH_FMC_CLK_PTP_LEADER_LOST);

			/* 0x50 - Clocking flags (already in 'status') */
			//LOG_INF("CLK flags: PPB_valid=%d WC_locked=%d WC_phasejump=%d "
			/*	"WC_configured=%d PTP_leader_lost=%d",
				disp_metrics.ppb_valid,
				disp_metrics.wc_locked,
				disp_metrics.wc_phasejump,
				disp_metrics.wc_configured,
				disp_metrics.ptp_leader_lost);
*/
			/* 0x51 - Ethernet flags */
			uint8_t eth_status;

			ret = eth_fmc_reg_read(fmc, ETH_FMC_REG_STATUS_ETH,
					       &eth_status);
			if (ret == 0) {
				unsigned speed_code = (eth_status &
						       ETH_FMC_ETH_SPEED_MASK) >>
						      ETH_FMC_ETH_SPEED_SHIFT;
				ARG_UNUSED(speed_code); /* Used conditionally below */

				//const char *speed_str =
				//	(speed_code == 0) ? "10M" :
				//	(speed_code == 1) ? "100M" :
				//	(speed_code == 2) ? "1G" : "?";
				//LOG_INF("ETH flags: link_up=%d speed=%s",
				//	!!(eth_status & ETH_FMC_ETH_LINK_UP),
				//	speed_str);

				disp_metrics.link_up =
					!!(eth_status & ETH_FMC_ETH_LINK_UP);
				disp_metrics.speed_code = speed_code;

				/* ---- Link state tracking with DHCP restart ---- */
				static bool link_prev_up = false;
				static int64_t link_down_since = 0; /* uptime ms when link went down */
				static bool link_down_handled = false;

				if (disp_metrics.link_up && !link_prev_up) {
					/* Link just came up */
					LOG_INF("Ethernet link up");

					if (link_down_handled) {
						/* Link was down > 1s — restart DHCP
						 * and allow PHY to settle first */
						k_msleep(500);
						dhcp_restart();
						link_down_handled = false;
					} else if (g_ip_valid) {
						/* Short glitch (<1s) with valid IP — just rejoin multicast */
						LOG_INF("Short link flap — rejoining multicast groups");
						ptp_bmc_notify_link_up();
						sap_sdp_notify_link_up();
					}
				} else if (!disp_metrics.link_up && link_prev_up) {
					/* Link just went down */
					link_down_since = k_uptime_get();
					link_down_handled = false;
					LOG_INF("Ethernet link down");
				} else if (!disp_metrics.link_up && !link_prev_up
					   && !link_down_handled) {
					/* Still down — check if > 1 second */
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
				static bool prev_wc_locked = false;
				static bool prev_link_up = false;

				/* 48K LED: ON when wallclock is locked */
				if (disp_metrics.wc_locked != prev_wc_locked) {
					display_ctrl_set_sys_led(DC_SYSLED_48K,
						disp_metrics.wc_locked ? DC_SYSLED_ON : DC_SYSLED_OFF);
					prev_wc_locked = disp_metrics.wc_locked;
				}

				/* LIP LED: ON when Ethernet link is up */
				if (disp_metrics.link_up != prev_link_up) {
					display_ctrl_set_sys_led(DC_SYSLED_LIP,
						disp_metrics.link_up ? DC_SYSLED_ON : DC_SYSLED_OFF);
					prev_link_up = disp_metrics.link_up;
				}
			}
#endif

			/* 0x52 - Path delay */
			int32_t path_delay;

			ret = fpga_read_32(fmc, ETH_FMC_REG_PATH_DELAY,
					   &path_delay);
			if (ret == 0) {
				//LOG_INF("Path delay: %d ns", path_delay);
				disp_metrics.path_delay_ns = path_delay;
			}

			/* 0x53 - Leader offset */
			int32_t leader_offset;

			ret = fpga_read_32(fmc, ETH_FMC_REG_LEADER_OFFSET,
					   &leader_offset);
			if (ret == 0) {
				//LOG_INF("Leader offset: %d ns", leader_offset);
				disp_metrics.leader_offset_ns = leader_offset;
			}

			/* 0x54/0x55 - PPB offset (current reading, calculated from counters) */
			int32_t ppb_current;

			ret = fpga_read_ppb(fmc, &ppb_current, NULL, NULL);
			if (ret == 0) {
				//LOG_INF("PPB offset: %d  (total correction: %d)",
				//	ppb_current, pi_state.output);
				disp_metrics.ppb_offset = ppb_current;
			}

			disp_metrics.correction_ppb = pi_state.output;
			disp_metrics.cycle = pi_state.cycle;
			disp_metrics.outliers = pi_state.outliers;
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
			/* Still measuring — wait */
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

		/* Debug: log raw counter values */
		LOG_INF("PPB raw: count_wc=%u count_pll=%u ppb=%d",
			count_wc, count_pll, ppb_measured);

		/* ---- Outlier rejection (after warm-up) ---- */
		const struct aes67_device_config *cfg = aes67_config_get();

		if (pi_state.cycle >= cfg->pi_warmup_cycles) {
			int32_t abs_meas = (ppb_measured < 0) ? -ppb_measured
							      : ppb_measured;
			if (abs_meas > cfg->pi_outlier_ppb) {
				pi_state.outliers++;
				LOG_WRN("PPB outlier rejected: %d (wc=%u pll=%u) "
					"(outliers=%u cycle=%u)",
					ppb_measured, count_wc, count_pll,
					pi_state.outliers, pi_state.cycle);
				measurement_running = false;
				continue;
			}
		}

		pi_state.cycle++;

		/*
		 * PI controller:
		 *
		 * error = ppb_measured  (positive = PLL fast → need to slow down)
		 *
		 * P term:  correction_p = Kp * error
		 * I term:  integrator += error;  correction_i = Ki * integrator
		 * Output:  output -= (correction_p + correction_i)
		 *
		 * The output is the *absolute* ppb offset applied to the PLL
		 * via si5351a_adjust_ppb().
		 */
		int32_t error = ppb_measured;

		/* Read PI gains from global config */
		int32_t kp_num = cfg->pi_kp_num;
		int32_t kp_den = cfg->pi_kp_den;
		int32_t ki_num = cfg->pi_ki_num;
		int32_t ki_den = cfg->pi_ki_den;
		int32_t imax   = cfg->pi_imax;

		/* Proportional term */
		int32_t p_term = (int32_t)(((int64_t)error * kp_num) / kp_den);

		/* Integral term with anti-windup */
		pi_state.integrator += error;
		if (pi_state.integrator > (int64_t)imax * ki_den) {
			pi_state.integrator = (int64_t)imax * ki_den;
		} else if (pi_state.integrator < -(int64_t)imax * ki_den) {
			pi_state.integrator = -(int64_t)imax * ki_den;
		}
		int32_t i_term = (int32_t)(pi_state.integrator * ki_num / ki_den);

		/* Apply combined correction (subtract because positive error = too fast) */
		pi_state.output -= (p_term + i_term);

		//LOG_INF("PI: err=%d P=%d I=%d out=%d  cycle=%u",
		//	error, p_term, i_term, pi_state.output, pi_state.cycle);

		ret = si5351a_adjust_ppb(clkgen, 0, SI_CLK0_BASE_FREQ_HZ,
					 pi_state.output);
		if (ret < 0) {
			LOG_ERR("PPB poll: Si5351A adjust failed: %d", ret);
		}

		/* Start next measurement immediately */
		measurement_running = false;
	}
}

#define PPB_POLL_STACK_SIZE 2048
#define PPB_POLL_PRIORITY   K_PRIO_PREEMPT(10)

K_THREAD_STACK_DEFINE(ppb_poll_stack, PPB_POLL_STACK_SIZE);
static struct k_thread ppb_poll_thread_data;

int main(void)
{
	LOG_INF("AES67 System starting...");

	/* ---- Early Initialization (before FPGA ready) ---- */
	ui_display_init();

    /* ---- Si5351A Clock Generator Setup ---- */
    const struct device *clkgen = DEVICE_DT_GET(DT_NODELABEL(si5351a));

    if (!device_is_ready(clkgen)) {
        LOG_ERR("Si5351A device not ready");
    } else {
        LOG_INF("Si5351A device ready, configuring clocks...");

        /* CLK0: 24.576 MHz  – AES67 audio master clock (48 kHz × 512) */
        int ret = si5351a_set_frequency(clkgen, 0, 24576000);
        if (ret) {
            LOG_ERR("Failed to set CLK0: %d", ret);
        }
        /* Drive strength 8 mA for both outputs */
        si5351a_set_drive_strength(clkgen, 0, SI5351A_DRIVE_8MA);

        LOG_INF("Si5351A clocks configured: CLK0=24.576 MHz");
    }

#ifdef CONFIG_MI_CARD
    /* ---- MI Card 8-Channel ADC Preamp Setup ---- */
#ifdef CONFIG_MI_CARD_NRST_GPIO
    /* Initialize nRST GPIO first */
    if (mi_card_nrst_gpio_init() < 0) {
        LOG_WRN("MI card nRST GPIO init failed (hw reset unavailable)");
    }
#endif
    const struct device *mi_i2c = DEVICE_DT_GET(DT_NODELABEL(i2c2));
    if (!device_is_ready(mi_i2c)) {
        LOG_ERR("MI card I2C bus (i2c2) not ready");
    } else {
        int mi_ret = mi_card_init(mi_i2c);
        if (mi_ret < 0) {
            LOG_WRN("MI card init failed: %d (board may not be connected)", mi_ret);
        } else {
            LOG_INF("MI card initialized successfully");
            /* Example: Set channel 0 to 20 dB gain, enable HPF */
            /* mi_card_set_gain(0, 20); */
            /* mi_card_set_hpf(true); */
        }
    }
#endif

#ifdef CONFIG_LO_CARD
    /* ---- LO Card 8-Channel DAC Line Output Setup ---- */
#ifdef CONFIG_LO_CARD_NRST_GPIO
    /* Initialize nRST GPIO first */
    if (lo_card_nrst_gpio_init() < 0) {
        LOG_WRN("LO card nRST GPIO init failed (hw reset unavailable)");
    }
#endif
    const struct device *lo_i2c = DEVICE_DT_GET(DT_NODELABEL(i2c2));
    if (!device_is_ready(lo_i2c)) {
        LOG_ERR("LO card I2C bus (i2c2) not ready");
    } else {
        int lo_ret = lo_card_init(lo_i2c);
        if (lo_ret < 0) {
            LOG_WRN("LO card init failed: %d (board may not be connected)", lo_ret);
        } else {
            LOG_INF("LO card initialized successfully");
            /* Example: Set channel 0 clip to 0 dB, enable outputs */
            /* lo_card_set_clip(0, 0); */
            /* lo_card_enable_outputs(true); */
        }
    }
#endif

#ifdef CONFIG_DISPLAY_CTRL
    /* ---- Display Controller (LEDs, Buttons, 7-Segment) Setup ---- */
#ifdef CONFIG_DISPLAY_CTRL_NRST_GPIO
    /* Initialize nRST GPIO first */
    if (display_ctrl_nrst_gpio_init() < 0) {
        LOG_WRN("Display controller nRST GPIO init failed (hw reset unavailable)");
    } else {
        /* Perform hardware reset to ensure clean state */
        display_ctrl_hw_reset();
    }
#endif
    const struct device *dc_uart = DEVICE_DT_GET(DT_ALIAS(display_ctrl_uart));
    if (!device_is_ready(dc_uart)) {
        LOG_WRN("Display controller UART not ready");
    } else {
        int dc_ret = display_ctrl_init(dc_uart);
        if (dc_ret < 0) {
            LOG_WRN("Display controller init failed: %d", dc_ret);
        } else {
            LOG_INF("Display controller initialized");
            /* Clear all LEDs and displays on startup */
            display_ctrl_all_off();
            /* Show boot status: waiting for FPGA */
            display_ctrl_show_status("  FPGA");
        }
    }
#endif

    /* ---- Wait for FPGA to be ready before network operations ---- */
    if (eth_fmc_wait_for_fpga_ready(30000) < 0) {
        LOG_ERR("FPGA not ready after 30s - network services will not start");
        return -1;
    }

    /* ---- Network Initialization (after FPGA ready) ---- */
    struct net_if *iface = net_if_get_default();
    if (!iface) {
        LOG_ERR("No network interface found");
        return -1;
    }
    
    /* Cache interface for FPGA recovery callback */
    g_iface = iface;
    
    /* Register FPGA recovery callback for automatic reconfiguration */
    eth_fmc_register_fpga_recover_cb(fpga_reconfigure, NULL);

    /* ---- Write MAC address to FPGA ---- */
    fpga_write_mac_address(iface);

    /* ---- Register DHCP event handler to push IP to FPGA ---- */
    net_mgmt_init_event_callback(&dhcp_cb, on_dhcp_bound,
                                 NET_EVENT_IPV4_DHCP_BOUND);
    net_mgmt_add_event_callback(&dhcp_cb);

    /* ---- Wait for Ethernet link-up before starting DHCP ---- */
#ifdef CONFIG_DISPLAY_CTRL
    if (display_ctrl_ready()) {
        display_ctrl_show_status("  LINK");
    }
#endif

    wait_for_link_up(30000);

    /* Allow the PHY / switch to fully settle after link-up */
    k_msleep(500);

#ifdef CONFIG_DISPLAY_CTRL
    if (display_ctrl_ready()) {
        display_ctrl_show_status("  DHCP");
    }
#endif

    LOG_INF("Starting DHCP...");
    net_dhcpv4_start(iface);
    g_dhcp_running = true;

    /* ---- Start PTPv2 Best Master Clock algorithm ---- */
    ptp_bmc_register_change_cb(on_bmc_change);
    int bmc_ret = ptp_bmc_start(iface);
    if (bmc_ret < 0) {
        LOG_ERR("Failed to start PTP BMC: %d", bmc_ret);
    }

    /* ---- Start AES67 SAP/SDP announcements ---- */
    int sap_ret = sap_sdp_start(iface);
    if (sap_ret < 0) {
        LOG_ERR("Failed to start SAP/SDP: %d", sap_ret);
    }

    /* ---- Start HTTP server (REST API + Web UI) ---- */
    int web_ret = webserver_start();
    if (web_ret < 0) {
        LOG_ERR("Failed to start HTTP server: %d", web_ret);
    }

    /* ---- Start PPB measurement / PLL correction thread ---- */
    k_thread_create(&ppb_poll_thread_data, ppb_poll_stack,
                    PPB_POLL_STACK_SIZE,
                    fpga_status_poll_thread, NULL, NULL, NULL,
                    PPB_POLL_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&ppb_poll_thread_data, "ppb_poll");

    LOG_INF("System ready");
    return 0;

}
