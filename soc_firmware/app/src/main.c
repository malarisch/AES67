#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/net_event.h>
#include <zephyr/net/dhcpv4.h>
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
#include "fpga_regs.h"
#include "fpga_poll.h"
#include "pll_ctrl.h"
#include "ui_display.h"
#include "ptp_bmc.h"
#include "sap_sdp.h"
#include "webserver.h"
#include "aes67_config.h"
#ifdef CONFIG_SD_CONFIG
#include "sd_config.h"
#endif

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* Forward declarations */
static void fpga_reconfigure(const struct device *dev, void *user_data);

static struct net_if *g_iface;
static struct in_addr g_my_ip;
static bool g_ip_valid;
static bool g_dhcp_running;

/* ---- DHCP management ---- */

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

static struct net_mgmt_event_callback dhcp_cb;

static void on_dhcp_bound(struct net_mgmt_event_callback *cb,
			  uint64_t mgmt_event,
			  struct net_if *iface)
{
	if (mgmt_event != NET_EVENT_IPV4_DHCP_BOUND) {
		return;
	}

	const struct net_if_dhcpv4 *dhcpv4 =
		(const struct net_if_dhcpv4 *)cb->info;

	if (!dhcpv4) {
		LOG_WRN("DHCP bound but no info payload");
		return;
	}

	const uint8_t *ip = (const uint8_t *)&dhcpv4->requested_ip.s_addr;

	LOG_INF("DHCP bound: %u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);

	g_my_ip = dhcpv4->requested_ip;
	g_ip_valid = true;

	fpga_write_ip_address(&dhcpv4->requested_ip);
	ui_display_set_ip(&dhcpv4->requested_ip);
	fpga_poll_notify_ip_valid();

	ptp_bmc_notify_ip_ready();

#ifdef CONFIG_DISPLAY_CTRL
	if (display_ctrl_ready()) {
		display_ctrl_show_status("L  PTP");
	}
#endif

	sap_sdp_notify_ip_ready(&dhcpv4->requested_ip);
}

/* ---- FPGA Recovery Callback ---- */

static void fpga_reconfigure(const struct device *dev, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);

	LOG_INF("FPGA recovery: re-writing configuration registers");

	if (g_iface) {
		fpga_write_mac_address(g_iface);

		if (g_ip_valid) {
			fpga_write_ip_address(&g_my_ip);
		}
	}

	ptp_bmc_notify_fpga_ready();

	LOG_INF("FPGA recovery complete");
}

/* ---- BMC state change callback ---- */

static void on_bmc_change(enum ptp_bmc_role new_role)
{
	LOG_INF("PLL: BMC change (role=%d) — resetting PI controller", new_role);
	pll_ctrl_reset();

#ifdef CONFIG_DISPLAY_CTRL
	if (display_ctrl_ready()) {
		switch (new_role) {
		case PTP_ROLE_LEADER:
			display_ctrl_set_sys_led(DC_SYSLED_MSTR, DC_SYSLED_ON);
			display_ctrl_set_sys_led(DC_SYSLED_EXT, DC_SYSLED_OFF);
			display_ctrl_show_status("LEADDR");
			LOG_INF("LED: Master ON, Ext OFF");
			break;
		case PTP_ROLE_FOLLOWER:
			display_ctrl_set_sys_led(DC_SYSLED_MSTR, DC_SYSLED_OFF);
			display_ctrl_set_sys_led(DC_SYSLED_EXT, DC_SYSLED_ON);
			display_ctrl_show_status("SYNCNG");
			LOG_INF("LED: Master OFF, Ext ON");
			break;
		case PTP_ROLE_LISTENING:
		default:
			display_ctrl_set_sys_led(DC_SYSLED_MSTR, DC_SYSLED_BLINK1);
			display_ctrl_set_sys_led(DC_SYSLED_EXT, DC_SYSLED_OFF);
			display_ctrl_show_status("L  PTP");
			LOG_INF("LED: Master BLINK, Ext OFF (listening)");
			break;
		}
	}
#endif
}

/* ---- Entry point ---- */

int main(void)
{
	LOG_INF("AES67 System starting...");

	/* ---- Early Initialization (before FPGA ready) ---- */
	ui_display_init();

#ifdef CONFIG_SD_CONFIG
	/* ---- SD Card Initialization ---- */
	int sd_ret = sd_config_init();
	if (sd_ret < 0) {
		LOG_WRN("SD card not available: %d", sd_ret);
	}
#endif

	/* ---- Si5351A Clock Generator Setup ---- */
	const struct device *clkgen = DEVICE_DT_GET(DT_NODELABEL(si5351a));

	if (!device_is_ready(clkgen)) {
		LOG_ERR("Si5351A device not ready");
	} else {
		LOG_INF("Si5351A device ready, configuring clocks...");

		int ret = si5351a_set_frequency(clkgen, 0, 24576000);
		if (ret) {
			LOG_ERR("Failed to set CLK0: %d", ret);
		}
		si5351a_set_drive_strength(clkgen, 0, SI5351A_DRIVE_8MA);

		LOG_INF("Si5351A clocks configured: CLK0=24.576 MHz");
	}

#ifdef CONFIG_MI_CARD
	/* ---- MI Card 8-Channel ADC Preamp Setup ---- */
#ifdef CONFIG_MI_CARD_NRST_GPIO
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
		}
	}
#endif

#ifdef CONFIG_LO_CARD
	/* ---- LO Card 8-Channel DAC Line Output Setup ---- */
#ifdef CONFIG_LO_CARD_NRST_GPIO
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
		}
	}
#endif

#ifdef CONFIG_DISPLAY_CTRL
	/* ---- Display Controller (LEDs, Buttons, 7-Segment) Setup ---- */
#ifdef CONFIG_DISPLAY_CTRL_NRST_GPIO
	if (display_ctrl_nrst_gpio_init() < 0) {
		LOG_WRN("Display controller nRST GPIO init failed (hw reset unavailable)");
	} else {
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
			display_ctrl_all_off();
			display_ctrl_show_status("  FPGA");
		}
	}
#endif

	/* ---- Wait for FPGA to be ready before network operations ---- */
	if (eth_fmc_wait_for_fpga_ready(30000) < 0) {
		LOG_ERR("FPGA not ready after 30s - network services will not start");
		return -1;
	}

#ifdef CONFIG_SD_CONFIG
	/* ---- Load configuration from SD card (now that FPGA is ready) ---- */
	if (sd_config_is_ready()) {
		int cfg_ret = sd_config_load();
		if (cfg_ret == -ENOENT) {
			LOG_INF("No config file on SD card, using defaults");
			/* Save defaults to create the file */
			sd_config_save();
		} else if (cfg_ret < 0) {
			LOG_ERR("Failed to load config from SD: %d", cfg_ret);
		} else {
			LOG_INF("Configuration loaded from SD card");
		}
	}
#endif

	/* ---- Network Initialization (after FPGA ready) ---- */
	struct net_if *iface = net_if_get_default();
	if (!iface) {
		LOG_ERR("No network interface found");
		return -1;
	}

	g_iface = iface;

	eth_fmc_register_fpga_recover_cb(fpga_reconfigure, NULL);

	fpga_write_mac_address(iface);

	net_mgmt_init_event_callback(&dhcp_cb, on_dhcp_bound,
				     NET_EVENT_IPV4_DHCP_BOUND);
	net_mgmt_add_event_callback(&dhcp_cb);

#ifdef CONFIG_DISPLAY_CTRL
	if (display_ctrl_ready()) {
		display_ctrl_show_status("  LINK");
	}
#endif

	fpga_wait_for_link_up(30000);

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
	fpga_poll_start(dhcp_restart);

	LOG_INF("System ready");
	return 0;
}
