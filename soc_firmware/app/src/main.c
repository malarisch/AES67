#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/net_event.h>
#include <zephyr/net/dhcpv4.h>
#include <zephyr/net/hostname.h>
#include <zephyr/logging/log.h>
#include <string.h>

#ifdef CONFIG_SI5351A
#include "../drivers/si5351a/si5351a.h"
#endif
#include "../drivers/fpga_hal/fpga_hal.h"
#ifdef CONFIG_MI_CARD
#include "../drivers/mi_card/mi_card.h"
#endif
#ifdef CONFIG_LO_CARD
#include "../drivers/lo_card/lo_card.h"
#endif
#ifdef CONFIG_IO_CARD
#include "../drivers/io_card/io_card.h"
#endif
#include "card_manager.h"
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
// TODO: FW Update is broken, SPI flash reads garbage when in master mode
#ifdef CONFIG_SPI_FLASH_LITESPI
#include "fw_update.h"
#endif
#ifdef CONFIG_RTSP
#include "rtsp.h"
#endif
#ifdef CONFIG_SD_CONFIG
#include "sd_config.h"
#endif
#ifdef CONFIG_FLASH_CONFIG
#include "flash_config.h"
#endif
#ifdef CONFIG_MDNS_SD
#include "mdns_sd.h"
#endif

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* Forward declarations */
static void fpga_reconfigure(void *user_data);

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
	if (g_iface) {
		fpga_write_mac_address(g_iface);
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
	fpga_poll_notify_ip_valid(&dhcpv4->requested_ip);

	ptp_bmc_notify_ip_ready();

#ifdef CONFIG_DISPLAY_CTRL
	if (display_ctrl_ready()) {
		display_ctrl_show_status("L  PTP");
		display_ctrl_update_status_cycle_ip(&dhcpv4->requested_ip);
	}
#endif

	sap_sdp_notify_ip_ready(&dhcpv4->requested_ip);

#ifdef CONFIG_RTSP
	rtsp_notify_ip_ready(&dhcpv4->requested_ip);
#endif
}

/* ---- FPGA Recovery Callback ---- */

static void fpga_reconfigure(void *user_data)
{
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
			display_ctrl_stop_status_cycle();
			/* If wallclock already locked, go online immediately;
			 * otherwise let fpga_poll handle when wc_locked fires */
			if (fpga_hal_read_status() & FPGA_HAL_CLK_WC_LOCKED) {
				display_ctrl_loading_animation_stop();
				display_ctrl_start_metering();
				display_ctrl_start_status_cycle("LEADER",
					g_ip_valid ? &g_my_ip : NULL);
			}
			LOG_INF("LED: Master ON, Ext OFF");
			break;
		case PTP_ROLE_FOLLOWER:
			display_ctrl_set_sys_led(DC_SYSLED_MSTR, DC_SYSLED_OFF);
			display_ctrl_set_sys_led(DC_SYSLED_EXT, DC_SYSLED_ON);
			display_ctrl_stop_status_cycle();
			/* If wallclock already locked, go online;
			 * otherwise show SYNCNG and let fpga_poll handle transition */
			if (fpga_hal_read_status() & FPGA_HAL_CLK_WC_LOCKED) {
				display_ctrl_loading_animation_stop();
				display_ctrl_start_metering();
				display_ctrl_start_status_cycle("FOLLOW",
					g_ip_valid ? &g_my_ip : NULL);
			} else {
				display_ctrl_show_status("SYNCNG");
			}
			LOG_INF("LED: Master OFF, Ext ON");
			break;
		case PTP_ROLE_LISTENING:
		default:
			display_ctrl_set_sys_led(DC_SYSLED_MSTR, DC_SYSLED_BLINK1);
			display_ctrl_set_sys_led(DC_SYSLED_EXT, DC_SYSLED_OFF);
			display_ctrl_show_status("L  PTP");
			display_ctrl_stop_status_cycle();
			LOG_INF("LED: Master BLINK, Ext OFF (listening)");
			break;
		}
	}
#endif
}

/* ---- Shared nRST post-reset callback: rescan IO card ---- */
#if defined(CONFIG_DISPLAY_CTRL) && (defined(CONFIG_DISPLAY_CTRL_NRST_GPIO) || defined(CONFIG_DISPLAY_CTRL_NRST_HAL))
static void nrst_card_rescan_cb(void *unused)
{
	ARG_UNUSED(unused);
	card_manager_rescan();
}
#endif

/* ---- Entry point ---- */

int main(void)
{
	LOG_INF("AES67 System starting...");

	/* Hold the IO card in reset from the very start.
	 * The CS5368 ADCs must not leave reset until MCLK is stable.
	 * nRST will be released later by card_manager_init() after
	 * the Si5351A PLL has locked and clocks have settled. */
	fpga_hal_set_adda_nrst(false);
	LOG_INF("ADDA nRST asserted (held in reset)");

	/* ---- Early Initialization (before FPGA ready) ---- */
	ui_display_init();

#ifdef CONFIG_SD_CONFIG
	/* ---- SD Card Initialization ---- */
	int sd_ret = sd_config_init();
	if (sd_ret < 0) {
		LOG_WRN("SD card not available: %d", sd_ret);
	}
#endif

#ifdef CONFIG_FLASH_CONFIG
	/* ---- SPI Flash Config Initialization ---- */
	int flash_cfg_ret = flash_config_init();
	if (flash_cfg_ret < 0) {
		LOG_WRN("Flash config not available: %d", flash_cfg_ret);
	}
#endif

#ifdef CONFIG_SI5351A
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

		/* CS5368 ADCs require a stable MCLK before their reset is
		 * released — otherwise the I2C interface won't initialise.
		 * Give the Si5351A PLL time to lock and the clock to settle. */
		LOG_INF("Waiting 200 ms for MCLK to stabilise...");
		k_msleep(200);
	}
#endif

	/* ---- Shared nRST: reset display controller + IO card hardware ---- */
#ifdef CONFIG_DISPLAY_CTRL
#if defined(CONFIG_DISPLAY_CTRL_NRST_GPIO) || defined(CONFIG_DISPLAY_CTRL_NRST_HAL)
	if (display_ctrl_nrst_init() < 0) {
		LOG_WRN("Shared nRST init failed (hw reset unavailable)");
	} else {
		/* Pulse nRST before any driver init — this resets both the
		 * display controller and the IO card (shared reset line). */
		display_ctrl_hw_reset();
	}
#endif
#endif

#if DT_NODE_EXISTS(DT_NODELABEL(i2c2))
	/* ---- Analog I/O Card Autodetect ---- */
	{
		const struct device *card_i2c = DEVICE_DT_GET(DT_NODELABEL(i2c2));

		if (!device_is_ready(card_i2c)) {
			LOG_ERR("Card I2C bus (i2c2) not ready");
		} else {
			int cm_ret = card_manager_init(card_i2c);

			if (cm_ret < 0) {
				LOG_WRN("Card manager init error: %d", cm_ret);
			}
			/* Individual result is logged by card_manager */
		}
	}
#endif

#ifdef CONFIG_DISPLAY_CTRL
	/* ---- Display Controller (LEDs, Buttons, 7-Segment) Setup ---- */
	/* Register IO card re-init callback for runtime nRST resets */
#if defined(CONFIG_DISPLAY_CTRL_NRST_GPIO) || defined(CONFIG_DISPLAY_CTRL_NRST_HAL)
	display_ctrl_register_nrst_callback(nrst_card_rescan_cb, NULL);
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
			display_ctrl_boot_animation();
			display_ctrl_loading_animation_start();
			display_ctrl_show_status("  FPGA");
		}
	}
#endif

	/* ---- Wait for FPGA to be ready before network operations ---- */
	if (fpga_hal_wait_ready(30000) < 0) {
		LOG_ERR("FPGA not ready after 30s - network services will not start");
		return -1;
	}

#ifdef CONFIG_SD_CONFIG
	/* ---- Load configuration from SD card (now that FPGA is ready) ---- */
	bool config_loaded = false;

	if (sd_config_is_ready()) {
		int cfg_ret = sd_config_load();
		if (cfg_ret == -ENOENT) {
			LOG_INF("No config file on SD card, using defaults");
			/* Save defaults to create the file */
			sd_config_save();
			config_loaded = true;
		} else if (cfg_ret < 0) {
			LOG_ERR("Failed to load config from SD: %d", cfg_ret);
		} else {
			LOG_INF("Configuration loaded from SD card");
			config_loaded = true;
		}
	}
#else
	bool config_loaded = false;
#endif

#ifdef CONFIG_FLASH_CONFIG
	/* ---- Fall back to flash config if SD didn't load ---- */
	if (!config_loaded) {
		int fcfg_ret = flash_config_load();
		if (fcfg_ret == -ENOENT) {
			LOG_INF("No config in flash, using defaults");
			flash_config_save();
		} else if (fcfg_ret < 0) {
			LOG_ERR("Failed to load config from flash: %d",
				fcfg_ret);
		} else {
			LOG_INF("Configuration loaded from SPI flash");
		}
	}
#endif

	/* Apply configured hostname to network stack */
	char hostname_buf[AES67_NODE_ID_MAX];
	aes67_config_build_hostname(hostname_buf, sizeof(hostname_buf));
	net_hostname_set(hostname_buf, strlen(hostname_buf));
	LOG_INF("Hostname set to: %s", hostname_buf);

	/* ---- Network Initialization (after FPGA ready) ---- */
	struct net_if *iface = net_if_get_default();
	if (!iface) {
		LOG_ERR("No network interface found");
		return -1;
	}

	g_iface = iface;

	fpga_hal_register_recover_cb(fpga_reconfigure, NULL);

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
#ifdef CONFIG_SPI_FLASH_LITESPI
	fw_update_init();
#endif
	int web_ret = webserver_start();
	if (web_ret < 0) {
		LOG_ERR("Failed to start HTTP server: %d", web_ret);
	}

#ifdef CONFIG_RTSP
	/* ---- Start RAVENNA RTSP server ---- */
#ifdef CONFIG_RTSP_AUTO_START
	int rtsp_ret = rtsp_server_start();
	if (rtsp_ret < 0) {
		LOG_ERR("Failed to start RTSP server: %d", rtsp_ret);
	}
#else
	LOG_INF("RTSP auto-start disabled, use 'rtsp start' to enable");
#endif
#endif

#ifdef CONFIG_MDNS_SD
	/* ---- Start RAVENNA mDNS/DNS-SD service advertisement ---- */
	int mdns_ret = mdns_sd_start();
	if (mdns_ret < 0) {
		LOG_ERR("Failed to start mDNS/DNS-SD: %d", mdns_ret);
	}
#endif

	/* ---- Start PPB measurement / PLL correction thread ---- */
	fpga_poll_start(dhcp_restart);

	LOG_INF("System ready");
	return 0;
}
