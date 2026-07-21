#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/net_event.h>
#include <zephyr/net/dhcpv4.h>
#include <zephyr/net/hostname.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <stdio.h>

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
#ifdef CONFIG_AES67_FPGA_JTAG
#include "../drivers/fpga_jtag/fpga_jtag.h"
#endif
#include "fpga_regs.h"
#include "fpga_poll.h"
#include "pll_ctrl.h"
#include "ui_display.h"
#include "ptp_bmc.h"
#include "ptp_ctrl.h"
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

	/* A reset FPGA comes back with every domain held (reset CSR = all-ones);
	 * the node is already configured, so release them all to resume operation. */
	fpga_hal_set_resets(FPGA_HAL_RESET_ALL, false);

	ptp_bmc_notify_fpga_ready();

	LOG_INF("FPGA recovery complete");
}

/* ---- BMC state change callback ---- */

/* Unused when PTP runs in software (Zephyr stack), where the FPGA BMC is off. */
static __maybe_unused void on_bmc_change(enum ptp_bmc_role new_role)
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
			if (ptp_ctrl_wallclock_locked()) {
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
			if (ptp_ctrl_wallclock_locked()) {
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

/* ---- MAC address derived from the product serial number ----
 *
 * Applies the configured serial's MAC to the interface. Callable more than
 * once: if the address already matches it does nothing, so re-running it
 * after a config reload never bounces a working interface. A link address
 * can only be changed while the interface is not running, so the interface
 * is taken down around the change when necessary.
 */
static void apply_serial_mac(struct net_if *iface)
{
	struct net_linkaddr *cur;
	uint8_t mac[6];
	bool was_up;
	int ret;

	if (iface == NULL) {
		return;
	}

	aes67_config_build_mac(mac);

	cur = net_if_get_link_addr(iface);
	if (cur != NULL && cur->len == sizeof(mac) &&
	    memcmp(cur->addr, mac, sizeof(mac)) == 0) {
		return;		/* already correct */
	}

	was_up = net_if_is_admin_up(iface);
	if (was_up) {
		net_if_down(iface);
	}

	ret = net_if_set_link_addr(iface, mac, sizeof(mac), NET_LINK_ETHERNET);
	if (ret < 0) {
		LOG_WRN("Could not apply serial-derived MAC (err %d); keeping "
			"the driver default", ret);
	} else {
		LOG_INF("MAC %02X:%02X:%02X:%02X:%02X:%02X (from serial \"%s\")",
			mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
			aes67_config_get()->serial);
	}

	if (was_up) {
		net_if_up(iface);
	}
}

/* The PTP stack derives its clock identity (EUI-64) from the interface MAC
 * when it initialises, so the serial-derived address has to be in place
 * before that happens: after the network stack has created the interfaces
 * (POST_KERNEL/CONFIG_NET_INIT_PRIO) and before ptp_init()
 * (APPLICATION/CONFIG_PTP_INIT_PRIO). The stored config is loaded earlier
 * still, by flash_config's own pre-network SYS_INIT. */
#define AES67_MAC_INIT_PRIO 50

#if defined(CONFIG_PTP) && defined(CONFIG_PTP_INIT_PRIO)
BUILD_ASSERT(AES67_MAC_INIT_PRIO < CONFIG_PTP_INIT_PRIO,
	     "the MAC must be final before PTP derives its clock identity");
#endif

static int aes67_mac_init(void)
{
	apply_serial_mac(net_if_get_default());
	return 0;
}

SYS_INIT(aes67_mac_init, APPLICATION, AES67_MAC_INIT_PRIO);

#ifdef CONFIG_AES67_FPGA_JTAG_BOOT_LOAD
#ifdef CONFIG_DISPLAY_CTRL
/* Show the bitstream-upload progress on the 7-segment panel as "UP  NN".
 * Called from inside the JTAG shift (clock stalled), so keep it cheap and
 * only refresh every 5 %. */
static void jtag_upload_progress(uint8_t percent, void *ctx)
{
	ARG_UNUSED(ctx);

	if (percent % 5 != 0 && percent != 100) {
		return;
	}
	if (display_ctrl_ready()) {
		char buf[7];

		snprintf(buf, sizeof(buf), "UP%4u", (unsigned int)percent);
		display_ctrl_show_status(buf);
	}
}
#endif /* CONFIG_DISPLAY_CTRL */

/* Configure the FPGA over JTAG at boot. Runs after the display is up so the
 * upload can show progress; skips the upload if the FPGA already holds this
 * bitstream (USERCODE match). */
static void fpga_jtag_bringup(void)
{
	fpga_jtag_progress_cb cb = NULL;

#ifdef CONFIG_DISPLAY_CTRL
	cb = jtag_upload_progress;
#endif

	int ret = fpga_jtag_boot_load(cb, NULL);

	if (ret < 0) {
		LOG_ERR("FPGA JTAG configuration failed (err %d); the FPGA will "
			"not respond over spibone", ret);
#ifdef CONFIG_DISPLAY_CTRL
		if (display_ctrl_ready()) {
			display_ctrl_show_status("FPGAER");
		}
#endif
		return;
	}

	if (ret == 1) {
		LOG_INF("FPGA already configured — bitstream upload skipped");
	} else {
		LOG_INF("FPGA configured over JTAG");
	}
#ifdef CONFIG_DISPLAY_CTRL
	if (display_ctrl_ready()) {
		display_ctrl_show_status("  FPGA");
	}
#endif
}
#endif /* CONFIG_AES67_FPGA_JTAG_BOOT_LOAD */

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

#ifdef CONFIG_AES67_FPGA_JTAG_BOOT_LOAD
	/* ---- Configure the FPGA over JTAG (display is up, shows progress) ----
	 * Scans the chain, skips the upload if the FPGA already holds this
	 * bitstream, otherwise loads it with a 7-segment progress bar. */
	fpga_jtag_bringup();
#endif

#if defined(CONFIG_FPGA_HAL_SPI) && defined(CONFIG_AES67_SPIBONE_PROBE)
	/* ---- Verify the Wishbone bus answers now that the FPGA is up ---- */
	fpga_hal_spibone_probe();
#endif

	/* ---- Wait for FPGA to be ready before network operations ---- */
	if (fpga_hal_wait_ready(30000) < 0) {
		LOG_ERR("FPGA not ready after 30s - network services will not start");
		return -1;
	}

#if DT_NODE_EXISTS(DT_NODELABEL(i2c2))
	/* ---- Analog I/O Card Autodetect (depends on the FPGA) ----
	 * The AD/DA cards are clocked by the FPGA, so they stay silent on I2C
	 * until it is configured and answering — which the wait_ready above
	 * has just confirmed. card_manager_init() releases the ADDA nRST and
	 * scans the bus, so it must run here, not during early init. */
#ifdef CONFIG_DISPLAY_CTRL
	if (display_ctrl_ready()) {
		display_ctrl_show_status(" CARDS");
	}
#endif
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

	/* Re-assert the serial-derived MAC. Normally a no-op — aes67_mac_init()
	 * already applied it before the PTP stack started — but the stored
	 * config may have been reloaded above (e.g. from SD) with a different
	 * serial. */
	apply_serial_mac(iface);

	fpga_write_mac_address(iface);

	net_mgmt_init_event_callback(&dhcp_cb, on_dhcp_bound,
				     NET_EVENT_IPV4_DHCP_BOUND);
	net_mgmt_add_event_callback(&dhcp_cb);

#ifdef CONFIG_DISPLAY_CTRL
	if (display_ctrl_ready()) {
		display_ctrl_show_status("  LINK");
	}
#endif

	/* The FPGA powers up with every module held in reset (reset CSR = all-ones).
	 * Release ALL domains in a single write, before the link comes up — the
	 * equivalent of the manual `devmem <reset_csr> 0` that is known to work.
	 * Do NOT stage this: taking a domain (notably audio TX/RX) out of reset
	 * *after* the Ethernet datapath is already live bounces the link and wedges
	 * the buffer-bridge TX — eth_tx_done stops asserting, so DHCP TX times out.
	 * Bring everything up at once and never touch the reset CSR again while
	 * traffic flows. */
	fpga_hal_set_resets(FPGA_HAL_RESET_ALL, false);

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

	/* ---- Start PTP ---- */
	/* All reset domains (PTP + audio included) were already released together
	 * before the link came up; see the note there on why staging is unsafe. */
#ifdef CONFIG_AES67_PTP_SOFTWARE
	/* Software PTP: Zephyr's IEEE 1588 stack (CONFIG_PTP) auto-starts via
	 * SYS_INIT and disciplines the FPGA wallclock through the aes67 PHC; the
	 * FPGA hardware BMC is not used. Push the stored PTP config (priorities,
	 * clock quality, log intervals) into the stack — it booted with its
	 * Kconfig defaults. */
	LOG_INF("PTP: software stack (Zephyr CONFIG_PTP) disciplining FPGA wallclock");
	ptp_ctrl_apply_config();
	/* ptp_bmc is not running: the fpga_poll thread samples the Zephyr
	 * stack's role and fires the same display handler on changes. */
	fpga_poll_register_role_change_cb(on_bmc_change);
#else
	ptp_bmc_register_change_cb(on_bmc_change);
	int bmc_ret = ptp_bmc_start(iface);
	if (bmc_ret < 0) {
		LOG_ERR("Failed to start PTP BMC: %d", bmc_ret);
	}
#endif

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
