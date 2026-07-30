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
#include <zephyr/sys/reboot.h>
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
#include "aes67_conn.h"
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
#ifdef CONFIG_NMOS
#include "nmos/nmos.h"
#endif
#ifdef CONFIG_AES67_PTP_SOFTWARE
#include "../drivers/eth_litex/eth_litex.h"
#include <zephyr/net/ptp.h>  /* ptp_start() — application-managed stack start */
#endif

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* Forward declarations */
static void fpga_reconfigure(void *user_data);

/* ---- Boot faults ----
 *
 * Anything the node cannot do its job without stops the boot right here
 * instead of limping on half-configured: a node that "booted fine" but has no
 * FPGA bus, no MCLK, no MAC in the FPGA or no PTP is far harder to diagnose
 * than one that says what broke and stays put.
 *
 * The analog outputs are forced into their safe state (the failing step may
 * have left the converters unclocked or live), the panel shows @p code and the
 * reason is repeated on the console forever. The shell and the JTAG health
 * monitor keep running for a post-mortem; nothing else is started.
 *
 * @p code is a 6-character 7-segment string (0-9, A-U, X, Y, space, minus).
 */
static FUNC_NORETURN void boot_fatal(const char *code, const char *what, int err)
{
#if DT_NODE_EXISTS(DT_NODELABEL(i2c2))
	{
		const struct device *i2c = DEVICE_DT_GET(DT_NODELABEL(i2c2));

		if (device_is_ready(i2c)) {
			card_manager_early_mute(i2c);
		}
	}
#endif

#ifdef CONFIG_DISPLAY_CTRL
	if (display_ctrl_ready()) {
		display_ctrl_stop_status_cycle();
		display_ctrl_loading_animation_stop();
		display_ctrl_show_status(code);
	}
#endif

	while (1) {
		LOG_ERR("BOOT FAILED [%s]: %s (err %d) - node stopped", code,
			what, err);
		k_sleep(K_SECONDS(10));
	}
}

static struct net_if *g_iface;
static struct in_addr g_my_ip;
static bool g_ip_valid;
static bool g_ip_is_ll;
/* The node's address is no longer on the interface (lease lost / link went
 * down) but is kept as the announced identity until the DHCP grace period
 * has run out. */
static bool g_ip_stale;
static bool g_dhcp_running;

/* ---- Node address: DHCP first, link-local fallback after a grace period ----
 *
 * A deliberately dumb state machine:
 *
 *   boot -> link up -> DHCP start -> no lease after AES67_DHCP_GRACE_MS?
 *        -> put the link-local 169.254.x.y address (RFC 3927, MAC-derived,
 *           reboot-stable) on the interface and run on it; DHCP keeps trying
 *        -> a lease binds (any time): switch to it, take the link-local
 *           address OFF the interface again
 *   link down -> back to start: the identity is kept, and the next link-up
 *        re-arms the full grace period before link-local is (re)considered.
 *
 * Until the first address exists the node is silent: every service
 * (SAP/RTSP/mDNS/streams) gates on the ip-ready notification, and the stack
 * drops stray transmissions for lack of a source address. Adopting the
 * link-local address any earlier just made those services start up and act
 * on a config that is wrong the moment the lease lands.
 *
 * Zephyr's own autoconf (CONFIG_NET_IPV4_AUTO) is deliberately not used: it
 * force-selects the ACD module, whose lock order deadlocks against any
 * address removal from another thread (see prj.conf). The trade-off is no
 * conflict detection — acceptable because the address is derived from a
 * unique MAC rather than randomly drawn.
 *
 * Applying an address is a deep call chain: an SPI write to the FPGA, a panel
 * repaint over UART, IGMP joins for the restored RX streams. It therefore runs
 * on this module's own thread — the net-mgmt event thread that used to do it
 * has a small stack, and the system workqueue is shared with the network stack
 * itself. g_my_ip/g_ip_valid/g_ip_is_ll are only ever written there.
 */
#define IPCFG_STACK_SIZE  8192
#define LL_RETRY_MS       5000

/* How long DHCP gets to (re)produce a lease before the node adopts the
 * link-local address as its identity. Counted from every link-up (boot
 * included) and from losing a lease — a cable pull must not flip the
 * announced identity to 169.254.x.y and back within seconds. */
#define AES67_DHCP_GRACE_MS 30000

static K_THREAD_STACK_DEFINE(ipcfg_stack, IPCFG_STACK_SIZE);
static struct k_work_q ipcfg_wq;
static struct k_work_delayable ll_work;
static struct k_work apply_ip_work;

static struct k_spinlock pending_ip_lock;
static struct in_addr pending_ip;
static bool pending_ip_is_ll;
static bool pending_ip_valid;

static void ll_addr_from_mac(struct in_addr *out);

#ifdef CONFIG_AES67_PTP_SOFTWARE
/* Start the Zephyr PTP stack exactly once, deferred to the FIRST node
 * address (DHCP lease or link-local fallback). Started any earlier the
 * stack cannot transmit ("src addr is unspecified" -> send failure ->
 * port FAULT -> re-init), which showed up as the BMC role flapping
 * FOLLOWER<->LISTENING once a second until an address existed. Runs on
 * the ipcfg thread via apply_node_ip(). */
static void ptp_sw_start_once(void)
{
	static bool ptp_started;
	int ret;

	if (ptp_started || !fpga_hal_ptp_in_software()) {
		return;
	}

	ret = ptp_start();
	if (ret < 0) {
		boot_fatal("E  PTP", "the software PTP stack did not start - "
			   "the node can neither follow nor be a grandmaster",
			   ret);
	}

	/* Push the stored PTP config (priorities, clock quality, log
	 * intervals) — the stack comes up with its Kconfig defaults. */
	ptp_ctrl_apply_config();
	ptp_started = true;
	LOG_INF("PTP: software stack started");
}
#endif /* CONFIG_AES67_PTP_SOFTWARE */

/* Hand a freshly obtained address to everything that needs to know about it.
 * Callable repeatedly — the switch from link-local to a DHCP lease (or to a
 * new lease) just runs it again with the new address. */
static void apply_node_ip(const struct in_addr *addr, bool link_local)
{
	const uint8_t *ip = (const uint8_t *)&addr->s_addr;
	int ret;

	g_my_ip = *addr;
	g_ip_valid = true;
	g_ip_is_ll = link_local;
	g_ip_stale = false;

	LOG_INF("Node address %u.%u.%u.%u (%s)", ip[0], ip[1], ip[2], ip[3],
		link_local ? "link-local" : "DHCP");

	ret = fpga_write_ip_address(addr);
	if (ret < 0) {
		LOG_ERR("FPGA did not take the node IP (err %d) - RTP and "
			"hardware PTP will use a stale address", ret);
	}

	ui_display_set_ip(addr);
	fpga_poll_notify_ip_valid(addr);

#ifdef CONFIG_AES67_PTP_SOFTWARE
	/* Software PTP starts with the first address (no-op afterwards and
	 * in hardware-PTP mode). */
	ptp_sw_start_once();
#endif
	ptp_bmc_notify_ip_ready();

#ifdef CONFIG_DISPLAY_CTRL
	if (display_ctrl_ready()) {
		display_ctrl_show_status("L  PTP");
		display_ctrl_update_status_cycle_ip(addr);
	}
#endif

	aes67_conn_notify_ip_ready(addr);
	sap_sdp_notify_ip_ready(addr);

#ifdef CONFIG_NMOS
	nmos_notify_ip_ready(addr);
#endif

#ifdef CONFIG_RTSP
	rtsp_notify_ip_ready(addr);
#endif

	/* Steady state carries exactly one address: once a DHCP lease is the
	 * identity, the link-local fallback comes off the interface again.
	 * (Both exist only for the instant of the switchover — Zephyr's DHCP
	 * client adds its address on ACK before we run, which is why
	 * NET_IF_UNICAST_IPV4_ADDR_COUNT is 2.) */
	if (!link_local) {
		struct in_addr ll;
		struct net_if *found = NULL;

		ll_addr_from_mac(&ll);
		if (net_if_ipv4_addr_lookup(&ll, &found) != NULL &&
		    found == g_iface) {
			net_if_ipv4_addr_rm(g_iface, &ll);
			LOG_INF("Link-local address removed (lease active)");
		}
	}
}

/* Runs on the ipcfg thread, so it may call apply_node_ip() directly. */
static void apply_ip_work_fn(struct k_work *work)
{
	k_spinlock_key_t key;
	struct in_addr addr;
	bool is_ll;

	ARG_UNUSED(work);

	key = k_spin_lock(&pending_ip_lock);
	if (!pending_ip_valid) {
		k_spin_unlock(&pending_ip_lock, key);
		return;
	}
	addr = pending_ip;
	is_ll = pending_ip_is_ll;
	pending_ip_valid = false;
	k_spin_unlock(&pending_ip_lock, key);

	apply_node_ip(&addr, is_ll);
}

/* Hand an address over to the ipcfg thread (callable from any context). */
static void node_ip_set(const struct in_addr *addr, bool link_local)
{
	k_spinlock_key_t key = k_spin_lock(&pending_ip_lock);

	pending_ip = *addr;
	pending_ip_is_ll = link_local;
	pending_ip_valid = true;
	k_spin_unlock(&pending_ip_lock, key);

	k_work_submit_to_queue(&ipcfg_wq, &apply_ip_work);
}

/* RFC 3927 §2.1 picks 169.254.1.0 – 169.254.254.255 pseudo-randomly from a
 * stable per-node identifier; the MAC is exactly that, and it keeps the
 * address the same across reboots. */
static void ll_addr_from_mac(struct in_addr *out)
{
	uint8_t mac[6];
	uint8_t *b = (uint8_t *)&out->s_addr;
	uint32_t hash = 2166136261u;		/* FNV-1a */
	uint32_t idx;

	aes67_config_build_mac(mac);

	for (size_t i = 0; i < sizeof(mac); i++) {
		hash = (hash ^ mac[i]) * 16777619u;
	}

	idx = hash % (254u * 256u);
	b[0] = 169;
	b[1] = 254;
	b[2] = 1 + (uint8_t)(idx / 256u);
	b[3] = (uint8_t)(idx % 256u);
}

/* Put the fallback address on the interface (idempotent). It is a manual
 * address, so the stack leaves it alone across carrier transitions — it is
 * simply always there, ready for whenever DHCP is not. */
static bool ll_addr_ensure(struct in_addr *out)
{
	struct in_addr netmask = { { { 255, 255, 0, 0 } } };
	struct net_if *found = NULL;
	struct in_addr ll;

	ll_addr_from_mac(&ll);
	*out = ll;

	if (net_if_ipv4_addr_lookup(&ll, &found) != NULL && found == g_iface) {
		return true;
	}

	if (net_if_ipv4_addr_add(g_iface, &ll, NET_ADDR_MANUAL, 0) == NULL) {
		LOG_ERR("Could not add the link-local address %u.%u.%u.%u - "
			"out of IPv4 address slots?",
			ll.s4_addr[0], ll.s4_addr[1], ll.s4_addr[2],
			ll.s4_addr[3]);
		return false;
	}

	net_if_ipv4_set_netmask_by_addr(g_iface, &ll, &netmask);

	LOG_INF("Link-local address %u.%u.%u.%u configured (derived from the "
		"MAC)", ll.s4_addr[0], ll.s4_addr[1], ll.s4_addr[2],
		ll.s4_addr[3]);
	return true;
}

/* No lease: make sure the fallback address exists and run on it. */
static void ll_work_fn(struct k_work *work)
{
	struct in_addr ll;

	ARG_UNUSED(work);

	if (!g_iface || (g_ip_valid && !g_ip_is_ll && !g_ip_stale)) {
		return;		/* running on a live lease — nothing to do */
	}

	if (!net_if_is_up(g_iface)) {
		/* No carrier: nobody can see an identity change, and DHCP has
		 * no chance to answer anyway. Stay as we are — link-up runs
		 * dhcp_restart(), which re-arms the grace period. */
		return;
	}

	if (!ll_addr_ensure(&ll)) {
		k_work_reschedule_for_queue(&ipcfg_wq, &ll_work,
					    K_MSEC(LL_RETRY_MS));
		return;
	}

	if (g_ip_is_ll && net_ipv4_addr_cmp(&g_my_ip, &ll)) {
		return;		/* already on it */
	}

	LOG_WRN("No DHCP lease - running on the link-local address (the DHCP "
		"client keeps trying)");
	apply_node_ip(&ll, true);
}

/* Fall back to the link-local address; used at boot and whenever the current
 * address disappears. */
static void ll_arm(k_timeout_t delay)
{
	k_work_reschedule_for_queue(&ipcfg_wq, &ll_work, delay);
}

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
	if (g_ip_valid && !g_ip_is_ll) {
		g_ip_stale = true;	/* keep the identity through the grace period */
	}
	net_dhcpv4_start(g_iface);
	g_dhcp_running = true;

	/* Grace period counts from link-up: the (possibly different) network
	 * gets AES67_DHCP_GRACE_MS to hand out a lease before the node's
	 * announced identity falls back to the link-local address. */
	ll_arm(K_MSEC(AES67_DHCP_GRACE_MS));
}

static struct net_mgmt_event_callback ipv4_cb;

static void on_dhcp_bound(struct net_mgmt_event_callback *cb)
{
	const struct net_if_dhcpv4 *dhcpv4 =
		(const struct net_if_dhcpv4 *)cb->info;

	if (!dhcpv4) {
		LOG_WRN("DHCP bound but no info payload");
		return;
	}

	const uint8_t *ip = (const uint8_t *)&dhcpv4->requested_ip.s_addr;

	LOG_INF("DHCP bound: %u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);

	if (g_ip_is_ll) {
		LOG_INF("Leaving the link-local address for the DHCP lease");
	}

	node_ip_set(&dhcpv4->requested_ip, false);
}

/* The DHCP address went away — lease expired/released, or the stack dropped
 * it on link-down. Keep it as the announced identity (FPGA registers, SAP,
 * RTSP, display all stay put) and give DHCP the grace period to bring a
 * lease back; only then fall back to link-local. */
static void on_ipv4_addr_del(const struct in_addr *addr)
{
	if (!g_ip_valid || g_ip_is_ll || !net_ipv4_addr_cmp(addr, &g_my_ip)) {
		return;
	}

	LOG_WRN("DHCP address removed (lease lost or link down) - keeping the "
		"identity for the %u s DHCP grace period",
		AES67_DHCP_GRACE_MS / 1000U);

	g_ip_stale = true;
	ll_arm(K_MSEC(AES67_DHCP_GRACE_MS));
}

static void on_ipv4_event(struct net_mgmt_event_callback *cb,
			  uint64_t mgmt_event,
			  struct net_if *iface)
{
	ARG_UNUSED(iface);

	switch (mgmt_event) {
	case NET_EVENT_IPV4_DHCP_BOUND:
		on_dhcp_bound(cb);
		break;
	case NET_EVENT_IPV4_ADDR_DEL:
		if (cb->info_length == sizeof(struct in_addr)) {
			on_ipv4_addr_del((const struct in_addr *)cb->info);
		}
		break;
	default:
		break;
	}
}

/* ---- FPGA Recovery Callback ---- */

static void fpga_reconfigure(void *user_data)
{
	ARG_UNUSED(user_data);

	LOG_INF("FPGA recovery: re-writing configuration registers");

	if (g_iface) {
		if (fpga_write_mac_address(g_iface) < 0) {
			LOG_ERR("FPGA recovery: MAC write failed - the node "
				"will not receive its own traffic");
		}

		if (g_ip_valid && fpga_write_ip_address(&g_my_ip) < 0) {
			LOG_ERR("FPGA recovery: IP write failed");
		}
	}

	/* The bitstream (and with it the static build configuration) may have
	 * been re-uploaded — refresh the cache before anything consults it. */
	if (fpga_hal_syscfg_load() < 0) {
		LOG_ERR("FPGA recovery: system_cfg re-read failed - keeping "
			"the previous configuration");
	}

	/* A reset FPGA comes back with every domain held (reset CSR = all-ones);
	 * the node is already configured, so release them all to resume operation. */
	int rst_ret = fpga_hal_set_resets(FPGA_HAL_RESET_ALL, false);

	if (rst_ret < 0) {
		LOG_ERR("FPGA recovery: reset domains stayed held (err %d) - "
			"the data plane is dead", rst_ret);
	}

	ptp_bmc_notify_fpga_ready();

	LOG_INF("FPGA recovery complete");
}

/* ---- BMC state change callback ---- */

/* Unused when PTP runs in software (Zephyr stack), where the FPGA BMC is off. */
static __maybe_unused void on_bmc_change(enum ptp_bmc_role new_role)
{
	LOG_INF("PLL: BMC change (role=%d) - resetting PI controller", new_role);
	pll_ctrl_reset();

#ifdef CONFIG_DISPLAY_CTRL
	if (display_ctrl_ready()) {
		/* While the Ethernet link is down the panel shows "  LINK"
		 * (set by fpga_poll); role fallout from the dead link (e.g.
		 * LEADER -> LISTENING) must not repaint the status text.
		 * The role LEDs stay live. */
		bool link_up = fpga_poll_link_is_up();

		switch (new_role) {
		case PTP_ROLE_LEADER:
			display_ctrl_set_sys_led(DC_SYSLED_MSTR, DC_SYSLED_ON);
			display_ctrl_set_sys_led(DC_SYSLED_EXT, DC_SYSLED_OFF);
			/* If wallclock already locked, go online immediately;
			 * otherwise let fpga_poll handle when wc_locked fires */
			if (link_up) {
				display_ctrl_stop_status_cycle();
				if (ptp_ctrl_wallclock_locked()) {
					display_ctrl_loading_animation_stop();
					display_ctrl_start_metering();
					display_ctrl_start_status_cycle("LEADER",
						g_ip_valid ? &g_my_ip : NULL);
				}
			}
			LOG_INF("LED: Master ON, Ext OFF");
			break;
		case PTP_ROLE_FOLLOWER:
			display_ctrl_set_sys_led(DC_SYSLED_MSTR, DC_SYSLED_OFF);
			display_ctrl_set_sys_led(DC_SYSLED_EXT, DC_SYSLED_ON);
			/* If wallclock already locked, go online;
			 * otherwise show SYNCNG and let fpga_poll handle transition */
			if (link_up) {
				display_ctrl_stop_status_cycle();
				if (ptp_ctrl_wallclock_locked()) {
					display_ctrl_loading_animation_stop();
					display_ctrl_start_metering();
					display_ctrl_start_status_cycle("FOLLOW",
						g_ip_valid ? &g_my_ip : NULL);
				} else {
					display_ctrl_show_status("SYNCNG");
				}
			}
			LOG_INF("LED: Master OFF, Ext ON");
			break;
		case PTP_ROLE_LISTENING:
		default:
			display_ctrl_set_sys_led(DC_SYSLED_MSTR, DC_SYSLED_BLINK1);
			display_ctrl_set_sys_led(DC_SYSLED_EXT, DC_SYSLED_OFF);
			if (link_up) {
				display_ctrl_show_status("L  PTP");
				display_ctrl_stop_status_cycle();
			}
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
 *
 * Returns 0, or a negative errno if the interface could not be brought back
 * up — a MAC that cannot be applied is survivable (the driver default stays
 * in place), a dead interface is not.
 */
static int apply_serial_mac(struct net_if *iface)
{
	struct net_linkaddr *cur;
	uint8_t mac[6];
	bool was_up;
	int ret;

	if (iface == NULL) {
		return -ENODEV;
	}

	aes67_config_build_mac(mac);

	cur = net_if_get_link_addr(iface);
	if (cur != NULL && cur->len == sizeof(mac) &&
	    memcmp(cur->addr, mac, sizeof(mac)) == 0) {
		return 0;	/* already correct */
	}

	was_up = net_if_is_admin_up(iface);
	if (was_up) {
		net_if_down(iface);
	}

	ret = net_if_set_link_addr(iface, mac, sizeof(mac), NET_LINK_ETHERNET);
	if (ret < 0) {
		LOG_ERR("Could not apply serial-derived MAC (err %d); keeping "
			"the driver default", ret);
	} else {
		LOG_INF("MAC %02X:%02X:%02X:%02X:%02X:%02X (from serial \"%s\")",
			mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
			aes67_config_get()->serial);
	}

	if (was_up) {
		ret = net_if_up(iface);
		if (ret < 0) {
			LOG_ERR("Interface stayed down after the MAC change "
				"(err %d)", ret);
			return ret;
		}
	}

	return 0;
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
	/* Reported as a SYS_INIT failure when the interface cannot be brought
	 * back up; main() turns that into a hard stop a moment later. */
	return apply_serial_mac(net_if_get_default());
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
 * bitstream (USERCODE match).
 *
 * Returns 0 on success or a negative errno; the caller ends the boot on
 * failure, but only after the health monitor is running so a marginal upload
 * still gets its retry. */
static int fpga_jtag_bringup(void)
{
	fpga_jtag_progress_cb cb = NULL;

#ifdef CONFIG_DISPLAY_CTRL
	cb = jtag_upload_progress;
#endif

	int ret = fpga_jtag_boot_load(cb, NULL);

	if (ret < 0) {
		LOG_ERR("FPGA JTAG configuration failed (err %d)", ret);
#ifdef CONFIG_DISPLAY_CTRL
		if (display_ctrl_ready()) {
			display_ctrl_show_status("FPGAER");
		}
#endif
		return ret;
	}

	if (ret == 1) {
		LOG_INF("FPGA already configured - bitstream upload skipped");
	} else {
		LOG_INF("FPGA configured over JTAG");
	}
#ifdef CONFIG_DISPLAY_CTRL
	if (display_ctrl_ready()) {
		display_ctrl_show_status("  FPGA");
	}
#endif
	return 0;
}

#ifdef CONFIG_AES67_FPGA_JTAG_MONITOR
/* Health-monitor events (runs in the fpga_jtag_mon thread). */
static void fpga_jtag_health_event(enum fpga_jtag_event evt,
				   enum fpga_jtag_health health, void *ctx)
{
	ARG_UNUSED(ctx);

	switch (evt) {
	case FPGA_JTAG_EVT_FAULT:
		/* The configuration SRAM is corrupt (SEU) or gone entirely —
		 * the TDM/DSP path may be driving garbage into the DACs.
		 * Force the analog outputs into their safe state (relays
		 * muted, converters in reset) before the FPGA is touched. */
#if DT_NODE_EXISTS(DT_NODELABEL(i2c2))
		{
			const struct device *i2c =
				DEVICE_DT_GET(DT_NODELABEL(i2c2));

			if (device_is_ready(i2c)) {
				card_manager_early_mute(i2c);
			}
		}
#endif
#ifdef CONFIG_DISPLAY_CTRL
		if (display_ctrl_ready()) {
			display_ctrl_stop_status_cycle();
			display_ctrl_loading_animation_stop();
			display_ctrl_show_status(
				health == FPGA_JTAG_HEALTH_CRC_ERROR ?
				"CRCERR" : "FPGAER");
		}
#endif
		break;

	case FPGA_JTAG_EVT_RECOVERED:
		/* The FPGA is reconfigured but blank: resets held, MAC/IP,
		 * stream tables, PTP discipline and card clocking all gone.
		 * Reboot and let the one proven bring-up path rebuild
		 * everything — the fresh USERCODE makes the boot-time upload
		 * skip, so this costs a boot, not another 6.5 s upload. */
		LOG_ERR("FPGA reconfigured after a fault - rebooting for a "
			"clean bring-up");
		k_msleep(100);
		sys_reboot(SYS_REBOOT_COLD);
		break;

	case FPGA_JTAG_EVT_RELOAD_FAILED:
		/* Outputs stay muted; the monitor retries after a hold-off. */
#ifdef CONFIG_DISPLAY_CTRL
		if (display_ctrl_ready()) {
			display_ctrl_show_status("FPGAER");
		}
#endif
		break;
	}
}
#endif /* CONFIG_AES67_FPGA_JTAG_MONITOR */
#endif /* CONFIG_AES67_FPGA_JTAG_BOOT_LOAD */

int main(void)
{
	LOG_INF("AES67 System starting...");

#ifdef CONFIG_DISPLAY_CTRL_NRST_GPIO
	/* Shared nRST line (display controller + card LPC + DSP): release it
	 * immediately so both boot and I2C/UART communication works. Never
	 * pulse this line at boot — an LPC reset reverts its GPOs to the
	 * power-on defaults, which leave the converters live while the DSP
	 * has no clock yet (FPGA unconfigured) = loud white noise. Instead
	 * the card is put into a safe state over I2C right below. */
	{
		int nrst_ret = fpga_hal_set_adda_nrst(true);

		if (nrst_ret < 0) {
			boot_fatal("E NRST",
				   "shared nRST could not be released - display "
				   "controller and card LPC stay in reset and "
				   "the outputs cannot be muted", nrst_ret);
		}
	}
	LOG_INF("Shared nRST released (display + card LPC)");

#if DT_NODE_EXISTS(DT_NODELABEL(i2c2))
	{
		const struct device *early_i2c =
			DEVICE_DT_GET(DT_NODELABEL(i2c2));
		int mute_ret;

		/* Give the LPC time to boot, then mute the outputs and put
		 * the converters into the card-level reset — BEFORE the
		 * lengthy FPGA bitstream upload. They stay that way until
		 * the first PTP/wallclock lock (card_manager_activate_outputs
		 * from fpga_poll). */
		k_msleep(200);
		if (!device_is_ready(early_i2c)) {
			boot_fatal("E  I2C",
				   "card I2C bus (i2c2) not ready - the outputs "
				   "cannot be put into their safe state",
				   -ENODEV);
		}

		mute_ret = card_manager_early_mute(early_i2c);
		if (mute_ret < 0) {
			boot_fatal("E  I2C",
				   "early mute failed - the card may drive "
				   "garbage into the converters", mute_ret);
		}
	}
#endif
#else
	/* Dedicated ADDA reset (no display on the line): hold the card in
	 * reset from the very start. The CS5368 ADCs must not leave reset
	 * until MCLK is stable; nRST is released by card_manager_init()
	 * after the Si5351A PLL has locked and the FPGA answers. */
	{
		int nrst_ret = fpga_hal_set_adda_nrst(false);

		if (nrst_ret < 0) {
			boot_fatal("E NRST",
				   "ADDA nRST could not be asserted - the "
				   "converters are not held in reset while the "
				   "FPGA is still unconfigured", nrst_ret);
		}
	}
	LOG_INF("ADDA nRST asserted (held in reset)");
#endif

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
		/* Not fatal — the node runs on defaults — but nothing the user
		 * configures from here on will survive a reboot. */
		LOG_ERR("Flash config store unavailable (err %d): settings will "
			"NOT persist", flash_cfg_ret);
	}
#endif

#ifdef CONFIG_SI5351A
	/* ---- Si5351A Clock Generator Setup ----
	 * CLK0 is the converters' MCLK. Without it the CS5368 ADCs never come
	 * up on I2C and the audio path is dead, so a failure here ends the
	 * boot rather than producing a silent node. */
	{
		const struct device *clkgen = DEVICE_DT_GET(DT_NODELABEL(si5351a));
		int ret;

		if (!device_is_ready(clkgen)) {
			boot_fatal("E  CLK",
				   "Si5351A not ready - no MCLK for the "
				   "converters", -ENODEV);
		}

		LOG_INF("Si5351A device ready, configuring clocks...");

		ret = si5351a_set_frequency(clkgen, 0, 24576000);
		if (ret) {
			boot_fatal("E  CLK",
				   "Si5351A CLK0 (24.576 MHz MCLK) could not be "
				   "set", ret);
		}

		ret = si5351a_set_drive_strength(clkgen, 0, SI5351A_DRIVE_8MA);
		if (ret) {
			boot_fatal("E  CLK",
				   "Si5351A CLK0 drive strength could not be set",
				   ret);
		}

		LOG_INF("Si5351A clocks configured: CLK0=24.576 MHz");

		/* CS5368 ADCs require a stable MCLK before their reset is
		 * released — otherwise the I2C interface won't initialise.
		 * Give the Si5351A PLL time to lock and the clock to settle. */
		LOG_INF("Waiting 200 ms for MCLK to stabilise...");
		k_msleep(200);
	}
#endif

	/* ---- Shared nRST: prepare runtime hardware-reset capability ---- */
#ifdef CONFIG_DISPLAY_CTRL
#if defined(CONFIG_DISPLAY_CTRL_NRST_GPIO) || defined(CONFIG_DISPLAY_CTRL_NRST_HAL)
	if (display_ctrl_nrst_init() < 0) {
		LOG_WRN("Shared nRST init failed (hw reset unavailable)");
	}
#if defined(CONFIG_DISPLAY_CTRL_NRST_HAL)
	else {
		/* Legacy CSR-driven line: pulse nRST before any driver init to
		 * reset display controller + IO card together. On the GPIO
		 * (shared with the card LPC) boards this pulse must NOT happen
		 * at boot — see the early-mute block at the top of main(). */
		display_ctrl_hw_reset();
	}
#endif
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
	int jtag_ret = fpga_jtag_bringup();

#ifdef CONFIG_AES67_FPGA_JTAG_MONITOR
	/* Watch the FPGA from here on: periodic JTAG health checks (IDCODE,
	 * USERCODE, CRC_ERROR pin). Started right after configuration rather
	 * than at "system ready" so a failed boot upload keeps being retried
	 * and a fault during the remaining bring-up is already caught. On a
	 * confirmed fault the handler above mutes the outputs, the bitstream
	 * is reloaded and the node reboots into a clean bring-up. */
	int mon_ret = fpga_jtag_monitor_start(fpga_jtag_health_event,
#ifdef CONFIG_DISPLAY_CTRL
					      jtag_upload_progress,
#else
					      NULL,
#endif
					      NULL);

	if (mon_ret < 0) {
		LOG_ERR("FPGA JTAG health monitor did not start (err %d) - a "
			"configuration fault will go unnoticed", mon_ret);
	}
#endif

	/* Only now: with the monitor armed a marginal upload still gets its
	 * retry (and reboots the node on recovery). Without a configured FPGA
	 * there is nothing left to boot into. */
	if (jtag_ret < 0) {
		boot_fatal("FPGAER",
			   "FPGA JTAG configuration failed - no data plane and "
			   "no spibone bus", jtag_ret);
	}
#endif

#if defined(CONFIG_FPGA_HAL_SPI) && defined(CONFIG_AES67_SPIBONE_PROBE)
	/* ---- Verify the Wishbone bus answers now that the FPGA is up ---- */
	{
		int probe_ret = fpga_hal_spibone_probe();

		if (probe_ret < 0) {
			boot_fatal("E  BUS",
				   "spibone probe failed - the FPGA is "
				   "configured but does not answer on the "
				   "Wishbone bus", probe_ret);
		}
	}
#endif

	/* ---- Wait for FPGA to be ready before network operations ---- */
	{
		int rdy_ret = fpga_hal_wait_ready(30000);

		if (rdy_ret < 0) {
			boot_fatal("E FPGA",
				   "FPGA did not become ready within 30 s",
				   rdy_ret);
		}
	}

	/* ---- Read the FPGA's static build configuration ----
	 * The system_cfg CSRs report what the loaded bitstream was built with
	 * (hardware vs. software PTP, metering, stream limits). Everything
	 * downstream — the Ethernet drivers' PTP capability and RX-trailer
	 * parsing, the ptp_ctrl dispatch, which PTP service gets started —
	 * keys off this cache, so it MUST be loaded before the reset domains
	 * are released and the first frame can arrive. */
#if defined(CONFIG_FPGA_HAL_LITEX) || defined(CONFIG_FPGA_HAL_SPI) || \
	defined(CONFIG_FPGA_HAL_MOCK)
	{
		int cfg_ret = fpga_hal_syscfg_load();

		if (cfg_ret < 0) {
			boot_fatal("E  CFG",
				   "the FPGA system_cfg register could not be "
				   "read - cannot tell which PTP mode the "
				   "gateware was built for", cfg_ret);
		}
	}
#endif

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
		int cm_ret;

		if (!device_is_ready(card_i2c)) {
			boot_fatal("E  I2C",
				   "card I2C bus (i2c2) not ready - no card "
				   "control and no way to mute the outputs",
				   -ENODEV);
		}

		/* An empty slot is legitimate (network-only node), so a scan
		 * that finds nothing is a warning, not the end of the boot. */
		cm_ret = card_manager_init(card_i2c);
		if (cm_ret < 0) {
			LOG_WRN("Card manager init error: %d", cm_ret);
		}
		/* Individual result is logged by card_manager */
	}
#endif

#ifdef CONFIG_SD_CONFIG
	/* ---- Load configuration from SD card (now that FPGA is ready) ---- */
	bool config_loaded = false;

	if (sd_config_is_ready()) {
		int cfg_ret = sd_config_load();
		if (cfg_ret == -ENOENT) {
			int save_ret;

			LOG_INF("No config file on SD card, using defaults");
			/* Save defaults to create the file */
			save_ret = sd_config_save();
			if (save_ret < 0) {
				LOG_ERR("Could not write the default config to "
					"SD: %d", save_ret);
			}
			config_loaded = true;
		} else if (cfg_ret < 0) {
			LOG_ERR("Failed to load config from SD: %d", cfg_ret);
		} else {
			LOG_INF("Configuration loaded from SD card");
			config_loaded = true;
		}
	}
#else
	bool config_loaded __maybe_unused = false;
#endif

#ifdef CONFIG_FLASH_CONFIG
	/* ---- Fall back to flash config if SD didn't load ---- */
	if (!config_loaded) {
		int fcfg_ret = flash_config_load();
		if (fcfg_ret == -ENOENT) {
			int save_ret;

			LOG_INF("No config in flash, using defaults");
			save_ret = flash_config_save();
			if (save_ret < 0) {
				LOG_ERR("Could not write the default config to "
					"flash: %d", save_ret);
			}
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
	int hn_ret;

	aes67_config_build_hostname(hostname_buf, sizeof(hostname_buf));
	hn_ret = net_hostname_set(hostname_buf, strlen(hostname_buf));
	if (hn_ret < 0) {
		LOG_ERR("Hostname \"%s\" could not be set: %d - DHCP and mDNS "
			"will advertise the built-in default", hostname_buf,
			hn_ret);
	} else {
		LOG_INF("Hostname set to: %s", hostname_buf);
	}

	/* ---- Network Initialization (after FPGA ready) ---- */
	struct net_if *iface = net_if_get_default();

	if (!iface) {
		boot_fatal("E  NET", "no network interface - the Ethernet "
			   "driver did not come up", -ENODEV);
	}

	g_iface = iface;

	fpga_hal_register_recover_cb(fpga_reconfigure, NULL);

	/* Re-assert the serial-derived MAC. Normally a no-op — aes67_mac_init()
	 * already applied it before the PTP stack started — but the stored
	 * config may have been reloaded above (e.g. from SD) with a different
	 * serial. */
	int mac_ret = apply_serial_mac(iface);

	if (mac_ret < 0) {
		boot_fatal("E  MAC", "the interface stayed down after the MAC "
			   "change", mac_ret);
	}

	/* Without the MAC in the FPGA the hardware filter drops every frame
	 * addressed to this node — RTP and hardware PTP included. */
	mac_ret = fpga_write_mac_address(iface);
	if (mac_ret < 0) {
		boot_fatal("E  MAC", "the MAC address could not be written to "
			   "the FPGA", mac_ret);
	}

	net_mgmt_init_event_callback(&ipv4_cb, on_ipv4_event,
				     NET_EVENT_IPV4_DHCP_BOUND |
				     NET_EVENT_IPV4_ADDR_DEL);
	net_mgmt_add_event_callback(&ipv4_cb);

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
	int rst_ret = fpga_hal_set_resets(FPGA_HAL_RESET_ALL, false);

	if (rst_ret < 0) {
		boot_fatal("E  RST", "the FPGA reset domains could not be "
			   "released - MAC, PTP and audio stay held", rst_ret);
	}

	fpga_wait_for_link_up();

	k_msleep(500);

#ifdef CONFIG_DISPLAY_CTRL
	if (display_ctrl_ready()) {
		display_ctrl_show_status("  DHCP");
	}
#endif

	/* Address configuration runs on its own thread — see the node-address
	 * block at the top of this file. */
	static const struct k_work_queue_config ipcfg_wq_cfg = { .name = "ipcfg" };

	k_work_queue_init(&ipcfg_wq);
	k_work_queue_start(&ipcfg_wq, ipcfg_stack,
			   K_THREAD_STACK_SIZEOF(ipcfg_stack),
			   K_PRIO_PREEMPT(8), &ipcfg_wq_cfg);
	k_work_init(&apply_ip_work, apply_ip_work_fn);
	k_work_init_delayable(&ll_work, ll_work_fn);

	LOG_INF("Starting DHCP (%u s grace before the link-local fallback)...",
		AES67_DHCP_GRACE_MS / 1000U);
	net_dhcpv4_start(iface);
	g_dhcp_running = true;
	/* The link is already up (fpga_wait_for_link_up above), so this is the
	 * link-up edge of the state machine: DHCP gets the full grace period
	 * before ll_work adopts the link-local address. */
	ll_arm(K_MSEC(AES67_DHCP_GRACE_MS));

	/* ---- Start PTP ----
	 * Both PTP services are compiled in; which one runs is decided here,
	 * from the gateware's static build configuration (system_cfg CSRs,
	 * loaded above). All reset domains (PTP + audio included) were already
	 * released together before the link came up; see the note there on why
	 * staging is unsafe. */
#ifdef CONFIG_AES67_PTP_SOFTWARE
	if (fpga_hal_ptp_in_software()) {
		/* Software PTP: the gateware has no hardware BMC/servo; Zephyr's
		 * IEEE 1588 stack disciplines the FPGA wallclock through the
		 * aes67 PHC. The stack no longer auto-starts via SYS_INIT
		 * (CONFIG_PTP_APP_MANAGED_START) — start it now that the FPGA
		 * (and on external MCUs its configuration) is up, then push the
		 * stored PTP config (priorities, clock quality, log intervals):
		 * it comes up with its Kconfig defaults.
		 *
		 * NOT started here: at this point DHCP has only just begun and
		 * the interface has no address yet. A stack started now opens
		 * its sockets fine but every Delay_Req transmit fails ("src
		 * addr is unspecified"), faulting the port — observed as the
		 * BMC role flapping FOLLOWER<->LISTENING once a second for the
		 * whole 30 s link-local grace period. The actual ptp_start()
		 * is chained to the first node address in apply_node_ip(). */
		LOG_INF("PTP: software stack (Zephyr CONFIG_PTP) disciplining "
			"FPGA wallclock - starting with the first node address");
		/* A warm FPGA (JTAG upload skipped, firmware rebooted) still
		 * carries the previous run's rate correction — worst case the
		 * ±524287 ppb clamp. Start from a neutral frequency now, so the
		 * wallclock free-runs clean while we wait for an address; the
		 * servo takes over on its first Sync. */
		aes67_ptp_rate_reset();
		/* ptp_bmc is not running: the fpga_poll thread samples the
		 * Zephyr stack's role and fires the same display handler on
		 * changes. (Safe before ptp_start(): the stack's static
		 * datasets read as an empty ports list -> LISTENING.) */
		fpga_poll_register_role_change_cb(on_bmc_change);
	} else
#endif /* CONFIG_AES67_PTP_SOFTWARE */
	{
		/* Hardware PTP: the FPGA runs BMC + servo; ptp_bmc feeds it
		 * announce data and monitors the BMA outcome. The Zephyr PTP
		 * stack (if compiled in) is never started — no thread, no
		 * sockets, no traffic. */
		LOG_INF("PTP: FPGA hardware engine (BMC + servo)");
		ptp_bmc_register_change_cb(on_bmc_change);

		int bmc_ret = ptp_bmc_start(iface);

		if (bmc_ret < 0) {
			boot_fatal("E  PTP", "the PTP BMC did not start - the "
				   "node can neither follow nor be a "
				   "grandmaster", bmc_ret);
		}
	}

	/* ---- Start AES67 connection management + SAP announcements ---- */
	int conn_ret = aes67_conn_init(iface);

	if (conn_ret < 0) {
		boot_fatal("E CONN", "connection management did not start - no "
			   "stream setup", conn_ret);
	}

	int sap_ret = sap_sdp_start(iface);

	if (sap_ret < 0) {
		boot_fatal("E  SAP", "SAP/SDP did not start - the node would be "
			   "invisible to other AES67 devices", sap_ret);
	}

	/* ---- Start HTTP server (REST API + Web UI) ----
	 * From here on failures are loud but not fatal: the audio path is up
	 * and killing a streaming node over a missing management service is
	 * worse than running without it. */
#ifdef CONFIG_SPI_FLASH_LITESPI
	fw_update_init();
#endif
	int web_ret = webserver_start();
	if (web_ret < 0) {
		LOG_ERR("HTTP server did not start (err %d) - no web UI and no "
			"REST API", web_ret);
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

#ifdef CONFIG_NMOS
	/* ---- Arm NMOS IS-04 version bookkeeping (API is served by the
	 * HTTP server started above) ---- */
	int nmos_ret = nmos_start();
	if (nmos_ret < 0) {
		LOG_ERR("NMOS did not start (err %d) - node invisible to "
			"NMOS controllers", nmos_ret);
	}
#endif

	/* ---- Start PPB measurement / PLL correction thread ---- */
	fpga_poll_start(dhcp_restart);

	LOG_INF("System ready");
	return 0;
}
