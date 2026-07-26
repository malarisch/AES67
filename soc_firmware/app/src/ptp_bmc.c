/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: Apache-2.0
 *
 * PTPv2 BMC — thin SoC shim.
 *
 * The IEEE 1588 Best Master Clock algorithm now runs inside the FPGA.
 * This module is responsible only for:
 *   - pushing configuration to the FPGA (GM quality, log intervals,
 *     time source) and keeping it in sync with the runtime config;
 *   - exposing the FPGA BMA result (role + leader ID) to the rest of
 *     the firmware by polling the status CSR;
 *   - firing a change callback when the observed role transitions;
 *   - proxy-joining the PTP multicast group via IGMP so snooping
 *     switches forward 224.0.1.129 to the FPGA.
 *
 * No Announce parsing, no foreign-master table, no multicast socket.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/igmp.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include "ptp_bmc.h"
#include "aes67_config.h"
#include "../drivers/fpga_hal/fpga_hal.h"

LOG_MODULE_REGISTER(ptp_bmc, LOG_LEVEL_INF);

/* ---- Poll thread ---- */
/* The poll loop runs the role-change callback (display text + LED writes
 * through the FPGA HAL) and FPGA status reads on this stack — over the
 * SPI bridge on the external-MCU build that chain is deep. */
#define PTP_POLL_STACK_SIZE   2048
#define PTP_POLL_PRIORITY     K_PRIO_PREEMPT(8)
#define PTP_POLL_PERIOD_MS    250

K_THREAD_STACK_DEFINE(ptp_poll_stack, PTP_POLL_STACK_SIZE);
static struct k_thread ptp_poll_thread_data;

/* ---- State ---- */
static uint8_t my_clock_id[8];
static struct ptp_announce_dataset my_dataset;
static ptp_bmc_change_cb_t change_cb;
static enum ptp_bmc_role last_role = PTP_ROLE_LISTENING;

/* Statically initialised: the DHCP-bound handler in main.c notifies us
 * unconditionally, and can fire before ptp_bmc_start() (or without it ever
 * running, in CONFIG_AES67_PTP_SOFTWARE builds) — a k_sem_give on an
 * uninitialised k_sem walks a NULL wait queue and faults. */
static K_SEM_DEFINE(ip_ready_sem, 0, 1);
static bool started;
static struct net_if *ptp_iface;

/* PTP primary multicast group 224.0.1.129 (Announce/Sync/Delay). */
static const struct in_addr ptp_mcast_grp = { { { 224, 0, 1, 129 } } };

/*
 * Proxy IGMP join for the FPGA.
 *
 * The PTP event/general traffic terminates in the FPGA data plane — the
 * Zephyr stack never opens a socket for it.  IGMP-snooping switches only
 * forward 224.0.1.129 to ports from which they saw a membership report,
 * so the SoC must join the group on the FPGA's behalf.
 *
 * With `rejoin`, leave first so a fresh unsolicited report goes out even
 * if the group is still in the local table (net_ipv4_igmp_join() returns
 * -EALREADY without emitting anything otherwise) — used after link-up.
 */
static void ptp_proxy_igmp_join(bool rejoin)
{
	int ret;

	if (!ptp_iface) {
		return;
	}

	if (rejoin) {
		(void)net_ipv4_igmp_leave(ptp_iface, &ptp_mcast_grp);
	}

	ret = net_ipv4_igmp_join(ptp_iface, &ptp_mcast_grp, NULL);
	if (ret < 0 && ret != -EALREADY) {
		LOG_WRN("PTP: IGMP join 224.0.1.129 failed: %d", ret);
	} else if (ret == 0) {
		LOG_INF("PTP: IGMP joined 224.0.1.129 (proxy for FPGA)");
	}
}

/* ---- Derive EUI-64 clock identity from MAC ---- */
static void derive_clock_id(const uint8_t mac[6], uint8_t out[8])
{
	out[0] = mac[0] ^ 0x02;
	out[1] = mac[1];
	out[2] = mac[2];
	out[3] = 0xFF;
	out[4] = 0xFE;
	out[5] = mac[3];
	out[6] = mac[4];
	out[7] = mac[5];
}

/* ---- Push own dataset + log intervals + time source to FPGA ---- */
static void push_config_to_fpga(void)
{
	aes67_config_lock();
	const struct aes67_device_config *cfg = aes67_config_get();

	uint8_t pri1 = cfg->ptp_priority1;
	uint8_t pri2 = cfg->ptp_priority2;
	uint8_t cc   = cfg->ptp_clock_class;
	uint8_t acc  = cfg->ptp_clock_accuracy;
	int8_t  log_sync = cfg->ptp_log_sync_interval;
	int8_t  log_ann  = cfg->ptp_log_announce_interval;
	int32_t asym_ns  = cfg->ptp_delay_asymmetry_ns;
	aes67_config_unlock();

	my_dataset.gm_priority1 = pri1;
	my_dataset.gm_priority2 = pri2;
	my_dataset.gm_clock_class = cc;
	my_dataset.gm_clock_accuracy = acc;
	memcpy(my_dataset.gm_identity, my_clock_id, 8);
	memcpy(my_dataset.sender_clock_id, my_clock_id, 8);
	my_dataset.time_source = PTP_TIME_SRC_INTERNAL_OSC;

	int ret;

	ret = fpga_hal_write_ptp_gm_quality(pri1, pri2, cc, acc);
	if (ret < 0) {
		LOG_ERR("PTP: write GM quality failed: %d", ret);
	}

	ret = fpga_hal_write_ptp_config(PTP_TIME_SRC_INTERNAL_OSC,
					log_sync, log_ann);
	if (ret < 0) {
		LOG_ERR("PTP: write PTP config failed: %d", ret);
	}

	/* Persisted delayAsymmetry -> FPGA parser (read-modify-write keeps
	 * the other live-tunable fields untouched). */
	struct fpga_hal_ptp_tuning tuning;

	fpga_hal_read_ptp_tuning(&tuning);
	if (tuning.delay_asymmetry_ns != asym_ns) {
		tuning.delay_asymmetry_ns = asym_ns;
		ret = fpga_hal_write_ptp_tuning(&tuning);
		if (ret < 0) {
			LOG_ERR("PTP: write delayAsymmetry failed: %d", ret);
		}
	}

	LOG_INF("PTP: pushed config — pri1=%u pri2=%u class=%u acc=0x%02x "
		"logSync=%d logAnn=%d asym=%dns",
		pri1, pri2, cc, acc, log_sync, log_ann, asym_ns);
}

/* ---- Poll thread: watches FPGA status for role changes ---- */
static void ptp_poll_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	/* Block until IP is ready (matches old API contract). */
	k_sem_take(&ip_ready_sem, K_FOREVER);

	ptp_proxy_igmp_join(false);

	while (1) {
		enum ptp_bmc_role role = ptp_bmc_get_role();

		if (role != last_role) {
			LOG_INF("PTP: FPGA role changed %d -> %d",
				last_role, role);
			last_role = role;
			if (change_cb) {
				change_cb(role);
			}
		}

		k_msleep(PTP_POLL_PERIOD_MS);
	}
}

/* ================================================================
 * Public API
 * ================================================================ */

int ptp_bmc_start(struct net_if *iface)
{
	if (started) {
		return 0;
	}

	struct net_linkaddr *ll = net_if_get_link_addr(iface);
	if (!ll || ll->len < 6) {
		LOG_ERR("PTP: no MAC on interface");
		return -EINVAL;
	}

	ptp_iface = iface;
	derive_clock_id(ll->addr, my_clock_id);
	LOG_INF("PTP: clock identity %02x%02x%02x%02x%02x%02x%02x%02x",
		my_clock_id[0], my_clock_id[1], my_clock_id[2], my_clock_id[3],
		my_clock_id[4], my_clock_id[5], my_clock_id[6], my_clock_id[7]);

	push_config_to_fpga();

	k_thread_create(&ptp_poll_thread_data, ptp_poll_stack,
			K_THREAD_STACK_SIZEOF(ptp_poll_stack),
			ptp_poll_thread, NULL, NULL, NULL,
			PTP_POLL_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&ptp_poll_thread_data, "ptp_poll");

	started = true;
	return 0;
}

void ptp_bmc_register_change_cb(ptp_bmc_change_cb_t cb)
{
	change_cb = cb;
}

void ptp_bmc_notify_ip_ready(void)
{
	k_sem_give(&ip_ready_sem);
}

void ptp_bmc_notify_fpga_ready(void)
{
	/* No-op unless the FPGA BMC is in use (skipped in software-PTP mode). */
	if (started) {
		push_config_to_fpga();
	}
}

void ptp_bmc_notify_link_up(void)
{
	/* Re-announce the proxy IGMP membership (see ptp_proxy_igmp_join):
	 * the switch's snooping table is stale after a link bounce. */
	if (started) {
		ptp_proxy_igmp_join(true);
	}
}

enum ptp_bmc_role ptp_bmc_get_role(void)
{
	uint32_t s = fpga_hal_read_status();

	if (s & FPGA_HAL_PTP_IS_LEADER) {
		return PTP_ROLE_LEADER;
	}
	if (s & FPGA_HAL_PTP_IS_FOLLOWER) {
		return PTP_ROLE_FOLLOWER;
	}
	return PTP_ROLE_LISTENING;
}

int ptp_bmc_get_best_master_id(uint8_t out[8])
{
	if (!fpga_hal_read_ptp_leader_id(out)) {
		return -ENOENT;
	}
	return 0;
}

const struct ptp_announce_dataset *ptp_bmc_get_foreign_masters(int *count)
{
	/* FPGA BMA does not expose a foreign-master list to the SoC. */
	if (count) {
		*count = 0;
	}
	return NULL;
}

void ptp_bmc_get_clock_identity(uint8_t out[8])
{
	memcpy(out, my_clock_id, 8);
}

void ptp_bmc_update_own_dataset(void)
{
	push_config_to_fpga();
}

const struct ptp_announce_dataset *ptp_bmc_get_own_dataset(void)
{
	return &my_dataset;
}
