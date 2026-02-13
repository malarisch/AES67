/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: Apache-2.0
 *
 * PTPv2 Best Master Clock (BMC) implementation for AES67.
 *
 * - Listens on 224.0.1.129:320 for PTP Announce messages
 * - Maintains a foreign-master table
 * - Runs IEEE 1588 data-set comparison to elect best master
 * - Writes result to FPGA via FMC registers 0x55 and 0x50
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/igmp.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <zephyr/sys/byteorder.h>

#include "ptp_bmc.h"
#include "../drivers/eth_fmc_basic/eth_fmc_basic.h"

LOG_MODULE_REGISTER(ptp_bmc, LOG_LEVEL_INF);

/* ---- Thread resources ---- */
#define BMC_STACK_SIZE   4096
#define BMC_PRIORITY     K_PRIO_PREEMPT(8)

K_THREAD_STACK_DEFINE(bmc_stack, BMC_STACK_SIZE);
static struct k_thread bmc_thread_data;

/* ---- Our own clock identity (derived from MAC) ---- */
static uint8_t my_clock_id[8];

/* ---- Our own announce dataset (what we'd advertise) ---- */
static struct ptp_announce_dataset my_dataset;

/* ---- Foreign master table ---- */
static struct ptp_announce_dataset foreign_masters[PTP_MAX_FOREIGN_MASTERS];
static int foreign_master_count;
static struct k_mutex fm_mutex;

/* ---- Current BMC state ---- */
static enum ptp_bmc_role current_role = PTP_ROLE_LISTENING;
static uint8_t current_best_master_id[8];
static bool bmc_decision_valid;

/* ---- Change callback ---- */
static ptp_bmc_change_cb_t bmc_change_cb;

/* ---- Network resources ---- */
static struct net_if *bmc_iface;

/* ---- IP-ready gate ---- */
static struct k_sem ip_ready_sem;

/* ================================================================
 * Helper: Build EUI-64 clock identity from 48-bit MAC address
 *
 * IEEE 1588 Section 7.5.2.2.2:
 *   Clock ID = MAC[0] (with U/L bit toggled) | MAC[1] | MAC[2]
 *              | 0xFF | 0xFE | MAC[3] | MAC[4] | MAC[5]
 * ================================================================ */
static void mac_to_clock_identity(const uint8_t mac[6], uint8_t clock_id[8])
{
	clock_id[0] = mac[0] ^ 0x02;  /* Toggle U/L bit */
	clock_id[1] = mac[1];
	clock_id[2] = mac[2];
	clock_id[3] = 0xFF;
	clock_id[4] = 0xFE;
	clock_id[5] = mac[3];
	clock_id[6] = mac[4];
	clock_id[7] = mac[5];
}

/* ================================================================
 * Helper: Compare two 8-byte identities
 * ================================================================ */
static int clock_id_cmp(const uint8_t a[8], const uint8_t b[8])
{
	return memcmp(a, b, 8);
}

/* ================================================================
 * IEEE 1588 data-set comparison algorithm
 *
 * Returns < 0 if 'a' is better, > 0 if 'b' is better, 0 if equal.
 *
 * Comparison order (Section 9.3.4):
 *   1. GM priority1
 *   2. GM clockClass
 *   3. GM clockAccuracy
 *   4. GM offsetScaledLogVariance
 *   5. GM priority2
 *   6. GM identity (tie-breaker)
 *   7. stepsRemoved
 *   8. Sender identity (final tie-breaker)
 * ================================================================ */
static int dataset_comparison(const struct ptp_announce_dataset *a,
			      const struct ptp_announce_dataset *b)
{
	int d;

	/* 1. priority1 (lower is better) */
	d = (int)a->gm_priority1 - (int)b->gm_priority1;
	if (d != 0) return d;

	/* 2. clockClass (lower is better) */
	d = (int)a->gm_clock_class - (int)b->gm_clock_class;
	if (d != 0) return d;

	/* 3. clockAccuracy (lower is better) */
	d = (int)a->gm_clock_accuracy - (int)b->gm_clock_accuracy;
	if (d != 0) return d;

	/* 4. offsetScaledLogVariance (lower is better) */
	d = (int)a->gm_offset_scaled_log_variance -
	    (int)b->gm_offset_scaled_log_variance;
	if (d != 0) return d;

	/* 5. priority2 (lower is better) */
	d = (int)a->gm_priority2 - (int)b->gm_priority2;
	if (d != 0) return d;

	/* 6. GM identity (lexicographic, lower is better) */
	d = clock_id_cmp(a->gm_identity, b->gm_identity);
	if (d != 0) return d;

	/* 7. stepsRemoved (lower is better) */
	d = (int)a->steps_removed - (int)b->steps_removed;
	if (d != 0) return d;

	/* 8. Sender clock identity (final tie-breaker) */
	return clock_id_cmp(a->sender_clock_id, b->sender_clock_id);
}

/* ================================================================
 * Parse PTP Announce message from UDP payload
 *
 * PTPv2 common header (34 bytes) + Announce body:
 *   Offset 0:  messageType (lower nibble)
 *   Offset 1:  versionPTP
 *   Offset 2-3:  messageLength
 *   Offset 20-29: sourcePortIdentity (8-byte clockId + 2-byte portNum)
 *   Offset 33: logMessagePeriod
 *
 * Announce body starts at offset 34:
 *   +0..+9:   originTimestamp (10 bytes)
 *   +10..+11: currentUtcOffset
 *   +12:      reserved
 *   +13:      grandmasterPriority1
 *   +14:      grandmasterClockQuality.clockClass
 *   +15:      grandmasterClockQuality.clockAccuracy
 *   +16..+17: grandmasterClockQuality.offsetScaledLogVariance
 *   +18:      grandmasterPriority2
 *   +19..+26: grandmasterIdentity
 *   +27..+28: stepsRemoved
 *   +29:      timeSource
 * ================================================================ */
#define PTP_HDR_LEN          34
#define PTP_ANNOUNCE_BODY_LEN 30
#define PTP_ANNOUNCE_MIN_LEN (PTP_HDR_LEN + PTP_ANNOUNCE_BODY_LEN)

static int parse_announce(const uint8_t *buf, size_t len,
			  struct ptp_announce_dataset *ds)
{
	if (len < PTP_ANNOUNCE_MIN_LEN) {
		return -EINVAL;
	}

	/* Check message type (lower nibble of byte 0) */
	uint8_t msg_type = buf[0] & 0x0F;

	if (msg_type != PTP_MSG_ANNOUNCE) {
		return -ENOTSUP;
	}

	/* Check PTPv2 */
	if ((buf[1] & 0x0F) != 2) {
		return -ENOTSUP;
	}

	/* Source port identity (bytes 20..29) */
	memcpy(ds->sender_clock_id, &buf[20], 8);
	ds->sender_port_number = sys_get_be16(&buf[28]);

	/* Announce body (starts at offset 34) */
	const uint8_t *ann = &buf[PTP_HDR_LEN];

	/* ann[0..9]:   originTimestamp  (skip) */
	/* ann[10..11]: currentUtcOffset (skip) */
	/* ann[12]:     reserved         (skip) */

	ds->gm_priority1        = ann[13];
	ds->gm_clock_class      = ann[14];
	ds->gm_clock_accuracy   = ann[15];
	ds->gm_offset_scaled_log_variance = sys_get_be16(&ann[16]);
	ds->gm_priority2        = ann[18];
	memcpy(ds->gm_identity, &ann[19], 8);
	ds->steps_removed        = sys_get_be16(&ann[27]);
	ds->time_source          = ann[29];

	return 0;
}

/* ================================================================
 * Foreign master table management
 * ================================================================ */

/**
 * Find or create a slot in the foreign master table for the given
 * sender clock identity.
 */
static struct ptp_announce_dataset *fm_find_or_create(const uint8_t sender_id[8])
{
	int oldest_idx = -1;
	int64_t oldest_time = INT64_MAX;

	for (int i = 0; i < foreign_master_count; i++) {
		if (clock_id_cmp(foreign_masters[i].sender_clock_id,
				 sender_id) == 0) {
			return &foreign_masters[i];
		}
		if (foreign_masters[i].last_received_uptime_ms < oldest_time) {
			oldest_time = foreign_masters[i].last_received_uptime_ms;
			oldest_idx = i;
		}
	}

	/* Allocate new slot */
	if (foreign_master_count < PTP_MAX_FOREIGN_MASTERS) {
		int idx = foreign_master_count++;
		memset(&foreign_masters[idx], 0, sizeof(foreign_masters[idx]));
		return &foreign_masters[idx];
	}

	/* Table full — evict oldest entry */
	if (oldest_idx >= 0) {
		memset(&foreign_masters[oldest_idx], 0,
		       sizeof(foreign_masters[oldest_idx]));
		return &foreign_masters[oldest_idx];
	}

	return NULL; /* Should not happen */
}

/**
 * Remove expired foreign masters (no announce received within timeout).
 * Announce interval at logMsgInterval=0 is 1 second.
 * Timeout = ANNOUNCE_RECEIPT_TIMEOUT x 2^logMsgInterval seconds.
 * We use a conservative 4-second timeout.
 */
#define FM_TIMEOUT_MS  4000

static void fm_expire(void)
{
	int64_t now = k_uptime_get();

	for (int i = 0; i < foreign_master_count; ) {
		if ((now - foreign_masters[i].last_received_uptime_ms) > FM_TIMEOUT_MS) {
			LOG_INF("BMC: Foreign master expired: "
				"%02x%02x%02x%02x%02x%02x%02x%02x",
				foreign_masters[i].sender_clock_id[0],
				foreign_masters[i].sender_clock_id[1],
				foreign_masters[i].sender_clock_id[2],
				foreign_masters[i].sender_clock_id[3],
				foreign_masters[i].sender_clock_id[4],
				foreign_masters[i].sender_clock_id[5],
				foreign_masters[i].sender_clock_id[6],
				foreign_masters[i].sender_clock_id[7]);

			/* Compact: move last entry into this slot */
			foreign_master_count--;
			if (i < foreign_master_count) {
				foreign_masters[i] =
					foreign_masters[foreign_master_count];
			}
			/* Don't increment i — re-check the moved entry */
		} else {
			i++;
		}
	}
}

/* ================================================================
 * FPGA register write-back
 * ================================================================ */

static void fpga_apply_bmc_decision(enum ptp_bmc_role role,
				    const uint8_t leader_id[8],
				    uint8_t time_source,
				    int8_t log_msg_interval,
				    int8_t log_announce_interval)
{
	const struct device *fmc = device_get_binding("eth_fmc0");

	if (!fmc) {
		LOG_ERR("BMC: FMC device not found for write-back");
		return;
	}

	int ret;

	/* Write PTP configuration register 0x55 */
	ret = eth_fmc_write_ptp_config(fmc, leader_id, time_source,
				       log_msg_interval,
				       log_announce_interval);
	if (ret < 0) {
		LOG_ERR("BMC: Failed to write PTP config: %d", ret);
		return;
	}

	/* Write status flags register 0x50.
	 * Use set/clear to modify only the PTP_IS_LEADER / PTP_IS_FOLLOWER
	 * bits, leaving other bits (PPB start, resets) untouched. */
	if (role == PTP_ROLE_LEADER) {
		eth_fmc_status_clear_bits(fmc, ETH_FMC_FLAG_PTP_IS_FOLLOWER);
		ret = eth_fmc_status_set_bits(fmc, ETH_FMC_FLAG_PTP_IS_LEADER);
	} else {
		eth_fmc_status_clear_bits(fmc, ETH_FMC_FLAG_PTP_IS_LEADER);
		ret = eth_fmc_status_set_bits(fmc, ETH_FMC_FLAG_PTP_IS_FOLLOWER);
	}

	if (ret < 0) {
		LOG_ERR("BMC: Failed to write status flags: %d", ret);
		return;
	}

	LOG_INF("BMC: FPGA updated — role=%s leader=%02x%02x%02x%02x%02x%02x%02x%02x "
		"timeSrc=0x%02x logSyncInt=%d logAnnInt=%d",
		(role == PTP_ROLE_LEADER) ? "LEADER" : "FOLLOWER",
		leader_id[0], leader_id[1], leader_id[2], leader_id[3],
		leader_id[4], leader_id[5], leader_id[6], leader_id[7],
		time_source, log_msg_interval, log_announce_interval);
}

/* ================================================================
 * BMC decision logic
 *
 * Runs after every announce receipt and after expiry sweep.
 * Compares our own dataset against all known foreign masters.
 * ================================================================ */
static void run_bmc(void)
{
	k_mutex_lock(&fm_mutex, K_FOREVER);

	/* Expire stale entries first */
	fm_expire();

	/* Find the best foreign master */
	struct ptp_announce_dataset *best_foreign = NULL;

	for (int i = 0; i < foreign_master_count; i++) {
		/* Only consider masters that have sent at least 2 announces
		 * (IEEE 1588 qualification threshold) */
		if (foreign_masters[i].announce_count < 2) {
			continue;
		}

		if (!best_foreign ||
		    dataset_comparison(&foreign_masters[i], best_foreign) < 0) {
			best_foreign = &foreign_masters[i];
		}
	}

	/* Compare best foreign master against our own dataset */
	enum ptp_bmc_role new_role;
	const uint8_t *leader_id;
	uint8_t time_source;
	int8_t log_msg_interval;
	int8_t log_announce_interval;

	if (!best_foreign ||
	    dataset_comparison(&my_dataset, best_foreign) <= 0) {
		/* We are the best (or no foreign masters) → become leader */
		new_role = PTP_ROLE_LEADER;
		leader_id = my_clock_id;
		/* Crystal oscillator for self-clocked AES67 device */
		time_source = PTP_TIME_SRC_INTERNAL_OSC;
		/* AES67: logMessageInterval for sync = -3 (0.125 sec) */
		log_msg_interval = AES67_LOG_MSG_INTERVAL_SYNC;
		log_announce_interval = AES67_LOG_MSG_INTERVAL_ANNOUNCE;
	} else {
		/* Follow the best foreign master */
		new_role = PTP_ROLE_FOLLOWER;
		leader_id = best_foreign->gm_identity;
		time_source = best_foreign->time_source;
		/* Use the foreign master's message interval info.
		 * For the FPGA config we write the Sync interval. */
		log_msg_interval = AES67_LOG_MSG_INTERVAL_SYNC;
		log_announce_interval = AES67_LOG_MSG_INTERVAL_ANNOUNCE;
	}

	/* Only write to FPGA if something changed */
	bool role_changed = (new_role != current_role);
	bool leader_changed = (clock_id_cmp(leader_id,
					     current_best_master_id) != 0);

	if (role_changed || leader_changed || !bmc_decision_valid) {
		current_role = new_role;
		memcpy(current_best_master_id, leader_id, 8);
		bmc_decision_valid = true;

		fpga_apply_bmc_decision(new_role, leader_id, time_source,
					log_msg_interval,
					log_announce_interval);

		/* Notify registered listener (e.g. PLL reset) */
		if (bmc_change_cb) {
			bmc_change_cb(new_role);
		}

		if (role_changed) {
			LOG_INF("BMC: Role changed to %s",
				(new_role == PTP_ROLE_LEADER) ?
					"LEADER" : "FOLLOWER");
		}
	}

	k_mutex_unlock(&fm_mutex);
}

/* ================================================================
 * Grace period: collect announces without making a decision.
 *
 * Used at startup and after losing the current leader.
 * Listens for BMC_GRACE_PERIOD_MS, recording every foreign master
 * announce so that a subsequent run_bmc() has full data.
 * ================================================================ */
#define BMC_GRACE_PERIOD_MS  4000

static void bmc_grace_period(int sock, uint8_t *rx_buf, size_t rx_buf_len)
{
	int64_t listen_start = k_uptime_get();

	LOG_INF("BMC: Grace period — collecting announces for %d ms",
		BMC_GRACE_PERIOD_MS);

	while (1) {
		int64_t elapsed = k_uptime_get() - listen_start;

		if (elapsed >= BMC_GRACE_PERIOD_MS) {
			break;
		}

		int remaining_ms = BMC_GRACE_PERIOD_MS - (int)elapsed;
		struct zsock_pollfd pfd = {
			.fd = sock,
			.events = ZSOCK_POLLIN,
		};
		int pret = zsock_poll(&pfd, 1, remaining_ms);

		if (pret <= 0) {
			/* Timeout (0) or error (-1) → grace period done */
			break;
		}

		struct sockaddr_in src_addr;
		socklen_t src_len = sizeof(src_addr);

		ssize_t n = zsock_recvfrom(sock, rx_buf, rx_buf_len,
					   ZSOCK_MSG_DONTWAIT,
					   (struct sockaddr *)&src_addr,
					   &src_len);
		if (n < 0) {
			continue;
		}
		if (n < PTP_ANNOUNCE_MIN_LEN) {
			continue;
		}

		struct ptp_announce_dataset incoming;

		if (parse_announce(rx_buf, n, &incoming) < 0) {
			continue;
		}
		if (clock_id_cmp(incoming.sender_clock_id, my_clock_id) == 0) {
			continue;
		}

		LOG_INF("BMC: Grace — announce from "
			"%02x%02x%02x%02x%02x%02x%02x%02x",
			incoming.sender_clock_id[0], incoming.sender_clock_id[1],
			incoming.sender_clock_id[2], incoming.sender_clock_id[3],
			incoming.sender_clock_id[4], incoming.sender_clock_id[5],
			incoming.sender_clock_id[6], incoming.sender_clock_id[7]);

		k_mutex_lock(&fm_mutex, K_FOREVER);
		struct ptp_announce_dataset *fm =
			fm_find_or_create(incoming.sender_clock_id);
		if (fm) {
			fm->gm_priority1 = incoming.gm_priority1;
			fm->gm_clock_class = incoming.gm_clock_class;
			fm->gm_clock_accuracy = incoming.gm_clock_accuracy;
			fm->gm_offset_scaled_log_variance =
				incoming.gm_offset_scaled_log_variance;
			fm->gm_priority2 = incoming.gm_priority2;
			memcpy(fm->gm_identity, incoming.gm_identity, 8);
			fm->steps_removed = incoming.steps_removed;
			fm->time_source = incoming.time_source;
			memcpy(fm->sender_clock_id, incoming.sender_clock_id, 8);
			fm->sender_port_number = incoming.sender_port_number;
			fm->last_received_uptime_ms = k_uptime_get();
			fm->announce_count++;
		}
		k_mutex_unlock(&fm_mutex);
	}

	LOG_INF("BMC: Grace period over — %d foreign masters collected",
		foreign_master_count);
}

/* ================================================================
 * BMC listener thread
 *
 * Opens a UDP socket on the PTP general port (320), joins the
 * PTP multicast group (224.0.1.129), and processes Announce
 * messages as they arrive.
 * ================================================================ */
static void bmc_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	int sock;
	struct sockaddr_in bind_addr;
	uint8_t rx_buf[256];
	int ret;

	LOG_INF("BMC: Thread starting");

	/* Wait for the network interface to be up */
	while (!net_if_is_up(bmc_iface)) {
		k_msleep(500);
	}

	/* Wait until the MCU has a valid IP and it has been written to FPGA */
	LOG_INF("BMC: Waiting for valid IP address...");
	k_sem_take(&ip_ready_sem, K_FOREVER);
	LOG_INF("BMC: IP ready, opening socket");

	/* Create UDP socket */
	sock = zsock_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (sock < 0) {
		LOG_ERR("BMC: Failed to create socket: %d", errno);
		return;
	}

	/* Bind to PTP general port */
	memset(&bind_addr, 0, sizeof(bind_addr));
	bind_addr.sin_family = AF_INET;
	bind_addr.sin_port = htons(PTP_GENERAL_PORT);
	bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);

	ret = zsock_bind(sock, (struct sockaddr *)&bind_addr,
			 sizeof(bind_addr));
	if (ret < 0) {
		LOG_ERR("BMC: Failed to bind socket: %d", errno);
		zsock_close(sock);
		return;
	}

	/* Join PTP multicast group 224.0.1.129 */
	struct ip_mreqn mreq;

	memset(&mreq, 0, sizeof(mreq));
	zsock_inet_pton(AF_INET, PTP_MULTICAST_ADDR, &mreq.imr_multiaddr);
	mreq.imr_ifindex = net_if_get_by_iface(bmc_iface);

	ret = zsock_setsockopt(sock, IPPROTO_IP, IP_ADD_MEMBERSHIP,
			       &mreq, sizeof(mreq));
	if (ret < 0) {
		LOG_WRN("BMC: Failed to join multicast group: %d (will still try)", errno);
	} else {
		LOG_INF("BMC: Joined PTP multicast group %s", PTP_MULTICAST_ADDR);
	}

	LOG_INF("BMC: Listening for Announce messages on port %d",
		PTP_GENERAL_PORT);

	/* Initial grace period + first election */
	bmc_grace_period(sock, rx_buf, sizeof(rx_buf));
	run_bmc();

	/* ---- Main loop ---- */

	while (1) {
		/* Poll with 2 s timeout so we periodically run expiry
		 * even when no packets arrive. */
		struct zsock_pollfd pfd = {
			.fd = sock,
			.events = ZSOCK_POLLIN,
		};
		int pret = zsock_poll(&pfd, 1, 2000);

		if (pret == 0) {
			/* Timeout — no packet received for 2 s */
			if (current_role == PTP_ROLE_FOLLOWER) {
				int64_t now = k_uptime_get();
				bool leader_alive = false;

				k_mutex_lock(&fm_mutex, K_FOREVER);
				for (int i = 0; i < foreign_master_count; i++) {
					if (clock_id_cmp(foreign_masters[i].gm_identity,
							 current_best_master_id) == 0 &&
					    (now - foreign_masters[i].last_received_uptime_ms) <= FM_TIMEOUT_MS) {
						leader_alive = true;
						break;
					}
				}
				k_mutex_unlock(&fm_mutex);

				if (!leader_alive) {
					LOG_WRN("BMC: Leader lost — starting grace period");
					bmc_grace_period(sock, rx_buf, sizeof(rx_buf));
					run_bmc();
					continue;
				}
			}
			run_bmc();
			continue;
		}

		if (pret < 0) {
			LOG_ERR("BMC: poll error: %d", errno);
			k_msleep(1000);
			continue;
		}

		/* Data ready — read it without blocking */
		struct sockaddr_in src_addr;
		socklen_t src_len = sizeof(src_addr);

		ssize_t n = zsock_recvfrom(sock, rx_buf, sizeof(rx_buf),
					   ZSOCK_MSG_DONTWAIT,
					   (struct sockaddr *)&src_addr,
					   &src_len);
		if (n < 0) {
			continue;
		}

		if (n < PTP_ANNOUNCE_MIN_LEN) {
			/* Not an announce, or too short — ignore */
			continue;
		}

		/* Parse announce */
		struct ptp_announce_dataset incoming;

		ret = parse_announce(rx_buf, n, &incoming);
		if (ret < 0) {
			/* Not an Announce or not PTPv2 — skip */
			continue;
		}

		/* Don't track our own announce messages */
		if (clock_id_cmp(incoming.sender_clock_id, my_clock_id) == 0) {
			continue;
		}

		LOG_DBG("BMC: Announce from %02x%02x%02x%02x%02x%02x%02x%02x "
			"pri1=%u class=%u acc=0x%02x pri2=%u steps=%u",
			incoming.sender_clock_id[0],
			incoming.sender_clock_id[1],
			incoming.sender_clock_id[2],
			incoming.sender_clock_id[3],
			incoming.sender_clock_id[4],
			incoming.sender_clock_id[5],
			incoming.sender_clock_id[6],
			incoming.sender_clock_id[7],
			incoming.gm_priority1,
			incoming.gm_clock_class,
			incoming.gm_clock_accuracy,
			incoming.gm_priority2,
			incoming.steps_removed);

		/* Update foreign master table */
		k_mutex_lock(&fm_mutex, K_FOREVER);

		struct ptp_announce_dataset *fm =
			fm_find_or_create(incoming.sender_clock_id);

		if (fm) {
			/* Copy the dataset fields */
			fm->gm_priority1 = incoming.gm_priority1;
			fm->gm_clock_class = incoming.gm_clock_class;
			fm->gm_clock_accuracy = incoming.gm_clock_accuracy;
			fm->gm_offset_scaled_log_variance =
				incoming.gm_offset_scaled_log_variance;
			fm->gm_priority2 = incoming.gm_priority2;
			memcpy(fm->gm_identity, incoming.gm_identity, 8);
			fm->steps_removed = incoming.steps_removed;
			fm->time_source = incoming.time_source;
			memcpy(fm->sender_clock_id, incoming.sender_clock_id, 8);
			fm->sender_port_number = incoming.sender_port_number;
			fm->last_received_uptime_ms = k_uptime_get();
			fm->announce_count++;
		}

		k_mutex_unlock(&fm_mutex);

		/* Run BMC after each announce */
		run_bmc();
	}
}

/* ================================================================
 * Public API
 * ================================================================ */

int ptp_bmc_start(struct net_if *iface)
{
	if (!iface) {
		return -EINVAL;
	}

	bmc_iface = iface;

	/* Derive our clock identity from the interface MAC */
	struct net_linkaddr *ll = net_if_get_link_addr(iface);

	if (!ll || ll->len < 6) {
		LOG_ERR("BMC: No valid MAC address on interface");
		return -EINVAL;
	}

	mac_to_clock_identity(ll->addr, my_clock_id);

	LOG_INF("BMC: My clock identity: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
		my_clock_id[0], my_clock_id[1], my_clock_id[2], my_clock_id[3],
		my_clock_id[4], my_clock_id[5], my_clock_id[6], my_clock_id[7]);

	/* Initialize our own dataset — what we'd advertise if we were leader.
	 * AES67 default profile values. */
	memset(&my_dataset, 0, sizeof(my_dataset));
	my_dataset.gm_priority1 = PTP_CLOCK_CLASS_DEFAULT; /* 248 = default, not preferred */
	my_dataset.gm_clock_class = PTP_CLOCK_CLASS_DEFAULT;
	my_dataset.gm_clock_accuracy = PTP_CLOCK_ACCURACY_UNKNOWN;
	my_dataset.gm_offset_scaled_log_variance = 0xFFFF;
	my_dataset.gm_priority2 = PTP_CLOCK_CLASS_DEFAULT;
	memcpy(my_dataset.gm_identity, my_clock_id, 8);
	my_dataset.steps_removed = 0;
	my_dataset.time_source = PTP_TIME_SRC_INTERNAL_OSC;
	memcpy(my_dataset.sender_clock_id, my_clock_id, 8);
	my_dataset.sender_port_number = 1;

	/* Init sync primitives */
	k_mutex_init(&fm_mutex);
	k_sem_init(&ip_ready_sem, 0, 1);
	foreign_master_count = 0;
	current_role = PTP_ROLE_LISTENING;
	bmc_decision_valid = false;

	/* Start BMC thread */
	k_thread_create(&bmc_thread_data, bmc_stack, BMC_STACK_SIZE,
			bmc_thread_fn, NULL, NULL, NULL,
			BMC_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&bmc_thread_data, "ptp_bmc");

	LOG_INF("BMC: Started");
	return 0;
}

void ptp_bmc_register_change_cb(ptp_bmc_change_cb_t cb)
{
	bmc_change_cb = cb;
}

void ptp_bmc_notify_ip_ready(void)
{
	k_sem_give(&ip_ready_sem);
	LOG_INF("BMC: IP-ready notification received");
}

enum ptp_bmc_role ptp_bmc_get_role(void)
{
	return current_role;
}

int ptp_bmc_get_best_master_id(uint8_t out[8])
{
	if (!bmc_decision_valid) {
		return -ENOENT;
	}

	memcpy(out, current_best_master_id, 8);
	return 0;
}
