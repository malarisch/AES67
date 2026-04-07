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
#include "aes67_config.h"
#include "ieee1588_utils.h"
#include "../drivers/fpga_hal/fpga_hal.h"

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
static bool own_dataset_dirty;

/* ---- Change callback ---- */
static ptp_bmc_change_cb_t bmc_change_cb;

/* ---- Network resources ---- */
static struct net_if *bmc_iface;
static int bmc_sock = -1;  /* Socket for multicast rejoin */

/* ---- IP-ready gate ---- */
static struct k_sem ip_ready_sem;

/* ================================================================
 * Helper: Copy announce dataset fields into foreign master entry
 * ================================================================ */
static void fm_copy_announce(struct ptp_announce_dataset *fm,
			     const struct ptp_announce_dataset *src)
{
	fm->gm_priority1 = src->gm_priority1;
	fm->gm_clock_class = src->gm_clock_class;
	fm->gm_clock_accuracy = src->gm_clock_accuracy;
	fm->gm_offset_scaled_log_variance = src->gm_offset_scaled_log_variance;
	fm->gm_priority2 = src->gm_priority2;
	memcpy(fm->gm_identity, src->gm_identity, 8);
	fm->steps_removed = src->steps_removed;
	fm->time_source = src->time_source;
	memcpy(fm->sender_clock_id, src->sender_clock_id, 8);
	fm->sender_port_number = src->sender_port_number;
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
 *
 * IEEE 1588: timeout = announceReceiptTimeout × announceInterval.
 * With announceReceiptTimeout = 3 and logAnnounceInterval = 0 (1s),
 * the timeout is 3 seconds.  We use a configurable formula:
 *   timeout_ms = PTP_ANNOUNCE_RECEIPT_TIMEOUT × 2^logAnnounceInterval × 1000
 * Clamped to a minimum of 2000 ms.
 */
static int fm_timeout_ms(void)
{
	/* 2^logAnnounceInterval in ms = 1000 × 2^interval */
	int interval_ms = 1000;  /* logAnnounceInterval = 0 → 1 s */
	int8_t log_ann = AES67_LOG_MSG_INTERVAL_ANNOUNCE;

	if (log_ann > 0 && log_ann <= 4) {
		interval_ms = 1000 << log_ann;
	} else if (log_ann < 0 && log_ann >= -3) {
		interval_ms = 1000 >> (-log_ann);
	}

	int timeout = PTP_ANNOUNCE_RECEIPT_TIMEOUT * interval_ms;
	return (timeout < 2000) ? 2000 : timeout;
}

static void fm_expire(void)
{
	int64_t now = k_uptime_get();
	int timeout = fm_timeout_ms();

	for (int i = 0; i < foreign_master_count; ) {
		if ((now - foreign_masters[i].last_received_uptime_ms) > timeout) {
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
				    int8_t log_announce_interval,
				    uint8_t gm_priority1,
				    uint8_t gm_priority2,
				    uint8_t gm_clock_class,
				    uint8_t gm_clock_accuracy)
{
	int ret;

	/* Write PTP configuration */
	ret = fpga_hal_write_ptp_config(leader_id, time_source,
					log_msg_interval,
					log_announce_interval);
	if (ret < 0) {
		LOG_ERR("BMC: Failed to write PTP config: %d", ret);
		return;
	}

	/* Write GM quality fields */
	ret = fpga_hal_write_ptp_gm_quality(gm_priority1, gm_priority2,
					    gm_clock_class, gm_clock_accuracy);
	if (ret < 0) {
		LOG_ERR("BMC: Failed to write GM quality: %d", ret);
		return;
	}

	/* Write control flags.
	 * Use set/clear to modify only the PTP_IS_LEADER / PTP_IS_FOLLOWER
	 * bits, leaving other bits (PPB start, resets) untouched. */
	if (role == PTP_ROLE_LEADER) {
		fpga_hal_ctrl_clear_bits(FPGA_HAL_CTRL_PTP_IS_FOLLOWER);
		ret = fpga_hal_ctrl_set_bits(FPGA_HAL_CTRL_PTP_IS_LEADER);
	} else if (role == PTP_ROLE_FOLLOWER) {
		fpga_hal_ctrl_clear_bits(FPGA_HAL_CTRL_PTP_IS_LEADER);
		ret = fpga_hal_ctrl_set_bits(FPGA_HAL_CTRL_PTP_IS_FOLLOWER);
	} else {
		/* LISTENING: clear both flags — PTP core does nothing */
		fpga_hal_ctrl_clear_bits(FPGA_HAL_CTRL_PTP_IS_LEADER |
					 FPGA_HAL_CTRL_PTP_IS_FOLLOWER);
		ret = 0;
	}

	if (ret < 0) {
		LOG_ERR("BMC: Failed to write status flags: %d", ret);
		return;
	}

	LOG_INF("BMC: FPGA updated — role=%s leader=%02x%02x%02x%02x%02x%02x%02x%02x "
		"timeSrc=0x%02x logSyncInt=%d logAnnInt=%d "
		"pri1=%u pri2=%u class=%u acc=0x%02x",
		(role == PTP_ROLE_LEADER) ? "LEADER" :
		(role == PTP_ROLE_FOLLOWER) ? "FOLLOWER" : "LISTENING",
		leader_id[0], leader_id[1], leader_id[2], leader_id[3],
		leader_id[4], leader_id[5], leader_id[6], leader_id[7],
		time_source, log_msg_interval, log_announce_interval,
		gm_priority1, gm_priority2, gm_clock_class, gm_clock_accuracy);
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
		time_source = PTP_TIME_SRC_INTERNAL_OSC;
		log_msg_interval = AES67_LOG_MSG_INTERVAL_SYNC;
		log_announce_interval = AES67_LOG_MSG_INTERVAL_ANNOUNCE;
	} else {
		/* Follow the best foreign master */
		new_role = PTP_ROLE_FOLLOWER;
		leader_id = best_foreign->gm_identity;
		time_source = best_foreign->time_source;
		log_msg_interval = AES67_LOG_MSG_INTERVAL_SYNC;
		log_announce_interval = AES67_LOG_MSG_INTERVAL_ANNOUNCE;
	}

	/* GM quality CSRs always carry our own config — we are not a
	 * boundary clock, so the FPGA only needs our own values for
	 * building Announce messages when we are leader. */

	/* Only write to FPGA if something changed */
	bool role_changed = (new_role != current_role);
	bool leader_changed = (clock_id_cmp(leader_id,
					     current_best_master_id) != 0);
	bool dataset_dirty = own_dataset_dirty;
	own_dataset_dirty = false;

	if (role_changed || leader_changed || !bmc_decision_valid ||
	    dataset_dirty) {
		current_role = new_role;
		memcpy(current_best_master_id, leader_id, 8);
		bmc_decision_valid = true;

		fpga_apply_bmc_decision(new_role, leader_id, time_source,
					log_msg_interval,
					log_announce_interval,
					my_dataset.gm_priority1,
					my_dataset.gm_priority2,
					my_dataset.gm_clock_class,
					my_dataset.gm_clock_accuracy);

		/* Notify registered listener (e.g. PLL reset) */
		if (bmc_change_cb) {
			bmc_change_cb(new_role);
		}

		if (role_changed) {
			LOG_INF("BMC: Role changed to %s",
				(new_role == PTP_ROLE_LEADER) ?
				"LEADER" : (new_role == PTP_ROLE_FOLLOWER) ?
				"FOLLOWER" : "LISTENING");
		}
	}

	k_mutex_unlock(&fm_mutex);
}

/* ================================================================
 * Initial listening period.
 *
 * IEEE 1588: A clock starts in LISTENING state and waits for at
 * least one announce receipt timeout interval before making a BMC
 * decision.  This gives existing masters time to announce themselves
 * so we don't unnecessarily declare ourselves leader.
 *
 * During this period the PTP FPGA core is idle (both is_leader and
 * is_follower are cleared), and we collect foreign master announces.
 * ================================================================ */

static void bmc_initial_listening(int sock, uint8_t *rx_buf, size_t rx_buf_len)
{
	int listen_ms = fm_timeout_ms();

	LOG_INF("BMC: LISTENING — waiting %d ms for existing masters", listen_ms);

	/* Ensure FPGA is in idle state (LISTENING) */
	fpga_hal_ctrl_clear_bits(FPGA_HAL_CTRL_PTP_IS_LEADER |
				 FPGA_HAL_CTRL_PTP_IS_FOLLOWER);

	int64_t listen_start = k_uptime_get();

	while (1) {
		int64_t elapsed = k_uptime_get() - listen_start;

		if (elapsed >= listen_ms) {
			break;
		}

		int remaining = listen_ms - (int)elapsed;
		struct zsock_pollfd pfd = {
			.fd = sock,
			.events = ZSOCK_POLLIN,
		};
		int pret = zsock_poll(&pfd, 1, remaining);

		if (pret <= 0) {
			break;
		}

		struct sockaddr_in src_addr;
		socklen_t src_len = sizeof(src_addr);

		ssize_t n = zsock_recvfrom(sock, rx_buf, rx_buf_len,
					   ZSOCK_MSG_DONTWAIT,
					   (struct sockaddr *)&src_addr,
					   &src_len);
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

		LOG_INF("BMC: Listening — announce from "
			"%02x%02x%02x%02x%02x%02x%02x%02x",
			incoming.sender_clock_id[0], incoming.sender_clock_id[1],
			incoming.sender_clock_id[2], incoming.sender_clock_id[3],
			incoming.sender_clock_id[4], incoming.sender_clock_id[5],
			incoming.sender_clock_id[6], incoming.sender_clock_id[7]);

		k_mutex_lock(&fm_mutex, K_FOREVER);
		struct ptp_announce_dataset *fm =
			fm_find_or_create(incoming.sender_clock_id);
		if (fm) {
			fm_copy_announce(fm, &incoming);
			fm->last_received_uptime_ms = k_uptime_get();
			fm->announce_count++;
		}
		k_mutex_unlock(&fm_mutex);
	}

	LOG_INF("BMC: Listening period complete — %d foreign masters found",
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

	int sock = -1;
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
	bmc_sock = sock;  /* Store for multicast rejoin */

	/* Bind to PTP general port */
	memset(&bind_addr, 0, sizeof(bind_addr));
	bind_addr.sin_family = AF_INET;
	bind_addr.sin_port = htons(PTP_GENERAL_PORT);
	bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	LOG_INF("BMC: IP ready, opening socket 2");
	ret = zsock_bind(sock, (struct sockaddr *)&bind_addr,
			 sizeof(bind_addr));
	if (ret < 0) {
		LOG_ERR("BMC: Failed to bind socket: %d", errno);
		zsock_close(sock);
		return;
	}
	LOG_INF("BMC: IP ready, joining mcast");
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

	/* Initial listening period (IEEE 1588: start in LISTENING state) */
	bmc_initial_listening(sock, rx_buf, sizeof(rx_buf));
	run_bmc();

	/* ---- Main loop ----
	 *
	 * IEEE 1588 BMC runs continuously:
	 * - Every received Announce triggers a BMC re-evaluation
	 * - Periodic timeout runs expiry and BMC (catches lost masters)
	 * - No grace period on leader loss — BMC immediately decides
	 *   whether we should become leader or follow someone else
	 */

	while (1) {
		/* Poll with timeout matching the announce interval so we
		 * detect expired foreign masters promptly. */
		int poll_timeout_ms = fm_timeout_ms() / PTP_ANNOUNCE_RECEIPT_TIMEOUT;
		if (poll_timeout_ms < 500) {
			poll_timeout_ms = 500;
		}

		struct zsock_pollfd pfd = {
			.fd = sock,
			.events = ZSOCK_POLLIN,
		};
		int pret = zsock_poll(&pfd, 1, poll_timeout_ms);

		if (pret == 0) {
			/* Timeout — run BMC to expire stale masters and
			 * potentially transition to leader. */
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
			fm_copy_announce(fm, &incoming);
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

	/* Initialize our own dataset from runtime config.
	 * These values are what we'd advertise if we were leader. */
	memset(&my_dataset, 0, sizeof(my_dataset));

	aes67_config_lock();
	const struct aes67_device_config *cfg = aes67_config_get();
	my_dataset.gm_priority1 = cfg->ptp_priority1;
	my_dataset.gm_priority2 = cfg->ptp_priority2;
	my_dataset.gm_clock_class = cfg->ptp_clock_class;
	my_dataset.gm_clock_accuracy = cfg->ptp_clock_accuracy;
	aes67_config_unlock();

	my_dataset.gm_offset_scaled_log_variance = 0xFFFF;
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

void ptp_bmc_notify_fpga_ready(void)
{
	LOG_INF("BMC: FPGA-ready notification - re-applying BMC state");

	k_mutex_lock(&fm_mutex, K_FOREVER);

	if (bmc_decision_valid) {
		/* GM quality CSRs always carry our own config (not a
		 * boundary clock).  Time source depends on role. */
		uint8_t time_source = PTP_TIME_SRC_INTERNAL_OSC;

		if (current_role == PTP_ROLE_FOLLOWER) {
			/* Try to recover time_source from the foreign master */
			for (int i = 0; i < foreign_master_count; i++) {
				if (clock_id_cmp(current_best_master_id,
						 foreign_masters[i].gm_identity) == 0) {
					time_source = foreign_masters[i].time_source;
					break;
				}
			}
		}

		fpga_apply_bmc_decision(current_role, current_best_master_id,
					time_source,
					AES67_LOG_MSG_INTERVAL_SYNC,
					AES67_LOG_MSG_INTERVAL_ANNOUNCE,
					my_dataset.gm_priority1,
					my_dataset.gm_priority2,
					my_dataset.gm_clock_class,
					my_dataset.gm_clock_accuracy);
	} else {
		LOG_WRN("BMC: No valid BMC decision yet - FPGA not configured");
	}

	k_mutex_unlock(&fm_mutex);
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

const struct ptp_announce_dataset *ptp_bmc_get_foreign_masters(int *count)
{
	if (count) {
		*count = foreign_master_count;
	}
	return foreign_masters;
}

void ptp_bmc_get_clock_identity(uint8_t out[8])
{
	memcpy(out, my_clock_id, 8);
}

void ptp_bmc_update_own_dataset(void)
{
	aes67_config_lock();
	const struct aes67_device_config *cfg = aes67_config_get();

	k_mutex_lock(&fm_mutex, K_FOREVER);
	my_dataset.gm_priority1 = cfg->ptp_priority1;
	my_dataset.gm_priority2 = cfg->ptp_priority2;
	my_dataset.gm_clock_class = cfg->ptp_clock_class;
	my_dataset.gm_clock_accuracy = cfg->ptp_clock_accuracy;
	k_mutex_unlock(&fm_mutex);

	aes67_config_unlock();

	own_dataset_dirty = true;

	LOG_INF("BMC: Own dataset updated — pri1=%u pri2=%u class=%u acc=0x%02x",
		my_dataset.gm_priority1, my_dataset.gm_priority2,
		my_dataset.gm_clock_class, my_dataset.gm_clock_accuracy);

	/* Re-run BMC with updated dataset */
	run_bmc();
}

const struct ptp_announce_dataset *ptp_bmc_get_own_dataset(void)
{
	return &my_dataset;
}

/* ================================================================
 * Rejoin PTP multicast group after link up
 *
 * Called when Ethernet link transitions from down to up to ensure
 * IGMP membership is re-established.
 * ================================================================ */
void ptp_bmc_notify_link_up(void)
{
	if (bmc_sock < 0 || !bmc_iface) {
		LOG_DBG("BMC: link_up notify ignored (no socket yet)");
		return;
	}

	struct ip_mreqn mreq;

	memset(&mreq, 0, sizeof(mreq));
	zsock_inet_pton(AF_INET, PTP_MULTICAST_ADDR, &mreq.imr_multiaddr);
	mreq.imr_ifindex = net_if_get_by_iface(bmc_iface);

	int ret = zsock_setsockopt(bmc_sock, IPPROTO_IP, IP_ADD_MEMBERSHIP,
				   &mreq, sizeof(mreq));
	if (ret < 0 && errno != EADDRINUSE) {
		LOG_WRN("BMC: Failed to rejoin multicast %s: %d",
			PTP_MULTICAST_ADDR, errno);
	} else {
		LOG_INF("BMC: Rejoined PTP multicast group %s", PTP_MULTICAST_ADDR);
	}
}
