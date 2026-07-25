/*
 * AES67 connection management implementation.
 *
 * See aes67_conn.h for the module boundary: stream tables + FPGA writes,
 * IGMP proxy joins, foreign-stream registry. No discovery-protocol code
 * lives here — SAP/mDNS/RTSP feed and consume this module.
 */

#include <zephyr/kernel.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/net/igmp.h>
#include <zephyr/net/socket.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <string.h>
#include <stdio.h>

#include "aes67_conn.h"
#include "../drivers/fpga_hal/fpga_hal.h"
#ifdef CONFIG_IO_CARD
#include "../drivers/io_card/io_card.h"
#endif

LOG_MODULE_REGISTER(aes67_conn, LOG_LEVEL_INF);

/* Foreign entries expire after this long without a sighting/touch
 * (3x the default SAP announce interval; mDNS entries are kept alive
 * by periodic touch from the browser). */
#define FOREIGN_TIMEOUT_MS       (3 * 30 * 1000)
#define FOREIGN_EXPIRE_PERIOD    K_SECONDS(10)

#define MAX_TX_OBSERVERS 4

/* ---- State ---- */
static struct net_if *conn_iface;
static struct in_addr conn_my_ip;
static bool conn_ip_ready;

static struct aes67_tx_stream tx_streams[AES67_MAX_TX_STREAMS];
static struct aes67_rx_stream rx_streams[AES67_MAX_RX_STREAMS];
static struct aes67_foreign_stream foreign_streams[AES67_MAX_FOREIGN_STREAMS];

static aes67_conn_tx_observer_t tx_observers[MAX_TX_OBSERVERS];
static int num_tx_observers;

static K_MUTEX_DEFINE(conn_mutex);

/* ================================================================
 * IGMP proxy joins
 *
 * The FPGA terminates the RTP multicast flows itself, but IGMP-snooping
 * switches only forward a group to our port while the CPU keeps its
 * membership alive. net_ipv4_igmp_join() is idempotent (an existing
 * membership returns 0 without a new report), so a leave-first rejoin
 * is used where a fresh report must hit the wire (link bounce).
 * ================================================================ */

static bool is_multicast(const struct in_addr *addr)
{
	uint8_t first_octet = ntohl(addr->s_addr) >> 24;

	return first_octet >= 224 && first_octet <= 239;
}

static void igmp_join_group(const struct in_addr *group, bool drop_first)
{
	char addr_str[INET_ADDRSTRLEN];
	int ret;

	if (!conn_iface || !is_multicast(group)) {
		return;
	}

	if (drop_first) {
		(void)net_ipv4_igmp_leave(conn_iface, group);
	}

	ret = net_ipv4_igmp_join(conn_iface, group, NULL);

	zsock_inet_ntop(AF_INET, group, addr_str, sizeof(addr_str));
	if (ret < 0) {
		LOG_WRN("IGMP join %s failed: %d", addr_str, ret);
	} else {
		LOG_INF("IGMP joined %s", addr_str);
	}
}

/* Join the group of every active RX stream. Used to catch up on streams
 * restored from flash/SD config (configured before the IP existed) and
 * to refresh the switch's snooping table after a link bounce. */
static void join_rx_stream_groups(bool drop_first)
{
	k_mutex_lock(&conn_mutex, K_FOREVER);
	for (int i = 0; i < AES67_MAX_RX_STREAMS; i++) {
		if (rx_streams[i].active) {
			igmp_join_group(&rx_streams[i].dst_ip, drop_first);
		}
	}
	k_mutex_unlock(&conn_mutex);
}

/* ================================================================
 * TX observers
 * ================================================================ */

int aes67_conn_register_tx_observer(aes67_conn_tx_observer_t cb)
{
	if (!cb) {
		return -EINVAL;
	}
	if (num_tx_observers >= MAX_TX_OBSERVERS) {
		return -ENOMEM;
	}
	tx_observers[num_tx_observers++] = cb;
	return 0;
}

static void notify_tx_observers(uint8_t stream_id)
{
	for (int i = 0; i < num_tx_observers; i++) {
		tx_observers[i](stream_id);
	}
}

/* ================================================================
 * TX / RX stream configuration
 * ================================================================ */

int aes67_conn_configure_tx_stream(uint8_t stream_id,
				   const struct in_addr *dst_ip,
				   uint8_t channel_count,
				   uint8_t samples_per_pkt,
				   const uint8_t *ch_ids,
				   uint8_t num_ch_ids,
				   uint32_t ssrc,
				   const char *name)
{
	if (stream_id >= AES67_MAX_TX_STREAMS || !dst_ip || !ch_ids) {
		return -EINVAL;
	}

	/* Zero destination or channel count = deactivate; the zero config
	 * is still written to the FPGA to disable the stream in hardware. */
	bool activate = (dst_ip->s_addr != 0 && channel_count > 0);

	/* Auto-generate SSRC if not provided (use IP + stream_id as seed) */
	uint32_t effective_ssrc = ssrc;

	if (activate && effective_ssrc == 0) {
		effective_ssrc = sys_be32_to_cpu(dst_ip->s_addr) ^
				 (stream_id << 24) ^
				 sys_be32_to_cpu(conn_my_ip.s_addr);
	}

#ifdef CONFIG_IO_CARD
	/* Remap logical channel IDs to FPGA channel indices for 16in/8out board */
	uint8_t remapped_ch_ids[AES67_MAX_CH_PER_STREAM];

	if (io_card_is_io_board()) {
		for (uint8_t i = 0; i < num_ch_ids && i < AES67_MAX_CH_PER_STREAM; i++) {
			remapped_ch_ids[i] = io_card_logical_to_fpga_ch(ch_ids[i]);
		}
	} else {
		memcpy(remapped_ch_ids, ch_ids,
		       MIN(num_ch_ids, AES67_MAX_CH_PER_STREAM));
	}
	int ret = fpga_hal_write_tx_stream_config(stream_id, dst_ip,
						  channel_count,
						  samples_per_pkt,
						  remapped_ch_ids, num_ch_ids,
						  effective_ssrc);
#else
	int ret = fpga_hal_write_tx_stream_config(stream_id, dst_ip,
						  channel_count,
						  samples_per_pkt,
						  ch_ids, num_ch_ids,
						  effective_ssrc);
#endif
	if (ret < 0) {
		LOG_ERR("Failed to write TX stream %u to FPGA: %d",
			stream_id, ret);
		return ret;
	}

	/* Update local table */
	k_mutex_lock(&conn_mutex, K_FOREVER);
	tx_streams[stream_id].active = activate;
	tx_streams[stream_id].stream_id = stream_id;
	tx_streams[stream_id].dst_ip = *dst_ip;
	tx_streams[stream_id].channel_count = channel_count;
	tx_streams[stream_id].samples_per_packet = samples_per_pkt;
	memset(tx_streams[stream_id].ch_ids, 0,
	       sizeof(tx_streams[stream_id].ch_ids));
	for (uint8_t i = 0; i < num_ch_ids && i < AES67_MAX_CH_PER_STREAM; i++) {
		tx_streams[stream_id].ch_ids[i] = ch_ids[i];
	}
	tx_streams[stream_id].ssrc = effective_ssrc;

	/* Set stream name (auto-generate if NULL) */
	if (name && name[0] != '\0') {
		strncpy(tx_streams[stream_id].name, name,
			AES67_STREAM_NAME_MAX - 1);
		tx_streams[stream_id].name[AES67_STREAM_NAME_MAX - 1] = '\0';
	} else {
		snprintf(tx_streams[stream_id].name, AES67_STREAM_NAME_MAX,
			 "TX Stream %u", stream_id);
	}
	k_mutex_unlock(&conn_mutex);

	notify_tx_observers(stream_id);

	char addr_str[INET_ADDRSTRLEN];

	zsock_inet_ntop(AF_INET, dst_ip, addr_str, sizeof(addr_str));
	if (activate) {
		LOG_INF("TX stream %u configured: dst=%s ch=%u spp=%u ssrc=0x%08x",
			stream_id, addr_str, channel_count, samples_per_pkt,
			effective_ssrc);
	} else {
		LOG_INF("TX stream %u deactivated", stream_id);
	}

	return 0;
}

int aes67_conn_configure_rx_stream(uint8_t stream_id,
				   const struct in_addr *dst_ip,
				   uint16_t dst_port,
				   const uint8_t *ch_map,
				   uint8_t channel_count,
				   uint8_t output_delay,
				   uint8_t samples_per_channel)
{
	if (stream_id >= AES67_MAX_RX_STREAMS || !ch_map || !dst_ip ||
	    channel_count == 0 || channel_count > AES67_MAX_CH_PER_STREAM) {
		return -EINVAL;
	}

	/* Zero destination = deactivate; the zero config still goes to the
	 * FPGA, which stops matching packets for this stream. */
	bool activate = (dst_ip->s_addr != 0);

	int ret = fpga_hal_write_rx_stream_config(stream_id, dst_ip,
						  dst_port, ch_map,
						  channel_count, output_delay,
						  samples_per_channel);
	if (ret < 0) {
		LOG_ERR("Failed to write RX stream %u to FPGA: %d",
			stream_id, ret);
		return ret;
	}

	k_mutex_lock(&conn_mutex, K_FOREVER);
	rx_streams[stream_id].active = activate;
	rx_streams[stream_id].stream_id = stream_id;
	rx_streams[stream_id].dst_ip = *dst_ip;
	rx_streams[stream_id].dst_port = dst_port;
	rx_streams[stream_id].channel_count = channel_count;
	rx_streams[stream_id].output_delay = output_delay;
	rx_streams[stream_id].samples_per_channel = samples_per_channel;
	memset(rx_streams[stream_id].ch_map, 0,
	       sizeof(rx_streams[stream_id].ch_map));
	for (uint8_t i = 0; i < channel_count; i++) {
		rx_streams[stream_id].ch_map[i] = ch_map[i];
	}
	k_mutex_unlock(&conn_mutex);

	char addr_str[INET_ADDRSTRLEN];

	zsock_inet_ntop(AF_INET, dst_ip, addr_str, sizeof(addr_str));

	if (activate) {
		/* Proxy IGMP join for the FPGA-terminated group. Deferred to
		 * the IP-ready catch-up if the address isn't known yet (a
		 * report sourced from 0.0.0.0 would be useless anyway). */
		if (conn_ip_ready) {
			igmp_join_group(dst_ip, false);
		}
		LOG_INF("RX stream %u configured: dst=%s port=%u ch=%u spc=%u delay=%u",
			stream_id, addr_str, dst_port, channel_count,
			samples_per_channel, output_delay);
	} else {
		LOG_INF("RX stream %u deactivated", stream_id);
	}

	return 0;
}

const struct aes67_tx_stream *aes67_conn_get_tx_streams(void)
{
	return tx_streams;
}

const struct aes67_rx_stream *aes67_conn_get_rx_streams(void)
{
	return rx_streams;
}

bool aes67_conn_copy_tx_stream(uint8_t stream_id, struct aes67_tx_stream *out)
{
	bool active;

	if (stream_id >= AES67_MAX_TX_STREAMS || !out) {
		return false;
	}

	k_mutex_lock(&conn_mutex, K_FOREVER);
	active = tx_streams[stream_id].active;
	if (active) {
		*out = tx_streams[stream_id];
	}
	k_mutex_unlock(&conn_mutex);

	return active;
}

/* ================================================================
 * Foreign stream registry
 * ================================================================ */

void aes67_conn_report_foreign_stream(const struct aes67_foreign_stream *fs)
{
	if (!fs) {
		return;
	}

	k_mutex_lock(&conn_mutex, K_FOREVER);

	/* Handle deletion */
	if (!fs->valid) {
		for (int i = 0; i < AES67_MAX_FOREIGN_STREAMS; i++) {
			if (foreign_streams[i].valid &&
			    foreign_streams[i].id_hash == fs->id_hash) {
				LOG_INF("Foreign stream deleted: %s",
					foreign_streams[i].name);
				foreign_streams[i].valid = false;
			}
		}
		k_mutex_unlock(&conn_mutex);
		return;
	}

	/* Find existing entry or free slot */
	int free_idx = -1;

	for (int i = 0; i < AES67_MAX_FOREIGN_STREAMS; i++) {
		if (foreign_streams[i].valid &&
		    foreign_streams[i].id_hash == fs->id_hash &&
		    foreign_streams[i].origin_addr.s_addr ==
			    fs->origin_addr.s_addr) {
			/* Update existing */
			foreign_streams[i] = *fs;
			foreign_streams[i].last_seen_ms = k_uptime_get();
			k_mutex_unlock(&conn_mutex);
			return;
		}
		if (!foreign_streams[i].valid && free_idx < 0) {
			free_idx = i;
		}
	}

	/* New entry */
	if (free_idx >= 0) {
		foreign_streams[free_idx] = *fs;
		foreign_streams[free_idx].last_seen_ms = k_uptime_get();

		char addr_str[INET_ADDRSTRLEN];

		zsock_inet_ntop(AF_INET, &fs->mcast_addr,
				addr_str, sizeof(addr_str));
		LOG_INF("Discovered stream: %s @ %s:%u (%uch %ubit %uHz)",
			fs->name, addr_str, fs->port,
			fs->channels, fs->bit_depth, fs->sample_rate);
	} else {
		LOG_WRN("Foreign stream table full");
	}

	k_mutex_unlock(&conn_mutex);
}

void aes67_conn_touch_foreign_stream(uint16_t id_hash)
{
	k_mutex_lock(&conn_mutex, K_FOREVER);
	for (int i = 0; i < AES67_MAX_FOREIGN_STREAMS; i++) {
		if (foreign_streams[i].valid &&
		    foreign_streams[i].id_hash == id_hash) {
			foreign_streams[i].last_seen_ms = k_uptime_get();
		}
	}
	k_mutex_unlock(&conn_mutex);
}

const struct aes67_foreign_stream *aes67_conn_get_foreign_streams(int *count)
{
	int n = 0;

	for (int i = 0; i < AES67_MAX_FOREIGN_STREAMS; i++) {
		if (foreign_streams[i].valid) {
			n++;
		}
	}
	if (count) {
		*count = n;
	}
	return foreign_streams;
}

/* ---- Periodic expiry of stale entries ---- */

static void foreign_expire_work_fn(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(foreign_expire_work, foreign_expire_work_fn);

static void foreign_expire_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);

	int64_t now = k_uptime_get();

	k_mutex_lock(&conn_mutex, K_FOREVER);
	for (int i = 0; i < AES67_MAX_FOREIGN_STREAMS; i++) {
		if (foreign_streams[i].valid &&
		    (now - foreign_streams[i].last_seen_ms) > FOREIGN_TIMEOUT_MS) {
			LOG_INF("Foreign stream expired: %s",
				foreign_streams[i].name);
			foreign_streams[i].valid = false;
		}
	}
	k_mutex_unlock(&conn_mutex);

	k_work_schedule(&foreign_expire_work, FOREIGN_EXPIRE_PERIOD);
}

/* ================================================================
 * Lifecycle
 * ================================================================ */

int aes67_conn_init(struct net_if *iface)
{
	if (!iface) {
		return -EINVAL;
	}

	conn_iface = iface;
	k_work_schedule(&foreign_expire_work, FOREIGN_EXPIRE_PERIOD);

	LOG_INF("Connection management started");
	return 0;
}

void aes67_conn_notify_ip_ready(const struct in_addr *addr)
{
	if (addr) {
		conn_my_ip = *addr;
	}
	conn_ip_ready = true;

	/* RX streams restored from flash/SD were configured before an IP
	 * existed — catch up on their proxy IGMP joins now. */
	join_rx_stream_groups(false);
}

void aes67_conn_notify_link_up(void)
{
	if (!conn_ip_ready) {
		return;
	}
	join_rx_stream_groups(true);
}
