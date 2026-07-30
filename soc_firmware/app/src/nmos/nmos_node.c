/*
 * NMOS IS-04 node — resource model.
 *
 * Renders the IS-04 v1.3 resources (node, device, source, flow, sender,
 * receiver) from the live device state: identity from aes67_config,
 * streams from aes67_conn, PTP from ptp_ctrl. Nothing is cached except
 * the per-resource version timestamps, which the spec requires to move
 * exactly when the resource changes and to be monotonic TAI.
 *
 * Resource mapping:
 *  - one Node, one Device,
 *  - every TX stream slot is a Source + Flow + Sender and every RX slot
 *    a Receiver: the slots exist as capabilities regardless of their
 *    activation state (IS-05 activates them), subscription.active
 *    mirrors the slot. Inactive TX slots carry the device defaults;
 *    their sender manifest answers 404 (allowed from v1.2 while
 *    subscription.active is false).
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/hostname.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "nmos.h"
#include "nmos_priv.h"
#include "../aes67_config.h"
#include "../aes67_conn.h"
#include "../aes67_sdp_utils.h"
#include "../ptp_ctrl.h"
#include "../../drivers/fpga_hal/fpga_hal.h"
#include "../../drivers/eth_litex/litex_csr_compat.h"

LOG_MODULE_REGISTER(nmos_node, LOG_LEVEL_INF);

/* ================================================================
 * State
 * ================================================================ */

static struct in_addr nmos_ip;
static bool nmos_ip_valid;

static struct nmos_tai node_ver;
static struct nmos_tai device_ver;
static struct nmos_tai tx_vers[AES67_MAX_TX_STREAMS];
static struct nmos_tai rx_vers[AES67_MAX_RX_STREAMS];

/* P2P-mode DNS-SD collection counters (IS-04 ver_* TXT records) and the
 * registered-with-a-registry flag that withdraws the advertisement. */
static struct nmos_mdns_info mdns_info = {
	.advertise = true,
	.p2p = true,
};

static K_MUTEX_DEFINE(tai_lock);

/* ================================================================
 * TAI version timestamps
 *
 * Source is the PTP wallclock (TAI once disciplined). A follower resync
 * can step it backwards, and the resolution of repeated reads can
 * collide — the monotonic latch keeps every issued timestamp strictly
 * greater than the previous one, which registries rely on (a POSTed
 * version older than the held one MAY be rejected with 400).
 * ================================================================ */

static bool wallclock_read(uint64_t *sec, uint32_t *nsec)
{
	uint32_t lo, hi, ns, lo2, hi2;

	/* Coherent snapshot: re-read seconds around the nanoseconds read
	 * (same protocol as the SW-PTP PHC driver). */
	for (int retry = 0; retry < 3; retry++) {
		if (fpga_hal_csr_read(CSR_AES67_CSR_WALLCLOCK_SECONDS_IN_LO_ADDR, &lo) < 0 ||
		    fpga_hal_csr_read(CSR_AES67_CSR_WALLCLOCK_SECONDS_IN_HI_ADDR, &hi) < 0 ||
		    fpga_hal_csr_read(CSR_AES67_CSR_WALLCLOCK_NANOSECONDS_IN_ADDR, &ns) < 0 ||
		    fpga_hal_csr_read(CSR_AES67_CSR_WALLCLOCK_SECONDS_IN_LO_ADDR, &lo2) < 0 ||
		    fpga_hal_csr_read(CSR_AES67_CSR_WALLCLOCK_SECONDS_IN_HI_ADDR, &hi2) < 0) {
			return false;
		}
		if (lo == lo2 && hi == hi2) {
			*sec = ((uint64_t)(hi & 0xFFFF) << 32) | lo;
			*nsec = ns & 0x3FFFFFFF;
			return true;
		}
	}
	return false;
}

void nmos_tai_now(struct nmos_tai *t)
{
	static struct nmos_tai last;
	uint64_t sec;
	uint32_t nsec;

	if (!wallclock_read(&sec, &nsec)) {
		int64_t ms = k_uptime_get();

		sec = (uint64_t)(ms / 1000);
		nsec = (uint32_t)(ms % 1000) * 1000000u;
	}

	k_mutex_lock(&tai_lock, K_FOREVER);
	if (sec < last.sec || (sec == last.sec && nsec <= last.nsec)) {
		last.nsec++;
		if (last.nsec >= 1000000000u) {
			last.nsec = 0;
			last.sec++;
		}
	} else {
		last.sec = sec;
		last.nsec = nsec;
	}
	*t = last;
	k_mutex_unlock(&tai_lock);
}

static void put_version(struct json_out *jo, const struct nmos_tai *v)
{
	jo_fmt(jo, "\"version\":\"%llu:%u\",",
	       (unsigned long long)v->sec, v->nsec);
}

/* ================================================================
 * UUIDs — deterministic per (device identity, kind, index)
 *
 * IS-04 requires resource IDs to survive a reboot, so they are derived
 * from the configured identity rather than generated randomly: two
 * FNV-1a-64 passes over "urn:x-aes67:<vendor product serial>:<kind>:<idx>"
 * fill the 16 bytes, then the RFC 4122 version/variant bits are stamped
 * (v4 layout — the derivation is our own, not RFC 4122 §4.3 name-based).
 * ================================================================ */

static uint64_t fnv1a64(uint64_t h, const void *data, size_t len)
{
	const uint8_t *p = data;

	while (len--) {
		h ^= *p++;
		h *= 0x100000001b3ULL;
	}
	return h;
}

void nmos_uuid(enum nmos_res_kind kind, int index, char out[NMOS_UUID_STR_LEN])
{
	char node_id[AES67_NODE_ID_MAX];
	char seed[AES67_NODE_ID_MAX + 32];
	uint8_t u[16];
	int n;

	aes67_config_build_node_id(node_id, sizeof(node_id));
	n = snprintf(seed, sizeof(seed), "urn:x-aes67:%s:%d:%d",
		     node_id, (int)kind, index);

	uint64_t h1 = fnv1a64(0xcbf29ce484222325ULL, seed, n);
	uint64_t h2 = fnv1a64(0x84222325cbf29ce4ULL, seed, n);

	/* Re-mix so the halves differ even for identical input. */
	h2 = fnv1a64(h2, &h1, sizeof(h1));

	for (int i = 0; i < 8; i++) {
		u[i] = (uint8_t)(h1 >> (56 - 8 * i));
		u[8 + i] = (uint8_t)(h2 >> (56 - 8 * i));
	}
	u[6] = (u[6] & 0x0F) | 0x40; /* version 4 */
	u[8] = (u[8] & 0x3F) | 0x80; /* variant 10 */

	snprintf(out, NMOS_UUID_STR_LEN,
		 "%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x",
		 u[0], u[1], u[2], u[3], u[4], u[5], u[6], u[7],
		 u[8], u[9], u[10], u[11], u[12], u[13], u[14], u[15]);
}

/* ================================================================
 * Shared helpers
 * ================================================================ */

void nmos_ip_str(char *buf, size_t sz)
{
	if (!nmos_ip_valid) {
		strncpy(buf, "0.0.0.0", sz - 1);
		buf[sz - 1] = '\0';
		return;
	}
	zsock_inet_ntop(AF_INET, &nmos_ip, buf, sz);
}

/* The stream slot counts come from the gateware, not the firmware: the
 * system_cfg CSRs expose what the bitstream was built with. Before the
 * syscfg cache is loaded it reads all-zero — fall back to the firmware
 * table size (which also bounds a bogus CSR value). */
/* Stream-slot counts from the gateware build configuration. A loaded
 * syscfg is authoritative — 0 is a real answer (e.g. an output-only
 * device has no senders); the all-slots fallback only covers the
 * window before the first successful syscfg load. */
int nmos_tx_count(void)
{
	uint8_t n = fpga_hal_syscfg()->tx_max_streams;

	if (!fpga_hal_syscfg_valid()) {
		return AES67_MAX_TX_STREAMS;
	}
	return (n > AES67_MAX_TX_STREAMS) ? AES67_MAX_TX_STREAMS : n;
}

int nmos_rx_count(void)
{
	uint8_t n = fpga_hal_syscfg()->rx_max_streams;

	if (!fpga_hal_syscfg_valid()) {
		return AES67_MAX_RX_STREAMS;
	}
	return (n > AES67_MAX_RX_STREAMS) ? AES67_MAX_RX_STREAMS : n;
}

static void copy_device_label(char *buf, size_t sz)
{
	aes67_config_lock();
	strncpy(buf, aes67_config_get()->device_name, sz - 1);
	buf[sz - 1] = '\0';
	aes67_config_unlock();
}

/* "<hostname>/<kind>/a<idx>" — same labelling scheme as the nmos-cpp
 * reference node, so resources from different nodes stay
 * distinguishable in a shared registry. */
static void res_label(char *buf, size_t sz, const char *kind, int idx)
{
	snprintf(buf, sz, "%s/%s/a%d", net_hostname_get(), kind, idx);
}

/* Snapshot a TX slot. Inactive slots yield a default-configured
 * placeholder so the Source/Flow/Sender triple exists persistently.
 * Exported: the IS-05 activation path reuses the same placeholder
 * semantics when enabling a previously unconfigured slot. */
bool nmos_tx_snapshot(int idx, struct aes67_tx_stream *out)
{
	if (aes67_conn_copy_tx_stream(idx, out)) {
		return true;
	}

	uint8_t channels;

	aes67_config_lock();
	channels = aes67_config_get()->default_channels;
	aes67_config_unlock();
	if (channels == 0 || channels > AES67_MAX_CH_PER_STREAM) {
		channels = AES67_DEFAULT_CHANNELS;
	}

	memset(out, 0, sizeof(*out));
	out->stream_id = (uint8_t)idx;
	out->channel_count = channels;
	for (uint8_t i = 0; i < channels; i++) {
		out->ch_ids[i] = i;
	}
	snprintf(out->name, sizeof(out->name), "TX Stream %d", idx);
	return false;
}

/* id/version/label/description/tags — required on every resource. */
static void put_core(struct json_out *jo, const char *id,
		     const struct nmos_tai *ver, const char *label,
		     const char *desc)
{
	jo_str(jo, "id", id);
	put_version(jo, ver);
	jo_str(jo, "label", label);
	jo_str(jo, "description", desc);
	jo_key(jo, "tags");
	jo_obj_begin(jo);
	jo_obj_end(jo);
}

static void put_empty_obj(struct json_out *jo, const char *key)
{
	jo_key(jo, key);
	jo_obj_begin(jo);
	jo_obj_end(jo);
}

static void put_uuid_str(struct json_out *jo, const char *key,
			 enum nmos_res_kind kind, int index)
{
	char uuid[NMOS_UUID_STR_LEN];

	nmos_uuid(kind, index, uuid);
	jo_str(jo, key, uuid);
}

static void put_null(struct json_out *jo, const char *key)
{
	jo_key(jo, key);
	jo_raw(jo, "null,");
}

static void fmt_eui(char *buf, size_t sz, const uint8_t *id, int len)
{
	size_t pos = 0;

	buf[0] = '\0';
	for (int i = 0; i < len && pos + 4 <= sz; i++) {
		pos += snprintf(buf + pos, sz - pos, "%s%02x",
				i > 0 ? "-" : "", id[i]);
	}
}

/* ================================================================
 * Node (/self)
 * ================================================================ */

void nmos_build_self(struct json_out *jo)
{
	char uuid[NMOS_UUID_STR_LEN];
	char ip[INET_ADDRSTRLEN];
	char label[AES67_DEVICE_NAME_MAX];
	char buf[80];
	uint8_t mac[6];
	struct ptp_ctrl_status st;

	nmos_uuid(NMOS_RES_NODE, 0, uuid);
	nmos_ip_str(ip, sizeof(ip));
	copy_device_label(label, sizeof(label));

	jo_obj_begin(jo);
	put_core(jo, uuid, &node_ver, label, "AES67 audio node");
	jo_str(jo, "hostname", net_hostname_get());

	/* href is the API root (scheme://host:port/) — controllers and the
	 * nmos-testing tool compare it against api.endpoints entries. */
	snprintf(buf, sizeof(buf), "http://%s:%u/", ip, NMOS_HTTP_PORT);
	jo_str(jo, "href", buf);

	jo_key(jo, "api");
	jo_obj_begin(jo);
	jo_key(jo, "versions");
	jo_arr_begin(jo);
	jo_raw(jo, "\"" NMOS_API_VERSION "\",");
	jo_arr_end(jo);
	jo_key(jo, "endpoints");
	jo_arr_begin(jo);
	jo_obj_begin(jo);
	jo_str(jo, "host", ip);
	jo_uint(jo, "port", NMOS_HTTP_PORT);
	jo_str(jo, "protocol", "http");
	jo_obj_end(jo);
	jo_arr_end(jo);
	jo_obj_end(jo);

	put_empty_obj(jo, "caps");
	jo_key(jo, "services");
	jo_arr_begin(jo);
	jo_arr_end(jo);

	/* clk0 mirrors the PTP wallclock. gmid: the elected grandmaster,
	 * or our own identity while we are (or could become) leader. */
	ptp_ctrl_get_status(&st);
	jo_key(jo, "clocks");
	jo_arr_begin(jo);
	jo_obj_begin(jo);
	jo_str(jo, "name", "clk0");
	jo_str(jo, "ref_type", "ptp");
	jo_bool(jo, "traceable", false);
	jo_str(jo, "version", "IEEE1588-2008");
	fmt_eui(buf, sizeof(buf), st.gm_valid ? st.gm_id : st.clock_id, 8);
	jo_str(jo, "gmid", buf);
	jo_bool(jo, "locked", ptp_ctrl_wallclock_locked());
	jo_obj_end(jo);
	jo_arr_end(jo);

	aes67_config_build_mac(mac);
	fmt_eui(buf, sizeof(buf), mac, 6);
	jo_key(jo, "interfaces");
	jo_arr_begin(jo);
	jo_obj_begin(jo);
	jo_str(jo, "chassis_id", buf);
	jo_str(jo, "port_id", buf);
	jo_str(jo, "name", NMOS_IFACE_NAME);
	jo_obj_end(jo);
	jo_arr_end(jo);

	jo_obj_end(jo);
}

/* ================================================================
 * Device
 * ================================================================ */

void nmos_build_device(struct json_out *jo)
{
	char uuid[NMOS_UUID_STR_LEN];
	char label[AES67_DEVICE_NAME_MAX];
	char desc[96];
	const struct fpga_hal_system_cfg *cfg = fpga_hal_syscfg();

	nmos_uuid(NMOS_RES_DEVICE, 0, uuid);
	copy_device_label(label, sizeof(label));

	/* Surface the gateware build configuration (syscfg generic read at
	 * boot): TX/AD = inputs, RX/DA = outputs. */
	if (fpga_hal_syscfg_valid()) {
		snprintf(desc, sizeof(desc),
			 "AES67 audio device, %u in / %u out, "
			 "buffers %u/%u samples",
			 cfg->tx_channels, cfg->rx_channels,
			 cfg->tx_buffer_depth, cfg->rx_buffer_depth);
	} else {
		snprintf(desc, sizeof(desc), "AES67 audio device");
	}

	jo_obj_begin(jo);
	put_core(jo, uuid, &device_ver, label, desc);
	jo_str(jo, "type", "urn:x-nmos:device:generic");
	put_uuid_str(jo, "node_id", NMOS_RES_NODE, 0);

	/* senders/receivers arrays are deprecated since v1.2 but still
	 * schema-required. */
	jo_key(jo, "senders");
	jo_arr_begin(jo);
	for (int i = 0; i < nmos_tx_count(); i++) {
		nmos_uuid(NMOS_RES_SENDER, i, uuid);
		jo_fmt(jo, "\"%s\",", uuid);
	}
	jo_arr_end(jo);

	jo_key(jo, "receivers");
	jo_arr_begin(jo);
	for (int i = 0; i < nmos_rx_count(); i++) {
		nmos_uuid(NMOS_RES_RECEIVER, i, uuid);
		jo_fmt(jo, "\"%s\",", uuid);
	}
	jo_arr_end(jo);

	jo_key(jo, "controls");
	jo_arr_begin(jo);
#ifdef CONFIG_NMOS_IS05
	{
		char ip[INET_ADDRSTRLEN];
		char href[96];

		nmos_ip_str(ip, sizeof(ip));
		snprintf(href, sizeof(href),
			 "http://%s:%u/x-nmos/connection/" NMOS_IS05_VERSION "/",
			 ip, NMOS_HTTP_PORT);
		jo_obj_begin(jo);
		jo_str(jo, "href", href);
		jo_str(jo, "type",
		       "urn:x-nmos:control:sr-ctrl/" NMOS_IS05_VERSION);
		jo_bool(jo, "authorization", false);
		jo_obj_end(jo);
	}
#endif
	jo_arr_end(jo);

	jo_obj_end(jo);
}

/* ================================================================
 * Source / Flow / Sender — one triple per active TX stream
 * ================================================================ */

bool nmos_build_source(struct json_out *jo, int idx)
{
	char uuid[NMOS_UUID_STR_LEN];
	char label[NMOS_LABEL_MAX];
	char buf[24];
	struct aes67_tx_stream tx;

	(void)nmos_tx_snapshot(idx, &tx);
	nmos_uuid(NMOS_RES_SOURCE, idx, uuid);

	jo_obj_begin(jo);
	res_label(label, sizeof(label), "source", idx);
	put_core(jo, uuid, &tx_vers[idx], label, tx.name);
	put_empty_obj(jo, "caps");
	put_uuid_str(jo, "device_id", NMOS_RES_DEVICE, 0);
	jo_key(jo, "parents");
	jo_arr_begin(jo);
	jo_arr_end(jo);
	jo_str(jo, "clock_name", "clk0");
	jo_str(jo, "format", "urn:x-nmos:format:audio");

	jo_key(jo, "channels");
	jo_arr_begin(jo);
	for (int c = 0; c < tx.channel_count; c++) {
		jo_obj_begin(jo);
		snprintf(buf, sizeof(buf), "Input %u", tx.ch_ids[c]);
		jo_str(jo, "label", buf);
		jo_obj_end(jo);
	}
	jo_arr_end(jo);

	jo_obj_end(jo);
	return true;
}

bool nmos_build_flow(struct json_out *jo, int idx)
{
	char uuid[NMOS_UUID_STR_LEN];
	char label[NMOS_LABEL_MAX];
	struct aes67_tx_stream tx;
	uint32_t rate;
	uint8_t depth;

	(void)nmos_tx_snapshot(idx, &tx);

	aes67_config_lock();
	rate = aes67_config_get()->default_sample_rate;
	depth = aes67_config_get()->default_bit_depth;
	aes67_config_unlock();

	nmos_uuid(NMOS_RES_FLOW, idx, uuid);

	jo_obj_begin(jo);
	res_label(label, sizeof(label), "flow", idx);
	put_core(jo, uuid, &tx_vers[idx], label, tx.name);
	put_uuid_str(jo, "source_id", NMOS_RES_SOURCE, idx);
	put_uuid_str(jo, "device_id", NMOS_RES_DEVICE, 0);
	jo_key(jo, "parents");
	jo_arr_begin(jo);
	jo_arr_end(jo);
	jo_str(jo, "format", "urn:x-nmos:format:audio");
	jo_key(jo, "sample_rate");
	jo_obj_begin(jo);
	jo_uint(jo, "numerator", rate);
	jo_obj_end(jo);
	jo_fmt(jo, "\"media_type\":\"audio/L%u\",", depth);
	jo_uint(jo, "bit_depth", depth);
	jo_obj_end(jo);
	return true;
}

bool nmos_build_sender(struct json_out *jo, int idx)
{
	char uuid[NMOS_UUID_STR_LEN];
	char ip[INET_ADDRSTRLEN];
	char label[NMOS_LABEL_MAX];
	char buf[96];
	struct aes67_tx_stream tx;
	bool active = nmos_tx_snapshot(idx, &tx);

	nmos_uuid(NMOS_RES_SENDER, idx, uuid);
	nmos_ip_str(ip, sizeof(ip));

	jo_obj_begin(jo);
	res_label(label, sizeof(label), "sender", idx);
	put_core(jo, uuid, &tx_vers[idx], label, tx.name);
	put_uuid_str(jo, "flow_id", NMOS_RES_FLOW, idx);
	jo_str(jo, "transport", "urn:x-nmos:transport:rtp.mcast");
	put_uuid_str(jo, "device_id", NMOS_RES_DEVICE, 0);
	snprintf(buf, sizeof(buf), "http://%s/x-nmos/manifest/%s", ip, uuid);
	jo_str(jo, "manifest_href", buf);
	jo_key(jo, "interface_bindings");
	jo_arr_begin(jo);
	jo_raw(jo, "\"" NMOS_IFACE_NAME "\",");
	jo_arr_end(jo);
	jo_key(jo, "subscription");
	jo_obj_begin(jo);
#ifdef CONFIG_NMOS_IS05
	{
		char rid[NMOS_UUID_STR_LEN];
		bool enabled;

		ARG_UNUSED(active);
		if (nmos_is05_sub(true, idx, rid, &enabled)) {
			jo_str(jo, "receiver_id", rid);
		} else {
			put_null(jo, "receiver_id");
		}
		jo_bool(jo, "active", enabled);
	}
#else
	put_null(jo, "receiver_id");
	jo_bool(jo, "active", active);
#endif
	jo_obj_end(jo);
	jo_obj_end(jo);
	return true;
}

/* ================================================================
 * Receiver — every RX slot
 * ================================================================ */

bool nmos_build_receiver(struct json_out *jo, int idx)
{
	char uuid[NMOS_UUID_STR_LEN];
	char label[NMOS_LABEL_MAX];
	const struct aes67_rx_stream *rx = &aes67_conn_get_rx_streams()[idx];
	bool active = rx->active;

	nmos_uuid(NMOS_RES_RECEIVER, idx, uuid);

	/* The label stays slot-bound; the name of the subscribed stream
	 * (rx->name follows the sender's SDP session name) only goes into
	 * the description, so the receiver does not appear to be renamed
	 * after the sender in a registry. */
	res_label(label, sizeof(label), "receiver", idx);

	jo_obj_begin(jo);
	put_core(jo, uuid, &rx_vers[idx], label,
		 (active && rx->name[0] != '\0') ? rx->name
						 : "AES67 RTP receiver");
	put_uuid_str(jo, "device_id", NMOS_RES_DEVICE, 0);
	jo_str(jo, "transport", "urn:x-nmos:transport:rtp");
	jo_key(jo, "interface_bindings");
	jo_arr_begin(jo);
	jo_raw(jo, "\"" NMOS_IFACE_NAME "\",");
	jo_arr_end(jo);
	jo_str(jo, "format", "urn:x-nmos:format:audio");
	jo_key(jo, "caps");
	jo_obj_begin(jo);
	jo_key(jo, "media_types");
	jo_arr_begin(jo);
	jo_raw(jo, "\"audio/L24\",\"audio/L16\",");
	jo_arr_end(jo);
	jo_obj_end(jo);
	jo_key(jo, "subscription");
	jo_obj_begin(jo);
#ifdef CONFIG_NMOS_IS05
	{
		char sid[NMOS_UUID_STR_LEN];
		bool enabled;

		ARG_UNUSED(active);
		if (nmos_is05_sub(false, idx, sid, &enabled)) {
			jo_str(jo, "sender_id", sid);
		} else {
			put_null(jo, "sender_id");
		}
		jo_bool(jo, "active", enabled);
	}
#else
	put_null(jo, "sender_id");
	jo_bool(jo, "active", active);
#endif
	jo_obj_end(jo);
	jo_obj_end(jo);
	return true;
}

/* ================================================================
 * Sender manifest (SDP)
 * ================================================================ */

int nmos_build_manifest(char *buf, size_t sz, int idx)
{
	struct aes67_tx_stream tx;
	struct ptp_ctrl_status st;
	uint32_t rate;
	uint16_t port;
	uint8_t depth, pt, domain;

	if (!aes67_conn_copy_tx_stream(idx, &tx)) {
		return -ENOENT;
	}

	aes67_config_lock();
	const struct aes67_device_config *cfg = aes67_config_get();

	rate = cfg->default_sample_rate;
	depth = cfg->default_bit_depth;
	port = cfg->default_port;
	pt = cfg->default_payload_type;
	domain = cfg->ptp_domain;
	aes67_config_unlock();

	ptp_ctrl_get_status(&st);

	struct aes67_sdp_params params = {
		.origin_addr = nmos_ip,
		.connection_addr = tx.dst_ip,
		.stream_id = tx.stream_id,
		.channel_count = tx.channel_count,
		.bit_depth = depth,
		.sample_rate = rate,
		.samples_per_packet = tx.samples_per_packet,
		.port = port,
		.payload_type = pt,
		.ssrc = tx.ssrc,
		.clock_id = st.gm_valid ? st.gm_id : NULL,
		.stream_name = tx.name,
		.ptp_domain = domain,
		.sync_time = 0,
	};

	return aes67_sdp_build(buf, sz, &params);
}

/* ================================================================
 * Change tracking + lifecycle
 * ================================================================ */

static void touch_all(void)
{
	nmos_tai_now(&node_ver);
	nmos_tai_now(&device_ver);
	for (int i = 0; i < AES67_MAX_TX_STREAMS; i++) {
		nmos_tai_now(&tx_vers[i]);
	}
	for (int i = 0; i < AES67_MAX_RX_STREAMS; i++) {
		nmos_tai_now(&rx_vers[i]);
	}
}

static void nmos_tx_observer(uint8_t stream_id)
{
	if (stream_id < AES67_MAX_TX_STREAMS) {
		nmos_tai_now(&tx_vers[stream_id]);
	}
	/* The device's senders list follows TX slot activation. */
	nmos_tai_now(&device_ver);
	mdns_info.ver_src++;
	mdns_info.ver_flw++;
	mdns_info.ver_snd++;
	mdns_info.ver_dvc++;
}

static void nmos_rx_observer(uint8_t stream_id)
{
	if (stream_id < AES67_MAX_RX_STREAMS) {
		nmos_tai_now(&rx_vers[stream_id]);
	}
	mdns_info.ver_rcv++;
}

void nmos_notify_ip_ready(const struct in_addr *addr)
{
	nmos_ip = *addr;
	nmos_ip_valid = true;
	/* href, api.endpoints and every manifest_href follow the IP. */
	touch_all();
	mdns_info.ver_slf++;
	mdns_info.ver_snd++;
}

void nmos_get_mdns_info(struct nmos_mdns_info *out)
{
	*out = mdns_info;
}

bool nmos_have_ip(void)
{
	return nmos_ip_valid;
}

/* IS-04 v1.3: the _nmos-node._tcp advertisement SHOULD NOT exist while
 * a Registration API is PRESENT on the network — presence, not
 * registration success, is what withdraws it (a node backing off from
 * failing registries must stay quiet, IS-04-01 test_12_01). */
static bool registry_present;
static bool is_registered;

static void update_advert_state(void)
{
	bool advertise = !registry_present && !is_registered;

	mdns_info.advertise = advertise;
	mdns_info.p2p = advertise;
}

void nmos_set_registered(bool registered)
{
	is_registered = registered;
	update_advert_state();
}

void nmos_set_registry_present(bool present)
{
	registry_present = present;
	update_advert_state();
}

int nmos_start(void)
{
	int ret;

	touch_all();

	ret = aes67_conn_register_tx_observer(nmos_tx_observer);
	if (ret < 0) {
		return ret;
	}
	ret = aes67_conn_register_rx_observer(nmos_rx_observer);
	if (ret < 0) {
		return ret;
	}

#ifdef CONFIG_NMOS_IS05
	nmos_is05_init();
#endif

#ifdef CONFIG_NMOS_REGISTRATION
	nmos_reg_start();
#endif

	LOG_INF("NMOS: IS-04 %s node API at /x-nmos/node/", NMOS_API_VERSION);
	return 0;
}
