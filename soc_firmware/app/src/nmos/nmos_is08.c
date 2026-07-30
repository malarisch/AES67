/*
 * NMOS IS-08 v1.0 Audio Channel Mapping — model, map store, activations.
 *
 * Model (fixed, derived from the gateware build configuration):
 *   Inputs:  "ain"        physical AD input channels (parent null). The
 *                         tx_router allows fan-out and arbitrary pick, so
 *                         caps are {reordering:true, block_size:1}.
 *            "rx0".."rxN" one per RX stream slot (parent = the IS-04
 *                         receiver). The FPGA rx_ringbuffer routes every
 *                         channel of an active stream to a distinct DA
 *                         channel — there is no discard sink — so these
 *                         inputs carry block_size = <stream channels>:
 *                         a stream is routed completely or not at all.
 *   Outputs: "tx0".."txM" one per TX stream slot (source_id = the IS-04
 *                         source). routable_inputs = ["ain"] WITHOUT
 *                         null: a transmitted channel always carries an
 *                         input (ch_ids has no silence source).
 *            "aout"       physical DA output channels (source_id null).
 *                         routable_inputs = all "rx*" plus null.
 *
 * The map store here is the desired routing. aes67_conn stays the truth
 * for stream transport state: tx maps are applied as ch_ids, the aout
 * map as the per-stream ch_map. The IS-05 activation paths consult this
 * store instead of resetting maps to identity. A fully unrouted RX
 * stream is "muted": its FPGA slot is written disabled while the IS-05
 * subscription stays active; the last transport parameters are kept for
 * re-routing. External map changes (shell, SAP subscribe, web UI) are
 * folded back into the store via the aes67_conn observers.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "nmos.h"
#include "nmos_priv.h"
#include "nmos_json.h"
#include "../aes67_config.h"
#include "../aes67_conn.h"
#include "../../drivers/fpga_hal/fpga_hal.h"

LOG_MODULE_REGISTER(nmos_is08, LOG_LEVEL_INF);

/* ================================================================
 * Model dimensions
 * ================================================================ */

/* The rx_ringbuffer channel map field is 4 bits wide. */
#define IS08_MAX_PHYS_CH   16

/* Output table: 0..MAX_TX-1 = "tx<i>", then "aout". */
#define OUT_AOUT           AES67_MAX_TX_STREAMS
#define OUT_TABLE_SIZE     (AES67_MAX_TX_STREAMS + 1)
/* Input table: 0 = "ain", 1..MAX_RX = "rx<i-1>". */
#define IN_AIN             0
#define IN_RX(i)           (1 + (i))
#define IN_TABLE_SIZE      (1 + AES67_MAX_RX_STREAMS)

static int ain_channels(void)
{
	uint8_t n = fpga_hal_syscfg()->tx_channels;

	if (!fpga_hal_syscfg_valid()) {
		return AES67_MAX_CH_PER_STREAM;
	}
	return (n > IS08_MAX_PHYS_CH) ? IS08_MAX_PHYS_CH : n;
}

static int aout_channels(void)
{
	uint8_t n = fpga_hal_syscfg()->rx_channels;

	if (!fpga_hal_syscfg_valid()) {
		return AES67_MAX_CH_PER_STREAM;
	}
	return (n > IS08_MAX_PHYS_CH) ? IS08_MAX_PHYS_CH : n;
}

/* ================================================================
 * State
 * ================================================================ */

/* Desired TX routing: ch_ids per slot, kept across slot deactivation. */
static uint8_t tx_map[AES67_MAX_TX_STREAMS][AES67_MAX_CH_PER_STREAM];

/* Desired DA routing: feeding (stream, channel) per physical output. */
struct rx_route {
	int8_t stream;   /* -1 = unrouted */
	int8_t ch;
};
static struct rx_route out_map[IS08_MAX_PHYS_CH];

/* Streams muted through IS-08 (fully unrouted while IS-05-active); the
 * saved transport parameters allow re-enabling on re-route. */
static bool rx_muted[AES67_MAX_RX_STREAMS];
static struct aes67_rx_stream saved_rx[AES67_MAX_RX_STREAMS];

static struct k_mutex is08_lock;   /* Zephyr mutexes nest per-owner */
/* Non-zero while we write aes67_conn ourselves (HTTP thread and the
 * scheduled worker can overlap): our observers skip the store sync and
 * the IS-05 observers skip their state refresh. */
static atomic_t applying_cnt;

bool nmos_is08_applying(void)
{
	return atomic_get(&applying_cnt) > 0;
}

/* Pending scheduled activations + the last completed one. */
#define IS08_MAX_ENTRIES  48
#define IS08_MAX_PENDING  4

struct is08_entry {
	uint8_t out;      /* output table index */
	uint8_t ch;       /* output channel */
	int8_t in;        /* input table index, -1 = null */
	int8_t in_ch;     /* input channel, -1 = null */
};

struct is08_act {
	bool used;
	uint32_t id;
	uint8_t mode;                       /* 2 = absolute, 3 = relative */
	char req_str[NMOS_TAI_STR_MAX];
	struct nmos_tai when;
	uint8_t n_entries;
	struct is08_entry e[IS08_MAX_ENTRIES];
};

static struct is08_act pending[IS08_MAX_PENDING] NMOS_BIG_BSS;
static uint32_t next_act_id = 1;

static struct {
	bool valid;
	uint8_t mode;                       /* 1 = immediate, 2, 3 */
	char req_str[NMOS_TAI_STR_MAX];     /* empty = null */
	struct nmos_tai when;
} last_act;

static K_SEM_DEFINE(is08_wake, 0, 1);

/* ================================================================
 * Model helpers (call with is08_lock held where state is read)
 * ================================================================ */

static bool out_exists(int out)
{
	if (out == OUT_AOUT) {
		return aout_channels() > 0;
	}
	return out >= 0 && out < nmos_tx_count();
}

static bool in_exists(int in)
{
	if (in == IN_AIN) {
		return ain_channels() > 0;
	}
	return in >= 1 && (in - 1) < nmos_rx_count();
}

/* Channel count of an RX-stream input: live table when active, saved
 * copy while muted, device default otherwise (placeholder). */
static int rx_in_channels(int idx)
{
	const struct aes67_rx_stream *rx = &aes67_conn_get_rx_streams()[idx];
	uint8_t n = 0;

	if (rx->active) {
		n = rx->channel_count;
	} else if (rx_muted[idx]) {
		n = saved_rx[idx].channel_count;
	}
	if (n == 0 || n > AES67_MAX_CH_PER_STREAM) {
		aes67_config_lock();
		n = aes67_config_get()->default_channels;
		aes67_config_unlock();
	}
	if (n == 0 || n > AES67_MAX_CH_PER_STREAM) {
		n = AES67_DEFAULT_CHANNELS;
	}
	return n;
}

static int in_channels(int in)
{
	return (in == IN_AIN) ? ain_channels() : rx_in_channels(in - 1);
}

static int out_channels(int out)
{
	if (out == OUT_AOUT) {
		return aout_channels();
	}

	struct aes67_tx_stream tx;

	(void)nmos_tx_snapshot(out, &tx);
	return (tx.channel_count == 0) ? 1 : tx.channel_count;
}

static void in_id(int in, char *buf, size_t sz)
{
	if (in == IN_AIN) {
		snprintf(buf, sz, "ain");
	} else {
		snprintf(buf, sz, "rx%d", in - 1);
	}
}

static void out_id(int out, char *buf, size_t sz)
{
	if (out == OUT_AOUT) {
		snprintf(buf, sz, "aout");
	} else {
		snprintf(buf, sz, "tx%d", out);
	}
}

static int parse_index(const char *s)
{
	char *end;
	long v;

	if (s[0] == '\0' || (s[0] == '0' && s[1] != '\0')) {
		return -1;      /* no leading zeros */
	}
	v = strtol(s, &end, 10);
	if (*end != '\0' || v < 0 || v > 255) {
		return -1;
	}
	return (int)v;
}

static int in_by_id(const char *id)
{
	if (strcmp(id, "ain") == 0) {
		return in_exists(IN_AIN) ? IN_AIN : -1;
	}
	if (strncmp(id, "rx", 2) == 0) {
		int i = parse_index(id + 2);

		if (i >= 0 && in_exists(IN_RX(i))) {
			return IN_RX(i);
		}
	}
	return -1;
}

static int out_by_id(const char *id)
{
	if (strcmp(id, "aout") == 0) {
		return out_exists(OUT_AOUT) ? OUT_AOUT : -1;
	}
	if (strncmp(id, "tx", 2) == 0) {
		int i = parse_index(id + 2);

		if (i >= 0 && i < AES67_MAX_TX_STREAMS && out_exists(i)) {
			return i;
		}
	}
	return -1;
}

/* ================================================================
 * Store <-> device application
 * ================================================================ */

/* Build the ch_map for RX slot idx from out_map; unreferenced stream
 * channels are parked on free (else lowest) outputs — the store should
 * never be partial thanks to the block validation, this is a fallback. */
static void build_chmap(int idx, uint8_t channels,
			uint8_t ch_map[AES67_MAX_CH_PER_STREAM])
{
	int n_out = aout_channels();
	bool set[AES67_MAX_CH_PER_STREAM] = {false};
	bool used[IS08_MAX_PHYS_CH] = {false};

	memset(ch_map, 0, AES67_MAX_CH_PER_STREAM);
	for (int c = 0; c < n_out; c++) {
		if (out_map[c].stream == idx &&
		    out_map[c].ch < channels) {
			ch_map[out_map[c].ch] = (uint8_t)c;
			set[out_map[c].ch] = true;
			used[c] = true;
		}
	}
	for (int i = 0; i < channels; i++) {
		if (set[i]) {
			continue;
		}
		int free_c = 0;

		for (int c = 0; c < n_out; c++) {
			if (!used[c]) {
				free_c = c;
				break;
			}
		}
		ch_map[i] = (uint8_t)free_c;
		used[free_c] = true;
	}
}

/* True if out_map holds at least one entry for stream idx. */
static bool stream_routed(int idx)
{
	for (int c = 0; c < aout_channels(); c++) {
		if (out_map[c].stream == idx) {
			return true;
		}
	}
	return false;
}

static void clear_stream_entries(int idx)
{
	for (int c = 0; c < IS08_MAX_PHYS_CH; c++) {
		if (out_map[c].stream == idx) {
			out_map[c].stream = -1;
			out_map[c].ch = -1;
		}
	}
}

/* Lock ordering: aes67_conn observers take is05_lock (IS-05 refresh),
 * and the IS-05 apply paths call into our getters (is08_lock). To keep
 * that acyclic, is08_lock is NEVER held across aes67_conn_configure_*
 * calls: commit_entries() mutates the store under the lock, the
 * apply_*_slot() helpers take it only to copy what they need. */

/* Push the stored aout routing of RX slot idx into the FPGA: reconfigure
 * when routed (restoring saved parameters if we muted it), disable when
 * fully unrouted. Call WITHOUT the lock held. */
static int apply_rx_slot(int idx)
{
	struct aes67_rx_stream st;
	uint8_t ch_map[AES67_MAX_CH_PER_STREAM] = {0};
	bool restore = false, disable = false;
	int ret = 0;

	k_mutex_lock(&is08_lock, K_FOREVER);
	const struct aes67_rx_stream *live = &aes67_conn_get_rx_streams()[idx];

	if (stream_routed(idx)) {
		if (live->active) {
			st = *live;
			restore = true;
		} else if (rx_muted[idx]) {
			st = saved_rx[idx];
			restore = true;
		}
		if (restore) {
			build_chmap(idx, st.channel_count, ch_map);
			rx_muted[idx] = false;
		}
	} else if (live->active) {
		saved_rx[idx] = *live;
		rx_muted[idx] = true;
		disable = true;
	}
	k_mutex_unlock(&is08_lock);

	if (restore) {
		atomic_inc(&applying_cnt);
		ret = aes67_conn_configure_rx_stream((uint8_t)idx,
						     &st.dst_ip, st.dst_port,
						     ch_map, st.channel_count,
						     st.output_delay,
						     st.samples_per_channel,
						     st.name, st.sender_name,
						     &st.sender_ip);
		atomic_dec(&applying_cnt);
	} else if (disable) {
		struct in_addr zero = {0};

		atomic_inc(&applying_cnt);
		ret = aes67_conn_configure_rx_stream((uint8_t)idx, &zero, 0,
						     ch_map, 1, 0,
						     AES67_DEFAULT_SAMPLES_PER_PKT,
						     NULL, NULL, NULL);
		atomic_dec(&applying_cnt);
	}
	return ret;
}

/* Push tx_map[idx] into the FPGA if the slot is active. Call WITHOUT
 * the lock held. */
static int apply_tx_slot(int idx)
{
	struct aes67_tx_stream tx;
	uint8_t ids[AES67_MAX_CH_PER_STREAM];

	if (!nmos_tx_snapshot(idx, &tx)) {
		return 0;   /* inactive: applied on next IS-05 activation */
	}
	k_mutex_lock(&is08_lock, K_FOREVER);
	memcpy(ids, tx_map[idx], AES67_MAX_CH_PER_STREAM);
	k_mutex_unlock(&is08_lock);

	atomic_inc(&applying_cnt);
	int ret = aes67_conn_configure_tx_stream((uint8_t)idx, &tx.dst_ip,
						 tx.channel_count,
						 tx.samples_per_packet,
						 ids, tx.channel_count,
						 tx.ssrc, tx.name);
	atomic_dec(&applying_cnt);
	return ret;
}

struct is08_touched {
	bool tx[AES67_MAX_TX_STREAMS];
	bool aout;
};

/* Validate + commit a set of map entries into the store. Returns 0 or
 * -EINVAL with *errmsg set. Lock held. */
static int commit_entries(const struct is08_entry *e, int n,
			  struct is08_touched *t, const char **errmsg)
{
	struct rx_route nout[IS08_MAX_PHYS_CH];
	uint8_t ntx[AES67_MAX_TX_STREAMS][AES67_MAX_CH_PER_STREAM];

	memset(t, 0, sizeof(*t));
	memcpy(nout, out_map, sizeof(nout));
	memcpy(ntx, tx_map, sizeof(ntx));

	for (int i = 0; i < n; i++) {
		if (e[i].out == OUT_AOUT) {
			nout[e[i].ch].stream =
				(e[i].in < 0) ? -1 : (int8_t)(e[i].in - 1);
			nout[e[i].ch].ch = e[i].in_ch;
			t->aout = true;
		} else {
			ntx[e[i].out][e[i].ch] = (uint8_t)e[i].in_ch;
			t->tx[e[i].out] = true;
		}
	}

	/* Block rule: a stream referenced by the new aout map must have
	 * every channel routed exactly once (rx inputs advertise
	 * block_size = channel count). */
	if (t->aout) {
		for (int s = 0; s < nmos_rx_count(); s++) {
			int seen = 0;
			bool dup = false;
			bool have[AES67_MAX_CH_PER_STREAM] = {false};
			int want = rx_in_channels(s);

			for (int c = 0; c < aout_channels(); c++) {
				if (nout[c].stream != s) {
					continue;
				}
				seen++;
				if (nout[c].ch < 0 ||
				    nout[c].ch >= want ||
				    have[nout[c].ch]) {
					dup = true;
				} else {
					have[nout[c].ch] = true;
				}
			}
			if (seen != 0 && (dup || seen != want)) {
				*errmsg = "rx inputs route as one block: "
					  "every stream channel exactly once";
				return -EINVAL;
			}
		}
	}

	memcpy(out_map, nout, sizeof(out_map));
	memcpy(tx_map, ntx, sizeof(tx_map));
	return 0;
}

/* Push all touched slots to the device. Call WITHOUT the lock held. */
static void apply_touched(const struct is08_touched *t)
{
	for (int i = 0; i < AES67_MAX_TX_STREAMS; i++) {
		if (t->tx[i] && apply_tx_slot(i) < 0) {
			LOG_WRN("IS-08: applying tx%d map failed", i);
		}
	}
	if (t->aout) {
		for (int s = 0; s < nmos_rx_count(); s++) {
			if (apply_rx_slot(s) < 0) {
				LOG_WRN("IS-08: applying rx%d map failed", s);
			}
		}
	}
	nmos_device_touch();
}

/* ================================================================
 * IS-05 integration
 * ================================================================ */

void nmos_is08_tx_chids(int idx, uint8_t ch_ids[AES67_MAX_CH_PER_STREAM])
{
	k_mutex_lock(&is08_lock, K_FOREVER);
	memcpy(ch_ids, tx_map[idx], AES67_MAX_CH_PER_STREAM);
	k_mutex_unlock(&is08_lock);
}

/* IS-05 activation of an RX slot: an explicit (re)subscription always
 * un-mutes. Adopts an identity block into the store when the stream has
 * no routing yet, so plain IS-05 use behaves as before IS-08. */
bool nmos_is08_rx_chmap(int idx, uint8_t channels,
			uint8_t ch_map[AES67_MAX_CH_PER_STREAM])
{
	k_mutex_lock(&is08_lock, K_FOREVER);
	rx_muted[idx] = false;
	if (!stream_routed(idx)) {
		int n_out = aout_channels();

		clear_stream_entries(idx);
		for (int i = 0; i < channels && i < n_out; i++) {
			out_map[i].stream = (int8_t)idx;
			out_map[i].ch = (int8_t)i;
		}
	}
	build_chmap(idx, channels, ch_map);
	k_mutex_unlock(&is08_lock);
	return true;
}

/* ================================================================
 * Observers: fold external map changes back into the store
 * ================================================================ */

static void is08_tx_observer(uint8_t stream_id)
{
	if (nmos_is08_applying() || stream_id >= AES67_MAX_TX_STREAMS) {
		return;
	}

	struct aes67_tx_stream tx;

	if (!aes67_conn_copy_tx_stream(stream_id, &tx)) {
		return;
	}
	k_mutex_lock(&is08_lock, K_FOREVER);
	memcpy(tx_map[stream_id], tx.ch_ids, AES67_MAX_CH_PER_STREAM);
	k_mutex_unlock(&is08_lock);
}

static void is08_rx_observer(uint8_t stream_id)
{
	if (nmos_is08_applying() || stream_id >= AES67_MAX_RX_STREAMS) {
		return;
	}

	const struct aes67_rx_stream *rx =
		&aes67_conn_get_rx_streams()[stream_id];

	k_mutex_lock(&is08_lock, K_FOREVER);
	clear_stream_entries(stream_id);
	if (rx->active) {
		for (int i = 0; i < rx->channel_count; i++) {
			uint8_t c = rx->ch_map[i];

			if (c < IS08_MAX_PHYS_CH) {
				out_map[c].stream = (int8_t)stream_id;
				out_map[c].ch = (int8_t)i;
			}
		}
	}
	rx_muted[stream_id] = false;
	k_mutex_unlock(&is08_lock);
}

/* ================================================================
 * JSON builders
 * ================================================================ */

static void put_properties(struct json_out *jo, bool input, int idx)
{
	char name[AES67_STREAM_NAME_MAX];
	const char *desc;

	jo_obj_begin(jo);
	if (input && idx == IN_AIN) {
		strcpy(name, "Analog In");
		desc = "Physical AD input channels";
	} else if (!input && idx == OUT_AOUT) {
		strcpy(name, "Analog Out");
		desc = "Physical DA output channels";
	} else if (input) {
		const struct aes67_rx_stream *rx =
			&aes67_conn_get_rx_streams()[idx - 1];

		if (rx->active && rx->name[0] != '\0') {
			strncpy(name, rx->name, sizeof(name) - 1);
			name[sizeof(name) - 1] = '\0';
		} else {
			snprintf(name, sizeof(name), "RX Stream %d", idx - 1);
		}
		desc = "AES67 receiver channels";
	} else {
		struct aes67_tx_stream tx;

		(void)nmos_tx_snapshot(idx, &tx);
		strncpy(name, tx.name, sizeof(name) - 1);
		name[sizeof(name) - 1] = '\0';
		desc = "AES67 sender channels";
	}
	jo_str(jo, "name", name);
	jo_str(jo, "description", desc);
	jo_obj_end(jo);
}

static void put_parent(struct json_out *jo, int in)
{
	jo_obj_begin(jo);
	if (in == IN_AIN) {
		jo_raw(jo, "\"id\":null,\"type\":null,");
	} else {
		char uuid[NMOS_UUID_STR_LEN];

		nmos_uuid(NMOS_RES_RECEIVER, in - 1, uuid);
		jo_str(jo, "id", uuid);
		jo_str(jo, "type", "receiver");
	}
	jo_obj_end(jo);
}

static void put_channels(struct json_out *jo, bool input, int idx)
{
	int n = input ? in_channels(idx) : out_channels(idx);
	const char *fmt;

	if (input && idx == IN_AIN) {
		fmt = "Input %d";
	} else if (!input && idx == OUT_AOUT) {
		fmt = "Output %d";
	} else {
		fmt = "Ch %d";
	}

	jo_arr_begin(jo);
	for (int i = 0; i < n; i++) {
		char lbl[24];

		snprintf(lbl, sizeof(lbl), fmt, i);
		jo_obj_begin(jo);
		jo_str(jo, "label", lbl);
		jo_obj_end(jo);
	}
	jo_arr_end(jo);
}

static void put_in_caps(struct json_out *jo, int in)
{
	jo_obj_begin(jo);
	if (in == IN_AIN) {
		jo_bool(jo, "reordering", true);
		jo_uint(jo, "block_size", 1);
	} else {
		jo_bool(jo, "reordering", true);
		jo_uint(jo, "block_size", (uint32_t)rx_in_channels(in - 1));
	}
	jo_obj_end(jo);
}

static void put_out_caps(struct json_out *jo, int out)
{
	char id[8];

	jo_obj_begin(jo);
	jo_key(jo, "routable_inputs");
	jo_arr_begin(jo);
	if (out == OUT_AOUT) {
		for (int i = 0; i < nmos_rx_count(); i++) {
			in_id(IN_RX(i), id, sizeof(id));
			jo_fmt(jo, "\"%s\",", id);
		}
		jo_raw(jo, "null,");
	} else if (in_exists(IN_AIN)) {
		jo_raw(jo, "\"ain\",");
	}
	jo_arr_end(jo);
	jo_obj_end(jo);
}

/* sourceid: bare string or null. */
static void put_sourceid(struct json_out *jo, int out)
{
	if (out == OUT_AOUT) {
		jo_raw(jo, "null,");
	} else {
		char uuid[NMOS_UUID_STR_LEN];

		nmos_uuid(NMOS_RES_SOURCE, out, uuid);
		jo_fmt(jo, "\"%s\",", uuid);
	}
}

void nmos_is08_build_list(struct json_out *jo, bool inputs)
{
	char id[8];

	jo_arr_begin(jo);
	k_mutex_lock(&is08_lock, K_FOREVER);
	if (inputs) {
		for (int i = 0; i < IN_TABLE_SIZE; i++) {
			if (in_exists(i)) {
				in_id(i, id, sizeof(id));
				jo_fmt(jo, "\"%s/\",", id);
			}
		}
	} else {
		for (int o = 0; o < OUT_TABLE_SIZE; o++) {
			if (out_exists(o)) {
				out_id(o, id, sizeof(id));
				jo_fmt(jo, "\"%s/\",", id);
			}
		}
	}
	k_mutex_unlock(&is08_lock);
	jo_arr_end(jo);
}

bool nmos_is08_known_id(bool input, const char *id)
{
	k_mutex_lock(&is08_lock, K_FOREVER);
	int idx = input ? in_by_id(id) : out_by_id(id);

	k_mutex_unlock(&is08_lock);
	return idx >= 0;
}

bool nmos_is08_build_child(struct json_out *jo, bool input,
			   const char *id, int kind)
{
	k_mutex_lock(&is08_lock, K_FOREVER);
	int idx = input ? in_by_id(id) : out_by_id(id);

	if (idx < 0) {
		k_mutex_unlock(&is08_lock);
		return false;
	}
	switch (kind) {
	case 0:
		put_properties(jo, input, idx);
		break;
	case 1:
		if (input) {
			put_parent(jo, idx);
		} else {
			put_sourceid(jo, idx);
		}
		break;
	case 2:
		put_channels(jo, input, idx);
		break;
	case 3:
	default:
		if (input) {
			put_in_caps(jo, idx);
		} else {
			put_out_caps(jo, idx);
		}
		break;
	}
	k_mutex_unlock(&is08_lock);
	return true;
}

void nmos_is08_build_io(struct json_out *jo)
{
	char id[8];

	k_mutex_lock(&is08_lock, K_FOREVER);
	jo_obj_begin(jo);

	jo_key(jo, "inputs");
	jo_obj_begin(jo);
	for (int i = 0; i < IN_TABLE_SIZE; i++) {
		if (!in_exists(i)) {
			continue;
		}
		in_id(i, id, sizeof(id));
		jo_fmt(jo, "\"%s\":", id);
		jo_obj_begin(jo);
		jo_key(jo, "properties");
		put_properties(jo, true, i);
		jo_key(jo, "parent");
		put_parent(jo, i);
		jo_key(jo, "channels");
		put_channels(jo, true, i);
		jo_key(jo, "caps");
		put_in_caps(jo, i);
		jo_obj_end(jo);
	}
	jo_obj_end(jo);

	jo_key(jo, "outputs");
	jo_obj_begin(jo);
	for (int o = 0; o < OUT_TABLE_SIZE; o++) {
		if (!out_exists(o)) {
			continue;
		}
		out_id(o, id, sizeof(id));
		jo_fmt(jo, "\"%s\":", id);
		jo_obj_begin(jo);
		jo_key(jo, "properties");
		put_properties(jo, false, o);
		jo_key(jo, "source_id");
		put_sourceid(jo, o);
		jo_key(jo, "channels");
		put_channels(jo, false, o);
		jo_key(jo, "caps");
		put_out_caps(jo, o);
		jo_obj_end(jo);
	}
	jo_obj_end(jo);

	jo_obj_end(jo);
	k_mutex_unlock(&is08_lock);
}

/* One {"input":...,"channel_index":...} map cell. Lock held. */
static void put_map_cell(struct json_out *jo, int out, int ch)
{
	char id[8];

	jo_obj_begin(jo);
	if (out == OUT_AOUT) {
		if (out_map[ch].stream < 0 ||
		    !in_exists(IN_RX(out_map[ch].stream))) {
			jo_raw(jo, "\"input\":null,\"channel_index\":null,");
		} else {
			in_id(IN_RX(out_map[ch].stream), id, sizeof(id));
			jo_str(jo, "input", id);
			jo_uint(jo, "channel_index", (uint32_t)out_map[ch].ch);
		}
	} else {
		jo_str(jo, "input", "ain");
		jo_uint(jo, "channel_index", tx_map[out][ch]);
	}
	jo_obj_end(jo);
}

/* Lock held. */
static void put_out_map(struct json_out *jo, int out)
{
	char id[8];
	int n = out_channels(out);

	out_id(out, id, sizeof(id));
	jo_fmt(jo, "\"%s\":", id);
	jo_obj_begin(jo);
	for (int c = 0; c < n; c++) {
		jo_fmt(jo, "\"%d\":", c);
		put_map_cell(jo, out, c);
	}
	jo_obj_end(jo);
}

static void put_last_activation(struct json_out *jo)
{
	jo_key(jo, "activation");
	jo_obj_begin(jo);
	if (!last_act.valid) {
		jo_raw(jo, "\"mode\":null,\"requested_time\":null,"
			   "\"activation_time\":null,");
	} else {
		char buf[NMOS_TAI_STR_MAX];
		static const char *const modes[] = {
			NULL, "activate_immediate",
			"activate_scheduled_absolute",
			"activate_scheduled_relative",
		};

		jo_str(jo, "mode", modes[last_act.mode]);
		if (last_act.req_str[0] != '\0') {
			jo_str(jo, "requested_time", last_act.req_str);
		} else {
			jo_raw(jo, "\"requested_time\":null,");
		}
		nmos_tai_str(buf, sizeof(buf), &last_act.when);
		jo_str(jo, "activation_time", buf);
	}
	jo_obj_end(jo);
}

void nmos_is08_build_active(struct json_out *jo, const char *output_or_null)
{
	k_mutex_lock(&is08_lock, K_FOREVER);
	jo_obj_begin(jo);
	if (output_or_null == NULL) {
		put_last_activation(jo);
		jo_key(jo, "map");
		jo_obj_begin(jo);
		for (int o = 0; o < OUT_TABLE_SIZE; o++) {
			if (out_exists(o)) {
				put_out_map(jo, o);
			}
		}
		jo_obj_end(jo);
	} else {
		int out = out_by_id(output_or_null);

		jo_key(jo, "map");
		jo_obj_begin(jo);
		if (out >= 0) {
			put_out_map(jo, out);
		}
		jo_obj_end(jo);
	}
	jo_obj_end(jo);
	k_mutex_unlock(&is08_lock);
}

/* {activation, action} of a stored activation. Lock held. */
static void put_act_body(struct json_out *jo, const struct is08_act *a)
{
	char buf[NMOS_TAI_STR_MAX];
	char id[8];

	jo_obj_begin(jo);
	jo_key(jo, "activation");
	jo_obj_begin(jo);
	jo_str(jo, "mode", a->mode == 2 ? "activate_scheduled_absolute"
					: "activate_scheduled_relative");
	jo_str(jo, "requested_time", a->req_str);
	nmos_tai_str(buf, sizeof(buf), &a->when);
	jo_str(jo, "activation_time", buf);
	jo_obj_end(jo);

	jo_key(jo, "action");
	jo_obj_begin(jo);
	for (int o = 0; o < OUT_TABLE_SIZE; o++) {
		bool any = false;

		for (int i = 0; i < a->n_entries; i++) {
			if (a->e[i].out == o) {
				any = true;
				break;
			}
		}
		if (!any) {
			continue;
		}
		out_id(o, id, sizeof(id));
		jo_fmt(jo, "\"%s\":", id);
		jo_obj_begin(jo);
		for (int i = 0; i < a->n_entries; i++) {
			const struct is08_entry *e = &a->e[i];

			if (e->out != o) {
				continue;
			}
			jo_fmt(jo, "\"%d\":", e->ch);
			jo_obj_begin(jo);
			if (e->in < 0) {
				jo_raw(jo, "\"input\":null,"
					   "\"channel_index\":null,");
			} else {
				char iid[8];

				in_id(e->in, iid, sizeof(iid));
				jo_str(jo, "input", iid);
				jo_uint(jo, "channel_index",
					(uint32_t)e->in_ch);
			}
			jo_obj_end(jo);
		}
		jo_obj_end(jo);
	}
	jo_obj_end(jo);
	jo_obj_end(jo);
}

void nmos_is08_build_activations(struct json_out *jo)
{
	k_mutex_lock(&is08_lock, K_FOREVER);
	jo_obj_begin(jo);
	for (int i = 0; i < IS08_MAX_PENDING; i++) {
		if (!pending[i].used) {
			continue;
		}
		jo_fmt(jo, "\"%u\":", pending[i].id);
		put_act_body(jo, &pending[i]);
	}
	jo_obj_end(jo);
	k_mutex_unlock(&is08_lock);
}

static struct is08_act *find_pending(const char *act_id)
{
	int v = parse_index(act_id);

	if (v < 0) {
		return NULL;
	}
	for (int i = 0; i < IS08_MAX_PENDING; i++) {
		if (pending[i].used && pending[i].id == (uint32_t)v) {
			return &pending[i];
		}
	}
	return NULL;
}

bool nmos_is08_build_activation(struct json_out *jo, const char *act_id)
{
	k_mutex_lock(&is08_lock, K_FOREVER);
	struct is08_act *a = find_pending(act_id);

	if (a != NULL) {
		put_act_body(jo, a);
	}
	k_mutex_unlock(&is08_lock);
	return a != NULL;
}

bool nmos_is08_delete_activation(const char *act_id)
{
	k_mutex_lock(&is08_lock, K_FOREVER);
	struct is08_act *a = find_pending(act_id);

	if (a != NULL) {
		a->used = false;
	}
	k_mutex_unlock(&is08_lock);
	return a != NULL;
}

/* ================================================================
 * POST /map/activations
 * ================================================================ */

/* Parse and validate the action object into entries. Returns entry
 * count or -EINVAL with *errmsg. Lock held. */
static int parse_action(const struct nj_node *pool, const struct nj_node *act,
			struct is08_entry *e, int max, const char **errmsg)
{
	int n = 0;

	if (act == NULL || act->type != NJ_OBJ) {
		*errmsg = "action must be an object";
		return -EINVAL;
	}

	for (int oi = 0; oi < nj_count(pool, act); oi++) {
		const struct nj_node *omap = nj_item(pool, act, oi);
		char oid[12];

		if (omap->key == NULL || omap->key_len >= sizeof(oid)) {
			*errmsg = "unknown output";
			return -EINVAL;
		}
		memcpy(oid, omap->key, omap->key_len);
		oid[omap->key_len] = '\0';

		int out = out_by_id(oid);

		if (out < 0) {
			*errmsg = "unknown output";
			return -EINVAL;
		}
		if (omap->type != NJ_OBJ) {
			*errmsg = "output action must be an object";
			return -EINVAL;
		}

		for (int ci = 0; ci < nj_count(pool, omap); ci++) {
			const struct nj_node *cell = nj_item(pool, omap, ci);
			char chs[8];

			if (cell->key == NULL ||
			    cell->key_len >= sizeof(chs)) {
				*errmsg = "invalid channel index";
				return -EINVAL;
			}
			memcpy(chs, cell->key, cell->key_len);
			chs[cell->key_len] = '\0';

			int ch = parse_index(chs);

			if (ch < 0 || ch >= out_channels(out)) {
				*errmsg = "channel index out of range";
				return -EINVAL;
			}
			if (cell->type != NJ_OBJ) {
				*errmsg = "map entry must be an object";
				return -EINVAL;
			}

			const struct nj_node *jin = nj_get(pool, cell,
							   "input");
			const struct nj_node *jci = nj_get(pool, cell,
							   "channel_index");

			if (jin == NULL || jci == NULL) {
				*errmsg = "map entry needs input and "
					  "channel_index";
				return -EINVAL;
			}

			bool in_null = (jin->type == NJ_NULL);
			bool ci_null = (jci->type == NJ_NULL);

			if (in_null != ci_null) {
				*errmsg = "input and channel_index must be "
					  "null together";
				return -EINVAL;
			}

			int in = -1, in_ch = -1;

			if (!in_null) {
				char iid[12];

				if (jin->type != NJ_STR ||
				    nj_strcpy(jin, iid, sizeof(iid)) < 0) {
					*errmsg = "invalid input";
					return -EINVAL;
				}
				in = in_by_id(iid);
				if (in < 0) {
					*errmsg = "unknown input";
					return -EINVAL;
				}
				/* Constraint: tx outputs take "ain" only,
				 * aout takes "rx*" only (routable_inputs). */
				if ((out == OUT_AOUT) != (in != IN_AIN)) {
					*errmsg = "input is not routable to "
						  "this output";
					return -EINVAL;
				}
				if (jci->type != NJ_NUM || jci->num < 0 ||
				    jci->num >= in_channels(in)) {
					*errmsg = "input channel_index out "
						  "of range";
					return -EINVAL;
				}
				in_ch = (int)jci->num;
			} else if (out != OUT_AOUT) {
				/* No silence source in the tx_router:
				 * transmitted channels cannot be unrouted
				 * (routable_inputs has no null). */
				*errmsg = "output channels cannot be "
					  "unrouted on tx outputs";
				return -EINVAL;
			}

			if (n >= max) {
				*errmsg = "too many map entries";
				return -EINVAL;
			}
			e[n].out = (uint8_t)out;
			e[n].ch = (uint8_t)ch;
			e[n].in = (int8_t)in;
			e[n].in_ch = (int8_t)in_ch;
			n++;
		}
	}
	if (n == 0) {
		*errmsg = "action is empty";
		return -EINVAL;
	}
	return n;
}

/* True if any pending activation touches one of the entries' outputs. */
static bool outputs_locked(const struct is08_entry *e, int n)
{
	for (int i = 0; i < IS08_MAX_PENDING; i++) {
		if (!pending[i].used) {
			continue;
		}
		for (int j = 0; j < pending[i].n_entries; j++) {
			for (int k = 0; k < n; k++) {
				if (pending[i].e[j].out == e[k].out) {
					return true;
				}
			}
		}
	}
	return false;
}

int nmos_is08_post_activation(struct json_out *jo,
			      const struct nj_node *pool,
			      const struct nj_node *req,
			      const char **errmsg)
{
	static struct is08_entry entries[IS08_MAX_ENTRIES];
	struct nmos_tai now, when;
	char req_str[NMOS_TAI_STR_MAX] = "";
	int mode;

	const struct nj_node *jact = nj_get(pool, req, "activation");
	const struct nj_node *jaction = nj_get(pool, req, "action");

	if (jact == NULL || jact->type != NJ_OBJ) {
		*errmsg = "activation object missing";
		return 400;
	}

	const struct nj_node *jmode = nj_get(pool, jact, "mode");
	const struct nj_node *jreq = nj_get(pool, jact, "requested_time");

	if (jmode == NULL || jmode->type != NJ_STR) {
		*errmsg = "activation mode missing";
		return 400;
	}
	if (nj_streq(jmode, "activate_immediate")) {
		mode = 1;
	} else if (nj_streq(jmode, "activate_scheduled_absolute")) {
		mode = 2;
	} else if (nj_streq(jmode, "activate_scheduled_relative")) {
		mode = 3;
	} else {
		*errmsg = "invalid activation mode";
		return 400;
	}

	nmos_tai_now(&now);
	when = now;
	if (mode != 1) {
		struct nmos_tai req_t;

		if (jreq == NULL || jreq->type != NJ_STR ||
		    nj_strcpy(jreq, req_str, sizeof(req_str)) < 0 ||
		    !nmos_tai_parse(req_str, &req_t)) {
			*errmsg = "scheduled activation needs a valid "
				  "requested_time";
			return 400;
		}
		if (mode == 2) {
			when = req_t;
		} else {
			when.sec = now.sec + req_t.sec;
			when.nsec = now.nsec + req_t.nsec;
			if (when.nsec >= 1000000000u) {
				when.nsec -= 1000000000u;
				when.sec++;
			}
		}
	}

	k_mutex_lock(&is08_lock, K_FOREVER);

	int n = parse_action(pool, jaction, entries, IS08_MAX_ENTRIES,
			     errmsg);

	if (n < 0) {
		k_mutex_unlock(&is08_lock);
		return 400;
	}
	if (outputs_locked(entries, n)) {
		k_mutex_unlock(&is08_lock);
		*errmsg = "output locked by a pending activation";
		return 423;
	}

	uint32_t id = next_act_id++;

	if (mode == 1) {
		struct is08_touched touched;

		if (commit_entries(entries, n, &touched, errmsg) < 0) {
			k_mutex_unlock(&is08_lock);
			return 400;
		}
		last_act.valid = true;
		last_act.mode = 1;
		last_act.req_str[0] = '\0';
		last_act.when = now;
		k_mutex_unlock(&is08_lock);

		/* Device writes without the lock (see lock ordering note);
		 * the response must only go out once the map is applied. */
		apply_touched(&touched);

		/* Response: keyed by the new activation id (the test tool
		 * takes the first key as the id even for 200s). */
		jo_obj_begin(jo);
		jo_fmt(jo, "\"%u\":", id);
		/* immediate: requested_time null, mode immediate */
		{
			char buf[NMOS_TAI_STR_MAX];
			char oid[8];

			jo_obj_begin(jo);
			jo_key(jo, "activation");
			jo_obj_begin(jo);
			jo_str(jo, "mode", "activate_immediate");
			jo_raw(jo, "\"requested_time\":null,");
			nmos_tai_str(buf, sizeof(buf), &now);
			jo_str(jo, "activation_time", buf);
			jo_obj_end(jo);
			jo_key(jo, "action");
			jo_obj_begin(jo);
			for (int o = 0; o < OUT_TABLE_SIZE; o++) {
				bool any = false;

				for (int i = 0; i < n; i++) {
					if (entries[i].out == o) {
						any = true;
						break;
					}
				}
				if (!any) {
					continue;
				}
				out_id(o, oid, sizeof(oid));
				jo_fmt(jo, "\"%s\":", oid);
				jo_obj_begin(jo);
				for (int i = 0; i < n; i++) {
					if (entries[i].out != o) {
						continue;
					}
					jo_fmt(jo, "\"%d\":", entries[i].ch);
					jo_obj_begin(jo);
					if (entries[i].in < 0) {
						jo_raw(jo, "\"input\":null,"
						       "\"channel_index\":null,");
					} else {
						char iid[8];

						in_id(entries[i].in, iid,
						      sizeof(iid));
						jo_str(jo, "input", iid);
						jo_uint(jo, "channel_index",
							(uint32_t)entries[i].in_ch);
					}
					jo_obj_end(jo);
				}
				jo_obj_end(jo);
			}
			jo_obj_end(jo);
			jo_obj_end(jo);
		}
		jo_obj_end(jo);
		return 200;
	}

	/* Scheduled: store. */
	struct is08_act *slot = NULL;

	for (int i = 0; i < IS08_MAX_PENDING; i++) {
		if (!pending[i].used) {
			slot = &pending[i];
			break;
		}
	}
	if (slot == NULL) {
		k_mutex_unlock(&is08_lock);
		*errmsg = "too many pending activations";
		return 500;
	}
	slot->id = id;
	slot->mode = (uint8_t)mode;
	strncpy(slot->req_str, req_str, sizeof(slot->req_str) - 1);
	slot->req_str[sizeof(slot->req_str) - 1] = '\0';
	slot->when = when;
	slot->n_entries = (uint8_t)n;
	memcpy(slot->e, entries, sizeof(entries[0]) * n);
	slot->used = true;

	jo_obj_begin(jo);
	jo_fmt(jo, "\"%u\":", id);
	put_act_body(jo, slot);
	jo_obj_end(jo);

	k_mutex_unlock(&is08_lock);
	k_sem_give(&is08_wake);
	return 202;
}

/* ================================================================
 * Scheduled-activation worker
 * ================================================================ */

static bool tai_due(const struct nmos_tai *when, const struct nmos_tai *now)
{
	return now->sec > when->sec ||
	       (now->sec == when->sec && now->nsec >= when->nsec);
}

static void is08_worker(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	for (;;) {
		bool any = false;
		bool fired = false;
		struct is08_touched merged = {0};

		k_mutex_lock(&is08_lock, K_FOREVER);
		for (int i = 0; i < IS08_MAX_PENDING; i++) {
			if (!pending[i].used) {
				continue;
			}
			any = true;

			struct nmos_tai now;

			nmos_tai_now(&now);
			if (!tai_due(&pending[i].when, &now)) {
				continue;
			}

			const char *err;
			struct is08_touched t;

			if (commit_entries(pending[i].e,
					   pending[i].n_entries,
					   &t, &err) < 0) {
				LOG_WRN("IS-08 activation %u failed: %s",
					pending[i].id, err);
			} else {
				for (int o = 0; o < AES67_MAX_TX_STREAMS;
				     o++) {
					merged.tx[o] |= t.tx[o];
				}
				merged.aout |= t.aout;
				fired = true;
				last_act.valid = true;
				last_act.mode = pending[i].mode;
				strcpy(last_act.req_str, pending[i].req_str);
				last_act.when = pending[i].when;
			}
			pending[i].used = false;
		}
		k_mutex_unlock(&is08_lock);

		if (fired) {
			apply_touched(&merged);
		}
		if (any) {
			k_sleep(K_MSEC(100));
		} else {
			k_sem_take(&is08_wake, K_FOREVER);
		}
	}
}

K_THREAD_STACK_DEFINE(is08_stack, 4096);
static struct k_thread is08_thread;

/* ================================================================
 * Init
 * ================================================================ */

void nmos_is08_init(void)
{
	k_mutex_init(&is08_lock);

	for (int c = 0; c < IS08_MAX_PHYS_CH; c++) {
		out_map[c].stream = -1;
		out_map[c].ch = -1;
	}

	/* Adopt the routing of everything already configured (config
	 * restored from flash runs before nmos_start). */
	for (int i = 0; i < AES67_MAX_TX_STREAMS; i++) {
		struct aes67_tx_stream tx;

		if (aes67_conn_copy_tx_stream((uint8_t)i, &tx)) {
			memcpy(tx_map[i], tx.ch_ids,
			       AES67_MAX_CH_PER_STREAM);
		} else {
			int n_in = ain_channels();

			for (int ci = 0; ci < AES67_MAX_CH_PER_STREAM; ci++) {
				tx_map[i][ci] = (n_in > 0)
					? (uint8_t)(ci % n_in) : 0;
			}
		}
	}
	for (int i = 0; i < AES67_MAX_RX_STREAMS; i++) {
		const struct aes67_rx_stream *rx =
			&aes67_conn_get_rx_streams()[i];

		if (!rx->active) {
			continue;
		}
		for (int ci = 0; ci < rx->channel_count; ci++) {
			uint8_t c = rx->ch_map[ci];

			if (c < IS08_MAX_PHYS_CH) {
				out_map[c].stream = (int8_t)i;
				out_map[c].ch = (int8_t)ci;
			}
		}
	}

	(void)aes67_conn_register_tx_observer(is08_tx_observer);
	(void)aes67_conn_register_rx_observer(is08_rx_observer);

	k_thread_create(&is08_thread, is08_stack,
			K_THREAD_STACK_SIZEOF(is08_stack),
			is08_worker, NULL, NULL, NULL,
			K_PRIO_PREEMPT(10), 0, K_NO_WAIT);
	k_thread_name_set(&is08_thread, "nmos_is08");
}
