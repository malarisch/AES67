/*
 * Shared JSON serialization / parsing for device configuration.
 * Used by both sd_config (SD card) and flash_config (SPI flash), so the
 * two storage backends share one on-disk format.
 *
 * Built on Zephyr's descriptor-driven JSON library
 * (<zephyr/data/json.h>): the document is described once as a struct +
 * descriptor table, and the library does the lexing, escaping, bounds
 * checking and encoding. The DTO below deliberately mirrors the JSON
 * document rather than the runtime structures — it decouples the wire
 * format from the internal types, gives the string fields headroom over
 * their runtime counterparts (an over-long value in a hand-edited file
 * is truncated on apply instead of failing the whole load) and keeps
 * every key name in one place.
 *
 * Absent keys are simply not written by the parser, so the DTO is
 * pre-filled from the current state before parsing: a partial document
 * updates what it mentions and leaves everything else alone.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/socket.h>
#include <zephyr/data/json.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "config_json.h"
#include "aes67_config.h"
#include "aes67_conn.h"
#include "card_manager.h"
#include "card_settings.h"

LOG_MODULE_REGISTER(config_json, LOG_LEVEL_INF);

/* Headroom over the runtime field sizes: Zephyr's string decoder rejects
 * a value that does not fit its destination, and failing the entire load
 * over one long name would be a far worse outcome than truncating it. */
#define CFG_NAME_MAX 64
#define CFG_ADDR_MAX 24
#define CFG_TYPE_MAX 8

/* ================================================================
 * Document model
 * ================================================================ */

/* Layout rules for a struct used as an OBJ_ARRAY element: Zephyr derives
 * the element stride from the LAST descriptor (its offset plus its size,
 * rounded to the struct's alignment) and assumes an array field is
 * directly followed by its size_t length. Both are asserted below, so a
 * future field addition fails the build instead of silently shifting
 * every array element. The reserved bytes only exist to 8-align the
 * trailing channel array. */
struct cfg_tx_dto {
	char     name[CFG_NAME_MAX];
	char     dst_ip[CFG_ADDR_MAX];
	uint32_t ssrc;
	uint8_t  stream_id;
	uint8_t  channel_count;
	uint8_t  samples_per_packet;
	uint8_t  reserved;
	uint8_t  ch_ids[AES67_MAX_CH_PER_STREAM];
	size_t   ch_ids_len;
};

struct cfg_rx_dto {
	char     name[CFG_NAME_MAX];
	char     sender_name[CFG_NAME_MAX];
	char     sender_ip[CFG_ADDR_MAX];
	char     dst_ip[CFG_ADDR_MAX];
	uint16_t dst_port;
	uint8_t  stream_id;
	uint8_t  channel_count;
	uint8_t  output_delay;
	uint8_t  samples_per_channel;
	uint8_t  reserved[2];
	uint8_t  ch_map[AES67_MAX_CH_PER_STREAM];
	size_t   ch_map_len;
};

/* The length field must directly follow its array... */
BUILD_ASSERT(offsetof(struct cfg_tx_dto, ch_ids_len) ==
	     offsetof(struct cfg_tx_dto, ch_ids) + AES67_MAX_CH_PER_STREAM,
	     "ch_ids_len must directly follow ch_ids");
BUILD_ASSERT(offsetof(struct cfg_rx_dto, ch_map_len) ==
	     offsetof(struct cfg_rx_dto, ch_map) + AES67_MAX_CH_PER_STREAM,
	     "ch_map_len must directly follow ch_map");
/* ...and the last descriptor must cover the last member, or the derived
 * element stride disagrees with sizeof() and the array walks off. */
BUILD_ASSERT(sizeof(struct cfg_tx_dto) ==
	     ROUND_UP(offsetof(struct cfg_tx_dto, ch_ids_len) + sizeof(size_t),
		      __alignof__(struct cfg_tx_dto)),
	     "ch_ids/ch_ids_len must be the last members of cfg_tx_dto");
BUILD_ASSERT(sizeof(struct cfg_rx_dto) ==
	     ROUND_UP(offsetof(struct cfg_rx_dto, ch_map_len) + sizeof(size_t),
		      __alignof__(struct cfg_rx_dto)),
	     "ch_map/ch_map_len must be the last members of cfg_rx_dto");

struct cfg_doc {
	/* -- Device identification -- */
	char     vendor[CFG_NAME_MAX];
	char     product[CFG_NAME_MAX];
	char     serial[CFG_NAME_MAX];
	char     device_name[CFG_NAME_MAX];

	/* -- AES67 audio defaults -- */
	char     default_mcast_addr[CFG_ADDR_MAX];
	uint16_t default_port;
	uint8_t  default_channels;
	uint8_t  default_bit_depth;
	uint32_t default_sample_rate;
	uint16_t default_samples_per_pkt;
	uint8_t  default_payload_type;

	/* -- PTP -- */
	uint8_t  ptp_domain;
	uint8_t  ptp_priority1;
	uint8_t  ptp_priority2;
	uint8_t  ptp_clock_class;
	uint8_t  ptp_clock_accuracy;
	int8_t   ptp_log_sync_interval;
	int8_t   ptp_log_announce_interval;
	int32_t  ptp_delay_asymmetry_ns;

	/* -- PLL / PI controller -- */
	int32_t  pi_kp_num;
	int32_t  pi_kp_den;
	int32_t  pi_ki_num;
	int32_t  pi_ki_den;
	int32_t  pi_imax;
	int32_t  pi_outlier_ppb;
	uint32_t pi_warmup_cycles;

	/* -- SAP -- */
	uint32_t sap_announce_interval_s;
	bool     sap_announce_enabled;

	/* -- Analog card settings (0/1 for the boolean rows, so one
	 *    element type covers every array) -- */
	char     card_type[CFG_TYPE_MAX];
	int8_t   card_in_gain[CARD_SETTINGS_MAX_IN];
	size_t   card_in_gain_len;
	uint8_t  card_in_48v[CARD_SETTINGS_MAX_IN];
	size_t   card_in_48v_len;
	uint8_t  card_in_mute[CARD_SETTINGS_MAX_IN];
	size_t   card_in_mute_len;
	int8_t   card_out_gain[CARD_SETTINGS_MAX_OUT];
	size_t   card_out_gain_len;
	uint8_t  card_out_mute[CARD_SETTINGS_MAX_OUT];
	size_t   card_out_mute_len;

	/* -- Stream tables -- */
	struct cfg_tx_dto tx_streams[AES67_MAX_TX_STREAMS];
	size_t   tx_streams_len;
	struct cfg_rx_dto rx_streams[AES67_MAX_RX_STREAMS];
	size_t   rx_streams_len;
};

static const struct json_obj_descr tx_descr[] = {
	JSON_OBJ_DESCR_PRIM(struct cfg_tx_dto, stream_id, JSON_TOK_UINT),
	JSON_OBJ_DESCR_PRIM(struct cfg_tx_dto, name, JSON_TOK_STRING_BUF),
	JSON_OBJ_DESCR_PRIM(struct cfg_tx_dto, dst_ip, JSON_TOK_STRING_BUF),
	JSON_OBJ_DESCR_PRIM(struct cfg_tx_dto, channel_count, JSON_TOK_UINT),
	JSON_OBJ_DESCR_PRIM(struct cfg_tx_dto, samples_per_packet,
			    JSON_TOK_UINT),
	JSON_OBJ_DESCR_PRIM(struct cfg_tx_dto, ssrc, JSON_TOK_UINT),
	JSON_OBJ_DESCR_ARRAY(struct cfg_tx_dto, ch_ids,
			     AES67_MAX_CH_PER_STREAM, ch_ids_len,
			     JSON_TOK_UINT),
};

static const struct json_obj_descr rx_descr[] = {
	JSON_OBJ_DESCR_PRIM(struct cfg_rx_dto, stream_id, JSON_TOK_UINT),
	JSON_OBJ_DESCR_PRIM(struct cfg_rx_dto, name, JSON_TOK_STRING_BUF),
	JSON_OBJ_DESCR_PRIM(struct cfg_rx_dto, sender_name,
			    JSON_TOK_STRING_BUF),
	JSON_OBJ_DESCR_PRIM(struct cfg_rx_dto, sender_ip, JSON_TOK_STRING_BUF),
	JSON_OBJ_DESCR_PRIM(struct cfg_rx_dto, dst_ip, JSON_TOK_STRING_BUF),
	JSON_OBJ_DESCR_PRIM(struct cfg_rx_dto, dst_port, JSON_TOK_UINT),
	JSON_OBJ_DESCR_PRIM(struct cfg_rx_dto, channel_count, JSON_TOK_UINT),
	JSON_OBJ_DESCR_PRIM(struct cfg_rx_dto, output_delay, JSON_TOK_UINT),
	JSON_OBJ_DESCR_PRIM(struct cfg_rx_dto, samples_per_channel,
			    JSON_TOK_UINT),
	JSON_OBJ_DESCR_ARRAY(struct cfg_rx_dto, ch_map,
			     AES67_MAX_CH_PER_STREAM, ch_map_len,
			     JSON_TOK_UINT),
};

static const struct json_obj_descr doc_descr[] = {
	JSON_OBJ_DESCR_PRIM(struct cfg_doc, vendor, JSON_TOK_STRING_BUF),
	JSON_OBJ_DESCR_PRIM(struct cfg_doc, product, JSON_TOK_STRING_BUF),
	JSON_OBJ_DESCR_PRIM(struct cfg_doc, serial, JSON_TOK_STRING_BUF),
	JSON_OBJ_DESCR_PRIM(struct cfg_doc, device_name, JSON_TOK_STRING_BUF),
	JSON_OBJ_DESCR_PRIM(struct cfg_doc, default_mcast_addr,
			    JSON_TOK_STRING_BUF),
	JSON_OBJ_DESCR_PRIM(struct cfg_doc, default_port, JSON_TOK_UINT),
	JSON_OBJ_DESCR_PRIM(struct cfg_doc, default_channels, JSON_TOK_UINT),
	JSON_OBJ_DESCR_PRIM(struct cfg_doc, default_bit_depth, JSON_TOK_UINT),
	JSON_OBJ_DESCR_PRIM(struct cfg_doc, default_sample_rate,
			    JSON_TOK_UINT),
	JSON_OBJ_DESCR_PRIM(struct cfg_doc, default_samples_per_pkt,
			    JSON_TOK_UINT),
	JSON_OBJ_DESCR_PRIM(struct cfg_doc, default_payload_type,
			    JSON_TOK_UINT),
	JSON_OBJ_DESCR_PRIM(struct cfg_doc, ptp_domain, JSON_TOK_UINT),
	JSON_OBJ_DESCR_PRIM(struct cfg_doc, ptp_priority1, JSON_TOK_UINT),
	JSON_OBJ_DESCR_PRIM(struct cfg_doc, ptp_priority2, JSON_TOK_UINT),
	JSON_OBJ_DESCR_PRIM(struct cfg_doc, ptp_clock_class, JSON_TOK_UINT),
	JSON_OBJ_DESCR_PRIM(struct cfg_doc, ptp_clock_accuracy, JSON_TOK_UINT),
	JSON_OBJ_DESCR_PRIM(struct cfg_doc, ptp_log_sync_interval,
			    JSON_TOK_INT),
	JSON_OBJ_DESCR_PRIM(struct cfg_doc, ptp_log_announce_interval,
			    JSON_TOK_INT),
	JSON_OBJ_DESCR_PRIM(struct cfg_doc, ptp_delay_asymmetry_ns,
			    JSON_TOK_INT),
	JSON_OBJ_DESCR_PRIM(struct cfg_doc, pi_kp_num, JSON_TOK_INT),
	JSON_OBJ_DESCR_PRIM(struct cfg_doc, pi_kp_den, JSON_TOK_INT),
	JSON_OBJ_DESCR_PRIM(struct cfg_doc, pi_ki_num, JSON_TOK_INT),
	JSON_OBJ_DESCR_PRIM(struct cfg_doc, pi_ki_den, JSON_TOK_INT),
	JSON_OBJ_DESCR_PRIM(struct cfg_doc, pi_imax, JSON_TOK_INT),
	JSON_OBJ_DESCR_PRIM(struct cfg_doc, pi_outlier_ppb, JSON_TOK_INT),
	JSON_OBJ_DESCR_PRIM(struct cfg_doc, pi_warmup_cycles, JSON_TOK_UINT),
	JSON_OBJ_DESCR_PRIM(struct cfg_doc, sap_announce_interval_s,
			    JSON_TOK_UINT),
	JSON_OBJ_DESCR_PRIM(struct cfg_doc, sap_announce_enabled,
			    JSON_TOK_TRUE),
	JSON_OBJ_DESCR_PRIM(struct cfg_doc, card_type, JSON_TOK_STRING_BUF),
	JSON_OBJ_DESCR_ARRAY(struct cfg_doc, card_in_gain,
			     CARD_SETTINGS_MAX_IN, card_in_gain_len,
			     JSON_TOK_INT),
	JSON_OBJ_DESCR_ARRAY(struct cfg_doc, card_in_48v,
			     CARD_SETTINGS_MAX_IN, card_in_48v_len,
			     JSON_TOK_UINT),
	JSON_OBJ_DESCR_ARRAY(struct cfg_doc, card_in_mute,
			     CARD_SETTINGS_MAX_IN, card_in_mute_len,
			     JSON_TOK_UINT),
	JSON_OBJ_DESCR_ARRAY(struct cfg_doc, card_out_gain,
			     CARD_SETTINGS_MAX_OUT, card_out_gain_len,
			     JSON_TOK_INT),
	JSON_OBJ_DESCR_ARRAY(struct cfg_doc, card_out_mute,
			     CARD_SETTINGS_MAX_OUT, card_out_mute_len,
			     JSON_TOK_UINT),
	JSON_OBJ_DESCR_OBJ_ARRAY(struct cfg_doc, tx_streams,
				 AES67_MAX_TX_STREAMS, tx_streams_len,
				 tx_descr, ARRAY_SIZE(tx_descr)),
	JSON_OBJ_DESCR_OBJ_ARRAY(struct cfg_doc, rx_streams,
				 AES67_MAX_RX_STREAMS, rx_streams_len,
				 rx_descr, ARRAY_SIZE(rx_descr)),
};

/* ~3 KB — too much for the calling thread's stack (config saves run from
 * the shell, the web API and a work queue). Serialization and parsing
 * never overlap, so one instance under a mutex is enough. */
static struct cfg_doc doc;
static K_MUTEX_DEFINE(doc_mutex);

/* ================================================================
 * Card type <-> string
 * ================================================================ */

static const char *card_type_to_str(uint8_t type)
{
	switch (type) {
	case CARD_TYPE_MI: return "mi";
	case CARD_TYPE_LO: return "lo";
	case CARD_TYPE_IO: return "io";
	default:           return "";
	}
}

static uint8_t card_type_from_str(const char *s)
{
	if (s == NULL) {
		return CARD_TYPE_NONE;
	}
	if (strcmp(s, "mi") == 0) {
		return CARD_TYPE_MI;
	}
	if (strcmp(s, "lo") == 0) {
		return CARD_TYPE_LO;
	}
	if (strcmp(s, "io") == 0) {
		return CARD_TYPE_IO;
	}
	return CARD_TYPE_NONE;
}

/* ================================================================
 * Runtime state -> document
 * ================================================================ */

static void collect_device_config(struct cfg_doc *d)
{
	const struct aes67_device_config *cfg = aes67_config_get();

	aes67_config_lock();

	strncpy(d->vendor, cfg->vendor, sizeof(d->vendor) - 1);
	strncpy(d->product, cfg->product, sizeof(d->product) - 1);
	strncpy(d->serial, cfg->serial, sizeof(d->serial) - 1);
	strncpy(d->device_name, cfg->device_name, sizeof(d->device_name) - 1);
	strncpy(d->default_mcast_addr, cfg->default_mcast_addr,
		sizeof(d->default_mcast_addr) - 1);

	d->default_port             = cfg->default_port;
	d->default_channels         = cfg->default_channels;
	d->default_bit_depth        = cfg->default_bit_depth;
	d->default_sample_rate      = cfg->default_sample_rate;
	d->default_samples_per_pkt  = cfg->default_samples_per_pkt;
	d->default_payload_type     = cfg->default_payload_type;

	d->ptp_domain                = cfg->ptp_domain;
	d->ptp_priority1             = cfg->ptp_priority1;
	d->ptp_priority2             = cfg->ptp_priority2;
	d->ptp_clock_class           = cfg->ptp_clock_class;
	d->ptp_clock_accuracy        = cfg->ptp_clock_accuracy;
	d->ptp_log_sync_interval     = cfg->ptp_log_sync_interval;
	d->ptp_log_announce_interval = cfg->ptp_log_announce_interval;
	d->ptp_delay_asymmetry_ns    = cfg->ptp_delay_asymmetry_ns;

	d->pi_kp_num       = cfg->pi_kp_num;
	d->pi_kp_den       = cfg->pi_kp_den;
	d->pi_ki_num       = cfg->pi_ki_num;
	d->pi_ki_den       = cfg->pi_ki_den;
	d->pi_imax         = cfg->pi_imax;
	d->pi_outlier_ppb  = cfg->pi_outlier_ppb;
	d->pi_warmup_cycles = cfg->pi_warmup_cycles;

	d->sap_announce_interval_s = cfg->sap_announce_interval_s;
	d->sap_announce_enabled    = cfg->sap_announce_enabled;

	aes67_config_unlock();
}

static void collect_card_settings(struct cfg_doc *d)
{
	/* Refresh the shadow store from the live card if it is ready; a
	 * not-yet-activated card (LO before PTP lock) keeps the values
	 * loaded from the previous config. */
	(void)card_settings_capture();

	card_settings_lock();

	const struct card_settings *cs = card_settings_get();

	if (cs->valid && cs->card_type != CARD_TYPE_NONE) {
		strncpy(d->card_type, card_type_to_str(cs->card_type),
			sizeof(d->card_type) - 1);

		d->card_in_gain_len = MIN(cs->num_in, CARD_SETTINGS_MAX_IN);
		d->card_in_48v_len  = d->card_in_gain_len;
		d->card_in_mute_len = d->card_in_gain_len;
		for (size_t i = 0; i < d->card_in_gain_len; i++) {
			d->card_in_gain[i] = cs->in_gain[i];
			d->card_in_48v[i]  = cs->in_phantom[i] ? 1 : 0;
			d->card_in_mute[i] = cs->in_mute[i] ? 1 : 0;
		}

		d->card_out_gain_len = MIN(cs->num_out, CARD_SETTINGS_MAX_OUT);
		d->card_out_mute_len = d->card_out_gain_len;
		for (size_t i = 0; i < d->card_out_gain_len; i++) {
			d->card_out_gain[i] = cs->out_gain[i];
			d->card_out_mute[i] = cs->out_mute[i] ? 1 : 0;
		}
	}

	card_settings_unlock();
}

static void collect_streams(struct cfg_doc *d)
{
	const struct aes67_tx_stream *tx = aes67_conn_get_tx_streams();
	const struct aes67_rx_stream *rx = aes67_conn_get_rx_streams();

	for (int i = 0; i < AES67_MAX_TX_STREAMS; i++) {
		if (!tx[i].active) {
			continue;
		}

		struct cfg_tx_dto *e = &d->tx_streams[d->tx_streams_len++];

		e->stream_id = tx[i].stream_id;
		strncpy(e->name, tx[i].name, sizeof(e->name) - 1);
		zsock_inet_ntop(AF_INET, &tx[i].dst_ip, e->dst_ip,
				sizeof(e->dst_ip));
		e->channel_count      = tx[i].channel_count;
		e->samples_per_packet = tx[i].samples_per_packet;
		e->ssrc               = tx[i].ssrc;
		e->ch_ids_len = MIN(tx[i].channel_count,
				    AES67_MAX_CH_PER_STREAM);
		for (size_t j = 0; j < e->ch_ids_len; j++) {
			e->ch_ids[j] = tx[i].ch_ids[j];
		}
	}

	for (int i = 0; i < AES67_MAX_RX_STREAMS; i++) {
		if (!rx[i].active) {
			continue;
		}

		struct cfg_rx_dto *e = &d->rx_streams[d->rx_streams_len++];

		e->stream_id = rx[i].stream_id;
		strncpy(e->name, rx[i].name, sizeof(e->name) - 1);
		strncpy(e->sender_name, rx[i].sender_name,
			sizeof(e->sender_name) - 1);
		zsock_inet_ntop(AF_INET, &rx[i].sender_ip, e->sender_ip,
				sizeof(e->sender_ip));
		zsock_inet_ntop(AF_INET, &rx[i].dst_ip, e->dst_ip,
				sizeof(e->dst_ip));
		e->dst_port            = rx[i].dst_port;
		e->channel_count       = rx[i].channel_count;
		e->output_delay        = rx[i].output_delay;
		e->samples_per_channel = rx[i].samples_per_channel;
		e->ch_map_len = MIN(rx[i].channel_count,
				    AES67_MAX_CH_PER_STREAM);
		for (size_t j = 0; j < e->ch_map_len; j++) {
			e->ch_map[j] = rx[i].ch_map[j];
		}
	}
}

/* Defaults for stream members a hand-written document may leave out.
 * The array parser decodes into the element structs in place without
 * clearing them first, so pre-set values survive for absent keys. */
static void prefill_stream_defaults(struct cfg_doc *d)
{
	for (int i = 0; i < AES67_MAX_TX_STREAMS; i++) {
		d->tx_streams[i].samples_per_packet = 48;
	}
	for (int i = 0; i < AES67_MAX_RX_STREAMS; i++) {
		d->rx_streams[i].dst_port            = 5004;
		d->rx_streams[i].output_delay        = 48;
		d->rx_streams[i].samples_per_channel = 48;
	}
}

/* ================================================================
 * Document -> runtime state
 * ================================================================ */

static void apply_device_config(const struct cfg_doc *d)
{
	struct aes67_device_config *cfg = aes67_config_get();

	aes67_config_lock();

	strncpy(cfg->vendor, d->vendor, AES67_VENDOR_MAX - 1);
	cfg->vendor[AES67_VENDOR_MAX - 1] = '\0';
	strncpy(cfg->product, d->product, AES67_PRODUCT_MAX - 1);
	cfg->product[AES67_PRODUCT_MAX - 1] = '\0';
	strncpy(cfg->serial, d->serial, AES67_SERIAL_MAX - 1);
	cfg->serial[AES67_SERIAL_MAX - 1] = '\0';
	strncpy(cfg->device_name, d->device_name, AES67_DEVICE_NAME_MAX - 1);
	cfg->device_name[AES67_DEVICE_NAME_MAX - 1] = '\0';
	strncpy(cfg->default_mcast_addr, d->default_mcast_addr,
		sizeof(cfg->default_mcast_addr) - 1);
	cfg->default_mcast_addr[sizeof(cfg->default_mcast_addr) - 1] = '\0';

	cfg->default_port            = d->default_port;
	cfg->default_channels        = d->default_channels;
	cfg->default_bit_depth       = d->default_bit_depth;
	cfg->default_sample_rate     = d->default_sample_rate;
	cfg->default_samples_per_pkt = d->default_samples_per_pkt;
	cfg->default_payload_type    = d->default_payload_type;

	cfg->ptp_domain                = d->ptp_domain;
	cfg->ptp_priority1             = d->ptp_priority1;
	cfg->ptp_priority2             = d->ptp_priority2;
	cfg->ptp_clock_class           = d->ptp_clock_class;
	cfg->ptp_clock_accuracy        = d->ptp_clock_accuracy;
	cfg->ptp_log_sync_interval     = d->ptp_log_sync_interval;
	cfg->ptp_log_announce_interval = d->ptp_log_announce_interval;
	cfg->ptp_delay_asymmetry_ns    = d->ptp_delay_asymmetry_ns;

	cfg->pi_kp_num        = d->pi_kp_num;
	cfg->pi_kp_den        = d->pi_kp_den;
	cfg->pi_ki_num        = d->pi_ki_num;
	cfg->pi_ki_den        = d->pi_ki_den;
	cfg->pi_imax          = d->pi_imax;
	cfg->pi_outlier_ppb   = d->pi_outlier_ppb;
	cfg->pi_warmup_cycles = d->pi_warmup_cycles;

	cfg->sap_announce_interval_s = d->sap_announce_interval_s;
	cfg->sap_announce_enabled    = d->sap_announce_enabled;

	aes67_config_unlock();
	LOG_INF("Device configuration loaded");
}

static void apply_card_settings(const struct cfg_doc *d)
{
	uint8_t type = card_type_from_str(d->card_type);

	if (type == CARD_TYPE_NONE) {
		return;
	}

	card_settings_lock();

	struct card_settings *cs = card_settings_get();

	memset(cs, 0, sizeof(*cs));
	cs->card_type = type;
	cs->num_in  = (uint8_t)d->card_in_gain_len;
	cs->num_out = (uint8_t)d->card_out_gain_len;

	for (size_t i = 0; i < d->card_in_gain_len; i++) {
		cs->in_gain[i] = d->card_in_gain[i];
	}
	for (size_t i = 0; i < d->card_in_48v_len; i++) {
		cs->in_phantom[i] = d->card_in_48v[i] != 0;
	}
	for (size_t i = 0; i < d->card_in_mute_len; i++) {
		cs->in_mute[i] = d->card_in_mute[i] != 0;
	}
	for (size_t i = 0; i < d->card_out_gain_len; i++) {
		cs->out_gain[i] = d->card_out_gain[i];
	}
	for (size_t i = 0; i < d->card_out_mute_len; i++) {
		cs->out_mute[i] = d->card_out_mute[i] != 0;
	}
	cs->valid = true;

	card_settings_unlock();

	LOG_INF("Card settings loaded (type=%s, %u in / %u out)",
		card_type_to_str(type), cs->num_in, cs->num_out);
}

static void apply_streams(const struct cfg_doc *d)
{
	int count = 0;

	for (size_t i = 0; i < d->tx_streams_len; i++) {
		const struct cfg_tx_dto *e = &d->tx_streams[i];
		struct in_addr dst = { 0 };
		uint8_t ch_ids[AES67_MAX_CH_PER_STREAM] = { 0 };
		uint8_t num_ch = (uint8_t)e->ch_ids_len;

		zsock_inet_pton(AF_INET, e->dst_ip, &dst);
		for (size_t j = 0; j < e->ch_ids_len; j++) {
			ch_ids[j] = e->ch_ids[j];
		}
		if (num_ch == 0) {
			/* No explicit map: identity, one entry per channel. */
			num_ch = MIN(e->channel_count,
				     AES67_MAX_CH_PER_STREAM);
			for (uint8_t j = 0; j < num_ch; j++) {
				ch_ids[j] = j;
			}
		}

		int ret = aes67_conn_configure_tx_stream(
			e->stream_id, &dst, e->channel_count,
			e->samples_per_packet, ch_ids, num_ch, e->ssrc,
			e->name[0] ? e->name : NULL);

		if (ret == 0) {
			count++;
		} else {
			LOG_ERR("Failed to configure TX stream %u: %d",
				e->stream_id, ret);
		}
	}
	LOG_INF("Loaded %d TX streams from config", count);

	count = 0;
	for (size_t i = 0; i < d->rx_streams_len; i++) {
		const struct cfg_rx_dto *e = &d->rx_streams[i];
		struct in_addr dst = { 0 };
		struct in_addr sender = { 0 };
		uint8_t ch_map[AES67_MAX_CH_PER_STREAM] = { 0 };

		zsock_inet_pton(AF_INET, e->dst_ip, &dst);
		if (e->sender_ip[0] != '\0') {
			zsock_inet_pton(AF_INET, e->sender_ip, &sender);
		}
		for (size_t j = 0; j < e->ch_map_len; j++) {
			ch_map[j] = e->ch_map[j];
		}
		if (e->ch_map_len == 0) {
			for (uint8_t j = 0; j < e->channel_count &&
					    j < AES67_MAX_CH_PER_STREAM; j++) {
				ch_map[j] = j;
			}
		}

		int ret = aes67_conn_configure_rx_stream(
			e->stream_id, &dst, e->dst_port, ch_map,
			e->channel_count, e->output_delay,
			e->samples_per_channel,
			e->name[0] ? e->name : NULL,
			e->sender_name[0] ? e->sender_name : NULL,
			sender.s_addr ? &sender : NULL);

		if (ret == 0) {
			count++;
		} else {
			LOG_ERR("Failed to configure RX stream %u: %d",
				e->stream_id, ret);
		}
	}
	LOG_INF("Loaded %d RX streams from config", count);
}

/* ================================================================
 * Public API
 * ================================================================ */

int config_json_serialize(char *buf, size_t sz)
{
	int ret;

	if (buf == NULL || sz == 0) {
		return -EINVAL;
	}

	k_mutex_lock(&doc_mutex, K_FOREVER);

	memset(&doc, 0, sizeof(doc));
	collect_device_config(&doc);
	collect_card_settings(&doc);
	collect_streams(&doc);

	ret = json_obj_encode_buf(doc_descr, ARRAY_SIZE(doc_descr), &doc,
				  buf, sz);

	k_mutex_unlock(&doc_mutex);

	if (ret < 0) {
		/* Almost always -ENOMEM: the caller's buffer is too small. */
		buf[0] = '\0';
		return ret;
	}
	return (int)strlen(buf);
}

int config_json_parse_and_apply(char *json)
{
	int64_t ret;

	if (json == NULL) {
		return -EINVAL;
	}

	k_mutex_lock(&doc_mutex, K_FOREVER);

	/* Pre-fill from the current state: the parser only writes the keys
	 * the document actually carries, so anything absent keeps its
	 * current value instead of reverting to zero. Stream tables and
	 * card settings start empty — an absent array means "leave the
	 * tables alone", not "clear them". */
	memset(&doc, 0, sizeof(doc));
	collect_device_config(&doc);
	prefill_stream_defaults(&doc);

	ret = json_obj_parse(json, strlen(json), doc_descr,
			     ARRAY_SIZE(doc_descr), &doc);
	if (ret < 0) {
		k_mutex_unlock(&doc_mutex);
		LOG_ERR("Config JSON is malformed (%d) - keeping current "
			"configuration", (int)ret);
		return (int)ret;
	}

	apply_device_config(&doc);
	apply_card_settings(&doc);
	apply_streams(&doc);

	k_mutex_unlock(&doc_mutex);

	/* Push the card section into the drivers — a no-op if the card
	 * isn't detected/ready yet; card_manager re-applies on detect and
	 * on output activation. */
	card_settings_apply();
	return 0;
}
