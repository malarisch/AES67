/*
 *
 * /api/streams/tx            — configured TX streams (GET)
 * /api/streams/tx/{id}       — configure / delete one TX slot (PUT/DELETE)
 * /api/streams/rx            — configured RX streams (GET)
 * /api/streams/rx/{id}       — configure / delete one RX slot (PUT/DELETE)
 * /api/streams/discovered    — foreign streams seen via SAP/mDNS (GET)
 */

#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>

#include "webapi_priv.h"
#include "../aes67_conn.h"
#include "../../drivers/fpga_hal/fpga_hal.h"

static void format_ip(char *out, size_t sz, const struct in_addr *addr)
{
	zsock_inet_ntop(AF_INET, addr, out, sz);
}

static void lookup_foreign_metadata(const struct in_addr *dst_ip,
				    uint16_t dst_port,
				    char *name,
				    size_t name_sz,
				    char *sender_name,
				    size_t sender_name_sz,
				    struct in_addr *sender_ip)
{
	int count;
	int64_t name_seen = INT64_MIN;
	int64_t sender_seen = INT64_MIN;
	int64_t address_seen = INT64_MIN;
	const struct aes67_foreign_stream *foreign =
		aes67_conn_get_foreign_streams(&count);

	ARG_UNUSED(count);
	name[0] = '\0';
	sender_name[0] = '\0';
	*sender_ip = (struct in_addr){0};

	for (int i = 0; i < AES67_MAX_FOREIGN_STREAMS; i++) {
		if (!foreign[i].valid ||
		    foreign[i].mcast_addr.s_addr != dst_ip->s_addr ||
		    foreign[i].port != dst_port) {
			continue;
		}
		if (foreign[i].name[0] != '\0' &&
		    foreign[i].last_seen_ms > name_seen) {
			strncpy(name, foreign[i].name, name_sz - 1);
			name[name_sz - 1] = '\0';
			name_seen = foreign[i].last_seen_ms;
		}
		if (foreign[i].sender_name[0] != '\0' &&
		    foreign[i].last_seen_ms > sender_seen) {
			strncpy(sender_name, foreign[i].sender_name,
				sender_name_sz - 1);
			sender_name[sender_name_sz - 1] = '\0';
			sender_seen = foreign[i].last_seen_ms;
		}
		if (foreign[i].origin_addr.s_addr != 0 &&
		    foreign[i].last_seen_ms > address_seen) {
			*sender_ip = foreign[i].origin_addr;
			address_seen = foreign[i].last_seen_ms;
		}
	}
}

static bool rx_name_is_fallback(const char *name)
{
	return name[0] == '\0' || strncmp(name, "RX Stream ", 10) == 0;
}

/* ---------------- TX ---------------- */

static int get_tx_streams(struct webapi_request *req)
{
	struct json_out *jo = &req->out;
	const struct aes67_tx_stream *txs = aes67_conn_get_tx_streams();
	char tmp[INET_ADDRSTRLEN];

	jo_obj_begin(jo);
	jo_key(jo, "tx_streams");
	jo_arr_begin(jo);

	for (int i = 0; i < AES67_MAX_TX_STREAMS; i++) {
		if (!txs[i].active) {
			continue;
		}
		jo_obj_begin(jo);
		jo_uint(jo, "stream_id", txs[i].stream_id);
		jo_str(jo, "name", txs[i].name);
		format_ip(tmp, sizeof(tmp), &txs[i].dst_ip);
		jo_str(jo, "dst_ip", tmp);
		jo_uint(jo, "channel_count", txs[i].channel_count);
		jo_uint(jo, "samples_per_packet", txs[i].samples_per_packet);

		jo_key(jo, "ch_ids");
		jo_arr_begin(jo);
		for (int j = 0; j < txs[i].channel_count; j++) {
			jo_fmt(jo, "%u,", txs[i].ch_ids[j]);
		}
		jo_arr_end(jo);
		jo_obj_end(jo);
	}

	jo_arr_end(jo);
	jo_obj_end(jo);
	return 0;
}

/* PUT /api/streams/tx/{id} body. dst_ip is required — its presence is
 * checked through the parser's bitmap; everything else falls back to the
 * defaults pre-set before parsing. */
struct tx_stream_req {
	char     dst_ip[INET_ADDRSTRLEN];
	char     name[AES67_STREAM_NAME_MAX];
	int32_t  channel_count;
	int32_t  samples_per_pkt;
	uint8_t  ch_ids[AES67_MAX_CH_PER_STREAM];
	size_t   ch_ids_len;
};

enum { TX_F_DST_IP = 0 };

static const struct json_obj_descr tx_stream_descr[] = {
	[TX_F_DST_IP] = JSON_OBJ_DESCR_PRIM(struct tx_stream_req, dst_ip,
					    JSON_TOK_STRING_BUF),
	JSON_OBJ_DESCR_PRIM(struct tx_stream_req, name, JSON_TOK_STRING_BUF),
	JSON_OBJ_DESCR_PRIM(struct tx_stream_req, channel_count,
			    JSON_TOK_NUMBER),
	JSON_OBJ_DESCR_PRIM(struct tx_stream_req, samples_per_pkt,
			    JSON_TOK_NUMBER),
	JSON_OBJ_DESCR_ARRAY(struct tx_stream_req, ch_ids,
			     AES67_MAX_CH_PER_STREAM, ch_ids_len,
			     JSON_TOK_UINT),
};

static int put_tx_stream(struct webapi_request *req)
{
	struct tx_stream_req r = {
		.channel_count = 0,
		.samples_per_pkt = 48,
	};
	int64_t present;

	if (req->id < 0 || req->id >= AES67_MAX_TX_STREAMS) {
		return -EINVAL;
	}

	r.dst_ip[0] = '\0';
	r.name[0] = '\0';
	r.ch_ids_len = 0;

	present = webapi_parse_body(req, tx_stream_descr,
				    ARRAY_SIZE(tx_stream_descr), &r);
	if (present < 0 || !WEBAPI_HAS(present, TX_F_DST_IP)) {
		return -EINVAL;
	}

	struct in_addr dst;

	if (zsock_inet_pton(AF_INET, r.dst_ip, &dst) != 1) {
		return -EINVAL;
	}

	int32_t channel_count = r.channel_count;

	if (channel_count < 1 || channel_count > AES67_MAX_CH_PER_STREAM) {
		channel_count = 2;
	}

	uint8_t ch_ids[AES67_MAX_CH_PER_STREAM] = {0};

	for (size_t i = 0; i < r.ch_ids_len && i < (size_t)channel_count; i++) {
		ch_ids[i] = r.ch_ids[i];
	}
	if (r.ch_ids_len == 0) {
		/* Default: sequential channel IDs */
		for (int i = 0; i < channel_count; i++) {
			ch_ids[i] = (uint8_t)i;
		}
	}

	int ret = aes67_conn_configure_tx_stream((uint8_t)req->id, &dst,
					(uint8_t)channel_count,
					(uint8_t)r.samples_per_pkt,
					ch_ids, (uint8_t)channel_count,
					0,
					r.name[0] ? r.name : NULL);

	if (ret < 0) {
		return ret;
	}

	webapi_persist_config();
	return 0;
}

static int delete_tx_stream(struct webapi_request *req)
{
	if (req->id < 0 || req->id >= AES67_MAX_TX_STREAMS) {
		return -EINVAL;
	}

	/* Deactivate stream: write zero config to FPGA and clear local table */
	struct in_addr zero_ip = {.s_addr = 0};
	uint8_t zero_ch[AES67_MAX_CH_PER_STREAM] = {0};

	int ret = aes67_conn_configure_tx_stream((uint8_t)req->id, &zero_ip,
						 0, 0, zero_ch, 0, 0, NULL);

	if (ret < 0) {
		return ret;
	}

	webapi_persist_config();
	return 0;
}

/* ---------------- RX ---------------- */

static int get_rx_streams(struct webapi_request *req)
{
	struct json_out *jo = &req->out;
	const struct aes67_rx_stream *rxs = aes67_conn_get_rx_streams();
	char tmp[INET_ADDRSTRLEN];
	char discovered_name[AES67_STREAM_NAME_MAX];
	char discovered_sender[AES67_STREAM_NAME_MAX];
	struct in_addr discovered_sender_ip;
	/* Per-stream underrun flags from the FPGA status CSR (streams 3..0;
	 * the FPGA slot index equals the table index). */
	uint32_t rx_underrun = (fpga_hal_read_status() &
				FPGA_HAL_RX_UNDERRUN_MASK) >>
			       FPGA_HAL_RX_UNDERRUN_SHIFT;

	jo_obj_begin(jo);
	jo_key(jo, "rx_streams");
	jo_arr_begin(jo);

	for (int i = 0; i < AES67_MAX_RX_STREAMS; i++) {
		if (!rxs[i].active) {
			continue;
		}
		jo_obj_begin(jo);
		jo_uint(jo, "stream_id", rxs[i].stream_id);
		lookup_foreign_metadata(&rxs[i].dst_ip, rxs[i].dst_port,
					discovered_name,
					sizeof(discovered_name),
					discovered_sender,
					sizeof(discovered_sender),
					&discovered_sender_ip);
		jo_str(jo, "name",
		       rx_name_is_fallback(rxs[i].name) &&
		       discovered_name[0] != '\0' ?
		       discovered_name : rxs[i].name);
		jo_str(jo, "sender_name",
		       rxs[i].sender_name[0] != '\0' ?
		       rxs[i].sender_name : discovered_sender);
		const struct in_addr *sender_ip =
			rxs[i].sender_ip.s_addr != 0 ?
			&rxs[i].sender_ip : &discovered_sender_ip;

		if (sender_ip->s_addr != 0) {
			format_ip(tmp, sizeof(tmp), sender_ip);
			jo_str(jo, "sender_ip", tmp);
		} else {
			jo_str(jo, "sender_ip", "");
		}
		format_ip(tmp, sizeof(tmp), &rxs[i].dst_ip);
		jo_str(jo, "dst_ip", tmp);
		jo_uint(jo, "dst_port", rxs[i].dst_port);
		jo_uint(jo, "channel_count", rxs[i].channel_count);
		jo_uint(jo, "output_delay", rxs[i].output_delay);
		jo_uint(jo, "samples_per_channel", rxs[i].samples_per_channel);
		jo_bool(jo, "underrun", i < 4 && ((rx_underrun >> i) & 1U));

		jo_key(jo, "ch_map");
		jo_arr_begin(jo);
		for (int j = 0; j < rxs[i].channel_count; j++) {
			jo_fmt(jo, "%u,", rxs[i].ch_map[j]);
		}
		jo_arr_end(jo);
		jo_obj_end(jo);
	}

	jo_arr_end(jo);
	jo_obj_end(jo);
	return 0;
}

/* PUT /api/streams/rx/{id} body — same shape as the TX one, plus the
 * sender metadata the discovery table can fill in. */
struct rx_stream_req {
	char     dst_ip[INET_ADDRSTRLEN];
	char     name[AES67_STREAM_NAME_MAX];
	char     sender_name[AES67_STREAM_NAME_MAX];
	char     sender_ip[INET_ADDRSTRLEN];
	int32_t  dst_port;
	int32_t  channel_count;
	int32_t  output_delay;
	int32_t  samples_per_channel;
	uint8_t  ch_map[AES67_MAX_CH_PER_STREAM];
	size_t   ch_map_len;
};

enum { RX_F_DST_IP = 0 };

static const struct json_obj_descr rx_stream_descr[] = {
	[RX_F_DST_IP] = JSON_OBJ_DESCR_PRIM(struct rx_stream_req, dst_ip,
					    JSON_TOK_STRING_BUF),
	JSON_OBJ_DESCR_PRIM(struct rx_stream_req, name, JSON_TOK_STRING_BUF),
	JSON_OBJ_DESCR_PRIM(struct rx_stream_req, sender_name,
			    JSON_TOK_STRING_BUF),
	JSON_OBJ_DESCR_PRIM(struct rx_stream_req, sender_ip,
			    JSON_TOK_STRING_BUF),
	JSON_OBJ_DESCR_PRIM(struct rx_stream_req, dst_port, JSON_TOK_NUMBER),
	JSON_OBJ_DESCR_PRIM(struct rx_stream_req, channel_count,
			    JSON_TOK_NUMBER),
	JSON_OBJ_DESCR_PRIM(struct rx_stream_req, output_delay,
			    JSON_TOK_NUMBER),
	JSON_OBJ_DESCR_PRIM(struct rx_stream_req, samples_per_channel,
			    JSON_TOK_NUMBER),
	JSON_OBJ_DESCR_ARRAY(struct rx_stream_req, ch_map,
			     AES67_MAX_CH_PER_STREAM, ch_map_len,
			     JSON_TOK_UINT),
};

static int put_rx_stream(struct webapi_request *req)
{
	struct rx_stream_req r = {
		.dst_port = 5004,
		.channel_count = 0,
		.output_delay = 0,
		.samples_per_channel = 48,
	};
	struct in_addr sender_ip = {0};
	struct in_addr dst_ip = {0};
	int64_t present;

	if (req->id < 0 || req->id >= AES67_MAX_RX_STREAMS) {
		return -EINVAL;
	}

	r.dst_ip[0] = '\0';
	r.name[0] = '\0';
	r.sender_name[0] = '\0';
	r.sender_ip[0] = '\0';
	r.ch_map_len = 0;

	present = webapi_parse_body(req, rx_stream_descr,
				    ARRAY_SIZE(rx_stream_descr), &r);
	/* Destination IP address / multicast group (required) */
	if (present < 0 || !WEBAPI_HAS(present, RX_F_DST_IP)) {
		return -EINVAL;
	}
	if (zsock_inet_pton(AF_INET, r.dst_ip, &dst_ip) != 1) {
		return -EINVAL;
	}
	if (r.sender_ip[0] != '\0') {
		(void)zsock_inet_pton(AF_INET, r.sender_ip, &sender_ip);
	}

	int32_t dst_port_val = r.dst_port;
	int32_t channel_count = r.channel_count;
	int32_t output_delay = r.output_delay;
	int32_t samples_per_channel = r.samples_per_channel;
	char *stream_name = r.name;
	char *sender_name = r.sender_name;

	if (channel_count < 1 || channel_count > AES67_MAX_CH_PER_STREAM) {
		channel_count = 2;
	}
	if (samples_per_channel < 1 || samples_per_channel > 255) {
		samples_per_channel = 48;
	}

	uint8_t ch_map[AES67_MAX_CH_PER_STREAM] = {0};

	for (size_t i = 0; i < r.ch_map_len && i < (size_t)channel_count; i++) {
		ch_map[i] = r.ch_map[i];
	}
	if (r.ch_map_len == 0) {
		/* Default: identity mapping */
		for (int i = 0; i < channel_count; i++) {
			ch_map[i] = (uint8_t)i;
		}
	}

	/* Fill omitted metadata from the current discovery table. This keeps
	 * non-browser clients useful while making the subscription persistent
	 * even after the discovery sighting expires. */
	if (stream_name[0] == '\0' || sender_name[0] == '\0' ||
	    sender_ip.s_addr == 0) {
		char discovered_name[AES67_STREAM_NAME_MAX];
		char discovered_sender[AES67_STREAM_NAME_MAX];
		struct in_addr discovered_sender_ip;

		lookup_foreign_metadata(&dst_ip, (uint16_t)dst_port_val,
					discovered_name,
					sizeof(discovered_name),
					discovered_sender,
					sizeof(discovered_sender),
					&discovered_sender_ip);
		if (stream_name[0] == '\0' && discovered_name[0] != '\0') {
			strncpy(stream_name, discovered_name,
				sizeof(r.name) - 1);
			stream_name[sizeof(r.name) - 1] = '\0';
		}
		if (sender_name[0] == '\0' &&
		    discovered_sender[0] != '\0') {
			strncpy(sender_name, discovered_sender,
				sizeof(r.sender_name) - 1);
			sender_name[sizeof(r.sender_name) - 1] = '\0';
		}
		if (sender_ip.s_addr == 0) {
			sender_ip = discovered_sender_ip;
		}
	}

	int ret = aes67_conn_configure_rx_stream((uint8_t)req->id,
					&dst_ip,
					(uint16_t)dst_port_val,
					ch_map,
					(uint8_t)channel_count,
					(uint8_t)output_delay,
					(uint8_t)samples_per_channel,
					stream_name[0] ? stream_name : NULL,
					sender_name[0] ? sender_name : NULL,
					sender_ip.s_addr ? &sender_ip : NULL);

	if (ret < 0) {
		return ret;
	}

	webapi_persist_config();
	return 0;
}

static int delete_rx_stream(struct webapi_request *req)
{
	if (req->id < 0 || req->id >= AES67_MAX_RX_STREAMS) {
		return -EINVAL;
	}

	/* Zero IP+port config effectively disables the stream in the FPGA */
	struct in_addr zero_ip = {.s_addr = 0};
	uint8_t zero_map[AES67_MAX_CH_PER_STREAM] = {0};

	int ret = aes67_conn_configure_rx_stream((uint8_t)req->id, &zero_ip,
						 0, zero_map, 1, 0, 0,
						 NULL, NULL, NULL);

	if (ret < 0) {
		return ret;
	}

	webapi_persist_config();
	return 0;
}

/* ---------------- Discovered (foreign) streams ---------------- */

static int get_discovered(struct webapi_request *req)
{
	struct json_out *jo = &req->out;
	char tmp[INET_ADDRSTRLEN];
	char discovered_name[AES67_STREAM_NAME_MAX];
	char discovered_sender[AES67_STREAM_NAME_MAX];
	struct in_addr discovered_sender_ip;
	int sap_count = 0;
	const struct aes67_foreign_stream *foreign =
		aes67_conn_get_foreign_streams(&sap_count);

	jo_obj_begin(jo);
	jo_key(jo, "discovered_streams");
	jo_arr_begin(jo);

	for (int i = 0; i < AES67_MAX_FOREIGN_STREAMS; i++) {
		if (!foreign[i].valid) {
			continue;
		}

		/* Merge sightings of the same RTP flow (same group + port,
		 * e.g. announced via SAP *and* mDNS) into one row, like the
		 * Linux daemon's Registry::snapshot(): the freshest sighting
		 * is the representative, the "via" transports are OR-ed, and
		 * duplicates already emitted with an earlier row are skipped. */
		const struct aes67_foreign_stream *rep = &foreign[i];
		uint8_t via = foreign[i].via;
		bool already_emitted = false;

		for (int j = 0; j < AES67_MAX_FOREIGN_STREAMS; j++) {
			if (j == i || !foreign[j].valid ||
			    foreign[j].mcast_addr.s_addr !=
				    foreign[i].mcast_addr.s_addr ||
			    foreign[j].port != foreign[i].port) {
				continue;
			}
			if (j < i) {
				already_emitted = true;
				break;
			}
			via |= foreign[j].via;
			if (foreign[j].last_seen_ms > rep->last_seen_ms) {
				rep = &foreign[j];
			}
		}
		if (already_emitted) {
			continue;
		}

		lookup_foreign_metadata(&rep->mcast_addr, rep->port,
					discovered_name,
					sizeof(discovered_name),
					discovered_sender,
					sizeof(discovered_sender),
					&discovered_sender_ip);

		jo_obj_begin(jo);
		jo_str(jo, "name",
		       discovered_name[0] ? discovered_name : rep->name);
		jo_str(jo, "sender_name", discovered_sender);
		format_ip(tmp, sizeof(tmp), &rep->mcast_addr);
		jo_str(jo, "mcast_addr", tmp);
		jo_uint(jo, "port", rep->port);
		jo_uint(jo, "channels", rep->channels);
		jo_uint(jo, "bit_depth", rep->bit_depth);
		jo_uint(jo, "sample_rate", rep->sample_rate);
		jo_uint(jo, "samples_per_packet", rep->samples_per_packet);
		if (rep->ssrc != 0) {
			jo_fmt(jo, "\"ssrc\":\"%08X\",", rep->ssrc);
		} else {
			jo_raw(jo, "\"ssrc\":\"\",");
		}
		format_ip(tmp, sizeof(tmp),
			  discovered_sender_ip.s_addr ?
			  &discovered_sender_ip : &rep->origin_addr);
		jo_str(jo, "origin_addr", tmp);
		jo_str(jo, "via",
		       (via & AES67_VIA_SAP) &&
		       (via & AES67_VIA_MDNS) ? "SAP + mDNS" :
		       (via & AES67_VIA_MDNS) ? "mDNS" : "SAP");
		jo_obj_end(jo);
	}

	jo_arr_end(jo);
	jo_obj_end(jo);
	return 0;
}

static const struct webapi_route routes[] = {
	WEBAPI_ROUTE(HTTP_GET,    "/api/streams/tx",         get_tx_streams),
	WEBAPI_ROUTE(HTTP_PUT,    "/api/streams/tx/{id}",    put_tx_stream),
	WEBAPI_ROUTE(HTTP_DELETE, "/api/streams/tx/{id}",    delete_tx_stream),
	WEBAPI_ROUTE(HTTP_GET,    "/api/streams/rx",         get_rx_streams),
	WEBAPI_ROUTE(HTTP_PUT,    "/api/streams/rx/{id}",    put_rx_stream),
	WEBAPI_ROUTE(HTTP_DELETE, "/api/streams/rx/{id}",    delete_rx_stream),
	WEBAPI_ROUTE(HTTP_GET,    "/api/streams/discovered", get_discovered),
};

const struct webapi_module webapi_streams_module = {
	.routes = routes,
	.count = ARRAY_SIZE(routes),
};
