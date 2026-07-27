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

static int put_tx_stream(struct webapi_request *req)
{
	const char *json = req->body;
	size_t len = req->body_len;
	int32_t channel_count = 0;
	int32_t samples_per_pkt = 48;
	char dst_ip_str[INET_ADDRSTRLEN] = {0};
	char stream_name[AES67_STREAM_NAME_MAX] = {0};

	if (req->id < 0 || req->id >= AES67_MAX_TX_STREAMS) {
		return -EINVAL;
	}

	if (json_find_str(json, len, "dst_ip", dst_ip_str,
			  sizeof(dst_ip_str)) <= 0) {
		return -EINVAL;
	}

	/* Optional stream name */
	json_find_str(json, len, "name", stream_name, sizeof(stream_name));

	struct in_addr dst;

	if (zsock_inet_pton(AF_INET, dst_ip_str, &dst) != 1) {
		return -EINVAL;
	}

	json_find_int(json, len, "channel_count", &channel_count);
	json_find_int(json, len, "samples_per_pkt", &samples_per_pkt);

	if (channel_count < 1 || channel_count > AES67_MAX_CH_PER_STREAM) {
		channel_count = 2;
	}

	uint8_t ch_ids[AES67_MAX_CH_PER_STREAM] = {0};
	int n = json_find_u8_array(json, len, "ch_ids", ch_ids,
				   MIN(channel_count,
				       AES67_MAX_CH_PER_STREAM));

	if (n == 0) {
		/* Default: sequential channel IDs */
		for (int i = 0; i < channel_count; i++) {
			ch_ids[i] = (uint8_t)i;
		}
	}

	int ret = aes67_conn_configure_tx_stream((uint8_t)req->id, &dst,
					(uint8_t)channel_count,
					(uint8_t)samples_per_pkt,
					ch_ids, (uint8_t)channel_count,
					0,
					stream_name[0] ? stream_name : NULL);

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

static int put_rx_stream(struct webapi_request *req)
{
	const char *json = req->body;
	size_t len = req->body_len;
	int32_t channel_count = 0;
	int32_t output_delay = 0;
	int32_t samples_per_channel = 48;
	int32_t dst_port_val = 5004;

	if (req->id < 0 || req->id >= AES67_MAX_RX_STREAMS) {
		return -EINVAL;
	}

	/* Destination IP address / multicast group (required) */
	char ip_str[INET_ADDRSTRLEN] = {0};
	struct in_addr dst_ip = {0};

	if (json_find_str(json, len, "dst_ip", ip_str, sizeof(ip_str)) <= 0) {
		return -EINVAL;
	}
	if (zsock_inet_pton(AF_INET, ip_str, &dst_ip) != 1) {
		return -EINVAL;
	}

	json_find_int(json, len, "dst_port", &dst_port_val);
	json_find_int(json, len, "channel_count", &channel_count);
	json_find_int(json, len, "output_delay", &output_delay);
	json_find_int(json, len, "samples_per_channel", &samples_per_channel);

	if (channel_count < 1 || channel_count > AES67_MAX_CH_PER_STREAM) {
		channel_count = 2;
	}
	if (samples_per_channel < 1 || samples_per_channel > 255) {
		samples_per_channel = 48;
	}

	uint8_t ch_map[AES67_MAX_CH_PER_STREAM] = {0};
	int n = json_find_u8_array(json, len, "ch_map", ch_map,
				   MIN(channel_count,
				       AES67_MAX_CH_PER_STREAM));

	if (n == 0) {
		/* Default: identity mapping */
		for (int i = 0; i < channel_count; i++) {
			ch_map[i] = (uint8_t)i;
		}
	}

	int ret = aes67_conn_configure_rx_stream((uint8_t)req->id,
					&dst_ip,
					(uint16_t)dst_port_val,
					ch_map,
					(uint8_t)channel_count,
					(uint8_t)output_delay,
					(uint8_t)samples_per_channel);

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
						 0, zero_map, 1, 0, 0);

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

		jo_obj_begin(jo);
		jo_str(jo, "name", rep->name);
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
		format_ip(tmp, sizeof(tmp), &rep->origin_addr);
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
