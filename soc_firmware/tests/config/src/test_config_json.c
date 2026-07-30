/*
 * Unit tests for the persisted configuration format
 * (src/config_json.c) — the exact bytes written to SD card and SPI
 * flash, and read back at every boot.
 *
 * A round trip that loses a field silently reverts the device to
 * defaults after a power cycle, so the roundtrip test compares the
 * whole configuration and both stream tables.
 */

#include <zephyr/ztest.h>
#include <zephyr/net/net_ip.h>
#include <errno.h>
#include <string.h>

#include "aes67_config.h"
#include "aes67_conn.h"
#include "config_json.h"

#define JSON_BUF 8192

static char json[JSON_BUF];

static struct in_addr addr(const char *s)
{
	struct in_addr a;

	zassert_equal(net_addr_pton(AF_INET, s, &a), 0, "bad address %s", s);
	return a;
}

static const struct in_addr zero_addr;

static void clear_streams(void)
{
	static const uint8_t ch[AES67_MAX_CH_PER_STREAM] = { 0 };

	for (uint8_t i = 0; i < AES67_MAX_TX_STREAMS; i++) {
		(void)aes67_conn_configure_tx_stream(i, &zero_addr, 0, 48, ch,
						     0, 0, NULL);
	}
	for (uint8_t i = 0; i < AES67_MAX_RX_STREAMS; i++) {
		(void)aes67_conn_configure_rx_stream(i, &zero_addr, 0, ch, 1, 0,
						     48, NULL, NULL, NULL);
	}
}

static void reset_all(void *fixture)
{
	ARG_UNUSED(fixture);
	(void)aes67_config_get();
	aes67_config_reset_defaults();
	clear_streams();
	memset(json, 0, sizeof(json));
}

ZTEST(config_json, test_serialize_defaults_is_wellformed)
{
	int len = config_json_serialize(json, sizeof(json));

	zassert_true(len > 0 && len < (int)sizeof(json),
		     "serialize returned %d", len);
	zassert_equal((size_t)len, strlen(json));
	zassert_equal(json[0], '{');
	zassert_equal(json[len - 1], '}');

	/* No dangling comma before a closing brace/bracket. */
	for (int i = 0; i + 2 < len; i++) {
		if (json[i] != ',') {
			continue;
		}
		int j = i + 1;

		while (j < len && (json[j] == ' ' || json[j] == '\n' ||
				   json[j] == '\t' || json[j] == '\r')) {
			j++;
		}
		zassert_true(json[j] != '}' && json[j] != ']',
			     "trailing comma at offset %d", i);
	}

	/* Empty stream tables still emit their (empty) arrays. */
	zassert_not_null(strstr(json, "\"tx_streams\""));
	zassert_not_null(strstr(json, "\"rx_streams\""));
	zassert_not_null(strstr(json, "\"vendor\":\"AES67\""));
}

ZTEST(config_json, test_roundtrip_device_config)
{
	struct aes67_device_config *c = aes67_config_get();

	strcpy(c->vendor, "Acme");
	strcpy(c->product, "StageBox");
	strcpy(c->serial, "SN-42");
	strcpy(c->device_name, "Studio B");
	strcpy(c->default_mcast_addr, "239.70.1.2");
	c->default_port = 5006;
	c->default_channels = 8;
	c->default_bit_depth = 16;
	c->default_sample_rate = 96000;
	c->default_samples_per_pkt = 96;
	c->default_payload_type = 98;
	c->ptp_domain = 7;
	c->ptp_priority1 = 100;
	c->ptp_priority2 = 110;
	c->ptp_log_sync_interval = -4;
	c->ptp_log_announce_interval = 1;
	c->ptp_delay_asymmetry_ns = -1234;
	c->pi_kp_num = 3;
	c->pi_kp_den = 8;
	c->pi_ki_num = 5;
	c->pi_ki_den = 64;
	c->pi_imax = 400000;
	c->pi_outlier_ppb = 12345678;
	c->pi_warmup_cycles = 9;
	c->sap_announce_interval_s = 15;
	c->sap_announce_enabled = false;

	struct aes67_device_config saved = *c;
	int len = config_json_serialize(json, sizeof(json));

	zassert_true(len > 0 && len < (int)sizeof(json));

	aes67_config_reset_defaults();
	zassert_str_equal(c->vendor, "AES67", "precondition: defaults restored");

	zassert_equal(config_json_parse_and_apply(json), 0);

	zassert_str_equal(c->vendor, saved.vendor);
	zassert_str_equal(c->product, saved.product);
	zassert_str_equal(c->serial, saved.serial);
	zassert_str_equal(c->device_name, saved.device_name);
	zassert_str_equal(c->default_mcast_addr, saved.default_mcast_addr);
	zassert_equal(c->default_port, saved.default_port);
	zassert_equal(c->default_channels, saved.default_channels);
	zassert_equal(c->default_bit_depth, saved.default_bit_depth);
	zassert_equal(c->default_sample_rate, saved.default_sample_rate);
	zassert_equal(c->default_samples_per_pkt, saved.default_samples_per_pkt);
	zassert_equal(c->default_payload_type, saved.default_payload_type);
	zassert_equal(c->ptp_domain, saved.ptp_domain);
	zassert_equal(c->ptp_priority1, saved.ptp_priority1);
	zassert_equal(c->ptp_priority2, saved.ptp_priority2);
	zassert_equal(c->ptp_log_sync_interval, saved.ptp_log_sync_interval);
	zassert_equal(c->ptp_log_announce_interval,
		      saved.ptp_log_announce_interval);
	zassert_equal(c->ptp_delay_asymmetry_ns, saved.ptp_delay_asymmetry_ns);
	zassert_equal(c->pi_kp_num, saved.pi_kp_num);
	zassert_equal(c->pi_kp_den, saved.pi_kp_den);
	zassert_equal(c->pi_ki_num, saved.pi_ki_num);
	zassert_equal(c->pi_ki_den, saved.pi_ki_den);
	zassert_equal(c->pi_imax, saved.pi_imax);
	zassert_equal(c->pi_outlier_ppb, saved.pi_outlier_ppb);
	zassert_equal(c->pi_warmup_cycles, saved.pi_warmup_cycles);
	zassert_equal(c->sap_announce_interval_s, saved.sap_announce_interval_s);
	zassert_equal(c->sap_announce_enabled, saved.sap_announce_enabled);
}

ZTEST(config_json, test_roundtrip_streams)
{
	const uint8_t tx_ch[4] = { 7, 6, 5, 4 };
	const uint8_t rx_map[2] = { 2, 3 };
	struct in_addr tx_dst = addr("239.69.0.11");
	struct in_addr rx_dst = addr("239.69.1.22");
	struct in_addr sender = addr("10.1.2.3");

	zassert_equal(aes67_conn_configure_tx_stream(0, &tx_dst, 4, 24, tx_ch,
						     4, 0xDEADBEEF, "Front"),
		      0);
	zassert_equal(aes67_conn_configure_tx_stream(3, &tx_dst, 2, 48, tx_ch,
						     2, 0, NULL), 0);
	zassert_equal(aes67_conn_configure_rx_stream(2, &rx_dst, 5008, rx_map,
						     2, 32, 48, "Monitor",
						     "Console", &sender), 0);

	uint32_t tx3_ssrc = aes67_conn_get_tx_streams()[3].ssrc;
	int len = config_json_serialize(json, sizeof(json));

	zassert_true(len > 0 && len < (int)sizeof(json));

	clear_streams();
	zassert_false(aes67_conn_get_tx_streams()[0].active);

	zassert_equal(config_json_parse_and_apply(json), 0);

	const struct aes67_tx_stream *tx = aes67_conn_get_tx_streams();
	const struct aes67_rx_stream *rx = aes67_conn_get_rx_streams();

	zassert_true(tx[0].active);
	zassert_str_equal(tx[0].name, "Front");
	zassert_equal(tx[0].dst_ip.s_addr, tx_dst.s_addr);
	zassert_equal(tx[0].channel_count, 4);
	zassert_equal(tx[0].samples_per_packet, 24);
	zassert_equal(tx[0].ssrc, 0xDEADBEEF);
	zassert_mem_equal(tx[0].ch_ids, tx_ch, 4);

	/* The auto-generated SSRC is persisted, not regenerated. */
	zassert_true(tx[3].active);
	zassert_equal(tx[3].ssrc, tx3_ssrc);

	/* Slots that were inactive at save time stay inactive. */
	zassert_false(tx[1].active);
	zassert_false(tx[7].active);

	zassert_true(rx[2].active);
	zassert_str_equal(rx[2].name, "Monitor");
	zassert_str_equal(rx[2].sender_name, "Console");
	zassert_equal(rx[2].sender_ip.s_addr, sender.s_addr);
	zassert_equal(rx[2].dst_ip.s_addr, rx_dst.s_addr);
	zassert_equal(rx[2].dst_port, 5008);
	zassert_equal(rx[2].channel_count, 2);
	zassert_equal(rx[2].output_delay, 32);
	zassert_equal(rx[2].samples_per_channel, 48);
	zassert_mem_equal(rx[2].ch_map, rx_map, 2);
	zassert_false(rx[0].active);
}

ZTEST(config_json, test_partial_json_keeps_defaults)
{
	/* The parser NUL-terminates tokens in place, so the document has
	 * to live in writable memory — never a string literal. */
	char partial[] =
		"{\n  \"device_name\": \"Only This\",\n"
		"  \"ptp_domain\": 5\n}\n";
	struct aes67_device_config *c = aes67_config_get();

	zassert_equal(config_json_parse_and_apply(partial), 0);

	zassert_str_equal(c->device_name, "Only This");
	zassert_equal(c->ptp_domain, 5);
	/* Everything not mentioned keeps its current value. */
	zassert_str_equal(c->vendor, "AES67");
	zassert_equal(c->default_port, 5004);
	zassert_true(c->sap_announce_enabled);
	/* No stream arrays: the tables are left alone, not wiped. */
	zassert_false(aes67_conn_get_tx_streams()[0].active);
}

ZTEST(config_json, test_whitespace_and_key_order_are_irrelevant)
{
	/* Compact, reordered and unknown keys all have to parse — this is
	 * what a hand-edited or externally generated config looks like. */
	char compact[] =
		"{\"ptp_domain\":9,\"unknown_key\":{\"nested\":[1,2]},"
		"\"device_name\":\"Compact\",\"vendor\":\"X\"}";
	char spaced[] =
		"{ \"ptp_domain\" : 11 , \"device_name\" : \"Spaced\" }";
	struct aes67_device_config *c = aes67_config_get();

	zassert_equal(config_json_parse_and_apply(compact), 0);
	zassert_equal(c->ptp_domain, 9);
	zassert_str_equal(c->device_name, "Compact");
	zassert_str_equal(c->vendor, "X");

	zassert_equal(config_json_parse_and_apply(spaced), 0);
	zassert_equal(c->ptp_domain, 11);
	zassert_str_equal(c->device_name, "Spaced");
}

ZTEST(config_json, test_escaped_strings_roundtrip)
{
	struct aes67_device_config *c = aes67_config_get();

	strcpy(c->device_name, "Studio \"B\"\\Rack");

	int len = config_json_serialize(json, sizeof(json));

	zassert_true(len > 0);
	/* The value must be escaped on the way out... */
	zassert_not_null(strstr(json, "Studio \\\"B\\\"\\\\Rack"));

	aes67_config_reset_defaults();
	zassert_equal(config_json_parse_and_apply(json), 0);
	/* ...and unescaped on the way back in. */
	zassert_str_equal(c->device_name, "Studio \"B\"\\Rack");
}

ZTEST(config_json, test_malformed_json_is_rejected_atomically)
{
	struct aes67_device_config *c = aes67_config_get();
	char empty[] = "";
	char truncated[] = "{";
	char not_json[] = "not json at all";
	char half_stream[] = "{\"tx_streams\": [{\"stream_id\": ";
	/* A value that does not fit its field: rejected rather than
	 * silently truncated to 44 by a cast. */
	char out_of_range[] = "{\"device_name\":\"Bad\",\"default_channels\":300}";

	zassert_true(config_json_parse_and_apply(empty) < 0);
	zassert_true(config_json_parse_and_apply(truncated) < 0);
	zassert_true(config_json_parse_and_apply(not_json) < 0);
	zassert_true(config_json_parse_and_apply(half_stream) < 0);
	zassert_true(config_json_parse_and_apply(out_of_range) < 0);
	zassert_equal(config_json_parse_and_apply(NULL), -EINVAL);

	/* A rejected document applies nothing at all — not even the keys
	 * that appeared before the bad one. The device keeps running on
	 * its current configuration. */
	zassert_str_equal(c->vendor, "AES67");
	zassert_str_equal(c->device_name, "AES67 Node");
	zassert_equal(c->default_port, 5004);
	zassert_equal(c->default_channels, 2);
}

ZTEST(config_json, test_serialize_reports_overflow)
{
	/* Undersized buffers must report an error and leave the
	 * surrounding memory alone — no writing past the end, and no
	 * half-written document handed to the storage backend. */
	static struct {
		uint8_t front[32];
		char body[512];
		uint8_t back[32];
	} g;

	/* Give the serializer something to overflow with. */
	const uint8_t ch[2] = { 0, 1 };
	struct in_addr dst = addr("239.69.0.1");

	zassert_equal(aes67_conn_configure_tx_stream(0, &dst, 2, 48, ch, 2, 0,
						     "Guarded"), 0);

	memset(&g, 0xA5, sizeof(g));

	for (size_t sz = 1; sz <= sizeof(g.body); sz++) {
		int len = config_json_serialize(g.body, sz);

		zassert_true(len < 0,
			     "size %zu: expected an overflow report, got %d",
			     sz, len);
		zassert_equal(g.body[0], '\0',
			      "size %zu: a failed encode must not leave a "
			      "partial document behind", sz);
		for (size_t i = 0; i < sizeof(g.front); i++) {
			zassert_equal(g.front[i], 0xA5,
				      "size %zu: underrun at %zu", sz, i);
			zassert_equal(g.back[i], 0xA5,
				      "size %zu: overrun at %zu", sz, i);
		}
	}

	zassert_equal(config_json_serialize(NULL, 16), -EINVAL);
	zassert_equal(config_json_serialize(g.body, 0), -EINVAL);
}

ZTEST_SUITE(config_json, NULL, NULL, reset_all, NULL, NULL);
