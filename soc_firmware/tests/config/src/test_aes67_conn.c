/*
 * Unit tests for connection management (src/aes67_conn.c): the TX/RX
 * stream tables, their validation, the change observers and the
 * foreign-stream registry fed by SAP / mDNS / RTSP.
 *
 * The FPGA writes go to the mock HAL backend, so what is asserted here
 * is the table state every consumer (web API, NMOS, persistence,
 * announcers) reads back.
 */

#include <zephyr/ztest.h>
#include <zephyr/net/net_ip.h>
#include <errno.h>
#include <string.h>

#include "aes67_conn.h"

static struct in_addr addr(const char *s)
{
	struct in_addr a;

	zassert_equal(net_addr_pton(AF_INET, s, &a), 0, "bad address %s", s);
	return a;
}

static const struct in_addr zero_addr;

/* ---- Observer bookkeeping (registered once; no unregister API) ---- */

static int tx_notify_count;
static int rx_notify_count;
static uint8_t tx_last_id = 0xFF;
static uint8_t rx_last_id = 0xFF;

static void tx_observer(uint8_t stream_id)
{
	tx_notify_count++;
	tx_last_id = stream_id;
}

static void rx_observer(uint8_t stream_id)
{
	rx_notify_count++;
	rx_last_id = stream_id;
}

static void *conn_setup(void)
{
	zassert_equal(aes67_conn_register_tx_observer(tx_observer), 0);
	zassert_equal(aes67_conn_register_rx_observer(rx_observer), 0);
	return NULL;
}

/* Every test starts from empty tables — the module keeps global state
 * for the whole image. */
static void clear_all(void *fixture)
{
	static const uint8_t ch[AES67_MAX_CH_PER_STREAM] = { 0 };

	ARG_UNUSED(fixture);

	for (uint8_t i = 0; i < AES67_MAX_TX_STREAMS; i++) {
		zassert_equal(aes67_conn_configure_tx_stream(
				      i, &zero_addr, 0, 48, ch, 0, 0, NULL), 0);
	}
	for (uint8_t i = 0; i < AES67_MAX_RX_STREAMS; i++) {
		zassert_equal(aes67_conn_configure_rx_stream(
				      i, &zero_addr, 0, ch, 1, 0, 48, NULL,
				      NULL, NULL), 0);
	}

	int count;
	const struct aes67_foreign_stream *fs =
		aes67_conn_get_foreign_streams(&count);

	for (int i = 0; i < AES67_MAX_FOREIGN_STREAMS; i++) {
		if (fs[i].valid) {
			struct aes67_foreign_stream del = {
				.valid = false,
				.id_hash = fs[i].id_hash,
			};

			aes67_conn_report_foreign_stream(&del);
		}
	}

	tx_notify_count = 0;
	rx_notify_count = 0;
	tx_last_id = 0xFF;
	rx_last_id = 0xFF;
}

/* ================================================================
 * TX streams
 * ================================================================ */

ZTEST(aes67_conn, test_tx_configure)
{
	const uint8_t ch_ids[4] = { 3, 2, 1, 0 };
	struct in_addr dst = addr("239.69.0.5");

	zassert_equal(aes67_conn_configure_tx_stream(2, &dst, 4, 48, ch_ids, 4,
						     0xCAFEBABE, "Mic Bus"), 0);

	const struct aes67_tx_stream *tx = aes67_conn_get_tx_streams();

	zassert_true(tx[2].active);
	zassert_equal(tx[2].stream_id, 2);
	zassert_str_equal(tx[2].name, "Mic Bus");
	zassert_equal(tx[2].dst_ip.s_addr, dst.s_addr);
	zassert_equal(tx[2].channel_count, 4);
	zassert_equal(tx[2].samples_per_packet, 48);
	zassert_equal(tx[2].ssrc, 0xCAFEBABE);
	zassert_mem_equal(tx[2].ch_ids, ch_ids, 4);
	/* Unused channel slots are cleared, not left over from a previous
	 * configuration. */
	for (int i = 4; i < AES67_MAX_CH_PER_STREAM; i++) {
		zassert_equal(tx[2].ch_ids[i], 0);
	}

	/* Only the addressed slot is touched. */
	zassert_false(tx[0].active);
	zassert_false(tx[7].active);
}

ZTEST(aes67_conn, test_tx_auto_name_and_ssrc)
{
	const uint8_t ch_ids[2] = { 0, 1 };
	struct in_addr dst = addr("239.69.0.1");

	/* NULL name -> generated; SSRC 0 -> derived from dst IP + slot. */
	zassert_equal(aes67_conn_configure_tx_stream(3, &dst, 2, 48, ch_ids, 2,
						     0, NULL), 0);

	const struct aes67_tx_stream *tx = aes67_conn_get_tx_streams();

	zassert_str_equal(tx[3].name, "TX Stream 3");
	zassert_not_equal(tx[3].ssrc, 0);

	/* Deterministic for the same slot + destination... */
	uint32_t ssrc3 = tx[3].ssrc;

	zassert_equal(aes67_conn_configure_tx_stream(3, &dst, 2, 48, ch_ids, 2,
						     0, ""), 0);
	zassert_equal(tx[3].ssrc, ssrc3);
	zassert_str_equal(tx[3].name, "TX Stream 3",
			  "an empty name must also auto-generate");

	/* ...and different for a different slot. */
	zassert_equal(aes67_conn_configure_tx_stream(4, &dst, 2, 48, ch_ids, 2,
						     0, NULL), 0);
	zassert_not_equal(tx[4].ssrc, ssrc3);
}

ZTEST(aes67_conn, test_tx_deactivate)
{
	const uint8_t ch_ids[2] = { 0, 1 };
	struct in_addr dst = addr("239.69.0.9");

	zassert_equal(aes67_conn_configure_tx_stream(1, &dst, 2, 48, ch_ids, 2,
						     0, NULL), 0);
	zassert_true(aes67_conn_get_tx_streams()[1].active);

	/* A zero destination deactivates... */
	zassert_equal(aes67_conn_configure_tx_stream(1, &zero_addr, 2, 48,
						     ch_ids, 2, 0, NULL), 0);
	zassert_false(aes67_conn_get_tx_streams()[1].active);

	/* ...and so does a zero channel count. */
	zassert_equal(aes67_conn_configure_tx_stream(1, &dst, 2, 48, ch_ids, 2,
						     0, NULL), 0);
	zassert_true(aes67_conn_get_tx_streams()[1].active);
	zassert_equal(aes67_conn_configure_tx_stream(1, &dst, 0, 48, ch_ids, 0,
						     0, NULL), 0);
	zassert_false(aes67_conn_get_tx_streams()[1].active);
}

ZTEST(aes67_conn, test_tx_rejects_bad_arguments)
{
	const uint8_t ch_ids[2] = { 0, 1 };
	struct in_addr dst = addr("239.69.0.1");

	zassert_equal(aes67_conn_configure_tx_stream(AES67_MAX_TX_STREAMS, &dst,
						     2, 48, ch_ids, 2, 0, NULL),
		      -EINVAL);
	zassert_equal(aes67_conn_configure_tx_stream(255, &dst, 2, 48, ch_ids,
						     2, 0, NULL), -EINVAL);
	zassert_equal(aes67_conn_configure_tx_stream(0, NULL, 2, 48, ch_ids, 2,
						     0, NULL), -EINVAL);
	zassert_equal(aes67_conn_configure_tx_stream(0, &dst, 2, 48, NULL, 2, 0,
						     NULL), -EINVAL);
}

ZTEST(aes67_conn, test_tx_channel_list_is_bounded)
{
	/* More channel IDs than the stream can carry: the surplus must be
	 * dropped, not written past the fixed-size table entry. */
	const uint8_t ch_ids[16] = { 0, 1, 2, 3, 4, 5, 6, 7,
				     8, 9, 10, 11, 12, 13, 14, 15 };
	struct in_addr dst = addr("239.69.0.1");

	zassert_equal(aes67_conn_configure_tx_stream(0, &dst, 8, 48, ch_ids,
						     ARRAY_SIZE(ch_ids), 0,
						     NULL), 0);

	const struct aes67_tx_stream *tx = aes67_conn_get_tx_streams();

	zassert_mem_equal(tx[0].ch_ids, ch_ids, AES67_MAX_CH_PER_STREAM);
	/* The next table entry must still be untouched. */
	zassert_false(tx[1].active);
}

ZTEST(aes67_conn, test_tx_copy_under_lock)
{
	const uint8_t ch_ids[2] = { 6, 7 };
	struct in_addr dst = addr("239.69.0.3");
	struct aes67_tx_stream out;

	memset(&out, 0xAA, sizeof(out));

	/* Inactive slot: nothing copied, false returned. */
	zassert_false(aes67_conn_copy_tx_stream(5, &out));

	zassert_equal(aes67_conn_configure_tx_stream(5, &dst, 2, 12, ch_ids, 2,
						     0x11223344, "Copy"), 0);
	zassert_true(aes67_conn_copy_tx_stream(5, &out));
	zassert_str_equal(out.name, "Copy");
	zassert_equal(out.ssrc, 0x11223344);
	zassert_equal(out.samples_per_packet, 12);

	zassert_false(aes67_conn_copy_tx_stream(AES67_MAX_TX_STREAMS, &out));
	zassert_false(aes67_conn_copy_tx_stream(0, NULL));
}

/* ================================================================
 * RX streams
 * ================================================================ */

ZTEST(aes67_conn, test_rx_configure)
{
	const uint8_t ch_map[2] = { 4, 5 };
	struct in_addr dst = addr("239.69.1.1");
	struct in_addr sender = addr("10.0.0.7");

	zassert_equal(aes67_conn_configure_rx_stream(1, &dst, 5004, ch_map, 2,
						     16, 48, "Remote Mix",
						     "Horus", &sender), 0);

	const struct aes67_rx_stream *rx = aes67_conn_get_rx_streams();

	zassert_true(rx[1].active);
	zassert_equal(rx[1].stream_id, 1);
	zassert_str_equal(rx[1].name, "Remote Mix");
	zassert_str_equal(rx[1].sender_name, "Horus");
	zassert_equal(rx[1].sender_ip.s_addr, sender.s_addr);
	zassert_equal(rx[1].dst_ip.s_addr, dst.s_addr);
	zassert_equal(rx[1].dst_port, 5004);
	zassert_equal(rx[1].channel_count, 2);
	zassert_equal(rx[1].samples_per_channel, 48);
	/* The table keeps the raw playout delay; only the gateware register
	 * carries the +2*spp offset. */
	zassert_equal(rx[1].output_delay, 16);
	zassert_mem_equal(rx[1].ch_map, ch_map, 2);
}

ZTEST(aes67_conn, test_rx_auto_name_and_deactivate)
{
	const uint8_t ch_map[2] = { 0, 1 };
	struct in_addr dst = addr("239.69.1.2");

	zassert_equal(aes67_conn_configure_rx_stream(2, &dst, 5004, ch_map, 2,
						     0, 48, NULL, NULL, NULL),
		      0);

	const struct aes67_rx_stream *rx = aes67_conn_get_rx_streams();

	zassert_str_equal(rx[2].name, "RX Stream 2");
	zassert_str_equal(rx[2].sender_name, "");
	zassert_equal(rx[2].sender_ip.s_addr, 0);

	/* Deactivating clears the identity so a stale sender name can't
	 * survive into the next subscription. */
	zassert_equal(aes67_conn_configure_rx_stream(2, &zero_addr, 0, ch_map,
						     2, 0, 48, "x", "y", &dst),
		      0);
	zassert_false(rx[2].active);
	zassert_str_equal(rx[2].name, "");
	zassert_str_equal(rx[2].sender_name, "");
	zassert_equal(rx[2].sender_ip.s_addr, 0);
}

ZTEST(aes67_conn, test_rx_delay_clamp_is_accepted)
{
	const uint8_t ch_map[1] = { 0 };
	struct in_addr dst = addr("239.69.1.3");

	/* delay + 2*spp overflows the 8-bit gateware field; the call must
	 * still succeed (clamped in hardware) and keep the raw values. */
	zassert_equal(aes67_conn_configure_rx_stream(0, &dst, 5004, ch_map, 1,
						     200, 96, NULL, NULL, NULL),
		      0);

	const struct aes67_rx_stream *rx = aes67_conn_get_rx_streams();

	zassert_true(rx[0].active);
	zassert_equal(rx[0].output_delay, 200);
	zassert_equal(rx[0].samples_per_channel, 96);
}

ZTEST(aes67_conn, test_rx_rejects_bad_arguments)
{
	const uint8_t ch_map[2] = { 0, 1 };
	struct in_addr dst = addr("239.69.1.1");

	zassert_equal(aes67_conn_configure_rx_stream(AES67_MAX_RX_STREAMS, &dst,
						     5004, ch_map, 2, 0, 48,
						     NULL, NULL, NULL),
		      -EINVAL);
	zassert_equal(aes67_conn_configure_rx_stream(0, NULL, 5004, ch_map, 2,
						     0, 48, NULL, NULL, NULL),
		      -EINVAL);
	zassert_equal(aes67_conn_configure_rx_stream(0, &dst, 5004, NULL, 2, 0,
						     48, NULL, NULL, NULL),
		      -EINVAL);
	/* Channel count must be 1..AES67_MAX_CH_PER_STREAM. */
	zassert_equal(aes67_conn_configure_rx_stream(0, &dst, 5004, ch_map, 0,
						     0, 48, NULL, NULL, NULL),
		      -EINVAL);
	zassert_equal(aes67_conn_configure_rx_stream(0, &dst, 5004, ch_map,
						     AES67_MAX_CH_PER_STREAM + 1,
						     0, 48, NULL, NULL, NULL),
		      -EINVAL);
}

/* ================================================================
 * Observers
 * ================================================================ */

ZTEST(aes67_conn, test_observers_are_notified)
{
	const uint8_t ch[2] = { 0, 1 };
	struct in_addr dst = addr("239.69.2.1");

	zassert_equal(tx_notify_count, 0);
	zassert_equal(aes67_conn_configure_tx_stream(6, &dst, 2, 48, ch, 2, 0,
						     NULL), 0);
	zassert_equal(tx_notify_count, 1);
	zassert_equal(tx_last_id, 6);

	/* Deactivation is a change too — announcers must withdraw. */
	zassert_equal(aes67_conn_configure_tx_stream(6, &zero_addr, 0, 48, ch,
						     0, 0, NULL), 0);
	zassert_equal(tx_notify_count, 2);

	/* A rejected call must not fire the observer. */
	zassert_equal(aes67_conn_configure_tx_stream(99, &dst, 2, 48, ch, 2, 0,
						     NULL), -EINVAL);
	zassert_equal(tx_notify_count, 2);

	zassert_equal(rx_notify_count, 0);
	zassert_equal(aes67_conn_configure_rx_stream(6, &dst, 5004, ch, 2, 0,
						     48, NULL, NULL, NULL), 0);
	zassert_equal(rx_notify_count, 1);
	zassert_equal(rx_last_id, 6);

	zassert_equal(aes67_conn_configure_rx_stream(99, &dst, 5004, ch, 2, 0,
						     48, NULL, NULL, NULL),
		      -EINVAL);
	zassert_equal(rx_notify_count, 1);
}

/* ================================================================
 * Foreign-stream registry
 * ================================================================ */

static struct aes67_foreign_stream make_foreign(uint16_t hash,
						const char *origin,
						const char *name)
{
	struct aes67_foreign_stream fs = {
		.valid = true,
		.via = AES67_VIA_SAP,
		.id_hash = hash,
		.origin_addr = addr(origin),
		.mcast_addr = addr("239.69.9.1"),
		.port = 5004,
		.channels = 2,
		.bit_depth = 24,
		.sample_rate = 48000,
	};

	strncpy(fs.name, name, sizeof(fs.name) - 1);
	return fs;
}

static const struct aes67_foreign_stream *find_foreign(uint16_t hash)
{
	int count;
	const struct aes67_foreign_stream *fs =
		aes67_conn_get_foreign_streams(&count);

	for (int i = 0; i < AES67_MAX_FOREIGN_STREAMS; i++) {
		if (fs[i].valid && fs[i].id_hash == hash) {
			return &fs[i];
		}
	}
	return NULL;
}

ZTEST(aes67_conn, test_foreign_upsert_and_delete)
{
	struct aes67_foreign_stream a = make_foreign(0x1234, "10.0.0.1",
						     "Sender A");
	int count;

	aes67_conn_get_foreign_streams(&count);
	zassert_equal(count, 0);

	aes67_conn_report_foreign_stream(&a);
	aes67_conn_get_foreign_streams(&count);
	zassert_equal(count, 1);
	zassert_str_equal(find_foreign(0x1234)->name, "Sender A");

	/* Same hash + same origin updates in place rather than duplicating. */
	strcpy(a.name, "Sender A renamed");
	a.channels = 8;
	aes67_conn_report_foreign_stream(&a);
	aes67_conn_get_foreign_streams(&count);
	zassert_equal(count, 1);
	zassert_str_equal(find_foreign(0x1234)->name, "Sender A renamed");
	zassert_equal(find_foreign(0x1234)->channels, 8);

	/* Same hash from a different origin is a genuinely different
	 * sighting and gets its own slot. */
	struct aes67_foreign_stream b = make_foreign(0x1234, "10.0.0.2",
						     "Sender B");

	aes67_conn_report_foreign_stream(&b);
	aes67_conn_get_foreign_streams(&count);
	zassert_equal(count, 2);

	/* valid = false deletes every entry with that hash. */
	struct aes67_foreign_stream del = { .valid = false, .id_hash = 0x1234 };

	aes67_conn_report_foreign_stream(&del);
	aes67_conn_get_foreign_streams(&count);
	zassert_equal(count, 0);

	/* Deleting an unknown hash and reporting NULL are both harmless. */
	aes67_conn_report_foreign_stream(&del);
	aes67_conn_report_foreign_stream(NULL);
	aes67_conn_get_foreign_streams(&count);
	zassert_equal(count, 0);
}

ZTEST(aes67_conn, test_foreign_touch_refreshes)
{
	struct aes67_foreign_stream a = make_foreign(0x55, "10.0.0.1", "S");

	aes67_conn_report_foreign_stream(&a);

	int64_t first = find_foreign(0x55)->last_seen_ms;

	k_sleep(K_MSEC(20));
	aes67_conn_touch_foreign_stream(0x55);
	zassert_true(find_foreign(0x55)->last_seen_ms > first,
		     "touch must refresh the expiry timestamp");

	/* Touching an unknown hash must not disturb the table. */
	aes67_conn_touch_foreign_stream(0xBEEF);

	int count;

	aes67_conn_get_foreign_streams(&count);
	zassert_equal(count, 1);
}

ZTEST(aes67_conn, test_foreign_table_full)
{
	int count;

	for (int i = 0; i < AES67_MAX_FOREIGN_STREAMS; i++) {
		struct aes67_foreign_stream fs =
			make_foreign((uint16_t)(0x100 + i), "10.0.0.1", "S");

		aes67_conn_report_foreign_stream(&fs);
	}
	aes67_conn_get_foreign_streams(&count);
	zassert_equal(count, AES67_MAX_FOREIGN_STREAMS);

	/* One more sighting is dropped, and must not corrupt the table. */
	struct aes67_foreign_stream extra = make_foreign(0xFFFF, "10.0.0.1",
							 "Overflow");

	aes67_conn_report_foreign_stream(&extra);
	aes67_conn_get_foreign_streams(&count);
	zassert_equal(count, AES67_MAX_FOREIGN_STREAMS);
	zassert_is_null(find_foreign(0xFFFF));
	zassert_not_null(find_foreign(0x100));

	/* Freeing a slot lets the next sighting in. */
	struct aes67_foreign_stream del = { .valid = false, .id_hash = 0x100 };

	aes67_conn_report_foreign_stream(&del);
	aes67_conn_report_foreign_stream(&extra);
	aes67_conn_get_foreign_streams(&count);
	zassert_equal(count, AES67_MAX_FOREIGN_STREAMS);
	zassert_not_null(find_foreign(0xFFFF));
}

ZTEST_SUITE(aes67_conn, NULL, conn_setup, clear_all, NULL, NULL);
