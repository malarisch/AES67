/*
 * Unit tests for the IEEE 1588 helpers (src/ieee1588_utils.h).
 *
 * The clock identity is what the BMC compares nodes by and what the
 * SDP a=ts-refclk line advertises — a wrong EUI-64 expansion silently
 * makes two devices look like the same grandmaster.
 */

#include <zephyr/ztest.h>
#include <string.h>

#include "ieee1588_utils.h"

ZTEST(ieee1588, test_mac_to_clock_identity)
{
	/* IEEE 1588 7.5.2.2.2: MAC[0..2] with the U/L bit toggled,
	 * FF FE inserted, then MAC[3..5]. */
	const uint8_t mac[6] = { 0x02, 0xAA, 0xE6, 0x70, 0x00, 0x01 };
	const uint8_t expect[8] = {
		0x00, 0xAA, 0xE6, 0xFF, 0xFE, 0x70, 0x00, 0x01
	};
	uint8_t id[8];

	mac_to_clock_identity(mac, id);
	zassert_mem_equal(id, expect, sizeof(expect));
}

ZTEST(ieee1588, test_ul_bit_toggles_both_ways)
{
	/* A globally-unique OUI gets the local bit set, and applying the
	 * transform twice returns the original MAC byte. */
	const uint8_t global_mac[6] = { 0x00, 0x1B, 0x21, 0x11, 0x22, 0x33 };
	uint8_t id[8];

	mac_to_clock_identity(global_mac, id);
	zassert_equal(id[0], 0x02);
	zassert_equal(id[3], 0xFF);
	zassert_equal(id[4], 0xFE);

	uint8_t back[6] = { id[0], id[1], id[2], id[5], id[6], id[7] };
	uint8_t id2[8];

	mac_to_clock_identity(back, id2);
	zassert_equal(id2[0], global_mac[0]);
}

ZTEST(ieee1588, test_distinct_macs_stay_distinct)
{
	const uint8_t a[6] = { 0x02, 0xAA, 0xE6, 0x70, 0x00, 0x01 };
	const uint8_t b[6] = { 0x02, 0xAA, 0xE6, 0x70, 0x00, 0x02 };
	uint8_t ida[8], idb[8];

	mac_to_clock_identity(a, ida);
	mac_to_clock_identity(b, idb);
	zassert_true(memcmp(ida, idb, 8) != 0);
}

ZTEST(ieee1588, test_eth_speed_to_text)
{
	zassert_str_equal(eth_speed_to_text(0), "10M");
	zassert_str_equal(eth_speed_to_text(1), "100M");
	zassert_str_equal(eth_speed_to_text(2), "1G");
	zassert_str_equal(eth_speed_to_text(3), "?");
	zassert_str_equal(eth_speed_to_text(255), "?");
}

ZTEST_SUITE(ieee1588, NULL, NULL, NULL, NULL, NULL);
