/*
 * Unit tests for the global device configuration (src/aes67_config.c).
 *
 * The derived identities matter beyond this module: the MAC is what the
 * Ethernet driver programs into the FPGA, and the hostname is what mDNS,
 * RTSP and the NMOS node advertise. All three must be stable functions
 * of the stored serial / device name.
 */

#include <zephyr/ztest.h>
#include <string.h>

#include "aes67_config.h"

static void reset_config(void *fixture)
{
	ARG_UNUSED(fixture);
	(void)aes67_config_get();     /* first call installs the defaults */
	aes67_config_reset_defaults();
}

ZTEST(aes67_config, test_defaults)
{
	const struct aes67_device_config *c = aes67_config_get();

	zassert_str_equal(c->vendor, "AES67");
	zassert_str_equal(c->product, "AudioNode");
	zassert_str_equal(c->serial, "0001");
	zassert_str_equal(c->device_name, "AES67 Node");

	zassert_str_equal(c->default_mcast_addr, "239.69.0.1");
	zassert_equal(c->default_port, 5004);
	zassert_equal(c->default_channels, 2);
	zassert_equal(c->default_bit_depth, 24);
	zassert_equal(c->default_sample_rate, 48000);
	zassert_equal(c->default_samples_per_pkt, 48);
	zassert_equal(c->default_payload_type, 97);

	/* AES67 media profile: sync every 125 ms, announce every 1 s. */
	zassert_equal(c->ptp_domain, 0);
	zassert_equal(c->ptp_priority1, 128);
	zassert_equal(c->ptp_priority2, 128);
	zassert_equal(c->ptp_clock_class, 248);
	zassert_equal(c->ptp_clock_accuracy, 0xFE);
	zassert_equal(c->ptp_log_sync_interval, -3);
	zassert_equal(c->ptp_log_announce_interval, 0);
	zassert_equal(c->ptp_delay_asymmetry_ns, 0);

	zassert_equal(c->pi_kp_num, 1);
	zassert_equal(c->pi_kp_den, 4);
	zassert_equal(c->pi_ki_num, 1);
	zassert_equal(c->pi_ki_den, 32);

	zassert_equal(c->sap_announce_interval_s, 30);
	zassert_true(c->sap_announce_enabled);
}

ZTEST(aes67_config, test_reset_clears_previous_values)
{
	struct aes67_device_config *c = aes67_config_get();

	strcpy(c->device_name, "Changed");
	c->default_port = 1234;
	c->sap_announce_enabled = false;

	aes67_config_reset_defaults();

	zassert_str_equal(c->device_name, "AES67 Node");
	zassert_equal(c->default_port, 5004);
	zassert_true(c->sap_announce_enabled);
}

ZTEST(aes67_config, test_node_id)
{
	char buf[AES67_NODE_ID_MAX];

	zassert_equal_ptr(aes67_config_build_node_id(buf, sizeof(buf)), buf);
	zassert_str_equal(buf, "AES67 AudioNode 0001");

	struct aes67_device_config *c = aes67_config_get();

	strcpy(c->vendor, "Acme");
	strcpy(c->product, "Mixer");
	strcpy(c->serial, "4711");
	aes67_config_build_node_id(buf, sizeof(buf));
	zassert_str_equal(buf, "Acme Mixer 4711");

	/* A zero-length buffer must not be written to. */
	char guard = '#';

	zassert_equal_ptr(aes67_config_build_node_id(&guard, 0), &guard);
	zassert_equal(guard, '#');
	zassert_is_null(aes67_config_build_node_id(NULL, 16));
}

ZTEST(aes67_config, test_hostname_from_device_name)
{
	char buf[AES67_NODE_ID_MAX];

	/* Friendly name wins: spaces dropped, lowercased for DNS. */
	aes67_config_build_hostname(buf, sizeof(buf));
	zassert_str_equal(buf, "aes67node");

	struct aes67_device_config *c = aes67_config_get();

	strcpy(c->device_name, "Studio B Rack 2");
	aes67_config_build_hostname(buf, sizeof(buf));
	zassert_str_equal(buf, "studiobrack2");

	/* No friendly name: fall back to vendor_product_serial. */
	c->device_name[0] = '\0';
	aes67_config_build_hostname(buf, sizeof(buf));
	zassert_str_equal(buf, "aes67_audionode_0001");
}

ZTEST(aes67_config, test_hostname_is_bounded)
{
	struct aes67_device_config *c = aes67_config_get();
	char small[8];

	strcpy(c->device_name, "AVeryLongDeviceName");
	memset(small, 0x5A, sizeof(small));
	aes67_config_build_hostname(small, sizeof(small));
	zassert_equal(strlen(small), sizeof(small) - 1);
	zassert_str_equal(small, "averylo");
}

ZTEST(aes67_config, test_mac_from_numeric_serial)
{
	struct aes67_device_config *c = aes67_config_get();
	uint8_t mac[6];

	/* The historic default serial keeps its historic MAC. */
	aes67_config_build_mac(mac);
	zassert_mem_equal(mac, "\x02\xAA\xE6\x70\x00\x01", 6);

	strcpy(c->serial, "65535");
	aes67_config_build_mac(mac);
	zassert_mem_equal(mac, "\x02\xAA\xE6\x70\xFF\xFF", 6);

	strcpy(c->serial, "258");
	aes67_config_build_mac(mac);
	zassert_mem_equal(mac, "\x02\xAA\xE6\x70\x01\x02", 6);
}

ZTEST(aes67_config, test_mac_from_hashed_serial)
{
	struct aes67_device_config *c = aes67_config_get();
	uint8_t a[6], b[6], again[6];

	/* Out of the 16-bit range, non-numeric and empty serials are all
	 * folded — still locally administered, still deterministic. */
	static const char *const hashed[] = { "65536", "SN-ABC-123", "0001x",
					      "" };

	ARRAY_FOR_EACH(hashed, i) {
		strcpy(c->serial, hashed[i]);
		aes67_config_build_mac(a);
		aes67_config_build_mac(again);
		zassert_mem_equal(a, again, 6, "serial \"%s\" is not stable",
				  hashed[i]);
		zassert_mem_equal(a, "\x02\xAA\xE6\x70", 4,
				  "locally-administered prefix lost");
	}

	strcpy(c->serial, "SN-ABC-123");
	aes67_config_build_mac(a);
	strcpy(c->serial, "SN-ABC-124");
	aes67_config_build_mac(b);
	zassert_true(memcmp(a, b, 6) != 0,
		     "different serials must not collide");
}

ZTEST_SUITE(aes67_config, NULL, NULL, reset_config, NULL, NULL);
