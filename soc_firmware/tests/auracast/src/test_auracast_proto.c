/*
 * Unit tests for the Auracast card register protocol helpers.
 *
 * The S31 is only ever written through these pack functions and only
 * ever understood through the unpack functions, so a wrong byte order
 * or a missing terminator here silently misconfigures the broadcast.
 */

#include <zephyr/ztest.h>
#include <errno.h>
#include <string.h>

#include "auracast_card_proto.h"

ZTEST_SUITE(auracast_proto, NULL, NULL, NULL, NULL, NULL);

static struct auracast_config defaults(void)
{
	struct auracast_config cfg;

	aura_proto_config_defaults(&cfg);
	return cfg;
}

ZTEST(auracast_proto, test_defaults_match_header)
{
	struct auracast_config cfg = defaults();

	zassert_true(cfg.enabled);
	zassert_equal(cfg.preset, AURA_PRESET_24_2_1);
	zassert_equal(cfg.channels, AURA_CH_MONO_MIX);
	zassert_equal(cfg.context, 0x0040);
	zassert_equal(cfg.flags, AURA_FLAG_IMMEDIATE);
	zassert_equal(cfg.tx_power_dbm, 9);
	zassert_equal(cfg.pd_us, 20000);
	zassert_equal(cfg.fifo_target, 840);
	zassert_equal(cfg.broadcast_id, AURA_BROADCAST_ID_RANDOM);
	zassert_str_equal(cfg.name, "AES67 Auracast");
	zassert_str_equal(cfg.code, "");
	zassert_equal(aura_proto_config_validate(&cfg, NULL), 0);
}

ZTEST(auracast_proto, test_pack_config_little_endian)
{
	struct auracast_config cfg = defaults();
	uint8_t block[AURA_CFG_BLOCK_LEN];
	uint8_t name[AURA_NAME_SIZE], code[AURA_CODE_SIZE];
	static const uint8_t expect[AURA_CFG_BLOCK_LEN] = {
		1, 0, 0x34, 0x12, 0x01, 9, 0xEF, 0xBE, 0xBC, 0x0A,
		0xEF, 0xCD, 0xAB, 0x00,
	};

	cfg.context = 0x1234;
	cfg.pd_us = 0xBEEF;
	cfg.fifo_target = 0x0ABC;
	cfg.broadcast_id = 0x00ABCDEF;

	aura_proto_pack_config(&cfg, block, name, code);
	zassert_mem_equal(block, expect, sizeof(expect));
}

ZTEST(auracast_proto, test_pack_name_code_padded)
{
	struct auracast_config cfg = defaults();
	uint8_t block[AURA_CFG_BLOCK_LEN];
	uint8_t name[AURA_NAME_SIZE], code[AURA_CODE_SIZE];

	memset(name, 0xFF, sizeof(name));
	memset(code, 0xFF, sizeof(code));
	strcpy(cfg.name, "Main Hall");
	strcpy(cfg.code, "abc");

	aura_proto_pack_config(&cfg, block, name, code);

	zassert_mem_equal(name, "Main Hall", 9);
	for (int i = 9; i < AURA_NAME_SIZE; i++) {
		zassert_equal(name[i], 0, "name pad byte %d", i);
	}
	zassert_mem_equal(code, "abc", 3);
	for (int i = 3; i < AURA_CODE_SIZE; i++) {
		zassert_equal(code[i], 0, "code pad byte %d", i);
	}
}

ZTEST(auracast_proto, test_validate_name_bytes_not_chars)
{
	struct auracast_config cfg = defaults();
	uint8_t bad = 0;

	/* Two characters but four bytes: long enough by bytes. */
	strcpy(cfg.name, "\xc3\x84\xc3\x96");
	zassert_equal(strlen(cfg.name), 4);
	zassert_equal(aura_proto_config_validate(&cfg, &bad), 0);

	/* Three ASCII characters: too short. */
	strcpy(cfg.name, "abc");
	zassert_equal(aura_proto_config_validate(&cfg, &bad), -EINVAL);
	zassert_equal(bad, AURA_REG_NAME);

	/* 16 characters, 32 bytes: fills the register exactly. */
	for (int i = 0; i < 16; i++) {
		cfg.name[2 * i] = '\xc3';
		cfg.name[2 * i + 1] = '\x9c';
	}
	cfg.name[AURA_NAME_SIZE] = '\0';
	zassert_equal(aura_proto_config_validate(&cfg, &bad), 0);

	/* A 33rd byte without terminator inside the buffer must be
	 * rejected, not read past the field. */
	memset(cfg.name, 'A', sizeof(cfg.name));
	zassert_equal(aura_proto_config_validate(&cfg, &bad), -EINVAL);
	zassert_equal(bad, AURA_REG_NAME);
}

static void expect_bad(struct auracast_config *cfg, uint8_t reg)
{
	uint8_t bad = 0;

	zassert_equal(aura_proto_config_validate(cfg, &bad), -EINVAL,
		      "reg 0x%02x accepted", reg);
	zassert_equal(bad, reg, "wrong bad_reg 0x%02x for 0x%02x", bad, reg);
}

ZTEST(auracast_proto, test_validate_ranges)
{
	struct auracast_config cfg;

	cfg = defaults(); cfg.preset = AURA_PRESET_COUNT; expect_bad(&cfg, AURA_REG_PRESET);
	cfg = defaults(); cfg.channels = AURA_CH_COUNT; expect_bad(&cfg, AURA_REG_CHANNELS);
	cfg = defaults(); cfg.context = 0; expect_bad(&cfg, AURA_REG_CONTEXT);
	cfg = defaults(); cfg.context = 0x1040; expect_bad(&cfg, AURA_REG_CONTEXT);
	cfg = defaults(); cfg.flags = 0x02; expect_bad(&cfg, AURA_REG_FLAGS);
	cfg = defaults(); cfg.tx_power_dbm = -16; expect_bad(&cfg, AURA_REG_TX_POWER);
	cfg = defaults(); cfg.tx_power_dbm = 21; expect_bad(&cfg, AURA_REG_TX_POWER);
	cfg = defaults(); cfg.pd_us = 19999; expect_bad(&cfg, AURA_REG_PD_US);
	cfg = defaults(); cfg.pd_us = 40001; expect_bad(&cfg, AURA_REG_PD_US);
	cfg = defaults(); cfg.fifo_target = 839; expect_bad(&cfg, AURA_REG_FIFO_TARGET);
	cfg = defaults(); cfg.fifo_target = 3001; expect_bad(&cfg, AURA_REG_FIFO_TARGET);
	cfg = defaults(); cfg.broadcast_id = 0x01000000; expect_bad(&cfg, AURA_REG_BROADCAST_ID);

	cfg = defaults(); cfg.broadcast_id = AURA_BROADCAST_ID_RANDOM;
	zassert_equal(aura_proto_config_validate(&cfg, NULL), 0);
	cfg = defaults(); cfg.broadcast_id = 0xFFFFFF;
	zassert_equal(aura_proto_config_validate(&cfg, NULL), 0);
	cfg = defaults(); cfg.tx_power_dbm = -15;
	zassert_equal(aura_proto_config_validate(&cfg, NULL), 0);
	cfg = defaults(); cfg.tx_power_dbm = 20;
	zassert_equal(aura_proto_config_validate(&cfg, NULL), 0);
}

ZTEST(auracast_proto, test_unpack_config_full_fields_terminated)
{
	uint8_t block[AURA_CFG_BLOCK_LEN] = { 0 };
	uint8_t name[AURA_NAME_SIZE], code[AURA_CODE_SIZE];
	struct auracast_config out;

	memset(name, 'A', sizeof(name));
	memset(code, 'k', sizeof(code));
	memset(&out, 0x55, sizeof(out));
	out.enabled = false;

	aura_proto_unpack_config(block, name, code, &out);

	zassert_equal(out.name[AURA_NAME_SIZE], '\0');
	zassert_equal(strlen(out.name), AURA_NAME_SIZE);
	zassert_equal(out.code[AURA_CODE_SIZE], '\0');
	zassert_equal(strlen(out.code), AURA_CODE_SIZE);
	zassert_false(out.enabled, "unpack must not touch enabled");
}

ZTEST(auracast_proto, test_unpack_roundtrip)
{
	struct auracast_config cfg = defaults(), back;
	uint8_t block[AURA_CFG_BLOCK_LEN];
	uint8_t name[AURA_NAME_SIZE], code[AURA_CODE_SIZE];

	cfg.preset = AURA_PRESET_48_4_1;
	cfg.channels = AURA_CH_STEREO;
	cfg.context = 0x0004;
	cfg.flags = 0;
	cfg.tx_power_dbm = -7;
	cfg.pd_us = 31000;
	cfg.fifo_target = 1200;
	cfg.broadcast_id = 0x123456;
	strcpy(cfg.name, "Saal 1 \xc3\x9c");
	strcpy(cfg.code, "geheim");

	aura_proto_pack_config(&cfg, block, name, code);
	memset(&back, 0, sizeof(back));
	back.enabled = true;
	aura_proto_unpack_config(block, name, code, &back);

	zassert_equal(back.preset, cfg.preset);
	zassert_equal(back.channels, cfg.channels);
	zassert_equal(back.context, cfg.context);
	zassert_equal(back.flags, cfg.flags);
	zassert_equal(back.tx_power_dbm, cfg.tx_power_dbm);
	zassert_equal(back.pd_us, cfg.pd_us);
	zassert_equal(back.fifo_target, cfg.fifo_target);
	zassert_equal(back.broadcast_id, cfg.broadcast_id);
	zassert_str_equal(back.name, cfg.name);
	zassert_str_equal(back.code, cfg.code);
	zassert_true(back.enabled);
}

ZTEST(auracast_proto, test_matches_random_id_rule)
{
	struct auracast_config stored = defaults(), live = defaults();

	stored.broadcast_id = AURA_BROADCAST_ID_RANDOM;
	live.broadcast_id = 0x123456;
	zassert_true(aura_proto_config_matches(&stored, &live));

	stored.broadcast_id = 0x000001;
	live.broadcast_id = 0x000002;
	zassert_false(aura_proto_config_matches(&stored, &live));

	live.broadcast_id = 0x000001;
	stored.enabled = false;
	live.enabled = true;
	zassert_true(aura_proto_config_matches(&stored, &live),
		     "enabled must be ignored");

	strcpy(live.name, "Other");
	zassert_false(aura_proto_config_matches(&stored, &live));
}

ZTEST(auracast_proto, test_unpack_status)
{
	uint8_t raw[AURA_STATUS_LEN] = { 0 };
	struct auracast_status_block st;

	raw[0x00] = AURA_PIPE_RUN;
	raw[0x01] = 0x03;
	raw[0x02] = 0x48; raw[0x03] = 0x03;                /* 840 */
	raw[0x04] = 0x6A; raw[0x05] = 0xFF; raw[0x06] = 0xFF; raw[0x07] = 0xFF; /* -150 */
	raw[0x08] = 0x2C; raw[0x09] = 0x01;                /* 300 */
	raw[0x0C] = 0x00; raw[0x0D] = 0xFF; raw[0x0E] = 0xFF; raw[0x0F] = 0xFF;
	raw[0x10] = 7;                                     /* underrun */
	raw[0x14] = 8;                                     /* overflow */
	raw[0x18] = 9;                                     /* lost */
	raw[0x1C] = 10;                                    /* send_err */
	raw[0x20] = 0x10; raw[0x21] = 0x27;                /* sent 10000 */
	raw[0x24] = 0x56; raw[0x25] = 0x34; raw[0x26] = 0x12; /* active id */
	raw[0x2A] = 0x01;                                  /* uptime 0x00010000 */

	aura_proto_unpack_status(raw, &st);

	zassert_equal(st.pipe_state, AURA_PIPE_RUN);
	zassert_equal(st.status_flags, 0x03);
	zassert_equal(st.fifo_fill, 840);
	zassert_equal(st.drift_cppm, -150);
	zassert_equal(st.in_cppm, 300);
	zassert_equal(st.i2s_bits, 0xFFFFFF00u);
	zassert_equal(st.underrun, 7);
	zassert_equal(st.overflow, 8);
	zassert_equal(st.lost, 9);
	zassert_equal(st.send_err, 10);
	zassert_equal(st.sent, 10000);
	zassert_equal(st.active_id, 0x123456);
	zassert_equal(st.uptime_ms, 0x00010000);
}

ZTEST(auracast_proto, test_names)
{
	zassert_str_equal(aura_proto_state_name(AURA_STATE_STREAMING), "streaming");
	zassert_str_equal(aura_proto_state_name(9), "unknown");
	zassert_str_equal(aura_proto_error_name(AURA_ERR_START_FAILED), "start_failed");
	zassert_str_equal(aura_proto_pipe_name(AURA_PIPE_RATE_ERROR), "rate_error");
	zassert_str_equal(aura_proto_reg_name(AURA_REG_NAME), "name");
	zassert_str_equal(aura_proto_reg_name(AURA_REG_BROADCAST_ID), "broadcast_id");
	zassert_str_equal(aura_proto_reg_name(AURA_REG_COMMAND), "command");
	zassert_str_equal(aura_proto_reg_name(0x99), "unknown");
	zassert_true(strncmp(aura_proto_preset_label(AURA_PRESET_24_2_1), "24 kHz", 6) == 0);
	zassert_str_equal(aura_proto_channels_label(AURA_CH_STEREO), "Stereo (2 BIS)");
}
