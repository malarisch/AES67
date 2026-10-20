/*
 * Auracast card (ESP32-S31) register protocol helpers.
 *
 * Pure functions over the register map in auracast_i2c_regs.h: local
 * validation of a configuration, packing it into the register bytes the
 * S31 expects (little endian, zero-padded NAME/CODE), unpacking what the
 * S31 reports, the "may the running broadcast keep running" comparison
 * and the name tables used by the REST API. No I2C, no kernel objects,
 * so card_settings/config_json and the unit tests can use the struct and
 * the helpers without the driver.
 */

#ifndef AURACAST_CARD_PROTO_H_
#define AURACAST_CARD_PROTO_H_

#include <stdbool.h>
#include <stdint.h>

#include "auracast_i2c_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Stream configuration as the host keeps and persists it. */
struct auracast_config {
	bool     enabled;                  /* broadcast wanted (persisted, default true) */
	uint8_t  preset;                   /* AURA_PRESET_* */
	uint8_t  channels;                 /* AURA_CH_* */
	uint16_t context;                  /* streaming context bit mask */
	uint8_t  flags;                    /* AURA_FLAG_* */
	int8_t   tx_power_dbm;
	uint16_t pd_us;                    /* presentation delay */
	uint16_t fifo_target;              /* frames @ 48 kHz */
	uint32_t broadcast_id;             /* AURA_BROADCAST_ID_RANDOM or 0..0xFFFFFF */
	char     name[AURA_NAME_SIZE + 1]; /* UTF-8, 4..32 bytes */
	char     code[AURA_CODE_SIZE + 1]; /* 0..16 bytes, empty = unencrypted */
};

/* Configuration register block PRESET..BROADCAST_ID (0x10..0x1D). */
#define AURA_CFG_BLOCK_REG   AURA_REG_PRESET
#define AURA_CFG_BLOCK_LEN   14

/* Status block PIPE_STATE..UPTIME_MS (0x80..0xAB). */
#define AURA_STATUS_LEN      (AURA_REG_STATUS_END - AURA_REG_PIPE_STATE)

/** Decoded status block. */
struct auracast_status_block {
	uint8_t  pipe_state;     /* AURA_PIPE_* */
	uint8_t  status_flags;   /* AURA_STATUS_* */
	uint16_t fifo_fill;
	int32_t  drift_cppm;
	int32_t  in_cppm;
	uint32_t i2s_bits;
	uint32_t underrun;
	uint32_t overflow;
	uint32_t lost;
	uint32_t send_err;
	uint32_t sent;
	uint32_t active_id;
	uint32_t uptime_ms;
};

/** Power-on values of the S31 (AURA_DEFAULT_*), enabled = true. */
void aura_proto_config_defaults(struct auracast_config *cfg);

/**
 * Check every field against the protocol limits.
 *
 * @return 0, or -EINVAL with *bad_reg (may be NULL) set to the AURA_REG_*
 *         address of the first offending field, in register order.
 */
int aura_proto_config_validate(const struct auracast_config *cfg, uint8_t *bad_reg);

/** Pack into the register bytes: block 0x10..0x1D, NAME and CODE zero-padded. */
void aura_proto_pack_config(const struct auracast_config *cfg,
			    uint8_t block[AURA_CFG_BLOCK_LEN],
			    uint8_t name[AURA_NAME_SIZE],
			    uint8_t code[AURA_CODE_SIZE]);

/** Inverse of pack. out->enabled is left untouched; name/code are NUL-terminated. */
void aura_proto_unpack_config(const uint8_t block[AURA_CFG_BLOCK_LEN],
			      const uint8_t name[AURA_NAME_SIZE],
			      const uint8_t code[AURA_CODE_SIZE],
			      struct auracast_config *out);

/**
 * True when a broadcast the S31 is running with @p live may be kept for
 * the stored configuration @p stored: all fields equal, ignoring
 * `enabled`; a stored random broadcast ID matches any live ID.
 */
bool aura_proto_config_matches(const struct auracast_config *stored,
			       const struct auracast_config *live);

/** Decode the 44-byte status block. */
void aura_proto_unpack_status(const uint8_t raw[AURA_STATUS_LEN],
			      struct auracast_status_block *out);

/* Name tables; every function returns "unknown" for values it does not know. */
const char *aura_proto_state_name(uint8_t state);
const char *aura_proto_error_name(uint8_t err);
const char *aura_proto_pipe_name(uint8_t pipe);
const char *aura_proto_reg_name(uint8_t reg);
const char *aura_proto_preset_label(uint8_t preset);
const char *aura_proto_channels_label(uint8_t channels);

#ifdef __cplusplus
}
#endif

#endif /* AURACAST_CARD_PROTO_H_ */
