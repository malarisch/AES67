/*
 * Auracast card register protocol helpers. See auracast_card_proto.h.
 */

#include <errno.h>
#include <string.h>

#include "auracast_card_proto.h"

static void put_le16(uint8_t *p, uint16_t v)
{
	p[0] = (uint8_t)v;
	p[1] = (uint8_t)(v >> 8);
}

static void put_le32(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)v;
	p[1] = (uint8_t)(v >> 8);
	p[2] = (uint8_t)(v >> 16);
	p[3] = (uint8_t)(v >> 24);
}

static uint16_t get_le16(const uint8_t *p)
{
	return (uint16_t)(p[0] | (p[1] << 8));
}

static uint32_t get_le32(const uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
	       ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* Length of a NUL-terminated field, bounded by its register size. */
static size_t field_len(const char *s, size_t max)
{
	size_t n = 0;

	while (n < max && s[n] != '\0') {
		n++;
	}
	return n;
}

void aura_proto_config_defaults(struct auracast_config *cfg)
{
	memset(cfg, 0, sizeof(*cfg));
	cfg->enabled = true;
	cfg->preset = AURA_DEFAULT_PRESET;
	cfg->channels = AURA_DEFAULT_CHANNELS;
	cfg->context = AURA_DEFAULT_CONTEXT;
	cfg->flags = AURA_DEFAULT_FLAGS;
	cfg->tx_power_dbm = AURA_DEFAULT_TX_POWER;
	cfg->pd_us = AURA_DEFAULT_PD_US;
	cfg->fifo_target = AURA_DEFAULT_FIFO_TARGET;
	cfg->broadcast_id = AURA_DEFAULT_BROADCAST_ID;
	strncpy(cfg->name, AURA_DEFAULT_NAME, AURA_NAME_SIZE);
	cfg->name[AURA_NAME_SIZE] = '\0';
	cfg->code[0] = '\0';
}

int aura_proto_config_validate(const struct auracast_config *cfg, uint8_t *bad_reg)
{
	uint8_t reg = 0;
	size_t len;

	if (cfg->preset >= AURA_PRESET_COUNT) {
		reg = AURA_REG_PRESET;
	} else if (cfg->channels >= AURA_CH_COUNT) {
		reg = AURA_REG_CHANNELS;
	} else if (cfg->context == 0 || (cfg->context & ~AURA_CONTEXT_MASK) != 0) {
		reg = AURA_REG_CONTEXT;
	} else if ((cfg->flags & ~AURA_FLAG_IMMEDIATE) != 0) {
		reg = AURA_REG_FLAGS;
	} else if (cfg->tx_power_dbm < AURA_TX_POWER_MIN ||
		   cfg->tx_power_dbm > AURA_TX_POWER_MAX) {
		reg = AURA_REG_TX_POWER;
	} else if (cfg->pd_us < AURA_PD_US_MIN || cfg->pd_us > AURA_PD_US_MAX) {
		reg = AURA_REG_PD_US;
	} else if (cfg->fifo_target < AURA_FIFO_TARGET_MIN ||
		   cfg->fifo_target > AURA_FIFO_TARGET_MAX) {
		reg = AURA_REG_FIFO_TARGET;
	} else if (cfg->broadcast_id != AURA_BROADCAST_ID_RANDOM &&
		   cfg->broadcast_id > 0xFFFFFFu) {
		reg = AURA_REG_BROADCAST_ID;
	} else {
		/* Byte counts, not characters: the register holds 32 bytes. */
		len = field_len(cfg->name, sizeof(cfg->name));
		if (len < AURA_NAME_MIN_LEN || len > AURA_NAME_SIZE) {
			reg = AURA_REG_NAME;
		} else if (field_len(cfg->code, sizeof(cfg->code)) > AURA_CODE_SIZE) {
			reg = AURA_REG_CODE;
		}
	}

	if (reg != 0) {
		if (bad_reg != NULL) {
			*bad_reg = reg;
		}
		return -EINVAL;
	}
	return 0;
}

void aura_proto_pack_config(const struct auracast_config *cfg,
			    uint8_t block[AURA_CFG_BLOCK_LEN],
			    uint8_t name[AURA_NAME_SIZE],
			    uint8_t code[AURA_CODE_SIZE])
{
	block[AURA_REG_PRESET - AURA_CFG_BLOCK_REG] = cfg->preset;
	block[AURA_REG_CHANNELS - AURA_CFG_BLOCK_REG] = cfg->channels;
	put_le16(&block[AURA_REG_CONTEXT - AURA_CFG_BLOCK_REG], cfg->context);
	block[AURA_REG_FLAGS - AURA_CFG_BLOCK_REG] = cfg->flags;
	block[AURA_REG_TX_POWER - AURA_CFG_BLOCK_REG] = (uint8_t)cfg->tx_power_dbm;
	put_le16(&block[AURA_REG_PD_US - AURA_CFG_BLOCK_REG], cfg->pd_us);
	put_le16(&block[AURA_REG_FIFO_TARGET - AURA_CFG_BLOCK_REG], cfg->fifo_target);
	put_le32(&block[AURA_REG_BROADCAST_ID - AURA_CFG_BLOCK_REG], cfg->broadcast_id);

	memset(name, 0, AURA_NAME_SIZE);
	memcpy(name, cfg->name, field_len(cfg->name, AURA_NAME_SIZE));
	memset(code, 0, AURA_CODE_SIZE);
	memcpy(code, cfg->code, field_len(cfg->code, AURA_CODE_SIZE));
}

void aura_proto_unpack_config(const uint8_t block[AURA_CFG_BLOCK_LEN],
			      const uint8_t name[AURA_NAME_SIZE],
			      const uint8_t code[AURA_CODE_SIZE],
			      struct auracast_config *out)
{
	out->preset = block[AURA_REG_PRESET - AURA_CFG_BLOCK_REG];
	out->channels = block[AURA_REG_CHANNELS - AURA_CFG_BLOCK_REG];
	out->context = get_le16(&block[AURA_REG_CONTEXT - AURA_CFG_BLOCK_REG]);
	out->flags = block[AURA_REG_FLAGS - AURA_CFG_BLOCK_REG];
	out->tx_power_dbm = (int8_t)block[AURA_REG_TX_POWER - AURA_CFG_BLOCK_REG];
	out->pd_us = get_le16(&block[AURA_REG_PD_US - AURA_CFG_BLOCK_REG]);
	out->fifo_target = get_le16(&block[AURA_REG_FIFO_TARGET - AURA_CFG_BLOCK_REG]);
	out->broadcast_id = get_le32(&block[AURA_REG_BROADCAST_ID - AURA_CFG_BLOCK_REG]);

	memcpy(out->name, name, AURA_NAME_SIZE);
	out->name[AURA_NAME_SIZE] = '\0';
	memcpy(out->code, code, AURA_CODE_SIZE);
	out->code[AURA_CODE_SIZE] = '\0';
}

bool aura_proto_config_matches(const struct auracast_config *stored,
			       const struct auracast_config *live)
{
	if (stored->preset != live->preset ||
	    stored->channels != live->channels ||
	    stored->context != live->context ||
	    stored->flags != live->flags ||
	    stored->tx_power_dbm != live->tx_power_dbm ||
	    stored->pd_us != live->pd_us ||
	    stored->fifo_target != live->fifo_target) {
		return false;
	}
	if (stored->broadcast_id != AURA_BROADCAST_ID_RANDOM &&
	    stored->broadcast_id != live->broadcast_id) {
		return false;
	}
	if (strncmp(stored->name, live->name, AURA_NAME_SIZE) != 0 ||
	    strncmp(stored->code, live->code, AURA_CODE_SIZE) != 0) {
		return false;
	}
	return true;
}

void aura_proto_unpack_status(const uint8_t raw[AURA_STATUS_LEN],
			      struct auracast_status_block *out)
{
#define OFF(reg_) ((reg_) - AURA_REG_PIPE_STATE)
	out->pipe_state = raw[OFF(AURA_REG_PIPE_STATE)];
	out->status_flags = raw[OFF(AURA_REG_STATUS_FLAGS)];
	out->fifo_fill = get_le16(&raw[OFF(AURA_REG_FIFO_FILL)]);
	out->drift_cppm = (int32_t)get_le32(&raw[OFF(AURA_REG_DRIFT_CPPM)]);
	out->in_cppm = (int32_t)get_le32(&raw[OFF(AURA_REG_IN_CPPM)]);
	out->i2s_bits = get_le32(&raw[OFF(AURA_REG_I2S_BITS)]);
	out->underrun = get_le32(&raw[OFF(AURA_REG_UNDERRUN)]);
	out->overflow = get_le32(&raw[OFF(AURA_REG_OVERFLOW)]);
	out->lost = get_le32(&raw[OFF(AURA_REG_LOST)]);
	out->send_err = get_le32(&raw[OFF(AURA_REG_SEND_ERR)]);
	out->sent = get_le32(&raw[OFF(AURA_REG_SENT)]);
	out->active_id = get_le32(&raw[OFF(AURA_REG_ACTIVE_ID)]);
	out->uptime_ms = get_le32(&raw[OFF(AURA_REG_UPTIME_MS)]);
#undef OFF
}

const char *aura_proto_state_name(uint8_t state)
{
	switch (state) {
	case AURA_STATE_WAIT_CONFIG:  return "wait_config";
	case AURA_STATE_STARTING:     return "starting";
	case AURA_STATE_STREAMING:    return "streaming";
	case AURA_STATE_CONFIG_ERROR: return "config_error";
	case AURA_STATE_FAULT:        return "fault";
	default:                      return "unknown";
	}
}

const char *aura_proto_error_name(uint8_t err)
{
	switch (err) {
	case AURA_ERR_NONE:         return "none";
	case AURA_ERR_READ_ONLY:    return "read_only";
	case AURA_ERR_LOCKED:       return "locked";
	case AURA_ERR_BAD_VALUE:    return "bad_value";
	case AURA_ERR_BAD_COMMAND:  return "bad_command";
	case AURA_ERR_START_FAILED: return "start_failed";
	default:                    return "unknown";
	}
}

const char *aura_proto_pipe_name(uint8_t pipe)
{
	switch (pipe) {
	case AURA_PIPE_NO_SIGNAL:  return "no_signal";
	case AURA_PIPE_PRIME:      return "prime";
	case AURA_PIPE_RUN:        return "run";
	case AURA_PIPE_RATE_ERROR: return "rate_error";
	default:                   return "unknown";
	}
}

const char *aura_proto_reg_name(uint8_t reg)
{
	switch (reg) {
	case AURA_REG_PRESET:       return "preset";
	case AURA_REG_CHANNELS:     return "channels";
	case AURA_REG_CONTEXT:      return "context";
	case AURA_REG_FLAGS:        return "flags";
	case AURA_REG_TX_POWER:     return "tx_power";
	case AURA_REG_PD_US:        return "pd_us";
	case AURA_REG_FIFO_TARGET:  return "fifo_target";
	case AURA_REG_BROADCAST_ID: return "broadcast_id";
	case AURA_REG_NAME:         return "name";
	case AURA_REG_CODE:         return "code";
	case AURA_REG_COMMAND:      return "command";
	default:                    return "unknown";
	}
}

const char *aura_proto_preset_label(uint8_t preset)
{
	switch (preset) {
	case AURA_PRESET_16_2_1: return "16 kHz / 40 B / Standard";
	case AURA_PRESET_24_2_1: return "24 kHz / 60 B / Standard";
	case AURA_PRESET_48_2_1: return "48 kHz / 100 B / High";
	case AURA_PRESET_48_4_1: return "48 kHz / 120 B / High";
	case AURA_PRESET_48_6_1: return "48 kHz / 155 B / High";
	default:                 return "unknown";
	}
}

const char *aura_proto_channels_label(uint8_t channels)
{
	switch (channels) {
	case AURA_CH_MONO_MIX:   return "Mono (L+R)/2";
	case AURA_CH_STEREO:     return "Stereo (2 BIS)";
	case AURA_CH_MONO_LEFT:  return "Mono left";
	case AURA_CH_MONO_RIGHT: return "Mono right";
	default:                 return "unknown";
	}
}
