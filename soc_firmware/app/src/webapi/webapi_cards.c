/*
 * /api/cards                        — slot overview + last I2C scan (GET)
 * /api/cards/rescan                 — re-scan the card bus (POST)
 * /api/cards/mi[...]                — MI card (8-ch ADC preamp)
 * /api/cards/lo[...]                — LO card (8-ch line output)
 * /api/cards/io[...]                — IO card (16-in / 8-out combo)
 * /api/cards/auracast[...]          — Auracast card (ESP32-S31 BLE Audio TX):
 *                                     GET status+config, PATCH config (restarts
 *                                     the broadcast), POST start|stop|reset
 *
 * Per-card settings writes are persisted through the card_settings shadow
 * store with a debounced save (a dragged gain slider must not erase a
 * flash sector per step); reset/rescan re-apply the stored settings since
 * a re-initialised card is back at hardware defaults.
 */

#include <zephyr/kernel.h>
#include <stdio.h>
#include <string.h>

#include "webapi_priv.h"
#include "../card_manager.h"
#include "../card_settings.h"


#ifdef CONFIG_MI_CARD
#include "../../drivers/mi_card/mi_card.h"
#endif
#ifdef CONFIG_LO_CARD
#include "../../drivers/lo_card/lo_card.h"
#endif
#ifdef CONFIG_IO_CARD
#include "../../drivers/io_card/io_card.h"
#endif
#ifdef CONFIG_AURACAST_CARD
#include "../../drivers/auracast_card/auracast_card.h"
#endif
#ifdef CONFIG_DISPLAY_CTRL
#include "../../drivers/display_ctrl/display_ctrl.h"
#endif

/* ================================================================
 * Request bodies
 *
 * The card endpoints share two body shapes. Presence matters here: a
 * setter must only be called for a key the request actually carried,
 * otherwise a PATCH of one field would silently reprogram the others —
 * hence the parser's bitmap rather than pre-filled defaults.
 * ================================================================ */

#if defined(CONFIG_MI_CARD) || defined(CONFIG_LO_CARD) || defined(CONFIG_IO_CARD)

struct card_flags_req {
	bool hpf;
	bool f96khz;
	bool output_enable;
};

enum { CF_HPF, CF_F96KHZ, CF_OUTPUT_ENABLE };

static const struct json_obj_descr card_flags_descr[] = {
	[CF_HPF] = JSON_OBJ_DESCR_PRIM(struct card_flags_req, hpf,
				       JSON_TOK_TRUE),
	[CF_F96KHZ] = JSON_OBJ_DESCR_PRIM(struct card_flags_req, f96khz,
					  JSON_TOK_TRUE),
	[CF_OUTPUT_ENABLE] = JSON_OBJ_DESCR_PRIM(struct card_flags_req,
						 output_enable, JSON_TOK_TRUE),
};

struct card_chan_req {
	int32_t gain;
	int32_t clip;
	bool    phantom;
	bool    muted;
};

enum { CC_GAIN, CC_CLIP, CC_PHANTOM, CC_MUTED };

static const struct json_obj_descr card_chan_descr[] = {
	[CC_GAIN] = JSON_OBJ_DESCR_PRIM(struct card_chan_req, gain,
					JSON_TOK_NUMBER),
	[CC_CLIP] = JSON_OBJ_DESCR_PRIM(struct card_chan_req, clip,
					JSON_TOK_NUMBER),
	[CC_PHANTOM] = JSON_OBJ_DESCR_PRIM(struct card_chan_req, phantom,
					   JSON_TOK_TRUE),
	[CC_MUTED] = JSON_OBJ_DESCR_PRIM(struct card_chan_req, muted,
					 JSON_TOK_TRUE),
};

/* Both return a negative errno on a malformed body; an empty body is a
 * no-op patch, not an error. */
static int64_t parse_flags(struct webapi_request *req,
			   struct card_flags_req *out)
{
	if (req->body_len == 0) {
		return 0;
	}
	return webapi_parse_body(req, card_flags_descr,
				 ARRAY_SIZE(card_flags_descr), out);
}

static int64_t parse_chan(struct webapi_request *req,
			  struct card_chan_req *out)
{
	if (req->body_len == 0) {
		return 0;
	}
	return webapi_parse_body(req, card_chan_descr,
				 ARRAY_SIZE(card_chan_descr), out);
}

#endif /* any card */

/*
 * Hardware reset of the analog cards goes through the single nRST line
 * shared by the display controller and the AD/DA cards ("dc-nrst" alias
 * or the FPGA adda_nrst CSR), pulsed by display_ctrl_hw_reset(), which
 * also re-scans the cards through its post-reset callbacks. Without that
 * line the reset endpoints fall back to the cards' I2C soft reset.
 */
#if defined(CONFIG_DISPLAY_CTRL) &&                                          \
	((defined(CONFIG_DISPLAY_CTRL_NRST_GPIO) &&                          \
	  DT_NODE_EXISTS(DT_ALIAS(dc_nrst))) ||                              \
	 defined(CONFIG_DISPLAY_CTRL_NRST_HAL))
#define AES67_SHARED_NRST_AVAILABLE 1
#else
#define AES67_SHARED_NRST_AVAILABLE 0
#endif

/* Settings changed -> capture into the shadow store, save debounced.
 * __maybe_unused: not referenced when no card driver is compiled in. */
static __maybe_unused void settings_changed(void)
{
	if (card_settings_capture() == 0) {
		card_settings_schedule_save();
	}
}

/* ---------------- /api/cards ---------------- */

static void build_card_slot(struct json_out *jo, int slot)
{
	const struct card_info *info = card_manager_get_info(slot);

	jo_obj_begin(jo);
	jo_int(jo, "slot", slot);

	if (info == NULL) {
		jo_bool(jo, "present", false);
		jo_str(jo, "type", "none");
		jo_int(jo, "type_id", 0);
		jo_obj_end(jo);
		return;
	}

	jo_bool(jo, "present", info->present);
	jo_bool(jo, "initialized", info->initialized);
	jo_str(jo, "type", card_type_name(info->type));
	jo_int(jo, "type_id", (int)info->type);

	if (info->present) {
		jo_int(jo, "board_id", info->ident.board_id);
		jo_int(jo, "soft_id", info->ident.soft_id);
		jo_int(jo, "hard_rev", info->ident.hard_rev);
	}

	jo_obj_end(jo);
}

static void build_cards_overview(struct json_out *jo)
{
	jo_obj_begin(jo);

	jo_key(jo, "slots");
	jo_arr_begin(jo);
	for (int s = 0; s < CARD_MAX_SLOTS; s++) {
		build_card_slot(jo, s);
	}
	jo_arr_end(jo);

	/* Last I2C bus scan result */
	struct card_i2c_scan_result scan;

	card_manager_get_scan_result(&scan);

	jo_key(jo, "i2c_devices");
	jo_arr_begin(jo);
	for (int i = 0; i < scan.count; i++) {
		jo_fmt(jo, "%d,", scan.addr[i]);
	}
	jo_arr_end(jo);

	jo_obj_end(jo);
}

static int get_cards(struct webapi_request *req)
{
	build_cards_overview(&req->out);
	return 0;
}

static int post_rescan(struct webapi_request *req)
{
	card_manager_rescan();
	/* The re-scanned card comes up at hardware defaults. */
	card_settings_apply();
	build_cards_overview(&req->out);
	return 0;
}

#ifdef CONFIG_MI_CARD
/* ---------------- /api/cards/mi ---------------- */

static void build_mi_status(struct json_out *jo)
{
	jo_obj_begin(jo);

	jo_bool(jo, "hpf", mi_card_get_hpf() > 0);
	jo_bool(jo, "f96khz", mi_card_get_96khz() > 0);

	jo_key(jo, "channels");
	jo_arr_begin(jo);

	for (int ch = 0; ch < MI_NUM_CHANNELS; ch++) {
		jo_obj_begin(jo);
		jo_uint(jo, "id", ch);
		jo_int(jo, "gain", mi_card_get_gain(ch));
		jo_bool(jo, "phantom", mi_card_get_phantom(ch) > 0);
		jo_bool(jo, "muted", mi_card_get_mute(ch) > 0);
		jo_obj_end(jo);
	}

	jo_arr_end(jo);
	jo_obj_end(jo);
}

static int get_mi(struct webapi_request *req)
{
	build_mi_status(&req->out);
	return 0;
}

static int patch_mi(struct webapi_request *req)
{
	struct card_flags_req f = { 0 };
	int64_t present = parse_flags(req, &f);
	int ret;

	if (present < 0) {
		return -EINVAL;
	}
	if (WEBAPI_HAS(present, CF_HPF)) {
		ret = mi_card_set_hpf(f.hpf);
		if (ret < 0) {
			return ret;
		}
	}
	if (WEBAPI_HAS(present, CF_F96KHZ)) {
		ret = mi_card_set_96khz(f.f96khz);
		if (ret < 0) {
			return ret;
		}
	}

	settings_changed();
	build_mi_status(&req->out);
	return 0;
}

static int patch_mi_channel(struct webapi_request *req)
{
	struct card_chan_req c = { 0 };
	int64_t present;
	int ret;

	if (req->id < 0 || req->id >= MI_NUM_CHANNELS) {
		return -EINVAL;
	}

	present = parse_chan(req, &c);
	if (present < 0) {
		return -EINVAL;
	}
	if (WEBAPI_HAS(present, CC_GAIN)) {
		ret = mi_card_set_gain((uint8_t)req->id, (int8_t)c.gain);
		if (ret < 0) {
			return ret;
		}
	}
	if (WEBAPI_HAS(present, CC_PHANTOM)) {
		ret = mi_card_set_phantom((uint8_t)req->id, c.phantom);
		if (ret < 0) {
			return ret;
		}
	}
	if (WEBAPI_HAS(present, CC_MUTED)) {
		ret = mi_card_set_mute((uint8_t)req->id, c.muted);
		if (ret < 0) {
			return ret;
		}
	}

	settings_changed();
	build_mi_status(&req->out);
	return 0;
}

static int post_mi_reset(struct webapi_request *req)
{
	int ret;

#if AES67_SHARED_NRST_AVAILABLE
	ret = display_ctrl_hw_reset();
#else
	ret = mi_card_reset();
#endif
	if (ret < 0) {
		return ret;
	}

	card_settings_apply();

	jo_obj_begin(&req->out);
	jo_bool(&req->out, "ok", true);
	jo_str(&req->out, "message", "Board reset complete");
	jo_obj_end(&req->out);
	return 0;
}

static int get_mi_detect(struct webapi_request *req)
{
	struct mi_board_info info;

	if (!mi_card_detect(&info)) {
		return -ENODEV;
	}

	jo_obj_begin(&req->out);
	jo_int(&req->out, "soft_id", info.soft_id);
	jo_int(&req->out, "board_id", info.board_id);
	jo_int(&req->out, "rev", info.hard_rev);
	jo_obj_end(&req->out);
	return 0;
}
#endif /* CONFIG_MI_CARD */

#ifdef CONFIG_LO_CARD
/* ---------------- /api/cards/lo ---------------- */

static void build_lo_status(struct json_out *jo)
{
	jo_obj_begin(jo);

	jo_bool(jo, "f96khz", lo_card_get_96khz() > 0);
	jo_bool(jo, "output_enable", lo_card_get_output_enable() > 0);

	jo_key(jo, "channels");
	jo_arr_begin(jo);

	for (int ch = 0; ch < LO_NUM_CHANNELS; ch++) {
		jo_obj_begin(jo);
		jo_uint(jo, "id", ch);
		jo_int(jo, "clip", lo_card_get_clip(ch));
		jo_bool(jo, "muted", lo_card_get_mute(ch) > 0);
		jo_obj_end(jo);
	}

	jo_arr_end(jo);
	jo_obj_end(jo);
}

static int get_lo(struct webapi_request *req)
{
	build_lo_status(&req->out);
	return 0;
}

static int patch_lo(struct webapi_request *req)
{
	struct card_flags_req f = { 0 };
	int64_t present = parse_flags(req, &f);
	int ret;

	if (present < 0) {
		return -EINVAL;
	}
	if (WEBAPI_HAS(present, CF_F96KHZ)) {
		ret = lo_card_set_96khz(f.f96khz);
		if (ret < 0) {
			return ret;
		}
	}
	if (WEBAPI_HAS(present, CF_OUTPUT_ENABLE)) {
		ret = lo_card_enable_outputs(f.output_enable);
		if (ret < 0) {
			return ret;
		}
	}

	settings_changed();
	build_lo_status(&req->out);
	return 0;
}

static int patch_lo_channel(struct webapi_request *req)
{
	struct card_chan_req c = { 0 };
	int64_t present;
	int ret;

	if (req->id < 0 || req->id >= LO_NUM_CHANNELS) {
		return -EINVAL;
	}

	present = parse_chan(req, &c);
	if (present < 0) {
		return -EINVAL;
	}
	if (WEBAPI_HAS(present, CC_CLIP)) {
		ret = lo_card_set_clip((uint8_t)req->id, (int8_t)c.clip);
		if (ret < 0) {
			return ret;
		}
	}
	if (WEBAPI_HAS(present, CC_MUTED)) {
		ret = lo_card_set_mute((uint8_t)req->id, c.muted);
		if (ret < 0) {
			return ret;
		}
	}

	settings_changed();
	build_lo_status(&req->out);
	return 0;
}

static int post_lo_reset(struct webapi_request *req)
{
	int ret;

#if AES67_SHARED_NRST_AVAILABLE
	ret = display_ctrl_hw_reset();
#else
	ret = lo_card_reset();
#endif
	if (ret < 0) {
		return ret;
	}

	card_settings_apply();

	jo_obj_begin(&req->out);
	jo_bool(&req->out, "ok", true);
	jo_str(&req->out, "message", "Board reset complete");
	jo_obj_end(&req->out);
	return 0;
}

static int get_lo_detect(struct webapi_request *req)
{
	struct lo_board_info info;

	if (!lo_card_detect(&info)) {
		return -ENODEV;
	}

	jo_obj_begin(&req->out);
	jo_int(&req->out, "soft_id", info.soft_id);
	jo_int(&req->out, "board_id", info.board_id);
	jo_int(&req->out, "rev", info.hard_rev);
	jo_obj_end(&req->out);
	return 0;
}
#endif /* CONFIG_LO_CARD */

#ifdef CONFIG_IO_CARD
/* ---------------- /api/cards/io ---------------- */

static bool io_card_ready(void)
{
	const struct card_info *ci = card_manager_get_info(CARD_SLOT_MAIN);

	return ci != NULL && ci->type == CARD_TYPE_IO && ci->initialized;
}

static void build_io_status(struct json_out *jo)
{
	jo_obj_begin(jo);

	jo_bool(jo, "output_enable", io_card_get_output_enable() > 0);

	/* Input channels */
	uint16_t overflow_mask = io_card_get_in_overflow();

	jo_key(jo, "inputs");
	jo_arr_begin(jo);
	for (int ch = 0; ch < IO_NUM_IN_CHANNELS; ch++) {
		jo_obj_begin(jo);
		jo_uint(jo, "id", (uint32_t)ch);
		jo_int(jo, "gain", io_card_get_in_gain(ch));
		jo_bool(jo, "phantom", io_card_get_in_phantom(ch) > 0);
		jo_bool(jo, "muted", io_card_get_in_mute(ch) > 0);
		jo_bool(jo, "clip", (overflow_mask & (1U << ch)) != 0);
		jo_obj_end(jo);
	}
	jo_arr_end(jo);

	/* Output channels */
	jo_key(jo, "outputs");
	jo_arr_begin(jo);
	for (int ch = 0; ch < IO_NUM_OUT_CHANNELS; ch++) {
		jo_obj_begin(jo);
		jo_uint(jo, "id", (uint32_t)ch);
		jo_int(jo, "clip", io_card_get_out_clip(ch));
		jo_bool(jo, "muted", io_card_get_out_mute(ch) > 0);
		jo_obj_end(jo);
	}
	jo_arr_end(jo);

	jo_obj_end(jo);
}

static int get_io(struct webapi_request *req)
{
	if (!io_card_ready()) {
		return -ENODEV;
	}
	build_io_status(&req->out);
	return 0;
}

static int patch_io(struct webapi_request *req)
{
	struct card_flags_req f = { 0 };
	int64_t present;
	int ret;

	if (!io_card_ready()) {
		return -ENODEV;
	}

	present = parse_flags(req, &f);
	if (present < 0) {
		return -EINVAL;
	}
	if (WEBAPI_HAS(present, CF_OUTPUT_ENABLE)) {
		ret = io_card_enable_outputs(f.output_enable);
		if (ret < 0) {
			return ret;
		}
	}
	if (WEBAPI_HAS(present, CF_F96KHZ)) {
		ret = io_card_set_96khz(f.f96khz);
		if (ret < 0) {
			return ret;
		}
	}

	settings_changed();
	build_io_status(&req->out);
	return 0;
}

static int patch_io_input(struct webapi_request *req)
{
	struct card_chan_req c = { 0 };
	int64_t present;
	int ret;

	if (!io_card_ready()) {
		return -ENODEV;
	}
	if (req->id < 0 || req->id >= IO_NUM_IN_CHANNELS) {
		return -EINVAL;
	}

	present = parse_chan(req, &c);
	if (present < 0) {
		return -EINVAL;
	}
	if (WEBAPI_HAS(present, CC_GAIN)) {
		ret = io_card_set_in_gain((uint8_t)req->id, (int8_t)c.gain);
		if (ret < 0) {
			return ret;
		}
	}
	if (WEBAPI_HAS(present, CC_PHANTOM)) {
		ret = io_card_set_in_phantom((uint8_t)req->id, c.phantom);
		if (ret < 0) {
			return ret;
		}
	}
	if (WEBAPI_HAS(present, CC_MUTED)) {
		ret = io_card_set_in_mute((uint8_t)req->id, c.muted);
		if (ret < 0) {
			return ret;
		}
	}

	settings_changed();
	build_io_status(&req->out);
	return 0;
}

static int patch_io_output(struct webapi_request *req)
{
	struct card_chan_req c = { 0 };
	int64_t present;
	int ret;

	if (!io_card_ready()) {
		return -ENODEV;
	}
	if (req->id < 0 || req->id >= IO_NUM_OUT_CHANNELS) {
		return -EINVAL;
	}

	present = parse_chan(req, &c);
	if (present < 0) {
		return -EINVAL;
	}
	if (WEBAPI_HAS(present, CC_CLIP)) {
		ret = io_card_set_out_clip((uint8_t)req->id, (int8_t)c.clip);
		if (ret < 0) {
			return ret;
		}
	}
	if (WEBAPI_HAS(present, CC_MUTED)) {
		ret = io_card_set_out_mute((uint8_t)req->id, c.muted);
		if (ret < 0) {
			return ret;
		}
	}

	settings_changed();
	build_io_status(&req->out);
	return 0;
}

static int post_io_reset(struct webapi_request *req)
{
	const struct card_info *ci = card_manager_get_info(CARD_SLOT_MAIN);

	ARG_UNUSED(req);

	if (ci == NULL || ci->type != CARD_TYPE_IO) {
		return -ENODEV;
	}

	int ret = io_card_reset();

	if (ret < 0) {
		return ret;
	}

	card_settings_apply();
	return 0;
}
#endif /* CONFIG_IO_CARD */

#ifdef CONFIG_AURACAST_CARD
/* ---------------- /api/cards/auracast ----------------
 *
 * Unlike the analog cards nothing here is written live: every accepted
 * PATCH (and start/stop/reset) runs the driver's configuration cycle,
 * which RESETs the S31 when it is broadcasting and waits up to ~3 s for
 * it to come back. The handler blocks for that time on the shared
 * shared /api resource, so a parallel request may see a 409 meanwhile (known
 * behaviour of Zephyr's HTTP server, see the UI which pauses its poll
 * during these actions). GET reads only the driver's cache.
 *
 * A configuration the S31 or the driver rejects answers 422 with the
 * offending register named, so the form can mark the field.
 */

/* Name/code buffers are oversized on purpose: the JSON parser fails the
 * whole body (400) when a string does not fit, but a too-long name is a
 * field error (422 "name"), so it has to fit here and be measured. */
#define AURA_REQ_NAME_MAX  (3 * AURA_NAME_SIZE)
#define AURA_REQ_CODE_MAX  (3 * AURA_CODE_SIZE)

struct aura_cfg_req {
	bool    enabled;
	int32_t preset;
	int32_t channels;
	int32_t context;
	int32_t flags;
	int32_t tx_power;
	int32_t pd_us;
	int32_t fifo_target;
	int64_t broadcast_id;
	bool    broadcast_id_random;
	char    name[AURA_REQ_NAME_MAX + 1];
	char    code[AURA_REQ_CODE_MAX + 1];
};

enum {
	AC_ENABLED, AC_PRESET, AC_CHANNELS, AC_CONTEXT, AC_FLAGS, AC_TX_POWER,
	AC_PD_US, AC_FIFO_TARGET, AC_BROADCAST_ID, AC_BROADCAST_ID_RANDOM,
	AC_NAME, AC_CODE,
};

static const struct json_obj_descr aura_cfg_descr[] = {
	[AC_ENABLED] = JSON_OBJ_DESCR_PRIM(struct aura_cfg_req, enabled, JSON_TOK_TRUE),
	[AC_PRESET] = JSON_OBJ_DESCR_PRIM(struct aura_cfg_req, preset, JSON_TOK_NUMBER),
	[AC_CHANNELS] = JSON_OBJ_DESCR_PRIM(struct aura_cfg_req, channels, JSON_TOK_NUMBER),
	[AC_CONTEXT] = JSON_OBJ_DESCR_PRIM(struct aura_cfg_req, context, JSON_TOK_NUMBER),
	[AC_FLAGS] = JSON_OBJ_DESCR_PRIM(struct aura_cfg_req, flags, JSON_TOK_NUMBER),
	[AC_TX_POWER] = JSON_OBJ_DESCR_PRIM(struct aura_cfg_req, tx_power, JSON_TOK_NUMBER),
	[AC_PD_US] = JSON_OBJ_DESCR_PRIM(struct aura_cfg_req, pd_us, JSON_TOK_NUMBER),
	[AC_FIFO_TARGET] = JSON_OBJ_DESCR_PRIM(struct aura_cfg_req, fifo_target,
					       JSON_TOK_NUMBER),
	[AC_BROADCAST_ID] = JSON_OBJ_DESCR_PRIM(struct aura_cfg_req, broadcast_id,
						JSON_TOK_INT64),
	[AC_BROADCAST_ID_RANDOM] = JSON_OBJ_DESCR_PRIM(struct aura_cfg_req,
						       broadcast_id_random, JSON_TOK_TRUE),
	[AC_NAME] = JSON_OBJ_DESCR_PRIM(struct aura_cfg_req, name, JSON_TOK_STRING_BUF),
	[AC_CODE] = JSON_OBJ_DESCR_PRIM(struct aura_cfg_req, code, JSON_TOK_STRING_BUF),
};

static void build_aura_status(struct json_out *jo, const struct auracast_status *st)
{
	char buf[16];

	jo_key(jo, "status");
	jo_obj_begin(jo);
	jo_bool(jo, "present", st->present);
	jo_bool(jo, "ready", auracast_card_ready());
	jo_bool(jo, "stats_valid", st->stats_valid);
	snprintf(buf, sizeof(buf), "%u.%u.%u", st->fw_major, st->fw_minor, st->fw_patch);
	jo_str(jo, "fw_version", buf);
	jo_uint(jo, "state", st->state);
	jo_str(jo, "state_name", aura_proto_state_name(st->state));
	jo_uint(jo, "error", st->error);
	jo_str(jo, "error_name", aura_proto_error_name(st->error));
	jo_uint(jo, "error_reg", st->error_reg);
	jo_str(jo, "error_reg_name", aura_proto_reg_name(st->error_reg));
	jo_uint(jo, "reset_reason", st->reset_reason);
	jo_uint(jo, "pipe_state", st->stats.pipe_state);
	jo_str(jo, "pipe_state_name", aura_proto_pipe_name(st->stats.pipe_state));
	jo_bool(jo, "i2s_signal", (st->stats.status_flags & AURA_STATUS_I2S_SIGNAL) != 0);
	jo_bool(jo, "bis_streaming",
		(st->stats.status_flags & AURA_STATUS_BIS_STREAMING) != 0);
	jo_uint(jo, "fifo_fill", st->stats.fifo_fill);
	jo_int(jo, "drift_ppm", st->stats.drift_cppm / 100);
	jo_int(jo, "in_ppm", st->stats.in_cppm / 100);
	snprintf(buf, sizeof(buf), "0x%08x", st->stats.i2s_bits);
	jo_str(jo, "i2s_bits", buf);
	jo_uint(jo, "underrun", st->stats.underrun);
	jo_uint(jo, "overflow", st->stats.overflow);
	jo_uint(jo, "lost", st->stats.lost);
	jo_uint(jo, "send_err", st->stats.send_err);
	jo_uint(jo, "sent", st->stats.sent);
	if (st->stats_valid) {
		snprintf(buf, sizeof(buf), "0x%06x", st->stats.active_id & 0xFFFFFFu);
		jo_str(jo, "active_id", buf);
	} else {
		jo_raw(jo, "\"active_id\":null,");
	}
	jo_uint(jo, "uptime_ms", st->stats.uptime_ms);
	jo_int(jo, "last_errno", st->last_errno);
	jo_uint(jo, "age_ms", st->age_ms);
	jo_obj_end(jo);
}

static void build_aura_config(struct json_out *jo, const struct auracast_config *cfg)
{
	jo_key(jo, "config");
	jo_obj_begin(jo);
	jo_bool(jo, "enabled", cfg->enabled);
	jo_uint(jo, "preset", cfg->preset);
	jo_uint(jo, "channels", cfg->channels);
	jo_uint(jo, "context", cfg->context);
	jo_uint(jo, "flags", cfg->flags);
	jo_int(jo, "tx_power", cfg->tx_power_dbm);
	jo_uint(jo, "pd_us", cfg->pd_us);
	jo_uint(jo, "fifo_target", cfg->fifo_target);
	jo_uint(jo, "broadcast_id", cfg->broadcast_id);
	jo_bool(jo, "broadcast_id_random", cfg->broadcast_id == AURA_BROADCAST_ID_RANDOM);
	jo_str(jo, "name", cfg->name);
	jo_str(jo, "code", cfg->code);
	jo_obj_end(jo);
}

static void build_aura_choices(struct json_out *jo)
{
	jo_key(jo, "presets");
	jo_arr_begin(jo);
	for (uint8_t i = 0; i < AURA_PRESET_COUNT; i++) {
		jo_obj_begin(jo);
		jo_uint(jo, "id", i);
		jo_str(jo, "label", aura_proto_preset_label(i));
		jo_obj_end(jo);
	}
	jo_arr_end(jo);

	jo_key(jo, "channel_modes");
	jo_arr_begin(jo);
	for (uint8_t i = 0; i < AURA_CH_COUNT; i++) {
		jo_obj_begin(jo);
		jo_uint(jo, "id", i);
		jo_str(jo, "label", aura_proto_channels_label(i));
		jo_obj_end(jo);
	}
	jo_arr_end(jo);
}

static void build_aura(struct json_out *jo)
{
	struct auracast_status st;
	struct auracast_config cfg;

	(void)auracast_card_get_status(&st);
	(void)auracast_card_get_config(&cfg);

	jo_obj_begin(jo);
	build_aura_status(jo, &st);
	build_aura_config(jo, &cfg);
	build_aura_choices(jo);
	jo_obj_end(jo);
}

static bool aura_present(void)
{
	const struct card_info *info = card_manager_get_info(CARD_SLOT_MAIN);

	return info != NULL && info->present && info->type == CARD_TYPE_AURACAST &&
	       auracast_card_ready();
}

/* 422 with the rejected register named; the handler then returns 0 so
 * the core keeps this body instead of its generic error JSON. */
static void aura_reply_rejected(struct webapi_request *req, uint8_t error,
				uint8_t error_reg)
{
	struct auracast_status st;
	struct auracast_config cfg;

	(void)auracast_card_get_status(&st);
	(void)auracast_card_get_config(&cfg);

	req->status = HTTP_422_UNPROCESSABLE_ENTITY;
	jo_obj_begin(&req->out);
	jo_str(&req->out, "error", aura_proto_error_name(error));
	jo_uint(&req->out, "error_reg", error_reg);
	jo_str(&req->out, "error_reg_name", aura_proto_reg_name(error_reg));
	jo_str(&req->out, "message", "configuration rejected");
	build_aura_status(&req->out, &st);
	build_aura_config(&req->out, &cfg);
	jo_obj_end(&req->out);
}

/* Shared tail of PATCH/start/stop/reset: map the cycle result. */
static int aura_finish(struct webapi_request *req, int ret)
{
	if (ret == -EINVAL) {
		struct auracast_status st;

		(void)auracast_card_get_status(&st);
		aura_reply_rejected(req, st.error, st.error_reg);
		return 0;
	}
	if (ret < 0) {
		return ret;
	}
	card_settings_schedule_save();
	build_aura(&req->out);
	return 0;
}

static int get_aura(struct webapi_request *req)
{
	if (!aura_present()) {
		return -ENODEV;
	}
	build_aura(&req->out);
	return 0;
}

static int patch_aura(struct webapi_request *req)
{
	struct aura_cfg_req r = { 0 };
	struct auracast_config cfg;
	int64_t present;
	size_t len;

	if (!aura_present()) {
		return -ENODEV;
	}

	(void)auracast_card_get_config(&cfg);

	present = req->body_len ? webapi_parse_body(req, aura_cfg_descr,
						    ARRAY_SIZE(aura_cfg_descr), &r) : 0;
	if (present < 0) {
		return -EINVAL;
	}

	if (WEBAPI_HAS(present, AC_ENABLED)) {
		cfg.enabled = r.enabled;
	}
	if (WEBAPI_HAS(present, AC_PRESET)) {
		cfg.preset = (uint8_t)CLAMP(r.preset, 0, 255);
	}
	if (WEBAPI_HAS(present, AC_CHANNELS)) {
		cfg.channels = (uint8_t)CLAMP(r.channels, 0, 255);
	}
	if (WEBAPI_HAS(present, AC_CONTEXT)) {
		if (r.context < 0 || r.context > 0xFFFF) {
			aura_reply_rejected(req, AURA_ERR_BAD_VALUE, AURA_REG_CONTEXT);
			return 0;
		}
		cfg.context = (uint16_t)r.context;
	}
	if (WEBAPI_HAS(present, AC_FLAGS)) {
		cfg.flags = (uint8_t)CLAMP(r.flags, 0, 255);
	}
	if (WEBAPI_HAS(present, AC_TX_POWER)) {
		if (r.tx_power < INT8_MIN || r.tx_power > INT8_MAX) {
			aura_reply_rejected(req, AURA_ERR_BAD_VALUE, AURA_REG_TX_POWER);
			return 0;
		}
		cfg.tx_power_dbm = (int8_t)r.tx_power;
	}
	if (WEBAPI_HAS(present, AC_PD_US)) {
		if (r.pd_us < 0 || r.pd_us > 0xFFFF) {
			aura_reply_rejected(req, AURA_ERR_BAD_VALUE, AURA_REG_PD_US);
			return 0;
		}
		cfg.pd_us = (uint16_t)r.pd_us;
	}
	if (WEBAPI_HAS(present, AC_FIFO_TARGET)) {
		if (r.fifo_target < 0 || r.fifo_target > 0xFFFF) {
			aura_reply_rejected(req, AURA_ERR_BAD_VALUE, AURA_REG_FIFO_TARGET);
			return 0;
		}
		cfg.fifo_target = (uint16_t)r.fifo_target;
	}
	if (WEBAPI_HAS(present, AC_BROADCAST_ID)) {
		if (r.broadcast_id < 0 || r.broadcast_id > (int64_t)UINT32_MAX) {
			aura_reply_rejected(req, AURA_ERR_BAD_VALUE, AURA_REG_BROADCAST_ID);
			return 0;
		}
		cfg.broadcast_id = (uint32_t)r.broadcast_id;
	}
	if (WEBAPI_HAS(present, AC_BROADCAST_ID_RANDOM) && r.broadcast_id_random) {
		cfg.broadcast_id = AURA_BROADCAST_ID_RANDOM;
	}
	if (WEBAPI_HAS(present, AC_NAME)) {
		len = strlen(r.name);   /* bytes, not characters */
		if (len > AURA_NAME_SIZE) {
			aura_reply_rejected(req, AURA_ERR_BAD_VALUE, AURA_REG_NAME);
			return 0;
		}
		memcpy(cfg.name, r.name, len + 1);
	}
	if (WEBAPI_HAS(present, AC_CODE)) {
		len = strlen(r.code);
		if (len > AURA_CODE_SIZE) {
			aura_reply_rejected(req, AURA_ERR_BAD_VALUE, AURA_REG_CODE);
			return 0;
		}
		memcpy(cfg.code, r.code, len + 1);
	}

	return aura_finish(req, auracast_card_set_config(&cfg));
}

static int post_aura_start(struct webapi_request *req)
{
	if (!aura_present()) {
		return -ENODEV;
	}
	return aura_finish(req, auracast_card_start());
}

static int post_aura_stop(struct webapi_request *req)
{
	if (!aura_present()) {
		return -ENODEV;
	}
	return aura_finish(req, auracast_card_stop());
}

static int post_aura_reset(struct webapi_request *req)
{
	if (!aura_present()) {
		return -ENODEV;
	}
	return aura_finish(req, auracast_card_restart());
}
#endif /* CONFIG_AURACAST_CARD */

static const struct webapi_route routes[] = {
	WEBAPI_ROUTE(HTTP_GET,  "/api/cards",        get_cards),
	WEBAPI_ROUTE(HTTP_POST, "/api/cards/rescan", post_rescan),
#ifdef CONFIG_MI_CARD
	WEBAPI_ROUTE(HTTP_GET,   "/api/cards/mi",               get_mi),
	WEBAPI_ROUTE(HTTP_PATCH, "/api/cards/mi",               patch_mi),
	WEBAPI_ROUTE(HTTP_PATCH, "/api/cards/mi/channels/{id}", patch_mi_channel),
	WEBAPI_ROUTE(HTTP_POST,  "/api/cards/mi/reset",         post_mi_reset),
	WEBAPI_ROUTE(HTTP_GET,   "/api/cards/mi/detect",        get_mi_detect),
#endif
#ifdef CONFIG_LO_CARD
	WEBAPI_ROUTE(HTTP_GET,   "/api/cards/lo",               get_lo),
	WEBAPI_ROUTE(HTTP_PATCH, "/api/cards/lo",               patch_lo),
	WEBAPI_ROUTE(HTTP_PATCH, "/api/cards/lo/channels/{id}", patch_lo_channel),
	WEBAPI_ROUTE(HTTP_POST,  "/api/cards/lo/reset",         post_lo_reset),
	WEBAPI_ROUTE(HTTP_GET,   "/api/cards/lo/detect",        get_lo_detect),
#endif
#ifdef CONFIG_IO_CARD
	WEBAPI_ROUTE(HTTP_GET,   "/api/cards/io",               get_io),
	WEBAPI_ROUTE(HTTP_PATCH, "/api/cards/io",               patch_io),
	WEBAPI_ROUTE(HTTP_PATCH, "/api/cards/io/inputs/{id}",   patch_io_input),
	WEBAPI_ROUTE(HTTP_PATCH, "/api/cards/io/outputs/{id}",  patch_io_output),
	WEBAPI_ROUTE(HTTP_POST,  "/api/cards/io/reset",         post_io_reset),
#endif
#ifdef CONFIG_AURACAST_CARD
	WEBAPI_ROUTE(HTTP_GET,   "/api/cards/auracast",         get_aura),
	WEBAPI_ROUTE(HTTP_PATCH, "/api/cards/auracast",         patch_aura),
	WEBAPI_ROUTE(HTTP_POST,  "/api/cards/auracast/start",   post_aura_start),
	WEBAPI_ROUTE(HTTP_POST,  "/api/cards/auracast/stop",    post_aura_stop),
	WEBAPI_ROUTE(HTTP_POST,  "/api/cards/auracast/reset",   post_aura_reset),
#endif
};

const struct webapi_module webapi_cards_module = {
	.routes = routes,
	.count = ARRAY_SIZE(routes),
};
