/*
 * /api/cards                        — slot overview + last I2C scan (GET)
 * /api/cards/rescan                 — re-scan the card bus (POST)
 * /api/cards/mi[...]                — MI card (8-ch ADC preamp)
 * /api/cards/lo[...]                — LO card (8-ch line output)
 * /api/cards/io[...]                — IO card (16-in / 8-out combo)
 *
 * Per-card settings writes are persisted through the card_settings shadow
 * store with a debounced save (a dragged gain slider must not erase a
 * flash sector per step); reset/rescan re-apply the stored settings since
 * a re-initialised card is back at hardware defaults.
 */

#include <zephyr/kernel.h>

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
#ifdef CONFIG_DISPLAY_CTRL
#include "../../drivers/display_ctrl/display_ctrl.h"
#endif

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

/* Settings changed -> capture into the shadow store, save debounced. */
static void settings_changed(void)
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
	bool bval;
	int ret;

	if (json_find_bool(req->body, req->body_len, "hpf", &bval)) {
		ret = mi_card_set_hpf(bval);
		if (ret < 0) {
			return ret;
		}
	}
	if (json_find_bool(req->body, req->body_len, "f96khz", &bval)) {
		ret = mi_card_set_96khz(bval);
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
	int32_t val;
	bool bval;
	int ret;

	if (req->id < 0 || req->id >= MI_NUM_CHANNELS) {
		return -EINVAL;
	}

	if (json_find_int(req->body, req->body_len, "gain", &val)) {
		ret = mi_card_set_gain((uint8_t)req->id, (int8_t)val);
		if (ret < 0) {
			return ret;
		}
	}
	if (json_find_bool(req->body, req->body_len, "phantom", &bval)) {
		ret = mi_card_set_phantom((uint8_t)req->id, bval);
		if (ret < 0) {
			return ret;
		}
	}
	if (json_find_bool(req->body, req->body_len, "muted", &bval)) {
		ret = mi_card_set_mute((uint8_t)req->id, bval);
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
	bool bval;
	int ret;

	if (json_find_bool(req->body, req->body_len, "f96khz", &bval)) {
		ret = lo_card_set_96khz(bval);
		if (ret < 0) {
			return ret;
		}
	}
	if (json_find_bool(req->body, req->body_len, "output_enable", &bval)) {
		ret = lo_card_enable_outputs(bval);
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
	int32_t val;
	bool bval;
	int ret;

	if (req->id < 0 || req->id >= LO_NUM_CHANNELS) {
		return -EINVAL;
	}

	if (json_find_int(req->body, req->body_len, "clip", &val)) {
		ret = lo_card_set_clip((uint8_t)req->id, (int8_t)val);
		if (ret < 0) {
			return ret;
		}
	}
	if (json_find_bool(req->body, req->body_len, "muted", &bval)) {
		ret = lo_card_set_mute((uint8_t)req->id, bval);
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
	bool bval;
	int ret;

	if (!io_card_ready()) {
		return -ENODEV;
	}

	if (json_find_bool(req->body, req->body_len, "output_enable", &bval)) {
		ret = io_card_enable_outputs(bval);
		if (ret < 0) {
			return ret;
		}
	}
	if (json_find_bool(req->body, req->body_len, "f96khz", &bval)) {
		ret = io_card_set_96khz(bval);
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
	int32_t val;
	bool bval;
	int ret;

	if (!io_card_ready()) {
		return -ENODEV;
	}
	if (req->id < 0 || req->id >= IO_NUM_IN_CHANNELS) {
		return -EINVAL;
	}

	if (json_find_int(req->body, req->body_len, "gain", &val)) {
		ret = io_card_set_in_gain((uint8_t)req->id, (int8_t)val);
		if (ret < 0) {
			return ret;
		}
	}
	if (json_find_bool(req->body, req->body_len, "phantom", &bval)) {
		ret = io_card_set_in_phantom((uint8_t)req->id, bval);
		if (ret < 0) {
			return ret;
		}
	}
	if (json_find_bool(req->body, req->body_len, "muted", &bval)) {
		ret = io_card_set_in_mute((uint8_t)req->id, bval);
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
	int32_t val;
	bool bval;
	int ret;

	if (!io_card_ready()) {
		return -ENODEV;
	}
	if (req->id < 0 || req->id >= IO_NUM_OUT_CHANNELS) {
		return -EINVAL;
	}

	if (json_find_int(req->body, req->body_len, "clip", &val)) {
		ret = io_card_set_out_clip((uint8_t)req->id, (int8_t)val);
		if (ret < 0) {
			return ret;
		}
	}
	if (json_find_bool(req->body, req->body_len, "muted", &bval)) {
		ret = io_card_set_out_mute((uint8_t)req->id, bval);
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
};

const struct webapi_module webapi_cards_module = {
	.routes = routes,
	.count = ARRAY_SIZE(routes),
};
