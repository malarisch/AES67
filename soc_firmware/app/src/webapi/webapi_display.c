/*
 *
 * /api/debug/display[...] — front-panel display / LED tester used by the
 * /debug page. Only built with CONFIG_DISPLAY_CTRL.
 */

#include <zephyr/kernel.h>

#include "webapi_priv.h"
#include "../../drivers/display_ctrl/display_ctrl.h"

/* ---------------- GET /api/debug/display ---------------- */

static int get_display(struct webapi_request *req)
{
	struct json_out *jo = &req->out;

	jo_obj_begin(jo);
	jo_bool(jo, "ready", display_ctrl_ready());

	/* 7-segment state (character names) */
	static const char *vcc_names[] = {
		"0","1","2","3","4","5","6","7","8","9",
		"A","b","C","d","E","F","G","h","I","J",
		"L","M","n","O","P","q","r","S","t","U",
		"X","y"," ","-"
	};

	jo_key(jo, "segments");
	jo_obj_begin(jo);
	for (int d = 0; d < DC_NUM_DISPLAYS; d++) {
		const char *dname = d == 0 ? "left" : d == 1 ? "mid" : "right";

		jo_key(jo, dname);
		jo_obj_begin(jo);

		int l = display_ctrl_get_segment_left(d);
		int r = display_ctrl_get_segment_right(d);

		if (l >= 0 && l < VCC_LAST) {
			jo_str(jo, "left", vcc_names[l]);
		}
		if (r >= 0 && r < VCC_LAST) {
			jo_str(jo, "right", vcc_names[r]);
		}
		jo_obj_end(jo);
	}
	jo_obj_end(jo);

	/* Channel LEDs */
	jo_key(jo, "channel_leds");
	jo_obj_begin(jo);
	jo_uint(jo, "mute",
		display_ctrl_get_channel_led_pattern(DC_CHNLED_MUTE));
	jo_uint(jo, "signal",
		display_ctrl_get_channel_led_pattern(DC_CHNLED_SIGNAL));
	jo_uint(jo, "clip",
		display_ctrl_get_channel_led_pattern(DC_CHNLED_CLIP));
	jo_uint(jo, "phantom",
		display_ctrl_get_channel_led_pattern(DC_CHNLED_PHANTOM));
	jo_obj_end(jo);

	/* System LEDs */
	jo_key(jo, "sys_leds");
	jo_obj_begin(jo);
	static const char *sled_names[] = {
		"psu_b","lop","ext","96k","master","48k","lip","psu_a",
		"usb","eth","rem_lip","rem_lop","pwr"
	};
	for (int i = 0; i < DC_SYSLED_LAST && i < 13; i++) {
		jo_int(jo, sled_names[i], display_ctrl_get_sys_led(i));
	}
	jo_obj_end(jo);

	jo_int(jo, "max_channels", DC_MAX_CHANNELS);
	jo_obj_end(jo);
	return 0;
}

/* ---------------- POST endpoints ---------------- */

/* ASCII to VCC helper for web API */
static uint8_t ascii_to_vcc_web(char c)
{
	if (c >= '0' && c <= '9') {
		return VCC_NUM0 + (c - '0');
	}
	switch (c) {
	case 'A': case 'a': return VCC_A;
	case 'B': case 'b': return VCC_B;
	case 'C': case 'c': return VCC_C;
	case 'D': case 'd': return VCC_D;
	case 'E': case 'e': return VCC_E;
	case 'F': case 'f': return VCC_F;
	case 'G': case 'g': return VCC_G;
	case 'H': case 'h': return VCC_H;
	case 'I': case 'i': return VCC_I;
	case 'J': case 'j': return VCC_J;
	case 'L': case 'l': return VCC_L;
	case 'M': case 'm': return VCC_M;
	case 'N': case 'n': return VCC_N;
	case 'O': case 'o': return VCC_O;
	case 'P': case 'p': return VCC_P;
	case 'Q': case 'q': return VCC_Q;
	case 'R': case 'r': return VCC_R;
	case 'S': case 's': return VCC_S;
	case 'T': case 't': return VCC_T;
	case 'U': case 'u': return VCC_U;
	case 'X': case 'x': return VCC_X;
	case 'Y': case 'y': return VCC_Y;
	case '-': return VCC_MINUS;
	case ' ': default: return VCC_SPACE;
	}
}

/* Body: {"display":0, "left":"A", "right":"3"} */
static int post_segment(struct webapi_request *req)
{
	int32_t disp = 0;
	char left_str[4] = {0}, right_str[4] = {0};

	if (!display_ctrl_ready()) {
		return -ENODEV;
	}

	if (!json_find_int(req->body, req->body_len, "display", &disp)) {
		return -EINVAL;
	}
	if (disp < 0 || disp >= DC_NUM_DISPLAYS) {
		return -EINVAL;
	}

	json_find_str(req->body, req->body_len, "left",
		      left_str, sizeof(left_str));
	json_find_str(req->body, req->body_len, "right",
		      right_str, sizeof(right_str));

	/* Convert single characters to VCC codes */
	uint8_t lc = VCC_SPACE, rc = VCC_SPACE;

	if (left_str[0]) {
		lc = ascii_to_vcc_web(left_str[0]);
	}
	if (right_str[0]) {
		rc = ascii_to_vcc_web(right_str[0]);
	}

	return display_ctrl_set_segment(disp, lc, rc);
}

/* Body: {"text":"Ab Cd"} — up to 6 chars, spread across 3 displays */
static int post_text(struct webapi_request *req)
{
	char text[8] = {0};

	if (!display_ctrl_ready()) {
		return -ENODEV;
	}

	if (json_find_str(req->body, req->body_len, "text",
			  text, sizeof(text)) <= 0) {
		return -EINVAL;
	}

	return display_ctrl_show_status(text);
}

/* Body: {"led":0, "state":3}   (state: 0=off,1=blink1,2=blink2,3=on) */
static int post_sysled(struct webapi_request *req)
{
	int32_t led = -1, state = -1;

	if (!display_ctrl_ready()) {
		return -ENODEV;
	}

	if (!json_find_int(req->body, req->body_len, "led", &led) ||
	    !json_find_int(req->body, req->body_len, "state", &state)) {
		return -EINVAL;
	}
	if (led < 0 || led >= DC_SYSLED_LAST || state < 0 || state > 4) {
		return -EINVAL;
	}

	return display_ctrl_set_sys_led(led, state);
}

/* Body: {"channel":0, "type":0, "on":true}
 *   type: 0=mute, 1=signal, 2=clip, 3=phantom
 * OR set all at once:
 * Body: {"type":0, "pattern":255}  — bitmask of channels */
static int post_chnled(struct webapi_request *req)
{
	int32_t channel = -1, type = -1, pattern = -1;
	bool on = false;

	if (!display_ctrl_ready()) {
		return -ENODEV;
	}

	if (!json_find_int(req->body, req->body_len, "type", &type)) {
		return -EINVAL;
	}
	if (type < 0 || type >= DC_CHNLED_LAST) {
		return -EINVAL;
	}

	/* Bulk mode: set entire pattern */
	if (json_find_int(req->body, req->body_len, "pattern", &pattern)) {
		return display_ctrl_set_channel_leds_by_type(type,
							     (uint32_t)pattern);
	}

	/* Single channel mode */
	if (!json_find_int(req->body, req->body_len, "channel", &channel)) {
		return -EINVAL;
	}
	if (channel < 0 || channel >= DC_MAX_CHANNELS) {
		return -EINVAL;
	}
	json_find_bool(req->body, req->body_len, "on", &on);

	return display_ctrl_set_channel_led(channel, type, on);
}

static int post_test(struct webapi_request *req)
{
	int ret = display_ctrl_full_test();

	if (ret < 0) {
		return ret;
	}

	jo_obj_begin(&req->out);
	jo_bool(&req->out, "ok", true);
	jo_str(&req->out, "message", "All LEDs on, 888888");
	jo_obj_end(&req->out);
	return 0;
}

static int post_clear(struct webapi_request *req)
{
	int ret = display_ctrl_all_off();

	if (ret < 0) {
		return ret;
	}

	jo_obj_begin(&req->out);
	jo_bool(&req->out, "ok", true);
	jo_str(&req->out, "message", "All off");
	jo_obj_end(&req->out);
	return 0;
}

static int post_reset(struct webapi_request *req)
{
#if defined(CONFIG_DISPLAY_CTRL_NRST_GPIO) || defined(CONFIG_DISPLAY_CTRL_NRST_HAL)
	int ret = display_ctrl_hw_reset();

	if (ret < 0) {
		return ret;
	}

	/* Re-init display UART after hw reset */
	display_ctrl_init(display_ctrl_get_uart());

	jo_obj_begin(&req->out);
	jo_bool(&req->out, "ok", true);
	jo_str(&req->out, "message", "Shared nRST reset complete");
	jo_obj_end(&req->out);
	return 0;
#else
	ARG_UNUSED(req);
	return -ENOTSUP;
#endif
}

static const struct webapi_route routes[] = {
	WEBAPI_ROUTE(HTTP_GET,  "/api/debug/display",         get_display),
	WEBAPI_ROUTE(HTTP_POST, "/api/debug/display/segment", post_segment),
	WEBAPI_ROUTE(HTTP_POST, "/api/debug/display/text",    post_text),
	WEBAPI_ROUTE(HTTP_POST, "/api/debug/display/sysled",  post_sysled),
	WEBAPI_ROUTE(HTTP_POST, "/api/debug/display/chnled",  post_chnled),
	WEBAPI_ROUTE(HTTP_POST, "/api/debug/display/test",    post_test),
	WEBAPI_ROUTE(HTTP_POST, "/api/debug/display/clear",   post_clear),
	WEBAPI_ROUTE(HTTP_POST, "/api/debug/display/reset",   post_reset),
};

const struct webapi_module webapi_display_module = {
	.routes = routes,
	.count = ARRAY_SIZE(routes),
};
