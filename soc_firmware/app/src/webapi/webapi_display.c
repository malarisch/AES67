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
struct segment_req {
	int32_t display;
	char    left[4];
	char    right[4];
};

enum { SEG_F_DISPLAY, SEG_F_LEFT, SEG_F_RIGHT };

static const struct json_obj_descr segment_descr[] = {
	[SEG_F_DISPLAY] = JSON_OBJ_DESCR_PRIM(struct segment_req, display,
					      JSON_TOK_NUMBER),
	[SEG_F_LEFT] = JSON_OBJ_DESCR_PRIM(struct segment_req, left,
					   JSON_TOK_STRING_BUF),
	[SEG_F_RIGHT] = JSON_OBJ_DESCR_PRIM(struct segment_req, right,
					    JSON_TOK_STRING_BUF),
};

static int post_segment(struct webapi_request *req)
{
	struct segment_req r = { .display = 0, .left = {0}, .right = {0} };
	int64_t present;

	if (!display_ctrl_ready()) {
		return -ENODEV;
	}

	present = webapi_parse_body(req, segment_descr,
				    ARRAY_SIZE(segment_descr), &r);
	if (present < 0 || !WEBAPI_HAS(present, SEG_F_DISPLAY)) {
		return -EINVAL;
	}
	if (r.display < 0 || r.display >= DC_NUM_DISPLAYS) {
		return -EINVAL;
	}

	/* Convert single characters to VCC codes */
	uint8_t lc = VCC_SPACE, rc = VCC_SPACE;

	if (r.left[0]) {
		lc = ascii_to_vcc_web(r.left[0]);
	}
	if (r.right[0]) {
		rc = ascii_to_vcc_web(r.right[0]);
	}

	return display_ctrl_set_segment(r.display, lc, rc);
}

/* Body: {"text":"Ab Cd"} — up to 6 chars, spread across 3 displays */
struct text_req {
	char text[8];
};

enum { TEXT_F_TEXT };

static const struct json_obj_descr text_descr[] = {
	[TEXT_F_TEXT] = JSON_OBJ_DESCR_PRIM(struct text_req, text,
					    JSON_TOK_STRING_BUF),
};

static int post_text(struct webapi_request *req)
{
	struct text_req r = { .text = {0} };
	int64_t present;

	if (!display_ctrl_ready()) {
		return -ENODEV;
	}

	present = webapi_parse_body(req, text_descr, ARRAY_SIZE(text_descr),
				    &r);
	if (present < 0 || !WEBAPI_HAS(present, TEXT_F_TEXT) ||
	    r.text[0] == '\0') {
		return -EINVAL;
	}

	return display_ctrl_show_status(r.text);
}

/* Body: {"led":0, "state":3}   (state: 0=off,1=blink1,2=blink2,3=on) */
struct sysled_req {
	int32_t led;
	int32_t state;
};

enum { SYSLED_F_LED, SYSLED_F_STATE };

static const struct json_obj_descr sysled_descr[] = {
	[SYSLED_F_LED] = JSON_OBJ_DESCR_PRIM(struct sysled_req, led,
					     JSON_TOK_NUMBER),
	[SYSLED_F_STATE] = JSON_OBJ_DESCR_PRIM(struct sysled_req, state,
					       JSON_TOK_NUMBER),
};

static int post_sysled(struct webapi_request *req)
{
	struct sysled_req r = { .led = -1, .state = -1 };
	int64_t present;

	if (!display_ctrl_ready()) {
		return -ENODEV;
	}

	present = webapi_parse_body(req, sysled_descr,
				    ARRAY_SIZE(sysled_descr), &r);
	if (present < 0 || !WEBAPI_HAS(present, SYSLED_F_LED) ||
	    !WEBAPI_HAS(present, SYSLED_F_STATE)) {
		return -EINVAL;
	}
	if (r.led < 0 || r.led >= DC_SYSLED_LAST || r.state < 0 ||
	    r.state > 4) {
		return -EINVAL;
	}

	return display_ctrl_set_sys_led(r.led, r.state);
}

/* Body: {"channel":0, "type":0, "on":true}
 *   type: 0=mute, 1=signal, 2=clip, 3=phantom
 * OR set all at once:
 * Body: {"type":0, "pattern":255}  — bitmask of channels */
struct chnled_req {
	int32_t channel;
	int32_t type;
	int32_t pattern;
	bool    on;
};

enum { CHN_F_CHANNEL, CHN_F_TYPE, CHN_F_PATTERN, CHN_F_ON };

static const struct json_obj_descr chnled_descr[] = {
	[CHN_F_CHANNEL] = JSON_OBJ_DESCR_PRIM(struct chnled_req, channel,
					      JSON_TOK_NUMBER),
	[CHN_F_TYPE] = JSON_OBJ_DESCR_PRIM(struct chnled_req, type,
					   JSON_TOK_NUMBER),
	[CHN_F_PATTERN] = JSON_OBJ_DESCR_PRIM(struct chnled_req, pattern,
					      JSON_TOK_NUMBER),
	[CHN_F_ON] = JSON_OBJ_DESCR_PRIM(struct chnled_req, on,
					 JSON_TOK_TRUE),
};

static int post_chnled(struct webapi_request *req)
{
	struct chnled_req r = {
		.channel = -1, .type = -1, .pattern = -1, .on = false,
	};
	int64_t present;

	if (!display_ctrl_ready()) {
		return -ENODEV;
	}

	present = webapi_parse_body(req, chnled_descr,
				    ARRAY_SIZE(chnled_descr), &r);
	if (present < 0 || !WEBAPI_HAS(present, CHN_F_TYPE)) {
		return -EINVAL;
	}
	if (r.type < 0 || r.type >= DC_CHNLED_LAST) {
		return -EINVAL;
	}

	/* Bulk mode: set entire pattern */
	if (WEBAPI_HAS(present, CHN_F_PATTERN)) {
		return display_ctrl_set_channel_leds_by_type(
			r.type, (uint32_t)r.pattern);
	}

	/* Single channel mode */
	if (!WEBAPI_HAS(present, CHN_F_CHANNEL)) {
		return -EINVAL;
	}
	if (r.channel < 0 || r.channel >= DC_MAX_CHANNELS) {
		return -EINVAL;
	}

	return display_ctrl_set_channel_led(r.channel, r.type, r.on);
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
