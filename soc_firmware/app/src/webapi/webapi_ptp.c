/*
 *
 * /api/ptp          — clock status, own announce quality, foreign masters
 * /api/ptp/tuning   — FPGA PI-servo tuning + live monitor (GET/PATCH)
 * /api/ptp/reset    — reset the PTP module + wallclock (POST)
 */

#include <zephyr/kernel.h>

#include "webapi_priv.h"
#include "../aes67_config.h"
#include "../ptp_ctrl.h"
#include "../../drivers/fpga_hal/fpga_hal.h"

static void format_clock_id(char *out, size_t sz, const uint8_t id[8])
{
	snprintf(out, sz,
		 "%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X",
		 id[0], id[1], id[2], id[3],
		 id[4], id[5], id[6], id[7]);
}

void webapi_build_ptp(struct json_out *jo)
{
	struct ptp_ctrl_status st;
	struct ptp_ctrl_foreign fms[8];
	char id_str[32];

	ptp_ctrl_get_status(&st);

	jo_obj_begin(jo);
	jo_str(jo, "mode", st.mode);

	const char *role_str =
		(st.role == PTP_CTRL_ROLE_LEADER)   ? "leader" :
		(st.role == PTP_CTRL_ROLE_FOLLOWER) ? "follower" : "listening";
	jo_str(jo, "role", role_str);

	format_clock_id(id_str, sizeof(id_str), st.clock_id);
	jo_str(jo, "clock_identity", id_str);

	if (st.gm_valid) {
		format_clock_id(id_str, sizeof(id_str), st.gm_id);
		jo_str(jo, "best_master", id_str);
	} else {
		jo_str(jo, "best_master", "none");
	}

	jo_int(jo, "offset_ns", st.offset_ns);
	jo_int(jo, "path_delay_ns", st.path_delay_ns);
	jo_bool(jo, "locked", st.locked);
	jo_uint(jo, "steps_removed", st.steps_removed);

	/* Own announce quality — identical to the stored config in both modes. */
	aes67_config_lock();
	const struct aes67_device_config *cfg = aes67_config_get();

	jo_uint(jo, "own_priority1", cfg->ptp_priority1);
	jo_uint(jo, "own_priority2", cfg->ptp_priority2);
	jo_uint(jo, "own_clock_class", cfg->ptp_clock_class);
	jo_uint(jo, "own_clock_accuracy", cfg->ptp_clock_accuracy);
	aes67_config_unlock();

	/* Foreign masters (software mode; the FPGA BMA keeps no list). */
	int fm_count = ptp_ctrl_get_foreign_masters(fms, ARRAY_SIZE(fms));

	jo_key(jo, "foreign_masters");
	jo_arr_begin(jo);

	for (int i = 0; i < fm_count; i++) {
		jo_obj_begin(jo);
		format_clock_id(id_str, sizeof(id_str), fms[i].sender_id);
		jo_str(jo, "sender_id", id_str);
		format_clock_id(id_str, sizeof(id_str), fms[i].gm_id);
		jo_str(jo, "gm_identity", id_str);
		jo_uint(jo, "priority1", fms[i].priority1);
		jo_uint(jo, "clock_class", fms[i].clock_class);
		jo_uint(jo, "clock_accuracy", fms[i].clock_accuracy);
		jo_uint(jo, "priority2", fms[i].priority2);
		jo_uint(jo, "steps_removed", fms[i].steps_removed);
		jo_uint(jo, "announce_count", fms[i].announce_count);
		jo_obj_end(jo);
	}

	jo_arr_end(jo);
	jo_obj_end(jo);
}

static int get_ptp(struct webapi_request *req)
{
	webapi_build_ptp(&req->out);
	return 0;
}

/* ---------------- /api/ptp/tuning ---------------- */

static void build_tuning(struct json_out *jo)
{
	struct fpga_hal_ptp_tuning t;
	struct fpga_hal_ptp_monitor m;

	fpga_hal_read_ptp_tuning(&t);
	fpga_hal_read_ptp_monitor(&m);

	jo_obj_begin(jo);

	/* Tuning (current values read back from FPGA) */
	jo_key(jo, "tuning");
	jo_obj_begin(jo);
	jo_int(jo, "kp_gain",                  t.kp_gain);
	jo_int(jo, "ki_gain",                  t.ki_gain);
	jo_uint(jo, "gain_shift",              t.gain_shift);
	jo_uint(jo, "gain_shift_locked",       t.gain_shift_locked);
	jo_uint(jo, "ki_extra_shift",          t.ki_extra_shift);
	jo_uint(jo, "filter_shift",            t.filter_shift);
	jo_uint(jo, "warmup_samples",          t.warmup_samples);
	jo_uint(jo, "lock_threshold_ns",       t.lock_threshold_ns);
	jo_uint(jo, "unlock_threshold_ns",     t.unlock_threshold_ns);
	jo_uint(jo, "lock_count_threshold",    t.lock_count_threshold);
	jo_bool(jo, "min_filter_enable",       t.min_filter_enable);
	jo_uint(jo, "min_filter_active_depth", t.min_filter_active_depth);
	jo_obj_end(jo);

	/* Monitoring (live PI internal state) */
	jo_key(jo, "monitor");
	jo_obj_begin(jo);
	jo_int(jo, "filtered_offset",       m.filtered_offset);
	jo_int(jo, "integral_sum",          m.integral_sum);
	jo_int(jo, "pi_proportional",       m.pi_proportional);
	jo_int(jo, "pi_sum_raw",            m.pi_sum_raw);
	jo_uint(jo, "effective_gain_shift", m.effective_gain_shift);
	jo_uint(jo, "lock_counter",         m.lock_counter);
	jo_uint(jo, "sample_count",         m.sample_count);
	jo_bool(jo, "first_lock_achieved",  m.first_lock_achieved);
	jo_obj_end(jo);

	jo_obj_end(jo);
}

static int get_tuning(struct webapi_request *req)
{
	build_tuning(&req->out);
	return 0;
}

static int patch_tuning(struct webapi_request *req)
{
	const char *json = req->body;
	size_t len = req->body_len;
	struct fpga_hal_ptp_tuning t;
	int32_t val;
	bool bval;
	int ret;

	/* Read current state, then patch only fields present in the request */
	fpga_hal_read_ptp_tuning(&t);

	if (json_find_int(json, len, "kp_gain", &val)) {
		t.kp_gain = (int8_t)val;
	}
	if (json_find_int(json, len, "ki_gain", &val)) {
		t.ki_gain = (int8_t)val;
	}
	if (json_find_int(json, len, "gain_shift", &val)) {
		t.gain_shift = (uint8_t)val;
	}
	if (json_find_int(json, len, "gain_shift_locked", &val)) {
		t.gain_shift_locked = (uint8_t)val;
	}
	if (json_find_int(json, len, "ki_extra_shift", &val)) {
		t.ki_extra_shift = (uint8_t)val;
	}
	if (json_find_int(json, len, "filter_shift", &val)) {
		t.filter_shift = (uint8_t)val;
	}
	if (json_find_int(json, len, "warmup_samples", &val)) {
		t.warmup_samples = (uint8_t)val;
	}
	if (json_find_int(json, len, "lock_threshold_ns", &val)) {
		t.lock_threshold_ns = (uint32_t)val;
	}
	if (json_find_int(json, len, "unlock_threshold_ns", &val)) {
		t.unlock_threshold_ns = (uint32_t)val;
	}
	if (json_find_int(json, len, "lock_count_threshold", &val)) {
		t.lock_count_threshold = (uint8_t)val;
	}
	if (json_find_bool(json, len, "min_filter_enable", &bval)) {
		t.min_filter_enable = bval;
	}
	if (json_find_int(json, len, "min_filter_active_depth", &val)) {
		t.min_filter_active_depth = (uint8_t)val;
	}

	ret = fpga_hal_write_ptp_tuning(&t);
	if (ret < 0) {
		return ret;
	}

	build_tuning(&req->out);
	return 0;
}

/* ---------------- /api/ptp/reset ---------------- */

static int post_reset(struct webapi_request *req)
{
	bool held;

	/* Optional "held" key: if absent, do a quick pulse (assert + release). */
	if (json_find_bool(req->body, req->body_len, "held", &held)) {
		return fpga_hal_set_ptp_reset(held);
	}

	int ret = fpga_hal_set_ptp_reset(true);

	if (ret < 0) {
		return ret;
	}
	k_msleep(10);
	return fpga_hal_set_ptp_reset(false);
}

static const struct webapi_route routes[] = {
	WEBAPI_ROUTE(HTTP_GET,   "/api/ptp",        get_ptp),
	WEBAPI_ROUTE(HTTP_GET,   "/api/ptp/tuning", get_tuning),
	WEBAPI_ROUTE(HTTP_PATCH, "/api/ptp/tuning", patch_tuning),
	WEBAPI_ROUTE(HTTP_POST,  "/api/ptp/reset",  post_reset),
};

const struct webapi_module webapi_ptp_module = {
	.routes = routes,
	.count = ARRAY_SIZE(routes),
};
