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

/* The servo tuning struct is described in place — its field names are
 * the wire names. delay_asymmetry_ns is deliberately absent: it comes
 * from the persisted device config via ptp_ctrl, not from this endpoint,
 * and read-modify-write keeps whatever is programmed. */
static const struct json_obj_descr tuning_descr[] = {
	JSON_OBJ_DESCR_PRIM(struct fpga_hal_ptp_tuning, kp_gain,
			    JSON_TOK_INT),
	JSON_OBJ_DESCR_PRIM(struct fpga_hal_ptp_tuning, ki_gain,
			    JSON_TOK_INT),
	JSON_OBJ_DESCR_PRIM(struct fpga_hal_ptp_tuning, gain_shift,
			    JSON_TOK_UINT),
	JSON_OBJ_DESCR_PRIM(struct fpga_hal_ptp_tuning, gain_shift_locked,
			    JSON_TOK_UINT),
	JSON_OBJ_DESCR_PRIM(struct fpga_hal_ptp_tuning, ki_extra_shift,
			    JSON_TOK_UINT),
	JSON_OBJ_DESCR_PRIM(struct fpga_hal_ptp_tuning, filter_shift,
			    JSON_TOK_UINT),
	JSON_OBJ_DESCR_PRIM(struct fpga_hal_ptp_tuning, warmup_samples,
			    JSON_TOK_UINT),
	JSON_OBJ_DESCR_PRIM(struct fpga_hal_ptp_tuning, lock_threshold_ns,
			    JSON_TOK_UINT),
	JSON_OBJ_DESCR_PRIM(struct fpga_hal_ptp_tuning, unlock_threshold_ns,
			    JSON_TOK_UINT),
	JSON_OBJ_DESCR_PRIM(struct fpga_hal_ptp_tuning, lock_count_threshold,
			    JSON_TOK_UINT),
	JSON_OBJ_DESCR_PRIM(struct fpga_hal_ptp_tuning, min_filter_enable,
			    JSON_TOK_TRUE),
	JSON_OBJ_DESCR_PRIM(struct fpga_hal_ptp_tuning,
			    min_filter_active_depth, JSON_TOK_UINT),
};

static int patch_tuning(struct webapi_request *req)
{
	struct fpga_hal_ptp_tuning t;
	int ret;

	/* Read current state, then patch only the fields the request
	 * carries — absent keys are simply not written. */
	fpga_hal_read_ptp_tuning(&t);

	if (webapi_parse_body(req, tuning_descr, ARRAY_SIZE(tuning_descr),
			      &t) < 0) {
		return -EINVAL;
	}

	ret = fpga_hal_write_ptp_tuning(&t);
	if (ret < 0) {
		return ret;
	}

	build_tuning(&req->out);
	return 0;
}

/* ---------------- /api/ptp/reset ---------------- */

struct reset_req {
	bool held;
};

enum { RESET_F_HELD = 0 };

static const struct json_obj_descr reset_descr[] = {
	[RESET_F_HELD] = JSON_OBJ_DESCR_PRIM(struct reset_req, held,
					     JSON_TOK_TRUE),
};

static int post_reset(struct webapi_request *req)
{
	struct reset_req r = { .held = false };
	int64_t present = webapi_parse_body(req, reset_descr,
					    ARRAY_SIZE(reset_descr), &r);

	/* Optional "held" key: if absent (or no body at all), do a quick
	 * pulse (assert + release). */
	if (present > 0 && WEBAPI_HAS(present, RESET_F_HELD)) {
		return fpga_hal_set_ptp_reset(r.held);
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
