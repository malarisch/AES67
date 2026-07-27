/*
 *
 * /api/config — device configuration (GET full document, PATCH partial).
 * A successful PATCH persists the config and re-applies the PTP settings
 * to the active PTP stack.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "webapi_priv.h"
#include "../aes67_config.h"
#include "../ptp_ctrl.h"
#include "../sap_sdp.h"

LOG_MODULE_DECLARE(webapi);

static void build_config(struct json_out *jo)
{
	aes67_config_lock();
	const struct aes67_device_config *cfg = aes67_config_get();

	jo_obj_begin(jo);
	jo_str(jo, "device_name", cfg->device_name);
	jo_str(jo, "default_mcast_addr", cfg->default_mcast_addr);
	jo_uint(jo, "default_port", cfg->default_port);
	jo_uint(jo, "default_channels", cfg->default_channels);
	jo_uint(jo, "default_bit_depth", cfg->default_bit_depth);
	jo_uint(jo, "default_sample_rate", cfg->default_sample_rate);
	jo_uint(jo, "default_samples_per_pkt", cfg->default_samples_per_pkt);
	jo_uint(jo, "default_payload_type", cfg->default_payload_type);
	jo_uint(jo, "ptp_domain", cfg->ptp_domain);
	jo_uint(jo, "ptp_priority1", cfg->ptp_priority1);
	jo_uint(jo, "ptp_priority2", cfg->ptp_priority2);
	jo_uint(jo, "ptp_clock_class", cfg->ptp_clock_class);
	jo_uint(jo, "ptp_clock_accuracy", cfg->ptp_clock_accuracy);
	jo_int(jo, "ptp_log_sync_interval", cfg->ptp_log_sync_interval);
	jo_int(jo, "ptp_log_announce_interval",
	       cfg->ptp_log_announce_interval);
	jo_int(jo, "pi_kp_num", cfg->pi_kp_num);
	jo_int(jo, "pi_kp_den", cfg->pi_kp_den);
	jo_int(jo, "pi_ki_num", cfg->pi_ki_num);
	jo_int(jo, "pi_ki_den", cfg->pi_ki_den);
	jo_int(jo, "pi_imax", cfg->pi_imax);
	jo_int(jo, "pi_outlier_ppb", cfg->pi_outlier_ppb);
	jo_uint(jo, "pi_warmup_cycles", cfg->pi_warmup_cycles);
	jo_uint(jo, "sap_announce_interval_s", cfg->sap_announce_interval_s);
	jo_bool(jo, "sap_announce_enabled", cfg->sap_announce_enabled);

	aes67_config_unlock();
	jo_obj_end(jo);
}

static int get_config(struct webapi_request *req)
{
	build_config(&req->out);
	return 0;
}

static int patch_config(struct webapi_request *req)
{
	const char *json = req->body;
	size_t len = req->body_len;
	char str_tmp[AES67_DEVICE_NAME_MAX];
	int32_t i_tmp;
	bool b_tmp;
	bool sap_enable_changed = false;
	bool sap_enable_val = false;

	aes67_config_lock();
	struct aes67_device_config *cfg = aes67_config_get();

	if (json_find_str(json, len, "device_name",
			  str_tmp, sizeof(str_tmp)) > 0) {
		strncpy(cfg->device_name, str_tmp,
			AES67_DEVICE_NAME_MAX - 1);
	}

	if (json_find_str(json, len, "default_mcast_addr",
			  str_tmp, sizeof(str_tmp)) > 0) {
		strncpy(cfg->default_mcast_addr, str_tmp,
			sizeof(cfg->default_mcast_addr) - 1);
	}

	if (json_find_int(json, len, "default_port", &i_tmp)) {
		cfg->default_port = (uint16_t)i_tmp;
	}
	if (json_find_int(json, len, "default_channels", &i_tmp)) {
		cfg->default_channels = (uint8_t)i_tmp;
	}
	if (json_find_int(json, len, "default_bit_depth", &i_tmp)) {
		cfg->default_bit_depth = (uint8_t)i_tmp;
	}
	if (json_find_int(json, len, "default_sample_rate", &i_tmp)) {
		cfg->default_sample_rate = (uint32_t)i_tmp;
	}
	if (json_find_int(json, len, "default_samples_per_pkt", &i_tmp)) {
		cfg->default_samples_per_pkt = (uint16_t)i_tmp;
	}
	if (json_find_int(json, len, "default_payload_type", &i_tmp)) {
		cfg->default_payload_type = (uint8_t)i_tmp;
	}
	if (json_find_int(json, len, "ptp_domain", &i_tmp)) {
		cfg->ptp_domain = (uint8_t)i_tmp;
	}
	if (json_find_int(json, len, "ptp_priority1", &i_tmp)) {
		cfg->ptp_priority1 = (uint8_t)i_tmp;
	}
	if (json_find_int(json, len, "ptp_priority2", &i_tmp)) {
		cfg->ptp_priority2 = (uint8_t)i_tmp;
	}
	if (json_find_int(json, len, "ptp_clock_class", &i_tmp)) {
		cfg->ptp_clock_class = (uint8_t)i_tmp;
	}
	if (json_find_int(json, len, "ptp_clock_accuracy", &i_tmp)) {
		cfg->ptp_clock_accuracy = (uint8_t)i_tmp;
	}
	if (json_find_int(json, len, "ptp_log_sync_interval", &i_tmp)) {
		cfg->ptp_log_sync_interval = (int8_t)i_tmp;
	}
	if (json_find_int(json, len, "ptp_log_announce_interval", &i_tmp)) {
		cfg->ptp_log_announce_interval = (int8_t)i_tmp;
	}
	if (json_find_int(json, len, "pi_kp_num", &i_tmp)) {
		cfg->pi_kp_num = i_tmp;
	}
	if (json_find_int(json, len, "pi_kp_den", &i_tmp)) {
		cfg->pi_kp_den = i_tmp;
	}
	if (json_find_int(json, len, "pi_ki_num", &i_tmp)) {
		cfg->pi_ki_num = i_tmp;
	}
	if (json_find_int(json, len, "pi_ki_den", &i_tmp)) {
		cfg->pi_ki_den = i_tmp;
	}
	if (json_find_int(json, len, "pi_imax", &i_tmp)) {
		cfg->pi_imax = i_tmp;
	}
	if (json_find_int(json, len, "pi_outlier_ppb", &i_tmp)) {
		cfg->pi_outlier_ppb = i_tmp;
	}
	if (json_find_int(json, len, "pi_warmup_cycles", &i_tmp)) {
		cfg->pi_warmup_cycles = (uint32_t)i_tmp;
	}
	if (json_find_int(json, len, "sap_announce_interval_s", &i_tmp)) {
		cfg->sap_announce_interval_s = (uint32_t)i_tmp;
	}
	if (json_find_bool(json, len, "sap_announce_enabled", &b_tmp)) {
		cfg->sap_announce_enabled = b_tmp;
		sap_enable_changed = true;
		sap_enable_val = b_tmp;
	}

	aes67_config_unlock();

	if (sap_enable_changed) {
		sap_sdp_set_announce(sap_enable_val);
	}

	/* Propagate PTP config changes into the active PTP stack
	 * (FPGA hardware BMC or Zephyr software PTP). */
	ptp_ctrl_apply_config();

	webapi_persist_config();

	LOG_INF("WEB: Configuration updated via REST API");

	build_config(&req->out);
	return 0;
}

static const struct webapi_route routes[] = {
	WEBAPI_ROUTE(HTTP_GET,   "/api/config", get_config),
	WEBAPI_ROUTE(HTTP_PATCH, "/api/config", patch_config),
};

const struct webapi_module webapi_config_module = {
	.routes = routes,
	.count = ARRAY_SIZE(routes),
};
