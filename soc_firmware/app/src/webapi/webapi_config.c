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

/* PATCH /api/config body. The keys are exactly the device-config field
 * names; anything the request omits keeps its current value because the
 * struct is pre-filled from the live configuration before parsing. */
struct config_patch {
	char     device_name[AES67_DEVICE_NAME_MAX];
	char     default_mcast_addr[16];
	uint16_t default_port;
	uint8_t  default_channels;
	uint8_t  default_bit_depth;
	uint32_t default_sample_rate;
	uint16_t default_samples_per_pkt;
	uint8_t  default_payload_type;
	uint8_t  ptp_domain;
	uint8_t  ptp_priority1;
	uint8_t  ptp_priority2;
	uint8_t  ptp_clock_class;
	uint8_t  ptp_clock_accuracy;
	int8_t   ptp_log_sync_interval;
	int8_t   ptp_log_announce_interval;
	int32_t  pi_kp_num;
	int32_t  pi_kp_den;
	int32_t  pi_ki_num;
	int32_t  pi_ki_den;
	int32_t  pi_imax;
	int32_t  pi_outlier_ppb;
	uint32_t pi_warmup_cycles;
	uint32_t sap_announce_interval_s;
	bool     sap_announce_enabled;
};

enum { CFG_F_SAP_ENABLED = 23 };

static const struct json_obj_descr config_patch_descr[] = {
	JSON_OBJ_DESCR_PRIM(struct config_patch, device_name,
			    JSON_TOK_STRING_BUF),
	JSON_OBJ_DESCR_PRIM(struct config_patch, default_mcast_addr,
			    JSON_TOK_STRING_BUF),
	JSON_OBJ_DESCR_PRIM(struct config_patch, default_port, JSON_TOK_UINT),
	JSON_OBJ_DESCR_PRIM(struct config_patch, default_channels,
			    JSON_TOK_UINT),
	JSON_OBJ_DESCR_PRIM(struct config_patch, default_bit_depth,
			    JSON_TOK_UINT),
	JSON_OBJ_DESCR_PRIM(struct config_patch, default_sample_rate,
			    JSON_TOK_UINT),
	JSON_OBJ_DESCR_PRIM(struct config_patch, default_samples_per_pkt,
			    JSON_TOK_UINT),
	JSON_OBJ_DESCR_PRIM(struct config_patch, default_payload_type,
			    JSON_TOK_UINT),
	JSON_OBJ_DESCR_PRIM(struct config_patch, ptp_domain, JSON_TOK_UINT),
	JSON_OBJ_DESCR_PRIM(struct config_patch, ptp_priority1, JSON_TOK_UINT),
	JSON_OBJ_DESCR_PRIM(struct config_patch, ptp_priority2, JSON_TOK_UINT),
	JSON_OBJ_DESCR_PRIM(struct config_patch, ptp_clock_class,
			    JSON_TOK_UINT),
	JSON_OBJ_DESCR_PRIM(struct config_patch, ptp_clock_accuracy,
			    JSON_TOK_UINT),
	JSON_OBJ_DESCR_PRIM(struct config_patch, ptp_log_sync_interval,
			    JSON_TOK_INT),
	JSON_OBJ_DESCR_PRIM(struct config_patch, ptp_log_announce_interval,
			    JSON_TOK_INT),
	JSON_OBJ_DESCR_PRIM(struct config_patch, pi_kp_num, JSON_TOK_INT),
	JSON_OBJ_DESCR_PRIM(struct config_patch, pi_kp_den, JSON_TOK_INT),
	JSON_OBJ_DESCR_PRIM(struct config_patch, pi_ki_num, JSON_TOK_INT),
	JSON_OBJ_DESCR_PRIM(struct config_patch, pi_ki_den, JSON_TOK_INT),
	JSON_OBJ_DESCR_PRIM(struct config_patch, pi_imax, JSON_TOK_INT),
	JSON_OBJ_DESCR_PRIM(struct config_patch, pi_outlier_ppb,
			    JSON_TOK_INT),
	JSON_OBJ_DESCR_PRIM(struct config_patch, pi_warmup_cycles,
			    JSON_TOK_UINT),
	JSON_OBJ_DESCR_PRIM(struct config_patch, sap_announce_interval_s,
			    JSON_TOK_UINT),
	JSON_OBJ_DESCR_PRIM(struct config_patch, sap_announce_enabled,
			    JSON_TOK_TRUE),
};

BUILD_ASSERT(ARRAY_SIZE(config_patch_descr) == CFG_F_SAP_ENABLED + 1,
	     "CFG_F_SAP_ENABLED must index the last descriptor");

static int patch_config(struct webapi_request *req)
{
	struct config_patch p;
	bool sap_enable_changed;
	bool sap_enable_val;
	int64_t present;

	aes67_config_lock();
	struct aes67_device_config *cfg = aes67_config_get();

	strncpy(p.device_name, cfg->device_name, sizeof(p.device_name) - 1);
	p.device_name[sizeof(p.device_name) - 1] = '\0';
	strncpy(p.default_mcast_addr, cfg->default_mcast_addr,
		sizeof(p.default_mcast_addr) - 1);
	p.default_mcast_addr[sizeof(p.default_mcast_addr) - 1] = '\0';
	p.default_port            = cfg->default_port;
	p.default_channels        = cfg->default_channels;
	p.default_bit_depth       = cfg->default_bit_depth;
	p.default_sample_rate     = cfg->default_sample_rate;
	p.default_samples_per_pkt = cfg->default_samples_per_pkt;
	p.default_payload_type    = cfg->default_payload_type;
	p.ptp_domain              = cfg->ptp_domain;
	p.ptp_priority1           = cfg->ptp_priority1;
	p.ptp_priority2           = cfg->ptp_priority2;
	p.ptp_clock_class         = cfg->ptp_clock_class;
	p.ptp_clock_accuracy      = cfg->ptp_clock_accuracy;
	p.ptp_log_sync_interval   = cfg->ptp_log_sync_interval;
	p.ptp_log_announce_interval = cfg->ptp_log_announce_interval;
	p.pi_kp_num               = cfg->pi_kp_num;
	p.pi_kp_den               = cfg->pi_kp_den;
	p.pi_ki_num               = cfg->pi_ki_num;
	p.pi_ki_den               = cfg->pi_ki_den;
	p.pi_imax                 = cfg->pi_imax;
	p.pi_outlier_ppb          = cfg->pi_outlier_ppb;
	p.pi_warmup_cycles        = cfg->pi_warmup_cycles;
	p.sap_announce_interval_s = cfg->sap_announce_interval_s;
	p.sap_announce_enabled    = cfg->sap_announce_enabled;

	present = webapi_parse_body(req, config_patch_descr,
				    ARRAY_SIZE(config_patch_descr), &p);
	if (present < 0) {
		aes67_config_unlock();
		return -EINVAL;
	}

	strncpy(cfg->device_name, p.device_name, AES67_DEVICE_NAME_MAX - 1);
	cfg->device_name[AES67_DEVICE_NAME_MAX - 1] = '\0';
	strncpy(cfg->default_mcast_addr, p.default_mcast_addr,
		sizeof(cfg->default_mcast_addr) - 1);
	cfg->default_mcast_addr[sizeof(cfg->default_mcast_addr) - 1] = '\0';
	cfg->default_port            = p.default_port;
	cfg->default_channels        = p.default_channels;
	cfg->default_bit_depth       = p.default_bit_depth;
	cfg->default_sample_rate     = p.default_sample_rate;
	cfg->default_samples_per_pkt = p.default_samples_per_pkt;
	cfg->default_payload_type    = p.default_payload_type;
	cfg->ptp_domain              = p.ptp_domain;
	cfg->ptp_priority1           = p.ptp_priority1;
	cfg->ptp_priority2           = p.ptp_priority2;
	cfg->ptp_clock_class         = p.ptp_clock_class;
	cfg->ptp_clock_accuracy      = p.ptp_clock_accuracy;
	cfg->ptp_log_sync_interval   = p.ptp_log_sync_interval;
	cfg->ptp_log_announce_interval = p.ptp_log_announce_interval;
	cfg->pi_kp_num               = p.pi_kp_num;
	cfg->pi_kp_den               = p.pi_kp_den;
	cfg->pi_ki_num               = p.pi_ki_num;
	cfg->pi_ki_den               = p.pi_ki_den;
	cfg->pi_imax                 = p.pi_imax;
	cfg->pi_outlier_ppb          = p.pi_outlier_ppb;
	cfg->pi_warmup_cycles        = p.pi_warmup_cycles;
	cfg->sap_announce_interval_s = p.sap_announce_interval_s;
	cfg->sap_announce_enabled    = p.sap_announce_enabled;

	/* The announcer only needs poking when the flag actually appeared
	 * in the request. */
	sap_enable_changed = WEBAPI_HAS(present, CFG_F_SAP_ENABLED);
	sap_enable_val = p.sap_announce_enabled;

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
