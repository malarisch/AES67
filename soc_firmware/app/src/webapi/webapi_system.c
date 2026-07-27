/*
 *
 * /api/summary                — one-request dashboard aggregate
 * /api/system                 — system / persistence status
 * /api/system/reboot          — reboot the MCU (POST)
 * /api/system/storage/format  — format the SD card (POST)
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/reboot.h>

#include "webapi_priv.h"
#ifdef CONFIG_SD_CONFIG
#include "../sd_config.h"
#endif
#ifdef CONFIG_FLASH_CONFIG
#include "../flash_config.h"
#endif

LOG_MODULE_DECLARE(webapi);

/* ---------------- /api/summary ---------------- */

static int get_summary(struct webapi_request *req)
{
	struct json_out *jo = &req->out;

	jo_obj_begin(jo);

	jo_key(jo, "network");
	webapi_build_network(jo);

	jo_key(jo, "ptp");
	webapi_build_ptp(jo);

	jo_key(jo, "fpga");
	webapi_build_fpga(jo);

	jo_obj_end(jo);
	return 0;
}

/* ---------------- /api/system ---------------- */

static int get_system(struct webapi_request *req)
{
	struct json_out *jo = &req->out;

	jo_obj_begin(jo);
#ifdef CONFIG_SD_CONFIG
	jo_bool(jo, "sd_mounted", sd_config_is_ready());
	jo_str(jo, "sd_config_status",
	       sd_config_status_str(sd_config_get_load_status()));
#else
	jo_bool(jo, "sd_mounted", false);
	jo_str(jo, "sd_config_status", "disabled");
#endif
#ifdef CONFIG_FLASH_CONFIG
	jo_str(jo, "flash_config_status",
	       flash_config_status_str(flash_config_get_load_status()));
#else
	jo_str(jo, "flash_config_status", "disabled");
#endif
	jo_str(jo, "version", "1.0.0");
	jo_uint(jo, "uptime_s", (uint32_t)(k_uptime_get() / 1000));
	jo_obj_end(jo);
	return 0;
}

/* ---------------- /api/system/reboot ---------------- */

/* The old handler slept inside the HTTP callback and rebooted before the
 * server ever sent the response. Defer the reboot instead so the client
 * gets its 200 and the TCP segment leaves the box. */
static void reboot_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);
	sys_reboot(SYS_REBOOT_COLD);
}

static K_WORK_DELAYABLE_DEFINE(reboot_work, reboot_work_fn);

static int post_reboot(struct webapi_request *req)
{
	LOG_WRN("Reboot requested via REST API");

	jo_obj_begin(&req->out);
	jo_bool(&req->out, "ok", true);
	jo_str(&req->out, "message", "Rebooting...");
	jo_obj_end(&req->out);

	k_work_schedule(&reboot_work, K_MSEC(500));
	return 0;
}

/* ---------------- /api/system/storage/format ---------------- */

static int post_storage_format(struct webapi_request *req)
{
#ifdef CONFIG_SD_CONFIG
	int ret = sd_config_format();

	if (ret < 0) {
		return ret;
	}

	jo_obj_begin(&req->out);
	jo_bool(&req->out, "ok", true);
	jo_str(&req->out, "message", "SD card formatted");
	jo_obj_end(&req->out);
	return 0;
#else
	ARG_UNUSED(req);
	return -ENOTSUP;
#endif
}

static const struct webapi_route routes[] = {
	WEBAPI_ROUTE(HTTP_GET,  "/api/summary",               get_summary),
	WEBAPI_ROUTE(HTTP_GET,  "/api/system",                get_system),
	WEBAPI_ROUTE(HTTP_POST, "/api/system/reboot",         post_reboot),
	WEBAPI_ROUTE(HTTP_POST, "/api/system/storage/format", post_storage_format),
};

const struct webapi_module webapi_system_module = {
	.routes = routes,
	.count = ARRAY_SIZE(routes),
};
