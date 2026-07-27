/*
 *
 * /api/fpga — FPGA status metrics (clock discipline, link, RX health).
 */

#include <zephyr/kernel.h>

#include "webapi_priv.h"
#include "../ui_display.h"
#include "../ieee1588_utils.h"
#include "../ptp_ctrl.h"
#include "../../drivers/fpga_hal/fpga_hal.h"

int webapi_read_fpga_metrics(struct ui_fpga_metrics *m)
{
	uint32_t status = fpga_hal_read_status();

	m->ppb_valid        = !!(status & FPGA_HAL_CLK_PPB_VALID);
	m->wc_locked        = !!(status & FPGA_HAL_CLK_WC_LOCKED);
	m->wc_phasejump     = !!(status & FPGA_HAL_CLK_WC_PHASEJUMP);
	m->wc_configured    = !!(status & FPGA_HAL_CLK_WC_CONFIGURED);
	m->ptp_leader_lost  = !!(status & FPGA_HAL_CLK_PTP_LEADER_LOST);

	m->link_up    = !!(status & FPGA_HAL_ETH_LINK_UP);
	m->speed_code = (status & FPGA_HAL_ETH_SPEED_MASK) >>
			FPGA_HAL_ETH_SPEED_SHIFT;

	m->rx_underrun = (status & FPGA_HAL_RX_UNDERRUN_MASK) >>
			 FPGA_HAL_RX_UNDERRUN_SHIFT;
	m->rx_mute     = (status & FPGA_HAL_RX_MUTE_MASK) >>
			 FPGA_HAL_RX_MUTE_SHIFT;

	m->path_delay_ns = fpga_hal_read_path_delay();
	m->leader_offset_ns = fpga_hal_read_ptp_offset();

	/* With PTP in software (runtime-detected from the gateware's
	 * system_cfg) the FPGA servo readouts (lock/offset/path delay CSRs)
	 * read constant 0; source these from the Zephyr PTP stack instead so
	 * the dashboard (summary, FPGA tab, offset chart) shows the same
	 * picture in both PTP modes. */
	if (fpga_hal_ptp_in_software()) {
		struct ptp_ctrl_status pst;

		ptp_ctrl_get_status(&pst);
		m->wc_locked        = pst.locked;
		m->path_delay_ns    = pst.path_delay_ns;
		m->leader_offset_ns = pst.offset_ns;
		m->ptp_leader_lost  = false;
	}

	/* Read raw counters and calculate PPB */
	uint32_t count_wc = 0, count_pll = 0;

	fpga_hal_read_ppb_counts(&count_wc, &count_pll);

	if (count_wc > 0) {
		int32_t diff = (int32_t)count_pll - (int32_t)count_wc;

		m->ppb_offset = (int32_t)(((int64_t)diff * 1000000000LL) /
					  (int64_t)count_wc);
	} else {
		m->ppb_offset = 0;
	}

	return 0;
}

void webapi_build_fpga(struct json_out *jo)
{
	struct ui_fpga_metrics m = {0};

	jo_obj_begin(jo);

	if (webapi_read_fpga_metrics(&m) == 0) {
		jo_bool(jo, "ppb_valid", m.ppb_valid);
		jo_bool(jo, "wc_locked", m.wc_locked);
		jo_bool(jo, "wc_phasejump", m.wc_phasejump);
		jo_bool(jo, "wc_configured", m.wc_configured);
		jo_bool(jo, "ptp_leader_lost", m.ptp_leader_lost);
		jo_bool(jo, "link_up", m.link_up);
		jo_int(jo, "path_delay_ns", m.path_delay_ns);
		jo_int(jo, "leader_offset_ns", m.leader_offset_ns);
		jo_int(jo, "ppb_offset", m.ppb_offset);
		jo_str(jo, "phy_speed", eth_speed_to_text(m.speed_code));
		jo_int(jo, "rx_underrun", m.rx_underrun);
		jo_int(jo, "rx_mute", m.rx_mute);
	} else {
		jo_str(jo, "error", "FPGA not available");
	}

	jo_obj_end(jo);
}

static int get_fpga(struct webapi_request *req)
{
	webapi_build_fpga(&req->out);
	return 0;
}

static const struct webapi_route routes[] = {
	WEBAPI_ROUTE(HTTP_GET, "/api/fpga", get_fpga),
};

const struct webapi_module webapi_fpga_module = {
	.routes = routes,
	.count = ARRAY_SIZE(routes),
};
