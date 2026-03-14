#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>

#include "../drivers/si5351a/si5351a.h"
#include "aes67_config.h"
#include "pll_ctrl.h"

LOG_MODULE_REGISTER(pll_ctrl, LOG_LEVEL_INF);

/* Base frequency for CLK0 — must match initial si5351a_set_frequency() call */
#define SI_CLK0_BASE_FREQ_HZ 24576000U

/* PI controller state */
static struct {
	int64_t  integrator;  /* Accumulated integral term (ppb, scaled by KI_DEN) */
	int32_t  output;      /* Current total correction applied (ppb) */
	uint32_t cycle;       /* Total accepted measurement count */
	uint32_t outliers;    /* Rejected outlier count */
} pi_state;

void pll_ctrl_reset(void)
{
	pi_state.integrator = 0;
	pi_state.output     = 0;
	pi_state.cycle      = 0;
	pi_state.outliers   = 0;
}

struct pll_ctrl_state pll_ctrl_get_state(void)
{
	return (struct pll_ctrl_state){
		.output   = pi_state.output,
		.cycle    = pi_state.cycle,
		.outliers = pi_state.outliers,
	};
}

int pll_ctrl_update(int32_t ppb_measured, uint32_t count_wc, uint32_t count_pll)
{
	const struct device *clkgen = DEVICE_DT_GET(DT_NODELABEL(si5351a));

	if (!device_is_ready(clkgen)) {
		LOG_ERR("Si5351A device not ready");
		return -ENODEV;
	}

	LOG_DBG("PPB raw: count_wc=%u count_pll=%u ppb=%d",
		count_wc, count_pll, ppb_measured);

	/* Outlier rejection (after warm-up) */
	const struct aes67_device_config *cfg = aes67_config_get();

	if (pi_state.cycle >= cfg->pi_warmup_cycles) {
		int32_t abs_meas = (ppb_measured < 0) ? -ppb_measured : ppb_measured;

		if (abs_meas > cfg->pi_outlier_ppb) {
			pi_state.outliers++;
			LOG_WRN("PPB outlier rejected: %d (wc=%u pll=%u) "
				"(outliers=%u cycle=%u)",
				ppb_measured, count_wc, count_pll,
				pi_state.outliers, pi_state.cycle);
			return 1;
		}
	}

	pi_state.cycle++;

	/*
	 * PI controller:
	 *
	 * error = ppb_measured  (positive = PLL fast -> need to slow down)
	 *
	 * P term:  correction_p = Kp * error
	 * I term:  integrator += error;  correction_i = Ki * integrator
	 * Output:  output -= (correction_p + correction_i)
	 */
	int32_t error = ppb_measured;

	int32_t kp_num = cfg->pi_kp_num;
	int32_t kp_den = cfg->pi_kp_den;
	int32_t ki_num = cfg->pi_ki_num;
	int32_t ki_den = cfg->pi_ki_den;
	int32_t imax   = cfg->pi_imax;

	/* Proportional term */
	int32_t p_term = (int32_t)(((int64_t)error * kp_num) / kp_den);

	/* Integral term with anti-windup */
	pi_state.integrator += error;
	if (pi_state.integrator > (int64_t)imax * ki_den) {
		pi_state.integrator = (int64_t)imax * ki_den;
	} else if (pi_state.integrator < -(int64_t)imax * ki_den) {
		pi_state.integrator = -(int64_t)imax * ki_den;
	}
	int32_t i_term = (int32_t)(pi_state.integrator * ki_num / ki_den);

	/* Apply combined correction (subtract because positive error = too fast) */
	pi_state.output -= (p_term + i_term);

	int ret = si5351a_adjust_ppb(clkgen, 0, SI_CLK0_BASE_FREQ_HZ,
				     pi_state.output);
	if (ret < 0) {
		LOG_ERR("Si5351A adjust failed: %d", ret);
		return ret;
	}

	return 0;
}
