#ifndef PLL_CTRL_H
#define PLL_CTRL_H

#include <stdint.h>

/** Snapshot of PI controller state for display/diagnostics. */
struct pll_ctrl_state {
	int32_t  output;    /* Current total correction applied (ppb) */
	uint32_t cycle;     /* Total accepted measurement count */
	uint32_t outliers;  /* Rejected outlier count */
};

/**
 * @brief Reset the PI controller state.
 *
 * Called when PTP state changes (e.g. BMC role or leader change)
 * to restart convergence from scratch.
 */
void pll_ctrl_reset(void);

/**
 * @brief Get a snapshot of the current PI controller state.
 */
struct pll_ctrl_state pll_ctrl_get_state(void);

/**
 * @brief Run one PI controller iteration.
 *
 * Applies outlier rejection, updates the integrator, and adjusts
 * the Si5351A clock generator frequency.
 *
 * @param ppb_measured  Measured PPB offset (positive = PLL fast)
 * @param count_wc      Raw wallclock counter (for logging)
 * @param count_pll     Raw PLL counter (for logging)
 * @return 0 if sample was accepted, 1 if rejected as outlier, negative on error
 */
int pll_ctrl_update(int32_t ppb_measured, uint32_t count_wc, uint32_t count_pll);

#endif /* PLL_CTRL_H */
