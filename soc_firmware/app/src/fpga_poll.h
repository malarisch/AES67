#ifndef FPGA_POLL_H
#define FPGA_POLL_H

#include <zephyr/net/net_if.h>

#include "ptp_bmc.h"

/** Callback type for requesting a DHCP restart from main. */
typedef void (*fpga_poll_dhcp_restart_fn)(void);

/**
 * @brief Start the FPGA status polling thread.
 *
 * The thread polls FPGA status registers every 100 ms, manages
 * PPB measurements, drives the PLL PI controller, tracks link
 * state, and updates display metrics.
 *
 * @param dhcp_restart_cb  Callback to restart DHCP on link recovery
 */
void fpga_poll_start(fpga_poll_dhcp_restart_fn dhcp_restart_cb);

/**
 * @brief Notify the poll thread that IP is valid (for link recovery logic).
 *
 * @param ip  The assigned IP address (stored for status display)
 */
void fpga_poll_notify_ip_valid(const struct in_addr *ip);

/**
 * @brief Register a PTP role-change callback (software-PTP mode).
 *
 * When the gateware runs PTP in software (system_cfg → runtime dispatch)
 * the FPGA-BMC module (ptp_bmc.c) is not running, so nobody fires the
 * role-change callback that drives the front-panel role LEDs and status
 * cycle. In that mode the poll thread samples the Zephyr stack's port
 * state once per second and invokes @p cb on every change — same contract
 * as ptp_bmc_register_change_cb(). Never fired in hardware-PTP mode.
 */
void fpga_poll_register_role_change_cb(ptp_bmc_change_cb_t cb);

/**
 * @brief Current Ethernet link state as sampled by the poll thread.
 *
 * Used to gate display repaints: while the link is down the panel shows
 * "  LINK" and PTP role changes must not overwrite it. Reads as "up"
 * until the first status sample so early-boot updates are never
 * suppressed.
 */
bool fpga_poll_link_is_up(void);

#endif /* FPGA_POLL_H */
