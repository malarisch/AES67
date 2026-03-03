#ifndef FPGA_POLL_H
#define FPGA_POLL_H

#include <zephyr/net/net_if.h>

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
 */
void fpga_poll_notify_ip_valid(void);

#endif /* FPGA_POLL_H */
