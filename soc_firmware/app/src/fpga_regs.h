#ifndef FPGA_REGS_H
#define FPGA_REGS_H

#include <zephyr/device.h>
#include <zephyr/net/net_if.h>
#include <stdint.h>

/**
 * @brief Write the interface MAC address to the FPGA via FMC register 0x40.
 *
 * @param iface  Network interface whose MAC to send
 * @return 0 on success, negative errno on error
 */
int fpga_write_mac_address(struct net_if *iface);

/**
 * @brief Write an IPv4 address to the FPGA via FMC register 0x41.
 *
 * @param addr  IPv4 address in network byte order (4 bytes)
 * @return 0 on success, negative errno on error
 */
int fpga_write_ip_address(const struct in_addr *addr);

/**
 * @brief Wait for Ethernet link-up by polling FPGA register 0x51.
 *
 * @param timeout_ms  Maximum time to wait (0 = no timeout)
 * @return 0 on link-up, -ETIMEDOUT on timeout
 */
int fpga_wait_for_link_up(uint32_t timeout_ms);

/**
 * @brief Read a 4-byte (32-bit) value from a sequential FPGA register.
 *
 * @param fmc   The FMC device
 * @param reg   FPGA register address
 * @param val   Pointer to store the 32-bit result
 * @return 0 on success, negative errno on error
 */
int fpga_read_32(const struct device *fmc, uint8_t reg, int32_t *val);

/**
 * @brief Calculate PPB from raw edge counter values.
 *
 * @param count_wc   Wallclock edge count
 * @param count_pll  PLL edge count
 * @return PPB difference, or 0 if count_wc is zero
 */
int32_t fpga_calculate_ppb(uint32_t count_wc, uint32_t count_pll);

/**
 * @brief Read PPB meter values and calculate PPB.
 *
 * @param fmc           FMC device
 * @param ppb_out       Output: calculated PPB value
 * @param count_wc_out  Optional output: raw wallclock count (can be NULL)
 * @param count_pll_out Optional output: raw PLL count (can be NULL)
 * @return 0 on success, negative errno on error
 */
int fpga_read_ppb(const struct device *fmc, int32_t *ppb_out,
		  uint32_t *count_wc_out, uint32_t *count_pll_out);

#endif /* FPGA_REGS_H */
