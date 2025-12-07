/*
 * Basic SPI Ethernet Driver Header
 */

#ifndef ZEPHYR_DRIVERS_ETHERNET_ETH_SPI_BASIC_H_
#define ZEPHYR_DRIVERS_ETHERNET_ETH_SPI_BASIC_H_

#include <zephyr/types.h>

/* Konfigurierbare Parameter */
#ifndef CONFIG_ETH_SPI_BASIC_RX_STACK_SIZE
#define CONFIG_ETH_SPI_BASIC_RX_STACK_SIZE 1024
#endif

#ifndef CONFIG_ETH_SPI_BASIC_TIMEOUT
#define CONFIG_ETH_SPI_BASIC_TIMEOUT 200
#endif

/* Chip-spezifische Definitionen */
#define ETH_SPI_BASIC_MAX_PKT_SIZE  1522

/* PTP Timestamp Structure */
struct eth_spi_basic_timestamp {
	uint64_t ns;      /* Nanoseconds */
	bool valid;       /* Timestamp valid flag */
};

/* PTP Timestamp API */
/**
 * @brief Get TX packet timestamp
 * 
 * @param dev Pointer to the device structure
 * @param ts Pointer to timestamp structure to fill
 * @return 0 on success, negative error code on failure
 */
int eth_spi_basic_get_tx_timestamp(const struct device *dev, 
                                    struct eth_spi_basic_timestamp *ts);

/**
 * @brief Get RX packet timestamp
 * 
 * @param dev Pointer to the device structure
 * @param ts Pointer to timestamp structure to fill
 * @return 0 on success, negative error code on failure
 */
int eth_spi_basic_get_rx_timestamp(const struct device *dev, 
                                    struct eth_spi_basic_timestamp *ts);

/**
 * @brief Acknowledge TX timestamp (clear valid flag)
 * 
 * @param dev Pointer to the device structure
 * @return 0 on success, negative error code on failure
 */
int eth_spi_basic_ack_tx_timestamp(const struct device *dev);

/**
 * @brief Acknowledge RX timestamp (clear valid flag)
 * 
 * @param dev Pointer to the device structure
 * @return 0 on success, negative error code on failure
 */
int eth_spi_basic_ack_rx_timestamp(const struct device *dev);

#endif /* ZEPHYR_DRIVERS_ETHERNET_ETH_SPI_BASIC_H_ */
