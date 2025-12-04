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

#endif /* ZEPHYR_DRIVERS_ETHERNET_ETH_SPI_BASIC_H_ */
