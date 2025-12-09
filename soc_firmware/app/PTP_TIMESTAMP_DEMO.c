/*
 * PTP Timestamp Usage Example
 * 
 * This file demonstrates how to use the PTP timestamping API
 * provided by the eth_spi_basic driver.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/net/net_if.h>
#include <zephyr/logging/log.h>
#include <inttypes.h>
#include "../drivers/eth_spi_basic/eth_spi_basic.h"

LOG_MODULE_REGISTER(ptp_example, LOG_LEVEL_DBG);

/* Example: Read and display TX timestamp after sending a packet */
void example_read_tx_timestamp(const struct device *eth_dev)
{
    struct eth_spi_basic_timestamp tx_ts;
    int ret;
    
    /* Poll for TX timestamp availability */
    ret = eth_spi_basic_get_tx_timestamp(eth_dev, &tx_ts);
    if (ret != 0) {
        LOG_ERR("Failed to read TX timestamp: %d", ret);
        return;
    }
    
    if (tx_ts.valid) {
        LOG_INF("TX Timestamp: %" PRIu64 " ns", tx_ts.ns);
        
        /* Convert to seconds and nanoseconds for display */
        uint32_t sec = (uint32_t)(tx_ts.ns / 1000000000ULL);
        uint32_t nsec = (uint32_t)(tx_ts.ns % 1000000000ULL);
        LOG_INF("  = %u.%09u seconds", sec, nsec);
        
        /* Acknowledge the timestamp to allow capture of next TX */
        ret = eth_spi_basic_ack_tx_timestamp(eth_dev);
        if (ret != 0) {
            LOG_ERR("Failed to acknowledge TX timestamp: %d", ret);
        }
    } else {
        LOG_DBG("No valid TX timestamp available");
    }
}

/* Example: Read and display RX timestamp after receiving a packet */
void example_read_rx_timestamp(const struct device *eth_dev)
{
    struct eth_spi_basic_timestamp rx_ts;
    int ret;
    
    /* Poll for RX timestamp availability */
    ret = eth_spi_basic_get_rx_timestamp(eth_dev, &rx_ts);
    if (ret != 0) {
        LOG_ERR("Failed to read RX timestamp: %d", ret);
        return;
    }
    
    if (rx_ts.valid) {
        LOG_INF("RX Timestamp: %" PRIu64 " ns", rx_ts.ns);
        
        /* Convert to seconds and nanoseconds for display */
        uint32_t sec = (uint32_t)(rx_ts.ns / 1000000000ULL);
        uint32_t nsec = (uint32_t)(rx_ts.ns % 1000000000ULL);
        LOG_INF("  = %u.%09u seconds", sec, nsec);
        
        /* Acknowledge the timestamp to allow capture of next RX */
        ret = eth_spi_basic_ack_rx_timestamp(eth_dev);
        if (ret != 0) {
            LOG_ERR("Failed to acknowledge RX timestamp: %d", ret);
        }
    } else {
        LOG_DBG("No valid RX timestamp available");
    }
}

/* Example: Calculate packet latency using TX and RX timestamps */
void example_calculate_latency(const struct device *eth_dev)
{
    struct eth_spi_basic_timestamp tx_ts, rx_ts;
    int ret;
    
    /* Read TX timestamp */
    ret = eth_spi_basic_get_tx_timestamp(eth_dev, &tx_ts);
    if (ret != 0 || !tx_ts.valid) {
        LOG_WRN("No valid TX timestamp");
        return;
    }
    
    /* Read RX timestamp */
    ret = eth_spi_basic_get_rx_timestamp(eth_dev, &rx_ts);
    if (ret != 0 || !rx_ts.valid) {
        LOG_WRN("No valid RX timestamp");
        return;
    }
    
    /* Calculate latency (assuming RX came after TX) */
    if (rx_ts.ns > tx_ts.ns) {
        uint64_t latency_ns = rx_ts.ns - tx_ts.ns;
        uint32_t latency_us = (uint32_t)(latency_ns / 1000ULL);
        LOG_INF("Packet latency: %" PRIu64 " ns (%u us)", latency_ns, latency_us);
    } else {
        LOG_WRN("RX timestamp before TX timestamp (counter wrap?)");
    }
    
    /* Acknowledge both timestamps */
    eth_spi_basic_ack_tx_timestamp(eth_dev);
    eth_spi_basic_ack_rx_timestamp(eth_dev);
}

/* Example: Periodic timestamp monitoring */
void timestamp_monitor_thread(void *p1, void *p2, void *p3)
{
    const struct device *eth_dev = p1;
    
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);
    
    LOG_INF("Starting timestamp monitor thread");
    
    while (1) {
        /* Check for new TX timestamps */
        example_read_tx_timestamp(eth_dev);
        
        /* Check for new RX timestamps */
        example_read_rx_timestamp(eth_dev);
        
        /* Sleep for a short interval */
        k_sleep(K_MSEC(100));
    }
}

/* Example: How to get the Ethernet device for timestamp access */
const struct device *get_ethernet_device(void)
{
    struct net_if *iface = net_if_get_default();
    if (!iface) {
        LOG_ERR("No network interface found");
        return NULL;
    }
    
    const struct device *dev = net_if_get_device(iface);
    if (!dev) {
        LOG_ERR("No device found for network interface");
        return NULL;
    }
    
    return dev;
}

/* Example: Integration point in main() */
void ptp_timestamp_init(void)
{
    const struct device *eth_dev = get_ethernet_device();
    if (!eth_dev) {
        LOG_ERR("Cannot initialize PTP timestamps - no Ethernet device");
        return;
    }
    
    LOG_INF("PTP timestamp support initialized");
    LOG_INF("Ethernet device: %s", eth_dev->name);
    
    /* You can now:
     * 1. Call timestamp functions directly after TX/RX
     * 2. Start a monitoring thread (see timestamp_monitor_thread)
     * 3. Integrate with Zephyr PTP stack
     */
    
    /* Example: Read initial timestamp state */
    struct eth_spi_basic_timestamp ts;
    if (eth_spi_basic_get_tx_timestamp(eth_dev, &ts) == 0) {
        LOG_INF("TX timestamp valid: %s", ts.valid ? "yes" : "no");
    }
    if (eth_spi_basic_get_rx_timestamp(eth_dev, &ts) == 0) {
        LOG_INF("RX timestamp valid: %s", ts.valid ? "yes" : "no");
    }
}
