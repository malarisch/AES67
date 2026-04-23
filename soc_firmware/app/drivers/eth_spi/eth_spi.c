/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * Ethernet driver carrying frames over the on-FPGA `spictrl` block.
 *
 * Register map (see FPGA/spictrl.vhd + config_ram_address_map.md):
 *   0x20  W  [len_hi, len_lo, data…]  — transmit frame
 *   0x21  R  [len_hi, len_lo]          — length of the pending RX frame
 *   0x22  R  data…                     — pending RX frame payload
 *   0x50  R  clocking status (bit2 = rx available, bit1 = rx overflow)
 *   0x50  W  flag byte (bit3 resets the FPGA Ethernet block)
 *
 * An external GPIO line (`int-gpios` on the fpga_spi DT node) is driven
 * by the FPGA's `mcu_irq_o` output (= `packet_available`). A level-high
 * IRQ wakes the RX thread, which drains the frame, then clears the
 * interrupt by doing the read itself.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include "eth_spi.h"
#include "../fpga_spi/fpga_spi.h"
#include "../fpga_hal/fpga_hal.h"

LOG_MODULE_REGISTER(eth_spi, CONFIG_ETH_SPI_LOG_LEVEL);

#define FPGA_SPI_NODE DT_CHOSEN(zephyr_fpga_spi)

#define ETH_SPI_HAS_IRQ_GPIO DT_NODE_HAS_PROP(FPGA_SPI_NODE, int_gpios)

struct eth_spi_config {
#if ETH_SPI_HAS_IRQ_GPIO
	struct gpio_dt_spec irq_gpio;
#endif
};

struct eth_spi_data {
	struct net_if *iface;
	const struct device *dev;
	uint8_t mac_addr[6];

	struct k_sem rx_sem;
#if ETH_SPI_HAS_IRQ_GPIO
	struct gpio_callback irq_cb;
#endif

	struct k_thread rx_thread;
	struct k_thread tx_thread;
	struct k_work_delayable link_work;

	bool link_up;

	uint8_t rx_buf[FPGA_SPI_ETH_MAX_PKT];
	uint8_t tx_buf[FPGA_SPI_ETH_MAX_PKT];

	K_KERNEL_STACK_MEMBER(rx_stack, CONFIG_ETH_SPI_RX_STACK_SIZE);
	K_KERNEL_STACK_MEMBER(tx_stack, CONFIG_ETH_SPI_RX_STACK_SIZE);
};

K_MSGQ_DEFINE(eth_spi_tx_queue, sizeof(struct net_pkt *),
	      CONFIG_ETH_SPI_TX_QUEUE_DEPTH, 4);

static struct eth_spi_data eth_spi_data_0;

static const struct eth_spi_config eth_spi_cfg_0 = {
#if ETH_SPI_HAS_IRQ_GPIO
	.irq_gpio = GPIO_DT_SPEC_GET(FPGA_SPI_NODE, int_gpios),
#endif
};

/* ---- IRQ / polling source ---- */

#if ETH_SPI_HAS_IRQ_GPIO
static void eth_spi_irq_handler(const struct device *port,
				struct gpio_callback *cb, uint32_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(pins);

	struct eth_spi_data *data = CONTAINER_OF(cb, struct eth_spi_data, irq_cb);
	k_sem_give(&data->rx_sem);
}
#endif

/* ---- RX path ---- */

/* Hard cap per wake-up. The FPGA IRQ line is level-based, so multiple
 * frames may be waiting, but we must not monopolise the CPU. After the
 * cap is hit, the thread yields and re-enters through the semaphore. */
#define ETH_SPI_RX_MAX_PER_WAKE 8

static void eth_spi_drain_rx(struct eth_spi_data *data)
{
	const struct device *bus = fpga_spi_get_dev();

	for (int i = 0; i < ETH_SPI_RX_MAX_PER_WAKE; i++) {
		int len = fpga_spi_eth_rx_len(bus);

		if (len < 0) {
			LOG_WRN("RX len read failed: %d", len);
			return;
		}
		if (len == 0) {
			return;
		}
		if (len < 14 || len > FPGA_SPI_ETH_MAX_PKT) {
			LOG_WRN("RX invalid len %d — dropping", len);
			/* Clear the frame by reading one byte so the FPGA drops
			 * packet_available, then leave. Better than getting stuck
			 * if the FPGA reports a garbage length. */
			uint8_t scratch;
			(void)fpga_spi_eth_rx_read(bus, &scratch, 1);
			return;
		}

		if (fpga_spi_eth_rx_read(bus, data->rx_buf, len) < 0) {
			return;
		}

		struct net_pkt *pkt = net_pkt_rx_alloc_with_buffer(
			data->iface, len, AF_UNSPEC, 0, K_NO_WAIT);
		if (!pkt) {
			LOG_ERR("RX net_pkt alloc failed (len=%d)", len);
			continue;
		}
		if (net_pkt_write(pkt, data->rx_buf, len) < 0) {
			LOG_ERR("RX net_pkt_write failed");
			net_pkt_unref(pkt);
			continue;
		}
		int ret = net_recv_data(data->iface, pkt);
		if (ret < 0) {
			LOG_ERR("RX deliver err %d", ret);
			net_pkt_unref(pkt);
		}
	}
}

static void eth_spi_rx_thread(void *p1, void *p2, void *p3)
{
	const struct device *dev = p1;
	struct eth_spi_data *data = dev->data;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		k_sem_take(&data->rx_sem,
			   K_MSEC(CONFIG_ETH_SPI_POLL_INTERVAL_MS));
		eth_spi_drain_rx(data);
		k_yield();
	}
}

/* ---- TX path ---- */

static void eth_spi_tx_thread(void *p1, void *p2, void *p3)
{
	const struct device *dev = p1;
	struct eth_spi_data *data = dev->data;
	const struct device *bus = fpga_spi_get_dev();
	struct net_pkt *pkt = NULL;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		k_msgq_get(&eth_spi_tx_queue, &pkt, K_FOREVER);

		size_t len = net_pkt_get_len(pkt);
		if (len == 0 || len > FPGA_SPI_ETH_MAX_PKT) {
			LOG_ERR("TX invalid len %zu", len);
			net_pkt_unref(pkt);
			continue;
		}
		if (net_pkt_read(pkt, data->tx_buf, len) < 0) {
			LOG_ERR("TX net_pkt_read failed");
			net_pkt_unref(pkt);
			continue;
		}

		int ret = fpga_spi_eth_tx_write(bus, data->tx_buf, len);
		if (ret < 0) {
			LOG_ERR("TX write failed: %d (len=%zu)", ret, len);
		} else {
			LOG_DBG("TX %zu bytes", len);
		}
		net_pkt_unref(pkt);
	}
}

static int eth_spi_send(const struct device *dev, struct net_pkt *pkt)
{
	ARG_UNUSED(dev);

	/* Keep the packet alive across the queue hand-off — see eth_litex for
	 * the reasoning. */
	net_pkt_ref(pkt);
	if (k_msgq_put(&eth_spi_tx_queue, &pkt, K_NO_WAIT) != 0) {
		net_pkt_unref(pkt);
		LOG_WRN("TX queue full, dropping (%zu bytes)", net_pkt_get_len(pkt));
		return -ENOMEM;
	}
	return 0;
}

/* ---- Link supervision ---- */

static void eth_spi_link_work(struct k_work *work)
{
	struct k_work_delayable *dwork = CONTAINER_OF(work, struct k_work_delayable, work);
	struct eth_spi_data *data = CONTAINER_OF(dwork, struct eth_spi_data, link_work);

	if (!data->iface) {
		return;
	}

	uint32_t st = fpga_hal_read_status();
	bool up = (st & FPGA_HAL_ETH_LINK_UP) != 0;

	if (up && !data->link_up) {
		data->link_up = true;
		net_if_carrier_on(data->iface);
		LOG_INF("Ethernet link up");
	} else if (!up && data->link_up) {
		data->link_up = false;
		net_if_carrier_off(data->iface);
		LOG_INF("Ethernet link down");
	}
	k_work_schedule(dwork, K_MSEC(500));
}

/* ---- Zephyr ethernet API ---- */

static void eth_spi_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct eth_spi_data *data = dev->data;

	data->iface = iface;
	ethernet_init(iface);

	/* Static locally-administered MAC. The FPGA learns it via the
	 * HAL-level `write_mac` call once the net stack is up. */
	data->mac_addr[0] = 0x02;
	data->mac_addr[1] = 0xAA;
	data->mac_addr[2] = 0xE6;
	data->mac_addr[3] = 0x70;
	data->mac_addr[4] = 0x00;
	data->mac_addr[5] = 0x01;

	net_if_set_link_addr(iface, data->mac_addr, sizeof(data->mac_addr),
			     NET_LINK_ETHERNET);

	(void)fpga_hal_write_mac(data->mac_addr);

	LOG_INF("MAC %02x:%02x:%02x:%02x:%02x:%02x",
		data->mac_addr[0], data->mac_addr[1], data->mac_addr[2],
		data->mac_addr[3], data->mac_addr[4], data->mac_addr[5]);
}

static enum ethernet_hw_caps eth_spi_get_capabilities(const struct device *dev)
{
	ARG_UNUSED(dev);
	return ETHERNET_LINK_100BASE;
}

static const struct ethernet_api eth_spi_api = {
	.iface_api.init   = eth_spi_iface_init,
	.get_capabilities = eth_spi_get_capabilities,
	.send             = eth_spi_send,
};

/* ---- Init ---- */

static int eth_spi_init(const struct device *dev)
{
	struct eth_spi_data *data = dev->data;
	const struct eth_spi_config *cfg = dev->config;

	data->dev = dev;
	k_sem_init(&data->rx_sem, 0, 1);

	if (fpga_spi_get_dev() == NULL) {
		LOG_ERR("fpga_spi bus driver not available");
		return -ENODEV;
	}

#if ETH_SPI_HAS_IRQ_GPIO
	if (!gpio_is_ready_dt(&cfg->irq_gpio)) {
		LOG_ERR("IRQ GPIO %s not ready", cfg->irq_gpio.port->name);
		return -ENODEV;
	}
	gpio_pin_configure_dt(&cfg->irq_gpio, GPIO_INPUT);
	gpio_init_callback(&data->irq_cb, eth_spi_irq_handler,
			   BIT(cfg->irq_gpio.pin));
	gpio_add_callback(cfg->irq_gpio.port, &data->irq_cb);
	/* Level-active: the FPGA pin reflects packet_available. While the
	 * line is asserted we keep waking the RX thread; once it drains the
	 * frame and the FPGA drops the line, the IRQ stops firing. */
	gpio_pin_interrupt_configure_dt(&cfg->irq_gpio,
					(cfg->irq_gpio.dt_flags & GPIO_ACTIVE_LOW)
						? GPIO_INT_LEVEL_LOW
						: GPIO_INT_LEVEL_HIGH);
	LOG_INF("IRQ on %s pin %d", cfg->irq_gpio.port->name, cfg->irq_gpio.pin);
#else
	ARG_UNUSED(cfg);
	LOG_INF("no IRQ GPIO — falling back to %dms polling",
		CONFIG_ETH_SPI_POLL_INTERVAL_MS);
#endif

	k_work_init_delayable(&data->link_work, eth_spi_link_work);

	/* Preemptible priorities — the SPI transport can (rarely) stall on a
	 * hung bus, and we never want that to freeze the shell or the rest
	 * of the system. COOP would do exactly that. */
	k_thread_create(&data->rx_thread, data->rx_stack,
			CONFIG_ETH_SPI_RX_STACK_SIZE,
			eth_spi_rx_thread, (void *)dev, NULL, NULL,
			K_PRIO_PREEMPT(7), 0, K_NO_WAIT);
	k_thread_name_set(&data->rx_thread, "eth_spi_rx");

	k_thread_create(&data->tx_thread, data->tx_stack,
			CONFIG_ETH_SPI_RX_STACK_SIZE,
			eth_spi_tx_thread, (void *)dev, NULL, NULL,
			K_PRIO_PREEMPT(8), 0, K_NO_WAIT);
	k_thread_name_set(&data->tx_thread, "eth_spi_tx");

	k_work_schedule(&data->link_work, K_MSEC(100));

	LOG_INF("eth_spi init complete");
	return 0;
}

ETH_NET_DEVICE_INIT(eth_spi0, "eth_spi0",
		    eth_spi_init, NULL,
		    &eth_spi_data_0, &eth_spi_cfg_0,
		    CONFIG_ETH_SPI_INIT_PRIORITY, &eth_spi_api,
		    NET_ETH_MTU);

const struct device *eth_spi_get_dev(void)
{
	return DEVICE_GET(eth_spi0);
}
