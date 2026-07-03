/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * Ethernet driver carrying frames over the spibone Wishbone bridge.
 *
 * External-MCU counterpart of eth_litex.c / the Linux aes67_eth driver: the
 * FPGA's litex_eth_buffer_bridge exposes dual-port RX/TX packet buffers plus
 * an EventManager, all reachable through the aes67_bridge register map:
 *
 *   ETH_BUF_RX_MEM / ETH_BUF_TX_MEM   packet buffers, 1 byte per 32-bit word
 *   eth_buf_rx_ready / rx_len / rx_ack  RX handshake
 *   eth_buf_tx_len + ctrl.eth_tx_request  TX handshake
 *   eth_buf_ev_*                      RX-ready event → FPGA `mcu_irq_o` pin
 *
 * Frames stream in one spibone burst per direction instead of a Wishbone
 * round-trip per byte.
 *
 * An external GPIO line (`int-gpios` on the chosen `zephyr,fpga-spi` node) is
 * driven by the FPGA's `mcu_irq_o` output (= eth_buf RX-ready event). A GPIO
 * edge wakes the RX thread, which drains the frame and acks it. Without the
 * line, a poll thread watches the rx_ready register instead.
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
#include "../spibone/spibone.h"
#include "../fpga_hal/fpga_hal.h"
/* Macro layer only (CSR_ETH_BUF_ / AES67_STATUS_ / AES67_CTRL_ bit fields and
 * the buffer base addresses); the eth_litex functions are not built here. */
#include "../eth_litex/eth_litex.h"

LOG_MODULE_REGISTER(eth_spi, CONFIG_ETH_SPI_LOG_LEVEL);

#define FPGA_SPI_NODE DT_CHOSEN(zephyr_fpga_spi)

#define ETH_SPI_HAS_IRQ_GPIO DT_NODE_HAS_PROP(FPGA_SPI_NODE, int_gpios)

/* Bounded eth_tx_done poll. Each status read is a full SPI transaction
 * (≥100 µs), while a 1518-byte frame needs ~122 µs on 100 Mbit wire — a
 * handful of iterations covers it; the cap only guards a wedged MAC. */
#define TX_DONE_POLL_LIMIT 64

/* Egress-timestamp latch poll (see eth_spi_tx_frame): bounds the wait for our
 * frame's capture to appear. Each iteration is two CSR reads (~60 µs at
 * 10 MHz); the frame itself needs µs on the wire plus MAC arbitration. */
#define TX_TS_POLL_LIMIT 100

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
#if !ETH_SPI_HAS_IRQ_GPIO
	struct k_thread poll_thread;
	K_KERNEL_STACK_MEMBER(poll_stack, 1024);
#endif
	struct k_work_delayable link_work;

	bool link_up;
	bool tx_started;   /* first eth_tx_request pulsed (tx_done valid) */

	/* RX holds the raw buffer contents: frame + 4-byte FCS (+ the 5-byte
	 * hardware-timestamp trailer in --ptp-in-software gateware). */
	uint8_t rx_buf[ETH_LITEX_MAX_PKT_SIZE + 4 + ETH_LITEX_RX_TRAILER_LEN];
	uint8_t tx_buf[ETH_LITEX_MAX_PKT_SIZE];

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

/* ---- IRQ source ----
 *
 * mcu_irq_o follows the eth_buf EventManager (rx_ready). We use an edge
 * trigger: the RX thread clears the pending event after draining, which
 * de-asserts the line; a queued second frame re-raises it (fresh edge).
 * The level is re-checked after each frame to close the no-edge gap.
 */

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

/* Drain one ready frame (if any). Mirrors the kernel driver's aes67_rx_one.
 * Returns true when a frame was taken out of the FPGA buffer (delivered or
 * dropped) — the caller keeps draining until this returns false. */
static bool eth_spi_rx_one(struct eth_spi_data *data)
{
	uint32_t ready, raw_len = 0;
	int len = 0;
	int ret;
#ifdef CONFIG_AES67_PTP_SOFTWARE
	struct net_ptp_time rx_ts;
	bool rx_have_ts = false;
#endif

	spibone_bus_lock();

	/* Clear the RX-ready event BEFORE looking at the buffer: any frame the
	 * FPGA queues after this write re-raises the event (fresh IRQ edge).
	 * Clearing it after the read would eat the notification for a frame
	 * that arrived during the (slow, SPI-paced) read — that frame then sat
	 * until the next unrelated packet or the poll timeout, which showed up
	 * as 100s-of-ms ping spikes under sustained PTP multicast load. */
	(void)spibone_write_locked(CSR_ETH_BUF_EV_PENDING_ADDR, ETH_BUF_EV_RX_READY);

	ret = spibone_read_locked(CSR_ETH_BUF_RX_READY_ADDR, &ready);
	if (ret < 0 || !(ready & 0x01)) {
		spibone_bus_unlock();
		return false;
	}

	ret = spibone_read_locked(CSR_ETH_BUF_RX_LEN_ADDR, &raw_len);
	if (ret == 0) {
		/* The MAC stores the frame including its 4-byte FCS (followed by
		 * the 5-byte timestamp trailer in --ptp-in-software gateware);
		 * the network stack expects a bare frame. */
		len = (int)(raw_len & 0xFFFF) - 4 - ETH_LITEX_RX_TRAILER_LEN;
		if (len < 14 || len > ETH_LITEX_MAX_PKT_SIZE) {
			LOG_WRN("RX invalid len %d (raw %u) — dropping", len, raw_len);
			len = 0;
		} else {
			/* In PTP mode read through the FCS to reach the trailer;
			 * otherwise stop at the frame end. */
			size_t rd_len = ETH_LITEX_RX_TRAILER_LEN != 0
					? (size_t)len + 4 + ETH_LITEX_RX_TRAILER_LEN
					: (size_t)len;

			ret = spibone_read_burst_locked(ETH_BUF_RX_MEM,
							data->rx_buf, rd_len);
			if (ret < 0) {
				LOG_WRN("RX burst read failed: %d (len=%d)", ret, len);
				len = 0;
			}
		}
	}

	/* Release the buffer for the next frame regardless. */
	(void)spibone_write_locked(CSR_ETH_BUF_RX_ACK_ADDR, 1);
	(void)spibone_write_locked(CSR_ETH_BUF_RX_ACK_ADDR, 0);

	spibone_bus_unlock();

	if (len <= 0) {
		return true; /* frame consumed (dropped) — keep draining */
	}

#ifdef CONFIG_AES67_PTP_SOFTWARE
	/* Trailer: [seconds(1)][nanoseconds(4, little-endian)] after the FCS.
	 * Reconstructed outside the bus lock — it reads the live wallclock
	 * seconds over the same bus. */
	{
		const uint8_t *tr = &data->rx_buf[len + 4];
		uint32_t ts_nsec = (uint32_t)tr[1] |
				   ((uint32_t)tr[2] << 8) |
				   ((uint32_t)tr[3] << 16) |
				   ((uint32_t)tr[4] << 24);

		aes67_ptp_reconstruct(tr[0], ts_nsec, &rx_ts);
		rx_have_ts = true;
	}
#endif

	struct net_pkt *pkt = net_pkt_rx_alloc_with_buffer(
		data->iface, len, AF_UNSPEC, 0, K_NO_WAIT);
	if (!pkt) {
		LOG_ERR("RX net_pkt alloc failed (len=%d)", len);
		return true;
	}
	if (net_pkt_write(pkt, data->rx_buf, len) < 0) {
		LOG_ERR("RX net_pkt_write failed");
		net_pkt_unref(pkt);
		return true;
	}
#ifdef CONFIG_AES67_PTP_SOFTWARE
	if (rx_have_ts) {
		net_pkt_set_timestamp(pkt, &rx_ts);
	}
#endif
	int rc = net_recv_data(data->iface, pkt);

	if (rc < 0) {
		LOG_ERR("RX deliver err %d (len=%d)", rc, len);
		net_pkt_unref(pkt);
	} else {
		LOG_DBG("RX %d bytes -> stack", len);
	}
	return true;
}

static void eth_spi_rx_thread(void *p1, void *p2, void *p3)
{
	const struct device *dev = p1;
	struct eth_spi_data *data = dev->data;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		/* IRQ-paced normally; the timeout is a safety net that keeps
		 * RX alive if an edge is ever lost (rx_one starts with a cheap
		 * rx_ready probe, so a timeout wake costs one short read). */
		k_sem_take(&data->rx_sem, K_MSEC(CONFIG_ETH_SPI_POLL_INTERVAL_MS));

		/* Drain until the rx_ready register (authoritative, unlike
		 * the IRQ pin level) reports the buffer empty. Bounded so a
		 * saturated line cannot monopolise same-priority threads; on
		 * hitting the cap we self-wake and continue immediately. */
		for (int burst = 0; eth_spi_rx_one(data); burst++) {
			if (burst >= 64) {
				k_sem_give(&data->rx_sem);
				break;
			}
		}
	}
}

/* ---- TX path ---- */

/* Wait for a previous transmit to finish (bounded, see TX_DONE_POLL_LIMIT).
 * tx_done starts de-asserted after reset, so only wait once we have actually
 * requested a transmit — mirrors eth_litex.c / the kernel driver. */
static void eth_spi_wait_tx_done_locked(struct eth_spi_data *data)
{
	uint32_t status;

	if (!data->tx_started) {
		return;
	}
	for (int i = 0; i < TX_DONE_POLL_LIMIT; i++) {
		if (spibone_read_locked(CSR_AES67_CSR_STATUS_ADDR, &status) < 0) {
			return;
		}
		if (status & AES67_STATUS_ETH_TX_DONE) {
			return;
		}
	}
	LOG_WRN("TX done timeout");
}

static int eth_spi_tx_frame(struct eth_spi_data *data, struct net_pkt *pkt,
			    size_t len)
{
	uint32_t ctrl;
	int ret;
#ifdef CONFIG_AES67_PTP_SOFTWARE
	bool want_ts = (pkt != NULL) && net_pkt_is_tx_timestamping(pkt);
	uint32_t sec_pre = 0, nsec_pre = 0;
#endif

	spibone_bus_lock();

	eth_spi_wait_tx_done_locked(data);

#ifdef CONFIG_AES67_PTP_SOFTWARE
	/* Remember the current TX-timestamp latch so we can tell OUR frame's
	 * capture apart from the previous frame's (see below). */
	if (want_ts) {
		(void)spibone_read_locked(CSR_AES67_CSR_TX_TIMESTAMP_SEC_IN_ADDR,
					  &sec_pre);
		(void)spibone_read_locked(CSR_AES67_CSR_TX_TIMESTAMP_NSEC_IN_ADDR,
					  &nsec_pre);
	}
#endif

	/* Stream the whole frame in one burst, then latch its length. */
	ret = spibone_write_burst_locked(ETH_BUF_TX_MEM, data->tx_buf, len);
	if (ret == 0) {
		ret = spibone_write_locked(CSR_ETH_BUF_TX_LEN_ADDR, len);
	}

	/* Pulse eth_tx_request. Each SPI transaction takes ≥100 µs, so the
	 * FPGA's mac_tx_clock CDC capture window is trivially satisfied. */
	if (ret == 0) {
		ret = spibone_read_locked(CSR_AES67_CSR_CTRL_ADDR, &ctrl);
	}
	if (ret == 0) {
		ret = spibone_write_locked(CSR_AES67_CSR_CTRL_ADDR,
					   ctrl | AES67_CTRL_ETH_TX_REQUEST);
	}
	if (ret == 0) {
		ret = spibone_write_locked(CSR_AES67_CSR_CTRL_ADDR,
					   ctrl & ~AES67_CTRL_ETH_TX_REQUEST);
		data->tx_started = true;
	}

#ifdef CONFIG_AES67_PTP_SOFTWARE
	/* PTP event frames need their egress timestamp reported back to the
	 * stack BEFORE the peer's response can be processed — a fast master
	 * answers a Delay_Req in well under a millisecond, and Zephyr's
	 * Delay_Resp handler silently consumes the request with its k_uptime
	 * placeholder t3 if the real timestamp hasn't landed yet. Reporting
	 * while still holding the bus lock makes the ordering watertight: the
	 * RX thread cannot drain the response from the FPGA until we release,
	 * and net_if_add_tx_timestamp wakes the cooperative tx_tstamp thread
	 * immediately, so the PTP stack sees t3 first. (The wallclock reads in
	 * aes67_ptp_reconstruct re-enter the bus mutex — it is recursive.)
	 *
	 * The tx_timestamp CSR latches at the end of every MCU-buffer frame,
	 * and eth_tx_done is a level that is still high from the previous
	 * frame until the MAC actually starts ours — which can lag behind
	 * RTP traffic. Waiting on tx_done alone therefore hands out the
	 * PREVIOUS frame's capture (seen as a wild ~hundreds-of-ms path
	 * delay). Poll until the latch content changes from its pre-send
	 * value instead. */
	if (ret == 0 && want_ts) {
		uint32_t sec = sec_pre, nsec = nsec_pre;
		struct net_ptp_time tx_ts;
		int i;

		for (i = 0; i < TX_TS_POLL_LIMIT; i++) {
			(void)spibone_read_locked(
				CSR_AES67_CSR_TX_TIMESTAMP_SEC_IN_ADDR, &sec);
			(void)spibone_read_locked(
				CSR_AES67_CSR_TX_TIMESTAMP_NSEC_IN_ADDR, &nsec);
			if (sec != sec_pre || nsec != nsec_pre) {
				break;
			}
		}
		if (i == TX_TS_POLL_LIMIT) {
			/* Frame not (yet) on the wire — the latch still holds
			 * the previous frame's capture. Report a null
			 * timestamp: the PTP stack's Delay_Req callback treats
			 * it as "no timestamp available" and discards the
			 * whole exchange, keeping the last good path delay
			 * instead of computing one from a stale t3. */
			LOG_WRN("TX timestamp latch unchanged — invalidating exchange");
			tx_ts.second = 0;
			tx_ts.nanosecond = 0;
		} else {
			aes67_ptp_reconstruct((uint8_t)(sec & 0xF),
					      nsec & 0x3FFFFFFF, &tx_ts);
			/* Temporary SW-PTP bring-up diagnostics. */
			LOG_WRN("TXTS report %llu.%09u (raw sec=%u nsec=%u, polls=%d)",
				tx_ts.second, tx_ts.nanosecond, sec & 0xF,
				nsec & 0x3FFFFFFF, i);
		}
		net_pkt_set_timestamp(pkt, &tx_ts);
		net_if_add_tx_timestamp(pkt);
	}
#endif

	spibone_bus_unlock();
	return ret;
}

static void eth_spi_tx_thread(void *p1, void *p2, void *p3)
{
	const struct device *dev = p1;
	struct eth_spi_data *data = dev->data;
	struct net_pkt *pkt = NULL;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		k_msgq_get(&eth_spi_tx_queue, &pkt, K_FOREVER);

		size_t len = net_pkt_get_len(pkt);

		if (len < 14 || len > sizeof(data->tx_buf)) {
			LOG_ERR("TX invalid len %zu", len);
			net_pkt_unref(pkt);
			continue;
		}
		if (net_pkt_read(pkt, data->tx_buf, len) < 0) {
			LOG_ERR("TX net_pkt_read failed");
			net_pkt_unref(pkt);
			continue;
		}

		int ret = eth_spi_tx_frame(data, pkt, len);

		if (ret < 0) {
			LOG_ERR("TX failed: %d (len=%zu)", ret, len);
		} else {
			LOG_DBG("TX %zu bytes", len);
		}
		net_pkt_unref(pkt);
	}
}

#if !ETH_SPI_HAS_IRQ_GPIO
/* Without an IRQ line we poll the eth_buf rx_ready register every
 * CONFIG_ETH_SPI_POLL_INTERVAL_MS. The RX thread is woken on the rising
 * edge — equivalent to the IRQ path's "one wake per frame". */
static void eth_spi_poll_thread(void *p1, void *p2, void *p3)
{
	const struct device *dev = p1;
	struct eth_spi_data *data = dev->data;
	bool prev_avail = false;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		k_msleep(CONFIG_ETH_SPI_POLL_INTERVAL_MS);

		uint32_t ready;

		if (spibone_read(CSR_ETH_BUF_RX_READY_ADDR, &ready) < 0) {
			continue;
		}
		bool avail = (ready & 0x01) != 0;

		if (avail && !prev_avail) {
			k_sem_give(&data->rx_sem);
		}
		prev_avail = avail;
	}
}
#endif /* !ETH_SPI_HAS_IRQ_GPIO */

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
#ifdef CONFIG_AES67_PTP_SOFTWARE
	return ETHERNET_LINK_100BASE | ETHERNET_PTP;
#else
	return ETHERNET_LINK_100BASE;
#endif
}

#ifdef CONFIG_AES67_PTP_SOFTWARE
static const struct device *eth_spi_get_ptp_clock(const struct device *dev)
{
	ARG_UNUSED(dev);
	return aes67_ptp_clock_device();
}
#endif

static const struct ethernet_api eth_spi_api = {
	.iface_api.init   = eth_spi_iface_init,
	.get_capabilities = eth_spi_get_capabilities,
#ifdef CONFIG_AES67_PTP_SOFTWARE
	.get_ptp_clock    = eth_spi_get_ptp_clock,
#endif
	.send             = eth_spi_send,
};

/* ---- Init ---- */

static int eth_spi_init(const struct device *dev)
{
	struct eth_spi_data *data = dev->data;
	const struct eth_spi_config *cfg = dev->config;

	data->dev = dev;
	k_sem_init(&data->rx_sem, 0, 1);

	if (spibone_get_dev() == NULL || !device_is_ready(spibone_get_dev())) {
		LOG_ERR("spibone bus driver not available");
		return -ENODEV;
	}

	/* Release a possibly stale RX buffer, clear the pending event, then
	 * enable the RX-ready event so mcu_irq_o follows it. */
	spibone_bus_lock();
	(void)spibone_write_locked(CSR_ETH_BUF_RX_ACK_ADDR, 1);
	(void)spibone_write_locked(CSR_ETH_BUF_RX_ACK_ADDR, 0);
	(void)spibone_write_locked(CSR_ETH_BUF_EV_PENDING_ADDR, ETH_BUF_EV_RX_READY);
	(void)spibone_write_locked(CSR_ETH_BUF_EV_ENABLE_ADDR, ETH_BUF_EV_RX_READY);
	spibone_bus_unlock();

#if ETH_SPI_HAS_IRQ_GPIO
	if (!gpio_is_ready_dt(&cfg->irq_gpio)) {
		LOG_ERR("IRQ GPIO %s not ready", cfg->irq_gpio.port->name);
		return -ENODEV;
	}
	gpio_pin_configure_dt(&cfg->irq_gpio, GPIO_INPUT);
	gpio_init_callback(&data->irq_cb, eth_spi_irq_handler,
			   BIT(cfg->irq_gpio.pin));
	gpio_add_callback(cfg->irq_gpio.port, &data->irq_cb);
	/* Edge-triggered, single-shot per FPGA assertion. The RX thread
	 * re-checks the pin level after each frame to catch frames that
	 * arrived while the line stayed asserted (no fresh edge). */
	gpio_pin_interrupt_configure_dt(&cfg->irq_gpio,
					(cfg->irq_gpio.dt_flags & GPIO_ACTIVE_LOW)
						? GPIO_INT_EDGE_FALLING
						: GPIO_INT_EDGE_RISING);
	LOG_INF("IRQ on %s pin %d (edge)", cfg->irq_gpio.port->name, cfg->irq_gpio.pin);

	/* The line may already be asserted at boot (frame queued before
	 * we configured the IRQ), and an edge trigger only fires on
	 * transitions — so kick the RX thread once if needed. */
	if (gpio_pin_get_dt(&cfg->irq_gpio) == 1) {
		k_sem_give(&data->rx_sem);
	}
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

#if !ETH_SPI_HAS_IRQ_GPIO
	k_thread_create(&data->poll_thread, data->poll_stack,
			K_KERNEL_STACK_SIZEOF(data->poll_stack),
			eth_spi_poll_thread, (void *)dev, NULL, NULL,
			K_PRIO_PREEMPT(9), 0, K_NO_WAIT);
	k_thread_name_set(&data->poll_thread, "eth_spi_poll");
#endif

	k_work_schedule(&data->link_work, K_MSEC(100));

	LOG_INF("eth_spi init complete (spibone transport)");
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
