// SPDX-License-Identifier: GPL-2.0
/*
 * AES67 FPGA Ethernet driver — carries the FPGA eth_buf control-plane datapath
 * as a Linux net_device with hardware PTP timestamping, paired with the PHC in
 * aes67_phc.c so stock ptp4l can run on top.
 *
 * SPI transfers sleep, so (unlike a DMA NIC) none of the bus work may run in the
 * atomic ndo_start_xmit / softirq NAPI context. Following the enc28j60/ks8851
 * model, all bus I/O happens in process context: a threaded IRQ (or a poll
 * delayed-work fallback) drains RX, and an ordered workqueue drains a software
 * TX queue. bus_lock (in the bus layer) serialises everything onto the one link.
 */
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/net_tstamp.h>
#include <linux/of.h>
#include <linux/workqueue.h>

#include "aes67_eth.h"

static unsigned int poll_ms = 1;
module_param(poll_ms, uint, 0644);
MODULE_PARM_DESC(poll_ms, "RX poll interval (ms) when no IRQ is wired");

static bool rx_ts = true;
module_param(rx_ts, bool, 0644);
MODULE_PARM_DESC(rx_ts, "Parse the FPGA RX hardware-timestamp trailer");

/* Trailing bytes to strip from each RX frame (the MAC's FCS). Whether the FPGA
 * stores the 4-byte FCS in eth_buf is a gateware detail; if frames arrive
 * truncated (e.g. DHCP fails on bad checksum) set this to 0 to deliver the
 * buffer verbatim — a trailing FCS, if present, is harmless to the L3 stack. */
static unsigned int rx_strip = AES67_FCS_LEN;
module_param(rx_strip, uint, 0644);
MODULE_PARM_DESC(rx_strip, "Trailing bytes (FCS) to drop from each RX frame (default 4; try 0)");

#define TX_QUEUE_MAX        32
#define TX_QUEUE_WAKE       8
#define TX_DONE_POLL_LIMIT  64
#define MIN_ETH_FRAME       14

/* --- private state additions (see struct aes67_priv in aes67_eth.h) ------- */
/* Per-device TX queue + workers live here rather than the shared header so the
 * header stays a stable interface; we keep them in the priv via these fields. */

/* --- gateware helper sequences -------------------------------------------- */

static void aes67_set_fpga_mac(struct aes67_priv *p, const u8 *mac)
{
	u32 hi = ((u32)mac[0] << 8) | mac[1];
	u32 lo = ((u32)mac[2] << 24) | ((u32)mac[3] << 16) |
		 ((u32)mac[4] << 8) | mac[5];

	mutex_lock(&p->bus_lock);
	aes67_wb_write_locked(p, AES67_REG_AES67_CSR_MAC_ADDR_LO, lo);
	aes67_wb_write_locked(p, AES67_REG_AES67_CSR_MAC_ADDR_HI, hi);
	mutex_unlock(&p->bus_lock);
}

/* Poll eth_tx_done (bounded). The MAC de-asserts it at reset, so the very first
 * frame may never see it set — mirror the firmware/daemon and proceed anyway. */
static void aes67_wait_tx_done_locked(struct aes67_priv *p)
{
	u32 status;
	int i;

	for (i = 0; i < TX_DONE_POLL_LIMIT; i++) {
		if (aes67_wb_read_locked(p, AES67_REG_AES67_CSR_STATUS, &status))
			return;
		if (status & AES67_STATUS_ETH_TX_DONE)
			return;
	}
}

static void aes67_tx_pulse_request_locked(struct aes67_priv *p)
{
	u32 ctrl;

	if (aes67_wb_read_locked(p, AES67_REG_AES67_CSR_CTRL, &ctrl))
		return;
	aes67_wb_write_locked(p, AES67_REG_AES67_CSR_CTRL, ctrl | AES67_CTRL_ETH_TX_REQUEST);
	aes67_wb_write_locked(p, AES67_REG_AES67_CSR_CTRL, ctrl & ~AES67_CTRL_ETH_TX_REQUEST);
}

/* --- TX path -------------------------------------------------------------- */

static void aes67_tx_one(struct aes67_priv *p, struct sk_buff *skb)
{
	struct net_device *ndev = p->netdev;
	bool want_ts = p->hwts_tx_on &&
		       (skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP);
	u32 tx_base = AES67_MEM_ETH_BUF + AES67_TX_REGION_OFFSET;
	u32 ts_sec = 0, ts_nsec = 0;
	unsigned int i;
	int ret = 0;

	if (skb->len > AES67_MAX_FRAME || skb->len < MIN_ETH_FRAME) {
		ndev->stats.tx_dropped++;
		dev_kfree_skb_any(skb);
		return;
	}

	mutex_lock(&p->bus_lock);
	aes67_wait_tx_done_locked(p);
	for (i = 0; i < skb->len && !ret; i++)
		ret = aes67_wb_write_locked(p, tx_base + 4 * i, skb->data[i]);
	if (!ret)
		ret = aes67_wb_write_locked(p, AES67_REG_ETH_BUF_TX_LEN, skb->len);
	if (!ret)
		aes67_tx_pulse_request_locked(p);

	/* The TX timestamp CSR holds only the *last* transmitted frame, so read
	 * it back here while we still hold the bus and before any other TX. */
	if (!ret && want_ts) {
		aes67_wait_tx_done_locked(p);
		aes67_wb_read_locked(p, AES67_REG_AES67_CSR_TX_TIMESTAMP_SEC_IN, &ts_sec);
		aes67_wb_read_locked(p, AES67_REG_AES67_CSR_TX_TIMESTAMP_NSEC_IN, &ts_nsec);
	}
	mutex_unlock(&p->bus_lock);

	if (ret) {
		ndev->stats.tx_errors++;
		dev_kfree_skb_any(skb);
		return;
	}

	ndev->stats.tx_packets++;
	ndev->stats.tx_bytes += skb->len;

	if (want_ts) {
		struct skb_shared_hwtstamps hwts = {};
		u64 ns;

		if (!aes67_ts_reconstruct(p, ts_sec & 0xf, ts_nsec, &ns)) {
			hwts.hwtstamp = ns_to_ktime(ns);
			skb_tstamp_tx(skb, &hwts);
		}
	}
	dev_consume_skb_any(skb);
}

static void aes67_tx_work(struct work_struct *work)
{
	struct aes67_priv *p = container_of(work, struct aes67_priv, tx_work);
	struct sk_buff *skb;

	while ((skb = skb_dequeue(&p->txq))) {
		aes67_tx_one(p, skb);
		if (netif_queue_stopped(p->netdev) &&
		    skb_queue_len(&p->txq) < TX_QUEUE_WAKE)
			netif_wake_queue(p->netdev);
	}
}

static netdev_tx_t aes67_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct aes67_priv *p = netdev_priv(ndev);

	/* Tell the stack a hardware TX timestamp will follow on the error queue. */
	if (p->hwts_tx_on && (skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP))
		skb_shinfo(skb)->tx_flags |= SKBTX_IN_PROGRESS;

	skb_queue_tail(&p->txq, skb);
	if (skb_queue_len(&p->txq) >= TX_QUEUE_MAX)
		netif_stop_queue(ndev);
	queue_work(p->wq, &p->tx_work);
	return NETDEV_TX_OK;
}

/* --- RX path (process context) -------------------------------------------- */

/* Drain one ready frame. Returns true if a frame was processed. */
static bool aes67_rx_one(struct aes67_priv *p)
{
	struct net_device *ndev = p->netdev;
	u32 rx_base = AES67_MEM_ETH_BUF;
	unsigned int trailer = rx_ts ? AES67_RX_TS_TRAILER_LEN : 0;
	struct sk_buff *skb;
	u32 raw_len, word;
	unsigned int total, payload, i;
	u8 cap_sec = 0;
	u32 cap_nsec = 0;
	int ret;

	mutex_lock(&p->bus_lock);
	ret = aes67_wb_read_locked(p, AES67_REG_ETH_BUF_RX_LEN, &raw_len);
	if (ret) {
		mutex_unlock(&p->bus_lock);
		return false;
	}
	total = raw_len & 0xffff;

	/* total = payload + rx_strip (FCS) + (optional) timestamp trailer. */
	if (total < MIN_ETH_FRAME + rx_strip + trailer ||
	    total > AES67_MAX_FRAME + rx_strip + trailer) {
		ndev->stats.rx_length_errors++;
		goto release;   /* drop, but still release the buffer */
	}

	for (i = 0; i < total; i++) {
		ret = aes67_wb_read_locked(p, rx_base + 4 * i, &word);
		if (ret) {
			ndev->stats.rx_errors++;
			goto release;
		}
		p->rx_buf[i] = word & 0xff;
	}

	if (trailer) {
		const u8 *t = &p->rx_buf[total - trailer];

		cap_sec  = t[0] & 0xf;
		cap_nsec = ((u32)t[1]) | ((u32)t[2] << 8) |
			   ((u32)t[3] << 16) | ((u32)t[4] << 24);
	}
	payload = total - rx_strip - trailer;

release:
	/* Release the buffer + clear the RX-ready event regardless. */
	aes67_wb_write_locked(p, AES67_REG_ETH_BUF_RX_ACK, 1);
	aes67_wb_write_locked(p, AES67_REG_ETH_BUF_RX_ACK, 0);
	aes67_wb_write_locked(p, AES67_REG_ETH_BUF_EV_PENDING, AES67_EV_RX_READY);
	mutex_unlock(&p->bus_lock);

	if (ret || payload < MIN_ETH_FRAME)
		return true;   /* a frame was consumed even if dropped */

	skb = netdev_alloc_skb_ip_align(ndev, payload);
	if (!skb) {
		ndev->stats.rx_dropped++;
		return true;
	}
	skb_put_data(skb, p->rx_buf, payload);

	if (rx_ts && p->hwts_rx_on) {
		u64 ns;

		if (!aes67_ts_reconstruct(p, cap_sec, cap_nsec, &ns))
			skb_hwtstamps(skb)->hwtstamp = ns_to_ktime(ns);
	}

	skb->protocol = eth_type_trans(skb, ndev);
	ndev->stats.rx_packets++;
	ndev->stats.rx_bytes += payload;
	netif_rx(skb);
	return true;
}

/* Update carrier from the FPGA PHY link bit, then drain all ready RX frames. */
static void aes67_service(struct aes67_priv *p)
{
	u32 status;

	if (!aes67_wb_read(p, AES67_REG_AES67_CSR_STATUS, &status)) {
		bool up = status & AES67_STATUS_ETH_LINK_UP;

		if (up && !netif_carrier_ok(p->netdev))
			netif_carrier_on(p->netdev);
		else if (!up && netif_carrier_ok(p->netdev))
			netif_carrier_off(p->netdev);
	}

	for (;;) {
		u32 ready;

		if (aes67_wb_read(p, AES67_REG_ETH_BUF_RX_READY, &ready))
			break;
		if (!(ready & 1))
			break;
		if (!aes67_rx_one(p))
			break;
	}
}

static irqreturn_t aes67_irq_thread(int irq, void *dev_id)
{
	aes67_service(dev_id);
	return IRQ_HANDLED;
}

static void aes67_poll_work(struct work_struct *work)
{
	struct aes67_priv *p = container_of(to_delayed_work(work),
					    struct aes67_priv, poll_work);

	aes67_service(p);
	if (netif_running(p->netdev))
		queue_delayed_work(p->wq, &p->poll_work,
				   msecs_to_jiffies(poll_ms));
}

/* --- netdev ops ----------------------------------------------------------- */

static int aes67_open(struct net_device *ndev)
{
	struct aes67_priv *p = netdev_priv(ndev);
	int ret;

	aes67_set_fpga_mac(p, ndev->dev_addr);

	/* Enable the eth_buf RX-ready event (drives the IRQ line). */
	aes67_wb_write(p, AES67_REG_ETH_BUF_EV_ENABLE, AES67_EV_RX_READY);

	if (p->irq > 0) {
		ret = request_threaded_irq(p->irq, NULL, aes67_irq_thread,
					   IRQF_ONESHOT, ndev->name, p);
		if (ret) {
			netdev_err(ndev, "failed to request IRQ %d: %d\n",
				   p->irq, ret);
			return ret;
		}
	}

	netif_carrier_off(ndev);
	netif_start_queue(ndev);
	/* Always run the poll worker: it primes carrier + drains any pending RX
	 * immediately, then keeps polling as a safety net even when an IRQ is
	 * configured (in case eth_buf_irq is mis-wired or never fires). With a
	 * working IRQ you can raise poll_ms to make this a slow backstop. */
	queue_delayed_work(p->wq, &p->poll_work, 0);
	return 0;
}

static int aes67_stop(struct net_device *ndev)
{
	struct aes67_priv *p = netdev_priv(ndev);
	struct sk_buff *skb;

	netif_stop_queue(ndev);

	if (p->irq > 0)
		free_irq(p->irq, p);
	cancel_delayed_work_sync(&p->poll_work);
	cancel_work_sync(&p->tx_work);

	while ((skb = skb_dequeue(&p->txq)))
		dev_kfree_skb(skb);

	netif_carrier_off(ndev);
	return 0;
}

static int aes67_set_mac_address(struct net_device *ndev, void *addr)
{
	struct aes67_priv *p = netdev_priv(ndev);
	int ret = eth_mac_addr(ndev, addr);

	if (ret)
		return ret;
	aes67_set_fpga_mac(p, ndev->dev_addr);
	return 0;
}

static int aes67_hwtstamp_set(struct net_device *ndev, struct ifreq *ifr)
{
	struct aes67_priv *p = netdev_priv(ndev);
	struct hwtstamp_config cfg;

	if (copy_from_user(&cfg, ifr->ifr_data, sizeof(cfg)))
		return -EFAULT;

	switch (cfg.tx_type) {
	case HWTSTAMP_TX_OFF:
		p->hwts_tx_on = false;
		break;
	case HWTSTAMP_TX_ON:
		p->hwts_tx_on = true;
		break;
	default:
		return -ERANGE;
	}

	if (cfg.rx_filter == HWTSTAMP_FILTER_NONE) {
		p->hwts_rx_on = false;
	} else {
		p->hwts_rx_on = true;
		cfg.rx_filter = HWTSTAMP_FILTER_ALL;
	}

	return copy_to_user(ifr->ifr_data, &cfg, sizeof(cfg)) ? -EFAULT : 0;
}

static int aes67_hwtstamp_get(struct net_device *ndev, struct ifreq *ifr)
{
	struct aes67_priv *p = netdev_priv(ndev);
	struct hwtstamp_config cfg = {
		.tx_type   = p->hwts_tx_on ? HWTSTAMP_TX_ON : HWTSTAMP_TX_OFF,
		.rx_filter = p->hwts_rx_on ? HWTSTAMP_FILTER_ALL : HWTSTAMP_FILTER_NONE,
	};

	return copy_to_user(ifr->ifr_data, &cfg, sizeof(cfg)) ? -EFAULT : 0;
}

static int aes67_eth_ioctl(struct net_device *ndev, struct ifreq *ifr, int cmd)
{
	switch (cmd) {
	case SIOCSHWTSTAMP:
		return aes67_hwtstamp_set(ndev, ifr);
	case SIOCGHWTSTAMP:
		return aes67_hwtstamp_get(ndev, ifr);
	default:
		return -EOPNOTSUPP;
	}
}

static const struct net_device_ops aes67_netdev_ops = {
	.ndo_open            = aes67_open,
	.ndo_stop            = aes67_stop,
	.ndo_start_xmit      = aes67_start_xmit,
	.ndo_set_mac_address = aes67_set_mac_address,
	.ndo_eth_ioctl       = aes67_eth_ioctl,
	.ndo_validate_addr   = eth_validate_addr,
};

/* --- ethtool -------------------------------------------------------------- */

static void aes67_get_drvinfo(struct net_device *ndev,
			      struct ethtool_drvinfo *info)
{
	strscpy(info->driver, "aes67_eth", sizeof(info->driver));
}

static int aes67_get_ts_info(struct net_device *ndev,
			     struct kernel_ethtool_ts_info *info)
{
	struct aes67_priv *p = netdev_priv(ndev);

	info->so_timestamping = SOF_TIMESTAMPING_TX_HARDWARE |
				SOF_TIMESTAMPING_RX_HARDWARE |
				SOF_TIMESTAMPING_RAW_HARDWARE;
	info->phc_index = p->ptp_clock ? ptp_clock_index(p->ptp_clock) : -1;
	info->tx_types  = BIT(HWTSTAMP_TX_OFF) | BIT(HWTSTAMP_TX_ON);
	info->rx_filters = BIT(HWTSTAMP_FILTER_NONE) | BIT(HWTSTAMP_FILTER_ALL);
	return 0;
}

static const struct ethtool_ops aes67_ethtool_ops = {
	.get_drvinfo = aes67_get_drvinfo,
	.get_link    = ethtool_op_get_link,
	.get_ts_info = aes67_get_ts_info,
};

/* --- probe / remove ------------------------------------------------------- */

static int aes67_probe(struct spi_device *spi)
{
	struct net_device *ndev;
	struct aes67_priv *p;
	u32 lo, hi;
	int ret;

	ndev = alloc_etherdev(sizeof(*p));
	if (!ndev)
		return -ENOMEM;
	SET_NETDEV_DEV(ndev, &spi->dev);

	p = netdev_priv(ndev);
	p->spi = spi;
	p->netdev = ndev;
	p->irq = spi->irq;
	mutex_init(&p->bus_lock);
	spin_lock_init(&p->tx_ts_lock);
	skb_queue_head_init(&p->txq);
	INIT_WORK(&p->tx_work, aes67_tx_work);
	INIT_DELAYED_WORK(&p->poll_work, aes67_poll_work);

	p->wq = alloc_ordered_workqueue("aes67_%s", 0, dev_name(&spi->dev));
	if (!p->wq) {
		ret = -ENOMEM;
		goto err_free;
	}

	ndev->netdev_ops  = &aes67_netdev_ops;
	ndev->ethtool_ops = &aes67_ethtool_ops;
	ndev->watchdog_timeo = msecs_to_jiffies(5000);

	spi_set_drvdata(spi, p);

	/* Seed the MAC from the FPGA (set by a prior boot/daemon) if non-zero,
	 * otherwise assign a random one and program it back in aes67_open(). */
	if (!aes67_wb_read(p, AES67_REG_AES67_CSR_MAC_ADDR_LO, &lo) &&
	    !aes67_wb_read(p, AES67_REG_AES67_CSR_MAC_ADDR_HI, &hi) &&
	    (lo || (hi & 0xffff))) {
		u8 mac[ETH_ALEN] = {
			(hi >> 8) & 0xff, hi & 0xff,
			(lo >> 24) & 0xff, (lo >> 16) & 0xff,
			(lo >> 8) & 0xff, lo & 0xff,
		};
		eth_hw_addr_set(ndev, mac);
	} else {
		eth_hw_addr_random(ndev);
	}

	ret = aes67_phc_register(p);
	if (ret) {
		dev_err(&spi->dev, "PHC register failed: %d\n", ret);
		goto err_wq;
	}

	ret = aes67_ctl_register(p);
	if (ret) {
		dev_err(&spi->dev, "control device register failed: %d\n", ret);
		goto err_phc;
	}

	ret = register_netdev(ndev);
	if (ret) {
		dev_err(&spi->dev, "register_netdev failed: %d\n", ret);
		goto err_ctl;
	}

	dev_info(&spi->dev, "AES67 netdev %s, PHC ptp%d, %s RX\n",
		 ndev->name, ptp_clock_index(p->ptp_clock),
		 p->irq > 0 ? "IRQ" : "polled");
	return 0;

err_ctl:
	aes67_ctl_unregister(p);
err_phc:
	aes67_phc_unregister(p);
err_wq:
	destroy_workqueue(p->wq);
err_free:
	free_netdev(ndev);
	return ret;
}

static void aes67_remove(struct spi_device *spi)
{
	struct aes67_priv *p = spi_get_drvdata(spi);

	unregister_netdev(p->netdev);
	aes67_ctl_unregister(p);
	aes67_phc_unregister(p);
	destroy_workqueue(p->wq);
	free_netdev(p->netdev);
}

static const struct of_device_id aes67_of_match[] = {
	{ .compatible = "aes67,spibone" },
	{ }
};
MODULE_DEVICE_TABLE(of, aes67_of_match);

/* The SPI core derives the modalias from the DT compatible by stripping the
 * vendor prefix ("aes67,spibone" -> "spibone"), so the id_table entry must be
 * named "spibone" to match (otherwise: "has no spi_device_id" warning). */
static const struct spi_device_id aes67_spi_ids[] = {
	{ "spibone", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, aes67_spi_ids);

static struct spi_driver aes67_spi_driver = {
	.driver = {
		.name = "aes67_eth",
		.of_match_table = aes67_of_match,
	},
	.id_table = aes67_spi_ids,
	.probe = aes67_probe,
	.remove = aes67_remove,
};
module_spi_driver(aes67_spi_driver);

MODULE_DESCRIPTION("AES67 FPGA Ethernet + PHC driver");
MODULE_AUTHOR("AES67 project");
MODULE_LICENSE("GPL");
