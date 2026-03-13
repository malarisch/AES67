/*
 * LiteX SoC Ethernet driver for AES67 FPGA
 *
 * Accesses the FPGA via:
 *   - AES67_CSR registers (control/status)
 *   - ETH_BUF CSR registers (EventManager + rx_len/ack/tx_len)
 *   - ETH_BUF memory (Wishbone-mapped RX/TX packet buffers)
 *   - IRQ (eth_buf EventManager rx_ready)
 *
 * All addresses are imported from LiteX-generated headers (csr.h, mem.h, soc.h).
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/spinlock.h>
#include <string.h>

#include "eth_litex.h"

LOG_MODULE_REGISTER(eth_litex, CONFIG_ETHERNET_LOG_LEVEL);

/* ---- Driver data structures ---- */

struct eth_litex_config {
	uint32_t irq_num;
};

struct eth_litex_data {
	struct net_if *iface;
	const struct device *dev;
	uint8_t mac_addr[6];

	struct k_thread rx_thread;
	struct k_thread tx_thread;
	struct k_work_delayable link_work;

	struct k_spinlock ctrl_lock;   /* Protects AES67_CSR_CTRL read-modify-write */
	struct k_sem rx_sem;           /* Signaled by ISR on RX packet */

	uint8_t rx_buf[ETH_LITEX_MAX_PKT_SIZE];

	K_KERNEL_STACK_MEMBER(rx_stack, CONFIG_ETH_LITEX_RX_STACK_SIZE);
	K_KERNEL_STACK_MEMBER(tx_stack, CONFIG_ETH_LITEX_RX_STACK_SIZE);
};

K_MSGQ_DEFINE(litex_tx_queue, sizeof(struct net_pkt *), 4, 4);

/* Forward-declare the device so DEVICE_GET(eth_litex0) works in init */
DEVICE_DECLARE(eth_litex0);

/* ---- CSR convenience (thread-safe ctrl register) ---- */

int eth_litex_ctrl_set_bits(const struct device *dev, uint32_t bits)
{
	struct eth_litex_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->ctrl_lock);

	uint32_t val = litex_csr_read(CSR_AES67_CSR_CTRL_ADDR);
	val |= bits;
	litex_csr_write(CSR_AES67_CSR_CTRL_ADDR, val);

	k_spin_unlock(&data->ctrl_lock, key);
	return 0;
}

int eth_litex_ctrl_clear_bits(const struct device *dev, uint32_t bits)
{
	struct eth_litex_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->ctrl_lock);

	uint32_t val = litex_csr_read(CSR_AES67_CSR_CTRL_ADDR);
	val &= ~bits;
	litex_csr_write(CSR_AES67_CSR_CTRL_ADDR, val);

	k_spin_unlock(&data->ctrl_lock, key);
	return 0;
}

/* ---- Public helpers ---- */

int eth_litex_write_mac(const struct device *dev, const uint8_t mac[6])
{
	ARG_UNUSED(dev);

	uint32_t lo = (uint32_t)mac[0] |
		      ((uint32_t)mac[1] << 8) |
		      ((uint32_t)mac[2] << 16) |
		      ((uint32_t)mac[3] << 24);
	uint32_t hi = (uint32_t)mac[4] |
		      ((uint32_t)mac[5] << 8);

	litex_csr_write(CSR_AES67_CSR_MAC_ADDR_LO_ADDR, lo);
	litex_csr_write(CSR_AES67_CSR_MAC_ADDR_HI_ADDR, hi);

	return 0;
}

int eth_litex_write_ip(const struct device *dev, const struct in_addr *ip)
{
	ARG_UNUSED(dev);
	litex_csr_write(CSR_AES67_CSR_IP_ADDR_ADDR, ip->s_addr);
	return 0;
}

int eth_litex_write_ptp_config(const struct device *dev,
			       const uint8_t leader_clock_id[8],
			       uint8_t time_source,
			       int8_t log_msg_interval,
			       int8_t log_announce_interval)
{
	ARG_UNUSED(dev);

	uint32_t id_lo, id_hi;
	memcpy(&id_lo, &leader_clock_id[0], 4);
	memcpy(&id_hi, &leader_clock_id[4], 4);

	litex_csr_write(CSR_AES67_CSR_PTP_LEADER_ID_LO_ADDR, id_lo);
	litex_csr_write(CSR_AES67_CSR_PTP_LEADER_ID_HI_ADDR, id_hi);
	litex_csr_write(CSR_AES67_CSR_PTP_TIME_SOURCE_ADDR, time_source);
	litex_csr_write(CSR_AES67_CSR_PTP_LOG_MSG_INTERVAL_ADDR, (uint8_t)log_msg_interval);
	litex_csr_write(CSR_AES67_CSR_PTP_ANNOUNCE_MSG_INTERVAL_ADDR, (uint8_t)log_announce_interval);

	return 0;
}

bool eth_litex_read_ppb_counts(const struct device *dev,
			       uint32_t *wc_count, uint32_t *pll_count)
{
	ARG_UNUSED(dev);

	uint32_t status = litex_csr_read(CSR_AES67_CSR_PLL_PPB_STATUS_ADDR);
	*wc_count = litex_csr_read(CSR_AES67_CSR_PLL_PPB_WC_COUNT_ADDR) & 0x3FFFFF;
	*pll_count = litex_csr_read(CSR_AES67_CSR_PLL_PPB_PLL_COUNT_ADDR) & 0x3FFFFF;

	return (status & AES67_PPB_STATUS_VALID) != 0;
}

uint32_t eth_litex_read_path_delay(const struct device *dev)
{
	ARG_UNUSED(dev);
	return litex_csr_read(CSR_AES67_CSR_PTP_PATH_DELAY_ADDR);
}

uint32_t eth_litex_read_ptp_offset(const struct device *dev)
{
	ARG_UNUSED(dev);
	return litex_csr_read(CSR_AES67_CSR_PTP_OFFSET_ADDR);
}

uint32_t eth_litex_read_status(const struct device *dev)
{
	ARG_UNUSED(dev);
	return litex_csr_read(CSR_AES67_CSR_STATUS_ADDR);
}

/* ---- Packet buffer access ----
 *
 * The EthPacketBuffer is Wishbone-mapped with 1 byte per 32-bit word
 * (only the lower 8 bits are valid per word address).
 * RX buffer: word addresses 0x000..0x5DB (byte offsets 0..1499)
 * TX buffer: word addresses 0x800..0xDDB (byte offsets 0..1499)
 *
 * From the CPU, each byte is at a 4-byte-aligned address.
 */

static inline void eth_buf_write_byte(uint16_t offset, uint8_t val)
{
	volatile uint32_t *p = (volatile uint32_t *)(ETH_BUF_TX_MEM + ((uint32_t)offset << 2));
	*p = val;
}

static inline uint8_t eth_buf_read_byte(uint16_t offset)
{
	volatile uint32_t *p = (volatile uint32_t *)(ETH_BUF_RX_MEM + ((uint32_t)offset << 2));
	return (uint8_t)(*p & 0xFF);
}

static void eth_buf_read_packet(uint8_t *dst, uint16_t len)
{
	for (uint16_t i = 0; i < len; i++) {
		dst[i] = eth_buf_read_byte(i);
	}
}

static void eth_buf_write_packet(const uint8_t *src, uint16_t len)
{
	for (uint16_t i = 0; i < len; i++) {
		eth_buf_write_byte(i, src[i]);
	}
}

/* ---- ISR ---- */

static void eth_litex_isr(const struct device *dev)
{
	struct eth_litex_data *data = dev->data;

	uint32_t pending = litex_csr_read(CSR_ETH_BUF_EV_PENDING_ADDR);
	if (pending & ETH_BUF_EV_RX_READY) {
		/* Clear pending bit (write-1-to-clear) */
		litex_csr_write(CSR_ETH_BUF_EV_PENDING_ADDR, ETH_BUF_EV_RX_READY);
		k_sem_give(&data->rx_sem);
	}
}

/* ---- TX path ---- */

static void eth_litex_tx_thread(void *p1, void *p2, void *p3)
{
	const struct device *dev = p1;
	struct net_pkt *pkt = NULL;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		k_msgq_get(&litex_tx_queue, &pkt, K_FOREVER);

		size_t len = net_pkt_get_len(pkt);
		if (len > ETH_LITEX_MAX_PKT_SIZE) {
			LOG_ERR("TX too large: %zu", len);
			net_pkt_unref(pkt);
			continue;
		}

		uint8_t buf[ETH_LITEX_MAX_PKT_SIZE];
		if (net_pkt_read(pkt, buf, len) < 0) {
			LOG_ERR("TX pkt read failed");
			net_pkt_unref(pkt);
			continue;
		}

		/* Wait for previous TX to complete (eth_tx_done status bit) */
		int wait = 0;
		while (!(litex_csr_read(CSR_AES67_CSR_STATUS_ADDR) & AES67_STATUS_ETH_TX_DONE)) {
			k_busy_wait(5);
			if (++wait > 20000) { /* ~100ms timeout */
				LOG_WRN("TX done timeout");
				break;
			}
		}

		/* Write packet data to TX buffer */
		eth_buf_write_packet(buf, len);

		/* Set TX length */
		litex_csr_write(CSR_ETH_BUF_TX_LEN_ADDR, len);

		/* Assert eth_tx_request (toggle: set then clear) */
		eth_litex_ctrl_set_bits(dev, AES67_CTRL_ETH_TX_REQUEST);
		eth_litex_ctrl_clear_bits(dev, AES67_CTRL_ETH_TX_REQUEST);

		net_pkt_unref(pkt);
	}
}

/* ---- RX path ---- */

static void eth_litex_rx_thread(void *p1, void *p2, void *p3)
{
	const struct device *dev = p1;
	struct eth_litex_data *data = dev->data;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		k_sem_take(&data->rx_sem, K_MSEC(CONFIG_ETH_LITEX_POLL_INTERVAL_MS));

		/* Check if a packet is ready */
		uint32_t ready = litex_csr_read(CSR_ETH_BUF_RX_READY_ADDR);
		if (!(ready & 0x01)) {
			continue;
		}

		uint16_t pkt_len = litex_csr_read(CSR_ETH_BUF_RX_LEN_ADDR) & 0xFFFF;

		if (pkt_len < 14 || pkt_len > ETH_LITEX_MAX_PKT_SIZE) {
			LOG_WRN("RX invalid len %u", pkt_len);
			litex_csr_write(CSR_ETH_BUF_RX_ACK_ADDR, 1);
			litex_csr_write(CSR_ETH_BUF_RX_ACK_ADDR, 0);
			continue;
		}

		/* Read packet data from RX buffer */
		eth_buf_read_packet(data->rx_buf, pkt_len);

		/* ACK: release the RX buffer for the next packet */
		litex_csr_write(CSR_ETH_BUF_RX_ACK_ADDR, 1);
		litex_csr_write(CSR_ETH_BUF_RX_ACK_ADDR, 0);

		/* Allocate net_pkt and deliver to stack */
		struct net_pkt *pkt = net_pkt_rx_alloc_with_buffer(
			data->iface, pkt_len, AF_UNSPEC, 0, K_NO_WAIT);
		if (!pkt) {
			LOG_ERR("RX alloc failed");
			continue;
		}

		if (net_pkt_write(pkt, data->rx_buf, pkt_len)) {
			LOG_ERR("RX write failed");
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

/* ---- Zephyr ethernet API ---- */

static int eth_litex_send(const struct device *dev, struct net_pkt *pkt)
{
	ARG_UNUSED(dev);

	if (k_msgq_put(&litex_tx_queue, &pkt, K_NO_WAIT) != 0) {
		return -ENOMEM;
	}
	net_pkt_ref(pkt);
	return 0;
}

static void eth_litex_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct eth_litex_data *data = dev->data;

	data->iface = iface;
	ethernet_init(iface);

	/* Static locally administered MAC */
	data->mac_addr[0] = 0x02;
	data->mac_addr[1] = 0x1C;
	data->mac_addr[2] = 0x23;
	data->mac_addr[3] = 0x17;
	data->mac_addr[4] = 0x4A;
	data->mac_addr[5] = 0xCC;

	net_if_set_link_addr(iface, data->mac_addr, sizeof(data->mac_addr),
			     NET_LINK_ETHERNET);

	/* Write MAC to FPGA so hardware knows our address */
	eth_litex_write_mac(dev, data->mac_addr);

	LOG_INF("MAC %02x:%02x:%02x:%02x:%02x:%02x",
		data->mac_addr[0], data->mac_addr[1], data->mac_addr[2],
		data->mac_addr[3], data->mac_addr[4], data->mac_addr[5]);
}

static enum ethernet_hw_caps eth_litex_get_capabilities(const struct device *dev)
{
	ARG_UNUSED(dev);
	return ETHERNET_LINK_100BASE;
}

static void eth_litex_link_work(struct k_work *work)
{
	struct k_work_delayable *dwork = CONTAINER_OF(work, struct k_work_delayable, work);
	struct eth_litex_data *data = CONTAINER_OF(dwork, struct eth_litex_data, link_work);

	if (data->iface) {
		uint32_t status = litex_csr_read(CSR_AES67_CSR_STATUS_ADDR);
		if (status & AES67_STATUS_ETH_LINK_UP) {
			net_if_carrier_on(data->iface);
			net_if_up(data->iface);
			LOG_INF("Ethernet link up");
		} else {
			k_work_schedule(dwork, K_MSEC(500));
		}
	}
}

static const struct ethernet_api eth_litex_api = {
	.iface_api.init = eth_litex_iface_init,
	.get_capabilities = eth_litex_get_capabilities,
	.send = eth_litex_send,
};

/* ---- Initialization ---- */

static int eth_litex_init(const struct device *dev)
{
	struct eth_litex_data *data = dev->data;
	const struct eth_litex_config *cfg = dev->config;

	data->dev = dev;

	LOG_INF("Initializing LiteX Ethernet driver (IRQ %u)", cfg->irq_num);

	k_sem_init(&data->rx_sem, 0, 1);

	/* Enable the EventManager rx_ready interrupt */
	litex_csr_write(CSR_ETH_BUF_EV_PENDING_ADDR, ETH_BUF_EV_RX_READY);
	litex_csr_write(CSR_ETH_BUF_EV_ENABLE_ADDR, ETH_BUF_EV_RX_READY);

	/* Connect and enable IRQ.
	 * ETH_BUF_INTERRUPT comes from generated/soc.h */
	IRQ_CONNECT(ETH_BUF_INTERRUPT, 0, eth_litex_isr,
		    DEVICE_GET(eth_litex0), 0);
	irq_enable(ETH_BUF_INTERRUPT);

	/* Make sure RX buffer is released */
	litex_csr_write(CSR_ETH_BUF_RX_ACK_ADDR, 1);
	litex_csr_write(CSR_ETH_BUF_RX_ACK_ADDR, 0);

	k_work_init_delayable(&data->link_work, eth_litex_link_work);

	/* Create RX thread */
	k_thread_create(&data->rx_thread, data->rx_stack,
			CONFIG_ETH_LITEX_RX_STACK_SIZE,
			eth_litex_rx_thread, (void *)dev, NULL, NULL,
			K_PRIO_COOP(1), 0, K_NO_WAIT);
	k_thread_name_set(&data->rx_thread, "eth_litex_rx");

	/* Create TX thread */
	k_thread_create(&data->tx_thread, data->tx_stack,
			CONFIG_ETH_LITEX_RX_STACK_SIZE,
			eth_litex_tx_thread, (void *)dev, NULL, NULL,
			K_PRIO_PREEMPT(2), 0, K_NO_WAIT);
	k_thread_name_set(&data->tx_thread, "eth_litex_tx");

	/* Schedule link-up check */
	k_work_schedule(&data->link_work, K_MSEC(100));

	return 0;
}

/* ---- Device instantiation ---- */

static struct eth_litex_data eth_litex_data_0;

static const struct eth_litex_config eth_litex_cfg_0 = {
	.irq_num = ETH_BUF_INTERRUPT,
};

ETH_NET_DEVICE_INIT(eth_litex0, "eth_litex0",
		    eth_litex_init, NULL,
		    &eth_litex_data_0, &eth_litex_cfg_0,
		    CONFIG_ETH_INIT_PRIORITY, &eth_litex_api,
		    NET_ETH_MTU);
