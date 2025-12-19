/*
 * FMC Ethernet bridge driver for the FPGA MAC interface
 * Minimal, PTP-less parallel variant matching the SPI register map.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <string.h>

LOG_MODULE_REGISTER(eth_fmc_basic, CONFIG_ETHERNET_LOG_LEVEL);

/* FPGA Ready Pin */
static const struct gpio_dt_spec fpga_ready = GPIO_DT_SPEC_GET(DT_NODELABEL(fpga_ready), gpios);

bool fpga_ready_status = false;

static bool is_fpga_ready(void)
{
	if (!gpio_is_ready_dt(&fpga_ready)) {
		if (fpga_ready_status) {
			fpga_ready_status = false;
			LOG_ERR("FPGA got unavailable");

		}
		LOG_INF("FPGA is down");
		return false;
	}
	if(!fpga_ready_status) {
		LOG_INF("FPGA got up");
		fpga_ready_status = true;
	}
	
	
	return gpio_pin_get_dt(&fpga_ready) > 0;
}

/* Register map identical to SPI bridge */
#define REG_TX_LEN_L      0x00
#define REG_TX_LEN_H      0x01
#define REG_TX_CTRL       0x02
#define REG_TX_WINDOW     0x10  /* auto-increment window */

#define REG_RX_LEN_L      0x20
#define REG_RX_LEN_H      0x21
#define REG_RX_STATUS     0x22
#define REG_RX_WINDOW     0x30  /* auto-increment window */

#define RX_STATUS_READY   BIT(0)
#define RX_STATUS_OVF     BIT(1)

#define ETH_FMC_MAX_PKT_SIZE 1518

struct eth_fmc_basic_config {
	uintptr_t base;
};

struct eth_fmc_basic_data {
	struct net_if *iface;
	uint8_t mac_addr[6];
	struct k_thread rx_thread;
	struct k_thread tx_thread;
	struct k_work_delayable link_work;
	struct k_mutex io_lock;
	K_KERNEL_STACK_MEMBER(rx_stack, CONFIG_ETH_FMC_BASIC_RX_STACK_SIZE);
	K_KERNEL_STACK_MEMBER(tx_stack, CONFIG_ETH_FMC_BASIC_RX_STACK_SIZE);
};

K_MSGQ_DEFINE(tx_queue, sizeof(struct net_pkt *), 4, 4);

/* Convenience accessors */
static inline void fmc_write8(uintptr_t base, uint8_t addr, uint8_t val)
{
	*((volatile uint8_t *)(base + addr)) = val;
}

static inline uint8_t fmc_read8(uintptr_t base, uint8_t addr)
{
	return *((volatile uint8_t *)(base + addr));
}

static inline void fmc_write_block(uintptr_t base, uint8_t addr, const uint8_t *buf, size_t len)
{
	volatile uint8_t *p = (volatile uint8_t *)(base + addr);
	for (size_t i = 0; i < len; i++) {
		p[0] = buf[i];
	}
}

static inline void fmc_read_block(uintptr_t base, uint8_t addr, uint8_t *buf, size_t len)
{
	volatile uint8_t *p = (volatile uint8_t *)(base + addr);
	for (size_t i = 0; i < len; i++) {
		buf[i] = p[0];
	}
}

/* TX path */
static void eth_fmc_basic_tx_thread(void *p1, void *p2, void *p3)
{
	const struct device *dev = p1;
	const struct eth_fmc_basic_config *cfg = dev->config;
	struct net_pkt *pkt = NULL;
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		k_msgq_get(&tx_queue, &pkt, K_FOREVER);

		if (!is_fpga_ready()) {
			/* Drop packet if FPGA is not ready */
			net_pkt_unref(pkt);
			continue;
		}

		size_t len = net_pkt_get_len(pkt);
		if (len > ETH_FMC_MAX_PKT_SIZE) {
			LOG_ERR("TX too large: %zu", len);
			net_pkt_unref(pkt);
			continue;
		}

		uint8_t buf[ETH_FMC_MAX_PKT_SIZE];
		if (net_pkt_read(pkt, buf, len) < 0) {
			LOG_ERR("pkt read failed");
			net_pkt_unref(pkt);
			continue;
		}
		LOG_INF("Sending Packet with lenght %u", len);

		k_mutex_lock(&((struct eth_fmc_basic_data *)dev->data)->io_lock, K_FOREVER);
		fmc_write8(cfg->base, REG_TX_LEN_L, len & 0xFF);
		fmc_write8(cfg->base, REG_TX_LEN_H, (len >> 8) & 0xFF);
		fmc_write_block(cfg->base, REG_TX_WINDOW, buf, len);
		fmc_write8(cfg->base, REG_TX_CTRL, 0x00);
		fmc_write8(cfg->base, REG_TX_CTRL, 0x01);
		k_mutex_unlock(&((struct eth_fmc_basic_data *)dev->data)->io_lock);

		net_pkt_unref(pkt);
	}
}

/* RX path */
static void eth_fmc_basic_rx_thread(void *p1, void *p2, void *p3)
{
	const struct device *dev = p1;
	const struct eth_fmc_basic_config *cfg = dev->config;
	struct eth_fmc_basic_data *data = dev->data;
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		k_sleep(K_MSEC(CONFIG_ETH_FMC_POLL_INTERVAL_MS));

		if (!is_fpga_ready()) {
			continue;
		}

		k_mutex_lock(&data->io_lock, K_FOREVER);
		uint8_t len_l = fmc_read8(cfg->base, REG_RX_LEN_L);
		uint8_t len_h = fmc_read8(cfg->base, REG_RX_LEN_H);
		uint8_t status = fmc_read8(cfg->base, REG_RX_STATUS);
		
		if (!(status & RX_STATUS_READY)) {
			if (status & RX_STATUS_OVF) {
				LOG_WRN("RX overflow detected");
				fmc_write8(cfg->base, REG_RX_STATUS, RX_STATUS_READY);
			}
			k_mutex_unlock(&data->io_lock);
			
			continue;
		}
		if (status & RX_STATUS_OVF) {
			LOG_WRN("RX overflow detected");
		}
		uint16_t pkt_len = ((uint16_t)len_h << 8) | len_l;
		
		if (pkt_len < 14 || pkt_len > ETH_FMC_MAX_PKT_SIZE + 4) {
			LOG_WRN("RX invalid len %u", pkt_len);
			fmc_write8(cfg->base, REG_RX_STATUS, RX_STATUS_READY);
			k_mutex_unlock(&data->io_lock);
			continue;
		}
		LOG_INF("RX valid len %u (low: %02x, high: %02x, status: %02x)", pkt_len, len_l, len_h, status);
		uint8_t buf[ETH_FMC_MAX_PKT_SIZE + 4];
		//LOG_DBG("Reading packet data from FMC");
		fmc_read_block(cfg->base, REG_RX_WINDOW, buf, pkt_len);
		//LOG_HEXDUMP_DBG(buf, pkt_len, "RX Packet Data");
		fmc_write8(cfg->base, REG_RX_STATUS, RX_STATUS_READY);
		//LOG_DBG("Wrote RX_STATUS_READY (%02x) to REG_RX_STATUS (%02x)", RX_STATUS_READY, REG_RX_STATUS);
		k_mutex_unlock(&data->io_lock);

		/* Drop FCS */
		if (pkt_len >= 4) {
			pkt_len -= 4;
		}

		struct net_pkt *pkt = net_pkt_rx_alloc_with_buffer(data->iface, pkt_len,
								   AF_UNSPEC, 0, K_NO_WAIT);
		if (!pkt) {
			LOG_ERR("rx alloc failed");
			continue;
		}

		if (net_pkt_write(pkt, buf, pkt_len)) {
			LOG_ERR("rx write failed");
			net_pkt_unref(pkt);
			continue;
		}

		int ret = net_recv_data(data->iface, pkt);
		if (ret < 0) {
			LOG_ERR("rx deliver err %d", ret);
			net_pkt_unref(pkt);
		}
	}
}

static int eth_fmc_basic_send(const struct device *dev, struct net_pkt *pkt)
{
	ARG_UNUSED(dev);
	/* Hand off to TX thread */
	if (k_msgq_put(&tx_queue, &pkt, K_NO_WAIT) != 0) {
		return -ENOMEM;
	}
	net_pkt_ref(pkt);
	return 0;
}

static void eth_fmc_basic_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct eth_fmc_basic_data *data = dev->data;

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

	LOG_INF("MAC %02x:%02x:%02x:%02x:%02x:%02x",
		data->mac_addr[0], data->mac_addr[1], data->mac_addr[2],
		data->mac_addr[3], data->mac_addr[4], data->mac_addr[5]);
}

static enum ethernet_hw_caps eth_fmc_basic_get_capabilities(const struct device *dev)
{
	ARG_UNUSED(dev);
	return ETHERNET_LINK_10BASE | ETHERNET_LINK_100BASE;
}

static void eth_fmc_basic_link_work(struct k_work *work)
{
	struct k_work_delayable *dwork = CONTAINER_OF(work, struct k_work_delayable, work);
	struct eth_fmc_basic_data *data = CONTAINER_OF(dwork, struct eth_fmc_basic_data, link_work);

	if (data->iface) {
		net_if_carrier_on(data->iface);
		net_if_up(data->iface);
	}
}

static const struct ethernet_api eth_fmc_basic_api = {
	.iface_api.init = eth_fmc_basic_iface_init,
	.get_capabilities = eth_fmc_basic_get_capabilities,
	.send = eth_fmc_basic_send,
};

static int eth_fmc_basic_init(const struct device *dev)
{
	struct eth_fmc_basic_data *data = dev->data;

	LOG_INF("Initializing FMC Ethernet Bridge at 0x%lx", 
		((const struct eth_fmc_basic_config *)dev->config)->base);

	if (!gpio_is_ready_dt(&fpga_ready)) {
		LOG_ERR("FPGA Ready GPIO not ready");
		return -ENODEV;
	}
	gpio_pin_configure_dt(&fpga_ready, GPIO_INPUT);

	k_mutex_init(&data->io_lock);
	k_work_init_delayable(&data->link_work, eth_fmc_basic_link_work);

	k_thread_create(&data->rx_thread, data->rx_stack,
			CONFIG_ETH_FMC_BASIC_RX_STACK_SIZE,
			eth_fmc_basic_rx_thread, (void *)dev, NULL, NULL,
			K_PRIO_COOP(20), 0, K_NO_WAIT);
	k_thread_name_set(&data->rx_thread, "eth_fmc_rx");

	k_thread_create(&data->tx_thread, data->tx_stack,
			CONFIG_ETH_FMC_BASIC_RX_STACK_SIZE,
			eth_fmc_basic_tx_thread, (void *)dev, NULL, NULL,
			K_PRIO_PREEMPT(10), 0, K_NO_WAIT);
	k_thread_name_set(&data->tx_thread, "eth_fmc_tx");

	k_work_schedule(&data->link_work, K_MSEC(50));

	return 0;
}

static struct eth_fmc_basic_data eth_fmc_basic_data_0;

static const struct eth_fmc_basic_config eth_fmc_basic_cfg_0 = {
	.base = CONFIG_ETH_FMC_BASE_ADDR,
};

ETH_NET_DEVICE_INIT(eth_fmc0, "eth_fmc0",
		    eth_fmc_basic_init, NULL,
		    &eth_fmc_basic_data_0, &eth_fmc_basic_cfg_0,
		    CONFIG_ETH_INIT_PRIORITY, &eth_fmc_basic_api,
		    NET_ETH_MTU);
