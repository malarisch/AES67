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
#include <zephyr/sys/atomic.h>
#include <string.h>

#include <zephyr/spinlock.h>

#include "eth_fmc_basic.h"

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
	struct gpio_dt_spec interrupt;
};

struct eth_fmc_basic_data {
	struct net_if *iface;
	const struct device *dev; /* Added back-pointer to device */
	uint8_t mac_addr[6];
	struct k_thread rx_thread;
	struct k_thread tx_thread;
	struct k_work_delayable link_work;
	struct k_spinlock lock;
	struct k_sem int_sem;
	struct gpio_callback gpio_cb;
	bool use_interrupt;
	atomic_t rx_wake_count;
	
	/* Shared state between ISR and Thread */
	volatile uint16_t rx_len_cached;
	volatile uint8_t rx_status_cached;
	volatile bool rx_cached_valid;
	uint8_t rx_buf[ETH_FMC_MAX_PKT_SIZE + 4];

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

/* ---- Public register access API ---- */

int eth_fmc_reg_write(const struct device *dev, uint8_t reg,
		      const uint8_t *data, size_t len)
{
	const struct eth_fmc_basic_config *cfg = dev->config;
	struct eth_fmc_basic_data *drv_data = dev->data;
	k_spinlock_key_t key;

	if (!dev || !data || len == 0) {
		return -EINVAL;
	}

	key = k_spin_lock(&drv_data->lock);
	for (size_t i = 0; i < len; i++) {
		fmc_write8(cfg->base, reg, data[i]);
	}
	k_spin_unlock(&drv_data->lock, key);

	return 0;
}

int eth_fmc_reg_read(const struct device *dev, uint8_t reg, uint8_t *val)
{
	const struct eth_fmc_basic_config *cfg = dev->config;
	struct eth_fmc_basic_data *drv_data = dev->data;
	k_spinlock_key_t key;

	if (!dev || !val) {
		return -EINVAL;
	}

	key = k_spin_lock(&drv_data->lock);
	*val = fmc_read8(cfg->base, reg);
	k_spin_unlock(&drv_data->lock, key);

	return 0;
}

int eth_fmc_reg_read_block(const struct device *dev, uint8_t reg,
			   uint8_t *data, size_t len)
{
	const struct eth_fmc_basic_config *cfg = dev->config;
	struct eth_fmc_basic_data *drv_data = dev->data;
	k_spinlock_key_t key;

	if (!dev || !data || len == 0) {
		return -EINVAL;
	}

	key = k_spin_lock(&drv_data->lock);
	for (size_t i = 0; i < len; i++) {
		data[i] = fmc_read8(cfg->base, reg);
	}
	k_spin_unlock(&drv_data->lock, key);

	return 0;
}

/* ---- Convenience helpers ---- */

/* Shadow copy of register 0x50 write value, protected by its own spinlock
 * so that multiple threads (PPB poll, BMC, shell) can safely set/clear
 * individual bits without clobbering each other. */
static uint8_t status_shadow;
static struct k_spinlock status_lock;

int eth_fmc_status_set_bits(const struct device *dev, uint8_t bits)
{
	k_spinlock_key_t key = k_spin_lock(&status_lock);

	status_shadow |= bits;
	uint8_t val = status_shadow;

	k_spin_unlock(&status_lock, key);

	return eth_fmc_reg_write(dev, ETH_FMC_REG_STATUS_WR, &val, 1);
}

int eth_fmc_status_clear_bits(const struct device *dev, uint8_t bits)
{
	k_spinlock_key_t key = k_spin_lock(&status_lock);

	status_shadow &= ~bits;
	uint8_t val = status_shadow;

	k_spin_unlock(&status_lock, key);

	return eth_fmc_reg_write(dev, ETH_FMC_REG_STATUS_WR, &val, 1);
}

int eth_fmc_write_ptp_config(const struct device *dev,
			     const uint8_t leader_clock_id[8],
			     uint8_t time_source,
			     int8_t log_msg_interval,
			     int8_t log_announce_interval)
{
	uint8_t buf[ETH_FMC_PTP_CONFIG_LEN];

	/* Bytes 0..7: Leader clock identity (EUI-64, big-endian) */
	memcpy(buf, leader_clock_id, 8);
	/* Byte 8: PTP time source */
	buf[8] = time_source;
	/* Byte 9: logMessageInterval for Sync (signed, cast to unsigned for wire) */
	buf[9] = (uint8_t)log_msg_interval;
	/* Byte 10: logAnnounceInterval (signed, cast to unsigned for wire) */
	buf[10] = (uint8_t)log_announce_interval;
	/* Byte 11: Dummy — triggers the FPGA system_config_done latch
	 * (byte_count reaches 11, which fires the done pulse that
	 *  copies RAM to output registers in system_config_reg) */
	buf[11] = 0x00;

	return eth_fmc_reg_write(dev, ETH_FMC_REG_PTP_CONFIG, buf,
				 ETH_FMC_PTP_CONFIG_LEN);
}

int eth_fmc_write_tx_stream_config(const struct device *dev,
				   uint8_t stream_id,
				   const struct in_addr *dst_ip,
				   uint8_t channel_count,
				   uint8_t samples_per_pkt,
				   const uint8_t *ch_ids,
				   uint8_t num_ch_ids)
{
	uint8_t buf[ETH_FMC_TX_STREAM_CFG_LEN];
	const uint8_t *ip = (const uint8_t *)&dst_ip->s_addr;

	memset(buf, 0, sizeof(buf));

	buf[0]  = stream_id & 0x07;
	buf[1]  = ip[0];
	buf[2]  = ip[1];
	buf[3]  = ip[2];
	buf[4]  = ip[3];
	buf[5]  = channel_count;
	buf[6]  = samples_per_pkt;

	for (uint8_t i = 0; i < num_ch_ids && i < 8; i++) {
		buf[7 + i] = ch_ids[i];
	}

	return eth_fmc_reg_write(dev, ETH_FMC_REG_TX_STREAM_CFG, buf,
				 ETH_FMC_TX_STREAM_CFG_LEN);
}

/* TX path */
static void eth_fmc_basic_tx_thread(void *p1, void *p2, void *p3)
{
	const struct device *dev = p1;
	const struct eth_fmc_basic_config *cfg = dev->config;
	struct eth_fmc_basic_data *data = dev->data;
	struct net_pkt *pkt = NULL;
	k_spinlock_key_t key;
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
		//LOG_INF("Sending Packet with lenght %u", len);

		key = k_spin_lock(&data->lock);
		fmc_write8(cfg->base, REG_TX_LEN_L, len & 0xFF);
		fmc_write8(cfg->base, REG_TX_LEN_H, (len >> 8) & 0xFF);
		fmc_write_block(cfg->base, REG_TX_WINDOW, buf, len);
		fmc_write8(cfg->base, REG_TX_CTRL, 0x00);
		fmc_write8(cfg->base, REG_TX_CTRL, 0x01);
		k_spin_unlock(&data->lock, key);

		net_pkt_unref(pkt);
	}
}

/* GPIO Interrupt Handler */
static void eth_fmc_basic_gpio_callback(const struct device *port,
                                        struct gpio_callback *cb,
                                        gpio_port_pins_t pins)
{
    struct eth_fmc_basic_data *data = 
        CONTAINER_OF(cb, struct eth_fmc_basic_data, gpio_cb);
	const struct eth_fmc_basic_config *cfg = data->dev->config;
	k_spinlock_key_t key;

    ARG_UNUSED(port);
    ARG_UNUSED(pins);

	key = k_spin_lock(&data->lock);

	/* Read status immediately to satisfy latency requirement */
	uint8_t status = fmc_read8(cfg->base, REG_RX_STATUS);
	if (status & RX_STATUS_READY) {
		uint8_t len_l = fmc_read8(cfg->base, REG_RX_LEN_L);
		uint8_t len_h = fmc_read8(cfg->base, REG_RX_LEN_H);
		uint16_t pkt_len = ((uint16_t)len_h << 8) | len_l;

		if (pkt_len <= ETH_FMC_MAX_PKT_SIZE + 4) {
			/* Read data in ISR to eliminate context switch latency gap */
			fmc_read_block(cfg->base, REG_RX_WINDOW, data->rx_buf, pkt_len);

			data->rx_status_cached = status;
			data->rx_len_cached = pkt_len;
			data->rx_cached_valid = true;
		}

		/* Clear status to acknowledge IRQ — one write covers both
		 * READY and OVF bits since the FPGA resets both on
		 * rx_clear_mac_pulse. */
		fmc_write8(cfg->base, REG_RX_STATUS, RX_STATUS_READY);

		k_sem_give(&data->int_sem);
	} else if (status & RX_STATUS_OVF) {
		/* Overflow without a ready frame — clear and wake the
		 * RX thread so it can re-poll.  Do NOT log here (ISR
		 * context) to avoid looping if the FPGA keeps the
		 * interrupt asserted across the CDC delay. */
		fmc_write8(cfg->base, REG_RX_STATUS, RX_STATUS_READY);
		k_sem_give(&data->int_sem);
	}	
	
	k_spin_unlock(&data->lock, key);
}

/* RX path */
static void eth_fmc_basic_rx_thread(void *p1, void *p2, void *p3)
{
	const struct device *dev = p1;
	const struct eth_fmc_basic_config *cfg = dev->config;
	struct eth_fmc_basic_data *data = dev->data;
	k_spinlock_key_t key;
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		if (data->use_interrupt) {
			k_sem_take(&data->int_sem, K_FOREVER);
			(void)atomic_inc(&data->rx_wake_count);
		} else {
			k_sleep(K_MSEC(CONFIG_ETH_FMC_POLL_INTERVAL_MS));
			if (!is_fpga_ready()) {
				continue;
			}
		}

		/* Loop while interrupt pin is active or first pass.
		 * Limit iterations to avoid starving other threads if
		 * the FPGA keeps asserting the interrupt (e.g. during
		 * sustained overflow). */
		int burst = 0;
		const int max_burst = 16;
		bool first_pass = true;
		while ((first_pass || (data->use_interrupt && gpio_pin_get_dt(&cfg->interrupt) == 1))
		       && burst < max_burst) {
			first_pass = false;
			burst++;

			uint16_t pkt_len;
			uint8_t status;
			bool from_isr = false;

			key = k_spin_lock(&data->lock);

			/* Check if we have cached data from ISR */
			if (data->rx_cached_valid) {
				pkt_len = data->rx_len_cached;
				status = data->rx_status_cached;
				data->rx_cached_valid = false;
				from_isr = true;
			} else {
				/* Polling or subsequent packet in burst */
				uint8_t len_l = fmc_read8(cfg->base, REG_RX_LEN_L);
				uint8_t len_h = fmc_read8(cfg->base, REG_RX_LEN_H);
				status = fmc_read8(cfg->base, REG_RX_STATUS);
				pkt_len = ((uint16_t)len_h << 8) | len_l;
			}

			if (!from_isr && !(status & RX_STATUS_READY)) {
				if (status & RX_STATUS_OVF) {
					LOG_WRN("RX overflow detected (poll)");
					fmc_write8(cfg->base, REG_RX_STATUS, RX_STATUS_READY);
					/* Give the FPGA a few µs for the CDC
					 * clear to propagate before re-reading */
					k_spin_unlock(&data->lock, key);
					k_busy_wait(5);
					continue;
				}
				k_spin_unlock(&data->lock, key);
				break;
			}
			if (status & RX_STATUS_OVF) {
				LOG_WRN("RX overflow (frame also ready)");
			}
			
			if (pkt_len < 14 || pkt_len > ETH_FMC_MAX_PKT_SIZE + 4) {
				if (!from_isr) {
					LOG_WRN("RX invalid len %u", pkt_len);
					fmc_write8(cfg->base, REG_RX_STATUS, RX_STATUS_READY);
				}
				k_spin_unlock(&data->lock, key);
				continue;
			}

			/* If not from ISR, we need to read the data now */
			if (!from_isr) {
				fmc_read_block(cfg->base, REG_RX_WINDOW, data->rx_buf, pkt_len);
				fmc_write8(cfg->base, REG_RX_STATUS, RX_STATUS_READY);
			}
			
			k_spin_unlock(&data->lock, key);

			/* Drop FCS */
			if (pkt_len >= 4) {
				pkt_len -= 4;
			}

			struct net_pkt *pkt = net_pkt_rx_alloc_with_buffer(data->iface, pkt_len,
									AF_UNSPEC, 0, K_NO_WAIT);


			//LOG_HEXDUMP_INF(data->rx_buf, pkt_len, "Received Packet:");
			if (!pkt) {
				LOG_ERR("rx alloc failed");
				continue;
			}

			if (net_pkt_write(pkt, data->rx_buf, pkt_len)) {
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
	/* Clear RX status from overflows happening while booting up */
}

static const struct ethernet_api eth_fmc_basic_api = {
	.iface_api.init = eth_fmc_basic_iface_init,
	.get_capabilities = eth_fmc_basic_get_capabilities,
	.send = eth_fmc_basic_send,
};

static int eth_fmc_basic_init(const struct device *dev)
{
	struct eth_fmc_basic_data *data = dev->data;
	const struct eth_fmc_basic_config *cfg = dev->config;
	int ret;

	data->dev = dev; /* Store device pointer for ISR access */

	LOG_INF("Initializing FMC Ethernet Bridge at 0x%lx", 
		cfg->base);

	if (!gpio_is_ready_dt(&fpga_ready)) {
		LOG_ERR("FPGA Ready GPIO not ready");
		return -ENODEV;
	}
	gpio_pin_configure_dt(&fpga_ready, GPIO_INPUT);

	//k_mutex_init(&data->io_lock);
	k_sem_init(&data->int_sem, 0, 1);
	//atomic_clear(&data->irq_count);
	atomic_clear(&data->rx_wake_count);
	data->use_interrupt = false;
	data->rx_cached_valid = false;

	if (cfg->interrupt.port) {
		if (!gpio_is_ready_dt(&cfg->interrupt)) {
			LOG_WRN("GPIO interrupt not ready, falling back to polling");
		} else {
			ret = gpio_pin_configure_dt(&cfg->interrupt, GPIO_INPUT);
			if (ret != 0) {
				LOG_ERR("Failed to configure GPIO interrupt (%d)", ret);
				return ret;
			}

			gpio_init_callback(&data->gpio_cb, eth_fmc_basic_gpio_callback,
							  BIT(cfg->interrupt.pin));

			ret = gpio_add_callback(cfg->interrupt.port, &data->gpio_cb);
			if (ret != 0) {
				LOG_ERR("Failed to add GPIO callback (%d)", ret);
				return ret;
			}

			ret = gpio_pin_interrupt_configure_dt(&cfg->interrupt,
												 GPIO_INT_EDGE_TO_ACTIVE);
			if (ret != 0) {
				LOG_ERR("Failed to configure GPIO interrupt (%d)", ret);
				return ret;
			}

			data->use_interrupt = true;
			LOG_INF("Using IRQ on %s pin %d",
					cfg->interrupt.port->name,
					cfg->interrupt.pin);
		}
		

	}

	k_work_init_delayable(&data->link_work, eth_fmc_basic_link_work);

	k_thread_create(&data->rx_thread, data->rx_stack,
			CONFIG_ETH_FMC_BASIC_RX_STACK_SIZE,
			eth_fmc_basic_rx_thread, (void *)dev, NULL, NULL,
			K_PRIO_COOP(1), 0, K_NO_WAIT);
	k_thread_name_set(&data->rx_thread, "eth_fmc_rx");

	k_thread_create(&data->tx_thread, data->tx_stack,
			CONFIG_ETH_FMC_BASIC_RX_STACK_SIZE,
			eth_fmc_basic_tx_thread, (void *)dev, NULL, NULL,
			K_PRIO_PREEMPT(2), 0, K_NO_WAIT);
	k_thread_name_set(&data->tx_thread, "eth_fmc_tx");

	k_work_schedule(&data->link_work, K_MSEC(50));
		LOG_INF("Clearing RX status for Startup");
		fmc_write8(cfg->base, REG_RX_STATUS, RX_STATUS_READY);
	
	return 0;
}

static struct eth_fmc_basic_data eth_fmc_basic_data_0;

static const struct eth_fmc_basic_config eth_fmc_basic_cfg_0 = {
	.base = CONFIG_ETH_FMC_BASE_ADDR,
	.interrupt = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(fpga_int), gpios, {0}),
};

ETH_NET_DEVICE_INIT(eth_fmc0, "eth_fmc0",
		    eth_fmc_basic_init, NULL,
		    &eth_fmc_basic_data_0, &eth_fmc_basic_cfg_0,
		    CONFIG_ETH_INIT_PRIORITY, &eth_fmc_basic_api,
		    NET_ETH_MTU);
