/*
 * SPI Ethernet bridge driver for the FPGA MAC interface
 */

#define DT_DRV_COMPAT vendor_spi_ethernet

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/ptp_time.h>
#include <zephyr/drivers/ptp_clock.h>
#include <zephyr/random/random.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/linker/sections.h>
#include <stdbool.h>

/* STM32 EXTI low-level diagnostics */
#include <stm32_ll_exti.h>
#include <zephyr/drivers/interrupt_controller/gpio_intc_stm32.h>

#include "eth_spi_basic.h"

LOG_MODULE_REGISTER(eth_spi_basic, CONFIG_ETHERNET_LOG_LEVEL);

/* Register map of the FPGA SPI client */
#define REG_TX_LEN_L      0x00
#define REG_TX_LEN_H      0x01
#define REG_TX_CTRL       0x02
#define REG_TX_WINDOW     0x10  /* 0x10.. auto-increment */

#define REG_RX_LEN_L      0x20
#define REG_RX_LEN_H      0x21
#define REG_RX_STATUS     0x22
#define REG_RX_WINDOW     0x30  /* 0x30.. auto-increment */

#define REG_PTP_TIME_L    0x40
#define REG_PTP_TIME_H    0x41 /* ... 0x47 */
#define REG_TX_TS_L       0x50
#define REG_RX_TS_L       0x60
#define REG_PTP_INC       0x70

#define RX_STATUS_READY   BIT(0)
#define RX_STATUS_OVF     BIT(1)

/* Helper to build the first SPI byte */
#define SPI_CMD_WRITE(addr7)   (0x80 | ((addr7) & 0x7F))
#define SPI_CMD_READ(addr7)    ((addr7) & 0x7F)

/* Device Datenstruktur */
struct eth_spi_basic_data {
    struct net_if *iface;
    uint8_t mac_addr[6];
    struct k_sem int_sem;
    struct k_mutex spi_lock; /* Protects shared SPI buffers */
    struct gpio_callback gpio_cb;
    struct k_thread rx_thread;
    struct k_thread tx_thread;
    bool use_interrupt;
    atomic_t irq_count;
    atomic_t rx_wake_count;
    uint8_t rx_buf[ETH_SPI_BASIC_MAX_PKT_SIZE];
    struct k_work_delayable link_work;
    const struct device *ptp_clock;
    K_KERNEL_STACK_MEMBER(rx_stack, CONFIG_ETH_SPI_BASIC_RX_STACK_SIZE);
    K_KERNEL_STACK_MEMBER(tx_stack, CONFIG_ETH_SPI_BASIC_RX_STACK_SIZE);
};

/* Device Konfiguration */
struct eth_spi_basic_config {
    struct spi_dt_spec spi;
    struct gpio_dt_spec interrupt;
    struct gpio_dt_spec cs_gpio;
    bool has_cs_gpio;
    uint8_t full_duplex;
};

/* SPI Hilfsfunktionen */
/* Globale TX/RX-Puffer, um Stack zu sparen und den ursprünglichen Burst-Stil beizubehalten. */
/* STM32H7 requires 32-byte alignment for DMA buffers (D-Cache) */
#define SPI_BUF_ALIGNMENT 32
#define SPI_BUF_SIZE      ROUND_UP(ETH_SPI_BASIC_MAX_PKT_SIZE + 1, SPI_BUF_ALIGNMENT)

/* TX Pipeline Configuration */
#define TX_QUEUE_SIZE 3
struct tx_msg {
    uint8_t *buf;
    size_t len;
    struct net_pkt *pkt;
};

static struct k_mem_slab tx_slab;
static uint8_t __aligned(SPI_BUF_ALIGNMENT) __nocache tx_slab_buffer[TX_QUEUE_SIZE * SPI_BUF_SIZE];

K_MSGQ_DEFINE(tx_queue, sizeof(struct tx_msg), TX_QUEUE_SIZE, 4);

static uint8_t __aligned(SPI_BUF_ALIGNMENT) __nocache spi_tx_buf[SPI_BUF_SIZE];
static uint8_t __aligned(SPI_BUF_ALIGNMENT) __nocache spi_rx_buf[SPI_BUF_SIZE];

static int spi_basic_read_bytes_internal(const struct device *dev, uint8_t addr,
                                uint8_t *buf, size_t len)
{
    const struct eth_spi_basic_config *cfg = dev->config;
    /* Assumes mutex is already locked! */

    if (len > ETH_SPI_BASIC_MAX_PKT_SIZE) {
        return -EMSGSIZE;
    }

    spi_tx_buf[0] = SPI_CMD_READ(addr & 0x7F);
    memset(&spi_tx_buf[1], 0, len);

    struct spi_buf tx = {
        .buf = spi_tx_buf,
        .len = len + 1,
    };
    struct spi_buf rx = {
        .buf = spi_rx_buf,
        .len = len + 1,
    };
    const struct spi_buf_set tx_set = {
        .buffers = &tx,
        .count = 1,
    };
    const struct spi_buf_set rx_set = {
        .buffers = &rx,
        .count = 1,
    };

    int ret = spi_transceive_dt(&cfg->spi, &tx_set, &rx_set);
    if (ret == 0) {
        memcpy(buf, &spi_rx_buf[1], len); /* Byte 0 ist Echo des CMD */
    }
    
    return ret;
}

static int spi_basic_write_bytes_internal(const struct device *dev, uint8_t addr,
                                 const uint8_t *buf, size_t len)
{
    const struct eth_spi_basic_config *cfg = dev->config;
    /* Assumes mutex is already locked! */

    if (len > ETH_SPI_BASIC_MAX_PKT_SIZE) {
        return -EMSGSIZE;
    }

    spi_tx_buf[0] = SPI_CMD_WRITE(addr & 0x7F);
    memcpy(&spi_tx_buf[1], buf, len);

    const struct spi_buf tx = {
        .buf = spi_tx_buf,
        .len = len + 1,
    };
    const struct spi_buf_set tx_set = {
        .buffers = &tx,
        .count = 1,
    };

    return spi_write_dt(&cfg->spi, &tx_set);
}

static int spi_basic_read_bytes(const struct device *dev, uint8_t addr,
                                uint8_t *buf, size_t len)
{
    struct eth_spi_basic_data *data = dev->data;
    k_mutex_lock(&data->spi_lock, K_FOREVER);
    int ret = spi_basic_read_bytes_internal(dev, addr, buf, len);
    k_mutex_unlock(&data->spi_lock);
    return ret;
}

static int spi_basic_write_bytes(const struct device *dev, uint8_t addr,
                                 const uint8_t *buf, size_t len)
{
    struct eth_spi_basic_data *data = dev->data;
    k_mutex_lock(&data->spi_lock, K_FOREVER);
    int ret = spi_basic_write_bytes_internal(dev, addr, buf, len);
    k_mutex_unlock(&data->spi_lock);
    return ret;
}

static int spi_basic_read_reg(const struct device *dev, uint8_t addr, uint8_t *val)
{
    //LOG_DBG("rr a%02X v%02X", addr, *val);
    return spi_basic_read_bytes(dev, addr, val, 1);
}

static int spi_basic_write_reg(const struct device *dev, uint8_t addr, uint8_t val)
{
    //LOG_DBG("wr a=0x%02X v=0x%02X", addr, val);
    return spi_basic_write_bytes(dev, addr, &val, 1);
}



/* PTP Helper Functions */
static int eth_spi_ptp_read_time(const struct device *dev, uint64_t *ns)
{
    uint8_t buf[8];
    int ret = spi_basic_read_bytes(dev, REG_PTP_TIME_L, buf, 8);
    if (ret == 0) {
        *ns = 0;
        for (int i = 0; i < 8; i++) {
            *ns |= ((uint64_t)buf[i] << (i * 8));
        }
    }
    return ret;
}

static int eth_spi_ptp_write_time(const struct device *dev, uint64_t ns)
{
    uint8_t buf[8];
    for (int i = 0; i < 8; i++) {
        buf[i] = (ns >> (i * 8)) & 0xFF;
    }
    return spi_basic_write_bytes(dev, REG_PTP_TIME_L, buf, 8);
}

/* PTP Clock API Implementation */
static int eth_spi_ptp_set(const struct device *dev, struct net_ptp_time *tm)
{
    struct eth_spi_basic_data *data = dev->data;
    uint64_t ns = tm->second * NSEC_PER_SEC + tm->nanosecond;
    return eth_spi_ptp_write_time(net_if_get_device(data->iface), ns);
}

static int eth_spi_ptp_get(const struct device *dev, struct net_ptp_time *tm)
{
    struct eth_spi_basic_data *data = dev->data;
    uint64_t ns;
    int ret = eth_spi_ptp_read_time(net_if_get_device(data->iface), &ns);
    if (ret == 0) {
        tm->second = ns / NSEC_PER_SEC;
        tm->nanosecond = ns % NSEC_PER_SEC;
    }
    return ret;
}

static int eth_spi_ptp_adjust(const struct device *dev, int increment)
{
    struct eth_spi_basic_data *data = dev->data;
    uint64_t ns;
    int ret = eth_spi_ptp_read_time(net_if_get_device(data->iface), &ns);
    if (ret == 0) {
        if (increment > 0) {
            ns += increment;
        } else {
            ns -= (-increment);
        }
        ret = eth_spi_ptp_write_time(net_if_get_device(data->iface), ns);
    }
    return ret;
}

static int eth_spi_ptp_rate_adjust(const struct device *dev, double ratio)
{
    struct eth_spi_basic_data *data = dev->data;
    /* Default increment is 4ns (250 MHz). ratio is e.g. 1.00001 */
    /* We use 8.24 fixed point increment. */
    /* Base increment = 4.0 */
    /* New increment = 4.0 * ratio */
    
    double inc_f = 4.0 * ratio;
    uint32_t inc_fixed = (uint32_t)(inc_f * (double)(1 << 24));
    
    uint8_t buf[4];
    buf[0] = inc_fixed & 0xFF;
    buf[1] = (inc_fixed >> 8) & 0xFF;
    buf[2] = (inc_fixed >> 16) & 0xFF;
    buf[3] = (inc_fixed >> 24) & 0xFF;
    
    return spi_basic_write_bytes(net_if_get_device(data->iface), REG_PTP_INC, buf, 4);
}

static const struct ptp_clock_driver_api eth_spi_ptp_api = {
    .set = eth_spi_ptp_set,
    .get = eth_spi_ptp_get,
    .adjust = eth_spi_ptp_adjust,
    .rate_adjust = eth_spi_ptp_rate_adjust,
};

/* TX Thread - sendet Pakete aus der Queue */
static void eth_spi_basic_tx_thread(void *p1, void *p2, void *p3)
{
    const struct device *dev = p1;
    struct eth_spi_basic_data *data = dev->data;
    const struct eth_spi_basic_config *cfg = dev->config;
    struct tx_msg msg;
    int ret;

    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    while (1) {
        /* Wait for packet in queue */
        k_msgq_get(&tx_queue, &msg, K_FOREVER);

        /* 1. Write TX Length */
        uint8_t len_bytes[2] = { msg.len & 0xff, msg.len >> 8 };
        ret = spi_basic_write_bytes(dev, REG_TX_LEN_L, len_bytes, sizeof(len_bytes));
        if (ret) {
            LOG_ERR("Failed to write TX length: %d", ret);
            goto tx_done;
        }

        /* 2. Write Payload */
        /* msg.buf is aligned. msg.buf[0] is CMD. msg.buf[1...] is Payload. */
        msg.buf[0] = SPI_CMD_WRITE(REG_TX_WINDOW);
        
        struct spi_buf tx_buf_struct = {
            .buf = msg.buf,
            .len = msg.len + 1,
        };
        struct spi_buf_set tx_set = {
            .buffers = &tx_buf_struct,
            .count = 1,
        };

        k_mutex_lock(&data->spi_lock, K_FOREVER);
        ret = spi_write_dt(&cfg->spi, &tx_set);
        k_mutex_unlock(&data->spi_lock);

        if (ret) {
            LOG_ERR("Failed to write TX payload: %d", ret);
            goto tx_done;
        }

        /* 3. Trigger TX */
        /* Toggle TX start (write 0 then 1 to ensure a rising edge) */
        (void)spi_basic_write_reg(dev, REG_TX_CTRL, 0x00);
        ret = spi_basic_write_reg(dev, REG_TX_CTRL, 0x01);
        if (ret) {
            LOG_ERR("Failed to trigger TX: %d", ret);
        }
        
        /* Timestamping */
        if (msg.pkt && data->ptp_clock) {
            /* Wait a bit for FPGA to latch timestamp (it happens on start pulse) */
            /* Since we are in a thread, we can yield or busy wait. SPI access takes time anyway. */
            /* Read TX Timestamp */
            uint8_t ts_buf[8];
            ret = spi_basic_read_bytes(dev, REG_TX_TS_L, ts_buf, 8);
            if (ret == 0) {
                uint64_t ns = 0;
                for (int i = 0; i < 8; i++) {
                    ns |= ((uint64_t)ts_buf[i] << (i * 8));
                }
                struct net_ptp_time ptp_ts;
                ptp_ts.second = ns / NSEC_PER_SEC;
                ptp_ts.nanosecond = ns % NSEC_PER_SEC;
                net_pkt_set_timestamp(msg.pkt, &ptp_ts);
                net_if_add_tx_timestamp(msg.pkt);
            }
        }

tx_done:
        /* Free buffer back to slab */
        k_mem_slab_free(&tx_slab, (void *)msg.buf);
        if (msg.pkt) {
            net_pkt_unref(msg.pkt);
        }
    }
}

/* Ethernet Packet senden */
static int eth_spi_basic_tx(const struct device *dev, struct net_pkt *pkt)
{
    size_t pkt_len = net_pkt_get_len(pkt);
    struct tx_msg msg;
    int ret;
    
    LOG_DBG("TX packet, len=%zu", pkt_len);
    
    if (pkt_len > ETH_SPI_BASIC_MAX_PKT_SIZE) {
        LOG_ERR("Packet too large: %zu", pkt_len);
        return -EMSGSIZE;
    }

    /* Allocate buffer from slab */
    ret = k_mem_slab_alloc(&tx_slab, (void **)&msg.buf, K_NO_WAIT);
    if (ret) {
        LOG_ERR("TX queue full, dropping packet");
        return -ENOMEM;
    }

    msg.len = pkt_len;
    msg.pkt = NULL;

    /* Copy payload to offset 1 (offset 0 is for CMD) */
    if (net_pkt_read(pkt, &msg.buf[1], pkt_len) < 0) {
        LOG_ERR("Failed to read packet data");
        k_mem_slab_free(&tx_slab, (void *)msg.buf);
        return -EIO;
    }
    
    /* Check if timestamp is required */
    /* We need to keep the packet if timestamping is enabled for this packet */
    /* How to check? net_pkt_timestamp(pkt) returns the timestamp struct, but we need to know if we should capture it. */
    /* Usually the stack sets a flag or callback. */
    /* net_if_add_tx_timestamp checks if there is a callback. */
    /* We can just ref it always if PTP is enabled, or check atomic flag on pkt? */
    /* For simplicity, let's ref it. */
    struct eth_spi_basic_data *data = dev->data;
    if (data->ptp_clock) {
         msg.pkt = net_pkt_ref(pkt);
    }

    /* Queue the message */
    k_msgq_put(&tx_queue, &msg, K_NO_WAIT);
    
    return 0;
}

/* RX Thread - empfängt Pakete */
static void eth_spi_basic_rx_thread(void *p1, void *p2, void *p3)
{
    const struct device *dev = p1;
    struct eth_spi_basic_data *data = dev->data;
    const struct eth_spi_basic_config *cfg = dev->config;
    struct net_pkt *pkt;
    uint8_t status_buf[3];
    uint8_t status;
    uint16_t pkt_len;
    int ret;
    uint8_t clear_val = 0x01;
    
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);
    
    while (1) {
        /* Wait for an interrupt (or periodically wake as a safety net), then
         * DRAIN all pending RX packets.
         *
         * Important for your IRQ semantics (mode 2): IRQ is asserted while RX
         * is pending and only deasserts when we clear RX. That means we MUST
         * drain until RX_STATUS_READY is 0 before going back to sleep,
         * otherwise we can miss edges and throttle.
         */
        if (data->use_interrupt) {
            /* Wait for interrupt (Edge) */
            k_sem_take(&data->int_sem, K_FOREVER);
            (void)atomic_inc(&data->rx_wake_count);
        } else {
            k_sleep(K_MSEC(1));
        }

        /* Loop while the line is active (Level) to handle missed edges or stuck lines */
        bool first_pass = true;
        /* Note: gpio_pin_get_dt returns logical level (1=Active). 
         * For Button (Active High): 1=Pressed.
         * For FPGA (Active Low): 1=Low (Asserted).
         */
        while (first_pass || (data->use_interrupt && gpio_pin_get_dt(&cfg->interrupt) == 1)) {
            first_pass = false;
            /* OPTIMIZATION: Lock Mutex ONCE for the SPI sequence */
            k_mutex_lock(&data->spi_lock, K_FOREVER);

            /* 1. Read Length (L, H) and Status. Assumes REG_RX_LEN_L=0x20, contiguous */
            ret = spi_basic_read_bytes_internal(dev, REG_RX_LEN_L, status_buf, 3);
            if (ret < 0) {
                k_mutex_unlock(&data->spi_lock);
                break;
            }

            pkt_len = ((uint16_t)status_buf[0] | ((uint16_t)status_buf[1] << 8));
            status = status_buf[2];

            if (!(status & RX_STATUS_READY)) {
                if (status & RX_STATUS_OVF) {
                    spi_basic_write_bytes_internal(dev, REG_RX_STATUS, &clear_val, 1);
                }
                k_mutex_unlock(&data->spi_lock);
                break; /* nothing more to drain */
            }

            if (pkt_len == 0 || pkt_len > ETH_SPI_BASIC_MAX_PKT_SIZE) {
                spi_basic_write_bytes_internal(dev, REG_RX_STATUS, &clear_val, 1);
                k_mutex_unlock(&data->spi_lock);
                continue;
            }

            /* 2. Read Payload */
            if (spi_basic_read_bytes_internal(dev, REG_RX_WINDOW, data->rx_buf, pkt_len)) {
                spi_basic_write_bytes_internal(dev, REG_RX_STATUS, &clear_val, 1);
                k_mutex_unlock(&data->spi_lock);
                continue;
            }
            
            /* 3. Read RX Timestamp */
            struct net_ptp_time rx_ts = {0};
            bool has_ts = false;
            if (data->ptp_clock) {
                uint8_t ts_buf[8];
                if (spi_basic_read_bytes_internal(dev, REG_RX_TS_L, ts_buf, 8) == 0) {
                    uint64_t ns = 0;
                    for (int i = 0; i < 8; i++) {
                        ns |= ((uint64_t)ts_buf[i] << (i * 8));
                    }
                    rx_ts.second = ns / NSEC_PER_SEC;
                    rx_ts.nanosecond = ns % NSEC_PER_SEC;
                    has_ts = true;
                }
            }

            /* 4. Clear RX Ready (IRQ deasserts on clear in your design) */
            spi_basic_write_bytes_internal(dev, REG_RX_STATUS, &clear_val, 1);

            /* Unlock Mutex - SPI bus is free now */
            k_mutex_unlock(&data->spi_lock);

            /* 5. Process Packet (Outside Lock) */
        
        /* Debug: Ziel-MAC prüfen */
        bool dst_broadcast = true;
        bool dst_match = true;
        for (int i = 0; i < 6; i++) {
            if (data->rx_buf[i] != 0xFF) {
                dst_broadcast = false;
            }
            if (data->rx_buf[i] != data->mac_addr[i]) {
                dst_match = false;
            }
        }
        /* uint16_t eth_type = ((uint16_t)data->rx_buf[12] << 8) | data->rx_buf[13]; */
        if (!dst_broadcast && !dst_match) {
            /* Manche Gegenstellen antworten auf die im MAC-Core konfigurierte Adresse.
             * Damit der Zephyr-Stack das Paket akzeptiert, schreibe die Zieladresse
             * auf unsere Interface-MAC um.
             */
            memcpy(data->rx_buf, data->mac_addr, 6);
        }

        /* FPGA liefert das komplette Ethernet-Frame inkl. 4-Byte-FCS.
         * Zephyr erwartet Frames ohne FCS -> 4 Bytes abziehen, aber nur wenn genügend lang.
         */
        uint16_t payload_len = pkt_len;
        if (payload_len >= 4U) {
            payload_len -= 4U;
        } else {
            continue;
        }
        /* Filter offensichtlichen Müll: kleiner als Ethernet-Header */
        if (payload_len < 14U) {
            continue;
        }

        pkt = net_pkt_rx_alloc_with_buffer(data->iface, payload_len,
                                           AF_UNSPEC, 0, K_NO_WAIT);
        if (!pkt) {
            continue;
        }
        
        if (has_ts) {
            net_pkt_set_timestamp(pkt, &rx_ts);
        }
        
        if (net_pkt_write(pkt, data->rx_buf, payload_len)) {
            net_pkt_unref(pkt);
            continue;
        }

        if (net_recv_data(data->iface, pkt) < 0) {
            net_pkt_unref(pkt);
        }

        if (status & RX_STATUS_OVF) {
            /* LOG_WRN("RX overflow flagged"); */
        }

            /* Continue draining in case another packet became ready before/after clear */
        }
        /* End of while(first_pass || ...) loop */
    }
}

/* GPIO Interrupt Handler */
static void eth_spi_basic_gpio_callback(const struct device *port,
                                        struct gpio_callback *cb,
                                        gpio_port_pins_t pins)
{
//    printk("eth_spi_basic IRQ\n");
    struct eth_spi_basic_data *data = 
        CONTAINER_OF(cb, struct eth_spi_basic_data, gpio_cb);

    ARG_UNUSED(port);
    ARG_UNUSED(pins);

    (void)atomic_inc(&data->irq_count);
    
    k_sem_give(&data->int_sem);
}

/* Ethernet Interface initialisieren */
static void eth_spi_basic_iface_init(struct net_if *iface)
{
    const struct device *dev = net_if_get_device(iface);
    struct eth_spi_basic_data *data = dev->data;
    
    data->iface = iface;
    ethernet_init(iface);
    data->mac_addr[0] = 0x00; /* Locally administered */
    data->mac_addr[1] = 0x1C;
    data->mac_addr[2] = 0x23;
    data->mac_addr[3] = 0x17;
    data->mac_addr[4] = 0x4A;
    data->mac_addr[5] = 0xCC;
    /* MAC Adresse setzen */
    net_if_set_link_addr(iface, data->mac_addr, sizeof(data->mac_addr),
                        NET_LINK_ETHERNET);

    LOG_INF("MAC: %02x:%02x:%02x:%02x:%02x:%02x",
            data->mac_addr[0], data->mac_addr[1],
            data->mac_addr[2], data->mac_addr[3],
            data->mac_addr[4], data->mac_addr[5]);
}

/* Capabilities */
static enum ethernet_hw_caps eth_spi_basic_get_capabilities(const struct device *dev)
{
    ARG_UNUSED(dev);
    return ETHERNET_LINK_10BASE | ETHERNET_LINK_100BASE | ETHERNET_PTP;
}

static const struct device *eth_spi_basic_get_ptp_clock(const struct device *dev)
{
    struct eth_spi_basic_data *data = dev->data;
    return data->ptp_clock;
}

static void eth_spi_basic_link_work_handler(struct k_work *work)
{
    struct k_work_delayable *dwork = CONTAINER_OF(work, struct k_work_delayable, work);
    struct eth_spi_basic_data *data = CONTAINER_OF(dwork, struct eth_spi_basic_data, link_work);

    if (data->iface) {
        net_if_carrier_on(data->iface);
        net_if_up(data->iface);
    }
}

/* Ethernet API */
static const struct ethernet_api eth_spi_basic_api = {
    .iface_api.init = eth_spi_basic_iface_init,
    .get_capabilities = eth_spi_basic_get_capabilities,
    .send = eth_spi_basic_tx,
    .get_ptp_clock = eth_spi_basic_get_ptp_clock,
};

/* Device Initialisierung */
static int eth_spi_basic_init(const struct device *dev)
{
    struct eth_spi_basic_data *data = dev->data;
    const struct eth_spi_basic_config *cfg = dev->config;
    int ret;
    
    if (!spi_is_ready_dt(&cfg->spi)) {
        LOG_ERR("SPI device not ready");
        return -ENODEV;
    }

    /* Semaphoren/Mutex initialisieren (must be ready before enabling IRQ) */
    k_sem_init(&data->int_sem, 0, 1);
    k_mutex_init(&data->spi_lock);
    atomic_clear(&data->irq_count);
    atomic_clear(&data->rx_wake_count);

    data->use_interrupt = false;

    if (cfg->interrupt.port) {
        if (!gpio_is_ready_dt(&cfg->interrupt)) {
            LOG_WRN("GPIO interrupt not ready, falling back to polling");
        } else {
            ret = gpio_pin_configure_dt(&cfg->interrupt, GPIO_INPUT);
            if (ret != 0) {
                LOG_ERR("Failed to configure GPIO interrupt (%d)", ret);
                return ret;
            }

            /* Sanity-check: read the pin level immediately after configuring it.
             * If this never changes even when you hard-drive the pin, we are
             * very likely on the wrong physical pin / wrong mapping.
             */
            
                int level_active = gpio_pin_get_dt(&cfg->interrupt);
                int level_raw = gpio_pin_get_raw(cfg->interrupt.port, cfg->interrupt.pin);
                printk("eth_spi_basic: IRQ %s pin %d dt_flags=0x%x level(active)=%d level(raw)=%d\n",
                       cfg->interrupt.port->name,
                       cfg->interrupt.pin,
                       cfg->interrupt.dt_flags,
                       level_active,
                       level_raw);
            

            gpio_init_callback(&data->gpio_cb, eth_spi_basic_gpio_callback,
                              BIT(cfg->interrupt.pin));

            ret = gpio_add_callback(cfg->interrupt.port, &data->gpio_cb);
            if (ret != 0) {
                LOG_ERR("Failed to add GPIO callback (%d)", ret);
                return ret;
            }
            printk("eth_spi_basic: gpio_add_callback ok\n");

            /* Debug: trigger on both edges so we catch any transition while
             * bringing up the EXTI/IRQ path.
             */
            ret = gpio_pin_interrupt_configure_dt(&cfg->interrupt,
                                                 GPIO_INT_EDGE_BOTH);
            if (ret != 0) {
                LOG_ERR("Failed to configure GPIO interrupt (%d)", ret);
                return ret;
            }
            printk("eth_spi_basic: gpio_pin_interrupt_configure_dt ok\n");

            data->use_interrupt = true;
            LOG_INF("Using IRQ on %s pin %d (active %s)",
                    cfg->interrupt.port->name,
                    cfg->interrupt.pin,
                    (cfg->interrupt.dt_flags & GPIO_ACTIVE_LOW) ? "low" : "high");
        }
    }

    /* Initialize TX Slab in nocache memory */
    k_mem_slab_init(&tx_slab, tx_slab_buffer, SPI_BUF_SIZE, TX_QUEUE_SIZE);

    /* Link bring-up verzögert, damit Net-Stack vollständig steht. */
    k_work_init_delayable(&data->link_work, eth_spi_basic_link_work_handler);
    
    /* MAC Adresse vom Chip lesen oder generieren */
    //sys_rand_get(data->mac_addr, sizeof(data->mac_addr));
    data->mac_addr[0] = 0x00; /* Locally administered */
    data->mac_addr[1] = 0x1C;
    data->mac_addr[2] = 0x23;
    data->mac_addr[3] = 0x17;
    data->mac_addr[4] = 0x4A;
    data->mac_addr[5] = 0xCB;
    
    /* RX Thread starten */
    k_thread_create(&data->rx_thread, data->rx_stack,
                   CONFIG_ETH_SPI_BASIC_RX_STACK_SIZE,
                   eth_spi_basic_rx_thread,
                   (void *)dev, NULL, NULL,
                   K_PRIO_COOP(2), 0, K_NO_WAIT);
    
    k_thread_name_set(&data->rx_thread, "eth_spi_rx");

    /* TX Thread starten */
    k_thread_create(&data->tx_thread, data->tx_stack,
                   CONFIG_ETH_SPI_BASIC_RX_STACK_SIZE,
                   eth_spi_basic_tx_thread,
                   (void *)dev, NULL, NULL,
                   K_PRIO_COOP(2), 0, K_NO_WAIT);
    
    k_thread_name_set(&data->tx_thread, "eth_spi_tx");
    
    LOG_INF("SPI Ethernet initialized");

    /* Link später hochziehen. */
    k_work_schedule(&data->link_work, K_MSEC(50));
    
    return 0;
}

/* PTP Device Definition */
#define ETH_SPI_PTP_DEFINE(n) \
    DEVICE_DEFINE(eth_spi_ptp_##n, "eth_spi_ptp_" #n, \
                  NULL, NULL, \
                  &eth_spi_basic_data_##n, NULL, \
                  POST_KERNEL, CONFIG_ETH_INIT_PRIORITY, &eth_spi_ptp_api);

/* Device Makro */
#define ETH_SPI_BASIC_INIT(n)                                           \
    static struct eth_spi_basic_data eth_spi_basic_data_##n;            \
    ETH_SPI_PTP_DEFINE(n)                                               \
    static struct eth_spi_basic_data eth_spi_basic_data_##n = {         \
        .ptp_clock = DEVICE_GET(eth_spi_ptp_##n),                       \
    };                                                                  \
                                                                        \
    static const struct eth_spi_basic_config eth_spi_basic_config_##n = { \
        .spi = SPI_DT_SPEC_INST_GET(n,                                  \
            SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_TRANSFER_MSB,    \
            0),                                                         \
        .interrupt = GPIO_DT_SPEC_INST_GET(n, int_gpios),              \
        .full_duplex = DT_INST_PROP(n, full_duplex),                   \
        .cs_gpio = GPIO_DT_SPEC_INST_GET_OR(n, cs_gpios, {0}),         \
        .has_cs_gpio = DT_NODE_HAS_PROP(DT_DRV_INST(n), cs_gpios),     \
    };                                                                  \
                                                                        \
    ETH_NET_DEVICE_DT_INST_DEFINE(n,                                    \
                                 eth_spi_basic_init,                    \
                                 NULL,                                  \
                                 &eth_spi_basic_data_##n,               \
                                 &eth_spi_basic_config_##n,             \
                                 CONFIG_ETH_INIT_PRIORITY,              \
                                 &eth_spi_basic_api,                    \
                                 NET_ETH_MTU);

DT_INST_FOREACH_STATUS_OKAY(ETH_SPI_BASIC_INIT)
