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
#include <zephyr/random/random.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <stdbool.h>

#include "eth_spi_basic.h"

LOG_MODULE_REGISTER(eth_spi_basic, CONFIG_ETHERNET_LOG_LEVEL);

/* Register map of the FPGA SPI client */
#define REG_TX_LEN_L      0x00
#define REG_TX_LEN_H      0x01
#define REG_TX_CTRL       0x02
#define REG_TX_TS_BASE    0x03  /* 0x03-0x0A: TX timestamp (8 bytes) */
#define REG_TX_TS_STATUS  0x0B
#define REG_TX_WINDOW     0x10  /* 0x10.. auto-increment */

#define REG_RX_LEN_L      0x20
#define REG_RX_LEN_H      0x21
#define REG_RX_STATUS     0x22
#define REG_RX_TS_BASE    0x23  /* 0x23-0x2A: RX timestamp (8 bytes) */
#define REG_RX_TS_STATUS  0x2B
#define REG_RX_WINDOW     0x30  /* 0x30.. auto-increment */

#define RX_STATUS_READY   BIT(0)
#define RX_STATUS_OVF     BIT(1)

#define TS_STATUS_VALID   BIT(0)

/* Helper to build the first SPI byte */
#define SPI_CMD_WRITE(addr7)   (0x80 | ((addr7) & 0x7F))
#define SPI_CMD_READ(addr7)    ((addr7) & 0x7F)

/* Device Datenstruktur */
struct eth_spi_basic_data {
    struct net_if *iface;
    uint8_t mac_addr[6];
    struct k_sem tx_sem;
    struct k_sem int_sem;
    struct gpio_callback gpio_cb;
    struct k_thread rx_thread;
    bool use_interrupt;
    uint8_t tx_buf[ETH_SPI_BASIC_MAX_PKT_SIZE];
    uint8_t rx_buf[ETH_SPI_BASIC_MAX_PKT_SIZE];
    struct k_work_delayable link_work;
    K_KERNEL_STACK_MEMBER(rx_stack, CONFIG_ETH_SPI_BASIC_RX_STACK_SIZE);
};

/* Device Konfiguration */
struct eth_spi_basic_config {
    struct spi_dt_spec spi;
    struct gpio_dt_spec interrupt;
    uint8_t full_duplex;
};

/* SPI Hilfsfunktionen */
/* Globale TX/RX-Puffer, um Stack zu sparen und den ursprünglichen Burst-Stil beizubehalten. */
static uint8_t spi_tx_buf[ETH_SPI_BASIC_MAX_PKT_SIZE + 1];
static uint8_t spi_rx_buf[ETH_SPI_BASIC_MAX_PKT_SIZE + 1];

static int spi_basic_read_bytes(const struct device *dev, uint8_t addr,
                                uint8_t *buf, size_t len)
{
    const struct eth_spi_basic_config *cfg = dev->config;

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

static int spi_basic_write_bytes(const struct device *dev, uint8_t addr,
                                 const uint8_t *buf, size_t len)
{
    const struct eth_spi_basic_config *cfg = dev->config;

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

/* Lies RX_LEN_L, RX_LEN_H und RX_STATUS in einem CS-low-Burst (auto-increment im FPGA). */
static int spi_basic_read_len_status(const struct device *dev, uint8_t *len_l, uint8_t *len_h, uint8_t *status)
{
    uint8_t tmp[3] = {0};
    /* Start bei 0x20, drei Bytes: len_l, len_h, status */
    int ret = spi_basic_read_bytes(dev, REG_RX_LEN_L, tmp, sizeof(tmp));
    if (ret == 0) {
        *len_l  = tmp[0];
        *len_h  = tmp[1];
        *status = tmp[2];
    }
    //LOG_DBG("Read len_l=0x%02X len_h=0x%02X status=0x%02X", tmp[0], tmp[1], tmp[2]);
    return ret;
}

static int spi_basic_clear_rx(const struct device *dev)
{
    //LOG_DBG("clrrx");
    /* Writing bit0 toggles the clear line inside the FPGA */
    return spi_basic_write_reg(dev, REG_RX_STATUS, RX_STATUS_READY);
}

/* Ethernet Packet senden */
static int eth_spi_basic_tx(const struct device *dev, struct net_pkt *pkt)
{
    struct eth_spi_basic_data *data = dev->data;
    size_t pkt_len = net_pkt_get_len(pkt);
    int ret;
    
    LOG_DBG("TX packet, len=%zu", pkt_len);
    
    k_sem_take(&data->tx_sem, K_FOREVER);
    
    if (pkt_len > ETH_SPI_BASIC_MAX_PKT_SIZE) {
        LOG_ERR("Packet too large: %zu", pkt_len);
        ret = -EMSGSIZE;
        goto done;
    }

    if (net_pkt_read(pkt, data->tx_buf, pkt_len) < 0) {
        LOG_ERR("Failed to read packet data");
        ret = -EIO;
        goto done;
    }

    /* Program TX length */
    uint8_t len_bytes[2] = { pkt_len & 0xff, pkt_len >> 8 };
    ret = spi_basic_write_bytes(dev, REG_TX_LEN_L, len_bytes, sizeof(len_bytes));
    if (ret) {
        LOG_ERR("Failed to write TX length: %d", ret);
        goto done;
    }
    //for (size_t i = 0; i < pkt_len; i+=1) {
    //        if (i % 16 == 0) {
    //            printf("\n %04X ", i);
    //        }
    //        printf(" %d 0x%02X", i, data->tx_buf[i]);
    //        if (i % 16 == 0) {
    //            printf("\n");
    //        }
    //    }
    //LOG_DBG("total packet %u bytes", pkt_len);
    /* Push payload into TX window (auto-increment) */
    ret = spi_basic_write_bytes(dev, REG_TX_WINDOW, data->tx_buf, pkt_len);
    if (ret) {
        LOG_ERR("Failed to write TX payload: %d", ret);
        goto done;
    }

    /* Toggle TX start (write 0 then 1 to ensure a rising edge) */
    (void)spi_basic_write_reg(dev, REG_TX_CTRL, 0x00);
    ret = spi_basic_write_reg(dev, REG_TX_CTRL, 0x01);
    if (ret) {
        LOG_ERR("Failed to trigger TX: %d", ret);
    }
    
done:
    k_sem_give(&data->tx_sem);
    return ret;
}

/* RX Thread - empfängt Pakete */
static void eth_spi_basic_rx_thread(void *p1, void *p2, void *p3)
{
    const struct device *dev = p1;
    struct eth_spi_basic_data *data = dev->data;
    struct net_pkt *pkt;
    uint8_t status;
    uint8_t len_l, len_h;
    uint16_t pkt_len;
    
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);
    
    while (1) {
        if (data->use_interrupt) {
            (void)k_sem_take(&data->int_sem, K_MSEC(50));
        } else {
            k_sleep(K_MSEC(5));
        }

        if (spi_basic_read_len_status(dev, &len_l, &len_h, &status) || !(status & RX_STATUS_READY)) {
            if (status & RX_STATUS_OVF) {
                LOG_WRN("RX status overflow without READY (status=0x%02X)", status);
                spi_basic_clear_rx(dev);
            }
            continue;
        }
        pkt_len = ((uint16_t)len_l | ((uint16_t)len_h << 8));
        LOG_DBG("RX ready len=%u status=0x%02X", pkt_len, status);
        //printf("\n-- start ---\nRX packet, len=%u\n", pkt_len);
        if (pkt_len == 0 || pkt_len > ETH_SPI_BASIC_MAX_PKT_SIZE) {
            LOG_ERR("Invalid RX length %u", pkt_len);
            spi_basic_clear_rx(dev);
            continue;
        }

        if (spi_basic_read_bytes(dev, REG_RX_WINDOW, data->rx_buf, pkt_len)) {
            LOG_ERR("RX payload read failed");
            spi_basic_clear_rx(dev);
            continue;
        }

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
        uint16_t eth_type = ((uint16_t)data->rx_buf[12] << 8) | data->rx_buf[13];
        if (!dst_broadcast && !dst_match) {
            LOG_DBG("RX dst mismatch %02x:%02x:%02x:%02x:%02x:%02x (mine %02x:%02x:%02x:%02x:%02x:%02x) type=0x%04X",
                    data->rx_buf[0], data->rx_buf[1], data->rx_buf[2],
                    data->rx_buf[3], data->rx_buf[4], data->rx_buf[5],
                    data->mac_addr[0], data->mac_addr[1], data->mac_addr[2],
                    data->mac_addr[3], data->mac_addr[4], data->mac_addr[5],
                    eth_type);
            /* Manche Gegenstellen antworten auf die im MAC-Core konfigurierte Adresse.
             * Damit der Zephyr-Stack das Paket akzeptiert, schreibe die Zieladresse
             * auf unsere Interface-MAC um.
             */
            memcpy(data->rx_buf, data->mac_addr, 6);
        } else {
            LOG_DBG("RX dst %s type=0x%04X", dst_broadcast ? "broadcast" : "unicast", eth_type);
        }

        /* FPGA liefert das komplette Ethernet-Frame inkl. 4-Byte-FCS.
         * Zephyr erwartet Frames ohne FCS -> 4 Bytes abziehen, aber nur wenn genügend lang.
         */
        uint16_t payload_len = pkt_len;
        if (payload_len >= 4U) {
            payload_len -= 4U;
        } else {
            LOG_ERR("RX length too small to strip FCS (%u)", payload_len);
            spi_basic_clear_rx(dev);
            continue;
        }
        /* Filter offensichtlichen Müll: kleiner als Ethernet-Header */
        if (payload_len < 14U) {
            LOG_WRN("RX frame too short (%u), dropping", payload_len);
            spi_basic_clear_rx(dev);
            continue;
        }

        pkt = net_pkt_rx_alloc_with_buffer(data->iface, payload_len,
                                           AF_UNSPEC, 0, K_NO_WAIT);
        if (!pkt) {
            LOG_ERR("net_pkt allocation failed");
            spi_basic_clear_rx(dev);
            continue;
        }
      //  for (size_t i = 0; i < pkt_len; i+=1) {
      //      if (i % 16 == 0) {
      //         printf("\n %04X ", i);
      //      }
      //      printf(" %02d 0x%02X", i, data->rx_buf[i]);
      //      
      //  }
        //for (size_t i = 0; i < pkt_len; i+=8) {
        //    LOG_DBG("Packet %d 0x%02X %d 0x%02X %d 0x%02X %d 0x%02X %d 0x%02X %d 0x%02X %d 0x%02X %d 0x%02X", i, data->rx_buf[i],
         //       i+1, data->rx_buf[i+1], i+2, data->rx_buf[i+2], i+3, data->rx_buf[i+3],
          //      i+4, data->rx_buf[i+4], i+5, data->rx_buf[i+5], i+6, data->rx_buf[i+6],
           //     i+7, data->rx_buf[i+7]);
        //}
     //   printf("\n-- end -- total packet %u bytes\n", pkt_len);
        if (net_pkt_write(pkt, data->rx_buf, payload_len)) {
            LOG_ERR("RX buffer write failed");
            net_pkt_unref(pkt);
            spi_basic_clear_rx(dev);
            continue;
        }

        int ret = net_recv_data(data->iface, pkt);
        if (ret < 0) {
            LOG_ERR("RX deliver failed (%d)", ret);
            net_pkt_unref(pkt);
        } else {
            LOG_DBG("RX delivered len=%u", payload_len);
        }

        if (status & RX_STATUS_OVF) {
            LOG_WRN("RX overflow flagged");
        }

        spi_basic_clear_rx(dev);
    }
}

/* GPIO Interrupt Handler */
static void eth_spi_basic_gpio_callback(const struct device *port,
                                        struct gpio_callback *cb,
                                        gpio_port_pins_t pins)
{
    struct eth_spi_basic_data *data = 
        CONTAINER_OF(cb, struct eth_spi_basic_data, gpio_cb);
    
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
    return ETHERNET_LINK_10BASE | ETHERNET_LINK_100BASE;
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

    data->use_interrupt = false;

    if (cfg->interrupt.port) {
        if (!gpio_is_ready_dt(&cfg->interrupt)) {
            LOG_WRN("GPIO interrupt not ready, falling back to polling");
        } else {
            ret = gpio_pin_configure_dt(&cfg->interrupt, GPIO_INPUT);
            if (ret != 0) {
                LOG_ERR("Failed to configure GPIO interrupt");
                return ret;
            }

            gpio_init_callback(&data->gpio_cb, eth_spi_basic_gpio_callback,
                              BIT(cfg->interrupt.pin));

            ret = gpio_add_callback(cfg->interrupt.port, &data->gpio_cb);
            if (ret != 0) {
                LOG_ERR("Failed to add GPIO callback");
                return ret;
            }

            ret = gpio_pin_interrupt_configure_dt(&cfg->interrupt,
                                                 GPIO_INT_EDGE_TO_ACTIVE);
            if (ret != 0) {
                LOG_ERR("Failed to configure GPIO interrupt");
                return ret;
            }
            data->use_interrupt = true;
        }
    }
    
    /* Semaphoren initialisieren */
    k_sem_init(&data->tx_sem, 1, 1);
    k_sem_init(&data->int_sem, 0, 1);

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
    
    LOG_INF("SPI Ethernet initialized");

    /* Link später hochziehen. */
    k_work_schedule(&data->link_work, K_MSEC(50));
    
    return 0;
}

/* PTP Timestamp Helper Functions */
/* Assemble 64-bit timestamp from 8-byte little-endian array */
static inline uint64_t assemble_timestamp(const uint8_t *bytes)
{
    return (uint64_t)bytes[0]        |
           ((uint64_t)bytes[1] << 8)  |
           ((uint64_t)bytes[2] << 16) |
           ((uint64_t)bytes[3] << 24) |
           ((uint64_t)bytes[4] << 32) |
           ((uint64_t)bytes[5] << 40) |
           ((uint64_t)bytes[6] << 48) |
           ((uint64_t)bytes[7] << 56);
}

/* Generic timestamp read function */
static int read_timestamp(const struct device *dev, uint8_t status_reg, 
                         uint8_t data_reg, struct eth_spi_basic_timestamp *ts)
{
    uint8_t ts_bytes[8];
    uint8_t status;
    int ret;
    
    /* Read timestamp status first */
    ret = spi_basic_read_reg(dev, status_reg, &status);
    if (ret) {
        return ret;
    }
    
    ts->valid = (status & TS_STATUS_VALID) != 0;
    
    if (!ts->valid) {
        ts->ns = 0;
        return 0;
    }
    
    /* Read 8-byte timestamp (little-endian) */
    ret = spi_basic_read_bytes(dev, data_reg, ts_bytes, sizeof(ts_bytes));
    if (ret) {
        return ret;
    }
    
    /* Assemble 64-bit timestamp */
    ts->ns = assemble_timestamp(ts_bytes);
    
    return 0;
}

/* PTP Timestamp Functions */
int eth_spi_basic_get_tx_timestamp(const struct device *dev, 
                                    struct eth_spi_basic_timestamp *ts)
{
    if (!dev || !ts) {
        return -EINVAL;
    }
    
    return read_timestamp(dev, REG_TX_TS_STATUS, REG_TX_TS_BASE, ts);
}

int eth_spi_basic_get_rx_timestamp(const struct device *dev, 
                                    struct eth_spi_basic_timestamp *ts)
{
    if (!dev || !ts) {
        return -EINVAL;
    }
    
    return read_timestamp(dev, REG_RX_TS_STATUS, REG_RX_TS_BASE, ts);
}

int eth_spi_basic_ack_tx_timestamp(const struct device *dev)
{
    if (!dev) {
        return -EINVAL;
    }
    
    /* Write TS_STATUS_VALID to acknowledge/clear the TX timestamp valid flag */
    return spi_basic_write_reg(dev, REG_TX_TS_STATUS, TS_STATUS_VALID);
}

int eth_spi_basic_ack_rx_timestamp(const struct device *dev)
{
    if (!dev) {
        return -EINVAL;
    }
    
    /* Write TS_STATUS_VALID to acknowledge/clear the RX timestamp valid flag */
    return spi_basic_write_reg(dev, REG_RX_TS_STATUS, TS_STATUS_VALID);
}

/* Device Makro */
#define ETH_SPI_BASIC_INIT(n)                                           \
    static struct eth_spi_basic_data eth_spi_basic_data_##n;            \
                                                                        \
    static const struct eth_spi_basic_config eth_spi_basic_config_##n = { \
        .spi = SPI_DT_SPEC_INST_GET(n,                                  \
            SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_HOLD_ON_CS, \
            0),                                                         \
        .interrupt = GPIO_DT_SPEC_INST_GET(n, int_gpios),              \
        .full_duplex = DT_INST_PROP(n, full_duplex),                   \
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
