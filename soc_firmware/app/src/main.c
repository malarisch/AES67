#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/net_event.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/dhcpv4.h>
#include <zephyr/sys/byteorder.h>
#include <string.h>
#include <zephyr/logging/log.h>

#include "../drivers/si5351a/si5351a.h"
#include "../drivers/eth_fmc_basic/eth_fmc_basic.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* ---- FPGA configuration helpers ---- */

/**
 * @brief Write the interface MAC address to the FPGA via FMC register 0x40.
 *
 * The FPGA expects 6 consecutive byte-writes to address 0x40.
 * Each write auto-increments the internal byte counter.
 *
 * @param iface  Network interface whose MAC to send
 * @return 0 on success, negative errno on error
 */
static int fpga_write_mac_address(struct net_if *iface)
{
	const struct device *fmc = device_get_binding("eth_fmc0");
	struct net_linkaddr *ll;

	if (!fmc) {
		LOG_ERR("FMC device not found");
		return -ENODEV;
	}

	ll = net_if_get_link_addr(iface);
	if (!ll || ll->len < 6) {
		LOG_ERR("No valid MAC address on interface");
		return -EINVAL;
	}

	int ret = eth_fmc_reg_write(fmc, ETH_FMC_REG_MAC_ADDR, ll->addr, 6);
	if (ret < 0) {
		LOG_ERR("Failed to write MAC to FPGA: %d", ret);
		return ret;
	}

	LOG_INF("FPGA MAC set to %02x:%02x:%02x:%02x:%02x:%02x",
		ll->addr[0], ll->addr[1], ll->addr[2],
		ll->addr[3], ll->addr[4], ll->addr[5]);

	return 0;
}

/**
 * @brief Write an IPv4 address to the FPGA via FMC register 0x41.
 *
 * The FPGA expects 4 consecutive byte-writes to address 0x41.
 * Each write auto-increments the internal byte counter.
 *
 * @param addr  IPv4 address in network byte order (4 bytes)
 * @return 0 on success, negative errno on error
 */
static int fpga_write_ip_address(const struct in_addr *addr)
{
	const struct device *fmc = device_get_binding("eth_fmc0");

	if (!fmc) {
		LOG_ERR("FMC device not found");
		return -ENODEV;
	}

	int ret = eth_fmc_reg_write(fmc, ETH_FMC_REG_IP_ADDR,
				    (const uint8_t *)&addr->s_addr, 4);
	if (ret < 0) {
		LOG_ERR("Failed to write IP to FPGA: %d", ret);
		return ret;
	}

	LOG_INF("FPGA IP set to %u.%u.%u.%u",
		((const uint8_t *)&addr->s_addr)[0],
		((const uint8_t *)&addr->s_addr)[1],
		((const uint8_t *)&addr->s_addr)[2],
		((const uint8_t *)&addr->s_addr)[3]);

	return 0;
}

/* ---- DHCP event handling ---- */

static struct net_mgmt_event_callback dhcp_cb;

static void on_dhcp_bound(struct net_mgmt_event_callback *cb,
			  uint32_t mgmt_event,
			  struct net_if *iface)
{
	if (mgmt_event != NET_EVENT_IPV4_DHCP_BOUND) {
		return;
	}

	/* Retrieve the assigned address */
	struct net_if_config *if_cfg = net_if_get_config(iface);

	if (!if_cfg || if_cfg->ip.ipv4 == NULL) {
		LOG_WRN("DHCP bound but no IPv4 config available");
		return;
	}

	struct net_if_addr *unicast = &if_cfg->ip.ipv4->unicast[0].ipv4;

	LOG_INF("DHCP bound: %u.%u.%u.%u",
		((const uint8_t *)&unicast->address.in_addr.s_addr)[0],
		((const uint8_t *)&unicast->address.in_addr.s_addr)[1],
		((const uint8_t *)&unicast->address.in_addr.s_addr)[2],
		((const uint8_t *)&unicast->address.in_addr.s_addr)[3]);

	/* Push the new IP address to the FPGA */
	fpga_write_ip_address(&unicast->address.in_addr);
}

/* ---- Legacy test code (unused) ---- */

#define RAW_PAYLOAD_START_LEN 30
#define RAW_PAYLOAD_MAX_LEN 1500
#define SEND_INTERVAL_MS 1000

/* Allocate worst-case payload so we never overflow when length ramps up. */
static uint8_t payload[RAW_PAYLOAD_MAX_LEN];
size_t frame_len = RAW_PAYLOAD_START_LEN;

static int send_raw_frame(struct net_if *iface, uint8_t seq)
{
    

	payload[0] = 0xAC;
	payload[1] = 0xAB;
	payload[2] = 0x16;
	payload[3] = 0x10;
	payload[4] = 0x00;
	payload[5] = 0xBE;
	payload[6] = 0xEF;


	payload[7] = 0x01;
	payload[8] = 0x02;
	payload[9] = 0x03;
	payload[10] = 0x04;
	payload[11] = 0x05;
	payload[12] = 0x06;

	payload[13] = 0x07;
	payload[14] = 0x08;
	

	frame_len++;
	if (frame_len >= RAW_PAYLOAD_MAX_LEN) {
		frame_len = RAW_PAYLOAD_START_LEN;
	}
	for (size_t i = 15; i < frame_len && i < RAW_PAYLOAD_MAX_LEN; i++) {
		payload[i] = seq;
	}
	struct net_pkt *pkt = net_pkt_alloc_with_buffer(iface, frame_len, AF_UNSPEC, 0, K_MSEC(100));
	if (!pkt) {
		LOG_ERR("pkt alloc failed");
		return -ENOMEM;
	}

	net_pkt_set_ll_proto_type(pkt, 0xDEAD);

		 // ||	    net_pkt_write(pkt, payload, RAW_PAYLOAD_LEN)
	if (net_pkt_write(pkt, payload, frame_len) < 0) {
		LOG_ERR("pkt write failed");
		net_pkt_unref(pkt);
		return -EIO;
	}

	int ret = net_send_data(pkt);
	if (ret < 0) {
		LOG_ERR("net_send_data failed (%d)", ret);
		net_pkt_unref(pkt);
		return ret;
	}

	return 0;
}

int main(void)
{
	struct net_if *iface = net_if_get_default();
	// uint8_t seq = 0;

	LOG_INF("Starting raw Ethernet TX demo");

	printk("MAIN: Starting loop\n");

    if (!iface) {
        LOG_ERR("No network interface found");
        return -1;
    }
	/*while (1) {
		static uint8_t seq = 0;
		if (send_raw_frame(iface, seq) == 0) {
			LOG_INF("Sent raw Ethernet frame with seq %u and length %u", seq, frame_len);
			seq++;
		} else {
			LOG_ERR("Failed to send raw Ethernet frame");
		}
		k_msleep(SEND_INTERVAL_MS);
	}'*/
    /* ---- Si5351A Clock Generator Setup ---- */
    const struct device *clkgen = DEVICE_DT_GET(DT_NODELABEL(si5351a));

    if (!device_is_ready(clkgen)) {
        LOG_ERR("Si5351A device not ready");
    } else {
        LOG_INF("Si5351A device ready, configuring clocks...");

        /* CLK0: 24.576 MHz  – AES67 audio master clock (48 kHz × 512) */
        int ret = si5351a_set_frequency(clkgen, 0, 24576000);
        if (ret) {
            LOG_ERR("Failed to set CLK0: %d", ret);
        }

        /* CLK1: 12.288 MHz  – I2S MCLK (48 kHz × 256) */
        ret = si5351a_set_frequency(clkgen, 1, 12288000);
        if (ret) {
            LOG_ERR("Failed to set CLK1: %d", ret);
        }

        /* Drive strength 8 mA for both outputs */
        si5351a_set_drive_strength(clkgen, 0, SI5351A_DRIVE_8MA);
        si5351a_set_drive_strength(clkgen, 1, SI5351A_DRIVE_8MA);

        LOG_INF("Si5351A clocks configured: CLK0=24.576 MHz, CLK1=12.288 MHz");
    }

    /* ---- Write MAC address to FPGA ---- */
    fpga_write_mac_address(iface);

    /* ---- Register DHCP event handler to push IP to FPGA ---- */
    net_mgmt_init_event_callback(&dhcp_cb, on_dhcp_bound,
                                 NET_EVENT_IPV4_DHCP_BOUND);
    net_mgmt_add_event_callback(&dhcp_cb);

    LOG_INF("Starting DHCP...");
    net_dhcpv4_start(iface);

    LOG_INF("System ready");
    return 0;

}