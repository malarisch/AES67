#include <zephyr/kernel.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/sys/byteorder.h>
#include <string.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#define RAW_PAYLOAD_LEN 30
#define SEND_INTERVAL_MS 100

static uint8_t payload[RAW_PAYLOAD_LEN];
size_t frame_len = 30;

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
	if (frame_len >= 1500) {
		frame_len = RAW_PAYLOAD_LEN;
	}
	for (size_t i = 15; i < frame_len; i++) {
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
	uint8_t seq = 0;

	LOG_INF("Starting raw Ethernet TX demo");

    if (!iface) {
        LOG_ERR("No network interface found");
        return -1;
    }

    LOG_INF("Starting DHCP...");
    net_dhcpv4_start(iface);

    LOG_INF("System ready");
    return 0;

}
