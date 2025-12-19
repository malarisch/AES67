#include <zephyr/kernel.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/sys/byteorder.h>
#include <string.h>
#include <zephyr/logging/log.h>


#include "ptp/clock.h"
#include "ptp/port.h"


LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

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


static int get_current_status(void)
{
	struct ptp_port *port;
	sys_slist_t *ports_list = ptp_clock_ports_list();

	if (!ports_list || sys_slist_len(ports_list) == 0) {
		return -EINVAL;
	}

	port = CONTAINER_OF(sys_slist_peek_head(ports_list), struct ptp_port, node);

	if (!port) {
		return -EINVAL;
	}

	switch (ptp_port_state(port)) {
	case PTP_PS_INITIALIZING:
	case PTP_PS_FAULTY:
	case PTP_PS_DISABLED:
	case PTP_PS_LISTENING:
	case PTP_PS_PRE_TIME_TRANSMITTER:
	case PTP_PS_PASSIVE:
	case PTP_PS_UNCALIBRATED:
		printk("FAIL\n");
		return 0;
	case PTP_PS_TIME_RECEIVER:
		printk("TIME RECEIVER\n");
		return 2;
	case PTP_PS_TIME_TRANSMITTER:
	case PTP_PS_GRAND_MASTER:
		printk("TIME TRANSMITTER\n");
		return 1;
	}

	return -1;
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
    LOG_INF("Starting DHCP...");
    net_dhcpv4_start(iface);

    LOG_INF("System ready");
	while (true) {
		printf("STATUS: %02d", get_current_status());
		k_sleep(K_MSEC(1000));
	}
    return 0;

}
