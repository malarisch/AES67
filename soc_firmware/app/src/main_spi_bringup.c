/*
 * SPI-only bring-up main() for the AES67 external-MCU build (currently
 * ESP32-S3). Programs a set of default registers into the FPGA at boot
 * and then polls status periodically so the wiring can be validated
 * before the full control-plane features (networking, RTSP, HTTP, mDNS)
 * get ported over. All defaults can be overwritten live via the `fpga`
 * shell commands.
 */

#include <zephyr/kernel.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/logging/log.h>

#include "../drivers/fpga_hal/fpga_hal.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* ---- Boot defaults ---- */

/* Locally-administered MAC (bit 1 of the first byte set). */
static const uint8_t DEFAULT_MAC[6] = { 0x02, 0xaa, 0xe6, 0x70, 0x00, 0x01 };

/* 192.168.88.210 */
#define DEFAULT_IP_A 192
#define DEFAULT_IP_B 168
#define DEFAULT_IP_C 88
#define DEFAULT_IP_D 210

/* PTP config — sensible AES67 defaults. */
#define DEFAULT_PTP_TIME_SOURCE        0xA0  /* internal oscillator */
#define DEFAULT_PTP_LOG_SYNC           (-3)  /* 2^-3 s = 125 ms */
#define DEFAULT_PTP_LOG_ANNOUNCE       0     /* 1 s */

/* GM quality — default profile (not GPS-locked). */
#define DEFAULT_PTP_PRIORITY1          128
#define DEFAULT_PTP_PRIORITY2          128
#define DEFAULT_PTP_CLOCK_CLASS        248
#define DEFAULT_PTP_CLOCK_ACCURACY     0xFE

static void apply_defaults(void)
{
	struct in_addr ip;
	int ret;

	ret = fpga_hal_write_mac(DEFAULT_MAC);
	LOG_INF("default MAC %02x:%02x:%02x:%02x:%02x:%02x -> %d",
		DEFAULT_MAC[0], DEFAULT_MAC[1], DEFAULT_MAC[2],
		DEFAULT_MAC[3], DEFAULT_MAC[4], DEFAULT_MAC[5], ret);

	/* s_addr is network byte order: first octet in the MSB. */
	ip.s_addr = htonl(((uint32_t)DEFAULT_IP_A << 24) |
			  ((uint32_t)DEFAULT_IP_B << 16) |
			  ((uint32_t)DEFAULT_IP_C << 8)  |
			  ((uint32_t)DEFAULT_IP_D));
	ret = fpga_hal_write_ip(&ip);
	LOG_INF("default IP %u.%u.%u.%u -> %d",
		DEFAULT_IP_A, DEFAULT_IP_B, DEFAULT_IP_C, DEFAULT_IP_D, ret);

	ret = fpga_hal_write_ptp_config(DEFAULT_PTP_TIME_SOURCE,
					DEFAULT_PTP_LOG_SYNC,
					DEFAULT_PTP_LOG_ANNOUNCE);
	LOG_INF("default PTP cfg (src=0x%02x sync=%d ann=%d) -> %d",
		DEFAULT_PTP_TIME_SOURCE, DEFAULT_PTP_LOG_SYNC,
		DEFAULT_PTP_LOG_ANNOUNCE, ret);

	ret = fpga_hal_write_ptp_gm_quality(DEFAULT_PTP_PRIORITY1,
					    DEFAULT_PTP_PRIORITY2,
					    DEFAULT_PTP_CLOCK_CLASS,
					    DEFAULT_PTP_CLOCK_ACCURACY);
	LOG_INF("default PTP GM (p1=%u p2=%u class=%u acc=0x%02x) -> %d",
		DEFAULT_PTP_PRIORITY1, DEFAULT_PTP_PRIORITY2,
		DEFAULT_PTP_CLOCK_CLASS, DEFAULT_PTP_CLOCK_ACCURACY, ret);
}

int main(void)
{
	LOG_INF("AES67 SPI bring-up (no networking on this board yet)");

	if (!fpga_hal_is_ready()) {
		LOG_ERR("FPGA HAL not ready - check SPI wiring and zephyr,fpga-spi");
		return 0;
	}

	apply_defaults();

	while (1) {
		/*uint32_t status = fpga_hal_read_status();
		int32_t  offset = fpga_hal_read_ptp_offset();
		int32_t  delay  = fpga_hal_read_path_delay();
		uint8_t  gmid[8];
		bool     have_gm = fpga_hal_read_ptp_leader_id(gmid);

		LOG_INF("status=0x%08x path_delay=%d offset=%d", status, delay, offset);
		if (have_gm) {
			LOG_INF("PTP GM id: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
				gmid[0], gmid[1], gmid[2], gmid[3],
				gmid[4], gmid[5], gmid[6], gmid[7]);
		} */
		k_sleep(K_SECONDS(1));
	} 
	return 0;
}
