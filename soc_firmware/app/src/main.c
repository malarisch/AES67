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
			  uint64_t mgmt_event,
			  struct net_if *iface)
{
	if (mgmt_event != NET_EVENT_IPV4_DHCP_BOUND) {
		return;
	}

	/* The event info payload is a struct net_if_dhcpv4 containing
	 * the assigned IP in 'requested_ip'.
	 */
	const struct net_if_dhcpv4 *dhcpv4 =
		(const struct net_if_dhcpv4 *)cb->info;

	if (!dhcpv4) {
		LOG_WRN("DHCP bound but no info payload");
		return;
	}

	const uint8_t *ip = (const uint8_t *)&dhcpv4->requested_ip.s_addr;

	LOG_INF("DHCP bound: %u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);

	/* Push the new IP address to the FPGA */
	fpga_write_ip_address(&dhcpv4->requested_ip);
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

/* ---- FPGA status polling & PLL correction ---- */

/**
 * @brief Read a 4-byte (32-bit) value from a sequential FPGA register.
 *
 * The FPGA auto-increments the internal byte pointer on each read
 * to the same address.  Reads LSB first.
 *
 * @param fmc   The FMC device
 * @param reg   FPGA register address (0x52, 0x53, 0x54)
 * @param val   Pointer to store the 32-bit result
 * @return 0 on success, negative errno on error
 */
static int fpga_read_32(const struct device *fmc, uint8_t reg, int32_t *val)
{
	uint8_t buf[4];
	int ret;

	ret = eth_fmc_reg_read_block(fmc, reg, buf, 4);
	if (ret < 0) {
		return ret;
	}

	*val = (int32_t)((uint32_t)buf[0] |
			 ((uint32_t)buf[1] << 8) |
			 ((uint32_t)buf[2] << 16) |
			 ((uint32_t)buf[3] << 24));
	return 0;
}

/* Base frequency for CLK0 – must match initial si5351a_set_frequency() call */
#define SI_CLK0_BASE_FREQ_HZ 24576000U

/* Polling interval for FPGA status registers */
#define FPGA_POLL_INTERVAL_MS 100

/* =====================================================================
 * PI controller for PLL frequency discipline
 *
 * The FPGA measures the PPB error between the PLL clock and the PTP
 * wallclock once per second.  We use a PI controller to smoothly
 * converge on zero error.
 *
 * Controller output = Kp * error + Ki * integral(error)
 *
 * Gains are expressed as fixed-point fractions to avoid floating point:
 *   Kp = PI_KP_NUM / PI_KP_DEN   (proportional gain)
 *   Ki = PI_KI_NUM / PI_KI_DEN   (integral gain per sample)
 *
 * Typical tuning:
 *   Kp = 1/4 = 0.25  → convergence in ~4 steps, no overshoot
 *   Ki = 1/32 = 0.03  → slowly eliminates residual offset
 *
 * Anti-windup: integrator clamped to ±PI_IMAX ppb
 * Outlier rejection: samples > PI_OUTLIER_PPB rejected after warm-up
 * ===================================================================== */

#define PI_KP_NUM          1       /* Proportional numerator */
#define PI_KP_DEN          4       /* Proportional denominator (Kp = 0.25) */
#define PI_KI_NUM          1       /* Integral numerator */
#define PI_KI_DEN          32      /* Integral denominator   (Ki = 0.03125) */
#define PI_IMAX            500000  /* Anti-windup: max integrator magnitude (ppb) */
#define PI_OUTLIER_PPB     50000   /* Max plausible single-measurement error (ppb) */
#define PI_WARMUP_CYCLES   3       /* First N cycles: no outlier rejection */

/* PI controller state */
static struct {
	int64_t integrator;    /* Accumulated integral term (ppb, scaled by KI_DEN) */
	int32_t output;        /* Current total correction applied (ppb) */
	uint32_t cycle;        /* Total accepted measurement count */
	uint32_t outliers;     /* Rejected outlier count */
} pi_state;

/**
 * @brief Background thread that continuously measures the PPB offset
 *        between the PLL and the PTP wallclock, and applies correction
 *        to the Si5351A clock generator.
 *
 * Flow:
 *   1. Write bit[0] of register 0x50 to start measurement.
 *   2. Poll register 0x50 every 100 ms until PPB-valid flag appears.
 *   3. Read the 32-bit signed PPB value from register 0x54.
 *   4. Apply the negated PPB as a correction to the Si5351A PLL.
 *   5. Start the next measurement immediately and repeat.
 */
static void fpga_status_poll_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	const struct device *fmc = device_get_binding("eth_fmc0");
	const struct device *clkgen = DEVICE_DT_GET(DT_NODELABEL(si5351a));
	bool measurement_running = false;
	uint8_t status;
	int ret;
	uint32_t poll_count = 0;   /* Counts 100ms ticks; print at 10 = 1s */

	if (!fmc) {
		LOG_ERR("PPB poll: FMC device not found");
		return;
	}
	if (!device_is_ready(clkgen)) {
		LOG_ERR("PPB poll: Si5351A device not ready");
		return;
	}

	LOG_INF("PPB poll thread started");

	while (1) {
		k_msleep(FPGA_POLL_INTERVAL_MS);
		poll_count++;

		/* Read clocking status flags (register 0x50 read) */
		ret = eth_fmc_reg_read(fmc, ETH_FMC_REG_STATUS_CLK, &status);
		if (ret < 0) {
			LOG_WRN("PPB poll: failed to read status: %d", ret);
			continue;
		}

		/* ---- Print all status registers every 1 second ---- */
		if (poll_count >= 10) {
			poll_count = 0;

			/* 0x50 - Clocking flags (already in 'status') */
			LOG_INF("CLK flags: PPB_valid=%d WC_locked=%d WC_phasejump=%d "
				"WC_configured=%d PTP_leader_lost=%d",
				!!(status & ETH_FMC_CLK_PPB_VALID),
				!!(status & ETH_FMC_CLK_WC_LOCKED),
				!!(status & ETH_FMC_CLK_WC_PHASEJUMP),
				!!(status & ETH_FMC_CLK_WC_CONFIGURED),
				!!(status & ETH_FMC_CLK_PTP_LEADER_LOST));

			/* 0x51 - Ethernet flags */
			uint8_t eth_status;

			ret = eth_fmc_reg_read(fmc, ETH_FMC_REG_STATUS_ETH,
					       &eth_status);
			if (ret == 0) {
				unsigned speed_code = (eth_status &
						       ETH_FMC_ETH_SPEED_MASK) >>
						      ETH_FMC_ETH_SPEED_SHIFT;
				const char *speed_str =
					(speed_code == 0) ? "10M" :
					(speed_code == 1) ? "100M" :
					(speed_code == 2) ? "1G" : "?";

				LOG_INF("ETH flags: link_up=%d speed=%s",
					!!(eth_status & ETH_FMC_ETH_LINK_UP),
					speed_str);
			}

			/* 0x52 - Path delay */
			int32_t path_delay;

			ret = fpga_read_32(fmc, ETH_FMC_REG_PATH_DELAY,
					   &path_delay);
			if (ret == 0) {
				LOG_INF("Path delay: %d ns", path_delay);
			}

			/* 0x53 - Leader offset */
			int32_t leader_offset;

			ret = fpga_read_32(fmc, ETH_FMC_REG_LEADER_OFFSET,
					   &leader_offset);
			if (ret == 0) {
				LOG_INF("Leader offset: %d ns", leader_offset);
			}

			/* 0x54 - PPB offset (current reading) */
			int32_t ppb_current;

			ret = fpga_read_32(fmc, ETH_FMC_REG_PPB_OFFSET,
					   &ppb_current);
			if (ret == 0) {
				LOG_INF("PPB offset: %d  (total correction: %d)",
					ppb_current, pi_state.output);
			}
		}

		/* If no measurement is running, start one */
		if (!measurement_running) {
			uint8_t cmd = ETH_FMC_FLAG_PPB_START;

			ret = eth_fmc_reg_write(fmc, ETH_FMC_REG_STATUS_WR,
						&cmd, 1);
			if (ret < 0) {
				LOG_WRN("PPB poll: failed to start measurement: %d", ret);
				continue;
			}
			measurement_running = true;
			LOG_DBG("PPB measurement started");
			continue;
		}

		/* Check if measurement is complete */
		if (!(status & ETH_FMC_CLK_PPB_VALID)) {
			/* Still measuring — wait */
			continue;
		}

		/* Measurement complete — read the PPB value */
		int32_t ppb_measured;

		ret = fpga_read_32(fmc, ETH_FMC_REG_PPB_OFFSET, &ppb_measured);
		if (ret < 0) {
			LOG_WRN("PPB poll: failed to read PPB: %d", ret);
			measurement_running = false;
			continue;
		}

		/* ---- Outlier rejection (after warm-up) ---- */
		if (pi_state.cycle >= PI_WARMUP_CYCLES) {
			int32_t abs_meas = (ppb_measured < 0) ? -ppb_measured
							      : ppb_measured;
			if (abs_meas > PI_OUTLIER_PPB) {
				pi_state.outliers++;
				LOG_WRN("PPB outlier rejected: %d  "
					"(outliers=%u cycle=%u)",
					ppb_measured, pi_state.outliers,
					pi_state.cycle);
				measurement_running = false;
				continue;
			}
		}

		pi_state.cycle++;

		/*
		 * PI controller:
		 *
		 * error = ppb_measured  (positive = PLL fast → need to slow down)
		 *
		 * P term:  correction_p = Kp * error
		 * I term:  integrator += error;  correction_i = Ki * integrator
		 * Output:  output -= (correction_p + correction_i)
		 *
		 * The output is the *absolute* ppb offset applied to the PLL
		 * via si5351a_adjust_ppb().
		 */
		int32_t error = ppb_measured;

		/* Proportional term */
		int32_t p_term = (int32_t)(((int64_t)error * PI_KP_NUM) / PI_KP_DEN);

		/* Integral term with anti-windup */
		pi_state.integrator += error;
		if (pi_state.integrator > (int64_t)PI_IMAX * PI_KI_DEN) {
			pi_state.integrator = (int64_t)PI_IMAX * PI_KI_DEN;
		} else if (pi_state.integrator < -(int64_t)PI_IMAX * PI_KI_DEN) {
			pi_state.integrator = -(int64_t)PI_IMAX * PI_KI_DEN;
		}
		int32_t i_term = (int32_t)(pi_state.integrator * PI_KI_NUM / PI_KI_DEN);

		/* Apply combined correction (subtract because positive error = too fast) */
		pi_state.output -= (p_term + i_term);

		LOG_INF("PI: err=%d P=%d I=%d out=%d  cycle=%u",
			error, p_term, i_term, pi_state.output, pi_state.cycle);

		ret = si5351a_adjust_ppb(clkgen, 0, SI_CLK0_BASE_FREQ_HZ,
					 pi_state.output);
		if (ret < 0) {
			LOG_ERR("PPB poll: Si5351A adjust failed: %d", ret);
		}

		/* Start next measurement immediately */
		measurement_running = false;
	}
}

#define PPB_POLL_STACK_SIZE 2048
#define PPB_POLL_PRIORITY   K_PRIO_PREEMPT(10)

K_THREAD_STACK_DEFINE(ppb_poll_stack, PPB_POLL_STACK_SIZE);
static struct k_thread ppb_poll_thread_data;

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

    /* ---- Start PPB measurement / PLL correction thread ---- */
    k_thread_create(&ppb_poll_thread_data, ppb_poll_stack,
                    PPB_POLL_STACK_SIZE,
                    fpga_status_poll_thread, NULL, NULL, NULL,
                    PPB_POLL_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&ppb_poll_thread_data, "ppb_poll");

    LOG_INF("System ready");
    return 0;

}