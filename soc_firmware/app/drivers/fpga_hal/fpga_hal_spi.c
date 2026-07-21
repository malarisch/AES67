/*

 * FPGA HAL backend — SPI (spibone) implementation.
 *
 * External-MCU path (e.g. ESP32-S3): the same aes67_bridge CSR map the other
 * hosts use, reached over the LiteX spibone SPI-to-Wishbone bridge instead of
 * a memory-mapped bus. Register names/addresses come from the LiteX-generated
 * aes67_bridge headers (litex_soc/build/aes67_bridge/software/include); the
 * register sequences mirror fpga_hal_litex.c / eth_litex.c and the Linux
 * kernel driver.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include "fpga_hal.h"
#include "../spibone/spibone.h"
/* Macro layer only (AES67_STATUS_ / AES67_CTRL_ bit fields, stream-cfg and
 * eth_buf base addresses derived from the generated headers). The eth_litex
 * functions declared there are not built in this backend. */
#include "../eth_litex/eth_litex.h"

LOG_MODULE_REGISTER(fpga_hal, LOG_LEVEL_INF);

/* ---- Device accessor ---- */

const struct device *fpga_hal_get_dev(void)
{
	return spibone_get_dev();
}

/* ---- FPGA ready / recovery ---- */

bool fpga_hal_is_ready(void)
{
	const struct device *dev = spibone_get_dev();
	uint32_t status;

	if (dev == NULL || !device_is_ready(dev)) {
		return false;
	}
	/* Probe an actual bus transaction: the bridge answers reads only once
	 * the FPGA is configured and the SPI wiring is sound. */
	return spibone_read(CSR_AES67_CSR_STATUS_ADDR, &status) == 0;
}

int fpga_hal_wait_ready(uint32_t timeout_ms)
{
	int64_t deadline = k_uptime_get() + timeout_ms;

	while (!fpga_hal_is_ready()) {
		if (timeout_ms != 0 && k_uptime_get() >= deadline) {
			return -ETIMEDOUT;
		}
		k_msleep(10);
	}
	return 0;
}

void fpga_hal_register_recover_cb(fpga_hal_recover_cb_t cb, void *user_data)
{
	ARG_UNUSED(cb);
	ARG_UNUSED(user_data);
	/* No recovery signalling on the SPI transport. */
}

/* ---- Configuration writes ---- */

int fpga_hal_write_mac(const uint8_t mac[6])
{
	/* mac[0..1] → mac_addr_hi (bits 47..32), mac[2..5] → mac_addr_lo. */
	uint32_t hi = ((uint32_t)mac[0] << 8) |
		      (uint32_t)mac[1];
	uint32_t lo = ((uint32_t)mac[2] << 24) |
		      ((uint32_t)mac[3] << 16) |
		      ((uint32_t)mac[4] << 8) |
		      (uint32_t)mac[5];
	int ret;

	spibone_bus_lock();
	ret = spibone_write_locked(CSR_AES67_CSR_MAC_ADDR_LO_ADDR, lo);
	if (ret == 0) {
		ret = spibone_write_locked(CSR_AES67_CSR_MAC_ADDR_HI_ADDR, hi);
	}
	spibone_bus_unlock();
	return ret;
}

int fpga_hal_write_ip(const struct in_addr *ip)
{
	/* s_addr is network byte order; the CSR wants octet 0 in bits 31..24. */
	const uint8_t *b = (const uint8_t *)&ip->s_addr;
	uint32_t val = ((uint32_t)b[0] << 24) |
		       ((uint32_t)b[1] << 16) |
		       ((uint32_t)b[2] << 8) |
		       (uint32_t)b[3];

	return spibone_write(CSR_AES67_CSR_IP_ADDR_ADDR, val);
}

int fpga_hal_write_ptp_config(uint8_t time_source,
			      int8_t log_msg_interval,
			      int8_t log_announce_interval)
{
	int ret;

	spibone_bus_lock();
	ret = spibone_write_locked(CSR_AES67_CSR_PTP_TIME_SOURCE_ADDR, time_source);
	if (ret == 0) {
		ret = spibone_write_locked(CSR_AES67_CSR_PTP_LOG_MSG_INTERVAL_ADDR,
					   (uint8_t)log_msg_interval);
	}
	if (ret == 0) {
		ret = spibone_write_locked(CSR_AES67_CSR_PTP_ANNOUNCE_MSG_INTERVAL_ADDR,
					   (uint8_t)log_announce_interval);
	}
	spibone_bus_unlock();
	return ret;
}

bool fpga_hal_read_ptp_leader_id(uint8_t leader_clock_id[8])
{
	uint32_t id_lo, id_hi;
	int ret;

	spibone_bus_lock();
	ret = spibone_read_locked(CSR_AES67_CSR_PTP_LEADER_ID_LO_ADDR, &id_lo);
	if (ret == 0) {
		ret = spibone_read_locked(CSR_AES67_CSR_PTP_LEADER_ID_HI_ADDR, &id_hi);
	}
	spibone_bus_unlock();

	if (ret < 0) {
		memset(leader_clock_id, 0, 8);
		return false;
	}

	leader_clock_id[0] = (uint8_t)(id_hi >> 24);
	leader_clock_id[1] = (uint8_t)(id_hi >> 16);
	leader_clock_id[2] = (uint8_t)(id_hi >> 8);
	leader_clock_id[3] = (uint8_t)(id_hi);
	leader_clock_id[4] = (uint8_t)(id_lo >> 24);
	leader_clock_id[5] = (uint8_t)(id_lo >> 16);
	leader_clock_id[6] = (uint8_t)(id_lo >> 8);
	leader_clock_id[7] = (uint8_t)(id_lo);

	return (id_lo | id_hi) != 0;
}

int fpga_hal_write_ptp_gm_quality(uint8_t priority1, uint8_t priority2,
				  uint8_t clock_class, uint8_t clock_accuracy)
{
	int ret;

	spibone_bus_lock();
	ret = spibone_write_locked(CSR_AES67_CSR_PTP_GM_PRIORITY1_ADDR, priority1);
	if (ret == 0) {
		ret = spibone_write_locked(CSR_AES67_CSR_PTP_GM_PRIORITY2_ADDR, priority2);
	}
	if (ret == 0) {
		ret = spibone_write_locked(CSR_AES67_CSR_PTP_GM_CLOCK_CLASS_ADDR, clock_class);
	}
	if (ret == 0) {
		ret = spibone_write_locked(CSR_AES67_CSR_PTP_GM_CLOCK_ACCURACY_ADDR,
					   clock_accuracy);
	}
	spibone_bus_unlock();
	return ret;
}

/* ---- Stream config RAMs ----
 *
 * One config byte per 32-bit Wishbone word (byte i of stream s at word index
 * s*32 + i), so a stream's record is one auto-incrementing burst write.
 * Byte layouts identical to eth_litex.c / the Rust aes67-config crate.
 */

int fpga_hal_write_tx_stream_config(uint8_t stream_id,
				    const struct in_addr *dst_ip,
				    uint8_t channel_count,
				    uint8_t samples_per_pkt,
				    const uint8_t *ch_ids,
				    uint8_t num_ch_ids,
				    uint32_t ssrc)
{
	uint8_t buf[20];
	const uint8_t *ip = (const uint8_t *)&dst_ip->s_addr;

	if (stream_id > 7 || num_ch_ids > 8) {
		return -EINVAL;
	}

	memset(buf, 0, sizeof(buf));
	buf[0] = stream_id & 0x07;
	buf[1] = ip[0];
	buf[2] = ip[1];
	buf[3] = ip[2];
	buf[4] = ip[3];
	buf[5] = channel_count;
	buf[6] = samples_per_pkt;
	for (uint8_t i = 0; i < num_ch_ids; i++) {
		buf[7 + i] = ch_ids[i];
	}
	/* Byte 15 reserved, bytes 16-19: SSRC (big-endian) */
	buf[16] = (uint8_t)(ssrc >> 24);
	buf[17] = (uint8_t)(ssrc >> 16);
	buf[18] = (uint8_t)(ssrc >> 8);
	buf[19] = (uint8_t)(ssrc);

	return spibone_write_burst(TX_STREAM_CFG_BASE + (((uint32_t)stream_id * 32) << 2),
				   buf, sizeof(buf));
}

int fpga_hal_write_rx_stream_config(uint8_t stream_id,
				    const struct in_addr *dst_ip,
				    uint16_t dst_port,
				    const uint8_t *ch_map,
				    uint8_t channel_count,
				    uint8_t output_delay,
				    uint8_t samples_per_channel)
{
	uint8_t buf[17];
	const uint8_t *ip = (const uint8_t *)&dst_ip->s_addr;

	memset(buf, 0, sizeof(buf));
	buf[0] = ip[0];
	buf[1] = ip[1];
	buf[2] = ip[2];
	buf[3] = ip[3];
	buf[4] = (uint8_t)(dst_port >> 8);
	buf[5] = (uint8_t)(dst_port);
	for (uint8_t i = 0; i < 8 && i < channel_count; i++) {
		buf[6 + i] = ch_map[i];
	}
	buf[14] = channel_count;
	buf[15] = output_delay;
	buf[16] = samples_per_channel;

	return spibone_write_burst(RX_STREAM_CFG_BASE + (((uint32_t)stream_id * 32) << 2),
				   buf, sizeof(buf));
}

/* ---- Control register ---- */

static uint32_t hal_to_ctrl_bits(uint32_t bits)
{
	uint32_t out = 0;

	if (bits & FPGA_HAL_CTRL_PPB_START) {
		out |= AES67_CTRL_PPB_START;
	}
	/* The FPGA_HAL_CTRL_RESET_* flags map to the dedicated reset CSR
	 * (fpga_hal_set_resets), not the control register — same as the
	 * LiteX backend. */
	return out;
}

static int ctrl_update(uint32_t set_mask, uint32_t clear_mask)
{
	uint32_t val;
	int ret;

	spibone_bus_lock();
	ret = spibone_read_locked(CSR_AES67_CSR_CTRL_ADDR, &val);
	if (ret == 0) {
		val &= ~clear_mask;
		val |= set_mask;
		ret = spibone_write_locked(CSR_AES67_CSR_CTRL_ADDR, val);
	}
	spibone_bus_unlock();
	return ret;
}

int fpga_hal_ctrl_set_bits(uint32_t bits)
{
	return ctrl_update(hal_to_ctrl_bits(bits), 0);
}

int fpga_hal_ctrl_clear_bits(uint32_t bits)
{
	return ctrl_update(0, hal_to_ctrl_bits(bits));
}

/* On external-MCU boards the card's nRST pin can be wired to an MCU GPIO
 * instead of the FPGA's adda_nrst CSR output — declared as an
 * "aes67,adda-nrst" node (label adda_nrst) in the board overlay. When the
 * node exists it replaces the CSR bit entirely; the GPIO also works before
 * the SPI bridge is up, which main() relies on to assert reset early. */
#define ADDA_NRST_NODE DT_NODELABEL(adda_nrst)

#if DT_NODE_EXISTS(ADDA_NRST_NODE)

static const struct gpio_dt_spec adda_nrst_gpio = GPIO_DT_SPEC_GET(ADDA_NRST_NODE, gpios);
static bool adda_nrst_configured;

int fpga_hal_set_adda_nrst(bool released)
{
	int ret;

	if (!gpio_is_ready_dt(&adda_nrst_gpio)) {
		return -ENODEV;
	}

	if (!adda_nrst_configured) {
		/* Come up asserted (active = in reset); the first caller is
		 * main() holding the card in reset anyway. */
		ret = gpio_pin_configure_dt(&adda_nrst_gpio, GPIO_OUTPUT_ACTIVE);
		if (ret < 0) {
			return ret;
		}
		adda_nrst_configured = true;
	}

	return gpio_pin_set_dt(&adda_nrst_gpio, released ? 0 : 1);
}

#else /* !DT_NODE_EXISTS(ADDA_NRST_NODE) */

int fpga_hal_set_adda_nrst(bool released)
{
	if (released) {
		return ctrl_update(AES67_CTRL_ADDA_NRST, 0);
	}
	return ctrl_update(0, AES67_CTRL_ADDA_NRST);
}

#endif /* DT_NODE_EXISTS(ADDA_NRST_NODE) */

/* ---- Status reads ---- */

uint32_t fpga_hal_read_status(void)
{
	uint32_t raw, hal = 0;
	uint32_t ppb_wc, ppb_pll;

	if (spibone_read(CSR_AES67_CSR_STATUS_ADDR, &raw) < 0) {
		return 0;
	}

	if (raw & AES67_STATUS_WC_LOCKED) {
		hal |= FPGA_HAL_CLK_WC_LOCKED;
	}
	if (raw & AES67_STATUS_WC_CONFIGURED) {
		hal |= FPGA_HAL_CLK_WC_CONFIGURED;
	}
	if (raw & AES67_STATUS_ETH_LINK_UP) {
		hal |= FPGA_HAL_ETH_LINK_UP;
	}
	if (raw & AES67_STATUS_PTP_IS_LEADER) {
		hal |= FPGA_HAL_PTP_IS_LEADER;
	}
	if (raw & AES67_STATUS_PTP_IS_FOLLOWER) {
		hal |= FPGA_HAL_PTP_IS_FOLLOWER;
	}

	uint32_t speed = (raw & AES67_STATUS_ETH_SPEED_MASK) >> AES67_STATUS_ETH_SPEED_SHIFT;

	hal |= (speed << FPGA_HAL_ETH_SPEED_SHIFT);

	if (fpga_hal_read_ppb_counts(&ppb_wc, &ppb_pll)) {
		hal |= FPGA_HAL_CLK_PPB_VALID;
	}

	return hal;
}

int32_t fpga_hal_read_path_delay(void)
{
	/* Always in the bridge CSR map; a software-PTP FPGA reads 0 and the
	 * caller sources the value from the Zephyr PTP stack instead. */
	uint32_t val;

	if (spibone_read(CSR_AES67_CSR_PTP_PATH_DELAY_ADDR, &val) < 0) {
		return 0;
	}
	return (int32_t)val;
}

int32_t fpga_hal_read_ptp_offset(void)
{
	uint32_t val;

	if (spibone_read(CSR_AES67_CSR_PTP_OFFSET_ADDR, &val) < 0) {
		return 0;
	}
	return (int32_t)val;
}

bool fpga_hal_read_ppb_counts(uint32_t *wc_count, uint32_t *pll_count)
{
	uint32_t status, wc, pll;
	int ret;

	*wc_count = 0;
	*pll_count = 0;

	spibone_bus_lock();
	ret = spibone_read_locked(CSR_AES67_CSR_PLL_PPB_STATUS_ADDR, &status);
	if (ret == 0) {
		ret = spibone_read_locked(CSR_AES67_CSR_PLL_PPB_WC_COUNT_ADDR, &wc);
	}
	if (ret == 0) {
		ret = spibone_read_locked(CSR_AES67_CSR_PLL_PPB_PLL_COUNT_ADDR, &pll);
	}
	spibone_bus_unlock();

	if (ret < 0) {
		return false;
	}
	*wc_count = wc & 0x3FFFFF;
	*pll_count = pll & 0x3FFFFF;
	return (status & AES67_PPB_STATUS_VALID) != 0;
}

/* ---- PTP servo / parser tuning + monitoring ---- */

int fpga_hal_write_ptp_tuning(const struct fpga_hal_ptp_tuning *t)
{
	int ret = 0;

	if (!t) {
		return -EINVAL;
	}

	spibone_bus_lock();
	/* Servo tuning CSRs exist only when the aes67_bridge is built with the
	 * servo (dropped via --no-servo-csr); the parser CSRs are always there. */
#ifdef CSR_AES67_CSR_SERVO_KP_GAIN_ADDR
	ret |= spibone_write_locked(CSR_AES67_CSR_SERVO_KP_GAIN_ADDR, (uint8_t)t->kp_gain);
	ret |= spibone_write_locked(CSR_AES67_CSR_SERVO_KI_GAIN_ADDR, (uint8_t)t->ki_gain);
	ret |= spibone_write_locked(CSR_AES67_CSR_SERVO_GAIN_SHIFT_ADDR, t->gain_shift & 0x1F);
	ret |= spibone_write_locked(CSR_AES67_CSR_SERVO_GAIN_SHIFT_LOCKED_ADDR,
				    t->gain_shift_locked & 0x1F);
	ret |= spibone_write_locked(CSR_AES67_CSR_SERVO_KI_EXTRA_SHIFT_ADDR,
				    t->ki_extra_shift & 0x1F);
	ret |= spibone_write_locked(CSR_AES67_CSR_SERVO_FILTER_SHIFT_ADDR,
				    t->filter_shift & 0x1F);
	ret |= spibone_write_locked(CSR_AES67_CSR_SERVO_WARMUP_SAMPLES_ADDR, t->warmup_samples);
	ret |= spibone_write_locked(CSR_AES67_CSR_SERVO_LOCK_THRESHOLD_NS_ADDR,
				    t->lock_threshold_ns);
	ret |= spibone_write_locked(CSR_AES67_CSR_SERVO_UNLOCK_THRESHOLD_NS_ADDR,
				    t->unlock_threshold_ns);
	ret |= spibone_write_locked(CSR_AES67_CSR_SERVO_LOCK_COUNT_THRESHOLD_ADDR,
				    t->lock_count_threshold);
#endif

	uint32_t mf = (t->min_filter_enable ? 1u : 0u) |
		      ((uint32_t)t->min_filter_active_depth << 8);

	ret |= spibone_write_locked(CSR_AES67_CSR_PARSER_MIN_FILTER_ADDR, mf);
	ret |= spibone_write_locked(CSR_AES67_CSR_PARSER_DELAY_ASYMMETRY_NS_ADDR,
				    (uint32_t)t->delay_asymmetry_ns);
	spibone_bus_unlock();

	return ret < 0 ? -EIO : 0;
}

/* Locked read helper for the bulk getters: on error, report 0. */
static uint32_t rd0(uint32_t addr)
{
	uint32_t val;

	if (spibone_read_locked(addr, &val) < 0) {
		return 0;
	}
	return val;
}

void fpga_hal_read_ptp_tuning(struct fpga_hal_ptp_tuning *t)
{
	if (!t) {
		return;
	}
	memset(t, 0, sizeof(*t));

	spibone_bus_lock();
#ifdef CSR_AES67_CSR_SERVO_KP_GAIN_ADDR
	t->kp_gain              = (int8_t)rd0(CSR_AES67_CSR_SERVO_KP_GAIN_ADDR);
	t->ki_gain              = (int8_t)rd0(CSR_AES67_CSR_SERVO_KI_GAIN_ADDR);
	t->gain_shift           = (uint8_t)(rd0(CSR_AES67_CSR_SERVO_GAIN_SHIFT_ADDR) & 0x1F);
	t->gain_shift_locked    = (uint8_t)(rd0(CSR_AES67_CSR_SERVO_GAIN_SHIFT_LOCKED_ADDR) & 0x1F);
	t->ki_extra_shift       = (uint8_t)(rd0(CSR_AES67_CSR_SERVO_KI_EXTRA_SHIFT_ADDR) & 0x1F);
	t->filter_shift         = (uint8_t)(rd0(CSR_AES67_CSR_SERVO_FILTER_SHIFT_ADDR) & 0x1F);
	t->warmup_samples       = (uint8_t)rd0(CSR_AES67_CSR_SERVO_WARMUP_SAMPLES_ADDR);
	t->lock_threshold_ns    = rd0(CSR_AES67_CSR_SERVO_LOCK_THRESHOLD_NS_ADDR);
	t->unlock_threshold_ns  = rd0(CSR_AES67_CSR_SERVO_UNLOCK_THRESHOLD_NS_ADDR);
	t->lock_count_threshold = (uint8_t)rd0(CSR_AES67_CSR_SERVO_LOCK_COUNT_THRESHOLD_ADDR);
#endif

	uint32_t mf = rd0(CSR_AES67_CSR_PARSER_MIN_FILTER_ADDR);

	t->min_filter_enable       = (mf & 0x1) != 0;
	t->min_filter_active_depth = (uint8_t)((mf >> 8) & 0xFF);
	t->delay_asymmetry_ns      = (int32_t)rd0(CSR_AES67_CSR_PARSER_DELAY_ASYMMETRY_NS_ADDR);
	spibone_bus_unlock();
}

void fpga_hal_read_ptp_monitor(struct fpga_hal_ptp_monitor *m)
{
	if (!m) {
		return;
	}
	memset(m, 0, sizeof(*m));

#ifdef CSR_AES67_CSR_SERVO_MON_STATUS_ADDR
	spibone_bus_lock();
	m->filtered_offset = (int32_t)rd0(CSR_AES67_CSR_SERVO_MON_FILTERED_OFFSET_ADDR);
	m->integral_sum    = (int32_t)rd0(CSR_AES67_CSR_SERVO_MON_INTEGRAL_SUM_ADDR);
	m->pi_proportional = (int32_t)rd0(CSR_AES67_CSR_SERVO_MON_PI_PROPORTIONAL_ADDR);
	m->pi_sum_raw      = (int32_t)rd0(CSR_AES67_CSR_SERVO_MON_PI_SUM_RAW_ADDR);

	uint32_t status = rd0(CSR_AES67_CSR_SERVO_MON_STATUS_ADDR);

	m->effective_gain_shift = (uint8_t)(status & 0xFF);
	m->lock_counter         = (uint16_t)((status >> 8) & 0xFFFF);
	m->first_lock_achieved  = ((status >> 24) & 0x1) != 0;

	m->sample_count = (uint16_t)rd0(CSR_AES67_CSR_SERVO_MON_SAMPLE_COUNT_ADDR);
	spibone_bus_unlock();
#endif
}

/* ---- Reset domains ---- */

static int reset_update(uint32_t csr_bits, bool held)
{
	uint32_t reg;
	int ret;

	spibone_bus_lock();
	ret = spibone_read_locked(CSR_AES67_CSR_RESET_ADDR, &reg);
	if (ret == 0) {
		if (held) {
			reg |= csr_bits;
		} else {
			reg &= ~csr_bits;
		}
		ret = spibone_write_locked(CSR_AES67_CSR_RESET_ADDR, reg);
	}
	spibone_bus_unlock();
	return ret;
}

int fpga_hal_set_ptp_reset(bool held_in_reset)
{
	return reset_update(BIT(CSR_AES67_CSR_RESET_PTP_OFFSET), held_in_reset);
}

int fpga_hal_set_resets(uint32_t domains, bool held)
{
	uint32_t csr_bits = 0;

	if (domains & FPGA_HAL_RESET_PTP) {
		csr_bits |= BIT(CSR_AES67_CSR_RESET_PTP_OFFSET);
	}
	if (domains & FPGA_HAL_RESET_TX) {
		csr_bits |= BIT(CSR_AES67_CSR_RESET_TX_OFFSET);
	}
	if (domains & FPGA_HAL_RESET_RX) {
		csr_bits |= BIT(CSR_AES67_CSR_RESET_RX_OFFSET);
	}
	if (domains & FPGA_HAL_RESET_ETH) {
		csr_bits |= BIT(CSR_AES67_CSR_RESET_ETH_OFFSET);
	}

	return reset_update(csr_bits, held);
}

/* ---- Audio metering ---- */

void fpga_hal_read_metering(uint16_t *rx_signal, uint16_t *rx_clip,
			    uint16_t *tx_signal, uint16_t *tx_clip)
{
	/* Metering CSRs exist only when the aes67_bridge is built with metering
	 * (dropped via --no-metering-csr). Without them, report zeros. */
#ifdef CSR_AES67_CSR_RX_METER_SIGNAL_ADDR
	spibone_bus_lock();
	*rx_signal = (uint16_t)rd0(CSR_AES67_CSR_RX_METER_SIGNAL_ADDR);
	*rx_clip   = (uint16_t)rd0(CSR_AES67_CSR_RX_METER_CLIP_ADDR);
	*tx_signal = (uint16_t)rd0(CSR_AES67_CSR_TX_METER_SIGNAL_ADDR);
	*tx_clip   = (uint16_t)rd0(CSR_AES67_CSR_TX_METER_CLIP_ADDR);

	/* Toggle the clear bit so the FPGA resets its sticky detectors. */
	uint32_t cur = rd0(CSR_AES67_CSR_METER_CLEAR_ADDR);

	(void)spibone_write_locked(CSR_AES67_CSR_METER_CLEAR_ADDR, cur ^ 1);
	spibone_bus_unlock();
#else
	*rx_signal = 0;
	*rx_clip   = 0;
	*tx_signal = 0;
	*tx_clip   = 0;
#endif
}

/* ---- spibone bus sanity probe ---- */

#ifdef CONFIG_AES67_SPIBONE_PROBE
/*
 * Confirm the FPGA actually answers over spibone before the rest of the app
 * relies on it. main() calls this explicitly during boot bring-up, right
 * after the FPGA has been configured over JTAG and before it releases the
 * reset domains — which itself reads the reset CSR over this bus, so a dead
 * bus would otherwise fail there without a clear cause.
 *
 * The scratch registers are pure storage with no side effects and are
 * reachable regardless of the AES67 reset domains (they sit in the CSR
 * infrastructure, not the datapath).
 *
 * Returns 0 if the bus round-trips cleanly, -EIO otherwise.
 */
int fpga_hal_spibone_probe(void)
{
	static const uint32_t patterns[] = {
		0x12345678u, 0xA5A5A5A5u, 0x00000000u, 0xFFFFFFFFu,
	};
	uint32_t v;
	int ret;
	bool ok = true;

#ifdef CSR_CTRL_SCRATCH_ADDR
	/* Known-constant test, no write needed: the LiteX control block's
	 * scratch register powers up as 0x12345678. */
	ret = spibone_read(CSR_CTRL_SCRATCH_ADDR, &v);
	if (ret < 0) {
		LOG_ERR("spibone probe: ctrl_scratch read failed (err %d) — "
			"bus not responding", ret);
		ok = false;
	} else if (v != 0x12345678u) {
		LOG_ERR("spibone probe: ctrl_scratch = 0x%08X, expected "
			"0x12345678 — bus/address-decode fault", v);
		ok = false;
	} else {
		LOG_INF("spibone probe: ctrl_scratch = 0x12345678 (OK)");
	}
#endif

	/* Round-trip patterns through the AES67 scratch register: this also
	 * exercises the data lines in both directions and catches stuck or
	 * swapped bits that a single constant read could miss. */
	for (size_t i = 0; i < ARRAY_SIZE(patterns) && ok; i++) {
		ret = spibone_write(CSR_AES67_CSR_SCRATCH_ADDR, patterns[i]);
		if (ret == 0) {
			ret = spibone_read(CSR_AES67_CSR_SCRATCH_ADDR, &v);
		}
		if (ret < 0) {
			LOG_ERR("spibone probe: scratch transfer failed "
				"(err %d)", ret);
			ok = false;
		} else if (v != patterns[i]) {
			LOG_ERR("spibone probe: scratch wrote 0x%08X read 0x%08X"
				" — data-line fault", patterns[i], v);
			ok = false;
		}
	}

	if (ok) {
		LOG_INF("spibone probe: bus OK (scratch round-trip verified)");
	} else {
		LOG_ERR("spibone probe: FAILED — the FPGA is configured but not "
			"answering on the Wishbone bus. Check the SPI2 wiring "
			"(SCK/MOSI/MISO/CS), the bus frequency, and that the "
			"gateware's spibone clock is running.");
	}

	return ok ? 0 : -EIO;
}
#endif /* CONFIG_AES67_SPIBONE_PROBE */
