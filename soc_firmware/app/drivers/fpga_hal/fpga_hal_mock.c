/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * FPGA HAL mock backend — QEMU / CI builds without hardware.
 *
 * Emulates the aes67_bridge CSR map far enough for the whole control
 * plane (aes67_conn, discovery, NMOS, web API, ptp_bmc, fpga_poll) to
 * run unmodified:
 *  - a sparse CSR register file backs fpga_hal_csr_read/write, so raw
 *    peek/poke (shell, debug) round-trips,
 *  - the system_cfg CSRs are pre-populated from Kconfig, packed with
 *    the same generated field macros the real gateware uses — the
 *    backend-neutral fpga_hal_syscfg.c parses them unchanged,
 *  - the PTP wallclock CSRs synthesize a monotonic TAI clock from
 *    k_uptime (latched on the SECONDS_LO read, so the firmware's
 *    coherent-snapshot protocol works),
 *  - everything else (stream configs, resets, PTP tuning) is stored
 *    and read back, status reads report link-up / wallclock-locked.
 *
 * The register map addresses come from the generated aes67_bridge
 * headers (litex_soc/build/aes67_bridge) — identical to the SPI
 * backend, so no address is hard-coded here either.
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/sys_io.h>
#include <string.h>

#include "fpga_hal.h"
#include "../eth_litex/eth_litex.h"

LOG_MODULE_REGISTER(fpga_hal_mock, LOG_LEVEL_INF);

/* ================================================================
 * Sparse CSR register file
 * ================================================================ */

#define MOCK_REGFILE_SIZE 128

static struct {
	uint32_t addr;
	uint32_t val;
	bool used;
} regfile[MOCK_REGFILE_SIZE];

static struct k_spinlock reg_lock;

static int regfile_set(uint32_t addr, uint32_t val)
{
	k_spinlock_key_t key = k_spin_lock(&reg_lock);
	int free_slot = -1;

	for (int i = 0; i < MOCK_REGFILE_SIZE; i++) {
		if (regfile[i].used && regfile[i].addr == addr) {
			regfile[i].val = val;
			k_spin_unlock(&reg_lock, key);
			return 0;
		}
		if (!regfile[i].used && free_slot < 0) {
			free_slot = i;
		}
	}
	if (free_slot < 0) {
		k_spin_unlock(&reg_lock, key);
		return -ENOMEM;
	}
	regfile[free_slot].addr = addr;
	regfile[free_slot].val = val;
	regfile[free_slot].used = true;
	k_spin_unlock(&reg_lock, key);
	return 0;
}

static uint32_t regfile_get(uint32_t addr)
{
	k_spinlock_key_t key = k_spin_lock(&reg_lock);
	uint32_t val = 0;

	for (int i = 0; i < MOCK_REGFILE_SIZE; i++) {
		if (regfile[i].used && regfile[i].addr == addr) {
			val = regfile[i].val;
			break;
		}
	}
	k_spin_unlock(&reg_lock, key);
	return val;
}

/* ================================================================
 * Wallclock emulation
 *
 * TAI = boot epoch + uptime. The full timestamp is latched when
 * SECONDS_LO is read; HI and NANOSECONDS are served from the latch —
 * exactly the coherence the lo/hi/ns/lo/hi re-read protocol expects.
 * ================================================================ */

/* Fallback epoch, roughly "now" (2026, TAI). Only the monotonicity
 * matters for the consumers (NMOS version timestamps, scheduling). */
#define MOCK_TAI_EPOCH 1785000000ULL

/* UTC -> TAI offset (leap seconds), matching what nmos-testing and real
 * PTP grandmasters use. */
#define TAI_UTC_OFFSET 37

static uint64_t wc_latch_sec;
static uint32_t wc_latch_ns;

static void wallclock_now(uint64_t *sec, uint32_t *nsec)
{
#ifdef CONFIG_FPGA_HAL_MOCK_GOLDFISH_RTC
	/* QEMU's virt machine carries a Goldfish RTC at a fixed MMIO
	 * address (no guest DT node, no Zephyr driver): TIME_LOW latches
	 * the host CLOCK_REALTIME in ns, TIME_HIGH completes it. Serving
	 * host time (+ leap seconds) makes the wallclock agree with the
	 * test host's TAI — absolute scheduled activations (IS-05/IS-08
	 * conformance tests) depend on that to within 100 ms. */
	uint32_t lo = sys_read32(CONFIG_FPGA_HAL_MOCK_GOLDFISH_RTC_BASE);
	uint32_t hi = sys_read32(CONFIG_FPGA_HAL_MOCK_GOLDFISH_RTC_BASE + 4);
	uint64_t ns = ((uint64_t)hi << 32) | lo;

	*sec = ns / NSEC_PER_SEC + TAI_UTC_OFFSET;
	*nsec = (uint32_t)(ns % NSEC_PER_SEC);
#else
	int64_t up_ms = k_uptime_get();

	*sec = MOCK_TAI_EPOCH + (uint64_t)(up_ms / 1000);
	*nsec = (uint32_t)(up_ms % 1000) * 1000000u;
#endif
}

/* ================================================================
 * CSR read/write
 * ================================================================ */

int fpga_hal_csr_read(uint32_t addr, uint32_t *val)
{
	if (val == NULL) {
		return -EINVAL;
	}

	switch (addr) {
	case CSR_AES67_CSR_WALLCLOCK_SECONDS_IN_LO_ADDR:
		wallclock_now(&wc_latch_sec, &wc_latch_ns);
		*val = (uint32_t)(wc_latch_sec & 0xFFFFFFFFu);
		return 0;
	case CSR_AES67_CSR_WALLCLOCK_SECONDS_IN_HI_ADDR:
		*val = (uint32_t)(wc_latch_sec >> 32) & 0xFFFF;
		return 0;
	case CSR_AES67_CSR_WALLCLOCK_NANOSECONDS_IN_ADDR:
		*val = wc_latch_ns & 0x3FFFFFFF;
		return 0;
	default:
		*val = regfile_get(addr);
		return 0;
	}
}

int fpga_hal_csr_write(uint32_t addr, uint32_t val)
{
	return regfile_set(addr, val);
}

/* ================================================================
 * Ready / recovery
 * ================================================================ */

const struct device *fpga_hal_get_dev(void)
{
	return NULL;
}

bool fpga_hal_is_ready(void)
{
	return true;
}

int fpga_hal_wait_ready(uint32_t timeout_ms)
{
	ARG_UNUSED(timeout_ms);
	return 0;
}

void fpga_hal_register_recover_cb(fpga_hal_recover_cb_t cb, void *user_data)
{
	/* The mock FPGA never resets. */
	ARG_UNUSED(cb);
	ARG_UNUSED(user_data);
}

/* ================================================================
 * Configuration writes (stored, logged, acknowledged)
 * ================================================================ */

static struct {
	uint8_t mac[6];
	struct in_addr ip;
	uint8_t gm_priority1, gm_priority2, gm_class, gm_accuracy;
	uint32_t ctrl_bits;
	uint32_t reset_bits;
	bool adda_nrst_released;
	bool ptp_reset_held;
	struct fpga_hal_ptp_tuning tuning;
} mock;

int fpga_hal_write_mac(const uint8_t mac[6])
{
	memcpy(mock.mac, mac, 6);
	return 0;
}

int fpga_hal_write_ip(const struct in_addr *ip)
{
	mock.ip = *ip;
	return 0;
}

int fpga_hal_write_ptp_config(uint8_t time_source, int8_t log_msg_interval,
			      int8_t log_announce_interval)
{
	ARG_UNUSED(time_source);
	ARG_UNUSED(log_msg_interval);
	ARG_UNUSED(log_announce_interval);
	return 0;
}

bool fpga_hal_read_ptp_leader_id(uint8_t leader_clock_id[8])
{
	/* No BMA in the mock: no foreign leader is ever elected — the
	 * node acts as grandmaster (ptp_bmc handles the announcing). */
	memset(leader_clock_id, 0, 8);
	return false;
}

int fpga_hal_write_ptp_gm_quality(uint8_t priority1, uint8_t priority2,
				  uint8_t clock_class, uint8_t clock_accuracy)
{
	mock.gm_priority1 = priority1;
	mock.gm_priority2 = priority2;
	mock.gm_class = clock_class;
	mock.gm_accuracy = clock_accuracy;
	return 0;
}

int fpga_hal_write_tx_stream_config(uint8_t stream_id,
				    const struct in_addr *dst_ip,
				    uint8_t channel_count,
				    uint8_t samples_per_pkt,
				    const uint8_t *ch_ids,
				    uint8_t num_ch_ids,
				    uint32_t ssrc)
{
	ARG_UNUSED(dst_ip);
	ARG_UNUSED(ch_ids);
	ARG_UNUSED(num_ch_ids);
	ARG_UNUSED(ssrc);
	LOG_DBG("tx stream %u: ch=%u spp=%u", stream_id, channel_count,
		samples_per_pkt);
	return 0;
}

int fpga_hal_write_rx_stream_config(uint8_t stream_id,
				    const struct in_addr *dst_ip,
				    uint16_t dst_port,
				    const uint8_t *ch_map,
				    uint8_t channel_count,
				    uint8_t output_delay,
				    uint8_t samples_per_channel)
{
	ARG_UNUSED(dst_ip);
	ARG_UNUSED(ch_map);
	LOG_DBG("rx stream %u: port=%u ch=%u delay=%u spc=%u", stream_id,
		dst_port, channel_count, output_delay, samples_per_channel);
	return 0;
}

/* ================================================================
 * Control / resets
 * ================================================================ */

int fpga_hal_ctrl_set_bits(uint32_t bits)
{
	mock.ctrl_bits |= bits;
	return 0;
}

int fpga_hal_ctrl_clear_bits(uint32_t bits)
{
	mock.ctrl_bits &= ~bits;
	return 0;
}

int fpga_hal_set_adda_nrst(bool released)
{
	mock.adda_nrst_released = released;
	return 0;
}

int fpga_hal_set_resets(uint32_t domains, bool held)
{
	if (held) {
		mock.reset_bits |= domains;
	} else {
		mock.reset_bits &= ~domains;
	}
	return 0;
}

int fpga_hal_set_ptp_reset(bool held_in_reset)
{
	mock.ptp_reset_held = held_in_reset;
	return 0;
}

/* ================================================================
 * Status / measurements
 * ================================================================ */

uint32_t fpga_hal_read_status(void)
{
	/* Link up at 100 Mbit, wallclock configured + locked, PPB valid,
	 * PTP leader (matches read_ptp_leader_id: no foreign leader). */
	return FPGA_HAL_ETH_LINK_UP |
	       (1u << FPGA_HAL_ETH_SPEED_SHIFT) |
	       FPGA_HAL_CLK_WC_CONFIGURED |
	       FPGA_HAL_CLK_WC_LOCKED |
	       FPGA_HAL_CLK_PPB_VALID |
	       FPGA_HAL_PTP_IS_LEADER;
}

int32_t fpga_hal_read_path_delay(void)
{
	return 0;
}

int32_t fpga_hal_read_ptp_offset(void)
{
	return 0;
}

bool fpga_hal_read_ppb_counts(uint32_t *wc_count, uint32_t *pll_count)
{
	/* Identical counters: measured PPB error = 0. */
	*wc_count = 0x200000;
	*pll_count = 0x200000;
	return true;
}

int fpga_hal_write_ptp_tuning(const struct fpga_hal_ptp_tuning *t)
{
	mock.tuning = *t;
	return 0;
}

void fpga_hal_read_ptp_tuning(struct fpga_hal_ptp_tuning *t)
{
	*t = mock.tuning;
}

void fpga_hal_read_ptp_monitor(struct fpga_hal_ptp_monitor *m)
{
	memset(m, 0, sizeof(*m));
	m->first_lock_achieved = true;
}

void fpga_hal_read_metering(uint16_t *rx_signal, uint16_t *rx_clip,
			    uint16_t *tx_signal, uint16_t *tx_clip)
{
	*rx_signal = 0;
	*rx_clip = 0;
	*tx_signal = 0;
	*tx_clip = 0;
}

/* ================================================================
 * Init: pre-populate the system_cfg CSRs from Kconfig
 * ================================================================ */

#define SYSCFG_PACK(reg, field, v)                                          \
	(((uint32_t)(v) &                                                   \
	  ((1UL << CSR_AES67_CSR_SYSTEM_CFG_##reg##_##field##_SIZE) - 1))   \
	 << CSR_AES67_CSR_SYSTEM_CFG_##reg##_##field##_OFFSET)

static int fpga_hal_mock_init(void)
{
	uint32_t flags = SYSCFG_PACK(FLAGS, PTP_IN_SOFTWARE, 0) |
			 SYSCFG_PACK(FLAGS, STATIC_PTP_CONFIG, 0) |
			 SYSCFG_PACK(FLAGS, METERING, 1);
	uint32_t rx = SYSCFG_PACK(RX, MAX_STREAMS,
				  CONFIG_FPGA_HAL_MOCK_RX_STREAMS) |
		      SYSCFG_PACK(RX, CHANNELS,
				  CONFIG_FPGA_HAL_MOCK_RX_CHANNELS) |
		      SYSCFG_PACK(RX, BUFFER_DEPTH, 256);
	uint32_t tx = SYSCFG_PACK(TX, MAX_STREAMS,
				  CONFIG_FPGA_HAL_MOCK_TX_STREAMS) |
		      SYSCFG_PACK(TX, CHANNELS,
				  CONFIG_FPGA_HAL_MOCK_TX_CHANNELS) |
		      SYSCFG_PACK(TX, BUFFER_DEPTH, 64);

	(void)regfile_set(CSR_AES67_CSR_SYSTEM_CFG_FLAGS_ADDR, flags);
	(void)regfile_set(CSR_AES67_CSR_SYSTEM_CFG_RX_ADDR, rx);
	(void)regfile_set(CSR_AES67_CSR_SYSTEM_CFG_TX_ADDR, tx);

	/* The FPGA powers up with every reset domain held. */
	mock.reset_bits = FPGA_HAL_RESET_ALL;

	LOG_INF("FPGA HAL mock: TX %d streams/%d ch, RX %d streams/%d ch",
		CONFIG_FPGA_HAL_MOCK_TX_STREAMS,
		CONFIG_FPGA_HAL_MOCK_TX_CHANNELS,
		CONFIG_FPGA_HAL_MOCK_RX_STREAMS,
		CONFIG_FPGA_HAL_MOCK_RX_CHANNELS);
	return 0;
}

SYS_INIT(fpga_hal_mock_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);
