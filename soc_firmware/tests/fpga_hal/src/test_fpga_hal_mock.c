/*
 * Unit tests for the FPGA HAL mock backend
 * (drivers/fpga_hal/fpga_hal_mock.c).
 *
 * Every QEMU-hosted test — including the AMWA NMOS conformance runs —
 * asserts against this emulation, so its own contract needs holding
 * down: register round-trip, the coherent wallclock snapshot protocol
 * and the status word the control plane keys its state machines off.
 */

#include <zephyr/ztest.h>
#include <errno.h>
#include <string.h>

#include "fpga_hal.h"
#include "eth_litex/eth_litex.h"

/* A scratch address the emulation does not special-case. */
#define SCRATCH_ADDR CSR_AES67_CSR_SCRATCH_ADDR

ZTEST(fpga_hal_mock, test_csr_roundtrip)
{
	uint32_t v = 0;

	zassert_equal(fpga_hal_csr_write(SCRATCH_ADDR, 0xDEADBEEF), 0);
	zassert_equal(fpga_hal_csr_read(SCRATCH_ADDR, &v), 0);
	zassert_equal(v, 0xDEADBEEF);

	/* Overwriting reuses the slot rather than consuming a new one. */
	zassert_equal(fpga_hal_csr_write(SCRATCH_ADDR, 0), 0);
	zassert_equal(fpga_hal_csr_read(SCRATCH_ADDR, &v), 0);
	zassert_equal(v, 0);

	/* Reads of never-written registers are zero, not garbage. */
	v = 0x5A5A5A5A;
	zassert_equal(fpga_hal_csr_read(SCRATCH_ADDR + 0x400, &v), 0);
	zassert_equal(v, 0);

	zassert_equal(fpga_hal_csr_read(SCRATCH_ADDR, NULL), -EINVAL);
}

ZTEST(fpga_hal_mock, test_wallclock_snapshot_is_coherent)
{
	uint32_t lo, hi, ns, hi2, ns2;

	/* The firmware's read protocol: SECONDS_LO latches the whole
	 * timestamp, HI and NANOSECONDS are then served from that latch,
	 * so re-reading them cannot straddle a second boundary. */
	zassert_equal(fpga_hal_csr_read(
		CSR_AES67_CSR_WALLCLOCK_SECONDS_IN_LO_ADDR, &lo), 0);
	zassert_equal(fpga_hal_csr_read(
		CSR_AES67_CSR_WALLCLOCK_SECONDS_IN_HI_ADDR, &hi), 0);
	zassert_equal(fpga_hal_csr_read(
		CSR_AES67_CSR_WALLCLOCK_NANOSECONDS_IN_ADDR, &ns), 0);

	k_sleep(K_MSEC(50));

	zassert_equal(fpga_hal_csr_read(
		CSR_AES67_CSR_WALLCLOCK_SECONDS_IN_HI_ADDR, &hi2), 0);
	zassert_equal(fpga_hal_csr_read(
		CSR_AES67_CSR_WALLCLOCK_NANOSECONDS_IN_ADDR, &ns2), 0);
	zassert_equal(hi, hi2, "HI must come from the latch, not live time");
	zassert_equal(ns, ns2, "NS must come from the latch, not live time");

	zassert_true(ns < 1000000000u, "nanoseconds out of range: %u", ns);
	/* The CSR is 30 bits wide. */
	zassert_equal(ns & ~0x3FFFFFFFu, 0);
}

ZTEST(fpga_hal_mock, test_wallclock_advances)
{
	uint32_t lo1, ns1, lo2, ns2, hi;
	uint64_t t1, t2;

	zassert_equal(fpga_hal_csr_read(
		CSR_AES67_CSR_WALLCLOCK_SECONDS_IN_LO_ADDR, &lo1), 0);
	zassert_equal(fpga_hal_csr_read(
		CSR_AES67_CSR_WALLCLOCK_NANOSECONDS_IN_ADDR, &ns1), 0);

	k_sleep(K_MSEC(250));

	zassert_equal(fpga_hal_csr_read(
		CSR_AES67_CSR_WALLCLOCK_SECONDS_IN_LO_ADDR, &lo2), 0);
	zassert_equal(fpga_hal_csr_read(
		CSR_AES67_CSR_WALLCLOCK_SECONDS_IN_HI_ADDR, &hi), 0);
	zassert_equal(fpga_hal_csr_read(
		CSR_AES67_CSR_WALLCLOCK_NANOSECONDS_IN_ADDR, &ns2), 0);

	t1 = (uint64_t)lo1 * 1000000000ull + ns1;
	t2 = (uint64_t)lo2 * 1000000000ull + ns2;
	zassert_true(t2 > t1, "wallclock did not advance");
	zassert_true(t2 - t1 >= 200000000ull,
		     "wallclock advanced by only %llu ns over 250 ms",
		     (unsigned long long)(t2 - t1));

	/* Seconds are a 48-bit field: the high word must stay in range. */
	zassert_equal(hi & ~0xFFFFu, 0);
}

ZTEST(fpga_hal_mock, test_status_word)
{
	uint32_t st = fpga_hal_read_status();

	/* The emulated node presents a usable link and a locked wallclock,
	 * which is what main() and fpga_poll gate the bring-up on. */
	zassert_true(st & FPGA_HAL_ETH_LINK_UP);
	zassert_true(st & FPGA_HAL_CLK_WC_CONFIGURED);
	zassert_true(st & FPGA_HAL_CLK_WC_LOCKED);
	zassert_true(st & FPGA_HAL_CLK_PPB_VALID);
	/* No foreign grandmaster exists in the mock, so it is the leader —
	 * this must agree with read_ptp_leader_id() below. */
	zassert_true(st & FPGA_HAL_PTP_IS_LEADER);
	zassert_false(st & FPGA_HAL_CLK_PTP_LEADER_LOST);
	zassert_equal((st >> FPGA_HAL_ETH_SPEED_SHIFT) & 0x3, 1,
		      "expected 100 Mbit/s link speed");
}

ZTEST(fpga_hal_mock, test_no_foreign_leader)
{
	uint8_t id[8];

	memset(id, 0xFF, sizeof(id));
	zassert_false(fpga_hal_read_ptp_leader_id(id));
	for (int i = 0; i < 8; i++) {
		zassert_equal(id[i], 0, "leader id must be cleared");
	}
}

ZTEST(fpga_hal_mock, test_ready_and_recovery)
{
	zassert_true(fpga_hal_is_ready());
	zassert_equal(fpga_hal_wait_ready(0), 0);
	zassert_equal(fpga_hal_wait_ready(1000), 0);
	zassert_is_null(fpga_hal_get_dev());
	/* The mock never resets; registering a callback is a no-op, not a
	 * crash. */
	fpga_hal_register_recover_cb(NULL, NULL);
}

ZTEST(fpga_hal_mock, test_ptp_tuning_roundtrip)
{
	struct fpga_hal_ptp_tuning in = {
		.kp_gain = -3,
		.ki_gain = 7,
		.gain_shift = 10,
		.gain_shift_locked = 16,
		.ki_extra_shift = 2,
		.filter_shift = 4,
		.warmup_samples = 5,
		.lock_threshold_ns = 1000,
		.unlock_threshold_ns = 5000,
		.lock_count_threshold = 8,
		.min_filter_enable = true,
		.min_filter_active_depth = 3,
		.delay_asymmetry_ns = -250,
	};
	struct fpga_hal_ptp_tuning out;

	memset(&out, 0, sizeof(out));
	zassert_equal(fpga_hal_write_ptp_tuning(&in), 0);
	fpga_hal_read_ptp_tuning(&out);

	zassert_equal(out.kp_gain, in.kp_gain);
	zassert_equal(out.ki_gain, in.ki_gain);
	zassert_equal(out.gain_shift, in.gain_shift);
	zassert_equal(out.gain_shift_locked, in.gain_shift_locked);
	zassert_equal(out.lock_threshold_ns, in.lock_threshold_ns);
	zassert_equal(out.unlock_threshold_ns, in.unlock_threshold_ns);
	zassert_equal(out.min_filter_enable, in.min_filter_enable);
	zassert_equal(out.delay_asymmetry_ns, in.delay_asymmetry_ns);
}

ZTEST(fpga_hal_mock, test_measurements_are_neutral)
{
	uint32_t wc, pll;
	uint16_t rx_sig, rx_clip, tx_sig, tx_clip;
	struct fpga_hal_ptp_monitor mon;

	/* Identical edge counters = zero measured PPB error, so the PI
	 * controller in pll_ctrl has nothing to chase in CI. */
	zassert_true(fpga_hal_read_ppb_counts(&wc, &pll));
	zassert_equal(wc, pll);
	zassert_not_equal(wc, 0, "a zero wallclock count would mean no PPB");

	zassert_equal(fpga_hal_read_path_delay(), 0);
	zassert_equal(fpga_hal_read_ptp_offset(), 0);

	fpga_hal_read_metering(&rx_sig, &rx_clip, &tx_sig, &tx_clip);
	zassert_equal(rx_sig, 0);
	zassert_equal(rx_clip, 0);
	zassert_equal(tx_sig, 0);
	zassert_equal(tx_clip, 0);

	memset(&mon, 0xFF, sizeof(mon));
	fpga_hal_read_ptp_monitor(&mon);
	zassert_true(mon.first_lock_achieved);
}

ZTEST(fpga_hal_mock, test_config_writes_are_accepted)
{
	const uint8_t mac[6] = { 0x02, 0xAA, 0xE6, 0x70, 0x00, 0x01 };
	const uint8_t ch[2] = { 0, 1 };
	struct in_addr ip;

	zassert_equal(net_addr_pton(AF_INET, "192.168.1.50", &ip), 0);

	zassert_equal(fpga_hal_write_mac(mac), 0);
	zassert_equal(fpga_hal_write_ip(&ip), 0);
	zassert_equal(fpga_hal_write_ptp_config(0, -3, 0), 0);
	zassert_equal(fpga_hal_write_ptp_gm_quality(128, 128, 248, 0xFE), 0);
	zassert_equal(fpga_hal_write_tx_stream_config(0, &ip, 2, 48, ch, 2,
						      0x1234), 0);
	zassert_equal(fpga_hal_write_rx_stream_config(0, &ip, 5004, ch, 2, 48,
						      48), 0);

	zassert_equal(fpga_hal_ctrl_set_bits(BIT(0)), 0);
	zassert_equal(fpga_hal_ctrl_clear_bits(BIT(0)), 0);
	zassert_equal(fpga_hal_set_adda_nrst(true), 0);
	zassert_equal(fpga_hal_set_resets(FPGA_HAL_RESET_ALL, false), 0);
	zassert_equal(fpga_hal_set_ptp_reset(false), 0);
}

ZTEST_SUITE(fpga_hal_mock, NULL, NULL, NULL, NULL, NULL);
