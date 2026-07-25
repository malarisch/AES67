/*
 * AES67 PTP Hardware Clock for Zephyr — exposes the FPGA wallclock as a Zephyr
 * ptp_clock device so the in-tree IEEE 1588 stack (CONFIG_PTP) can discipline it.
 *
 * Wallclock register contract (FPGA/ptp/wallclock.vhd, --ptp-in-software CSRs):
 *   get         : read wallclock_seconds_in (48b) + nanoseconds_in (30b).
 *   set         : write *_out, pulse wallclock_ctrl.set — absolute, unsigned.
 *   adjust      : write a signed {seconds,nanoseconds} delta to *_out, pulse
 *                 wallclock_ctrl.phasejump. The HW adds seconds_i to seconds and
 *                 the 30-bit sign-extended nanoseconds_i to nanoseconds
 *                 separately, then normalises — so the nanoseconds component must
 *                 stay within 30-bit signed range; we split the delta so it does.
 *   rate_adjust : convert the frequency ratio to signed 20-bit ppb and write
 *                 wallclock_ppb (freq_correction_ppb_i).
 *
 * The same module also reconstructs full RX/TX hardware timestamps from the
 * 4-bit captured-seconds fields (aes67_ptp_reconstruct), used by the eth_litex
 * and eth_spi drivers. Only built when CONFIG_AES67_PTP_SOFTWARE is set (the
 * CSRs above exist only in a `--ptp-in-software` aes67_bridge gateware build).
 *
 * Register access is backend-dispatched: memory-mapped CSRs on the integrated
 * softcore (FPGA_HAL_LITEX), spibone transactions on an external MCU
 * (FPGA_HAL_SPI).
 */

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/ptp_clock.h>
#include <zephyr/net/ptp_time.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>

#include "eth_litex.h"

LOG_MODULE_REGISTER(ptp_aes67, LOG_LEVEL_INF);

#ifdef CONFIG_FPGA_HAL_SPI
/* A failed spibone transaction degrades to reading 0 / dropping the write;
 * the PTP servo treats that like a missed sample and recovers on the next. */
#include "../spibone/spibone.h"

static uint32_t wc_csr_read(uintptr_t addr)
{
	uint32_t v = 0;

	(void)spibone_read((uint32_t)addr, &v);
	return v;
}

static void wc_csr_write(uintptr_t addr, uint32_t val)
{
	(void)spibone_write((uint32_t)addr, val);
}
#else
#define wc_csr_read(addr)       litex_csr_read(addr)
#define wc_csr_write(addr, val) litex_csr_write(addr, val)
#endif

#define NS_PER_SEC   1000000000LL
/* Wallclock freq correction is a signed 20-bit ppb value (±524287). */
#define AES67_MAX_PPB 524287

/* Last rate correction written to the wallclock. Written by the PTP
 * servo (rate_adjust) while following and by the leader relax tick
 * while not — the role gates the two writers, so a plain variable with
 * benign last-writer-wins races at role changes is sufficient. */
static int32_t applied_ppb;

/* ---- NCO (media clock) rate loop ----
 *
 * The wallclock follows the servo tightly (accurate timestamps), but
 * routing every servo twitch into the audio NCO lets network jitter
 * reach the media clock. In SW-PTP gateware the whole NCO ppb loop
 * therefore lives here: the servo ppb is smoothed over a long window,
 * converted to 48-bit NCO increment units and written to the
 * nco_ppb_adj CSRs, which the gateware adds to the NCO base increment
 * as-is (valid = 1 bypasses its own multiply and 8-sample average).
 *
 * Conversion: mirrors wallclock.vhd's NCO_PPB_SCALE for the platform
 * constants fs = 48 kHz, sys_clk = 125 MHz —
 *   round(48000*512 / 125e6 * 2^32 / 1e9 * 65536) = 55340
 * increment units per ppb. The Q16 smoothing accumulator feeds the
 * multiply before the >>16, so sub-ppb resolution survives into the
 * register (1 unit = 1/55340 ppb ≈ 18 nppb).
 *
 * Smoothing: Q16 EWMA over the applied wallclock ppb, updated on every
 * rate write. The linreg servo applies roughly once per second, so
 * SHIFT 10 gives a ~1024-sample (~17 min) time constant: the audio NCO
 * sees only the long-term crystal drift, ±700 ppb servo wander is
 * attenuated to the tens-of-ppb range. Fast transients (boot, GM
 * change, relax ramp) bypass the window via the snap below; the
 * residual lag from slow tempco drift is absorbed by the gateware's
 * phase pull. Written only when the value changes; the EWMA settles
 * exactly, so a locked clock costs no extra bus traffic. */
#define NCO_SMOOTH_SHIFT   7
#define NCO_UNITS_PER_PPB  55340LL

/* Snap threshold for the rate EWMA: on larger steps the accumulator
 * jumps instead of slewing. */
#define NCO_RATE_SNAP_PPB  2000

static int64_t nco_acc_q16;
static int64_t nco_written_units;
static bool nco_written_once;

/* The servo rate path (PTP thread) and the leader relax tick
 * (fpga_poll thread) both end in the LO/HI register pair; interleaved
 * writers could tear the 48-bit commit (HI is the sign extension — a
 * mismatch is a ±2^40 glitch). */
static K_MUTEX_DEFINE(nco_lock);

static void nco_write_adj(int64_t units)
{
	/* LO, HI (committed atomically CSR-side on the HI write), then a
	 * valid pulse: the wallclock captures the stable 48-bit value on
	 * any synchronized valid edge. */
	wc_csr_write(CSR_AES67_CSR_NCO_PPB_ADJ_LO_ADDR,
		     (uint32_t)((uint64_t)units & 0xFFFFFFFFu));
	wc_csr_write(CSR_AES67_CSR_NCO_PPB_ADJ_HI_ADDR,
		     (uint32_t)(((uint64_t)units >> 32) & 0xFFFFu));
	wc_csr_write(CSR_AES67_CSR_NCO_PPB_ADJ_VALID_ADDR, 1);
	wc_csr_write(CSR_AES67_CSR_NCO_PPB_ADJ_VALID_ADDR, 0);
	nco_written_units = units;

	if (!nco_written_once) {
		nco_written_once = true;
		LOG_INF("NCO: software rate loop active");
	}
}

/* Call with nco_lock held. */
static void nco_recompute(void)
{
	/* acc: ±524287 ppb << 16 ≈ ±2^35; × 55340 ≈ ±2^51 — fits int64,
	 * and the >>16 result (±2^35) fits the 48-bit register. */
	int64_t units = (nco_acc_q16 * NCO_UNITS_PER_PPB) >> 16;

	if (units != nco_written_units || !nco_written_once) {
		nco_write_adj(units);
	}
}

static void nco_smooth_update(int32_t ppb)
{
	int64_t target = (int64_t)ppb << 16;
	int64_t diff;

	k_mutex_lock(&nco_lock, K_FOREVER);
	/* Snap instead of slew on large steps (boot warm-up from 0, GM
	 * changes, leader relax ramps): the long EWMA slewing up to a
	 * ~9 ppm crystal offset would leave the NCO thousands of ppb slow
	 * for a minute. The long window is for steady-state jitter, not
	 * acquisition. */
	diff = target - nco_acc_q16;
	if (diff > ((int64_t)NCO_RATE_SNAP_PPB << 16) ||
	    diff < -((int64_t)NCO_RATE_SNAP_PPB << 16)) {
		nco_acc_q16 = target;
	} else {
		nco_acc_q16 += diff >> NCO_SMOOTH_SHIFT;
	}
	nco_recompute();
	k_mutex_unlock(&nco_lock);
}

void aes67_ptp_nco_status(struct aes67_nco_status *st)
{
	k_mutex_lock(&nco_lock, K_FOREVER);
	st->rate_ppb_m  = (int32_t)((nco_acc_q16 * 1000) >> 16);
	st->written_units = nco_written_units;
	st->active = nco_written_once;
	k_mutex_unlock(&nco_lock);
}

static void wc_write_ppb(int32_t ppb)
{
	applied_ppb = ppb;
	wc_csr_write(CSR_AES67_CSR_WALLCLOCK_PPB_ADDR, (uint32_t)ppb & 0xFFFFF);
	nco_smooth_update(ppb);
}

void aes67_ptp_rate_reset(void)
{
	/* Neutral frequency for both register sets — a warm FPGA still
	 * carries last run's values. Forcing nco_written_once low makes
	 * wc_write_ppb() push adj=0 even though the local mirror already
	 * reads 0. */
	k_mutex_lock(&nco_lock, K_FOREVER);
	nco_acc_q16 = 0;
	nco_written_units = 0;
	nco_written_once = false;
	k_mutex_unlock(&nco_lock);

	wc_write_ppb(0);
	LOG_INF("Wallclock + NCO rate corrections reset to 0 ppb");
}

bool aes67_ptp_rate_relax_step(int32_t step_ppb)
{
	int32_t ppb = applied_ppb;

	if (ppb == 0) {
		return true;
	}

	if (ppb > 0) {
		ppb -= MIN(ppb, step_ppb);
	} else {
		ppb += MIN(-ppb, step_ppb);
	}

	wc_write_ppb(ppb);
	if (ppb == 0) {
		LOG_INF("Rate correction relaxed to 0 ppb");
	}
	return ppb == 0;
}

/* Read the live 48-bit wallclock seconds. */
static uint64_t aes67_wc_read_seconds(void)
{
	uint32_t lo = wc_csr_read(CSR_AES67_CSR_WALLCLOCK_SECONDS_IN_LO_ADDR);
	uint32_t hi = wc_csr_read(CSR_AES67_CSR_WALLCLOCK_SECONDS_IN_HI_ADDR) & 0xFFFF;

	return ((uint64_t)hi << 32) | lo;
}

static int ptp_aes67_get(const struct device *dev, struct net_ptp_time *tm)
{
	uint64_t sec, sec2;
	uint32_t nsec;

	ARG_UNUSED(dev);

	/* Coherent snapshot: re-read seconds around the nanoseconds read and retry
	 * if it advanced (guards a torn read across a one-second boundary). */
	do {
		sec  = aes67_wc_read_seconds();
		nsec = wc_csr_read(CSR_AES67_CSR_WALLCLOCK_NANOSECONDS_IN_ADDR) & 0x3FFFFFFF;
		sec2 = aes67_wc_read_seconds();
	} while (sec != sec2);

	tm->second = sec;
	tm->nanosecond = nsec;
	return 0;
}

static int ptp_aes67_set(const struct device *dev, struct net_ptp_time *tm)
{
	ARG_UNUSED(dev);

	wc_csr_write(CSR_AES67_CSR_WALLCLOCK_SECONDS_OUT_LO_ADDR, (uint32_t)tm->second);
	wc_csr_write(CSR_AES67_CSR_WALLCLOCK_SECONDS_OUT_HI_ADDR,
			(uint32_t)(tm->second >> 32) & 0xFFFF);
	wc_csr_write(CSR_AES67_CSR_WALLCLOCK_NANOSECONDS_OUT_ADDR, tm->nanosecond & 0x3FFFFFFF);
	/* Pulse set (1 then 0); the bus latency between writes exceeds the CDC
	 * capture window in the gateware. */
	wc_csr_write(CSR_AES67_CSR_WALLCLOCK_CTRL_ADDR,
			BIT(CSR_AES67_CSR_WALLCLOCK_CTRL_SET_OFFSET));
	wc_csr_write(CSR_AES67_CSR_WALLCLOCK_CTRL_ADDR, 0);
	return 0;
}

static int ptp_aes67_adjust(const struct device *dev, int increment)
{
	int64_t delta = increment;
	int64_t sec, nsec;

	ARG_UNUSED(dev);

	/* Split so the nanoseconds component lands in [-0.5s, 0.5s], well inside
	 * the 30-bit signed range the HW sign-extends. */
	sec  = (delta + (delta >= 0 ? NS_PER_SEC / 2 : -(NS_PER_SEC / 2))) / NS_PER_SEC;
	nsec = delta - sec * NS_PER_SEC;

	wc_csr_write(CSR_AES67_CSR_WALLCLOCK_SECONDS_OUT_LO_ADDR, (uint32_t)sec);
	wc_csr_write(CSR_AES67_CSR_WALLCLOCK_SECONDS_OUT_HI_ADDR,
			(uint32_t)((uint64_t)sec >> 32) & 0xFFFF);
	wc_csr_write(CSR_AES67_CSR_WALLCLOCK_NANOSECONDS_OUT_ADDR, (uint32_t)nsec & 0x3FFFFFFF);
	wc_csr_write(CSR_AES67_CSR_WALLCLOCK_CTRL_ADDR,
			BIT(CSR_AES67_CSR_WALLCLOCK_CTRL_PHASEJUMP_OFFSET));
	wc_csr_write(CSR_AES67_CSR_WALLCLOCK_CTRL_ADDR, 0);
	return 0;
}

static int ptp_aes67_rate_adjust(const struct device *dev, double ratio)
{
	int64_t ppb;

	ARG_UNUSED(dev);

	/* The servo expresses the correction as a frequency ratio; the wallclock
	 * takes it as parts-per-billion (ppb = (ratio - 1) * 1e9). */
	ppb = (int64_t)((ratio - 1.0) * 1e9);
	ppb = CLAMP(ppb, -AES67_MAX_PPB, AES67_MAX_PPB);

	/* Temporary SW-PTP bring-up diagnostics: one line per second at the
	 * AES67 8 Hz sync rate. A healthy locked servo settles near the
	 * crystal offset (typically a few 1000 ppb, constant); a value pinned
	 * at ±524287 means clamp/windup, wild swings mean loop instability. */
	{
		static uint32_t cnt;

		if ((++cnt % 8U) == 0) {
			LOG_WRN("PPB applied %lld", (long long)ppb);
		}
	}

	wc_write_ppb((int32_t)ppb);
	return 0;
}

/* Reconstruct a full timestamp from a captured (4-bit sec, 30-bit ns) pair by
 * reading the live wallclock seconds. The captured seconds field wraps every
 * 16 s; capture-to-read latency is far below that, so the low 4 bits uniquely
 * place the timestamp within the live seconds' 16 s block. */
void aes67_ptp_reconstruct(uint8_t cap_sec, uint32_t cap_nsec, struct net_ptp_time *out)
{
	uint64_t wc_sec = aes67_wc_read_seconds();
	uint64_t full_sec;

	cap_sec &= 0xF;
	full_sec = (wc_sec & ~0xFULL) | cap_sec;
	if (cap_sec > (uint8_t)(wc_sec & 0xF)) {
		full_sec -= 16; /* captured just before the 16 s boundary */
	}

	out->second = full_sec;
	out->nanosecond = cap_nsec & 0x3FFFFFFF;
}

static DEVICE_API(ptp_clock, ptp_aes67_api) = {
	.set = ptp_aes67_set,
	.get = ptp_aes67_get,
	.adjust = ptp_aes67_adjust,
	.rate_adjust = ptp_aes67_rate_adjust,
};

static int ptp_aes67_init(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}

DEVICE_DEFINE(ptp_clock_aes67, PTP_CLOCK_NAME, ptp_aes67_init, NULL, NULL, NULL,
	      POST_KERNEL, CONFIG_PTP_CLOCK_INIT_PRIORITY, &ptp_aes67_api);

const struct device *aes67_ptp_clock_device(void)
{
	return DEVICE_GET(ptp_clock_aes67);
}
