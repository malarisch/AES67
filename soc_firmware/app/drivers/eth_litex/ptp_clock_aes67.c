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

	wc_csr_write(CSR_AES67_CSR_WALLCLOCK_PPB_ADDR, (uint32_t)ppb & 0xFFFFF);
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
