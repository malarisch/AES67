// SPDX-License-Identifier: GPL-2.0
/*
 * AES67 PTP Hardware Clock — exposes the FPGA wallclock as a Linux PHC so stock
 * ptp4l can discipline it.
 *
 * Wallclock register contract (FPGA/ptp/wallclock.vhd, --ptp-in-software CSRs):
 *   gettime  : read wallclock_seconds_in (48b) + nanoseconds_in (30b).
 *   settime  : write *_out, pulse wallclock_ctrl.set — absolute, unsigned.
 *   adjtime  : write a *signed* {seconds,nanoseconds} delta to *_out, pulse
 *              wallclock_ctrl.phasejump. The HW adds seconds_i to seconds and
 *              the 30-bit sign-extended nanoseconds_i to nanoseconds separately,
 *              then normalises — so the nanoseconds component must stay within
 *              30-bit signed range (|ns| < 2^29 ≈ 0.537 s); we split accordingly.
 *   adjfine  : write signed 20-bit ppb to wallclock_ppb (freq_correction_ppb_i).
 */
#include <linux/math64.h>

#include "aes67_eth.h"

#define NS_PER_SEC 1000000000LL

static struct aes67_priv *info_to_priv(struct ptp_clock_info *info)
{
	return container_of(info, struct aes67_priv, ptp_info);
}

/* Read the live 48-bit wallclock seconds (caller holds bus_lock). */
static int read_wc_seconds_locked(struct aes67_priv *p, u64 *sec)
{
	u32 lo, hi;
	int ret;

	ret = aes67_wb_read_locked(p, AES67_REG_AES67_CSR_WALLCLOCK_SECONDS_IN_LO, &lo);
	if (ret)
		return ret;
	ret = aes67_wb_read_locked(p, AES67_REG_AES67_CSR_WALLCLOCK_SECONDS_IN_HI, &hi);
	if (ret)
		return ret;
	*sec = ((u64)(hi & 0xffff) << 32) | lo;
	return 0;
}

static int aes67_phc_gettime(struct ptp_clock_info *info, struct timespec64 *ts)
{
	struct aes67_priv *p = info_to_priv(info);
	u64 sec, sec2;
	u32 nsec;
	int ret;

	mutex_lock(&p->bus_lock);
	/* Coherent snapshot: re-read seconds around the nanoseconds read and
	 * retry if it advanced (guards a torn read across a second boundary). */
	do {
		ret = read_wc_seconds_locked(p, &sec);
		if (ret)
			goto out;
		ret = aes67_wb_read_locked(p, AES67_REG_AES67_CSR_WALLCLOCK_NANOSECONDS_IN, &nsec);
		if (ret)
			goto out;
		ret = read_wc_seconds_locked(p, &sec2);
		if (ret)
			goto out;
	} while (sec != sec2);
out:
	mutex_unlock(&p->bus_lock);
	if (ret)
		return ret;

	ts->tv_sec  = sec;
	ts->tv_nsec = nsec & 0x3fffffff;
	return 0;
}

static int aes67_phc_settime(struct ptp_clock_info *info,
			     const struct timespec64 *ts)
{
	struct aes67_priv *p = info_to_priv(info);
	u64 sec = ts->tv_sec;
	int ret;

	mutex_lock(&p->bus_lock);
	ret =  aes67_wb_write_locked(p, AES67_REG_AES67_CSR_WALLCLOCK_SECONDS_OUT_LO,
				     (u32)sec);
	ret |= aes67_wb_write_locked(p, AES67_REG_AES67_CSR_WALLCLOCK_SECONDS_OUT_HI,
				     (u32)(sec >> 32) & 0xffff);
	ret |= aes67_wb_write_locked(p, AES67_REG_AES67_CSR_WALLCLOCK_NANOSECONDS_OUT,
				     (u32)ts->tv_nsec & 0x3fffffff);
	/* Pulse set (1 then 0); the bus latency between writes exceeds the CDC
	 * capture window in gateware. */
	ret |= aes67_wb_write_locked(p, AES67_REG_AES67_CSR_WALLCLOCK_CTRL, AES67_WC_CTRL_SET);
	ret |= aes67_wb_write_locked(p, AES67_REG_AES67_CSR_WALLCLOCK_CTRL, 0);
	mutex_unlock(&p->bus_lock);

	return ret ? -EIO : 0;
}

static int aes67_phc_adjtime(struct ptp_clock_info *info, s64 delta_ns)
{
	struct aes67_priv *p = info_to_priv(info);
	s64 sec, nsec;
	int ret;

	/* Split so the nanoseconds component lands in [-0.5s, 0.5s], well inside
	 * the 30-bit signed range the HW sign-extends. */
	sec  = div_s64(delta_ns + (delta_ns >= 0 ? NS_PER_SEC / 2 : -(NS_PER_SEC / 2)),
		       NS_PER_SEC);
	nsec = delta_ns - sec * NS_PER_SEC;

	mutex_lock(&p->bus_lock);
	ret =  aes67_wb_write_locked(p, AES67_REG_AES67_CSR_WALLCLOCK_SECONDS_OUT_LO,
				     (u32)sec);
	ret |= aes67_wb_write_locked(p, AES67_REG_AES67_CSR_WALLCLOCK_SECONDS_OUT_HI,
				     (u32)((u64)sec >> 32) & 0xffff);
	ret |= aes67_wb_write_locked(p, AES67_REG_AES67_CSR_WALLCLOCK_NANOSECONDS_OUT,
				     (u32)nsec & 0x3fffffff);
	ret |= aes67_wb_write_locked(p, AES67_REG_AES67_CSR_WALLCLOCK_CTRL, AES67_WC_CTRL_PHASEJUMP);
	ret |= aes67_wb_write_locked(p, AES67_REG_AES67_CSR_WALLCLOCK_CTRL, 0);
	mutex_unlock(&p->bus_lock);

	return ret ? -EIO : 0;
}

static int aes67_phc_adjfine(struct ptp_clock_info *info, long scaled_ppm)
{
	struct aes67_priv *p = info_to_priv(info);
	s64 ppb;

	/* scaled_ppm is ppm * 2^16 (signed). ppb = ppm * 1000. */
	ppb = div_s64((s64)scaled_ppm * 1000, 65536);
	ppb = clamp_t(s64, ppb, -AES67_MAX_PPB, AES67_MAX_PPB);

	return aes67_wb_write(p, AES67_REG_AES67_CSR_WALLCLOCK_PPB,
			      (u32)ppb & 0xfffff);
}

static int aes67_phc_enable(struct ptp_clock_info *info,
			    struct ptp_clock_request *req, int on)
{
	return -EOPNOTSUPP;
}

/* Reconstruct a full timestamp from a captured (4-bit sec, 30-bit ns) pair by
 * reading the live wallclock seconds. The captured seconds field wraps every
 * 16 s; capture-to-read latency is far below that, so the low 4 bits uniquely
 * place the timestamp within the live seconds' 16 s block. */
int aes67_ts_reconstruct(struct aes67_priv *p, u8 cap_sec, u32 cap_nsec,
			 u64 *ns_out)
{
	u64 wc_sec, full_sec;
	int ret;

	mutex_lock(&p->bus_lock);
	ret = read_wc_seconds_locked(p, &wc_sec);
	mutex_unlock(&p->bus_lock);
	if (ret)
		return ret;

	cap_sec &= 0xf;
	full_sec = (wc_sec & ~0xfULL) | cap_sec;
	if (cap_sec > (u8)(wc_sec & 0xf))
		full_sec -= 16;   /* captured just before the 16 s boundary */

	*ns_out = full_sec * NS_PER_SEC + (cap_nsec & 0x3fffffff);
	return 0;
}

int aes67_phc_register(struct aes67_priv *p)
{
	p->ptp_info = (struct ptp_clock_info){
		.owner     = THIS_MODULE,
		.max_adj   = AES67_MAX_PPB,
		.adjfine   = aes67_phc_adjfine,
		.adjtime   = aes67_phc_adjtime,
		.gettime64 = aes67_phc_gettime,
		.settime64 = aes67_phc_settime,
		.enable    = aes67_phc_enable,
	};
	scnprintf(p->ptp_info.name, sizeof(p->ptp_info.name), "aes67_wallclock");

	p->ptp_clock = ptp_clock_register(&p->ptp_info, &p->spi->dev);
	if (IS_ERR(p->ptp_clock)) {
		int ret = PTR_ERR(p->ptp_clock);

		p->ptp_clock = NULL;
		return ret;
	}
	return 0;
}

void aes67_phc_unregister(struct aes67_priv *p)
{
	if (p->ptp_clock) {
		ptp_clock_unregister(p->ptp_clock);
		p->ptp_clock = NULL;
	}
}
