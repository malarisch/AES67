/*
 * Unit tests for the PPB arithmetic in src/fpga_regs.c.
 *
 * fpga_calculate_ppb() turns the two clock_ppb_meter edge counters into
 * the frequency error the Si5351A PI controller acts on. A sign flip or
 * an overflow here walks the external PLL away from the wallclock.
 */

#include <zephyr/ztest.h>
#include <stdint.h>

#include "fpga_regs.h"

ZTEST(fpga_regs, test_ppb_zero_error)
{
	zassert_equal(fpga_calculate_ppb(0x200000, 0x200000), 0);
	zassert_equal(fpga_calculate_ppb(1, 1), 0);
}

ZTEST(fpga_regs, test_ppb_sign_and_magnitude)
{
	/* The PLL counted more edges than the wallclock: it runs fast, so
	 * the reported error is positive. */
	zassert_true(fpga_calculate_ppb(1000000, 1000001) > 0);
	zassert_true(fpga_calculate_ppb(1000000, 999999) < 0);

	/* 1 extra edge in 1e6 = 1000 ppb, and the result scales linearly. */
	zassert_equal(fpga_calculate_ppb(1000000, 1000001), 1000);
	zassert_equal(fpga_calculate_ppb(1000000, 999999), -1000);
	zassert_equal(fpga_calculate_ppb(1000000, 1000100), 100000);

	/* 1 edge in 2^21 (the meter's usual window). */
	zassert_equal(fpga_calculate_ppb(0x200000, 0x200001), 476);
}

ZTEST(fpga_regs, test_ppb_guards_division_by_zero)
{
	/* No wallclock edges counted: report no error rather than trap. */
	zassert_equal(fpga_calculate_ppb(0, 0), 0);
	zassert_equal(fpga_calculate_ppb(0, 12345), 0);
}

ZTEST(fpga_regs, test_ppb_saturates)
{
	/* A tiny window with a huge difference would overflow int32; the
	 * result must clamp instead of wrapping to the opposite sign. */
	zassert_equal(fpga_calculate_ppb(1, 1000000000), INT32_MAX);
	zassert_equal(fpga_calculate_ppb(1, 0x7FFFFFFF), INT32_MAX);
	zassert_true(fpga_calculate_ppb(2, 0x7FFFFFFF) > 0,
		     "saturation must not flip the sign");

	/* Same, mirrored: the PLL counter far below the wallclock's. */
	zassert_equal(fpga_calculate_ppb(0x7FFFFFFF, 0), -1000000000);
	zassert_true(fpga_calculate_ppb(1, 0) == -1000000000);
}

ZTEST_SUITE(fpga_regs, NULL, NULL, NULL, NULL, NULL);
