/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: Apache-2.0
 *
 * Public API for the Si5351A I2C programmable clock generator driver.
 *
 * Reference: Silicon Labs Si5351A/B/C Datasheet + AN619
 */

#ifndef SI5351A_H_
#define SI5351A_H_

#include <zephyr/device.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------- Number of outputs on the Si5351A variant ---------- */
#define SI5351A_OUTPUT_COUNT  3   /* CLK0, CLK1, CLK2 */

/* ---------- Enumerations ---------------------------------------- */

/** PLL selection */
enum si5351a_pll {
	SI5351A_PLL_A = 0,
	SI5351A_PLL_B = 1,
};

/** Output drive strength */
enum si5351a_drive {
	SI5351A_DRIVE_2MA = 0,
	SI5351A_DRIVE_4MA = 1,
	SI5351A_DRIVE_6MA = 2,
	SI5351A_DRIVE_8MA = 3,
};

/** Crystal load capacitance */
enum si5351a_xtal_load {
	SI5351A_XTAL_LOAD_6PF  = (1 << 6),
	SI5351A_XTAL_LOAD_8PF  = (2 << 6),
	SI5351A_XTAL_LOAD_10PF = (3 << 6),
};

/** R output divider (divides output by 2^value) */
enum si5351a_r_div {
	SI5351A_R_DIV_1   = 0,
	SI5351A_R_DIV_2   = 1,
	SI5351A_R_DIV_4   = 2,
	SI5351A_R_DIV_8   = 3,
	SI5351A_R_DIV_16  = 4,
	SI5351A_R_DIV_32  = 5,
	SI5351A_R_DIV_64  = 6,
	SI5351A_R_DIV_128 = 7,
};

/* --------- Low-level API ---------------------------------------- */

/**
 * @brief Configure a PLL (feedback multisynth).
 *
 * Sets the VCO frequency: fVCO = fXTAL * (mult + num / denom)
 *
 * @param dev    Si5351A device
 * @param pll    SI5351A_PLL_A or SI5351A_PLL_B
 * @param mult   Integer multiplier (15 .. 90)
 * @param num    Fractional numerator   (0 .. 1 048 575)
 * @param denom  Fractional denominator (1 .. 1 048 575)
 * @return 0 on success, negative errno on error.
 */
int si5351a_setup_pll(const struct device *dev, enum si5351a_pll pll,
		      uint32_t mult, uint32_t num, uint32_t denom);

/**
 * @brief Configure a Multisynth output divider.
 *
 * fOUT = fVCO / (div + num / denom) / 2^r_div
 *
 * @param dev    Si5351A device
 * @param output Output index (0, 1, or 2)
 * @param pll    Which PLL feeds this output
 * @param div    Integer divider (4 .. 900 for MS0-MS5)
 * @param num    Fractional numerator   (0 .. 1 048 575)
 * @param denom  Fractional denominator (1 .. 1 048 575)
 * @param r_div  R output divider exponent (0..7 → /1../128)
 * @return 0 on success, negative errno on error.
 */
int si5351a_setup_multisynth(const struct device *dev, uint8_t output,
			     enum si5351a_pll pll, uint32_t div,
			     uint32_t num, uint32_t denom, uint8_t r_div);

/**
 * @brief Enable or disable a clock output.
 *
 * @param dev    Si5351A device
 * @param output Output index (0, 1, or 2)
 * @param enable true = output on, false = output off
 * @return 0 on success, negative errno on error.
 */
int si5351a_output_enable(const struct device *dev, uint8_t output,
			  bool enable);

/**
 * @brief Set the drive strength of a clock output.
 *
 * @param dev    Si5351A device
 * @param output Output index (0, 1, or 2)
 * @param drive  Drive strength (2 / 4 / 6 / 8 mA)
 * @return 0 on success, negative errno on error.
 */
int si5351a_set_drive_strength(const struct device *dev, uint8_t output,
			       enum si5351a_drive drive);

/**
 * @brief Apply a PLL soft reset (required after any PLL/MS change).
 *
 * @param dev  Si5351A device
 * @param pll  SI5351A_PLL_A or SI5351A_PLL_B
 * @return 0 on success, negative errno on error.
 */
int si5351a_pll_reset(const struct device *dev, enum si5351a_pll pll);

/* ---------- High-level convenience API -------------------------- */

/**
 * @brief Set an output to the requested frequency (Hz).
 *
 * Automatically configures PLL and Multisynth parameters for the
 * best achievable jitter.  Uses an integer Multisynth divider and
 * a fractional PLL to hit the target frequency.
 *
 * PLL assignment: CLK0 → PLLA, CLK1 → PLLB, CLK2 → PLLB.
 * Changing CLK1 may therefore affect CLK2 and vice-versa when both
 * use PLLB.  For fully independent outputs use the low-level API
 * with explicit PLL assignment.
 *
 * @param dev     Si5351A device
 * @param output  Output index (0, 1, or 2)
 * @param freq_hz Desired frequency in Hz (8 kHz .. 200 MHz)
 * @return 0 on success, negative errno on error.
 */
int si5351a_set_frequency(const struct device *dev, uint8_t output,
			  uint32_t freq_hz);

#ifdef __cplusplus
}
#endif

#endif /* SI5351A_H_ */
