/*
 * Output Card Driver - 8-Channel DAC Line Output Board
 *
 * I2C control interface for the Line Output board with:
 * - LPC microcontroller at 0x40 (output enable, mute control)
 * - DSP chip at 0x14 (audio processing)
 * - 4x DAC chips at 0x4C-0x4F (clip level, mode control)
 *
 */

#ifndef LO_CARD_H
#define LO_CARD_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * I2C Device Addresses
 ******************************************************************************/

#define LO_LPC_ADDR             0x40    /* LPC microcontroller (output control) */
#define LO_DSP_ADDR             0x14    /* DSP chip */

/* DAC device addresses (4x CS55xx, 2 channels each) */
#define LO_DAC_DEV0             0x4C    /* DAC 0: channels 0-1 */
#define LO_DAC_DEV1             0x4D    /* DAC 1: channels 2-3 */
#define LO_DAC_DEV2             0x4E    /* DAC 2: channels 4-5 */
#define LO_DAC_DEV3             0x4F    /* DAC 3: channels 6-7 */

/*******************************************************************************
 * LPC Register Map
 ******************************************************************************/

#define LO_LPC_GLB_REG          0x40    /* Global control register */
#define LO_LPC_OE_REG           0x41    /* Output enable register (bit per channel) */
#define LO_LPC_HEARTBEAT        0x42    /* Heartbeat register */
#define LO_LPC_SOFT_ID          0x70    /* Software ID */
#define LO_LPC_BOARD_ID         0x71    /* Board ID (0x02 for LO card) */
#define LO_LPC_HARD_REV         0x72    /* Hardware revision */

/*******************************************************************************
 * Global Register Bits (LO_LPC_GLB_REG)
 ******************************************************************************/

#define LO_GLB_NRST             0x08    /* Active-high reset release */
#define LO_GLB_F96KHZ           0x40    /* 96 kHz sample rate mode */
#define LO_GLB_HPF              0x80    /* High-pass filter enable */

/*******************************************************************************
 * DAC Register Addresses
 ******************************************************************************/

#define LO_DAC_MODE_REG         0x02    /* Mode control register */
#define LO_DAC_VOL_A_REG        0x05    /* Volume/Clip control channel A */
#define LO_DAC_VOL_B_REG        0x06    /* Volume/Clip control channel B */
#define LO_DAC_MISC_REG         0x08    /* Miscellaneous control register */

/*******************************************************************************
 * DSP Register Addresses (16-bit addressing)
 ******************************************************************************/

#define LO_DSP_CORE_CTRL        0x0A52  /* Core control register */
#define LO_DSP_SER_OUT          0x0A54  /* Serial output control */
#define LO_DSP_SER_IN           0x0A56  /* Serial input control */

/*******************************************************************************
 * Constants
 ******************************************************************************/

#define LO_NUM_CHANNELS         8       /* Number of output channels */
#define LO_CLIP_MIN             -9      /* Minimum clip level in dB */
#define LO_CLIP_MAX             24      /* Maximum clip level in dB */
#define LO_BOARD_ID_EXPECTED    0x02    /* Expected board ID for LO card */

/* Clip value conversion: maps dB to DAC register value */
#define LO_CONVERT_CLIP(db)     (((LO_CLIP_MAX - LO_CLIP_MIN) - (db)) * 2)

/*******************************************************************************
 * Data Structures
 ******************************************************************************/

/**
 * @brief Board info structure
 */
struct lo_board_info {
	uint8_t soft_id;
	uint8_t board_id;
	uint8_t hard_rev;
};

/**
 * @brief Channel configuration
 */
struct lo_channel_config {
	int8_t clip_db;         /* Clip level in dB (-9 to +24) */
	bool muted;             /* Channel mute state */
};

/**
 * @brief LO card driver state
 */
struct lo_card_data {
	const struct device *i2c_dev;
	uint8_t glb_reg;
	uint8_t output_enable;          /* Bit per channel: 1=enabled, 0=disabled */
	bool global_enable;             /* Master output enable */
	int8_t clip_db[LO_NUM_CHANNELS]; /* Per-channel clip levels */
	bool initialized;
	struct k_mutex lock;
};

/*******************************************************************************
 * API Functions
 ******************************************************************************/

/**
 * @brief Initialize the LO card driver
 *
 * @param i2c_dev I2C device to use
 * @return 0 on success, negative errno on failure
 */
int lo_card_init(const struct device *i2c_dev);

/**
 * @brief Check if the LO board is present
 *
 * @param info Optional pointer to receive board info
 * @return true if board detected, false otherwise
 */
bool lo_card_detect(struct lo_board_info *info);

/**
 * @brief Set channel clip level
 *
 * @param channel Channel number (0-7)
 * @param clip_db Clip level in dB (-9 to +24)
 * @return 0 on success, negative errno on failure
 */
int lo_card_set_clip(uint8_t channel, int8_t clip_db);

/**
 * @brief Get channel clip level
 *
 * @param channel Channel number (0-7)
 * @return Clip level in dB, or negative errno on failure
 */
int lo_card_get_clip(uint8_t channel);

/**
 * @brief Set channel mute state
 *
 * @param channel Channel number (0-7)
 * @param mute true to mute, false to unmute
 * @return 0 on success, negative errno on failure
 */
int lo_card_set_mute(uint8_t channel, bool mute);

/**
 * @brief Get channel mute state
 *
 * @param channel Channel number (0-7)
 * @return 1 if muted, 0 if unmuted, negative errno on failure
 */
int lo_card_get_mute(uint8_t channel);

/**
 * @brief Enable or disable all outputs globally
 *
 * @param enable true to enable outputs, false to disable
 * @return 0 on success, negative errno on failure
 */
int lo_card_enable_outputs(bool enable);

/**
 * @brief Get global output enable state
 *
 * @return 1 if enabled, 0 if disabled, negative errno on failure
 */
int lo_card_get_output_enable(void);

/**
 * @brief Set 96 kHz sample rate mode
 *
 * @param enable true for 96 kHz, false for 48 kHz
 * @return 0 on success, negative errno on failure
 */
int lo_card_set_96khz(bool enable);

/**
 * @brief Get sample rate mode
 *
 * @return 1 if 96 kHz mode, 0 if 48 kHz mode, negative errno on failure
 */
int lo_card_get_96khz(void);

/**
 * @brief Reset the LO card (software reset via I2C)
 *
 * @return 0 on success, negative errno on failure
 */
int lo_card_reset(void);

#ifdef CONFIG_LO_CARD_NRST_GPIO
/**
 * @brief Hardware reset the LO card via nRST GPIO
 *
 * @return 0 on success, negative errno on failure
 */
int lo_card_hw_reset(void);

/**
 * @brief Initialize the nRST GPIO pin
 *
 * Must be called before lo_card_hw_reset() can be used.
 *
 * @return 0 on success, negative errno on failure
 */
int lo_card_nrst_gpio_init(void);
#endif /* CONFIG_LO_CARD_NRST_GPIO */

#ifdef __cplusplus
}
#endif

#endif /* LO_CARD_H */
