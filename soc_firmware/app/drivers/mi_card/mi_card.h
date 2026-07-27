/*
 * Input Card Driver - 8-Channel ADC Preamp Board
 *
 * I2C control interface for the Input preamp board with:
 * - microcontroller at 0x40 (preamp gain, phantom, HPF)
 * - DSP chip at 0x14 (audio processing)
 *
 * 
 */

#ifndef MI_CARD_H
#define MI_CARD_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * I2C Device Addresses
 ******************************************************************************/

#define MI_LPC_ADDR             0x40    /* LPC microcontroller (preamp control) */
#define MI_LPC_ADDR1            0x41    /* Extended LPC address 1 */
#define MI_LPC_ADDR2            0x43    /* Extended LPC address 2 */
#define MI_DSP_ADDR             0x14    /* DSP chip */

/*******************************************************************************
 * LPC Register Map
 ******************************************************************************/

#define MI_LPC_CHN_BASE         0x30    /* Channel registers base (2 bytes per channel) */
#define MI_LPC_GLB_REG          0x40    /* Global control register (PORT) */
#define MI_LPC_MUTE_STATUS      0x41    /* Mute status: 0=unmuted, 1=muted per channel */
#define MI_LPC_HEARTBEAT        0x42    /* Heartbeat enable register */
#define MI_LPC_SOFT_ID          0x70    /* Software ID */
#define MI_LPC_BOARD_ID         0x71    /* Board ID */
#define MI_LPC_HARD_REV         0x72    /* Hardware revision */

/*******************************************************************************
 * Global Register Bits (MI_LPC_GLB_REG)
 ******************************************************************************/

#define MI_GLB_NRST             0x08    /* Active-high reset release */
#define MI_GLB_F96KHZ           0x40    /* 96 kHz sample rate mode */
#define MI_GLB_HPF              0x80    /* High-pass filter enable */

/*******************************************************************************
 * Channel Register Format (2 bytes per channel at CHN_BASE + 2*ch)
 ******************************************************************************/

/*
 * High byte (offset 0):
 *   Bit 0: Attenuation (PAD -20 dB)
 *   Bit 1: Phantom power (48V)
 *   Bit 2-5: Reserved
 *   Bit 6: Common mode (CM)
 *   Bit 7: DC coupling
 *
 * Low byte (offset 1):
 *   Bit 0-5: Gain (0-63)
 *   Bit 6-7: Reserved
 */

/* High byte bit definitions */
#define MI_CHN_ATTENUATION      0x01
#define MI_CHN_PHANTOM          0x02
#define MI_CHN_CM               0x40
#define MI_CHN_DC               0x80

/* Gain mask for low byte */
#define MI_CHN_GAIN_MASK        0x3F

/*******************************************************************************
 * DSP Register Addresses (16-bit addressing)
 ******************************************************************************/

#define MI_DSP_CORE_CTRL        0x0A52  /* Core control register */
#define MI_DSP_SER_OUT          0x0A54  /* Serial output control */
#define MI_DSP_SER_IN           0x0A56  /* Serial input control */
#define MI_DSP_SAFELOAD_DATA0   0x0A40  /* Safeload data registers */
#define MI_DSP_SAFELOAD_ADDR0   0x0A45  /* Safeload address registers */

/*******************************************************************************
 * Constants
 ******************************************************************************/

#define MI_NUM_CHANNELS         8       /* Number of input channels */
#define MI_GAIN_MIN             -6      /* Minimum gain in dB */
#define MI_GAIN_MAX             66      /* Maximum gain in dB */
#define MI_GAIN_ENTRIES         73      /* Number of gain table entries */
#define MI_GAIN_PREAMP_MAX      48      /* Maximum analog preamp gain in dB */

/*******************************************************************************
 * Data Structures
 ******************************************************************************/

/**
 * @brief Channel register structure
 */
struct mi_chn_reg {
	uint8_t high;   /* Attenuation, phantom, CM, DC */
	uint8_t low;    /* Gain value */
};

/**
 * @brief Channel configuration
 */
struct mi_channel_config {
	int8_t gain_db;         /* Gain in dB (-6 to +66) */
	bool phantom;           /* 48V phantom power */
	bool hpf;               /* High-pass filter */
	bool muted;             /* Channel mute state */
};

/**
 * @brief Board info structure
 */
struct mi_board_info {
	uint8_t soft_id;
	uint8_t board_id;
	uint8_t hard_rev;
};

/**
 * @brief MI card driver state
 */
struct mi_card_data {
	const struct device *i2c_dev;
	struct mi_chn_reg chn_reg[MI_NUM_CHANNELS];
	uint8_t glb_reg;
	uint8_t mute_status;    /* Bit per channel: 0=unmuted, 1=muted */
	bool initialized;
	struct k_mutex lock;
};

/*******************************************************************************
 * API Functions
 ******************************************************************************/

/**
 * @brief Initialize the MI card driver
 *
 * @param i2c_dev I2C device to use
 * @return 0 on success, negative errno on failure
 */
int mi_card_init(const struct device *i2c_dev);

/**
 * @brief Check if the MI board is present
 *
 * @param info Optional pointer to receive board info
 * @return true if board detected, false otherwise
 */
bool mi_card_detect(struct mi_board_info *info);

/**
 * @brief Set channel preamp gain
 *
 * @param channel Channel number (0-7)
 * @param gain_db Gain in dB (-6 to +66)
 * @return 0 on success, negative errno on failure
 */
int mi_card_set_gain(uint8_t channel, int8_t gain_db);

/**
 * @brief Get channel preamp gain
 *
 * @param channel Channel number (0-7)
 * @return Current gain in dB, or negative errno on failure
 */
int mi_card_get_gain(uint8_t channel);

/**
 * @brief Set channel phantom power
 *
 * @param channel Channel number (0-7)
 * @param enable true to enable 48V phantom, false to disable
 * @return 0 on success, negative errno on failure
 */
int mi_card_set_phantom(uint8_t channel, bool enable);

/**
 * @brief Get channel phantom power state
 *
 * @param channel Channel number (0-7)
 * @return 1 if enabled, 0 if disabled, negative errno on failure
 */
int mi_card_get_phantom(uint8_t channel);

/**
 * @brief Set channel mute state
 *
 * @param channel Channel number (0-7)
 * @param mute true to mute, false to unmute
 * @return 0 on success, negative errno on failure
 */
int mi_card_set_mute(uint8_t channel, bool mute);

/**
 * @brief Get channel mute state
 *
 * @param channel Channel number (0-7)
 * @return 1 if muted, 0 if unmuted, negative errno on failure
 */
int mi_card_get_mute(uint8_t channel);

/**
 * @brief Set 96 kHz sample rate mode
 *
 * @param enable true for 96 kHz, false for 48 kHz
 * @return 0 on success, negative errno on failure
 */
int mi_card_set_96khz(bool enable);

/**
 * @brief Get sample rate mode
 *
 * @return 1 if 96 kHz mode, 0 if 48 kHz mode, negative errno on failure
 */
int mi_card_get_96khz(void);

/**
 * @brief Set global high-pass filter
 *
 * @param enable true to enable HPF, false to disable
 * @return 0 on success, negative errno on failure
 */
int mi_card_set_hpf(bool enable);

/**
 * @brief Get high-pass filter state
 *
 * @return 1 if enabled, 0 if disabled, negative errno on failure
 */
int mi_card_get_hpf(void);

/**
 * @brief Get full channel configuration
 *
 * @param channel Channel number (0-7)
 * @param config Pointer to receive configuration
 * @return 0 on success, negative errno on failure
 */
int mi_card_get_channel_config(uint8_t channel, struct mi_channel_config *config);

/**
 * @brief Reset the MI card (software reset via I2C)
 *
 * Reinitializes the MI card via I2C communication.
 *
 * @return 0 on success, negative errno on failure
 */
int mi_card_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* MI_CARD_H */
