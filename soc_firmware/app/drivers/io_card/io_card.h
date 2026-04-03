/*
 * IO Card Driver - 16-Channel ADC Input / 8-Channel DAC Output Board
 *
 * Hardware layout:
 *   Inputs (16 ch, "Add-Channels"):
 *     - 2x LPC microcontrollers at 0x40, 0x41  — directly on I2C bus
 *     - 2x CS5368 ADCs at 0x4C, 0x4D           — directly on I2C bus
 *
 *   Outputs (8 ch, "Drop-Channels"):
 *     - 1x LPC microcontroller at 0x43          — behind PCA9540B MUX (0x70)
 *     - 1x CS4385 DAC at 0x18 (8 channels)     — directly on I2C bus
 *
 * I2C MUX (PCA9540B at 0x70):
 *   Only used to reach the output LPC at 0x43.
 *   MUX channel 0 (cmd 0x04): output LPC
 *   MUX deselect  (cmd 0x00): back to main bus
 *
 * Board identification via LPC_SOFT_ID / LPC_BOARD_ID registers:
 *   Input LPC 0 (0x40): board_id = 0x31
 *   Output LPC  (0x43): board_id = 0x71 or 0xF2 (depending on board type)
 */

#ifndef IO_CARD_H
#define IO_CARD_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * I2C Device Addresses
 ******************************************************************************/

/* Input side - directly on main I2C bus */
#define IO_IN_LPC_ADDR0         0x40    /* LPC 0: input channels 0-7 */
#define IO_IN_LPC_ADDR1         0x41    /* LPC 1: input channels 8-15 */
#define IO_IN_ADC_ADDR0         0x4C    /* CS5368 ADC 0: input channels 0-7 / 0-3, 8-11 */
#define IO_IN_ADC_ADDR1         0x4D    /* CS5368 ADC 1: input channels 4-7, 12-15 */

/* Output side - behind I2C multiplexer (DEVADDR2 = 0x43) */
#define IO_OUT_LPC_ADDR         0x43    /* LPC for output control, selected via MUX */
#define IO_OUT_DAC_ADDR0        0x18    /* CS4385 DAC: all 8 output channels */

/*******************************************************************************
 * I2C Multiplexer Channel Selectors (PCA9540B)
 *
 * The MUX is only used to reach the output LPC at 0x43.
 * DAC (0x18) is directly on the main I2C bus.
 ******************************************************************************/

#define IO_MUX_CH0              0      /* MUX channel 0 - output LPC (0x43) */
#define IO_MUX_NONE             0xFF   /* Deselect MUX (return to main bus) */

/*******************************************************************************
 * LPC Register Map (same for input and output LPCs)
 ******************************************************************************/

#define IO_LPC_CHN_BASE         0x30    /* Channel registers base (2 bytes per channel) */
#define IO_LPC_GLB_REG          0x40    /* Global control register */
#define IO_LPC_OE_REG           0x41    /* Output enable register (output LPC only) */
#define IO_LPC_HEARTBEAT        0x42    /* Heartbeat register */
#define IO_LPC_SOFT_ID          0x70    /* Software ID */
#define IO_LPC_BOARD_ID         0x71    /* Board ID */
#define IO_LPC_HARD_REV         0x72    /* Hardware revision */

/*******************************************************************************
 * Global Register Bits (IO_LPC_GLB_REG)
 ******************************************************************************/

#define IO_GLB_NRST             0x08    /* Active-high reset release */
#define IO_GLB_F96KHZ           0x40    /* 96 kHz sample rate mode */
#define IO_GLB_HPF              0x80    /* High-pass filter enable (input only) */

/*******************************************************************************
 * Input Channel Register Format (at IO_LPC_CHN_BASE + 2*ch)
 *
 * High byte (offset 0):
 *   Bit 0: Attenuation/PAD (-20 dB)
 *   Bit 1: Phantom power (48V)
 *   Bit 2-5: Reserved
 *   Bit 6: Common mode (CM)
 *   Bit 7: DC coupling
 *
 * Low byte (offset 1):
 *   Bit 0-5: Analog gain register value
 *   Bit 6-7: Reserved
 ******************************************************************************/

#define IO_CHN_ATTENUATION      0x01
#define IO_CHN_PHANTOM          0x02
#define IO_CHN_CM               0x40
#define IO_CHN_DC               0x80
#define IO_CHN_GAIN_MASK        0x3F

/*******************************************************************************
 * ADC (CS5368) Registers
 ******************************************************************************/

#define IO_ADC_GMCR_REG         0x01    /* General Mode Configuration Register (GCTL) */
#define IO_ADC_OVFL_REG         0x02    /* Overflow Status Register (active low, sticky) */
#define IO_ADC_MUTE_REG         0x08    /* Mute Control Register (8 bits = 8 channels) */
#define IO_ADC_GMCR_VALUE       0x9B    /* CP-EN|MDIV1|DIF=TDM|MODE=Slave */

/* CS5368 I2C addresses depend on AD1/AD0 pin strapping (= DIF1/DIF0 in Stand-Alone Mode).
 * For TDM wiring (DIF1=1, DIF0=0): AD1=1, AD0=0 → 0x4E / 0x4F
 * For I2S wiring (DIF1=0, DIF0=1): AD1=0, AD0=1 → 0x4C / 0x4D (legacy)
 * The driver probes both pairs. */
#define IO_IN_ADC_ADDR0_ALT     0x4E    /* CS5368 ADC 0: TDM-strapped variant */
#define IO_IN_ADC_ADDR1_ALT     0x4F    /* CS5368 ADC 1: TDM-strapped variant */

/*******************************************************************************
 * DAC Registers (CS4XXX output side)
 ******************************************************************************/

#define IO_DAC_GMCR2_REG        0x02    /* General Mode Config Register 2 */
#define IO_DAC_PCM_REG          0x03    /* PCM control register */
#define IO_DAC_VOL_BASE_REG     0x0B    /* Volume/clip register base */

/* DAC init values from original firmware */
#define IO_DAC_GMCR2_INIT       0x81    /* Initial mode config */
#define IO_DAC_PCM_INIT         0xC3    /* PCM control init */
#define IO_DAC_GMCR2_PDN        0x80    /* Power-down mode (pull PDN) */

/*******************************************************************************
 * Constants
 ******************************************************************************/

#define IO_NUM_IN_CHANNELS      16      /* Number of input channels */
#define IO_NUM_OUT_CHANNELS     8       /* Number of output channels */
#define IO_NUM_IN_LPC           2       /* Number of input LPC chips */
#define IO_NUM_OUT_LPC_BANKS    1       /* One output LPC bank (8-ch card) */
#define IO_NUM_DAC              1       /* Number of output DAC chips */

#define IO_IN_GAIN_MIN          -6      /* Minimum input gain in dB */
#define IO_IN_GAIN_MAX          66      /* Maximum input gain in dB */
#define IO_IN_GAIN_ENTRIES      73      /* Number of gain table entries */

#define IO_OUT_CLIP_MIN         -9      /* Minimum output clip level in dB */
#define IO_OUT_CLIP_MAX         24      /* Maximum output clip level in dB */

/* Clip value conversion: (MAX - dB) * 2 */
#define IO_CONVERT_CLIP(db)     ((IO_OUT_CLIP_MAX - (int)(db)) * 2)

/* Board ID values read from LPC_BOARD_ID */
#define IO_IN_LPC0_BOARD_ID     0x31
#define IO_OUT_LPC_BOARD_ID_A   0x71    /* IOT_IO type */
#define IO_OUT_LPC_BOARD_ID_B   0xF2    /* IOT_OI type (scrambled channels) */

/*******************************************************************************
 * Data Structures
 ******************************************************************************/

/**
 * @brief Input channel register (2 bytes per channel on LPC)
 */
struct io_in_chn_reg {
	uint8_t high;   /* Attenuation, phantom, CM, DC */
	uint8_t low;    /* Gain value (bits 0-5) */
};

/**
 * @brief Board info structure
 */
struct io_board_info {
	uint8_t soft_id;
	uint8_t board_id;
	uint8_t hard_rev;
};

/**
 * @brief Input channel configuration
 */
struct io_in_channel_config {
	int8_t gain_db;     /* Preamp gain in dB (IO_IN_GAIN_MIN to IO_IN_GAIN_MAX) */
	bool phantom;       /* 48V phantom power */
	bool hpf;           /* High-pass filter (global per 8-ch bank) */
	bool muted;         /* Channel mute state */
};

/**
 * @brief Output channel configuration
 */
struct io_out_channel_config {
	int8_t clip_db;     /* Clip/output level in dB (IO_OUT_CLIP_MIN to IO_OUT_CLIP_MAX) */
	bool muted;         /* Channel mute state */
};

/**
 * @brief IO card driver state
 */
struct io_card_data {
	const struct device *i2c_dev;

	/* Input side state */
	struct io_in_chn_reg in_chn_reg[IO_NUM_IN_CHANNELS];
	uint8_t in_glb_reg[IO_NUM_IN_LPC];     /* Global reg per LPC chip */

	/* ADC state */
	uint8_t adc_addr[IO_NUM_IN_LPC];       /* Detected ADC I2C addresses */
	uint8_t adc_mute[IO_NUM_IN_LPC];       /* Mute register cache per ADC */
	uint16_t in_overflow;                   /* Logical channel overflow bitmask */
	uint8_t clip_hold[IO_NUM_IN_CHANNELS]; /* Per-channel clip hold counter */

	/* Output side state */
	uint8_t out_glb_reg[IO_NUM_OUT_LPC_BANKS];
	uint8_t out_enable[IO_NUM_OUT_LPC_BANKS]; /* Output enable bits per MUX bank */
	bool global_out_enable;                    /* Master output enable gate */
	int8_t out_clip_db[IO_NUM_OUT_CHANNELS];   /* Per-channel clip levels */

	bool initialized;
	struct k_mutex lock;
};

/*******************************************************************************
 * API Functions
 ******************************************************************************/

/**
 * @brief Initialize the IO card driver
 *
 * Detects and initializes both input (16-ch ADC preamp) and output
 * (8-ch DAC) sections of the IO card.
 *
 * @param i2c_dev I2C device to use
 * @return 0 on success, negative errno on failure
 */
int io_card_init(const struct device *i2c_dev);

/**
 * @brief Check if IO board input side is present
 *
 * @param info Optional pointer to receive LPC0 board info
 * @return true if board detected, false otherwise
 */
bool io_card_detect(struct io_board_info *info);

/* ---- Input (Add-Channel) API ---- */

/**
 * @brief Set input channel preamp gain
 *
 * @param channel Input channel (0-15)
 * @param gain_db Gain in dB (IO_IN_GAIN_MIN to IO_IN_GAIN_MAX)
 * @return 0 on success, negative errno on failure
 */
int io_card_set_in_gain(uint8_t channel, int8_t gain_db);

/**
 * @brief Get input channel preamp gain
 *
 * @param channel Input channel (0-15)
 * @return Gain in dB, or negative errno on failure
 */
int io_card_get_in_gain(uint8_t channel);

/**
 * @brief Set input channel phantom power
 *
 * @param channel Input channel (0-15)
 * @param enable true to enable 48V phantom
 * @return 0 on success, negative errno on failure
 */
int io_card_set_in_phantom(uint8_t channel, bool enable);

/**
 * @brief Get input channel phantom power state
 *
 * @param channel Input channel (0-15)
 * @return 1 if enabled, 0 if disabled, negative errno on failure
 */
int io_card_get_in_phantom(uint8_t channel);

/**
 * @brief Set input channel mute state
 *
 * @param channel Input channel (0-15)
 * @param mute true to mute, false to unmute
 * @return 0 on success, negative errno on failure
 */
int io_card_set_in_mute(uint8_t channel, bool mute);

/**
 * @brief Get input channel mute state
 *
 * @param channel Input channel (0-15)
 * @return 1 if muted, 0 if unmuted, negative errno on failure
 */
int io_card_get_in_mute(uint8_t channel);

/**
 * @brief Set 96 kHz sample rate mode (applies to both input and output)
 *
 * @param enable true for 96 kHz, false for 48 kHz
 * @return 0 on success, negative errno on failure
 */
int io_card_set_96khz(bool enable);

/* ---- Output (Drop-Channel) API ---- */

/**
 * @brief Set output channel clip level
 *
 * @param channel Output channel (0-7)
 * @param clip_db Clip level in dB (IO_OUT_CLIP_MIN to IO_OUT_CLIP_MAX)
 * @return 0 on success, negative errno on failure
 */
int io_card_set_out_clip(uint8_t channel, int8_t clip_db);

/**
 * @brief Get output channel clip level
 *
 * @param channel Output channel (0-7)
 * @return Clip level in dB, or negative errno on failure
 */
int io_card_get_out_clip(uint8_t channel);

/**
 * @brief Set output channel mute state
 *
 * @param channel Output channel (0-7)
 * @param mute true to mute, false to unmute
 * @return 0 on success, negative errno on failure
 */
int io_card_set_out_mute(uint8_t channel, bool mute);

/**
 * @brief Get output channel mute state
 *
 * @param channel Output channel (0-7)
 * @return 1 if muted, 0 if unmuted, negative errno on failure
 */
int io_card_get_out_mute(uint8_t channel);

/**
 * @brief Enable or disable all outputs globally
 *
 * When disabled, all output enable bits are forced to 0 regardless of
 * per-channel mute state.
 *
 * @param enable true to enable outputs, false to disable all
 * @return 0 on success, negative errno on failure
 */
int io_card_enable_outputs(bool enable);

/**
 * @brief Get global output enable state
 *
 * @return 1 if enabled, 0 if disabled, negative errno on failure
 */
int io_card_get_output_enable(void);

/**
 * @brief Reset the IO card (software reset via I2C)
 *
 * @return 0 on success, negative errno on failure
 */
int io_card_reset(void);

/**
 * @brief Get current input overflow status
 *
 * Returns a 16-bit bitmask where each bit corresponds to a logical
 * input channel. A bit is set if that channel has had an overflow
 * within the clip hold period.
 *
 * @return 16-bit overflow bitmask (logical channel mapping)
 */
uint16_t io_card_get_in_overflow(void);

/**
 * @brief Check if the IO board (16-in/8-out) is present and initialized
 *
 * @return true if IO board is active
 */
bool io_card_is_io_board(void);

/**
 * @brief Remap a logical input channel to the FPGA channel index
 *
 * On the 16in/8out IO board, ADC channels are not sequentially mapped.
 * This function converts a user-visible logical channel number to the
 * physical FPGA channel index used in TX stream configuration.
 *
 * @param logical_ch Logical channel (0-15)
 * @return FPGA channel index, or logical_ch unchanged if out of range
 */
uint8_t io_card_logical_to_fpga_ch(uint8_t logical_ch);

#ifdef __cplusplus
}
#endif

#endif /* IO_CARD_H */
