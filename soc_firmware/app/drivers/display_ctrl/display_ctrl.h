/*
 * Display Controller Driver - LEDs, Buttons, 7-Segment Displays
 *
 * UART control interface (115200 baud) for the front panel:
 * - 8 channels x 4 LED types (Mute, Signal, Clip, Phantom/48V)
 * - System LEDs (USB, ETH, LIP, LOP, etc.)
 * - 3 pairs of 7-segment displays (6 digits total)
 * - Button/Key input
 *
 * 
 */

#ifndef DISPLAY_CTRL_H
#define DISPLAY_CTRL_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * UART Command Addresses
 ******************************************************************************/

#define DC_CHNLEDBASE           0x80    /* Base address of channel LEDs */
#define DC_SYSLEDBASE           0x88    /* Base address of system LEDs */
#define DC_SEGMENTBASE          0x8C    /* Base address of 7-segments */
#define DC_BLINKBASE            0x92    /* Blink control */
#define DC_POINTBASE            0x93    /* Decimal points */
#define DC_CHNLEDBASEHI         0x94    /* Upper channel LEDs (>8 channels) */

/*******************************************************************************
 * Keyboard Constants
 ******************************************************************************/

#define DC_SCANOFFSET           0x41    /* Keyboard scan code offset */
#define DC_MAKEMASK             0x20    /* Make bit (key pressed) */
#define DC_CODEMASK             0x1F    /* Key code mask */

/*******************************************************************************
 * Display Indices
 ******************************************************************************/

enum dc_display {
	DC_LEFTDISPLAY = 0,
	DC_MIDDISPLAY = 1,
	DC_RIGHTDISPLAY = 2,
	DC_NUM_DISPLAYS = 3
};

/*******************************************************************************
 * Channel LED Types
 ******************************************************************************/

enum dc_chn_led_type {
	DC_CHNLED_MUTE = 0,         /* Mute LED (red) */
	DC_CHNLED_SIGNAL = 1,       /* Signal present LED (green) */
	DC_CHNLED_CLIP = 2,         /* Clip/overload LED (red) */
	DC_CHNLED_PHANTOM = 3,      /* Phantom/48V LED (amber/yellow) */
	DC_CHNLED_LAST = 4
};

#define DC_CHNLED_48V           DC_CHNLED_PHANTOM   /* Alias for 48V LED */
#define DC_CHNLED_FRAME         DC_CHNLED_PHANTOM   /* Frame sync LED */

/*******************************************************************************
 * System LED Indices
 ******************************************************************************/

enum dc_sys_led {
	DC_SYSLED_PSUB = 0,         /* Power supply B */
	DC_SYSLED_LOP = 1,          /* Link output */
	DC_SYSLED_EXT = 2,          /* External */
	DC_SYSLED_96K = 3,          /* 96 kHz mode */
	DC_SYSLED_MSTR = 4,         /* Master */
	DC_SYSLED_48K = 5,          /* 48 kHz mode */
	DC_SYSLED_LIP = 6,          /* Link input */
	DC_SYSLED_PSUA = 7,         /* Power supply A */
	DC_SYSLED_USB = 8,          /* USB activity */
	DC_SYSLED_ETH = 9,          /* Ethernet activity */
	DC_SYSLED_REM_LIP = 10,     /* Remote link input */
	DC_SYSLED_REM_LOP = 11,     /* Remote link output */
	DC_SYSLED_PWR = 12,         /* Power */
	DC_SYSLED_LAST
};

/*******************************************************************************
 * System LED States
 ******************************************************************************/

enum dc_sys_led_state {
	DC_SYSLED_OFF = 0,
	DC_SYSLED_BLINK1 = 1,       /* Slow blink */
	DC_SYSLED_BLINK2 = 2,       /* Fast blink */
	DC_SYSLED_ON = 3,
	DC_SYSLED_FLASH = 4
};

/*******************************************************************************
 * Scan Codes (Key IDs)
 ******************************************************************************/

enum dc_scan_code {
	SC_CHNSEL0 = 0,  SC_CHNSEL1,  SC_CHNSEL2,  SC_CHNSEL3,
	SC_CHNSEL4,      SC_CHNSEL5,  SC_CHNSEL6,  SC_CHNSEL7,
	SC_CHNSET0,      SC_CHNSET1,  SC_CHNSET2,  SC_CHNSET3,
	SC_CHNSET4,      SC_CHNSET5,  SC_CHNSET6,  SC_CHNSET7,
	SC_LEFTDEC,      SC_LEFTINC,  SC_MIDDEC,   SC_MIDINC,
	SC_RIGHTDEC,     SC_RIGHTINC, SC_LAST
};

/*******************************************************************************
 * Virtual Character Codes (7-Segment)
 ******************************************************************************/

enum dc_char_code {
	VCC_NUM0 = 0,  VCC_NUM1,  VCC_NUM2,  VCC_NUM3,  VCC_NUM4,
	VCC_NUM5,      VCC_NUM6,  VCC_NUM7,  VCC_NUM8,  VCC_NUM9,
	VCC_A,         VCC_B,     VCC_C,     VCC_D,     VCC_E,
	VCC_F,         VCC_G,     VCC_H,     VCC_I,     VCC_J,
	VCC_L,         VCC_M,     VCC_N,     VCC_O,     VCC_P,
	VCC_Q,         VCC_R,     VCC_S,     VCC_T,     VCC_U,
	VCC_X,         VCC_Y,     VCC_SPACE, VCC_MINUS,
	VCC_LAST
};

/*******************************************************************************
 * Button Event Structure
 ******************************************************************************/

struct dc_button_event {
	uint8_t code;       /* Scan code (SC_xxx) */
	bool pressed;       /* true = pressed, false = released */
	uint32_t timestamp; /* Uptime in ms when event occurred */
};

/*******************************************************************************
 * Button Callback Type
 ******************************************************************************/

typedef void (*dc_button_callback_t)(const struct dc_button_event *event,
				     void *user_data);

/*******************************************************************************
 * Driver State Structure
 ******************************************************************************/

struct display_ctrl_data {
	const struct device *uart_dev;
	struct k_mutex lock;
	
	/* LED state cache */
	uint8_t chn_led[DC_CHNLED_LAST]; /* 8 bits per type */
	uint8_t sys_led[4];              /* System LED registers */
	
	/* 7-segment state cache */
	uint8_t segment[DC_NUM_DISPLAYS][2]; /* Left and right digit per display */
	uint8_t blink_field;
	uint8_t point_mask;
	
	/* Button handling */
	dc_button_callback_t button_cb;
	void *button_cb_user_data;
	uint32_t button_state;          /* Current button bitfield */
	
	/* RX handling */
	struct k_work_delayable rx_work;
	uint8_t rx_buf[16];
	size_t rx_idx;
	
	bool initialized;
};

/*******************************************************************************
 * API Functions
 ******************************************************************************/

/**
 * @brief Initialize the display controller driver
 *
 * @param uart_dev UART device to use (115200 baud expected)
 * @return 0 on success, negative errno on failure
 */
int display_ctrl_init(const struct device *uart_dev);

/**
 * @brief Check if display controller is initialized
 *
 * @return true if initialized, false otherwise
 */
bool display_ctrl_ready(void);

/*******************************************************************************
 * 7-Segment Display Functions
 ******************************************************************************/

/**
 * @brief Set a 7-segment display pair
 *
 * @param which Display index (DC_LEFTDISPLAY, DC_MIDDISPLAY, DC_RIGHTDISPLAY)
 * @param left Left digit character code (VCC_xxx)
 * @param right Right digit character code (VCC_xxx)
 * @return 0 on success, negative errno on failure
 */
int display_ctrl_set_segment(enum dc_display which, uint8_t left, uint8_t right);

/**
 * @brief Display a numeric value (0-99 or signed -9 to 99)
 *
 * @param which Display index
 * @param value Value to display
 * @return 0 on success, negative errno on failure
 */
int display_ctrl_show_number(enum dc_display which, int8_t value);

/**
 * @brief Display a hex value (00-FF)
 *
 * @param which Display index
 * @param value Value to display
 * @return 0 on success, negative errno on failure
 */
int display_ctrl_show_hex(enum dc_display which, uint8_t value);

/**
 * @brief Set display blink state
 *
 * @param which Display index
 * @param blink true to blink, false for steady
 * @return 0 on success, negative errno on failure
 */
int display_ctrl_set_blink(enum dc_display which, bool blink);

/**
 * @brief Set decimal point mask
 *
 * @param mask Bitmask of decimal points (bit 0 = rightmost)
 * @return 0 on success, negative errno on failure
 */
int display_ctrl_set_points(uint8_t mask);

/**
 * @brief Display all 8s (test pattern)
 *
 * @return 0 on success, negative errno on failure
 */
int display_ctrl_test_pattern(void);

/**
 * @brief Clear all displays (show spaces)
 *
 * @return 0 on success, negative errno on failure
 */
int display_ctrl_clear(void);

/*******************************************************************************
 * Channel LED Functions
 ******************************************************************************/

/**
 * @brief Set a single channel LED
 *
 * @param channel Channel number (0-7)
 * @param type LED type (DC_CHNLED_xxx)
 * @param on true to turn on, false to turn off
 * @return 0 on success, negative errno on failure
 */
int display_ctrl_set_channel_led(uint8_t channel, enum dc_chn_led_type type, bool on);

/**
 * @brief Get a single channel LED state
 *
 * @param channel Channel number (0-7)
 * @param type LED type (DC_CHNLED_xxx)
 * @return 1 if on, 0 if off, negative errno on failure
 */
int display_ctrl_get_channel_led(uint8_t channel, enum dc_chn_led_type type);

/**
 * @brief Set all LEDs of a specific type
 *
 * @param type LED type (DC_CHNLED_xxx)
 * @param pattern Bitmask (bit N = channel N, 1=on, 0=off)
 * @return 0 on success, negative errno on failure
 */
int display_ctrl_set_channel_leds_by_type(enum dc_chn_led_type type, uint8_t pattern);

/**
 * @brief Set mute LED for a channel
 *
 * @param channel Channel number (0-7)
 * @param on true to turn on, false to turn off
 * @return 0 on success, negative errno on failure
 */
static inline int display_ctrl_set_mute_led(uint8_t channel, bool on)
{
	return display_ctrl_set_channel_led(channel, DC_CHNLED_MUTE, on);
}

/**
 * @brief Set signal LED for a channel
 *
 * @param channel Channel number (0-7)
 * @param on true to turn on, false to turn off
 * @return 0 on success, negative errno on failure
 */
static inline int display_ctrl_set_signal_led(uint8_t channel, bool on)
{
	return display_ctrl_set_channel_led(channel, DC_CHNLED_SIGNAL, on);
}

/**
 * @brief Set clip LED for a channel
 *
 * @param channel Channel number (0-7)
 * @param on true to turn on, false to turn off
 * @return 0 on success, negative errno on failure
 */
static inline int display_ctrl_set_clip_led(uint8_t channel, bool on)
{
	return display_ctrl_set_channel_led(channel, DC_CHNLED_CLIP, on);
}

/**
 * @brief Set 48V/phantom LED for a channel
 *
 * @param channel Channel number (0-7)
 * @param on true to turn on, false to turn off
 * @return 0 on success, negative errno on failure
 */
static inline int display_ctrl_set_48v_led(uint8_t channel, bool on)
{
	return display_ctrl_set_channel_led(channel, DC_CHNLED_PHANTOM, on);
}

/*******************************************************************************
 * System LED Functions
 ******************************************************************************/

/**
 * @brief Set a system LED state
 *
 * @param which System LED index (DC_SYSLED_xxx)
 * @param state LED state (DC_SYSLED_OFF, DC_SYSLED_ON, etc.)
 * @return 0 on success, negative errno on failure
 */
int display_ctrl_set_sys_led(enum dc_sys_led which, enum dc_sys_led_state state);

/*******************************************************************************
 * Button Input Functions
 ******************************************************************************/

/**
 * @brief Register a button callback
 *
 * @param callback Callback function
 * @param user_data User data passed to callback
 * @return 0 on success, negative errno on failure
 */
int display_ctrl_register_button_callback(dc_button_callback_t callback,
					  void *user_data);

/**
 * @brief Get current button state
 *
 * @return Bitmask of currently pressed buttons
 */
uint32_t display_ctrl_get_button_state(void);

/**
 * @brief Check if a specific button is pressed
 *
 * @param code Scan code (SC_xxx)
 * @return true if pressed, false otherwise
 */
bool display_ctrl_button_pressed(enum dc_scan_code code);

/*******************************************************************************
 * Utility Functions
 ******************************************************************************/

/**
 * @brief Display a 6-character status text across all displays
 *
 * Converts ASCII characters to 7-segment codes. Supported: 0-9, A-U, X, Y, space, minus.
 * Characters not representable on 7-segment will be shown as space.
 *
 * @param text 6-character string (null-padded if shorter)
 * @return 0 on success, negative errno on failure
 */
int display_ctrl_show_status(const char *text);

/**
 * @brief Perform full display test (all LEDs on)
 *
 * @return 0 on success, negative errno on failure
 */
int display_ctrl_full_test(void);

/**
 * @brief Turn off all LEDs and clear displays
 *
 * @return 0 on success, negative errno on failure
 */
int display_ctrl_all_off(void);

#ifdef CONFIG_DISPLAY_CTRL_NRST_GPIO
/**
 * @brief Hardware reset the display controller via nRST GPIO
 *
 * Pulses the nRST GPIO low to perform a hardware reset,
 * then waits for the controller to recover.
 *
 * @return 0 on success, negative errno on failure
 */
int display_ctrl_hw_reset(void);

/**
 * @brief Initialize the nRST GPIO pin
 *
 * Must be called before display_ctrl_hw_reset() can be used.
 *
 * @return 0 on success, negative errno on failure
 */
int display_ctrl_nrst_gpio_init(void);
#endif /* CONFIG_DISPLAY_CTRL_NRST_GPIO */

#ifdef __cplusplus
}
#endif

#endif /* DISPLAY_CTRL_H */
