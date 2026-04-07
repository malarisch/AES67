/*
 * Display Controller Driver - LEDs, Buttons, 7-Segment Displays
 *
 * UART control interface (115200 baud) for the front panel. Protocol reverse engineered.
 */

#include "display_ctrl.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <stdio.h>

#if defined(CONFIG_DISPLAY_CTRL_NRST_GPIO)
#include <zephyr/drivers/gpio.h>
#define DC_NRST_NODE DT_ALIAS(dc_nrst)
#if DT_NODE_EXISTS(DC_NRST_NODE)
static const struct gpio_dt_spec dc_nrst_gpio = GPIO_DT_SPEC_GET(DC_NRST_NODE, gpios);
#define DC_NRST_GPIO_VALID 1
#else
#define DC_NRST_GPIO_VALID 0
#endif
static bool dc_nrst_gpio_ready = false;
#endif /* CONFIG_DISPLAY_CTRL_NRST_GPIO */

LOG_MODULE_REGISTER(display_ctrl, CONFIG_DISPLAY_CTRL_LOG_LEVEL);

/*******************************************************************************
 * 7-Segment Character Generator
 * Byte format: cgbdafe (c=bit6, g=bit5, b=bit4, d=bit3, a=bit2, f=bit1, e=bit0)
 ******************************************************************************/

static const uint8_t dc_char_generator[VCC_LAST] = {
	0x5F, /* 0: 1011111 */
	0x50, /* 1: 1010000 */
	0x3D, /* 2: 0111101 */
	0x7C, /* 3: 1111100 */
	0x72, /* 4: 1110010 */
	0x6E, /* 5: 1101110 */
	0x6F, /* 6: 1101111 */
	0x54, /* 7: 1010100 */
	0x7F, /* 8: 1111111 */
	0x7E, /* 9: 1111110 */
	0x77, /* A: 1110111 */
	0x6B, /* b: 1101011 */
	0x0F, /* C: 0001111 */
	0x79, /* d: 1111001 */
	0x2F, /* E: 0101111 */
	0x27, /* F: 0100111 */
	0x6F, /* G: 1101111 */
	0x63, /* h: 1100011 */
	0x03, /* I: 0000011 */
	0x5D, /* J: 1011101 */
	0x0B, /* L: 0001011 */
	0x45, /* M: 1000101 (approx) */
	0x61, /* n: 1100001 */
	0x5F, /* O: 1011111 */
	0x37, /* P: 0110111 */
	0x76, /* q: 1110110 */
	0x21, /* r: 0100001 */
	0x6E, /* S: 1101110 */
	0x2B, /* t: 0101011 */
	0x5B, /* U: 1011011 */
	0x73, /* X: 1110011 (approx) */
	0x7A, /* y: 1111010 */
	0x00, /* space: 0000000 */
	0x20, /* minus: 0100000 */
};

/* Driver instance data */
static struct display_ctrl_data dc_data = {
	.uart_dev = NULL,
	.initialized = false
};

/*******************************************************************************
 * Internal UART Functions
 ******************************************************************************/

/**
 * @brief Write data to UART
 */
static int uart_write_buf(const uint8_t *buf, size_t len)
{
	if (dc_data.uart_dev == NULL) {
		return -ENODEV;
	}

	for (size_t i = 0; i < len; i++) {
		uart_poll_out(dc_data.uart_dev, buf[i]);
	}

	return 0;
}

/**
 * @brief Send channel LED update command
 *
 * For 8-channel cards: single 3-byte UART message.
 * For extended cards (>8 channels): three 3-byte messages covering
 * channels 0-7, 8-15, and 16-23 respectively.
 */
static int send_channel_leds(enum dc_chn_led_type type)
{
	uint8_t buf[3];
	uint32_t pattern = dc_data.chn_led[type];
	int ret = 0;

#if DC_MAX_CHANNELS > 8
	/* Extended mode: 3 UART writes for up to 24 channels */

	/* Channels 16-23 */
	buf[0] = DC_CHNLEDBASE + (type * 2);
	buf[1] = (pattern >> 16) & 0x0F;
	buf[2] = (pattern >> 20) & 0x0F;
	ret = uart_write_buf(buf, 3);
	if (ret < 0) {
		return ret;
	}

	/* Channels 8-15 */
	buf[0] = DC_CHNLEDBASEHI + (type * 2);
	buf[1] = (pattern >> 8) & 0x0F;
	buf[2] = (pattern >> 12) & 0x0F;
	ret = uart_write_buf(buf, 3);
	if (ret < 0) {
		return ret;
	}

	/* Channels 0-7 */
	buf[0] = DC_CHNLEDBASEHI + (type * 2) + 0x08;
	buf[1] = pattern & 0x0F;
	buf[2] = (pattern >> 4) & 0x0F;
#else
	/* Standard 8-channel mode: single UART write */
	buf[0] = DC_CHNLEDBASE + (type * 2);
	buf[1] = pattern & 0x0F;         /* Channels 0-3 */
	buf[2] = (pattern >> 4) & 0x0F;  /* Channels 4-7 */
#endif

	LOG_DBG("Channel LED type %d: pattern=0x%06x", type, pattern);

	ret = uart_write_buf(buf, 3);
	return ret;
}

/**
 * @brief Send system LED update command
 */
static int send_sys_leds(uint8_t index)
{
	uint8_t buf[2];

	if (index >= 4) {
		return -EINVAL;
	}

	buf[0] = DC_SYSLEDBASE + index;
	buf[1] = dc_data.sys_led[index];

	LOG_DBG("System LED index %d: mask=0x%02x", index, buf[1]);

	return uart_write_buf(buf, 2);
}

/**
 * @brief Send 7-segment update command
 */
static int send_segment(enum dc_display which)
{
	uint8_t buf[3];

	if (which >= DC_NUM_DISPLAYS) {
		return -EINVAL;
	}

	buf[0] = DC_SEGMENTBASE + (which * 2);
	buf[1] = dc_char_generator[dc_data.segment[which][0]];
	buf[2] = dc_char_generator[dc_data.segment[which][1]];

	LOG_DBG("Segment %d: chars=[%d, %d], raw=[0x%02x, 0x%02x]",
		which, dc_data.segment[which][0], dc_data.segment[which][1],
		buf[1], buf[2]);

	return uart_write_buf(buf, 3);
}

/**
 * @brief Send blink field command
 */
static int send_blink_field(void)
{
	uint8_t buf[2];

	buf[0] = DC_BLINKBASE;
	buf[1] = dc_data.blink_field;

	return uart_write_buf(buf, 2);
}

/**
 * @brief Send decimal points command
 */
static int send_points(void)
{
	uint8_t buf[2];

	buf[0] = DC_POINTBASE;
	buf[1] = dc_data.point_mask;

	return uart_write_buf(buf, 2);
}

/*******************************************************************************
 * UART RX Handling (Button Input)
 ******************************************************************************/

static void uart_rx_callback(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	if (!uart_irq_update(dev)) {
		return;
	}

	while (uart_irq_rx_ready(dev)) {
		uint8_t c;
		int ret = uart_fifo_read(dev, &c, 1);

		if (ret != 1) {
			continue;
		}

		/* Process received scan code */
		if (dc_data.rx_idx < sizeof(dc_data.rx_buf)) {
			dc_data.rx_buf[dc_data.rx_idx++] = c;
		}

		/* Parse keyboard scan code */
		uint8_t code = c - DC_SCANOFFSET;
		bool pressed = (code & DC_MAKEMASK) != 0;
		uint8_t key = code & DC_CODEMASK;

		if (key < SC_LAST) {
			/* Update button state */
			if (pressed) {
				dc_data.button_state |= (1 << key);
			} else {
				dc_data.button_state &= ~(1 << key);
			}

			/* Call callback if registered */
			if (dc_data.button_cb) {
				struct dc_button_event event = {
					.code = key,
					.pressed = pressed,
					.timestamp = k_uptime_get_32()
				};
				dc_data.button_cb(&event, dc_data.button_cb_user_data);
			}

			LOG_DBG("Button: key=%d, pressed=%d, state=0x%08x",
				key, pressed, dc_data.button_state);
		}
	}
}

/*******************************************************************************
 * Public API Implementation
 ******************************************************************************/

int display_ctrl_init(const struct device *uart_dev)
{
	if (uart_dev == NULL) {
		LOG_ERR("UART device is NULL");
		return -EINVAL;
	}

	if (!device_is_ready(uart_dev)) {
		LOG_ERR("UART device not ready");
		return -ENODEV;
	}

	k_mutex_init(&dc_data.lock);
	k_mutex_lock(&dc_data.lock, K_FOREVER);

	dc_data.uart_dev = uart_dev;

	/* Initialize state cache */
	memset(dc_data.chn_led, 0, sizeof(dc_data.chn_led));
	memset(dc_data.sys_led, 0, sizeof(dc_data.sys_led));
	memset(dc_data.segment, VCC_SPACE, sizeof(dc_data.segment));
	dc_data.blink_field = 0;
	dc_data.point_mask = 0;
	dc_data.button_state = 0;
	dc_data.button_cb = NULL;
	dc_data.button_cb_user_data = NULL;
	dc_data.rx_idx = 0;
	/* nrst_callbacks are preserved across re-init */

	/* Configure UART for interrupt-driven RX */
	uart_irq_callback_user_data_set(uart_dev, uart_rx_callback, NULL);
	uart_irq_rx_enable(uart_dev);

	dc_data.initialized = true;

	k_mutex_unlock(&dc_data.lock);

	LOG_INF("Display controller initialized");
	return 0;
}

bool display_ctrl_ready(void)
{
	return dc_data.initialized;
}

const struct device *display_ctrl_get_uart(void)
{
	return dc_data.uart_dev;
}

/*******************************************************************************
 * 7-Segment Display Functions
 ******************************************************************************/

int display_ctrl_set_segment(enum dc_display which, uint8_t left, uint8_t right)
{
	int ret;

	if (which >= DC_NUM_DISPLAYS) {
		return -EINVAL;
	}

	if (left >= VCC_LAST || right >= VCC_LAST) {
		return -EINVAL;
	}

	if (!dc_data.initialized) {
		return -ENODEV;
	}

	k_mutex_lock(&dc_data.lock, K_FOREVER);

	dc_data.segment[which][0] = left;
	dc_data.segment[which][1] = right;
	ret = send_segment(which);

	k_mutex_unlock(&dc_data.lock);

	return ret;
}

int display_ctrl_show_number(enum dc_display which, int8_t value)
{
	uint8_t high, low;

	if (value < 0) {
		high = VCC_MINUS;
		low = VCC_NUM0 + (-value % 10);
	} else {
		high = VCC_NUM0 + (value / 10);
		low = VCC_NUM0 + (value % 10);
	}

	return display_ctrl_set_segment(which, high, low);
}

int display_ctrl_show_hex(enum dc_display which, uint8_t value)
{
	uint8_t high = VCC_NUM0 + ((value >> 4) & 0x0F);
	uint8_t low = VCC_NUM0 + (value & 0x0F);

	return display_ctrl_set_segment(which, high, low);
}

int display_ctrl_get_segment_left(enum dc_display which)
{
	if (which >= DC_NUM_DISPLAYS) {
		return -EINVAL;
	}
	return dc_data.segment[which][0];
}

int display_ctrl_get_segment_right(enum dc_display which)
{
	if (which >= DC_NUM_DISPLAYS) {
		return -EINVAL;
	}
	return dc_data.segment[which][1];
}

int display_ctrl_set_blink(enum dc_display which, bool blink)
{
	int ret;

	if (which >= DC_NUM_DISPLAYS) {
		return -EINVAL;
	}

	if (!dc_data.initialized) {
		return -ENODEV;
	}

	k_mutex_lock(&dc_data.lock, K_FOREVER);

	/* Each display uses 2 bits in blink field (for left and right digit) */
	uint8_t mask = 0x30 >> (which * 2);

	if (blink) {
		dc_data.blink_field |= mask;
	} else {
		dc_data.blink_field &= ~mask;
	}

	ret = send_blink_field();

	k_mutex_unlock(&dc_data.lock);

	return ret;
}

int display_ctrl_set_points(uint8_t mask)
{
	int ret;

	if (!dc_data.initialized) {
		return -ENODEV;
	}

	k_mutex_lock(&dc_data.lock, K_FOREVER);

	dc_data.point_mask = mask;
	ret = send_points();

	k_mutex_unlock(&dc_data.lock);

	return ret;
}

int display_ctrl_test_pattern(void)
{
	int ret = 0;

	ret |= display_ctrl_set_segment(DC_LEFTDISPLAY, VCC_NUM8, VCC_NUM8);
	ret |= display_ctrl_set_segment(DC_MIDDISPLAY, VCC_NUM8, VCC_NUM8);
	ret |= display_ctrl_set_segment(DC_RIGHTDISPLAY, VCC_NUM8, VCC_NUM8);
	ret |= display_ctrl_set_points(0x3F);

	return ret;
}

int display_ctrl_clear(void)
{
	int ret = 0;

	ret |= display_ctrl_set_segment(DC_LEFTDISPLAY, VCC_SPACE, VCC_SPACE);
	ret |= display_ctrl_set_segment(DC_MIDDISPLAY, VCC_SPACE, VCC_SPACE);
	ret |= display_ctrl_set_segment(DC_RIGHTDISPLAY, VCC_SPACE, VCC_SPACE);
	ret |= display_ctrl_set_points(0x00);

	return ret;
}

/**
 * @brief Convert ASCII character to VCC code
 */
static uint8_t ascii_to_vcc(char c)
{
	if (c >= '0' && c <= '9') {
		return VCC_NUM0 + (c - '0');
	}
	switch (c) {
	case 'A': case 'a': return VCC_A;
	case 'B': case 'b': return VCC_B;
	case 'C': case 'c': return VCC_C;
	case 'D': case 'd': return VCC_D;
	case 'E': case 'e': return VCC_E;
	case 'F': case 'f': return VCC_F;
	case 'G': case 'g': return VCC_G;
	case 'H': case 'h': return VCC_H;
	case 'I': case 'i': return VCC_I;
	case 'J': case 'j': return VCC_J;
	case 'L': case 'l': return VCC_L;
	case 'M': case 'm': return VCC_M;
	case 'N': case 'n': return VCC_N;
	case 'O': case 'o': return VCC_O;
	case 'P': case 'p': return VCC_P;
	case 'Q': case 'q': return VCC_Q;
	case 'R': case 'r': return VCC_R;
	case 'S': case 's': return VCC_S;
	case 'T': case 't': return VCC_T;
	case 'U': case 'u': return VCC_U;
	case 'X': case 'x': return VCC_X;
	case 'Y': case 'y': return VCC_Y;
	case '-': return VCC_MINUS;
	case ' ': default: return VCC_SPACE;
	}
}

int display_ctrl_show_status(const char *text)
{
	int ret = 0;
	uint8_t vcc[6] = {VCC_SPACE, VCC_SPACE, VCC_SPACE, VCC_SPACE, VCC_SPACE, VCC_SPACE};

	if (!dc_data.initialized) {
		return -ENODEV;
	}

	/* Convert up to 6 characters */
	for (int i = 0; i < 6 && text && text[i] != '\0'; i++) {
		vcc[i] = ascii_to_vcc(text[i]);
	}

	/* Display: LEFT=[0][1], MIDDLE=[2][3], RIGHT=[4][5] */
	ret |= display_ctrl_set_segment(DC_LEFTDISPLAY, vcc[0], vcc[1]);
	ret |= display_ctrl_set_segment(DC_MIDDISPLAY, vcc[2], vcc[3]);
	ret |= display_ctrl_set_segment(DC_RIGHTDISPLAY, vcc[4], vcc[5]);

	return ret;
}

/*******************************************************************************
 * Channel LED Functions
 ******************************************************************************/

int display_ctrl_set_channel_led(uint8_t channel, enum dc_chn_led_type type, bool on)
{
	int ret;

	if (channel >= DC_MAX_CHANNELS || type >= DC_CHNLED_LAST) {
		return -EINVAL;
	}

	if (!dc_data.initialized) {
		return -ENODEV;
	}

	k_mutex_lock(&dc_data.lock, K_FOREVER);

	if (on) {
		dc_data.chn_led[type] |= (1U << channel);
	} else {
		dc_data.chn_led[type] &= ~(1U << channel);
	}

	ret = send_channel_leds(type);

	k_mutex_unlock(&dc_data.lock);

	if (ret == 0) {
		LOG_DBG("Channel %d %s LED %s",
			channel,
			type == DC_CHNLED_MUTE ? "mute" :
			type == DC_CHNLED_SIGNAL ? "signal" :
			type == DC_CHNLED_CLIP ? "clip" : "48V",
			on ? "on" : "off");
	}

	return ret;
}

int display_ctrl_get_channel_led(uint8_t channel, enum dc_chn_led_type type)
{
	if (channel >= DC_MAX_CHANNELS || type >= DC_CHNLED_LAST) {
		return -EINVAL;
	}

	if (!dc_data.initialized) {
		return -ENODEV;
	}

	return (dc_data.chn_led[type] & (1U << channel)) ? 1 : 0;
}

int display_ctrl_set_channel_leds_by_type(enum dc_chn_led_type type, uint32_t pattern)
{
	int ret;

	if (type >= DC_CHNLED_LAST) {
		return -EINVAL;
	}

	if (!dc_data.initialized) {
		return -ENODEV;
	}

	k_mutex_lock(&dc_data.lock, K_FOREVER);

	dc_data.chn_led[type] = pattern;
	ret = send_channel_leds(type);

	k_mutex_unlock(&dc_data.lock);

	return ret;
}

uint32_t display_ctrl_get_channel_led_pattern(enum dc_chn_led_type type)
{
	if (type >= DC_CHNLED_LAST) {
		return 0;
	}
	return dc_data.chn_led[type];
}

/*******************************************************************************
 * System LED Functions
 ******************************************************************************/

int display_ctrl_set_sys_led(enum dc_sys_led which, enum dc_sys_led_state state)
{
	int ret;
	uint8_t index, shift;

	if (which >= DC_SYSLED_LAST) {
		return -EINVAL;
	}

	if (!dc_data.initialized) {
		return -ENODEV;
	}

	/* First 8 system LEDs are grouped in 4 registers, 2 LEDs per register */
	if (which < 8) {
		index = which / 2;
		shift = (which & 1) * 2;

		k_mutex_lock(&dc_data.lock, K_FOREVER);

		dc_data.sys_led[index] &= ~(3 << shift);
		dc_data.sys_led[index] |= (state & 3) << shift;
		ret = send_sys_leds(index);

		k_mutex_unlock(&dc_data.lock);
	} else {
		/* Higher system LEDs (USB, ETH, etc.) use SPI interface */
		/* Not implemented - would need SPI writes */
		LOG_WRN("System LED %d not supported (requires SPI)", which);
		ret = -ENOTSUP;
	}

	return ret;
}

int display_ctrl_get_sys_led(enum dc_sys_led which)
{
	if (which >= DC_SYSLED_LAST) {
		return -EINVAL;
	}
	if (which < 8) {
		uint8_t index = which / 2;
		uint8_t shift = (which & 1) * 2;
		return (dc_data.sys_led[index] >> shift) & 3;
	}
	return -ENOTSUP;
}

/*******************************************************************************
 * Button Input Functions
 ******************************************************************************/

int display_ctrl_register_button_callback(dc_button_callback_t callback,
					  void *user_data)
{
	if (!dc_data.initialized) {
		return -ENODEV;
	}

	k_mutex_lock(&dc_data.lock, K_FOREVER);

	dc_data.button_cb = callback;
	dc_data.button_cb_user_data = user_data;

	k_mutex_unlock(&dc_data.lock);

	return 0;
}

uint32_t display_ctrl_get_button_state(void)
{
	return dc_data.button_state;
}

bool display_ctrl_button_pressed(enum dc_scan_code code)
{
	if (code >= SC_LAST) {
		return false;
	}

	return (dc_data.button_state & (1 << code)) != 0;
}

/*******************************************************************************
 * Boot Animation
 ******************************************************************************/

#ifndef CONFIG_DISPLAY_CTRL_BOOT_ANIM_DELAY_MS
#define CONFIG_DISPLAY_CTRL_BOOT_ANIM_DELAY_MS 30
#endif

int display_ctrl_boot_animation(void)
{
	if (!dc_data.initialized) {
		return -ENODEV;
	}

	const int delay = CONFIG_DISPLAY_CTRL_BOOT_ANIM_DELAY_MS;
	const int num_input = (DC_MAX_CHANNELS > 16) ? 16 : DC_MAX_CHANNELS;
	const int num_columns = num_input / 2; /* Ch1-8 columns, each has Ch_n + Ch_{n+8} + Out_{n} */
	const int out_base = 16; /* Output channels start at 16 */

	/*
	 * Per-column LED order (4 time steps):
	 *   Step 0: Mute   (input ch_n, input ch_{n+8}, output ch_n)
	 *   Step 1: Signal (input ch_n, input ch_{n+8}), [empty slot for output]
	 *   Step 2: Clip   (input ch_n, input ch_{n+8}), Signal (output ch_n)
	 *   Step 3: 48V    (input ch_n, input ch_{n+8}), Clip   (output ch_n)
	 */

	/* Phase 1: Turn on LEDs column-by-column, step-by-step */
	for (int col = 0; col < num_columns; col++) {
		int ch_lo = col;               /* Ch 1-8  (indices 0-7) */
		int ch_hi = col + num_columns; /* Ch 9-16 (indices 8-15) */
		int ch_out = out_base + col;   /* Out 1-8 (indices 16-23) */

		/* Step 0: Mute (all three) */
		display_ctrl_set_channel_led(ch_lo, DC_CHNLED_MUTE, true);
		display_ctrl_set_channel_led(ch_hi, DC_CHNLED_MUTE, true);
		if (ch_out < DC_MAX_CHANNELS) {
			display_ctrl_set_channel_led(ch_out, DC_CHNLED_MUTE, true);
		}
		k_msleep(delay);

		/* Step 1: Signal (inputs only; output has empty slot) */
		display_ctrl_set_channel_led(ch_lo, DC_CHNLED_SIGNAL, true);
		display_ctrl_set_channel_led(ch_hi, DC_CHNLED_SIGNAL, true);
		k_msleep(delay);

		/* Step 2: Clip (inputs), Signal (output) */
		display_ctrl_set_channel_led(ch_lo, DC_CHNLED_CLIP, true);
		display_ctrl_set_channel_led(ch_hi, DC_CHNLED_CLIP, true);
		if (ch_out < DC_MAX_CHANNELS) {
			display_ctrl_set_channel_led(ch_out, DC_CHNLED_SIGNAL, true);
		}
		k_msleep(delay);

		/* Step 3: 48V (inputs), Clip (output) */
		display_ctrl_set_channel_led(ch_lo, DC_CHNLED_PHANTOM, true);
		display_ctrl_set_channel_led(ch_hi, DC_CHNLED_PHANTOM, true);
		if (ch_out < DC_MAX_CHANNELS) {
			display_ctrl_set_channel_led(ch_out, DC_CHNLED_CLIP, true);
		}
		k_msleep(delay);
	}

	///* Brief pause while all LEDs are on */
	//k_msleep(delay * 3);

	/* Phase 2: Turn off LEDs in the same order */
	for (int col = 0; col < num_columns; col++) {
		int ch_lo = col;
		int ch_hi = col + num_columns;
		int ch_out = out_base + col;

		/* Step 0: Mute off */
		display_ctrl_set_channel_led(ch_lo, DC_CHNLED_MUTE, false);
		display_ctrl_set_channel_led(ch_hi, DC_CHNLED_MUTE, false);
		if (ch_out < DC_MAX_CHANNELS) {
			display_ctrl_set_channel_led(ch_out, DC_CHNLED_MUTE, false);
		}
		k_msleep(delay);

		/* Step 1: Signal off (inputs only) */
		display_ctrl_set_channel_led(ch_lo, DC_CHNLED_SIGNAL, false);
		display_ctrl_set_channel_led(ch_hi, DC_CHNLED_SIGNAL, false);
		k_msleep(delay);

		/* Step 2: Clip off (inputs), Signal off (output) */
		display_ctrl_set_channel_led(ch_lo, DC_CHNLED_CLIP, false);
		display_ctrl_set_channel_led(ch_hi, DC_CHNLED_CLIP, false);
		if (ch_out < DC_MAX_CHANNELS) {
			display_ctrl_set_channel_led(ch_out, DC_CHNLED_SIGNAL, false);
		}
		k_msleep(delay);

		/* Step 3: 48V off (inputs), Clip off (output) */
		display_ctrl_set_channel_led(ch_lo, DC_CHNLED_PHANTOM, false);
		display_ctrl_set_channel_led(ch_hi, DC_CHNLED_PHANTOM, false);
		if (ch_out < DC_MAX_CHANNELS) {
			display_ctrl_set_channel_led(ch_out, DC_CHNLED_CLIP, false);
		}
		k_msleep(delay);
	}

	return 0;
}

/*******************************************************************************
 * Loading Animation (background thread, single-LED chase)
 ******************************************************************************/
#ifndef CONFIG_DISPLAY_CTRL_LOAD_ANIM_DELAY_MS
#define CONFIG_DISPLAY_CTRL_LOAD_ANIM_DELAY_MS 15
#endif
#define LOADING_ANIM_STACK_SIZE 512

static K_THREAD_STACK_DEFINE(loading_anim_stack, LOADING_ANIM_STACK_SIZE);
static struct k_thread loading_anim_thread;
static volatile bool loading_anim_running;

static void loading_anim_entry(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	const int delay = CONFIG_DISPLAY_CTRL_LOAD_ANIM_DELAY_MS;
	const int num_input = (DC_MAX_CHANNELS > 16) ? 16 : DC_MAX_CHANNELS;
	const int num_columns = num_input / 2;
	const int out_base = 16;

	/*
	 * Same traversal order as boot animation but only one LED per row
	 * is lit at a time (turn off previous before turning on next).
	 *
	 * Input rows:  Mute → Signal → Clip → 48V
	 * Output rows: Mute → [skip] → Signal → Clip
	 *
	 * "Row" = one channel strip (ch_lo, ch_hi, ch_out move together).
	 */

	/* Previous-step tracking: which LEDs to turn off before next step */
	int prev_col = -1;
	enum dc_chn_led_type prev_in_type = 0;
	enum dc_chn_led_type prev_out_type = 0;
	bool prev_out_valid = false;
	bool prev_in_only = false; /* step had no output LED */

	while (loading_anim_running) {
		for (int col = 0; col < num_columns && loading_anim_running; col++) {
			int ch_lo = col;
			int ch_hi = col + num_columns;
			int ch_out = out_base + col;
			bool has_out = (ch_out < DC_MAX_CHANNELS);

			/*
			 * 4 steps per column, matching boot animation order:
			 *   Step 0: Mute (in+out)
			 *   Step 1: Signal (in only)
			 *   Step 2: Clip (in) + Signal (out)
			 *   Step 3: 48V (in) + Clip (out)
			 */
			struct {
				enum dc_chn_led_type in_type;
				enum dc_chn_led_type out_type;
				bool out_on;  /* whether output LED lights this step */
			} steps[4] = {
				{ DC_CHNLED_MUTE,    DC_CHNLED_MUTE,    true  },
				{ DC_CHNLED_SIGNAL,  0,                  false },
				{ DC_CHNLED_CLIP,    DC_CHNLED_SIGNAL,   true  },
				{ DC_CHNLED_PHANTOM, DC_CHNLED_CLIP,     true  },
			};

			for (int s = 0; s < 4 && loading_anim_running; s++) {
				/* Turn off previous step's LEDs */
				if (prev_col >= 0) {
					int p_lo = prev_col;
					int p_hi = prev_col + num_columns;
					int p_out = out_base + prev_col;

					display_ctrl_set_channel_led(p_lo, prev_in_type, false);
					display_ctrl_set_channel_led(p_hi, prev_in_type, false);
					if (prev_out_valid && p_out < DC_MAX_CHANNELS) {
						display_ctrl_set_channel_led(p_out, prev_out_type, false);
					}
				}

				/* Turn on this step's LEDs */
				display_ctrl_set_channel_led(ch_lo, steps[s].in_type, true);
				display_ctrl_set_channel_led(ch_hi, steps[s].in_type, true);
				if (has_out && steps[s].out_on) {
					display_ctrl_set_channel_led(ch_out, steps[s].out_type, true);
				}

				/* Remember for next step */
				prev_col = col;
				prev_in_type = steps[s].in_type;
				prev_out_type = steps[s].out_type;
				prev_out_valid = steps[s].out_on;

				k_msleep(delay);
			}
		}
	}

	/* Clean up: turn off last lit LEDs */
	if (prev_col >= 0) {
		int p_lo = prev_col;
		int p_hi = prev_col + num_columns;
		int p_out = out_base + prev_col;

		display_ctrl_set_channel_led(p_lo, prev_in_type, false);
		display_ctrl_set_channel_led(p_hi, prev_in_type, false);
		if (prev_out_valid && p_out < DC_MAX_CHANNELS) {
			display_ctrl_set_channel_led(p_out, prev_out_type, false);
		}
	}
}

int display_ctrl_loading_animation_start(void)
{
	if (!dc_data.initialized) {
		return -ENODEV;
	}

	if (loading_anim_running) {
		return -EALREADY;
	}

	loading_anim_running = true;

	k_thread_create(&loading_anim_thread, loading_anim_stack,
			LOADING_ANIM_STACK_SIZE,
			loading_anim_entry, NULL, NULL, NULL,
			K_PRIO_PREEMPT(10), 0, K_NO_WAIT);
	k_thread_name_set(&loading_anim_thread, "led_loading");

	return 0;
}

int display_ctrl_loading_animation_stop(void)
{
	if (!loading_anim_running) {
		return 0;
	}

	loading_anim_running = false;
	k_thread_join(&loading_anim_thread, K_SECONDS(5));

	return 0;
}

/*******************************************************************************
 * Utility Functions
 ******************************************************************************/

int display_ctrl_full_test(void)
{
	int ret = 0;

	LOG_INF("Running display test...");

	/* All 7-segments to 8 */
	ret |= display_ctrl_test_pattern();

	/* All channel LEDs on */
	uint32_t all_on = (1U << DC_MAX_CHANNELS) - 1;

	ret |= display_ctrl_set_channel_leds_by_type(DC_CHNLED_MUTE, all_on);
	ret |= display_ctrl_set_channel_leds_by_type(DC_CHNLED_SIGNAL, all_on);
	ret |= display_ctrl_set_channel_leds_by_type(DC_CHNLED_CLIP, all_on);
	ret |= display_ctrl_set_channel_leds_by_type(DC_CHNLED_PHANTOM, all_on);

	/* All system LEDs on */
	for (int i = 0; i < 8; i++) {
		ret |= display_ctrl_set_sys_led(i, DC_SYSLED_ON);
	}

	return ret;
}

int display_ctrl_all_off(void)
{
	int ret = 0;

	/* Clear displays */
	ret |= display_ctrl_clear();

	/* All channel LEDs off */
	ret |= display_ctrl_set_channel_leds_by_type(DC_CHNLED_MUTE, 0x00);
	ret |= display_ctrl_set_channel_leds_by_type(DC_CHNLED_SIGNAL, 0x00);
	ret |= display_ctrl_set_channel_leds_by_type(DC_CHNLED_CLIP, 0x00);
	ret |= display_ctrl_set_channel_leds_by_type(DC_CHNLED_PHANTOM, 0x00);

	/* All system LEDs off */
	for (int i = 0; i < 8; i++) {
		ret |= display_ctrl_set_sys_led(i, DC_SYSLED_OFF);
	}

	/* Clear blink */
	ret |= display_ctrl_set_blink(DC_LEFTDISPLAY, false);
	ret |= display_ctrl_set_blink(DC_MIDDISPLAY, false);
	ret |= display_ctrl_set_blink(DC_RIGHTDISPLAY, false);

	return ret;
}

/*******************************************************************************
 * Audio Metering Thread (polls FPGA metering CSRs → signal/clip LEDs)
 ******************************************************************************/

#if defined(CONFIG_DISPLAY_CTRL_NRST_HAL) || defined(CONFIG_FPGA_HAL_LITEX)
#include "../fpga_hal/fpga_hal.h"

#define METERING_STACK_SIZE 512
#define METERING_POLL_MS    20

/* FPGA TX channel → logical input channel (inverse of logical_to_fpga in io_card.c)
 *   logical  0- 3 → FPGA  0- 3    logical  4- 7 → FPGA  8-11
 *   logical  8-11 → FPGA  4- 7    logical 12-15 → FPGA 12-15  */
static const uint8_t fpga_to_logical_in[16] = {
	 0,  1,  2,  3,   /* FPGA  0- 3 → logical  0- 3 */
	 8,  9, 10, 11,   /* FPGA  4- 7 → logical  8-11 */
	 4,  5,  6,  7,   /* FPGA  8-11 → logical  4- 7 */
	12, 13, 14, 15,   /* FPGA 12-15 → logical 12-15 */
};

/**
 * @brief Remap a 16-bit FPGA channel bitmask to logical channel bitmask.
 */
static uint32_t remap_fpga_to_logical(uint16_t fpga_mask, const uint8_t *map)
{
	uint32_t out = 0;

	while (fpga_mask) {
		int bit = __builtin_ctz(fpga_mask);
		out |= (1U << map[bit]);
		fpga_mask &= fpga_mask - 1;
	}
	return out;
}

static K_THREAD_STACK_DEFINE(metering_stack, METERING_STACK_SIZE);
static struct k_thread metering_thread;

static void metering_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	uint16_t rx_signal, rx_clip, tx_signal, tx_clip;
	uint32_t poll_count = 0;

	while (1) {
		k_msleep(METERING_POLL_MS);

		if (!dc_data.initialized) {
			continue;
		}

		fpga_hal_read_metering(&rx_signal, &rx_clip,
				       &tx_signal, &tx_clip);

		/* Log raw CSR values every ~1s (every 10th poll) */
		if ((poll_count % 10) == 0) {
			LOG_DBG("meter: rx_sig=0x%04x rx_clip=0x%04x "
				"tx_sig=0x%04x tx_clip=0x%04x",
				rx_signal, rx_clip, tx_signal, tx_clip);
		}
		poll_count++;

		/* TX (inputs) on LED channels 0-15, remapped from FPGA channel order.
		 * RX (outputs) on LED channels 16-23 (1:1, no remap needed). */
		uint32_t signal_pattern = remap_fpga_to_logical(tx_signal, fpga_to_logical_in)
					| ((uint32_t)rx_signal << 16);
		uint32_t clip_pattern   = remap_fpga_to_logical(tx_clip, fpga_to_logical_in)
					| ((uint32_t)rx_clip << 16);

		display_ctrl_set_channel_leds_by_type(DC_CHNLED_SIGNAL, signal_pattern);
		display_ctrl_set_channel_leds_by_type(DC_CHNLED_CLIP, clip_pattern);
	}
}

void display_ctrl_start_metering(void)
{
	static bool started;

	if (started) {
		return;
	}
	started = true;

	k_thread_create(&metering_thread, metering_stack,
			METERING_STACK_SIZE, metering_thread_fn,
			NULL, NULL, NULL,
			K_PRIO_PREEMPT(10), 0, K_NO_WAIT);
	k_thread_name_set(&metering_thread, "dc_meter");
	LOG_INF("Metering polling thread started (%d ms interval)", METERING_POLL_MS);
}

/*******************************************************************************
 * Status Display Cycling Thread
 * Cycles every 2 s: "ONLINE" → "LEADER"/"FOLLOW" → IP part1 → IP part2
 ******************************************************************************/

#define STATUS_CYCLE_STACK_SIZE 512
#define STATUS_CYCLE_INTERVAL_MS 2000

static K_THREAD_STACK_DEFINE(status_cycle_stack, STATUS_CYCLE_STACK_SIZE);
static struct k_thread status_cycle_thread;
static volatile bool status_cycle_running;

static struct {
	char role_str[7];   /* "LEADER" or "FOLLOW" */
	char ip_part1[7];   /* e.g. "192168" */
	char ip_part2[7];   /* e.g. "001050" */
} status_cycle_data;

static void status_cycle_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	int phase = 0;

	while (status_cycle_running) {
		switch (phase) {
		case 0:
			display_ctrl_show_status("ONLINE");
			break;
		case 1:
			display_ctrl_show_status(status_cycle_data.role_str);
			break;
		case 2:
			display_ctrl_show_status(status_cycle_data.ip_part1);
			break;
		case 3:
			display_ctrl_show_status(status_cycle_data.ip_part2);
			break;
		}

		phase = (phase + 1) % 4;
		k_msleep(STATUS_CYCLE_INTERVAL_MS);
	}
}

void display_ctrl_start_status_cycle(const char *role, const struct in_addr *ip)
{
	if (!dc_data.initialized || status_cycle_running) {
		return;
	}

	/* Store role string */
	strncpy(status_cycle_data.role_str, role,
		sizeof(status_cycle_data.role_str) - 1);
	status_cycle_data.role_str[sizeof(status_cycle_data.role_str) - 1] = '\0';

	/* Format IP address as two 6-char strings: "192168" + "001050" */
	if (ip) {
		const uint8_t *a = (const uint8_t *)&ip->s_addr;

		snprintf(status_cycle_data.ip_part1,
			 sizeof(status_cycle_data.ip_part1),
			 "%03u%03u", a[0], a[1]);
		snprintf(status_cycle_data.ip_part2,
			 sizeof(status_cycle_data.ip_part2),
			 "%03u%03u", a[2], a[3]);
	} else {
		strncpy(status_cycle_data.ip_part1, "000000", 7);
		strncpy(status_cycle_data.ip_part2, "000000", 7);
	}

	status_cycle_running = true;

	k_thread_create(&status_cycle_thread, status_cycle_stack,
			STATUS_CYCLE_STACK_SIZE, status_cycle_thread_fn,
			NULL, NULL, NULL,
			K_PRIO_PREEMPT(12), 0, K_NO_WAIT);
	k_thread_name_set(&status_cycle_thread, "dc_status");

	LOG_INF("Status cycle started: %s, IP=%s.%s",
		status_cycle_data.role_str,
		status_cycle_data.ip_part1,
		status_cycle_data.ip_part2);
}

void display_ctrl_stop_status_cycle(void)
{
	if (!status_cycle_running) {
		return;
	}

	status_cycle_running = false;
	k_thread_join(&status_cycle_thread, K_SECONDS(5));
}

void display_ctrl_update_status_cycle_ip(const struct in_addr *ip)
{
	if (!ip) {
		return;
	}

	const uint8_t *a = (const uint8_t *)&ip->s_addr;

	snprintf(status_cycle_data.ip_part1,
		 sizeof(status_cycle_data.ip_part1),
		 "%03u%03u", a[0], a[1]);
	snprintf(status_cycle_data.ip_part2,
		 sizeof(status_cycle_data.ip_part2),
		 "%03u%03u", a[2], a[3]);
}

#endif /* CONFIG_DISPLAY_CTRL_NRST_HAL || CONFIG_FPGA_HAL_LITEX */

#if defined(CONFIG_DISPLAY_CTRL_NRST_GPIO) || defined(CONFIG_DISPLAY_CTRL_NRST_HAL)

/**
 * @brief Invoke all registered post-reset callbacks
 */
static void nrst_notify_callbacks(void)
{
	LOG_INF("Shared nRST reset complete, notifying %d callbacks",
		dc_data.num_nrst_callbacks);

	for (int i = 0; i < dc_data.num_nrst_callbacks; i++) {
		if (dc_data.nrst_callbacks[i].cb) {
			dc_data.nrst_callbacks[i].cb(
				dc_data.nrst_callbacks[i].user_data);
		}
	}
}

#if defined(CONFIG_DISPLAY_CTRL_NRST_HAL)
/* ---- HAL backend (LiteX): nRST via FPGA CSR bit ---- */
#include "../fpga_hal/fpga_hal.h"

int display_ctrl_hw_reset(void)
{
	LOG_INF("Performing hardware reset via FPGA HAL adda_nrst "
		"(display + IO card)");

	k_mutex_lock(&dc_data.lock, K_FOREVER);

	/* Assert reset (nRST low → released=false) */
	fpga_hal_set_adda_nrst(false);

	/* Hold reset for configured duration */
	k_sleep(K_MSEC(CONFIG_DISPLAY_CTRL_NRST_PULSE_MS));

	/* Release reset (nRST high → released=true) */
	fpga_hal_set_adda_nrst(true);

	/* Wait for controllers to recover */
	k_sleep(K_MSEC(CONFIG_DISPLAY_CTRL_NRST_RECOVERY_MS));

	k_mutex_unlock(&dc_data.lock);

	nrst_notify_callbacks();
	return 0;
}

int display_ctrl_nrst_init(void)
{
	/* HAL is always available — nothing to initialise */
	LOG_INF("Shared nRST via FPGA HAL initialized (display + IO card)");
	return 0;
}

#elif defined(CONFIG_DISPLAY_CTRL_NRST_GPIO)
/* ---- GPIO backend (STM32H7): nRST via direct GPIO pin ---- */

int display_ctrl_hw_reset(void)
{
#if DC_NRST_GPIO_VALID
	if (!dc_nrst_gpio_ready) {
		LOG_ERR("nRST GPIO not initialized");
		return -ENODEV;
	}

	LOG_INF("Performing hardware reset via shared nRST GPIO "
		"(display + IO card)");

	k_mutex_lock(&dc_data.lock, K_FOREVER);

	/* Assert reset (active low, so set to logical 1 with GPIO_ACTIVE_LOW) */
	gpio_pin_set_dt(&dc_nrst_gpio, 1);

	/* Hold reset for configured duration */
	k_sleep(K_MSEC(CONFIG_DISPLAY_CTRL_NRST_PULSE_MS));

	/* Release reset */
	gpio_pin_set_dt(&dc_nrst_gpio, 0);

	/* Wait for controllers to recover */
	k_sleep(K_MSEC(CONFIG_DISPLAY_CTRL_NRST_RECOVERY_MS));

	k_mutex_unlock(&dc_data.lock);

	nrst_notify_callbacks();
	return 0;
#else
	LOG_WRN("nRST GPIO not defined in device tree");
	return -ENOTSUP;
#endif
}

int display_ctrl_nrst_init(void)
{
#if DC_NRST_GPIO_VALID
	int ret;

	if (!gpio_is_ready_dt(&dc_nrst_gpio)) {
		LOG_ERR("nRST GPIO device not ready");
		return -ENODEV;
	}

	/* Configure as output, initially deasserted (not in reset) */
	ret = gpio_pin_configure_dt(&dc_nrst_gpio, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		LOG_ERR("Failed to configure nRST GPIO: %d", ret);
		return ret;
	}

	dc_nrst_gpio_ready = true;
	LOG_INF("Shared nRST GPIO initialized (display + IO card)");
	return 0;
#else
	LOG_WRN("nRST GPIO not defined in device tree");
	return -ENOTSUP;
#endif
}
#endif /* GPIO vs HAL */

int display_ctrl_register_nrst_callback(dc_nrst_callback_t cb, void *user_data)
{
	if (dc_data.num_nrst_callbacks >= DC_MAX_NRST_CALLBACKS) {
		LOG_ERR("nRST callback table full");
		return -ENOMEM;
	}

	dc_data.nrst_callbacks[dc_data.num_nrst_callbacks].cb = cb;
	dc_data.nrst_callbacks[dc_data.num_nrst_callbacks].user_data = user_data;
	dc_data.num_nrst_callbacks++;

	LOG_DBG("nRST callback registered (%d/%d)",
		dc_data.num_nrst_callbacks, DC_MAX_NRST_CALLBACKS);
	return 0;
}
#endif /* CONFIG_DISPLAY_CTRL_NRST_GPIO || CONFIG_DISPLAY_CTRL_NRST_HAL */
