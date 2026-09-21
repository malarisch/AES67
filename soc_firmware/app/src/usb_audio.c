/*
 * USB Audio Class 2 <-> FPGA I2S bridge.
 *
 * Topology (see boards/esp32s3_devkitc.overlay, node uac2_fpga):
 *
 *   host --ISO OUT--> usb_out_terminal --> fpga_tx (I2S TX, ESP -> FPGA)
 *   host <--ISO IN--- usb_in_terminal  <-- fpga_rx (I2S RX, FPGA -> ESP)
 *
 * Clocking: the FPGA drives BCLK/WS from its PTP-disciplined media clock,
 * the ESP32 I2S port is a target (slave) in both directions. USB is
 * asynchronous with implicit feedback: every SOF the IN stream carries
 * exactly as many frames as the I2S port delivered (nominal 48, +/-1 when
 * the receive ring drifts from its target level) and the host mirrors
 * those packet sizes on the OUT stream. Both directions run on the same
 * clock, so the playback queue stays level without further regulation.
 *
 * Buffers: USB packets are 24-bit samples in 32-bit little-endian subslots,
 * which is exactly the I2S DMA block layout for 32-bit slots, so playback
 * packets go to i2s_write() as they are (zero copy, like the upstream
 * uac2_implicit_feedback sample). Record data is copied through a small
 * frame ring because the I2S RX blocks (48 frames) and the USB packets
 * (47..49 frames) do not line up.
 *
 * Threading: every UAC2 callback (SOF, terminal update, buffer handling)
 * runs on the cooperative usbd thread, so the context below needs no
 * locking; the shell only reads counters.
 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/usb/class/usbd_uac2.h>
#include <zephyr/logging/log.h>
#include <stdio.h>
#include <string.h>

#include "usb_audio.h"
#include "usb_audio_i2s.h"
#include "aes67_config.h"

LOG_MODULE_REGISTER(usb_audio, LOG_LEVEL_INF);

/* ---- Devicetree entities ---- */
#define UA_UAC2_NODE       DT_NODELABEL(uac2_fpga)
#define UA_OUT_TERMINAL_ID UAC2_ENTITY_ID(DT_NODELABEL(usb_out_terminal))
#define UA_IN_TERMINAL_ID  UAC2_ENTITY_ID(DT_NODELABEL(usb_in_terminal))

/* ---- Stream format (must match the uac2_fpga node) ----
 * USB carries 24-bit samples in 3-byte little-endian subslots (S24_3LE on
 * the host); the I2S rings hold 32-bit words with the sample MSB-aligned.
 * The copies below repack.
 */
#define UA_SAMPLE_RATE      48000
#define UA_FRAMES_PER_SOF   (UA_SAMPLE_RATE / 1000) /* Full-Speed: 1 ms frames */
#define UA_CHANNELS         UA_I2S_CHANNELS
#define UA_BYTES_PER_SAMPLE DT_PROP(DT_NODELABEL(as_iso_out), subslot_size)
#define UA_BYTES_PER_FRAME  (UA_CHANNELS * UA_BYTES_PER_SAMPLE)
#define UA_PKT_MAX_FRAMES   (UA_FRAMES_PER_SOF + 1)
#define UA_PKT_MAX_BYTES    (UA_PKT_MAX_FRAMES * UA_BYTES_PER_FRAME)
#define UA_PKT_BUF_BYTES    ROUND_UP(UA_PKT_MAX_BYTES, UDC_BUF_GRANULARITY)

BUILD_ASSERT(UA_BYTES_PER_SAMPLE >= 2 && UA_BYTES_PER_SAMPLE <= 4);
BUILD_ASSERT(DT_PROP(DT_NODELABEL(as_iso_in), subslot_size) == UA_BYTES_PER_SAMPLE);
BUILD_ASSERT(DT_PROP_BY_IDX(DT_NODELABEL(uac_aclk), sampling_frequencies, 0) == UA_SAMPLE_RATE);

/* USB packet buffers (UDC alignment): OUT receive buffers the class owns
 * until data_recv_cb, IN buffers until buf_release_cb (double-buffered).
 */
#define UA_OUT_BUFS (CONFIG_USBD_AUDIO2_QUEUE_DEPTH + 2)
#define UA_IN_BUFS  (CONFIG_USBD_AUDIO2_QUEUE_DEPTH + 2)

/* DMA ring geometry (frames): UA_I2S_BLOCKS x UA_I2S_BLOCK_FRAMES, 10.7 ms. */
#define UA_RING_FRAMES UA_I2S_RING_FRAMES
#define UA_RING_MASK   (UA_RING_FRAMES - 1)
BUILD_ASSERT((UA_RING_FRAMES & UA_RING_MASK) == 0);

/* Playback: keep the write pointer this far ahead of the block the DMA
 * is draining (6 ms latency: the USB packets reach us through two
 * cooperative threads that other cooperative threads hold off for up to
 * a few ms at a time, measured on the ESP32-S3 with the PTP TX timestamp
 * thread doing SPI). Below 1.5 blocks = the DMA caught up with us
 * (underrun), above the ring minus two blocks = we lapped it.
 */
#define UA_TX_TARGET   (6 * UA_FRAMES_PER_SOF)
#define UA_TX_MIN      (UA_I2S_BLOCK_FRAMES + UA_I2S_BLOCK_FRAMES / 2)
#define UA_TX_MAX      ((UA_I2S_BLOCKS - 2) * UA_I2S_BLOCK_FRAMES)

/* Record: steady-state fill sits at UA_RX_TARGET .. + one block; the band
 * is the hysteresis of the +/-1 regulator around the EWMA of the fill.
 */
#define UA_RX_TARGET   (2 * UA_FRAMES_PER_SOF)
#define UA_RX_MAX      ((UA_I2S_BLOCKS - 2) * UA_I2S_BLOCK_FRAMES)
#define UA_RX_BAND     8

/* Diagnostics: a sample-to-sample step above this (1/8 full scale) cannot
 * come from band-limited audio at sane levels and marks a discontinuity.
 */
#define UA_JUMP_THRESHOLD 0x10000000

K_MEM_SLAB_DEFINE_STATIC(ua_out_slab, UA_PKT_BUF_BYTES, UA_OUT_BUFS, UDC_BUF_ALIGN);
K_MEM_SLAB_DEFINE_STATIC(ua_in_slab, UA_PKT_BUF_BYTES, UA_IN_BUFS, UDC_BUF_ALIGN);

struct ua_ctx {
	const struct device *uac2_dev;
	uint32_t tx_last;      /* last ch0 sample written (jump detector) */
	uint32_t rx_last;      /* last ch0 sample read (jump detector) */

	bool usb_enabled;
	bool out_enabled;
	bool in_enabled;
	bool i2s_started;
	bool out_synced;        /* tx_wr is positioned relative to the DMA */
	bool in_started;

	uint32_t tx_wr;         /* playback write position, frames (monotonic) */
	uint32_t rx_rd;         /* record read position, frames (monotonic) */
	int32_t fill_avg_q4;    /* EWMA of the record fill level, frames * 16 */

	struct usb_audio_status st;
};

static struct ua_ctx ua;

/* ---------------------------------------------------------------------- */
/* Ring helpers                                                             */

static inline int32_t tx_fill(const struct ua_ctx *c)
{
	return (int32_t)(c->tx_wr - ua_i2s_tx_done() * UA_I2S_BLOCK_FRAMES);
}

static inline int32_t rx_fill(const struct ua_ctx *c)
{
	return (int32_t)(ua_i2s_rx_done() * UA_I2S_BLOCK_FRAMES - c->rx_rd);
}

/* Little-endian USB subslot (2, 3 or 4 bytes) <-> MSB-aligned 32-bit I2S word. */
#define UA_SUBSLOT_SHIFT(b) (32 - 8 * UA_BYTES_PER_SAMPLE + 8 * (b))

/* Unpack @frames frames of USB samples from @src into the playback ring at
 * tx_wr (one 32-bit word per sample, MSB-aligned).
 */
static void tx_ring_write(struct ua_ctx *c, const uint8_t *src, uint32_t frames)
{
	uint32_t *ring = ua_i2s_tx_ring();
	uint32_t samples = frames * UA_CHANNELS;

	for (uint32_t i = 0; i < samples; i++, src += UA_BYTES_PER_SAMPLE) {
		uint32_t pos = (c->tx_wr & UA_RING_MASK) * UA_CHANNELS + (i % UA_CHANNELS);

		uint32_t w = 0;

		for (int b = 0; b < UA_BYTES_PER_SAMPLE; b++) {
			w |= (uint32_t)src[b] << UA_SUBSLOT_SHIFT(b);
		}
		ring[pos] = w;
		if ((i % UA_CHANNELS) == 0) {
			/* Diagnostics: discontinuity detector on channel 0 */
			int32_t d = (int32_t)w - (int32_t)c->tx_last;

			if (d > UA_JUMP_THRESHOLD || d < -UA_JUMP_THRESHOLD) {
				c->st.out_jumps++;
			}
			c->tx_last = w;
		}
		if ((i % UA_CHANNELS) == UA_CHANNELS - 1) {
			c->tx_wr++;
		}
	}
}

/* Pack @frames frames from the record ring at rx_rd into @dst as USB samples. */
static void rx_ring_read(struct ua_ctx *c, uint8_t *dst, uint32_t frames)
{
	const uint32_t *ring = ua_i2s_rx_ring();
	uint32_t samples = frames * UA_CHANNELS;

	for (uint32_t i = 0; i < samples; i++, dst += UA_BYTES_PER_SAMPLE) {
		uint32_t pos = (c->rx_rd & UA_RING_MASK) * UA_CHANNELS + (i % UA_CHANNELS);
		uint32_t w = ring[pos];

		if ((i % UA_CHANNELS) == 0) {
			int32_t d = (int32_t)w - (int32_t)c->rx_last;

			if (d > UA_JUMP_THRESHOLD || d < -UA_JUMP_THRESHOLD) {
				c->st.in_jumps++;
			}
			c->rx_last = w;
		}
		for (int b = 0; b < UA_BYTES_PER_SAMPLE; b++) {
			dst[b] = (uint8_t)(w >> UA_SUBSLOT_SHIFT(b));
		}
		if ((i % UA_CHANNELS) == UA_CHANNELS - 1) {
			c->rx_rd++;
		}
	}
}

/* ---------------------------------------------------------------------- */
/* I2S control                                                              */

static void ua_i2s_run(struct ua_ctx *c)
{
	int ret;

	if (c->i2s_started) {
		return;
	}
	ret = ua_i2s_start();
	if (ret < 0) {
		LOG_ERR("I2S start: %d", ret);
		return;
	}
	c->i2s_started = true;
	c->out_synced = false;
	c->in_started = false;
	c->tx_wr = 0;
	c->rx_rd = 0;
	c->fill_avg_q4 = 0;
	LOG_INF("I2S running");
}

static void ua_i2s_halt(struct ua_ctx *c)
{
	if (!c->i2s_started) {
		return;
	}
	ua_i2s_stop();
	c->i2s_started = false;
	c->out_synced = false;
	c->in_started = false;
	LOG_INF("I2S stopped");
}

/* Frames for the next record packet: nominal, +1 while the ring runs
 * above target, -1 while below. The EWMA smooths the block-granular
 * fill level; the band keeps the regulator from chattering.
 */
static uint16_t ua_next_in_frames(struct ua_ctx *c)
{
	int32_t fill = rx_fill(c);

	c->fill_avg_q4 += ((fill << 4) - c->fill_avg_q4) >> 3;

	int32_t avg = c->fill_avg_q4 >> 4;

	if (avg > UA_RX_TARGET + UA_RX_BAND) {
		c->st.in_plus++;
		return UA_FRAMES_PER_SOF + 1;
	}
	if (avg < UA_RX_TARGET - UA_RX_BAND) {
		c->st.in_minus++;
		return UA_FRAMES_PER_SOF - 1;
	}
	return UA_FRAMES_PER_SOF;
}

static void ua_send_in_packet(struct ua_ctx *c)
{
	int32_t fill = rx_fill(c);
	uint16_t frames;
	uint32_t consumed;
	void *buf;
	int ret;

	if (fill > UA_RX_MAX) {
		/* We fell behind by most of the ring: the DMA is about to
		 * overwrite unread data. Skip ahead to the target level.
		 */
		c->st.ring_overflow++;
		c->rx_rd = ua_i2s_rx_done() * UA_I2S_BLOCK_FRAMES - UA_RX_TARGET;
		c->fill_avg_q4 = UA_RX_TARGET << 4;
	}

	frames = ua_next_in_frames(c);

	if (k_mem_slab_alloc(&ua_in_slab, &buf, K_NO_WAIT) != 0) {
		c->st.oom++;
		return;
	}

	fill = rx_fill(c);
	if (fill < frames) {
		/* Ring ran dry (I2S clock gone?): pad with silence. */
		c->st.in_underrun++;
		memset(buf, 0, frames * UA_BYTES_PER_FRAME);
		consumed = MAX(fill, 0);
		rx_ring_read(c, buf, consumed);
	} else {
		consumed = frames;
		rx_ring_read(c, buf, frames);
	}

	ret = usbd_uac2_send(c->uac2_dev, UA_IN_TERMINAL_ID, buf,
			     frames * UA_BYTES_PER_FRAME);
	if (ret < 0) {
		/* -EAGAIN: class queue full because our SOF callbacks came in
		 * a burst after the thread was held off. Keep the samples in
		 * the ring: the host mirrors what it receives from us back
		 * onto playback, so every frame dropped here would drain the
		 * playback ring by the same amount.
		 */
		c->st.in_busy++;
		c->rx_rd -= consumed;
		k_mem_slab_free(&ua_in_slab, buf);
		return;
	}
	c->st.in_pkts++;
	c->st.in_frames += frames;
}

/* ---------------------------------------------------------------------- */
/* UAC2 callbacks (usbd thread)                                             */

static void ua_terminal_update_cb(const struct device *dev, uint8_t terminal,
				  bool enabled, bool microframes, void *user_data)
{
	struct ua_ctx *c = user_data;

	ARG_UNUSED(dev);
	ARG_UNUSED(microframes); /* Full-Speed only: always false */

	if (terminal == UA_OUT_TERMINAL_ID) {
		c->out_enabled = enabled;
		c->out_synced = false;
		LOG_INF("playback %s", enabled ? "on" : "off");
	} else if (terminal == UA_IN_TERMINAL_ID) {
		c->in_enabled = enabled;
		c->in_started = false;
		LOG_INF("record %s", enabled ? "on" : "off");
	}

	if (c->out_enabled || c->in_enabled) {
		ua_i2s_run(c);
	} else {
		ua_i2s_halt(c);
	}
}

static void *ua_get_recv_buf(const struct device *dev, uint8_t terminal,
			     uint16_t size, void *user_data)
{
	struct ua_ctx *c = user_data;
	void *buf = NULL;

	ARG_UNUSED(dev);

	if (terminal != UA_OUT_TERMINAL_ID) {
		return NULL;
	}
	__ASSERT_NO_MSG(size <= UA_PKT_BUF_BYTES);

	if (!c->out_enabled) {
		return NULL;
	}
	if (k_mem_slab_alloc(&ua_out_slab, &buf, K_NO_WAIT) != 0) {
		c->st.oom++;
		return NULL;
	}
	return buf;
}

static void ua_data_recv_cb(const struct device *dev, uint8_t terminal,
			    void *buf, uint16_t size, void *user_data)
{
	struct ua_ctx *c = user_data;
	uint32_t frames = size / UA_BYTES_PER_FRAME; /* whole frames only */
	int32_t fill;

	ARG_UNUSED(dev);
	ARG_UNUSED(terminal);

	c->st.out_calls++;
	if (frames == 0) {
		c->st.out_empty++;
	}
	if (frames == 0 || !c->out_enabled || !c->i2s_started) {
		goto out;
	}
	c->st.out_pkts++;
	c->st.out_frames += frames;

	fill = tx_fill(c);
	if (!c->out_synced || fill < UA_TX_MIN) {
		/* First packet after (re)start, or the DMA caught up with
		 * the write pointer (the ISR has been playing silence since):
		 * re-position ahead of the block in flight.
		 */
		if (c->out_synced) {
			c->st.out_underrun++;
		}
		c->tx_wr = ua_i2s_tx_done() * UA_I2S_BLOCK_FRAMES + UA_TX_TARGET;
		c->out_synced = true;
	} else if (fill + frames > UA_TX_MAX) {
		/* Host is ahead of the I2S clock by most of the ring: drop. */
		c->st.out_drop++;
		goto out;
	}

	tx_ring_write(c, buf, frames);

out:
	k_mem_slab_free(&ua_out_slab, buf);
}

static void ua_buf_release_cb(const struct device *dev, uint8_t terminal,
			      void *buf, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);

	if (terminal == UA_IN_TERMINAL_ID) {
		k_mem_slab_free(&ua_in_slab, buf);
	}
}

static void ua_sof_cb(const struct device *dev, void *user_data)
{
	struct ua_ctx *c = user_data;

	ARG_UNUSED(dev);

	if (!c->i2s_started || !c->in_enabled) {
		return;
	}

	/* Start record packets only once the ring holds the target level,
	 * so the regulator starts from its set point.
	 */
	if (!c->in_started) {
		if (rx_fill(c) < UA_RX_TARGET) {
			return;
		}
		c->in_started = true;
		c->fill_avg_q4 = rx_fill(c) << 4;
	}

	ua_send_in_packet(c);
}

static const struct uac2_ops ua_uac2_ops = {
	.sof_cb = ua_sof_cb,
	.terminal_update_cb = ua_terminal_update_cb,
	.get_recv_buf = ua_get_recv_buf,
	.data_recv_cb = ua_data_recv_cb,
	.buf_release_cb = ua_buf_release_cb,
};

/* ---------------------------------------------------------------------- */
/* USB device                                                               */

USBD_DEVICE_DEFINE(ua_usbd, DEVICE_DT_GET(DT_NODELABEL(zephyr_udc0)),
		   CONFIG_AES67_USB_AUDIO_VID, CONFIG_AES67_USB_AUDIO_PID);

USBD_DESC_LANG_DEFINE(ua_lang);
USBD_DESC_MANUFACTURER_DEFINE(ua_mfr, CONFIG_AES67_USB_AUDIO_MANUFACTURER);
/* Product string "<hostname> <CONFIG_AES67_USB_AUDIO_PRODUCT>", filled in at
 * setup time from the persisted config (the hostname is what the device is
 * known by on the network, so the host sees the same name on USB).
 */
#define UA_PRODUCT_ASCII_MAX 64
static uint8_t ua_product_ascii[UA_PRODUCT_ASCII_MAX];
static struct usbd_desc_node ua_product = {
	.str = {
		.utype = USBD_DUT_STRING_PRODUCT,
		.ascii7 = true,
	},
	.ptr = ua_product_ascii,
	.bLength = 0, /* set by ua_product_string_build() */
	.bDescriptorType = USB_DESC_STRING,
};
IF_ENABLED(CONFIG_HWINFO, (USBD_DESC_SERIAL_NUMBER_DEFINE(ua_sn)));
USBD_DESC_CONFIG_DEFINE(ua_fs_cfg_desc, "AES67 USB audio");

/* Bus powered, no remote wakeup. */
USBD_CONFIGURATION_DEFINE(ua_fs_config, 0, CONFIG_AES67_USB_AUDIO_MAX_POWER,
			  &ua_fs_cfg_desc);

static void ua_usbd_msg_cb(struct usbd_context *const ctx,
			   const struct usbd_msg *const msg)
{
	ARG_UNUSED(ctx);

	switch (msg->type) {
	case USBD_MSG_CONFIGURATION:
		LOG_INF("USB configured (%d)", msg->status);
		break;
	case USBD_MSG_SUSPEND:
	case USBD_MSG_RESUME:
	case USBD_MSG_RESET:
	case USBD_MSG_VBUS_READY:
	case USBD_MSG_VBUS_REMOVED:
		LOG_INF("USB %s", usbd_msg_type_string(msg->type));
		break;
	case USBD_MSG_UDC_ERROR:
	case USBD_MSG_STACK_ERROR:
		LOG_ERR("USB %s: %d", usbd_msg_type_string(msg->type), msg->status);
		break;
	default:
		break;
	}
}

static void ua_product_string_build(void)
{
	char host[AES67_NODE_ID_MAX];
	size_t len;

	aes67_config_build_hostname(host, sizeof(host));
	if (host[0] != '\0') {
		snprintf((char *)ua_product_ascii, sizeof(ua_product_ascii),
			 "%.48s %s", host, CONFIG_AES67_USB_AUDIO_PRODUCT);
	} else {
		snprintf((char *)ua_product_ascii, sizeof(ua_product_ascii),
			 "%s", CONFIG_AES67_USB_AUDIO_PRODUCT);
	}

	/* Only printable ASCII-7 reaches the wire; anything else is asserted
	 * on by the stack when the descriptor is served.
	 */
	len = strlen((char *)ua_product_ascii);
	for (size_t i = 0; i < len; i++) {
		if (ua_product_ascii[i] <= 0x1F || ua_product_ascii[i] >= 0x7F) {
			ua_product_ascii[i] = '_';
		}
	}
	/* bLength counts the 2-byte header plus UTF-16LE characters. */
	ua_product.bLength = 2U + 2U * len;
}

static int ua_usbd_setup(void)
{
	int ret;

	ua_product_string_build();

	ret = usbd_add_descriptor(&ua_usbd, &ua_lang);
	if (ret == 0) {
		ret = usbd_add_descriptor(&ua_usbd, &ua_mfr);
	}
	if (ret == 0) {
		ret = usbd_add_descriptor(&ua_usbd, &ua_product);
	}
	IF_ENABLED(CONFIG_HWINFO, (
		if (ret == 0) {
			ret = usbd_add_descriptor(&ua_usbd, &ua_sn);
		}
	))
	if (ret < 0) {
		LOG_ERR("USB string descriptors: %d", ret);
		return ret;
	}

	ret = usbd_add_configuration(&ua_usbd, USBD_SPEED_FS, &ua_fs_config);
	if (ret < 0) {
		LOG_ERR("USB FS configuration: %d", ret);
		return ret;
	}

	/* Registers every class instance in the image: only uac2_fpga. */
	ret = usbd_register_all_classes(&ua_usbd, USBD_SPEED_FS, 1, NULL);
	if (ret < 0) {
		LOG_ERR("USB class registration: %d", ret);
		return ret;
	}

	/* UAC2 spans several interfaces joined by an Interface Association
	 * Descriptor; the device descriptor has to announce that.
	 */
	usbd_device_set_code_triple(&ua_usbd, USBD_SPEED_FS,
				    USB_BCC_MISCELLANEOUS, 0x02, 0x01);
	usbd_self_powered(&ua_usbd, false);

	ret = usbd_msg_register_cb(&ua_usbd, ua_usbd_msg_cb);
	if (ret < 0) {
		LOG_ERR("USB message callback: %d", ret);
		return ret;
	}

	ret = usbd_init(&ua_usbd);
	if (ret < 0) {
		LOG_ERR("USB device init: %d", ret);
		return ret;
	}

	return 0;
}

/* ---------------------------------------------------------------------- */
/* Public API                                                               */

int usb_audio_init(void)
{
	struct ua_ctx *c = &ua;
	int ret;

	c->uac2_dev = DEVICE_DT_GET(UA_UAC2_NODE);
	if (!device_is_ready(c->uac2_dev)) {
		LOG_ERR("UAC2 device not ready");
		return -ENODEV;
	}

	ret = ua_i2s_init(UA_SAMPLE_RATE);
	if (ret < 0) {
		return ret;
	}

	usbd_uac2_set_ops(c->uac2_dev, &ua_uac2_ops, c);

	ret = ua_usbd_setup();
	if (ret < 0) {
		return ret;
	}

	LOG_INF("USB audio: %u in / %u out, %u Hz, %u bit in %u-byte subslots, "
		"I2S target of the FPGA clock",
		UA_CHANNELS, UA_CHANNELS, UA_SAMPLE_RATE,
		DT_PROP(DT_NODELABEL(as_iso_out), bit_resolution),
		UA_BYTES_PER_SAMPLE);
	return 0;
}

int usb_audio_start(void)
{
	int ret;

	if (ua.usb_enabled) {
		return 0;
	}
	ret = usbd_enable(&ua_usbd);
	if (ret < 0) {
		LOG_ERR("USB enable: %d", ret);
		return ret;
	}
	ua.usb_enabled = true;
	LOG_INF("USB device enabled");
	return 0;
}

int usb_audio_stop(void)
{
	int ret;

	if (!ua.usb_enabled) {
		return 0;
	}
	ret = usbd_disable(&ua_usbd);
	if (ret < 0) {
		LOG_ERR("USB disable: %d", ret);
		return ret;
	}
	ua.usb_enabled = false;
	ua.out_enabled = false;
	ua.in_enabled = false;
	ua_i2s_halt(&ua);
	LOG_INF("USB device disabled");
	return 0;
}

void usb_audio_get_status(struct usb_audio_status *st)
{
	*st = ua.st;
	st->usb_enabled = ua.usb_enabled;
	st->out_enabled = ua.out_enabled;
	st->in_enabled = ua.in_enabled;
	st->i2s_running = ua.i2s_started;
	st->in_streaming = ua.in_started;
	st->rx_fill = ua.i2s_started ? rx_fill(&ua) : 0;
	st->tx_fill = (ua.i2s_started && ua.out_synced) ? tx_fill(&ua) : 0;
	st->tx_blocks = ua_i2s_tx_done();
	st->rx_blocks = ua_i2s_rx_done();
	st->i2s_errors = ua_i2s_errors();
	ua_i2s_hung(&st->i2s_tx_hung, &st->i2s_rx_hung);
}

/* ---------------------------------------------------------------------- */
/* Shell                                                                    */

#ifdef CONFIG_SHELL
#include <zephyr/shell/shell.h>

static int cmd_ua_status(const struct shell *sh, size_t argc, char **argv)
{
	struct usb_audio_status st;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	usb_audio_get_status(&st);
	shell_print(sh, "USB:       %s", st.usb_enabled ? "enabled" : "disabled");
	shell_print(sh, "playback:  %s   record: %s",
		    st.out_enabled ? "open" : "closed",
		    st.in_enabled ? (st.in_streaming ? "streaming" : "open") : "closed");
	shell_print(sh, "I2S:       %s  blocks tx %u rx %u  errors %u",
		    st.i2s_running ? "running" : "stopped",
		    st.tx_blocks, st.rx_blocks, st.i2s_errors);
	shell_print(sh, "playback fill %d frames (target %u)  record fill %d frames (target %u)",
		    st.tx_fill, UA_TX_TARGET, st.rx_fill, UA_RX_TARGET);
	shell_print(sh, "OUT pkts %u  (completions %u, empty %u)  underrun %u  dropped %u",
		    st.out_pkts, st.out_calls, st.out_empty, st.out_underrun, st.out_drop);
	shell_print(sh, "IN  pkts %u  +1 %u  -1 %u  underrun %u  overflow %u",
		    st.in_pkts, st.in_plus, st.in_minus, st.in_underrun, st.ring_overflow);
	shell_print(sh, "OOM %u  in busy %u  frames out %u in %u  i2s tx %u rx %u",
		    st.oom, st.in_busy, st.out_frames, st.in_frames,
		    st.tx_blocks * UA_I2S_BLOCK_FRAMES, st.rx_blocks * UA_I2S_BLOCK_FRAMES);
	shell_print(sh, "jumps out %u in %u  i2s fifo hung tx %u rx %u",
		    st.out_jumps, st.in_jumps, st.i2s_tx_hung, st.i2s_rx_hung);
	return 0;
}

static int cmd_ua_start(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	int ret = usb_audio_start();

	if (ret < 0) {
		shell_error(sh, "start failed: %d", ret);
	}
	return ret;
}

static int cmd_ua_stop(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	int ret = usb_audio_stop();

	if (ret < 0) {
		shell_error(sh, "stop failed: %d", ret);
	}
	return ret;
}

static int cmd_ua_clear(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(sh);
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	memset(&ua.st, 0, sizeof(ua.st));
	return 0;
}

/* Bring-up diagnostics: raw DWC2 registers of the two audio endpoints. */
#include <usb_dwc2_hw.h>

static int cmd_ua_dwc2(const struct shell *sh, size_t argc, char **argv)
{
	struct usb_dwc2_reg *const base =
		(struct usb_dwc2_reg *)DT_REG_ADDR(DT_NODELABEL(usb_otg));

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "gintsts 0x%08x gintmsk 0x%08x dcfg 0x%08x dctl 0x%08x dsts 0x%08x",
		    base->gintsts, base->gintmsk, base->dcfg, base->dctl, base->dsts);
	shell_print(sh, "ghwcfg3 0x%08x gdfifocfg 0x%08x grxfsiz 0x%08x gnptxfsiz 0x%08x dieptxf1 0x%08x",
		    base->ghwcfg3, base->gdfifocfg, base->grxfsiz, base->gnptxfsiz,
		    base->dieptxf[0]);
	shell_print(sh, "daint 0x%08x daintmsk 0x%08x doepmsk 0x%08x diepmsk 0x%08x",
		    base->daint, base->daintmsk, base->doepmsk, base->diepmsk);
	shell_print(sh, "OUT1 doepctl 0x%08x doepint 0x%08x doeptsiz 0x%08x doepdma 0x%08x",
		    base->out_ep[1].doepctl, base->out_ep[1].doepint,
		    base->out_ep[1].doeptsiz, base->out_ep[1].doepdma);
	shell_print(sh, "IN1  diepctl 0x%08x diepint 0x%08x dieptsiz 0x%08x diepdma 0x%08x dtxfsts 0x%08x",
		    base->in_ep[1].diepctl, base->in_ep[1].diepint,
		    base->in_ep[1].dieptsiz, base->in_ep[1].diepdma, base->in_ep[1].dtxfsts);
	shell_print(sh, "gsnpsid 0x%08x", base->gsnpsid);
	return 0;
}


SHELL_STATIC_SUBCMD_SET_CREATE(sub_usbaudio,
	SHELL_CMD(status, NULL, "Bridge state and counters", cmd_ua_status),
	SHELL_CMD(dwc2, NULL, "Dump DWC2 endpoint registers", cmd_ua_dwc2),
	SHELL_CMD(start, NULL, "Enable the USB device", cmd_ua_start),
	SHELL_CMD(stop, NULL, "Disable the USB device", cmd_ua_stop),
	SHELL_CMD(clear, NULL, "Reset the counters", cmd_ua_clear),
	SHELL_SUBCMD_SET_END);
SHELL_CMD_REGISTER(usbaudio, &sub_usbaudio, "USB audio interface (UAC2 <-> FPGA I2S)", NULL);
#endif /* CONFIG_SHELL */
