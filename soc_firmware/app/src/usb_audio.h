/*
 * USB Audio Class 2 <-> FPGA I2S bridge (ESP32-S3 external-MCU build).
 *
 * The ESP32-S3 enumerates as a UAC2 device (2 in / 2 out, 48 kHz, 24 bit)
 * and moves the samples between the isochronous endpoints and an I2S port
 * on which it is the target (slave) of the FPGA's media clock. See
 * soc_firmware/docs/usb-audio-plan.md for the design.
 */
#ifndef USB_AUDIO_H
#define USB_AUDIO_H

#include <stdbool.h>
#include <stdint.h>

/* Snapshot of the bridge state for the shell / web API. */
struct usb_audio_status {
	bool usb_enabled;      /* usbd_enable() succeeded */
	bool out_enabled;      /* host opened the playback (OUT) stream */
	bool in_enabled;       /* host opened the record (IN) stream */
	bool i2s_running;      /* I2S TX+RX DMA rings started */
	bool in_streaming;     /* record packets are being sent */
	int32_t tx_fill;       /* playback frames ahead of the DMA */
	int32_t rx_fill;       /* record frames waiting for the host */
	uint32_t tx_blocks;    /* I2S TX blocks played since start */
	uint32_t rx_blocks;    /* I2S RX blocks captured since start */
	uint32_t i2s_errors;   /* GDMA error interrupts */
	uint32_t i2s_tx_hung;  /* I2S TX FIFO timeout events */
	uint32_t i2s_rx_hung;  /* I2S RX FIFO timeout events */
	uint32_t out_jumps;    /* playback ch0 sample steps > 1/2 full scale */
	uint32_t in_jumps;     /* record ch0 sample steps > 1/2 full scale */
	uint32_t out_calls;    /* OUT transfer completions (any length) */
	uint32_t out_empty;    /* ... of which zero-length (missed frame / cancelled) */
	uint32_t out_pkts;     /* playback packets received from the host */
	uint32_t out_underrun; /* DMA caught up with playback, re-synced */
	uint32_t out_drop;     /* playback packets dropped (ring full) */
	uint32_t in_pkts;      /* record packets sent */
	uint32_t in_busy;      /* record packets not queued (class queue full) */
	uint32_t out_frames;   /* playback frames received (sum) */
	uint32_t in_frames;    /* record frames sent (sum) */
	uint32_t in_plus;      /* record packets with nominal + 1 frames */
	uint32_t in_minus;     /* record packets with nominal - 1 frames */
	uint32_t in_underrun;  /* record packets padded (ring ran dry) */
	uint32_t ring_overflow;/* record read position re-synced (lapped) */
	uint32_t oom;          /* USB buffer allocation failures */
};

/* Set up I2S + the USB device context. Does not enable USB. */
int usb_audio_init(void);

/* Enable / disable the USB device (enumeration towards the host). */
int usb_audio_start(void);
int usb_audio_stop(void);

void usb_audio_get_status(struct usb_audio_status *st);

#endif /* USB_AUDIO_H */
