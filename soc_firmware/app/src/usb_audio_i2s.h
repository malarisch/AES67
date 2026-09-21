/*
 * Gapless I2S target (slave) port with GDMA descriptor rings, ESP32-S3.
 *
 * Why not the Zephyr I2S driver: drivers/i2s/i2s_esp32.c programs the
 * GDMA one block at a time and restarts it from the completion interrupt,
 * so every block boundary costs the ISR latency in samples (measured 0.5 to
 * 2 % at 1 ms blocks). The Zephyr GDMA driver has no cyclic mode to fix
 * that. This port builds the descriptor rings itself, the way ESP-IDF's I2S
 * driver does: the DMA never stops, the EOF interrupt only advances a
 * block counter.
 *
 * Data model: each direction is one contiguous buffer of UA_I2S_BLOCKS
 * blocks x block_frames frames (interleaved L/R 32-bit words). The
 * hardware works block by block; the application reads/writes at frame
 * granularity against the completed-block counters:
 *
 *   TX: app writes at tx_wr (frames), DMA has consumed tx_done() x block
 *       frames. Keep tx_wr ahead of the block in flight.
 *   RX: DMA has produced rx_done() x block frames, app reads at rx_rd.
 *
 * The TX EOF handler clears the block it just finished, so an underrun
 * plays silence instead of stale data.
 */
#ifndef USB_AUDIO_I2S_H
#define USB_AUDIO_I2S_H

#include <stdint.h>
#include <stddef.h>

/* Powers of two throughout, so ring positions are monotonic counters
 * masked on use and survive the 32-bit wrap (2^32 is a multiple of the
 * ring size). 32-frame blocks = an EOF interrupt every 0.67 ms at 48 kHz;
 * 32 blocks = 21 ms per direction, room for a 6 ms playback lead plus
 * bursts of late packets.
 */
#define UA_I2S_CHANNELS     2
#define UA_I2S_BLOCK_FRAMES 32
#define UA_I2S_BLOCKS       32
#define UA_I2S_RING_FRAMES  (UA_I2S_BLOCKS * UA_I2S_BLOCK_FRAMES)

/* Configure clocks, pins, slots (48 kHz, 2 x 32-bit, I2S, target mode),
 * GDMA channel + descriptor rings and the interrupts. Idempotent.
 */
int ua_i2s_init(uint32_t sample_rate);

/* Start both directions from a clean state: TX ring must be pre-filled
 * by the caller (frames [0, prefill) are played first), RX ring is empty.
 * Counters restart at zero.
 */
int ua_i2s_start(void);
void ua_i2s_stop(void);

/* Monotonic block completion counters (advance in the EOF interrupt). */
uint32_t ua_i2s_tx_done(void);
uint32_t ua_i2s_rx_done(void);

/* Ring storage, UA_I2S_RING_FRAMES x UA_I2S_CHANNELS words each. */
uint32_t *ua_i2s_tx_ring(void);
const uint32_t *ua_i2s_rx_ring(void);

/* Error interrupt counter (descriptor errors, RX error EOF). */
uint32_t ua_i2s_errors(void);

/* I2S FIFO timeout ("hung") events since start: the TX FIFO ran dry or
 * the RX FIFO was not drained. Reads and clears the raw status bits.
 */
void ua_i2s_hung(uint32_t *tx, uint32_t *rx);

#endif /* USB_AUDIO_I2S_H */
