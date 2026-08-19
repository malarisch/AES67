/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * Wishbone-over-SPI transport — the LiteX `spibone` 4-wire bridge.
 *
 * Zephyr port of the Linux bus layer (driver/aes67_eth/aes67_bus.c), which in
 * turn mirrors config_tool/crates/transport/src/spi.rs byte-for-byte:
 *
 *   write: [0x00][addr BE][value BE]; the device holds MISO high (0xff) until
 *          the Wishbone write completes, then returns a 0x00 ack byte.
 *   read:  [0x01][addr BE]; the device holds MISO high until data is ready,
 *          then a 0x01 sync byte followed by the 32-bit value (big-endian).
 *
 * spibone drops the low two address bits in gateware, so the full byte
 * address goes on the wire. Each transaction is one full-duplex SPI transfer
 * (CS held low throughout) with trailing 0xff padding clocked out to capture
 * the variable-latency response.
 *
 * The frame hot paths additionally use the repo-local spibone fork's burst
 * commands (litex_soc/spi_bone.py, `with_burst`):
 *   [0x02][addr BE][count BE16][data0 BE32]..  auto-incrementing burst write
 *   [0x03][addr BE][count BE16]                burst read, per word a 0x01
 *                                              sync byte + BE32 value
 * `count` is a word count; the payload packing per word depends on the
 * target region:
 *   - stream-cfg RAMs (and legacy eth_buf gateware): one payload byte in
 *     the low byte of each word — the *_burst() helpers.
 *   - eth_buf on packed gateware (eth_buf_bytes_per_word CSR reads 4):
 *     four payload bytes per word, little-endian lanes (frame byte 4i+k
 *     in word i bits [8k+7:8k]) — the *_burst_packed_locked() helpers.
 */

#ifndef SPIBONE_H_
#define SPIBONE_H_

#include <zephyr/device.h>
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Get the spibone bus device pointer (NULL if not ready).
 */
const struct device *spibone_get_dev(void);

/**
 * @brief Bus utilization counters (CONFIG_SPIBONE_STATS).
 *
 * Times are in k_cycle_get_32() cycles (CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC).
 * All counters accumulate since boot; consumers diff two snapshots for
 * windowed rates. Exception: @p lock_wait_max_cyc is reset by every
 * spibone_stats_get() call, so it is the maximum within the sample window.
 */
struct spibone_stats {
	uint64_t lock_wait_cyc;     /* time threads spent blocked on the bus mutex */
	uint64_t lock_hold_cyc;     /* time the bus mutex was held */
	uint64_t xfer_cyc;          /* time inside SPI transfers (wire busy) */
	uint64_t xfer_bytes;        /* bytes clocked per transfer (full duplex counted once) */
	uint32_t xfer_count;        /* SPI transfers */
	uint32_t xfer_errors;       /* transfers that returned an error */
	uint32_t lock_count;        /* bus_lock() calls (incl. recursive re-locks) */
	uint32_t lock_contended;    /* bus_lock() calls that had to wait */
	uint32_t lock_wait_max_cyc; /* longest single wait since the last _get() */
};

/**
 * @brief Snapshot the utilization counters (and reset the windowed max).
 * All-zero when CONFIG_SPIBONE_STATS is disabled.
 */
void spibone_stats_get(struct spibone_stats *out);

/**
 * @brief Take / release the bus lock.
 *
 * The single-transaction helpers below lock internally; use these only to
 * make a read-modify-write sequence (or a multi-register update) atomic
 * with respect to other bus users, together with the *_locked variants.
 */
void spibone_bus_lock(void);
void spibone_bus_unlock(void);

/** Single 32-bit Wishbone read/write at byte address @p addr.
 *  @return 0 on success, -EIO on protocol error, -ETIMEDOUT if the device
 *  never answered, or a negative SPI driver error. */
int spibone_read_locked(uint32_t addr, uint32_t *val);
int spibone_write_locked(uint32_t addr, uint32_t val);

/** Convenience wrappers that take the bus lock themselves. */
int spibone_read(uint32_t addr, uint32_t *val);
int spibone_write(uint32_t addr, uint32_t val);

/**
 * @brief Burst write: store @p n bytes as @p n consecutive 32-bit words
 * starting at byte address @p addr (one payload byte in the low byte of each
 * word — the eth_buf / stream-cfg packing). Falls back to single-word writes
 * when CONFIG_SPIBONE_BURST is disabled (pre-burst gateware).
 */
int spibone_write_burst_locked(uint32_t addr, const uint8_t *bytes, size_t n);

/**
 * @brief Burst read: fetch @p n consecutive 32-bit words starting at byte
 * address @p addr, returning the low byte of each into @p bytes.
 */
int spibone_read_burst_locked(uint32_t addr, uint8_t *bytes, size_t n);

int spibone_write_burst(uint32_t addr, const uint8_t *bytes, size_t n);
int spibone_read_burst(uint32_t addr, uint8_t *bytes, size_t n);

/**
 * @brief Packed burst write: store @p n payload bytes as ceil(n/4)
 * consecutive 32-bit words starting at byte address @p addr, four bytes per
 * word in little-endian lanes (packed eth_buf layout). A final partial word
 * is zero-padded on the wire. Falls back to single-word writes without
 * CONFIG_SPIBONE_BURST.
 */
int spibone_write_burst_packed_locked(uint32_t addr, const uint8_t *bytes,
				      size_t n);

/**
 * @brief Packed burst read: fetch @p n payload bytes from ceil(n/4)
 * consecutive 32-bit words starting at byte address @p addr (four bytes per
 * word, little-endian lanes). Never writes more than @p n bytes to @p bytes;
 * surplus lanes of the final word are discarded.
 */
int spibone_read_burst_packed_locked(uint32_t addr, uint8_t *bytes, size_t n);

#ifdef __cplusplus
}
#endif

#endif /* SPIBONE_H_ */
