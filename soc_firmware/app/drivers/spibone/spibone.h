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
 * One byte of payload per 32-bit word (eth_buf / stream-cfg packing).
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

#ifdef __cplusplus
}
#endif

#endif /* SPIBONE_H_ */
