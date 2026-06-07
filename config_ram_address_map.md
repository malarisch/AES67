# SPI Register Map

The SPI command byte is `[R/W][addr6..addr0]` where bit 7 = 1 → write, 0 → read.
CS_N is not used for framing (some masters toggle it mid-transfer); transaction
length is determined by the register's declared payload length. After the last
payload byte the controller returns to IDLE, so back-to-back transactions
without CS toggle are legal.

Byte order on the wire is **not uniform** — it is per register, so check each
entry below:

- **Writes** to scalar registers (MAC, IP) are **big-endian / network byte
  order** (byte 0 = MSB).
- **Multi-byte reads** of PTP status (`0x52` path delay, `0x53` offset, `0x54`
  PPB counters, `0x55` GM identity) are **little-endian** (byte 0 = bits 7..0).
- The TX-stream **SSRC** field is big-endian; stream IP/port match fields are
  network byte order.

Multi-byte writes to scalar registers (MAC, IP, PTP config, flags) are
**atomic**: bytes are collected in a shadow register and only committed to the
FPGA outputs on the last byte of the transaction. Stream-config writes pass
through byte-wise to block RAM (tx_router / rx_ringbuffer).

---

## Reads

### 0x00 — FPGA info (8 B)
| Offset | Field              |
|--------|--------------------|
| 0      | FPGA version MSB   |
| 1      | FPGA version LSB   |
| 2      | TX streams         |
| 3      | RX streams         |
| 4      | TX channels        |
| 5      | RX channels        |
| 6      | Bit depth          |
| 7      | Sample rate (kHz)  |

### 0x21 — RX Ethernet frame length (2 B)
Big-endian: byte 0 = length[10:8], byte 1 = length[7:0]. Valid when the RX-frame-
available bit (read `0x50` bit 2) is set.

### 0x22 — RX Ethernet frame data (variable)
Streams `length - 1` bytes of the pending received frame straight out of the RX
packet RAM (length from `0x21`). Reading this register releases the frame and
clears the available flag.

### 0x30 — Channel metering (variable)
`(RXCHANNELS/8)*2 + (TXCHANNELS/8)*2` bytes: RX signal bitmap, RX clip bitmap,
TX signal bitmap, TX clip bitmap (one bit per channel). The snapshot is cleared
automatically once all bytes have been read.

### 0x50 — Clocking status (1 B)
| Bit | Meaning                      |
|-----|------------------------------|
| 7   | PLL PPB measurement valid    |
| 6   | Wallclock locked             |
| 5   | Wallclock configured         |
| 4   | PTP is leader                |
| 3   | PTP is follower              |
| 2   | Ethernet RX frame available  |
| 1   | RX overflow                  |
| 0   | reserved                     |

### 0x51 — Ethernet status (1 B)
| Bit | Meaning                       |
|-----|-------------------------------|
| 7   | Link up                       |
| 6–5 | Link speed (00=10, 01=100, 10=1000 Mbps) |
| 4–0 | reserved                      |

### 0x52 — PTP mean path delay (4 B)
32-bit unsigned, little-endian byte order on the wire (byte 0 = bits 7..0).

### 0x53 — PTP leader offset (4 B)
32-bit signed, little-endian byte order on the wire.

### 0x54 — PLL/WC PPB counters (8 B)
Bytes 0–3: PLL counter (little-endian). Bytes 4–7: wallclock counter (little-endian).

### 0x55 — Current grandmaster clock identity (8 B)
Little-endian byte order on the wire.

### 0x61 — PTP servo monitoring (22 B)
Only meaningful when the core is built with dynamic PTP tuning
(`STATIC_PTP_CONF /= "TRUE"`); otherwise reads zero. Little-endian fields:
filtered offset (4 B), integral sum (4 B), PI proportional (4 B), PI sum raw
(4 B), effective gain shift (1 B), lock counter (2 B), sample count (2 B),
first-lock-achieved flag (1 B).

---

## Writes

### 0x20 — TX Ethernet frame (variable)
Frame to be transmitted by the FPGA on behalf of the host. Byte 0..1 = frame
length (big-endian, 11-bit), followed by the frame payload. Because the ESP32
splits long transfers into ≤64-byte CS bursts, this register keeps the
transaction open across short CS-high gaps (see `PACKET_CS_GAP_TIMEOUT`).

### 0x40 — MAC address (6 B, atomic)
Network byte order: byte 0 = MAC[47:40], byte 5 = MAC[7:0].

### 0x41 — IP address (4 B, atomic)
Network byte order: byte 0 = IP[31:24], byte 3 = IP[7:0].

### 0x50 — Control flags (1 B, atomic)
| Bit | Meaning                                 |
|-----|-----------------------------------------|
| 0   | Start PLL PPB measurement (level; held until `pll_meas_valid` falls, then auto-cleared) |
| 1   | Reset wallclock (level)                 |
| 2   | Reset PTP (level)                       |
| 3   | Reset Ethernet (level)                  |
| 4   | reserved (unused)                       |
| 5   | ADDA nRST (level, high = run)           |
| 6–7 | reserved                                |

Note: there is no "meter clear" flag bit. The metering snapshot (read `0x30`)
self-clears once the host has read out all `metering_bytes` of it.

### 0x55 — PTP configuration (7 B, atomic)
| Offset | Field                             |
|--------|-----------------------------------|
| 0      | PTP time source                   |
| 1      | Log message interval (Sync)       |
| 2      | Log message interval (Announce)   |
| 3      | GM priority1                      |
| 4      | GM priority2                      |
| 5      | GM clock class                    |
| 6      | GM clock accuracy                 |

Note: the current leader clock identity is read-only status (see read 0x55)
and is determined on-chip by the BMC algorithm; it cannot be written.

### 0x58 — TX stream configuration (20 B, byte-wise to RAM)
Target RAM base address = `stream_id * 32` (computed by spictrl from byte 0).
Bytes 1..19 of the SPI payload are written to RAM offsets 0x01..0x13.
Byte 0 (stream_id) is also echoed to RAM offset 0x00 for consistency.

| SPI byte | RAM offset | Field                                    |
|----------|------------|------------------------------------------|
| 0        | 0x00       | stream_id (0..7) — selects RAM base      |
| 1–4      | 0x01–0x04  | Destination IP (network byte order)      |
| 5        | 0x05       | Channel count (1..8)                     |
| 6        | 0x06       | Samples per packet per channel           |
| 7–14     | 0x07–0x0E  | Channel IDs (up to 8, one byte each)     |
| 15       | 0x0F       | reserved                                 |
| 16–19    | 0x10–0x13  | SSRC (32-bit, big-endian)                |

### 0x59 — RX stream configuration (18 B, byte-wise to RAM)
Target RAM base address = `stream_id * 32` (computed by spictrl from byte 0).
Bytes 1..17 of the SPI payload are written to RAM offsets 0x00..0x10.

| SPI byte | RAM offset | Field                                       |
|----------|------------|---------------------------------------------|
| 0        | —          | stream_id (0..7) — selects RAM base, not stored |
| 1–4      | 0x00–0x03  | Destination IP (big-endian) — match filter  |
| 5–6      | 0x04–0x05  | Destination UDP port (big-endian) — match   |
| 7–14     | 0x06–0x0D  | Channel output map (low nibble used)        |
| 15       | 0x0E       | Channel count (1..8)                        |
| 16       | 0x0F       | Output delay (samples)                      |
| 17       | 0x10       | Samples per channel per packet              |

### 0x60 — PTP servo / parser tuning (18 B, atomic)
Applied only when the core is built with dynamic PTP tuning
(`STATIC_PTP_CONF /= "TRUE"`); ignored otherwise. Byte order:

| Offset | Field                                   |
|--------|-----------------------------------------|
| 0      | servo Kp gain (signed)                  |
| 1      | servo Ki gain (signed)                  |
| 2      | servo gain shift (5-bit)                |
| 3      | servo gain shift when locked (5-bit)    |
| 4      | servo Ki extra shift (5-bit)            |
| 5      | servo filter shift (5-bit)              |
| 6      | servo warmup samples                    |
| 7–10   | lock threshold ns (32-bit, little-endian) |
| 11–14  | unlock threshold ns (32-bit, little-endian) |
| 15     | lock count threshold                    |
| 16     | parser min-filter enable (bit 0)        |
| 17     | parser min-filter active depth          |
