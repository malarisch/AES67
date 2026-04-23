# SPI Register Map

The SPI command byte is `[R/W][addr6..addr0]` where bit 7 = 1 → write, 0 → read.
CS_N is not used for framing (some masters toggle it mid-transfer); transaction
length is determined by the register's declared payload length. After the last
payload byte the controller returns to IDLE, so back-to-back transactions
without CS toggle are legal.

All multi-byte fields are **big-endian (MSB first)** on the wire. Multi-byte
writes to scalar registers (MAC, IP, PTP config, flags) are **atomic**: bytes
are collected in a shadow register and only committed to the FPGA outputs on
the last byte of the transaction. Stream-config writes pass through byte-wise
to block RAM (tx_router / rx_ringbuffer).

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

### 0x50 — Clocking status (1 B)
| Bit | Meaning                      |
|-----|------------------------------|
| 7   | PLL PPB measurement valid    |
| 6   | Wallclock locked             |
| 5   | Wallclock configured         |
| 4   | PTP is leader                |
| 3   | PTP is follower              |
| 2–0 | reserved                     |

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

---

## Writes

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
| 4   | Meter clear (pulse, 1 sys_clk)          |
| 5   | ADDA nRST (level, high = run)           |
| 6–7 | reserved                                |

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
