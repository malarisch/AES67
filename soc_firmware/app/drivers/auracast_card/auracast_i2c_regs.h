/*
 * Vendored from the auracast-s31 repository,
 * components/ctrl_regs/include/auracast_i2c_regs.h @ 68392be + uncommitted 2026-10-20 (exact-length read rule).
 * Do not edit here; update by copying the file again.
 */
/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * I2C control protocol of the auracast-s31 bridge (S31 = I2C slave, 7-bit address, default 0x3A).
 * Self-contained (stdint only) so the I2C master firmware (Zephyr) can include it as is.
 *
 * Access: 8-bit register address, auto-increment (0xFF wraps to 0x00), multi-byte values little endian.
 *   Write: [reg][data...]                         e.g. i2c_burst_write()
 *   Read:  [reg], repeated start, read N bytes    e.g. i2c_write_read()
 * The S31 stretches SCL at the start of a read until the data is ready (typically well below 1 ms):
 * the master must allow clock stretching. Reading an address without a register returns 0x00.
 * A read must take exactly AURA_READ_MAX bytes (longer fields, e.g. NAME or the status block, take two
 * reads) and must always be preceded by a register address write (write_read). A shorter read leaves one
 * byte pre-loaded in the slave's shift register, which then prefixes every later read; a read of
 * AURA_READ_MAX + 1 bytes drains it.
 *
 * Life cycle:
 *   boot -> WAIT_CONFIG: nothing is broadcast; the master writes the configuration registers
 *   (power-on values below, any order), then writes AURA_CMD_START to COMMAND.
 *   START -> STARTING -> STREAMING, or CONFIG_ERROR (ERROR/ERROR_REG name the offending register;
 *   fix it and send START again), or FAULT (the Bluetooth stack refused: only RESET helps).
 *   From STARTING on, configuration writes are ignored (ERROR = LOCKED). To change the configuration,
 *   write AURA_CMD_RESET: the S31 restarts ~10 ms later and is back in WAIT_CONFIG.
 *   The configuration lives in RAM only: after any restart (also watchdog/brownout) the S31 waits again.
 * Rejected writes never NAK; they set ERROR (and ERROR_REG) instead.
 */
#pragma once

#include <stdint.h>

#define AURA_I2C_DEFAULT_ADDR      0x3A
#define AURA_PROTO_VERSION         1
#define AURA_READ_MAX              31      /* bytes per read transaction, exactly (one hardware FIFO fill) */

/* ---- Identification and control ---------------------------------------- */
#define AURA_REG_MAGIC             0x00    /* R  4  'A','U','R','A' */
#define AURA_REG_PROTO_VERSION     0x04    /* R  1  AURA_PROTO_VERSION */
#define AURA_REG_FW_VERSION        0x05    /* R  3  major, minor, patch */
#define AURA_REG_STATE             0x08    /* R  1  AURA_STATE_* */
#define AURA_REG_ERROR             0x09    /* R  1  AURA_ERR_* of the last rejection, 0 = none */
#define AURA_REG_ERROR_REG         0x0A    /* R  1  register address the error refers to */
#define AURA_REG_RESET_REASON      0x0B    /* R  1  esp_reset_reason_t of the last restart */
#define AURA_REG_COMMAND           0x0C    /* W  1  AURA_CMD_*, reads 0 */

/* ---- Configuration (writable in WAIT_CONFIG and CONFIG_ERROR) ---------- */
#define AURA_REG_PRESET            0x10    /* RW 1  AURA_PRESET_* */
#define AURA_REG_CHANNELS          0x11    /* RW 1  AURA_CH_* */
#define AURA_REG_CONTEXT           0x12    /* RW 2  streaming context bit mask, != 0, within AURA_CONTEXT_MASK (Live 0x0040, Media 0x0004) */
#define AURA_REG_FLAGS             0x14    /* RW 1  AURA_FLAG_*, other bits must be 0 */
#define AURA_REG_TX_POWER          0x15    /* RW 1  int8 dBm, AURA_TX_POWER_MIN..MAX */
#define AURA_REG_PD_US             0x16    /* RW 2  presentation delay in us, AURA_PD_US_MIN..MAX */
#define AURA_REG_FIFO_TARGET       0x18    /* RW 2  input FIFO set point in frames @ 48 kHz */
#define AURA_REG_BROADCAST_ID      0x1A    /* RW 4  AURA_BROADCAST_ID_RANDOM or 0..0xFFFFFF */
#define AURA_REG_NAME              0x20    /* RW 32 Broadcast_Name, UTF-8, 4..32 bytes up to the first 0x00 */
#define AURA_REG_CODE              0x40    /* RW 16 Broadcast_Code, 0..16 bytes up to the first 0x00; empty = unencrypted */
/* NAME and CODE: always write the whole field, padded with 0x00, or the tail of the old value remains */

#define AURA_NAME_SIZE             32
#define AURA_CODE_SIZE             16

/* ---- Status (read only, refreshed every 500 ms, meaningful from STREAMING on) */
#define AURA_REG_PIPE_STATE        0x80    /* R  1  AURA_PIPE_* */
#define AURA_REG_STATUS_FLAGS      0x81    /* R  1  AURA_STATUS_* */
#define AURA_REG_FIFO_FILL         0x82    /* R  2  input FIFO fill in frames @ 48 kHz */
#define AURA_REG_DRIFT_CPPM        0x84    /* R  4  int32, drift controller correction in 0.01 ppm */
#define AURA_REG_IN_CPPM           0x88    /* R  4  int32, measured input rate vs. 48 kHz in 0.01 ppm */
#define AURA_REG_I2S_BITS          0x8C    /* R  4  OR of all I2S slot words since the last refresh (24 bit: low byte 0) */
#define AURA_REG_UNDERRUN          0x90    /* R  4  FIFO underruns */
#define AURA_REG_OVERFLOW          0x94    /* R  4  FIFO overflows */
#define AURA_REG_LOST              0x98    /* R  4  I2S signal losses */
#define AURA_REG_SEND_ERR          0x9C    /* R  4  SDUs the controller refused */
#define AURA_REG_SENT              0xA0    /* R  4  SDU sets sent */
#define AURA_REG_ACTIVE_ID         0xA4    /* R  4  Broadcast_ID in use */
#define AURA_REG_UPTIME_MS         0xA8    /* R  4  ms since boot */
#define AURA_REG_STATUS_END        0xAC

/* ---- Values ------------------------------------------------------------- */
#define AURA_CMD_START             0x5A
#define AURA_CMD_RESET             0xA5

#define AURA_STATE_WAIT_CONFIG     0
#define AURA_STATE_STARTING        1
#define AURA_STATE_STREAMING       2
#define AURA_STATE_CONFIG_ERROR    3
#define AURA_STATE_FAULT           4

/* ERROR/ERROR_REG hold the last rejection; an accepted START clears them */
#define AURA_ERR_NONE              0
#define AURA_ERR_READ_ONLY         1       /* write to a read-only or reserved address */
#define AURA_ERR_LOCKED            2       /* configuration write or START after START */
#define AURA_ERR_BAD_VALUE         3       /* START: ERROR_REG holds an invalid value */
#define AURA_ERR_BAD_COMMAND       4       /* unknown COMMAND value */
#define AURA_ERR_START_FAILED      5       /* Bluetooth stack refused the configuration (state FAULT) */

/* LC3 broadcast presets, all with 10 ms frames */
#define AURA_PRESET_16_2_1         0       /* 16 kHz, 40 B, Standard Quality */
#define AURA_PRESET_24_2_1         1       /* 24 kHz, 60 B, Standard Quality */
#define AURA_PRESET_48_2_1         2       /* 48 kHz, 100 B, High Quality */
#define AURA_PRESET_48_4_1         3       /* 48 kHz, 120 B, High Quality */
#define AURA_PRESET_48_6_1         4       /* 48 kHz, 155 B, High Quality */
#define AURA_PRESET_COUNT          5

#define AURA_CH_MONO_MIX           0       /* one BIS, (L+R)/2 */
#define AURA_CH_STEREO             1       /* two BIS, front left / front right */
#define AURA_CH_MONO_LEFT          2       /* one BIS, left input only */
#define AURA_CH_MONO_RIGHT         3       /* one BIS, right input only */
#define AURA_CH_COUNT              4

#define AURA_FLAG_IMMEDIATE        0x01    /* announce Broadcast_Audio_Immediate_Rendering_Flag */

#define AURA_CONTEXT_MASK          0x0FFF  /* defined context type bits; the others are reserved */

#define AURA_TX_POWER_MIN          (-15)
#define AURA_TX_POWER_MAX          20
#define AURA_PD_US_MIN             20000
#define AURA_PD_US_MAX             40000
#define AURA_FIFO_TARGET_MIN       840
#define AURA_FIFO_TARGET_MAX       3000
#define AURA_BROADCAST_ID_RANDOM   0xFFFFFFFFu
#define AURA_NAME_MIN_LEN          4

#define AURA_PIPE_NO_SIGNAL        0
#define AURA_PIPE_PRIME            1
#define AURA_PIPE_RUN              2
#define AURA_PIPE_RATE_ERROR       3

#define AURA_STATUS_I2S_SIGNAL     0x01    /* I2S clock present */
#define AURA_STATUS_BIS_STREAMING  0x02    /* all BIS are streaming */

/* ---- Power-on values of the configuration registers (Live mode for hearing aids) */
#define AURA_DEFAULT_PRESET        AURA_PRESET_24_2_1
#define AURA_DEFAULT_CHANNELS      AURA_CH_MONO_MIX
#define AURA_DEFAULT_CONTEXT       0x0040  /* Live */
#define AURA_DEFAULT_FLAGS         AURA_FLAG_IMMEDIATE
#define AURA_DEFAULT_TX_POWER      9
#define AURA_DEFAULT_PD_US         20000
#define AURA_DEFAULT_FIFO_TARGET   840
#define AURA_DEFAULT_BROADCAST_ID  AURA_BROADCAST_ID_RANDOM
#define AURA_DEFAULT_NAME          "AES67 Auracast"
