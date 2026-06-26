// SPDX-License-Identifier: GPL-2.0
/*
 * AES67 Wishbone-over-SPI bus layer + /dev/aes67ctl control char device.
 *
 * Implements the LiteX spibone 4-wire wire protocol (see
 * config_tool/crates/transport/src/spi.rs, which this mirrors byte-for-byte):
 *
 *   write: [0x00][addr BE][value BE]; device holds MISO high (0xff) until the
 *          Wishbone write completes, then returns a 0x00 ack byte.
 *   read:  [0x01][addr BE]; device holds MISO high until data is ready, then a
 *          0x01 sync byte followed by the 32-bit value (big-endian).
 *
 * spibone drops the low two address bits in gateware, so the full byte address
 * goes on the wire. Each transaction is one full-duplex SPI transfer with
 * trailing 0xff padding clocked out to capture the variable-latency response.
 *
 * For the frame hot paths we also use the repo-local spibone fork's burst
 * commands (aes67_wb_{write,read}_burst_locked): [0x02|0x03][addr BE][count
 * BE16][...] streams many auto-incrementing words per SPI transfer instead of
 * one round-trip per word. The single-word ops above are unchanged.
 */
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/unaligned.h>

#include "aes67_eth.h"
#include "aes67_uapi.h"

#define CMD_WRITE 0x00
#define CMD_READ  0x01
/* Burst variants (repo-local spibone fork): auto-incrementing multi-word
 * transfers framed as [cmd][addr BE][count BE16][...]. */
#define CMD_BURST_WRITE 0x02
#define CMD_BURST_READ  0x03
/* Padding bytes clocked out to capture the device response; spibone answers
 * within a couple of bytes at any sane clock, so this is generous. */
#define RESPONSE_SLACK 24
/* Burst response slack: a few extra 0xff bytes to clock out the write ack /
 * the last read word's tail. */
#define BURST_WR_SLACK 8
#define BURST_RD_SLACK 16

/* Per-transfer SPI clock override (Hz). 0 = use the DT spi-max-frequency.
 * spibone tops out at sys_clk/4 (~18.75 MHz at 75 MHz); keep below that. The
 * default 1 MHz matches the userspace tool and is safe for bring-up. */
static unsigned int spi_hz = 1000000;
module_param(spi_hz, uint, 0644);
MODULE_PARM_DESC(spi_hz, "SPI clock in Hz (0 = DT default). Must stay below spibone's sys_clk/4.");

/* Use the spibone burst commands (0x02/0x03) on the frame hot paths. Requires a
 * burst-capable bitstream (litex_soc/spi_bone.py with_burst). Set to 0 to fall
 * back to single-word transfers — necessary when the loaded FPGA gateware
 * predates burst support (every burst would otherwise time out). */
static bool use_burst = true;
module_param(use_burst, bool, 0644);
MODULE_PARM_DESC(use_burst, "Use spibone burst transfers (0 = single-word fallback for pre-burst gateware)");

static int aes67_spi_xfer(struct aes67_priv *p, const u8 *tx, u8 *rx, size_t len)
{
	struct spi_transfer t = {
		.tx_buf    = tx,
		.rx_buf    = rx,
		.len       = len,
		.speed_hz  = spi_hz,   /* 0 → controller falls back to the DT max */
	};
	struct spi_message m;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return spi_sync(p->spi, &m);
}

int aes67_wb_read_locked(struct aes67_priv *p, u32 addr, u32 *val)
{
	u8 tx[5 + RESPONSE_SLACK];
	u8 rx[5 + RESPONSE_SLACK];
	int ret, i;

	lockdep_assert_held(&p->bus_lock);

	tx[0] = CMD_READ;
	put_unaligned_be32(addr, &tx[1]);
	memset(&tx[5], 0xff, RESPONSE_SLACK);

	ret = aes67_spi_xfer(p, tx, rx, sizeof(tx));
	if (ret)
		return ret;

	/* Scan past the address echo for the sync byte, skipping the 0xff the
	 * device drives while the read is in flight. */
	for (i = 5; i + 4 < (int)sizeof(rx); i++) {
		if (rx[i] == CMD_READ) {
			*val = get_unaligned_be32(&rx[i + 1]);
			return 0;
		}
		if (rx[i] != 0xff)
			return -EIO;
	}
	return -ETIMEDOUT;
}

int aes67_wb_write_locked(struct aes67_priv *p, u32 addr, u32 val)
{
	u8 tx[9 + RESPONSE_SLACK];
	u8 rx[9 + RESPONSE_SLACK];
	int ret, i;

	lockdep_assert_held(&p->bus_lock);

	tx[0] = CMD_WRITE;
	put_unaligned_be32(addr, &tx[1]);
	put_unaligned_be32(val, &tx[5]);
	memset(&tx[9], 0xff, RESPONSE_SLACK);

	ret = aes67_spi_xfer(p, tx, rx, sizeof(tx));
	if (ret)
		return ret;

	for (i = 9; i < (int)sizeof(rx); i++) {
		if (rx[i] == CMD_WRITE)
			return 0;
		if (rx[i] != 0xff)
			return -EIO;
	}
	return -ETIMEDOUT;
}

/*
 * Burst write: store `n` bytes as `n` consecutive 32-bit words starting at word
 * address `addr` (eth_buf packs one byte per word, in the low byte). The device
 * commits each word as it streams in (auto-incrementing the address) and clocks
 * out a single 0x00 ack at the end. One SPI transfer replaces `n` single-word
 * round-trips; we chunk to the per-device scratch buffer.
 */
int aes67_wb_write_burst_locked(struct aes67_priv *p, u32 addr,
				const u8 *bytes, unsigned int n)
{
	lockdep_assert_held(&p->bus_lock);

	/* Fallback for gateware without burst support: one word per byte. */
	if (!use_burst) {
		unsigned int i;
		int ret;

		for (i = 0; i < n; i++) {
			ret = aes67_wb_write_locked(p, addr + 4 * i, bytes[i]);
			if (ret)
				return ret;
		}
		return 0;
	}

	while (n) {
		unsigned int chunk = min(n, AES67_BURST_WR_CHUNK);
		u8 *tx = p->spi_tx;
		u8 *rx = p->spi_rx;
		unsigned int len, i;
		int ret;

		tx[0] = CMD_BURST_WRITE;
		put_unaligned_be32(addr, &tx[1]);
		put_unaligned_be16((u16)chunk, &tx[5]);
		for (i = 0; i < chunk; i++) {
			tx[7 + 4 * i + 0] = 0;
			tx[7 + 4 * i + 1] = 0;
			tx[7 + 4 * i + 2] = 0;
			tx[7 + 4 * i + 3] = bytes[i];
		}
		len = 7 + 4 * chunk;
		memset(&tx[len], 0xff, BURST_WR_SLACK);
		len += BURST_WR_SLACK;

		ret = aes67_spi_xfer(p, tx, rx, len);
		if (ret)
			return ret;

		/* Confirm the device clocked out its 0x00 completion ack. */
		for (i = 7 + 4 * chunk; i < len; i++) {
			if (rx[i] == CMD_WRITE)   /* 0x00 ack */
				break;
			if (rx[i] != 0xff)
				return -EIO;
		}
		if (i == len)
			return -ETIMEDOUT;

		addr  += 4 * chunk;
		bytes += chunk;
		n     -= chunk;
	}
	return 0;
}

/*
 * Burst read: fetch `n` consecutive 32-bit words starting at word address
 * `addr`, returning the low byte of each (the eth_buf payload byte) into
 * `bytes`. Per word the device drives 0xff until the read completes, then a
 * 0x01 sync byte and the big-endian value; we scan for each sync exactly like a
 * single read. Chunked to the per-device scratch buffer.
 */
int aes67_wb_read_burst_locked(struct aes67_priv *p, u32 addr,
			       u8 *bytes, unsigned int n)
{
	lockdep_assert_held(&p->bus_lock);

	/* Fallback for gateware without burst support: one word per byte. */
	if (!use_burst) {
		unsigned int i;
		int ret;
		u32 word;

		for (i = 0; i < n; i++) {
			ret = aes67_wb_read_locked(p, addr + 4 * i, &word);
			if (ret)
				return ret;
			bytes[i] = word & 0xff;
		}
		return 0;
	}

	while (n) {
		unsigned int chunk = min(n, AES67_BURST_RD_CHUNK);
		u8 *tx = p->spi_tx;
		u8 *rx = p->spi_rx;
		unsigned int len, i, w;
		int ret;

		tx[0] = CMD_BURST_READ;
		put_unaligned_be32(addr, &tx[1]);
		put_unaligned_be16((u16)chunk, &tx[5]);
		/* 7-byte header echo + up to 7 bytes/word ([pad][sync][4 data]) +
		 * slack; clock 0xff across the whole response window. */
		len = 7 + 7 * chunk + BURST_RD_SLACK;
		memset(&tx[7], 0xff, len - 7);

		ret = aes67_spi_xfer(p, tx, rx, len);
		if (ret)
			return ret;

		i = 7;
		for (w = 0; w < chunk; w++) {
			/* Skip the 0xff the device drives during read latency. */
			while (i < len && rx[i] == 0xff)
				i++;
			if (i + 5 > len || rx[i] != CMD_READ)  /* 0x01 + 4 data */
				return -ETIMEDOUT;
			bytes[w] = rx[i + 4];   /* low byte of the BE32 word */
			i += 5;
		}

		addr  += 4 * chunk;
		bytes += chunk;
		n     -= chunk;
	}
	return 0;
}

int aes67_wb_read(struct aes67_priv *p, u32 addr, u32 *val)
{
	int ret;

	mutex_lock(&p->bus_lock);
	ret = aes67_wb_read_locked(p, addr, val);
	mutex_unlock(&p->bus_lock);
	return ret;
}

int aes67_wb_write(struct aes67_priv *p, u32 addr, u32 val)
{
	int ret;

	mutex_lock(&p->bus_lock);
	ret = aes67_wb_write_locked(p, addr, val);
	mutex_unlock(&p->bus_lock);
	return ret;
}

/* --- /dev/aes67ctl: raw peek/poke for userspace (aes67d / aes67cfg) -------- */

static struct aes67_priv *ctl_to_priv(struct file *f)
{
	struct miscdevice *m = f->private_data;

	return container_of(m, struct aes67_priv, ctl_dev);
}

static long aes67_ctl_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
	struct aes67_priv *p = ctl_to_priv(f);
	struct aes67_wb_xfer x;
	void __user *uarg = (void __user *)arg;
	int ret;

	switch (cmd) {
	case AES67_IOC_PEEK:
		if (copy_from_user(&x, uarg, sizeof(x)))
			return -EFAULT;
		ret = aes67_wb_read(p, x.addr, &x.val);
		if (ret)
			return ret;
		if (copy_to_user(uarg, &x, sizeof(x)))
			return -EFAULT;
		return 0;
	case AES67_IOC_POKE:
		if (copy_from_user(&x, uarg, sizeof(x)))
			return -EFAULT;
		return aes67_wb_write(p, x.addr, x.val);
	default:
		return -ENOTTY;
	}
}

static const struct file_operations aes67_ctl_fops = {
	.owner          = THIS_MODULE,
	.unlocked_ioctl = aes67_ctl_ioctl,
	.compat_ioctl   = compat_ptr_ioctl,
};

int aes67_ctl_register(struct aes67_priv *p)
{
	p->ctl_dev.minor = MISC_DYNAMIC_MINOR;
	p->ctl_dev.name  = "aes67ctl";
	p->ctl_dev.fops  = &aes67_ctl_fops;
	return misc_register(&p->ctl_dev);
}

void aes67_ctl_unregister(struct aes67_priv *p)
{
	misc_deregister(&p->ctl_dev);
}
