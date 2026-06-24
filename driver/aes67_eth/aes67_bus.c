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
 */
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/unaligned.h>

#include "aes67_eth.h"
#include "aes67_uapi.h"

#define CMD_WRITE 0x00
#define CMD_READ  0x01
/* Padding bytes clocked out to capture the device response; spibone answers
 * within a couple of bytes at any sane clock, so this is generous. */
#define RESPONSE_SLACK 24

/* Per-transfer SPI clock override (Hz). 0 = use the DT spi-max-frequency.
 * spibone tops out at sys_clk/4 (~18.75 MHz at 75 MHz); keep below that. The
 * default 1 MHz matches the userspace tool and is safe for bring-up. */
static unsigned int spi_hz = 1000000;
module_param(spi_hz, uint, 0644);
MODULE_PARM_DESC(spi_hz, "SPI clock in Hz (0 = DT default). Must stay below spibone's sys_clk/4.");

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
