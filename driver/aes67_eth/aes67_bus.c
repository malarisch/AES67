// SPDX-License-Identifier: GPL-2.0
/*
 * AES67 register access on the LiteX SPIBone bus (driver/spibone).
 *
 * The spibone bus driver owns the SPI link and the Wishbone protocol; this layer
 * only adds what is AES67 specific:
 *   - the window offset: aes67_regs.h holds the addresses of the aes67_bridge map
 *     (window at AES67_WIN_BASE); a core whose window sits elsewhere on the bus
 *     (reg of its DT node, e.g. the second core of OpenAudioFPGA at 0x90020000)
 *     is reached by adding the difference
 *   - the eth_buf layouts on top of the word bursts: one payload byte per word
 *     (legacy gateware) or four bytes per word in little-endian lanes (packed)
 */
#include <linux/kernel.h>

#include "aes67_eth.h"

static inline u32 aes67_addr(struct aes67_priv *p, u32 addr)
{
	return addr + p->base_off;
}

int aes67_wb_read_locked(struct aes67_priv *p, u32 addr, u32 *val)
{
	return spibone_read_locked(p->sb, aes67_addr(p, addr), val);
}

int aes67_wb_write_locked(struct aes67_priv *p, u32 addr, u32 val)
{
	return spibone_write_locked(p->sb, aes67_addr(p, addr), val);
}

int aes67_wb_read(struct aes67_priv *p, u32 addr, u32 *val)
{
	return spibone_read(p->sb, aes67_addr(p, addr), val);
}

int aes67_wb_write(struct aes67_priv *p, u32 addr, u32 val)
{
	return spibone_write(p->sb, aes67_addr(p, addr), val);
}

/* one payload byte per word, in the low byte */
int aes67_wb_write_burst_locked(struct aes67_priv *p, u32 addr,
				const u8 *bytes, unsigned int n)
{
	unsigned int i;

	if (n > AES67_WBUF_WORDS)
		return -EINVAL;
	for (i = 0; i < n; i++)
		p->wbuf[i] = bytes[i];
	return spibone_burst_write_locked(p->sb, aes67_addr(p, addr), p->wbuf, n);
}

int aes67_wb_read_burst_locked(struct aes67_priv *p, u32 addr,
			       u8 *bytes, unsigned int n)
{
	unsigned int i;
	int ret;

	if (n > AES67_WBUF_WORDS)
		return -EINVAL;
	ret = spibone_burst_read_locked(p->sb, aes67_addr(p, addr), p->wbuf, n);
	if (ret)
		return ret;
	for (i = 0; i < n; i++)
		bytes[i] = p->wbuf[i] & 0xff;
	return 0;
}

/* four payload bytes per word, little-endian lanes (byte 4i+k in bits 8k+7..8k);
 * a final partial word is zero-padded on write, its surplus lanes are dropped on read */
int aes67_wb_write_burst_packed_locked(struct aes67_priv *p, u32 addr,
				       const u8 *bytes, unsigned int n)
{
	unsigned int words = DIV_ROUND_UP(n, 4u);
	unsigned int w, k;

	if (words > AES67_WBUF_WORDS)
		return -EINVAL;
	for (w = 0; w < words; w++) {
		u32 word = 0;

		for (k = 0; k < 4 && 4 * w + k < n; k++)
			word |= (u32)bytes[4 * w + k] << (8 * k);
		p->wbuf[w] = word;
	}
	return spibone_burst_write_locked(p->sb, aes67_addr(p, addr), p->wbuf, words);
}

int aes67_wb_read_burst_packed_locked(struct aes67_priv *p, u32 addr,
				      u8 *bytes, unsigned int n)
{
	unsigned int words = DIV_ROUND_UP(n, 4u);
	unsigned int w, k;
	int ret;

	if (words > AES67_WBUF_WORDS)
		return -EINVAL;
	ret = spibone_burst_read_locked(p->sb, aes67_addr(p, addr), p->wbuf, words);
	if (ret)
		return ret;
	for (w = 0; w < words; w++)
		for (k = 0; k < 4 && 4 * w + k < n; k++)
			bytes[4 * w + k] = (u8)(p->wbuf[w] >> (8 * k));
	return 0;
}
