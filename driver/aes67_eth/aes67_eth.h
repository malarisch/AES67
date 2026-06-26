/* SPDX-License-Identifier: GPL-2.0 */
/*
 * AES67 FPGA Ethernet + PHC driver — shared definitions.
 *
 * The FPGA (CPU-less aes67_bridge target, built with --ptp-in-software) hangs
 * off an SPI link via the LiteX spibone Wishbone bridge. This driver is the sole
 * Wishbone master: it carries the eth_buf control-plane datapath as a netdev,
 * exposes the FPGA wallclock as a PHC, and hardware-timestamps PTP frames so
 * stock ptp4l can discipline the clock. Config traffic from userspace rides the
 * same bus through /dev/aes67ctl (see aes67_uapi.h).
 */
#ifndef AES67_ETH_H
#define AES67_ETH_H

#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/netdevice.h>
#include <linux/ptp_clock_kernel.h>
#include <linux/skbuff.h>
#include <linux/spi/spi.h>
#include <linux/workqueue.h>

#include "aes67_regs.h"

/* --- Gateware register-bit contract (see FPGA/aes67_csr.py) --------------- */
#define AES67_CTRL_ETH_TX_REQUEST  BIT(3)   /* aes67_csr_ctrl: pulse to send TX */
#define AES67_STATUS_ETH_LINK_UP   BIT(4)   /* aes67_csr_status: PHY link up    */
#define AES67_STATUS_ETH_TX_DONE   BIT(7)   /* aes67_csr_status: last TX done   */
#define AES67_EV_RX_READY          BIT(0)   /* eth_buf EventManager: RX ready   */
#define AES67_WC_CTRL_SET          BIT(0)   /* wallclock_ctrl: load out regs    */
#define AES67_WC_CTRL_PHASEJUMP    BIT(1)   /* wallclock_ctrl: add signed delta */

#define AES67_TX_REGION_OFFSET     0x2000u  /* TX buffer base within eth_buf    */

/* RX framing. The MAC includes the 4-byte FCS in the buffer; the FPGA appends a
 * 5-byte hardware-timestamp trailer after it (1 byte seconds[3:0], then 4 bytes
 * little-endian nanoseconds[29:0]). rx_len counts payload + FCS + trailer.
 * NOTE: this is the contract the FPGA RX FSM must honour (see plan / the
 * litex_eth_buffer_bridge.vhd RX_WRITE_* states). */
#define AES67_FCS_LEN              4
#define AES67_RX_TS_TRAILER_LEN    5
#define AES67_WC_SEC_CAP_BITS      4        /* captured seconds field width     */

#define AES67_MAX_FRAME            1518

/* adjfine: the wallclock_ppb output is signed 20-bit (parts-per-billion). */
#define AES67_MAX_PPB              ((1 << 19) - 1)   /* +/- 524287 ppb */

/* --- SPI burst transfers (spibone burst read/write) ----------------------- *
 * The frame RX/TX hot paths move one eth_buf byte per 32-bit word. spibone's
 * burst commands (0x02 write / 0x03 read) stream many words in a single SPI
 * transfer, auto-incrementing the word address in gateware, instead of one
 * spi_sync round-trip per word. A per-device DMA-safe scratch pair holds one
 * chunk; bus_lock serialises their use. The chunk sizes keep every transfer
 * within AES67_BURST_BUF including framing + response slack:
 *   write: 7 (cmd+addr+count) + 4*N (data) + 8  (ack slack)
 *   read:  7 (header echo)    + 7*N (<=pad+sync+4 data) + 16 (slack) */
#define AES67_BURST_BUF        2048u
#define AES67_BURST_WR_CHUNK   256u   /* 7 + 4*256 + 8  = 1039 B */
#define AES67_BURST_RD_CHUNK   240u   /* 7 + 7*240 + 16 = 1703 B */

/* --- Driver private state ------------------------------------------------- */
struct aes67_priv {
	struct spi_device   *spi;
	struct net_device   *netdev;

	/* Serialises every Wishbone access: netdev RX/TX bursts, PHC ops, and
	 * the /dev/aes67ctl peek/poke. The whole bus has a single owner. */
	struct mutex         bus_lock;

	/* All bus I/O runs in process context (SPI sleeps): an ordered wq drains
	 * the software TX queue and the poll fallback; a threaded IRQ drains RX. */
	struct workqueue_struct *wq;
	struct sk_buff_head      txq;
	struct work_struct       tx_work;
	struct delayed_work      poll_work;
	int                      irq;

	/* Scratch buffer for one RX frame (payload + FCS + timestamp trailer). */
	u8 rx_buf[AES67_MAX_FRAME + AES67_FCS_LEN + AES67_RX_TS_TRAILER_LEN];

	/* DMA-safe scratch for SPI burst transfers (devm_kmalloc'd, not embedded
	 * here so they never land in vmalloc'd netdev priv memory). */
	u8 *spi_tx;
	u8 *spi_rx;

	/* PHC */
	struct ptp_clock      *ptp_clock;
	struct ptp_clock_info  ptp_info;

	/* HW timestamping enables (set via SIOCSHWTSTAMP). The FPGA keeps only
	 * the *last* TX timestamp, so timestamped TX is serialised by the wq. */
	spinlock_t            tx_ts_lock;
	bool                  hwts_tx_on;
	bool                  hwts_rx_on;

	/* Control char device (/dev/aes67ctl). */
	struct miscdevice     ctl_dev;
};

/* --- Bus layer (aes67_bus.c) ---------------------------------------------- *
 * The __ variants assume bus_lock is held (for multi-word sequences); the
 * plain variants take it for a single transaction. */
int  aes67_wb_read_locked(struct aes67_priv *p, u32 addr, u32 *val);
int  aes67_wb_write_locked(struct aes67_priv *p, u32 addr, u32 val);
int  aes67_wb_read(struct aes67_priv *p, u32 addr, u32 *val);
int  aes67_wb_write(struct aes67_priv *p, u32 addr, u32 val);

/* Burst variants for the frame hot paths (bus_lock held). Each moves `n`
 * consecutive 32-bit words (eth_buf stores one byte per word) in a single SPI
 * transfer via the spibone burst commands, chunked to the per-device scratch. */
int  aes67_wb_write_burst_locked(struct aes67_priv *p, u32 addr,
				 const u8 *bytes, unsigned int n);
int  aes67_wb_read_burst_locked(struct aes67_priv *p, u32 addr,
				u8 *bytes, unsigned int n);

int  aes67_ctl_register(struct aes67_priv *p);
void aes67_ctl_unregister(struct aes67_priv *p);

/* --- PHC (aes67_phc.c) ---------------------------------------------------- */
int  aes67_phc_register(struct aes67_priv *p);
void aes67_phc_unregister(struct aes67_priv *p);
/* Reconstruct a full 64-bit ns timestamp from a captured (4-bit sec, 30-bit ns)
 * pair by reading the live wallclock seconds. Used by RX and TX timestamping.
 * Takes bus_lock internally. */
int  aes67_ts_reconstruct(struct aes67_priv *p, u8 cap_sec, u32 cap_nsec,
			  u64 *ns_out);

#endif /* AES67_ETH_H */
