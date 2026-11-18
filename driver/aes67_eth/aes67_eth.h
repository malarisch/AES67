/* SPDX-License-Identifier: GPL-2.0 */
/*
 * AES67 FPGA Ethernet + PHC driver — shared definitions.
 *
 * The FPGA (CPU-less aes67_bridge target) hangs off an SPI link via the LiteX
 * SPIBone Wishbone bridge, owned by the spibone bus driver (driver/spibone). This
 * driver is a client of that bus (a platform device of a child node of the
 * spibone DT node): it carries the eth_buf control-plane datapath as a netdev,
 * exposes the FPGA wallclock as a PHC, and — on gateware built with
 * PTP_IN_SOFTWARE — hardware-timestamps PTP frames so stock ptp4l can discipline
 * the clock. On hardware-PTP gateware the FPGA servo owns the wallclock and the
 * PHC is read-only (phc2sys can still follow it). The PTP mode is read from the
 * system_cfg CSRs at probe. Config traffic from userspace reaches the same bus
 * through the spibone driver (/dev/spiboneN).
 */
#ifndef AES67_ETH_H
#define AES67_ETH_H

#include <linux/netdevice.h>
#include <linux/ptp_clock_kernel.h>
#include <linux/skbuff.h>
#include <linux/workqueue.h>

#include "aes67_regs.h"
#include "spibone.h"

/* --- Gateware register-bit contract (see FPGA/aes67_csr.py) --------------- */
#define AES67_CTRL_ETH_TX_REQUEST  BIT(3)   /* aes67_csr_ctrl: pulse to send TX */
#define AES67_STATUS_ETH_LINK_UP   BIT(4)   /* aes67_csr_status: PHY link up    */
#define AES67_STATUS_ETH_TX_DONE   BIT(7)   /* aes67_csr_status: last TX done   */
#define AES67_EV_RX_READY          BIT(0)   /* eth_buf EventManager: RX ready   */
#define AES67_WC_CTRL_SET          BIT(0)   /* wallclock_ctrl: load out regs    */
#define AES67_WC_CTRL_PHASEJUMP    BIT(1)   /* wallclock_ctrl: add signed delta */
#define AES67_SYSCFG_PTP_IN_SW     BIT(0)   /* system_cfg_flags: software PTP   */
#define AES67_SYSCFG_STATIC_PTP    BIT(1)   /* system_cfg_flags: static servo   */
#define AES67_SYSCFG_METERING      BIT(2)   /* system_cfg_flags: metering live  */

#define AES67_TX_REGION_OFFSET     0x2000u  /* TX buffer base within eth_buf    */

/* RX framing. The MAC includes the 4-byte FCS in the buffer. Software-PTP
 * gateware appends a 5-byte hardware-timestamp trailer after it (1 byte
 * seconds[3:0], then 4 bytes little-endian nanoseconds[29:0]) and rx_len counts
 * payload + FCS + trailer; hardware-PTP gateware strips the trailer in
 * litex_eth_buffer_bridge.vhd, so rx_len is payload + FCS. */
#define AES67_FCS_LEN              4
#define AES67_RX_TS_TRAILER_LEN    5
#define AES67_WC_SEC_CAP_BITS      4        /* captured seconds field width     */

#define AES67_MAX_FRAME            1518

/* adjfine: the wallclock_ppb output is signed 20-bit (parts-per-billion). */
#define AES67_MAX_PPB              ((1 << 19) - 1)   /* +/- 524287 ppb */

/* aes67_regs.h holds the addresses of the aes67_bridge map, whose window starts
 * here; the reg of the DT node gives the window of this core on the bus. */
#define AES67_WIN_BASE             0x90000000u

/* Word buffer for the eth_buf bursts: one RX frame in the byte-per-word layout
 * (payload + FCS + timestamp trailer). */
#define AES67_WBUF_WORDS           (AES67_MAX_FRAME + AES67_FCS_LEN + AES67_RX_TS_TRAILER_LEN)

/* --- Driver private state ------------------------------------------------- */
struct aes67_priv {
	struct device       *dev;
	struct net_device   *netdev;

	/* The bus (spibone driver). Its lock (aes67_bus_lock) serialises every
	 * Wishbone access: netdev RX/TX bursts, PHC ops, and the other users of
	 * the bus (userspace through /dev/spiboneN, other clients). */
	struct spibone      *sb;
	/* reg of the DT node - AES67_WIN_BASE, added to every aes67_regs.h address */
	u32                  base_off;

	/* All bus I/O runs in process context (SPI sleeps): an ordered wq drains
	 * the software TX queue and the poll fallback; a threaded IRQ drains RX. */
	struct workqueue_struct *wq;
	struct sk_buff_head      txq;
	struct work_struct       tx_work;
	struct delayed_work      poll_work;
	int                      irq;

	/* Scratch buffer for one RX frame (payload + FCS + timestamp trailer). */
	u8 rx_buf[AES67_MAX_FRAME + AES67_FCS_LEN + AES67_RX_TS_TRAILER_LEN];

	/* word buffer of the eth_buf bursts (AES67_WBUF_WORDS), guarded by the bus lock */
	u32 *wbuf;

	/* PHC */
	struct ptp_clock      *ptp_clock;
	struct ptp_clock_info  ptp_info;

	/* HW timestamping enables (set via SIOCSHWTSTAMP). The FPGA keeps only
	 * the *last* TX timestamp, so timestamped TX is serialised by the wq. */
	spinlock_t            tx_ts_lock;
	bool                  hwts_tx_on;
	bool                  hwts_rx_on;

	/* Gateware build configuration (system_cfg_flags), read at probe. */
	u32                   syscfg_flags;
	/* Gateware built with PTP_IN_SOFTWARE: ptp4l disciplines the wallclock
	 * through this PHC. False: the FPGA servo owns it, the PHC is read-only. */
	bool                  ptp_sw;
	/* RX frames carry the 5-byte timestamp trailer (rx_ts param, default:
	 * follows ptp_sw). */
	bool                  rx_trailer;

	/* Software media-clock (NCO) rate loop, SW-PTP only (aes67_phc.c): the
	 * applied wallclock ppb is smoothed and written to nco_ppb_adj, the only
	 * frequency input of the audio NCO in software-PTP gateware. Guarded by
	 * the bus lock. */
	s64                   nco_acc_q16;
	s64                   nco_written_units;
	bool                  nco_written_once;

	/* eth_buf word packing, probed from the bytes_per_word CSR: true on
	 * packed gateware (4 payload bytes per word, little-endian lanes),
	 * false on legacy bitstreams (1 byte per word). */
	bool                  buf_packed;

};

/* --- Bus layer (aes67_bus.c) ---------------------------------------------- *
 * The _locked variants assume the bus lock is held (for multi-word sequences);
 * the plain variants take it for a single transaction. */
static inline void aes67_bus_lock(struct aes67_priv *p)   { spibone_lock(p->sb); }
static inline void aes67_bus_unlock(struct aes67_priv *p) { spibone_unlock(p->sb); }

int  aes67_wb_read_locked(struct aes67_priv *p, u32 addr, u32 *val);
int  aes67_wb_write_locked(struct aes67_priv *p, u32 addr, u32 val);
int  aes67_wb_read(struct aes67_priv *p, u32 addr, u32 *val);
int  aes67_wb_write(struct aes67_priv *p, u32 addr, u32 val);

/* Burst variants for the frame hot paths (bus lock held). Each moves `n`
 * consecutive 32-bit words (eth_buf stores one byte per word) with the spibone
 * burst commands; n <= AES67_WBUF_WORDS. */
int  aes67_wb_write_burst_locked(struct aes67_priv *p, u32 addr,
				 const u8 *bytes, unsigned int n);
int  aes67_wb_read_burst_locked(struct aes67_priv *p, u32 addr,
				u8 *bytes, unsigned int n);

/* Packed-layout variants (eth_buf with 4 payload bytes per word, little-
 * endian lanes). `n` is a byte count; a final partial word is zero-padded
 * on write and its surplus lanes are discarded on read (never writing more
 * than `n` bytes to the caller's buffer). */
int  aes67_wb_write_burst_packed_locked(struct aes67_priv *p, u32 addr,
					const u8 *bytes, unsigned int n);
int  aes67_wb_read_burst_packed_locked(struct aes67_priv *p, u32 addr,
				       u8 *bytes, unsigned int n);


/* --- PHC (aes67_phc.c) ---------------------------------------------------- */
/* Reset both rate registers to neutral (SW-PTP), then register the PHC. */
int  aes67_phc_register(struct aes67_priv *p);
void aes67_phc_unregister(struct aes67_priv *p);
/* Reconstruct a full 64-bit ns timestamp from a captured (4-bit sec, 30-bit ns)
 * pair by reading the live wallclock seconds. Used by RX and TX timestamping.
 * Takes the bus lock internally. */
int  aes67_ts_reconstruct(struct aes67_priv *p, u8 cap_sec, u32 cap_nsec,
			  u64 *ns_out);

#endif /* AES67_ETH_H */
