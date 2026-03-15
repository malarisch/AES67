/*
 * LiteX SoC Ethernet driver for AES67 FPGA
 *
 * Accesses the FPGA via:
 *   - AES67_CSR registers (control/status)
 *   - ETH_BUF CSR registers (EventManager + rx_len/ack/tx_len)
 *   - ETH_BUF memory (Wishbone-mapped RX/TX packet buffers)
 *   - IRQ (eth_buf EventManager rx_ready)
 *
 * All addresses are imported from LiteX-generated headers (csr.h, mem.h, soc.h).
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/spinlock.h>
#include <string.h>

#include "eth_litex.h"

LOG_MODULE_REGISTER(eth_litex, LOG_LEVEL_DBG);

/* ---- Driver data structures ---- */

struct eth_litex_config {
	uint32_t irq_num;
};

struct eth_litex_data {
	struct net_if *iface;
	const struct device *dev;
	uint8_t mac_addr[6];

	struct k_thread rx_thread;
	struct k_thread tx_thread;
	struct k_work_delayable link_work;

	struct k_spinlock ctrl_lock;   /* Protects AES67_CSR_CTRL read-modify-write */
	struct k_sem rx_sem;           /* Signaled by ISR on RX packet */

	uint8_t rx_buf[ETH_LITEX_MAX_PKT_SIZE];
	uint8_t tx_buf[ETH_LITEX_MAX_PKT_SIZE];

	K_KERNEL_STACK_MEMBER(rx_stack, CONFIG_ETH_LITEX_RX_STACK_SIZE);
	K_KERNEL_STACK_MEMBER(tx_stack, CONFIG_ETH_LITEX_RX_STACK_SIZE);
};

K_MSGQ_DEFINE(litex_tx_queue, sizeof(struct net_pkt *),
	      CONFIG_ETH_LITEX_TX_QUEUE_DEPTH, 4);

/* Forward-declare the device so DEVICE_GET(eth_litex0) works in init */
DEVICE_DECLARE(eth_litex0);

/* ---- CSR convenience (thread-safe ctrl register) ---- */

int eth_litex_ctrl_set_bits(const struct device *dev, uint32_t bits)
{
	struct eth_litex_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->ctrl_lock);

	uint32_t val = litex_csr_read(CSR_AES67_CSR_CTRL_ADDR);
	val |= bits;
	litex_csr_write(CSR_AES67_CSR_CTRL_ADDR, val);

	k_spin_unlock(&data->ctrl_lock, key);
	return 0;
}

int eth_litex_ctrl_clear_bits(const struct device *dev, uint32_t bits)
{
	struct eth_litex_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->ctrl_lock);

	uint32_t val = litex_csr_read(CSR_AES67_CSR_CTRL_ADDR);
	val &= ~bits;
	litex_csr_write(CSR_AES67_CSR_CTRL_ADDR, val);

	k_spin_unlock(&data->ctrl_lock, key);
	return 0;
}

/* ---- Public helpers ---- */

int eth_litex_write_mac(const struct device *dev, const uint8_t mac[6])
{
	ARG_UNUSED(dev);

	uint32_t lo = (uint32_t)mac[0] |
		      ((uint32_t)mac[1] << 8) |
		      ((uint32_t)mac[2] << 16) |
		      ((uint32_t)mac[3] << 24);
	uint32_t hi = (uint32_t)mac[4] |
		      ((uint32_t)mac[5] << 8);

	litex_csr_write(CSR_AES67_CSR_MAC_ADDR_LO_ADDR, lo);
	litex_csr_write(CSR_AES67_CSR_MAC_ADDR_HI_ADDR, hi);

	return 0;
}

int eth_litex_write_ip(const struct device *dev, const struct in_addr *ip)
{
	ARG_UNUSED(dev);
	litex_csr_write(CSR_AES67_CSR_IP_ADDR_ADDR, ip->s_addr);
	return 0;
}

int eth_litex_write_ptp_config(const struct device *dev,
			       const uint8_t leader_clock_id[8],
			       uint8_t time_source,
			       int8_t log_msg_interval,
			       int8_t log_announce_interval)
{
	ARG_UNUSED(dev);

	uint32_t id_lo, id_hi;
	memcpy(&id_lo, &leader_clock_id[0], 4);
	memcpy(&id_hi, &leader_clock_id[4], 4);

	litex_csr_write(CSR_AES67_CSR_PTP_LEADER_ID_LO_ADDR, id_lo);
	litex_csr_write(CSR_AES67_CSR_PTP_LEADER_ID_HI_ADDR, id_hi);
	litex_csr_write(CSR_AES67_CSR_PTP_TIME_SOURCE_ADDR, time_source);
	litex_csr_write(CSR_AES67_CSR_PTP_LOG_MSG_INTERVAL_ADDR, (uint8_t)log_msg_interval);
	litex_csr_write(CSR_AES67_CSR_PTP_ANNOUNCE_MSG_INTERVAL_ADDR, (uint8_t)log_announce_interval);

	return 0;
}

bool eth_litex_read_ppb_counts(const struct device *dev,
			       uint32_t *wc_count, uint32_t *pll_count)
{
	ARG_UNUSED(dev);

	uint32_t status = litex_csr_read(CSR_AES67_CSR_PLL_PPB_STATUS_ADDR);
	*wc_count = litex_csr_read(CSR_AES67_CSR_PLL_PPB_WC_COUNT_ADDR) & 0x3FFFFF;
	*pll_count = litex_csr_read(CSR_AES67_CSR_PLL_PPB_PLL_COUNT_ADDR) & 0x3FFFFF;

	return (status & AES67_PPB_STATUS_VALID) != 0;
}

uint32_t eth_litex_read_path_delay(const struct device *dev)
{
	ARG_UNUSED(dev);
	return litex_csr_read(CSR_AES67_CSR_PTP_PATH_DELAY_ADDR);
}

uint32_t eth_litex_read_ptp_offset(const struct device *dev)
{
	ARG_UNUSED(dev);
	return litex_csr_read(CSR_AES67_CSR_PTP_OFFSET_ADDR);
}

uint32_t eth_litex_read_status(const struct device *dev)
{
	ARG_UNUSED(dev);
	return litex_csr_read(CSR_AES67_CSR_STATUS_ADDR);
}

/* ---- Packet buffer access ----
 *
 * The EthPacketBuffer is Wishbone-mapped with 1 byte per 32-bit word
 * (only the lower 8 bits are valid per word address).
 * RX buffer: word addresses 0x000..0x5ED (byte offsets 0..1517)
 * TX buffer: word addresses 0x800..0xDED (byte offsets 0..1517)
 *
 * From the CPU, each byte is at a 4-byte-aligned address.
 */

static inline void eth_buf_write_byte(uint16_t offset, uint8_t val)
{
	volatile uint32_t *p = (volatile uint32_t *)(ETH_BUF_TX_MEM + ((uint32_t)offset << 2));
	*p = val;
}

static inline uint8_t eth_buf_read_byte(uint16_t offset)
{
	volatile uint32_t *p = (volatile uint32_t *)(ETH_BUF_RX_MEM + ((uint32_t)offset << 2));
	return (uint8_t)(*p & 0xFF);
}

/*
 * Hand-written tight loops for packet buffer access.
 *
 * The CPU fetches instructions from HyperRAM which shares the Wishbone bus.
 * A C for-loop generates ~6 instructions per byte, each requiring a HyperRAM
 * fetch (~40+ cycles).  By using a tiny asm loop (3 instructions, fits in one
 * ICache line), the instruction fetches happen only once and the CPU spends
 * almost all its time on the actual Wishbone data transfers.
 */

static void eth_buf_read_packet(uint8_t *dst, uint16_t len)
{
	__asm__ volatile("fence" ::: "memory");

	if (len == 0) return;

	uintptr_t src_base = ETH_BUF_RX_MEM;

	/* a0 = dst pointer, a1 = src (Wishbone word addr), a2 = remaining */
	__asm__ volatile(
		"1:\n"
		"    lw   t0, 0(%[src])\n"      /* read 32-bit word from RX buffer */
		"    sb   t0, 0(%[dst])\n"       /* store low byte to dst */
		"    addi %[dst], %[dst], 1\n"
		"    addi %[src], %[src], 4\n"   /* next Wishbone word (4 bytes apart) */
		"    addi %[rem], %[rem], -1\n"
		"    bnez %[rem], 1b\n"
		: [dst] "+r"(dst), [src] "+r"(src_base), [rem] "+r"(len)
		:
		: "t0", "memory"
	);
}

static void eth_buf_write_packet(const uint8_t *src, uint16_t len)
{
	if (len == 0) return;

	uintptr_t dst_base = ETH_BUF_TX_MEM;

	__asm__ volatile(
		"1:\n"
		"    lbu  t0, 0(%[src])\n"       /* load byte from src */
		"    sw   t0, 0(%[dst])\n"       /* write 32-bit word to TX buffer */
		"    addi %[src], %[src], 1\n"
		"    addi %[dst], %[dst], 4\n"   /* next Wishbone word */
		"    addi %[rem], %[rem], -1\n"
		"    bnez %[rem], 1b\n"
		: [src] "+r"(src), [dst] "+r"(dst_base), [rem] "+r"(len)
		:
		: "t0", "memory"
	);

	__asm__ volatile("fence" ::: "memory");
}

/* ---- ISR ---- */

static volatile uint32_t isr_count;
static volatile uint32_t isr_spurious_count;

static void eth_litex_isr(const struct device *dev)
{
	struct eth_litex_data *data = dev->data;

	isr_count++;

	/* Unconditionally disable EventManager + clear pending.
	 * Skip the ev_pending READ — if the Wishbone bus hangs on
	 * CSR reads during IRQ context, this avoids it entirely.
	 * RX thread will re-enable after processing. */
	litex_csr_write(CSR_ETH_BUF_EV_ENABLE_ADDR, 0);
	litex_csr_write(CSR_ETH_BUF_EV_PENDING_ADDR, ETH_BUF_EV_RX_READY);
	k_sem_give(&data->rx_sem);
}

/* ---- TX path ---- */

static volatile uint32_t tx_count;
static volatile uint32_t tx_timeout_count;

static void eth_litex_tx_thread(void *p1, void *p2, void *p3)
{
	const struct device *dev = p1;
	struct eth_litex_data *data = dev->data;
	struct net_pkt *pkt = NULL;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	LOG_DBG("TX thread started");

	while (1) {
		k_msgq_get(&litex_tx_queue, &pkt, K_FOREVER);

		size_t len = net_pkt_get_len(pkt);
		LOG_DBG("TX pkt len=%zu", len);

		if (len > ETH_LITEX_MAX_PKT_SIZE) {
			LOG_ERR("TX too large: %zu", len);
			net_pkt_unref(pkt);
			continue;
		}

		if (net_pkt_read(pkt, data->tx_buf, len) < 0) {
			LOG_ERR("TX pkt read failed");
			net_pkt_unref(pkt);
			continue;
		}

		/* Wait for previous TX to complete (eth_tx_done status bit).
		 * On the very first call tx_done starts de-asserted (FPGA reset
		 * default), so we only wait if a transmit is actually in-flight
		 * (i.e. we previously pulsed eth_tx_request).
		 *
		 * A 1518-byte frame at 100 Mbps takes ~122µs on the wire.
		 * We use k_busy_wait() because the system tick (10ms at 100Hz)
		 * is far too coarse for this — k_usleep/k_sleep would round up
		 * to 10ms per iteration, making TX unbearably slow.
		 * We yield once before spinning so other threads get a chance. */
		static bool first_tx = true;
		if (!first_tx) {
			uint32_t status_before = litex_csr_read(CSR_AES67_CSR_STATUS_ADDR);
			LOG_DBG("TX wait tx_done, status=0x%08x", status_before);
			k_yield();
			int wait = 0;
			while (!(litex_csr_read(CSR_AES67_CSR_STATUS_ADDR) & AES67_STATUS_ETH_TX_DONE)) {
				k_busy_wait(10);
				if (++wait > 20000) { /* ~200ms timeout */
					tx_timeout_count++;
					LOG_WRN("TX done timeout (status=0x%08x, cnt=%u)",
						litex_csr_read(CSR_AES67_CSR_STATUS_ADDR),
						tx_timeout_count);
					break;
				}
			}
			if (wait > 0 && wait <= 20000) {
				LOG_DBG("TX done after %d iters (~%d us)", wait, wait * 10);
			}
		}
		first_tx = false;

		/* Write packet data to TX buffer */
		uint32_t t0 = k_cycle_get_32();
		eth_buf_write_packet(data->tx_buf, len);
		uint32_t t1 = k_cycle_get_32();
		uint32_t us = k_cyc_to_us_floor32(t1 - t0);

		/* Set TX length */
		litex_csr_write(CSR_ETH_BUF_TX_LEN_ADDR, len);

		uint32_t ctrl_before = litex_csr_read(CSR_AES67_CSR_CTRL_ADDR);
		LOG_DBG("TX fire: len=%zu write=%uus ctrl=0x%08x", len, us, ctrl_before);

		/* Assert eth_tx_request and hold long enough for CDC capture.
		 * The FPGA synchronises this signal through a 2-FF chain clocked
		 * by mac_tx_clock (≥2.5 MHz → ≤400ns period).  We need the pulse
		 * to span at least 3 mac_tx_clock edges, so 2µs is safe. */
		eth_litex_ctrl_set_bits(dev, AES67_CTRL_ETH_TX_REQUEST);
		k_busy_wait(2);
		eth_litex_ctrl_clear_bits(dev, AES67_CTRL_ETH_TX_REQUEST);

		tx_count++;
		LOG_DBG("TX #%u sent (%zu bytes)", tx_count, len);

		net_pkt_unref(pkt);
	}
}

/* ---- RX path ---- */

static volatile uint32_t rx_count;
static volatile uint32_t rx_poll_wakeup_count;
static volatile uint32_t rx_irq_wakeup_count;

static void eth_litex_rx_thread(void *p1, void *p2, void *p3)
{
	const struct device *dev = p1;
	struct eth_litex_data *data = dev->data;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	LOG_DBG("RX thread started");

	while (1) {
		int sem_ret = k_sem_take(&data->rx_sem,
					 K_MSEC(CONFIG_ETH_LITEX_POLL_INTERVAL_MS));

		if (sem_ret == 0) {
			rx_irq_wakeup_count++;
		} else {
			rx_poll_wakeup_count++;
		}

		/* Check if a packet is ready */
		uint32_t ready = litex_csr_read(CSR_ETH_BUF_RX_READY_ADDR);
		if (!(ready & 0x01)) {
			/* Re-enable EventManager in case ISR disabled it but
			 * no packet was actually ready (race with ack). */
			litex_csr_write(CSR_ETH_BUF_EV_ENABLE_ADDR, ETH_BUF_EV_RX_READY);
			/* Periodic debug dump every ~5s (poll interval based) */
			if ((rx_poll_wakeup_count % 50) == 0) {
				uint32_t status = litex_csr_read(CSR_AES67_CSR_STATUS_ADDR);
				uint32_t ev_pend = litex_csr_read(CSR_ETH_BUF_EV_PENDING_ADDR);
				uint32_t ev_en = litex_csr_read(CSR_ETH_BUF_EV_ENABLE_ADDR);
				LOG_DBG("RX idle: status=0x%08x ev_pend=0x%x ev_en=0x%x "
					"isr=%u(spur=%u) rx=%u tx=%u(tmo=%u) "
					"irq_wake=%u poll_wake=%u",
					status, ev_pend, ev_en,
					isr_count, isr_spurious_count,
					rx_count, tx_count, tx_timeout_count,
					rx_irq_wakeup_count, rx_poll_wakeup_count);
			}
			continue;
		}

		uint16_t pkt_len = litex_csr_read(CSR_ETH_BUF_RX_LEN_ADDR) & 0xFFFF;

		/* The MAC includes the 4-byte FCS in the frame.  Strip it
		 * before handing to the network stack which expects frames
		 * without FCS. */
		if (pkt_len >= 4) {
			pkt_len -= 4;
		}

		LOG_DBG("RX ready=0x%x len=%u (wakeup=%s)",
			ready, pkt_len,
			(sem_ret == 0) ? "irq" : "poll");

		if (pkt_len < 14 || pkt_len > ETH_LITEX_MAX_PKT_SIZE) {
			LOG_WRN("RX invalid len %u (ready=0x%x)", pkt_len, ready);
			litex_csr_write(CSR_ETH_BUF_RX_ACK_ADDR, 1);
			litex_csr_write(CSR_ETH_BUF_RX_ACK_ADDR, 0);
			litex_csr_write(CSR_ETH_BUF_EV_ENABLE_ADDR, ETH_BUF_EV_RX_READY);
			continue;
		}

		/* Read packet data from RX buffer */
		uint32_t rt0 = k_cycle_get_32();
		eth_buf_read_packet(data->rx_buf, pkt_len);
		uint32_t rt1 = k_cycle_get_32();

		LOG_DBG("RX #%u: %u bytes (read=%uus) [%02x:%02x:%02x:%02x:%02x:%02x -> "
			"%02x:%02x:%02x:%02x:%02x:%02x type=%02x%02x]",
			rx_count + 1, pkt_len,
			k_cyc_to_us_floor32(rt1 - rt0),
			data->rx_buf[6], data->rx_buf[7], data->rx_buf[8],
			data->rx_buf[9], data->rx_buf[10], data->rx_buf[11],
			data->rx_buf[0], data->rx_buf[1], data->rx_buf[2],
			data->rx_buf[3], data->rx_buf[4], data->rx_buf[5],
			data->rx_buf[12], data->rx_buf[13]);

		/* Hex dump first 34 bytes (ETH hdr + IP hdr) for debugging */
		if (pkt_len >= 34) {
			LOG_HEXDUMP_DBG(data->rx_buf, 34, "RX raw");
		}

		/* ACK: release the RX buffer for the next packet */
		litex_csr_write(CSR_ETH_BUF_RX_ACK_ADDR, 1);
		litex_csr_write(CSR_ETH_BUF_RX_ACK_ADDR, 0);

		/* Re-enable EventManager IRQ (ISR disables it to prevent
		 * re-entry while the pending clear propagates). */
		litex_csr_write(CSR_ETH_BUF_EV_ENABLE_ADDR, ETH_BUF_EV_RX_READY);

		rx_count++;

		/* Allocate net_pkt and deliver to stack */
		struct net_pkt *pkt = net_pkt_rx_alloc_with_buffer(
			data->iface, pkt_len, AF_UNSPEC, 0, K_NO_WAIT);
		if (!pkt) {
			LOG_ERR("RX alloc failed (pkt #%u)", rx_count);
			continue;
		}

		if (net_pkt_write(pkt, data->rx_buf, pkt_len)) {
			LOG_ERR("RX write failed (pkt #%u)", rx_count);
			net_pkt_unref(pkt);
			continue;
		}

		int ret = net_recv_data(data->iface, pkt);
		if (ret < 0) {
			LOG_ERR("RX deliver err %d (pkt #%u)", ret, rx_count);
			net_pkt_unref(pkt);
		}
	}
}

/* ---- Zephyr ethernet API ---- */

static int eth_litex_send(const struct device *dev, struct net_pkt *pkt)
{
	ARG_UNUSED(dev);

	/* Must be non-blocking: Zephyr L2 send() is called from the
	 * network processing context — blocking here deadlocks TCP. */
	if (k_msgq_put(&litex_tx_queue, &pkt, K_NO_WAIT) != 0) {
		LOG_WRN("TX queue full, dropping pkt (%zu bytes)", net_pkt_get_len(pkt));
		return -ENOMEM;
	}
	net_pkt_ref(pkt);
	return 0;
}

static void eth_litex_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct eth_litex_data *data = dev->data;

	LOG_DBG("iface_init called, dev=%p iface=%p", dev, iface);

	data->iface = iface;
	ethernet_init(iface);

	/* Static locally administered MAC */
	data->mac_addr[0] = 0x02;
	data->mac_addr[1] = 0x1C;
	data->mac_addr[2] = 0x23;
	data->mac_addr[3] = 0x17;
	data->mac_addr[4] = 0x4A;
	data->mac_addr[5] = 0xCC;

	net_if_set_link_addr(iface, data->mac_addr, sizeof(data->mac_addr),
			     NET_LINK_ETHERNET);

	/* Write MAC to FPGA so hardware knows our address */
	eth_litex_write_mac(dev, data->mac_addr);

	LOG_INF("MAC %02x:%02x:%02x:%02x:%02x:%02x",
		data->mac_addr[0], data->mac_addr[1], data->mac_addr[2],
		data->mac_addr[3], data->mac_addr[4], data->mac_addr[5]);

	LOG_DBG("iface_init complete, scheduling link check");
}

static enum ethernet_hw_caps eth_litex_get_capabilities(const struct device *dev)
{
	ARG_UNUSED(dev);
	return ETHERNET_LINK_100BASE;
}

static void eth_litex_link_work(struct k_work *work)
{
	struct k_work_delayable *dwork = CONTAINER_OF(work, struct k_work_delayable, work);
	struct eth_litex_data *data = CONTAINER_OF(dwork, struct eth_litex_data, link_work);

	if (data->iface) {
		uint32_t status = litex_csr_read(CSR_AES67_CSR_STATUS_ADDR);
		LOG_DBG("Link check: status=0x%08x link_up=%u",
			status, !!(status & AES67_STATUS_ETH_LINK_UP));
		if (status & AES67_STATUS_ETH_LINK_UP) {
			net_if_carrier_on(data->iface);
			net_if_up(data->iface);
			LOG_INF("Ethernet link up (status=0x%08x)", status);
		} else {
			k_work_schedule(dwork, K_MSEC(500));
		}
	}
}

static const struct ethernet_api eth_litex_api = {
	.iface_api.init = eth_litex_iface_init,
	.get_capabilities = eth_litex_get_capabilities,
	.send = eth_litex_send,
};

/* ---- Initialization ---- */

static int eth_litex_init(const struct device *dev)
{
	struct eth_litex_data *data = dev->data;
	const struct eth_litex_config *cfg = dev->config;

	data->dev = dev;

	LOG_INF("Initializing LiteX Ethernet driver (IRQ %u)", cfg->irq_num);

	/* Dump all relevant CSR addresses for verification */
	LOG_DBG("CSR addrs: ctrl=0x%lx status=0x%lx",
		(unsigned long)CSR_AES67_CSR_CTRL_ADDR,
		(unsigned long)CSR_AES67_CSR_STATUS_ADDR);
	LOG_DBG("CSR addrs: ev_pending=0x%lx ev_enable=0x%lx ev_status=0x%lx",
		(unsigned long)CSR_ETH_BUF_EV_PENDING_ADDR,
		(unsigned long)CSR_ETH_BUF_EV_ENABLE_ADDR,
		(unsigned long)CSR_ETH_BUF_EV_STATUS_ADDR);
	LOG_DBG("CSR addrs: rx_ready=0x%lx rx_len=0x%lx rx_ack=0x%lx tx_len=0x%lx",
		(unsigned long)CSR_ETH_BUF_RX_READY_ADDR,
		(unsigned long)CSR_ETH_BUF_RX_LEN_ADDR,
		(unsigned long)CSR_ETH_BUF_RX_ACK_ADDR,
		(unsigned long)CSR_ETH_BUF_TX_LEN_ADDR);
	LOG_DBG("MEM addrs: rx_mem=0x%lx tx_mem=0x%lx",
		(unsigned long)ETH_BUF_RX_MEM,
		(unsigned long)ETH_BUF_TX_MEM);

	/* Read initial state of all CSRs */
	LOG_DBG("Initial CSR state:");
	LOG_DBG("  ctrl     = 0x%08x", litex_csr_read(CSR_AES67_CSR_CTRL_ADDR));
	LOG_DBG("  status   = 0x%08x", litex_csr_read(CSR_AES67_CSR_STATUS_ADDR));
	LOG_DBG("  rx_ready = 0x%08x", litex_csr_read(CSR_ETH_BUF_RX_READY_ADDR));
	LOG_DBG("  rx_len   = 0x%08x", litex_csr_read(CSR_ETH_BUF_RX_LEN_ADDR));
	LOG_DBG("  ev_pend  = 0x%08x", litex_csr_read(CSR_ETH_BUF_EV_PENDING_ADDR));
	LOG_DBG("  ev_en    = 0x%08x", litex_csr_read(CSR_ETH_BUF_EV_ENABLE_ADDR));
	LOG_DBG("  ev_stat  = 0x%08x", litex_csr_read(CSR_ETH_BUF_EV_STATUS_ADDR));

	k_sem_init(&data->rx_sem, 0, 1);

	/* Clear any stale pending events before enabling IRQ */
	LOG_DBG("Clearing pending events...");
	litex_csr_write(CSR_ETH_BUF_EV_PENDING_ADDR, ETH_BUF_EV_RX_READY);

	/* Connect ISR BEFORE enabling EventManager or CPU IRQ */
	LOG_DBG("Connecting IRQ %u...", (unsigned)ETH_BUF_INTERRUPT);
	IRQ_CONNECT(ETH_BUF_INTERRUPT, 0, eth_litex_isr,
		    DEVICE_GET(eth_litex0), 0);

	/* Enable EventManager — from here, pending can set on new packets,
	 * but CPU IRQ mask is not yet set so no trap yet. */
	litex_csr_write(CSR_ETH_BUF_EV_ENABLE_ADDR, ETH_BUF_EV_RX_READY);
	LOG_DBG("EventManager enabled, ev_en=0x%08x",
		litex_csr_read(CSR_ETH_BUF_EV_ENABLE_ADDR));

	/* Now enable CPU IRQ — if an IRQ is already pending, the ISR
	 * will fire immediately but is safe (ISR disables ev_enable
	 * to prevent re-entry). */
	irq_enable(ETH_BUF_INTERRUPT);
	LOG_DBG("IRQ %u enabled", (unsigned)ETH_BUF_INTERRUPT);

	/* Make sure RX buffer is released */
	LOG_DBG("Releasing RX buffer (ack pulse)...");
	litex_csr_write(CSR_ETH_BUF_RX_ACK_ADDR, 1);
	litex_csr_write(CSR_ETH_BUF_RX_ACK_ADDR, 0);
	LOG_DBG("  rx_ready after ack = 0x%08x",
		litex_csr_read(CSR_ETH_BUF_RX_READY_ADDR));

	k_work_init_delayable(&data->link_work, eth_litex_link_work);

	/* Create RX thread */
	LOG_DBG("Creating RX thread (stack=%u)...", CONFIG_ETH_LITEX_RX_STACK_SIZE);
	k_thread_create(&data->rx_thread, data->rx_stack,
			CONFIG_ETH_LITEX_RX_STACK_SIZE,
			eth_litex_rx_thread, (void *)dev, NULL, NULL,
			K_PRIO_COOP(1), 0, K_NO_WAIT);
	k_thread_name_set(&data->rx_thread, "eth_litex_rx");

	/* Create TX thread */
	LOG_DBG("Creating TX thread...");
	k_thread_create(&data->tx_thread, data->tx_stack,
			CONFIG_ETH_LITEX_RX_STACK_SIZE,
			eth_litex_tx_thread, (void *)dev, NULL, NULL,
			K_PRIO_COOP(2), 0, K_NO_WAIT);
	k_thread_name_set(&data->tx_thread, "eth_litex_tx");

	/* Schedule link-up check */
	k_work_schedule(&data->link_work, K_MSEC(100));

	LOG_INF("LiteX Ethernet driver init complete");
	return 0;
}

/* ---- Device instantiation ---- */

static struct eth_litex_data eth_litex_data_0;

static const struct eth_litex_config eth_litex_cfg_0 = {
	.irq_num = ETH_BUF_INTERRUPT,
};

ETH_NET_DEVICE_INIT(eth_litex0, "eth_litex0",
		    eth_litex_init, NULL,
		    &eth_litex_data_0, &eth_litex_cfg_0,
		    CONFIG_ETH_INIT_PRIORITY, &eth_litex_api,
		    NET_ETH_MTU);
