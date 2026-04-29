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

LOG_MODULE_REGISTER(eth_litex, LOG_LEVEL_INF);

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
	bool link_up;                  /* Cached link state for edge detection */

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

	/* FPGA expects mac_addr(47 downto 0) in big-endian bit order:
	 *   mac[0] → mac_addr(47..40), mac[1] → mac_addr(39..32), ...
	 * mac_addr_lo holds bits 31..0, mac_addr_hi holds bits 47..32.
	 * So mac[0..1] go into hi, mac[2..5] go into lo. */
	uint32_t hi = ((uint32_t)mac[0] << 8) |
		      (uint32_t)mac[1];
	uint32_t lo = ((uint32_t)mac[2] << 24) |
		      ((uint32_t)mac[3] << 16) |
		      ((uint32_t)mac[4] << 8) |
		      (uint32_t)mac[5];

	litex_csr_write(CSR_AES67_CSR_MAC_ADDR_LO_ADDR, lo);
	litex_csr_write(CSR_AES67_CSR_MAC_ADDR_HI_ADDR, hi);

	return 0;
}

int eth_litex_write_ip(const struct device *dev, const struct in_addr *ip)
{
	ARG_UNUSED(dev);

	/* ip->s_addr is in network byte order (big-endian).  On a
	 * little-endian CPU a direct 32-bit store would byte-swap it,
	 * putting the octets in the wrong CSR bit positions.
	 * The FPGA expects ip_addr(31 downto 24) = first octet, so we
	 * need the value in big-endian order in the CSR register. */
	const uint8_t *b = (const uint8_t *)&ip->s_addr;
	uint32_t val = ((uint32_t)b[0] << 24) |
		       ((uint32_t)b[1] << 16) |
		       ((uint32_t)b[2] << 8) |
		       (uint32_t)b[3];

	litex_csr_write(CSR_AES67_CSR_IP_ADDR_ADDR, val);
	return 0;
}

int eth_litex_write_ptp_config(const struct device *dev,
			       uint8_t time_source,
			       int8_t log_msg_interval,
			       int8_t log_announce_interval)
{
	ARG_UNUSED(dev);

	litex_csr_write(CSR_AES67_CSR_PTP_TIME_SOURCE_ADDR, time_source);
	litex_csr_write(CSR_AES67_CSR_PTP_LOG_MSG_INTERVAL_ADDR, (uint8_t)log_msg_interval);
	litex_csr_write(CSR_AES67_CSR_PTP_ANNOUNCE_MSG_INTERVAL_ADDR, (uint8_t)log_announce_interval);

	return 0;
}

bool eth_litex_read_ptp_leader_id(const struct device *dev, uint8_t leader_clock_id[8])
{
	ARG_UNUSED(dev);

	uint32_t id_lo = litex_csr_read(CSR_AES67_CSR_PTP_LEADER_ID_LO_ADDR);
	uint32_t id_hi = litex_csr_read(CSR_AES67_CSR_PTP_LEADER_ID_HI_ADDR);

	leader_clock_id[0] = (uint8_t)(id_hi >> 24);
	leader_clock_id[1] = (uint8_t)(id_hi >> 16);
	leader_clock_id[2] = (uint8_t)(id_hi >> 8);
	leader_clock_id[3] = (uint8_t)(id_hi);
	leader_clock_id[4] = (uint8_t)(id_lo >> 24);
	leader_clock_id[5] = (uint8_t)(id_lo >> 16);
	leader_clock_id[6] = (uint8_t)(id_lo >> 8);
	leader_clock_id[7] = (uint8_t)(id_lo);

	return (id_lo | id_hi) != 0;
}

int eth_litex_write_ptp_gm_quality(const struct device *dev,
				   uint8_t priority1, uint8_t priority2,
				   uint8_t clock_class, uint8_t clock_accuracy)
{
	ARG_UNUSED(dev);

	litex_csr_write(CSR_AES67_CSR_PTP_GM_PRIORITY1_ADDR, priority1);
	litex_csr_write(CSR_AES67_CSR_PTP_GM_PRIORITY2_ADDR, priority2);
	litex_csr_write(CSR_AES67_CSR_PTP_GM_CLOCK_CLASS_ADDR, clock_class);
	litex_csr_write(CSR_AES67_CSR_PTP_GM_CLOCK_ACCURACY_ADDR, clock_accuracy);

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

/* ---- Stream config RAM access ----
 *
 * The StreamConfigRAM modules are Wishbone-mapped with 1 byte per 32-bit word.
 * TX_STREAM_CFG_BASE = 0x90004000, RX_STREAM_CFG_BASE = 0x90005000.
 * Each stream occupies 32 bytes (addresses stream_id*32 .. stream_id*32+31).
 * The byte layout matches the FMC register protocol (config_ram_address_map.md).
 */

static inline void stream_cfg_write_byte(uintptr_t base, uint8_t addr, uint8_t val)
{
	volatile uint32_t *p = (volatile uint32_t *)(base + ((uint32_t)addr << 2));
	*p = val;
}

int eth_litex_write_tx_stream_config(const struct device *dev,
				     uint8_t stream_id,
				     const struct in_addr *dst_ip,
				     uint8_t channel_count,
				     uint8_t samples_per_pkt,
				     const uint8_t *ch_ids,
				     uint8_t num_ch_ids,
				     uint32_t ssrc)
{
	ARG_UNUSED(dev);

	if (stream_id > 7) {
		return -EINVAL;
	}

	uint8_t buf[20];
	const uint8_t *ip = (const uint8_t *)&dst_ip->s_addr;

	memset(buf, 0, sizeof(buf));

	buf[0]  = stream_id & 0x07;
	buf[1]  = ip[0];
	buf[2]  = ip[1];
	buf[3]  = ip[2];
	buf[4]  = ip[3];
	buf[5]  = channel_count;
	buf[6]  = samples_per_pkt;

	for (uint8_t i = 0; i < num_ch_ids && i < 8; i++) {
		buf[7 + i] = ch_ids[i];
	}

	/* Byte 15 reserved, bytes 16-19: SSRC (big-endian) */
	buf[16] = (ssrc >> 24) & 0xFF;
	buf[17] = (ssrc >> 16) & 0xFF;
	buf[18] = (ssrc >> 8) & 0xFF;
	buf[19] = ssrc & 0xFF;

	/* Write all 20 bytes to the TX stream config RAM.
	 * The FPGA-side StreamConfigRAM computes the actual RAM address
	 * from stream_id (byte 0), so we write to Wishbone offset 0..19. */
	uint8_t base_addr = stream_id * 32;
	for (uint8_t i = 0; i < 20; i++) {
		stream_cfg_write_byte(TX_STREAM_CFG_BASE, base_addr + i, buf[i]);
	}

	LOG_DBG("TX stream %u configured: ip=%u.%u.%u.%u ch=%u spp=%u ssrc=0x%08x",
		stream_id, ip[0], ip[1], ip[2], ip[3],
		channel_count, samples_per_pkt, ssrc);

	return 0;
}

int eth_litex_write_rx_stream_config(const struct device *dev,
				     uint8_t stream_id,
				     const struct in_addr *dst_ip,
				     uint16_t dst_port,
				     const uint8_t *ch_map,
				     uint8_t channel_count,
				     uint8_t output_delay,
				     uint8_t samples_per_channel)
{
	ARG_UNUSED(dev);

	/* The Wishbone StreamConfigRAM writes directly into the rx_ringbuffer's
	 * stream_ram — no base-address indirection like the FMC protocol (0x59).
	 * Layout in stream_ram (17 bytes at stream_id * 32):
	 *   0..3:  dest IP (big-endian)
	 *   4..5:  dest UDP port (big-endian)
	 *   6..13: channel output map
	 *   14:    channel count
	 *   15:    output delay
	 *   16:    samples per channel per packet
	 */
	uint8_t buf[17];
	const uint8_t *ip = (const uint8_t *)&dst_ip->s_addr;

	memset(buf, 0, sizeof(buf));

	/* Bytes 0..3: destination IP address (network byte order) */
	buf[0] = ip[0];
	buf[1] = ip[1];
	buf[2] = ip[2];
	buf[3] = ip[3];
	/* Bytes 4..5: destination UDP port (big-endian) */
	buf[4] = (dst_port >> 8) & 0xFF;
	buf[5] =  dst_port       & 0xFF;
	/* Bytes 6..13: output channel map */
	for (uint8_t i = 0; i < 8 && i < channel_count; i++) {
		buf[6 + i] = ch_map[i];
	}
	/* Byte 14: channel count */
	buf[14] = channel_count;
	/* Byte 15: output delay in samples */
	buf[15] = output_delay;
	/* Byte 16: samples per channel per packet */
	buf[16] = samples_per_channel;

	/* Write 17 config bytes directly to the stream config RAM */
	uint8_t base_addr = stream_id * 32;
	for (uint8_t i = 0; i < 17; i++) {
		stream_cfg_write_byte(RX_STREAM_CFG_BASE, base_addr + i, buf[i]);
	}

	LOG_DBG("RX stream %u configured: ip=%u.%u.%u.%u port=%u ch=%u delay=%u spc=%u",
		stream_id, ip[0], ip[1], ip[2], ip[3],
		dst_port, channel_count, output_delay, samples_per_channel);

	return 0;
}

/* ---- PTP servo / parser tuning + monitoring ---- */

int eth_litex_write_ptp_tuning(const struct device *dev,
			       const struct eth_litex_ptp_tuning *t)
{
	ARG_UNUSED(dev);

	if (!t) {
		return -EINVAL;
	}

	litex_csr_write(CSR_AES67_CSR_SERVO_KP_GAIN_ADDR, (uint8_t)t->kp_gain);
	litex_csr_write(CSR_AES67_CSR_SERVO_KI_GAIN_ADDR, (uint8_t)t->ki_gain);
	litex_csr_write(CSR_AES67_CSR_SERVO_GAIN_SHIFT_ADDR, t->gain_shift & 0x1F);
	litex_csr_write(CSR_AES67_CSR_SERVO_GAIN_SHIFT_LOCKED_ADDR, t->gain_shift_locked & 0x1F);
	litex_csr_write(CSR_AES67_CSR_SERVO_KI_EXTRA_SHIFT_ADDR, t->ki_extra_shift & 0x1F);
	litex_csr_write(CSR_AES67_CSR_SERVO_FILTER_SHIFT_ADDR, t->filter_shift & 0x1F);
	litex_csr_write(CSR_AES67_CSR_SERVO_WARMUP_SAMPLES_ADDR, t->warmup_samples);
	litex_csr_write(CSR_AES67_CSR_SERVO_LOCK_THRESHOLD_NS_ADDR, t->lock_threshold_ns);
	litex_csr_write(CSR_AES67_CSR_SERVO_UNLOCK_THRESHOLD_NS_ADDR, t->unlock_threshold_ns);
	litex_csr_write(CSR_AES67_CSR_SERVO_LOCK_COUNT_THRESHOLD_ADDR, t->lock_count_threshold);

	uint32_t mf = (t->min_filter_enable ? 1u : 0u) |
		      ((uint32_t)t->min_filter_active_depth << 8);
	litex_csr_write(CSR_AES67_CSR_PARSER_MIN_FILTER_ADDR, mf);

	return 0;
}

void eth_litex_read_ptp_tuning(const struct device *dev,
			       struct eth_litex_ptp_tuning *t)
{
	ARG_UNUSED(dev);

	if (!t) {
		return;
	}

	t->kp_gain              = (int8_t)litex_csr_read(CSR_AES67_CSR_SERVO_KP_GAIN_ADDR);
	t->ki_gain              = (int8_t)litex_csr_read(CSR_AES67_CSR_SERVO_KI_GAIN_ADDR);
	t->gain_shift           = (uint8_t)(litex_csr_read(CSR_AES67_CSR_SERVO_GAIN_SHIFT_ADDR) & 0x1F);
	t->gain_shift_locked    = (uint8_t)(litex_csr_read(CSR_AES67_CSR_SERVO_GAIN_SHIFT_LOCKED_ADDR) & 0x1F);
	t->ki_extra_shift       = (uint8_t)(litex_csr_read(CSR_AES67_CSR_SERVO_KI_EXTRA_SHIFT_ADDR) & 0x1F);
	t->filter_shift         = (uint8_t)(litex_csr_read(CSR_AES67_CSR_SERVO_FILTER_SHIFT_ADDR) & 0x1F);
	t->warmup_samples       = (uint8_t)litex_csr_read(CSR_AES67_CSR_SERVO_WARMUP_SAMPLES_ADDR);
	t->lock_threshold_ns    = litex_csr_read(CSR_AES67_CSR_SERVO_LOCK_THRESHOLD_NS_ADDR);
	t->unlock_threshold_ns  = litex_csr_read(CSR_AES67_CSR_SERVO_UNLOCK_THRESHOLD_NS_ADDR);
	t->lock_count_threshold = (uint8_t)litex_csr_read(CSR_AES67_CSR_SERVO_LOCK_COUNT_THRESHOLD_ADDR);

	uint32_t mf = litex_csr_read(CSR_AES67_CSR_PARSER_MIN_FILTER_ADDR);
	t->min_filter_enable       = (mf & 0x1) != 0;
	t->min_filter_active_depth = (uint8_t)((mf >> 8) & 0xFF);
}

void eth_litex_read_ptp_monitor(const struct device *dev,
				struct eth_litex_ptp_monitor *m)
{
	ARG_UNUSED(dev);

	if (!m) {
		return;
	}

	m->filtered_offset = (int32_t)litex_csr_read(CSR_AES67_CSR_SERVO_MON_FILTERED_OFFSET_ADDR);
	m->integral_sum    = (int32_t)litex_csr_read(CSR_AES67_CSR_SERVO_MON_INTEGRAL_SUM_ADDR);
	m->pi_proportional = (int32_t)litex_csr_read(CSR_AES67_CSR_SERVO_MON_PI_PROPORTIONAL_ADDR);
	m->pi_sum_raw      = (int32_t)litex_csr_read(CSR_AES67_CSR_SERVO_MON_PI_SUM_RAW_ADDR);

	uint32_t status = litex_csr_read(CSR_AES67_CSR_SERVO_MON_STATUS_ADDR);
	m->effective_gain_shift = (uint8_t)(status & 0xFF);
	m->lock_counter         = (uint16_t)((status >> 8) & 0xFFFF);
	m->first_lock_achieved  = ((status >> 24) & 0x1) != 0;

	m->sample_count = (uint16_t)litex_csr_read(CSR_AES67_CSR_SERVO_MON_SAMPLE_COUNT_ADDR);
}

int eth_litex_set_ptp_reset(const struct device *dev, bool held_in_reset)
{
	ARG_UNUSED(dev);
	litex_csr_write(CSR_AES67_CSR_PTP_RESET_ADDR, held_in_reset ? 1 : 0);
	return 0;
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

static void eth_litex_isr(const struct device *dev)
{
	struct eth_litex_data *data = dev->data;

	/* Unconditionally disable EventManager + clear pending.
	 * Skip the ev_pending READ — if the Wishbone bus hangs on
	 * CSR reads during IRQ context, this avoids it entirely.
	 * RX thread will re-enable after processing. */
	litex_csr_write(CSR_ETH_BUF_EV_ENABLE_ADDR, 0);
	litex_csr_write(CSR_ETH_BUF_EV_PENDING_ADDR, ETH_BUF_EV_RX_READY);
	k_sem_give(&data->rx_sem);
}

/* ---- TX path ---- */

static void eth_litex_tx_thread(void *p1, void *p2, void *p3)
{
	const struct device *dev = p1;
	struct eth_litex_data *data = dev->data;
	struct net_pkt *pkt = NULL;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		k_msgq_get(&litex_tx_queue, &pkt, K_FOREVER);

		size_t len = net_pkt_get_len(pkt);
		LOG_DBG("TX: dequeued pkt len=%zu", len);

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
			k_yield();
			int wait = 0;
			while (!(litex_csr_read(CSR_AES67_CSR_STATUS_ADDR) & AES67_STATUS_ETH_TX_DONE)) {
				k_busy_wait(10);
				if (++wait > 20000) { /* ~200ms timeout */
					LOG_WRN("TX done timeout (status=0x%08x)",
						litex_csr_read(CSR_AES67_CSR_STATUS_ADDR));
					break;
				}
			}
		}
		first_tx = false;

		LOG_DBG("TX: writing %zu bytes to buffer", len);

		/* Write packet data to TX buffer */
		eth_buf_write_packet(data->tx_buf, len);

		/* Set TX length */
		litex_csr_write(CSR_ETH_BUF_TX_LEN_ADDR, len);

		LOG_DBG("TX: pulsing tx_request");

		/* Assert eth_tx_request and hold long enough for CDC capture.
		 * The FPGA synchronises this signal through a 2-FF chain clocked
		 * by mac_tx_clock (≥2.5 MHz → ≤400ns period).  We need the pulse
		 * to span at least 3 mac_tx_clock edges, so 2µs is safe. */
		eth_litex_ctrl_set_bits(dev, AES67_CTRL_ETH_TX_REQUEST);
		k_busy_wait(2);
		eth_litex_ctrl_clear_bits(dev, AES67_CTRL_ETH_TX_REQUEST);

		LOG_DBG("TX: done, unref pkt");
		net_pkt_unref(pkt);
	}
}

/* ---- RX path ---- */

static void eth_litex_rx_thread(void *p1, void *p2, void *p3)
{
	const struct device *dev = p1;
	struct eth_litex_data *data = dev->data;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		k_sem_take(&data->rx_sem,
			   K_MSEC(CONFIG_ETH_LITEX_POLL_INTERVAL_MS));

		/* Check if a packet is ready */
		uint32_t ready = litex_csr_read(CSR_ETH_BUF_RX_READY_ADDR);
		if (!(ready & 0x01)) {
			/* Re-enable EventManager in case ISR disabled it but
			 * no packet was actually ready (race with ack). */
			litex_csr_write(CSR_ETH_BUF_EV_ENABLE_ADDR, ETH_BUF_EV_RX_READY);
			continue;
		}

		uint16_t pkt_len = litex_csr_read(CSR_ETH_BUF_RX_LEN_ADDR) & 0xFFFF;

		/* The MAC includes the 4-byte FCS in the frame.  Strip it
		 * before handing to the network stack which expects frames
		 * without FCS. */
		if (pkt_len >= 4) {
			pkt_len -= 4;
		}

		if (pkt_len < 14 || pkt_len > ETH_LITEX_MAX_PKT_SIZE) {
			LOG_WRN("RX invalid len %u", pkt_len);
			litex_csr_write(CSR_ETH_BUF_RX_ACK_ADDR, 1);
			litex_csr_write(CSR_ETH_BUF_RX_ACK_ADDR, 0);
			litex_csr_write(CSR_ETH_BUF_EV_ENABLE_ADDR, ETH_BUF_EV_RX_READY);
			continue;
		}

		/* Read packet data from RX buffer */
		eth_buf_read_packet(data->rx_buf, pkt_len);

		/* Debug: log ethertype and dst port for TCP packets */
		{
			uint16_t ethertype = ((uint16_t)data->rx_buf[12] << 8) |
					     data->rx_buf[13];
			if (ethertype == 0x0800 && pkt_len >= 34) {
				uint8_t proto = data->rx_buf[23];
				if (proto == 6) { /* TCP */
					uint16_t dst_port =
						((uint16_t)data->rx_buf[36] << 8) |
						data->rx_buf[37];
					if (dst_port == 80) {
						uint16_t ip_total_len =
							((uint16_t)data->rx_buf[16] << 8) |
							data->rx_buf[17];
						uint8_t ip_ihl =
							(data->rx_buf[14] & 0x0F) * 4;
						uint8_t tcp_doff =
							((data->rx_buf[14 + ip_ihl + 12] >> 4) & 0xF) * 4;
						uint16_t tcp_payload =
							ip_total_len - ip_ihl - tcp_doff;
						uint8_t tcp_flags =
							data->rx_buf[14 + ip_ihl + 13];
						LOG_DBG("RX TCP:80 eth=%u ip=%u ihl=%u "
							"doff=%u payload=%u flags=0x%02x "
							"seq=%02x%02x%02x%02x",
							pkt_len, ip_total_len,
							ip_ihl, tcp_doff,
							tcp_payload, tcp_flags,
							data->rx_buf[38],
							data->rx_buf[39],
							data->rx_buf[40],
							data->rx_buf[41]);
					}
				}
			}
		}

		/* ACK: release the RX buffer for the next packet */
		litex_csr_write(CSR_ETH_BUF_RX_ACK_ADDR, 1);
		litex_csr_write(CSR_ETH_BUF_RX_ACK_ADDR, 0);

		/* Re-enable EventManager IRQ (ISR disables it to prevent
		 * re-entry while the pending clear propagates). */
		litex_csr_write(CSR_ETH_BUF_EV_ENABLE_ADDR, ETH_BUF_EV_RX_READY);

		/* Allocate net_pkt and deliver to stack */
		struct net_pkt *pkt = net_pkt_rx_alloc_with_buffer(
			data->iface, pkt_len, AF_UNSPEC, 0, K_NO_WAIT);
		if (!pkt) {
			LOG_ERR("RX alloc failed (len=%u)", pkt_len);
			continue;
		}

		if (net_pkt_write(pkt, data->rx_buf, pkt_len)) {
			LOG_ERR("RX write failed");
			net_pkt_unref(pkt);
			continue;
		}

		int ret = net_recv_data(data->iface, pkt);
		if (ret < 0) {
			LOG_ERR("RX deliver err %d", ret);
			net_pkt_unref(pkt);
		}
	}
}

/* ---- Zephyr ethernet API ---- */

static int eth_litex_send(const struct device *dev, struct net_pkt *pkt)
{
	ARG_UNUSED(dev);

	/* Take a reference BEFORE posting to the TX queue.  The TX thread
	 * runs at K_PRIO_COOP(2) and will preempt any preemptive-priority
	 * caller as soon as k_msgq_put wakes it.  If we ref after the put,
	 * the TX thread can dequeue, process, and unref the pkt (freeing it)
	 * before we get a chance to ref — causing a use-after-free crash. */
	net_pkt_ref(pkt);

	if (k_msgq_put(&litex_tx_queue, &pkt, K_NO_WAIT) != 0) {
		net_pkt_unref(pkt);
		LOG_WRN("TX queue full, dropping pkt (%zu bytes)", net_pkt_get_len(pkt));
		return -ENOMEM;
	}
	LOG_DBG("TX: queued pkt (%zu bytes)", net_pkt_get_len(pkt));
	return 0;
}

static void eth_litex_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct eth_litex_data *data = dev->data;

	data->iface = iface;
	ethernet_init(iface);

	/* Static locally administered MAC */
	data->mac_addr[0] = 0x02;
	data->mac_addr[1] = 0x1C;
	data->mac_addr[2] = 0x23;
	data->mac_addr[3] = 0x17;
	data->mac_addr[4] = 0xAC;
	data->mac_addr[5] = 0xAB;

	net_if_set_link_addr(iface, data->mac_addr, sizeof(data->mac_addr),
			     NET_LINK_ETHERNET);

	/* Write MAC to FPGA so hardware knows our address */
	eth_litex_write_mac(dev, data->mac_addr);

	LOG_INF("MAC %02x:%02x:%02x:%02x:%02x:%02x",
		data->mac_addr[0], data->mac_addr[1], data->mac_addr[2],
		data->mac_addr[3], data->mac_addr[4], data->mac_addr[5]);

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

	if (!data->iface) {
		return;
	}

	uint32_t status = litex_csr_read(CSR_AES67_CSR_STATUS_ADDR);
	bool up = (status & AES67_STATUS_ETH_LINK_UP) != 0;

	LOG_DBG("link poll: status=0x%08x up=%d cached=%d", status, up, data->link_up);

	if (up && !data->link_up) {
		data->link_up = true;
		net_if_carrier_on(data->iface);
		LOG_INF("Ethernet link up");
	} else if (!up && data->link_up) {
		data->link_up = false;
		net_if_carrier_off(data->iface);
		LOG_INF("Ethernet link down");
	}

	k_work_schedule(dwork, K_MSEC(500));
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

	k_sem_init(&data->rx_sem, 0, 1);

	/* Clear any stale pending events before enabling IRQ */
	litex_csr_write(CSR_ETH_BUF_EV_PENDING_ADDR, ETH_BUF_EV_RX_READY);

	/* Connect ISR BEFORE enabling EventManager or CPU IRQ */
	IRQ_CONNECT(ETH_BUF_INTERRUPT, 0, eth_litex_isr,
		    DEVICE_GET(eth_litex0), 0);

	/* Enable EventManager — from here, pending can set on new packets,
	 * but CPU IRQ mask is not yet set so no trap yet. */
	litex_csr_write(CSR_ETH_BUF_EV_ENABLE_ADDR, ETH_BUF_EV_RX_READY);

	/* Now enable CPU IRQ — if an IRQ is already pending, the ISR
	 * will fire immediately but is safe (ISR disables ev_enable
	 * to prevent re-entry). */
	irq_enable(ETH_BUF_INTERRUPT);

	/* Make sure RX buffer is released */
	litex_csr_write(CSR_ETH_BUF_RX_ACK_ADDR, 1);
	litex_csr_write(CSR_ETH_BUF_RX_ACK_ADDR, 0);

	k_work_init_delayable(&data->link_work, eth_litex_link_work);

	k_thread_create(&data->rx_thread, data->rx_stack,
			CONFIG_ETH_LITEX_RX_STACK_SIZE,
			eth_litex_rx_thread, (void *)dev, NULL, NULL,
			K_PRIO_COOP(1), 0, K_NO_WAIT);
	k_thread_name_set(&data->rx_thread, "eth_litex_rx");

	k_thread_create(&data->tx_thread, data->tx_stack,
			CONFIG_ETH_LITEX_RX_STACK_SIZE,
			eth_litex_tx_thread, (void *)dev, NULL, NULL,
			K_PRIO_COOP(2), 0, K_NO_WAIT);
	k_thread_name_set(&data->tx_thread, "eth_litex_tx");

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
