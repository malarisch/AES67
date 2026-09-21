/*
 * Gapless I2S target port with GDMA descriptor rings (ESP32-S3).
 * See usb_audio_i2s.h for the rationale and the data model.
 *
 * Hardware ownership: this file programs I2S0 and one GDMA channel pair
 * directly through the ESP HAL. The Zephyr I2S and GDMA drivers must stay
 * disabled (CONFIG_I2S=n, `dma` node disabled) so nobody else touches the
 * GDMA registers; the SPI master driver compiles the GDMA HAL sources but
 * does not use the engine unless `dma-enabled` is set on its node.
 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/interrupt_controller/intc_esp32.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include <soc.h>
#include <esp_clk_tree.h>
#include <hal/i2s_hal.h>
#include <hal/i2s_ll.h>
#include <hal/gdma_hal.h>
#include <hal/gdma_hal_ahb.h>
#include <hal/gdma_ll.h>
#include <hal/dma_types.h>

#include "usb_audio_i2s.h"

LOG_MODULE_REGISTER(usb_audio_i2s, LOG_LEVEL_INF);

#define I2S_NODE DT_NODELABEL(i2s0)
#define DMA_NODE DT_NODELABEL(dma)

/* GDMA channel pair used for I2S0. Channel 0 is free: the Zephyr GDMA
 * driver is not built and the SPI master runs without DMA.
 */
#define UA_GDMA_CH 0
/* GDMA trigger id of I2S0 (peri_sel), same value as ESP_GDMA_TRIG_PERIPH_I2S0. */
#define UA_GDMA_PERIPH_I2S0 3

#define UA_SLOT_BITS   32
#define UA_BLOCK_WORDS (UA_I2S_BLOCK_FRAMES * UA_I2S_CHANNELS)
#define UA_BLOCK_BYTES (UA_BLOCK_WORDS * sizeof(uint32_t))
#define UA_RING_WORDS  (UA_I2S_BLOCKS * UA_BLOCK_WORDS)

#ifndef UA_I2S_INPUT_DELAY
#define UA_I2S_INPUT_DELAY 0
#endif

BUILD_ASSERT((UA_I2S_BLOCKS & (UA_I2S_BLOCKS - 1)) == 0, "block count must be a power of two");
BUILD_ASSERT((UA_I2S_BLOCK_FRAMES & (UA_I2S_BLOCK_FRAMES - 1)) == 0, "block size must be a power of two");
BUILD_ASSERT(UA_BLOCK_BYTES <= DMA_DESCRIPTOR_BUFFER_MAX_SIZE_4B_ALIGNED);
BUILD_ASSERT((UA_BLOCK_BYTES % 4) == 0);

PINCTRL_DT_DEFINE(I2S_NODE);

/* DMA buffers and descriptors live in internal DRAM: no cache maintenance
 * needed on the S3 (the data cache only fronts flash/PSRAM).
 */
static uint32_t tx_ring[UA_RING_WORDS] __aligned(4);
static uint32_t rx_ring[UA_RING_WORDS] __aligned(4);
static dma_descriptor_t tx_desc[UA_I2S_BLOCKS] __aligned(4);
static dma_descriptor_t rx_desc[UA_I2S_BLOCKS] __aligned(4);

static struct {
	i2s_hal_context_t i2s;
	gdma_hal_context_t gdma;
	struct intr_handle_data_t *rx_irq;
	struct intr_handle_data_t *tx_irq;
	bool initialised;
	bool running;
	volatile uint32_t tx_done;
	volatile uint32_t rx_done;
	volatile uint32_t errors;
	uint32_t tx_hung;
	uint32_t rx_hung;
} st;

/* ---------------------------------------------------------------------- */
/* Interrupts                                                               */

/* Blocks completed since the last count, derived from the descriptor the
 * hardware reports as its last EOF, so coalesced interrupts are not lost.
 */
static inline uint32_t blocks_completed(uint32_t done, uintptr_t eof_addr, const dma_descriptor_t *ring)
{
	uint32_t idx = (eof_addr - (uintptr_t)ring) / sizeof(dma_descriptor_t);
	uint32_t delta = (idx + 1 - done) & (UA_I2S_BLOCKS - 1);

	return delta ? delta : UA_I2S_BLOCKS;
}

static void ua_i2s_rx_isr(void *arg)
{
	ARG_UNUSED(arg);
	uint32_t status = gdma_hal_read_intr_status(&st.gdma, UA_GDMA_CH,
						    GDMA_CHANNEL_DIRECTION_RX, false);

	gdma_hal_clear_intr(&st.gdma, UA_GDMA_CH, GDMA_CHANNEL_DIRECTION_RX, status);

	if (status & GDMA_LL_EVENT_RX_SUC_EOF) {
		uintptr_t eof = gdma_ll_rx_get_success_eof_desc_addr(st.gdma.dev, UA_GDMA_CH);

		st.rx_done += blocks_completed(st.rx_done, eof, rx_desc);
	}
	if (status & (GDMA_LL_EVENT_RX_ERR_EOF | GDMA_LL_EVENT_RX_DESC_ERROR)) {
		st.errors++;
	}
}

static void ua_i2s_tx_isr(void *arg)
{
	ARG_UNUSED(arg);
	uint32_t status = gdma_hal_read_intr_status(&st.gdma, UA_GDMA_CH,
						    GDMA_CHANNEL_DIRECTION_TX, false);

	gdma_hal_clear_intr(&st.gdma, UA_GDMA_CH, GDMA_CHANNEL_DIRECTION_TX, status);

	if (status & GDMA_LL_EVENT_TX_EOF) {
		uintptr_t eof = gdma_ll_tx_get_eof_desc_addr(st.gdma.dev, UA_GDMA_CH);
		uint32_t done = st.tx_done;
		uint32_t n = blocks_completed(done, eof, tx_desc);

		/* Silence the blocks the DMA just left behind: if the
		 * application falls behind, the ring replays silence
		 * instead of the last millisecond of audio.
		 */
		for (uint32_t b = 0; b < n; b++) {
			uint32_t *blk = &tx_ring[((done + b) & (UA_I2S_BLOCKS - 1)) * UA_BLOCK_WORDS];

			for (uint32_t w = 0; w < UA_BLOCK_WORDS; w++) {
				blk[w] = 0;
			}
		}
		st.tx_done = done + n;
	}
	if (status & GDMA_LL_EVENT_TX_DESC_ERROR) {
		st.errors++;
	}
}

/* ---------------------------------------------------------------------- */
/* Setup                                                                    */

static void ua_i2s_build_rings(void)
{
	for (int i = 0; i < UA_I2S_BLOCKS; i++) {
		int next = (i + 1) & (UA_I2S_BLOCKS - 1);

		memset(&tx_desc[i], 0, sizeof(tx_desc[i]));
		tx_desc[i].buffer = &tx_ring[i * UA_BLOCK_WORDS];
		tx_desc[i].dw0.size = UA_BLOCK_BYTES;
		tx_desc[i].dw0.length = UA_BLOCK_BYTES;
		tx_desc[i].dw0.suc_eof = 1; /* EOF interrupt per block */
		tx_desc[i].dw0.owner = DMA_DESCRIPTOR_BUFFER_OWNER_DMA;
		tx_desc[i].next = &tx_desc[next];

		memset(&rx_desc[i], 0, sizeof(rx_desc[i]));
		rx_desc[i].buffer = &rx_ring[i * UA_BLOCK_WORDS];
		rx_desc[i].dw0.size = UA_BLOCK_BYTES;
		rx_desc[i].dw0.owner = DMA_DESCRIPTOR_BUFFER_OWNER_DMA;
		rx_desc[i].next = &rx_desc[next];
	}
}

static int ua_i2s_setup_gdma(void)
{
	const struct device *clk = DEVICE_DT_GET(DT_CLOCKS_CTLR(DMA_NODE));
	gdma_hal_config_t hal_cfg = {
		.group_id = GDMA_LL_AHB_GROUP_START_ID,
	};
	int ret;

	ret = clock_control_on(clk, (clock_control_subsys_t)DT_CLOCKS_CELL(DMA_NODE, offset));
	if (ret < 0 && ret != -EALREADY) {
		LOG_ERR("GDMA clock: %d", ret);
		return ret;
	}

	gdma_ahb_hal_init(&st.gdma, &hal_cfg);
	gdma_ll_force_enable_reg_clock(st.gdma.dev, true);

	for (int dir = 0; dir < 2; dir++) {
		gdma_channel_direction_t d = dir ? GDMA_CHANNEL_DIRECTION_TX
						 : GDMA_CHANNEL_DIRECTION_RX;
		uint32_t mask = dir ? GDMA_LL_TX_EVENT_MASK : GDMA_LL_RX_EVENT_MASK;

		gdma_hal_reset(&st.gdma, UA_GDMA_CH, d);
		gdma_hal_connect_peri(&st.gdma, UA_GDMA_CH, d, UA_GDMA_PERIPH_I2S0);
		/* No owner check: the rings are circular and the CPU never
		 * hands descriptors back; no descriptor write-back either.
		 */
		gdma_hal_set_strategy(&st.gdma, UA_GDMA_CH, d, false, false, false);
		/* Audio must win the AHB arbitration against the SPI and USB DMA
		 * bursts: a refill that comes a few bit clocks late shifts the
		 * whole I2S frame (experiment C). */
		gdma_hal_set_priority(&st.gdma, UA_GDMA_CH, d, GDMA_LL_CHANNEL_MAX_PRIORITY);
		gdma_hal_enable_intr(&st.gdma, UA_GDMA_CH, d, mask, false);
		gdma_hal_clear_intr(&st.gdma, UA_GDMA_CH, d, mask);
	}

	/* Channel 0 interrupt sources: index 0 = IN (RX), 1 = OUT (TX) in the
	 * SoC DTS `dma` node.
	 */
	ret = esp_intr_alloc(DT_IRQ_BY_IDX(DMA_NODE, 0, irq),
			     ESP_PRIO_TO_FLAGS(DT_IRQ_BY_IDX(DMA_NODE, 0, priority)),
			     (intr_handler_t)ua_i2s_rx_isr, NULL, &st.rx_irq);
	if (ret != 0) {
		LOG_ERR("GDMA RX interrupt: %d", ret);
		return -EIO;
	}
	ret = esp_intr_alloc(DT_IRQ_BY_IDX(DMA_NODE, 1, irq),
			     ESP_PRIO_TO_FLAGS(DT_IRQ_BY_IDX(DMA_NODE, 1, priority)),
			     (intr_handler_t)ua_i2s_tx_isr, NULL, &st.tx_irq);
	if (ret != 0) {
		LOG_ERR("GDMA TX interrupt: %d", ret);
		return -EIO;
	}

	return 0;
}

static int ua_i2s_setup_port(uint32_t sample_rate)
{
	const struct device *clk = DEVICE_DT_GET(DT_CLOCKS_CTLR(I2S_NODE));
	i2s_hal_context_t *hal = &st.i2s;
	i2s_hal_slot_config_t slot = {0};
	i2s_hal_clock_info_t ci;
	uint32_t sclk = 0;
	int ret;

	ret = clock_control_on(clk, (clock_control_subsys_t)DT_CLOCKS_CELL(I2S_NODE, offset));
	if (ret < 0 && ret != -EALREADY) {
		LOG_ERR("I2S clock: %d", ret);
		return ret;
	}

	ret = pinctrl_apply_state(PINCTRL_DT_DEV_CONFIG_GET(I2S_NODE), PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("I2S pins: %d", ret);
		return ret;
	}

	hal->dev = (i2s_dev_t *)DT_REG_ADDR(I2S_NODE);
	i2s_ll_enable_core_clock(hal->dev, true);

	/* Philips I2S, 32-bit slots, 32-bit data (24 valid, MSB-aligned). */
	slot.data_bit_width = UA_SLOT_BITS;
	slot.slot_bit_width = UA_SLOT_BITS;
	slot.slot_mode = I2S_SLOT_MODE_STEREO;
	slot.std.slot_mask = I2S_STD_SLOT_BOTH;
	slot.std.ws_width = UA_SLOT_BITS;
	slot.std.ws_pol = false;
	slot.std.bit_shift = true;
	slot.std.left_align = true;
	slot.std.big_endian = false;
	slot.std.bit_order_lsb = false;

	/* Target mode: BCLK/WS come from the pins. The module still samples
	 * them with its internal clock, which must run at >= 8 x BCLK
	 * (ESP-IDF fixes bclk_div = 8 and derives MCLK from BCLK). 16 x was
	 * tried against the single-frame bit slips seen on the FPGA side
	 * (2026-09-21) and made no difference.
	 */
	esp_clk_tree_src_get_freq_hz(I2S_CLK_SRC_DEFAULT, ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED,
				     &sclk);
	ci.sclk = sclk;
	ci.bclk = sample_rate * UA_I2S_CHANNELS * UA_SLOT_BITS;
	ci.bclk_div = 8;
	ci.mclk = ci.bclk * ci.bclk_div;
	ci.mclk_div = ci.mclk ? ci.sclk / ci.mclk : 0;
	if (ci.mclk_div == 0) {
		LOG_ERR("I2S module clock %u Hz too slow for BCLK %u Hz", sclk, ci.bclk);
		return -EINVAL;
	}

	i2s_hal_std_set_rx_slot(hal, true, &slot);
	i2s_hal_set_rx_clock(hal, &ci, I2S_CLK_SRC_DEFAULT, NULL);
	i2s_ll_rx_enable_std(hal->dev);

	i2s_hal_std_set_tx_slot(hal, true, &slot);
	i2s_hal_set_tx_clock(hal, &ci, I2S_CLK_SRC_DEFAULT, NULL);
	i2s_ll_tx_enable_std(hal->dev);

	/* RX follows the TX module's BCLK/WS inputs. */
	i2s_ll_share_bck_ws(hal->dev, true);

	LOG_INF("I2S0 target: %u Hz, BCLK %u Hz, module clock %u/%u",
		sample_rate, ci.bclk, sclk, ci.mclk_div);
	return 0;
}

int ua_i2s_init(uint32_t sample_rate)
{
	int ret;

	if (st.initialised) {
		return 0;
	}

	ua_i2s_build_rings();

	ret = ua_i2s_setup_gdma();
	if (ret < 0) {
		return ret;
	}
	ret = ua_i2s_setup_port(sample_rate);
	if (ret < 0) {
		return ret;
	}

	st.initialised = true;
	return 0;
}

/* ---------------------------------------------------------------------- */
/* Run control                                                              */

int ua_i2s_start(void)
{
	i2s_hal_context_t *hal = &st.i2s;

	if (!st.initialised) {
		return -ENODEV;
	}
	if (st.running) {
		return 0;
	}

	memset(rx_ring, 0, sizeof(rx_ring));
	st.tx_done = 0;
	st.rx_done = 0;

	/* RX: I2S raises EOF every block, the GDMA follows the ring. */
	i2s_hal_rx_stop(hal);
	i2s_hal_rx_reset(hal);
	i2s_hal_rx_reset_fifo(hal);
	gdma_hal_reset(&st.gdma, UA_GDMA_CH, GDMA_CHANNEL_DIRECTION_RX);
	gdma_hal_clear_intr(&st.gdma, UA_GDMA_CH, GDMA_CHANNEL_DIRECTION_RX, GDMA_LL_RX_EVENT_MASK);
	gdma_hal_enable_intr(&st.gdma, UA_GDMA_CH, GDMA_CHANNEL_DIRECTION_RX,
			     GDMA_LL_EVENT_RX_SUC_EOF | GDMA_LL_EVENT_RX_ERR_EOF |
			     GDMA_LL_EVENT_RX_DESC_ERROR, true);
	i2s_ll_rx_set_eof_num(hal->dev, UA_BLOCK_BYTES);
	gdma_hal_start_with_desc(&st.gdma, UA_GDMA_CH, GDMA_CHANNEL_DIRECTION_RX,
				 (intptr_t)&rx_desc[0]);

	/* TX: descriptors carry EOF themselves. */
	i2s_hal_tx_stop(hal);
	i2s_hal_tx_reset(hal);
	i2s_hal_tx_reset_fifo(hal);
	gdma_hal_reset(&st.gdma, UA_GDMA_CH, GDMA_CHANNEL_DIRECTION_TX);
	gdma_hal_clear_intr(&st.gdma, UA_GDMA_CH, GDMA_CHANNEL_DIRECTION_TX, GDMA_LL_TX_EVENT_MASK);
	gdma_hal_enable_intr(&st.gdma, UA_GDMA_CH, GDMA_CHANNEL_DIRECTION_TX,
			     GDMA_LL_EVENT_TX_EOF | GDMA_LL_EVENT_TX_DESC_ERROR, true);
	gdma_hal_start_with_desc(&st.gdma, UA_GDMA_CH, GDMA_CHANNEL_DIRECTION_TX,
				 (intptr_t)&tx_desc[0]);

	esp_intr_enable(st.rx_irq);
	esp_intr_enable(st.tx_irq);

#if UA_I2S_INPUT_DELAY
	/* Experiment D: register the external BCLK/WS/SD inputs through the
	 * module's delay stage ("delay by pos edge") instead of feeding the
	 * asynchronous pins straight into the edge detector.
	 */
	hal->dev->tx_timing.tx_bck_in_dm = 1;
	hal->dev->tx_timing.tx_ws_in_dm = 1;
	hal->dev->rx_timing.rx_bck_in_dm = 1;
	hal->dev->rx_timing.rx_ws_in_dm = 1;
	hal->dev->rx_timing.rx_sd_in_dm = 1;
#endif

	/* FIFO timeout detector for diagnostics (tx_hung / rx_hung). */
	hal->dev->lc_hung_conf.fifo_timeout = 1;
	hal->dev->lc_hung_conf.fifo_timeout_shift = 7;
	hal->dev->lc_hung_conf.fifo_timeout_ena = 1;
	hal->dev->int_clr.val = hal->dev->int_raw.val;
	st.tx_hung = 0;
	st.rx_hung = 0;

	i2s_hal_rx_start(hal);
	i2s_hal_tx_start(hal);

	st.running = true;
	return 0;
}

void ua_i2s_stop(void)
{
	i2s_hal_context_t *hal = &st.i2s;

	if (!st.running) {
		return;
	}

	i2s_hal_tx_stop(hal);
	i2s_hal_rx_stop(hal);
	gdma_hal_stop(&st.gdma, UA_GDMA_CH, GDMA_CHANNEL_DIRECTION_TX);
	gdma_hal_stop(&st.gdma, UA_GDMA_CH, GDMA_CHANNEL_DIRECTION_RX);
	gdma_hal_enable_intr(&st.gdma, UA_GDMA_CH, GDMA_CHANNEL_DIRECTION_TX,
			     GDMA_LL_TX_EVENT_MASK, false);
	gdma_hal_enable_intr(&st.gdma, UA_GDMA_CH, GDMA_CHANNEL_DIRECTION_RX,
			     GDMA_LL_RX_EVENT_MASK, false);
	esp_intr_disable(st.tx_irq);
	esp_intr_disable(st.rx_irq);

	st.running = false;
}

uint32_t ua_i2s_tx_done(void)
{
	return st.tx_done;
}

uint32_t ua_i2s_rx_done(void)
{
	return st.rx_done;
}

uint32_t *ua_i2s_tx_ring(void)
{
	return tx_ring;
}

const uint32_t *ua_i2s_rx_ring(void)
{
	return rx_ring;
}

void ua_i2s_hung(uint32_t *tx, uint32_t *rx)
{
	i2s_dev_t *dev = st.i2s.dev;

	if (dev != NULL) {
		uint32_t raw = dev->int_raw.val;

		if (raw & BIT(3)) {
			st.tx_hung++;
		}
		if (raw & BIT(2)) {
			st.rx_hung++;
		}
		dev->int_clr.val = raw;
	}
	*tx = st.tx_hung;
	*rx = st.rx_hung;
}

uint32_t ua_i2s_errors(void)
{
	return st.errors;
}
