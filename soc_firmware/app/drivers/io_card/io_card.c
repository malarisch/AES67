/*
 * IO Card Driver - 16-Channel ADC Input / 8-Channel DAC Output Board
 *
 * Ported from original IoCard.c (c. kuehnel, intek, 24.3.2007).
 *
 * Hardware topology (verified):
 *   - 2x LPC preamp controllers (0x40, 0x41)  — directly on I2C bus
 *   - 2x CS5368 8-ch ADCs (0x4C, 0x4D)        — directly on I2C bus
 *   - 1x LPC output controller (0x43)          
 *   - 1x CS4385 8-ch DAC (0x18)               — directly on I2C bus
 *
 * I2C MUX usage:
 *   select_mux(IO_MUX_CH0)  -> enables channel 0, reaches output LPC (0x43)
 *   select_mux(IO_MUX_NONE) -> deselects MUX, returns to main bus
 *
 * Output channel scrambling (IOT_OI board type, board_id == IO_OUT_LPC_BOARD_ID_B):
 *   Physical address inside DAC depends on channel index. See io_out_clip_addr().
 */

#include "io_card.h"

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(io_card, CONFIG_IO_CARD_LOG_LEVEL);

/*******************************************************************************
 * Gain Lookup Table
 *
 * Index 0-72 maps gain_db = index + IO_IN_GAIN_MIN (-6..+66 dB) to
 * two-byte LPC channel register value:
 *   High byte (>> 8): bit 0 = attenuation pad active
 *   Low byte  (& 0xFF): bits 0-5 = analog gain register
 *
 * Indices 0-23:  Attenuation ON  (high byte = 0x01)
 * Indices 24-72: Attenuation OFF (high byte = 0x00)
 ******************************************************************************/
static const uint16_t gain_table[IO_IN_GAIN_ENTRIES] = {
	/* -6 to +17 dB: Attenuation ON */
	0x0102, 0x0103, 0x0104, 0x0105, 0x0106, 0x0107, 0x0108, 0x0109,
	0x010A, 0x010B, 0x010C, 0x010D, 0x010E, 0x010F, 0x0110, 0x0111,
	0x0112, 0x0113, 0x0114, 0x0115, 0x0116, 0x0117, 0x0118, 0x0119,
	/* +18 to +66 dB: Attenuation OFF */
	0x0002, 0x0003, 0x0004, 0x0005, 0x0006, 0x0007, 0x0008, 0x0009,
	0x000A, 0x000B, 0x000C, 0x000D, 0x000E, 0x000F, 0x0010, 0x0011,
	0x0012, 0x0013, 0x0014, 0x0015, 0x0016, 0x0017, 0x0018, 0x0019,
	0x001A, 0x001B, 0x001C, 0x001D, 0x001E, 0x001F, 0x0020, 0x0021,
	0x0022, 0x0023, 0x0024, 0x0025, 0x0026, 0x0027, 0x0028, 0x0029,
	0x002A, 0x002B, 0x002C, 0x002D, 0x002E, 0x002F, 0x0030, 0x0031,
	0x0032
};

/* I2C addresses for the two input LPC controllers */
static const uint8_t in_lpc_addr[IO_NUM_IN_LPC] = {
	IO_IN_LPC_ADDR0,
	IO_IN_LPC_ADDR1,
};

/* I2C addresses for the two ADC chips */
static const uint8_t in_adc_addr[IO_NUM_IN_LPC] = {
	IO_IN_ADC_ADDR0,
	IO_IN_ADC_ADDR1,
};

/* Driver instance */
static struct io_card_data io_data;

/* Whether the output side uses scrambled channel addressing (IOT_OI) */
static bool out_scrambled;

/* Whether a PCA9540B/TCA9543A MUX is present on the bus.
 * Auto-detected during init.  When false, 0x43 is accessed directly. */
static bool mux_present;

/*******************************************************************************
 * Internal: I2C helpers
 ******************************************************************************/

/**
 * @brief Write registers directly on the main I2C bus (no MUX)
 */
static int lpc_write(uint8_t addr, uint8_t reg, const uint8_t *data, size_t len)
{
	uint8_t buf[16];

	if (len > sizeof(buf) - 1) {
		return -EINVAL;
	}

	buf[0] = reg;
	memcpy(&buf[1], data, len);

	int ret = i2c_write(io_data.i2c_dev, buf, len + 1, addr);

	if (ret < 0) {
		LOG_ERR("I2C write failed: addr=0x%02x reg=0x%02x err=%d", addr, reg, ret);
	}
	return ret;
}

/**
 * @brief Read registers directly on the main I2C bus (no MUX)
 */
static int lpc_read(uint8_t addr, uint8_t reg, uint8_t *data, size_t len)
{
	int ret = i2c_write_read(io_data.i2c_dev, addr, &reg, 1, data, len);

	if (ret < 0) {
		LOG_ERR("I2C read failed: addr=0x%02x reg=0x%02x err=%d", addr, reg, ret);
	}
	return ret;
}

/**
 * @brief Select a MUX channel before accessing devices behind it.
 *
 * Writes the channel selector to the MUX device at CONFIG_IO_CARD_MUX_ADDR.
 * Pass IO_MUX_NONE to deselect all channels.
 *
 * This driver uses a simple 1-byte MUX command as used by PCA9540B / TCA9543A:
 *   0x04 = enable channel 0
 *   0x05 = enable channel 1
 *   0x00 = disable all channels
 */
static int select_mux(uint8_t channel)
{
	uint8_t cmd = (channel == IO_MUX_NONE) ? 0x00 : (0x04 | (channel & 0x01));

	int ret = i2c_write(io_data.i2c_dev, &cmd, 1, CONFIG_IO_CARD_MUX_ADDR);

	if (ret < 0) {
		LOG_ERR("MUX select failed: ch=%d err=%d", channel, ret);
	}
	return ret;
}

/**
 * @brief Write to the output LPC (0x43).
 *
 * Uses MUX channel 0 if a MUX is present, otherwise accesses directly.
 */
static int out_lpc_write(uint8_t reg, const uint8_t *data, size_t len)
{
	int ret;

	if (mux_present) {
		ret = select_mux(IO_MUX_CH0);
		if (ret < 0) {
			return ret;
		}
	}

	ret = lpc_write(IO_OUT_LPC_ADDR, reg, data, len);

	if (mux_present) {
		select_mux(IO_MUX_NONE);
	}
	return ret;
}

/**
 * @brief Read from the output LPC (0x43).
 *
 * Uses MUX channel 0 if a MUX is present, otherwise accesses directly.
 */
static int out_lpc_read(uint8_t reg, uint8_t *data, size_t len)
{
	int ret;

	if (mux_present) {
		ret = select_mux(IO_MUX_CH0);
		if (ret < 0) {
			return ret;
		}
	}

	ret = lpc_read(IO_OUT_LPC_ADDR, reg, data, len);

	if (mux_present) {
		select_mux(IO_MUX_NONE);
	}
	return ret;
}

/*******************************************************************************
 * Internal: register helpers
 ******************************************************************************/

/**
 * @brief Push a single input channel's register pair to the correct LPC chip.
 */
static int update_in_channel_reg(uint8_t channel)
{
	uint8_t lpc_idx = channel / 8;
	uint8_t chn_local = channel % 8;
	uint8_t reg = IO_LPC_CHN_BASE + chn_local * 2;

	return lpc_write(in_lpc_addr[lpc_idx], reg,
			 (uint8_t *)&io_data.in_chn_reg[channel], 2);
}

/**
 * @brief Push the global register for an input LPC chip.
 */
static int update_in_glb_reg(uint8_t lpc_idx)
{
	return lpc_write(in_lpc_addr[lpc_idx], IO_LPC_GLB_REG,
			 &io_data.in_glb_reg[lpc_idx], 1);
}

/**
 * @brief Push the output-enable register to the output LPC via MUX.
 */
static int update_out_enable_reg(void)
{
	return out_lpc_write(IO_LPC_OE_REG, &io_data.out_enable[0], 1);
}

/**
 * @brief Push the global register to the output LPC via MUX.
 */
static int update_out_glb_reg(void)
{
	return out_lpc_write(IO_LPC_GLB_REG, &io_data.out_glb_reg[0], 1);
}

/**
 * @brief Compute the DAC register address for a clip level write.
 *
 * CS4385 volume registers are arranged in pairs with a mixing control
 * register between each pair:
 *   Pair 1: 0x0B (A1), 0x0C (B1), 0x0D (mix)
 *   Pair 2: 0x0E (A2), 0x0F (B2), 0x10 (mix)
 *   Pair 3: 0x11 (A3), 0x12 (B3), 0x13 (mix)
 *   Pair 4: 0x14 (A4), 0x15 (B4)
 *
 * Formula: VOL_BASE_REG + ch + ch/2 gives the correct register for ch 0-7.
 * All 8 channels reside on a single CS4385 at IO_OUT_DAC_ADDR0.
 */
static uint8_t io_out_clip_addr(uint8_t channel)
{
	/* DAC volume register layout: 3 registers per 2 channels (A, B, mix)
	 * VOL_BASE_REG + ch + ch/2
	 */
	return (uint8_t)(IO_DAC_VOL_BASE_REG + channel + channel / 2);
}

/*******************************************************************************
 * Internal: gain index conversion
 ******************************************************************************/

static int gain_db_to_index(int8_t gain_db)
{
	int index = (int)gain_db - IO_IN_GAIN_MIN;

	if (index < 0 || index >= IO_IN_GAIN_ENTRIES) {
		return -EINVAL;
	}
	return index;
}

/*******************************************************************************
 * Public API
 ******************************************************************************/

int io_card_init(const struct device *i2c_dev)
{
	int ret;
	uint8_t data8;

	if (i2c_dev == NULL) {
		LOG_ERR("I2C device is NULL");
		return -EINVAL;
	}

	if (!device_is_ready(i2c_dev)) {
		LOG_ERR("I2C device not ready");
		return -ENODEV;
	}

	memset(&io_data, 0, sizeof(io_data));
	io_data.i2c_dev = i2c_dev;
	k_mutex_init(&io_data.lock);
	k_mutex_lock(&io_data.lock, K_FOREVER);

	/* ---- Detect input board ---- */
	struct io_board_info info;

	if (!io_card_detect(&info)) {
		LOG_WRN("IO card input LPC not detected at 0x%02x", IO_IN_LPC_ADDR0);
		k_mutex_unlock(&io_data.lock);
		return -ENODEV;
	}

	LOG_INF("IO card detected: soft_id=0x%02x board_id=0x%02x rev=0x%02x",
		info.soft_id, info.board_id, info.hard_rev);

	/* ---- Detect MUX and output LPC (0x43) ---- */
	{
		uint8_t buf[3];

		/* Probe MUX first */
		uint8_t mux_probe = 0x00;

		if (i2c_write(io_data.i2c_dev, &mux_probe, 1,
			      CONFIG_IO_CARD_MUX_ADDR) == 0) {
			mux_present = true;
			LOG_INF("I2C MUX detected at 0x%02x", CONFIG_IO_CARD_MUX_ADDR);
		} else {
			mux_present = false;
			LOG_INF("No I2C MUX at 0x%02x - output LPC accessed directly",
				CONFIG_IO_CARD_MUX_ADDR);
		}

		ret = out_lpc_read(IO_LPC_SOFT_ID, buf, 3);
		if (ret == 0) {
			out_scrambled = (buf[1] == IO_OUT_LPC_BOARD_ID_B);
			LOG_INF("Output LPC: soft_id=0x%02x board_id=0x%02x rev=0x%02x (%s)",
				buf[0], buf[1], buf[2],
				out_scrambled ? "IOT_OI" : "IOT_IO");
		} else {
			LOG_WRN("Output LPC 0x%02x not detected (err=%d) - outputs unavailable",
				IO_OUT_LPC_ADDR, ret);
			out_scrambled = false;
		}
	}

	/* ---- Init input LPCs (release from reset, set sample rate) ---- */
	for (int i = 0; i < IO_NUM_IN_LPC; i++) {
		io_data.in_glb_reg[i] = IO_GLB_NRST;
		ret = update_in_glb_reg(i);
		if (ret < 0) {
			LOG_ERR("Failed to init input LPC %d", i);
			k_mutex_unlock(&io_data.lock);
			return ret;
		}
	}


	k_msleep(50);

	/* ---- Init input channels: default gain 0 dB, no phantom ---- */
	int gain_idx = gain_db_to_index(0);
	uint16_t gain_val = gain_table[gain_idx];

	for (int i = 0; i < IO_NUM_IN_CHANNELS; i++) {
		io_data.in_chn_reg[i].high = (uint8_t)((gain_val >> 8) & 0xFF);
		io_data.in_chn_reg[i].low  = (uint8_t)(gain_val & 0xFF);

		ret = update_in_channel_reg(i);
		if (ret < 0) {
			LOG_ERR("Failed to init input channel %d", i);
			k_mutex_unlock(&io_data.lock);
			return ret;
		}
	}

	/* ---- Init ADC chips (CS5368): TDM slave mode ---- */
	/*
	 * CS5368 I2C address = 0b10011[AD1][AD0].
	 * AD1/AD0 pins are shared with DIF1/DIF0 (Stand-Alone Mode format pins).
	 * Probe all four possible addresses with retries — the CS5368 may need
	 * extra time after nRST de-assertion before the I2C port responds.
	 */
	{
		static const uint8_t adc_addrs[] = {
			IO_IN_ADC_ADDR0, IO_IN_ADC_ADDR1,         /* 0x4C, 0x4D */
			IO_IN_ADC_ADDR0_ALT, IO_IN_ADC_ADDR1_ALT, /* 0x4E, 0x4F */
		};
		uint8_t found_addr[IO_NUM_IN_LPC] = {0};
		int found_count = 0;

		/* Retry with increasing delays — total worst case ~2s */
		for (int attempt = 0; attempt < 10 && found_count < IO_NUM_IN_LPC; attempt++) {
			if (attempt > 0) {
				LOG_INF("ADC probe retry %d/10 (waiting 200 ms)...", attempt);
				k_msleep(200);
			}

			for (int a = 0; a < ARRAY_SIZE(adc_addrs); a++) {
				uint8_t addr = adc_addrs[a];
				uint8_t id = 0;

				/* Skip addresses we already found */
				bool already = false;
				for (int f = 0; f < found_count; f++) {
					if (found_addr[f] == addr) {
						already = true;
						break;
					}
				}
				if (already) {
					continue;
				}

				/* Try reading the chip ID register (0x00) — a clean
				 * way to check if the chip is alive without side effects */
				ret = lpc_read(addr, 0x00, &id, 1);
				if (ret < 0) {
					continue;
				}

				LOG_INF("ADC at 0x%02x alive (REVI=0x%02x), attempt %d",
					addr, id, attempt);

				/* Write CP-EN first, then full config */
				data8 = 0x80;
				ret = lpc_write(addr, IO_ADC_GMCR_REG, &data8, 1);
				if (ret < 0) {
					LOG_ERR("ADC 0x%02x CP-EN write failed", addr);
					continue;
				}

				data8 = IO_ADC_GMCR_VALUE;
				ret = lpc_write(addr, IO_ADC_GMCR_REG, &data8, 1);
				if (ret < 0) {
					LOG_ERR("ADC 0x%02x GCTL write failed", addr);
					continue;
				}

				/* Verify */
				uint8_t readback = 0;
				ret = lpc_read(addr, IO_ADC_GMCR_REG, &readback, 1);
				if (ret == 0 && readback == IO_ADC_GMCR_VALUE) {
					LOG_INF("ADC 0x%02x GCTL=0x%02x OK (TDM slave)",
						addr, readback);
				} else {
					LOG_WRN("ADC 0x%02x readback mismatch: 0x%02x",
						addr, readback);
				}

				if (found_count < IO_NUM_IN_LPC) {
					found_addr[found_count++] = addr;
				}
			}
		}

		if (found_count == 0) {
			LOG_WRN("No CS5368 ADCs found after retries");
		} else {
			LOG_INF("Found %d CS5368 ADC(s)", found_count);
		}
	}

	/* ---- Init output LPC (release from reset, clear OE) via MUX ---- */
	io_data.out_glb_reg[0] = IO_GLB_NRST;
	ret = update_out_glb_reg();
	if (ret < 0) {
		LOG_WRN("Output LPC GLB init failed (err=%d)", ret);
	}

	io_data.out_enable[0] = 0x00;
	ret = update_out_enable_reg();
	if (ret < 0) {
		LOG_WRN("Output LPC OE clear failed (err=%d)", ret);
	}

	/* ---- Init DAC chip (CS4385 at 0x18, all 8 channels) ---- */
	{
		data8 = IO_DAC_GMCR2_INIT;
		ret = lpc_write(IO_OUT_DAC_ADDR0, IO_DAC_GMCR2_REG, &data8, 1);
		if (ret < 0) {
			LOG_WRN("DAC 0x%02x GMCR2 init failed (err=%d)",
				IO_OUT_DAC_ADDR0, ret);
		} else {
			data8 = IO_DAC_PCM_INIT;
			ret = lpc_write(IO_OUT_DAC_ADDR0, IO_DAC_PCM_REG, &data8, 1);
			if (ret < 0) {
				LOG_WRN("DAC 0x%02x PCM init failed (err=%d)",
					IO_OUT_DAC_ADDR0, ret);
			} else {
				data8 = IO_DAC_GMCR2_PDN;
				ret = lpc_write(IO_OUT_DAC_ADDR0, IO_DAC_GMCR2_REG,
						&data8, 1);
				if (ret < 0) {
					LOG_WRN("DAC 0x%02x PDN failed (err=%d)",
						IO_OUT_DAC_ADDR0, ret);
				} else {
					LOG_INF("DAC 0x%02x initialized (8-ch)",
						IO_OUT_DAC_ADDR0);
				}
			}
		}
	}

	/* ---- Set default clip levels for all output channels ---- */
	for (int ch = 0; ch < IO_NUM_OUT_CHANNELS; ch++) {
		io_data.out_clip_db[ch] = IO_OUT_CLIP_MAX;
		ret = io_card_set_out_clip(ch, IO_OUT_CLIP_MAX);
		if (ret < 0) {
			LOG_WRN("Output channel %d clip init failed", ch);
		}
	}

	/* ---- Default output enable: all channels on ---- */
	io_data.out_enable[0] = 0xFF;
	io_data.global_out_enable = true;

	io_data.initialized = true;
	k_mutex_unlock(&io_data.lock);

	LOG_INF("IO card initialized: 16 inputs, 8 outputs");
	return 0;
}

bool io_card_detect(struct io_board_info *info)
{
	uint8_t buf[3];

	if (io_data.i2c_dev == NULL) {
		return false;
	}

	int ret = lpc_read(IO_IN_LPC_ADDR0, IO_LPC_SOFT_ID, buf, 3);

	if (ret < 0) {
		return false;
	}

	if (buf[1] != IO_IN_LPC0_BOARD_ID) {
		LOG_DBG("Input LPC0 unexpected board_id=0x%02x", buf[1]);
		return false;
	}

	if (info != NULL) {
		info->soft_id  = buf[0];
		info->board_id = buf[1];
		info->hard_rev = buf[2];
	}

	return true;
}

/*******************************************************************************
 * Input (Add-Channel) API
 ******************************************************************************/

int io_card_set_in_gain(uint8_t channel, int8_t gain_db)
{
	if (channel >= IO_NUM_IN_CHANNELS) {
		return -EINVAL;
	}
	if (!io_data.initialized) {
		return -ENODEV;
	}

	int idx = gain_db_to_index(gain_db);

	if (idx < 0) {
		LOG_ERR("Gain %d dB out of range [%d, %d]",
			gain_db, IO_IN_GAIN_MIN, IO_IN_GAIN_MAX);
		return -EINVAL;
	}

	uint16_t gain_val = gain_table[idx];

	k_mutex_lock(&io_data.lock, K_FOREVER);

	/* Preserve phantom bit, update gain and attenuation */
	uint8_t phantom_bit = io_data.in_chn_reg[channel].high & IO_CHN_PHANTOM;

	io_data.in_chn_reg[channel].high = ((gain_val >> 8) & 0xFF) | phantom_bit;
	io_data.in_chn_reg[channel].low  = (gain_val & 0xFF);

	int ret = update_in_channel_reg(channel);

	k_mutex_unlock(&io_data.lock);

	if (ret == 0) {
		LOG_DBG("In ch%d gain=%d dB (reg=0x%04x)", channel, gain_db, gain_val);
	}
	return ret;
}

int io_card_get_in_gain(uint8_t channel)
{
	if (channel >= IO_NUM_IN_CHANNELS || !io_data.initialized) {
		return -EINVAL;
	}

	k_mutex_lock(&io_data.lock, K_FOREVER);

	uint8_t atten    = io_data.in_chn_reg[channel].high & IO_CHN_ATTENUATION;
	uint8_t gain_reg = io_data.in_chn_reg[channel].low  & IO_CHN_GAIN_MASK;

	/* gain_reg == 2 corresponds to 0 dB base; offset 24 when attenuation off */
	int preamp_gain = (int)(gain_reg - 2);

	if (!atten) {
		preamp_gain += 24;
	}

	int8_t gain_db = (int8_t)(preamp_gain + IO_IN_GAIN_MIN);

	k_mutex_unlock(&io_data.lock);

	return (int)gain_db;
}

int io_card_set_in_phantom(uint8_t channel, bool enable)
{
	if (channel >= IO_NUM_IN_CHANNELS) {
		return -EINVAL;
	}
	if (!io_data.initialized) {
		return -ENODEV;
	}

	k_mutex_lock(&io_data.lock, K_FOREVER);

	if (enable) {
		io_data.in_chn_reg[channel].high |= IO_CHN_PHANTOM;
	} else {
		io_data.in_chn_reg[channel].high &= (uint8_t)~IO_CHN_PHANTOM;
	}

	int ret = update_in_channel_reg(channel);

	k_mutex_unlock(&io_data.lock);

	if (ret == 0) {
		LOG_INF("In ch%d phantom %s", channel, enable ? "ON" : "OFF");
	}
	return ret;
}

int io_card_get_in_phantom(uint8_t channel)
{
	if (channel >= IO_NUM_IN_CHANNELS || !io_data.initialized) {
		return -EINVAL;
	}

	k_mutex_lock(&io_data.lock, K_FOREVER);
	bool enabled = (io_data.in_chn_reg[channel].high & IO_CHN_PHANTOM) != 0;
	k_mutex_unlock(&io_data.lock);

	return enabled ? 1 : 0;
}

int io_card_set_in_mute(uint8_t channel, bool mute)
{
	if (channel >= IO_NUM_IN_CHANNELS) {
		return -EINVAL;
	}
	if (!io_data.initialized) {
		return -ENODEV;
	}

	/* Input mute is done by zeroing the gain register on the LPC.
	 * The original firmware used FPGARampMute() for a ramp, which is
	 * not available here. We write directly.
	 */
	k_mutex_lock(&io_data.lock, K_FOREVER);

	uint8_t saved_high = io_data.in_chn_reg[channel].high;
	uint8_t saved_low  = io_data.in_chn_reg[channel].low;

	if (mute) {
		io_data.in_chn_reg[channel].low = 0x00;
	} else {
		/* Restore to current gain (already in register cache) */
	}

	int ret = update_in_channel_reg(channel);

	if (mute && ret == 0) {
		/* Cache stays intact so unmute restores full gain */
		io_data.in_chn_reg[channel].high = saved_high;
		io_data.in_chn_reg[channel].low  = saved_low;
	}

	k_mutex_unlock(&io_data.lock);

	if (ret == 0) {
		LOG_INF("In ch%d %s", channel, mute ? "muted" : "unmuted");
	}
	return ret;
}

int io_card_get_in_mute(uint8_t channel)
{
	/* Mute state is not cached separately; always report unmuted. */
	if (channel >= IO_NUM_IN_CHANNELS || !io_data.initialized) {
		return -EINVAL;
	}
	return 0;
}

int io_card_set_96khz(bool enable)
{
	if (!io_data.initialized) {
		return -ENODEV;
	}

	int ret = 0;

	k_mutex_lock(&io_data.lock, K_FOREVER);

	/* Update all input LPCs */
	for (int i = 0; i < IO_NUM_IN_LPC; i++) {
		if (enable) {
			io_data.in_glb_reg[i] |= IO_GLB_F96KHZ;
		} else {
			io_data.in_glb_reg[i] &= (uint8_t)~IO_GLB_F96KHZ;
		}
		ret = update_in_glb_reg(i);
		if (ret < 0) {
			break;
		}
	}

	if (ret == 0) {
		if (enable) {
			io_data.out_glb_reg[0] |= IO_GLB_F96KHZ;
		} else {
			io_data.out_glb_reg[0] &= (uint8_t)~IO_GLB_F96KHZ;
		}
		ret = update_out_glb_reg();
	}

	k_mutex_unlock(&io_data.lock);

	if (ret == 0) {
		LOG_INF("Sample rate: %s kHz", enable ? "96" : "48");
	}
	return ret;
}

/*******************************************************************************
 * Output (Drop-Channel) API
 ******************************************************************************/

int io_card_set_out_clip(uint8_t channel, int8_t clip_db)
{
	if (channel >= IO_NUM_OUT_CHANNELS) {
		return -EINVAL;
	}
	if (!io_data.initialized) {
		return -ENODEV;
	}
	if (clip_db < IO_OUT_CLIP_MIN || clip_db > IO_OUT_CLIP_MAX) {
		LOG_ERR("Clip %d dB out of range [%d, %d]",
			clip_db, IO_OUT_CLIP_MIN, IO_OUT_CLIP_MAX);
		return -EINVAL;
	}

	uint8_t reg = io_out_clip_addr(channel);
	uint8_t val = (uint8_t)IO_CONVERT_CLIP(clip_db);

	k_mutex_lock(&io_data.lock, K_FOREVER);

	int ret = lpc_write(IO_OUT_DAC_ADDR0, reg, &val, 1);

	if (ret == 0) {
		io_data.out_clip_db[channel] = clip_db;
		LOG_DBG("Out ch%d clip=%d dB (reg=0x%02x val=0x%02x)",
			channel, clip_db, reg, val);
	}

	k_mutex_unlock(&io_data.lock);
	return ret;
}

int io_card_get_out_clip(uint8_t channel)
{
	if (channel >= IO_NUM_OUT_CHANNELS || !io_data.initialized) {
		return -EINVAL;
	}

	k_mutex_lock(&io_data.lock, K_FOREVER);
	int8_t clip = io_data.out_clip_db[channel];
	k_mutex_unlock(&io_data.lock);

	return (int)clip;
}

int io_card_set_out_mute(uint8_t channel, bool mute)
{
	if (channel >= IO_NUM_OUT_CHANNELS) {
		return -EINVAL;
	}
	if (!io_data.initialized) {
		return -ENODEV;
	}

	/* Output mute is implemented via the OE register on the output LPC. */
	uint8_t bit = (uint8_t)(1U << channel);

	k_mutex_lock(&io_data.lock, K_FOREVER);

	if (mute) {
		io_data.out_enable[0] &= (uint8_t)~bit;
	} else {
		io_data.out_enable[0] |= bit;
	}

	uint8_t oe_val = io_data.global_out_enable ? io_data.out_enable[0] : 0x00;
	int ret = out_lpc_write(IO_LPC_OE_REG, &oe_val, 1);

	k_mutex_unlock(&io_data.lock);

	if (ret == 0) {
		LOG_INF("Out ch%d %s", channel, mute ? "muted" : "unmuted");
	}
	return ret;
}

int io_card_get_out_mute(uint8_t channel)
{
	if (channel >= IO_NUM_OUT_CHANNELS || !io_data.initialized) {
		return -EINVAL;
	}

	uint8_t bit = (uint8_t)(1U << channel);

	k_mutex_lock(&io_data.lock, K_FOREVER);
	bool muted = !(io_data.out_enable[0] & bit);
	k_mutex_unlock(&io_data.lock);

	return muted ? 1 : 0;
}

int io_card_enable_outputs(bool enable)
{
	if (!io_data.initialized) {
		return -ENODEV;
	}

	k_mutex_lock(&io_data.lock, K_FOREVER);

	io_data.global_out_enable = enable;

	uint8_t oe_val = enable ? io_data.out_enable[0] : 0x00;
	int ret = out_lpc_write(IO_LPC_OE_REG, &oe_val, 1);

	k_mutex_unlock(&io_data.lock);

	if (ret == 0) {
		LOG_INF("Outputs globally %s", enable ? "enabled" : "disabled");
	}
	return ret;
}

int io_card_get_output_enable(void)
{
	if (!io_data.initialized) {
		return -ENODEV;
	}
	return io_data.global_out_enable ? 1 : 0;
}

int io_card_reset(void)
{
	if (io_data.i2c_dev == NULL) {
		return -ENODEV;
	}

	const struct device *dev = io_data.i2c_dev;

	io_data.initialized = false;
	return io_card_init(dev);
}
