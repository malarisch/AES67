/*
 * Output Card Driver - 8-Channel DAC Line Output Board
 *

 *
 */

#include "lo_card.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <string.h>

#if defined(CONFIG_LO_CARD_NRST_GPIO)
#include <zephyr/drivers/gpio.h>
#define LO_NRST_NODE DT_ALIAS(lo_nrst)
#if DT_NODE_EXISTS(LO_NRST_NODE)
static const struct gpio_dt_spec lo_nrst_gpio = GPIO_DT_SPEC_GET(LO_NRST_NODE, gpios);
#define LO_NRST_GPIO_VALID 1
#else
#define LO_NRST_GPIO_VALID 0
#endif
#endif /* CONFIG_LO_CARD_NRST_GPIO */

LOG_MODULE_REGISTER(lo_card, CONFIG_LO_CARD_LOG_LEVEL);

/* Driver instance data */
static struct lo_card_data lo_data = {
	.i2c_dev = NULL,
	.glb_reg = 0,
	.output_enable = 0,
	.global_enable = false,
	.initialized = false,
};

#if defined(CONFIG_LO_CARD_NRST_GPIO) && LO_NRST_GPIO_VALID
static bool nrst_gpio_ready = false;
#endif

/*******************************************************************************
 * Internal helper functions
 ******************************************************************************/

/**
 * @brief Write to LPC register with retry and optional verify (1-byte address)
 */
static int lpc_write(uint8_t reg, const uint8_t *data, size_t len)
{
	uint8_t buf[16];
	int ret;

	if (len > sizeof(buf) - 1) {
		return -EINVAL;
	}

	buf[0] = reg;
	memcpy(&buf[1], data, len);

	for (int try = 0; try < CONFIG_LO_CARD_I2C_RETRIES; try++) {
		ret = i2c_write(lo_data.i2c_dev, buf, len + 1, LO_LPC_ADDR);
		if (ret < 0) {
			LOG_WRN("LPC write retry %d: reg=0x%02x, err=%d", try, reg, ret);
			k_sleep(K_MSEC(CONFIG_LO_CARD_I2C_RETRY_DELAY_MS));
			continue;
		}

#if defined(CONFIG_LO_CARD_I2C_VERIFY_WRITES)
		/* Verify by read-back */
		uint8_t verify[16];
		ret = i2c_write_read(lo_data.i2c_dev, LO_LPC_ADDR,
				     &reg, 1, verify, len);
		if (ret < 0 || memcmp(data, verify, len) != 0) {
			LOG_WRN("LPC verify failed: reg=0x%02x, try=%d", reg, try);
			k_sleep(K_MSEC(CONFIG_LO_CARD_I2C_RETRY_DELAY_MS));
			continue;
		}
#endif
		return 0; /* Success */
	}

	LOG_ERR("LPC write failed after %d retries: reg=0x%02x",
		CONFIG_LO_CARD_I2C_RETRIES, reg);
	return ret < 0 ? ret : -EIO;
}

/**
 * @brief Read from LPC register with retry (1-byte address)
 */
static int lpc_read(uint8_t reg, uint8_t *data, size_t len)
{
	int ret;

	for (int try = 0; try < CONFIG_LO_CARD_I2C_RETRIES; try++) {
		ret = i2c_write_read(lo_data.i2c_dev, LO_LPC_ADDR,
				     &reg, 1, data, len);
		if (ret == 0) {
			return 0;
		}
		LOG_WRN("LPC read retry %d: reg=0x%02x, err=%d", try, reg, ret);
		k_sleep(K_MSEC(CONFIG_LO_CARD_I2C_RETRY_DELAY_MS));
	}

	LOG_ERR("LPC read failed after %d retries: reg=0x%02x",
		CONFIG_LO_CARD_I2C_RETRIES, reg);
	return ret;
}

/**
 * @brief Write to DAC register with retry and optional verify (1-byte address)
 *
 * Note: MISC register (0x08) cannot be verified because it contains
 * hardware status bits that differ from what was written.
 */
static int dac_write(uint8_t dac_addr, uint8_t reg, uint8_t data)
{
	uint8_t buf[2] = { reg, data };
	int ret;
	bool can_verify = (reg != LO_DAC_MISC_REG); /* MISC has status bits */

	for (int try = 0; try < CONFIG_LO_CARD_I2C_RETRIES; try++) {
		ret = i2c_write(lo_data.i2c_dev, buf, 2, dac_addr);
		if (ret < 0) {
			LOG_WRN("DAC write retry %d: dev=0x%02x reg=0x%02x, err=%d",
				try, dac_addr, reg, ret);
			k_sleep(K_MSEC(CONFIG_LO_CARD_I2C_RETRY_DELAY_MS));
			continue;
		}

#if defined(CONFIG_LO_CARD_I2C_VERIFY_WRITES)
		/* Verify by read-back (skip registers with status bits) */
		if (can_verify) {
			uint8_t verify;
			ret = i2c_write_read(lo_data.i2c_dev, dac_addr,
					     &reg, 1, &verify, 1);
			if (ret < 0 || verify != data) {
				LOG_WRN("DAC verify failed: dev=0x%02x reg=0x%02x (wrote=0x%02x, read=0x%02x)",
					dac_addr, reg, data, verify);
				k_sleep(K_MSEC(CONFIG_LO_CARD_I2C_RETRY_DELAY_MS));
				continue;
			}
		}
#endif
		return 0; /* Success */
	}

	LOG_ERR("DAC write failed after %d retries: dev=0x%02x reg=0x%02x",
		CONFIG_LO_CARD_I2C_RETRIES, dac_addr, reg);
	return ret < 0 ? ret : -EIO;
}

/**
 * @brief Write to DSP register (2-byte address)
 */
static int dsp_write(uint16_t reg, const uint8_t *data, size_t len)
{
	uint8_t buf[16];

	if (len > sizeof(buf) - 2) {
		return -EINVAL;
	}

	/* DSP uses big-endian 16-bit register addresses */
	buf[0] = (reg >> 8) & 0xFF;
	buf[1] = reg & 0xFF;
	memcpy(&buf[2], data, len);

	int ret = i2c_write(lo_data.i2c_dev, buf, len + 2, LO_DSP_ADDR);
	if (ret < 0) {
		LOG_ERR("DSP write failed: reg=0x%04x, err=%d", reg, ret);
	}
	return ret;
}

/**
 * @brief Update output enable register on LPC
 */
static int update_output_enable(void)
{
	uint8_t data;

	if (lo_data.global_enable) {
		data = lo_data.output_enable;
	} else {
		data = 0;
	}

	return lpc_write(LO_LPC_OE_REG, &data, 1);
}

/**
 * @brief Update global register on LPC
 */
static int update_global_reg(void)
{
	return lpc_write(LO_LPC_GLB_REG, &lo_data.glb_reg, 1);
}

/**
 * @brief Get DAC address and volume register for a given channel
 */
static void channel_to_dac(uint8_t channel, uint8_t *dac_addr, uint8_t *vol_reg)
{
	*dac_addr = LO_DAC_DEV0 + (channel / 2);
	*vol_reg = (channel & 1) ? LO_DAC_VOL_B_REG : LO_DAC_VOL_A_REG;
}

/*******************************************************************************
 * Public API
 ******************************************************************************/

int lo_card_early_mute(const struct device *i2c_dev)
{
	/* Standalone boot-path helper: runs before lo_card_init() (no mutex,
	 * no module state), straight after the shared nRST release. The LPC
	 * powers up with the converters live and the DSP has no clock until
	 * the FPGA is configured — kill the noise path immediately. */
	const uint8_t glb_reset[2] = { LO_LPC_GLB_REG, 0x00 };
	const uint8_t oe_mute[2]   = { LO_LPC_OE_REG, 0x00 };
	int ret = -ENODEV;

	if (i2c_dev == NULL || !device_is_ready(i2c_dev)) {
		return -ENODEV;
	}

	for (int try = 0; try < 3; try++) {
		ret = i2c_write(i2c_dev, oe_mute, sizeof(oe_mute), LO_LPC_ADDR);
		if (ret == 0) {
			break;
		}
		k_sleep(K_MSEC(10));
	}
	if (ret < 0) {
		return -ENODEV;	/* no LO card (or LPC not booted) */
	}

	ret = i2c_write(i2c_dev, glb_reset, sizeof(glb_reset), LO_LPC_ADDR);

	LOG_INF("Early mute: outputs muted, DSP+DACs held in reset");
	return ret;
}

int lo_card_init(const struct device *i2c_dev)
{
	int ret;

	if (i2c_dev == NULL) {
		LOG_ERR("I2C device is NULL");
		return -EINVAL;
	}

	if (!device_is_ready(i2c_dev)) {
		LOG_ERR("I2C device not ready");
		return -ENODEV;
	}

	k_mutex_init(&lo_data.lock);
	k_mutex_lock(&lo_data.lock, K_FOREVER);

	lo_data.i2c_dev = i2c_dev;
	lo_data.output_enable = 0;
	lo_data.global_enable = false;
	lo_data.detected = false;
	memset(lo_data.clip_db, 0, sizeof(lo_data.clip_db));

#if defined(CONFIG_LO_CARD_NRST_GPIO) && LO_NRST_GPIO_VALID
	/* Perform hardware reset before detection */
	if (nrst_gpio_ready) {
		LOG_INF("Performing board hardware reset via nRST GPIO...");

		/* Assert reset (active low) */
		gpio_pin_set_dt(&lo_nrst_gpio, 1);
		k_sleep(K_MSEC(CONFIG_LO_CARD_NRST_PULSE_MS));

		/* Release reset */
		gpio_pin_set_dt(&lo_nrst_gpio, 0);

		/* Wait for board to initialize */
		k_sleep(K_MSEC(CONFIG_LO_CARD_NRST_RECOVERY_MS));
	}
#endif

	/* Check if board is present */
	struct lo_board_info info;
	if (!lo_card_detect(&info)) {
		LOG_WRN("LO card not detected");
		k_mutex_unlock(&lo_data.lock);
		return -ENODEV;
	}

	LOG_INF("LO card detected: soft_id=0x%02x, board_id=0x%02x, rev=0x%02x",
		info.soft_id, info.board_id, info.hard_rev);

	/* ---- Safe state until the media clock is valid ----
	 * Relays muted, DSP + DACs held in the card-level reset. The
	 * converters are only brought up by lo_card_activate(), which the
	 * card manager gates on PTP/wallclock lock. */
	uint8_t oe_off = 0;

	ret = lpc_write(LO_LPC_OE_REG, &oe_off, 1);
	if (ret < 0) {
		LOG_ERR("Failed to mute outputs");
		k_mutex_unlock(&lo_data.lock);
		return ret;
	}

	lo_data.glb_reg = 0;
	ret = update_global_reg();
	if (ret < 0) {
		LOG_ERR("Failed to assert converter reset");
		k_mutex_unlock(&lo_data.lock);
		return ret;
	}

	lo_data.detected = true;
	k_mutex_unlock(&lo_data.lock);

	LOG_INF("LO card in safe state (muted, converters in reset) — "
		"waiting for media clock");
	return 0;
}

int lo_card_activate(void)
{
	int ret;

	k_mutex_lock(&lo_data.lock, K_FOREVER);

	if (!lo_data.detected) {
		k_mutex_unlock(&lo_data.lock);
		return -ENODEV;
	}
	if (lo_data.initialized) {
		k_mutex_unlock(&lo_data.lock);
		return 0;
	}

	LOG_INF("Activating converters (media clock valid)");

	/* ---- Reset sequence (matching MI card / enabling DSP) ----
	 * The DSP (AD1941) requires a proper reset cycle to initialize.
	 * 1. Assert reset (GlbReg = 0) — hold everything in reset
	 * 2. Wait 500ms in reset state
	 * 3. Release reset (GlbReg = NRST)
	 * 4. Wait 500ms for boot
	 */

	/* Step 1: Assert reset (normally already asserted since init) */
	lo_data.glb_reg = 0;
	ret = update_global_reg();
	if (ret < 0) {
		LOG_ERR("Failed to assert reset");
		k_mutex_unlock(&lo_data.lock);
		return ret;
	}
	LOG_DBG("Asserted reset (GlbReg=0x00)");

	/* Step 2: Wait in reset state */
	k_sleep(K_MSEC(500));

	/* Step 3: Release from reset */
	lo_data.glb_reg = LO_GLB_NRST;
	ret = update_global_reg();
	if (ret < 0) {
		k_mutex_unlock(&lo_data.lock);
		return ret;
	}
	LOG_DBG("Released reset (GlbReg=0x%02x)", lo_data.glb_reg);

	/* Step 4: Wait after reset release for DSP boot */
	k_sleep(K_MSEC(500));

	/* All channels unmuted (output_enable bits set) */
	lo_data.output_enable = 0xFF;

	/* Keep outputs globally disabled — the caller unmutes via
	 * lo_card_enable_outputs(true) once activation succeeded. */
	uint8_t oe_off = 0;
	ret = lpc_write(LO_LPC_OE_REG, &oe_off, 1);
	if (ret < 0) {
		LOG_ERR("Failed to disable outputs");
		k_mutex_unlock(&lo_data.lock);
		return ret;
	}

	/* ---- Initialize 4x DAC devices ---- */
	for (uint8_t addr = LO_DAC_DEV0; addr <= LO_DAC_DEV3; addr++) {
		/* Miscellaneous control: 0x40 */
		ret = dac_write(addr, LO_DAC_MISC_REG, 0x40);
		if (ret < 0) {
			LOG_ERR("DAC 0x%02x misc init failed", addr);
			k_mutex_unlock(&lo_data.lock);
			return ret;
		}

		/* Mode control: 0x10 = 48 kHz */
		ret = dac_write(addr, LO_DAC_MODE_REG, 0x10);
		if (ret < 0) {
			LOG_ERR("DAC 0x%02x mode init failed", addr);
			k_mutex_unlock(&lo_data.lock);
			return ret;
		}

		/* Clip levels: default 0 dB for both channels */
		uint8_t clip_val = (uint8_t)LO_CONVERT_CLIP(0);
		uint8_t ix = (addr - LO_DAC_DEV0) * 2;

		ret = dac_write(addr, LO_DAC_VOL_A_REG, clip_val);
		if (ret < 0) {
			LOG_ERR("DAC 0x%02x vol A init failed", addr);
			k_mutex_unlock(&lo_data.lock);
			return ret;
		}
		lo_data.clip_db[ix] = 0;

		ret = dac_write(addr, LO_DAC_VOL_B_REG, clip_val);
		if (ret < 0) {
			LOG_ERR("DAC 0x%02x vol B init failed", addr);
			k_mutex_unlock(&lo_data.lock);
			return ret;
		}
		lo_data.clip_db[ix + 1] = 0;
	}

	LOG_INF("4x DAC devices initialized (0x4C-0x4F)");

	/* ---- Initialize DSP (AD1941) ----
	 * The AD1941 DSP takes time to boot after reset. It needs MCLK to be
	 * running and may take 200-500ms after power-up before responding to I2C.
	 * Wait extra time here for the DSP to complete self-boot. */
	LOG_INF("Waiting for DSP boot...");
	k_sleep(K_MSEC(500));

	/* Reference firmware does fire-and-forget DSP writes (no error check).
	 * The DSP may not ACK on all boards — treat failures as non-fatal.
	 * Use retries (reference uses 5 retries for DSP writes). */
	uint8_t dsp_buf[2];
	int dsp_ret;
	bool dsp_ok = true;

	/* Serial Input Control = 0x00 */
	dsp_buf[0] = 0x00;
	for (int try = 0; try < 5; try++) {
		dsp_ret = dsp_write(LO_DSP_SER_IN, dsp_buf, 1);
		if (dsp_ret == 0) {
			break;
		}
		k_sleep(K_MSEC(10));
	}
	if (dsp_ret != 0) {
		LOG_WRN("DSP SER_IN init failed (err=%d) — DSP may not be present", dsp_ret);
		dsp_ok = false;
	}

	/* Serial Output Control = 0x0000 */
	dsp_buf[0] = 0x00;
	dsp_buf[1] = 0x00;
	for (int try = 0; try < 5; try++) {
		dsp_ret = dsp_write(LO_DSP_SER_OUT, dsp_buf, 2);
		if (dsp_ret == 0) {
			break;
		}
		k_sleep(K_MSEC(10));
	}
	if (dsp_ret != 0 && dsp_ok) {
		LOG_WRN("DSP SER_OUT init failed (err=%d)", dsp_ret);
		dsp_ok = false;
	}

	/* Core Control Register
	 * Reference firmware writes Data=0x0102 as (UINT8*)&Data with Length=2.
	 * On little-endian ARM this sends bytes 0x02, 0x01. */
	dsp_buf[0] = 0x02;
	dsp_buf[1] = 0x01;
	for (int try = 0; try < 5; try++) {
		dsp_ret = dsp_write(LO_DSP_CORE_CTRL, dsp_buf, 2);
		if (dsp_ret == 0) {
			break;
		}
		k_sleep(K_MSEC(10));
	}
	if (dsp_ret != 0 && dsp_ok) {
		LOG_WRN("DSP CORE_CTRL init failed (err=%d)", dsp_ret);
		dsp_ok = false;
	}

	if (dsp_ok) {
		LOG_INF("DSP (AD1941) initialized at 0x%02x", LO_DSP_ADDR);
	} else {
		LOG_WRN("DSP init failed — card will operate without DSP");
	}

	lo_data.initialized = true;
	k_mutex_unlock(&lo_data.lock);

	LOG_INF("LO card activated (outputs still muted)");
	return 0;
}

bool lo_card_detect(struct lo_board_info *info)
{
	uint8_t buf[3];
	int ret;

	if (lo_data.i2c_dev == NULL) {
		return false;
	}

	ret = lpc_read(LO_LPC_SOFT_ID, buf, 3);
	if (ret < 0) {
		return false;
	}

	/* Board ID should be 0x02 for LO card */
	if (buf[1] != LO_BOARD_ID_EXPECTED) {
		LOG_DBG("Unexpected board_id: 0x%02x (expected 0x%02x)",
			buf[1], LO_BOARD_ID_EXPECTED);
		return false;
	}

	if (info != NULL) {
		info->soft_id = buf[0];
		info->board_id = buf[1];
		info->hard_rev = buf[2];
	}

	return true;
}

int lo_card_set_clip(uint8_t channel, int8_t clip_db)
{
	int ret;

	if (channel >= LO_NUM_CHANNELS) {
		return -EINVAL;
	}

	if (clip_db < LO_CLIP_MIN || clip_db > LO_CLIP_MAX) {
		LOG_ERR("Clip %d dB out of range [%d, %d]",
			clip_db, LO_CLIP_MIN, LO_CLIP_MAX);
		return -EINVAL;
	}

	if (!lo_data.initialized) {
		return -ENODEV;
	}

	uint8_t dac_addr, vol_reg;
	channel_to_dac(channel, &dac_addr, &vol_reg);

	uint8_t clip_val = (uint8_t)LO_CONVERT_CLIP(clip_db);

	k_mutex_lock(&lo_data.lock, K_FOREVER);

	ret = dac_write(dac_addr, vol_reg, clip_val);
	if (ret == 0) {
		lo_data.clip_db[channel] = clip_db;
		LOG_DBG("Channel %d clip set to %d dB (reg: 0x%02x)",
			channel, clip_db, clip_val);
	}

	k_mutex_unlock(&lo_data.lock);

	return ret;
}

int lo_card_get_clip(uint8_t channel)
{
	if (channel >= LO_NUM_CHANNELS) {
		return -EINVAL;
	}

	if (!lo_data.initialized) {
		return -ENODEV;
	}

	k_mutex_lock(&lo_data.lock, K_FOREVER);
	int8_t clip = lo_data.clip_db[channel];
	k_mutex_unlock(&lo_data.lock);

	return clip;
}

int lo_card_set_mute(uint8_t channel, bool mute)
{
	int ret;

	if (channel >= LO_NUM_CHANNELS) {
		return -EINVAL;
	}

	if (!lo_data.initialized) {
		return -ENODEV;
	}

	k_mutex_lock(&lo_data.lock, K_FOREVER);

	if (mute) {
		lo_data.output_enable &= ~(1 << channel);
	} else {
		lo_data.output_enable |= (1 << channel);
	}

	ret = update_output_enable();

	k_mutex_unlock(&lo_data.lock);

	if (ret == 0) {
		LOG_INF("Channel %d %s", channel, mute ? "muted" : "unmuted");
	}

	return ret;
}

int lo_card_get_mute(uint8_t channel)
{
	if (channel >= LO_NUM_CHANNELS) {
		return -EINVAL;
	}

	if (!lo_data.initialized) {
		return -ENODEV;
	}

	k_mutex_lock(&lo_data.lock, K_FOREVER);
	/* Muted = output enable bit is NOT set */
	bool muted = (lo_data.output_enable & (1 << channel)) == 0;
	k_mutex_unlock(&lo_data.lock);

	return muted ? 1 : 0;
}

int lo_card_enable_outputs(bool enable)
{
	int ret;

	if (!lo_data.initialized) {
		return -ENODEV;
	}

	k_mutex_lock(&lo_data.lock, K_FOREVER);

	lo_data.global_enable = enable;
	ret = update_output_enable();

	k_mutex_unlock(&lo_data.lock);

	if (ret == 0) {
		LOG_INF("Outputs %s", enable ? "enabled" : "disabled");
	}

	return ret;
}

int lo_card_get_output_enable(void)
{
	if (!lo_data.initialized) {
		return -ENODEV;
	}

	k_mutex_lock(&lo_data.lock, K_FOREVER);
	bool enabled = lo_data.global_enable;
	k_mutex_unlock(&lo_data.lock);

	return enabled ? 1 : 0;
}

int lo_card_set_96khz(bool enable)
{
	int ret;

	if (!lo_data.initialized) {
		return -ENODEV;
	}

	k_mutex_lock(&lo_data.lock, K_FOREVER);

	/* Update global register: NRST always set, HPF preserved, toggle F96KHZ */
	if (enable) {
		lo_data.glb_reg |= LO_GLB_F96KHZ;
	} else {
		lo_data.glb_reg &= ~LO_GLB_F96KHZ;
	}

	ret = update_global_reg();
	if (ret < 0) {
		k_mutex_unlock(&lo_data.lock);
		return ret;
	}

	/* Update all 4 DAC mode registers */
	uint8_t mode = enable ? 0x11 : 0x10;
	for (uint8_t addr = LO_DAC_DEV0; addr <= LO_DAC_DEV3; addr++) {
		ret = dac_write(addr, LO_DAC_MODE_REG, mode);
		if (ret < 0) {
			LOG_ERR("DAC 0x%02x mode update failed", addr);
			k_mutex_unlock(&lo_data.lock);
			return ret;
		}
	}

	k_mutex_unlock(&lo_data.lock);

	if (ret == 0) {
		LOG_INF("Sample rate set to %s kHz", enable ? "96" : "48");
	}

	return ret;
}

int lo_card_get_96khz(void)
{
	if (!lo_data.initialized) {
		return -ENODEV;
	}

	k_mutex_lock(&lo_data.lock, K_FOREVER);
	bool enabled = (lo_data.glb_reg & LO_GLB_F96KHZ) != 0;
	k_mutex_unlock(&lo_data.lock);

	return enabled ? 1 : 0;
}

/* Re-run detect + safe state; if the card was already activated (media
 * clock valid), bring it straight back up to its previous output state. */
static int reinit_preserving_activation(void)
{
	bool was_active = lo_data.initialized;
	bool outputs_on = lo_data.global_enable;
	int ret;

	lo_data.initialized = false;
	ret = lo_card_init(lo_data.i2c_dev);
	if (ret < 0 || !was_active) {
		return ret;
	}

	ret = lo_card_activate();
	if (ret == 0 && outputs_on) {
		ret = lo_card_enable_outputs(true);
	}
	return ret;
}

int lo_card_reset(void)
{
	if (lo_data.i2c_dev == NULL) {
		return -ENODEV;
	}

	return reinit_preserving_activation();
}

#if defined(CONFIG_LO_CARD_NRST_GPIO)
int lo_card_hw_reset(void)
{
#if LO_NRST_GPIO_VALID
	if (!nrst_gpio_ready) {
		LOG_ERR("nRST GPIO not initialized");
		return -ENODEV;
	}

	LOG_INF("Performing hardware reset via nRST GPIO");

	k_mutex_lock(&lo_data.lock, K_FOREVER);

	/* Assert reset (active low, so set to logical 1 with GPIO_ACTIVE_LOW) */
	gpio_pin_set_dt(&lo_nrst_gpio, 1);

	/* Hold reset for configured duration */
	k_sleep(K_MSEC(CONFIG_LO_CARD_NRST_PULSE_MS));

	/* Release reset */
	gpio_pin_set_dt(&lo_nrst_gpio, 0);

	/* Wait for board to recover */
	k_sleep(K_MSEC(CONFIG_LO_CARD_NRST_RECOVERY_MS));

	k_mutex_unlock(&lo_data.lock);

	/* Reinitialize via I2C */
	return reinit_preserving_activation();
#else
	LOG_WRN("nRST GPIO not defined in device tree");
	return -ENOTSUP;
#endif
}

int lo_card_nrst_gpio_init(void)
{
#if LO_NRST_GPIO_VALID
	int ret;

	if (!gpio_is_ready_dt(&lo_nrst_gpio)) {
		LOG_ERR("nRST GPIO device not ready");
		return -ENODEV;
	}

	/* Configure as output, initially deasserted (not in reset) */
	ret = gpio_pin_configure_dt(&lo_nrst_gpio, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		LOG_ERR("Failed to configure nRST GPIO: %d", ret);
		return ret;
	}

	nrst_gpio_ready = true;
	LOG_INF("LO card nRST GPIO initialized");
	return 0;
#else
	LOG_WRN("nRST GPIO not defined in device tree");
	return -ENOTSUP;
#endif
}
#endif /* CONFIG_LO_CARD_NRST_GPIO */
