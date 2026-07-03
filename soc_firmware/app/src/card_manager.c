/*
 * Card Manager - I2C board detection and unified initialisation
 *
 * Detection protocol:
 *   On a floating or weakly-pulled bus the I²C core can sample SDA low
 *   during the ACK slot and report ACK for many addresses even with
 *   nothing connected.  Therefore we NEVER rely on a simple ACK-based
 *   scan for detection decisions.
 *
 *   Instead, detection works by content verification:
 *     1. Attempt i2c_write_read() to the primary LPC address (0x40).
 *     2. Read 3 bytes from register 0x70 (SOFT_ID / BOARD_ID / HARD_REV).
 *     3. board_id must be one of the known values (0x01, 0x02, 0x31)
 *        AND must not be 0x00 or 0xFF (floating bus artefacts).
 *     4. Only if that passes: call the matching driver init().
 *
 *   A separate informational bus scan is still performed for the web UI
 *   (GET /api/cards), but its results are NEVER used for detection.
 */

#include "card_manager.h"

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include "../drivers/fpga_hal/fpga_hal.h"

#ifdef CONFIG_MI_CARD
#include "../drivers/mi_card/mi_card.h"
#endif
#ifdef CONFIG_LO_CARD
#include "../drivers/lo_card/lo_card.h"
#endif
#ifdef CONFIG_IO_CARD
#include "../drivers/io_card/io_card.h"
#endif

LOG_MODULE_REGISTER(card_manager, LOG_LEVEL_INF);

/* LPC register map shared by all card types */
#define LPC_PRIMARY_ADDR    0x40
#define LPC_REG_SOFT_ID     0x70    /* 3 bytes: soft_id, board_id, hard_rev */

/* Board IDs */
#define BOARD_ID_MI         0x01
#define BOARD_ID_LO         0x02
#define BOARD_ID_IO_IN      0x31    /* IO-card input LPC (primary detection point) */

/* I2C scan range */
#define I2C_SCAN_FIRST      0x08
#define I2C_SCAN_LAST       0x77

/* -----------------------------------------------------------------------
 * Module state
 * --------------------------------------------------------------------- */

static const struct device     *s_i2c_dev;
static struct card_info         s_cards[CARD_MAX_SLOTS];
static struct card_i2c_scan_result s_scan;
static struct k_mutex           s_lock;

/* -----------------------------------------------------------------------
 * Helpers
 * --------------------------------------------------------------------- */

static int lpc_read_ident(uint8_t addr, struct card_ident *out)
{
	uint8_t reg = LPC_REG_SOFT_ID;
	uint8_t buf[3];

	int ret = i2c_write_read(s_i2c_dev, addr, &reg, 1, buf, 3);

	if (ret < 0) {
		return ret;
	}

	out->lpc_addr = addr;
	out->soft_id  = buf[0];
	out->board_id = buf[1];
	out->hard_rev = buf[2];
	return 0;
}

/*
 * Informational bus scan for the web UI.
 *
 * IMPORTANT: results are NEVER used for detection decisions.
 *
 * When SDA floats (no device, weak pull-up, or bus not driven) the I²C
 * core can sample 0 during the ACK slot and report a false ACK.  This
 * causes large runs of false-positive addresses (typically 0x08–0x1F or
 * similar) even with nothing connected.
 *
 * We therefore perform the scan only to populate the informational list
 * shown in the web UI.  We additionally verify each hit with a 1-byte
 * register read to weed out the worst false positives — but we still
 * cannot guarantee accuracy.  Detection itself uses content verification
 * (see detect_and_init_slot0).
 */
static void do_scan(void)
{
	s_scan.count = 0;

	/*
	 * Only probe the specific addresses we actually expect hardware at.
	 * This avoids the false-ACK storm and keeps scan time short.
	 */
	static const uint8_t known_addrs[] = {
		/* Input LPCs */
		0x40, 0x41,
		/* Unknown device seen at 0x42 on some boards */
		0x42,
		/* Output LPC (behind MUX, may not respond directly) */
		0x43,
		/* ADC chips (CS5368): AD1=0 → 0x4C/0x4D, AD1=1 → 0x4E/0x4F */
		0x4C, 0x4D, 0x4E, 0x4F,
		/* DAC chips */
		0x18, 0x19,
		/* MUX variants: PCA9540B/TCA9543A (0x70-0x73), PCA9542 (0x70) */
		0x70, 0x71, 0x72, 0x73,
		/* Si5351A, SSD1306 (same shared bus as the cards) */
		0x60, 0x3C,
		/* DSP (AD1941) */
		0x14,
	};

	for (size_t i = 0; i < ARRAY_SIZE(known_addrs); i++) {
		uint8_t addr = known_addrs[i];
		uint8_t dummy;

		if (i2c_read(s_i2c_dev, &dummy, 1, addr) == 0) {
			if (s_scan.count < CARD_MGR_MAX_I2C_DEVICES) {
				s_scan.addr[s_scan.count++] = addr;
			}
			LOG_DBG("I2C scan: device at 0x%02x", addr);
		}
	}

	LOG_INF("I2C scan complete: %d device(s) found (probed known addresses only)",
		s_scan.count);
}

/* -----------------------------------------------------------------------
 * Detection & init for slot 0
 * --------------------------------------------------------------------- */

/*
 * Valid board_id values.  0x00 and 0xFF are floating-bus artefacts and
 * must be rejected even if the i2c_write_read() call returned 0.
 */
static bool board_id_is_known(uint8_t id)
{
	return id == BOARD_ID_MI || id == BOARD_ID_LO || id == BOARD_ID_IO_IN;
}

static int detect_and_init_slot0(void)
{
	struct card_info *card = &s_cards[CARD_SLOT_MAIN];

	card->present     = false;
	card->initialized = false;
	card->type        = CARD_TYPE_NONE;
	memset(&card->ident, 0, sizeof(card->ident));

	/*
	 * Detection by content verification — NOT by ACK scan.
	 *
	 * Attempt to read the identity registers from the primary LPC address.
	 * We read twice and require consistent results to guard against
	 * single-bit glitches on a floating bus.
	 */
	struct card_ident ident_a, ident_b;

	if (lpc_read_ident(LPC_PRIMARY_ADDR, &ident_a) < 0) {
		LOG_INF("No card detected (no response at 0x%02x)", LPC_PRIMARY_ADDR);
		return 0;
	}

	/* Reject bus-float values */
	if (!board_id_is_known(ident_a.board_id)) {
		LOG_INF("No card detected (board_id=0x%02x not recognised — "
			"floating bus?)", ident_a.board_id);
		return 0;
	}

	/* Second read — must match first */
	if (lpc_read_ident(LPC_PRIMARY_ADDR, &ident_b) < 0 ||
	    ident_b.board_id != ident_a.board_id) {
		LOG_WRN("Board ID unstable (0x%02x vs 0x%02x) — bus noise, "
			"skipping init", ident_a.board_id, ident_b.board_id);
		return 0;
	}

	card->ident   = ident_a;
	card->present = true;

	LOG_INF("LPC at 0x%02x: soft_id=0x%02x board_id=0x%02x rev=0x%02x",
		ident_a.lpc_addr, ident_a.soft_id, ident_a.board_id, ident_a.hard_rev);

	int ret = 0;

	switch (ident_a.board_id) {
#ifdef CONFIG_MI_CARD
	case BOARD_ID_MI:
		card->type = CARD_TYPE_MI;
		LOG_INF("Detected: MI card (8-ch ADC preamp)");
		ret = mi_card_init(s_i2c_dev);
		break;
#endif
#ifdef CONFIG_LO_CARD
	case BOARD_ID_LO:
		card->type = CARD_TYPE_LO;
		LOG_INF("Detected: LO card (8-ch DAC output)");
		ret = lo_card_init(s_i2c_dev);
		break;
#endif
#ifdef CONFIG_IO_CARD
	case BOARD_ID_IO_IN:
		card->type = CARD_TYPE_IO;
		LOG_INF("Detected: IO card (16-ch ADC + 8-ch DAC)");
		ret = io_card_init(s_i2c_dev);
		break;
#endif
	default:
		LOG_WRN("Unknown board_id=0x%02x at 0x40 — not initialised",
			ident_a.board_id);
		return 0;
	}

	if (ret < 0) {
		LOG_ERR("Card init failed: %d", ret);
		card->initialized = false;
	} else {
		card->initialized = true;
		LOG_INF("Card initialised: %s", card_type_name(card->type));
	}

	return ret;
}

/* -----------------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------------- */

int card_manager_init(const struct device *i2c_dev)
{
	if (i2c_dev == NULL || !device_is_ready(i2c_dev)) {
		LOG_ERR("I2C device not ready");
		return -ENODEV;
	}

	k_mutex_init(&s_lock);
	s_i2c_dev = i2c_dev;

	/* nRST has been held LOW since early boot (set in main()).
	 * MCLK should be stable by now (Si5351A + 2 s settling).
	 * Release nRST and give all card ICs time to initialise. */
	LOG_INF("Card manager: releasing ADDA nRST...");
	fpga_hal_set_adda_nrst(true);
	k_msleep(200);  /* wait for all card ICs to come out of reset before I2C scan */
	LOG_INF("Card manager: ADDA nRST released");

	return card_manager_rescan();
}

int card_manager_rescan(void)
{
	if (s_i2c_dev == NULL) {
		return -ENODEV;
	}

	k_mutex_lock(&s_lock, K_FOREVER);

	/*
	 * Step 1: informational scan of known addresses (for the web UI).
	 * Results are NOT used for detection — see do_scan() comment.
	 */
	LOG_INF("Card manager: probing known I2C addresses...");
	do_scan();

	/*
	 * Step 2: content-based detection.
	 * Reads and validates LPC identity registers directly.
	 */
	int ret = detect_and_init_slot0();

	k_mutex_unlock(&s_lock);
	return ret;
}

const struct card_info *card_manager_get_info(int slot)
{
	if (slot < 0 || slot >= CARD_MAX_SLOTS) {
		return NULL;
	}
	return &s_cards[slot];
}

void card_manager_get_scan_result(struct card_i2c_scan_result *out)
{
	if (out == NULL) {
		return;
	}
	k_mutex_lock(&s_lock, K_FOREVER);
	*out = s_scan;
	k_mutex_unlock(&s_lock);
}

const char *card_type_name(card_type_t type)
{
	switch (type) {
	case CARD_TYPE_MI:  return "MI (8-ch ADC preamp)";
	case CARD_TYPE_LO:  return "LO (8-ch DAC output)";
	case CARD_TYPE_IO:  return "IO (16-ch ADC + 8-ch DAC)";
	default:            return "none";
	}
}
