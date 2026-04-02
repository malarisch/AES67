/*
 * Card Manager - I2C board autodetect and unified card state
 *
 * Scans the I2C bus and identifies which analog I/O boards are connected:
 *   - MI card  (8-ch ADC preamp,  LPC board_id 0x01 at 0x40)
 *   - LO card  (8-ch DAC output,  LPC board_id 0x02 at 0x40)
 *   - IO card  (16-ch ADC + 8-ch DAC, LPC board_id 0x31 at 0x40)
 *
 * After detect, the appropriate driver is initialised.  The manager
 * tracks runtime state so the web server can expose a single unified
 * /api/cards API regardless of which card type is plugged in.
 */

#ifndef CARD_MANAGER_H_
#define CARD_MANAGER_H_

#include <zephyr/device.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -----------------------------------------------------------------------
 * Card types
 * --------------------------------------------------------------------- */

/** Card types discoverable by the manager */
typedef enum {
	CARD_TYPE_NONE  = 0,    /**< No card detected */
	CARD_TYPE_MI    = 1,    /**< 8-ch ADC preamp (MI card) */
	CARD_TYPE_LO    = 2,    /**< 8-ch DAC line output (LO card) */
	CARD_TYPE_IO    = 3,    /**< 16-ch ADC + 8-ch DAC (IO card) */
} card_type_t;

/** Slot index — the system supports one card per slot */
#define CARD_SLOT_MAIN  0
#define CARD_MAX_SLOTS  1

/* -----------------------------------------------------------------------
 * Per-device info stored after detection
 * --------------------------------------------------------------------- */

/** Raw identity info read from a detected LPC board */
struct card_ident {
	uint8_t lpc_addr;   /**< I2C address of the primary LPC */
	uint8_t soft_id;
	uint8_t board_id;
	uint8_t hard_rev;
};

/** Full descriptor for one detected slot */
struct card_info {
	card_type_t   type;
	bool          present;
	bool          initialized;
	struct card_ident ident;
};

/* -----------------------------------------------------------------------
 * I2C device scan result
 * --------------------------------------------------------------------- */

#define CARD_MGR_MAX_I2C_DEVICES 24

struct card_i2c_scan_result {
	uint8_t  addr[CARD_MGR_MAX_I2C_DEVICES];
	uint8_t  count;
};

/* -----------------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------------- */

/**
 * @brief Initialise the card manager.
 *
 * Performs an I2C bus scan, identifies connected boards, and calls the
 * appropriate driver init functions.
 *
 * @param i2c_dev  I2C device to use for card communication.
 * @return 0 on success, negative errno on hard failure.
 */
int card_manager_init(const struct device *i2c_dev);

/**
 * @brief Re-scan the bus and reinitialise all cards.
 *
 * Can be called at runtime (e.g. from the web UI) to recover from a
 * card that was briefly disconnected or to pick up a newly inserted card.
 *
 * @return 0 on success, negative errno on hard failure.
 */
int card_manager_rescan(void);

/**
 * @brief Return info about the card in a given slot.
 *
 * @param slot  Slot index (0 = CARD_SLOT_MAIN).
 * @return Pointer to the card info struct, or NULL for invalid slot.
 */
const struct card_info *card_manager_get_info(int slot);

/**
 * @brief Return the result of the last I2C bus scan.
 *
 * @param out  Pointer to a scan-result struct to fill.
 */
void card_manager_get_scan_result(struct card_i2c_scan_result *out);

/**
 * @brief Human-readable name for a card type.
 */
const char *card_type_name(card_type_t type);

#ifdef __cplusplus
}
#endif

#endif /* CARD_MANAGER_H_ */
