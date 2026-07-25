/*
 * Persistent analog-card settings (gain / 48V phantom / mute / clip).
 *
 * Shadow store between the card drivers (mi/lo/io) and the JSON config
 * persistence (flash/SD):
 *
 *  - capture: reads the live driver state into the store — done before
 *    every config serialization, so the saved config reflects the state
 *    at save time. Skipped (store kept) while the card isn't ready,
 *    e.g. an LO card before its PTP-lock activation.
 *  - apply:   pushes the store into the drivers — done when the stored
 *    config is loaded and again when the card becomes ready (detect /
 *    activate), whichever happens last.
 *  - schedule_save: debounced persist for the web API's card endpoints,
 *    so dragging a gain slider doesn't erase a flash sector per step.
 *
 * Settings are tagged with the card type they belong to; a different
 * card in the slot ignores them.
 */

#ifndef CARD_SETTINGS_H_
#define CARD_SETTINGS_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CARD_SETTINGS_MAX_IN   16   /* IO card: 16 input channels */
#define CARD_SETTINGS_MAX_OUT  8    /* LO/IO card: 8 output channels */

struct card_settings {
	bool    valid;       /* loaded from config or captured at least once */
	uint8_t card_type;   /* card_type_t the settings belong to */
	uint8_t num_in;      /* used input channels (MI 8, IO 16, LO 0) */
	uint8_t num_out;     /* used output channels (LO/IO 8, MI 0) */
	int8_t  in_gain[CARD_SETTINGS_MAX_IN];     /* preamp gain (dB) */
	bool    in_phantom[CARD_SETTINGS_MAX_IN];  /* 48V phantom power */
	bool    in_mute[CARD_SETTINGS_MAX_IN];
	int8_t  out_gain[CARD_SETTINGS_MAX_OUT];   /* LO/IO clip level (dB) */
	bool    out_mute[CARD_SETTINGS_MAX_OUT];
};

/**
 * @brief Access the store. Hold the lock while reading/writing it.
 */
struct card_settings *card_settings_get(void);
void card_settings_lock(void);
void card_settings_unlock(void);

/**
 * @brief Read the live card state into the store.
 *
 * @return 0 on success, -ENODEV if no card is present, -EAGAIN if the
 *         card isn't ready yet (store left untouched).
 */
int card_settings_capture(void);

/**
 * @brief Push the stored settings into the card drivers.
 *
 * No-op if the store is empty, no card is present, the card type does
 * not match, or the card isn't ready yet.
 *
 * @return 0 (best-effort; individual channel failures are logged)
 */
int card_settings_apply(void);

/**
 * @brief Persist the configuration soon (debounced, ~2 s after the
 *        last call). Captures at save time via config serialization.
 */
void card_settings_schedule_save(void);

#ifdef __cplusplus
}
#endif

#endif /* CARD_SETTINGS_H_ */
