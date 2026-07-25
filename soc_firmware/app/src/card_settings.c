/*
 *
 * Persistent analog-card settings — see card_settings.h for the model.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include "card_settings.h"
#include "card_manager.h"

#ifdef CONFIG_MI_CARD
#include "../drivers/mi_card/mi_card.h"
#endif
#ifdef CONFIG_LO_CARD
#include "../drivers/lo_card/lo_card.h"
#endif
#ifdef CONFIG_IO_CARD
#include "../drivers/io_card/io_card.h"
#endif
#ifdef CONFIG_SD_CONFIG
#include "sd_config.h"
#endif
#ifdef CONFIG_FLASH_CONFIG
#include "flash_config.h"
#endif

LOG_MODULE_REGISTER(card_settings, LOG_LEVEL_INF);

#define MI_NUM_CHANNELS      8
#define LO_OUT_CHANNELS      8
#define IO_IN_CHANNELS       16
#define IO_OUT_CHANNELS      8

static struct card_settings s_store;
static K_MUTEX_DEFINE(s_store_lock);

struct card_settings *card_settings_get(void)
{
	return &s_store;
}

void card_settings_lock(void)
{
	k_mutex_lock(&s_store_lock, K_FOREVER);
}

void card_settings_unlock(void)
{
	k_mutex_unlock(&s_store_lock);
}

/* ================================================================
 * Capture: live driver state -> store
 * ================================================================ */

int card_settings_capture(void)
{
	const struct card_info *card = card_manager_get_info(CARD_SLOT_MAIN);
	struct card_settings tmp;

	if (card == NULL || !card->present) {
		return -ENODEV;
	}

	memset(&tmp, 0, sizeof(tmp));
	tmp.valid = true;
	tmp.card_type = card->type;

	switch (card->type) {
#ifdef CONFIG_MI_CARD
	case CARD_TYPE_MI:
		/* Readiness probe: getters return -ENODEV before init.
		 * (Gain values themselves may be legitimately negative.) */
		if (mi_card_get_96khz() < 0) {
			return -EAGAIN;
		}
		tmp.num_in = MI_NUM_CHANNELS;
		for (uint8_t ch = 0; ch < MI_NUM_CHANNELS; ch++) {
			tmp.in_gain[ch]    = (int8_t)mi_card_get_gain(ch);
			tmp.in_phantom[ch] = mi_card_get_phantom(ch) > 0;
			tmp.in_mute[ch]    = mi_card_get_mute(ch) > 0;
		}
		break;
#endif
#ifdef CONFIG_LO_CARD
	case CARD_TYPE_LO:
		if (lo_card_get_96khz() < 0) {
			return -EAGAIN;	/* not activated yet (pre PTP lock) */
		}
		tmp.num_out = LO_OUT_CHANNELS;
		for (uint8_t ch = 0; ch < LO_OUT_CHANNELS; ch++) {
			tmp.out_gain[ch] = (int8_t)lo_card_get_clip(ch);
			tmp.out_mute[ch] = lo_card_get_mute(ch) > 0;
		}
		break;
#endif
#ifdef CONFIG_IO_CARD
	case CARD_TYPE_IO:
		if (io_card_get_output_enable() < 0) {
			return -EAGAIN;
		}
		tmp.num_in = IO_IN_CHANNELS;
		tmp.num_out = IO_OUT_CHANNELS;
		for (uint8_t ch = 0; ch < IO_IN_CHANNELS; ch++) {
			tmp.in_gain[ch]    = (int8_t)io_card_get_in_gain(ch);
			tmp.in_phantom[ch] = io_card_get_in_phantom(ch) > 0;
			tmp.in_mute[ch]    = io_card_get_in_mute(ch) > 0;
		}
		for (uint8_t ch = 0; ch < IO_OUT_CHANNELS; ch++) {
			tmp.out_gain[ch] = (int8_t)io_card_get_out_clip(ch);
			tmp.out_mute[ch] = io_card_get_out_mute(ch) > 0;
		}
		break;
#endif
	default:
		return -ENODEV;
	}

	card_settings_lock();
	s_store = tmp;
	card_settings_unlock();

	return 0;
}

/* ================================================================
 * Apply: store -> drivers
 * ================================================================ */

int card_settings_apply(void)
{
	const struct card_info *card = card_manager_get_info(CARD_SLOT_MAIN);
	struct card_settings s;
	int failed = 0;

	card_settings_lock();
	s = s_store;
	card_settings_unlock();

	if (!s.valid || card == NULL || !card->present) {
		return 0;
	}
	if (card->type != s.card_type) {
		LOG_WRN("Stored settings are for a different card type "
			"(%u != %u) — ignored", s.card_type, card->type);
		return 0;
	}

	switch (card->type) {
#ifdef CONFIG_MI_CARD
	case CARD_TYPE_MI:
		if (mi_card_get_96khz() < 0) {
			return 0;	/* not ready yet — applied on init */
		}
		for (uint8_t ch = 0; ch < MIN(s.num_in, MI_NUM_CHANNELS); ch++) {
			failed += mi_card_set_gain(ch, s.in_gain[ch]) < 0;
			failed += mi_card_set_phantom(ch, s.in_phantom[ch]) < 0;
			failed += mi_card_set_mute(ch, s.in_mute[ch]) < 0;
		}
		break;
#endif
#ifdef CONFIG_LO_CARD
	case CARD_TYPE_LO:
		if (lo_card_get_96khz() < 0) {
			return 0;	/* not activated yet (pre PTP lock) */
		}
		for (uint8_t ch = 0; ch < MIN(s.num_out, LO_OUT_CHANNELS); ch++) {
			failed += lo_card_set_clip(ch, s.out_gain[ch]) < 0;
			failed += lo_card_set_mute(ch, s.out_mute[ch]) < 0;
		}
		break;
#endif
#ifdef CONFIG_IO_CARD
	case CARD_TYPE_IO:
		if (io_card_get_output_enable() < 0) {
			return 0;
		}
		for (uint8_t ch = 0; ch < MIN(s.num_in, IO_IN_CHANNELS); ch++) {
			failed += io_card_set_in_gain(ch, s.in_gain[ch]) < 0;
			failed += io_card_set_in_phantom(ch, s.in_phantom[ch]) < 0;
			failed += io_card_set_in_mute(ch, s.in_mute[ch]) < 0;
		}
		for (uint8_t ch = 0; ch < MIN(s.num_out, IO_OUT_CHANNELS); ch++) {
			failed += io_card_set_out_clip(ch, s.out_gain[ch]) < 0;
			failed += io_card_set_out_mute(ch, s.out_mute[ch]) < 0;
		}
		break;
#endif
	default:
		return 0;
	}

	if (failed) {
		LOG_WRN("Card settings applied with %d failed writes", failed);
	} else {
		LOG_INF("Stored card settings applied (%s)",
			card_type_name(card->type));
	}
	return 0;
}

/* ================================================================
 * Debounced persist
 * ================================================================ */

#define SAVE_DEBOUNCE  K_SECONDS(2)

static void save_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);

	/* config_json_serialize() runs card_settings_capture() itself, so
	 * the saved snapshot reflects the state at save time. */
#ifdef CONFIG_SD_CONFIG
	sd_config_save();
#endif
#ifdef CONFIG_FLASH_CONFIG
	flash_config_save();
#endif
	LOG_INF("Card settings persisted");
}

static K_WORK_DELAYABLE_DEFINE(save_work, save_work_fn);

void card_settings_schedule_save(void)
{
	k_work_reschedule(&save_work, SAVE_DEBOUNCE);
}
