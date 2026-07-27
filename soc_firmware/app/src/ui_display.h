#ifndef UI_DISPLAY_H_
#define UI_DISPLAY_H_

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/net/net_ip.h>

struct ui_fpga_metrics {
	bool ppb_valid;
	bool wc_locked;
	bool wc_phasejump;
	bool wc_configured;
	bool ptp_leader_lost;
	bool link_up;
	uint8_t speed_code;
	int32_t path_delay_ns;
	int32_t leader_offset_ns;
	int32_t ppb_offset;
	int32_t correction_ppb;
	uint32_t cycle;
	uint32_t outliers;
	/* RX stream-underrun diagnostics: per-stream flags / per-channel mute */
	uint8_t rx_underrun;
	uint8_t rx_mute;
};

void ui_display_init(void);
void ui_display_set_ip(const struct in_addr *addr);
void ui_display_set_metrics(const struct ui_fpga_metrics *metrics);

#endif /* UI_DISPLAY_H_ */
