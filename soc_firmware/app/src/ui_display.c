#include "ui_display.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/display/cfb.h>
#include <zephyr/drivers/display.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(ui_display, LOG_LEVEL_INF);

#if defined(CONFIG_DISPLAY) && defined(CONFIG_CHARACTER_FRAMEBUFFER) && DT_HAS_CHOSEN(zephyr_display)

struct ui_display_state {
	const struct device *dev;
	bool ready;
	bool metrics_valid;
	bool render_thread_started;
	char ip_text[16];
	uint16_t x_res;
	uint16_t y_res;
	uint8_t char_advance;
	uint8_t line_advance;
	uint8_t max_lines;
	uint8_t max_chars_per_line;
	struct ui_fpga_metrics metrics;
};

static struct ui_display_state ui_state = {
	.ip_text = "0.0.0.0",
	.char_advance = 4,
	.line_advance = 6,
	.max_lines = 1,
	.max_chars_per_line = 1,
};

K_MUTEX_DEFINE(ui_display_lock);
K_SEM_DEFINE(ui_display_refresh_sem, 0, 1);

#define UI_DISPLAY_STACK_SIZE 3072
#define UI_DISPLAY_PRIORITY K_PRIO_PREEMPT(12)

K_THREAD_STACK_DEFINE(ui_display_stack, UI_DISPLAY_STACK_SIZE);
static struct k_thread ui_display_thread_data;

#define UI_TINY_FONT_W 3
#define UI_TINY_FONT_H 5
#define UI_LINES_MAX 10
#define UI_LINE_CHARS_MAX 40
#define UI_MARGIN_LEFT 1
#define UI_MARGIN_TOP 1

static void ui_display_request_refresh(void)
{
	k_sem_give(&ui_display_refresh_sem);
}

static const char *eth_speed_to_text(uint8_t speed_code)
{
	switch (speed_code) {
	case 0:
		return "10M";
	case 1:
		return "100M";
	case 2:
		return "1G";
	default:
		return "?";
	}
}

static const uint8_t *ui_tiny_glyph(char c)
{
	static const uint8_t g_space[UI_TINY_FONT_H] = {0, 0, 0, 0, 0};
	static const uint8_t g_qmark[UI_TINY_FONT_H] = {6, 1, 2, 0, 2};
	static const uint8_t g_dot[UI_TINY_FONT_H] = {0, 0, 0, 0, 2};
	static const uint8_t g_colon[UI_TINY_FONT_H] = {0, 2, 0, 2, 0};
	static const uint8_t g_plus[UI_TINY_FONT_H] = {0, 2, 7, 2, 0};
	static const uint8_t g_minus[UI_TINY_FONT_H] = {0, 0, 7, 0, 0};
	static const uint8_t g_slash[UI_TINY_FONT_H] = {1, 1, 2, 4, 4};
	static const uint8_t g_0[UI_TINY_FONT_H] = {7, 5, 5, 5, 7};
	static const uint8_t g_1[UI_TINY_FONT_H] = {2, 6, 2, 2, 7};
	static const uint8_t g_2[UI_TINY_FONT_H] = {6, 1, 7, 4, 7};
	static const uint8_t g_3[UI_TINY_FONT_H] = {6, 1, 7, 1, 6};
	static const uint8_t g_4[UI_TINY_FONT_H] = {5, 5, 7, 1, 1};
	static const uint8_t g_5[UI_TINY_FONT_H] = {7, 4, 7, 1, 7};
	static const uint8_t g_6[UI_TINY_FONT_H] = {7, 4, 7, 5, 7};
	static const uint8_t g_7[UI_TINY_FONT_H] = {7, 1, 1, 1, 1};
	static const uint8_t g_8[UI_TINY_FONT_H] = {7, 5, 7, 5, 7};
	static const uint8_t g_9[UI_TINY_FONT_H] = {7, 5, 7, 1, 7};
	static const uint8_t g_A[UI_TINY_FONT_H] = {2, 5, 7, 5, 5};
	static const uint8_t g_B[UI_TINY_FONT_H] = {6, 5, 6, 5, 6};
	static const uint8_t g_C[UI_TINY_FONT_H] = {3, 4, 4, 4, 3};
	static const uint8_t g_D[UI_TINY_FONT_H] = {6, 5, 5, 5, 6};
	static const uint8_t g_E[UI_TINY_FONT_H] = {7, 4, 6, 4, 7};
	static const uint8_t g_F[UI_TINY_FONT_H] = {7, 4, 6, 4, 4};
	static const uint8_t g_G[UI_TINY_FONT_H] = {3, 4, 5, 5, 3};
	static const uint8_t g_H[UI_TINY_FONT_H] = {5, 5, 7, 5, 5};
	static const uint8_t g_I[UI_TINY_FONT_H] = {7, 2, 2, 2, 7};
	static const uint8_t g_J[UI_TINY_FONT_H] = {1, 1, 1, 5, 2};
	static const uint8_t g_K[UI_TINY_FONT_H] = {5, 5, 6, 5, 5};
	static const uint8_t g_L[UI_TINY_FONT_H] = {4, 4, 4, 4, 7};
	static const uint8_t g_M[UI_TINY_FONT_H] = {5, 7, 7, 5, 5};
	static const uint8_t g_N[UI_TINY_FONT_H] = {5, 7, 7, 7, 5};
	static const uint8_t g_O[UI_TINY_FONT_H] = {2, 5, 5, 5, 2};
	static const uint8_t g_P[UI_TINY_FONT_H] = {6, 5, 6, 4, 4};
	static const uint8_t g_Q[UI_TINY_FONT_H] = {2, 5, 5, 3, 1};
	static const uint8_t g_R[UI_TINY_FONT_H] = {6, 5, 6, 5, 5};
	static const uint8_t g_S[UI_TINY_FONT_H] = {3, 4, 2, 1, 6};
	static const uint8_t g_T[UI_TINY_FONT_H] = {7, 2, 2, 2, 2};
	static const uint8_t g_U[UI_TINY_FONT_H] = {5, 5, 5, 5, 7};
	static const uint8_t g_V[UI_TINY_FONT_H] = {5, 5, 5, 5, 2};
	static const uint8_t g_W[UI_TINY_FONT_H] = {5, 5, 7, 7, 5};
	static const uint8_t g_X[UI_TINY_FONT_H] = {5, 5, 2, 5, 5};
	static const uint8_t g_Y[UI_TINY_FONT_H] = {5, 5, 2, 2, 2};
	static const uint8_t g_Z[UI_TINY_FONT_H] = {7, 1, 2, 4, 7};

	char uc = (char)toupper((unsigned char)c);

	switch (uc) {
	case ' ':
		return g_space;
	case '.':
		return g_dot;
	case ':':
		return g_colon;
	case '+':
		return g_plus;
	case '-':
		return g_minus;
	case '/':
		return g_slash;
	case '0':
		return g_0;
	case '1':
		return g_1;
	case '2':
		return g_2;
	case '3':
		return g_3;
	case '4':
		return g_4;
	case '5':
		return g_5;
	case '6':
		return g_6;
	case '7':
		return g_7;
	case '8':
		return g_8;
	case '9':
		return g_9;
	case 'A':
		return g_A;
	case 'B':
		return g_B;
	case 'C':
		return g_C;
	case 'D':
		return g_D;
	case 'E':
		return g_E;
	case 'F':
		return g_F;
	case 'G':
		return g_G;
	case 'H':
		return g_H;
	case 'I':
		return g_I;
	case 'J':
		return g_J;
	case 'K':
		return g_K;
	case 'L':
		return g_L;
	case 'M':
		return g_M;
	case 'N':
		return g_N;
	case 'O':
		return g_O;
	case 'P':
		return g_P;
	case 'Q':
		return g_Q;
	case 'R':
		return g_R;
	case 'S':
		return g_S;
	case 'T':
		return g_T;
	case 'U':
		return g_U;
	case 'V':
		return g_V;
	case 'W':
		return g_W;
	case 'X':
		return g_X;
	case 'Y':
		return g_Y;
	case 'Z':
		return g_Z;
	default:
		return g_qmark;
	}
}

static void ui_draw_tiny_char(const struct device *dev, uint16_t x, uint16_t y, char c)
{
	struct cfb_position pos;
	const uint8_t *rows = ui_tiny_glyph(c);

	for (uint8_t row = 0; row < UI_TINY_FONT_H; row++) {
		uint8_t bits = rows[row];

		for (uint8_t col = 0; col < UI_TINY_FONT_W; col++) {
			if ((bits & (1U << (UI_TINY_FONT_W - 1U - col))) == 0U) {
				continue;
			}

			pos.x = x + col;
			pos.y = y + row;
			if (pos.x < ui_state.x_res && pos.y < ui_state.y_res) {
				(void)cfb_draw_point(dev, &pos);
			}
		}
	}
}

static void ui_draw_tiny_text(const struct device *dev, uint16_t x, uint16_t y,
			      const char *text, uint8_t max_chars)
{
	uint8_t count = 0U;

	while (*text != '\0' && count < max_chars) {
		ui_draw_tiny_char(dev, x, y, *text);
		x += ui_state.char_advance;
		text++;
		count++;
	}
}

static void ui_display_render_locked(void)
{
	char lines[UI_LINES_MAX][UI_LINE_CHARS_MAX];
	const struct ui_fpga_metrics *m = &ui_state.metrics;
	uint8_t lines_to_draw = ui_state.max_lines;
	uint32_t page = 0U;

	if (!ui_state.ready) {
		return;
	}
	if (lines_to_draw > ARRAY_SIZE(lines)) {
		lines_to_draw = ARRAY_SIZE(lines);
	}
	for (uint8_t i = 0; i < ARRAY_SIZE(lines); i++) {
		lines[i][0] = '\0';
	}

	if (!ui_state.metrics_valid) {
		snprintf(lines[0], sizeof(lines[0]), "IP %s", ui_state.ip_text);
		snprintf(lines[1], sizeof(lines[1]), "WAIT METRICS");
	} else if (lines_to_draw < 5U) {
		page = (k_uptime_get_32() / 3000U) % 2U;
		if (page == 0U) {
			snprintf(lines[0], sizeof(lines[0]), "IP %s", ui_state.ip_text);
			snprintf(lines[1], sizeof(lines[1]), "E%c%s V%d",
				 m->link_up ? 'U' : 'D',
				 eth_speed_to_text(m->speed_code),
				 m->ppb_valid ? 1 : 0);
			snprintf(lines[2], sizeof(lines[2]), "L%d C%d PJ%d",
				 m->wc_locked ? 1 : 0,
				 m->wc_configured ? 1 : 0,
				 m->wc_phasejump ? 1 : 0);
			if (lines_to_draw > 3U) {
				snprintf(lines[3], sizeof(lines[3]), "LS%d P%+ld",
					 m->ptp_leader_lost ? 1 : 0,
					 (long)m->ppb_offset);
			}
		} else {
			snprintf(lines[0], sizeof(lines[0]), "IP %s", ui_state.ip_text);
			snprintf(lines[1], sizeof(lines[1]), "P%+ld C%+ld",
				 (long)m->ppb_offset,
				 (long)m->correction_ppb);
			snprintf(lines[2], sizeof(lines[2]), "D%+ld O%+ld",
				 (long)m->path_delay_ns,
				 (long)m->leader_offset_ns);
			if (lines_to_draw > 3U) {
				snprintf(lines[3], sizeof(lines[3]), "CY%lu OT%lu",
					 (unsigned long)m->cycle,
					 (unsigned long)m->outliers);
			}
		}
	} else {
		snprintf(lines[0], sizeof(lines[0]), "IP %s", ui_state.ip_text);
		snprintf(lines[1], sizeof(lines[1]), "E%c%s V%d L%d C%d",
			 m->link_up ? 'U' : 'D',
			 eth_speed_to_text(m->speed_code),
			 m->ppb_valid ? 1 : 0,
			 m->wc_locked ? 1 : 0,
			 m->wc_configured ? 1 : 0);
		snprintf(lines[2], sizeof(lines[2]), "PJ%d LS%d",
			 m->wc_phasejump ? 1 : 0,
				 m->ptp_leader_lost ? 1 : 0);
		snprintf(lines[3], sizeof(lines[3]), "P%+ld C%+ld",
			 (long)m->ppb_offset,
			 (long)m->correction_ppb);
		snprintf(lines[4], sizeof(lines[4]), "D%+ld O%+ld",
			 (long)m->path_delay_ns,
			 (long)m->leader_offset_ns);
		if (lines_to_draw > 5U) {
			snprintf(lines[5], sizeof(lines[5]), "CY%lu OT%lu",
				 (unsigned long)m->cycle,
				 (unsigned long)m->outliers);
		}
	}

	cfb_framebuffer_clear(ui_state.dev, false);
	for (uint8_t i = 0; i < lines_to_draw; i++) {
		uint16_t y = UI_MARGIN_TOP + (i * ui_state.line_advance);
		if (y + UI_TINY_FONT_H > ui_state.y_res) {
			break;
		}
		ui_draw_tiny_text(ui_state.dev, UI_MARGIN_LEFT, y, lines[i],
				  ui_state.max_chars_per_line);
	}
	(void)cfb_framebuffer_finalize(ui_state.dev);
}

static void ui_display_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		k_sem_take(&ui_display_refresh_sem, K_FOREVER);

		k_mutex_lock(&ui_display_lock, K_FOREVER);
		ui_display_render_locked();
		k_mutex_unlock(&ui_display_lock);
	}
}

void ui_display_init(void)
{
	ui_state.dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
	if (!device_is_ready(ui_state.dev)) {
		LOG_WRN("Display device not ready");
		return;
	}

	if (display_set_pixel_format(ui_state.dev, PIXEL_FORMAT_MONO10) != 0 &&
	    display_set_pixel_format(ui_state.dev, PIXEL_FORMAT_MONO01) != 0) {
		LOG_WRN("Failed to set display pixel format");
		return;
	}

	if (cfb_framebuffer_init(ui_state.dev) != 0) {
		LOG_WRN("Framebuffer initialization failed");
		return;
	}

	ui_state.x_res = cfb_get_display_parameter(ui_state.dev, CFB_DISPLAY_WIDTH);
	ui_state.y_res = cfb_get_display_parameter(ui_state.dev, CFB_DISPLAY_HEIGHT);
	if (ui_state.char_advance > 0U) {
		uint16_t usable_w = (ui_state.x_res > UI_MARGIN_LEFT) ?
			(ui_state.x_res - UI_MARGIN_LEFT) : 0U;
		ui_state.max_chars_per_line = usable_w / ui_state.char_advance;
	}
	if (ui_state.line_advance > 0U) {
		uint16_t usable_h = (ui_state.y_res > UI_MARGIN_TOP) ?
			(ui_state.y_res - UI_MARGIN_TOP) : 0U;
		ui_state.max_lines = usable_h / ui_state.line_advance;
	}
	if (ui_state.max_chars_per_line == 0) {
		ui_state.max_chars_per_line = 1;
	}
	if (ui_state.max_lines == 0) {
		ui_state.max_lines = 1;
	}

	cfb_framebuffer_clear(ui_state.dev, true);
	(void)display_blanking_off(ui_state.dev);

	ui_state.ready = true;
	if (!ui_state.render_thread_started) {
		k_thread_create(&ui_display_thread_data, ui_display_stack,
				UI_DISPLAY_STACK_SIZE, ui_display_thread,
				NULL, NULL, NULL, UI_DISPLAY_PRIORITY,
				0, K_NO_WAIT);
		k_thread_name_set(&ui_display_thread_data, "ui_display");
		ui_state.render_thread_started = true;
	}

	ui_display_request_refresh();

	LOG_INF("OLED display ready (tiny %ux%u, lines=%u, cols=%u)",
		UI_TINY_FONT_W, UI_TINY_FONT_H,
		ui_state.max_lines, ui_state.max_chars_per_line);
}

void ui_display_set_ip(const struct in_addr *addr)
{
	const uint8_t *ip;

	if (!ui_state.ready || !addr) {
		return;
	}

	ip = (const uint8_t *)&addr->s_addr;

	k_mutex_lock(&ui_display_lock, K_FOREVER);
	snprintf(ui_state.ip_text, sizeof(ui_state.ip_text),
		 "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
	k_mutex_unlock(&ui_display_lock);
	ui_display_request_refresh();
}

void ui_display_set_metrics(const struct ui_fpga_metrics *metrics)
{
	if (!ui_state.ready || !metrics) {
		return;
	}

	k_mutex_lock(&ui_display_lock, K_FOREVER);
	ui_state.metrics = *metrics;
	ui_state.metrics_valid = true;
	k_mutex_unlock(&ui_display_lock);
	ui_display_request_refresh();
}

#else

void ui_display_init(void)
{
}

void ui_display_set_ip(const struct in_addr *addr)
{
	(void)addr;
}

void ui_display_set_metrics(const struct ui_fpga_metrics *metrics)
{
	(void)metrics;
}

#endif
