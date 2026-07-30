/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * Zephyr shell bindings for the FPGA HAL.
 *
 * Works on every backend (LiteX, SPI) since everything is routed
 * through the backend-independent fpga_hal_*() API.
 */

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/net/net_ip.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include "fpga_hal.h"

#ifdef CONFIG_AES67_FPGA_JTAG
#include "../fpga_jtag/fpga_jtag.h"
#endif

/* ---- small helpers ---- */

static int parse_u32(const char *s, uint32_t *out)
{
	char *end;
	unsigned long v = strtoul(s, &end, 0); /* 0 -> auto (0x/0/dec) */

	if (*s == '\0' || *end != '\0') {
		return -EINVAL;
	}
	*out = (uint32_t)v;
	return 0;
}

static int parse_u8(const char *s, uint8_t *out)
{
	uint32_t v;

	if (parse_u32(s, &v) < 0 || v > 0xFF) {
		return -EINVAL;
	}
	*out = (uint8_t)v;
	return 0;
}

static int parse_i8(const char *s, int8_t *out)
{
	char *end;
	long v = strtol(s, &end, 0);

	if (*s == '\0' || *end != '\0' || v < -128 || v > 127) {
		return -EINVAL;
	}
	*out = (int8_t)v;
	return 0;
}

static int parse_i32(const char *s, int32_t *out)
{
	char *end;
	long v = strtol(s, &end, 0);

	if (*s == '\0' || *end != '\0' || v < INT32_MIN || v > INT32_MAX) {
		return -EINVAL;
	}
	*out = (int32_t)v;
	return 0;
}

static int parse_ipv4(const char *s, struct in_addr *out)
{
	unsigned int o[4];
	char extra;

	if (sscanf(s, "%u.%u.%u.%u%c",
		   &o[0], &o[1], &o[2], &o[3], &extra) != 4) {
		return -EINVAL;
	}
	for (int i = 0; i < 4; i++) {
		if (o[i] > 255) {
			return -EINVAL;
		}
	}
	/* s_addr is network byte order: o[0] is the MSB. */
	out->s_addr = htonl(((uint32_t)o[0] << 24) |
			    ((uint32_t)o[1] << 16) |
			    ((uint32_t)o[2] << 8)  |
			    ((uint32_t)o[3]));
	return 0;
}

static int parse_mac(const char *s, uint8_t mac[6])
{
	unsigned int b[6];

	if (sscanf(s, "%x:%x:%x:%x:%x:%x",
		   &b[0], &b[1], &b[2], &b[3], &b[4], &b[5]) != 6) {
		return -EINVAL;
	}
	for (int i = 0; i < 6; i++) {
		if (b[i] > 0xFF) {
			return -EINVAL;
		}
		mac[i] = (uint8_t)b[i];
	}
	return 0;
}

/* Named bit table for `fpga ctrl set/clear`. */
struct ctrl_bit {
	const char *name;
	uint32_t    mask;
};

static const struct ctrl_bit CTRL_BITS[] = {
	{ "ppb_start",      FPGA_HAL_CTRL_PPB_START       },
	{ "wc_reset",       FPGA_HAL_CTRL_RESET_WALLCLOCK },
	{ "ptp_reset",      FPGA_HAL_CTRL_RESET_PTP       },
	{ "eth_reset",      FPGA_HAL_CTRL_RESET_ETHERNET  },
};

static uint32_t ctrl_parse_names(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t mask = 0;

	for (size_t i = 0; i < argc; i++) {
		bool found = false;

		for (size_t j = 0; j < ARRAY_SIZE(CTRL_BITS); j++) {
			if (strcmp(argv[i], CTRL_BITS[j].name) == 0) {
				mask |= CTRL_BITS[j].mask;
				found = true;
				break;
			}
		}
		if (!found) {
			shell_error(sh, "unknown ctrl bit '%s'", argv[i]);
			return 0;
		}
	}
	return mask;
}

/* ---- commands: status ---- */

static int cmd_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc); ARG_UNUSED(argv);

	uint32_t s = fpga_hal_read_status();
	uint32_t speed = (s & FPGA_HAL_ETH_SPEED_MASK) >> FPGA_HAL_ETH_SPEED_SHIFT;
	const char *speed_str[] = {"10M", "100M", "1G", "?"};

	shell_print(sh, "status = 0x%08x", s);
	shell_print(sh, "  wc_locked     = %d", !!(s & FPGA_HAL_CLK_WC_LOCKED));
	shell_print(sh, "  wc_configured = %d", !!(s & FPGA_HAL_CLK_WC_CONFIGURED));
	shell_print(sh, "  wc_phasejump  = %d", !!(s & FPGA_HAL_CLK_WC_PHASEJUMP));
	shell_print(sh, "  ppb_valid     = %d", !!(s & FPGA_HAL_CLK_PPB_VALID));
	shell_print(sh, "  ptp_leader    = %d", !!(s & FPGA_HAL_PTP_IS_LEADER));
	shell_print(sh, "  ptp_follower  = %d", !!(s & FPGA_HAL_PTP_IS_FOLLOWER));
	shell_print(sh, "  ptp_lost      = %d", !!(s & FPGA_HAL_CLK_PTP_LEADER_LOST));
	shell_print(sh, "  eth_link_up   = %d", !!(s & FPGA_HAL_ETH_LINK_UP));
	shell_print(sh, "  eth_speed     = %s", speed_str[speed & 0x3]);
	shell_print(sh, "path_delay = %d ns", fpga_hal_read_path_delay());
	shell_print(sh, "ptp_offset = %d ns", fpga_hal_read_ptp_offset());

	uint32_t wc, pll;
	if (fpga_hal_read_ppb_counts(&wc, &pll)) {
		shell_print(sh, "ppb: wc=%u pll=%u", wc, pll);
	} else {
		shell_print(sh, "ppb: <measurement not valid>");
	}
	return 0;
}

static int cmd_gmid(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc); ARG_UNUSED(argv);

	uint8_t id[8];
	bool have = fpga_hal_read_ptp_leader_id(id);

	if (!have) {
		shell_print(sh, "GM id: <none>");
		return 0;
	}
	shell_print(sh, "GM id: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
		    id[0], id[1], id[2], id[3], id[4], id[5], id[6], id[7]);
	return 0;
}

/* ---- commands: config writes ---- */

static int cmd_mac(const struct shell *sh, size_t argc, char **argv)
{
	uint8_t mac[6];

	if (argc != 2 || parse_mac(argv[1], mac) < 0) {
		shell_error(sh, "usage: fpga mac <aa:bb:cc:dd:ee:ff>");
		return -EINVAL;
	}
	int ret = fpga_hal_write_mac(mac);
	shell_print(sh, "write_mac: %d", ret);
	return ret;
}

static int cmd_ip(const struct shell *sh, size_t argc, char **argv)
{
	struct in_addr ip;

	if (argc != 2 || parse_ipv4(argv[1], &ip) < 0) {
		shell_error(sh, "usage: fpga ip <a.b.c.d>");
		return -EINVAL;
	}
	int ret = fpga_hal_write_ip(&ip);
	shell_print(sh, "write_ip: %d", ret);
	return ret;
}

static int cmd_ptp(const struct shell *sh, size_t argc, char **argv)
{
	uint8_t src;
	int8_t  log_sync, log_ann;

	if (argc != 4 ||
	    parse_u8(argv[1], &src) < 0 ||
	    parse_i8(argv[2], &log_sync) < 0 ||
	    parse_i8(argv[3], &log_ann) < 0) {
		shell_error(sh, "usage: fpga ptp <time_source> <log_sync> <log_announce>");
		return -EINVAL;
	}
	int ret = fpga_hal_write_ptp_config(src, log_sync, log_ann);
	shell_print(sh, "write_ptp_config: %d", ret);
	return ret;
}

static int cmd_gm(const struct shell *sh, size_t argc, char **argv)
{
	uint8_t p1, p2, cls, acc;

	if (argc != 5 ||
	    parse_u8(argv[1], &p1)  < 0 ||
	    parse_u8(argv[2], &p2)  < 0 ||
	    parse_u8(argv[3], &cls) < 0 ||
	    parse_u8(argv[4], &acc) < 0) {
		shell_error(sh, "usage: fpga gm <priority1> <priority2> <class> <accuracy>");
		return -EINVAL;
	}
	int ret = fpga_hal_write_ptp_gm_quality(p1, p2, cls, acc);
	shell_print(sh, "write_ptp_gm_quality: %d", ret);
	return ret;
}

/* ---- commands: ctrl ---- */

static int cmd_ctrl_set(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 2) {
		shell_error(sh, "usage: fpga ctrl set <bit> [<bit> …]");
		return -EINVAL;
	}
	uint32_t mask = ctrl_parse_names(sh, argc - 1, &argv[1]);
	if (mask == 0) {
		return -EINVAL;
	}
	int ret = fpga_hal_ctrl_set_bits(mask);
	shell_print(sh, "ctrl_set 0x%08x: %d", mask, ret);
	return ret;
}

static int cmd_ctrl_clear(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 2) {
		shell_error(sh, "usage: fpga ctrl clear <bit> [<bit> …]");
		return -EINVAL;
	}
	uint32_t mask = ctrl_parse_names(sh, argc - 1, &argv[1]);
	if (mask == 0) {
		return -EINVAL;
	}
	int ret = fpga_hal_ctrl_clear_bits(mask);
	shell_print(sh, "ctrl_clear 0x%08x: %d", mask, ret);
	return ret;
}

static int cmd_ctrl_list(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc); ARG_UNUSED(argv);
	shell_print(sh, "known ctrl bits:");
	for (size_t i = 0; i < ARRAY_SIZE(CTRL_BITS); i++) {
		shell_print(sh, "  %-12s (mask 0x%08x)",
			    CTRL_BITS[i].name, CTRL_BITS[i].mask);
	}
	return 0;
}

static int cmd_adda(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_error(sh, "usage: fpga adda <0|1>   (1 = released, 0 = held in reset)");
		return -EINVAL;
	}
	uint8_t v;
	if (parse_u8(argv[1], &v) < 0 || v > 1) {
		return -EINVAL;
	}
	int ret = fpga_hal_set_adda_nrst(v != 0);
	shell_print(sh, "adda_nrst=%u: %d", v, ret);
	return ret;
}

/* ---- commands: metering ---- */

static int cmd_meter(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc); ARG_UNUSED(argv);

	uint16_t rxs, rxc, txs, txc;
	fpga_hal_read_metering(&rxs, &rxc, &txs, &txc);
	shell_print(sh, "rx signal=0x%04x clip=0x%04x", rxs, rxc);
	shell_print(sh, "tx signal=0x%04x clip=0x%04x", txs, txc);
	return 0;
}

/* ---- commands: streams ---- */

static int cmd_tx(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 6 || argc > 6 + 8) {
		shell_error(sh,
			"usage: fpga tx <stream> <dst_ip> <ch_cnt> <spp> <ssrc> [ch_id …]");
		return -EINVAL;
	}
	uint8_t  stream, ch_cnt, spp;
	uint32_t ssrc;
	struct in_addr ip;
	uint8_t  ch_ids[8] = {0};
	uint8_t  n_ch = (uint8_t)(argc - 6);

	if (parse_u8(argv[1], &stream) < 0 ||
	    parse_ipv4(argv[2], &ip) < 0 ||
	    parse_u8(argv[3], &ch_cnt) < 0 ||
	    parse_u8(argv[4], &spp)    < 0 ||
	    parse_u32(argv[5], &ssrc)  < 0) {
		shell_error(sh, "invalid argument");
		return -EINVAL;
	}
	for (uint8_t i = 0; i < n_ch; i++) {
		if (parse_u8(argv[6 + i], &ch_ids[i]) < 0) {
			shell_error(sh, "invalid ch_id '%s'", argv[6 + i]);
			return -EINVAL;
		}
	}
	int ret = fpga_hal_write_tx_stream_config(stream, &ip, ch_cnt, spp,
						  ch_ids, n_ch, ssrc);
	shell_print(sh, "write_tx_stream: %d", ret);
	return ret;
}

static int cmd_rx(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 7 || argc > 7 + 8) {
		shell_error(sh,
			"usage: fpga rx <stream> <dst_ip> <port> <ch_cnt> <delay> <spc> [ch_map …]");
		return -EINVAL;
	}
	uint8_t  stream, ch_cnt, delay, spc;
	uint32_t port32;
	struct in_addr ip;
	uint8_t  ch_map[8] = {0};
	uint8_t  n_ch = (uint8_t)(argc - 7);

	if (parse_u8(argv[1], &stream)       < 0 ||
	    parse_ipv4(argv[2], &ip) < 0 ||
	    parse_u32(argv[3], &port32)      < 0 || port32 > 0xFFFF ||
	    parse_u8(argv[4], &ch_cnt)       < 0 ||
	    parse_u8(argv[5], &delay)        < 0 ||
	    parse_u8(argv[6], &spc)          < 0) {
		shell_error(sh, "invalid argument");
		return -EINVAL;
	}
	for (uint8_t i = 0; i < n_ch; i++) {
		if (parse_u8(argv[7 + i], &ch_map[i]) < 0) {
			shell_error(sh, "invalid ch_map '%s'", argv[7 + i]);
			return -EINVAL;
		}
	}
	int ret = fpga_hal_write_rx_stream_config(stream, &ip,
						  (uint16_t)port32, ch_map,
						  ch_cnt, delay, spc);
	shell_print(sh, "write_rx_stream: %d", ret);
	return ret;
}

/* ---- commands: PTP delayAsymmetry calibration ---- */

static int cmd_asym(const struct shell *sh, size_t argc, char **argv)
{
	struct fpga_hal_ptp_tuning t;

	fpga_hal_read_ptp_tuning(&t);

	if (argc == 1) {
		shell_print(sh, "delay_asymmetry = %d ns", t.delay_asymmetry_ns);
		shell_print(sh, "  (positive = M->S path longer than S->M)");
		return 0;
	}

	int32_t ns;

	if (argc != 2 || parse_i32(argv[1], &ns) < 0) {
		shell_error(sh, "usage: fpga asym [<signed_ns>]");
		return -EINVAL;
	}

	t.delay_asymmetry_ns = ns;
	int ret = fpga_hal_write_ptp_tuning(&t);

	if (ret == 0) {
		shell_print(sh, "delay_asymmetry <- %d ns", ns);
	} else {
		shell_error(sh, "write failed: %d", ret);
	}
	return ret;
}

static int cmd_peek(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t addr, val;

	ARG_UNUSED(argc);

	if (parse_u32(argv[1], &addr) < 0) {
		shell_error(sh, "usage: fpga peek <byte-addr>");
		return -EINVAL;
	}

	int ret = fpga_hal_csr_read(addr, &val);

	if (ret < 0) {
		shell_error(sh, "read failed: %d", ret);
		return ret;
	}
	shell_print(sh, "[0x%08x] = 0x%08x (%u)", addr, val, val);
	return 0;
}

static int cmd_poke(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t addr, val;

	ARG_UNUSED(argc);

	if (parse_u32(argv[1], &addr) < 0 || parse_u32(argv[2], &val) < 0) {
		shell_error(sh, "usage: fpga poke <byte-addr> <value>");
		return -EINVAL;
	}

	int ret = fpga_hal_csr_write(addr, val);

	if (ret < 0) {
		shell_error(sh, "write failed: %d", ret);
		return ret;
	}
	shell_print(sh, "[0x%08x] <- 0x%08x", addr, val);
	return 0;
}

/* Body compiled out (not the function) so the SHELL_COND_CMD ternary below
 * always has a symbol to reference. */
static int cmd_jtag(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

#ifdef CONFIG_AES67_FPGA_JTAG
	uint8_t bsc[76];
	enum fpga_jtag_health h = fpga_jtag_health_check();
	int bits = fpga_jtag_sample_boundary(bsc, sizeof(bsc));

	shell_print(sh, "FPGA health:   %s", fpga_jtag_health_str(h));
	shell_print(sh, "USERCODE:      0x%08X", fpga_jtag_read_usercode());
	shell_print(sh, "CRC_ERROR pin: %s",
		    fpga_jtag_crc_error() ? "ASSERTED" : "clear");
	if (bits > 0) {
		shell_print(sh, "Boundary sample (%d bits, LSB first - bit i "
			    "= BSC cell i, CRC_ERROR = cell 588):", bits);
		shell_hexdump(sh, bsc, (bits + 7) / 8);
	}
	return (h == FPGA_JTAG_HEALTH_OK) ? 0 : -EIO;
#else
	shell_error(sh, "JTAG driver not enabled");
	return -ENOTSUP;
#endif
}

/* ---- command tree ---- */

SHELL_STATIC_SUBCMD_SET_CREATE(fpga_ctrl_cmds,
	SHELL_CMD_ARG(set,   NULL,
		"Set ctrl bits (names). Usage: fpga ctrl set <bit> [<bit> …]",
		cmd_ctrl_set,   2, ARRAY_SIZE(CTRL_BITS)),
	SHELL_CMD_ARG(clear, NULL,
		"Clear ctrl bits (names). Usage: fpga ctrl clear <bit> [<bit> …]",
		cmd_ctrl_clear, 2, ARRAY_SIZE(CTRL_BITS)),
	SHELL_CMD(list,  NULL, "List known ctrl bit names.", cmd_ctrl_list),
	SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(fpga_cmds,
	SHELL_CMD(status,   NULL, "Read FPGA status register and PTP counters.", cmd_status),
	SHELL_CMD(gmid,     NULL, "Read current PTP grandmaster clock identity.",  cmd_gmid),
	SHELL_CMD_ARG(mac,  NULL, "Write MAC address. Usage: fpga mac <aa:bb:cc:dd:ee:ff>", cmd_mac, 2, 0),
	SHELL_CMD_ARG(ip,   NULL, "Write IPv4 address. Usage: fpga ip <a.b.c.d>",   cmd_ip,   2, 0),
	SHELL_CMD_ARG(ptp,  NULL,
		"Write PTP config. Usage: fpga ptp <time_source> <log_sync> <log_announce>",
		cmd_ptp, 4, 0),
	SHELL_CMD_ARG(gm,   NULL,
		"Write GM quality. Usage: fpga gm <priority1> <priority2> <class> <accuracy>",
		cmd_gm, 5, 0),
	SHELL_CMD(ctrl,     &fpga_ctrl_cmds, "Control-bit set/clear/list.", NULL),
	SHELL_CMD_ARG(adda, NULL, "AD/DA nRST control. Usage: fpga adda <0|1>", cmd_adda, 2, 0),
	SHELL_CMD(meter,    NULL, "Read metering registers (and clear sticky bits).", cmd_meter),
	SHELL_COND_CMD(CONFIG_AES67_FPGA_JTAG, jtag, NULL,
		"JTAG health check (IDCODE, USERCODE, CRC_ERROR sample).",
		cmd_jtag),
	SHELL_CMD_ARG(tx,   NULL,
		"Configure TX stream. Usage: fpga tx <stream> <dst_ip> <ch_cnt> <spp> <ssrc> [ch_id …]",
		cmd_tx, 6, 8),
	SHELL_CMD_ARG(rx,   NULL,
		"Configure RX stream. Usage: fpga rx <stream> <dst_ip> <port> <ch_cnt> <delay> <spc> [ch_map …]",
		cmd_rx, 7, 8),
	SHELL_CMD_ARG(asym, NULL,
		"Read/write PTP delayAsymmetry (ns, signed; +ve = M->S longer). "
		"Usage: fpga asym [<signed_ns>]",
		cmd_asym, 1, 1),
	SHELL_CMD_ARG(peek, NULL,
		"Read a CSR word (absolute byte address, see csr.csv). "
		"Usage: fpga peek <addr>",
		cmd_peek, 2, 0),
	SHELL_CMD_ARG(poke, NULL,
		"Write a CSR word. Usage: fpga poke <addr> <value>",
		cmd_poke, 3, 0),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(fpga, &fpga_cmds, "AES67 FPGA control", NULL);
