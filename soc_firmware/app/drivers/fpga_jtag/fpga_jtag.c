/*
 * Cyclone FPGA JTAG (IEEE 1149.1) bit-bang: chain scan + SRAM configuration.
 *
 * TAP handling and the configuration sequence mirror openFPGALoader
 * (src/jtag.cpp detectChain, src/altera.cpp programMem). JTAG SRAM
 * configuration works regardless of the MSEL straps and takes precedence
 * over AS/PS, which is why it is used here instead of Passive Serial.
 *
 * Bit-bang timing (IEEE 1149.1): TMS and TDI are sampled by the device on
 * the rising TCK edge; TDO updates on the falling edge. We set TMS/TDI
 * while TCK is low, sample TDO (valid from the previous falling edge),
 * then pulse TCK high and low.
 *
 * All shift_* primitives start and end in Run-Test/Idle, so the
 * configuration sequence is a flat list of shifts and idle-clock runs.
 */

#define DT_DRV_COMPAT aes67_cyclone_jtag

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "fpga_jtag.h"

#ifdef CONFIG_AES67_FPGA_JTAG_BOOT_LOAD
#include "fpga_jtag_meta.h"	/* FPGA_JTAG_USERCODE (from the SVF) */
#endif

LOG_MODULE_REGISTER(fpga_jtag, LOG_LEVEL_INF);

struct fpga_jtag_config {
	struct gpio_dt_spec tck;
	struct gpio_dt_spec tms;
	struct gpio_dt_spec tdi;
	struct gpio_dt_spec tdo;
};

/* Known Altera Cyclone II IDCODEs (version nibble masked off).
 * Values derived from the Cyclone II Handbook vol. 1, Table 3-3.
 */
struct fpga_jtag_id {
	uint32_t idcode;
	const char *name;
};

static const struct fpga_jtag_id fpga_jtag_ids[] = {
	{ 0x020B10DD, "Altera EP2C5 (Cyclone II)" },
	{ 0x020B20DD, "Altera EP2C8 (Cyclone II)" },
	{ 0x020B30DD, "Altera EP2C15/EP2C20 (Cyclone II)" },
	{ 0x020B40DD, "Altera EP2C35 (Cyclone II)" },
	{ 0x020B50DD, "Altera EP2C50 (Cyclone II)" },
	{ 0x020B60DD, "Altera EP2C70 (Cyclone II)" },
};

/* Altera Cyclone JTAG instructions (IR length 10), from openFPGALoader
 * altera.cpp / the Quartus SVF configuration flow. */
#define ALTERA_IR_LEN         10
#define ALTERA_IR_PROGRAM     0x002
#define ALTERA_IR_STARTUP     0x003
#define ALTERA_IR_CHECK_STATUS 0x004
#define ALTERA_IR_SAMPLE      0x005
#define ALTERA_IR_USERCODE    0x007
#define ALTERA_IR_BYPASS      0x3FF

/* Idle TCK counts between configuration steps. These are the exact values
 * from the Quartus-generated JTAG SVF for this device (EP2C8); the generic
 * openFPGALoader sequence uses different counts and does not configure a
 * Cyclone II. */
#define ALTERA_TCK_PROGRAM    25000U
#define ALTERA_TCK_CHECK      125U
#define ALTERA_TCK_STARTUP    125U
#define ALTERA_TCK_STARTUP2   512U
#define ALTERA_TCK_BYPASS     25000U
/* CHECK_STATUS data register length and the bit that must read 1 for a
 * successful configuration (CONF_DONE equivalent). Both taken from the
 * Quartus SVF's own 597-bit "SDR ... TDO ... MASK" status check: the MASK
 * selects exactly bit 229 (LSB-indexed) and expects it set. */
#define ALTERA_STATUS_DR_BITS 597U
#define ALTERA_STATUS_DONE_BIT 229U

/* SAMPLE/PRELOAD captures the pin states into the boundary-scan register —
 * which is the same 597-bit register CHECK_STATUS shifts (BSDL EP2C8Q208
 * BOUNDARY_LENGTH = 597; that is why the LSB-indexed bit numbers here and
 * above line up with our shift order, cell 0 shifts out first). Cell 588 is
 * the input cell of I/O pin 3 ("BSC group 196" in the BSDL), the
 * dual-purpose CRC_ERROR output: with CRC_ERROR_CHECKING ON in the .qsf the
 * device drives it low in user mode and latches it high when its background
 * CRC check finds corrupted configuration SRAM (SEU). Cyclone II has no
 * JTAG error-message register (that came with Cyclone III), so sampling
 * this pin is the only way to see the CRC result over JTAG. SAMPLE is
 * non-invasive per IEEE 1149.1 — safe while the device operates. */
#define ALTERA_BSC_CRC_ERROR_BIT 588U

/* Serialises the JTAG pins between the boot path, the health monitor and
 * the shell. Zephyr mutexes nest, so public API calling public API is fine. */
static K_MUTEX_DEFINE(fpga_jtag_lock);

static const struct fpga_jtag_config *jtag_cfg(void)
{
	return DEVICE_DT_INST_GET(0)->config;
}

/* Half a TCK period. Hand wiring works fine well below 1 MHz; the GPIO
 * call overhead alone is a large part of the period, so a small delay
 * keeps the bit-bang comfortably slow without dragging the (multi-Mbit)
 * SRAM load out too far. */
static inline void jtag_delay(void)
{
#if CONFIG_AES67_FPGA_JTAG_TCK_DELAY_US > 0
	k_busy_wait(CONFIG_AES67_FPGA_JTAG_TCK_DELAY_US);
#endif
}

/* One TCK cycle without reading TDO (used for the config data stream and
 * TAP navigation). */
static inline void jtag_clk(const struct fpga_jtag_config *cfg, int tms, int tdi)
{
	gpio_pin_set_dt(&cfg->tms, tms);
	gpio_pin_set_dt(&cfg->tdi, tdi);
	jtag_delay();
	gpio_pin_set_dt(&cfg->tck, 1);
	jtag_delay();
	gpio_pin_set_dt(&cfg->tck, 0);
}

/* One TCK cycle returning the TDO bit shifted out this cycle. */
static inline int jtag_clk_io(const struct fpga_jtag_config *cfg, int tms, int tdi)
{
	int tdo;

	gpio_pin_set_dt(&cfg->tms, tms);
	gpio_pin_set_dt(&cfg->tdi, tdi);
	jtag_delay();
	tdo = gpio_pin_get_dt(&cfg->tdo);
	gpio_pin_set_dt(&cfg->tck, 1);
	jtag_delay();
	gpio_pin_set_dt(&cfg->tck, 0);

	return tdo;
}

/* Force the TAP to Test-Logic-Reset: >=5 TCK with TMS high. */
static void jtag_tap_reset(const struct fpga_jtag_config *cfg)
{
	for (int i = 0; i < 6; i++) {
		jtag_clk(cfg, 1, 0);
	}
}

/* Test-Logic-Reset -> Run-Test/Idle. */
static void jtag_tlr_to_rti(const struct fpga_jtag_config *cfg)
{
	jtag_clk(cfg, 0, 0);
}

/* Shift an instruction (ALTERA_IR_LEN bits, LSB first). Starts and ends in
 * Run-Test/Idle; the instruction takes effect at Update-IR. */
static void jtag_shift_ir(const struct fpga_jtag_config *cfg, uint16_t ir)
{
	/* RTI -> Select-DR -> Select-IR -> Capture-IR -> Shift-IR */
	jtag_clk(cfg, 1, 0);
	jtag_clk(cfg, 1, 0);
	jtag_clk(cfg, 0, 0);
	jtag_clk(cfg, 0, 0);

	for (int i = 0; i < ALTERA_IR_LEN; i++) {
		int last = (i == ALTERA_IR_LEN - 1);

		jtag_clk(cfg, last, (ir >> i) & 1);
	}

	/* Exit1-IR -> Update-IR -> Run-Test/Idle */
	jtag_clk(cfg, 1, 0);
	jtag_clk(cfg, 0, 0);
}

/* Enter Shift-DR from Run-Test/Idle. */
static void jtag_rti_to_shift_dr(const struct fpga_jtag_config *cfg)
{
	jtag_clk(cfg, 1, 0);
	jtag_clk(cfg, 0, 0);
	jtag_clk(cfg, 0, 0);
}

/* Exit Shift-DR (last bit already shifted with TMS=1 -> Exit1-DR) back to
 * Run-Test/Idle. */
static void jtag_exit1_dr_to_rti(const struct fpga_jtag_config *cfg)
{
	jtag_clk(cfg, 1, 0);
	jtag_clk(cfg, 0, 0);
}

/* Shift @nbits of @data (LSB first) out on TDI through DR, discarding TDO.
 * Starts and ends in Run-Test/Idle. If @cb is non-NULL it is called with
 * the percentage shifted so far, at most once per percent. */
static void jtag_shift_dr_out(const struct fpga_jtag_config *cfg,
			      const uint8_t *data, size_t nbits,
			      fpga_jtag_progress_cb cb, void *ctx)
{
	uint8_t last_pct = 0;

	jtag_rti_to_shift_dr(cfg);

	if (cb != NULL) {
		cb(0, ctx);
	}

	for (size_t i = 0; i < nbits; i++) {
		int bit = (data[i >> 3] >> (i & 7)) & 1;
		int last = (i == nbits - 1);

		jtag_clk(cfg, last, bit);

		if (cb != NULL) {
			uint8_t pct = (uint8_t)(((uint64_t)(i + 1) * 100) / nbits);

			if (pct != last_pct) {
				last_pct = pct;
				cb(pct, ctx);
			}
		}
	}

	jtag_exit1_dr_to_rti(cfg);
}

/* Shift @nbits through DR feeding zeros, collecting TDO into @rx (LSB
 * first). Starts and ends in Run-Test/Idle. */
static void jtag_shift_dr_in(const struct fpga_jtag_config *cfg,
			     uint8_t *rx, size_t nbits)
{
	memset(rx, 0, (nbits + 7) / 8);
	jtag_rti_to_shift_dr(cfg);

	for (size_t i = 0; i < nbits; i++) {
		int last = (i == nbits - 1);

		if (jtag_clk_io(cfg, last, 0)) {
			rx[i >> 3] |= (1U << (i & 7));
		}
	}

	jtag_exit1_dr_to_rti(cfg);
}

/* Run @n TCK cycles in Run-Test/Idle (TMS low). */
static void jtag_run_test_idle(const struct fpga_jtag_config *cfg, uint32_t n)
{
	for (uint32_t i = 0; i < n; i++) {
		jtag_clk(cfg, 0, 0);
	}
}

/* Read a single device's IDCODE from Shift-DR (32 bits, LSB first). */
static uint32_t jtag_read_idcode(const struct fpga_jtag_config *cfg)
{
	uint32_t val = 0;

	for (int i = 0; i < 32; i++) {
		if (jtag_clk_io(cfg, 0, 1)) {
			val |= (1U << i);
		}
	}

	return val;
}

static int fpga_jtag_scan_chain_locked(uint32_t *idcodes, size_t max)
{
	const struct fpga_jtag_config *cfg = jtag_cfg();
	int count = 0;

	jtag_tap_reset(cfg);
	jtag_tlr_to_rti(cfg);
	jtag_rti_to_shift_dr(cfg);

	for (size_t i = 0; i < max; i++) {
		uint32_t id = jtag_read_idcode(cfg);

		if (id == 0xFFFFFFFF) {
			break;
		}
		if (id == 0x00000000) {
			jtag_tap_reset(cfg);
			return count > 0 ? count : -EIO;
		}

		idcodes[i] = id;
		count++;
	}

	jtag_tap_reset(cfg);

	if (count == 0) {
		return -EIO;
	}

	return count;
}

int fpga_jtag_scan_chain(uint32_t *idcodes, size_t max)
{
	int ret;

	k_mutex_lock(&fpga_jtag_lock, K_FOREVER);
	ret = fpga_jtag_scan_chain_locked(idcodes, max);
	k_mutex_unlock(&fpga_jtag_lock);

	return ret;
}

const char *fpga_jtag_idcode_name(uint32_t idcode)
{
	for (size_t i = 0; i < ARRAY_SIZE(fpga_jtag_ids); i++) {
		if (fpga_jtag_ids[i].idcode == (idcode & 0x0FFFFFFF)) {
			return fpga_jtag_ids[i].name;
		}
	}

	return NULL;
}

static int fpga_jtag_configure(const uint8_t *data, size_t len,
			       fpga_jtag_progress_cb cb, void *ctx)
{
	const struct fpga_jtag_config *cfg = jtag_cfg();
	static uint8_t status_dr[(ALTERA_STATUS_DR_BITS + 7) / 8];
	uint32_t start = k_uptime_get_32();

	if (data == NULL || len == 0) {
		return -EINVAL;
	}

	jtag_tap_reset(cfg);
	jtag_tlr_to_rti(cfg);

	/* This is a 1:1 replay of the Quartus JTAG SVF sequence for this
	 * device. @data is the JTAG-format configuration blob extracted from
	 * that SVF (svf_to_bin.py) — NOT the serial RBF, which Altera
	 * organises differently and which does not configure over JTAG. */

	/* PROGRAM, then idle while the device prepares to receive data. */
	jtag_shift_ir(cfg, ALTERA_IR_PROGRAM);
	jtag_run_test_idle(cfg, ALTERA_TCK_PROGRAM);

	/* Stream the configuration data through DR, LSB first. */
	jtag_shift_dr_out(cfg, data, len * 8, cb, ctx);

	/* CHECK_STATUS: shift the status DR and test the CONF_DONE bit that
	 * the Quartus SVF's own MASK selects. Unlike the earlier (wrong,
	 * 864-bit) DR, this is the exact register/bit Quartus verifies. */
	jtag_shift_ir(cfg, ALTERA_IR_CHECK_STATUS);
	jtag_run_test_idle(cfg, ALTERA_TCK_CHECK);
	jtag_shift_dr_in(cfg, status_dr, ALTERA_STATUS_DR_BITS);

	int done = (status_dr[ALTERA_STATUS_DONE_BIT >> 3] >>
		    (ALTERA_STATUS_DONE_BIT & 7)) & 1;

	/* STARTUP: release the device into user mode + init clocks. */
	jtag_shift_ir(cfg, ALTERA_IR_STARTUP);
	jtag_run_test_idle(cfg, ALTERA_TCK_STARTUP);
	jtag_run_test_idle(cfg, ALTERA_TCK_STARTUP2);

	/* Park the TAP in BYPASS; the SVF ends in Run-Test/Idle (no TAP
	 * reset — that differs from openFPGALoader and was in the old code). */
	jtag_shift_ir(cfg, ALTERA_IR_BYPASS);
	jtag_run_test_idle(cfg, ALTERA_TCK_BYPASS);

	LOG_DBG("JTAG sequence done: %zu bytes in %u ms, CONF_DONE hint=%d",
		len, k_uptime_get_32() - start, done);
	return done ? 0 : -EIO;
}

/* Read the 32-bit USERCODE register (Quartus Auto-Usercode = the
 * programming-file checksum). After a good configuration it reads that
 * checksum; otherwise it reads the device default. A full 32-bit compare
 * is a far stronger success test than the single CONF_DONE status bit,
 * and — being read over the same marginal wiring — a match is extremely
 * unlikely to occur by chance, so it also gates the retry loop. */
uint32_t fpga_jtag_read_usercode(void)
{
	const struct fpga_jtag_config *cfg = jtag_cfg();
	uint32_t uc;

	k_mutex_lock(&fpga_jtag_lock, K_FOREVER);
	jtag_tap_reset(cfg);
	jtag_tlr_to_rti(cfg);
	jtag_shift_ir(cfg, ALTERA_IR_USERCODE);
	jtag_rti_to_shift_dr(cfg);
	uc = jtag_read_idcode(cfg);   /* 32 bits, LSB first, from the current DR */
	jtag_tap_reset(cfg);
	k_mutex_unlock(&fpga_jtag_lock);

	return uc;
}

/* SAMPLE the boundary-scan register (a snapshot of every pin) and return
 * the CRC_ERROR pin's input cell. Only meaningful on a configured device. */
static bool fpga_jtag_crc_error_locked(const struct fpga_jtag_config *cfg)
{
	uint8_t bsc[(ALTERA_STATUS_DR_BITS + 7) / 8];

	jtag_tap_reset(cfg);
	jtag_tlr_to_rti(cfg);
	jtag_shift_ir(cfg, ALTERA_IR_SAMPLE);
	jtag_shift_dr_in(cfg, bsc, ALTERA_STATUS_DR_BITS);
	jtag_tap_reset(cfg);

	return (bsc[ALTERA_BSC_CRC_ERROR_BIT >> 3] >>
		(ALTERA_BSC_CRC_ERROR_BIT & 7)) & 1;
}

enum fpga_jtag_health fpga_jtag_health_check(void)
{
	const struct fpga_jtag_config *cfg = jtag_cfg();
	uint32_t idcodes[FPGA_JTAG_MAX_DEVICES];
	enum fpga_jtag_health res = FPGA_JTAG_HEALTH_OK;
	int n;

	k_mutex_lock(&fpga_jtag_lock, K_FOREVER);

	/* Gate everything on the IDCODE: it proves the whole
	 * TCK/TMS/TDI/TDO round trip. Without it a wiring dropout would be
	 * indistinguishable from a lost configuration — both read USERCODE
	 * as all-ones. */
	n = fpga_jtag_scan_chain_locked(idcodes, ARRAY_SIZE(idcodes));
	if (n != 1 || fpga_jtag_idcode_name(idcodes[0]) == NULL) {
		res = FPGA_JTAG_HEALTH_LINK_DOWN;
		goto out;
	}

#if defined(CONFIG_AES67_FPGA_JTAG_BOOT_LOAD) && (FPGA_JTAG_USERCODE != 0u)
	if (fpga_jtag_read_usercode() != FPGA_JTAG_USERCODE) {
		res = FPGA_JTAG_HEALTH_UNCONFIGURED;
		goto out;
	}
#endif

	if (fpga_jtag_crc_error_locked(cfg)) {
		res = FPGA_JTAG_HEALTH_CRC_ERROR;
	}
out:
	k_mutex_unlock(&fpga_jtag_lock);

	return res;
}

bool fpga_jtag_crc_error(void)
{
	const struct fpga_jtag_config *cfg = jtag_cfg();
	bool err;

	k_mutex_lock(&fpga_jtag_lock, K_FOREVER);
	err = fpga_jtag_crc_error_locked(cfg);
	k_mutex_unlock(&fpga_jtag_lock);

	return err;
}

int fpga_jtag_sample_boundary(uint8_t *buf, size_t buf_len)
{
	const struct fpga_jtag_config *cfg = jtag_cfg();

	if (buf == NULL || buf_len < (ALTERA_STATUS_DR_BITS + 7) / 8) {
		return -EINVAL;
	}

	k_mutex_lock(&fpga_jtag_lock, K_FOREVER);
	jtag_tap_reset(cfg);
	jtag_tlr_to_rti(cfg);
	jtag_shift_ir(cfg, ALTERA_IR_SAMPLE);
	jtag_shift_dr_in(cfg, buf, ALTERA_STATUS_DR_BITS);
	jtag_tap_reset(cfg);
	k_mutex_unlock(&fpga_jtag_lock);

	return ALTERA_STATUS_DR_BITS;
}

const char *fpga_jtag_health_str(enum fpga_jtag_health health)
{
	switch (health) {
	case FPGA_JTAG_HEALTH_OK:
		return "OK";
	case FPGA_JTAG_HEALTH_LINK_DOWN:
		return "JTAG chain unreadable";
	case FPGA_JTAG_HEALTH_UNCONFIGURED:
		return "configuration lost (USERCODE mismatch)";
	case FPGA_JTAG_HEALTH_CRC_ERROR:
		return "configuration SRAM corrupt (CRC_ERROR latched)";
	default:
		return "?";
	}
}

/* Signal-integrity self-test: shift a pseudo-random pattern through the
 * device's 1-bit BYPASS register (TDI -> one flop -> TDO) at the exact TCK
 * rate the configuration uses, and count how many bits come back wrong. In
 * BYPASS the bit shifted out equals the previous bit shifted in (the
 * register captures 0 at Shift-DR entry), so the expected TDO is just the
 * previously sent bit. This exercises the same TCK/TDI/TDO wiring as
 * configuration and turns "config sometimes fails" into a hard number: a
 * non-zero error count means the hand-wiring corrupts the config stream, so
 * raise CONFIG_AES67_FPGA_JTAG_TCK_DELAY_US / fix the wiring until it is 0.
 * Returns the bit-error count (0 = clean link). */
static uint32_t fpga_jtag_bypass_link_test(const struct fpga_jtag_config *cfg,
					   uint32_t nbits)
{
	uint32_t rng = 0x1234ABCDU;	/* xorshift32 PRNG, any non-zero seed */
	uint32_t errors = 0;
	int prev = 0;			/* BYPASS captures 0 at Shift-DR entry */

	jtag_tap_reset(cfg);
	jtag_tlr_to_rti(cfg);
	jtag_shift_ir(cfg, ALTERA_IR_BYPASS);
	jtag_rti_to_shift_dr(cfg);

	for (uint32_t i = 0; i < nbits; i++) {
		int last = (i == nbits - 1);
		int s;

		rng ^= rng << 13;
		rng ^= rng >> 17;
		rng ^= rng << 5;
		s = rng & 1;

		/* jtag_clk_io reads TDO (the bit shifted out this cycle, i.e.
		 * the previously sent bit) before clocking in the new bit. */
		if (jtag_clk_io(cfg, last, s) != prev) {
			errors++;
		}
		prev = s;
	}

	jtag_exit1_dr_to_rti(cfg);
	jtag_tap_reset(cfg);

	return errors;
}

#if defined(CONFIG_AES67_FPGA_JTAG_SCAN_AT_BOOT) && \
	!defined(CONFIG_AES67_FPGA_JTAG_BOOT_LOAD)
static void fpga_jtag_report_scan(void)
{
	uint32_t idcodes[FPGA_JTAG_MAX_DEVICES];
	int n = fpga_jtag_scan_chain(idcodes, ARRAY_SIZE(idcodes));

	if (n < 0) {
		LOG_ERR("JTAG chain scan failed (err %d): check TCK/TMS/TDI/TDO "
			"wiring and FPGA power", n);
		return;
	}

	LOG_INF("JTAG chain: %d device(s)", n);
	for (int i = 0; i < n; i++) {
		const char *name = fpga_jtag_idcode_name(idcodes[i]);

		LOG_INF("  [%d] IDCODE 0x%08X  %s", i, idcodes[i],
			name ? name : "(unknown device)");
	}
}
#endif

#ifdef CONFIG_AES67_FPGA_JTAG_BOOT_LOAD
static const uint8_t fpga_jtag_bitstream[] = {
#include "fpga_bitstream_jtag.inc"
};

/* The retried, USERCODE-verified configuration itself — shared by the boot
 * load and the health monitor's forced reload. Caller holds the lock. */
static int fpga_jtag_do_load(fpga_jtag_progress_cb cb, void *ctx)
{
	for (int attempt = 1; attempt <= CONFIG_AES67_FPGA_JTAG_CONFIG_RETRIES;
	     attempt++) {
		(void)fpga_jtag_configure(fpga_jtag_bitstream,
					  sizeof(fpga_jtag_bitstream), cb, ctx);

#if FPGA_JTAG_USERCODE != 0u
		uint32_t uc = fpga_jtag_read_usercode();

		if (uc == FPGA_JTAG_USERCODE) {
			LOG_INF("FPGA configured over JTAG, USERCODE 0x%08X "
				"verified (attempt %d)", uc, attempt);
			return 0;
		}

		LOG_WRN("Config attempt %d/%d: USERCODE 0x%08X != expected "
			"0x%08X, retrying", attempt,
			CONFIG_AES67_FPGA_JTAG_CONFIG_RETRIES, uc,
			FPGA_JTAG_USERCODE);
#else
		/* No USERCODE in the SVF: cannot verify, trust the first pass. */
		LOG_WRN("No USERCODE in SVF - configured without read-back "
			"verification");
		return 0;
#endif
	}

	LOG_ERR("FPGA JTAG configuration failed after %d attempts (USERCODE "
		"never matched). Likely TDI/TCK signal integrity - raise "
		"CONFIG_AES67_FPGA_JTAG_TCK_DELAY_US or improve the wiring.",
		CONFIG_AES67_FPGA_JTAG_CONFIG_RETRIES);
	return -EIO;
}

static int fpga_jtag_boot_load_locked(fpga_jtag_progress_cb cb, void *ctx)
{
	uint32_t idcodes[FPGA_JTAG_MAX_DEVICES];
	int n = fpga_jtag_scan_chain_locked(idcodes, ARRAY_SIZE(idcodes));

	/* Verify a single, known device before shifting a bitstream into
	 * it: JTAG has a read-back, so unlike Passive Serial we never
	 * configure blind. */
	if (n < 0) {
		LOG_ERR("JTAG scan failed (err %d); not configuring FPGA", n);
		return n;
	}
	if (n != 1) {
		LOG_ERR("Expected a single device on the JTAG chain, found %d; "
			"not configuring", n);
		return -ENOTSUP;
	}

	const char *name = fpga_jtag_idcode_name(idcodes[0]);

	if (name == NULL) {
		LOG_ERR("Unknown IDCODE 0x%08X on JTAG chain; not configuring",
			idcodes[0]);
		return -ENODEV;
	}

	LOG_INF("JTAG chain: %s (IDCODE 0x%08X)", name, idcodes[0]);

#if FPGA_JTAG_USERCODE != 0u
	/* Skip the upload if the FPGA already holds this exact bitstream — a
	 * warm reset that kept the SRAM, or a device programmed out-of-band
	 * (e.g. a USB-Blaster). The USERCODE is the bitstream's checksum. */
	{
		uint32_t uc = fpga_jtag_read_usercode();

		if (uc == FPGA_JTAG_USERCODE) {
			LOG_INF("FPGA already configured (USERCODE 0x%08X) - "
				"skipping bitstream upload", uc);
			return 1;
		}
		LOG_INF("FPGA not configured (USERCODE 0x%08X) - uploading "
			"bitstream", uc);
	}
#endif

#if CONFIG_AES67_FPGA_JTAG_LINK_TEST_BITS > 0
	{
		const struct fpga_jtag_config *cfg = jtag_cfg();
		uint32_t nbits = CONFIG_AES67_FPGA_JTAG_LINK_TEST_BITS;
		uint32_t errs = fpga_jtag_bypass_link_test(cfg, nbits);

		if (errs == 0) {
			LOG_INF("JTAG link test: %u/%u bit errors - clean, wiring "
				"can carry the config stream", errs, nbits);
		} else {
			LOG_ERR("JTAG link test: %u/%u bit errors (BER ~%u.%02u%%) "
				"- the wiring corrupts data; JTAG config will not "
				"succeed until this is 0. Raise "
				"CONFIG_AES67_FPGA_JTAG_TCK_DELAY_US, shorten the "
				"leads, add a TCK pull-down and a solid GND return.",
				errs, nbits, (errs * 100U) / nbits,
				((errs * 10000U) / nbits) % 100U);
		}
	}
#endif

	LOG_INF("Configuring %s over JTAG (%zu bytes)", name,
		sizeof(fpga_jtag_bitstream));

	return fpga_jtag_do_load(cb, ctx);
}

int fpga_jtag_boot_load(fpga_jtag_progress_cb cb, void *ctx)
{
	int ret;

	k_mutex_lock(&fpga_jtag_lock, K_FOREVER);
	ret = fpga_jtag_boot_load_locked(cb, ctx);
	k_mutex_unlock(&fpga_jtag_lock);

	return ret;
}

int fpga_jtag_reload(fpga_jtag_progress_cb cb, void *ctx)
{
	int ret;

	k_mutex_lock(&fpga_jtag_lock, K_FOREVER);
	LOG_WRN("Force-reloading the FPGA bitstream over JTAG (%zu bytes)",
		sizeof(fpga_jtag_bitstream));
	ret = fpga_jtag_do_load(cb, ctx);
	k_mutex_unlock(&fpga_jtag_lock);

	return ret;
}
#endif

#ifdef CONFIG_AES67_FPGA_JTAG_MONITOR
static K_THREAD_STACK_DEFINE(fpga_jtag_monitor_stack,
			     CONFIG_AES67_FPGA_JTAG_MONITOR_STACK_SIZE);
static struct k_thread fpga_jtag_monitor_thread;
static fpga_jtag_event_cb fpga_jtag_monitor_event_cb;
static fpga_jtag_progress_cb fpga_jtag_monitor_progress_cb;
static void *fpga_jtag_monitor_ctx;

/* A failed reload almost always fails again immediately (wiring, dead
 * device) — pace the multi-second upload attempts instead of running them
 * back to back. */
#define FPGA_JTAG_MONITOR_RELOAD_HOLDOFF_MS (30 * MSEC_PER_SEC)

/* Rate-limit the link-down warning to roughly every 30 s of ticks. */
#define FPGA_JTAG_MONITOR_LINK_LOG_TICKS 30U

/* CRC-based recovery must prove itself before it may reconfigure: the
 * CRC_ERROR sample is trusted only after it has read clear for this many
 * consecutive checks after the monitor starts (long enough for at least
 * one full background CRC pass over the SRAM). If it is asserted during
 * the window, either the pin/BSC readout is unusable on this board or —
 * the documented failure mode of this hand wiring — every JTAG load
 * leaves residual bit errors that a reload can never fix; reconfiguring
 * (and rebooting) on that signal would loop forever. In both cases CRC
 * recovery is disabled for the session and only logged; IDCODE/USERCODE
 * monitoring stays armed. */
#define FPGA_JTAG_MONITOR_CRC_BASELINE_TICKS 10U

static void fpga_jtag_monitor_fn(void *p1, void *p2, void *p3)
{
	uint32_t link_down_ticks = 0;
	uint32_t crc_clean_ticks = 0;
	uint32_t crc_dead_ticks = 0;
	bool crc_armed = false;
	bool crc_dead = false;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	for (;;) {
		k_msleep(CONFIG_AES67_FPGA_JTAG_MONITOR_INTERVAL_MS);

		enum fpga_jtag_health h = fpga_jtag_health_check();

		if (h == FPGA_JTAG_HEALTH_OK) {
			if (link_down_ticks != 0) {
				LOG_INF("FPGA health: JTAG chain readable again");
				link_down_ticks = 0;
			}
			if (!crc_armed && !crc_dead &&
			    ++crc_clean_ticks >= FPGA_JTAG_MONITOR_CRC_BASELINE_TICKS) {
				crc_armed = true;
				LOG_INF("FPGA health: CRC_ERROR clear through the "
					"baseline window - SEU auto-recovery armed");
			}
			continue;
		}

		if (h == FPGA_JTAG_HEALTH_CRC_ERROR && !crc_armed) {
			if (!crc_dead) {
				crc_dead = true;
				LOG_ERR("FPGA health: CRC_ERROR already asserted "
					"during the baseline window. Either this "
					"JTAG load left residual bit errors in the "
					"configuration SRAM (marginal wiring - a "
					"reload cannot fix that) or the pin sample "
					"is unusable. CRC auto-recovery DISABLED "
					"for this session; IDCODE/USERCODE "
					"monitoring continues. Diagnose: load via "
					"USB-Blaster, then run 'fpga jtag'.");
			} else if ((crc_dead_ticks++ % 600U) == 0U) {
				LOG_WRN("FPGA health: CRC_ERROR still asserted "
					"(auto-recovery disabled)");
			}
			continue;
		}

		if (h == FPGA_JTAG_HEALTH_LINK_DOWN) {
			/* A dead chain is a wiring/power problem, not proof of
			 * an FPGA fault — and a reload over it cannot succeed.
			 * Keep watching instead of reconfiguring blind. */
			if ((link_down_ticks++ %
			     FPGA_JTAG_MONITOR_LINK_LOG_TICKS) == 0U) {
				LOG_WRN("FPGA health: JTAG chain unreadable - "
					"check wiring/power; not reconfiguring");
			}
			continue;
		}
		link_down_ticks = 0;

		/* Confirm before committing to a multi-second reconfiguration:
		 * both fault states are persistent (CRC_ERROR stays latched
		 * until reconfig, a lost configuration stays lost), so a
		 * genuine fault reads identically twice — a single corrupted
		 * shift on marginal hand-wiring does not. */
		if (fpga_jtag_health_check() != h) {
			LOG_WRN("FPGA health: transient \"%s\" reading - ignored",
				fpga_jtag_health_str(h));
			continue;
		}

		LOG_ERR("FPGA fault: %s - muting outputs and reloading the "
			"bitstream", fpga_jtag_health_str(h));

		if (fpga_jtag_monitor_event_cb != NULL) {
			fpga_jtag_monitor_event_cb(FPGA_JTAG_EVT_FAULT, h,
						   fpga_jtag_monitor_ctx);
		}

		if (fpga_jtag_reload(fpga_jtag_monitor_progress_cb,
				     fpga_jtag_monitor_ctx) == 0) {
			if (fpga_jtag_monitor_event_cb != NULL) {
				/* The handler is expected to reboot; if it
				 * returns instead, resume watching. */
				fpga_jtag_monitor_event_cb(
					FPGA_JTAG_EVT_RECOVERED, h,
					fpga_jtag_monitor_ctx);
			}
		} else {
			if (fpga_jtag_monitor_event_cb != NULL) {
				fpga_jtag_monitor_event_cb(
					FPGA_JTAG_EVT_RELOAD_FAILED, h,
					fpga_jtag_monitor_ctx);
			}
			k_msleep(FPGA_JTAG_MONITOR_RELOAD_HOLDOFF_MS);
		}
	}
}

int fpga_jtag_monitor_start(fpga_jtag_event_cb evt_cb,
			    fpga_jtag_progress_cb progress_cb, void *ctx)
{
	static bool started;

	if (started) {
		return -EALREADY;
	}
	started = true;

	fpga_jtag_monitor_event_cb = evt_cb;
	fpga_jtag_monitor_progress_cb = progress_cb;
	fpga_jtag_monitor_ctx = ctx;

	k_thread_create(&fpga_jtag_monitor_thread, fpga_jtag_monitor_stack,
			K_THREAD_STACK_SIZEOF(fpga_jtag_monitor_stack),
			fpga_jtag_monitor_fn, NULL, NULL, NULL,
			K_PRIO_PREEMPT(10), 0, K_NO_WAIT);
	k_thread_name_set(&fpga_jtag_monitor_thread, "fpga_jtag_mon");

	LOG_INF("FPGA health monitor started (every %d ms)",
		CONFIG_AES67_FPGA_JTAG_MONITOR_INTERVAL_MS);

	return 0;
}
#endif /* CONFIG_AES67_FPGA_JTAG_MONITOR */

static int fpga_jtag_init(const struct device *dev)
{
	const struct fpga_jtag_config *cfg = dev->config;
	int ret;

	ret = gpio_pin_configure_dt(&cfg->tck, GPIO_OUTPUT_INACTIVE);
	ret |= gpio_pin_configure_dt(&cfg->tms, GPIO_OUTPUT_ACTIVE);
	ret |= gpio_pin_configure_dt(&cfg->tdi, GPIO_OUTPUT_INACTIVE);
	ret |= gpio_pin_configure_dt(&cfg->tdo, GPIO_INPUT);
	if (ret) {
		LOG_ERR("Failed to configure JTAG GPIOs");
		return -EIO;
	}

	/* The GPIOs are now driven safely. Configuration itself is
	 * orchestrated later from main() via fpga_jtag_boot_load() so the
	 * display is up and the upload can report progress. When boot-load is
	 * disabled we still scan the chain here as a wiring/bring-up aid. */
#if !defined(CONFIG_AES67_FPGA_JTAG_BOOT_LOAD) && \
	defined(CONFIG_AES67_FPGA_JTAG_SCAN_AT_BOOT)
	fpga_jtag_report_scan();
#endif

	return 0;
}

static const struct fpga_jtag_config fpga_jtag_config_inst = {
	.tck = GPIO_DT_SPEC_INST_GET(0, tck_gpios),
	.tms = GPIO_DT_SPEC_INST_GET(0, tms_gpios),
	.tdi = GPIO_DT_SPEC_INST_GET(0, tdi_gpios),
	.tdo = GPIO_DT_SPEC_INST_GET(0, tdo_gpios),
};

DEVICE_DT_INST_DEFINE(0, fpga_jtag_init, NULL, NULL, &fpga_jtag_config_inst,
		      POST_KERNEL, CONFIG_AES67_FPGA_JTAG_INIT_PRIORITY, NULL);
