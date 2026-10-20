/*
 * Auracast Card Driver - see auracast_card.h for the model.
 */

#include "auracast_card.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/atomic.h>
#include <errno.h>
#include <string.h>

LOG_MODULE_REGISTER(auracast_card, CONFIG_AURACAST_CARD_LOG_LEVEL);

#define AURA_ADDR            CONFIG_AURACAST_CARD_I2C_ADDR

#define POLL_INTERVAL_MS     1000
#define START_SETTLE_MS      50     /* START -> first STATE read */
#define START_WAIT_MS        1500   /* STARTING -> STREAMING/FAULT, polled */
#define RESET_POLL_MS        100    /* MAGIC probe interval after RESET */
#define RESET_TIMEOUT_MS     3000   /* S31 restarts in well under 1 s */
#define PRESENT_FAIL_LIMIT   3      /* consecutive poll failures -> absent */

/* Retry delays after a failed recovery cycle (S31 restarted, FAULT, ...). */
static const uint32_t backoff_ms[] = { 1000, 2000, 5000, 10000 };

/* ---------------------------------------------------------------------
 * State
 * ------------------------------------------------------------------- */

static const struct device *s_i2c;
static bool s_ready;
static atomic_t s_outputs_allowed; /* PTP lock seen (set without s_lock) */
static bool s_activated_once;      /* first cycle after activate() done */
static bool s_expect_running;      /* we started (or adopted) a broadcast */
static bool s_retry_pending;       /* last cycle failed for a transient reason */
static uint8_t s_backoff_idx;
static uint8_t s_fail_count;
static int64_t s_last_ok_uptime;

static struct auracast_config s_cfg = {
	/* Filled with aura_proto_config_defaults() on first use. */
	.name = "",
};
static bool s_cfg_initialised;
static struct auracast_status s_status;

/* s_lock serialises the I2C cycle (which sleeps while the S31 restarts).
 * Readers must never wait on it: the status and configuration are
 * published as snapshots under a spinlock after every change. */
static K_MUTEX_DEFINE(s_lock);
static struct k_spinlock s_pub_lock;
static struct auracast_status s_pub_status;
static struct auracast_config s_pub_cfg;
static bool s_pub_ready;

/* The configuration cycle sleeps up to RESET_TIMEOUT_MS while the S31
 * restarts; that must not stall the system work queue (networking,
 * logging), so the driver owns a small work queue of its own. */
#define WQ_STACK_SIZE 4096
#define WQ_PRIORITY   K_PRIO_PREEMPT(10)
K_THREAD_STACK_DEFINE(s_wq_stack, WQ_STACK_SIZE);
static struct k_work_q s_wq;
static bool s_wq_started;

static void poll_fn(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(s_poll_work, poll_fn);

/* ---------------------------------------------------------------------
 * I2C primitives
 * ------------------------------------------------------------------- */

/*
 * Every read takes exactly AURA_READ_MAX bytes, the amount the S31 queues
 * per request. The ESP32 slave hardware pre-loads the next FIFO byte into
 * its shift register as soon as the master stops early; a FIFO reset does
 * not remove it, so a shorter read would prefix every later read with one
 * stale byte, for good. Reading the whole fill leaves nothing behind.
 */
static int rd_raw(const struct device *dev, uint8_t reg, uint8_t *buf, size_t len)
{
	return i2c_write_read(dev, AURA_ADDR, &reg, 1, buf, len);
}

static int rd(uint8_t reg, uint8_t *buf, size_t len)
{
	uint8_t raw[AURA_READ_MAX];
	int ret;

	__ASSERT(len <= AURA_READ_MAX, "read longer than the S31 TX FIFO");

	ret = rd_raw(s_i2c, reg, raw, sizeof(raw));
	if (ret < 0) {
		LOG_DBG("read reg 0x%02x failed: %d", reg, ret);
		return ret;
	}
	memcpy(buf, raw, len);
	return 0;
}

/*
 * Identity probe, shared with the card manager. Reads MAGIC..FW_VERSION
 * and, when the answer is shifted by one stale byte (see rd()), drains
 * the slave with one AURA_READ_MAX + 1 byte read and retries once.
 */
int auracast_card_probe(const struct device *dev, uint8_t ident[AURA_IDENT_LEN])
{
	uint8_t raw[AURA_READ_MAX + 1];
	int ret;

	for (int attempt = 0; attempt < 2; attempt++) {
		ret = rd_raw(dev, AURA_REG_MAGIC, raw, AURA_READ_MAX);
		if (ret < 0) {
			return ret;
		}
		if (memcmp(raw, "AURA", 4) == 0) {
			memcpy(ident, raw, AURA_IDENT_LEN);
			return 0;
		}
		if (attempt == 0 && memcmp(raw + 1, "AURA", 4) == 0) {
			LOG_WRN("S31 answer shifted by one stale byte, flushing its TX FIFO");
			ret = rd_raw(dev, AURA_REG_MAGIC, raw, sizeof(raw));
			if (ret < 0) {
				return ret;
			}
			continue;
		}
		break;
	}
	memcpy(ident, raw, AURA_IDENT_LEN);
	return -ENODEV;
}

static int wr(uint8_t reg, const uint8_t *buf, size_t len)
{
	int ret = i2c_burst_write(s_i2c, AURA_ADDR, reg, buf, len);

	if (ret < 0) {
		LOG_WRN("write reg 0x%02x (%u bytes) failed: %d", reg, (unsigned int)len, ret);
	}
	return ret;
}

static int wr_cmd(uint8_t cmd)
{
	return wr(AURA_REG_COMMAND, &cmd, 1);
}

/* ---------------------------------------------------------------------
 * Register reads into the cache (caller holds s_lock)
 * ------------------------------------------------------------------- */

static void ensure_defaults_locked(void)
{
	if (!s_cfg_initialised) {
		aura_proto_config_defaults(&s_cfg);
		s_cfg_initialised = true;
	}
}

/* Copy the live state into the reader snapshot (caller holds s_lock). */
static void publish_locked(void)
{
	k_spinlock_key_t key = k_spin_lock(&s_pub_lock);

	s_pub_status = s_status;
	s_pub_cfg = s_cfg;
	s_pub_ready = s_ready;
	k_spin_unlock(&s_pub_lock, key);
}

/* STATE, ERROR, ERROR_REG, RESET_REASON. */
static int read_state_locked(void)
{
	uint8_t buf[4];
	int ret = rd(AURA_REG_STATE, buf, sizeof(buf));

	if (ret < 0) {
		return ret;
	}
	s_status.state = buf[0];
	s_status.error = buf[1];
	s_status.error_reg = buf[2];
	s_status.reset_reason = buf[3];
	s_status.stats_valid = (buf[0] == AURA_STATE_STARTING ||
				buf[0] == AURA_STATE_STREAMING);
	return 0;
}

/* MAGIC, PROTO_VERSION, FW_VERSION; then the state registers. */
static int read_ident_locked(void)
{
	uint8_t buf[AURA_IDENT_LEN];
	int ret = auracast_card_probe(s_i2c, buf);

	if (ret < 0) {
		LOG_WRN("no MAGIC at 0x%02x (%02x %02x %02x %02x)", AURA_ADDR,
			buf[0], buf[1], buf[2], buf[3]);
		return -ENODEV;
	}
	if (buf[4] != AURA_PROTO_VERSION) {
		LOG_WRN("S31 protocol version %u, driver speaks %u", buf[4],
			AURA_PROTO_VERSION);
		return -ENOTSUP;
	}
	s_status.fw_major = buf[5];
	s_status.fw_minor = buf[6];
	s_status.fw_patch = buf[7];

	return read_state_locked();
}

static int read_live_config_locked(struct auracast_config *out)
{
	uint8_t block[AURA_CFG_BLOCK_LEN];
	uint8_t name[AURA_NAME_SIZE];
	uint8_t code[AURA_CODE_SIZE];
	int ret;

	ret = rd(AURA_CFG_BLOCK_REG, block, sizeof(block));
	if (ret < 0) {
		return ret;
	}
	/* NAME is 32 bytes: two reads, the FIFO holds 31. */
	ret = rd(AURA_REG_NAME, name, AURA_NAME_SIZE / 2);
	if (ret < 0) {
		return ret;
	}
	ret = rd(AURA_REG_NAME + AURA_NAME_SIZE / 2, name + AURA_NAME_SIZE / 2,
		 AURA_NAME_SIZE / 2);
	if (ret < 0) {
		return ret;
	}
	ret = rd(AURA_REG_CODE, code, AURA_CODE_SIZE);
	if (ret < 0) {
		return ret;
	}

	aura_proto_unpack_config(block, name, code, out);
	return 0;
}

static int read_status_block_locked(void)
{
	uint8_t raw[AURA_STATUS_LEN];
	const size_t first = AURA_READ_MAX;                               /* 31 */
	int ret;

	ret = rd(AURA_REG_PIPE_STATE, raw, first);
	if (ret < 0) {
		return ret;
	}
	ret = rd(AURA_REG_PIPE_STATE + first, raw + first, AURA_STATUS_LEN - first); /* 13 */
	if (ret < 0) {
		return ret;
	}
	aura_proto_unpack_status(raw, &s_status.stats);
	return 0;
}

/* ---------------------------------------------------------------------
 * Configuration cycle (caller holds s_lock)
 * ------------------------------------------------------------------- */

static int read_uptime_locked(uint32_t *uptime_ms)
{
	uint8_t b[4];
	int ret = rd(AURA_REG_UPTIME_MS, b, sizeof(b));

	if (ret == 0) {
		*uptime_ms = (uint32_t)b[0] | ((uint32_t)b[1] << 8) |
			     ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
	}
	return ret;
}

/*
 * After RESET: wait until the S31 answers again and sits in WAIT_CONFIG.
 * @p uptime_before (UPTIME_MS read before the RESET) tells a restart
 * apart from a RESET that was ignored: an S31 that comes back with a
 * smaller uptime but already STREAMING was built with AURACAST_AUTOSTART
 * and cannot be configured over I2C at all (-ENOTSUP, no retry).
 */
static int wait_wait_config_locked(uint32_t uptime_before)
{
	uint8_t magic[4];
	uint8_t state;
	uint32_t uptime;

	for (int elapsed = 0; elapsed <= RESET_TIMEOUT_MS; elapsed += RESET_POLL_MS) {
		k_msleep(RESET_POLL_MS);
		if (rd(AURA_REG_MAGIC, magic, sizeof(magic)) < 0 ||
		    memcmp(magic, "AURA", 4) != 0 ||
		    rd(AURA_REG_STATE, &state, 1) < 0) {
			continue;   /* restarting */
		}
		if (state == AURA_STATE_WAIT_CONFIG) {
			LOG_INF("S31 back in WAIT_CONFIG after %d ms", elapsed + RESET_POLL_MS);
			return 0;
		}
		if (elapsed >= 1000 && read_uptime_locked(&uptime) == 0 &&
		    uptime < uptime_before) {
			LOG_ERR("S31 restarted straight into %s (uptime %u ms): built with "
				"AURACAST_AUTOSTART, it cannot be configured over I2C",
				aura_proto_state_name(state), uptime);
			return -ENOTSUP;
		}
	}
	LOG_ERR("S31 did not come back within %d ms after RESET (state %s)",
		RESET_TIMEOUT_MS, aura_proto_state_name(state));
	return -ETIMEDOUT;
}

static int write_config_locked(void)
{
	uint8_t block[AURA_CFG_BLOCK_LEN];
	uint8_t name[AURA_NAME_SIZE];
	uint8_t code[AURA_CODE_SIZE];
	uint8_t err[2];
	int ret;

	aura_proto_pack_config(&s_cfg, block, name, code);

	ret = wr(AURA_CFG_BLOCK_REG, block, sizeof(block));
	if (ret == 0) {
		ret = wr(AURA_REG_NAME, name, sizeof(name));
	}
	if (ret == 0) {
		ret = wr(AURA_REG_CODE, code, sizeof(code));
	}
	if (ret < 0) {
		return -ENODEV;
	}

	/* Rejected writes are ACKed; the S31 reports them here instead. */
	ret = rd(AURA_REG_ERROR, err, sizeof(err));
	if (ret < 0) {
		return -ENODEV;
	}
	if (err[0] != AURA_ERR_NONE) {
		s_status.error = err[0];
		s_status.error_reg = err[1];
		LOG_ERR("S31 rejected configuration: %s (%s)",
			aura_proto_error_name(err[0]), aura_proto_reg_name(err[1]));
		return -EINVAL;
	}
	return 0;
}

/*
 * Bring the S31 in line with s_cfg. With @p adopt, a broadcast already
 * running with an identical configuration is kept.
 */
static int cycle_locked(bool adopt)
{
	int ret;

	ensure_defaults_locked();

	ret = read_state_locked();
	if (ret < 0) {
		ret = -ENODEV;
		goto out;
	}

	if (adopt && s_status.stats_valid && s_cfg.enabled) {
		struct auracast_config live;

		if (read_live_config_locked(&live) == 0 &&
		    aura_proto_config_matches(&s_cfg, &live)) {
			LOG_INF("adopted running broadcast \"%s\" (%s)", live.name,
				aura_proto_state_name(s_status.state));
			s_expect_running = true;
			ret = 0;
			goto out;
		}
		LOG_INF("running broadcast differs from stored configuration, restarting");
	}

	/* CONFIG_ERROR included: the S31 keeps ERROR/ERROR_REG of the
	 * rejection until an accepted START, so a corrected configuration
	 * would otherwise be mistaken for another rejection. RESET clears
	 * them (~0.5 s). */
	if (s_status.state != AURA_STATE_WAIT_CONFIG) {
		uint32_t uptime_before = UINT32_MAX;

		(void)read_uptime_locked(&uptime_before);
		s_expect_running = false;
		ret = wr_cmd(AURA_CMD_RESET);
		if (ret < 0) {
			ret = -ENODEV;
			goto out;
		}
		ret = wait_wait_config_locked(uptime_before);
		if (ret < 0) {
			goto out;
		}
	}

	if (!s_cfg.enabled) {
		LOG_INF("broadcast disabled, S31 left in WAIT_CONFIG");
		s_expect_running = false;
		ret = 0;
		goto out;
	}

	ret = write_config_locked();
	if (ret < 0) {
		goto out;
	}

	ret = wr_cmd(AURA_CMD_START);
	if (ret < 0) {
		ret = -ENODEV;
		goto out;
	}
	k_msleep(START_SETTLE_MS);

	/* STARTING is set synchronously by the S31; the Bluetooth stack
	 * answers later with STREAMING or FAULT. Give it a moment so the
	 * caller (web PATCH) learns the real outcome. */
	for (int waited = 0;; waited += RESET_POLL_MS) {
		ret = read_state_locked();
		if (ret < 0) {
			ret = -ENODEV;
			goto out;
		}
		if (s_status.state != AURA_STATE_STARTING || waited >= START_WAIT_MS) {
			break;
		}
		k_msleep(RESET_POLL_MS);
	}
	switch (s_status.state) {
	case AURA_STATE_STARTING:
	case AURA_STATE_STREAMING:
		LOG_INF("broadcast \"%s\" started (%s)", s_cfg.name,
			aura_proto_state_name(s_status.state));
		s_expect_running = true;
		ret = 0;
		break;
	case AURA_STATE_CONFIG_ERROR:
		LOG_ERR("START rejected: %s (%s)", aura_proto_error_name(s_status.error),
			aura_proto_reg_name(s_status.error_reg));
		ret = -EINVAL;
		break;
	case AURA_STATE_FAULT:
		LOG_ERR("S31 in FAULT: %s", aura_proto_error_name(s_status.error));
		ret = -EIO;
		break;
	default:
		LOG_WRN("unexpected state %u after START", s_status.state);
		ret = -EIO;
		break;
	}

out:
	if (ret < 0 && ret != -EINVAL) {
		s_expect_running = false;
	}
	s_status.last_errno = ret;
	(void)read_state_locked();
	publish_locked();
	return ret;
}

/* ---------------------------------------------------------------------
 * Poll
 * ------------------------------------------------------------------- */

static void schedule_poll(uint32_t delay_ms)
{
	k_work_reschedule_for_queue(&s_wq, &s_poll_work, K_MSEC(delay_ms));
}

static void poll_fn(struct k_work *work)
{
	uint32_t delay = POLL_INTERVAL_MS;
	int ret;

	ARG_UNUSED(work);

	k_mutex_lock(&s_lock, K_FOREVER);

	if (!s_ready) {
		k_mutex_unlock(&s_lock);
		return;
	}

	ret = read_state_locked();
	if (ret < 0) {
		if (s_fail_count < PRESENT_FAIL_LIMIT) {
			s_fail_count++;
		}
		if (s_fail_count >= PRESENT_FAIL_LIMIT && s_status.present) {
			LOG_WRN("S31 at 0x%02x stopped answering", AURA_ADDR);
			s_status.present = false;
			s_status.stats_valid = false;
		}
		publish_locked();
		k_mutex_unlock(&s_lock);
		schedule_poll(delay);
		return;
	}

	if (!s_status.present && s_fail_count >= PRESENT_FAIL_LIMIT) {
		LOG_INF("S31 at 0x%02x answering again", AURA_ADDR);
	}
	s_fail_count = 0;
	s_status.present = true;
	s_last_ok_uptime = k_uptime_get();

	if (s_status.stats_valid) {
		(void)read_status_block_locked();
	}

	if (atomic_get(&s_outputs_allowed)) {
		bool run_cycle = false;

		if (!s_activated_once) {
			run_cycle = true;
		} else if (s_expect_running && s_status.state == AURA_STATE_WAIT_CONFIG) {
			LOG_WRN("S31 restarted unexpectedly (reset reason %u), reconfiguring",
				s_status.reset_reason);
			run_cycle = true;
		} else if (s_expect_running && s_status.state == AURA_STATE_FAULT) {
			LOG_ERR("S31 went to FAULT (%s), restarting it",
				aura_proto_error_name(s_status.error));
			s_status.last_errno = -EIO;
			run_cycle = true;
		} else if (s_retry_pending) {
			run_cycle = true;
		}

		if (run_cycle) {
			/* Adopt: a broadcast that is in fact running with our
			 * configuration (e.g. after a transient bus error) must
			 * not be restarted. */
			ret = cycle_locked(true);
			s_activated_once = true;
			/* CONFIG_ERROR needs a corrected configuration, an
			 * autostarting S31 a different build: no retry. */
			s_retry_pending = (ret < 0 && ret != -EINVAL && ret != -ENOTSUP);
			if (s_retry_pending) {
				delay = backoff_ms[s_backoff_idx];
				if (s_backoff_idx < ARRAY_SIZE(backoff_ms) - 1) {
					s_backoff_idx++;
				}
			} else {
				s_backoff_idx = 0;
			}
		}
	}

	publish_locked();
	k_mutex_unlock(&s_lock);
	schedule_poll(delay);
}

/* ---------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------- */

int auracast_card_init(const struct device *i2c_dev)
{
	int ret;

	if (i2c_dev == NULL || !device_is_ready(i2c_dev)) {
		return -ENODEV;
	}

	if (!s_wq_started) {
		k_work_queue_init(&s_wq);
		k_work_queue_start(&s_wq, s_wq_stack, K_THREAD_STACK_SIZEOF(s_wq_stack),
				   WQ_PRIORITY, NULL);
		k_thread_name_set(k_work_queue_thread_get(&s_wq), "auracast");
		s_wq_started = true;
	}

	k_mutex_lock(&s_lock, K_FOREVER);
	ensure_defaults_locked();

	s_i2c = i2c_dev;
	s_ready = false;
	s_activated_once = false;
	s_expect_running = false;
	s_retry_pending = false;
	s_backoff_idx = 0;
	s_fail_count = 0;
	memset(&s_status, 0, sizeof(s_status));

	ret = read_ident_locked();
	if (ret < 0) {
		publish_locked();
		k_mutex_unlock(&s_lock);
		LOG_ERR("init failed: %d", ret);
		return ret;
	}

	s_status.present = true;
	s_last_ok_uptime = k_uptime_get();
	s_ready = true;
	publish_locked();

	LOG_INF("S31 fw %u.%u.%u, state %s, reset reason %u", s_status.fw_major,
		s_status.fw_minor, s_status.fw_patch,
		aura_proto_state_name(s_status.state), s_status.reset_reason);

	k_mutex_unlock(&s_lock);

	schedule_poll(POLL_INTERVAL_MS);
	return 0;
}

bool auracast_card_ready(void)
{
	return s_ready;
}

int auracast_card_activate(void)
{
	if (!s_ready) {
		return -ENODEV;
	}
	/* No s_lock: the fpga_poll thread must not wait behind a cycle. */
	atomic_set(&s_outputs_allowed, 1);
	schedule_poll(0);
	return 0;
}

int auracast_card_get_config(struct auracast_config *out)
{
	k_spinlock_key_t key;

	if (out == NULL) {
		return -EINVAL;
	}
	key = k_spin_lock(&s_pub_lock);
	if (!s_cfg_initialised) {
		/* Nothing published yet (driver never initialised). */
		k_spin_unlock(&s_pub_lock, key);
		aura_proto_config_defaults(out);
		return 0;
	}
	*out = s_pub_cfg;
	k_spin_unlock(&s_pub_lock, key);
	return 0;
}

/*
 * Store @p cfg (NULL = keep) and run the cycle if broadcasting is
 * allowed. With @p adopt an unchanged, running broadcast is kept (store
 * re-apply, "Apply" without changes); without it the S31 is restarted.
 */
static int apply(const struct auracast_config *cfg, bool adopt)
{
	int ret = 0;

	k_mutex_lock(&s_lock, K_FOREVER);
	ensure_defaults_locked();

	if (cfg != NULL) {
		uint8_t bad_reg = 0;

		ret = aura_proto_config_validate(cfg, &bad_reg);
		if (ret < 0) {
			s_status.error = AURA_ERR_BAD_VALUE;
			s_status.error_reg = bad_reg;
			s_status.last_errno = ret;
			LOG_WRN("configuration rejected locally: %s",
				aura_proto_reg_name(bad_reg));
			publish_locked();
			k_mutex_unlock(&s_lock);
			return ret;
		}
		s_cfg = *cfg;
	}

	if (!s_ready) {
		ret = -ENODEV;
	} else if (!atomic_get(&s_outputs_allowed)) {
		LOG_INF("configuration stored, waiting for PTP lock");
	} else {
		ret = cycle_locked(adopt);
		s_activated_once = true;
		s_retry_pending = (ret < 0 && ret != -EINVAL && ret != -ENOTSUP);
		s_backoff_idx = 0;
	}

	publish_locked();
	k_mutex_unlock(&s_lock);
	return ret;
}

int auracast_card_set_config(const struct auracast_config *cfg)
{
	if (cfg == NULL) {
		return -EINVAL;
	}
	return apply(cfg, true);
}

int auracast_card_start(void)
{
	struct auracast_config cfg;

	(void)auracast_card_get_config(&cfg);
	cfg.enabled = true;
	return apply(&cfg, true);
}

int auracast_card_stop(void)
{
	struct auracast_config cfg;

	(void)auracast_card_get_config(&cfg);
	cfg.enabled = false;
	return apply(&cfg, true);
}

int auracast_card_restart(void)
{
	return apply(NULL, false);
}

int auracast_card_get_status(struct auracast_status *out)
{
	k_spinlock_key_t key;
	bool ready;

	if (out == NULL) {
		return -EINVAL;
	}
	key = k_spin_lock(&s_pub_lock);
	*out = s_pub_status;
	ready = s_pub_ready;
	k_spin_unlock(&s_pub_lock, key);
	out->age_ms = out->present
		? (uint32_t)(k_uptime_get() - s_last_ok_uptime) : 0;
	return ready ? 0 : -ENODEV;
}
