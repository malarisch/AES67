/*
 * PTP control/monitoring HAL — software backend.
 *
 * Zephyr IEEE 1588 stack (CONFIG_PTP). Zephyr's public PTP header
 * (zephyr/net/ptp.h) carries no status or configuration API, so this
 * backend reads the stack's internal datasets (subsys/net/lib/ptp is on
 * the include path for this build). Writes go into the live default/port
 * datasets — announce messages and the BTCA consume them on the fly —
 * followed by a state-decision request and an eventfd kick of the PTP
 * thread.
 *
 * Only built with CONFIG_AES67_PTP_SOFTWARE; only *dispatched to* when the
 * gateware reports PTP_IN_SOFTWARE (see ptp_ctrl.c).
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <stdlib.h>

#include "ptp_ctrl_internal.h"
#include "aes67_config.h"

#include "clock.h"
#include "port.h"
#include "ds.h"
#include "state_machine.h"

LOG_MODULE_REGISTER(ptp_ctrl_sw, LOG_LEVEL_INF);

/* An offset this close to the grandmaster counts as "locked" — the same
 * order of magnitude the FPGA hardware servo uses for its lock window. */
#define PTP_CTRL_LOCK_THRESHOLD_NS 1000

static void read_ptp_config(uint8_t *p1, uint8_t *p2, uint8_t *cc, uint8_t *acc,
			    uint8_t *domain, int8_t *log_sync, int8_t *log_ann,
			    int32_t *asym_ns)
{
	aes67_config_lock();
	const struct aes67_device_config *cfg = aes67_config_get();

	*p1  = cfg->ptp_priority1;
	*p2  = cfg->ptp_priority2;
	*cc  = cfg->ptp_clock_class;
	*acc = cfg->ptp_clock_accuracy;
	*domain   = cfg->ptp_domain;
	*log_sync = cfg->ptp_log_sync_interval;
	*log_ann  = cfg->ptp_log_announce_interval;
	*asym_ns  = cfg->ptp_delay_asymmetry_ns;
	aes67_config_unlock();
}

static struct ptp_port *ptp_ctrl_port(void)
{
	sys_slist_t *list = ptp_clock_ports_list();
	sys_snode_t *node = list ? sys_slist_peek_head(list) : NULL;

	return node ? CONTAINER_OF(node, struct ptp_port, node) : NULL;
}

void ptp_ctrl_sw_get_status(struct ptp_ctrl_status *st)
{
	const struct ptp_default_ds *dds = ptp_clock_default_ds();
	const struct ptp_parent_ds  *pds = ptp_clock_parent_ds();
	const struct ptp_current_ds *cds = ptp_clock_current_ds();
	struct ptp_port *port;

	memset(st, 0, sizeof(*st));
	st->mode = "software";

	/* The PTP thread mutates these concurrently; a scheduler lock is
	 * enough for a consistent snapshot on this single-core target. */
	k_sched_lock();

	port = ptp_ctrl_port();
	memcpy(st->clock_id, dds->clk_id.id, 8);

	enum ptp_port_state s = port ? ptp_port_state(port)
				     : PTP_PS_INITIALIZING;

	switch (s) {
	case PTP_PS_TIME_TRANSMITTER:
	case PTP_PS_GRAND_MASTER:
		st->role = PTP_CTRL_ROLE_LEADER;
		break;
	case PTP_PS_TIME_RECEIVER:
	case PTP_PS_UNCALIBRATED:
		st->role = PTP_CTRL_ROLE_FOLLOWER;
		break;
	default:
		st->role = PTP_CTRL_ROLE_LISTENING;
		break;
	}

	/* The parent dataset mirrors our own identity until a foreign
	 * grandmaster wins the BTCA. */
	memcpy(st->gm_id, pds->gm_id.id, 8);
	st->gm_valid = memcmp(st->gm_id, st->clock_id, 8) != 0;
	st->gm_priority1      = pds->gm_priority1;
	st->gm_priority2      = pds->gm_priority2;
	st->gm_clock_class    = pds->gm_clk_quality.cls;
	st->gm_clock_accuracy = pds->gm_clk_quality.accuracy;

	st->steps_removed = cds->steps_rm;
	/* ptp_timeinterval is nanoseconds scaled by 2^16. */
	st->offset_ns     = (int32_t)(cds->offset_from_tt >> 16);
	st->path_delay_ns = (int32_t)(cds->mean_delay >> 16);

	st->locked = st->role == PTP_CTRL_ROLE_FOLLOWER &&
		     cds->mean_delay != 0 &&
		     abs(st->offset_ns) <= PTP_CTRL_LOCK_THRESHOLD_NS;

	k_sched_unlock();
}

bool ptp_ctrl_sw_wallclock_locked(void)
{
	struct ptp_ctrl_status st;

	ptp_ctrl_sw_get_status(&st);

	/* Same rule as the gateware's wallclock_locked signal:
	 * eff_is_leader OR servo locked. */
	return st.role == PTP_CTRL_ROLE_LEADER || st.locked;
}

int ptp_ctrl_sw_get_foreign_masters(struct ptp_ctrl_foreign *out, int max)
{
	struct ptp_foreign_tt_clock *foreign;
	struct ptp_port *port;
	int n = 0;

	k_sched_lock();

	port = ptp_ctrl_port();
	if (!port) {
		k_sched_unlock();
		return 0;
	}

	SYS_SLIST_FOR_EACH_CONTAINER(&port->foreign_list, foreign, node) {
		if (n >= max) {
			break;
		}

		struct ptp_ctrl_foreign *f = &out[n++];

		memcpy(f->sender_id, foreign->port_id.clk_id.id, 8);
		f->sender_port    = foreign->port_id.port_number;
		memcpy(f->gm_id, foreign->dataset.clk_id.id, 8);
		f->priority1      = foreign->dataset.priority1;
		f->priority2      = foreign->dataset.priority2;
		f->clock_class    = foreign->dataset.clk_quality.cls;
		f->clock_accuracy = foreign->dataset.clk_quality.accuracy;
		f->steps_removed  = foreign->dataset.steps_rm;
		f->announce_count = foreign->messages_count;
	}

	k_sched_unlock();
	return n;
}

void ptp_ctrl_sw_apply_config(void)
{
	/* The accessors return const pointers, but the datasets are the
	 * stack's live (mutable) state and announce/BTCA read them on every
	 * cycle — Zephyr just offers no setter API. */
	struct ptp_default_ds *dds =
		(struct ptp_default_ds *)ptp_clock_default_ds();
	struct ptp_port *port;
	uint8_t p1, p2, cc, acc, domain;
	int8_t log_sync, log_ann;
	int32_t asym_ns;

	read_ptp_config(&p1, &p2, &cc, &acc, &domain, &log_sync, &log_ann,
			&asym_ns);

	k_sched_lock();

	dds->priority1 = p1;
	dds->priority2 = p2;
	dds->clk_quality.cls = cc;
	dds->clk_quality.accuracy = acc;
	dds->domain = domain;

	/* Announce content is built from the parent dataset, which the stack
	 * copies from the default dataset only in clock_update_grandmaster()
	 * (static, runs on grandmaster state decisions). Refresh the GM fields
	 * here too so in-flight announces carry the new values immediately —
	 * observed stale announces otherwise while the BMC already compared
	 * with the new defaults. Only touch it while WE are the grandmaster:
	 * as a follower the parent dataset describes the foreign master. */
	struct ptp_parent_ds *pds = (struct ptp_parent_ds *)ptp_clock_parent_ds();

	if (memcmp(pds->gm_id.id, dds->clk_id.id, sizeof(dds->clk_id.id)) == 0) {
		pds->gm_priority1 = p1;
		pds->gm_priority2 = p2;
		pds->gm_clk_quality.cls = cc;
		pds->gm_clk_quality.accuracy = acc;
	}

	SYS_SLIST_FOR_EACH_CONTAINER(ptp_clock_ports_list(), port, node) {
		port->port_ds.log_announce_interval = log_ann;
		port->port_ds.log_sync_interval = log_sync;
		/* IEEE 1588 delayAsymmetry (TimeInterval = ns << 16). The
		 * stack folds it into the sync correction (E2E) and the
		 * pdelay computation (P2P) — compensates PHYs with unequal
		 * RX/TX latencies. */
		port->port_ds.delay_asymmetry =
			(ptp_timeinterval)((int64_t)asym_ns << 16);
		/* Timers pick the new intervals up on their next re-arm. */
	}

	k_sched_unlock();

	/* Our announce content changed — re-run the BTCA and wake the
	 * PTP thread so the outcome applies without waiting for traffic. */
	ptp_clock_state_decision_req();
	ptp_clock_signal_timeout();

	LOG_INF("PTP(sw): applied config - pri1=%u pri2=%u class=%u acc=0x%02x "
		"domain=%u logSync=%d logAnn=%d asym=%dns",
		p1, p2, cc, acc, domain, log_sync, log_ann, asym_ns);
}
