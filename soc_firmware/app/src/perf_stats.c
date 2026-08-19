/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * `perf cpu` — per-thread CPU share since the previous invocation.
 *
 * Phase-0 measurement for the multicore evaluation: run the load scenario
 * (scripts/webload.sh + PTP follower + SAP/NMOS), call `perf cpu` once to
 * open the window, wait, call it again and read the per-thread shares.
 * Together with `spibus` this answers whether the ESP32 is CPU-bound
 * (userland threads dominate) or SPI-bus-bound (idle high, bus held high).
 *
 * Requires CONFIG_THREAD_RUNTIME_STATS (+ THREAD_MONITOR/THREAD_NAME,
 * which the shell already pulls in).
 */

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <string.h>

#define PERF_MAX_THREADS 48

#ifdef CONFIG_THREAD_MAX_NAME_LEN
#define PERF_NAME_LEN CONFIG_THREAD_MAX_NAME_LEN
#else
#define PERF_NAME_LEN 12	/* "%p" fallback */
#endif

struct perf_slot {
	struct k_thread *thread;
	char name[PERF_NAME_LEN];
	uint64_t cycles;	/* snapshot at the previous call */
	uint64_t delta;
	int prio;
	bool seen;
};

static struct perf_slot slots[PERF_MAX_THREADS];
static int64_t prev_ms;
static bool have_prev;
static bool table_full;

static struct perf_slot *slot_for(struct k_thread *t)
{
	struct perf_slot *free_slot = NULL;

	for (int i = 0; i < PERF_MAX_THREADS; i++) {
		if (slots[i].thread == t) {
			return &slots[i];
		}
		if (slots[i].thread == NULL && free_slot == NULL) {
			free_slot = &slots[i];
		}
	}
	if (free_slot != NULL) {
		free_slot->thread = t;
		free_slot->cycles = 0;
	}
	return free_slot;
}

static void perf_visit(const struct k_thread *cthread, void *user_data)
{
	struct k_thread *t = (struct k_thread *)cthread;
	k_thread_runtime_stats_t rt;
	struct perf_slot *s;
	const char *name;

	ARG_UNUSED(user_data);

	if (k_thread_runtime_stats_get(t, &rt) != 0) {
		return;
	}

	s = slot_for(t);
	if (s == NULL) {
		table_full = true;
		return;
	}

	/* A slot can be recycled by a new thread at the same address (or the
	 * stats backend reset); a shrinking counter means the old snapshot is
	 * meaningless — restart this thread's window at zero. */
	if (rt.execution_cycles < s->cycles) {
		s->cycles = 0;
	}

	s->delta = rt.execution_cycles - s->cycles;
	s->cycles = rt.execution_cycles;
	s->prio = t->base.prio;
	s->seen = true;

	name = k_thread_name_get(t);
	if (name != NULL && name[0] != '\0') {
		strncpy(s->name, name, sizeof(s->name) - 1);
		s->name[sizeof(s->name) - 1] = '\0';
	} else {
		snprintk(s->name, sizeof(s->name), "%p", (void *)t);
	}
}

static int cmd_perf_cpu(const struct shell *sh, size_t argc, char **argv)
{
	int64_t now_ms = k_uptime_get();
	int64_t elapsed_ms;
	uint32_t cps = sys_clock_hw_cycles_per_sec();
	uint64_t wall_cyc, sum = 0;

	for (int i = 0; i < PERF_MAX_THREADS; i++) {
		slots[i].seen = false;
	}
	table_full = false;

	k_thread_foreach(perf_visit, NULL);

	/* Threads that exited since the last call: free their slots. */
	for (int i = 0; i < PERF_MAX_THREADS; i++) {
		if (slots[i].thread != NULL && !slots[i].seen) {
			slots[i].thread = NULL;
		}
	}

	elapsed_ms = now_ms - prev_ms;
	prev_ms = now_ms;

	if (!have_prev) {
		have_prev = true;
		shell_print(sh, "(window = since boot; call again for a load window)");
	}
	if (elapsed_ms <= 0) {
		return 0;
	}
	wall_cyc = (uint64_t)elapsed_ms * cps / 1000U;

	shell_print(sh, "window %u.%02u s",
		    (uint32_t)(elapsed_ms / 1000),
		    (uint32_t)((elapsed_ms % 1000) / 10));
	shell_print(sh, "   %%  prio  thread");

	/* Selection print: highest remaining delta each round — the table is
	 * small, O(n^2) is fine and needs no extra storage. */
	for (int printed = 0; printed < PERF_MAX_THREADS; printed++) {
		struct perf_slot *best = NULL;

		for (int i = 0; i < PERF_MAX_THREADS; i++) {
			if (slots[i].thread != NULL && slots[i].seen &&
			    (best == NULL || slots[i].delta > best->delta)) {
				best = &slots[i];
			}
		}
		if (best == NULL) {
			break;
		}

		uint32_t pm = (uint32_t)(best->delta * 1000U / wall_cyc);

		/* Skip the long idle tail of never-scheduled threads. */
		if (best->delta > 0 || pm > 0) {
			shell_print(sh, "%3u.%u  %4d  %s",
				    pm / 10, pm % 10, best->prio, best->name);
		}
		sum += best->delta;
		best->seen = false;	/* consumed */
	}

	uint32_t sum_pm = (uint32_t)(sum * 1000U / wall_cyc);

	shell_print(sh, "sum %u.%u %% (remainder: ISRs + exited threads)",
		    sum_pm / 10, sum_pm % 10);
	if (table_full) {
		shell_warn(sh, "thread table full (>%d) — results incomplete",
			   PERF_MAX_THREADS);
	}
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(perf_cmds,
	SHELL_CMD(cpu, NULL,
		  "Per-thread CPU share since the previous call",
		  cmd_perf_cpu),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(perf, &perf_cmds, "CPU load measurement", NULL);
