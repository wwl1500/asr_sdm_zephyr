/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * CPU monitor — iterates over all threads, computes per-thread CPU usage
 * since the last sample, and logs the results.
 *
 * Requires CONFIG_THREAD_RUNTIME_STATS (auto-selects CONFIG_SCHED_THREAD_USAGE).
 */

#include <asr/cpu_monitor.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(asr_cpu_monitor, LOG_LEVEL_INF);

static void log_thread_stats(const struct k_thread *thread, void *user_data)
{
	k_thread_runtime_stats_t *all_stats = user_data;
	k_thread_runtime_stats_t thread_stats;
	const char *name;

	if (k_thread_runtime_stats_get((k_tid_t)thread, &thread_stats) != 0) {
		return;
	}

	name = k_thread_name_get((k_tid_t)thread);
	if (name == NULL || name[0] == '\0') {
		name = "?";
	}

	unsigned int pct = 0;

	if (all_stats->execution_cycles > 0) {
		pct = (unsigned int)((thread_stats.execution_cycles * 100U) /
				     all_stats->execution_cycles);
	}

	LOG_INF("  %-20s  CPU: %3u%%", name, pct);
}

void asr_cpu_monitor_update(void)
{
	k_thread_runtime_stats_t all_stats;

	if (k_thread_runtime_stats_all_get(&all_stats) != 0) {
		LOG_ERR("failed to get aggregate runtime stats");
		return;
	}

	LOG_INF("--- CPU usage snapshot ---");
	k_thread_foreach(log_thread_stats, &all_stats);
	LOG_INF("--------------------------");
}
