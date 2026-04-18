/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * CPU monitor — collects and logs per-thread CPU usage.
 */

#ifndef ASR_CPU_MONITOR_H_
#define ASR_CPU_MONITOR_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Sample thread runtime stats and log CPU usage.
 *
 * Iterates over all threads, computes the CPU percentage each consumed
 * since the last call, and logs the results.
 */
void asr_cpu_monitor_update(void);

#ifdef __cplusplus
}
#endif

#endif /* ASR_CPU_MONITOR_H_ */
