/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * CPU monitor background thread.
 */

#ifndef ASR_CPU_MONITOR_THREAD_H_
#define ASR_CPU_MONITOR_THREAD_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Start the CPU monitor background thread.
 *
 * @return 0 on success, negative errno on failure.
 */
int asr_cpu_monitor_thread_start(void);

#ifdef __cplusplus
}
#endif

#endif /* ASR_CPU_MONITOR_THREAD_H_ */
