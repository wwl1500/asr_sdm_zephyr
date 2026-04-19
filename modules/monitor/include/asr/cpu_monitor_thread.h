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
 * @brief Create the CPU monitor thread in suspended state.
 *
 * @return 0 on success, -EALREADY if already initialised.
 */
int asr_cpu_monitor_thread_init(void);

/**
 * @brief Start the CPU monitor thread (k_thread_start; must call init first).
 *
 * @return 0 on success, -EINVAL if not initialised.
 */
int asr_cpu_monitor_thread_start(void);

#ifdef __cplusplus
}
#endif

#endif /* ASR_CPU_MONITOR_THREAD_H_ */
