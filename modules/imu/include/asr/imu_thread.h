/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Background thread that periodically reads and caches IMU samples.
 */

#ifndef ASR_IMU_THREAD_H_
#define ASR_IMU_THREAD_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Start the IMU background thread.
 *
 * @return 0 on success, -EALREADY when already started, or another negative
 *         errno on failure.
 */
int asr_imu_thread_start(void);

#ifdef __cplusplus
}
#endif

#endif /* ASR_IMU_THREAD_H_ */
