/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Background thread that logs IMU samples after the USB CDC ACM host connects.
 */

#ifndef ASR_IMU_THREAD_H_
#define ASR_IMU_THREAD_H_

#include <asr/imu.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Start the IMU logging thread.
 *
 * @return 0 on success, -EALREADY when already started, or another negative
 *         errno on failure.
 */
int asr_imu_thread_start(void);

/**
 * Get the latest IMU sample cached by the background thread.
 *
 * @param sample  Output storage; must not be NULL.
 * @return 0 on success, -ENODATA if no sample has been captured yet.
 */
int asr_imu_thread_get_latest(struct asr_imu_sample *sample);

#ifdef __cplusplus
}
#endif

#endif /* ASR_IMU_THREAD_H_ */
