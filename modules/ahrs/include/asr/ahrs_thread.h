/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Background thread that periodically fuses IMU samples into AHRS outputs.
 */

#ifndef ASR_AHRS_THREAD_H_
#define ASR_AHRS_THREAD_H_

#ifdef __cplusplus
extern "C" {
#endif

int asr_ahrs_thread_init(void);
int asr_ahrs_thread_start(void);

#ifdef __cplusplus
}
#endif

#endif /* ASR_AHRS_THREAD_H_ */
