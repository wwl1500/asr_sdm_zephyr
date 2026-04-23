/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * AHRS helper API built on top of the Fusion algorithm library.
 */

#ifndef ASR_AHRS_H_
#define ASR_AHRS_H_

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

struct asr_ahrs_sample {
    float quaternion[4];
    float euler_deg[3];
    float gravity[3];
    float linear_accel[3];
    float earth_accel[3];
    bool initialising;
    bool angular_rate_recovery;
    bool acceleration_recovery;
    bool magnetic_recovery;
};

int asr_ahrs_init(void);
int asr_ahrs_reset(void);
int asr_ahrs_update_from_imu(const float accel_mps2[3], const float gyro_rads[3], float dt_s);
int asr_ahrs_get_latest(struct asr_ahrs_sample *sample);
bool asr_ahrs_is_ready(void);

#ifdef __cplusplus
}
#endif

#endif /* ASR_AHRS_H_ */
