/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * IMU driver for the project's ICM-42688 sensor node.
 */

#ifndef ASR_IMU_H_
#define ASR_IMU_H_

#include <zephyr/drivers/sensor.h>

#ifdef __cplusplus
extern "C" {
#endif

struct asr_imu_sample {
	struct sensor_value accel[3];
	struct sensor_value gyro[3];
	struct sensor_value temp;
};

/**
 * Initialize the IMU device selected by the module's devicetree node.
 * @return 0 on success, negative errno on failure.
 */
int asr_imu_init(void);

/**
 * Fetch and read a single IMU sample.
 * @param sample Output sample storage; must not be NULL.
 * @return 0 on success, negative errno on failure.
 */
int asr_imu_read(struct asr_imu_sample *sample);

#ifdef __cplusplus
}
#endif

#endif /* ASR_IMU_H_ */
