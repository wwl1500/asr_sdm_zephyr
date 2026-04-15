/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <asr/imu.h>

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(asr_imu, LOG_LEVEL_INF);

#define IMU_SENSOR_NODE DT_NODELABEL(icm42688)

#if DT_NODE_EXISTS(IMU_SENSOR_NODE) && DT_NODE_HAS_STATUS(IMU_SENSOR_NODE, okay)

#define IMU_DEV DEVICE_DT_GET(IMU_SENSOR_NODE)

int asr_imu_init(void)
{
	if (!device_is_ready(IMU_DEV)) {
		LOG_ERR("IMU 设备未就绪");
		return -ENODEV;
	}

	return 0;
}

int asr_imu_read(struct asr_imu_sample *sample)
{
	int ret;

	if (sample == NULL) {
		return -EINVAL;
	}

	ret = asr_imu_init();
	if (ret < 0) {
		return ret;
	}

	ret = sensor_sample_fetch(IMU_DEV);
	if (ret < 0) {
		return ret;
	}

	ret = sensor_channel_get(IMU_DEV, SENSOR_CHAN_ACCEL_XYZ, sample->accel);
	if (ret < 0) {
		return ret;
	}

	ret = sensor_channel_get(IMU_DEV, SENSOR_CHAN_GYRO_XYZ, sample->gyro);
	if (ret < 0) {
		return ret;
	}

	return sensor_channel_get(IMU_DEV, SENSOR_CHAN_DIE_TEMP, &sample->temp);
}

#else

int asr_imu_init(void)
{
	LOG_ERR("缺少启用的 icm42688 节点");
	return -ENODEV;
}

int asr_imu_read(struct asr_imu_sample *sample)
{
	(void)sample;
	LOG_ERR("缺少启用的 icm42688 节点");
	return -ENODEV;
}

#endif
