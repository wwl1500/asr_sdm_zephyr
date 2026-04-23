/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <asr/imu.h>
#include <asr/robot_base.h>

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(asr_imu, LOG_LEVEL_INF);

#define IMU_SENSOR_NODE DT_NODELABEL(icm42688)
#define ASR_IMU_LOG_INTERVAL_MS 1000

static int64_t last_log_ms;

/* --- cached sample ------------------------------------------------------- */

static struct asr_imu_sample cached_sample;
static bool cached_sample_valid;
static K_MUTEX_DEFINE(cached_sample_mutex);

int asr_imu_get_latest(struct asr_imu_sample *sample)
{
    int ret = -ENODATA;

    k_mutex_lock(&cached_sample_mutex, K_FOREVER);
    if (cached_sample_valid)
    {
        *sample = cached_sample;
        ret = 0;
    }
    k_mutex_unlock(&cached_sample_mutex);
    return ret;
}

/* --- sensor access ------------------------------------------------------- */

#if DT_NODE_EXISTS(IMU_SENSOR_NODE) && DT_NODE_HAS_STATUS(IMU_SENSOR_NODE, okay)

#define IMU_DEV DEVICE_DT_GET(IMU_SENSOR_NODE)

int asr_imu_init(void)
{
    if (!device_is_ready(IMU_DEV))
    {
        LOG_ERR("IMU device not ready");
        return -ENODEV;
    }

    return 1;
}

int asr_imu_read(struct asr_imu_sample *sample)
{
    int ret;

    if (sample == NULL)
    {
        return -EINVAL;
    }

    ret = sensor_sample_fetch(IMU_DEV);
    if (ret < 0)
    {
        return ret;
    }

    ret = sensor_channel_get(IMU_DEV, SENSOR_CHAN_ACCEL_XYZ, sample->accel);
    if (ret < 0)
    {
        return ret;
    }

    ret = sensor_channel_get(IMU_DEV, SENSOR_CHAN_GYRO_XYZ, sample->gyro);
    if (ret < 0)
    {
        return ret;
    }

    return sensor_channel_get(IMU_DEV, SENSOR_CHAN_DIE_TEMP, &sample->temp);
}

int asr_imu_update(void)
{
    struct asr_imu_sample sample;
    int ret;

    ret = asr_imu_read(&sample);
    if (ret < 0)
        return ret;

    k_mutex_lock(&cached_sample_mutex, K_FOREVER);
    cached_sample = sample;
    cached_sample_valid = true;
    k_mutex_unlock(&cached_sample_mutex);

    k_mutex_lock(&unit_status_mutex, K_FOREVER);
    for (int i = 0; i < 3; i++)
    {
        unit_status.imu_float_data.accel[i] = sensor_value_to_float(&sample.accel[i]);
        unit_status.imu_float_data.gyro[i] = sensor_value_to_float(&sample.gyro[i]);
    }
    unit_status.imu_float_data.temperature = sensor_value_to_float(&sample.temp);
    k_mutex_unlock(&unit_status_mutex);

    int64_t now_ms = k_uptime_get();

    if ((now_ms - last_log_ms) >= ASR_IMU_LOG_INTERVAL_MS)
    {
        last_log_ms = now_ms;
        LOG_INF("accel [m/s2]: X=%10.4f  Y=%10.4f  Z=%10.4f", (double)unit_status.imu_float_data.accel[0],
                (double)unit_status.imu_float_data.accel[1], (double)unit_status.imu_float_data.accel[2]);
        LOG_INF("gyro [rad/s]: X=%10.4f  Y=%10.4f  Z=%10.4f", (double)unit_status.imu_float_data.gyro[0],
                (double)unit_status.imu_float_data.gyro[1], (double)unit_status.imu_float_data.gyro[2]);
        LOG_INF("die temp: %.1f C", (double)unit_status.imu_float_data.temperature);
        LOG_INF("------------------------------------------");
    }

    return 0;
}

#else

int asr_imu_init(void)
{
    LOG_ERR("missing enabled icm42688 node");
    return -ENODEV;
}

int asr_imu_read(struct asr_imu_sample *sample)
{
    (void)sample;
    LOG_ERR("missing enabled icm42688 node");
    return -ENODEV;
}

int asr_imu_update(void)
{
    LOG_ERR("missing enabled icm42688 node");
    return -ENODEV;
}

#endif
