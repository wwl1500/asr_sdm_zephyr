/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <asr/ahrs.h>

#include "FusionAhrs.h"
#include "FusionAxes.h"

#include <errno.h>
#include <math.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(asr_ahrs, LOG_LEVEL_INF);

#define ASR_AHRS_GRAVITY_MPS2 9.80665f
#define ASR_AHRS_GAIN 0.5f
#define ASR_AHRS_SAMPLE_PERIOD_S ((float)CONFIG_ASR_AHRS_PERIOD_MS / 1000.0f)
#define ASR_AHRS_SAMPLE_RATE_HZ MAX(1U, DIV_ROUND_CLOSEST(1000U, CONFIG_ASR_AHRS_PERIOD_MS))
#define ASR_AHRS_AXES_ALIGNMENT FusionAxesAlignmentPXPYPZ

static fusion_ahrs_t ahrs_state;
static struct asr_ahrs_sample latest_sample;
static bool latest_sample_valid;
static K_MUTEX_DEFINE(ahrs_mutex);

static FusionVector asr_ahrs_make_vector(const float values[3], float scale)
{
    const FusionVector vector = {
        .axis = {
            .x = values[0] * scale,
            .y = values[1] * scale,
            .z = values[2] * scale,
        },
    };

    return FusionAxesSwap(vector, ASR_AHRS_AXES_ALIGNMENT);
}

static void asr_ahrs_store_vector(float out[3], FusionVector vector, float scale)
{
    out[0] = vector.axis.x * scale;
    out[1] = vector.axis.y * scale;
    out[2] = vector.axis.z * scale;
}

static float asr_ahrs_wrap_degrees(float degrees)
{
    while (degrees > 180.0f)
    {
        degrees -= 360.0f;
    }

    while (degrees <= -180.0f)
    {
        degrees += 360.0f;
    }

    return degrees;
}

static void asr_ahrs_fill_latest_sample(void)
{
    const FusionQuaternion quaternion = FusionAhrsGetQuaternion(&ahrs_state);
    const FusionEuler euler = FusionQuaternionToEuler(quaternion);
    const FusionVector gravity = FusionAhrsGetGravity(&ahrs_state);
    const FusionVector linear_accel = FusionAhrsGetLinearAcceleration(&ahrs_state);
    const FusionVector earth_accel = FusionAhrsGetEarthAcceleration(&ahrs_state);
    const FusionAhrsFlags flags = FusionAhrsGetFlags(&ahrs_state);
    const float gravity_yz = sqrtf(gravity.axis.y * gravity.axis.y + gravity.axis.z * gravity.axis.z);
    const float raw_roll = FusionRadiansToDegrees(atan2f(gravity.axis.y, gravity.axis.z));

    latest_sample.quaternion[0] = quaternion.element.w;
    latest_sample.quaternion[1] = quaternion.element.x;
    latest_sample.quaternion[2] = quaternion.element.y;
    latest_sample.quaternion[3] = quaternion.element.z;

    latest_sample.euler_deg[0] = asr_ahrs_wrap_degrees(raw_roll + 180.0f);
    latest_sample.euler_deg[1] = FusionRadiansToDegrees(atan2f(-gravity.axis.x, gravity_yz));
    latest_sample.euler_deg[2] = euler.angle.yaw;

    asr_ahrs_store_vector(latest_sample.gravity, gravity, ASR_AHRS_GRAVITY_MPS2);
    asr_ahrs_store_vector(latest_sample.linear_accel, linear_accel, ASR_AHRS_GRAVITY_MPS2);
    asr_ahrs_store_vector(latest_sample.earth_accel, earth_accel, ASR_AHRS_GRAVITY_MPS2);

    latest_sample.initialising = flags.initialising;
    latest_sample.angular_rate_recovery = flags.angularRateRecovery;
    latest_sample.acceleration_recovery = flags.accelerationRecovery;
    latest_sample.magnetic_recovery = flags.magneticRecovery;
    latest_sample_valid = true;
}

int asr_ahrs_init(void)
{
    const FusionAhrsSettings settings = {
        .convention = FusionConventionNwu,
        .sample_rate = ASR_AHRS_SAMPLE_RATE_HZ,
        .sample_period = ASR_AHRS_SAMPLE_PERIOD_S,
        .gain = ASR_AHRS_GAIN,
        .gyroscopeRange = 0.0f,
        .accelerationRejection = 90.0f,
        .magneticRejection = 90.0f,
        .recoveryTriggerPeriod = 0,
    };

    k_mutex_lock(&ahrs_mutex, K_FOREVER);
    fusion_ahrs_init(&ahrs_state, ASR_AHRS_SAMPLE_RATE_HZ);
    fusionAhrs_set_settings(&ahrs_state, &settings);
    fusion_ahrs_reset(&ahrs_state);
    latest_sample = (struct asr_ahrs_sample){0};
    latest_sample_valid = false;
    k_mutex_unlock(&ahrs_mutex);

    LOG_INF("AHRS initialised, period: %u ms", CONFIG_ASR_AHRS_PERIOD_MS);
    return 0;
}

int asr_ahrs_reset(void)
{
    k_mutex_lock(&ahrs_mutex, K_FOREVER);
    fusion_ahrs_reset(&ahrs_state);
    latest_sample = (struct asr_ahrs_sample){0};
    latest_sample_valid = false;
    k_mutex_unlock(&ahrs_mutex);
    return 0;
}

int asr_ahrs_update_from_imu(const float accel_mps2[3], const float gyro_rads[3], float dt_s)
{
    if ((accel_mps2 == NULL) || (gyro_rads == NULL))
    {
        return -EINVAL;
    }

    if (dt_s <= 0.0f)
    {
        dt_s = ASR_AHRS_SAMPLE_PERIOD_S;
    }

    FusionVector accelerometer = asr_ahrs_make_vector(accel_mps2, 1.0f / ASR_AHRS_GRAVITY_MPS2);
    FusionVector gyroscope = asr_ahrs_make_vector(gyro_rads, 180.0f / ((float)M_PI));

    k_mutex_lock(&ahrs_mutex, K_FOREVER);
    gyroscope = fusion_offset_update(&ahrs_state.offset, gyroscope);
    fusion_ahrs_update_no_magnetometer(&ahrs_state, gyroscope, accelerometer, dt_s);
    asr_ahrs_fill_latest_sample();
    k_mutex_unlock(&ahrs_mutex);

    return 0;
}

int asr_ahrs_get_latest(struct asr_ahrs_sample *sample)
{
    int ret = -ENODATA;

    if (sample == NULL)
    {
        return -EINVAL;
    }

    k_mutex_lock(&ahrs_mutex, K_FOREVER);
    if (latest_sample_valid)
    {
        *sample = latest_sample;
        ret = 0;
    }
    k_mutex_unlock(&ahrs_mutex);

    return ret;
}

bool asr_ahrs_is_ready(void)
{
    bool ready;

    k_mutex_lock(&ahrs_mutex, K_FOREVER);
    ready = latest_sample_valid;
    k_mutex_unlock(&ahrs_mutex);

    return ready;
}
