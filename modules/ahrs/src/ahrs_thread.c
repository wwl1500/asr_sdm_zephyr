/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <asr/ahrs.h>
#include <asr/ahrs_thread.h>
#include <asr/imu.h>
#include <asr/robot_base.h>

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(asr_ahrs, LOG_LEVEL_INF);

#define AHRS_THREAD_STACK_SIZE 2048U
#define AHRS_THREAD_PRIO 7
#define AHRS_STARTUP_DELAY_MS CONFIG_ASR_AHRS_STARTUP_DELAY_MS
#define AHRS_THREAD_PERIOD_MS CONFIG_ASR_AHRS_PERIOD_MS
#define ASR_AHRS_LOG_INTERVAL_MS 1000

BUILD_ASSERT(AHRS_THREAD_PERIOD_MS > 0U, "CONFIG_ASR_AHRS_PERIOD_MS must be > 0");

static K_THREAD_STACK_DEFINE(ahrs_thread_stack, AHRS_THREAD_STACK_SIZE);
static struct k_thread ahrs_thread_data;
static bool ahrs_thread_started;

static K_SEM_DEFINE(ahrs_tick_sem, 0, 1);
static struct k_timer ahrs_timer;

static void ahrs_timer_expiry(struct k_timer *timer)
{
    ARG_UNUSED(timer);
    k_sem_give(&ahrs_tick_sem);
}

static void ahrs_copy_sample_to_status(const struct asr_ahrs_sample *sample)
{
    for (int i = 0; i < 4; i++)
    {
        unit_status.ahrs.quaternion[i] = sample->quaternion[i];
    }

    for (int i = 0; i < 3; i++)
    {
        unit_status.ahrs.euler_deg[i] = sample->euler_deg[i];
        unit_status.ahrs.gravity[i] = sample->gravity[i];
        unit_status.ahrs.linear_accel[i] = sample->linear_accel[i];
        unit_status.ahrs.earth_accel[i] = sample->earth_accel[i];
    }

    unit_status.ahrs.ready = true;
    unit_status.ahrs_flags.initialising = sample->initialising;
    unit_status.ahrs_flags.angular_rate_recovery = sample->angular_rate_recovery;
    unit_status.ahrs_flags.acceleration_recovery = sample->acceleration_recovery;
    unit_status.ahrs_flags.magnetic_recovery = sample->magnetic_recovery;
}

static void ahrs_thread_entry(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    struct asr_imu_sample imu_sample;
    struct asr_ahrs_sample ahrs_sample;
    int64_t last_update_ms = 0;
    int64_t last_log_ms = 0;

    if (AHRS_STARTUP_DELAY_MS > 0U)
    {
        k_sleep(K_MSEC(AHRS_STARTUP_DELAY_MS));
    }

    if (asr_ahrs_init() < 0)
    {
        return;
    }

    LOG_INF("AHRS thread ready, period: %u ms", AHRS_THREAD_PERIOD_MS);

    k_timer_init(&ahrs_timer, ahrs_timer_expiry, NULL);
    k_timer_start(&ahrs_timer, K_MSEC(AHRS_THREAD_PERIOD_MS), K_MSEC(AHRS_THREAD_PERIOD_MS));

    for (;;)
    {
        float accel_mps2[3];
        float gyro_rads[3];
        float dt_s;
        int64_t now_ms;

        k_sem_take(&ahrs_tick_sem, K_FOREVER);

        if (asr_imu_get_latest(&imu_sample) < 0)
        {
            continue;
        }

        for (int i = 0; i < 3; i++)
        {
            accel_mps2[i] = sensor_value_to_float(&imu_sample.accel[i]);
            gyro_rads[i] = sensor_value_to_float(&imu_sample.gyro[i]);
        }

        now_ms = k_uptime_get();
        if (last_update_ms == 0)
        {
            dt_s = (float)AHRS_THREAD_PERIOD_MS / 1000.0f;
        }
        else
        {
            dt_s = (float)(now_ms - last_update_ms) / 1000.0f;
        }
        last_update_ms = now_ms;

        if (asr_ahrs_update_from_imu(accel_mps2, gyro_rads, dt_s) < 0)
        {
            LOG_ERR("AHRS update failed");
            continue;
        }

        if (asr_ahrs_get_latest(&ahrs_sample) < 0)
        {
            continue;
        }

        k_mutex_lock(&unit_status_mutex, K_FOREVER);
        ahrs_copy_sample_to_status(&ahrs_sample);
        k_mutex_unlock(&unit_status_mutex);

        if ((now_ms - last_log_ms) >= ASR_AHRS_LOG_INTERVAL_MS)
        {
            last_log_ms = now_ms;
            LOG_INF("euler [deg]: roll=%10.4f  pitch=%10.4f  yaw=%10.4f", (double)ahrs_sample.euler_deg[0],
                    (double)ahrs_sample.euler_deg[1], (double)ahrs_sample.euler_deg[2]);
        }
    }
}

int asr_ahrs_thread_init(void)
{
    k_thread_create(&ahrs_thread_data, ahrs_thread_stack, K_THREAD_STACK_SIZEOF(ahrs_thread_stack), ahrs_thread_entry,
                    NULL, NULL, NULL, AHRS_THREAD_PRIO, 0, K_FOREVER);
    k_thread_name_set(&ahrs_thread_data, "ahrs_thread");
    ahrs_thread_started = true;
    LOG_INF("AHRS thread initialised (suspended), prio %d", k_thread_priority_get(&ahrs_thread_data));
    return 0;
}

int asr_ahrs_thread_start(void)
{
    if (!ahrs_thread_started)
    {
        LOG_ERR("AHRS thread not initialised");
        return -EINVAL;
    }

    LOG_INF("AHRS thread started");
    k_thread_start(&ahrs_thread_data);
    return 0;
}
