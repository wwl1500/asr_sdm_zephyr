/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * IMU background thread.
 *
 * A periodic timer gives a semaphore every IMU_THREAD_PERIOD_MS.  The thread
 * blocks on the semaphore and calls asr_imu_update() (in imu.c) which fetches
 * sensor data, caches the sample, and writes float values to unit_status.
 */

#include <asr/imu.h>
#include <asr/imu_thread.h>

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(asr_imu, LOG_LEVEL_INF);

#define IMU_THREAD_STACK_SIZE 2048U
#define IMU_THREAD_PRIO 7
#define IMU_STARTUP_DELAY_MS CONFIG_ASR_IMU_STARTUP_DELAY_MS
#define IMU_THREAD_PERIOD_MS CONFIG_ASR_IMU_PERIOD_MS

BUILD_ASSERT(IMU_THREAD_PERIOD_MS > 0U, "CONFIG_ASR_IMU_PERIOD_MS must be > 0");

static K_THREAD_STACK_DEFINE(imu_thread_stack, IMU_THREAD_STACK_SIZE);
static struct k_thread imu_thread_data;
static bool imu_thread_started;

static K_SEM_DEFINE(imu_tick_sem, 0, 1);
static struct k_timer imu_timer;

static void imu_timer_expiry(struct k_timer *timer)
{
    ARG_UNUSED(timer);
    k_sem_give(&imu_tick_sem);
}

static void imu_thread_entry(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    if (IMU_STARTUP_DELAY_MS > 0U)
    {
        k_sleep(K_MSEC(IMU_STARTUP_DELAY_MS));
    }

    int ret = asr_imu_init();
    if (ret < 0)
        return;

    LOG_INF("IMU device ready, period: %u ms", IMU_THREAD_PERIOD_MS);

    k_timer_init(&imu_timer, imu_timer_expiry, NULL);
    k_timer_start(&imu_timer, K_MSEC(IMU_THREAD_PERIOD_MS), K_MSEC(IMU_THREAD_PERIOD_MS));

    for (;;)
    {
        k_sem_take(&imu_tick_sem, K_FOREVER);

        if (asr_imu_update() < 0)
        {
            LOG_ERR("IMU read failed");
        }
    }
}

int asr_imu_thread_init(void)
{
    // if (imu_thread_started)
    // {
    //     LOG_WRN("IMU thread already initialised");
    //     return -EALREADY;
    // }

    k_thread_create(&imu_thread_data, imu_thread_stack, K_THREAD_STACK_SIZEOF(imu_thread_stack), imu_thread_entry, NULL,
                    NULL, NULL, IMU_THREAD_PRIO, 0, K_FOREVER);
    k_thread_name_set(&imu_thread_data, "imu_thread");
    imu_thread_started = true;
    LOG_INF("IMU thread initialised (suspended), prio %d", k_thread_priority_get(&imu_thread_data));
    return 0;
}

int asr_imu_thread_start(void)
{
    if (!imu_thread_started)
    {
        LOG_ERR("IMU thread not initialised");
        return -EINVAL;
    }
    LOG_INF("IMU thread started");
    k_thread_start(&imu_thread_data);
    return 0;
}
