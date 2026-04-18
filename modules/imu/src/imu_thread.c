/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * IMU background thread.
 *
 * Periodically calls asr_imu_update() (in imu.c) which fetches sensor data,
 * caches the sample, and writes float values to unit_status.
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

static void imu_thread_entry(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	for (;;) {
		if (asr_imu_update() < 0) {
			LOG_ERR("IMU read failed");
		}
		k_sleep(K_MSEC(IMU_THREAD_PERIOD_MS));
	}
}

int asr_imu_thread_start(void)
{
	int ret;

	if (imu_thread_started) {
		LOG_WRN("IMU thread already started");
		return -EALREADY;
	}

	if (IMU_STARTUP_DELAY_MS > 0U) {
		k_sleep(K_MSEC(IMU_STARTUP_DELAY_MS));
	}

	ret = asr_imu_init();
	if (ret < 0) {
		return ret;
	}

	LOG_INF("==========================================");
	LOG_INF("ASR IMU thread");
	LOG_INF("==========================================");
	LOG_INF("IMU device ready");
	LOG_INF("thread period: %u ms", IMU_THREAD_PERIOD_MS);
	LOG_INF("");

	k_thread_create(&imu_thread_data, imu_thread_stack,
			K_THREAD_STACK_SIZEOF(imu_thread_stack),
			imu_thread_entry, NULL, NULL, NULL,
			IMU_THREAD_PRIO, 0, K_NO_WAIT);
	k_thread_name_set(&imu_thread_data, "imu_thread");
	imu_thread_started = true;
	return 0;
}
