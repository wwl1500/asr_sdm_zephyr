/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * ICM-42688 sensor demo application for Seeed XIAO RP2350 development board.
 * The application delegates LED, IMU, and UART communication activity to the
 * ASR helper modules.
 */

#include <asr/comm_thread.h>
#include <asr/imu_thread.h>
#include <asr/led_thread.h>
#include <asr/robot_base.h>

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(icm42688_demo, LOG_LEVEL_INF);

unit_status_t unit_status;
K_MUTEX_DEFINE(unit_status_mutex);

static void handle_led_write(bool state)
{
	asr_led_thread_set_rgb_blink(state);
}

static const struct asr_comm_callbacks comm_callbacks = {
	.on_led_write = handle_led_write,
};

int main(void)
{
	int ret;

	LOG_INF("==========================================");
	LOG_INF("ICM-42688 传感器示例应用");
	LOG_INF("==========================================");

	ret = asr_led_thread_start();
	if (ret < 0) {
		LOG_ERR("LED 后台线程启动失败: %d", ret);
		return ret;
	}

	ret = asr_imu_thread_start();
	if (ret < 0) {
		LOG_ERR("IMU 后台线程启动失败: %d", ret);
		return ret;
	}

	asr_comm_register_callbacks(&comm_callbacks);

	ret = asr_comm_thread_start();
	if (ret < 0) {
		LOG_ERR("通信线程启动失败: %d", ret);
		return ret;
	}

	LOG_INF("后台线程已启动");
	return 0;
}
