/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <asr/imu.h>
#include <asr/imu_thread.h>

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(asr_imu_thread, LOG_LEVEL_INF);

#define IMU_UART_NODE           DT_NODELABEL(cdc_acm_uart0)
#define IMU_THREAD_STACK_SIZE   2048U
#define IMU_THREAD_PRIO         7
#define IMU_STARTUP_DELAY_MS    CONFIG_ASR_IMU_STARTUP_DELAY_MS
#define IMU_THREAD_PERIOD_MS    CONFIG_ASR_IMU_PERIOD_MS
#define IMU_USB_POLL_PERIOD_MS  CONFIG_ASR_IMU_DISCONNECTED_POLL_PERIOD_MS

BUILD_ASSERT(IMU_THREAD_PERIOD_MS > 0U, "CONFIG_ASR_IMU_PERIOD_MS must be > 0");
BUILD_ASSERT(IMU_USB_POLL_PERIOD_MS > 0U,
	     "CONFIG_ASR_IMU_DISCONNECTED_POLL_PERIOD_MS must be > 0");

#if DT_NODE_EXISTS(IMU_UART_NODE) && DT_NODE_HAS_STATUS(IMU_UART_NODE, okay)

#define USB_DEV DEVICE_DT_GET(IMU_UART_NODE)

K_THREAD_STACK_DEFINE(imu_thread_stack, IMU_THREAD_STACK_SIZE);
static struct k_thread imu_thread_data;
static bool imu_thread_started;

static bool usb_cdc_host_connected(const struct device *cdc_acm)
{
	uint32_t dtr = 0U;

	if (!device_is_ready(cdc_acm)) {
		return false;
	}

	if (uart_line_ctrl_get(cdc_acm, UART_LINE_CTRL_DTR, &dtr) < 0) {
		return false;
	}

	return dtr != 0U;
}

static void imu_thread_entry(void *p1, void *p2, void *p3)
{
	struct asr_imu_sample sample;
	bool host_was_connected = false;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	for (;;) {
		const bool host_connected = usb_cdc_host_connected(USB_DEV);
		int ret;

		if (host_connected != host_was_connected) {
			host_was_connected = host_connected;
			if (host_connected) {
				LOG_INF("USB 串口已连接。");
			} else {
				LOG_INF("USB 串口未连接 (关闭串口或断开)；等待重新连接...");
			}
		}

		if (!host_connected) {
			k_sleep(K_MSEC(IMU_USB_POLL_PERIOD_MS));
			continue;
		}

		ret = asr_imu_read(&sample);
		if (ret < 0) {
			LOG_ERR("读取 IMU 数据失败: %d", ret);
			k_sleep(K_MSEC(IMU_THREAD_PERIOD_MS));
			continue;
		}

		LOG_INF("加速度 [m/s²]: X=%10.4f  Y=%10.4f  Z=%10.4f",
			sensor_value_to_double(&sample.accel[0]),
			sensor_value_to_double(&sample.accel[1]),
			sensor_value_to_double(&sample.accel[2]));
		LOG_INF("角速度 [rad/s]: X=%10.4f  Y=%10.4f  Z=%10.4f",
			sensor_value_to_double(&sample.gyro[0]),
			sensor_value_to_double(&sample.gyro[1]),
			sensor_value_to_double(&sample.gyro[2]));
		LOG_INF("芯片温度: %.1f °C", sensor_value_to_double(&sample.temp));
		LOG_INF("------------------------------------------");

		k_sleep(K_MSEC(IMU_THREAD_PERIOD_MS));
	}
}

int asr_imu_thread_start(void)
{
	int ret;

	if (imu_thread_started) {
		LOG_WRN("IMU 后台线程已启动");
		return -EALREADY;
	}

	if (IMU_STARTUP_DELAY_MS > 0U) {
		k_sleep(K_MSEC(IMU_STARTUP_DELAY_MS));
	}

	ret = asr_imu_init();
	if (ret < 0) {
		return ret;
	}

	if (!device_is_ready(USB_DEV)) {
		LOG_ERR("IMU UART 设备未就绪");
		return -ENODEV;
	}

	LOG_INF("==========================================");
	LOG_INF("ASR IMU 后台线程");
	LOG_INF("==========================================");
	LOG_INF("IMU 设备已就绪");
	LOG_INF("线程周期: %u ms", IMU_THREAD_PERIOD_MS);
	LOG_INF("设备已就绪，等待 USB 串口连接后开始采集...");
	LOG_INF("");

	k_thread_create(&imu_thread_data, imu_thread_stack,
			K_THREAD_STACK_SIZEOF(imu_thread_stack), imu_thread_entry,
			NULL, NULL, NULL, IMU_THREAD_PRIO, 0, K_NO_WAIT);
	k_thread_name_set(&imu_thread_data, "imu_thread");
	imu_thread_started = true;
	return 0;
}

#else

int asr_imu_thread_start(void)
{
	LOG_ERR("缺少启用的 cdc_acm_uart0 节点");
	return -ENODEV;
}

#endif
