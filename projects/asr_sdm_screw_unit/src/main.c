/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * The application delegates LED, IMU, and UART communication activity to the
 * ASR helper modules.
 */

#include <asr/ahrs_thread.h>
#include <asr/comm_thread.h>
#include <asr/cpu_monitor_thread.h>
#include <asr/imu.h>
#include <asr/imu_thread.h>
#include <asr/led_thread.h>
#include <asr/robot_base.h>
#include <asr/usb_thread.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(unit_system, LOG_LEVEL_INF);

unit_status_t unit_status;
K_MUTEX_DEFINE(unit_status_mutex);

static void handle_led_write(bool state) { asr_led_thread_set_rgb_blink(state); }

static int handle_imu_read(uint8_t buf[ASR_COMM_MSG_SIZE])
{
    struct asr_imu_sample sample;
    int ret = asr_imu_get_latest(&sample);

    if (ret < 0)
    {
        return ret;
    }

    int16_t ax = (int16_t)sensor_value_to_milli(&sample.accel[0]);
    int16_t ay = (int16_t)sensor_value_to_milli(&sample.accel[1]);
    int16_t az = (int16_t)sensor_value_to_milli(&sample.accel[2]);

    buf[0] = ASR_COMM_CMD_READ;
    buf[1] = ASR_COMM_PARAM_IMU;
    buf[2] = (uint8_t)(ax >> 8);
    buf[3] = (uint8_t)(ax & 0xFF);
    buf[4] = (uint8_t)(ay >> 8);
    buf[5] = (uint8_t)(ay & 0xFF);
    buf[6] = (uint8_t)(az >> 8);
    buf[7] = (uint8_t)(az & 0xFF);

    return 0;
}

static const struct asr_comm_callbacks comm_callbacks = {
    .on_led_write = handle_led_write,
    .on_imu_read = handle_imu_read,
};

int main(void)
{
    int ret;

    LOG_INF("==========================================");
    LOG_INF("Unit system application");
    LOG_INF("==========================================");

    /* ---- Phase 1: initialise all modules (threads created suspended) ---- */

    ret = asr_led_thread_init();
    if (ret < 0)
    {
        LOG_WRN("LED thread init failed: %d (continuing without LED)", ret);
    }

    ret = asr_usb_protocol_thread_init();
    if (ret < 0)
    {
        LOG_WRN("USB protocol thread init failed: %d (continuing without USB)", ret);
    }

    ret = asr_cpu_monitor_thread_init();
    if (ret < 0)
    {
        LOG_WRN("CPU monitor thread init failed: %d (continuing without CPU monitor)", ret);
    }

    asr_comm_register_callbacks(&comm_callbacks);

    ret = asr_comm_thread_init();
    if (ret < 0)
    {
        LOG_WRN("comm thread init failed: %d (continuing without comm)", ret);
    }

    ret = asr_imu_thread_init();
    if (ret < 0)
    {
        LOG_WRN("IMU thread init failed: %d (continuing without IMU)", ret);
    }

    ret = asr_ahrs_thread_init();
    if (ret < 0)
    {
        LOG_WRN("AHRS thread init failed: %d (continuing without AHRS)", ret);
    }

    /* ---- Phase 2: resume all threads together ---- */
    LOG_INF("all modules initialised, starting threads");
    k_sleep(K_MSEC(10));

    asr_led_thread_start();
    asr_usb_protocol_thread_start();
    asr_cpu_monitor_thread_start();
    asr_comm_thread_start();
    asr_imu_thread_start();
    asr_ahrs_thread_start();

    LOG_INF("all background threads started");
    return 0;
}
