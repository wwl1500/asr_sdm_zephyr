/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <asr/led_thread.h>
#include <asr/rgb_led.h>
#include <asr/usr_led.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/printk.h>

LOG_MODULE_REGISTER(asr_led_thread, LOG_LEVEL_INF);

#define LED_THREAD_STACK_SIZE 1024U
#define LED_THREAD_PRIO 8
#define RGB_BLINK_MS CONFIG_ASR_LED_THREAD_PERIOD_MS
#define RGB_BLUE_BRIGHTNESS 32U

K_THREAD_STACK_DEFINE(led_thread_stack, LED_THREAD_STACK_SIZE);
static struct k_thread led_thread_data;

static atomic_t rgb_blink_enabled = ATOMIC_INIT(1);

static K_SEM_DEFINE(led_tick_sem, 0, 1);
static struct k_timer led_timer;

static void led_timer_expiry(struct k_timer *timer)
{
    ARG_UNUSED(timer);
    k_sem_give(&led_tick_sem);
}

void asr_led_thread_set_rgb_blink(bool enable) { atomic_set(&rgb_blink_enabled, enable ? 1 : 0); }

static void led_thread_entry(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    int ret = asr_usr_led_init();

    if (ret < 0)
    {
        LOG_ERR("User LED init failed: %d", ret);
    }
    else
    {
        asr_usr_led_on();
    }

    ret = asr_rgb_led_init();
    if (ret < 0)
    {
        LOG_ERR("RGB LED init failed: %d", ret);
    }

    k_timer_init(&led_timer, led_timer_expiry, NULL);
    k_timer_start(&led_timer, K_MSEC(RGB_BLINK_MS), K_MSEC(RGB_BLINK_MS));

    bool rgb_on = false;

    for (;;)
    {
        k_sem_take(&led_tick_sem, K_FOREVER);

        if (atomic_get(&rgb_blink_enabled))
        {
            rgb_on = !rgb_on;
            (void)asr_rgb_led_set(0, 0, rgb_on ? RGB_BLUE_BRIGHTNESS : 0);
        }
        else if (rgb_on)
        {
            rgb_on = false;
            (void)asr_rgb_led_off();
        }
    }
}

int asr_led_thread_init(void)
{
    k_thread_create(&led_thread_data, led_thread_stack, K_THREAD_STACK_SIZEOF(led_thread_stack), led_thread_entry, NULL,
                    NULL, NULL, LED_THREAD_PRIO, 0, K_FOREVER);
    k_thread_name_set(&led_thread_data, "led_thread");
    LOG_INF("LED thread initialised (suspended), prio %d", k_thread_priority_get(&led_thread_data));
    return 0;
}

int asr_led_thread_start(void)
{
    LOG_INF("LED thread started");
    k_thread_start(&led_thread_data);
    return 0;
}
