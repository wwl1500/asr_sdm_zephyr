/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <asr/led_thread.h>
#include <asr/rgb_led.h>
#include <asr/usr_led.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(asr_led_thread, LOG_LEVEL_INF);

#define LED_THREAD_STACK_SIZE 1024U
#define LED_THREAD_PRIO       8
#define RGB_BLINK_MS          500U
#define RGB_BLUE_BRIGHTNESS   32U

K_THREAD_STACK_DEFINE(led_thread_stack, LED_THREAD_STACK_SIZE);
static struct k_thread led_thread_data;

static void led_thread_entry(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	int ret = asr_usr_led_init();

	if (ret < 0) {
		LOG_ERR("User LED init failed: %d", ret);
	} else {
		asr_usr_led_on();
	}

	ret = asr_rgb_led_init();
	if (ret < 0) {
		LOG_ERR("RGB LED init failed: %d", ret);
	}

	bool rgb_on = false;

	for (;;) {
		rgb_on = !rgb_on;
		(void)asr_rgb_led_set(0, 0, rgb_on ? RGB_BLUE_BRIGHTNESS : 0);
		k_sleep(K_MSEC(RGB_BLINK_MS));
	}
}

int asr_led_thread_start(void)
{
	k_thread_create(&led_thread_data, led_thread_stack,
			K_THREAD_STACK_SIZEOF(led_thread_stack), led_thread_entry,
			NULL, NULL, NULL, LED_THREAD_PRIO, 0, K_NO_WAIT);
	k_thread_name_set(&led_thread_data, "led_thread");
	return 0;
}
