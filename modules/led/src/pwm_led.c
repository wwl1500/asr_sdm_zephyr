/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <asr/pwm_led.h>

#include <errno.h>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/led.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(asr_pwm_led, LOG_LEVEL_INF);

#if DT_HAS_COMPAT_STATUS_OKAY(pwm_leds)

#define PWM_LEDS_DEV  DEVICE_DT_GET(DT_COMPAT_GET_ANY_STATUS_OKAY(pwm_leds))
#define PWM_LED_INDEX 0U

#define PWM_LED_THREAD_STACK_SIZE 1024U
#define PWM_LED_THREAD_PRIO       8

K_THREAD_STACK_DEFINE(pwm_led_stack, PWM_LED_THREAD_STACK_SIZE);
static struct k_thread pwm_led_thread_data;

static void pwm_led_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	if (!device_is_ready(PWM_LEDS_DEV)) {
		LOG_ERR("PWM LED (pwm-leds) 未就绪");
		return;
	}

	(void)led_off(PWM_LEDS_DEV, PWM_LED_INDEX);

	for (;;) {
		static bool pwm_bright;
		const int r =
			led_set_brightness(PWM_LEDS_DEV, PWM_LED_INDEX, pwm_bright ? 0U : 60U);

		if (r < 0) {
			LOG_ERR("PWM LED 设置失败: %d", r);
		}
		pwm_bright = !pwm_bright;
		k_sleep(K_MSEC(500));
	}
}

int asr_pwm_led_blink_start(void)
{
	k_thread_create(&pwm_led_thread_data, pwm_led_stack,
			K_THREAD_STACK_SIZEOF(pwm_led_stack), pwm_led_thread, NULL, NULL, NULL,
			PWM_LED_THREAD_PRIO, 0, K_NO_WAIT);
	return 0;
}

#else /* !DT_HAS_COMPAT_STATUS_OKAY(pwm_leds) */

int asr_pwm_led_blink_start(void)
{
	return -ENODEV;
}

#endif
