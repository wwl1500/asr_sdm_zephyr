/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * PWM LED (pwm-leds) — background thread that toggles brightness on the first LED.
 */

#ifndef ASR_PWM_LED_H_
#define ASR_PWM_LED_H_

#include <zephyr/sys/util.h>

#ifdef __cplusplus
extern "C" {
#endif

#if IS_ENABLED(CONFIG_ASR_PWM_LED_THREAD)
/**
 * Start a low-priority thread that toggles brightness on the first pwm-leds LED (index 0).
 *
 * @return 0 on success, negative errno on failure (e.g. device not ready, no pwm_leds in DT).
 */
int asr_pwm_led_blink_start(void);
#else
static inline int asr_pwm_led_blink_start(void)
{
	return 0;
}
#endif

#ifdef __cplusplus
}
#endif

#endif /* ASR_PWM_LED_H_ */
