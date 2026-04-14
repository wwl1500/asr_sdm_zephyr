/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * User LED driver for XIAO RP2350 onboard LED (GP25).
 * Supports GPIO on/off/toggle and optional PWM brightness control.
 */

#ifndef ASR_USR_LED_H_
#define ASR_USR_LED_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize the user LED hardware.
 * @return 0 on success, negative errno on failure.
 */
int asr_usr_led_init(void);

void asr_usr_led_on(void);
void asr_usr_led_off(void);
void asr_usr_led_toggle(void);

/**
 * Set user LED brightness (PWM mode only; pwm-leds must be enabled in DT).
 * @param brightness 0–100 percent.
 * @return 0 on success, -ENOTSUP if PWM unavailable, negative errno on failure.
 */
int asr_usr_led_set_brightness(uint8_t brightness);

#ifdef __cplusplus
}
#endif

#endif /* ASR_USR_LED_H_ */
