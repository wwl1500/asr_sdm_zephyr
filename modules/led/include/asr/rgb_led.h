/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * WS2812 RGB LED driver for XIAO RP2350 onboard NeoPixel (GP22, PIO0).
 */

#ifndef ASR_RGB_LED_H_
#define ASR_RGB_LED_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize the RGB LED strip device and turn it off.
 * @return 0 on success, negative errno on failure.
 */
int asr_rgb_led_init(void);

/**
 * Set the RGB LED color and update the strip.
 * @param r Red   0–255.
 * @param g Green 0–255.
 * @param b Blue  0–255.
 * @return 0 on success, negative errno on failure.
 */
int asr_rgb_led_set(uint8_t r, uint8_t g, uint8_t b);

/** Turn the RGB LED off (black). */
int asr_rgb_led_off(void);

#ifdef __cplusplus
}
#endif

#endif /* ASR_RGB_LED_H_ */
