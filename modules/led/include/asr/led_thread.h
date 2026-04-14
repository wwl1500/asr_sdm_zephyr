/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Background thread that drives the user LED and RGB LED.
 */

#ifndef ASR_LED_THREAD_H_
#define ASR_LED_THREAD_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Start the LED control thread.
 * - User LED: held on (steady).
 * - RGB LED:  blinks blue (500 ms on / 500 ms off).
 *
 * @return 0 on success, negative errno on failure.
 */
int asr_led_thread_start(void);

#ifdef __cplusplus
}
#endif

#endif /* ASR_LED_THREAD_H_ */
