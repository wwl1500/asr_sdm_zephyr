/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Background thread that drives the user LED and RGB LED.
 */

#ifndef ASR_LED_THREAD_H_
#define ASR_LED_THREAD_H_

#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * Start the LED control thread.
     * - User LED: held on (steady).
     * - RGB LED:  blinks blue (500 ms on / 500 ms off) when enabled.
     *
     * @return 0 on success, negative errno on failure.
     */
    int asr_led_thread_start(void);

    /**
     * Enable or disable the RGB LED toggle in the background thread.
     * When disabled the LED is turned off; when re-enabled it resumes blinking.
     *
     * @param enable  true to blink, false to stop and turn off.
     */
    void asr_led_thread_set_rgb_blink(bool enable);

#ifdef __cplusplus
}
#endif

#endif /* ASR_LED_THREAD_H_ */
