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
     * Initialise and create the LED thread in suspended state.
     *
     * @return 0 on success, negative errno on failure.
     */
    int asr_led_thread_init(void);

    /**
     * Start the LED thread (k_thread_start; must call asr_led_thread_init first).
     *
     * @return 0 on success.
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
