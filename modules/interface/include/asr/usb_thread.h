/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Background RX thread for the USB CDC ACM protocol.
 */

#ifndef ASR_USB_THREAD_H_
#define ASR_USB_THREAD_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialise the USB protocol and create the RX thread in suspended state.
 *
 * @return 0 on success, -EALREADY when already initialised, or another
 *         negative errno on failure.
 */
int asr_usb_protocol_thread_init(void);

/**
 * Start the USB protocol RX thread (k_thread_start; must call init first).
 *
 * @return 0 on success, -EINVAL if not initialised.
 */
int asr_usb_protocol_thread_start(void);

#ifdef __cplusplus
}
#endif

#endif /* ASR_USB_THREAD_H_ */
