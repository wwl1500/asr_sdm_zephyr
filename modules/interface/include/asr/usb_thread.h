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
 * Initialise the USB protocol and start the RX processing thread.
 *
 * @return 0 on success, -EALREADY when already started, or another negative
 *         errno on failure.
 */
int asr_usb_protocol_thread_start(void);

#ifdef __cplusplus
}
#endif

#endif /* ASR_USB_THREAD_H_ */
