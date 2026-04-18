/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Raw USB CDC ACM data pipe.
 */

#ifndef ASR_USB_PROTOCOL_H_
#define ASR_USB_PROTOCOL_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*asr_usb_rx_cb_t)(const uint8_t *data, size_t len);

/**
 * Register a callback invoked (from the RX thread) for each received chunk.
 *
 * @param cb  Callback function, or NULL to clear.
 */
void asr_usb_protocol_register_rx_cb(asr_usb_rx_cb_t cb);

/**
 * Initialise the USB CDC ACM device and enable IRQ-driven RX.
 *
 * @return 0 on success, negative errno on failure.
 */
int asr_usb_protocol_init(void);

/**
 * Send raw bytes over USB CDC ACM (blocking poll-out).
 *
 * @param data  Pointer to the data to send.
 * @param len   Number of bytes to send.
 * @return 0 on success, negative errno on failure.
 */
int asr_usb_protocol_send(const uint8_t *data, size_t len);

/**
 * Check whether the USB host has the virtual serial port open (DTR asserted).
 *
 * @return true if connected, false otherwise.
 */
bool asr_usb_protocol_is_connected(void);

#ifdef __cplusplus
}
#endif

#endif /* ASR_USB_PROTOCOL_H_ */
