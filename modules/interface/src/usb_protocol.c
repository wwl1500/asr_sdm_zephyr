/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * USB CDC ACM device access: IRQ-driven RX, poll-out TX, DTR check.
 *
 * Received bytes are accumulated into fixed-size chunks and posted to
 * usb_rx_msgq for the background thread (usb_thread.c) to deliver.
 */

#include <asr/usb_protocol.h>

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(asr_usb_if, LOG_LEVEL_INF);

#define USB_CDC_NODE DT_NODELABEL(cdc_acm_uart0)

BUILD_ASSERT(DT_NODE_EXISTS(USB_CDC_NODE),
	     "cdc_acm_uart0 node must exist in devicetree");

#define RX_BUF_SIZE CONFIG_ASR_USB_INTERFACE_RX_BUF_SIZE

static const struct device *const usb_dev = DEVICE_DT_GET(USB_CDC_NODE);

struct usb_rx_chunk {
	uint8_t len;
	uint8_t data[RX_BUF_SIZE];
};

#define RX_QUEUE_DEPTH 4
K_MSGQ_DEFINE(usb_rx_msgq, sizeof(struct usb_rx_chunk), RX_QUEUE_DEPTH, 4);

asr_usb_rx_cb_t usb_rx_cb;

/* --- RX ISR -------------------------------------------------------------- */

static void uart_isr_cb(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		if (!uart_irq_rx_ready(dev)) {
			continue;
		}

		struct usb_rx_chunk chunk;

		chunk.len = 0;
		while (chunk.len < RX_BUF_SIZE) {
			uint8_t byte;

			if (uart_fifo_read(dev, &byte, 1) != 1) {
				break;
			}
			chunk.data[chunk.len++] = byte;
		}

		if (chunk.len > 0) {
			k_msgq_put(&usb_rx_msgq, &chunk, K_NO_WAIT);
		}
	}
}

/* --- public API ---------------------------------------------------------- */

void asr_usb_protocol_register_rx_cb(asr_usb_rx_cb_t cb)
{
	usb_rx_cb = cb;
}

bool asr_usb_protocol_is_connected(void)
{
	uint32_t dtr = 0U;

	if (!device_is_ready(usb_dev)) {
		return false;
	}

	if (uart_line_ctrl_get(usb_dev, UART_LINE_CTRL_DTR, &dtr) < 0) {
		return false;
	}

	return dtr != 0U;
}

int asr_usb_protocol_send(const uint8_t *data, size_t len)
{
	if (!device_is_ready(usb_dev)) {
		return -ENODEV;
	}

	for (size_t i = 0; i < len; i++) {
		uart_poll_out(usb_dev, data[i]);
	}

	return 0;
}

int asr_usb_protocol_init(void)
{
	if (!device_is_ready(usb_dev)) {
		LOG_ERR("USB CDC ACM device not ready");
		return -ENODEV;
	}

	uart_irq_callback_set(usb_dev, uart_isr_cb);
	uart_irq_rx_enable(usb_dev);

	LOG_INF("USB CDC ACM protocol initialised");
	return 0;
}
