/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * USB protocol RX background thread.
 *
 * Dequeues received chunks posted by the ISR (in usb_protocol.c) and
 * delivers them to the registered callback.
 */

#include <asr/usb_protocol.h>
#include <asr/usb_thread.h>

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(asr_usb_if, LOG_LEVEL_INF);

#define RX_BUF_SIZE CONFIG_ASR_USB_INTERFACE_RX_BUF_SIZE

/* Provided by usb_protocol.c */
struct usb_rx_chunk
{
    uint8_t len;
    uint8_t data[RX_BUF_SIZE];
};

extern struct k_msgq usb_rx_msgq;
extern asr_usb_rx_cb_t usb_rx_cb;

static K_THREAD_STACK_DEFINE(rx_stack, CONFIG_ASR_USB_INTERFACE_THREAD_STACK_SIZE);
static struct k_thread rx_thread_data;
static bool thread_started;

static void rx_thread_entry(void *p1, void *p2, void *p3)
{
    struct usb_rx_chunk chunk;

    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    if (asr_usb_protocol_init() < 0)
    {
        LOG_ERR("USB CDC ACM init failed, thread exiting");
        return;
    }

    for (;;)
    {
        if (k_msgq_get(&usb_rx_msgq, &chunk, K_FOREVER) == 0)
        {
            if (usb_rx_cb != NULL)
            {
                usb_rx_cb(chunk.data, chunk.len);
            }
        }
    }
}

int asr_usb_protocol_thread_init(void)
{
    if (thread_started)
    {
        LOG_WRN("USB protocol thread already initialised");
        return -EALREADY;
    }

    k_thread_create(&rx_thread_data, rx_stack, K_THREAD_STACK_SIZEOF(rx_stack), rx_thread_entry, NULL, NULL, NULL,
                    CONFIG_ASR_USB_INTERFACE_THREAD_PRIORITY, 0, K_FOREVER);
    k_thread_name_set(&rx_thread_data, "usb_proto_rx");

    thread_started = true;
    LOG_INF("USB protocol thread initialised (suspended), prio %d", k_thread_priority_get(&rx_thread_data));
    return 0;
}

int asr_usb_protocol_thread_start(void)
{
    if (!thread_started)
    {
        LOG_ERR("USB protocol thread not initialised");
        return -EINVAL;
    }
    LOG_INF("USB protocol thread started");
    k_thread_start(&rx_thread_data);
    return 0;
}
