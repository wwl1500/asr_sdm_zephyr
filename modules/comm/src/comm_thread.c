/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Communication processing thread for the ASR SDM protocol.
 *
 * Dequeues framed messages assembled by the UART ISR (in protocol.c) and
 * dispatches them through the protocol handler.
 */

#include <asr/comm_thread.h>

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(asr_comm, LOG_LEVEL_INF);

/* Provided by protocol.c */
extern struct k_msgq rx_msgq;
extern struct k_sem rx_frame_sem;
extern int asr_comm_uart_init(void);
extern bool protocol_init(void);
extern bool protocol_update(const uint8_t msg[ASR_COMM_MSG_SIZE]);

static K_THREAD_STACK_DEFINE(comm_stack, CONFIG_ASR_COMM_THREAD_STACK_SIZE);
static struct k_thread comm_thread_data;
static bool comm_started;

static void comm_thread_entry(void *p1, void *p2, void *p3)
{
    uint8_t msg[ASR_COMM_MSG_SIZE];

    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    protocol_init();

    if (asr_comm_uart_init() < 0)
    {
        LOG_ERR("UART0 init failed, comm thread exiting");
        return;
    }

    for (;;)
    {
        k_sem_take(&rx_frame_sem, K_FOREVER);

        if (k_msgq_get(&rx_msgq, msg, K_NO_WAIT) == 0)
        {
            if (protocol_update(msg))
            {
                LOG_DBG("processed cmd=0x%02x param=0x%02x", msg[2], msg[3]);
            }
        }
    }
}

int asr_comm_thread_init(void)
{
    if (comm_started)
    {
        LOG_WRN("communication thread already initialised");
        return -EALREADY;
    }

    k_thread_create(&comm_thread_data, comm_stack, K_THREAD_STACK_SIZEOF(comm_stack), comm_thread_entry, NULL, NULL,
                    NULL, CONFIG_ASR_COMM_THREAD_PRIORITY, 0, K_FOREVER);
    k_thread_name_set(&comm_thread_data, "comm_thread");
    comm_started = true;

    LOG_INF("communication thread initialised (suspended), prio %d", k_thread_priority_get(&comm_thread_data));
    return 0;
}

int asr_comm_thread_start(void)
{
    if (!comm_started)
    {
        LOG_ERR("communication thread not initialised");
        return -EINVAL;
    }
    LOG_INF("communication thread started");
    k_thread_start(&comm_thread_data);
    return 0;
}
