/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * UART0 transport layer for the ASR SDM protocol.
 *
 * Interrupt-driven RX reassembles framed messages through a byte-level state
 * machine and enqueues complete payloads for the processing thread.  TX uses
 * blocking poll-out (suitable for low-frequency command responses).
 *
 * Wire frame:
 *   [0xAA] [0x55] [LEN] [DATA_0 .. DATA_N-1] [CHK]
 *   CHK = LEN ^ DATA_0 ^ ... ^ DATA_N-1
 */

#include <asr/comm_thread.h>

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(asr_comm, LOG_LEVEL_INF);

#define COMM_UART_NODE DT_NODELABEL(uart0)

#define FRAME_SYNC_HI    0xAAU
#define FRAME_SYNC_LO    0x55U
#define FRAME_MAX_PAYLOAD ASR_COMM_MSG_SIZE

BUILD_ASSERT(DT_NODE_EXISTS(COMM_UART_NODE),
	     "uart0 node must exist in devicetree");

/* --- RX state machine ---------------------------------------------------- */

enum rx_state {
	RX_SYNC1,
	RX_SYNC2,
	RX_LENGTH,
	RX_DATA,
	RX_CHECKSUM,
};

static const struct device *const uart_dev = DEVICE_DT_GET(COMM_UART_NODE);

static K_THREAD_STACK_DEFINE(comm_stack, CONFIG_ASR_COMM_THREAD_STACK_SIZE);
static struct k_thread comm_thread_data;
static bool comm_started;

K_MSGQ_DEFINE(rx_msgq, ASR_COMM_MSG_SIZE, CONFIG_ASR_COMM_RX_QUEUE_DEPTH, 4);

static struct {
	enum rx_state state;
	uint8_t buf[FRAME_MAX_PAYLOAD];
	uint8_t expected_len;
	uint8_t idx;
	uint8_t checksum;
} rx_ctx;

/* Provided by protocol.c */
extern bool protocol_init(void);
extern bool protocol_update(const uint8_t msg[ASR_COMM_MSG_SIZE]);

static void rx_process_byte(uint8_t byte)
{
	switch (rx_ctx.state) {
	case RX_SYNC1:
		if (byte == FRAME_SYNC_HI) {
			rx_ctx.state = RX_SYNC2;
		}
		break;

	case RX_SYNC2:
		rx_ctx.state = (byte == FRAME_SYNC_LO) ? RX_LENGTH : RX_SYNC1;
		break;

	case RX_LENGTH:
		if (byte == 0U || byte > FRAME_MAX_PAYLOAD) {
			rx_ctx.state = RX_SYNC1;
			break;
		}
		rx_ctx.expected_len = byte;
		rx_ctx.idx = 0U;
		rx_ctx.checksum = byte;
		rx_ctx.state = RX_DATA;
		break;

	case RX_DATA:
		rx_ctx.buf[rx_ctx.idx++] = byte;
		rx_ctx.checksum ^= byte;
		if (rx_ctx.idx >= rx_ctx.expected_len) {
			rx_ctx.state = RX_CHECKSUM;
		}
		break;

	case RX_CHECKSUM:
		if (byte == rx_ctx.checksum) {
			if (rx_ctx.expected_len < ASR_COMM_MSG_SIZE) {
				memset(&rx_ctx.buf[rx_ctx.expected_len], 0,
				       ASR_COMM_MSG_SIZE - rx_ctx.expected_len);
			}
			k_msgq_put(&rx_msgq, rx_ctx.buf, K_NO_WAIT);
		} else {
			LOG_WRN("frame checksum mismatch");
		}
		rx_ctx.state = RX_SYNC1;
		break;
	}
}

static void uart_isr_cb(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		if (!uart_irq_rx_ready(dev)) {
			continue;
		}
		uint8_t byte;

		while (uart_fifo_read(dev, &byte, 1) == 1) {
			rx_process_byte(byte);
		}
	}
}

/* --- processing thread --------------------------------------------------- */

static void comm_thread_entry(void *p1, void *p2, void *p3)
{
	uint8_t msg[ASR_COMM_MSG_SIZE];

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	for (;;) {
		if (k_msgq_get(&rx_msgq, msg, K_FOREVER) == 0) {
			if (protocol_update(msg)) {
				LOG_DBG("processed cmd=0x%02x param=0x%02x",
					msg[2], msg[3]);
			}
		}
	}
}

/* --- public API ---------------------------------------------------------- */

int asr_comm_init(void)
{
	if (!device_is_ready(uart_dev)) {
		LOG_ERR("UART0 device not ready");
		return -ENODEV;
	}

	protocol_init();

	LOG_INF("UART0 communication module initialised");
	return 0;
}

int asr_comm_thread_start(void)
{
	int ret;

	if (comm_started) {
		LOG_WRN("communication thread already running");
		return -EALREADY;
	}

	ret = asr_comm_init();
	if (ret < 0) {
		return ret;
	}

	uart_irq_callback_set(uart_dev, uart_isr_cb);
	uart_irq_rx_enable(uart_dev);

	k_thread_create(&comm_thread_data, comm_stack,
			K_THREAD_STACK_SIZEOF(comm_stack),
			comm_thread_entry, NULL, NULL, NULL,
			CONFIG_ASR_COMM_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&comm_thread_data, "comm_thread");
	comm_started = true;

	LOG_INF("UART0 communication thread started (prio %d)",
		CONFIG_ASR_COMM_THREAD_PRIORITY);
	return 0;
}

int asr_comm_send(const uint8_t data[ASR_COMM_MSG_SIZE])
{
	if (!device_is_ready(uart_dev)) {
		return -ENODEV;
	}

	uint8_t checksum = ASR_COMM_MSG_SIZE;

	uart_poll_out(uart_dev, FRAME_SYNC_HI);
	uart_poll_out(uart_dev, FRAME_SYNC_LO);
	uart_poll_out(uart_dev, ASR_COMM_MSG_SIZE);

	for (int i = 0; i < ASR_COMM_MSG_SIZE; i++) {
		uart_poll_out(uart_dev, data[i]);
		checksum ^= data[i];
	}

	uart_poll_out(uart_dev, checksum);
	return 0;
}
