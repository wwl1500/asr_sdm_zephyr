/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Protocol handler for the ASR SDM communication module.
 *
 * Parses 8-byte messages (originally CAN frames, now carried over UART) and
 * dispatches Write commands that modify unit state and optionally invoke
 * hardware callbacks.  Read commands are parsed but left as stubs for the
 * application to extend.
 */

#include <asr/comm_thread.h>
#include <string.h>

#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(asr_comm, LOG_LEVEL_INF);

static asr_unit_status_t unit_status;
static const struct asr_comm_callbacks *hw_cb;

void asr_comm_register_callbacks(const struct asr_comm_callbacks *cb)
{
	hw_cb = cb;
}

const asr_unit_status_t *asr_comm_get_status(void)
{
	return &unit_status;
}

bool protocol_init(void)
{
	memset(&unit_status, 0, sizeof(unit_status));

	unit_status.flash_data[0] = 0x00;
	unit_status.flash_data[1] = 0x01;
	unit_status.unit_id = (uint16_t)unit_status.flash_data[0] << 3 |
			      unit_status.flash_data[1];

	return true;
}

bool protocol_update(const uint8_t msg[ASR_COMM_MSG_SIZE])
{
	if (msg[0] == 0xFF && msg[1] == 0xFD) {
		return false;
	}

	if (msg[2] == ASR_COMM_CMD_READ) {
		switch (msg[3]) {
		case ASR_COMM_PARAM_BOARD_ID:
		case ASR_COMM_PARAM_CAN_ID:
		case ASR_COMM_PARAM_LED_ENABLE:
		case ASR_COMM_PARAM_LED_STATUS:
		case ASR_COMM_PARAM_MOTOR:
		case ASR_COMM_PARAM_JOINT1:
		case ASR_COMM_PARAM_JOINT2:
		case ASR_COMM_PARAM_JOINT1_TORQUE:
		case ASR_COMM_PARAM_JOINT2_TORQUE:
			break;
		case ASR_COMM_PARAM_IMU: {
			uint8_t reply[ASR_COMM_MSG_SIZE] = {0};

			if (hw_cb && hw_cb->on_imu_read &&
			    hw_cb->on_imu_read(reply) == 0) {
				asr_comm_send(reply);
			}
			break;
		}
		default:
			break;
		}
	} else if (msg[2] == ASR_COMM_CMD_WRITE) {
		switch (msg[3]) {
		case ASR_COMM_PARAM_CAN_ID:
			unit_status.flash_data[0] = msg[6];
			unit_status.flash_data[1] = msg[7];
			if (hw_cb && hw_cb->on_flash_write) {
				hw_cb->on_flash_write(unit_status.flash_data,
						      sizeof(unit_status.flash_data));
			}
			break;

		case ASR_COMM_PARAM_LED_ENABLE:
			if (msg[7] == 1) {
				unit_status.led_enable = true;
			} else if (msg[7] == 0) {
				unit_status.led_enable = false;
				unit_status.led_status = true;
				if (hw_cb && hw_cb->on_led_write) {
					hw_cb->on_led_write(true);
				}
			}
			break;

		case ASR_COMM_PARAM_LED_STATUS:
			break;

		case ASR_COMM_PARAM_MOTOR:
			unit_status.cmd_motor[0] = msg[6];
			unit_status.cmd_motor[1] = msg[7];
			if (hw_cb && hw_cb->on_motor_set) {
				hw_cb->on_motor_set(0, unit_status.cmd_motor[0]);
				hw_cb->on_motor_set(1, unit_status.cmd_motor[1]);
			}
			break;

		case ASR_COMM_PARAM_JOINT1:
			unit_status.cmd_joint1[0] = msg[4];
			unit_status.cmd_joint1[1] = msg[5];
			unit_status.cmd_joint1[2] = msg[6];
			unit_status.cmd_joint1[3] = msg[7];
			break;

		case ASR_COMM_PARAM_JOINT2:
			unit_status.cmd_joint2[0] = msg[4];
			unit_status.cmd_joint2[1] = msg[5];
			unit_status.cmd_joint2[2] = msg[6];
			unit_status.cmd_joint2[3] = msg[7];
			break;

		case ASR_COMM_PARAM_JOINT1_TORQUE:
			unit_status.dynamixel_enable[ASR_DXL_1] = (bool)msg[4];
			if (hw_cb && hw_cb->on_dynamixel_torque) {
				hw_cb->on_dynamixel_torque(
					ASR_DXL_1,
					unit_status.dynamixel_enable[ASR_DXL_1]);
			}
			break;

		case ASR_COMM_PARAM_JOINT2_TORQUE:
			unit_status.dynamixel_enable[ASR_DXL_2] = (bool)msg[4];
			if (hw_cb && hw_cb->on_dynamixel_torque) {
				hw_cb->on_dynamixel_torque(
					ASR_DXL_2,
					unit_status.dynamixel_enable[ASR_DXL_2]);
			}
			break;

		default:
			break;
		}
	}

	return true;
}
