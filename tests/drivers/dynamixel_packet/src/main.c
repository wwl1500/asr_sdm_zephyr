/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <zephyr/ztest.h>

#include "dynamixel_packet.h"

ZTEST(dynamixel_packet, test_build_read_present_position_packet)
{
	static const uint8_t expected[] = {
		0xFF, 0xFF, 0xFD, 0x00,
		0x01, 0x07, 0x00, 0x02,
		0x84, 0x00, 0x04, 0x00,
		0x1D, 0x15,
	};
	uint8_t params[] = {0x84, 0x00, 0x04, 0x00};
	uint8_t packet[ASR_DYNAMIXEL_MAX_PACKET_SIZE];
	size_t packet_len;

	packet_len = asr_dynamixel_build_instruction_packet(
		0x01, ASR_DYNAMIXEL_INST_READ, params, sizeof(params),
		packet, sizeof(packet));

	zassert_equal(packet_len, sizeof(expected), "unexpected packet length");
	zassert_mem_equal(packet, expected, sizeof(expected),
			  "unexpected read packet");
}

ZTEST(dynamixel_packet, test_build_write_goal_position_packet)
{
	static const uint8_t expected[] = {
		0xFF, 0xFF, 0xFD, 0x00,
		0x01, 0x09, 0x00, 0x03,
		0x74, 0x00, 0x00, 0x02,
		0x00, 0x00, 0xCA, 0x89,
	};
	uint8_t params[] = {0x74, 0x00, 0x00, 0x02, 0x00, 0x00};
	uint8_t packet[ASR_DYNAMIXEL_MAX_PACKET_SIZE];
	size_t packet_len;

	packet_len = asr_dynamixel_build_instruction_packet(
		0x01, ASR_DYNAMIXEL_INST_WRITE, params, sizeof(params),
		packet, sizeof(packet));

	zassert_equal(packet_len, sizeof(expected), "unexpected packet length");
	zassert_mem_equal(packet, expected, sizeof(expected),
			  "unexpected write packet");
}

ZTEST(dynamixel_packet, test_build_packet_applies_byte_stuffing)
{
	uint8_t params[] = {0xFF, 0xFF, 0xFD};
	uint8_t packet[ASR_DYNAMIXEL_MAX_PACKET_SIZE];
	size_t packet_len;

	packet_len = asr_dynamixel_build_instruction_packet(
		0x01, ASR_DYNAMIXEL_INST_WRITE, params, sizeof(params),
		packet, sizeof(packet));

	zassert_equal(packet_len, 14U, "unexpected stuffed packet length");
	zassert_equal(packet[5], 0x07, "unexpected stuffed length low byte");
	zassert_equal(packet[6], 0x00, "unexpected stuffed length high byte");
	zassert_equal(packet[7], ASR_DYNAMIXEL_INST_WRITE,
		      "unexpected instruction");
	zassert_equal(packet[8], 0xFF, "missing first stuffed byte");
	zassert_equal(packet[9], 0xFF, "missing second stuffed byte");
	zassert_equal(packet[10], 0xFD, "missing header sequence byte");
	zassert_equal(packet[11], 0xFD, "missing stuffed byte");
}

ZTEST(dynamixel_packet, test_parse_status_packet)
{
	static const uint8_t packet[] = {
		0xFF, 0xFF, 0xFD, 0x00,
		0x01, 0x08, 0x00, 0x55,
		0x00, 0xA6, 0x00, 0x00,
		0x00, 0x8C, 0xC0,
	};
	static const uint8_t expected_params[] = {0xA6, 0x00, 0x00, 0x00};
	struct asr_dynamixel_status_packet status = {0};
	int ret;

	ret = asr_dynamixel_parse_status_packet(packet, sizeof(packet), &status);

	zassert_equal(ret, 0, "status parsing failed");
	zassert_equal(status.id, 0x01, "unexpected id");
	zassert_false(status.alert, "unexpected alert bit");
	zassert_equal(status.error, 0x00, "unexpected error");
	zassert_equal(status.param_len, 4U, "unexpected parameter length");
	zassert_mem_equal(status.params, expected_params,
			  sizeof(expected_params), "unexpected parameters");
}

ZTEST(dynamixel_packet, test_parse_status_packet_rejects_bad_crc)
{
	uint8_t packet[] = {
		0xFF, 0xFF, 0xFD, 0x00,
		0x01, 0x08, 0x00, 0x55,
		0x00, 0xA6, 0x00, 0x00,
		0x00, 0x8C, 0xC0,
	};
	struct asr_dynamixel_status_packet status = {0};
	int ret;

	packet[14] ^= 0x01;
	ret = asr_dynamixel_parse_status_packet(packet, sizeof(packet), &status);
	zassert_equal(ret, -EBADMSG, "unexpected CRC handling");
}

ZTEST(dynamixel_packet, test_parse_status_packet_rejects_bad_header)
{
	uint8_t packet[] = {
		0x00, 0xFF, 0xFD, 0x00,
		0x01, 0x08, 0x00, 0x55,
		0x00, 0xA6, 0x00, 0x00,
		0x00, 0x8C, 0xC0,
	};
	struct asr_dynamixel_status_packet status = {0};
	int ret;

	ret = asr_dynamixel_parse_status_packet(packet, sizeof(packet), &status);
	zassert_equal(ret, -EPROTO, "unexpected header validation");
}

ZTEST(dynamixel_packet, test_parse_status_packet_rejects_length_mismatch)
{
	uint8_t packet[] = {
		0xFF, 0xFF, 0xFD, 0x00,
		0x01, 0x07, 0x00, 0x55,
		0x00, 0xA6, 0x00, 0x00,
		0x00, 0x8C, 0xC0,
	};
	struct asr_dynamixel_status_packet status = {0};
	int ret;

	ret = asr_dynamixel_parse_status_packet(packet, sizeof(packet), &status);
	zassert_equal(ret, -EMSGSIZE, "unexpected length validation");
}

ZTEST(dynamixel_packet, test_parse_status_packet_rejects_bad_instruction)
{
	uint8_t packet[] = {
		0xFF, 0xFF, 0xFD, 0x00,
		0x01, 0x08, 0x00, 0x54,
		0x00, 0xA6, 0x00, 0x00,
		0x00, 0xF9, 0x10,
	};
	struct asr_dynamixel_status_packet status = {0};
	int ret;

	ret = asr_dynamixel_parse_status_packet(packet, sizeof(packet), &status);
	zassert_equal(ret, -EPROTO, "unexpected instruction validation");
}

ZTEST(dynamixel_packet, test_status_error_mapping)
{
	zassert_equal(asr_dynamixel_map_status_error(0x00), 0,
		      "unexpected success mapping");
	zassert_equal(asr_dynamixel_map_status_error(0x02), -ENOTSUP,
		      "unexpected instruction error mapping");
	zassert_equal(asr_dynamixel_map_status_error(0x05), -EMSGSIZE,
		      "unexpected length error mapping");
	zassert_equal(asr_dynamixel_map_status_error(0x07), -EACCES,
		      "unexpected access error mapping");
}

ZTEST_SUITE(dynamixel_packet, NULL, NULL, NULL, NULL, NULL);
