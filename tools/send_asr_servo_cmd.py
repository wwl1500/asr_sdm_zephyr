#!/usr/bin/env python3
"""
Send ASR UART0 control frames to the RP2350 firmware.

The wire format is:
  [0xAA] [0x55] [LEN] [8-byte payload] [CHECKSUM]

CHECKSUM = LEN ^ payload[0] ^ ... ^ payload[7]
"""

from __future__ import annotations

import argparse
import struct
import sys
import time


FRAME_SYNC_HI = 0xAA
FRAME_SYNC_LO = 0x55
PAYLOAD_LEN = 8

CMD_READ = 0x02
CMD_WRITE = 0x03
PARAM_JOINT1 = 0x06
PARAM_JOINT1_TORQUE = 0x08


def build_frame(payload: bytes) -> bytes:
    if len(payload) != PAYLOAD_LEN:
        raise ValueError(f"payload must be {PAYLOAD_LEN} bytes")

    checksum = PAYLOAD_LEN
    for byte in payload:
        checksum ^= byte

    return bytes([FRAME_SYNC_HI, FRAME_SYNC_LO, PAYLOAD_LEN]) + payload + bytes([checksum])


def build_torque_payload(enable: bool) -> bytes:
    # payload[0:2] are currently unused by the firmware. Keep them zero so the
    # frame is not misidentified as a Dynamixel packet header.
    return bytes([
        0x00,
        0x00,
        CMD_WRITE,
        PARAM_JOINT1_TORQUE,
        0x01 if enable else 0x00,
        0x00,
        0x00,
        0x00,
    ])


def build_goal_payload(position: int) -> bytes:
    if not (-2**31 <= position <= 2**31 - 1):
        raise ValueError("goal position must fit in signed int32")

    return bytes([
        0x00,
        0x00,
        CMD_WRITE,
        PARAM_JOINT1,
    ]) + struct.pack("<i", position)


def build_read_position_payload() -> bytes:
    return bytes([
        0x00,
        0x00,
        CMD_READ,
        PARAM_JOINT1,
        0x00,
        0x00,
        0x00,
        0x00,
    ])


def format_hex(data: bytes) -> str:
    return " ".join(f"{byte:02X}" for byte in data)


def read_frame(ser, timeout: float) -> bytes:
    deadline = time.monotonic() + timeout
    state = 0

    while time.monotonic() < deadline:
        byte = ser.read(1)
        if not byte:
            continue

        value = byte[0]
        if state == 0:
            if value == FRAME_SYNC_HI:
                state = 1
            continue

        if state == 1:
            if value == FRAME_SYNC_LO:
                state = 2
            elif value != FRAME_SYNC_HI:
                state = 0
            continue

        payload_len = value
        payload = ser.read(payload_len)
        checksum = ser.read(1)
        if len(payload) != payload_len or len(checksum) != 1:
            raise RuntimeError("timed out while reading reply frame")

        expected_checksum = payload_len
        for payload_byte in payload:
            expected_checksum ^= payload_byte

        if checksum[0] != expected_checksum:
            raise RuntimeError(
                f"reply checksum mismatch: expected 0x{expected_checksum:02X}, got 0x{checksum[0]:02X}"
            )

        return bytes([FRAME_SYNC_HI, FRAME_SYNC_LO, payload_len]) + payload + checksum

    raise RuntimeError("timed out waiting for reply frame")


def send_frame(port: str, baud: int, timeout: float, frame: bytes, expect_reply: bool) -> bytes | None:
    try:
        import serial  # type: ignore
    except ImportError as exc:
        raise RuntimeError(
            "pyserial is required for live serial transmission. "
            "Install it with 'python3 -m pip install pyserial', or use --dry-run."
        ) from exc

    with serial.Serial(port=port, baudrate=baud, timeout=timeout) as ser:
        ser.write(frame)
        ser.flush()
        if not expect_reply:
            return None
        return read_frame(ser, timeout)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Send ASR UART0 servo-control frames to the RP2350 firmware."
    )
    parser.add_argument(
        "--port",
        help="serial port connected to RP2350 UART0 (for example /dev/ttyUSB0 or COM3)",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="UART0 baud rate (default: 115200)",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=1.0,
        help="serial open/write timeout in seconds (default: 1.0)",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="print the final frame without opening a serial port",
    )

    subparsers = parser.add_subparsers(dest="command", required=True)

    torque_parser = subparsers.add_parser("torque", help="set JOINT1 torque on or off")
    torque_parser.add_argument("state", choices=("on", "off"))

    goal_parser = subparsers.add_parser("goal", help="send a JOINT1 goal position")
    goal_parser.add_argument("position", type=int, help="signed int32 goal position")

    subparsers.add_parser("read-position", help="read the current JOINT1 position")

    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    expect_reply = args.command == "read-position"

    try:
        if args.command == "torque":
            payload = build_torque_payload(args.state == "on")
        elif args.command == "goal":
            payload = build_goal_payload(args.position)
        else:
            payload = build_read_position_payload()

        frame = build_frame(payload)
    except ValueError as exc:
        parser.error(str(exc))

    print(f"payload: {format_hex(payload)}")
    print(f"frame:   {format_hex(frame)}")

    if args.dry_run:
        return 0

    if not args.port:
        parser.error("--port is required unless --dry-run is used")

    try:
        reply = send_frame(args.port, args.baud, args.timeout, frame, expect_reply)
    except Exception as exc:  # pragma: no cover - exercised via CLI/runtime
        print(f"error: {exc}", file=sys.stderr)
        return 1

    print(f"sent {len(frame)} bytes to {args.port} @ {args.baud} bps")

    if reply is not None:
        reply_payload = reply[3:-1]
        print(f"reply payload: {format_hex(reply_payload)}")
        print(f"reply frame:   {format_hex(reply)}")
        if len(reply_payload) == PAYLOAD_LEN:
            position = struct.unpack("<i", reply_payload[4:8])[0]
            print(f"position: {position}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
