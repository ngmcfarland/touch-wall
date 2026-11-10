#!/usr/bin/env python3
"""Poll touch nodes, maintain the 568-sensor wall state, and publish OSC deltas.

Install dependencies with:
    python -m pip install pyserial python-osc

Example:
    python wall_touch_receiver.py --port /dev/ttyUSB0 \
        --osc-host 127.0.0.1 --osc-port 9000
"""

import argparse
import os
import sys
import time
from typing import Iterable, List, Optional, Sequence, Tuple

import serial
from dotenv import load_dotenv
from pythonosc.udp_client import SimpleUDPClient


BOARD_COUNT: int = 24
SENSORS_PER_BOARD: int = 24
FINAL_BOARD_SENSOR_COUNT: int = 16
SHORT_BOARD_ID: int = 23
TOTAL_SENSOR_COUNT: int = (BOARD_COUNT - 1) * SENSORS_PER_BOARD + FINAL_BOARD_SENSOR_COUNT

UART_BAUDRATE: int = 115200
SOF: int = 0xA5
EOF_MARKER: int = 0x5A
POLL_COMMAND: int = 0x01
STATE_COMMAND: int = 0x81


def crc8(data: Iterable[int]) -> int:
    """Return the CRC-8/ATM checksum for the provided byte sequence."""
    crc: int = 0
    for value in data:
        crc ^= value
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def expected_sensor_count(board_id: int) -> int:
    """Return the number of sensor channels mounted on a board."""
    if not 0 <= board_id < BOARD_COUNT:
        raise ValueError("board_id must be between 0 and 23")
    return FINAL_BOARD_SENSOR_COUNT if board_id == SHORT_BOARD_ID else SENSORS_PER_BOARD


def first_global_sensor(board_id: int) -> int:
    """Map a board-local sensor index to its index in the full wall array."""
    offset: int = FINAL_BOARD_SENSOR_COUNT - SENSORS_PER_BOARD
    return board_id * SENSORS_PER_BOARD + (offset if board_id > SHORT_BOARD_ID else 0)


def pack_poll_request(board_id: int) -> bytes:
    """Create the 4-byte poll request understood by the wall touch nodes."""
    body: bytes = bytes((POLL_COMMAND, board_id))
    return bytes((SOF,)) + body + bytes((crc8(body),))


def unpack_state_payload(payload: bytes, sensor_count: int) -> List[bool]:
    """Unpack a board payload into local sensor touched/not-touched states."""
    return [
        bool(payload[index // 8] & (1 << (index % 8)))
        for index in range(sensor_count)
    ]


def read_state_response(
    port: serial.Serial,
    expected_board_id: int,
    timeout: float,
) -> Optional[Tuple[int, List[bool]]]:
    """Read and validate one addressed board response, or return None on timeout."""
    sensor_count: int = expected_sensor_count(expected_board_id)
    payload_size: int = (sensor_count + 7) // 8
    frame_size: int = payload_size + 7
    buffer: bytearray = bytearray()
    deadline: float = time.monotonic() + timeout

    while time.monotonic() < deadline:
        byte = port.read(1)
        if not byte:
            continue
        buffer.extend(byte)

        while buffer:
            if buffer[0] != SOF:
                del buffer[0]
                continue
            if len(buffer) < frame_size:
                break

            frame = bytes(buffer[:frame_size])
            body = frame[1:-2]
            valid = (
                frame[1] == STATE_COMMAND
                and frame[2] == expected_board_id
                and frame[3] == sensor_count
                and frame[-2] == crc8(body)
                and frame[-1] == EOF_MARKER
            )
            if not valid:
                del buffer[0]
                continue

            del buffer[:frame_size]
            sequence: int = frame[4]
            payload: bytes = frame[5:-2]
            return sequence, unpack_state_payload(payload, sensor_count)

    return None


class WallState:
    """Track the latest state and initialization status of every wall sensor."""

    def __init__(self) -> None:
        """Create an empty state snapshot for the full wall and each board."""
        self.states: List[bool] = [False] * TOTAL_SENSOR_COUNT
        self.initialized_boards: List[bool] = [False] * BOARD_COUNT

    def update_board(
        self,
        board_id: int,
        local_states: Sequence[bool],
    ) -> List[Tuple[int, bool]]:
        """Apply a board's sensor state and return every changed global sensor."""
        expected: int = expected_sensor_count(board_id)
        if len(local_states) != expected:
            raise ValueError(
                f"board {board_id} reported {len(local_states)} sensors; expected {expected}"
            )

        first_sensor: int = first_global_sensor(board_id)
        first_update: bool = not self.initialized_boards[board_id]
        changed: List[Tuple[int, bool]] = []
        for local_index, touched in enumerate(local_states):
            global_index: int = first_sensor + local_index
            touched = bool(touched)
            if first_update or self.states[global_index] != touched:
                changed.append((global_index, touched))
            self.states[global_index] = touched

        self.initialized_boards[board_id] = True
        return changed


def poll_board(
    port: serial.Serial,
    board_id: int,
    response_timeout: float,
) -> Optional[Tuple[int, List[bool]]]:
    """Send a poll request to one board and return its decoded response."""
    request: bytes = pack_poll_request(board_id)
    written: int = port.write(request)  # type: ignore
    if written != len(request):
        raise IOError(
            f"Serial write incomplete for board {board_id} ({written} of {len(request)} bytes)"
        )
    return read_state_response(port, board_id, response_timeout)


def parse_bool(value: str) -> bool:
    """Parse a boolean command-line or environment value."""
    normalized: str = value.strip().lower()
    if normalized in ("1", "true", "yes", "on"):
        return True
    if normalized in ("0", "false", "no", "off"):
        return False
    raise argparse.ArgumentTypeError(
        "expected a boolean value (true/false, yes/no, on/off, or 1/0)"
    )


def run(
    port_name: str,
    osc_host: str,
    osc_port: int,
    response_timeout: float,
    send_osc: bool = True,
) -> None:
    """Continuously poll every board and optionally emit per-sensor OSC deltas."""
    wall = WallState()

    with SimpleUDPClient(osc_host, osc_port) as osc_client, serial.Serial(
        port=port_name,
        baudrate=UART_BAUDRATE,
        timeout=0.01,
        write_timeout=1,
    ) as port:
        print(f"Polling {port_name} at {UART_BAUDRATE} baud;")
        if send_osc:
            print(f"Sending OSC deltas to {osc_host}:{osc_port}")
        else:
            print("OSC sending disabled")

        while True:
            for board_id in range(BOARD_COUNT):
                response = poll_board(port, board_id, response_timeout)
                if response is None:
                    print(
                        f"No valid response from board {board_id}",
                        file=sys.stderr,
                    )
                    continue

                sequence, local_states = response
                first_update: bool = not wall.initialized_boards[board_id]
                changes: List[Tuple[int, bool]] = wall.update_board(
                    board_id, local_states
                )
                for sensor_index, touched in changes:
                    value = 1 if touched else 0
                    if send_osc:
                        osc_client.send_message(  # type: ignore[no-untyped-call]
                            f"/touch/{sensor_index}",
                            value,
                        )
                    else:
                        print(f"/touch/{sensor_index} {value}")

                if first_update:
                    print(f"Board {board_id} initialized (sequence {sequence})")


def build_parser() -> argparse.ArgumentParser:
    """Create the command-line parser, using environment variables as defaults."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--port",
        default=os.getenv("TOUCH_WALL_PORT"),
        help="USB serial port, e.g. COM3 or /dev/ttyUSB0",
    )
    parser.add_argument(
        "--baudrate",
        type=int,
        default=os.getenv("TOUCH_WALL_BAUDRATE", str(UART_BAUDRATE)),
    )
    parser.add_argument(
        "--osc-host", default=os.getenv("TOUCH_WALL_OSC_HOST", "127.0.0.1")
    )
    parser.add_argument(
        "--osc-port",
        type=int,
        default=os.getenv("TOUCH_WALL_OSC_PORT", "9000"),
    )
    parser.add_argument(
        "--response-timeout",
        type=float,
        default=os.getenv("TOUCH_WALL_RESPONSE_TIMEOUT", "0.25"),
        help="Seconds to wait for each board response",
    )
    parser.add_argument(
        "--send-osc",
        type=parse_bool,
        default=os.getenv("TOUCH_WALL_SEND_OSC", "true"),
        metavar="BOOL",
        help="Send OSC messages (true/false; default: true)",
    )
    return parser


def main() -> None:
    """Parse configuration and start the wall receiver loop."""
    load_dotenv()
    parser = build_parser()
    args = parser.parse_args()

    if args.port is None:
        parser.error("provide --port or set TOUCH_WALL_PORT in .env")
    if args.baudrate != UART_BAUDRATE:
        parser.error("baudrate must match the node firmware (115200)")
    if not 0 < args.osc_port <= 65535:
        parser.error("osc-port must be between 1 and 65535")
    if args.response_timeout <= 0:
        parser.error("response-timeout must be greater than zero")

    run(
        args.port,
        args.osc_host,
        args.osc_port,
        args.response_timeout,
        args.send_osc,
    )


if __name__ == "__main__":
    main()
