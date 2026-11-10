import os
import unittest
from unittest.mock import Mock, patch

from touch_wall import wall_touch_receiver as receiver


class FakeSerial:
    def __init__(self, incoming=b"", write_result=None):
        self.incoming = bytearray(incoming)
        self.writes = []
        self.write_result = write_result

    def read(self, size=1):
        chunk = self.incoming[:size]
        del self.incoming[:size]
        return bytes(chunk)

    def write(self, data):
        self.writes.append(data)
        if self.write_result is not None:
            return self.write_result
        return len(data)


def state_frame(
    board_id,
    sequence,
    payload,
    *,
    checksum=None,
    end=receiver.EOF_MARKER,
):
    body = bytes(
        (
            receiver.STATE_COMMAND,
            board_id,
            receiver.expected_sensor_count(board_id),
            sequence,
        )
    ) + payload
    if checksum is None:
        checksum = receiver.crc8(body)
    return bytes((receiver.SOF,)) + body + bytes((checksum, end))


class ReceiverTests(unittest.TestCase):
    def test_parser_uses_environment_defaults_and_cli_overrides(self):
        config = {
            "TOUCH_WALL_PORT": "/dev/ttyUSB1",
            "TOUCH_WALL_BAUDRATE": "115200",
            "TOUCH_WALL_OSC_HOST": "192.168.1.10",
            "TOUCH_WALL_OSC_PORT": "9001",
            "TOUCH_WALL_RESPONSE_TIMEOUT": "0.5",
            "TOUCH_WALL_SEND_OSC": "false",
        }
        with patch.dict(os.environ, config):
            args = receiver.build_parser().parse_args([])
            self.assertEqual(args.port, "/dev/ttyUSB1")
            self.assertEqual(args.baudrate, 115200)
            self.assertEqual(args.osc_host, "192.168.1.10")
            self.assertEqual(args.osc_port, 9001)
            self.assertEqual(args.response_timeout, 0.5)
            self.assertFalse(args.send_osc)

            args = receiver.build_parser().parse_args(
                [
                    "--port",
                    "/dev/ttyUSB2",
                    "--osc-port",
                    "9002",
                    "--send-osc",
                    "yes",
                ]
            )
            self.assertEqual(args.port, "/dev/ttyUSB2")
            self.assertEqual(args.osc_port, 9002)
            self.assertTrue(args.send_osc)

    def test_parser_defaults_osc_sending(self):
        with patch.dict(os.environ, {}, clear=True):
            args = receiver.build_parser().parse_args([])
        self.assertTrue(args.send_osc)

    def test_parser_rejects_invalid_send_osc_value(self):
        with self.assertRaises(SystemExit):
            receiver.build_parser().parse_args(["--send-osc", "sometimes"])

    def test_run_polls_every_board_without_sending_osc_when_disabled(self):
        osc_client = Mock()
        osc_client.__enter__ = Mock(return_value=osc_client)
        osc_client.__exit__ = Mock(return_value=None)
        port = Mock()
        port.__enter__ = Mock(return_value=port)
        port.__exit__ = Mock(return_value=None)

        with (
            patch.object(receiver, "SimpleUDPClient", return_value=osc_client),
            patch.object(receiver.serial, "Serial", return_value=port),
            patch.object(receiver, "BOARD_COUNT", 2),
            patch.object(
                receiver,
                "poll_board",
                side_effect=[(1, [True] * 24), (1, [True] * 24), KeyboardInterrupt],
            ) as poll_board,
        ):
            with self.assertRaises(KeyboardInterrupt):
                receiver.run("/dev/ttyUSB0", "127.0.0.1", 9000, 0.1, False)

        self.assertEqual(
            [call.args[1] for call in poll_board.call_args_list],
            [0, 1, 0],
        )
        osc_client.send_message.assert_not_called()

    def test_crc_and_poll_request(self):
        self.assertEqual(receiver.crc8(b"123456789"), 0xF4)
        self.assertEqual(
            receiver.pack_poll_request(5),
            bytes((receiver.SOF, receiver.POLL_COMMAND, 5, receiver.crc8((1, 5)))),
        )

    def test_expected_counts_and_global_offsets(self):
        self.assertEqual(receiver.expected_sensor_count(0), 24)
        self.assertEqual(receiver.expected_sensor_count(22), 24)
        self.assertEqual(receiver.expected_sensor_count(23), 16)
        self.assertEqual(receiver.first_global_sensor(0), 0)
        self.assertEqual(receiver.first_global_sensor(22), 528)
        self.assertEqual(receiver.first_global_sensor(23), 552)
        with self.assertRaises(ValueError):
            receiver.expected_sensor_count(24)

    def test_unpack_payload_uses_lsb_first_order(self):
        self.assertEqual(
            receiver.unpack_state_payload(bytes((0b10000001, 0b00000010)), 10),
            [True, False, False, False, False, False, False, True, False, True],
        )

    def test_reads_response_after_noise_and_invalid_frame(self):
        payload = bytes((0b00000101, 0b00000010, 0))
        invalid = state_frame(3, 10, payload, checksum=0)
        valid = state_frame(3, 11, payload)
        port = FakeSerial(b"\x00" + invalid + valid)

        self.assertEqual(
            receiver.read_state_response(port, 3, 0.1),
            (11, receiver.unpack_state_payload(payload, 24)),
        )

    def test_rejects_wrong_board_and_bad_terminator(self):
        payload = bytes(3)
        wrong_board = state_frame(4, 1, payload)
        bad_terminator = state_frame(3, 2, payload, end=0)
        port = FakeSerial(wrong_board + bad_terminator)

        self.assertIsNone(receiver.read_state_response(port, 3, 0.001))

    def test_poll_board_writes_request_and_returns_response(self):
        payload = bytes(3)
        port = FakeSerial(state_frame(2, 7, payload))

        self.assertEqual(receiver.poll_board(port, 2, 0.1), (7, [False] * 24))
        self.assertEqual(port.writes, [receiver.pack_poll_request(2)])

    def test_poll_board_reports_incomplete_write(self):
        port = FakeSerial(write_result=1)

        with self.assertRaises(IOError):
            receiver.poll_board(port, 0, 0.1)

    def test_wall_state_initializes_board_and_reports_only_changes(self):
        wall = receiver.WallState()
        self.assertEqual(
            wall.update_board(23, [False] * 16),
            [(index, False) for index in range(552, 568)],
        )
        self.assertEqual(wall.update_board(23, [False] * 16), [])

        local_states = [False] * 24
        local_states[0] = True
        changes = wall.update_board(22, local_states)
        self.assertEqual(changes[0], (528, True))
        self.assertEqual(changes[1:], [(index, False) for index in range(529, 552)])
        self.assertTrue(wall.states[528])
        with self.assertRaises(ValueError):
            wall.update_board(23, [False] * 24)


if __name__ == "__main__":
    unittest.main()
