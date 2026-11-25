import unittest
from unittest.mock import Mock, call, patch

from touch_wall import wall_touch_node as node


class FakePin:
    def __init__(self):
        self.value = None
        self.direction = None


class FakeUART:
    def __init__(self, incoming=b""):
        self.incoming = bytearray(incoming)
        self.writes = []

    @property
    def in_waiting(self):
        return len(self.incoming)

    def read(self, size=1):
        chunk = self.incoming[:size]
        del self.incoming[:size]
        return bytes(chunk)

    def write(self, data):
        self.writes.append(bytes(data))
        return len(data)


class NodeTests(unittest.TestCase):
    def setUp(self):
        node.rs485_direction = FakePin()
        node.uart = FakeUART()
        node.sequence_number = 0
        node.sensor_values = [0] * node.SENSOR_COUNT
        node.request_buffer.clear()

    def test_normalize_raw_values_clamps_and_scales_from_baseline(self):
        baseline = 1000
        self.assertEqual(node.normalize_raw_value(baseline + 2, baseline), 0)
        self.assertEqual(node.normalize_raw_value(baseline, baseline), 0)
        self.assertEqual(node.normalize_raw_value(baseline - 30, baseline), 8)
        self.assertEqual(node.normalize_raw_value(baseline - 60, baseline), 15)
        self.assertEqual(node.normalize_raw_value(baseline - 100, baseline), 15)

    def test_crc_and_sensor_value_packing(self):
        self.assertEqual(node.crc8(b"123456789"), 0xF4)
        values = [0] * node.SENSOR_COUNT
        values[0] = 1
        values[1] = 15
        values[2] = 7
        values[-1] = 8

        payload = node.pack_values(values)
        self.assertEqual(len(payload), 12)
        self.assertEqual(payload[0], 0xF1)
        self.assertEqual(payload[1], 0x07)
        self.assertEqual(payload[-1], 0x80)
        with self.assertRaises(ValueError):
            node.pack_values([16] + [0] * (node.SENSOR_COUNT - 1))

    def test_handle_loop_reads_both_mpr121_boards_and_services_uart(self):
        board_a = Mock()
        board_a.filtered_data.side_effect = lambda pin: 100 - pin * 5
        board_a.baseline_data.return_value = 100
        board_b = Mock()
        board_b.filtered_data.side_effect = lambda pin: 100 - pin * 10
        board_b.baseline_data.return_value = 100
        node.touch_inputs = (board_a, board_b)

        with patch.object(node, "service_uart") as service_uart:
            node.handle_loop()

        self.assertEqual(
            board_a.filtered_data.call_args_list,
            [call(pin=pin) for pin in range(12)],
        )
        self.assertEqual(
            board_a.baseline_data.call_args_list,
            [call(pin=pin) for pin in range(12)],
        )
        self.assertEqual(
            board_b.filtered_data.call_args_list,
            [call(pin=pin) for pin in range(12)],
        )
        self.assertEqual(
            board_b.baseline_data.call_args_list,
            [call(pin=pin) for pin in range(12)],
        )
        self.assertEqual(
            node.sensor_values[:12], [0, 1, 3, 4, 5, 6, 8, 9, 10, 11, 13, 14]
        )
        self.assertEqual(
            node.sensor_values[12:], [0, 3, 5, 8, 10, 13, 15, 15, 15, 15, 15, 15]
        )
        self.assertEqual(service_uart.call_count, node.SENSOR_COUNT)

    def test_send_state_frame_and_sequence_wrap(self):
        node.sequence_number = 255
        node.sensor_values[0] = 15
        node.send_state()

        frame = node.uart.writes[0]
        body = frame[1:-2]
        self.assertEqual(frame[0], node.SOF)
        self.assertEqual(
            body[:4],
            bytes((node.STATE_COMMAND, node.BOARD_ID, node.SENSOR_COUNT, 255)),
        )
        self.assertEqual(len(frame), 19)
        self.assertEqual(body[4], 0x0F)
        self.assertEqual(frame[-2], node.crc8(body))
        self.assertEqual(frame[-1], node.EOF_MARKER)
        self.assertEqual(node.sequence_number, 0)
        self.assertFalse(node.rs485_direction.value)

    def test_request_parser_handles_partial_invalid_and_other_board_requests(self):
        other_request = bytes(
            (
                node.SOF,
                node.POLL_COMMAND,
                node.BOARD_ID + 1,
                node.crc8((node.POLL_COMMAND, node.BOARD_ID + 1)),
            )
        )
        request = bytes(
            (
                node.SOF,
                node.POLL_COMMAND,
                node.BOARD_ID,
                node.crc8((node.POLL_COMMAND, node.BOARD_ID)),
            )
        )
        send_state = Mock()
        with patch.object(node, "send_state", send_state):
            for value in b"\x00\xA5\x01\x00\x00" + other_request + request:
                node.handle_request_byte(value)

        send_state.assert_called_once_with()
        self.assertEqual(node.request_buffer, bytearray())

    def test_service_uart_drains_all_received_bytes(self):
        request = bytes(
            (
                node.SOF,
                node.POLL_COMMAND,
                node.BOARD_ID,
                node.crc8((node.POLL_COMMAND, node.BOARD_ID)),
            )
        )
        node.uart = FakeUART(request)
        with patch.object(node, "handle_request_byte") as handle:
            node.service_uart()

        self.assertEqual(handle.call_count, 4)


if __name__ == "__main__":
    unittest.main()
