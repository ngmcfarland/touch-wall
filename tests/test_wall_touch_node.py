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
        node.select_pins = [FakePin() for _ in range(4)]
        node.rs485_direction = FakePin()
        node.uart = FakeUART()
        node.sequence_number = 0
        node.touch_states = [False] * node.SENSOR_COUNT
        node.baseline_values = [100] * node.SENSOR_COUNT
        node.request_buffer.clear()

    def test_crc_and_sensor_packing(self):
        self.assertEqual(node.crc8(b"123456789"), 0xF4)
        states = [False] * node.SENSOR_COUNT
        states[0] = True
        states[7] = True
        states[8] = True
        states[-1] = True

        payload = node.pack_states(states)
        self.assertEqual(payload[0], 0b10000001)
        self.assertEqual(payload[1], 0b00000001)
        self.assertEqual(payload[-1], 0b10000000)

    def test_mux_channel_selects_address_bits(self):
        node.set_mux_channel(0b1010)

        self.assertEqual(
            [pin.value for pin in node.select_pins], [False, True, False, True]
        )

    def test_handle_loop_reads_both_muxes_and_services_uart_per_channel(self):
        node.touch_inputs = (Mock(raw_value=120), Mock(raw_value=90))
        update = Mock()
        with (
            patch.object(node, "set_mux_channel") as set_channel,
            patch.object(node, "update_touch_state_for_index", update),
            patch.object(node, "service_uart") as service_uart,
            patch.object(node.time, "sleep") as sleep,
        ):
            node.handle_loop()

        self.assertEqual(
            set_channel.call_args_list,
            [call(channel) for channel in range(node.SENSORS_PER_MUX)],
        )
        self.assertEqual(
            update.call_args_list,
            [
                expected_call
                for channel in range(node.SENSORS_PER_MUX)
                for expected_call in (
                    call(raw_value=120, idx=channel),
                    call(raw_value=90, idx=channel + node.SENSORS_PER_MUX),
                )
            ],
        )
        self.assertEqual(service_uart.call_count, node.SENSORS_PER_MUX)
        self.assertEqual(sleep.call_count, node.SENSORS_PER_MUX)
        sleep.assert_called_with(node.MUX_SETTLE_SECONDS)

    def test_touch_thresholds_hysteresis_and_baseline_adaptation(self):
        node.baseline_values = [100] * node.SENSOR_COUNT

        node.update_touch_state_for_index(
            raw_value=100 * node.TOUCH_ON_RATIO,
            idx=0,
        )
        self.assertTrue(node.touch_states[0])

        node.update_touch_state_for_index(
            raw_value=100 * node.TOUCH_OFF_RATIO,
            idx=0,
        )
        self.assertTrue(node.touch_states[0])
        self.assertEqual(node.baseline_values[0], 100)

        node.update_touch_state_for_index(
            raw_value=100 * node.TOUCH_OFF_RATIO - 1,
            idx=0,
        )
        self.assertFalse(node.touch_states[0])
        self.assertEqual(node.baseline_values[0], 100)

        node.update_touch_state_for_index(
            raw_value=90,
            idx=1,
        )
        self.assertFalse(node.touch_states[1])
        self.assertEqual(node.baseline_values[1], 99)

    def test_baseline_never_falls_below_one(self):
        node.baseline_values = [1] * node.SENSOR_COUNT
        node.update_touch_state_for_index(0, 0)

        self.assertEqual(node.baseline_values[0], 1)

    def test_send_state_frame_and_sequence_wrap(self):
        node.sequence_number = 255
        node.touch_states[0] = True
        node.send_state()

        frame = node.uart.writes[0]
        body = frame[1:-2]
        self.assertEqual(frame[0], node.SOF)
        self.assertEqual(
            body[:4],
            bytes((node.STATE_COMMAND, node.BOARD_ID, node.SENSOR_COUNT, 255)),
        )
        self.assertEqual(body[4], 1)
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
