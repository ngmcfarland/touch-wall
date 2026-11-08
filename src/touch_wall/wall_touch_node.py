import time


# Assign IDs 0-23, one per PCB. By default, board 23 is the 16-sensor board.
BOARD_ID = 0
SENSOR_COUNT = 16 if BOARD_ID == 23 else 24

# Tune these values after measuring raw_value on the installed wall.
TOUCH_ON_RATIO = 1.10
TOUCH_OFF_RATIO = 1.06
BASELINE_ALPHA = 0.01
MUX_SETTLE_SECONDS = 0.001

UART_BAUDRATE = 115200
# Confirm the breakout's RTS polarity; true means GP2 high while transmitting.
RS485_TX_ENABLE_LEVEL = True
UART_BITS_PER_BYTE = 10  # 8 data bits, start bit, and stop bit
TX_TURNAROUND_MARGIN_SECONDS = 0.001

SOF = 0xA5
EOF_MARKER = 0x5A
POLL_COMMAND = 0x01
STATE_COMMAND = 0x81

SENSORS_PER_MUX = SENSOR_COUNT // 2
PAYLOAD_SIZE = (SENSOR_COUNT + 7) // 8

if not 0 <= BOARD_ID <= 23:
    raise ValueError("BOARD_ID must be between 0 and 23")
if SENSOR_COUNT not in (16, 24) or SENSOR_COUNT % 2:
    raise ValueError("SENSOR_COUNT must be 16 or 24")
if not 0 < TOUCH_OFF_RATIO <= TOUCH_ON_RATIO:
    raise ValueError("Touch ratios must satisfy 0 < OFF <= ON")


def set_mux_channel(channel):
    select_pins[0].value = bool(channel & 0x01)
    select_pins[1].value = bool(channel & 0x02)
    select_pins[2].value = bool(channel & 0x04)
    select_pins[3].value = bool(channel & 0x08)


def read_raw_values():
    values = [0] * SENSOR_COUNT
    for mux_index, touch_input in enumerate(touch_inputs):
        first_sensor = mux_index * SENSORS_PER_MUX
        for channel in range(SENSORS_PER_MUX):
            set_mux_channel(channel)
            time.sleep(MUX_SETTLE_SECONDS)
            values[first_sensor + channel] = touch_input.raw_value
    return values


def crc8(data):
    crc = 0
    for value in data:
        crc ^= value
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def pack_states(states):
    payload = bytearray(PAYLOAD_SIZE)
    for sensor_index, touched in enumerate(states):
        if touched:
            payload[sensor_index // 8] |= 1 << (sensor_index % 8)
    return payload


def send_state():
    global sequence_number

    body = bytearray((STATE_COMMAND, BOARD_ID, SENSOR_COUNT, sequence_number))
    body.extend(pack_states(touch_states))

    frame = bytearray((SOF,))
    frame.extend(body)
    frame.append(crc8(body))
    frame.append(EOF_MARKER)

    rs485_direction.value = RS485_TX_ENABLE_LEVEL
    uart.write(frame)
    time.sleep(
        len(frame) * UART_BITS_PER_BYTE / UART_BAUDRATE
        + TX_TURNAROUND_MARGIN_SECONDS
    )
    rs485_direction.value = not RS485_TX_ENABLE_LEVEL
    sequence_number = (sequence_number + 1) & 0xFF


request_buffer = bytearray()


def handle_request_byte(value):
    request_buffer.append(value)
    while len(request_buffer) >= 4:
        if request_buffer[0] != SOF:
            del request_buffer[0]
            continue

        request = request_buffer[:4]
        if request[1] != POLL_COMMAND or request[3] != crc8(request[1:3]):
            del request_buffer[0]
            continue

        del request_buffer[:4]
        if request[2] == BOARD_ID:
            send_state()


def service_uart():
    while uart.in_waiting:
        received = uart.read(1)
        if received:
            handle_request_byte(received[0])


def update_touch_states(raw_values):
    for sensor_index, raw_value in enumerate(raw_values):
        baseline = baseline_values[sensor_index]
        was_touched = touch_states[sensor_index]

        if was_touched:
            touched = raw_value / baseline >= TOUCH_OFF_RATIO
        else:
            touched = raw_value / baseline >= TOUCH_ON_RATIO

        touch_states[sensor_index] = touched
        if not touched:
            baseline_values[sensor_index] = max(
                1,
                int(baseline + (raw_value - baseline) * BASELINE_ALPHA),
            )


def main():
    import board
    import busio
    import touchio
    from digitalio import DigitalInOut, Direction

    global baseline_values
    global rs485_direction
    global select_pins
    global sequence_number
    global touch_inputs
    global touch_states
    global uart

    # Both multiplexers share their address/select lines.
    select_pins = [
        DigitalInOut(board.GP6),
        DigitalInOut(board.GP7),
        DigitalInOut(board.GP8),
        DigitalInOut(board.GP9),
    ]
    for pin in select_pins:
        pin.direction = Direction.OUTPUT

    rs485_direction = DigitalInOut(board.GP2)
    rs485_direction.direction = Direction.OUTPUT
    rs485_direction.value = not RS485_TX_ENABLE_LEVEL

    touch_a = touchio.TouchIn(board.GP10, pull=touchio.Pull.UP)
    touch_b = touchio.TouchIn(board.GP11, pull=touchio.Pull.UP)
    touch_inputs = (touch_a, touch_b)

    uart = busio.UART(
        board.GP0,
        board.GP1,
        baudrate=UART_BAUDRATE,
        timeout=0,
        receiver_buffer_size=64,
    )

    # Request:  A5 01 board_id crc8(command, board_id)
    # Response: A5 81 board_id sensor_count sequence payload... crc8(body) 5A
    # The payload is LSB-first: bit 0 is local sensor 0. Sensors 0..N/2-1
    # are mux A; sensors N/2..N-1 are mux B. Only the host polls; nodes never
    # transmit spontaneously, which prevents collisions on the shared RS-485 bus.
    sequence_number = 0
    touch_states = [False] * SENSOR_COUNT
    # Start with every pad untouched so each sensor gets an unpressed baseline.
    baseline_values = [max(value, 1) for value in read_raw_values()]

    while True:
        raw_values = read_raw_values()
        update_touch_states(raw_values)
        service_uart()


if __name__ == "__main__":
    main()
