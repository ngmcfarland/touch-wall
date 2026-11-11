import time


BOARD_ID = 0
SENSOR_COUNT = 24  # Half on one MUX and half on the other

# Tune these values after measuring raw_value on the installed wall.
TOUCH_ON_RATIO = 1.5
TOUCH_OFF_RATIO = 1.1
BASELINE_ALPHA = 0.01
MUX_SETTLE_SECONDS = 0.005

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
if SENSOR_COUNT < 2 or SENSOR_COUNT > 32 or SENSOR_COUNT % 2:
    raise ValueError("SENSOR_COUNT must be even and between 2 and 32")
if not 0 < TOUCH_OFF_RATIO <= TOUCH_ON_RATIO:
    raise ValueError("Touch ratios must satisfy 0 < OFF <= ON")


def set_mux_channel(channel):
    select_pins[0].value = bool(channel & 0x01)
    select_pins[1].value = bool(channel & 0x02)
    select_pins[2].value = bool(channel & 0x04)
    select_pins[3].value = bool(channel & 0x08)


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
    global request_buffer
    request_buffer.append(value)
    while len(request_buffer) >= 4:
        if request_buffer[0] != SOF:
            request_buffer = request_buffer[1:]
            continue

        request = request_buffer[:4]
        if request[1] != POLL_COMMAND or request[3] != crc8(request[1:3]):
            request_buffer = request_buffer[1:]
            continue

        # Slice the buffer to remove the processed 4-byte request
        request_buffer = request_buffer[4:]
        if request[2] == BOARD_ID:
            send_state()


def service_uart():
    while uart.in_waiting:
        received = uart.read(1)
        if received:
            handle_request_byte(received[0])


def update_touch_state_for_index(raw_value, idx):
    baseline = baseline_values[idx]
    was_touched = touch_states[idx]

    if was_touched:
        touched = raw_value / baseline >= TOUCH_OFF_RATIO
    else:
        touched = raw_value / baseline >= TOUCH_ON_RATIO

    touch_states[idx] = touched
    if not touched:
        baseline_values[idx] = max(
            1,
            int(baseline + (raw_value - baseline) * BASELINE_ALPHA),
        )


def handle_loop() -> None:
    for channel in range(SENSORS_PER_MUX):
        set_mux_channel(channel)
        time.sleep(MUX_SETTLE_SECONDS)
        # Read and update the touch states for both MUX inputs
        update_touch_state_for_index(
            raw_value=touch_inputs[0].raw_value,
            idx=channel,
        )
        update_touch_state_for_index(
            raw_value=touch_inputs[1].raw_value,
            idx=channel + SENSORS_PER_MUX,
        )
        # Check for RS-485 message
        service_uart()


def main():
    import board
    import touchio
    from digitalio import DigitalInOut, Direction, Pull
    import adafruit_pio_uart

    global baseline_values
    global rs485_direction
    global select_pins
    global sequence_number
    global touch_inputs
    global touch_states
    global uart

    led = DigitalInOut(board.LED)
    led.direction = Direction.OUTPUT

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

    touch_a = touchio.TouchIn(board.GP10, pull=Pull.UP)
    touch_b = touchio.TouchIn(board.GP11, pull=Pull.UP)
    touch_inputs = (touch_a, touch_b)

    uart = adafruit_pio_uart.UART(
        tx=board.GP1,
        rx=board.GP0,
        baudrate=UART_BAUDRATE,
        timeout=0,
    )

    # Request:  A5 01 board_id crc8(command, board_id)
    # Response: A5 81 board_id sensor_count sequence payload... crc8(body) 5A
    # The payload is LSB-first: bit 0 is local sensor 0. Sensors 0..N/2-1
    # are mux A; sensors N/2..N-1 are mux B. Only the host polls; nodes never
    # transmit spontaneously, which prevents collisions on the shared RS-485 bus.
    sequence_number = 0
    touch_states = [False] * SENSOR_COUNT
    # Start with every pad untouched so each sensor gets an unpressed baseline.
    baseline_values = [0] * SENSOR_COUNT
    for channel in range(SENSORS_PER_MUX):
        set_mux_channel(channel)
        time.sleep(MUX_SETTLE_SECONDS * 2)  # Extra settle time for baseline
        baseline_values[channel] = max(touch_inputs[0].raw_value, 1)
        baseline_values[channel + SENSORS_PER_MUX] = max(touch_inputs[1].raw_value, 1)

    # Flash the LED to signal the board is ready
    led.value = True
    time.sleep(0.5)
    led.value = False

    while True:
        handle_loop()


if __name__ == "__main__":
    main()
