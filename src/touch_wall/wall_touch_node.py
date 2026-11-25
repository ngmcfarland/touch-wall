import os
import time


BOARD_ID = int(os.getenv("BOARD_ID", "0"))
SENSOR_COUNT = int(os.getenv("SENSOR_COUNT", "24"))
MAX_TOUCH_DELTA = int(os.getenv("MAX_TOUCH_DELTA", "60"))

NORMALIZED_VALUE_MAX = 15

UART_BAUDRATE = 115200
# RS-485 breakout's RTS polarity: true means GP2 high while transmitting.
RS485_TX_ENABLE_LEVEL = True
UART_BITS_PER_BYTE = 10  # 8 data bits, start bit, and stop bit
TX_TURNAROUND_MARGIN_SECONDS = 0.001

SOF = 0xA5
EOF_MARKER = 0x5A
POLL_COMMAND = 0x01
STATE_COMMAND = 0x81

PAYLOAD_SIZE = (SENSOR_COUNT + 1) // 2

if not 0 <= BOARD_ID <= 23:
    raise ValueError("BOARD_ID must be between 0 and 23")
if SENSOR_COUNT < 1 or SENSOR_COUNT > 24:
    raise ValueError("SENSOR_COUNT must be between 1 and 24")
if MAX_TOUCH_DELTA < 1:
    raise ValueError("MAX_TOUCH_DELTA must be greater than 0")


def crc8(data: bytes | bytearray | tuple[int, ...]) -> int:
    """Compute a CRC-8/ATM checksum (polynomial 0x07, initial value 0)."""
    crc = 0
    for value in data:
        crc ^= value
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def normalize_raw_value(raw_value: int, baseline: int) -> int:
    """ Normalize the raw value based on the baseline recorded by the MPR121 board.

    When not touched, there is typically a jitter of +/- 2.
    When touched, the raw value drops below the baseline by some delta.
    Find the unsigned difference between the raw value and baseline
    and normalize that using the MAX_TOUCH_DELTA to be between 0 and
    the NORMALIZED_VALUE_MAX value.

    Args:
        raw_value: The current reading from the MPR121 pin
        baseline: The baseline recorded by the MPR121 on initialization

    Returns:
        A normalized value between 0 and NORMALIZED_VALUE_MAX
    """
    touch_delta = baseline - raw_value
    clamped_value = min(max(touch_delta, 0), MAX_TOUCH_DELTA)
    return (
        clamped_value * NORMALIZED_VALUE_MAX + MAX_TOUCH_DELTA // 2
    ) // MAX_TOUCH_DELTA


def pack_values(values: list[int]) -> bytearray:
    """ Pack the list of integers into a bytearray to be sent over RS-485. """
    payload = bytearray(PAYLOAD_SIZE)
    for sensor_index, value in enumerate(values):
        if not 0 <= value <= NORMALIZED_VALUE_MAX:
            raise ValueError(f"sensor values must be between 0 and {NORMALIZED_VALUE_MAX}")
        payload[sensor_index // 2] |= value << (4 * (sensor_index % 2))
    return payload


def send_state() -> None:
    """ Send the full state of all touch spots over the RS-485 bus. """
    global sequence_number

    body = bytearray((STATE_COMMAND, BOARD_ID, SENSOR_COUNT, sequence_number))
    body.extend(pack_values(sensor_values))

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


def handle_request_byte(value: int) -> None:
    """ Parse the incoming RS-485 message to determine if and how to respond.

    Args:
        value: The latest int value from the RS-485 stream
    """
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


def service_uart() -> None:
    """ Read from UART and handle the incoming values if found. """
    while uart.in_waiting:
        received: bytes | None = uart.read(1)
        if received:
            handle_request_byte(received[0])


def handle_loop() -> None:
    """ Loop through the channels and update each with new values while continuously checking for RS-485 messages. """
    for channel in range(SENSOR_COUNT):
        touch_input = touch_inputs[0] if channel < 12 else touch_inputs[1]
        # Read and update the normalized value from MPR121 board
        sensor_values[channel] = normalize_raw_value(
            raw_value=touch_input.filtered_data(pin=channel % 12),
            baseline=touch_input.baseline_data(pin=channel % 12),
        )
        # Check for RS-485 message
        service_uart()


def main():
    # Imports done here to allow for unit tests on computers
    from digitalio import DigitalInOut, Direction
    from mpr121_custom import MPR121
    import board
    import busio

    global rs485_direction
    global sequence_number
    global touch_inputs
    global sensor_values
    global uart

    led = DigitalInOut(board.LED)
    led.direction = Direction.OUTPUT

    rs485_direction = DigitalInOut(board.GP2)
    rs485_direction.direction = Direction.OUTPUT
    rs485_direction.value = not RS485_TX_ENABLE_LEVEL

    i2c = busio.I2C(board.GP5, board.GP4)
    mpr121_a = MPR121(i2c, 0x5A)  # Default address, no modifications to MPR121 breakout board
    mpr121_b = MPR121(i2c, 0x5B)  # Cut trace between ADDR and GND, connect ADDR to 3.3V

    touch_inputs = (mpr121_a, mpr121_b)

    uart = busio.UART(
        tx=board.GP0,
        rx=board.GP1,
        baudrate=UART_BAUDRATE,
        timeout=0,
    )

    # Request:  A5 01 board_id crc8(command, board_id)
    # Response: A5 81 board_id sensor_count sequence payload... crc8(body) 5A
    # The payload packs two values per byte, low nibble first. Sensors 0..11
    # are MPR121 A; sensors 12..23 are MPR121 B. Only the host polls; nodes
    # never transmit spontaneously, which prevents collisions on the RS-485 bus.
    sequence_number = 0
    sensor_values = [0] * SENSOR_COUNT

    # Flash the LED to signal the board is ready
    led.value = True
    time.sleep(0.5)
    led.value = False

    while True:
        handle_loop()


if __name__ == "__main__":
    main()
