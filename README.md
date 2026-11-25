# Touch Wall

## Running unit tests

The unit tests run on a regular computer (Python 3.14 or newer) and do not
require a CircuitPython board. The receiver tests use a fake serial port. The
node tests use fake pins and UART, and cover protocol and touch-state logic;
they do not verify electrical connections, CircuitPython drivers, or the
physical RS-485 bus.

From the repository root, sync the project dependencies (including the `dev`
group containing pytest) and run the tests:

```sh
uv sync
uv run pytest
```

## Running the wall touch receiver

Create a local `.env` file from the example and set the serial port for your
system:

```sh
cp .env.example .env
```

The receiver reads `TOUCH_WALL_PORT`, `TOUCH_WALL_BAUDRATE`,
`TOUCH_WALL_OSC_HOST`, `TOUCH_WALL_OSC_PORT`, and
`TOUCH_WALL_RESPONSE_TIMEOUT` from `.env`. It polls every board.
`TOUCH_WALL_SEND_OSC` defaults to `true`; set it to `false` to disable OSC
messages while testing locally. Command-line options override these values.
The baud rate must remain `115200` to match the node firmware.

Run the receiver from the repository root:

```sh
uv run python src/touch_wall/wall_touch_receiver.py
```

The port can also be supplied directly with `--port`.

## Configuring the CircuitPython node

The node reads `BOARD_ID`, `SENSOR_COUNT`, and `MAX_TOUCH_DELTA` from the
microcontroller's root-level `settings.toml`. The MPR121 driver reads
`MPR121_CONFIG1` and `MPR121_CONFIG2` from the same file. Values are read as
strings and converted to integers; the MPR121 configuration values accept
decimal or `0x`-prefixed hexadecimal.

For example:

```toml
BOARD_ID = "0"
SENSOR_COUNT = "24"
MAX_TOUCH_DELTA = "60"
MPR121_CONFIG1 = "0x30"
MPR121_CONFIG2 = "0x3A"
```

These are the defaults, so only settings that need to differ must be added.
The receiver continues to read its settings from the computer's `.env` file.

The node's hardware modules are imported inside `main()`, so its reusable logic
can be imported by desktop tests. On the board, deploy `wall_touch_node.py` as
the CircuitPython entry point (typically `code.py`) and deploy
`mpr121_custom.py` alongside it (or in `lib/`). Install the
`adafruit_bus_device` CircuitPython library with Circup from the repository
root:

```sh
uv run circup install adafruit_bus_device
```

These files initialize the hardware and start the polling loop.