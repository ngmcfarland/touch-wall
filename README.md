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

The node's hardware modules are imported inside `main()`, so its reusable logic
can be imported by desktop tests. On the board, deploy `wall_touch_node.py` as
the CircuitPython entry point (typically `code.py`) to initialize the hardware
and start the polling loop.