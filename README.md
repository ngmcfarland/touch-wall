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

The node's hardware modules are imported inside `main()`, so its reusable logic
can be imported by desktop tests. On the board, deploy `wall_touch_node.py` as
the CircuitPython entry point (typically `code.py`) to initialize the hardware
and start the polling loop.