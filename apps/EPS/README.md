See <u>Files</u> for function documentation.

# EPS — Electrical Power System Firmware

EPS is the UT-CORE CubeSat's power distribution board, running Zephyr and controlling 12 switched power loads and 4 motor 12V rails, while monitoring power draw across 21 channels via 7 INA3221 power monitor chips.

Unlike CDH, EPS currently runs as a single loop rather than multiple threads: it toggles a status LED, samples every INA3221 channel, and prints voltage/current/power telemetry to the console on a fixed interval.

Load and motor switching functions exist in the codebase but aren't currently wired to any trigger — no CAN bus, command handler, or other caller invokes them in this build.

On boot it configures every load and motor GPIO as an output, checking each for readiness first, then enters the main loop.

## Hardware

- MCU: STM32, Zephyr RTOS
- Loads: 12 switched power loads (active-high GPIO enable)
- Motors: 4 motor 12V power rails
- Sensors: 7 INA3221 power monitor chips, 3 channels each (21 total)

## Architecture

EPS is a single-loop design, not multi-threaded like CDH:

| Step | Role |
|---|---|
| `init_gpios` | Configures all load and motor GPIOs as outputs at boot |
| `read_power_monitors` | Samples voltage/current/power on all 21 INA3221 channels |
| `print_telemetry` | Prints the latest readings to the console |
| Main loop | Toggles the status LED and repeats the read/print cycle on a fixed interval |

There's also a `demo.c` alongside `main.c` — functionally near-identical, likely a standalone bench-test build for verifying GPIO and INA3221 wiring independent of the rest of the satellite's CAN network.
