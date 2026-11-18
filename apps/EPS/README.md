See <u>Files</u> above for function documentation.

Back to [Contents](@ref index)

# EPS — Electrical Power System Firmware

EPS is the UT-CORE CubeSat's power distribution board, running Zephyr and controlling 12 switched power loads and 4 motor 12V rails, while monitoring power draw across 21 channels via 7 INA3221 power monitor chips.

EPS currently runs two threads: the main thread brings up GPIOs, sensors, and the CAN bus, then loops forever sending a heartbeat, toggling a status LED, and sampling/printing power telemetry on fixed intervals. A separate can_rx_thread listens for incoming CAN Commands. 

Load switching is wired to CAN: an OP_SET_PWR_STATE command received over CAN calls enable_load()/disable_load() directly, with a status reply sent back. Motor switching (enable_motor()/disable_motor()) exists but isn't yet wired to any CAN command.

On boot it configures every load and motor GPIO as an output, checking each for readiness first, then enters the main loop.

## Hardware

- MCU: STM32, Zephyr RTOS
- Loads: 12 switched power loads (active-high GPIO enable)
- Motors: 4 motor 12V power rails
- Sensors: 7 INA3221 power monitor chips, 3 channels each (21 total)

## Architecture

EPS is a multi-threaded design, simpler than CDH's 7-thread design:

| Thread | Role |
|---|---|
| `main` | Initializes GPIOs, INA3221 sensors, and the CAN bus; then loops forever, sending a heartbeat every `HEARTBEAT_INTERVAL_MS` and sampling/printing telemetry every `TELEMETRY_INTERVAL_MS` |
| `can_rx_thread` | Waits for incoming CAN frames; on `OP_SET_PWR_STATE`, calls `enable_load()`/`disable_load()` and replies with an `OP_RESP_SET_PWR_STATE` status frame|
