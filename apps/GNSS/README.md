See <u>Files</u> above for function documentation.

Back to [Contents](@ref index)

# GNSS — Orion B16 GNSS Driver + CAN Telemetry

GNSS is the UT-CORE CubeSat's position board, running Zephyr and driving an Orion B16 GNSS receiver over UART, reporting fix data to CDH over a CAN bus.

A background thread drains incoming CAN frames and dispatches them by message class, handling heartbeat messages and commands from CDH (position queries, update-rate changes). The main loop broadcasts its own heartbeat, sends periodic state-of-health frames (fix mode, satellite count, DOP), and reports position telemetry as scaled lat/lon values, alongside driver stats logged to console.

A nav-update callback registered with the Orion driver fires whenever a new fix is available, logging fix mode, position, altitude, and satellite count.

## Hardware

- Receiver: Orion B16 GNSS module, connected via UART
- Bus: CAN (FDCAN1), 500 kbit/s, 29-bit extended IDs
- Transceiver: TCAN3403
- Constellations supported: GPS, GLONASS, Galileo, BeiDou

## Architecture

| Component | Role |
|---|---|
| `orion_driver_*` | Orion B16 GNSS driver: init, start, nav data, config, stats |
| `on_nav_update` | Callback fired on every new nav fix from the driver |
| `can_rx_thread` | Drains incoming CAN frames, decodes and dispatches by message class |
| Main loop | Broadcasts heartbeat, state-of-health, and position telemetry on independent fixed intervals |

On boot, the firmware verifies GPIOA is ready, initializes and starts the GNSS driver, applies default CubeSat GNSS config, then wakes the CAN transceiver and brings up the bus before entering the main loop.
