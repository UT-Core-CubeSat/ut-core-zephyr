See <u>Files</u> for function documentation.

# CDH — Command and Data Handling Firmware

CDH is the UT-CORE CubeSat's central board, running Zephyr on an STM32U5 and coordinating with the rest of the satellite (EPS, COMMS, ADCS, motor, GNSS, star tracker, solar) over a 500 kbit/s FDCAN bus.

It's built as seven priority-ordered threads: a watchdog kicker at the top so the board can't hang, a CAN RX thread that drains incoming frames into a queue, and a processing thread that decodes and dispatches those frames by message class.

Lower-priority threads broadcast heartbeats, poll GNSS position, read onboard temperature sensors and track other nodes' liveness, and assemble telemetry for downlink.

On boot it verifies its own MCU identity, brings up the CAN transceiver and bus, then runs all seven threads concurrently, driving the satellite's command routing and state-of-health monitoring for the rest of the mission.

## Hardware

- MCU: STM32U5, Zephyr RTOS
- Bus: FDCAN1, 500 kbit/s, 29-bit extended IDs
- Sensors: up to 6 I2C temperature sensors (TMP1xx-family, 0x48–0x4D)
- Transceiver: TCAN3403

## Architecture

Seven threads, priority order (highest to lowest):

| Priority | Thread | Role |
|---|---|---|
| 0 | `watchdog_thread` | Kicks hardware watchdog; must never starve |
| 1 | `can_rx_thread` | Drains hardware FIFO into SW queue |
| 2 | `can_process_thread` | Decodes and dispatches frames |
| 3 | `scheduler_thread` | Heartbeat broadcast + future mission scheduling |
| 4 | `gnss_thread` | Requests and stores GNSS position data |
| 5 | `soh_thread` | Reads temps, checks node-alive timeouts |
| 6 | `telemetry_thread` | Assembles and downlinks telemetry (placeholder) |

On boot, the firmware verifies MCU identity, brings up the CAN bus, then hands off to the threads above, which coordinate over CAN with the other satellite subsystems (EPS, COMMS, ADCS, MOTOR, GNSS, STAR, SOLAR).
