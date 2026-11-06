See <u>Files</u> above for function documentation.

Back to [Contents](@ref index)

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

## Known Software Requirements - 9/23/26 (Cason)

- Manage CAN BUS Tx/Rx
    - **Complete** can_rx_thread, can_process_thread, can_dispatch, send_set_board_pwr, and tcan3403_wakeup are all functional.
- Compile SOH (State of Health) for subsystems
    - **Incomplete** soh_thread reads sensor data, but nothing else happens with said data. telemetry_thread should do that , but it's unimplemented currently.
- Handle data routing
    - **Complete** can_dispatch routes data by message class. can_process_thread filters by destination node/broadcast.
- Read Internal sensor data
    - **Incomplete** soh_thread does real I2C reads of the onboard temperature sensors via temp_telemetry_read_all. Temperature is covered, but no other internal sensor types are wired in.
- Determine Uplink/downlink schedule
    - **Incomplete** telemetry_thread is just a placeholder.
- Determine experiment pass time (determine when we want to take the photos)
    - **Incomplete** No code related to this exists.
- Based off GNSS data determine if we are where we need to be to take our photos, update mode etc.
    - **Incomplete** The gnss_thread only requests position. handle_telemetry parses and stores the fix into gnss_latest. Further decision logic needs to be added, as nothing evaluates that position or triggers any change.
- Tell subsystems what to do/when to do it
    - **Incomplete** send_set_board_pwr sends commands to EPS, but only when told by ground. Needs autonomous subsystem commanding
- Respond to ground commands from COMMS board
    - **Incomplete** handle_command mostly is complete, but needs two adjustments. OP_SET_MODE needs guardrails for the value. OP_REBOOT is a no-op, to be implemented after the watchdog sequence is confirmed.
- Determine modes (ex: standby, sleep, deep sleep, low power)
    - **Incomplete** cdh_mode_t defines setup, standard, mission, and error modes. standby, sleep, deep sleep, and low power don't exist.
- Monitor watchdog/pings
    - **Incomplete** the kick sequence is commented out and flagged @todo, and it only sends a line to the log. The node pings, which include handle_heartbeat and check_node_timeouts are fully working.
- Schedule downlinks
    - **Incomplete** This should fall under telemetry_thread, which is incomplete.

## Interactions with other subsystems
- EPS - CDH sets the power via OP_SET_PWR_STATE and reads back a CLS_CMD_RESP confirmation.
- GNSS - polls GNSS every GNSS_POLL_MS (defined in board_config.h) using op code OP_GET_POS. This response is parsed in the handle_telemetry function, which currently does nothing with it.
- COMMS - Inbound commands come via CLS_COMMAND and get handled via the class_command function. No outgoing path back through COMMS exists. This will need to be implemented when we get start on the COMMS board.
- ADCS - Currently gets heartbeat and checks for node timeouts. OP codes are defined for OP_ADCS_SOH_ATTITUDE, OP_ADCS_WHEEL_RPM, and OP_ADCS_MTQ_DIPOLE but CDH never sends any of these.
- MOTOR - Currently gets heartbeat and checks for node timeouts.
- SOLAR - Currently gets heartbeat and checks for node timeouts.

**Takeaway** Inter-board communication seems solid. I know they had it working at the end of Spring 2026, so that makes sense. What we need to do is implement inter-board commands, and get sensor/telemetry data from all the boards. And do stuff with COMMS, but we probably need to start on that board's code before we implement anything for it in CDH.