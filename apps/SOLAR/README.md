See <u>Files</u> above for function documentation.

Back to [Contents](@ref index)

# SOLAR — Magnetorquer Driver Firmware

SOLAR is the UT-CORE CubeSat's magnetorquer (MTQ) board, running Zephyr and driving 4 coil-face PWM outputs (+X/-X/+Y/-Y) that generate a magnetic dipole for attitude control, coordinating with ADCS over a CAN bus.

A CAN RX thread decodes incoming frames and dispatches them by message class. Commanded magnetic dipole moments (mx/my/mz) are currently actuated sign-only per axis — each face is driven fully on or fully off based on whether that axis's component is positive, negative, or zero; magnitude isn't yet honored. Z is received but discarded, since this board revision has no Z-axis torquer.

The main loop broadcasts a heartbeat and sends state-of-health telemetry (current duty and up to 4 onboard temperature readings) on independent fixed intervals.

## Hardware

- Bus: CAN (FDCAN1), 500 kbit/s, 29-bit extended IDs
- Transceiver: TCAN3403
- Actuators: 4 magnetorquer coil-face PWM channels (+X, -X, +Y, -Y), 20 kHz
- Sensors: up to 4 I2C temperature sensors

## Architecture

| Component | Role |
|---|---|
| `set_face_pwm` | Drives each coil face's PWM duty; -Y path (TIM1_CH4) is electrically inverted and compensated for in software |
| `handle_command` | Decodes ADCS's magnetic dipole command and translates sign into per-face actuation |
| `can_rx_thread` | Drains incoming CAN frames, decodes and dispatches by message class |
| Main loop | Broadcasts heartbeat and state-of-health telemetry on independent fixed intervals |

On boot, the firmware verifies GPIO/PWM devices are ready, forces all 4 MTQ outputs off, wakes the CAN transceiver, and brings up the bus before entering the main loop.
