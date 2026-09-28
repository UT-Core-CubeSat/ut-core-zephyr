See <u>Files</u> above for function documentation.

Back to [Contents](@ref index)

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

## Software Requirements 9-25-26 (Cason)  

Currently, demo.c is the more complete of the two files. Unless noted specifically, the behavior described below is based on demo.c, because main.c only has a basic loop that reads and prints the power sensors.  

- Receive Serial data from external sensors
    - **Incomplete**: No UART code currently exists. In prj.conf, CONFIG-SERIAL is turned off. This is possibly because the EPS board doesn't have the logging potential over UART yet, but I'm not sure. We'd have to turn that on, as well as CONFIG_UART_CONSOLE, CONFIG_LOG_BACKEND_UART and turn off CONFIG_LOG_BACKEND_RTT. Future steps: write a function to handle receiving sensor dta, and define what the message format from the sensors should look like.
- Watch for pings from the CDH
    - **Complete** Basic CAN functionality exists. The only thing that might be needed is additional logic to handle CDH being unresponsive.
- Monitor power usage and generation
    - **Incomplete** read_power_monitors gets voltage, current, and power from the 7 INA3221 chips with 3 channels each and puts the data into telemetry[]. Right now those results only get printed to the console. We probably need to have a calculation of total power drawn and total power generated. There also is a lack of a link between a solar panel (or similar test input) to a channel. Also should probably have some kind of limit check for overcurrent or undervoltage. The ut_core.overlay also has some TODOs about confirming chip address, we'll probably have to work with whichever engineers are working on this board to get that info.
- Monitor each subsystems power data
    - **Incomplete** As far as code interacting with hardware, it should work. Each channel is ready, and the loads and motor rails can be toggled. Sending an OP_SET_PWR_STATE over CAN works and gets a reply. However, no documentation exists on which channel belongs to which other subsystem, readings are labeled only by chip and channel. We'll want to define those as more legible names. There are no limitations for how much power a subsystem can draw and no kind of shut-off. The motor rails don't have any interaction with CAN currently. Finally, EPS is listening for OPS_SET_PWR_STATE, but the actual definition in can_proto.h is OP_SET_EPS_LOAD. We need to pick one and go with it.
- Monitor battery data
    - **Incomplete** This hasn't been started. There's a possiblity that one of the channels has been designated as 'battery' but no documentation for that is found in any part of these files.
- Package SOH data and send to CDH
    - **Incomplete** The only thing sent currently is the heartbeat. ADCS has its own SOH opcodes, so will probably need to implement those for EPS. We'll eventually also need to figure out how we want to divide telemetry and other data across multiple CAN messages.  

## Subsystem Interactions

- Eventually EPS will be the driver for power across the CubeSAT via commands across the CAN bus. Currently it only interacts with CDH in the most basic of ways. We need to build a more robust set of functions to both send commands. I'd also suggest some kind of ACK response from the boards eventually that EPS can act on.