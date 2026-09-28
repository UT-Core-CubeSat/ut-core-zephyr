Back to [Contents](@ref index)

# ADCS Firmware

@todo Please work with the control team to better document this.

I believe much of this code is a direct copy of the [ut-core-controls](https://github.com/UT-Core-CubeSat/ut-core-controls) repository. I am unsure which version.

@warning **PLEASE do not continue to copy the control code over to here! It contributed to a lot of GIT BLOAT that took a lot of effort for me to remove and cull!**
Instead, please replace the copied control code with a git submodule of the control repository. More about submodules [here](https://www.geeksforgeeks.org/git/git-submodule/).

@note If any files are missing, check the old [ut-core-zephyr](https://github.com/UT-Core-CubeSat/ut-core-zephyr_old) repository. It might've gotten taken out when I culled a lot of the garbage attached to the `.git`.

@warning **SERIOUSLY, don't just copy the control code to over here!**


## Control Code vs Board Code

- **`forOrbit/ADCSCore/` and `airBearingDemo/ADCSCore/` are the control team's code**, copied in
  from the controls repo. These do not contain Zephyer OS at all. They are portable C++ that
  builds anywhere, which is why they ship with a desktop `plant/` simulation. This
  must not be re-copied and should become a git submodule pinned to a known commit.

- **`src/mainOrbit.cpp` and `src/mainAirBearing.cpp` are the actual board firmware.** They
  are Zephyr applications that read real sensors and feed `ADCSCore`.

---

## Current Status

ADCS runs the attitude determination and control algorithms in `ADCSCore` against real IMU and
magnetometer readings on the `ut_core` board. The estimator and controllers are implemented, 
but the board doesn't have CAN setup, so it can't receive commands or report to the CDH.

I havent yet been able to flash the ADCS board because of a reset switch issue so next steps
also include trying to flash the board in the cubesat and ordering the v3 acds board

### Build Targets

One app switches which software to flash by CMake option in `CMakeLists.txt`:

If Air Bearing is set to on it flashes mainAirBearing.cpp otherwise it flashes mainOrbit.cpp

I was having issues with this build process so I added a specific make build-air and make flash-air
to build an dflash the airBearing code specifically

### Hardware

- MCU: STM32U5, Zephyr RTOS
- IMU: 2x LSM6DSV 6-axis (I2C 0x6A, 0x6B)
- Gyro: 2x I3G4250D 3-axis (I2C 0x68, 0x69)
- Magnetometer: 3x MLX90393 (I2C 0x0C, 0x0D, 0x0F)
- I2C1 remapped to PB8 (SCL) / PB9 (SDA) via the app overlay
- Sensor power rail enabled on PC9

### Requirements Status

Based on the ADCS software requirements in the waterfall requirements document:

| Requirement | Status | Notes |
|---|---|---|
| Read magnetometers and IMUs | **Implemented** | 7 sensors over I2C with `WHO_AM_I` validation, per-sensor health flags, and averaging across whichever sensors responded |
| Reports attitude to CDH (vector + rates) | **Not implemented** | Attitude and rates are computed by `ADCSCore`, but only printed to console. `mainOrbit.cpp` contains no CAN code |
| SOH to CDH | **Not implemented** | No SOH packaging. Opcodes are already reserved in `common/can_proto.h` (`OP_ADCS_SOH_ATTITUDE`, `OP_ADCS_WHEEL_RPM`, `OP_ADCS_MTQ_DIPOLE`) |
| Responds to commands (sun pointing / nadir / ground point) | **Not implemented** | Mode is hardcoded to `DETUMBLE` and never changes; there is no command receive path. `ADCS::MissionMode` already defines `STANDBY` (sun), `IMAGING` (nadir/target) and `DOWNLINK` (ground point), so this is missing plumbing rather than missing algorithms |
| Takes in star tracker vector | **Not implemented** | `star_quat` is a hardcoded identity quaternion placeholder |

### Needed For Flight

1. **CAN is not implemented in `mainOrbit.cpp`.** `CONFIG_CAN` and `CONFIG_CAN_STM32_FDCAN` are enabled in
   `prj.conf` but arent used.

2. **The estimator never receives an absolute attitude reference.** `star_quat` is fed a
   constant, and the observer only accepts a star update when the quaternion *changes*
   (`q_diff > 1e-6`), so that update never fires. `css_currents` is fed zeros, so the computed
   sun current sum is always below the eclipse threshold and the QUEST branch never fires
   either. The observer therefore only ever propagates the gyro, producing open-loop dead
   reckoning whose bias is never corrected and whose attitude drifts without bound. Note that
   `estimator_valid` is only a NaN check, so it still reports valid during this drift.

3. **Control output actuates nothing.** `mtq_dipole` is computed and printed, but never
   transmitted. The SOLAR board already implements a handler for ADCS's dipole command, so it
   is waiting on a message that is never sent.

4. **`unix_time` is not Unix time.** `mainOrbit.cpp` passes `k_uptime_get() / 1000.0` (seconds
   since boot) into a field documented as coming from GPS or RTC. `ADCSCore` converts it to a
   Julian date for the solar ephemeris and the ECI/ECEF rotation, so any pointing mode would
   compute its references for 1 Jan 1970. This is currently masked only because those code
   paths never execute. Real epoch time needs to come from GNSS or CDH over CAN.

5. **Placeholder inputs.** `star_quat`, `css_currents`, `gps_ecef` and `wheel_speeds` are all
   waiting on star or sun sensor

6. **No watchdog and no modes**

### Improvements Needed

- **No "no data" signal.** If every IMU and magnetometer fails it passes zeros to the core making
  satellite think its not moving
- **Sensor init return values are ignored.** The board enters its main loop regardless of how
  many sensors came up, and the count is never reported beyond the boot log.
- **Magnetometers are averaged component-wise** with no per-sensor mounting alignment and no
  hard/soft-iron calibration, using scale factors the comments mark as approximate. See
  `apps/tests/magCalTest` for a starting point.
- **Bench instrumentation runs in the flight loop.** Three multi-line float `printk` calls fire
  every cycle (~10 ms), commented as being for a MATLAB viewer, and the boot banner still reads
  `ADCS Compute Test`. The CMake project is likewise still named `adcsComputeTest`.
- **Duplicate code**`ADCSCore` forOrbit code and airBearing code are duplicated into the 
`apps/tests/adcsComputeTest/` so we probably need to delete the duplicates eventaully

### Software Work

1. Port CAN from `mainAirBearing.cpp` into `mainOrbit.cpp` — heartbeat, attitude/rate telemetry
   and SOH. This closes two requirements using opcodes that already exist.
2. Add the command receive path to satisfy the mode requirements.
3. Transmit `mtq_dipole` to SOLAR and wheel commands to MOTOR.
4. Feed real epoch time from GNSS or CDH in place of boot uptime.
5. Calculate a real attitude based on star tracker/solar panel sun sensor once the sensor is implemented