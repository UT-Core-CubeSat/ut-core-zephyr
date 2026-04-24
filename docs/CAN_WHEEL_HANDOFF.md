# Wheel CAN Link — Handoff Notes

Hey. Arick wired the ADCS-compute board to the BLDC motor-driver board over
FDCAN1 this session so the NDI torque output actually reaches the wheels.
Everything below describes exactly what's on disk now, what is _not_ on disk
and still needs your call, and the assumptions that were baked in.

Nothing here modifies ADCSCore physics. The torque→RPM conversion lives in
the app bridge layer on the ADCS side. Core keeps outputting `AdcsOutput::
wheel_torque` (N·m) just like before.

---

## 1. TL;DR of the dataflow

```
┌─────────────────────────── ADCS board ──────────────────────────────┐
│ sensors  ─►  ADCSCore::update()  ─►  AdcsOutput.wheel_torque [4×Nm] │
│                                            │                         │
│                                            ▼                         │
│                                 wheel_bridge_step()                  │
│                           (ω_next = ω_meas + dt/Jw · τ_sat)          │
│                                            │                         │
│                                            ▼                         │
│                              adcs_can_send_wheel_rpm()               │
│                                            │                         │
└────────────────────────────────────────────┼─────────────────────────┘
                                             │ FDCAN1, 500 kbit/s, 29-bit ID
                                             │ CLS_WHEEL_CMD @ 40 Hz
                                             ▼
┌──────────────────────── Motor driver board ─────────────────────────┐
│ motor_can_rx_thread ─► rpm_ref_cmd[NMOTORS]                          │
│                             │                                        │
│                             ▼                                        │
│                     existing PI loop @ 5 ms (unchanged)              │
│                             │                                        │
│                             ▼                                        │
│                    speed_rpm_ctrl_filt[]  (measured RPM)             │
│                             │                                        │
│                             ▼                                        │
│            motor_can_tx_thread ─► CLS_WHEEL_TLM @ 20 Hz              │
└────────────────────────────────────────────┼─────────────────────────┘
                                             │
                                             ▼
                               ADCS's RX thread publishes latest
                               ω_meas into SensorData::wheel_speeds
                               (which the observer was zeroing before).
```

---

## 2. The new CAN protocol (read this first)

It extends `common/can_proto.h` without touching it. New file:
**`common/can_wheel.h`**.

### 2.1. Frame layout

Both directions use **29-bit extended IDs** and a full **8-byte payload of
4 × int16 little-endian signed RPM values**. No `src`/`op` wrapper — all 8
bytes are data, so this bypasses `can_fill_payload()`. One frame carries all
four wheels.

| Field       | Value                                          |
|-------------|------------------------------------------------|
| `CLS_WHEEL_CMD` | `0x20` — ADCS → MOTOR, 40 Hz               |
| `CLS_WHEEL_TLM` | `0x21` — MOTOR → ADCS, 20 Hz               |
| `WHEEL_RPM_LIMIT` | `10000` RPM (mechanical, signed)         |
| `WHEEL_PRIO_CMD`  | 2                                        |
| `WHEEL_PRIO_TLM`  | 4                                        |

### 2.2. Full IDs (baked in, so filters can match SRC+CLASS together)

```c
WHEEL_CMD_ID = CAN_ID_FULL(WHEEL_PRIO_CMD, ADCS_ID,  MOTOR_ID, CLS_WHEEL_CMD)
WHEEL_TLM_ID = CAN_ID_FULL(WHEEL_PRIO_TLM, MOTOR_ID, ADCS_ID,  CLS_WHEEL_TLM)
WHEEL_ID_MASK_29 = PRIO|SRC|DST|CLASS (all 29 bits significant — exact-ID filter)
```

### 2.3. Payload struct

```c
struct __attribute__((packed)) wheel_rpm_frame {
    int16_t rpm[4];   // LE, signed, mechanical RPM at the rotor
};
```

`rpm[0]` is wheel 1, `rpm[3]` is wheel 4. Sign convention is whatever the
motor board considers "forward" — flip `M*_RPM_MEAS_SIGN` in `main.c` (at
[app/bldcHallL6234/src/main.c:49](app/bldcHallL6234/src/main.c#L49)) during
polarity bring-up if it reads backward on a given wheel.

### 2.4. Helpers (both sides use these)

- `wheel_rpm_pack(const float rpm[4], struct wheel_rpm_frame *f)` —
  rounds, saturates to `±WHEEL_RPM_LIMIT`, writes int16 LE.
- `wheel_rpm_unpack(const struct wheel_rpm_frame *f, float rpm[4])` —
  simple int16→float.

---

## 3. Exactly what changed (deltas)

### 3.1. New files

| File                                                     | Purpose                                      |
|----------------------------------------------------------|----------------------------------------------|
| `common/can_wheel.h`                                     | Shared CAN protocol (see §2)                 |
| `app/adcsComputeTest/src/wheel_bridge.hpp` / `.cpp`      | Torque → ω_ref → RPM conversion              |
| `app/adcsComputeTest/src/adcs_can.hpp` / `.cpp`          | ADCS-side FDCAN1 init + TX + RX              |
| `app/bldcHallL6234/src/motor_can.h` / `.c`               | Motor-side FDCAN1 init + RX + TX             |
| `docs/CAN_WHEEL_HANDOFF.md`                              | This file                                    |

### 3.2. Modified files

**`app/adcsComputeTest/src/mainAirBearing.cpp`** (the only existing file touched on ADCS side)
- Added `#include "wheel_bridge.hpp"` and `"adcs_can.hpp"`.
- Added `ADCS_TICK_PERIOD_MS = 25` (40 Hz) and `WHEEL_TLM_MAX_AGE_MS = 200`.
- `adcs_can_init()` is now called once during boot, after sensor init.
- **Deleted**: `sd.wheel_speeds = Math::Vec<4>::Zero();`
- **Added at the same spot**: `adcs_can_get_wheel_speeds(sd.wheel_speeds, 200)`.
  Returns stale/zero fallback if no telemetry within 200 ms.
- After `Core::update(...)`: now calls `wheel_bridge_step(...)` and
  `adcs_can_send_wheel_rpm(...)` every tick.
- Loop now paces itself to a fixed 25 ms period using an accumulator
  (`next_tick += ADCS_TICK_PERIOD_MS`, with catch-up on overruns).
- Telemetry printk lines added: `rpm_ref`, `omega_meas`, `tx/rx counts`,
  `last_rx_age`, `fresh` flag.

**`app/bldcHallL6234/src/main.c`**
- Added `#include "motor_can.h"` + `"can_wheel.h"`.
- Added `#define CAN_RX_WATCHDOG_MS 200`.
- **Deleted**:
  - `FAKE_CAN_REF_MS`, `FAKE_CAN_THREAD_PRIO` macros.
  - `fake_can_thread_data`, `fake_can_stack` thread plumbing.
  - The `fake_can_ref_thread()` function body entirely.
  - The `k_thread_join(&fake_can_thread_data, ...)` in the shutdown path.
  - Boot-time preload of `rpm_ref_cmd[]` from the waveform generator (the
    `motor_speed_ref_at_uptime(mi, 0)` call in `main()` init).
- **Added**:
  - `can_apply_setpoint_cb(mi, rpm_signed)` — called by the motor_can RX
    thread. Clamps to `±WHEEL_RPM_LIMIT` and writes `rpm_ref_cmd[mi]`.
  - `can_measure_rpm_cb(mi)` — called by the motor_can TX thread. Returns
    `speed_rpm_ctrl_filt[mi]` with sign from `m_use_rev[mi]`.
  - **200 ms watchdog** at the top of `motor_ctrl_thread`'s loop: if
    `motor_can_rx_age_ms() > 200`, zero all `rpm_ref_cmd[]`. Re-arms
    automatically on the next received frame.
  - `motor_can_init(&cfg)` is now called from `main()` (replaces the
    `fake_can_thread` creation site).
  - Telemetry `printk` line `CANLOG rx=… tx=… age=… state=…` once per
    500 ms (existing telemetry thread).
- **Kept, but marked `__attribute__((unused))`** (so the build stays warning-
  clean but the code survives one bench iteration, in case CAN is down and
  you need the old waveform sweeps to verify the PI loop in isolation):
  - `motor_speed_ref_triangle`
  - `motor_speed_ref_step`
  - `motor_speed_ref_sine`
  - `motor_speed_ref_at_uptime`
  - **Feel free to delete these entirely** once the CAN link is up. The
    `M*_REF_*` macros can go with them.
- The boot banner that printed per-motor waveform mode/period was replaced
  with a shorter one describing the CAN-driven config.

**`app/adcsComputeTest/prj.conf`** — added:
```
CONFIG_CAN=y
CONFIG_CAN_STM32_FDCAN=y
CONFIG_CAN_FD_MODE=n
```

**`app/bldcHallL6234/prj.conf`** — same three lines added.

**`app/adcsComputeTest/CMakeLists.txt`** — added `wheel_bridge.cpp` and
`adcs_can.cpp` to sources; added `common/` to include dirs.

**`app/bldcHallL6234/CMakeLists.txt`** — added `motor_can.c` to sources;
added `common/` to include dirs.

---

## 4. The bridge math (so you know what it's actually doing)

Per wheel, every ADCS tick, in `wheel_bridge_step()`:

```
τ_sat     = clamp(τ_cmd,      -τ_max,       +τ_max)            [N·m]
ω_next    = clamp(ω_meas + dt/J_w · τ_sat,  -ω_cap,  +ω_cap)   [rad/s]
rpm_ref   = ω_next · 60 / (2π)                                 [RPM]
```

**Key choices (flag these if you disagree):**

- `J_w`      comes from `Param::Actuators::I_wheel` (= 1.13e-6 kg·m²).
- `τ_max`    comes from `Param::Actuators::tau_w_max` (= 0.13 N·m).
- `ω_cap`    is `min(Param::Actuators::omega_w_max, 10000 RPM·2π/60)`.
  **This is a deliberate mismatch fix**: `omega_w_max` is 12000 RPM in
  `core_Parameters.hpp`, but the motor board caps at 10000 RPM
  (`WHEEL_RPM_ABS_MAX` in `main.c:52`). The bridge saturates at whichever
  is tighter so the ADCS integrator doesn't accumulate torque the motor
  will silently clip away. **Decide which number is truth** and align both
  if you don't like `min()` as the reconciliation.
- `dt`       uses the actually-measured loop dt (`k_uptime_get()` delta),
  not nominal 25 ms. This way scheduler jitter doesn't show up as an
  integration error.
- `ω_meas`   comes from motor-board telemetry. If no CAN RX in 200 ms,
  the bridge gets zeros — the observer was already seeing zeros before
  this patch, so CAN-dropout behavior is no worse than the pre-patch state.

The conversion is **stateless across ticks** — it uses `ω_meas` each tick,
not the previous `ω_ref`. That's intentional: if the motor board can't
track our RPM request, ω_meas diverges from ω_ref, and the bridge sees the
truth next tick instead of drifting away from reality.

---

## 5. What you still need to decide / verify

### 5.1. Wheel ↔ body-axis mapping

Arick's spin matrix in ADCSCore works out to this:
- Wheel 1 ↔ +X
- Wheel 2 ↔ −X
- Wheel 3 ↔ +Y
- Wheel 4 ↔ −Y

Controller allocation matrix `S` in `core_Parameters.hpp:121-134` already
assumes this; nothing on the bridge side cares about it. The motor board
just reads `rpm[0..3]` and drives the 4 motors in slot order. **Polarity
validation is still a hardware step** — spin each wheel one at a time from
a commanded `τ>0` and confirm the body reacts the expected direction on
the air bearing. If any wheel reacts backward, flip `M*_RPM_MEAS_SIGN` in
`app/bldcHallL6234/src/main.c` (around line 49/86/102/118).

### 5.2. The PA9/PA10 transceiver-GPIO conflict ⚠

`boards/arm/ut_core/ut_core-common.dtsi` aliases PA10 = `canengpio` and
PA9 = `canstbgpio` (the CAN transceiver enable/standby lines).

**Both app overlays repurpose these pins.** PA10 is LED0 on ADCS and M1
phase-B enable on the motor board. PA9 is something similar on motor side.

Consequence: the firmware **cannot drive the transceiver enable/standby
from software on either board**. The CAN init code therefore does **not**
touch those pins. The bus will only work if the transceiver IC is
hardware-default ON at power-up (which many transceivers are, but check
your schematic).

**Action items for you**:
1. Confirm on the schematic that the transceiver is wired always-on, or
   wired to a fixed pull that enables it.
2. If not, you'll need either an overlay rework (drop the LED-on-PA10 and
   rewire that LED, for example) or a hardware patch to decouple
   transceiver control from the reused MCU pins.

This is flagged as a comment at the top of `common/can_wheel.h` too.

### 5.3. Failsafe policy on CAN dropout

Currently: **zero `rpm_ref_cmd[]` after 200 ms of RX silence**. PI loop
then coasts the wheels to rest (no active braking — duty just goes to 0).

If you want a different policy (ramp-to-zero over N ms, or hold last
setpoint), change the watchdog block inside `motor_ctrl_thread` —
[app/bldcHallL6234/src/main.c:1191](app/bldcHallL6234/src/main.c#L1191)ish.

### 5.4. Telemetry cadence

Motor→ADCS telemetry runs at 20 Hz (50 ms period). Changing it: tweak
`WHEEL_TX_PERIOD_MS` in `motor_can.c`. The ADCS age tolerance is 200 ms
(`WHEEL_TLM_MAX_AGE_MS` in `mainAirBearing.cpp`) — keep this at ~4× the
TX period so one dropped frame doesn't panic the freshness check.

### 5.5. ADCS loop cadence

Now fixed at **40 Hz (25 ms)** via an accumulator timer at the bottom of
`adcs_loop()`. It was ~50 Hz before but driven by incidental I²C delays,
not a fixed period. If you want a different rate, change
`ADCS_TICK_PERIOD_MS` in `mainAirBearing.cpp`. The bridge uses measured
dt regardless, so cadence tuning is decoupled from integrator correctness.

### 5.6. Thread priorities

Both sides have CAN RX on priority 1–2 (just below the real-time motor
loop on the motor board). If you want to restructure, search for
`PRIO` / `THREAD_PRIO` in `adcs_can.cpp` and `motor_can.c`.

---

## 6. How to bring it up on the bench

In order:

1. **Build both apps** with `-DAIR_BEARING=ON` for the ADCS app and the
   normal config for the motor app. Confirm `CONFIG_CAN_STM32_FDCAN` shows
   up in the generated `.config`.

2. **Loopback smoke (motor board only)**.
   Temporarily swap `CAN_MODE_NORMAL` → `CAN_MODE_LOOPBACK` in
   `motor_can_init()`. In a test build, send a wheel-cmd frame to
   yourself and check that `can_apply_setpoint_cb` fires with the right
   per-wheel values. Same trick on the ADCS side with `adcs_can.cpp`.
   Revert to `CAN_MODE_NORMAL` before moving on.

3. **Two-board connection**. Wire CAN-H / CAN-L between boards, 120 Ω
   terminations on both ends. Power both. Watch the motor-board UART:
   ```
   CANLOG rx=<n> tx=<n> age=<ms> state=fresh
   ```
   `rx` incrementing = ADCS is transmitting and the motor board is
   listening on the right filter.

4. **Sanity torque ramp**. Force `MissionMode::BEARING` on ADCS with a
   small artificial τ (e.g. temporarily hand-edit the allocation or
   inject a reference). With `τ = 0.01 N·m` on wheel 1:
   ```
   α = τ/Jw = 0.01 / 1.13e-6 ≈ 8850 rad/s² ≈ 84500 RPM/s
   ```
   So wheel 1 should reach the 10000 RPM cap in ~120 ms. **Turn τ way
   down** to watch the ramp in real time (`τ = 1e-4` gives ~850 RPM/s).

5. **Failsafe check**. Command a steady mid-scale setpoint (say 3000
   RPM). Pull the CAN cable. Within 200 ms the motor-board `CANLOG`
   state should flip to `stale-WATCHDOG`, `rpm_ref_cmd[]` zeros, and the
   wheels coast down.

6. **Observer feedback check**. On ADCS telemetry, `omega_meas` (line
   `[ADCS] omega_meas [rad/s]: …`) should be non-zero and track motor
   telemetry within ~50 ms lag + filter time. Before this patch it was
   hard-zero. If it's still zero with a live link, check `rx` counter —
   filters or IDs are likely mismatched.

7. **Polarity spin** per §5.1.

---

## 7. One-glance file map

```
common/
  can_proto.h      (unchanged — existing protocol)
  can_wheel.h      NEW — wheel-specific protocol extension

app/adcsComputeTest/
  prj.conf         MODIFIED (+CAN kconfig)
  CMakeLists.txt   MODIFIED (+sources, +include)
  src/
    mainAirBearing.cpp  MODIFIED (see §3.2)
    wheel_bridge.hpp    NEW
    wheel_bridge.cpp    NEW
    adcs_can.hpp        NEW
    adcs_can.cpp        NEW

app/bldcHallL6234/
  prj.conf         MODIFIED (+CAN kconfig)
  CMakeLists.txt   MODIFIED (+motor_can.c, +include)
  src/
    main.c         MODIFIED (see §3.2; core PI loop untouched)
    motor_can.h    NEW
    motor_can.c    NEW
```

---

## 8. Things Arick wants flagged for your review

1. **ω_cap reconciliation** (§4): hard-min'd to 10000 RPM in the bridge.
   If you'd rather bump the motor cap to 12000 instead, edit
   `WHEEL_RPM_LIMIT` in `can_wheel.h` AND `WHEEL_RPM_ABS_MAX` in
   `main.c:52`, and let me know so I can remove the `min()` in
   `wheel_bridge.cpp`.

2. **Transceiver control** (§5.2): not driven from firmware on either
   board, pending your call on the PA9/PA10 overlay conflict.

3. **Waveform ref generators still present but unused**: one-iteration
   insurance for bench tests where CAN is down. Delete them when you
   trust the link.

4. **`motor_has_speed_ctrl(mi)` gate on the RX callback**: the setpoint
   callback only writes `rpm_ref_cmd[mi]` for wheels that have speed
   control enabled. If you ever add speed-controlled wheels beyond
   indexes 0/1/M3_IDX/M4_IDX, update that predicate too
   ([main.c:323](app/bldcHallL6234/src/main.c#L323)).

5. **ADCS loop cadence is now fixed 40 Hz**. Was previously ~50 Hz,
   driven by I²C wait padding. If you want ADCS faster, the bridge
   handles variable dt correctly — the only thing that scales with rate
   is CAN bus load (40 frames/s now; at 100 Hz that'd be 100 frames/s
   on cmd + 20 on tlm = 120 frames/s out of ~500 frame/s budget at
   500 kbit/s, so plenty of headroom).
