# cdhModeTest

Hardware-in-the-loop test of the CDH mode state machine: with CDH in
STANDBY, a GNSS fix over Utah must switch it to EXPERIMENT.

## Setup

- CDH board running its normal firmware (`apps/CDH`).
- GNSS board flashed with **this test instead of the GNSS firmware**. CDH only
  accepts positions from `GNSS_ID` (0x06), so the test answers as that node.
  Reflash `apps/GNSS` afterwards.
- Both boards on the same CAN bus. No COMMS board on the bus: the test sends
  ground commands as `COMMS_ID`.

## Run

```
make build
make flash
```

Watch the GNSS board's RTT output. The test:

1. Waits up to 30 s for a CDH heartbeat (CDH's mode is read from heartbeat byte 2).
2. Sends a fix outside Utah (Las Vegas), then ground `OP_SET_MODE` SAFE then
   STANDBY. CDH must reach STANDBY and stay there for 3 s.
3. Sends a fix over Utah (Salt Lake City). CDH must reach EXPERIMENT within 5 s.

It then sends CDH back to SAFE and prints `=== PASS ===` or `=== FAIL ===`
with the reason. If STANDBY is rejected, CDH's own log says which STANDBY
guard failed.
