# COMMS: plan, status and open items

The build plan for the COMMS firmware: what is done, what the stages are, what to do next, the bench test
that is the first hardware milestone, the radio driver notes, and the open decisions. Last updated
2026-11-04.

Where things are: the board overview and architecture are in [README.md](README.md), the wire format and its
rationale are in [PROTOCOL.md](PROTOCOL.md), and this file holds everything about progress and next steps.

Conventions: **(spec)** means checked against the CCSDS text in `external_documents/`. **(verify)** means not yet
checked. Radio facts come from the RFM26W datasheet and AN625 (Si446x API descriptions), both in
`external_documents/`.

## 1. Status at a glance

| | |
|---|---|
| Firmware | **Stage 1 of 7 done.** The link-layer codecs exist and are tested. `src/main.c` is a stage 0 scaffold that only brings up CAN and logs a heartbeat. There is no protocol traffic yet |
| Radio driver | Complete enough for the MVP: SPI command layer, start/shutdown/configure, block TX and RX through the FIFOs with nIRQ handling in its own work queue, RSSI, statistics. Stubs: the continuous-wave test and the PM action (phase 4) |
| Tests | 51 firmware host tests (`apps/tests/commsTest`) and 27 radio driver tests (`apps/tests/rfm26wEmulTest`), all passing on `native_sim` |
| Hardware | Not available for at least 2 months. Nothing has run on a real board or radio. The radio profile table is an empty placeholder until WDS output exists (section 5.4) |
| Next | **Stage 2**: the uplink and downlink pipeline against the radio driver, and PING (section 3) |

## 2. Firmware stages

| Stage | Goal | Status |
|---|---|---|
| 0 | Foundations: docs, module layout, build config, minimal `main.c`, shared radio model, test app, Make rules | **done** 2026-11-02 |
| 1 | Link-layer codecs with tests: CRC, TC frame, Space Packet, downlink block, CAN record | **done** 2026-11-02 (51 tests, 14 mutation checks all caught) |
| 2 | Pipeline against the radio driver: `radio_port`, uplink, router, downlink queue, PING. Integration tests with the radio model | todo |
| 3 | CAN bridge and MVP completion: GROUND_ID filter, uplink policy, CAN_CMD, telemetry cache, TLM and beacon, scheduler, watchdog, test switches. MVP acceptance tests with CAN loopback | todo |
| 4 | Security: SDLS AES-GCM (256-bit), replay window, development key flag | todo |
| 5 | CFDP Class 2 receive subset, staging interface | todo |
| 6 | Firmware push to other boards over CAN, image signature check | todo |
| 7 | Hardening: UNP12-91 cease transmission, state gating with CDH, watchdog supervisor, stack audit, S-band hook, Reed-Solomon, real pins | todo |

The **MVP** (section 4) is stages 0 to 3. Stages 4 to 7 are the rest of PROTOCOL.md.

### Stage 0 and 1 notes (done)

- The old `main.c` (2,751 lines, UT-LINK frames, AES-GCM with a 4-byte `seq`, FRAM, AT86RF215 stub) was removed.
  It is not a base for anything.
- `Kconfig` is empty on purpose. Each stage adds the options it uses. `prj.conf` keeps the PSA crypto lines,
  unused until stage 4, because they were the tested ones.
- `boards/ut_core.overlay` has a placeholder radio node (chip select PA0, nIRQ PA1, SDN PA2, all invented) and
  the 30 MHz reference clock assumption with its warning. `BUILD_ASSERT` in `main.c` ties `COMMS_BLOCK_LEN` to the
  radio node's `block-length`.
- The firmware build links no protocol code yet: `main.c` does not call the modules, so the linker drops them.

**Stage 1 decisions** (all documented in the headers). Bit layouts were checked against the CCSDS text before
coding (TC header against `space_data_link_protocol.txt` figure 4-2 and sections 4.1.2.2 to 4.1.2.8, Space Packet
header against `ccsds.txt` sections 4.1.3.2 to 4.1.3.5).
- **TC receive order** follows PROTOCOL.md section 5: frame length and FECF first, then version, spare bits, type,
  spacecraft ID, virtual channel. A frame with a bad FECF is reported as `TC_ERR_FECF` even if it also has the wrong
  spacecraft ID. A test pins this order.
- **Strictness:** spare bits must be 00 (`TC_ERR_SPARE`). The frame sequence number is not checked on receive.
  Bytes after the frame are ignored.
- An empty frame data field is structurally valid at the frame layer. The Space Packet layer rejects it.
- **Space Packet:** sequence flags other than 11 are rejected (`SP_ERR_FLAGS`), because segmentation is CFDP's job.
  APID 0x7FF parses like any other. The router decides what to do with it.
- **Downlink block:** CRC checked before length. Padding content is not checked.
- Sequence counters are one array per direction, indexed by APID, with no locking: one thread owns each.
- **CAN record:** rejects reserved ID bits (25 to 22) and frames that are not classic, extended, 8-byte data frames.
  The destination is in the CAN ID, data byte 0 is `src`, data byte 1 is the opcode, so a frame carries
  **6 bytes of application data**. (`common/can_proto.h`, `apps/CDH/src/main.c:271` and `apps/EPS/src/demo.c:311`
  agree. If some board really puts the destination in the data, say so: it matters for stage 6.)

**Stage 1 tests** (51): `environment` 2, `crc16` 4, `tc_frame` 14, `space_packet` 12, `dl_block` 10, `can_record` 9.
They include exact header byte vectors worked out by hand from the spec figures, an independent bit-by-bit
reference CRC (so the firmware CRC is not checked with itself), round trips at every legal length, every
single-bit error rejected, limits, truncated and oversized input, and 20,000 random blocks per layer that must
never crash and are essentially never accepted. **Mutation checks:** 14 deliberate bugs (FECF range off by one,
frame length without the minus 1, bypass and control bits swapped, SCID check removed, sequence-count mask wrong
or not wrapping, downlink CRC over the wrong length, length limits removed, reflected CRC `crc16_ccitt`, CAN
reserved bits allowed, CAN ID little-endian) were each put in, caught by at least one test, and restored.

## 3. Next: stage 2, the pipeline against the radio driver

Needs nothing from the user.
1. `app/radio_port`: a thin layer over `ut/drivers/rfm26w.h`. The receive callback copies the block into a queue
   with `K_NO_WAIT` and counts a drop if full. Send goes through one function so `radio_tx` is the only sender.
   Make it replaceable in tests.
2. `app/uplink`: block to `tc_frame_parse` to `sp_parse`, one counter per drop reason (use the `*_status_str`
   helpers for logs). Stage 4 inserts SDLS between the frame and the packet: the `tc_frame` API already returns
   the data field and its length.
3. `app/router`: dispatch by APID. Stage 2 handles `0x0F0` (ping) only. Unknown APIDs are counted and dropped.
4. `app/downlink`: a queue of packets, `sp_build` with `sp_seq_next`, `dl_block_pack`, `radio_tx`. Ping reply on
   `0x0F1`: `ping_id` (4) and the payload returned unchanged (PROTOCOL.md section 14).
5. Tests: unit tests for each module, then integration tests in `commsTest` using the radio model
   (`rfm26w_emul_inject_rx`, `rfm26w_emul_take_tx`): inject a TC frame carrying a ping, expect a downlink block
   with a correct reply. Cover lost blocks, corrupt blocks, back-to-back pings, a full TX queue. Then mutation
   checks.
6. `main.c` gets the stage's threads and queues, each thread body a few lines that call a module (for example the
   `uplink` thread takes a block from its queue, calls `uplink_handle_block()` and enqueues the ping reply). The
   logic stays in the modules so the host tests can reach it. Update the thread table in the `main.c` header
   comment as each thread is added.
7. Update this file (the stage table and these notes) and the thread table in README.md.

**Layout decision (2026-11-04):** the app keeps its logic in separate modules under `src/` rather than one
`main.c` like CDH and EPS. Nothing in Zephyr or the mission requires a single file, and the split is what lets the
51 host tests call the codecs without the board's hardware. `main.c` stays the front door (overview comment,
threads, init order). Merging the modules into `main.c` was considered and would need the tests to `#include`
`main.c` under a `COMMS_HOST_TEST` guard. Merging the small modules into one `.c`/`.h` pair per layer (link,
pipeline, can_bridge, sec, cfdp) is a possible later step if the file count becomes a nuisance.

**What each later stage needs from the user first:**
- Stage 3: the CAN target board, a harmless request-and-reply command and the allowlist (open item 26), and how the
  callsign is carried (open item 3).
- Stage 4: where the replay counter is stored across resets (open item 1), and key provisioning (item 5).
- Stage 5: where the staging flash is (item 1).
- Stage 6: the CAN firmware-transfer protocol (item 10). Needs the user's decisions.
- Stage 7: the state-gating table from CDH, and the board.

**Stage 3 scope:** `can/can_bridge`, `uplink_policy`, `telemetry`. The common `can_setup()` installs filters only for
this node's ID and broadcast, so COMMS also needs a filter for `dst` = `COMMS_GROUND_ID` (replies from boards are
addressed to the ground). Add it in stage 3, not in `common/`.

**Stage 4 notes:** AES-GCM goes through PSA (`psa_aead_decrypt`) with a 256-bit key. The old self-test used a
128-bit key, so it changes. Uplink order in code: cheap IV check, then decrypt and verify, then commit the new IV
and act. Never use plaintext from a failed frame.

**Stage 6 notes:** firmware to another board is COMMS to target over CAN with the target verifying the signature
itself (PROTOCOL.md section 7). COMMS self-update is the risky one: MCUboot test swap with automatic revert, update
COMMS last and one board at a time.

**Keeping COMMS responsive** (design rules for all stages):
- Radio RX and the CAN forwarding path run at the highest thread priority. Flash writes go through a low-priority
  worker with a bounded queue. If the queue is full the PDU is dropped and CFDP's NAK recovers it.
- Erase the staging area when the Metadata PDU arrives, not mid-transfer. **(verify)** read-while-write on the
  U5A5 in dual-bank mode. If not supported, an erase can stall code execution.
- A hardware watchdog is fed by a supervisor that checks RX progress, so a wedged transfer path causes a reset
  instead of a deaf radio.
- CFDP Cancel is always accepted and takes priority over any transfer.
- Hardware backstop: CDH power-cycling COMMS via the EPS load switch (`OP_SET_EPS_LOAD`), if COMMS sits on a
  switchable rail. **(verify with the EPS design)**
- Beacon: lowest-priority thread. Beacon TX briefly interrupts RX, so an RX window right after each beacon is an
  opportunity.

## 4. The MVP: bench link test

The first test to run when the finished COMMS board arrives. It is a simplified profile of PROTOCOL.md (the wire
differences are in PROTOCOL.md section 14: no SDLS, plus test-only ping APIDs).

**Goal:** show that COMMS and a ground station, in the same room, can exchange data in **both directions** over the
RF link, and that COMMS can put an uplinked command on the CAN bus. The test is about whether the radio path works,
not about the protocol being complete.

**Not in the MVP:** encryption or authentication (SDLS), replay protection, CFDP and any file transfer,
Reed-Solomon, persistent storage, S-band, firmware update, state gating of commands, UNP12-91.

**Radio settings:** as PROTOCOL.md section 2, at 9.6 kb/s (a working value, not a decision). TX power at the
**minimum** setting (same room, so no more is needed; the datasheet range starts at -40 dBm, to verify on the
module). Half-duplex: both ends sit in RX and switch to TX only to send, then return to RX. A block with preamble
and sync word is 243 bytes on the air, about **203 ms**, so a ping round trip is at least about 0.4 s.

### Tests

Each test can be turned on and off on its own: a Kconfig option per test (`COMMS_MVP_PING`, `COMMS_MVP_TLM`,
`COMMS_MVP_CAN_CMD`, default on in the MVP build), plus a runtime switch if it is cheap to add. With a test off,
COMMS ignores that APID (uplink tests) or does not transmit it (downlink tests). The block, CRC and Space Packet
layers are always on.

**Run one test at a time.** The link is half-duplex with no collision avoidance. If telemetry transmits while
ground sends a ping, the blocks can collide, and the loss looks like a link problem. Running all tests together is
optional and its results are not counted against the pass thresholds.

| Test | Needs | Direction exercised |
|---|---|---|
| PING | ground radio only | up and down |
| TLM | ground radio only (receive) | down only |
| CAN_CMD | ground radio, plus another board on the CAN bus | up, then CAN, then down |

**PING (round trip and loss).** Ground sends a request (`0x0F0`) every 1 s by default. COMMS returns the same
`ping_id` and payload on `0x0F1`. Ground records sent, received, lost, round-trip time and RSSI per reply, and
prints a summary. *Pass (proposal):* at least 99% of 1000 pings come back with matching `ping_id` and payload, with
no CRC failures other than ones the ground counts as lost (about 17 minutes). Record the loss rate, round-trip time
and RSSI.

**TLM (periodic downlink, no uplink involved).** COMMS sends a telemetry packet on `0x020` at a fixed period
(default 5 s) with an incrementing `tlm_counter` and fixed test records. A beacon on `0x021` goes out at its own
slower period (placeholder 60 s) with 1-3 records. Ground checks `tlm_counter` and the Space Packet sequence count
for gaps and checks that record contents match. *Pass (proposal):* 10 minutes with no missing counter values.

**CAN_CMD (command to the CAN bus and back).** Ground sends a command packet on `0x010` containing a CAN record.
COMMS checks it: `src` equals GROUND_ID (proposed `0x10`) and `dst`/`class`/`op` are on a small allowlist. A failing
record is dropped and counted. COMMS puts the record on CAN with `src` = GROUND_ID and `dst` = the target board
(which must accept `src` = GROUND_ID). The target's reply is addressed to GROUND_ID. COMMS picks it up with a CAN
filter and sends it down on `0x020`. The bench uses the existing ad-hoc CAN network with the other boards. Pick a
harmless request-and-reply command (for example a status or telemetry request). *Pass (proposal):* a command sent
from ground produces the expected CAN frame on the bus (seen on the target or a bus logger), and the reply arrives
at ground intact.

### Ground station (stub)

The ground radio hardware is not yet known: a radio connected to a Raspberry Pi (open item 19). The ground side is
specified by behavior only. Ground software must:
- Build and parse the blocks and the Space Packet header exactly as in PROTOCOL.md (uplink as a TC frame, downlink
  as `len` + packet + CRC).
- Match the on-air settings (modulation, rate, sync word, preamble, 223-byte receive length), and read RSSI per
  received block, if the hardware reports it. If the ground radio is also an RFM26W, the latched RSSI is available
  through `GET_MODEM_STATUS`.
- Send ping requests and command packets.
- Receive and log everything with timestamps: block CRC failures, packet type, counters, RSSI.
- Report ping loss, round-trip time and telemetry gaps.
- Never transmit while it is expected to be receiving the reply (half-duplex).

To fill in once known: ground radio hardware, connection to the Pi (SPI, USB, UART), software language and
libraries (Python is likely), antenna.

### Bring-up order

Run these in order. Do not move on until the step passes.
1. **Radio alive.** Initialize the RFM26W, read back the part number with `PART_INFO` and confirm the CTS handshake
   works. Write the configuration, read the key properties back with `GET_PROPERTY`, and confirm the sync word,
   preamble length and receive length hold what was written. Then check on the air, with a second receiver, that
   the sync word appears as `1A CF FC 1D` (section 5.3 and open item 13). Before trusting any on-air result,
   confirm the module's reference clock: the driver ASSUMES a 30 MHz crystal, and a wrong value silently shifts the
   carrier and data rate (open item 24, section 5.4).
2. **COMMS transmits, ground receives.** Enable TLM only. Ground sees valid blocks with correct CRC.
3. **Ground transmits, COMMS receives.** Send blocks from ground, COMMS logs CRC pass/fail and the Space Packet
   header on its debug console (RTT). No reply needed yet.
4. **PING both ways.** Run the loss test.
5. **CAN_CMD.** Run it on the ad-hoc CAN network.
6. **Results run.** Each test on its own, results recorded: loss rate, RSSI, round-trip time, any CRC failures.
   Optionally all tests together, noted as not counted.

### Firmware-level hardware checklist

The radio's own checklist is section 5.5. These are the firmware items only the board can settle. Add to this list
at each stage.
1. The firmware boots on the real board and logs the heartbeat over RTT.
2. `can_setup` brings up FDCAN with the real transceiver and the real pins (the current pins are CDH's).
3. The firmware's own TC frame, downlink block and Space Packet bytes are accepted by an independent ground
   decoder. Stage 1 checked them only against the spec text and hand-built vectors.
4. A TC frame sent by an independent implementation passes `tc_frame_parse` (spacecraft ID and virtual channel as
   configured on both ends).

### Placeholders and assumptions

Every one of these is a guess or an unmade decision. Change it in the file named, and nowhere else.

| Item | Value now | Where | Who decides | Open item |
|---|---|---|---|---|
| Spacecraft ID (10 bits) | `0x0A5` | `comms_config.h` `COMMS_TC_SCID` | CCSDS assigns it | 11 |
| Virtual channel ID | 0 | `comms_config.h` | proposal in PROTOCOL.md | 11 |
| Ground station node ID | `0x10` | `comms_config.h` `COMMS_GROUND_ID` | with CDH and the other boards | 2 |
| Callsign and SSID | `NOCALL`, 0 | `comms_config.h` | needs a licence | 3 |
| Radio reference clock | 30 MHz crystal, **assumed** | `boards/ut_core.overlay` `xo-frequency-hz` | check the module | 24 |
| Radio profile table | empty placeholder | `drivers/rfm26w/src/rfm26w_profiles.c` | needs WDS output | 20 |
| Radio pins, chip select | invented (PA0, PA1, PA2) | `boards/ut_core.overlay` | the schematic | |
| CAN pins and node | copied from CDH | `board_config.h`, overlay | the schematic | |
| Thread stacks, queue lengths | guesses | `board_config.h` | measure | |
| Bit rate | 9.6 kb/s, undecided | radio profile | | 7 |
| Bench frequency | not chosen | | limited by the module's band variant | 18 |

## 5. Radio driver (`drivers/rfm26w/`)

The radio has a driver in the Zephyr driver model with its own small API, so the application does not depend on the
chip. Block packing, Space Packets, SDLS, CFDP and CAN routing stay in the application. The driver moves fixed-size
blocks and reports RSSI. The public API, with full doc comments, is `include/ut/drivers/rfm26w.h` (the source of
truth).

> **WARNING: the radio's reference clock is an ASSUMPTION and may need adjusting.** The driver assumes a
> **30 MHz crystal**, because that is the Si446x default and the datasheet's "default crystal". Nothing confirms what
> the RFM26W module on the board actually uses (a different crystal, or a TCXO). It is set by the devicetree
> properties `xo-frequency-hz` (default 30000000) and `xo-tcxo` (default off), is sent to the chip in `POWER_UP`, and
> is used to compute the carrier frequency. **The chip cannot detect a wrong value:** the carrier frequency, data
> rate and deviation would all be off, and `PART_INFO` would still read correctly. Change the devicetree node if the
> module differs, then **regenerate the radio profile** (section 5.4). This is open item 24.

### 5.1 Phases

| Phase | Work | Status |
|---|---|---|
| 0 | Docs | done |
| 1 | Skeleton: module, binding, Kconfig, header, stub driver, `rfm26wTest` that builds | done |
| 2 | SPI command layer: CTS polling and timeout, `POWER_UP`, `PART_INFO`, `SET_PROPERTY`/`GET_PROPERTY`, profile table loading, `GET_INT_STATUS`, TX power (placeholder mapping), `start`, `shutdown`, `configure` | done, builds, **not run on hardware** |
| 3 | TX and RX: `START_TX`/`START_RX` with `RX_LEN` = block length, FIFO streaming from the work item, return to RX after a send, latched RSSI, statistics | done, builds, **not run on hardware** |
| 4 | Power and safety: PM actions, continuous-wave test mode | todo (stubs return `-ENOTSUP`) |
| 5 | Emulator tests: an SPI-level radio emulator on `native_sim` with ztest | done: `apps/tests/rfm26wEmulTest`, 27 tests |
| 6 | Hardware bring-up: section 4 bring-up order, verify the sync-word bit order and the RSSI conversion, produce the real property tables | todo, needs the board |

### 5.2 Design and behavior notes

**Module.** `apps/COMMS/drivers/rfm26w/` is a Zephyr out-of-tree module (not under the west-managed `modules/`
directory, which `make clean-purge` wipes). Contents: `zephyr/module.yml`, `CMakeLists.txt`, `Kconfig`,
`dts/bindings/radio/hoperf,rfm26w.yaml`, `include/ut/drivers/rfm26w.h`, `src/` (driver, `rfm26w_priv.h` with the
command codes, table walker and frequency math that need no Zephyr, `rfm26w_profiles.c`) and `emul/` (the software
radio model). Each app adds it before `find_package(Zephyr)`:
`list(APPEND EXTRA_ZEPHYR_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/../../COMMS/drivers/rfm26w)`. No Makefile change is
needed, which keeps the Makefile portable to Windows CMD.

**Devicetree binding** (`hoperf,rfm26w`, a child of the SPI bus): required `int-gpios` (nIRQ, must be
`GPIO_ACTIVE_LOW`) and `sdn-gpios` (must be `GPIO_ACTIVE_HIGH`), optional `tx-ant-gpios` and `rx-ant-gpios` (must
be active high), `block-length` (default 223, asserted 1 to 65535 because TX_LEN and RX_LEN are 16-bit),
`xo-frequency-hz`, `xo-tcxo`. The polarity rules are enforced by `BUILD_ASSERT` and an init check. Adding a chip
select to the board's SPI node means restating the whole `cs-gpios` list.

**Kconfig:** `CONFIG_RFM26W` (default y, depends on the node being enabled), `_INIT_PRIORITY` (80), `_LOG_LEVEL`,
`_CTS_TIMEOUT_MS`, `_POWER_UP_TIMEOUT_MS`, `_STARTUP_DELAY_MS`, `_VERIFY_PROFILE`, `_PA_LEVEL_MAX`, `_TX_THRESHOLD`,
`_RX_THRESHOLD`, `_WORKQ_STACK_SIZE`, `_WORKQ_PRIORITY` (default 1).

**API conventions** (follow `include/zephyr/drivers/lora.h`): a driver API struct of function pointers and a
`static inline` wrapper per operation. Internal functions are prefixed `rfm26w_drv_` so they do not clash with the
wrappers. This out-of-tree API class uses a plain `static const struct rfm26w_driver_api` and `dev->api`, not
`DEVICE_API()`. Optional operations return `-ENOSYS`, an operation already in flight returns `-EBUSY`, errors are
negative errno. With the node disabled, `CONFIG_RFM26W` is off and an app that references the device fails to
compile, which is intended. The driver never allocates memory. The data rate is chosen by profile
(`enum rfm26w_profile`), not by free parameters.

**Locking and threads.** The nIRQ GPIO callback only submits a `k_work` to the driver's own work queue thread
(`rfm26w`), because the FIFO deadlines must not wait behind other work. `data->lock` (reentrant) is held for each
SPI command sequence and by the work handler, never while waiting for a block to finish. `send()` writes the first
64 bytes and `START_TX`, releases the lock, and waits on a semaphore that the handler gives on `PACKET_SENT` or on an
error. One `send()` at a time (`tx_lock`). The work handler loops while nIRQ is low, because the GPIO interrupt is
edge triggered but the chip holds the line low while anything is pending. Each pass reads `GET_INT_STATUS` (which
also clears) and acts on the pending bits. The radio is left in shutdown after init and the interrupt stays disabled
until `rfm26w_start()`.

**FIFO streaming.** The FIFOs are 64 bytes each (`FIFO_MODE` can share them as one 128-byte buffer, still smaller
than the 223-byte block). The driver refills the TX FIFO on the almost-empty interrupt and drains the RX FIFO on the
almost-full interrupt. `PKT_TX_THRESHOLD` and `PKT_RX_THRESHOLD` (0 to 64) are written by the driver
(`CONFIG_RFM26W_TX_THRESHOLD`/`_RX_THRESHOLD`, default 48 each), leaving 16 bytes of slack: about **13 ms at 9.6 kb/s**,
2.6 ms at 50 kb/s. The radio interrupt handling therefore runs at the highest priority.

**Properties the driver owns.** After every profile load (start, and `configure()` when the profile changes) the driver
rewrites the FIFO thresholds, the interrupt enables and reads `MODEM_RSSI_COMP` (`rfm26w_apply_driver_props`),
because the sample WDS output switches every interrupt off. Interrupts enabled: packet sent, packet received, TX FIFO
almost empty, RX FIFO almost full, sync detect, FIFO under/overflow.

**Configure.** The carrier frequency passed to `rfm26w_configure()` overrides the frequency inside the profile.
If the radio is listening it is stopped around the change and started again, even when the change fails (a block that
is arriving is lost). It returns `-EBUSY` while a block is being transmitted.

**Receive.** The first `recv()` or `recv_async()` runs `START_RX` with `RX_LEN` = block length and all three next
states = RX, and the radio then keeps listening (arming again while armed does nothing, because START_RX would
destroy a block that is arriving). One block is kept for the next `recv()`. A second block arriving before that is
read out of the FIFO and dropped, counted in `rx_overruns`. After each block the driver asks `REQUEST_DEVICE_STATE`
and restarts receive if the chip left RX, because how the chip reuses `RX_LEN` across the RX-to-RX transition is
unverified. `recv()` and `recv_async()` are mutually exclusive (`-EBUSY`). `recv_async(NULL)` cancels only an
asynchronous receive. The callback runs in the radio thread with the lock released, must not block, and must not call
`rfm26w_send()`. The block pointer is valid only during the call.

**Send.** `send()` waits for a block that is already coming in (sync word seen, block not complete), up to the
timeout, then returns `-EBUSY` (open item 22). There is no collision avoidance on the air. The return to RX after a
send is done in software (`TXCOMPLETE_STATE` = Ready, then `START_RX` from the handler), not by `TXCOMPLETE_STATE` =
RX, so it does not depend on the chip remembering the earlier `RX_LEN`. This costs a little turnaround time. Try the
chip's own return to RX on hardware if the turnaround matters.

**RSSI.** The latched RSSI is read in the sync-detect interrupt, because the chip resets it at the start of the next
RX. This assumes the profile's `MODEM_RSSI_CONTROL` latches by the time sync is detected. The dBm conversion
`raw / 2 - MODEM_RSSI_COMP - 70` is **not in the provided documents** (open item 23), reported in tenths of dBm.

**TX power is a placeholder.** `rfm26w_set_tx_power()` maps the datasheet table 4 range (-40 to +11 dBm) straight
onto `PA_PWR_LVL` 0 to 127, which is not the real curve and has not been measured. What limits the output for now is
`CONFIG_RFM26W_PA_LEVEL_MAX` (default 32), a cap on the raw level. Raise it only after measuring (open item 17).

**Antenna switch.** If `tx-ant-gpios` and `rx-ant-gpios` are present they are driven to opposite levels (RX state:
RX-ANT low, TX-ANT high, TX state the reverse, from the datasheet pin table). The datasheet pin text mentions
"RFM24", so the polarity needs confirming on the board (open item 17).

**Frequency math** (`rfm26w_priv.h`): `ratio = F * outdiv / (2 * Fxo)`, `INTE = floor(ratio) - 1`,
`FRAC = floor((ratio - INTE) * 2^19)`, with bands 850 to 1050 MHz divider 4, 425 to 525 divider 8, 283 to 350 divider
12, 142 to 175 divider 24. This is **not in the provided documents** (open item 25). It reproduces one WDS header
exactly (433.92 MHz, 30 MHz clock: `INTE` 0x38, `FRAC` 0x0ED916, `MODEM_CLKGEN_BAND` 0x0A).

**Profile tables.** A table is the WDS `RADIO_CONFIGURATION_DATA_ARRAY` layout: `len, cmd, args...`, ending in a
zero byte. The driver sends each entry (skipping `POWER_UP`, because it powers up from the devicetree, and warning if
that entry's clock disagrees with the devicetree) and, with `CONFIG_RFM26W_VERIFY_PROFILE`, reads every
`SET_PROPERTY` range back. `start` fails with `-EIO` on a mismatch. `rfm26w_profile_table()` is `__weak` so tests can
supply a small table. Production builds use the driver's own (placeholder) table. Do not make a second table in the
firmware.

**Power management.** SUSPEND would map to SDN high and RESUME to SDN low plus reconfiguring. The same operations are
`rfm26w_shutdown()` and `rfm26w_start()`, which work without `CONFIG_PM_DEVICE` and double as the UNP12-91 hook.

**Emulator tests** (`apps/tests/rfm26wEmulTest`, `make -C apps/tests/rfm26wEmulTest test`).
- `drivers/rfm26w/emul/rfm26w_emul.c` is a software model of the radio on Zephyr's SPI and GPIO emulators, bound to
  the same devicetree node as the driver (the node label `rfm26w0` matters). It was written from AN625 and the
  datasheet **without sharing constants with the driver**, so one misreading is unlikely to be in both. It models the
  CTS handshake, properties, both FIFOs, the interrupt registers and the nIRQ pin, the TX and RX state machine, a
  byte clock (833 us per byte is 9.6 kb/s), the latched RSSI being cleared at the start of each RX, and SDN
  resetting the chip. After every test it checks the driver never sent a command before CTS, never sent a bad
  command, and never set the antenna pins wrongly.
- The 27 tests cover start and shutdown, profile loading and read-back, CTS timeout and slow CTS, SPI errors,
  `configure` and the TX power cap, send streaming, send timeout, TX underrun, receive (callback and blocking),
  back-to-back blocks, the one-block hold and drop, RX overflow recovery, the chip leaving RX, send during a
  reception, RSSI timing, and shutdown waking blocked callers.
- Checked by breaking the driver on purpose: five deliberate bugs were each caught (interrupt-clear arguments of
  `FF`, which leaves every interrupt pending; RSSI read from the current instead of the latched register; a TX
  refill that loses a byte; `START_RX` next state Ready instead of RX; no state check after a block).
- native_sim defaults to 10 ms ticks, so the test apps set `CONFIG_SYS_CLOCK_TICKS_PER_SEC=10000`. With the default,
  the driver's 1 ms CTS-poll sleeps become 10 ms and four tests fail for that reason alone.
- **What the model cannot show:** how the real chip behaves. Every place where the model and the chip could differ
  is in section 5.5. Only the simulated timing is real, so the tests show the logic is right, not that the few-ms
  FIFO deadline will be met on the STM32U5 (checklist item 9).

### 5.3 Radio programming notes (RFM26W)

**Interface** (datasheet section 3, AN625): SPI up to 10 MHz plus `nSEL`, `nIRQ` and `SDN`. A command interface, not
a register map. Every command is sent, then the host polls for CTS (`0xFF`, about 20 µs) before reading the reply or
sending the next command. Settings are properties set with `SET_PROPERTY`.

| Command | Code | Use |
|---|---|---|
| `PART_INFO` | `0x01` | Part number (for example `0x4463`, **verify** which) |
| `POWER_UP` | `0x02` | Boot options and crystal frequency |
| `SET_PROPERTY` / `GET_PROPERTY` | `0x11` / `0x12` | Write and read back configuration |
| `GPIO_PIN_CFG` | `0x13` | Configure GPIOs |
| `FIFO_INFO` | `0x15` | FIFO counts, FIFO reset |
| `GET_INT_STATUS` | `0x20` | Interrupt status (clears nIRQ) |
| `GET_MODEM_STATUS` | `0x22` | Latched RSSI. Takes no arguments (AN625 3.2.14) |
| `START_TX` / `START_RX` | `0x31` / `0x32` | Start a transmission or reception, set next state |
| `REQUEST_DEVICE_STATE` | `0x33` | Current state and channel |
| `CHANGE_STATE` | `0x34` | Move between states |
| `READ_CMD_BUFF` | `0x44` | Poll CTS and read the reply of the last command |
| `WRITE_TX_FIFO` / `READ_RX_FIFO` | `0x66` / `0x77` | Stream block bytes. Neither uses CTS |

**CTS procedure** (AN625 3.2.20, datasheet 3.1): send the command and its arguments in one chip-select window,
release `nSEL`, then clock `READ_CMD_BUFF` (`0x44`) with `nSEL` low. The first byte back is meaningless, the second is
CTS. `0xFF` means ready and the reply bytes (if any) follow in the same window. Any other value means try again. The
driver does this as one SPI transfer of `2 + reply length` bytes per poll, with a timeout. Reply layouts used (after
CTS): `PART_INFO` = chip revision, part (2, big-endian), part build, ID (2), customer, ROM ID. `FIFO_INFO` = RX count,
TX space. `GET_MODEM_STATUS` = pending, status, current RSSI, **latched RSSI**, antenna 1 RSSI, antenna 2 RSSI,
AFC offset (2). `GET_INT_STATUS` = INT_PEND, INT_STATUS, PH_PEND, PH_STATUS, MODEM_PEND, MODEM_STATUS, CHIP_PEND,
CHIP_STATUS. Its arguments are clear masks where a **0 clears** that pending interrupt and a 1 leaves it pending, so
"clear everything" is three zero bytes. (A sample project that used `FF FF 7F` was not clearing anything.)
`START_TX` condition byte = `TXCOMPLETE_STATE` in bits 7:4. `START_RX` next states default to Ready, so continuous
receive needs `RXVALID_STATE` and `RXINVALID_STATE` set to 8 (RX). Each of `START_TX` and `START_RX` takes next-state
fields so the radio can return to RX by itself. Ready to TX or RX takes about 100 µs.

- **Receiving the fixed block:** a nonzero `RX_LEN` in `START_RX` makes the radio receive with field 1 only and
  without packet handler features (no CRC, no whitening). Set it to 223.
- **Configuration comes from the WDS calculator** (Silicon Labs' settings tool). Whether it is usable in the build,
  or the properties are written by hand from AN625, is open item 20.
- **Sync word bit order (verify on the air):** AN625 says the preamble and sync word are always sent least
  significant bit first, regardless of `PKT_CONFIG1.BIT_ORDER`, and its own text has TODO notes around it. The CCSDS
  marker must reach the air as `1A CF FC 1D`, most significant bit first. The value programmed into `SYNC_CONFIG` and
  `SYNC_BITS_*` may therefore need each byte bit-reversed and the byte order swapped. Confirm with a second receiver
  before relying on it. Set `SYNC_CONFIG.LENGTH` to 32 bits.
- **Manchester:** clear it (`MODEM_MAP_CONTROL` enables it by default).
- **Preamble:** `PREAMBLE_TX_LENGTH` 16 with `LENGTH_CONFIG` in bytes, standard pattern `0101`.
- **Pins:** `TX-ANT` and `RX-ANT` must be driven to opposite levels. A radio GPIO can be configured as the switch
  control. `SDN` shuts the radio down completely and loses the configuration.
- **Zephyr has no driver** for the Si446x or RFM26W as of v4.4.0-rc3 (no driver or binding; a third-party driver
  outside the tree was not searched for), so this one is custom. CAN, PSA crypto, CRC and MCUboot are reused.
- **CRC:** use `crc16_itu_t(0xFFFF, buf, len)`. Not `crc16_ccitt()`, which is the reflected variant.

### 5.4 Regenerating the radio profile

Do this whenever the reference clock changes (`xo-frequency-hz` or `xo-tcxo`), or the data rate, deviation, bandwidth
or frame settings change. The WDS table bakes in the reference clock (data-rate dividers, filter coefficients, PLL
settings), so **the table and the devicetree value must always describe the same clock**. At start the driver warns
if the table's own `POWER_UP` entry disagrees with the devicetree.

1. Settle the real crystal or TCXO frequency and tolerance, from the module's documentation or by measuring it. Set
   `xo-frequency-hz` (and `xo-tcxo;` for a TCXO) in the board's devicetree node.
2. Open Silicon Labs' WDS tool (Wireless Development Suite, Windows). The sample header was made with WDS 3.2.11.0 for
   a Si4463 Rev B1. Pick the device that matches what `PART_INFO` reports on the board. Enter the same inputs that
   header lists in its "INPUT DATA" comment: crystal frequency and tolerance, modulation (2GFSK), symbol rate,
   deviation, receive bandwidth, Manchester off, AFC on or off, and the carrier frequency. Then set the packet side to
   our frame format: 16-byte preamble, 32-bit sync word `0x1ACFFC1D` (bit-order caveat in section 5.3), CRC off,
   whitening off, a fixed block length of 223.
3. Export the C header (`radio_config_*.h`). Keep the exported file, or at least its INPUT DATA comment, in the
   repository (for example `drivers/rfm26w/profiles/`) so the inputs can be reproduced.
4. Copy `RADIO_CONFIGURATION_DATA_ARRAY` into the matching table in `src/rfm26w_profiles.c`, keeping the
   `len, cmd, args...` layout and the `0x00` terminator. Do not edit the numbers by hand. A `POWER_UP` entry may stay
   in, it is skipped.
5. Rebuild and start the radio. With `CONFIG_RFM26W_VERIFY_PROFILE` on, `rfm26w_start()` reads every property back and
   fails with `-EIO` if one differs. Watch for the "profile was generated for XO ..." warning.
6. The carrier frequency does not need WDS: `rfm26w_configure()` computes it from the devicetree clock and overrides
   the table. If the clock changed, the modem settings in the table still depend on it, so steps 2 to 4 are still
   needed.
7. Confirm on the bench (phase 6): a second receiver, a spectrum analyzer or the ground radio shows the expected
   carrier frequency, deviation and data rate.

The exact menu names in WDS are not documented here. These steps follow the input fields in the sample header, so
whoever does this first should correct this section.

### 5.5 Hardware checklist (radio driver)

Nothing in the driver has run on a radio. These can only be settled on the bench. Do them in this order and write the
result next to each one.

1. **Part and CTS.** `PART_INFO` returns the expected part number (0x4463?) and chip revision. Note the revision,
   because the WDS output differs between revisions. CTS arrives within the 20 ms timeout, and `POWER_UP` within 100 ms.
2. **Reference clock.** Measure the carrier with a spectrum analyzer or a second radio, with a known frequency
   configured. If it is off by a fixed ratio, the crystal assumption (open item 24) is wrong.
3. **Profile read-back.** `rfm26w_start()` with the real WDS table returns 0 (every property reads back).
4. **Sync word.** A second receiver sees `1A CF FC 1D` on the air (open item 13), not a bit-reversed word.
5. **Frequency equation.** Carrier is where `rfm26w_configure()` says, in each band the module supports (open item 25).
6. **Antenna switch.** TX-ANT and RX-ANT levels in RX and in TX match the datasheet pin table (open item 17).
7. **Send.** A 223-byte block with a counting pattern arrives intact on a second receiver. Repeat 1000 times.
   `tx_underruns` stays 0. Try the thresholds at the chosen data rate.
8. **Receive.** The same from the ground side. `rx_overruns` stays 0. Receive blocks back to back with the shortest gap
   the transmitter can produce, and check none are lost or split.
9. **Streaming deadline.** At the chosen data rate, with the other COMMS threads busy (CAN, crypto), no FIFO overflow
   or underrun. Check `CONFIG_RFM26W_WORKQ_PRIORITY` against the application thread priorities.
10. **RX to RX.** After a block the radio is still in RX with the right `RX_LEN`. Count how often the
    `REQUEST_DEVICE_STATE` check has to restart receive. If never, the check can go.
11. **TX to RX turnaround.** Measure the time from `PACKET_SENT` to receive running again. Compare with letting the
    chip return to RX by itself (`TXCOMPLETE_STATE` = RX).
12. **RSSI.** Feed a known signal level. Check the latched RSSI at sync detect and the dBm conversion (open item 23),
    and that `MODEM_RSSI_CONTROL` in the profile latches early enough.
13. **TX power.** Measure the output at several `PA_PWR_LVL` values and replace the placeholder mapping (open item 17).
    Only then raise `CONFIG_RFM26W_PA_LEVEL_MAX`.
14. **Send during a reception.** Start a send while a block is arriving. The send waits, then returns `-EBUSY` on
    timeout, and the received block is not damaged.
15. **Shutdown.** `rfm26w_shutdown()` during a send or a receive wakes the waiting call with `-ESHUTDOWN`, and
    `rfm26w_start()` afterwards works again.
16. **Unplugged radio.** `rfm26w_start()` returns `-ETIMEDOUT` and leaves the radio in shutdown.

## 6. Open items

Decisions and unknowns, numbered (code comments cite these numbers).

1. **Storage** for the replay counter, CFDP state and staging area (PROTOCOL.md section 5). Staging options: the
   COMMS second flash slot (4 MB part, **verify**), external flash, or other-board storage.
2. GROUND_ID value and the uplink allowlist (with CDH).
3. Callsign placement and the ID interval required by the license (secondary header on every downlink packet vs
   periodic ID). If the header is required, the MVP includes it and the downlink data field is 206 bytes instead of
   213. Beacon period and exact contents.
4. APID table and CFDP entity IDs.
5. Key provisioning, rotation, and key-slot protection. Scope of UNP12-92: confirm whether the program documents say
   anything about the algorithm, key management, or whether bulk firmware data and the beacon are in or out of
   scope. (Key size is decided: 256-bit.)
6. Counter resync procedure if ground loses its IV state.
7. UHF data rate (9.6 kb/s proposed), deviation, channel bandwidth, AFC and Doppler margin, and frequency
   coordination for the chosen bandwidth. Public protocol documentation.
8. S-band (2.4 GHz): the radio is undecided, so modulation, framing and rates for payload images and other mission
   data are open. The RFM26W cannot do S-band.
9. Reed-Solomon (255,223) with the CCSDS randomizer: MCU encoder/decoder and C library licensing check.
10. COMMS-to-board firmware transfer protocol over CAN (PROTOCOL.md section 7): chunk and ACK design, or CFDP over a
    CAN transport. Confirm each board has flash or FRAM for a second image slot and can verify signatures itself. The
    CAN bus bit rate is not yet checked.
11. TC frame managed parameters: CCSDS spacecraft ID (10 bits, assigned by CCSDS; placeholder for now), the uplink
    virtual channel ID, and whether the spacecraft ID goes in the SDLS authentication mask (proposed yes).
12. Downlink framing: default custom block vs the TM Transfer Frame alternative (PROTOCOL.md section 10).
13. Confirm RFM26W behavior on hardware: the sync word bit and byte order, `RX_LEN` receive with the expected bit
    order, FIFO streaming at the chosen rate, RX/TX turnaround (section 5.3).
14. Confirm the receive path drops Type-AD and Type-C frames and that nothing relies on COP-1 or FARM.
15. Direct routing: the state-gating policy table with CDH, agreement from each board that it accepts `src` =
    GROUND_ID, and reply addressing (`dst` = GROUND_ID).
16. **UNP12-91, cease transmission on command:** which command, which layers it must survive (it should not depend
    on CFDP or an open transfer), whether it also needs a timeout or ground "resume", and whether CDH or EPS can
    enforce it in hardware as a backstop. The radio's `SDN` pin is one candidate.
17. **TX power and output stage.** The datasheet contradicts itself: +20 dBm maximum in the features list, -40 to
    +11 dBm in table 4. Confirm the real range of the module (and that "minimum" is selectable for the MVP), and
    confirm the `TX-ANT`/`RX-ANT` switch pin polarity.
18. **Module band variant.** The RFM26W is sold per band (the ordering example is `RFM26W-433-S2`). Confirm which
    variant is on the board, because it limits the usable frequencies, including the bench frequency.
19. **Ground radio hardware.** Not known yet. It is attached to a Raspberry Pi. The ground software is custom.
20. **WDS workflow** for the RFM26W (sections 5.3 and 5.4): whether the WDS tool is usable in the build or the
    properties are written by hand from AN625. The driver is written and waits for the real profile table.
21. AX.25 vs CCSDS framing, revisit if direct compatibility with standard amateur 9k6 stations matters (PROTOCOL.md
    section 13).
22. **`rfm26w_send()` during a reception.** Implemented as: wait for a block already being received, then give up
    with `-EBUSY`. Decide whether that is right, or whether sending should abort the reception, and how the
    application avoids collisions (the link has no collision avoidance).
23. **RSSI conversion.** The formula from the chip's latched RSSI value to dBm is not in the datasheet text that was
    read. Find it in AN625 or the Si446x documents. The driver uses `raw / 2 - MODEM_RSSI_COMP - 70` (from memory of
    Silicon Labs material), so it is unverified.
24. **Reference clock of the RFM26W module (crystal frequency, or TCXO).** The driver ASSUMES a 30 MHz crystal. Not
    confirmed for the module on the board. A wrong value is not detected by the chip and shifts the carrier frequency
    and data rate. If it is not a 30 MHz crystal, change the devicetree and regenerate the radio profile (section
    5.4). The module datasheet only says the chip's default crystal is 30 MHz and 25 to 32 MHz is allowed.
25. **Frequency equation and band table** in `rfm26w_priv.h` come from outside the provided documents. Verify them
    against the Si446x datasheet (section 5.2).
26. **MVP CAN_CMD:** which board is the CAN target, which harmless request-and-reply command is used, and what is on
    the allowlist.
27. **MVP pass thresholds** (99% of 1000 pings, 10 minutes of telemetry) are proposals, kept as drafted for now.
