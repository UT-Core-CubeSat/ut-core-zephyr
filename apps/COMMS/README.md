See <u>Files</u> above for function documentation.

Back to [Contents](@ref index)

# COMMS - Communications firmware

COMMS is the UT-CORE CubeSat's communications board: an STM32U5 running Zephyr, with a UHF radio (RFM26W) to the
ground station and the shared CAN bus to the other boards (CDH, EPS, ADCS and so on). It is the satellite's only
link to the ground. It receives and authenticates uplinked commands and firmware, puts accepted commands on the CAN
bus addressed to the target board, and sends telemetry, a beacon, file-transfer replies and (later, on S-band)
payload data down.

> **Read this file first.** It is the overview. [PROTOCOL.md](PROTOCOL.md) is the wire format and the reasons
> behind it. [PLANNED.md](PLANNED.md) is the build plan: what is done, the stages, what to do next, the MVP bench
> test, the radio driver notes and the open decisions.

## Status (2026-11-09)

- **Firmware: stage 3a of 7 done.** The link-layer codecs, the pipeline against the radio driver (PING) and the CAN_CMD
  path (uplink allowlist, CAN bridge, CAN ping and pong to CDH, board answers back down on `0x020`) are written and
  tested on the host, end to end through the radio model with a fake CAN bus. `src/main.c` starts the radio and the
  `uplink`, `radio_tx` and `can_proc` threads. CDH has a new `OP_PING` handler. None of it has run on hardware. Next is
  stage 3b: telemetry cache, TLM, beacon, scheduler, watchdog ([PLANNED.md](PLANNED.md) section 3).
- **Radio driver: complete enough for the MVP**, tested against a software radio model. The continuous-wave test and
  PM action are stubs. Its profile table is an empty placeholder until Silicon Labs WDS output exists.
- **Tests:** 51 firmware host tests and 27 radio driver tests, all passing on `native_sim`.
- **Hardware does not exist yet** (at least 2 months away). Nothing has run on a real board or radio. The pins, CAN
  setup and `prj.conf` hold placeholder values copied from CDH. Every placeholder and unconfirmed assumption is
  listed in [PLANNED.md](PLANNED.md) section 4, "Placeholders and assumptions".
- **The 30 MHz radio crystal is an assumption.** A wrong reference clock silently shifts the carrier and data rate
  (PLANNED.md section 5, open item 24).

## Hardware

- **UHF radio:** HopeRF RFM26W module (Si446x-class), SPI plus `nIRQ` and `SDN`, GFSK. Datasheet and AN625 are in
  `external_documents/`. The module is sold per band, so the variant on the board limits the usable frequency.
- **S-band radio:** not yet decided. The RFM26W is sub-GHz only. Payload images are meant to go down on S-band.
- **MCU:** STM32U5 (U5A5), Zephyr RTOS. **CAN:** FDCAN to the shared bus, `common/can_proto.h` defines the 29-bit ID
  layout and node IDs (CDH `0x01`, EPS `0x02`, COMMS `0x03`, others up to `0x08`). The ground station appears on the
  bus as `GROUND_ID` (`0x10`, in `common/can_proto.h`).
- **Storage:** no FRAM is assumed. COMMS has its own flash for staging uploads. The rest of the storage split is
  undecided.
- **Ground station:** a radio on a Raspberry Pi, details unknown.

## How the link works (summary)

The full spec is [PROTOCOL.md](PROTOCOL.md).

```
 application   CAN records (commands, telemetry, beacon)  |  CFDP PDUs (firmware, images)
 packet        CCSDS Space Packet (6-byte header, APID selects the contents)
 security      SDLS header + AES-GCM + MAC          (uplink only, inside the TC frame)
 frame         uplink: TC Transfer Frame, Type-BD   |  downlink: fixed block
 link block    fixed 223 bytes, zero fill
 PHY           RFM26W GFSK: preamble + sync word + block
```

- UHF is half-duplex and carries commands and firmware up, and telemetry, beacon and file-transfer replies down.
  Payload images never go down on UHF. The uplink is encrypted and authenticated. Downlinks are unencrypted and public.
- COMMS puts each accepted command record on the CAN bus addressed **directly** to the target board with
  `src` = GROUND_ID. CDH listens and gates by state but is not a relay. COMMS drops commands the system state forbids.
- Firmware for other boards is staged in COMMS and pushed over CAN. The target board verifies the signature itself.
- Every block is exactly 223 bytes, the data size of a Reed-Solomon (255,223) codeword, so adding Reed-Solomon later
  changes only the link block layer.

## Layout

```
apps/COMMS/
  src/              the firmware. main.c is the entry point (start here), board_config.h is hardware,
                    comms_config.h is protocol constants, link/ and can/ are the tested modules,
                    modules.cmake is the module list shared with the host tests
  drivers/rfm26w/   the radio driver, a Zephyr out-of-tree module: include/, src/, dts/, emul/ (software radio model)
  boards/           devicetree overlay for the ut_core board (placeholder radio node)
  external_documents/  reference specs and datasheets, git-ignored (see "External documents" below)
apps/tests/commsTest/       host tests for the firmware (native_sim)
apps/tests/rfm26wEmulTest/  host tests for the radio driver (native_sim)
apps/tests/rfm26wTest/      bring-up app for the real radio (target)
```

## External documents

`apps/COMMS/external_documents/` is git-ignored, so a fresh clone does not have it. The code and docs cite these
files by name (for example "`ccsds.txt` section 4.1.3.2"). To rebuild the folder, download the PDFs, convert the
CCSDS ones to text (for example `pdftotext -layout file.pdf file.txt`), and save them under the names below.
The CCSDS Blue Books are free. The editions are the ones the section numbers in the docs were checked against.

| File | Document | Used for | Source |
|---|---|---|---|
| `ccsds.txt` | CCSDS 133.0-B-2, Space Packet Protocol (June 2020) | Space Packet header, APIDs | https://ccsds.org/Pubs/133x0b2e2.pdf |
| `space_data_link_protocol.txt` | CCSDS 232.0-B-4, TC Space Data Link Protocol (Oct 2021, with Technical Corrigendum 1) | Uplink TC Transfer Frame, Type-BD | https://ccsds.org/Pubs/232x0b4e1c1.pdf |
| `ccsds_sdlp.txt` | CCSDS 132.0-B-3, TM Space Data Link Protocol (Oct 2021) | Downlink framing reference | https://ccsds.org/Pubs/132x0b3.pdf |
| `ccsds_tm.txt` | CCSDS 131.0-B-5, TM Synchronization and Channel Coding (Sept 2023) | Reed-Solomon (255,223) and sync reference | https://ccsds.org/Pubs/131x0b5.pdf |
| `ccsds_sdls.txt` | CCSDS 355.0-B-2, Space Data Link Security Protocol (July 2022) | SDLS header, AES-GCM, replay (stage 4) | https://ccsds.org/Pubs/355x0b2.pdf |
| `ccsds_cfdp.txt` | CCSDS 727.0-B-5, CCSDS File Delivery Protocol (July 2020) | File transfer for firmware and images (stage 5) | https://ccsds.org/Pubs/727x0b5e1.pdf |
| `RFM26WDatasheet_1695367537.pdf` | HopeRF RFM26W ISM Transceiver Module datasheet, V1.1 | Module pinout, bands, electrical limits | HopeRF website, product page for RFM26W (no direct link checked) |
| `AN625.pdf` | Silicon Labs AN625, Si446x API Descriptions | Radio commands and properties used by the driver | Silicon Labs website, search "AN625" (no direct link checked) |

The CCSDS links were checked on 2026-11-06 and may move. If one breaks, the publications list is at
https://ccsds.org (Publications, Blue Books).

## Build, test, flash

All from the repository root unless a directory is given. Host tests need Linux or WSL.

| Command | What it does |
|---|---|
| `make -C apps/COMMS build` | Build the firmware for the ut_core board |
| `make -C apps/COMMS test` | Run all host tests relevant to COMMS (firmware, then radio driver). Stops on the first failure |
| `make test APP=apps/tests/commsTest` | Build and run one host test app. The exit code is the test result |
| `make test-full` | Run every host test app |
| `make -C apps/COMMS flash` / `debug` / `gdb` / `listen` / `sim` / `help` | As for the other boards. `make help` lists them |
| `make build APP=apps/tests/rfm26wTest` | Build the radio bring-up app for the board |
| `west twister -T apps/tests/commsTest -p native_sim/native/64` | Zephyr's own runner, uses each test app's `testcase.yaml` |

Run a single test or suite: `build/apps/tests/commsTest/zephyr/zephyr.exe -test=tc_frame::test_round_trip_every_length`.

Things that bite:
- native_sim runs in simulated time. The test apps set `CONFIG_SYS_CLOCK_TICKS_PER_SEC=10000`, because the default
  10 ms tick turns 1 ms sleeps into 10 ms ones.
- Every `.c` file in `apps/tests/commsTest/src/` is a test file (one suite per file, picked up by a glob).
- The firmware and the host tests compile the same module list, `src/modules.cmake`. Add a module there, not in the
  two `CMakeLists.txt` files.

## Firmware architecture

**Start reading at `src/main.c`**, as in the other apps. Its header comment is the overview: the hardware, the
priority-ordered thread table, the data flow, the init order and the stage status. The thread bodies there are a few
lines each. They move data between queues and call into a module. The logic itself is in the modules under `src/`,
which is the one place this app differs from the single-`main.c` apps: the logic is split into files so it can be
unit-tested on a computer.

**Rules** (each exists because it makes the code testable on a computer with no radio):
1. Protocol logic lives in modules that take buffers and return codes, with no threads and no globals. Tests call
   them directly. Threads only move data between modules.
2. Every module returns distinct error codes, so statistics can say why a block was dropped.
3. Every protocol placeholder lives in `src/comms_config.h`, nowhere else. `board_config.h` holds hardware only.
4. The radio is used only through `ut/drivers/rfm26w.h`. The receive callback must not block and must not call
   send. `radio_tx` will be the only caller of send.
5. `main.c` is the entry point: the Doxygen group, the overview comment, the threads and queues, and the init order.
   It contains no protocol logic.

**Modules** ("done" means written and tested on the host, not run on hardware):

| Layer | Module (under `src/`) | Does | Stage | State |
|---|---|---|---|---|
| link | `link/crc16` | CRC-16/CCITT-FALSE, one routine for uplink FECF and downlink CRC | 1 | done |
| link | `link/tc_frame` | Parse and build the uplink TC frame, Type-BD only, separate error per drop reason | 1 | done |
| link | `link/space_packet` | Space Packet header, per-APID sequence counters, loss count | 1 | done |
| link | `link/dl_block` | Downlink block: `len`, packet, pad, CRC | 1 | done |
| can | `can/can_record` | The 12-byte CAN-equivalent record, conversion to and from `struct can_frame` | 1 | done |
| app | `app/radio_port`, `uplink`, `router`, `downlink`, `ping` | Radio to packet to APID dispatch, TX queue, ping | 2 | done |
| can | `can/can_bridge`, `uplink_policy`, `can_ping` | CAN send and receive filter, who may command what (default-deny allowlist), CAN ping and pong records | 3a | done |
| can | `can/telemetry`, beacon | Telemetry cache, TLM and beacon builders | 3b | todo |
| sec | `sec/sdls`, `replay` | AES-GCM frame security, IV replay window | 4 | todo |
| cfdp | `cfdp/cfdp_pdu`, `cfdp_rx`, `staging` | File transfer receive, storage interface | 5 | todo |

**Data flow (target design, stages 2 to 3):**
```
Uplink   radio --block--> radio_port --> uplink: tc_frame_parse [stage 4: sdls + replay] --> sp_parse
                                           --> router (by APID) --> 0x010 commands: uplink_policy --> can_bridge --> CAN
                                                               --> 0x0F0 ping --> downlink queue
Downlink CAN telemetry --> telemetry cache --> beacon/TLM builders --\
         ping reply, CFDP replies ----------------------------------- +--> sp_build --> dl_block_pack --> radio_tx --> radio
```

**Threads.** Priorities and sizes are in `board_config.h`. The radio driver adds its own work-queue thread
(`CONFIG_RFM26W_WORKQ_PRIORITY`, default 1) that runs FIFO streaming and the receive callback.

| Thread | Does | Stage | Exists |
|---|---|---|---|
| `heartbeat` (priority 8) | Logs once a second | 0 | yes |
| `uplink` (priority 5) | Takes a received block from the queue, frame to packet to router, queues any reply | 2 | yes |
| `radio_tx` (priority 6) | Sole owner of send: packet to block to radio | 2 | yes |

There is no `radio_rx` thread. The driver's receive callback runs in the driver's work queue and copies each block
straight into the uplink queue (priority 4 stays unused).
| `can_proc` (priority 2) | Frames addressed to ground (board answers) to the downlink queue | 3a | yes |
| `beacon`, `scheduler`, `watchdog` | Periodic beacon, statistics, hardware watchdog | 3b | no |

There is no `can_rx` thread either: the CAN driver's filters already buffer frames into the queue `can_proc` reads.

## Radio driver (summary)

`drivers/rfm26w/` is a Zephyr driver with its own small API (`include/ut/drivers/rfm26w.h`, the source of truth).
It moves fixed 223-byte blocks (`block-length` in the devicetree), streams them through the chip's 64-byte FIFOs from
the nIRQ handler in its own work queue thread, reads the latched RSSI per block, counts errors, and serializes all
calls with a mutex. Blocking `rfm26w_send()` and `rfm26w_recv()`, plus an asynchronous receive callback. The chip's
configuration comes from a Silicon Labs WDS table (`src/rfm26w_profiles.c`) that must match the reference clock in
the devicetree. Design notes, the profile regeneration procedure and the hardware checklist are in
[PLANNED.md](PLANNED.md) section 5. The software radio model used by the tests is `drivers/rfm26w/emul/`.

## Software requirements (original, Cason 2026-10-09)

Where each stands now (details in [PLANNED.md](PLANNED.md)):

| Requirement | Status |
|---|---|
| Transmit and receive data (UHF to ground, payload images over S-band) | UHF path designed and partly built (driver, codecs). S-band undecided |
| Collect data frames from uplink, deserialize, handle partial/dropped broadcasts | Stage 2 (parsing), stage 5 (CFDP retransmission of dropped data) |
| Collect data frames from CDH output to RF (telemetry, SOH) | Stage 3 (telemetry cache, TLM) |
| Package SOH data and send to CDH | Not started. Should resemble what GNSS and ADCS already do |
| Collect data from the Payload board (a thermal camera for now) and send via S-band | Not started. Needs the S-band radio decision and image buffering |
| State machine: behave according to the system state sent by CDH over CAN | Stage 7 (state gating with CDH) |
| Beacon | Stage 3 (APID `0x021`, lowest priority) |
| ACK / retransmission | Covered by CFDP Class 2 (stage 5), no custom scheme |

Decisions that were open in the original requirements and where they stand:
- **Firmware updates over the air:** yes, staged in COMMS and pushed over CAN, with an MCUboot signature check and
  test-swap with revert (PROTOCOL.md section 7). A failed COMMS self-update could mean mission failure, so COMMS is
  updated last.
- **Security:** CCSDS SDLS authenticated encryption with AES-GCM, 256-bit key, and an IV counter for replay protection
  (PROTOCOL.md section 5). This replaces the earlier HMAC and sequence-number idea.
- **Image buffering:** undecided (flash vs FRAM, lossless vs lossy compression).
- **UHF vs S-band priority:** undecided. Likely a thread per radio with a "transmitting" state that one thread checks.

## Traps

- Use `crc16_itu_t`, never `crc16_ccitt` (reflected, different result). `comms_crc16` wraps the right one.
- The common `can_setup()` installs filters only for this node's ID and broadcast. COMMS also needs a filter for
  `dst` = `GROUND_ID` (replies from boards are addressed to the ground). `can/can_bridge` adds it (stage 3a), not `common/`.
- The radio driver's profile table is `__weak` so tests can replace it. Do not make a second table in the firmware.
- The radio node label is `rfm26w0` in the firmware overlay and both test overlays. The radio model finds it by label.
- The devicetree must use `GPIO_ACTIVE_LOW` for the radio's `int-gpios`, and active high for `sdn-gpios` and the
  antenna pins. The driver checks this at build time.
- Do not edit `common/can_proto.[ch]` casually: every board links it.
- Doc references in code cite sections of PROTOCOL.md and PLANNED.md and the numbered open items. Keep those numbers
  stable when editing the docs.
