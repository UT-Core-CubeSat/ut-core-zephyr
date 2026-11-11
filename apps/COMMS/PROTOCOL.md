# COMMS Link Protocol

The protocol of record for the COMMS radio link: the wire format, the rules, and the reasons behind them. The
board overview is in [README.md](README.md). Build status, the firmware stages, the bench test (MVP) and the open
decisions are in [PLANNED.md](PLANNED.md). The bench subset of this protocol is section 14.

Status: draft. Values are proposals unless marked **(spec)**, meaning checked against the CCSDS text in
`external_documents/`. Choices that are still undecided are tagged **(proposed)** and are listed in the open items
of PLANNED.md. All multi-byte fields are big-endian.

## 1. Overview

```
 application   CAN records (commands, telemetry, beacon)  |  CFDP PDUs (firmware, images)
 packet        CCSDS Space Packet (6-byte header, APID selects the contents)
 security      SDLS header + AES-GCM + MAC          (uplink only, inside the TC frame)
 frame         uplink: TC Transfer Frame, Type-BD   |  downlink: fixed block
 link block    fixed 223 bytes, zero fill
 PHY           RFM26W GFSK: preamble + sync word + block
```

| Link | Direction | Carries |
|---|---|---|
| UHF | up | Commands, firmware upload (CFDP), NAK/ACK for S-band image downloads |
| UHF | down | Beacon, telemetry, CFDP replies. **Never payload images or bulk data** |
| S-band | down only | Payload images and other mission data (radio and framing TBD, see section 11) |

UHF is half-duplex: COMMS and the ground station take turns transmitting. Downlinks are unencrypted
and the format is public. The uplink is encrypted and authenticated.

## 2. Physical layer (UHF)

| Parameter | Value |
|---|---|
| Radio | RFM26W (Si446x-class) |
| Modulation | 2GFSK, BT 0.5 **(proposed)** |
| Data rate | 9.6 kb/s **(proposed, undecided)** |
| Deviation, channel bandwidth | Set by the radio configuration. TBD with the data rate |
| Frame on the air | `preamble | sync word | 223-byte block`. No radio length field, no radio CRC, no whitening, no Manchester coding |
| Preamble | 16 octets of `01010101` |
| Sync word | `0x1ACFFC1D` (the CCSDS attached sync marker), 32 bits, sent most significant bit first, byte `0x1A` first |
| Block length | Fixed, 223 bytes, in both directions |
| CRC-16 | CRC-16/CCITT-FALSE: polynomial `0x1021`, initial `0xFFFF`, no reflection, no final XOR. Check value for ASCII `123456789` is `0x29B1`. Computed in software. In Zephyr use `crc16_itu_t(0xFFFF, buf, len)`, **not** `crc16_ccitt()` (the reflected variant, a different result) |

### 2.1 Modulation and data rate

The RFM26W offers 2FSK, 2GFSK, 4FSK, 4GFSK and OOK/ASK, with no O-QPSK or OFDM. GFSK is the datasheet's recommended
type (best performance, cleanest spectrum), and it is used in both directions: one PHY, one framing, no
reconfiguration between receive and transmit. Data rate is 0.123 to 500 kb/s for GFSK (4GFSK to 1 Mb/s), channel
bandwidth 1.1 to 850 kHz, AFC available.

**Receiver sensitivity** (datasheet table 3, BER below 0.1%, GFSK, indicative only):

| Data rate | Sensitivity |
|---|---|
| 0.5 kb/s | -126 dBm |
| 40 kb/s | -110 dBm |
| 100 kb/s | -106 dBm |
| 125 kb/s | -103 dBm |
| 500 kb/s | -97 dBm |

Lower rate means better sensitivity (roughly 3 dB per halving, because the receive bandwidth narrows). The datasheet
claims a link budget of 146 dB at its best case. RSSI resolution is 0.5 dB, with a latched value readable after each
packet.

**The rate is undecided.** 9.6 kb/s is proposed (and used for the MVP): relaxed FIFO timing (about 13 ms of slack),
narrow bandwidth, and the best sensitivity of the practical options. Higher rates shorten transfers (section 3.1) at
the cost of sensitivity and bandwidth. 9.6 kb/s is also the common amateur cubesat rate. Commands and telemetry are
tens of bytes and fit easily. Only firmware upload is rate-sensitive, and it must be resumable across passes anyway.

**Bandwidth and Doppler:** channel bandwidth is configurable, and the selectivity figures are for 25 kHz channel
spacing. A low-orbit pass shifts the received frequency by several kHz at UHF, so the
channel bandwidth and AFC need margin, which gets harder as the rate drops. Not yet analyzed (open item 7).

### 2.2 Why these parameters

| Parameter | Reason |
|---|---|
| Sync word `0x1ACFFC1D` | The CCSDS attached sync marker, good autocorrelation. Unchanged when Reed-Solomon is added |
| 16-octet preamble | More time for receivers (including a ground receiver) to settle. AFC needs about one byte, so 16 is generous. About 3% more airtime than 8 octets at 50 kb/s |
| CRC-16/CCITT-FALSE in software | The variant CCSDS uses for its frame error-control field, so the uplink FECF and downlink CRC are one function. On the uplink the MAC already gives integrity, so the FECF is a cheap early reject. It is the only integrity check on the unencrypted downlink. The radio's own CRC options are not used, so the CRC is identical on both ends regardless of radio |
| Manchester, whitening off | Not part of the format. Manchester is **on** by default in the radio and must be cleared |
| Fixed block size | The radio could do variable length with a length field. A fixed size keeps the padding-free layout simple, fits Reed-Solomon, and needs one software CRC for both directions |
| 223 bytes | The data size of a Reed-Solomon (255,223) codeword, so adding Reed-Solomon later does not change the block layout |

## 3. Link block

Every block is exactly 223 bytes.

```
Uplink   | TC frame (section 4) .................................. | zero fill |
Downlink | len (2) | Space Packet (len bytes) | zero pad | CRC-16 (2) |
```

**Uplink:** the TC frame's own length field says where the frame ends. Bytes after it are zero fill and
are ignored. The frame's FECF is the integrity check, there is no separate block CRC.

**Downlink:** `len` is the Space Packet length in bytes. The CRC-16 covers `len` through the padding
(221 bytes) and is stored big-endian. A receiver drops a block whose CRC fails or whose `len` is above
219. One packet per block.

**Byte budget:**

| | Uplink | Downlink |
|---|---|---|
| Framing overhead | 7 (5 TC header + 2 FECF) | 4 (2 `len` + 2 CRC) |
| SDLS header + MAC | 30 (14 + 16) | 0 |
| Max Space Packet | **186** | **219** |
| Max packet data field (minus the 6-byte header; downlink minus 7 more if the callsign is carried) | 180 | 206 |

### 3.1 Airtime and throughput

UHF only (S-band is separate). Assumptions: 16-octet preamble, 4-byte sync, 223-byte block, 100% efficiency, no
turnaround. Bytes on the air per frame: 243 without Reed-Solomon (16 + 4 + 223), 275 with (16 + 4 + 255).

| | 9.6 kb/s | 19.2 kb/s | 50 kb/s |
|---|---|---|---|
| Airtime per frame, no Reed-Solomon | 203 ms | 101 ms | 38.9 ms |
| Airtime per frame, with Reed-Solomon | 229 ms | 115 ms | 44.0 ms |
| Effective file throughput (169 B per frame), no / with Reed-Solomon | 6.7 / 5.9 kb/s | 13.4 / 11.8 kb/s | 34.8 / 30.7 kb/s |
| 100 KB (102,400 B, 606 frames), no / with Reed-Solomon | about 123 s / 139 s | about 61 s / 69 s | about 24 s / 27 s |

Retransmits, NAK frames and RX/TX turnaround (about 100 µs for the radio itself) are excluded, so expect longer. A
pass over a ground station lasts roughly 5 to 10 minutes. **File data per uplink block** is 169 bytes (section 8),
against 186 in the original draft: SDLS costs 8 bytes more than the old 4-byte `seq`, the TC frame header costs 5,
and the CFDP header costs a few more. The TC frame costs 3 bytes compared with the earlier custom uplink block.

## 4. Uplink TC Transfer Frame (CCSDS 232.0-B-4)

```
| TC header (5) | SDLS header (14) | Space Packet, encrypted | MAC (16) | FECF (2) | zero fill |
```

**TC header, 5 bytes (spec):** version (2 bits, `00`), bypass flag (1), control command flag (1),
spare (2), spacecraft ID (10), virtual channel ID (6), frame length (10, total frame bytes minus 1),
frame sequence number (8).

| Field | Value |
|---|---|
| Frame type | **Type-BD**: bypass = 1, control command = 0 **(spec)** |
| Frame sequence number | `0` (COP does not use it on Type-B frames, it is all zeroes) **(spec)** |
| Spacecraft ID | Assigned by CCSDS **(spec)**. Placeholder until assigned |
| Virtual channel ID | `0` |
| Segment header | Not used |
| FECF | CRC-16 as in section 2, over the frame. Polynomial x^16+x^12+x^5+1, register preset to all ones **(spec)** |
| Frame length | Delimits the frame. Fill after it is ignored |

**Accept rule:** COMMS accepts **only Type-BD frames**. Type-AD frames and all Type-C (control) frames
are dropped, because Type-C carries no security header **(spec)**. COP-1 is not used.

## 5. Uplink security (SDLS, CCSDS 355.0-B-2)

Authenticated encryption with AES-GCM. The whole Space Packet, including its header, is the encrypted
data field.

| Parameter | Value |
|---|---|
| Service | Authenticated encryption, AES-GCM |
| Key | **256-bit** (decided). Matches the SDLS baseline in the informative Annex E |
| SPI (2 bytes) | `0x0001` = uplink security association. `0x0000` and `0xFFFF` are reserved **(spec)** |
| IV | 12 bytes, sent in the frame. Also the anti-replay counter |
| Sequence number field | 0 bytes (the IV does this job) **(spec, Annex E1 note 1)** |
| Pad length field | 0 bytes |
| MAC | 16 bytes |
| Security header | 14 bytes (SPI 2 + IV 12), as in the Annex E1 baseline. Trailer 16 bytes (MAC) |

**Authenticated fields (SDLS bit mask, spec 4.2.2.6.2, mission choice marked):**

| Field | Authenticated |
|---|---|
| Virtual channel ID | Yes (required) |
| Segment header (if present) | Yes (required) |
| Security header (SPI) | Yes (required). The IV is excluded from the mask but is an input to GCM itself |
| Frame data field | Yes (required) |
| Spacecraft ID | Yes **(mission choice, proposed)** |
| Frame length, flags, frame sequence number | No (default) |

**Replay rule:** the IV is a big-endian counter. COMMS accepts a frame only if its IV is greater than
the last accepted IV and no more than 2^20 above it.

**Receive order:**
1. Check the FECF and the frame length.
2. Drop the frame if it is not Type-BD, or has the wrong spacecraft ID or virtual channel.
3. Cheap IV check against the last accepted IV.
4. SDLS verify and decrypt.
5. Only if the MAC verifies: update the last accepted IV, then route the Space Packet.

Plaintext from a frame that failed verification is never used or forwarded.

**Send order (ground):** add the headers, apply SDLS, then compute the FECF.

### 5.1 Ground-side requirements

- **The ground must never reuse an IV under one key.** A repeat in GCM leaks the XOR of the two plaintexts and lets an
  attacker recover the authentication key and forge frames. The satellite only keeps the replay high-water mark.
- The ground needs a persistent IV counter that survives crashes: write the new value before sending.
- Key rotation: the SPI selects among preloaded keys. Rotation procedure TBD (open items 5 and 6).

### 5.2 Persistent storage

The original draft used 3x FRAM (majority vote). The plan now assumes **no FRAM**. The storage split is undecided
(open item 1), so each item depends on it:
- **Replay high-water mark (IV counter).** Without persistence, a COMMS reset opens a replay window. Placeholder:
  checkpoint to internal flash every N accepted frames, and after reset require `IV > checkpoint + N`. Needs a
  decision and a flash wear check.
- **CFDP transaction state** (file size, offsets received). Initial approach: keep it in RAM, so a reset cancels the
  transfer and ground restarts. Better: persist to flash once its location is decided.
- **Staging area** for received firmware and images: COMMS has flash for it (confirmed; the 4 MB part and
  read-while-write still to **(verify)**). Each other board is expected to have its own flash for its new image, or
  to use FRAM, once the hardware details are settled.

## 6. Space Packet (CCSDS 133.0-B-2)

**Header, 6 bytes (spec):** version `000` (3 bits), type (1 bit: 1 = uplink, 0 = downlink), secondary
header flag (1), APID (11), sequence flags (2), sequence count (14), data length (16, data field bytes
minus 1).

| Field | Value |
|---|---|
| Sequence flags | Always `11` (unsegmented) |
| Sequence count | Counter per APID, for loss detection only. Not used for security |
| Secondary header | Downlink: 7 bytes, callsign (6 ASCII characters, space padded, plus 1 byte SSID) **(proposed)**. Uplink: none |
| Packets per block | One |
| Idle packets | Not used. Fill is plain zeros |

**APIDs:**

| APID | Dir | Contents |
|---|---|---|
| `0x010` | up | Commands: N x 12-byte CAN records |
| `0x020` | down | Telemetry and replies: N x 12-byte CAN records |
| `0x021` | down | Beacon: 1-3 CAN records |
| `0x030` | up | CFDP PDUs, ground to satellite |
| `0x031` | down | CFDP PDUs, satellite to ground |
| `0x7FF` | none | Reserved by CCSDS for idle packets, unused **(spec)** |

**CAN-equivalent record (12 bytes):** the 29-bit CAN ID as 4 bytes (`can_proto.h` layout: priority,
src, dst, class), then the 8 payload bytes (`src, op, p2..p7`). DLC is always 8 and is not sent.

**Uplink record rules** (checked by COMMS before anything is put on the CAN bus):
- `src` must equal GROUND_ID, `0x10` (`common/can_proto.h`). Node IDs `0x01`-`0x08` are already used.
- `dst` must be a real board (not COMMS itself, not broadcast), `class` must be `CLS_COMMAND`, and `(dst, class, op)`
  must be on the allowlist in `src/can/uplink_policy.c`. Default deny. MVP: one entry, `OP_PING` to CDH. COMMS also forces
  the CAN priority to a fixed low value, whatever the record says.
- The current system state must allow the command (section 7).

## 7. Routing on the CAN bus

All boards share one CAN bus. Every record has a source and destination node ID, `0xFF` is broadcast.

- **Uplink:** COMMS puts each accepted record on the bus with `src` = GROUND_ID and `dst` = the target
  board. The target accepts commands whose `src` is GROUND_ID.
- **State gating:** COMMS drops commands the current system state forbids (for example, motor commands
  in safe mode). The state comes from CDH over CAN.
- **Logging:** CDH sees every uplinked command by filtering on `src` = GROUND_ID. No extra message.
- **Replies and telemetry:** boards address replies to GROUND_ID. COMMS accepts them with a CAN filter
  and sends them down on APID `0x020`.
- CDH is on the bus but is not a relay.
- **Why direct routing.** All boards share one bus and every record carries source and destination, so no board needs
  to relay. If CDH is down, commands and firmware can still reach other boards, which helps recovery.
- **Trust.** The CAN bus has no authentication, so the uplink checks (SDLS, GROUND_ID, allowlist, state gating) all
  happen in COMMS. A bug or compromise in COMMS could command any board, so the allowlist must be strict.

### 7.1 Firmware to another board (COMMS to target over CAN)

CDH is not in the data path. Sequence for a firmware update of board X:
1. Ground uploads the image to COMMS over UHF (section 8). COMMS holds it in its staging flash and verifies the
   signature and the anti-rollback counter (reject images whose version is not newer than the installed one, via the
   MCUboot security counter). CFDP's checksum is a transfer check only.
2. A command (APID `0x010`) asks CDH to put board X into update mode. CDH can power-cycle board X through EPS if
   needed. This is CDH's orchestration role, using the state it already owns.
3. COMMS sends the image to board X directly (`dst` = X), in chunks over CAN.
4. Board X writes it to its own secondary flash slot (or FRAM, per section 5.2) and **verifies the signature and
   anti-rollback itself** with MCUboot. COMMS is not trusted as the only check.
5. Board X reports success or failure to COMMS, which tells ground over the downlink. Install and confirm follow the
   MCUboot test-swap rules: the new image must prove itself and be confirmed, or it reverts. Update COMMS **last**
   and one board at a time.

The CAN transfer protocol is not yet designed (open item 10). Options: a small chunk and window-ACK protocol, or CFDP
carried over a CAN transport layer, since the CFDP header already carries a destination entity ID. One hop and one
staging copy avoid doubling the transfer time and the storage.

**CAN push estimate.** Our CAN carries 6 bytes of image data per frame (8-byte data field minus 1 byte `src` and 1
byte `op`; the destination is in the 29-bit CAN ID). A 256 KB image (example size, 262,144 B) is 43,691 frames, or
about 10 to 12 seconds of pure bus time at an assumed 500 kb/s (bus rate not checked), before protocol overhead and
other traffic. This assumes 29-bit extended IDs: a frame with 6 data bytes is 115 bits without bit stuffing and up
to about 140 with worst-case stuffing. Chunk headers inside the 6 bytes would lower the image data per frame further.

### 7.2 Firmware signing

Separate from link encryption: an asymmetric signature on the firmware image (MCUboot, proposal ECDSA P-256,
supported by MCUboot and PSA, verified in software since Zephyr has no U5 PKA driver). Link encryption protects the
link, the signature protects the image itself, so a captured board cannot be used to forge updates. Both apply to
firmware uploads.

## 8. File transfer (CFDP Class 2 subset, CCSDS 727.0-B-5)

Used for firmware upload (UHF uplink) and payload image download (S-band). PDUs ride in the Space
Packet data field on APIDs `0x030` and `0x031`.

**PDUs used (spec, table 5-4):** Metadata `07`, File Data, EOF `04`, ACK `06`, NAK `08`, Finished `05`.
Not used: Prompt, Keep Alive, filestore requests, proxy operations, large files.

**PDU header settings:** version `001`, acknowledged mode (0), CRC flag 0, large-file flag 0, 1-byte
entity IDs equal to the CAN node IDs (ground = GROUND_ID `0x10`), 1-byte transaction sequence number.
Header size with these settings is 7 bytes (4 fixed + three 1-byte fields).

**Firmware upload flow (UHF):**
1. Ground sends Metadata: file size, source name, destination name. The destination name carries the
   target board, for example `fw/EPS`.
2. Ground sends File Data PDUs, each with a 4-byte offset then data. Each PDU is its own SDLS frame.
3. Ground sends EOF with the file checksum. COMMS replies ACK(EOF).
4. COMMS sends NAK listing missing byte ranges (start and end offset, 8 bytes each). Ground resends
   only those ranges.
5. When the file is complete COMMS sends Finished, and ground sends ACK(Finished).
6. Install and confirm are ordinary commands on APID `0x010`.

**Cancel:** an EOF with condition code `1111` (Cancel.request received) is always accepted and takes
priority over any transfer **(spec, table 5-5)**.

**Image download (S-band):** the same PDUs. The ground's NAK and ACK go up on UHF, APID `0x030`.

**File data per uplink block:** 180 (packet data field) - 7 (CFDP header) - 4 (offset) = **169 bytes**.

## 9. Beacon

- APID `0x021`, unauthenticated, unencrypted.
- 1-3 CAN-format records (system state and health), callsign in the secondary header.
- Same block, framing and PHY settings as every other UHF downlink.
- Sent from a lowest-priority thread. Default period TBD (placeholder 60 s). The decoder and frame spec are
  published openly (SatNOGS, gr-satellites).

## 10. Downlink alternative: TM Transfer Frame (CCSDS 132.0-B-3) (proposed)

Under consideration instead of the `len` + CRC downlink block. Uplink, packets, APIDs and CFDP do not
change. The layout below is checked against CCSDS 132.0-B-3 **(spec)**.

```
| TM header (6) | data field: Space Packets / idle data (215) | FECF (2) |   = 223 bytes
```

- Header, 6 bytes **(spec)**: master channel ID (12 bits: version 2 + spacecraft ID 10), virtual channel ID
  (**3 bits**, not 6 as in the TC frame), OCF flag (1), master channel frame count (8), virtual channel
  frame count (8), data field status (16 bits: secondary header flag, synchronization flag, packet order
  flag, segment length identifier, 11-bit First Header Pointer).
- Constant length throughout a mission phase **(spec)**. No OCF and no secondary header (both optional).
  No SDLS.
- FECF: 2 bytes, optional, its presence is fixed per mission. CRC-16 as in section 2.
- A packet may span frames. With the synchronization flag at `0`, the First Header Pointer holds the
  position of the first packet that starts in the frame.
- A frame with no packet data is an Only Idle Data frame: First Header Pointer `11111111110`, and the
  data field holds a pseudo-noise sequence from a 32-cell LFSR (polynomial D0+D1+D2+D22+D32, initialization
  per annex D), not zeros **(spec)**.
- Frame counts give loss detection in addition to the packet sequence count.

## 11. Future

- **Reed-Solomon:** the 223-byte block gains 32 parity bytes to form a Reed-Solomon (255,223) codeword
  (corrects up to 16 byte errors, E = 16 **(spec, 131.0-B-5)**) and the CCSDS randomizer is applied. The
  block layout, sync word and everything above the link block stay the same. The block length on the air
  becomes 255 bytes.
- **S-band downlink:** the radio, modulation, framing and rates for payload images are not yet defined.
  The packet and CFDP layers are unchanged.

## 12. Requirements met

| Requirement | Met by |
|---|---|
| UNP12-92: uplink shall be encrypted | Section 5, SDLS authenticated encryption |
| UNP12-91: spacecraft should be capable of ceasing transmission on command | Not yet defined (PLANNED.md, open items) |

## 13. Design rationale and rejected alternatives

**Constraints.**
- **UNP12-92:** "Uplink communications shall be encrypted for all spacecrafts." The SDLS *TC baseline* is
  authentication-only (AES-CMAC) **(spec, Annex E2)**, so authenticated encryption is a mission choice on top of it.
- **UNP12-91:** the spacecraft should be able to cease transmission on command. The design is open (open item 16).
- **Amateur bands.** Downlinks (UHF telemetry, S-band mission data) stay unencrypted and the format must be publicly
  documented **(verify: jurisdiction)**. Encrypting telecommand to a space station is the allowed exception in the US
  (97.211) **(verify)**. Encrypted firmware upload has been confirmed to count as telecommand, so it can be encrypted
  like any other uplink frame.
- **Same record format as CDH.** Commands and telemetry reuse the `can_proto.h` 12-byte record so COMMS puts records
  on CAN without translation.
- **Small frames, lossy link.** Bulk transfers must be resumable across passes. Payload images never go down on UHF.
- **UHF is half-duplex.** The RFM26W is a time-division transceiver, so TX (telemetry, beacon) and RX (uplink) share
  one radio and share time.

**Why Type-BD and no COP-1.** CFDP supplies retransmission, so COP-1 is redundant. Type-C frames carry no security
header **(spec, 232.0-B-4 6.3.4.1)**, so accepting one would let an unauthenticated frame change receiver state.

**Why the Space Packet sequence count is not for security.** It is 14 bits and wraps at 16384. The SDLS IV is the
anti-replay counter.

**Why Space Packet idle packets are not used.** Space Packets are at least 7 octets, so a 1-6 byte gap cannot be one.
Fill is zeros, delimited by the frame length (uplink) or `len` (downlink).

**Why not AX.25.** The original argument (the radio could not make AX.25's 1200 or 9600 baud) no longer holds: 9.6 kb/s
GFSK is available on the RFM26W. The CCSDS stack is kept for other reasons: AX.25 framing (HDLC flags, bit-stuffing,
NRZI) is not done by the radio and would be software we write. Bit-stuffing makes frame length depend on the data,
which conflicts with the fixed block. AX.25 has no error correction, only a 16-bit checksum. And security (SDLS), file
transfer (CFDP) and the 12-byte CAN records are designed around Space Packets. What AX.25 would still give: frames
that standard amateur 9k6 packet stations understand directly. The decision can be revisited (open item 21).

**Why not Zephyr's radio APIs.** Checked against Zephyr v4.4.0-rc3 (`ieee802154_radio.h`, `ieee802154.h`, `lora.h`).
There is no generic FSK or raw-block radio API, and the RFM26W has no Zephyr driver.
- **IEEE 802.15.4.** The L2 layer builds the MAC header and FCS and caps the frame at 127 bytes (MTU 125), with about 9
  bytes of MAC header. With a 116-byte payload the uplink Space Packet would be about 79 bytes and file data per frame
  about 62, against 169 now: roughly 63% less, 2.7 times as many frames for an upload (estimates). Addressing, sequence
  number and FCS duplicate the callsign, CAN node IDs, Space Packet count and FECF, and 15.4 link security would overlap
  SDLS. The radio API does not expose modulation, rate, preamble or sync word, so the CCSDS sync marker and the fixed
  block do not carry over. The 802.15.4 PHY details are partly from memory **(verify)**.
- **LoRa.** The RFM26W is an FSK radio, and LoRa needs a different chip (Semtech SX126x/127x). The rates are far lower
  (hundreds of bits per second at the slowest settings, so a 100 KB upload takes hours, an estimate), and the CCSDS
  sync marker does not apply. LoRa has real strengths for satellites (sensitivity, Doppler tolerance), but using it
  would be a separate redesign.
- **Decision:** keep the custom framing and write a small RFM26W driver in the Zephyr driver model, using Zephyr's SPI
  and GPIO APIs. CAN, PSA crypto, CRC and MCUboot are still reused.

## 14. MVP bench profile

The MVP (PLANNED.md section 4) is a simplified profile of this protocol for a same-room bench test. **A frame built
for the MVP is not accepted by flight firmware**, because flight requires SDLS (section 5).

| MVP element | Relation to this document |
|---|---|
| Uplink block | **Direct subset**: the real TC frame (section 4), Type-BD, minus the SDLS header and MAC. The Space Packet is sent in the clear |
| Downlink block, Space Packet header | **Direct subset** (sections 3 and 6) |
| APIDs `0x010`, `0x020`, `0x021` | **Direct subset** (section 6) |
| APIDs `0x0F0` (ping request, up), `0x0F1` (ping reply, down) | **Deviation**: test-only, removed or reassigned in the full design |
| CAN record layout | **Direct subset**. Routing and record rules without state gating |
| Ping and telemetry counter payloads | **Deviation** (test content) |
| Security | **Deviation**: none. No SDLS header, no MAC, no encryption |
| PHY | Same as section 2, 9.6 kb/s, TX power at the minimum setting |

**MVP uplink block:** `| TC header (5) | Space Packet | FECF (2) | zero fill |` = 223 bytes. TC header values: version `00`,
bypass 1, control command 0 (Type-BD), spare `00`, spacecraft ID a placeholder kept in one constant, virtual channel
`0`, frame length = total frame bytes minus 1, frame sequence number `0`. The FECF is the CRC-16 of section 2 over the
header and the packet. COMMS drops anything that is not Type-BD, has the wrong spacecraft ID or virtual channel, or
fails the FECF or frame length check. Max Space Packet 216 bytes (223 minus 7), max packet data field 210.

**MVP downlink block:** as section 3. Max Space Packet 219, max data field 213 (206 if the callsign secondary header
is used, open item 3). The receiver drops any block whose CRC fails or whose `len` is above 219. No acknowledgement at
this layer.

**Space Packet header:** as section 6. The sequence count is per APID and direction, the sender increments, and the
receiver uses it to count lost packets.

**Payload layouts:**
- **Ping request/reply data field** (deviation): `ping_id` (4, big-endian, counts up from 0), `payload` (0-200 bytes, ground's
  choice, default 32). The reply returns `ping_id` and the payload unchanged, so ground can check them. The board adds
  nothing else. RSSI is read by ground.
- **Telemetry/beacon data field** (the counter is a deviation): `tlm_counter` (4), then N x 12-byte CAN-equivalent
  records. In the MVP the records may be fixed test values (for example an incrementing value from COMMS_ID) so ground
  can check content.
- **Command data field** (direct subset): N x 12-byte CAN records. The MVP accepts exactly one record per packet.
- **CAN ping and pong** (MVP CAN_CMD test, `src/can/can_ping.h`), using the existing `OP_PING` (0x01) and `OP_PONG` (0x02):
  ping record: `src` GROUND_ID, `dst` the board, class `CLS_COMMAND`, data `{GROUND_ID, OP_PING, t0..t5}`. Pong record:
  `src` the board, `dst` GROUND_ID, class `CLS_CMD_RESP`, data `{board, OP_PONG, t0..t5}`. The six token bytes come back
  unchanged. COMMS sends the pong down as a telemetry packet (APID `0x020`, `tlm_counter` then the 12-byte record).

## 15. Ground-station decoding

gr-satellites has `CCSDS Uncoded`, `CCSDS Reed-Solomon` and `CCSDS Concatenated` framings that take a configurable
`frame size`, with an optional scrambler **(spec: gr-satellites SatYAML docs)**. A downlink of fixed 223-byte frames
after the CCSDS sync marker should match "CCSDS Uncoded", and the same entry moves to "CCSDS Reed-Solomon" later. The
remaining work is a SatYAML file and a small telemetry parser for the downlink contents. **(verify)** whether it
requires a TM Transfer Frame header (which section 10 would satisfy), and what it does with the CRC. The uplink is
encrypted, so no one else decodes it and it needs no community support. Ground-station software requirements for the
bench test are in PLANNED.md section 4.
