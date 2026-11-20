/**
 * @file comms_config.h
 * @ingroup comms
 * @brief COMMS protocol constants, and the single home of every protocol placeholder
 *
 * Wire-format limits come from PROTOCOL.md. Anything marked PLACEHOLDER is a value somebody still has to
 * decide, and is listed in PLANNED.md section 4 ("Placeholders and assumptions"). Change it here and nowhere else.
 */
#ifndef COMMS_CONFIG_H_
#define COMMS_CONFIG_H_

/* ---- Link block (PROTOCOL.md section 3) ---- */

/** Every block on the air is exactly this many bytes, both directions. Must equal the radio node's
 *  `block-length` (checked by BUILD_ASSERT in main.c). It is the data size of a Reed-Solomon (255,223)
 *  codeword, so adding Reed-Solomon later does not change it. */
#define COMMS_BLOCK_LEN 223

/** How long a send may take before the port gives up: the air time of one block is about 203 ms at
 *  9.6 kb/s, and a block already being received is waited for first (open item 22). */
#define COMMS_RADIO_SEND_TIMEOUT_MS 1000

/** PLACEHOLDER: carrier frequency, 433.92 MHz (the one the sample WDS header was made for). The bench
 *  frequency is not chosen and the module's band variant is unconfirmed (open item 18). */
#define COMMS_RADIO_FREQ_HZ 433920000U

/** PLACEHOLDER: transmit power in dBm. The bench test uses the minimum (PLANNED.md section 4). */
#define COMMS_RADIO_TX_DBM (-40)

/* ---- Uplink: TC Transfer Frame (PROTOCOL.md section 4) ---- */

#define COMMS_TC_HDR_LEN  5 /**< TC primary header. */
#define COMMS_TC_FECF_LEN 2 /**< Frame error control field (CRC-16). */

/** Largest Space Packet in a TC frame with no security (the MVP profile): 223 - 5 - 2. */
#define COMMS_UL_MAX_PACKET_CLEAR (COMMS_BLOCK_LEN - COMMS_TC_HDR_LEN - COMMS_TC_FECF_LEN)

/** PLACEHOLDER: the CCSDS-assigned spacecraft ID (10 bits). Not assigned yet (PROTOCOL.md section 4,
 *  open item 11). Any value works for bench testing as long as ground uses the same one. */
#define COMMS_TC_SCID 0x0A5

/** Virtual channel ID of the uplink (6 bits). PROTOCOL.md section 4: one channel, value 0. */
#define COMMS_TC_VCID 0

/* ---- Downlink block (PROTOCOL.md section 3) ---- */

#define COMMS_DL_LEN_FIELD 2 /**< Length of the leading `len` field. */
#define COMMS_DL_CRC_LEN   2 /**< Trailing CRC-16. */

/** Largest Space Packet in a downlink block: 223 - 2 - 2. */
#define COMMS_DL_MAX_PACKET (COMMS_BLOCK_LEN - COMMS_DL_LEN_FIELD - COMMS_DL_CRC_LEN)

/* ---- Space Packet (PROTOCOL.md section 6) ---- */

#define COMMS_SP_HDR_LEN 6 /**< Space Packet primary header. */

/** A Space Packet has at least one data field byte, so at least 7 bytes. */
#define COMMS_SP_MIN_PACKET (COMMS_SP_HDR_LEN + 1)

/* ---- Addressing (PROTOCOL.md sections 6 and 7) ---- */

/** PLACEHOLDER: CAN arbitration priority (0 = highest) that COMMS forces onto every record it puts on the
 *  bus for ground, whatever priority ground wrote, so ground cannot win arbitration over flight traffic.
 *  Same value as PRIO_LOW in board_config.h. */
#define COMMS_CAN_UPLINK_PRIO 3

/** How long a CAN send may wait for a free transmit mailbox before it is counted as an error. */
#define COMMS_CAN_SEND_TIMEOUT_MS 10

/* The ground station's CAN node ID is GROUND_ID in common/can_proto.h (0x10). Every board must accept
 * commands from that source. */


/** Amateur radio callsign of the satellite (K7UTS, given 2026-11-20) and the SSID byte. Carried in a 7-byte
 *  secondary header on every downlink packet (PROTOCOL.md section 6): 6 ASCII characters, space padded, then
 *  the SSID. The SSID is not decided, 0 for now. */
#define COMMS_CALLSIGN "K7UTS"
#define COMMS_SSID     0

/** 1 puts the callsign secondary header on every downlink packet, which shrinks the downlink data field from
 *  213 to 206 bytes (open item 3). Decided 2026-11-20: on, as PROTOCOL.md proposes, so every packet identifies
 *  the station and no separate ID timer is needed. With 0 no callsign is transmitted at all. */
#define COMMS_CALLSIGN_IN_PACKETS 1

#define COMMS_CALLSIGN_LEN 6 /**< characters in the secondary header, space padded */
#define COMMS_SEC_HDR_LEN  7 /**< callsign (6) + SSID (1) */

/** Bytes the downlink secondary header takes from the data field (0 or COMMS_SEC_HDR_LEN). */
#define COMMS_DL_SEC_HDR_LEN (COMMS_CALLSIGN_IN_PACKETS ? COMMS_SEC_HDR_LEN : 0)

/* ---- Periodic downlink (stage 3b, PROTOCOL.md sections 9 and 14) ---- */

/** PLACEHOLDER: telemetry period (PLANNED.md section 4 proposes 5 s). */
#define COMMS_TLM_PERIOD_MS 5000U

/** PLACEHOLDER: beacon period (PROTOCOL.md section 9: TBD, placeholder 60 s). Open item 3. */
#define COMMS_BEACON_PERIOD_MS 60000U

/** Heartbeat COMMS broadcasts on the CAN bus. CDH watches for it (`check_node_timeouts`). */
#define COMMS_HEARTBEAT_PERIOD_MS 1000U

/** PLACEHOLDER: a cached record older than this is not sent, so ground is never shown stale data as current.
 *  Longer than the slowest board's telemetry period, which is not known yet. */
#define COMMS_TLM_MAX_AGE_MS 60000U

/** Records in one telemetry packet: (206 - 4) / 12 = 16 with the callsign header, 17 without. */
#define COMMS_TLM_MAX_RECORDS ((COMMS_DL_MAX_PACKET - COMMS_SP_HDR_LEN - COMMS_DL_SEC_HDR_LEN - 4) / 12)

/** Records in a beacon packet (PROTOCOL.md section 9: 1 to 3). */
#define COMMS_BEACON_MAX_RECORDS 3

#endif /* COMMS_CONFIG_H_ */
