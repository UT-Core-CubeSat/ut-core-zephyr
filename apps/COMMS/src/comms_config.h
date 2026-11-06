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

/** PLACEHOLDER: CAN node ID the ground station appears as. Proposed 0x10 (open item 2). Node IDs 0x01
 *  to 0x08 are taken in common/can_proto.h. Every board must accept commands from this source. */
#define COMMS_GROUND_ID 0x10

/** PLACEHOLDER: callsign (6 characters, space padded) and SSID for the downlink secondary header.
 *  Real callsign not available yet (open item 3). */
#define COMMS_CALLSIGN "NOCALL"
#define COMMS_SSID     0

#endif /* COMMS_CONFIG_H_ */
