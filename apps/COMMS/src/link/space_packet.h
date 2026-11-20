/**
 * @file space_packet.h
 * @ingroup comms
 * @brief CCSDS Space Packet primary header and per-APID sequence counters (PROTOCOL.md section 6)
 *
 * Header, 6 bytes, bit 0 is the most significant bit of byte 0 (checked against CCSDS 133.0-B-2 section 4.1.3):
 * ```
 * byte 0: version(3) = 000 | type(1) | secondary header flag(1) | APID bits 10-8 (3)
 * byte 1: APID bits 7-0
 * byte 2: sequence flags(2) = 11 | sequence count bits 13-8 (6)
 * byte 3: sequence count bits 7-0
 * byte 4-5: data length = data field bytes - 1, big-endian
 * ```
 * Type is 1 for a telecommand (uplink) packet and 0 for telemetry (downlink). The data field has at least one
 * byte, so a packet is at least 7 bytes. Sequence flags are always 11 (unsegmented): segmentation is done by
 * CFDP, not here.
 */
#ifndef COMMS_SPACE_PACKET_H_
#define COMMS_SPACE_PACKET_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* APIDs (PROTOCOL.md sections 6 and 14) */
#define APID_UL_CMD    0x010 /**< up: commands, N x 12-byte CAN records */
#define APID_DL_TLM    0x020 /**< down: telemetry and replies, N x 12-byte CAN records */
#define APID_DL_BEACON 0x021 /**< down: beacon, 1 to 3 CAN records */
#define APID_UL_CFDP   0x030 /**< up: CFDP PDUs, ground to satellite */
#define APID_DL_CFDP   0x031 /**< down: CFDP PDUs, satellite to ground */
#define APID_UL_PING   0x0F0 /**< up: ping request. MVP only, removed in the full design */
#define APID_DL_PING   0x0F1 /**< down: ping reply. MVP only */
#define APID_IDLE      0x7FF /**< reserved by CCSDS for idle packets. Unused */

#define SP_APID_COUNT     0x800  /**< APIDs are 11 bits */
#define SP_SEQ_COUNT_MASK 0x3FFF /**< the sequence count is 14 bits */
#define SP_MAX_DATA_LEN   65536  /**< 16-bit length field + 1 */

enum sp_status {
	SP_OK = 0,
	SP_ERR_ARG,     /**< NULL pointer */
	SP_ERR_SHORT,   /**< fewer than 7 bytes */
	SP_ERR_VERSION, /**< version is not 000 */
	SP_ERR_FLAGS,   /**< sequence flags are not 11 */
	SP_ERR_LEN,     /**< the data length field claims more bytes than the buffer holds */
	SP_STATUS_COUNT
};

/** A parsed packet. @c data points into the buffer that was parsed. */
struct sp_packet {
	bool is_tc;            /**< packet type: true for telecommand (uplink) */
	bool sec_hdr;          /**< secondary header flag. The header bytes are part of @c data */
	uint16_t apid;         /**< 11 bits */
	uint16_t seq_count;    /**< 14 bits */
	const uint8_t *data;   /**< data field */
	size_t data_len;       /**< data field bytes, at least 1 */
	size_t total_len;      /**< 6 + data_len: bytes the packet occupies in the buffer */
};

/**
 * @brief Parse a Space Packet from the start of a buffer.
 *
 * The buffer may be longer than the packet (zero fill follows it in a link block). Bytes after the packet
 * are not examined. Whether the APID is one COMMS handles is the router's decision, not this function's.
 */
enum sp_status sp_parse(const uint8_t *buf, size_t len, struct sp_packet *out);

/**
 * @brief Build a Space Packet.
 * @param out output buffer
 * @param cap its size
 * @param is_tc true for telecommand (uplink), false for telemetry
 * @param sec_hdr set the secondary header flag. The caller puts the secondary header at the start of @p data
 * @param apid 11 bits, at most 0x7FF
 * @param seq_count 14 bits, at most 0x3FFF
 * @param data data field, at least 1 byte
 * @param data_len its length, 1 to SP_MAX_DATA_LEN
 * @return packet length in bytes (6 + data_len), or a negative errno:
 *         -EINVAL (NULL pointer, empty or oversized data, APID or count out of range),
 *         -EMSGSIZE (it does not fit in @p cap)
 */
int sp_build(uint8_t *out, size_t cap, bool is_tc, bool sec_hdr, uint16_t apid, uint16_t seq_count,
	     const uint8_t *data, size_t data_len);

/** Sequence counters, one per APID, for one direction. Owned by one thread: no locking inside. */
struct sp_seq {
	uint16_t next[SP_APID_COUNT];
};

/** @brief Reset every counter to 0. */
void sp_seq_init(struct sp_seq *s);

/**
 * @brief Return the next sequence count for an APID and advance it. Wraps from 0x3FFF to 0.
 * @return the count to put in the packet. An APID above 0x7FF returns 0 and changes nothing.
 */
uint16_t sp_seq_next(struct sp_seq *s, uint16_t apid);

/**
 * @brief Packets missed between the one expected and the one received, modulo 16384.
 * @param expected the count that should have arrived next
 * @param received the count that did arrive
 */
uint16_t sp_seq_lost(uint16_t expected, uint16_t received);

/** @brief Short name of a status, for logs. */
const char *sp_status_str(enum sp_status s);

#endif /* COMMS_SPACE_PACKET_H_ */
