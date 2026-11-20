/**
 * @file can_record.h
 * @ingroup comms
 * @brief The 12-byte CAN-equivalent record carried in Space Packets (PROTOCOL.md section 6)
 *
 * A record is one CAN frame in a fixed layout, so COMMS can put commands on the bus and take telemetry off it
 * without translating:
 * ```
 * bytes 0-3  : the 29-bit CAN ID as a big-endian 32-bit value (common/can_proto.h layout)
 *              priority [28:26], reserved [25:22] (must be 0), src [21:14], dst [13:6], class [5:0]
 * bytes 4-11 : the 8 data bytes: src, op, p2, p3, p4, p5, p6, p7
 * ```
 * The DLC is always 8 and is not sent. Of the 8 data bytes, byte 0 is the source node and byte 1 is the opcode,
 * which leaves **6 bytes of application data (p2..p7)** per CAN frame. The destination is in the CAN ID, not in
 * the data. (common/can_proto.h `struct can_payload` and `can_fill_payload`.)
 *
 * This module checks structure only. Who may send a record, and which records are allowed, is the uplink
 * policy (stage 3).
 */
#ifndef COMMS_CAN_RECORD_H_
#define COMMS_CAN_RECORD_H_

#include <stddef.h>
#include <stdint.h>
#include <zephyr/drivers/can.h>

#define CAN_RECORD_LEN 12 /**< bytes per record on the wire */

/** Bits of the 29-bit ID that may be set. Bits 25-22 are reserved and must be 0. */
#define CAN_RECORD_ID_VALID_MASK 0x1C3FFFFFu

enum rec_status {
	REC_OK = 0,
	REC_ERR_ARG,   /**< NULL pointer */
	REC_ERR_LEN,   /**< not exactly CAN_RECORD_LEN bytes */
	REC_ERR_ID,    /**< ID has bits set above bit 28 or in the reserved field */
	REC_ERR_FRAME, /**< a CAN frame that is not a classic 8-byte extended data frame */
	REC_STATUS_COUNT
};

/** One record in memory. */
struct can_record {
	uint32_t id;      /**< 29-bit CAN ID */
	uint8_t data[8];  /**< src, op, p2..p7 */
};

/* Accessors for the ID fields (same layout as CAN_ID_FULL in common/can_proto.h) */
static inline uint8_t can_record_prio(const struct can_record *r) { return (uint8_t)((r->id >> 26) & 0x07); }
static inline uint8_t can_record_src(const struct can_record *r) { return (uint8_t)((r->id >> 14) & 0xFF); }
static inline uint8_t can_record_dst(const struct can_record *r) { return (uint8_t)((r->id >> 6) & 0xFF); }
static inline uint8_t can_record_class(const struct can_record *r) { return (uint8_t)(r->id & 0x3F); }
/** Opcode, the second data byte. */
static inline uint8_t can_record_op(const struct can_record *r) { return r->data[1]; }

/** @brief Write a record as 12 bytes. @p out must have room for CAN_RECORD_LEN bytes. */
void can_record_pack(uint8_t *out, const struct can_record *r);

/**
 * @brief Read a record from bytes and check its structure.
 * @param in the bytes
 * @param len their number, must be CAN_RECORD_LEN
 * @param r out
 * @return REC_OK, REC_ERR_ARG, REC_ERR_LEN or REC_ERR_ID
 */
enum rec_status can_record_unpack(const uint8_t *in, size_t len, struct can_record *r);

/**
 * @brief Take a record from a received CAN frame.
 * @return REC_OK, or REC_ERR_FRAME if the frame is not an extended, 8-byte, classic data frame
 *         (standard ID, remote frame, CAN FD, or any other length), or REC_ERR_ID for a bad ID.
 */
enum rec_status can_record_from_frame(const struct can_frame *f, struct can_record *r);

/**
 * @brief Make a CAN frame (extended ID, DLC 8) from a record, ready for can_send().
 * @return REC_OK, or REC_ERR_ID for a bad ID
 */
enum rec_status can_record_to_frame(const struct can_record *r, struct can_frame *f);

/**
 * @brief How many records a packet data field holds.
 * @param data_len data field length in bytes
 * @return the count (at least 1), or -EINVAL if it is zero or not a multiple of CAN_RECORD_LEN
 */
int can_record_count(size_t data_len);

/** @brief Short name of a status, for logs. */
const char *rec_status_str(enum rec_status s);

#endif /* COMMS_CAN_RECORD_H_ */
