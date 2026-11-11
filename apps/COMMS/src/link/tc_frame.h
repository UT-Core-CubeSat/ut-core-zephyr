/**
 * @file tc_frame.h
 * @ingroup comms
 * @brief Uplink TC Transfer Frame, Type-BD (PROTOCOL.md section 4, CCSDS 232.0-B-4)
 *
 * ```
 * | TC header (5) | frame data field | FECF (2) | zero fill |   = 223-byte block
 * ```
 *
 * The frame data field is the Space Packet (MVP profile, no security) or, from stage 4, the SDLS header,
 * the encrypted packet and the MAC. This module does not look inside it: it checks the frame and hands back
 * the data field.
 *
 * Header, 5 bytes, bit 0 is the most significant bit of byte 0 (CCSDS numbering, checked against 232.0-B-4
 * figure 4-2):
 * ```
 * byte 0: version(2) | bypass(1) | control(1) | spare(2) | spacecraft ID bits 9-8 (2)
 * byte 1: spacecraft ID bits 7-0
 * byte 2: virtual channel ID (6) | frame length bits 9-8 (2)
 * byte 3: frame length bits 7-0           (frame length field = total frame bytes - 1)
 * byte 4: frame sequence number           (0 for Type-B frames; ignored on receive)
 * ```
 * The FECF is CRC-16/CCITT-FALSE over every byte before it (header and data field), stored big-endian.
 *
 * COMMS accepts only Type-BD frames (bypass 1, control 0). Type-AD, Type-BC and the reserved combination
 * are dropped, because control frames carry no security header and COP-1 is not used.
 */
#ifndef COMMS_TC_FRAME_H_
#define COMMS_TC_FRAME_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/** Why a block was or was not accepted. The values index the drop counters, keep them dense. */
enum tc_status {
	TC_OK = 0,
	TC_ERR_ARG,     /**< NULL pointer */
	TC_ERR_SHORT,   /**< buffer shorter than the smallest possible frame (header + FECF) */
	TC_ERR_LEN,     /**< frame length field is smaller than header + FECF, or larger than the buffer */
	TC_ERR_FECF,    /**< the frame error control field does not match */
	TC_ERR_VERSION, /**< version is not 00 */
	TC_ERR_SPARE,   /**< spare bits are not 00 */
	TC_ERR_TYPE,    /**< not Type-BD */
	TC_ERR_SCID,    /**< wrong spacecraft ID */
	TC_ERR_VCID,    /**< wrong virtual channel ID */
	TC_STATUS_COUNT
};

/** A parsed, accepted frame. Pointers refer into the block that was parsed. */
struct tc_frame {
	uint16_t scid;       /**< spacecraft ID, 10 bits */
	uint8_t vcid;        /**< virtual channel ID, 6 bits */
	uint8_t seq;         /**< frame sequence number (not checked) */
	bool bypass;         /**< always true for an accepted frame */
	bool control;        /**< always false for an accepted frame */
	const uint8_t *data; /**< frame data field */
	size_t data_len;     /**< bytes in the frame data field (may be 0) */
	size_t frame_len;    /**< total frame bytes, header to FECF */
};

/**
 * @brief Check a received block and extract the frame data field.
 *
 * Check order follows PROTOCOL.md section 5: length and FECF first (integrity), then version, type,
 * spacecraft ID and virtual channel (policy). A frame with a bad FECF is reported as such even if it also
 * has the wrong spacecraft ID. Bytes after the frame (zero fill) are ignored and not checked.
 *
 * The spacecraft ID and virtual channel ID are compared with COMMS_TC_SCID and COMMS_TC_VCID
 * (comms_config.h).
 *
 * @param block received bytes (normally COMMS_BLOCK_LEN)
 * @param block_len number of bytes in @p block
 * @param out filled on TC_OK, untouched otherwise
 * @return TC_OK or the reason the block was dropped
 */
enum tc_status tc_frame_parse(const uint8_t *block, size_t block_len, struct tc_frame *out);

/**
 * @brief Build a Type-BD frame into a COMMS_BLOCK_LEN-byte block, zero filled after the FECF.
 * @param block output, COMMS_BLOCK_LEN bytes
 * @param scid spacecraft ID, at most 0x3FF
 * @param vcid virtual channel ID, at most 0x3F
 * @param data frame data field
 * @param data_len its length. At most COMMS_BLOCK_LEN - 7
 * @retval 0 built
 * @retval -EINVAL NULL pointer, or scid or vcid out of range
 * @retval -EMSGSIZE the frame does not fit in the block
 */
int tc_frame_build(uint8_t *block, uint16_t scid, uint8_t vcid, const uint8_t *data, size_t data_len);

/** @brief Short name of a status, for logs. */
const char *tc_status_str(enum tc_status s);

#endif /* COMMS_TC_FRAME_H_ */
