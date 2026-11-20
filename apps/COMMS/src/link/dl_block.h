/**
 * @file dl_block.h
 * @ingroup comms
 * @brief Downlink block: length, one Space Packet, zero pad, CRC-16 (PROTOCOL.md section 3)
 *
 * ```
 * | len (2) | Space Packet (len bytes) | zero pad | CRC-16 (2) |   = 223 bytes
 * ```
 *
 * `len` is the Space Packet length in bytes, big-endian. The CRC-16 (crc16.h) covers `len` through the
 * padding, which is the first 221 bytes, and is stored big-endian in the last two. A receiver drops a block
 * whose CRC fails or whose `len` is above 219 or below 7. One packet per block. No acknowledgement at this layer.
 */
#ifndef COMMS_DL_BLOCK_H_
#define COMMS_DL_BLOCK_H_

#include <stddef.h>
#include <stdint.h>

enum dl_status {
	DL_OK = 0,
	DL_ERR_ARG,  /**< NULL pointer */
	DL_ERR_SIZE, /**< the buffer is not COMMS_BLOCK_LEN bytes */
	DL_ERR_CRC,  /**< the CRC does not match */
	DL_ERR_LEN,  /**< len is above COMMS_DL_MAX_PACKET or below the smallest Space Packet */
	DL_STATUS_COUNT
};

/**
 * @brief Pack one Space Packet into a COMMS_BLOCK_LEN-byte block, zero padded, with its CRC.
 * @param block output, COMMS_BLOCK_LEN bytes
 * @param packet the Space Packet
 * @param packet_len its length: COMMS_SP_MIN_PACKET to COMMS_DL_MAX_PACKET
 * @retval 0 packed
 * @retval -EINVAL NULL pointer, or the packet is shorter than a Space Packet can be
 * @retval -EMSGSIZE the packet does not fit
 */
int dl_block_pack(uint8_t *block, const uint8_t *packet, size_t packet_len);

/**
 * @brief Check a received block and find its Space Packet.
 *
 * The CRC is checked first (integrity), then the length. Padding bytes are not checked.
 * @param block received bytes
 * @param block_len their number, must be COMMS_BLOCK_LEN
 * @param packet out: points into the block
 * @param packet_len out: bytes in the packet
 * @return DL_OK or the reason the block was dropped
 */
enum dl_status dl_block_unpack(const uint8_t *block, size_t block_len, const uint8_t **packet,
			       size_t *packet_len);

/** @brief Short name of a status, for logs. */
const char *dl_status_str(enum dl_status s);

#endif /* COMMS_DL_BLOCK_H_ */
