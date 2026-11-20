/**
 * @file dl_block.c
 * @ingroup comms
 * @brief Downlink block. See dl_block.h for the layout.
 */
#include "dl_block.h"

#include <errno.h>
#include <string.h>

#include "comms_config.h"
#include "crc16.h"

/** The CRC covers everything except its own two bytes. */
#define DL_CRC_COVERED (COMMS_BLOCK_LEN - COMMS_DL_CRC_LEN)

int dl_block_pack(uint8_t *block, const uint8_t *packet, size_t packet_len)
{
	uint16_t crc;

	if (block == NULL || packet == NULL || packet_len < COMMS_SP_MIN_PACKET) {
		return -EINVAL;
	}
	if (packet_len > COMMS_DL_MAX_PACKET) {
		return -EMSGSIZE;
	}

	memset(block, 0, COMMS_BLOCK_LEN);
	block[0] = (uint8_t)(packet_len >> 8);
	block[1] = (uint8_t)(packet_len & 0xFF);
	memcpy(block + COMMS_DL_LEN_FIELD, packet, packet_len);
	crc = comms_crc16(block, DL_CRC_COVERED);
	block[DL_CRC_COVERED] = (uint8_t)(crc >> 8);
	block[DL_CRC_COVERED + 1] = (uint8_t)(crc & 0xFF);
	return 0;
}

enum dl_status dl_block_unpack(const uint8_t *block, size_t block_len, const uint8_t **packet,
			       size_t *packet_len)
{
	uint16_t crc;
	size_t len;

	if (block == NULL || packet == NULL || packet_len == NULL) {
		return DL_ERR_ARG;
	}
	if (block_len != COMMS_BLOCK_LEN) {
		return DL_ERR_SIZE;
	}

	crc = (uint16_t)((block[DL_CRC_COVERED] << 8) | block[DL_CRC_COVERED + 1]);
	if (comms_crc16(block, DL_CRC_COVERED) != crc) {
		return DL_ERR_CRC;
	}
	len = (size_t)((block[0] << 8) | block[1]);
	if (len < COMMS_SP_MIN_PACKET || len > COMMS_DL_MAX_PACKET) {
		return DL_ERR_LEN;
	}

	*packet = block + COMMS_DL_LEN_FIELD;
	*packet_len = len;
	return DL_OK;
}

const char *dl_status_str(enum dl_status s)
{
	static const char *const names[DL_STATUS_COUNT] = {
		"ok", "bad argument", "wrong block size", "bad CRC", "bad length",
	};

	return ((unsigned int)s < DL_STATUS_COUNT) ? names[s] : "unknown";
}
