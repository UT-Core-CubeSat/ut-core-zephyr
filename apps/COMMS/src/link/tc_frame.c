/**
 * @file tc_frame.c
 * @ingroup comms
 * @brief Uplink TC Transfer Frame, Type-BD. See tc_frame.h for the layout.
 */
#include "tc_frame.h"

#include <errno.h>
#include <string.h>

#include "comms_config.h"
#include "crc16.h"

/* Header field positions, from CCSDS 232.0-B-4 figure 4-2 (bit 0 is the MSB of byte 0). */
#define H0_VERSION_SHIFT 6
#define H0_VERSION_MASK  0x03
#define H0_BYPASS        0x20
#define H0_CONTROL       0x10
#define H0_SPARE_MASK    0x0C
#define H0_SCID_HI_MASK  0x03
#define H2_VCID_SHIFT    2
#define H2_LEN_HI_MASK   0x03

/** The frame length field is 10 bits: it counts total frame bytes minus one. */
#define TC_MAX_FRAME_LEN 1024

enum tc_status tc_frame_parse(const uint8_t *block, size_t block_len, struct tc_frame *out)
{
	size_t frame_len;
	uint16_t fecf;
	uint16_t scid;
	uint8_t vcid;

	if (block == NULL || out == NULL) {
		return TC_ERR_ARG;
	}
	if (block_len < COMMS_TC_HDR_LEN + COMMS_TC_FECF_LEN) {
		return TC_ERR_SHORT;
	}

	frame_len = ((size_t)(block[2] & H2_LEN_HI_MASK) << 8 | block[3]) + 1;
	if (frame_len < COMMS_TC_HDR_LEN + COMMS_TC_FECF_LEN || frame_len > block_len) {
		return TC_ERR_LEN;
	}

	fecf = (uint16_t)((block[frame_len - 2] << 8) | block[frame_len - 1]);
	if (comms_crc16(block, frame_len - COMMS_TC_FECF_LEN) != fecf) {
		return TC_ERR_FECF;
	}

	/* The frame is intact. Now the policy checks. */
	if (((block[0] >> H0_VERSION_SHIFT) & H0_VERSION_MASK) != 0) {
		return TC_ERR_VERSION;
	}
	if ((block[0] & H0_SPARE_MASK) != 0) {
		return TC_ERR_SPARE;
	}
	if (!(block[0] & H0_BYPASS) || (block[0] & H0_CONTROL)) {
		return TC_ERR_TYPE;
	}
	scid = (uint16_t)(((block[0] & H0_SCID_HI_MASK) << 8) | block[1]);
	if (scid != COMMS_TC_SCID) {
		return TC_ERR_SCID;
	}
	vcid = (uint8_t)(block[2] >> H2_VCID_SHIFT);
	if (vcid != COMMS_TC_VCID) {
		return TC_ERR_VCID;
	}

	out->scid = scid;
	out->vcid = vcid;
	out->seq = block[4];
	out->bypass = true;
	out->control = false;
	out->data = block + COMMS_TC_HDR_LEN;
	out->data_len = frame_len - COMMS_TC_HDR_LEN - COMMS_TC_FECF_LEN;
	out->frame_len = frame_len;
	return TC_OK;
}

int tc_frame_build(uint8_t *block, uint16_t scid, uint8_t vcid, const uint8_t *data, size_t data_len)
{
	size_t frame_len;
	uint16_t fecf;

	if (block == NULL || (data == NULL && data_len > 0) || scid > 0x3FF || vcid > 0x3F) {
		return -EINVAL;
	}
	frame_len = COMMS_TC_HDR_LEN + data_len + COMMS_TC_FECF_LEN;
	if (frame_len > COMMS_BLOCK_LEN || frame_len > TC_MAX_FRAME_LEN) {
		return -EMSGSIZE;
	}

	memset(block, 0, COMMS_BLOCK_LEN);
	block[0] = (uint8_t)(H0_BYPASS | ((scid >> 8) & H0_SCID_HI_MASK)); /* version 00, Type-BD */
	block[1] = (uint8_t)(scid & 0xFF);
	block[2] = (uint8_t)((vcid << H2_VCID_SHIFT) | (((frame_len - 1) >> 8) & H2_LEN_HI_MASK));
	block[3] = (uint8_t)((frame_len - 1) & 0xFF);
	block[4] = 0; /* frame sequence number: 0 for Type-B frames */
	if (data_len > 0) {
		memcpy(block + COMMS_TC_HDR_LEN, data, data_len);
	}
	fecf = comms_crc16(block, frame_len - COMMS_TC_FECF_LEN);
	block[frame_len - 2] = (uint8_t)(fecf >> 8);
	block[frame_len - 1] = (uint8_t)(fecf & 0xFF);
	return 0;
}

const char *tc_status_str(enum tc_status s)
{
	static const char *const names[TC_STATUS_COUNT] = {
		"ok", "bad argument", "too short", "bad frame length", "bad FECF",
		"bad version", "bad spare bits", "not Type-BD", "wrong spacecraft ID",
		"wrong virtual channel",
	};

	return ((unsigned int)s < TC_STATUS_COUNT) ? names[s] : "unknown";
}
