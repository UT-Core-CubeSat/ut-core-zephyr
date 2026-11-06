/**
 * @file space_packet.c
 * @ingroup comms
 * @brief CCSDS Space Packet header. See space_packet.h for the layout.
 */
#include "space_packet.h"

#include <errno.h>
#include <string.h>

#include "comms_config.h"

/* Byte 0 and byte 2 field positions, from CCSDS 133.0-B-2 section 4.1.3 (bit 0 is the MSB of byte 0). */
#define B0_VERSION_SHIFT 5
#define B0_VERSION_MASK  0x07
#define B0_TYPE          0x10
#define B0_SEC_HDR       0x08
#define B0_APID_HI_MASK  0x07
#define B2_FLAGS_SHIFT   6
#define B2_FLAGS_UNSEG   0x03 /* '11' */
#define B2_COUNT_HI_MASK 0x3F

enum sp_status sp_parse(const uint8_t *buf, size_t len, struct sp_packet *out)
{
	size_t data_len;

	if (buf == NULL || out == NULL) {
		return SP_ERR_ARG;
	}
	if (len < COMMS_SP_MIN_PACKET) {
		return SP_ERR_SHORT;
	}
	if (((buf[0] >> B0_VERSION_SHIFT) & B0_VERSION_MASK) != 0) {
		return SP_ERR_VERSION;
	}
	if ((buf[2] >> B2_FLAGS_SHIFT) != B2_FLAGS_UNSEG) {
		return SP_ERR_FLAGS;
	}
	data_len = (size_t)((buf[4] << 8) | buf[5]) + 1;
	if (COMMS_SP_HDR_LEN + data_len > len) {
		return SP_ERR_LEN;
	}

	out->is_tc = (buf[0] & B0_TYPE) != 0;
	out->sec_hdr = (buf[0] & B0_SEC_HDR) != 0;
	out->apid = (uint16_t)(((buf[0] & B0_APID_HI_MASK) << 8) | buf[1]);
	out->seq_count = (uint16_t)(((buf[2] & B2_COUNT_HI_MASK) << 8) | buf[3]);
	out->data = buf + COMMS_SP_HDR_LEN;
	out->data_len = data_len;
	out->total_len = COMMS_SP_HDR_LEN + data_len;
	return SP_OK;
}

int sp_build(uint8_t *out, size_t cap, bool is_tc, bool sec_hdr, uint16_t apid, uint16_t seq_count,
	     const uint8_t *data, size_t data_len)
{
	size_t total;

	if (out == NULL || data == NULL || data_len == 0 || data_len > SP_MAX_DATA_LEN ||
	    apid >= SP_APID_COUNT || seq_count > SP_SEQ_COUNT_MASK) {
		return -EINVAL;
	}
	total = COMMS_SP_HDR_LEN + data_len;
	if (total > cap) {
		return -EMSGSIZE;
	}

	out[0] = (uint8_t)((is_tc ? B0_TYPE : 0) | (sec_hdr ? B0_SEC_HDR : 0) |
			   ((apid >> 8) & B0_APID_HI_MASK)); /* version 000 */
	out[1] = (uint8_t)(apid & 0xFF);
	out[2] = (uint8_t)((B2_FLAGS_UNSEG << B2_FLAGS_SHIFT) | ((seq_count >> 8) & B2_COUNT_HI_MASK));
	out[3] = (uint8_t)(seq_count & 0xFF);
	out[4] = (uint8_t)(((data_len - 1) >> 8) & 0xFF);
	out[5] = (uint8_t)((data_len - 1) & 0xFF);
	memcpy(out + COMMS_SP_HDR_LEN, data, data_len);
	return (int)total;
}

void sp_seq_init(struct sp_seq *s)
{
	memset(s, 0, sizeof(*s));
}

uint16_t sp_seq_next(struct sp_seq *s, uint16_t apid)
{
	uint16_t count;

	if (apid >= SP_APID_COUNT) {
		return 0;
	}
	count = s->next[apid];
	s->next[apid] = (uint16_t)((count + 1) & SP_SEQ_COUNT_MASK);
	return count;
}

uint16_t sp_seq_lost(uint16_t expected, uint16_t received)
{
	return (uint16_t)((received - expected) & SP_SEQ_COUNT_MASK);
}

const char *sp_status_str(enum sp_status s)
{
	static const char *const names[SP_STATUS_COUNT] = {
		"ok", "bad argument", "too short", "bad version", "bad sequence flags", "bad data length",
	};

	return ((unsigned int)s < SP_STATUS_COUNT) ? names[s] : "unknown";
}
