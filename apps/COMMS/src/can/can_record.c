/**
 * @file can_record.c
 * @ingroup comms
 * @brief The 12-byte CAN-equivalent record. See can_record.h for the layout.
 */
#include "can_record.h"

#include <errno.h>
#include <string.h>

void can_record_pack(uint8_t *out, const struct can_record *r)
{
	out[0] = (uint8_t)(r->id >> 24);
	out[1] = (uint8_t)(r->id >> 16);
	out[2] = (uint8_t)(r->id >> 8);
	out[3] = (uint8_t)(r->id);
	memcpy(out + 4, r->data, sizeof(r->data));
}

enum rec_status can_record_unpack(const uint8_t *in, size_t len, struct can_record *r)
{
	uint32_t id;

	if (in == NULL || r == NULL) {
		return REC_ERR_ARG;
	}
	if (len != CAN_RECORD_LEN) {
		return REC_ERR_LEN;
	}
	id = ((uint32_t)in[0] << 24) | ((uint32_t)in[1] << 16) | ((uint32_t)in[2] << 8) | in[3];
	if (id & ~CAN_RECORD_ID_VALID_MASK) {
		return REC_ERR_ID;
	}
	r->id = id;
	memcpy(r->data, in + 4, sizeof(r->data));
	return REC_OK;
}

enum rec_status can_record_from_frame(const struct can_frame *f, struct can_record *r)
{
	if (f == NULL || r == NULL) {
		return REC_ERR_ARG;
	}
	if (!(f->flags & CAN_FRAME_IDE) || (f->flags & (CAN_FRAME_RTR | CAN_FRAME_FDF)) || f->dlc != 8) {
		return REC_ERR_FRAME;
	}
	if (f->id & ~CAN_RECORD_ID_VALID_MASK) {
		return REC_ERR_ID;
	}
	r->id = f->id;
	memcpy(r->data, f->data, sizeof(r->data));
	return REC_OK;
}

enum rec_status can_record_to_frame(const struct can_record *r, struct can_frame *f)
{
	if (r == NULL || f == NULL) {
		return REC_ERR_ARG;
	}
	if (r->id & ~CAN_RECORD_ID_VALID_MASK) {
		return REC_ERR_ID;
	}
	memset(f, 0, sizeof(*f));
	f->id = r->id;
	f->flags = CAN_FRAME_IDE;
	f->dlc = 8;
	memcpy(f->data, r->data, sizeof(r->data));
	return REC_OK;
}

int can_record_count(size_t data_len)
{
	if (data_len == 0 || (data_len % CAN_RECORD_LEN) != 0) {
		return -EINVAL;
	}
	return (int)(data_len / CAN_RECORD_LEN);
}

const char *rec_status_str(enum rec_status s)
{
	static const char *const names[REC_STATUS_COUNT] = {
		"ok", "bad argument", "bad record length", "bad CAN ID", "not an 8-byte extended data frame",
	};

	return ((unsigned int)s < REC_STATUS_COUNT) ? names[s] : "unknown";
}
