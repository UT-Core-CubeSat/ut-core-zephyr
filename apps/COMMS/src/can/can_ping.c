/** @file can_ping.c @ingroup comms @brief CAN ping and pong records, see can_ping.h */
#include "can_ping.h"

#include <string.h>

#include "common/can_proto.h"

void comms_can_ping_record(struct can_record *out, uint8_t dst, const uint8_t token[COMMS_PING_TOKEN_LEN])
{
	out->id = CAN_ID_FULL(0, GROUND_ID, dst, CLS_COMMAND);
	out->data[0] = GROUND_ID;
	out->data[1] = OP_PING;
	memcpy(&out->data[2], token, COMMS_PING_TOKEN_LEN);
}

bool comms_can_pong_check(const struct can_record *rec, uint8_t from,
			  const uint8_t token[COMMS_PING_TOKEN_LEN])
{
	return can_record_src(rec) == from && can_record_dst(rec) == GROUND_ID &&
	       can_record_class(rec) == CLS_CMD_RESP && rec->data[0] == from && rec->data[1] == OP_PONG &&
	       memcmp(&rec->data[2], token, COMMS_PING_TOKEN_LEN) == 0;
}
