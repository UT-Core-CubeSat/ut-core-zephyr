/**
 * @file can_ping.h
 * @ingroup comms
 * @brief The CAN ping and pong records used by the bench CAN_CMD test
 *
 * Ground uplinks a ping record in a command packet (APID 0x010). COMMS puts it on the CAN bus, the target
 * board (CDH for the MVP) answers with a pong addressed to ground, and COMMS sends that down in a telemetry
 * packet (APID 0x020). Layout, using the standard opcodes OP_PING and OP_PONG of common/can_proto.h:
 * ```
 * ping: src GROUND_ID, dst <board>, class CLS_COMMAND,  data = {GROUND_ID, OP_PING, t0..t5}
 * pong: src <board>,   dst GROUND_ID, class CLS_CMD_RESP, data = {<board>,   OP_PONG, t0..t5}
 * ```
 * The six token bytes t0..t5 are ground's choice and come back unchanged, so ground can match each pong to
 * its ping. Ground software builds the same bytes (PROTOCOL.md section 14).
 */
#ifndef COMMS_CAN_PING_H_
#define COMMS_CAN_PING_H_

#include <stdbool.h>
#include <stdint.h>

#include "can/can_record.h"

#define COMMS_PING_TOKEN_LEN 6 /**< p2..p7 of the CAN payload */

/** Build the ping record ground uplinks, for target board @p dst. Priority is left at 0 (the bridge sets it). */
void comms_can_ping_record(struct can_record *out, uint8_t dst, const uint8_t token[COMMS_PING_TOKEN_LEN]);

/** True if @p rec is a pong from board @p from, addressed to ground, echoing @p token. */
bool comms_can_pong_check(const struct can_record *rec, uint8_t from,
			  const uint8_t token[COMMS_PING_TOKEN_LEN]);

#endif /* COMMS_CAN_PING_H_ */
