/**
 * @file uplink_policy.h
 * @ingroup comms
 * @brief Who may put what on the CAN bus through COMMS
 *
 * A command packet (APID 0x010) carries a CAN record from ground. Before it reaches the bus the record must
 * pass `policy_check()`. The rule is **default deny**: a record is accepted only if all of these hold.
 *  1. its source is GROUND_ID (the CAN node ID the ground station appears as),
 *  2. its destination is a real board: not COMMS itself (no loopback) and not broadcast,
 *  3. its class is CLS_COMMAND,
 *  4. (destination, class, opcode) is in the allowlist in uplink_policy.c.
 *
 * The MVP allowlist has one entry: OP_PING to CDH (read-only). Everything that changes board state stays off
 * the list until state gating with CDH exists (stage 7). See uplink_policy.c for the reasons, entry by entry.
 */
#ifndef COMMS_UPLINK_POLICY_H_
#define COMMS_UPLINK_POLICY_H_

#include "can/can_record.h"

enum policy_result {
	POLICY_OK = 0,
	POLICY_ERR_SRC,         /**< source is not GROUND_ID */
	POLICY_ERR_DST,         /**< destination is COMMS itself or broadcast */
	POLICY_ERR_CLASS,       /**< class is not CLS_COMMAND */
	POLICY_ERR_NOT_ALLOWED, /**< (destination, class, opcode) is not in the allowlist */
	POLICY_RESULT_COUNT
};

/** Check a record from ground. Returns POLICY_OK or the first rule it breaks, in the order above. */
enum policy_result policy_check(const struct can_record *r);

/** Short name of a result, for logs. */
const char *policy_result_str(enum policy_result r);

#endif /* COMMS_UPLINK_POLICY_H_ */
