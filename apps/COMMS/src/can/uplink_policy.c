/** @file uplink_policy.c @ingroup comms @brief Uplink allowlist, see uplink_policy.h */
#include "uplink_policy.h"

#include <stddef.h>

#include <zephyr/sys/util.h>

#include "common/can_proto.h"

struct allow_entry {
	uint8_t dst;
	uint8_t cls;
	uint8_t op;
};

/*
 * The allowlist. Add an entry only after the target board has agreed to accept the command from GROUND_ID.
 *
 * Enabled:
 *   CDH  OP_PING          read-only, CDH answers OP_PONG. The MVP CAN_CMD test.
 *
 * Not enabled yet (would be a one-line addition each):
 *   CDH  OP_SET_LED       visible and harmless, handy for a bench demo.
 *
 * Kept off until state gating with CDH exists (stage 7), because they change board state:
 *   OP_REBOOT, OP_SET_MODE (CDH does not validate the value), OP_SET_PWR_STATE, OP_SET_EPS_LOAD,
 *   OP_SET_WHEEL_RPM, OP_SET_MAG_DIPOLE.
 */
static const struct allow_entry allowlist[] = {
	{CDH_ID, CLS_COMMAND, OP_PING},
};

enum policy_result policy_check(const struct can_record *r)
{
	uint8_t dst = can_record_dst(r);

	if (can_record_src(r) != GROUND_ID) {
		return POLICY_ERR_SRC;
	}
	if (dst == COMMS_ID || dst == CAN_BROADCAST) {
		return POLICY_ERR_DST;
	}
	if (can_record_class(r) != CLS_COMMAND) {
		return POLICY_ERR_CLASS;
	}
	for (size_t i = 0; i < ARRAY_SIZE(allowlist); i++) {
		if (allowlist[i].dst == dst && allowlist[i].cls == can_record_class(r) &&
		    allowlist[i].op == can_record_op(r)) {
			return POLICY_OK;
		}
	}
	return POLICY_ERR_NOT_ALLOWED;
}

const char *policy_result_str(enum policy_result r)
{
	static const char *const names[POLICY_RESULT_COUNT] = {
		"ok", "source is not ground", "bad destination", "not a command", "not on the allowlist",
	};

	return (unsigned int)r < POLICY_RESULT_COUNT ? names[r] : "unknown";
}
