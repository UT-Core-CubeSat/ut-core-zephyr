/** @file test_uplink_policy.c @brief Tests for can/uplink_policy (default-deny allowlist) */
#include <zephyr/ztest.h>

#include "can/uplink_policy.h"
#include "common/can_proto.h"
#include "test_util.h"

ZTEST_SUITE(uplink_policy, NULL, NULL, NULL, NULL, NULL);

static struct can_record rec(uint8_t src_id, uint8_t dst, uint8_t cls, uint8_t src_data, uint8_t op)
{
	struct can_record r = {.id = CAN_ID_FULL(0, src_id, dst, cls)};

	r.data[0] = src_data;
	r.data[1] = op;
	return r;
}

ZTEST(uplink_policy, test_ping_from_ground_to_cdh_is_allowed)
{
	struct can_record r = rec(GROUND_ID, CDH_ID, CLS_COMMAND, GROUND_ID, OP_PING);

	zassert_equal(policy_check(&r), POLICY_OK);
}

ZTEST(uplink_policy, test_the_first_broken_rule_is_reported)
{
	struct can_record r;

	r = rec(EPS_ID, CDH_ID, CLS_COMMAND, EPS_ID, OP_PING);
	zassert_equal(policy_check(&r), POLICY_ERR_SRC);
	r = rec(GROUND_ID, COMMS_ID, CLS_COMMAND, GROUND_ID, OP_PING);
	zassert_equal(policy_check(&r), POLICY_ERR_DST, "no loopback to COMMS itself");
	r = rec(GROUND_ID, CAN_BROADCAST, CLS_COMMAND, GROUND_ID, OP_PING);
	zassert_equal(policy_check(&r), POLICY_ERR_DST, "no broadcast from ground");
	r = rec(GROUND_ID, CDH_ID, CLS_TELEMETRY, GROUND_ID, OP_PING);
	zassert_equal(policy_check(&r), POLICY_ERR_CLASS);
	r = rec(GROUND_ID, CDH_ID, CLS_CMD_RESP, GROUND_ID, OP_PING);
	zassert_equal(policy_check(&r), POLICY_ERR_CLASS, "ground may not send responses");
	r = rec(GROUND_ID, EPS_ID, CLS_COMMAND, GROUND_ID, OP_PING);
	zassert_equal(policy_check(&r), POLICY_ERR_NOT_ALLOWED, "ping is allowed to CDH only");
	r = rec(GROUND_ID, CDH_ID, CLS_COMMAND, GROUND_ID, OP_PONG);
	zassert_equal(policy_check(&r), POLICY_ERR_NOT_ALLOWED);
}

ZTEST(uplink_policy, test_commands_that_change_state_are_refused_for_every_board)
{
	const uint8_t ops[] = {OP_REBOOT,        OP_SET_MODE,     OP_SET_PWR_STATE, OP_SET_EPS_LOAD,
			       OP_SET_WHEEL_RPM, OP_SET_MAG_DIPOLE, OP_SET_LED,     OP_BUTTON};
	const uint8_t boards[] = {CDH_ID, EPS_ID, ADCS_ID, MOTOR_ID, GNSS_ID, STAR_ID, SOLAR_ID};

	for (size_t i = 0; i < ARRAY_SIZE(ops); i++) {
		for (size_t j = 0; j < ARRAY_SIZE(boards); j++) {
			struct can_record r = rec(GROUND_ID, boards[j], CLS_COMMAND, GROUND_ID, ops[i]);

			zassert_equal(policy_check(&r), POLICY_ERR_NOT_ALLOWED, "op 0x%02x to 0x%02x", ops[i],
				      boards[j]);
		}
	}
}

/* Walk every destination, class and opcode with a ground source. Exactly one combination may pass. */
ZTEST(uplink_policy, test_exactly_one_combination_is_allowed_in_total)
{
	int allowed = 0;

	for (int dst = 0; dst < 256; dst++) {
		for (int cls = 0; cls < 64; cls++) {
			for (int op = 0; op < 256; op++) {
				struct can_record r = rec(GROUND_ID, dst, cls, GROUND_ID, op);

				if (policy_check(&r) == POLICY_OK) {
					allowed++;
					zassert_equal(dst, CDH_ID);
					zassert_equal(cls, CLS_COMMAND);
					zassert_equal(op, OP_PING);
				}
			}
		}
	}
	zassert_equal(allowed, 1);
}

ZTEST(uplink_policy, test_no_other_source_is_ever_allowed)
{
	for (int src = 0; src < 256; src++) {
		struct can_record r = rec(src, CDH_ID, CLS_COMMAND, src, OP_PING);

		if (src != GROUND_ID) {
			zassert_equal(policy_check(&r), POLICY_ERR_SRC, "src 0x%02x", src);
		}
	}
}

ZTEST(uplink_policy, test_priority_in_the_id_does_not_matter_to_the_policy)
{
	for (int p = 0; p < 8; p++) {
		struct can_record r = rec(GROUND_ID, CDH_ID, CLS_COMMAND, GROUND_ID, OP_PING);

		r.id |= CAN_PRIO(p);
		zassert_equal(policy_check(&r), POLICY_OK);
	}
}

ZTEST(uplink_policy, test_result_names)
{
	for (int i = 0; i < POLICY_RESULT_COUNT; i++) {
		zassert_true(strlen(policy_result_str(i)) > 0);
	}
	zassert_equal(strcmp(policy_result_str(POLICY_RESULT_COUNT + 3), "unknown"), 0);
}
