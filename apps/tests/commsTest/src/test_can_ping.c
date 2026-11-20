/** @file test_can_ping.c @brief Tests for can/can_ping (the CAN ping and pong records) */
#include <zephyr/ztest.h>

#include "can/can_ping.h"
#include "can/uplink_policy.h"
#include "common/can_proto.h"
#include "test_util.h"

ZTEST_SUITE(can_ping, NULL, NULL, NULL, NULL, NULL);

static const uint8_t token[COMMS_PING_TOKEN_LEN] = {0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0xFF};

/* What a board answers: the same as the CDH handler, written out here independently. */
static struct can_record pong(uint8_t from, const uint8_t *tok)
{
	struct can_record r = {.id = CAN_ID_FULL(3, from, GROUND_ID, CLS_CMD_RESP)};

	r.data[0] = from;
	r.data[1] = OP_PONG;
	memcpy(&r.data[2], tok, COMMS_PING_TOKEN_LEN);
	return r;
}

ZTEST(can_ping, test_ping_record_layout)
{
	struct can_record r;

	comms_can_ping_record(&r, CDH_ID, token);
	zassert_equal(can_record_src(&r), GROUND_ID);
	zassert_equal(can_record_dst(&r), CDH_ID);
	zassert_equal(can_record_class(&r), CLS_COMMAND);
	zassert_equal(r.data[0], GROUND_ID);
	zassert_equal(r.data[1], OP_PING);
	zassert_mem_equal(&r.data[2], token, sizeof(token));
	zassert_equal(r.id & ~CAN_RECORD_ID_VALID_MASK, 0);
}

ZTEST(can_ping, test_the_ping_record_passes_the_policy_for_cdh_only)
{
	struct can_record r;

	comms_can_ping_record(&r, CDH_ID, token);
	zassert_equal(policy_check(&r), POLICY_OK);
	comms_can_ping_record(&r, EPS_ID, token);
	zassert_equal(policy_check(&r), POLICY_ERR_NOT_ALLOWED);
}

ZTEST(can_ping, test_ping_record_survives_the_wire_format)
{
	struct can_record r, back;
	uint8_t wire[CAN_RECORD_LEN];

	comms_can_ping_record(&r, CDH_ID, token);
	can_record_pack(wire, &r);
	zassert_equal(can_record_unpack(wire, sizeof(wire), &back), REC_OK);
	zassert_equal(back.id, r.id);
	zassert_mem_equal(back.data, r.data, 8);
}

ZTEST(can_ping, test_matching_pong_is_recognised)
{
	struct can_record p = pong(CDH_ID, token);

	zassert_true(comms_can_pong_check(&p, CDH_ID, token));
}

ZTEST(can_ping, test_pong_with_anything_wrong_is_rejected)
{
	uint8_t other[COMMS_PING_TOKEN_LEN];
	struct can_record p;

	memcpy(other, token, sizeof(other));
	other[5] ^= 1;
	p = pong(CDH_ID, other);
	zassert_false(comms_can_pong_check(&p, CDH_ID, token), "token differs in the last byte");

	p = pong(CDH_ID, token);
	zassert_false(comms_can_pong_check(&p, EPS_ID, token), "wrong board");

	p = pong(CDH_ID, token);
	p.data[1] = OP_PING;
	zassert_false(comms_can_pong_check(&p, CDH_ID, token), "not a pong opcode");

	p = pong(CDH_ID, token);
	p.id = CAN_ID_FULL(3, CDH_ID, GROUND_ID, CLS_COMMAND);
	zassert_false(comms_can_pong_check(&p, CDH_ID, token), "wrong class");

	p = pong(CDH_ID, token);
	p.id = CAN_ID_FULL(3, CDH_ID, EPS_ID, CLS_CMD_RESP);
	zassert_false(comms_can_pong_check(&p, CDH_ID, token), "not addressed to ground");

	p = pong(CDH_ID, token);
	p.data[0] = EPS_ID;
	zassert_false(comms_can_pong_check(&p, CDH_ID, token), "data source differs from ID source");
}
