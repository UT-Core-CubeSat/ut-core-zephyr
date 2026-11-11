/** @file test_can_bridge.c @brief Tests for can/can_bridge with a fake CAN bus */
#include <errno.h>
#include <zephyr/ztest.h>

#include "can/can_bridge.h"
#include "can/can_ping.h"
#include "common/can_proto.h"
#include "link/space_packet.h"
#include "test_fake_can.h"
#include "test_util.h"

static struct can_bridge br;

static void before(void *f)
{
	ARG_UNUSED(f);
	fake_can_attach(&br);
}

ZTEST_SUITE(can_bridge, NULL, NULL, before, NULL, NULL);

static const uint8_t token[COMMS_PING_TOKEN_LEN] = {1, 2, 3, 4, 5, 6};

static struct can_frame frame_from(uint32_t id, uint8_t src, uint8_t op, const uint8_t *tok)
{
	struct can_frame f = {.id = id};

	can_fill_payload(&f, src, op, tok[0], tok[1], tok[2], tok[3], tok[4], tok[5]);
	return f;
}

/* ---- ground to bus ---- */

ZTEST(can_bridge, test_forward_sends_one_extended_8_byte_frame_with_the_record_contents)
{
	struct can_record r;

	comms_can_ping_record(&r, CDH_ID, token);
	zassert_ok(can_bridge_forward(&br, &r));
	zassert_equal(fcan.count, 1);
	zassert_equal(fcan.frames[0].dlc, 8);
	zassert_true(fcan.frames[0].flags & CAN_FRAME_IDE);
	zassert_equal(fcan.frames[0].data[0], GROUND_ID);
	zassert_equal(fcan.frames[0].data[1], OP_PING);
	zassert_mem_equal(&fcan.frames[0].data[2], token, sizeof(token));
	zassert_equal(br.stats.forwarded, 1);
}

ZTEST(can_bridge, test_forward_replaces_the_priority_whatever_ground_wrote)
{
	for (int p = 0; p < 8; p++) {
		struct can_record r;
		uint32_t id;

		comms_can_ping_record(&r, CDH_ID, token);
		r.id = (r.id & ~CAN_PRIO(7)) | CAN_PRIO(p);
		zassert_ok(can_bridge_forward(&br, &r));
		id = fcan.frames[p].id;
		zassert_equal((id >> 26) & 7, COMMS_CAN_UPLINK_PRIO, "ground wrote priority %d", p);
		zassert_equal((id >> 14) & 0xFF, GROUND_ID, "source untouched");
		zassert_equal((id >> 6) & 0xFF, CDH_ID, "destination untouched");
		zassert_equal(id & 0x3F, CLS_COMMAND, "class untouched");
	}
}

ZTEST(can_bridge, test_forward_does_not_change_the_callers_record)
{
	struct can_record r, copy;

	comms_can_ping_record(&r, CDH_ID, token);
	r.id |= CAN_PRIO(0);
	copy = r;
	zassert_ok(can_bridge_forward(&br, &r));
	zassert_equal(r.id, copy.id);
}

ZTEST(can_bridge, test_forward_send_error_is_returned_and_counted)
{
	struct can_record r;

	comms_can_ping_record(&r, CDH_ID, token);
	fcan.result = -EAGAIN;
	zassert_equal(can_bridge_forward(&br, &r), -EAGAIN);
	zassert_equal(br.stats.send_errors, 1);
	zassert_equal(br.stats.forwarded, 0);
}

ZTEST(can_bridge, test_forward_rejects_an_invalid_id_without_sending)
{
	struct can_record r;

	comms_can_ping_record(&r, CDH_ID, token);
	r.id |= BIT(22); /* reserved */
	zassert_equal(can_bridge_forward(&br, &r), -EINVAL);
	zassert_equal(fcan.attempts, 0);
	zassert_equal(br.stats.bad_record, 1);
}

/* ---- bus to ground ---- */

ZTEST(can_bridge, test_a_frame_for_ground_becomes_a_telemetry_item_with_counter_and_record)
{
	struct can_frame f = frame_from(CAN_ID_FULL(3, CDH_ID, GROUND_ID, CLS_CMD_RESP), CDH_ID, OP_PONG,
					token);
	struct dl_item it = {0};
	struct can_record rec;

	zassert_equal(can_bridge_on_frame(&br, &f, &it), 1);
	zassert_equal(it.apid, APID_DL_TLM);
	zassert_equal(it.len, 4 + CAN_RECORD_LEN);
	zassert_equal(it.data[0], 0);
	zassert_equal(it.data[3], 0, "first counter value is 0");
	zassert_equal(can_record_unpack(&it.data[4], CAN_RECORD_LEN, &rec), REC_OK);
	zassert_true(comms_can_pong_check(&rec, CDH_ID, token));
	zassert_equal(br.stats.replies, 1);
}

ZTEST(can_bridge, test_the_counter_is_big_endian_and_increments_per_reply)
{
	struct can_frame f = frame_from(CAN_ID_FULL(3, CDH_ID, GROUND_ID, CLS_CMD_RESP), CDH_ID, OP_PONG,
					token);
	struct dl_item it;

	atomic_set(&fcan_counter, 0x01020304);
	zassert_equal(can_bridge_on_frame(&br, &f, &it), 1);
	zassert_equal(it.data[0], 0x01);
	zassert_equal(it.data[1], 0x02);
	zassert_equal(it.data[2], 0x03);
	zassert_equal(it.data[3], 0x04);
	zassert_equal(can_bridge_on_frame(&br, &f, &it), 1);
	zassert_equal(it.data[3], 0x05);
	zassert_equal(atomic_get(&fcan_counter), 0x01020306);
}

ZTEST(can_bridge, test_frames_for_other_nodes_are_ignored_and_use_no_counter_value)
{
	const uint8_t dsts[] = {CDH_ID, EPS_ID, COMMS_ID, CAN_BROADCAST};
	struct dl_item it;

	for (size_t i = 0; i < ARRAY_SIZE(dsts); i++) {
		struct can_frame f = frame_from(CAN_ID_FULL(3, EPS_ID, dsts[i], CLS_HEARTBEAT), EPS_ID,
						OP_HEARTBEAT, token);

		zassert_equal(can_bridge_on_frame(&br, &f, &it), 0, "dst 0x%02x", dsts[i]);
	}
	zassert_equal(br.stats.ignored_dst, ARRAY_SIZE(dsts));
	zassert_equal(atomic_get(&fcan_counter), 0);
}

ZTEST(can_bridge, test_frames_that_are_not_classic_extended_8_byte_data_are_ignored)
{
	struct can_frame f = frame_from(CAN_ID_FULL(3, CDH_ID, GROUND_ID, CLS_CMD_RESP), CDH_ID, OP_PONG,
					token);
	struct dl_item it;

	f.flags = 0; /* standard ID */
	zassert_equal(can_bridge_on_frame(&br, &f, &it), 0);
	f.flags = CAN_FRAME_IDE;
	f.dlc = 4;
	zassert_equal(can_bridge_on_frame(&br, &f, &it), 0);
	f.dlc = 8;
	f.flags = CAN_FRAME_IDE | CAN_FRAME_RTR;
	zassert_equal(can_bridge_on_frame(&br, &f, &it), 0);
	zassert_equal(br.stats.ignored_frame, 3);
	zassert_equal(br.stats.replies, 0);
}
