/** @file test_can_record.c @brief Tests for can/can_record (the 12-byte CAN-equivalent record) */
#include <errno.h>
#include <string.h>
#include <zephyr/ztest.h>

#include "can/can_record.h"
#include "common/can_proto.h"
#include "test_util.h"

ZTEST_SUITE(can_record, NULL, NULL, NULL, NULL, NULL);

ZTEST(can_record, test_wire_layout_matches_the_spec)
{
	struct can_record r = {
		/* priority 1, src 0x10, dst 0x03, class 2:
		 * 1<<26 = 0x04000000, 0x10<<14 = 0x00040000, 3<<6 = 0xC0, class 2 */
		.id = CAN_ID_FULL(1, GROUND_ID, COMMS_ID, CLS_COMMAND),
		.data = {GROUND_ID, OP_PING, 1, 2, 3, 4, 5, 6},
	};
	uint8_t wire[CAN_RECORD_LEN];

	zassert_equal(r.id, 0x040400C2u, "the test's own arithmetic");
	can_record_pack(wire, &r);
	zassert_equal(wire[0], 0x04, "the 29-bit ID is big-endian");
	zassert_equal(wire[1], 0x04);
	zassert_equal(wire[2], 0x00);
	zassert_equal(wire[3], 0xC2);
	zassert_equal(wire[4], GROUND_ID, "data byte 0 is src");
	zassert_equal(wire[5], OP_PING, "data byte 1 is the opcode");
	zassert_equal(wire[6], 1);
	zassert_equal(wire[11], 6);
}

ZTEST(can_record, test_round_trip_and_accessors)
{
	struct test_rng rng = {.s = 0x600DCAFE};

	for (int i = 0; i < 2000; i++) {
		struct can_record a;
		struct can_record b;
		uint8_t wire[CAN_RECORD_LEN];
		const uint8_t prio = test_rng_next(&rng) & 7;
		const uint8_t src = test_rng_next(&rng) & 0xFF;
		const uint8_t dst = test_rng_next(&rng) & 0xFF;
		const uint8_t cls = test_rng_next(&rng) & 0x3F;

		a.id = CAN_ID_FULL(prio, src, dst, cls);
		test_rng_fill(&rng, a.data, sizeof(a.data));
		can_record_pack(wire, &a);
		zassert_equal(can_record_unpack(wire, sizeof(wire), &b), REC_OK);
		zassert_equal(a.id, b.id);
		zassert_mem_equal(a.data, b.data, 8);
		zassert_equal(can_record_prio(&b), prio);
		zassert_equal(can_record_src(&b), src);
		zassert_equal(can_record_dst(&b), dst);
		zassert_equal(can_record_class(&b), cls);
		zassert_equal(can_record_op(&b), b.data[1]);
	}
}

ZTEST(can_record, test_unpack_rejects_bad_ids_and_lengths)
{
	uint8_t wire[CAN_RECORD_LEN] = {0};
	struct can_record r;

	zassert_equal(can_record_unpack(wire, sizeof(wire), &r), REC_OK, "all zero is a valid ID");

	for (int bit = 29; bit < 32; bit++) {
		memset(wire, 0, sizeof(wire));
		wire[0] = (uint8_t)(1U << (bit - 24));
		zassert_equal(can_record_unpack(wire, sizeof(wire), &r), REC_ERR_ID, "bit %d", bit);
	}
	for (int bit = 22; bit <= 25; bit++) {
		uint32_t id = 1U << bit;

		wire[0] = (uint8_t)(id >> 24);
		wire[1] = (uint8_t)(id >> 16);
		wire[2] = (uint8_t)(id >> 8);
		wire[3] = (uint8_t)id;
		zassert_equal(can_record_unpack(wire, sizeof(wire), &r), REC_ERR_ID, "reserved bit %d", bit);
	}

	memset(wire, 0, sizeof(wire));
	zassert_equal(can_record_unpack(wire, 11, &r), REC_ERR_LEN);
	zassert_equal(can_record_unpack(wire, 13, &r), REC_ERR_LEN);
	zassert_equal(can_record_unpack(wire, 0, &r), REC_ERR_LEN);
	zassert_equal(can_record_unpack(NULL, 12, &r), REC_ERR_ARG);
	zassert_equal(can_record_unpack(wire, 12, NULL), REC_ERR_ARG);
}

ZTEST(can_record, test_every_valid_id_bit_is_accepted)
{
	uint8_t wire[CAN_RECORD_LEN] = {0};
	struct can_record r;

	/* the 22 bits that are allowed: 0-21 and 26-28 */
	for (int bit = 0; bit < 29; bit++) {
		const uint32_t id = 1U << bit;
		const bool reserved = (bit >= 22 && bit <= 25);

		wire[0] = (uint8_t)(id >> 24);
		wire[1] = (uint8_t)(id >> 16);
		wire[2] = (uint8_t)(id >> 8);
		wire[3] = (uint8_t)id;
		zassert_equal(can_record_unpack(wire, sizeof(wire), &r), reserved ? REC_ERR_ID : REC_OK,
			      "bit %d", bit);
	}
}

ZTEST(can_record, test_frame_conversion_round_trip)
{
	struct can_frame f = {0};
	struct can_frame g;
	struct can_record r;
	struct can_record s;

	f.id = CAN_ID_FULL(2, GROUND_ID, EPS_ID, CLS_COMMAND);
	f.flags = CAN_FRAME_IDE;
	can_fill_payload(&f, GROUND_ID, OP_SET_EPS_LOAD, 3, 1, 0, 0, 0, 0);

	zassert_equal(can_record_from_frame(&f, &r), REC_OK);
	zassert_equal(r.id, f.id);
	zassert_mem_equal(r.data, f.data, 8);
	zassert_equal(can_record_dst(&r), EPS_ID);
	zassert_equal(can_record_op(&r), OP_SET_EPS_LOAD);

	zassert_equal(can_record_to_frame(&r, &g), REC_OK);
	zassert_equal(g.id, f.id);
	zassert_equal(g.dlc, 8);
	zassert_true(g.flags & CAN_FRAME_IDE, "must be an extended frame");
	zassert_false(g.flags & (CAN_FRAME_RTR | CAN_FRAME_FDF));
	zassert_mem_equal(g.data, f.data, 8);

	zassert_equal(can_record_from_frame(&g, &s), REC_OK);
	zassert_mem_equal(&s, &r, sizeof(r));
}

ZTEST(can_record, test_only_classic_extended_8_byte_data_frames_become_records)
{
	struct can_frame f = {0};
	struct can_record r;

	f.id = CAN_ID_FULL(1, GROUND_ID, 0x03, 2);
	can_fill_payload(&f, GROUND_ID, OP_PING, 0, 0, 0, 0, 0, 0);
	zassert_equal(can_record_from_frame(&f, &r), REC_OK);

	f.flags = 0; /* standard ID */
	zassert_equal(can_record_from_frame(&f, &r), REC_ERR_FRAME);
	f.flags = CAN_FRAME_IDE | CAN_FRAME_RTR;
	zassert_equal(can_record_from_frame(&f, &r), REC_ERR_FRAME, "remote frame");
	f.flags = CAN_FRAME_IDE | CAN_FRAME_FDF;
	zassert_equal(can_record_from_frame(&f, &r), REC_ERR_FRAME, "CAN FD");
	f.flags = CAN_FRAME_IDE;
	for (uint8_t dlc = 0; dlc <= 7; dlc++) {
		f.dlc = dlc;
		zassert_equal(can_record_from_frame(&f, &r), REC_ERR_FRAME, "dlc %u", dlc);
	}
	f.dlc = 8;
	f.id |= 1U << 23; /* reserved bit */
	zassert_equal(can_record_from_frame(&f, &r), REC_ERR_ID);
	zassert_equal(can_record_from_frame(NULL, &r), REC_ERR_ARG);
	zassert_equal(can_record_from_frame(&f, NULL), REC_ERR_ARG);
}

ZTEST(can_record, test_to_frame_rejects_a_bad_id)
{
	struct can_record r = {.id = 1U << 24};
	struct can_frame f;

	zassert_equal(can_record_to_frame(&r, &f), REC_ERR_ID);
	zassert_equal(can_record_to_frame(NULL, &f), REC_ERR_ARG);
	zassert_equal(can_record_to_frame(&r, NULL), REC_ERR_ARG);
}

ZTEST(can_record, test_record_count)
{
	zassert_equal(can_record_count(12), 1);
	zassert_equal(can_record_count(24), 2);
	zassert_equal(can_record_count(180), 15, "the largest clear uplink data field in whole records");
	zassert_equal(can_record_count(0), -EINVAL);
	zassert_equal(can_record_count(11), -EINVAL);
	zassert_equal(can_record_count(13), -EINVAL);
	zassert_equal(can_record_count(25), -EINVAL);
}

ZTEST(can_record, test_status_names)
{
	zassert_str_equal(rec_status_str(REC_OK), "ok");
	zassert_str_equal(rec_status_str(REC_STATUS_COUNT), "unknown");
	for (int s = 0; s < REC_STATUS_COUNT; s++) {
		zassert_true(strlen(rec_status_str((enum rec_status)s)) > 0);
	}
}
