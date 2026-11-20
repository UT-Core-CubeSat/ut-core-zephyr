/** @file test_telemetry.c @brief Tests for can/telemetry (the cache of what the boards broadcast) */
#include <zephyr/ztest.h>

#include "app/downlink_item.h"
#include "can/telemetry.h"
#include "common/can_proto.h"
#include "test_util.h"

static struct tlm_cache cache;

static void before(void *f)
{
	ARG_UNUSED(f);
	tlm_cache_init(&cache);
}

ZTEST_SUITE(telemetry, NULL, NULL, before, NULL, NULL);

static struct can_frame frame(uint8_t src, uint8_t dst, uint8_t cls, uint8_t op, uint8_t value)
{
	struct can_frame f = {.id = CAN_ID_FULL(3, src, dst, cls)};

	can_fill_payload(&f, src, op, value, 0, 0, 0, 0, 0);
	return f;
}

static struct can_frame bcast(uint8_t src, uint8_t cls, uint8_t op, uint8_t value)
{
	return frame(src, CAN_BROADCAST, cls, op, value);
}

ZTEST(telemetry, test_a_broadcast_heartbeat_is_stored_and_found)
{
	struct can_frame f = bcast(CDH_ID, CLS_HEARTBEAT, OP_HEARTBEAT, 7);
	struct can_record r;

	zassert_equal(tlm_cache_on_frame(&cache, &f, 1000), 1);
	zassert_true(tlm_cache_get(&cache, CDH_ID, CLS_HEARTBEAT, OP_HEARTBEAT, 1000, 100, &r));
	zassert_equal(r.data[2], 7);
	zassert_equal(cache.stats.stored, 1);
}

ZTEST(telemetry, test_the_same_key_is_replaced_not_added)
{
	struct can_frame a = bcast(EPS_ID, CLS_TELEMETRY, OP_ADCS_WHEEL_RPM, 1);
	struct can_frame b = bcast(EPS_ID, CLS_TELEMETRY, OP_ADCS_WHEEL_RPM, 2);
	struct can_record out[4];

	zassert_equal(tlm_cache_on_frame(&cache, &a, 10), 1);
	zassert_equal(tlm_cache_on_frame(&cache, &b, 20), 1);
	zassert_equal(tlm_cache_collect(&cache, 20, 1000, out, 4), 1);
	zassert_equal(out[0].data[2], 2, "the newer record wins");
	zassert_equal(cache.stats.stored, 1);
	zassert_equal(cache.stats.updated, 1);
}

ZTEST(telemetry, test_source_class_and_opcode_each_make_a_different_key)
{
	struct can_frame fs[] = {
		bcast(CDH_ID, CLS_TELEMETRY, 0x10, 0),
		bcast(EPS_ID, CLS_TELEMETRY, 0x10, 0),
		bcast(CDH_ID, CLS_HEALTH, 0x10, 0),
		bcast(CDH_ID, CLS_TELEMETRY, 0x11, 0),
	};
	struct can_record out[8];

	for (size_t i = 0; i < ARRAY_SIZE(fs); i++) {
		zassert_equal(tlm_cache_on_frame(&cache, &fs[i], 0), 1);
	}
	zassert_equal(tlm_cache_collect(&cache, 0, 1000, out, 8), 4);
}

ZTEST(telemetry, test_every_kept_class_is_kept_and_no_other)
{
	const uint8_t kept[] = {CLS_HEARTBEAT, CLS_TELEMETRY, CLS_HEALTH, CLS_SOH_CORE, CLS_SOH_CAN_STATS};

	for (unsigned int cls = 0; cls < 64; cls++) {
		struct can_frame f = bcast(GNSS_ID, cls, 0x22, 0);
		bool want = false;

		tlm_cache_init(&cache);
		for (size_t i = 0; i < sizeof(kept); i++) {
			want |= kept[i] == cls;
		}
		zassert_equal(tlm_cache_on_frame(&cache, &f, 0), want ? 1 : 0, "class %u", cls);
	}
}

ZTEST(telemetry, test_frames_to_one_node_commands_and_answers_are_not_cached)
{
	struct can_frame to_comms = frame(CDH_ID, COMMS_ID, CLS_TELEMETRY, 0x10, 0);
	struct can_frame to_ground = frame(CDH_ID, GROUND_ID, CLS_TELEMETRY, 0x10, 0);
	struct can_frame cmd = bcast(CDH_ID, CLS_COMMAND, OP_PING, 0);
	struct can_record out[4];

	zassert_equal(tlm_cache_on_frame(&cache, &to_comms, 0), 0);
	zassert_equal(tlm_cache_on_frame(&cache, &to_ground, 0), 0);
	zassert_equal(tlm_cache_on_frame(&cache, &cmd, 0), 0);
	zassert_equal(tlm_cache_collect(&cache, 0, 1000, out, 4), 0);
	zassert_equal(cache.stats.ignored, 3);
}

ZTEST(telemetry, test_comms_own_and_ground_frames_are_not_cached)
{
	struct can_frame own = bcast(COMMS_ID, CLS_HEARTBEAT, OP_HEARTBEAT, 0);
	struct can_frame gnd = bcast(GROUND_ID, CLS_HEARTBEAT, OP_HEARTBEAT, 0);

	zassert_equal(tlm_cache_on_frame(&cache, &own, 0), 0);
	zassert_equal(tlm_cache_on_frame(&cache, &gnd, 0), 0);
	zassert_equal(cache.stats.stored, 0);
}

ZTEST(telemetry, test_frames_that_are_not_8_byte_extended_data_frames_are_ignored)
{
	struct can_frame std = bcast(CDH_ID, CLS_HEARTBEAT, OP_HEARTBEAT, 0);
	struct can_frame shortf = bcast(CDH_ID, CLS_HEARTBEAT, OP_HEARTBEAT, 0);
	struct can_frame rtr = bcast(CDH_ID, CLS_HEARTBEAT, OP_HEARTBEAT, 0);

	std.flags = 0;
	shortf.dlc = 4;
	rtr.flags |= CAN_FRAME_RTR;
	zassert_equal(tlm_cache_on_frame(&cache, &std, 0), 0);
	zassert_equal(tlm_cache_on_frame(&cache, &shortf, 0), 0);
	zassert_equal(tlm_cache_on_frame(&cache, &rtr, 0), 0);
	zassert_equal(cache.stats.stored, 0);
}

ZTEST(telemetry, test_old_records_are_left_out_and_the_age_limit_is_inclusive)
{
	struct can_frame f = bcast(CDH_ID, CLS_HEARTBEAT, OP_HEARTBEAT, 0);
	struct can_record out[2];

	zassert_equal(tlm_cache_on_frame(&cache, &f, 1000), 1);
	zassert_equal(tlm_cache_collect(&cache, 1100, 100, out, 2), 1, "exactly at the limit");
	zassert_equal(tlm_cache_collect(&cache, 1101, 100, out, 2), 0, "one millisecond past it");
	zassert_false(tlm_cache_get(&cache, CDH_ID, CLS_HEARTBEAT, OP_HEARTBEAT, 1101, 100, out));
}

ZTEST(telemetry, test_a_refreshed_record_is_fresh_again)
{
	struct can_frame f = bcast(CDH_ID, CLS_HEARTBEAT, OP_HEARTBEAT, 0);
	struct can_record out[2];

	zassert_equal(tlm_cache_on_frame(&cache, &f, 0), 1);
	zassert_equal(tlm_cache_collect(&cache, 500, 100, out, 2), 0);
	zassert_equal(tlm_cache_on_frame(&cache, &f, 500), 1);
	zassert_equal(tlm_cache_collect(&cache, 550, 100, out, 2), 1);
}

ZTEST(telemetry, test_a_full_cache_drops_new_keys_counts_them_and_still_updates_old_ones)
{
	struct can_record out[TLM_CACHE_SLOTS + 4];

	for (int i = 0; i < TLM_CACHE_SLOTS; i++) {
		struct can_frame f = bcast(CDH_ID, CLS_TELEMETRY, (uint8_t)(0x80 + i), 0);

		zassert_equal(tlm_cache_on_frame(&cache, &f, 0), 1, "slot %d", i);
	}
	struct can_frame extra = bcast(CDH_ID, CLS_TELEMETRY, 0xF0, 0);
	struct can_frame again = bcast(CDH_ID, CLS_TELEMETRY, 0x80, 9);

	zassert_equal(tlm_cache_on_frame(&cache, &extra, 0), 0);
	zassert_equal(cache.stats.overflow, 1);
	zassert_equal(tlm_cache_on_frame(&cache, &again, 0), 1);
	zassert_equal(cache.stats.overflow, 1);
	zassert_equal(tlm_cache_collect(&cache, 0, 1000, out, ARRAY_SIZE(out)), TLM_CACHE_SLOTS);
	zassert_equal(out[0].data[2], 9);
}

ZTEST(telemetry, test_collect_stops_at_the_caller_limit_and_keeps_arrival_order)
{
	struct can_record out[4];

	for (int i = 0; i < 5; i++) {
		struct can_frame f = bcast(CDH_ID, CLS_TELEMETRY, (uint8_t)(0x80 + i), (uint8_t)i);

		zassert_equal(tlm_cache_on_frame(&cache, &f, 0), 1);
	}
	zassert_equal(tlm_cache_collect(&cache, 0, 1000, out, 3), 3);
	for (int i = 0; i < 3; i++) {
		zassert_equal(out[i].data[2], i);
	}
}

ZTEST(telemetry, test_the_cache_fits_one_telemetry_packet_with_the_health_record)
{
	zassert_equal(TLM_CACHE_SLOTS + 1, COMMS_TLM_MAX_RECORDS);
	zassert_true(4 + COMMS_TLM_MAX_RECORDS * CAN_RECORD_LEN <= DL_ITEM_MAX_DATA);
}
