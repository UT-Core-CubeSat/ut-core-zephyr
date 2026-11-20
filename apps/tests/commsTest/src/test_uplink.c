/** @file test_uplink.c @brief Tests for app/uplink: block to frame to packet to router, with drop counters */
#include <zephyr/ztest.h>

#include "app/uplink.h"
#include "test_util.h"

#include "test_fake_can.h"

static struct uplink_stats st;
static struct router rt;
static struct can_bridge br;
static struct dl_item reply;

static void before(void *f)
{
	ARG_UNUSED(f);
	memset(&st, 0, sizeof(st));
	memset(&reply, 0, sizeof(reply));
	fake_can_attach(&br);
	router_init(&rt, &br, (struct mvp_switches){.ping = true, .can_cmd = true});
}

ZTEST_SUITE(uplink, NULL, NULL, before, NULL, NULL);

static uint32_t total_drops(void)
{
	uint32_t n = st.not_tc;

	for (int i = 0; i < TC_STATUS_COUNT; i++) {
		n += st.tc_drop[i];
	}
	for (int i = 0; i < SP_STATUS_COUNT; i++) {
		n += st.sp_drop[i];
	}
	return n;
}

ZTEST(uplink, test_valid_ping_block_is_answered)
{
	uint8_t block[COMMS_BLOCK_LEN];
	const uint8_t payload[] = "bench";

	test_make_ping_block(block, 0xCAFEF00D, payload, sizeof(payload));
	zassert_equal(uplink_handle_block(&st, &rt, block, sizeof(block), &reply), ROUTE_REPLY);
	zassert_equal(reply.apid, APID_DL_PING);
	zassert_equal(reply.len, 4 + sizeof(payload));
	zassert_equal(reply.data[0], 0xCA);
	zassert_equal(reply.data[3], 0x0D);
	zassert_mem_equal(&reply.data[4], payload, sizeof(payload));
	zassert_equal(st.blocks, 1);
	zassert_equal(st.accepted, 1);
	zassert_equal(total_drops(), 0);
}

ZTEST(uplink, test_corrupt_block_is_counted_as_fecf)
{
	uint8_t block[COMMS_BLOCK_LEN];

	test_make_ping_block(block, 1, NULL, 0);
	block[10] ^= 0x01;
	zassert_true(uplink_handle_block(&st, &rt, block, sizeof(block), &reply) < 0);
	zassert_equal(st.tc_drop[TC_ERR_FECF], 1);
	zassert_equal(st.accepted, 0);
	zassert_equal(total_drops(), 1);
}

ZTEST(uplink, test_wrong_spacecraft_id_is_counted_as_scid)
{
	uint8_t block[COMMS_BLOCK_LEN];
	uint8_t pkt[16];
	const uint8_t data[4] = {0};
	size_t n = test_make_ul_packet(pkt, sizeof(pkt), APID_UL_PING, data, 4);

	zassert_ok(tc_frame_build(block, COMMS_TC_SCID ^ 0x001, COMMS_TC_VCID, pkt, n));
	zassert_true(uplink_handle_block(&st, &rt, block, sizeof(block), &reply) < 0);
	zassert_equal(st.tc_drop[TC_ERR_SCID], 1);
	zassert_equal(total_drops(), 1);
}

ZTEST(uplink, test_wrong_virtual_channel_is_counted)
{
	uint8_t block[COMMS_BLOCK_LEN];
	uint8_t pkt[16];
	const uint8_t data[4] = {0};
	size_t n = test_make_ul_packet(pkt, sizeof(pkt), APID_UL_PING, data, 4);

	zassert_ok(tc_frame_build(block, COMMS_TC_SCID, COMMS_TC_VCID + 1, pkt, n));
	zassert_true(uplink_handle_block(&st, &rt, block, sizeof(block), &reply) < 0);
	zassert_equal(st.tc_drop[TC_ERR_VCID], 1);
}

ZTEST(uplink, test_frame_with_garbage_packet_is_counted_at_packet_layer)
{
	uint8_t block[COMMS_BLOCK_LEN];
	const uint8_t junk[3] = {0x18, 0xF0, 0xC0}; /* shorter than a Space Packet header */

	zassert_ok(tc_frame_build(block, COMMS_TC_SCID, COMMS_TC_VCID, junk, sizeof(junk)));
	zassert_true(uplink_handle_block(&st, &rt, block, sizeof(block), &reply) < 0);
	zassert_equal(st.sp_drop[SP_ERR_SHORT], 1);
	zassert_equal(total_drops(), 1);
}

ZTEST(uplink, test_telemetry_type_packet_on_the_uplink_is_dropped)
{
	uint8_t block[COMMS_BLOCK_LEN];
	uint8_t pkt[16];
	const uint8_t data[4] = {0};
	int n = sp_build(pkt, sizeof(pkt), false, false, APID_UL_PING, 0, data, 4);

	zassert_true(n > 0);
	zassert_ok(tc_frame_build(block, COMMS_TC_SCID, COMMS_TC_VCID, pkt, n));
	zassert_true(uplink_handle_block(&st, &rt, block, sizeof(block), &reply) < 0);
	zassert_equal(st.not_tc, 1);
	zassert_equal(st.accepted, 0);
}

ZTEST(uplink, test_unknown_apid_reaches_the_router_and_is_counted_there)
{
	uint8_t block[COMMS_BLOCK_LEN];
	uint8_t pkt[16];
	const uint8_t data[4] = {0};
	size_t n = test_make_ul_packet(pkt, sizeof(pkt), 0x123, data, 4);

	zassert_ok(tc_frame_build(block, COMMS_TC_SCID, COMMS_TC_VCID, pkt, n));
	zassert_equal(uplink_handle_block(&st, &rt, block, sizeof(block), &reply), ROUTE_UNKNOWN);
	zassert_equal(st.accepted, 1);
	zassert_equal(rt.stats.unknown, 1);
	zassert_equal(total_drops(), 0);
}

ZTEST(uplink, test_random_blocks_never_crash_and_are_not_answered)
{
	uint8_t block[COMMS_BLOCK_LEN];
	struct test_rng r = {.s = 12345};

	for (int i = 0; i < 20000; i++) {
		test_rng_fill(&r, block, sizeof(block));
		zassert_not_equal(uplink_handle_block(&st, &rt, block, sizeof(block), &reply), ROUTE_REPLY);
	}
	zassert_equal(st.blocks, 20000);
}

ZTEST(uplink, test_short_and_zero_length_blocks_are_dropped)
{
	uint8_t block[COMMS_BLOCK_LEN] = {0};

	zassert_true(uplink_handle_block(&st, &rt, block, 0, &reply) < 0);
	zassert_true(uplink_handle_block(&st, &rt, block, 3, &reply) < 0);
	zassert_equal(total_drops(), 2);
}
