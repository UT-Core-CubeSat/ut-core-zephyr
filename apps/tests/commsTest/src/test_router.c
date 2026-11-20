/** @file test_router.c @brief Tests for app/router (dispatch by APID, test switches, command path) */
#include <zephyr/ztest.h>

#include "app/router.h"
#include "can/can_ping.h"
#include "common/can_proto.h"
#include "test_fake_can.h"
#include "test_util.h"

static struct router rt;
static struct can_bridge br;

static const struct mvp_switches all_on = {.ping = true, .can_cmd = true};

static void before(void *f)
{
	ARG_UNUSED(f);
	fake_can_attach(&br);
	router_init(&rt, &br, all_on);
}

ZTEST_SUITE(router, NULL, NULL, before, NULL, NULL);

static int dispatch(uint16_t apid, const uint8_t *data, size_t len, struct dl_item *reply)
{
	uint8_t buf[COMMS_UL_MAX_PACKET_CLEAR];
	struct sp_packet pkt;
	size_t n = test_make_ul_packet(buf, sizeof(buf), apid, data, len);

	zassert_equal(sp_parse(buf, n, &pkt), SP_OK);
	return router_dispatch(&rt, &pkt, reply);
}

static int dispatch_record(const struct can_record *rec)
{
	uint8_t wire[CAN_RECORD_LEN];
	struct dl_item reply;

	can_record_pack(wire, rec);
	return dispatch(APID_UL_CMD, wire, sizeof(wire), &reply);
}

static const uint8_t token[COMMS_PING_TOKEN_LEN] = {1, 2, 3, 4, 5, 6};

/* ---- ping ---- */

ZTEST(router, test_ping_request_gets_a_reply_on_the_ping_reply_apid)
{
	const uint8_t data[] = {0, 0, 0x12, 0x34, 'h', 'i'};
	struct dl_item reply = {0};

	zassert_equal(dispatch(APID_UL_PING, data, sizeof(data), &reply), ROUTE_REPLY);
	zassert_equal(reply.apid, APID_DL_PING);
	zassert_equal(reply.len, sizeof(data));
	zassert_mem_equal(reply.data, data, sizeof(data));
	zassert_equal(rt.stats.ping, 1);
}

ZTEST(router, test_ping_with_short_data_is_dropped_and_counted)
{
	const uint8_t data[] = {1, 2, 3};
	struct dl_item reply = {.apid = 0xABC};

	zassert_equal(dispatch(APID_UL_PING, data, sizeof(data), &reply), ROUTE_BAD);
	zassert_equal(rt.stats.bad_ping, 1);
	zassert_equal(rt.stats.ping, 0);
	zassert_equal(reply.apid, 0xABC, "no reply may be written for a bad request");
}

ZTEST(router, test_other_apids_are_unknown)
{
	const uint8_t data[] = {0, 0, 0, 0};
	const uint16_t apids[] = {APID_UL_CFDP, APID_DL_PING, APID_DL_TLM, APID_IDLE, 0x000, 0x123};
	struct dl_item reply;

	for (size_t i = 0; i < ARRAY_SIZE(apids); i++) {
		zassert_equal(dispatch(apids[i], data, sizeof(data), &reply), ROUTE_UNKNOWN, "apid 0x%x",
			      apids[i]);
	}
	zassert_equal(rt.stats.unknown, ARRAY_SIZE(apids));
	zassert_equal(rt.stats.ping, 0);
	zassert_equal(fcan.attempts, 0);
}

/* ---- command to the CAN bus ---- */

ZTEST(router, test_allowed_command_goes_on_the_bus_and_no_reply_is_sent_now)
{
	struct can_record rec;

	comms_can_ping_record(&rec, CDH_ID, token);
	zassert_equal(dispatch_record(&rec), ROUTE_DONE);
	zassert_equal(fcan.count, 1);
	zassert_equal(rt.stats.cmd_forwarded, 1);
	zassert_equal(fcan.frames[0].data[1], OP_PING);
	zassert_mem_equal(&fcan.frames[0].data[2], token, sizeof(token));
}

ZTEST(router, test_denied_command_is_counted_by_reason_and_not_sent)
{
	struct can_record rec;

	comms_can_ping_record(&rec, CDH_ID, token);
	rec.id = CAN_ID_FULL(0, EPS_ID, CDH_ID, CLS_COMMAND); /* not from ground */
	zassert_equal(dispatch_record(&rec), ROUTE_BAD);
	zassert_equal(rt.stats.cmd_denied[POLICY_ERR_SRC], 1);

	comms_can_ping_record(&rec, CDH_ID, token);
	rec.data[1] = OP_REBOOT;
	zassert_equal(dispatch_record(&rec), ROUTE_BAD);
	zassert_equal(rt.stats.cmd_denied[POLICY_ERR_NOT_ALLOWED], 1);

	zassert_equal(fcan.attempts, 0, "a denied record must never reach the bus");
	zassert_equal(rt.stats.cmd_forwarded, 0);
}

ZTEST(router, test_command_packet_that_is_not_exactly_one_record_is_dropped)
{
	uint8_t data[2 * CAN_RECORD_LEN] = {0};
	struct dl_item reply;

	zassert_equal(dispatch(APID_UL_CMD, data, CAN_RECORD_LEN - 1, &reply), ROUTE_BAD);
	zassert_equal(dispatch(APID_UL_CMD, data, CAN_RECORD_LEN + 1, &reply), ROUTE_BAD);
	zassert_equal(dispatch(APID_UL_CMD, data, 2 * CAN_RECORD_LEN, &reply), ROUTE_BAD);
	zassert_equal(rt.stats.cmd_bad_record, 3);
	zassert_equal(fcan.attempts, 0);
}

ZTEST(router, test_record_with_reserved_id_bits_is_dropped)
{
	uint8_t wire[CAN_RECORD_LEN];
	struct can_record rec;
	struct dl_item reply;

	comms_can_ping_record(&rec, CDH_ID, token);
	can_record_pack(wire, &rec);
	wire[1] |= 0x40; /* ID bit 22, reserved */
	zassert_equal(dispatch(APID_UL_CMD, wire, sizeof(wire), &reply), ROUTE_BAD);
	zassert_equal(rt.stats.cmd_bad_record, 1);
	zassert_equal(fcan.attempts, 0);
}

ZTEST(router, test_a_can_send_failure_is_counted_and_dropped)
{
	struct can_record rec;

	comms_can_ping_record(&rec, CDH_ID, token);
	fcan.result = -ENOMEM;
	zassert_equal(dispatch_record(&rec), ROUTE_BAD);
	zassert_equal(rt.stats.cmd_can_error, 1);
	zassert_equal(rt.stats.cmd_forwarded, 0);
}

/* ---- test switches ---- */

ZTEST(router, test_a_switched_off_test_is_dropped_and_counted)
{
	struct can_record rec;
	const uint8_t data[] = {0, 0, 0, 1};
	struct dl_item reply;

	comms_can_ping_record(&rec, CDH_ID, token);

	rt.sw.can_cmd = false;
	zassert_equal(dispatch_record(&rec), ROUTE_UNKNOWN);
	zassert_equal(fcan.attempts, 0);
	zassert_equal(dispatch(APID_UL_PING, data, sizeof(data), &reply), ROUTE_REPLY, "ping stays on");

	rt.sw.can_cmd = true;
	rt.sw.ping = false;
	zassert_equal(dispatch(APID_UL_PING, data, sizeof(data), &reply), ROUTE_UNKNOWN);
	zassert_equal(dispatch_record(&rec), ROUTE_DONE, "can_cmd stays on");
	zassert_equal(rt.stats.disabled, 2);
}
