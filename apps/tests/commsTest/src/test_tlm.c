/** @file test_tlm.c @brief Tests for app/tlm: telemetry and beacon packets */
#include <zephyr/ztest.h>

#include "app/tlm.h"
#include "common/can_proto.h"
#include "test_util.h"

static struct tlm_cache cache;
static struct tlm t;
static atomic_t counter;

static void before(void *f)
{
	ARG_UNUSED(f);
	tlm_cache_init(&cache);
	atomic_set(&counter, 0);
	tlm_init(&t, &cache, &counter);
}

ZTEST_SUITE(tlm, NULL, NULL, before, NULL, NULL);

static void feed(uint8_t src, uint8_t cls, uint8_t op, uint8_t value, int64_t now)
{
	struct can_frame f = {.id = CAN_ID_FULL(3, src, CAN_BROADCAST, cls)};

	can_fill_payload(&f, src, op, value, 0, 0, 0, 0, 0);
	zassert_equal(tlm_cache_on_frame(&cache, &f, now), 1);
}

static uint32_t item_counter(const struct dl_item *it)
{
	return ((uint32_t)it->data[0] << 24) | ((uint32_t)it->data[1] << 16) | ((uint32_t)it->data[2] << 8) |
	       it->data[3];
}

static struct can_record item_record(const struct dl_item *it, int i)
{
	struct can_record r;

	zassert_equal(can_record_unpack(&it->data[4 + i * CAN_RECORD_LEN], CAN_RECORD_LEN, &r), REC_OK);
	return r;
}

/* ---- health record ---- */

ZTEST(tlm, test_health_record_layout_is_written_out_byte_by_byte)
{
	struct can_record r;
	uint8_t raw[CAN_RECORD_LEN];
	/* id: priority 3 (bits 28-26), src 0x03 (bits 21-14), dst 0xFF (bits 13-6), class 10 (bits 5-0)
	 * = 0x0C000000 | 0x0000C000 | 0x00003FC0 | 0x0A = 0x0C00FFCA */
	const uint8_t want[CAN_RECORD_LEN] = {0x0C, 0x00, 0xFF, 0xCA, 0x03, 0x30, 0x01, 0x02, 0x03, 0x04, 0x05, 0};

	tlm_health_record(&r, 0x01020304, 5);
	can_record_pack(raw, &r);
	zassert_mem_equal(raw, want, sizeof(want));
}

/* ---- telemetry ---- */

ZTEST(tlm, test_with_an_empty_cache_telemetry_is_the_counter_and_the_health_record)
{
	struct dl_item it;
	struct can_record r;

	tlm_build_telemetry(&t, 12345, &it);
	zassert_equal(it.apid, APID_DL_TLM);
	zassert_equal(it.len, 4 + CAN_RECORD_LEN);
	zassert_equal(item_counter(&it), 0);
	r = item_record(&it, 0);
	zassert_equal(can_record_class(&r), CLS_HEALTH);
	zassert_equal(can_record_src(&r), COMMS_ID);
	zassert_equal(r.data[5], 12, "uptime in whole seconds");
	zassert_equal(r.data[6], 0, "no records follow");
}

ZTEST(tlm, test_telemetry_carries_every_fresh_cached_record_after_the_health_record)
{
	struct dl_item it;
	struct can_record r;

	feed(CDH_ID, CLS_HEARTBEAT, OP_HEARTBEAT, 0x11, 1000);
	feed(EPS_ID, CLS_TELEMETRY, 0x77, 0x22, 1000);
	tlm_build_telemetry(&t, 2000, &it);
	zassert_equal(it.len, 4 + 3 * CAN_RECORD_LEN);
	r = item_record(&it, 0);
	zassert_equal(r.data[6], 2, "health record says two records follow");
	r = item_record(&it, 1);
	zassert_equal(can_record_src(&r), CDH_ID);
	zassert_equal(r.data[2], 0x11);
	r = item_record(&it, 2);
	zassert_equal(can_record_src(&r), EPS_ID);
	zassert_equal(r.data[2], 0x22);
}

ZTEST(tlm, test_a_stale_record_is_not_sent)
{
	struct dl_item it;

	feed(CDH_ID, CLS_HEARTBEAT, OP_HEARTBEAT, 0, 0);
	tlm_build_telemetry(&t, COMMS_TLM_MAX_AGE_MS + 1, &it);
	zassert_equal(it.len, 4 + CAN_RECORD_LEN, "only the health record is left");
}

ZTEST(tlm, test_the_counter_advances_by_one_per_packet_across_telemetry_and_beacon)
{
	struct dl_item a, b, c;

	tlm_build_telemetry(&t, 0, &a);
	tlm_build_beacon(&t, 0, &b);
	tlm_build_telemetry(&t, 0, &c);
	zassert_equal(item_counter(&a), 0);
	zassert_equal(item_counter(&b), 1);
	zassert_equal(item_counter(&c), 2);
	zassert_equal(atomic_get(&counter), 3);
	zassert_equal(t.stats.telemetry, 2);
	zassert_equal(t.stats.beacons, 1);
}

ZTEST(tlm, test_the_counter_is_shared_with_other_users_of_it)
{
	struct dl_item it;

	atomic_set(&counter, 0x01020304);
	tlm_build_telemetry(&t, 0, &it);
	zassert_equal(it.data[0], 0x01);
	zassert_equal(it.data[3], 0x04, "big-endian");
	zassert_equal(atomic_get(&counter), 0x01020305);
}

ZTEST(tlm, test_a_full_cache_fills_the_largest_packet_and_it_fits_the_downlink_item)
{
	struct dl_item it;

	for (int i = 0; i < TLM_CACHE_SLOTS; i++) {
		feed(GNSS_ID, CLS_TELEMETRY, (uint8_t)(0x80 + i), (uint8_t)i, 0);
	}
	tlm_build_telemetry(&t, 0, &it);
	zassert_equal(it.len, 4 + COMMS_TLM_MAX_RECORDS * CAN_RECORD_LEN);
	zassert_true(it.len <= DL_ITEM_MAX_DATA);
	zassert_equal(item_record(&it, 0).data[6], TLM_CACHE_SLOTS);
}

/* ---- beacon ---- */

ZTEST(tlm, test_beacon_with_nothing_cached_is_just_the_health_record)
{
	struct dl_item it;

	tlm_build_beacon(&t, 5000, &it);
	zassert_equal(it.apid, APID_DL_BEACON);
	zassert_equal(it.len, 4 + CAN_RECORD_LEN);
}

ZTEST(tlm, test_beacon_adds_the_cdh_and_eps_heartbeats_in_that_order)
{
	struct dl_item it;
	struct can_record r;

	feed(EPS_ID, CLS_HEARTBEAT, OP_HEARTBEAT, 0x0E, 0);
	feed(CDH_ID, CLS_HEARTBEAT, OP_HEARTBEAT, 0x0C, 0);
	tlm_build_beacon(&t, 100, &it);
	zassert_equal(it.len, 4 + 3 * CAN_RECORD_LEN);
	zassert_equal(item_record(&it, 0).data[6], 2);
	r = item_record(&it, 1);
	zassert_equal(can_record_src(&r), CDH_ID);
	zassert_equal(r.data[2], 0x0C);
	r = item_record(&it, 2);
	zassert_equal(can_record_src(&r), EPS_ID);
}

ZTEST(tlm, test_beacon_skips_a_missing_or_stale_board_and_ignores_other_records)
{
	struct dl_item it;
	struct can_record r;

	feed(CDH_ID, CLS_HEARTBEAT, OP_HEARTBEAT, 1, 0);                 /* will be stale */
	feed(EPS_ID, CLS_HEARTBEAT, OP_HEARTBEAT, 2, 100000);
	feed(GNSS_ID, CLS_HEARTBEAT, OP_HEARTBEAT, 3, 100000);          /* not in the beacon */
	feed(EPS_ID, CLS_TELEMETRY, OP_HEARTBEAT, 4, 100000);           /* same op, not a heartbeat */
	tlm_build_beacon(&t, 100000, &it);
	zassert_equal(it.len, 4 + 2 * CAN_RECORD_LEN);
	r = item_record(&it, 1);
	zassert_equal(can_record_src(&r), EPS_ID);
	zassert_equal(can_record_class(&r), CLS_HEARTBEAT);
}

ZTEST(tlm, test_the_beacon_never_exceeds_three_records)
{
	struct dl_item it;

	feed(CDH_ID, CLS_HEARTBEAT, OP_HEARTBEAT, 0, 0);
	feed(EPS_ID, CLS_HEARTBEAT, OP_HEARTBEAT, 0, 0);
	tlm_build_beacon(&t, 0, &it);
	zassert_true((it.len - 4) / CAN_RECORD_LEN <= COMMS_BEACON_MAX_RECORDS);
}
