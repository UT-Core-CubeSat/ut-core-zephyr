/** @file test_scheduler.c @brief Tests for app/scheduler with a fake CAN bus and a plain queue */
#include <zephyr/ztest.h>

#include "app/scheduler.h"
#include "common/can_proto.h"
#include "test_fake_can.h"
#include "test_util.h"

K_MSGQ_DEFINE(s_tx_q, sizeof(struct dl_item), 4, 4);
K_MSGQ_DEFINE(s_unused_rx_q, COMMS_BLOCK_LEN, 1, 1);

static struct radio_port port;
static struct downlink dl;
static struct can_bridge br;
static struct tlm_cache cache;
static struct tlm tl;
static struct scheduler sc;
static struct mvp_switches sw;

#define STATS_MS 10000U

static void before(void *f)
{
	ARG_UNUSED(f);
	k_msgq_purge(&s_tx_q);
	radio_port_init(&port, NULL, &s_unused_rx_q);
	downlink_init(&dl, &s_tx_q, &port);
	fake_can_attach(&br);
	tlm_cache_init(&cache);
	tlm_init(&tl, &cache, &fcan_counter);
	sw = (struct mvp_switches){.ping = true, .can_cmd = true, .tlm = true};
	scheduler_init(&sc, 1000, STATS_MS, &tl, &dl, &br, &sw);
}

ZTEST_SUITE(scheduler, NULL, NULL, before, NULL, NULL);

static int queued(void)
{
	return (int)k_msgq_num_used_get(&s_tx_q);
}

ZTEST(scheduler, test_only_the_heartbeat_is_due_at_start)
{
	zassert_equal(scheduler_step(&sc, 1000), JOB_BIT(JOB_HEARTBEAT));
	zassert_equal(fcan.count, 1);
	zassert_equal(queued(), 0);
}

ZTEST(scheduler, test_the_heartbeat_frame_is_a_broadcast_from_comms_with_the_heartbeat_opcode)
{
	zassert_equal(scheduler_step(&sc, 1000), JOB_BIT(JOB_HEARTBEAT));
	zassert_true(fcan.frames[0].flags & CAN_FRAME_IDE);
	zassert_equal(fcan.frames[0].dlc, 8);
	zassert_equal((fcan.frames[0].id >> 14) & 0xFF, COMMS_ID);
	zassert_equal((fcan.frames[0].id >> 6) & 0xFF, CAN_BROADCAST);
	zassert_equal(fcan.frames[0].id & 0x3F, CLS_HEARTBEAT);
	zassert_equal(fcan.frames[0].data[0], COMMS_ID);
	zassert_equal(fcan.frames[0].data[1], OP_HEARTBEAT);
	zassert_equal(br.stats.heartbeats, 1);
}

ZTEST(scheduler, test_heartbeat_runs_every_period_with_no_drift)
{
	/* Called a little late each time: the due time still moves by exactly one period. */
	for (int i = 0; i < 5; i++) {
		zassert_equal(scheduler_step(&sc, 1000 + (int64_t)i * COMMS_HEARTBEAT_PERIOD_MS + 30),
			      JOB_BIT(JOB_HEARTBEAT));
	}
	zassert_equal(fcan.count, 5);
	zassert_equal(sc.next_ms[JOB_HEARTBEAT], 1000 + 5 * (int64_t)COMMS_HEARTBEAT_PERIOD_MS,
		      "the due time stays on the original grid");
	zassert_equal(scheduler_step(&sc, 1000 + 5 * (int64_t)COMMS_HEARTBEAT_PERIOD_MS - 1), 0,
		      "not due until the 6th period starts");
	zassert_equal(sc.stats.skipped[JOB_HEARTBEAT], 0);
}

ZTEST(scheduler, test_telemetry_and_beacon_come_one_period_after_init_not_at_start)
{
	zassert_equal(scheduler_step(&sc, 1000 + COMMS_TLM_PERIOD_MS - 1) & JOB_BIT(JOB_TLM), 0);
	zassert_equal(queued(), 0);
	zassert_not_equal(scheduler_step(&sc, 1000 + COMMS_TLM_PERIOD_MS) & JOB_BIT(JOB_TLM), 0);
	zassert_equal(queued(), 1);
}

ZTEST(scheduler, test_telemetry_runs_every_period_and_the_beacon_once_a_minute)
{
	struct dl_item it;

	/* Step often enough that nothing is skipped, for one beacon period. */
	for (int64_t t = 1000; t <= 1000 + COMMS_BEACON_PERIOD_MS; t += 500) {
		(void)scheduler_step(&sc, t);
		while (k_msgq_get(&s_tx_q, &it, K_NO_WAIT) == 0) {
		}
	}
	zassert_equal(sc.stats.run[JOB_TLM], COMMS_BEACON_PERIOD_MS / COMMS_TLM_PERIOD_MS);
	zassert_equal(sc.stats.run[JOB_BEACON], 1);
	zassert_equal(tl.stats.beacons, 1);
}

ZTEST(scheduler, test_queued_packet_apids)
{
	struct dl_item it;
	int tlm_seen = 0, beacon_seen = 0;

	/* Beacon and telemetry both due at the same moment: one step queues both. */
	sc.next_ms[JOB_TLM] = 5000;
	sc.next_ms[JOB_BEACON] = 5000;
	zassert_not_equal(scheduler_step(&sc, 5000) & JOB_BIT(JOB_TLM), 0);
	zassert_equal(queued(), 2);
	while (k_msgq_get(&s_tx_q, &it, K_NO_WAIT) == 0) {
		tlm_seen += it.apid == APID_DL_TLM;
		beacon_seen += it.apid == APID_DL_BEACON;
	}
	zassert_equal(tlm_seen, 1);
	zassert_equal(beacon_seen, 1);
}

ZTEST(scheduler, test_with_the_tlm_switch_off_nothing_is_queued_but_the_heartbeat_still_goes)
{
	sw.tlm = false;
	sc.next_ms[JOB_TLM] = 5000;
	sc.next_ms[JOB_BEACON] = 5000;
	(void)scheduler_step(&sc, 5000);
	zassert_equal(queued(), 0);
	zassert_equal(sc.stats.tlm_off, 2);
	zassert_equal(sc.stats.run[JOB_TLM], 0);
	zassert_equal(tl.stats.telemetry, 0);
	zassert_equal(atomic_get(&fcan_counter), 0, "no counter value is used while off");
	zassert_equal(sc.stats.run[JOB_HEARTBEAT], 1);
}

ZTEST(scheduler, test_the_switch_is_read_each_time_so_a_runtime_change_takes_effect)
{
	sw.tlm = false;
	sc.next_ms[JOB_TLM] = 5000;
	(void)scheduler_step(&sc, 5000);
	zassert_equal(queued(), 0);
	sw.tlm = true;
	(void)scheduler_step(&sc, 5000 + COMMS_TLM_PERIOD_MS);
	zassert_equal(queued(), 1);
}

ZTEST(scheduler, test_a_stall_of_more_than_a_period_does_not_cause_a_burst)
{
	sc.next_ms[JOB_TLM] = 5000;
	(void)scheduler_step(&sc, 5000 + 3 * COMMS_TLM_PERIOD_MS + 100);
	zassert_equal(queued(), 1, "one packet, not four");
	zassert_equal(sc.stats.skipped[JOB_TLM], 1);
	zassert_equal(sc.next_ms[JOB_TLM], 5000 + 4 * (int64_t)COMMS_TLM_PERIOD_MS + 100);
	zassert_equal(scheduler_step(&sc, 5000 + 3 * COMMS_TLM_PERIOD_MS + 200) & JOB_BIT(JOB_TLM), 0);
}

ZTEST(scheduler, test_a_full_queue_does_not_stop_the_scheduler_and_is_counted_by_the_downlink)
{
	struct dl_item filler = {.apid = APID_DL_PING, .len = 4};

	for (int i = 0; i < 4; i++) {
		zassert_ok(downlink_submit(&dl, &filler));
	}
	sc.next_ms[JOB_TLM] = 5000;
	(void)scheduler_step(&sc, 5000);
	zassert_equal(dl.stats.queue_full, 1);
	zassert_equal(sc.stats.run[JOB_TLM], 1);
}

ZTEST(scheduler, test_a_failing_can_send_is_counted_and_telemetry_continues)
{
	fcan.result = -EIO;
	sc.next_ms[JOB_TLM] = 1000;
	(void)scheduler_step(&sc, 1000);
	zassert_equal(br.stats.send_errors, 1);
	zassert_equal(br.stats.heartbeats, 0);
	zassert_equal(queued(), 1);
}

ZTEST(scheduler, test_the_stats_job_is_reported_to_the_caller_and_not_acted_on_here)
{
	zassert_equal(scheduler_step(&sc, 1000 + STATS_MS) & JOB_BIT(JOB_STATS), JOB_BIT(JOB_STATS));
	zassert_equal(sc.stats.run[JOB_STATS], 1);
}

ZTEST(scheduler, test_wait_is_the_time_to_the_next_job_and_zero_when_one_is_due)
{
	zassert_equal(scheduler_wait_ms(&sc, 1000), 0, "heartbeat due at once");
	(void)scheduler_step(&sc, 1000);
	zassert_equal(scheduler_wait_ms(&sc, 1000), COMMS_HEARTBEAT_PERIOD_MS);
	zassert_equal(scheduler_wait_ms(&sc, 1400), COMMS_HEARTBEAT_PERIOD_MS - 400);
	zassert_equal(scheduler_wait_ms(&sc, 99999), 0, "overdue");
}
