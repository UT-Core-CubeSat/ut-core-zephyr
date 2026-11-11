/** @file test_downlink.c @brief Tests for app/downlink with the radio replaced by a recorder */
#include <errno.h>
#include <zephyr/ztest.h>

#include "app/downlink.h"
#include "link/dl_block.h"
#include "test_util.h"

K_MSGQ_DEFINE(dl_q, sizeof(struct dl_item), 3, 4);
K_MSGQ_DEFINE(unused_rx_q, COMMS_BLOCK_LEN, 1, 1);

static struct radio_port port;
static struct downlink dl;
static uint8_t sent[8][COMMS_BLOCK_LEN];
static int send_calls;
static int send_result;

static int fake_send(const struct device *dev, const uint8_t *block, size_t len, k_timeout_t timeout)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(timeout);
	zassert_equal(len, COMMS_BLOCK_LEN);
	if (send_calls < 8) {
		memcpy(sent[send_calls], block, len);
	}
	send_calls++;
	return send_result;
}

static int fake_recv_async(const struct device *dev, rfm26w_rx_cb cb, void *arg)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(arg);
	return 0;
}

static const struct radio_port_ops fake_ops = {.send = fake_send, .recv_async = fake_recv_async};

static void before(void *f)
{
	ARG_UNUSED(f);
	k_msgq_purge(&dl_q);
	radio_port_init(&port, NULL, &unused_rx_q);
	port.ops = &fake_ops;
	downlink_init(&dl, &dl_q, &port);
	send_calls = 0;
	send_result = 0;
}

ZTEST_SUITE(downlink, NULL, NULL, before, NULL, NULL);

static struct dl_item item(uint16_t apid, uint8_t fill, uint16_t len)
{
	struct dl_item it = {.apid = apid, .len = len};

	memset(it.data, fill, sizeof(it.data));
	return it;
}

/* Decode sent block `i`: returns the packet, checks block framing on the way. */
static void decode(int i, struct sp_packet *pkt)
{
	const uint8_t *p;
	size_t plen;

	zassert_equal(dl_block_unpack(sent[i], COMMS_BLOCK_LEN, &p, &plen), DL_OK);
	zassert_equal(sp_parse(p, plen, pkt), SP_OK);
	zassert_false(pkt->is_tc, "downlink packets are telemetry type");
}

ZTEST(downlink, test_item_goes_out_as_a_valid_block_and_packet)
{
	struct dl_item it = item(APID_DL_PING, 0x77, 10);
	struct sp_packet pkt;

	zassert_ok(downlink_submit(&dl, &it));
	zassert_ok(downlink_service_one(&dl, K_NO_WAIT));
	zassert_equal(send_calls, 1);
	decode(0, &pkt);
	zassert_equal(pkt.apid, APID_DL_PING);
	zassert_equal(pkt.data_len, 10);
	zassert_equal(pkt.data[0], 0x77);
	zassert_equal(dl.stats.sent, 1);
}

ZTEST(downlink, test_sequence_counts_are_per_apid_and_increment)
{
	struct dl_item a = item(APID_DL_PING, 1, 4);
	struct dl_item b = item(APID_DL_TLM, 2, 4);
	struct sp_packet pkt;
	const uint16_t order[] = {APID_DL_PING, APID_DL_PING, APID_DL_TLM, APID_DL_PING};
	const uint16_t want[] = {0, 1, 0, 2};

	for (size_t i = 0; i < 3; i++) {
		zassert_ok(downlink_submit(&dl, order[i] == APID_DL_PING ? &a : &b));
		zassert_ok(downlink_service_one(&dl, K_NO_WAIT));
	}
	zassert_ok(downlink_submit(&dl, &a));
	zassert_ok(downlink_service_one(&dl, K_NO_WAIT));
	for (int i = 0; i < 4; i++) {
		decode(i, &pkt);
		zassert_equal(pkt.apid, order[i]);
		zassert_equal(pkt.seq_count, want[i], "block %d", i);
	}
}

ZTEST(downlink, test_sequence_count_wraps_at_14_bits)
{
	struct dl_item it = item(APID_DL_PING, 1, 4);
	struct sp_packet pkt;

	dl.seq.next[APID_DL_PING] = SP_SEQ_COUNT_MASK;
	for (int i = 0; i < 2; i++) {
		zassert_ok(downlink_submit(&dl, &it));
		zassert_ok(downlink_service_one(&dl, K_NO_WAIT));
	}
	decode(0, &pkt);
	zassert_equal(pkt.seq_count, SP_SEQ_COUNT_MASK);
	decode(1, &pkt);
	zassert_equal(pkt.seq_count, 0);
}

ZTEST(downlink, test_largest_item_fits)
{
	struct dl_item it = item(APID_DL_TLM, 0xEE, DL_ITEM_MAX_DATA);
	struct sp_packet pkt;

	zassert_ok(downlink_submit(&dl, &it));
	zassert_ok(downlink_service_one(&dl, K_NO_WAIT));
	decode(0, &pkt);
	zassert_equal(pkt.data_len, DL_ITEM_MAX_DATA);
}

ZTEST(downlink, test_full_queue_rejects_without_blocking_and_counts)
{
	struct dl_item it = item(APID_DL_PING, 1, 4);

	for (int i = 0; i < 3; i++) {
		zassert_ok(downlink_submit(&dl, &it));
	}
	zassert_equal(downlink_submit(&dl, &it), -ENOMSG);
	zassert_equal(dl.stats.queued, 3);
	zassert_equal(dl.stats.queue_full, 1);
}

ZTEST(downlink, test_empty_queue_times_out)
{
	zassert_equal(downlink_service_one(&dl, K_NO_WAIT), -EAGAIN);
	zassert_equal(send_calls, 0);
}

ZTEST(downlink, test_invalid_items_are_counted_and_do_not_burn_a_sequence_count)
{
	struct dl_item empty = item(APID_DL_PING, 1, 0);
	struct dl_item big = item(APID_DL_PING, 1, DL_ITEM_MAX_DATA);
	struct dl_item badapid = item(0x800, 1, 4);
	struct dl_item good = item(APID_DL_PING, 1, 4);
	struct sp_packet pkt;

	big.len = DL_ITEM_MAX_DATA + 1;
	zassert_ok(downlink_submit(&dl, &empty));
	zassert_ok(downlink_submit(&dl, &big));
	zassert_ok(downlink_submit(&dl, &badapid));
	for (int i = 0; i < 3; i++) {
		zassert_ok(downlink_service_one(&dl, K_NO_WAIT));
	}
	zassert_equal(dl.stats.build_errors, 3);
	zassert_equal(send_calls, 0);

	zassert_ok(downlink_submit(&dl, &good));
	zassert_ok(downlink_service_one(&dl, K_NO_WAIT));
	decode(0, &pkt);
	zassert_equal(pkt.seq_count, 0);
}

ZTEST(downlink, test_send_failure_is_counted_and_later_items_still_go)
{
	struct dl_item it = item(APID_DL_PING, 1, 4);
	struct sp_packet pkt;

	send_result = -EBUSY;
	zassert_ok(downlink_submit(&dl, &it));
	zassert_ok(downlink_service_one(&dl, K_NO_WAIT));
	zassert_equal(dl.stats.send_errors, 1);
	zassert_equal(dl.stats.sent, 0);

	send_result = 0;
	zassert_ok(downlink_submit(&dl, &it));
	zassert_ok(downlink_service_one(&dl, K_NO_WAIT));
	zassert_equal(dl.stats.sent, 1);
	/* The lost packet used count 0, so ground sees the gap. */
	decode(1, &pkt);
	zassert_equal(pkt.seq_count, 1);
}
