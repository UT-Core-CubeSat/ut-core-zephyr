/**
 * @file test_pipeline.c
 * @brief Integration tests: real radio driver on the software radio model, real uplink and downlink
 *
 * A block is injected as if received over the air, passes the radio driver, radio_port, uplink, router
 * and downlink, and the block the radio then transmits is taken from the model and decoded. The service
 * functions are called from the test thread instead of from separate threads, so the order is exact.
 */
#include <errno.h>
#include <zephyr/ztest.h>

#include <ut/drivers/rfm26w.h>

#include "app/downlink.h"
#include "app/ping.h"
#include "can/can_ping.h"
#include "common/can_proto.h"
#include "app/radio_port.h"
#include "app/uplink.h"
#include "link/dl_block.h"
#include "rfm26w_emul.h"
#include "test_fake_can.h"
#include "test_util.h"

#define RADIO_NODE DT_NODELABEL(rfm26w0)

K_MSGQ_DEFINE(p_uplink_q, COMMS_BLOCK_LEN, 4, 1);
K_MSGQ_DEFINE(p_tx_q, sizeof(struct dl_item), 2, 4);

static const struct device *const radio = DEVICE_DT_GET(RADIO_NODE);
static struct rfm26w_emul_data *emul;
static struct radio_port port;
static struct downlink dl;
static struct uplink ul;
static struct router rt;
static struct can_bridge br;

static void *setup(void)
{
	emul = RFM26W_EMUL_DATA();
	return NULL;
}

static void before(void *f)
{
	ARG_UNUSED(f);
	(void)rfm26w_shutdown(radio);
	rfm26w_emul_reset(emul);
	emul->us_per_byte = 50; /* fast air, so soak tests do not take minutes */
	k_msgq_purge(&p_uplink_q);
	k_msgq_purge(&p_tx_q);
	radio_port_init(&port, radio, &p_uplink_q);
	downlink_init(&dl, &p_tx_q, &port);
	fake_can_attach(&br);
	router_init(&rt, &br, (struct mvp_switches){.ping = true, .can_cmd = true});
	uplink_init(&ul, &p_uplink_q, &dl, &rt);
	zassert_ok(rfm26w_start(radio));
	zassert_ok(radio_port_start(&port));
}

static void after(void *f)
{
	ARG_UNUSED(f);
	zassert_equal(emul->cmds_while_busy, 0);
	zassert_equal(emul->bad_cmds, 0);
	zassert_equal(emul->ant_violations, 0);
	zassert_equal(emul->fifo_underflows, 0);
	zassert_equal(emul->fifo_overflows, 0);
	(void)rfm26w_shutdown(radio);
}

ZTEST_SUITE(pipeline, NULL, setup, before, after, NULL);

static void inject(const uint8_t *block)
{
	zassert_ok(rfm26w_emul_inject_rx(emul, block, COMMS_BLOCK_LEN, 100));
}

/* Take the block the radio transmitted and decode it down to the Space Packet. */
static void take_reply(struct sp_packet *pkt, uint8_t *keep)
{
	size_t len = COMMS_BLOCK_LEN;
	const uint8_t *p;
	size_t plen;

	zassert_ok(rfm26w_emul_take_tx(emul, keep, &len));
	zassert_equal(len, COMMS_BLOCK_LEN);
	zassert_equal(dl_block_unpack(keep, len, &p, &plen), DL_OK);
	zassert_equal(sp_parse(p, plen, pkt), SP_OK);
}

/* Run one ping through everything and return once the reply has been transmitted. */
static void ping_once(uint32_t id, const uint8_t *payload, size_t n)
{
	uint8_t block[COMMS_BLOCK_LEN];

	test_make_ping_block(block, id, payload, n);
	inject(block);
	WAIT_UNTIL(k_msgq_num_used_get(&p_uplink_q) >= 1, 1000);
	zassert_ok(uplink_service_one(&ul, K_NO_WAIT), "block did not reach the uplink queue");
	zassert_ok(downlink_service_one(&dl, K_NO_WAIT));
}

ZTEST(pipeline, test_ping_over_the_air_gets_the_matching_reply)
{
	const uint8_t payload[] = "hello, ground";
	uint8_t keep[COMMS_BLOCK_LEN];
	struct sp_packet pkt;

	ping_once(0x01020304, payload, sizeof(payload));
	take_reply(&pkt, keep);
	zassert_equal(pkt.apid, APID_DL_PING);
	zassert_equal(pkt.seq_count, 0);
	zassert_equal(pkt.data_len, 4 + sizeof(payload));
	zassert_equal(pkt.data[0], 0x01);
	zassert_equal(pkt.data[3], 0x04);
	zassert_mem_equal(&pkt.data[4], payload, sizeof(payload));
	zassert_equal(ul.stats.accepted, 1);
	zassert_equal(dl.stats.sent, 1);
	zassert_equal(emul->start_rx_count >= 2, true, "receive resumes after the reply");
}

ZTEST(pipeline, test_ping_with_no_payload_and_with_the_largest_payload)
{
	uint8_t big[PING_MAX_PAYLOAD];
	uint8_t keep[COMMS_BLOCK_LEN];
	struct sp_packet pkt;
	struct test_rng r = {.s = 4242};

	test_rng_fill(&r, big, sizeof(big));
	ping_once(7, NULL, 0);
	take_reply(&pkt, keep);
	zassert_equal(pkt.data_len, 4);
	ping_once(8, big, sizeof(big));
	take_reply(&pkt, keep);
	zassert_equal(pkt.data_len, 4 + sizeof(big));
	zassert_mem_equal(&pkt.data[4], big, sizeof(big));
}

ZTEST(pipeline, test_a_lost_block_means_no_reply_and_the_next_ping_still_works)
{
	uint8_t keep[COMMS_BLOCK_LEN];
	struct sp_packet pkt;
	size_t len = COMMS_BLOCK_LEN;

	ping_once(1, NULL, 0);
	take_reply(&pkt, keep);
	zassert_equal(pkt.data[3], 1);
	/* ping 2 never arrives: nothing is injected */
	zassert_equal(uplink_service_one(&ul, K_NO_WAIT), -EAGAIN);
	zassert_equal(rfm26w_emul_take_tx(emul, keep, &len), -ENOENT);
	ping_once(3, NULL, 0);
	take_reply(&pkt, keep);
	zassert_equal(pkt.data[3], 3);
	zassert_equal(pkt.seq_count, 1, "the sequence count only counts packets that were sent");
}

ZTEST(pipeline, test_a_corrupt_block_is_dropped_and_counted_and_nothing_is_sent)
{
	uint8_t block[COMMS_BLOCK_LEN];
	uint8_t keep[COMMS_BLOCK_LEN];
	size_t len = COMMS_BLOCK_LEN;

	test_make_ping_block(block, 5, NULL, 0);
	block[3] ^= 0x40;
	inject(block);
	WAIT_UNTIL(k_msgq_num_used_get(&p_uplink_q) >= 1, 1000);
	zassert_ok(uplink_service_one(&ul, K_NO_WAIT));
	zassert_equal(ul.stats.tc_drop[TC_ERR_FECF], 1);
	zassert_equal(k_msgq_num_used_get(&p_tx_q), 0);
	zassert_equal(rfm26w_emul_take_tx(emul, keep, &len), -ENOENT);
}

ZTEST(pipeline, test_a_ping_with_the_wrong_spacecraft_id_is_ignored)
{
	uint8_t block[COMMS_BLOCK_LEN];
	uint8_t pkt[16];
	const uint8_t data[4] = {0};
	size_t n = test_make_ul_packet(pkt, sizeof(pkt), APID_UL_PING, data, 4);

	zassert_ok(tc_frame_build(block, COMMS_TC_SCID ^ 0x10, COMMS_TC_VCID, pkt, n));
	inject(block);
	WAIT_UNTIL(k_msgq_num_used_get(&p_uplink_q) >= 1, 1000);
	zassert_ok(uplink_service_one(&ul, K_NO_WAIT));
	zassert_equal(ul.stats.tc_drop[TC_ERR_SCID], 1);
	zassert_equal(k_msgq_num_used_get(&p_tx_q), 0);
}

ZTEST(pipeline, test_back_to_back_pings_are_all_answered_in_order)
{
	uint8_t keep[COMMS_BLOCK_LEN];
	struct sp_packet pkt;
	uint8_t block[COMMS_BLOCK_LEN];

	for (uint32_t id = 10; id < 13; id++) {
		test_make_ping_block(block, id, NULL, 0);
		inject(block);
	}
	WAIT_UNTIL(k_msgq_num_used_get(&p_uplink_q) >= 3, 2000);
	zassert_equal(k_msgq_num_used_get(&p_uplink_q), 3, "dropped %u", port.stats.rx_dropped);
	for (int i = 0; i < 3; i++) {
		zassert_ok(uplink_service_one(&ul, K_NO_WAIT));
		zassert_ok(downlink_service_one(&dl, K_NO_WAIT));
	}
	for (uint32_t id = 10; id < 13; id++) {
		take_reply(&pkt, keep);
		zassert_equal(pkt.data[3], id);
		zassert_equal(pkt.seq_count, id - 10);
	}
}

ZTEST(pipeline, test_a_full_transmit_queue_drops_replies_and_counts_them)
{
	uint8_t block[COMMS_BLOCK_LEN];

	/* The transmit queue holds 2. Answer 4 pings without letting the transmit side run. */
	for (uint32_t id = 0; id < 4; id++) {
		test_make_ping_block(block, id, NULL, 0);
		inject(block);
		WAIT_UNTIL(k_msgq_num_used_get(&p_uplink_q) >= 1, 1000);
		zassert_ok(uplink_service_one(&ul, K_NO_WAIT));
	}
	zassert_equal(dl.stats.queued, 2);
	zassert_equal(dl.stats.queue_full, 2);
	zassert_equal(rt.stats.ping, 4);
	/* What is queued still goes out. */
	zassert_ok(downlink_service_one(&dl, K_NO_WAIT));
	zassert_ok(downlink_service_one(&dl, K_NO_WAIT));
	zassert_equal(dl.stats.sent, 2);
}

ZTEST(pipeline, test_full_uplink_queue_drops_blocks_and_counts_them)
{
	uint8_t block[COMMS_BLOCK_LEN];

	for (uint32_t id = 0; id < 6; id++) {
		test_make_ping_block(block, id, NULL, 0);
		inject(block);
		WAIT_UNTIL(port.stats.rx_blocks + port.stats.rx_dropped >= id + 1, 1000);
	}
	zassert_equal(port.stats.rx_blocks, 4);
	zassert_equal(port.stats.rx_dropped, 2);
}

ZTEST(pipeline, test_many_pings_in_a_row_all_come_back_intact)
{
	uint8_t keep[COMMS_BLOCK_LEN];
	uint8_t payload[32];
	struct sp_packet pkt;
	struct test_rng r = {.s = 2026};

	for (uint32_t id = 0; id < 300; id++) {
		test_rng_fill(&r, payload, sizeof(payload));
		ping_once(id, payload, sizeof(payload));
		take_reply(&pkt, keep);
		zassert_equal(pkt.apid, APID_DL_PING);
		zassert_equal(pkt.seq_count, id & SP_SEQ_COUNT_MASK);
		zassert_equal(pkt.data_len, 4 + sizeof(payload));
		zassert_equal(pkt.data[3], (uint8_t)id);
		zassert_mem_equal(&pkt.data[4], payload, sizeof(payload));
	}
	zassert_equal(dl.stats.send_errors, 0);
	zassert_equal(port.stats.rx_dropped, 0);
}

/* ---- CAN_CMD: radio to CAN bus and back ---- */

static void make_cmd_block(uint8_t *block, const struct can_record *rec)
{
	uint8_t wire[CAN_RECORD_LEN];
	uint8_t pkt[COMMS_UL_MAX_PACKET_CLEAR];
	size_t n;

	can_record_pack(wire, rec);
	n = test_make_ul_packet(pkt, sizeof(pkt), APID_UL_CMD, wire, sizeof(wire));
	zassert_ok(tc_frame_build(block, COMMS_TC_SCID, COMMS_TC_VCID, pkt, n));
}

/* Send a command over the air and run the uplink side. */
static void cmd_over_the_air(const struct can_record *rec)
{
	uint8_t block[COMMS_BLOCK_LEN];

	make_cmd_block(block, rec);
	inject(block);
	WAIT_UNTIL(k_msgq_num_used_get(&p_uplink_q) >= 1, 1000);
	zassert_ok(uplink_service_one(&ul, K_NO_WAIT), "block did not reach the uplink queue");
}

/* A stand-in for CDH, written separately from the firmware handler: answer a ping with a pong to its sender. */
static struct can_frame fake_cdh_answer(const struct can_frame *ping)
{
	struct can_frame f = {.id = CAN_ID_FULL(3, CDH_ID, (ping->id >> 14) & 0xFF, CLS_CMD_RESP)};

	zassert_equal(ping->data[1], OP_PING);
	can_fill_payload(&f, CDH_ID, OP_PONG, ping->data[2], ping->data[3], ping->data[4], ping->data[5],
			 ping->data[6], ping->data[7]);
	return f;
}

/* The board's answer arrives from the CAN bus and goes down the radio. */
static void answer_comes_back(const struct can_frame *answer)
{
	struct dl_item it;

	zassert_equal(can_bridge_on_frame(&br, answer, &it), 1);
	zassert_ok(downlink_submit(&dl, &it));
	zassert_ok(downlink_service_one(&dl, K_NO_WAIT));
}

ZTEST(pipeline, test_can_ping_goes_to_the_bus_and_the_pong_comes_back_over_the_radio)
{
	const uint8_t token[COMMS_PING_TOKEN_LEN] = {0xA1, 0xB2, 0xC3, 0xD4, 0xE5, 0xF6};
	uint8_t keep[COMMS_BLOCK_LEN];
	struct can_record ping, got;
	struct can_frame answer;
	struct sp_packet pkt;

	comms_can_ping_record(&ping, CDH_ID, token);
	cmd_over_the_air(&ping);

	zassert_equal(fcan.count, 1, "the ping must be on the bus");
	zassert_equal(rt.stats.cmd_forwarded, 1);
	zassert_equal(k_msgq_num_used_get(&p_tx_q), 0, "nothing goes down until the board answers");

	answer = fake_cdh_answer(&fcan.frames[0]);
	answer_comes_back(&answer);

	take_reply(&pkt, keep);
	zassert_equal(pkt.apid, APID_DL_TLM);
	zassert_equal(pkt.data_len, 4 + CAN_RECORD_LEN);
	zassert_equal(can_record_unpack(&pkt.data[4], CAN_RECORD_LEN, &got), REC_OK);
	zassert_true(comms_can_pong_check(&got, CDH_ID, token));
}

ZTEST(pipeline, test_a_denied_command_never_reaches_the_bus_and_nothing_goes_down)
{
	const uint8_t token[COMMS_PING_TOKEN_LEN] = {0};
	struct can_record r;
	uint8_t keep[COMMS_BLOCK_LEN];
	size_t len = COMMS_BLOCK_LEN;

	comms_can_ping_record(&r, CDH_ID, token);
	r.data[1] = OP_REBOOT;
	cmd_over_the_air(&r);

	comms_can_ping_record(&r, CDH_ID, token);
	r.id = CAN_ID_FULL(0, EPS_ID, CDH_ID, CLS_COMMAND); /* pretends to be another board */
	cmd_over_the_air(&r);

	zassert_equal(fcan.attempts, 0);
	zassert_equal(rt.stats.cmd_denied[POLICY_ERR_NOT_ALLOWED], 1);
	zassert_equal(rt.stats.cmd_denied[POLICY_ERR_SRC], 1);
	zassert_equal(rfm26w_emul_take_tx(emul, keep, &len), -ENOENT);
}

ZTEST(pipeline, test_two_can_pings_get_distinct_counters_and_sequence_counts)
{
	uint8_t keep[COMMS_BLOCK_LEN];
	struct sp_packet pkt;

	for (uint8_t i = 0; i < 2; i++) {
		const uint8_t token[COMMS_PING_TOKEN_LEN] = {i, 9, 9, 9, 9, 9};
		struct can_record ping;
		struct can_frame answer;

		comms_can_ping_record(&ping, CDH_ID, token);
		cmd_over_the_air(&ping);
		answer = fake_cdh_answer(&fcan.frames[i]);
		answer_comes_back(&answer);
	}
	for (uint8_t i = 0; i < 2; i++) {
		take_reply(&pkt, keep);
		zassert_equal(pkt.seq_count, i);
		zassert_equal(pkt.data[3], i, "tlm_counter");
		zassert_equal(pkt.data[4 + 4 + 2], i, "token byte 0 of the echoed record is data byte 2");
	}
}

ZTEST(pipeline, test_a_can_send_failure_is_counted_and_the_link_keeps_working)
{
	const uint8_t token[COMMS_PING_TOKEN_LEN] = {0};
	struct can_record ping;
	uint8_t keep[COMMS_BLOCK_LEN];
	struct sp_packet pkt;

	fcan.result = -EAGAIN;
	comms_can_ping_record(&ping, CDH_ID, token);
	cmd_over_the_air(&ping);
	zassert_equal(rt.stats.cmd_can_error, 1);
	zassert_equal(k_msgq_num_used_get(&p_tx_q), 0);

	fcan.result = 0;
	ping_once(77, NULL, 0); /* the radio ping still works */
	take_reply(&pkt, keep);
	zassert_equal(pkt.apid, APID_DL_PING);
}

ZTEST(pipeline, test_a_board_frame_for_someone_else_never_goes_down)
{
	struct can_frame f = {.id = CAN_ID_FULL(3, CDH_ID, EPS_ID, CLS_COMMAND)};
	struct dl_item it;

	can_fill_payload(&f, CDH_ID, OP_SET_PWR_STATE, 1, 1, 0, 0, 0, 0);
	zassert_equal(can_bridge_on_frame(&br, &f, &it), 0);
	zassert_equal(k_msgq_num_used_get(&p_tx_q), 0);
}
