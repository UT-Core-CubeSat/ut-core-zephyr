/** @file test_space_packet.c @brief Tests for link/space_packet */
#include <errno.h>
#include <string.h>
#include <zephyr/ztest.h>

#include "comms_config.h"
#include "link/space_packet.h"
#include "test_util.h"

ZTEST_SUITE(space_packet, NULL, NULL, NULL, NULL, NULL);

ZTEST(space_packet, test_header_bytes_match_the_spec_layout)
{
	uint8_t out[32];
	const uint8_t data[3] = {1, 2, 3};
	int n;

	/* telecommand, APID 0x010, count 5, 3 data bytes:
	 * byte 0 = 000 (version) 1 (type) 0 (sec hdr) 000 (APID high) = 0x10, byte 1 = 0x10,
	 * byte 2 = 11 (flags) 000000 (count high) = 0xC0, byte 3 = 5, length field = 3 - 1 = 2 */
	n = sp_build(out, sizeof(out), true, false, APID_UL_CMD, 5, data, 3);
	zassert_equal(n, 9);
	zassert_equal(out[0], 0x10);
	zassert_equal(out[1], 0x10);
	zassert_equal(out[2], 0xC0);
	zassert_equal(out[3], 0x05);
	zassert_equal(out[4], 0x00);
	zassert_equal(out[5], 0x02);
	zassert_mem_equal(out + 6, data, 3);

	/* telemetry, secondary header flag, APID 0x7FF, count 0x3FFF: byte 0 = 000 0 1 111 = 0x0F */
	n = sp_build(out, sizeof(out), false, true, 0x7FF, 0x3FFF, data, 3);
	zassert_equal(n, 9);
	zassert_equal(out[0], 0x0F);
	zassert_equal(out[1], 0xFF);
	zassert_equal(out[2], 0xFF);
	zassert_equal(out[3], 0xFF);
}

ZTEST(space_packet, test_data_length_field_is_split_across_two_bytes)
{
	uint8_t out[400];
	uint8_t data[300];
	struct sp_packet p;

	memset(data, 0x33, sizeof(data));
	/* 300 data bytes: length field 299 = 0x012B */
	zassert_equal(sp_build(out, sizeof(out), true, false, 0x100, 0, data, sizeof(data)), 306);
	zassert_equal(out[4], 0x01);
	zassert_equal(out[5], 0x2B);
	zassert_equal(sp_parse(out, sizeof(out), &p), SP_OK);
	zassert_equal(p.data_len, 300);
}

ZTEST(space_packet, test_round_trip_every_length_and_field_extreme)
{
	struct test_rng rng = {.s = 0x5EED5EED};
	uint8_t data[COMMS_DL_MAX_PACKET];
	uint8_t out[COMMS_DL_MAX_PACKET + 6];
	struct sp_packet p;

	for (size_t n = 1; n <= COMMS_DL_MAX_PACKET - COMMS_SP_HDR_LEN; n++) {
		const uint16_t apid = (uint16_t)(test_rng_next(&rng) & 0x7FF);
		const uint16_t count = (uint16_t)(test_rng_next(&rng) & 0x3FFF);
		const bool tc = (test_rng_next(&rng) & 1) != 0;
		const bool sec = (test_rng_next(&rng) & 1) != 0;

		test_rng_fill(&rng, data, n);
		zassert_equal(sp_build(out, sizeof(out), tc, sec, apid, count, data, n), (int)(6 + n));
		zassert_equal(sp_parse(out, sizeof(out), &p), SP_OK);
		zassert_equal(p.is_tc, tc);
		zassert_equal(p.sec_hdr, sec);
		zassert_equal(p.apid, apid);
		zassert_equal(p.seq_count, count);
		zassert_equal(p.data_len, n);
		zassert_equal(p.total_len, 6 + n);
		zassert_mem_equal(p.data, data, n);
	}
}

ZTEST(space_packet, test_trailing_bytes_are_not_part_of_the_packet)
{
	uint8_t out[64] = {0};
	const uint8_t data[5] = {1, 2, 3, 4, 5};
	struct sp_packet p;

	zassert_equal(sp_build(out, sizeof(out), true, false, APID_UL_PING, 0, data, 5), 11);
	memset(out + 11, 0xEE, sizeof(out) - 11);
	zassert_equal(sp_parse(out, sizeof(out), &p), SP_OK);
	zassert_equal(p.total_len, 11);
	zassert_equal(p.data_len, 5);
}

ZTEST(space_packet, test_parse_errors)
{
	uint8_t out[32];
	const uint8_t data[4] = {1, 2, 3, 4};
	struct sp_packet p;

	zassert_equal(sp_build(out, sizeof(out), true, false, 0x10, 0, data, 4), 10);

	zassert_equal(sp_parse(out, 6, &p), SP_ERR_SHORT, "needs at least 7 bytes");
	zassert_equal(sp_parse(out, 0, &p), SP_ERR_SHORT);
	zassert_equal(sp_parse(out, 9, &p), SP_ERR_LEN, "packet cut off one byte short");

	out[0] |= 0x20; /* version 001 */
	zassert_equal(sp_parse(out, sizeof(out), &p), SP_ERR_VERSION);
	out[0] &= (uint8_t)~0x20;
	zassert_equal(sp_parse(out, sizeof(out), &p), SP_OK);

	out[2] = (uint8_t)((out[2] & 0x3F) | 0x40); /* flags 01: first segment */
	zassert_equal(sp_parse(out, sizeof(out), &p), SP_ERR_FLAGS);
	out[2] = (uint8_t)(out[2] & 0x3F); /* flags 00: continuation */
	zassert_equal(sp_parse(out, sizeof(out), &p), SP_ERR_FLAGS);

	zassert_equal(sp_parse(NULL, 10, &p), SP_ERR_ARG);
	zassert_equal(sp_parse(out, 10, NULL), SP_ERR_ARG);
}

ZTEST(space_packet, test_a_length_field_larger_than_the_buffer_is_rejected)
{
	uint8_t out[16];
	const uint8_t data[2] = {1, 2};
	struct sp_packet p;

	zassert_equal(sp_build(out, sizeof(out), true, false, 0x10, 0, data, 2), 8);
	out[4] = 0xFF;
	out[5] = 0xFF; /* claims 65536 data bytes */
	zassert_equal(sp_parse(out, sizeof(out), &p), SP_ERR_LEN);
}

ZTEST(space_packet, test_build_argument_and_size_errors)
{
	uint8_t out[16];
	const uint8_t data[8] = {0};

	zassert_equal(sp_build(NULL, 16, true, false, 0, 0, data, 1), -EINVAL);
	zassert_equal(sp_build(out, 16, true, false, 0, 0, NULL, 1), -EINVAL);
	zassert_equal(sp_build(out, 16, true, false, 0, 0, data, 0), -EINVAL, "data field has at least 1 byte");
	zassert_equal(sp_build(out, 16, true, false, 0x800, 0, data, 1), -EINVAL, "APID is 11 bits");
	zassert_equal(sp_build(out, 16, true, false, 0, 0x4000, data, 1), -EINVAL, "count is 14 bits");
	zassert_equal(sp_build(out, 16, true, false, 0, 0, data, SP_MAX_DATA_LEN + 1), -EINVAL);
	zassert_equal(sp_build(out, 13, true, false, 0, 0, data, 8), -EMSGSIZE, "needs 14 bytes");
	zassert_equal(sp_build(out, 14, true, false, 0, 0, data, 8), 14, "fits exactly");
}

ZTEST(space_packet, test_the_sequence_counter_wraps_at_16384)
{
	struct sp_seq s;

	sp_seq_init(&s);
	zassert_equal(sp_seq_next(&s, APID_DL_TLM), 0);
	zassert_equal(sp_seq_next(&s, APID_DL_TLM), 1);
	zassert_equal(sp_seq_next(&s, APID_DL_TLM), 2);

	s.next[APID_DL_TLM] = 0x3FFF;
	zassert_equal(sp_seq_next(&s, APID_DL_TLM), 0x3FFF);
	zassert_equal(sp_seq_next(&s, APID_DL_TLM), 0, "wraps from 16383 to 0");
	zassert_equal(sp_seq_next(&s, APID_DL_TLM), 1);
}

ZTEST(space_packet, test_counters_are_independent_per_apid)
{
	struct sp_seq s;

	sp_seq_init(&s);
	sp_seq_next(&s, APID_DL_TLM);
	sp_seq_next(&s, APID_DL_TLM);
	zassert_equal(sp_seq_next(&s, APID_DL_BEACON), 0, "a different APID starts at 0");
	zassert_equal(sp_seq_next(&s, APID_DL_TLM), 2);
	zassert_equal(sp_seq_next(&s, APID_DL_BEACON), 1);
	zassert_equal(sp_seq_next(&s, 0x800), 0, "out of range APID");
	zassert_equal(sp_seq_next(&s, 0xFFFF), 0);
	zassert_equal(sp_seq_next(&s, 0x7FF), 0, "the largest APID is valid");
	zassert_equal(sp_seq_next(&s, 0x7FF), 1);
}

ZTEST(space_packet, test_lost_packet_count)
{
	zassert_equal(sp_seq_lost(5, 5), 0);
	zassert_equal(sp_seq_lost(5, 8), 3);
	zassert_equal(sp_seq_lost(0x3FFF, 1), 2, "across the wrap");
	zassert_equal(sp_seq_lost(0, 0x3FFF), 0x3FFF);
}

ZTEST(space_packet, test_random_buffers_never_crash)
{
	struct test_rng rng = {.s = 0xFACEFEED};
	uint8_t buf[COMMS_BLOCK_LEN];
	struct sp_packet p;

	for (int i = 0; i < 20000; i++) {
		const size_t len = test_rng_next(&rng) % (sizeof(buf) + 1);

		test_rng_fill(&rng, buf, sizeof(buf));
		if (sp_parse(buf, len, &p) == SP_OK) {
			zassert_true(p.total_len <= len, "an accepted packet must lie inside the buffer");
			zassert_true(p.data_len >= 1);
			zassert_equal(p.data, buf + 6);
		}
	}
}

ZTEST(space_packet, test_status_names)
{
	zassert_str_equal(sp_status_str(SP_OK), "ok");
	zassert_str_equal(sp_status_str(SP_STATUS_COUNT), "unknown");
	for (int s = 0; s < SP_STATUS_COUNT; s++) {
		zassert_true(strlen(sp_status_str((enum sp_status)s)) > 0);
	}
}
