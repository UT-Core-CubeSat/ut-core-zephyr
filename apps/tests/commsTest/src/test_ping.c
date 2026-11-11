/** @file test_ping.c @brief Tests for app/ping (PING reply data field) */
#include <errno.h>
#include <zephyr/ztest.h>

#include "app/ping.h"
#include "test_util.h"

ZTEST_SUITE(ping, NULL, NULL, NULL, NULL, NULL);

ZTEST(ping, test_reply_echoes_id_and_payload_unchanged)
{
	const uint8_t req[] = {0x01, 0x02, 0x03, 0x04, 'a', 'b', 'c', 0x00, 0xFF};
	uint8_t out[PING_MAX_DATA];

	zassert_equal(ping_make_reply(req, sizeof(req), out, sizeof(out)), (int)sizeof(req));
	zassert_mem_equal(out, req, sizeof(req));
}

ZTEST(ping, test_id_only_and_largest_payload)
{
	uint8_t req[PING_MAX_DATA];
	uint8_t out[PING_MAX_DATA];
	struct test_rng r = {.s = 7};

	test_rng_fill(&r, req, sizeof(req));
	zassert_equal(ping_make_reply(req, PING_ID_LEN, out, sizeof(out)), PING_ID_LEN);
	zassert_mem_equal(out, req, PING_ID_LEN);
	zassert_equal(ping_make_reply(req, PING_MAX_DATA, out, sizeof(out)), PING_MAX_DATA);
	zassert_mem_equal(out, req, PING_MAX_DATA);
}

ZTEST(ping, test_every_payload_length_round_trips)
{
	uint8_t req[PING_MAX_DATA];
	uint8_t out[PING_MAX_DATA];
	struct test_rng r = {.s = 99};

	for (size_t n = PING_ID_LEN; n <= PING_MAX_DATA; n++) {
		memset(out, 0, sizeof(out));
		test_rng_fill(&r, req, n);
		zassert_equal(ping_make_reply(req, n, out, sizeof(out)), (int)n);
		zassert_mem_equal(out, req, n);
	}
}

ZTEST(ping, test_length_limits)
{
	uint8_t req[PING_MAX_DATA + 1] = {0};
	uint8_t out[PING_MAX_DATA + 1];

	zassert_equal(ping_make_reply(req, 0, out, sizeof(out)), -EINVAL);
	zassert_equal(ping_make_reply(req, PING_ID_LEN - 1, out, sizeof(out)), -EINVAL);
	zassert_equal(ping_make_reply(req, PING_MAX_DATA + 1, out, sizeof(out)), -EINVAL);
}

ZTEST(ping, test_output_buffer_too_small_and_null_arguments)
{
	uint8_t req[8] = {0};
	uint8_t out[8];

	zassert_equal(ping_make_reply(req, 8, out, 7), -EMSGSIZE);
	zassert_equal(ping_make_reply(NULL, 8, out, 8), -EINVAL);
	zassert_equal(ping_make_reply(req, 8, NULL, 8), -EINVAL);
}

ZTEST(ping, test_reply_fits_a_downlink_packet)
{
	zassert_true(PING_MAX_DATA <= (COMMS_DL_MAX_PACKET - COMMS_SP_HDR_LEN));
}
