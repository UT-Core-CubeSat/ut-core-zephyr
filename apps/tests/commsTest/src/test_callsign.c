/** @file test_callsign.c @brief Tests for link/callsign */
#include <errno.h>
#include <zephyr/ztest.h>

#include "link/callsign.h"

ZTEST_SUITE(callsign, NULL, NULL, NULL, NULL, NULL);

ZTEST(callsign, test_five_characters_are_space_padded_to_six_then_the_ssid)
{
	uint8_t out[COMMS_SEC_HDR_LEN];
	const uint8_t want[] = {'K', '7', 'U', 'T', 'S', ' ', 9};

	zassert_ok(callsign_pack(out, "K7UTS", 9));
	zassert_mem_equal(out, want, sizeof(want));
}

ZTEST(callsign, test_one_and_six_characters_are_the_limits)
{
	uint8_t out[COMMS_SEC_HDR_LEN];

	zassert_ok(callsign_pack(out, "W", 0));
	zassert_mem_equal(out, "W     ", 6);
	zassert_ok(callsign_pack(out, "AB1CDE", 15));
	zassert_mem_equal(out, "AB1CDE", 6);
	zassert_equal(out[6], 15);
}

ZTEST(callsign, test_empty_too_long_and_bad_characters_are_refused)
{
	uint8_t out[COMMS_SEC_HDR_LEN];

	zassert_equal(callsign_pack(out, "", 0), -EINVAL);
	zassert_equal(callsign_pack(out, "ABCDEFG", 0), -EINVAL);
	zassert_equal(callsign_pack(out, "k7uts", 0), -EINVAL, "lower case");
	zassert_equal(callsign_pack(out, "K7 TS", 0), -EINVAL, "space");
	zassert_equal(callsign_pack(out, "K7-TS", 0), -EINVAL, "dash");
	zassert_equal(callsign_pack(NULL, "K7UTS", 0), -EINVAL);
	zassert_equal(callsign_pack(out, NULL, 0), -EINVAL);
}

ZTEST(callsign, test_a_refused_call_leaves_the_output_alone)
{
	uint8_t out[COMMS_SEC_HDR_LEN];

	memset(out, 0x5A, sizeof(out));
	zassert_equal(callsign_pack(out, "BAD CALL", 0), -EINVAL);
	for (size_t i = 0; i < sizeof(out); i++) {
		zassert_equal(out[i], 0x5A);
	}
}

ZTEST(callsign, test_the_configured_callsign_is_valid)
{
	uint8_t out[COMMS_SEC_HDR_LEN];

	zassert_ok(callsign_pack(out, COMMS_CALLSIGN, COMMS_SSID));
}
