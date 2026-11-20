/** @file test_crc16.c @brief Tests for link/crc16 */
#include <string.h>
#include <zephyr/ztest.h>

#include "link/crc16.h"
#include "test_util.h"

ZTEST_SUITE(crc16, NULL, NULL, NULL, NULL, NULL);

ZTEST(crc16, test_check_value)
{
	/* PROTOCOL.md sections 2 and 14: CRC-16/CCITT-FALSE of "123456789" is 0x29B1 */
	zassert_equal(comms_crc16((const uint8_t *)"123456789", 9), 0x29B1);
}

ZTEST(crc16, test_empty_is_the_initial_value)
{
	zassert_equal(comms_crc16(NULL, 0), 0xFFFF);
}

ZTEST(crc16, test_matches_the_independent_reference)
{
	struct test_rng rng = {.s = 0x12345678};
	uint8_t buf[300];

	test_rng_fill(&rng, buf, sizeof(buf));
	for (size_t len = 0; len <= sizeof(buf); len++) {
		zassert_equal(comms_crc16(buf, len), ref_crc16(buf, len), "length %u", (unsigned int)len);
	}
}

ZTEST(crc16, test_is_not_the_reflected_variant)
{
	/* Zephyr's crc16_ccitt() reflects and gives 0x2189 for the check string. Using it would be a bug. */
	zassert_not_equal(comms_crc16((const uint8_t *)"123456789", 9), 0x2189);
}
