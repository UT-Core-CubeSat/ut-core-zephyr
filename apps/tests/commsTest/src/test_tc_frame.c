/** @file test_tc_frame.c @brief Tests for link/tc_frame (uplink TC Type-BD frame) */
#include <errno.h>
#include <string.h>
#include <zephyr/ztest.h>

#include "comms_config.h"
#include "link/tc_frame.h"
#include "test_util.h"

ZTEST_SUITE(tc_frame, NULL, NULL, NULL, NULL, NULL);

/* Build a frame by hand, independent of tc_frame_build(): header fields exactly as CCSDS 232.0-B-4 figure
 * 4-2 gives them, FECF from the reference CRC. Returns the frame length. */
static size_t hand_frame(uint8_t *out, unsigned int version, unsigned int bypass, unsigned int control,
			 unsigned int spare, unsigned int scid, unsigned int vcid, const uint8_t *data,
			 size_t data_len)
{
	size_t frame_len = 5 + data_len + 2;

	memset(out, 0, 400);
	out[0] = (uint8_t)((version << 6) | (bypass << 5) | (control << 4) | (spare << 2) | (scid >> 8));
	out[1] = (uint8_t)(scid & 0xFF);
	out[2] = (uint8_t)((vcid << 2) | ((frame_len - 1) >> 8));
	out[3] = (uint8_t)((frame_len - 1) & 0xFF);
	out[4] = 0;
	memcpy(out + 5, data, data_len);
	test_seal_crc(out, frame_len);
	return frame_len;
}

#define GOOD_HAND(out, data, len) hand_frame(out, 0, 1, 0, 0, COMMS_TC_SCID, COMMS_TC_VCID, data, len)

ZTEST(tc_frame, test_header_bytes_match_the_spec_layout)
{
	uint8_t block[COMMS_BLOCK_LEN];
	const uint8_t data[1] = {0xAA};

	/* SCID 0x155 = 01 0101 0101, VCID 0, data 1 byte: frame length 8, length field 7.
	 * byte 0 = 00 (version) 1 (bypass) 0 (control) 00 (spare) 01 (SCID high) = 0x21 */
	zassert_ok(tc_frame_build(block, 0x155, 0, data, 1));
	zassert_equal(block[0], 0x21);
	zassert_equal(block[1], 0x55);
	zassert_equal(block[2], 0x00);
	zassert_equal(block[3], 0x07);
	zassert_equal(block[4], 0x00);
	zassert_equal(block[5], 0xAA);
	zassert_equal((block[6] << 8) | block[7], ref_crc16(block, 6), "FECF covers the header and the data");
	zassert_equal(block[8], 0x00, "zero fill after the frame");
	zassert_equal(block[COMMS_BLOCK_LEN - 1], 0x00);
}

ZTEST(tc_frame, test_vcid_and_a_full_block_land_in_the_right_bits)
{
	uint8_t block[COMMS_BLOCK_LEN];
	uint8_t data[COMMS_UL_MAX_PACKET_CLEAR];

	memset(data, 0x5A, sizeof(data));
	/* VCID 0x2A in bits 7-2 of byte 2. Full 223-byte frame: length field 222 = 0xDE. */
	zassert_ok(tc_frame_build(block, 0x3FF, 0x2A, data, sizeof(data)));
	zassert_equal(block[0], 0x20 | 0x03, "SCID 0x3FF high bits");
	zassert_equal(block[1], 0xFF);
	zassert_equal(block[2], (0x2A << 2) | 0x00);
	zassert_equal(block[3], 0xDE);
}

ZTEST(tc_frame, test_frame_length_high_bits_are_parsed)
{
	/* A block longer than ours, so the two high bits of the 10-bit length field are exercised:
	 * frame length 300, field 299 = 0x12B, so byte 2 low bits 01 and byte 3 0x2B. */
	uint8_t buf[400];
	uint8_t data[293];
	struct tc_frame f;
	size_t len;

	memset(data, 0x77, sizeof(data));
	len = GOOD_HAND(buf, data, sizeof(data));
	zassert_equal(len, 300);
	zassert_equal(buf[2] & 0x03, 0x01);
	zassert_equal(buf[3], 0x2B);
	zassert_equal(tc_frame_parse(buf, sizeof(buf), &f), TC_OK);
	zassert_equal(f.data_len, 293);
	zassert_equal(f.frame_len, 300);
}

ZTEST(tc_frame, test_round_trip_every_length)
{
	struct test_rng rng = {.s = 0xC0FFEE01};
	uint8_t block[COMMS_BLOCK_LEN];
	uint8_t data[COMMS_UL_MAX_PACKET_CLEAR];
	struct tc_frame f;

	for (size_t n = 0; n <= COMMS_UL_MAX_PACKET_CLEAR; n++) {
		test_rng_fill(&rng, data, n);
		zassert_ok(tc_frame_build(block, COMMS_TC_SCID, COMMS_TC_VCID, data, n), "build %u",
			   (unsigned int)n);
		zassert_equal(tc_frame_parse(block, sizeof(block), &f), TC_OK, "parse %u", (unsigned int)n);
		zassert_equal(f.data_len, n);
		zassert_equal(f.frame_len, 5 + n + 2);
		zassert_equal(f.scid, COMMS_TC_SCID);
		zassert_equal(f.vcid, COMMS_TC_VCID);
		zassert_true(f.bypass);
		zassert_false(f.control);
		zassert_equal(f.data, block + 5);
		if (n > 0) {
			zassert_mem_equal(f.data, data, n);
		}
		/* everything after the FECF is zero fill */
		for (size_t i = f.frame_len; i < COMMS_BLOCK_LEN; i++) {
			zassert_equal(block[i], 0, "fill byte %u at length %u", (unsigned int)i,
				      (unsigned int)n);
		}
	}
}

ZTEST(tc_frame, test_bytes_after_the_frame_are_ignored)
{
	uint8_t block[COMMS_BLOCK_LEN];
	const uint8_t data[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
	struct tc_frame f;

	zassert_ok(tc_frame_build(block, COMMS_TC_SCID, COMMS_TC_VCID, data, sizeof(data)));
	memset(block + 5 + 10 + 2, 0xEE, COMMS_BLOCK_LEN - 17);
	zassert_equal(tc_frame_parse(block, sizeof(block), &f), TC_OK);
	zassert_equal(f.data_len, 10);
}

ZTEST(tc_frame, test_only_type_bd_is_accepted)
{
	uint8_t buf[400];
	const uint8_t data[4] = {1, 2, 3, 4};
	struct tc_frame f;

	/* bypass, control: BD = (1,0) accepted. AD = (0,0), reserved = (0,1), BC = (1,1) all dropped. */
	hand_frame(buf, 0, 1, 0, 0, COMMS_TC_SCID, COMMS_TC_VCID, data, 4);
	zassert_equal(tc_frame_parse(buf, COMMS_BLOCK_LEN, &f), TC_OK);
	hand_frame(buf, 0, 0, 0, 0, COMMS_TC_SCID, COMMS_TC_VCID, data, 4);
	zassert_equal(tc_frame_parse(buf, COMMS_BLOCK_LEN, &f), TC_ERR_TYPE, "Type-AD");
	hand_frame(buf, 0, 0, 1, 0, COMMS_TC_SCID, COMMS_TC_VCID, data, 4);
	zassert_equal(tc_frame_parse(buf, COMMS_BLOCK_LEN, &f), TC_ERR_TYPE, "reserved combination");
	hand_frame(buf, 0, 1, 1, 0, COMMS_TC_SCID, COMMS_TC_VCID, data, 4);
	zassert_equal(tc_frame_parse(buf, COMMS_BLOCK_LEN, &f), TC_ERR_TYPE, "Type-BC");
}

ZTEST(tc_frame, test_header_policy_errors)
{
	uint8_t buf[400];
	const uint8_t data[4] = {1, 2, 3, 4};
	struct tc_frame f;

	hand_frame(buf, 1, 1, 0, 0, COMMS_TC_SCID, COMMS_TC_VCID, data, 4);
	zassert_equal(tc_frame_parse(buf, COMMS_BLOCK_LEN, &f), TC_ERR_VERSION);
	hand_frame(buf, 0, 1, 0, 1, COMMS_TC_SCID, COMMS_TC_VCID, data, 4);
	zassert_equal(tc_frame_parse(buf, COMMS_BLOCK_LEN, &f), TC_ERR_SPARE);
	hand_frame(buf, 0, 1, 0, 0, COMMS_TC_SCID ^ 1, COMMS_TC_VCID, data, 4);
	zassert_equal(tc_frame_parse(buf, COMMS_BLOCK_LEN, &f), TC_ERR_SCID);
	hand_frame(buf, 0, 1, 0, 0, COMMS_TC_SCID, COMMS_TC_VCID + 1, data, 4);
	zassert_equal(tc_frame_parse(buf, COMMS_BLOCK_LEN, &f), TC_ERR_VCID);
}

ZTEST(tc_frame, test_integrity_is_checked_before_policy)
{
	uint8_t buf[400];
	const uint8_t data[4] = {1, 2, 3, 4};
	struct tc_frame f;

	/* wrong SCID and a corrupt FECF: PROTOCOL.md section 5 says FECF and length come first */
	hand_frame(buf, 0, 1, 0, 0, COMMS_TC_SCID ^ 1, COMMS_TC_VCID, data, 4);
	buf[5 + 4] ^= 0x01; /* flip a FECF bit */
	zassert_equal(tc_frame_parse(buf, COMMS_BLOCK_LEN, &f), TC_ERR_FECF);
}

ZTEST(tc_frame, test_length_errors)
{
	uint8_t buf[400] = {0};
	const uint8_t data[4] = {1, 2, 3, 4};
	struct tc_frame f;
	size_t len;

	zassert_equal(tc_frame_parse(buf, 6, &f), TC_ERR_SHORT, "smaller than header + FECF");
	zassert_equal(tc_frame_parse(buf, 0, &f), TC_ERR_SHORT);

	len = GOOD_HAND(buf, data, 4);
	zassert_equal(tc_frame_parse(buf, len - 1, &f), TC_ERR_LEN, "frame longer than the buffer");

	/* length field says 6 bytes: smaller than header + FECF */
	GOOD_HAND(buf, data, 4);
	buf[2] = (uint8_t)(COMMS_TC_VCID << 2);
	buf[3] = 5;
	test_seal_crc(buf, 6);
	zassert_equal(tc_frame_parse(buf, COMMS_BLOCK_LEN, &f), TC_ERR_LEN);

	/* length field larger than the block */
	GOOD_HAND(buf, data, 4);
	buf[2] = (uint8_t)((COMMS_TC_VCID << 2) | 0x03);
	buf[3] = 0xFF;
	zassert_equal(tc_frame_parse(buf, COMMS_BLOCK_LEN, &f), TC_ERR_LEN);
}

ZTEST(tc_frame, test_null_arguments)
{
	uint8_t block[COMMS_BLOCK_LEN] = {0};
	struct tc_frame f;
	const uint8_t data[1] = {1};

	zassert_equal(tc_frame_parse(NULL, COMMS_BLOCK_LEN, &f), TC_ERR_ARG);
	zassert_equal(tc_frame_parse(block, COMMS_BLOCK_LEN, NULL), TC_ERR_ARG);
	zassert_equal(tc_frame_build(NULL, 0, 0, data, 1), -EINVAL);
	zassert_equal(tc_frame_build(block, 0, 0, NULL, 1), -EINVAL);
}

ZTEST(tc_frame, test_build_limits)
{
	uint8_t block[COMMS_BLOCK_LEN];
	uint8_t data[COMMS_BLOCK_LEN] = {0};

	zassert_ok(tc_frame_build(block, 0, 0, data, COMMS_UL_MAX_PACKET_CLEAR), "216 bytes fit");
	zassert_equal(tc_frame_build(block, 0, 0, data, COMMS_UL_MAX_PACKET_CLEAR + 1), -EMSGSIZE);
	zassert_equal(tc_frame_build(block, 0x400, 0, data, 1), -EINVAL, "SCID is 10 bits");
	zassert_equal(tc_frame_build(block, 0, 0x40, data, 1), -EINVAL, "VCID is 6 bits");
	zassert_ok(tc_frame_build(block, 0x3FF, 0x3F, data, 1), "largest SCID and VCID");
	zassert_ok(tc_frame_build(block, 0, 0, NULL, 0), "an empty data field is structurally fine");
}

ZTEST(tc_frame, test_every_single_bit_error_is_rejected)
{
	struct test_rng rng = {.s = 0xBADC0DE5};
	uint8_t block[COMMS_BLOCK_LEN];
	uint8_t flipped[COMMS_BLOCK_LEN];
	uint8_t data[60];
	struct tc_frame f;
	size_t frame_len;

	test_rng_fill(&rng, data, sizeof(data));
	zassert_ok(tc_frame_build(block, COMMS_TC_SCID, COMMS_TC_VCID, data, sizeof(data)));
	frame_len = 5 + sizeof(data) + 2;

	for (size_t byte = 0; byte < frame_len; byte++) {
		for (int bit = 0; bit < 8; bit++) {
			memcpy(flipped, block, sizeof(block));
			flipped[byte] ^= (uint8_t)(1U << bit);
			/* A flip in the length field changes which bytes the CRC covers, so for those two fields
			 * CRC theory does not guarantee rejection. It holds for this vector. Every other single-bit
			 * error is guaranteed by the CRC. */
			zassert_not_equal(tc_frame_parse(flipped, sizeof(flipped), &f), TC_OK,
					  "flip of byte %u bit %d was accepted", (unsigned int)byte, bit);
		}
	}
}

ZTEST(tc_frame, test_random_blocks_never_crash_and_accepted_ones_are_valid)
{
	struct test_rng rng = {.s = 0x0DDBA11};
	uint8_t block[COMMS_BLOCK_LEN];
	struct tc_frame f;
	int accepted = 0;

	for (int i = 0; i < 20000; i++) {
		test_rng_fill(&rng, block, sizeof(block));
		if (tc_frame_parse(block, sizeof(block), &f) == TC_OK) {
			accepted++;
			zassert_equal(ref_crc16(block, f.frame_len - 2),
				      (block[f.frame_len - 2] << 8) | block[f.frame_len - 1]);
		}
	}
	/* random data has about a 1 in 2^16 chance of a good FECF, and then must also pass the header checks */
	zassert_true(accepted <= 1, "random blocks should essentially never be accepted, got %d", accepted);
}

ZTEST(tc_frame, test_status_names)
{
	zassert_str_equal(tc_status_str(TC_OK), "ok");
	zassert_str_equal(tc_status_str(TC_ERR_FECF), "bad FECF");
	zassert_str_equal(tc_status_str(TC_STATUS_COUNT), "unknown");
	for (int s = 0; s < TC_STATUS_COUNT; s++) {
		zassert_true(strlen(tc_status_str((enum tc_status)s)) > 0);
	}
}
