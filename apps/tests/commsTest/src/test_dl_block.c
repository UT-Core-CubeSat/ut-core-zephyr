/** @file test_dl_block.c @brief Tests for link/dl_block (downlink block) */
#include <errno.h>
#include <string.h>
#include <zephyr/ztest.h>

#include "comms_config.h"
#include "link/dl_block.h"
#include "test_util.h"

ZTEST_SUITE(dl_block, NULL, NULL, NULL, NULL, NULL);

ZTEST(dl_block, test_layout_matches_the_spec)
{
	uint8_t block[COMMS_BLOCK_LEN];
	uint8_t packet[10];

	for (size_t i = 0; i < sizeof(packet); i++) {
		packet[i] = (uint8_t)(0xA0 + i);
	}
	zassert_ok(dl_block_pack(block, packet, sizeof(packet)));

	zassert_equal(block[0], 0x00, "len is big-endian");
	zassert_equal(block[1], 0x0A);
	zassert_mem_equal(block + 2, packet, sizeof(packet));
	for (size_t i = 2 + sizeof(packet); i < 221; i++) {
		zassert_equal(block[i], 0, "padding byte %u", (unsigned int)i);
	}
	/* the CRC covers len through the padding: the first 221 bytes. Big-endian in the last two. */
	zassert_equal((block[221] << 8) | block[222], ref_crc16(block, 221));
}

ZTEST(dl_block, test_round_trip_every_length)
{
	struct test_rng rng = {.s = 0x1DEA1DEA};
	uint8_t block[COMMS_BLOCK_LEN];
	uint8_t packet[COMMS_DL_MAX_PACKET];
	const uint8_t *got;
	size_t got_len;

	for (size_t n = COMMS_SP_MIN_PACKET; n <= COMMS_DL_MAX_PACKET; n++) {
		test_rng_fill(&rng, packet, n);
		zassert_ok(dl_block_pack(block, packet, n), "pack %u", (unsigned int)n);
		zassert_equal(dl_block_unpack(block, sizeof(block), &got, &got_len), DL_OK, "unpack %u",
			      (unsigned int)n);
		zassert_equal(got_len, n);
		zassert_equal(got, block + 2);
		zassert_mem_equal(got, packet, n);
	}
}

ZTEST(dl_block, test_pack_limits)
{
	uint8_t block[COMMS_BLOCK_LEN];
	uint8_t packet[COMMS_BLOCK_LEN] = {0};

	zassert_ok(dl_block_pack(block, packet, 7), "smallest Space Packet");
	zassert_ok(dl_block_pack(block, packet, 219), "largest");
	zassert_equal(dl_block_pack(block, packet, 6), -EINVAL, "shorter than a Space Packet");
	zassert_equal(dl_block_pack(block, packet, 0), -EINVAL);
	zassert_equal(dl_block_pack(block, packet, 220), -EMSGSIZE);
	zassert_equal(dl_block_pack(NULL, packet, 10), -EINVAL);
	zassert_equal(dl_block_pack(block, NULL, 10), -EINVAL);
}

ZTEST(dl_block, test_every_single_bit_error_is_rejected)
{
	struct test_rng rng = {.s = 0xABCD1234};
	uint8_t block[COMMS_BLOCK_LEN];
	uint8_t flipped[COMMS_BLOCK_LEN];
	uint8_t packet[100];
	const uint8_t *got;
	size_t got_len;

	test_rng_fill(&rng, packet, sizeof(packet));
	zassert_ok(dl_block_pack(block, packet, sizeof(packet)));

	/* The CRC covers a fixed 221 bytes, so it detects every single-bit error anywhere in the block. */
	for (size_t byte = 0; byte < COMMS_BLOCK_LEN; byte++) {
		for (int bit = 0; bit < 8; bit++) {
			memcpy(flipped, block, sizeof(block));
			flipped[byte] ^= (uint8_t)(1U << bit);
			zassert_not_equal(dl_block_unpack(flipped, sizeof(flipped), &got, &got_len), DL_OK,
					  "flip of byte %u bit %d was accepted", (unsigned int)byte, bit);
		}
	}
}

ZTEST(dl_block, test_a_valid_crc_with_a_bad_length_is_rejected)
{
	uint8_t block[COMMS_BLOCK_LEN] = {0};
	const uint8_t *got;
	size_t got_len;
	static const uint16_t bad[] = {0, 1, 6, 220, 221, 0x00FF, 0x0100, 0xFFFF};

	for (size_t i = 0; i < ARRAY_SIZE(bad); i++) {
		memset(block, 0, sizeof(block));
		block[0] = (uint8_t)(bad[i] >> 8);
		block[1] = (uint8_t)(bad[i] & 0xFF);
		test_seal_crc(block, sizeof(block));
		zassert_equal(dl_block_unpack(block, sizeof(block), &got, &got_len), DL_ERR_LEN,
			      "len %u", bad[i]);
	}
}

ZTEST(dl_block, test_crc_is_checked_before_length)
{
	uint8_t block[COMMS_BLOCK_LEN] = {0};
	const uint8_t *got;
	size_t got_len;

	block[1] = 0xFF; /* a bad length, and no CRC at all */
	zassert_equal(dl_block_unpack(block, sizeof(block), &got, &got_len), DL_ERR_CRC);
}

ZTEST(dl_block, test_wrong_size_and_null_arguments)
{
	uint8_t block[COMMS_BLOCK_LEN + 1] = {0};
	const uint8_t *got;
	size_t got_len;

	zassert_equal(dl_block_unpack(block, COMMS_BLOCK_LEN - 1, &got, &got_len), DL_ERR_SIZE);
	zassert_equal(dl_block_unpack(block, COMMS_BLOCK_LEN + 1, &got, &got_len), DL_ERR_SIZE);
	zassert_equal(dl_block_unpack(block, 0, &got, &got_len), DL_ERR_SIZE);
	zassert_equal(dl_block_unpack(NULL, COMMS_BLOCK_LEN, &got, &got_len), DL_ERR_ARG);
	zassert_equal(dl_block_unpack(block, COMMS_BLOCK_LEN, NULL, &got_len), DL_ERR_ARG);
	zassert_equal(dl_block_unpack(block, COMMS_BLOCK_LEN, &got, NULL), DL_ERR_ARG);
}

ZTEST(dl_block, test_padding_content_is_not_checked)
{
	uint8_t block[COMMS_BLOCK_LEN];
	const uint8_t packet[8] = {1, 2, 3, 4, 5, 6, 7, 8};
	const uint8_t *got;
	size_t got_len;

	zassert_ok(dl_block_pack(block, packet, sizeof(packet)));
	memset(block + 2 + sizeof(packet), 0x5A, 50);
	test_seal_crc(block, sizeof(block));
	zassert_equal(dl_block_unpack(block, sizeof(block), &got, &got_len), DL_OK);
}

ZTEST(dl_block, test_random_blocks_never_crash)
{
	struct test_rng rng = {.s = 0xDEADBEEF};
	uint8_t block[COMMS_BLOCK_LEN];
	const uint8_t *got;
	size_t got_len;
	int accepted = 0;

	for (int i = 0; i < 20000; i++) {
		test_rng_fill(&rng, block, sizeof(block));
		if (dl_block_unpack(block, sizeof(block), &got, &got_len) == DL_OK) {
			accepted++;
			zassert_true(got_len >= 7 && got_len <= 219);
		}
	}
	zassert_true(accepted <= 1, "got %d", accepted);
}

ZTEST(dl_block, test_status_names)
{
	zassert_str_equal(dl_status_str(DL_OK), "ok");
	zassert_str_equal(dl_status_str(DL_STATUS_COUNT), "unknown");
	for (int s = 0; s < DL_STATUS_COUNT; s++) {
		zassert_true(strlen(dl_status_str((enum dl_status)s)) > 0);
	}
}
