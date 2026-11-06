/**
 * @file test_util.h
 * @brief Helpers shared by the host tests
 *
 * The reference CRC here is a separate bit-by-bit implementation, so the tests do not check the firmware's
 * CRC with the firmware's CRC.
 */
#ifndef COMMS_TEST_UTIL_H_
#define COMMS_TEST_UTIL_H_

#include <stddef.h>
#include <stdint.h>

/** CRC-16/CCITT-FALSE, bit by bit: poly 0x1021, init 0xFFFF, no reflection, no final XOR. */
static inline uint16_t ref_crc16(const uint8_t *buf, size_t len)
{
	uint16_t crc = 0xFFFF;

	for (size_t i = 0; i < len; i++) {
		crc ^= (uint16_t)(buf[i] << 8);
		for (int b = 0; b < 8; b++) {
			crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
		}
	}
	return crc;
}

/** Small deterministic generator (xorshift32), so every run of a test sees the same data. */
struct test_rng {
	uint32_t s;
};

static inline uint32_t test_rng_next(struct test_rng *r)
{
	r->s ^= r->s << 13;
	r->s ^= r->s >> 17;
	r->s ^= r->s << 5;
	return r->s;
}

static inline void test_rng_fill(struct test_rng *r, uint8_t *buf, size_t len)
{
	for (size_t i = 0; i < len; i++) {
		buf[i] = (uint8_t)test_rng_next(r);
	}
}

/** Put a CRC-16 over buf[0 .. len-3] into its last two bytes, big-endian. */
static inline void test_seal_crc(uint8_t *buf, size_t len)
{
	uint16_t c = ref_crc16(buf, len - 2);

	buf[len - 2] = (uint8_t)(c >> 8);
	buf[len - 1] = (uint8_t)(c & 0xFF);
}

#endif /* COMMS_TEST_UTIL_H_ */
