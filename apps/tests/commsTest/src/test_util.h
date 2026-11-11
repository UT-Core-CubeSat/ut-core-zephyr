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
#include <string.h>

#include <zephyr/kernel.h>

#include "comms_config.h"
#include "link/space_packet.h"
#include "link/tc_frame.h"

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

/** Wait up to `ms` milliseconds for `cond` (needs CONFIG_SYS_CLOCK_TICKS_PER_SEC=10000 on native_sim). */
#define WAIT_UNTIL(cond, ms)                                                                       \
	do {                                                                                       \
		for (int _i = 0; !(cond) && _i < (ms); _i++) {                                     \
			k_msleep(1);                                                               \
		}                                                                                  \
	} while (0)

/** Uplink Space Packet (command type) with the given data field, into `pkt`. Returns its length. */
static inline size_t test_make_ul_packet(uint8_t *pkt, size_t cap, uint16_t apid, const uint8_t *data,
					 size_t len)
{
	return (size_t)sp_build(pkt, cap, true, false, apid, 0, data, len);
}

/** Uplink block carrying a ping request: `ping_id` big-endian, then `n` payload bytes. */
static inline void test_make_ping_block(uint8_t *block, uint32_t id, const uint8_t *payload, size_t n)
{
	uint8_t data[4 + 200];
	uint8_t pkt[COMMS_UL_MAX_PACKET_CLEAR];
	size_t plen;

	data[0] = (uint8_t)(id >> 24);
	data[1] = (uint8_t)(id >> 16);
	data[2] = (uint8_t)(id >> 8);
	data[3] = (uint8_t)id;
	if (n) {
		memcpy(&data[4], payload, n);
	}
	plen = test_make_ul_packet(pkt, sizeof(pkt), APID_UL_PING, data, 4 + n);
	(void)tc_frame_build(block, COMMS_TC_SCID, COMMS_TC_VCID, pkt, plen);
}

#endif /* COMMS_TEST_UTIL_H_ */
