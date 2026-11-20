/**
 * @file crc16.h
 * @ingroup comms
 * @brief CRC-16 used by every layer of the COMMS link (PROTOCOL.md section 2)
 *
 * CRC-16/CCITT-FALSE: polynomial 0x1021, initial value 0xFFFF, no input or output reflection, no final XOR.
 * Check value for the ASCII bytes "123456789" is 0x29B1. It is the same function as the CCSDS TC frame
 * error control field (FECF), so the uplink FECF and the downlink block CRC are one routine.
 *
 * Do not swap this for Zephyr's crc16_ccitt(): that one is the reflected variant and gives different results.
 */
#ifndef COMMS_CRC16_H_
#define COMMS_CRC16_H_

#include <stddef.h>
#include <stdint.h>

/** Initial value of the CRC register. */
#define COMMS_CRC16_INIT 0xFFFF

/**
 * @brief CRC-16/CCITT-FALSE of a buffer.
 * @param buf data, may be NULL only if len is 0
 * @param len number of bytes
 * @return the CRC. The CRC of zero bytes is COMMS_CRC16_INIT.
 */
uint16_t comms_crc16(const uint8_t *buf, size_t len);

#endif /* COMMS_CRC16_H_ */
