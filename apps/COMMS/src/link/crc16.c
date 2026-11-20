/**
 * @file crc16.c
 * @ingroup comms
 * @brief CRC-16/CCITT-FALSE, see crc16.h
 */
#include "crc16.h"

#include <zephyr/sys/crc.h>

uint16_t comms_crc16(const uint8_t *buf, size_t len)
{
	/* crc16_itu_t is the non-reflected CCITT polynomial. crc16_ccitt() is NOT: it reflects. */
	return crc16_itu_t(COMMS_CRC16_INIT, buf, len);
}
