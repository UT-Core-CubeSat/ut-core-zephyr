/**
 * @file callsign.h
 * @ingroup comms
 * @brief Callsign secondary header of a downlink Space Packet (PROTOCOL.md section 6)
 *
 * 7 bytes: the callsign as 6 ASCII characters, space padded on the right, then the SSID byte. It sits at the
 * start of the packet data field, with the Space Packet secondary header flag set.
 */
#ifndef COMMS_CALLSIGN_H_
#define COMMS_CALLSIGN_H_

#include <stdint.h>

#include "comms_config.h"

/**
 * @brief Write the 7-byte secondary header.
 * @param out  room for COMMS_SEC_HDR_LEN bytes
 * @param call 1 to 6 characters, each A-Z or 0-9 (amateur callsigns are upper case)
 * @param ssid the SSID byte
 * @return 0, or -EINVAL for a NULL pointer, an empty or too long callsign, or a character outside A-Z, 0-9
 */
int callsign_pack(uint8_t *out, const char *call, uint8_t ssid);

#endif /* COMMS_CALLSIGN_H_ */
