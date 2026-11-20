/**
 * @file downlink_item.h
 * @ingroup comms
 * @brief One packet waiting to go down: APID and data field
 *
 * The Space Packet header, the callsign secondary header and the block framing are added by the downlink module, at send time, so the
 * sequence counter is only ever touched by the transmit thread.
 */
#ifndef COMMS_DOWNLINK_ITEM_H_
#define COMMS_DOWNLINK_ITEM_H_

#include <stdint.h>

#include "comms_config.h"

/** Largest data field an item may carry: 219 - 6 = 213, less the callsign secondary header (7) that the
 *  downlink module puts in front of it when COMMS_CALLSIGN_IN_PACKETS is 1, so 206. */
#define DL_ITEM_MAX_DATA (COMMS_DL_MAX_PACKET - COMMS_SP_HDR_LEN - COMMS_DL_SEC_HDR_LEN)

struct dl_item {
	uint16_t apid;
	uint16_t len;
	uint8_t data[DL_ITEM_MAX_DATA];
};

#endif /* COMMS_DOWNLINK_ITEM_H_ */
