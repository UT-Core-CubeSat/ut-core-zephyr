/**
 * @file tlm.h
 * @ingroup comms
 * @brief Periodic downlink packets: telemetry (APID 0x020) and beacon (APID 0x021)
 *
 * Both have the data field `tlm_counter` (4 bytes, big-endian) then N x 12-byte records (PROTOCOL.md section
 * 14). The counter is shared with the CAN answers that `can_bridge` sends on 0x020, so ground sees one
 * counter for APID 0x020 and a gap in it means a lost telemetry packet. The beacon draws from the same
 * counter, so a gap there is a lost beacon as well.
 *
 * **COMMS health record** (first record of every telemetry packet and beacon, so they are never empty
 * and ground can check them with no other board on the bus): class CLS_HEALTH, source COMMS_ID, destination
 * broadcast, data `{COMMS_ID, OP_HEARTBEAT, uptime s (4, big-endian), records that follow (1), 0}`.
 *
 * **Telemetry** is the health record followed by every fresh record in the cache, up to COMMS_TLM_MAX_RECORDS.
 * **Beacon** is the health record plus, if fresh, the CDH and EPS heartbeats: the smallest set that says
 * whether the satellite's core boards are alive (PLACEHOLDER, open item 3).
 *
 * Called by the scheduler thread only. Building does not touch the downlink queue: the caller submits.
 */
#ifndef COMMS_TLM_H_
#define COMMS_TLM_H_

#include <stdint.h>

#include <zephyr/sys/atomic.h>

#include "app/downlink_item.h"
#include "can/telemetry.h"

struct tlm_stats {
	uint32_t telemetry; /**< telemetry packets built */
	uint32_t beacons;   /**< beacons built */
};

struct tlm {
	struct tlm_cache *cache;
	atomic_t *counter; /**< next `tlm_counter` value, shared with can_bridge */
	struct tlm_stats stats;
};

void tlm_init(struct tlm *t, struct tlm_cache *cache, atomic_t *counter);

/** Fill @p rec with the COMMS health record. */
void tlm_health_record(struct can_record *rec, uint32_t uptime_s, uint8_t cached);

/** Build a telemetry packet item for @p now_ms (uptime). Uses one `tlm_counter` value. Always 1 or more records. */
void tlm_build_telemetry(struct tlm *t, int64_t now_ms, struct dl_item *out);

/** Build a beacon item. Uses one `tlm_counter` value. 1 to COMMS_BEACON_MAX_RECORDS records. */
void tlm_build_beacon(struct tlm *t, int64_t now_ms, struct dl_item *out);

#endif /* COMMS_TLM_H_ */
