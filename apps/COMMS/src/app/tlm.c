/** @file tlm.c @ingroup comms @brief Telemetry and beacon packets, see tlm.h */
#include "tlm.h"

#include <string.h>

#include "common/can_proto.h"
#include "link/space_packet.h"

/** CAN priority written into the health record's ID. Same as the heartbeat. */
#define HEALTH_PRIO COMMS_CAN_UPLINK_PRIO

void tlm_init(struct tlm *t, struct tlm_cache *cache, atomic_t *counter)
{
	memset(&t->stats, 0, sizeof(t->stats));
	t->cache = cache;
	t->counter = counter;
}

void tlm_health_record(struct can_record *rec, uint32_t uptime_s, uint8_t cached)
{
	rec->id = CAN_ID_FULL(HEALTH_PRIO, COMMS_ID, CAN_BROADCAST, CLS_HEALTH);
	rec->data[0] = COMMS_ID;
	rec->data[1] = OP_HEARTBEAT;
	rec->data[2] = (uint8_t)(uptime_s >> 24);
	rec->data[3] = (uint8_t)(uptime_s >> 16);
	rec->data[4] = (uint8_t)(uptime_s >> 8);
	rec->data[5] = (uint8_t)uptime_s;
	rec->data[6] = cached;
	rec->data[7] = 0;
}

/* Header (counter) then the records. Returns the data field length. */
static uint16_t fill(struct tlm *t, uint16_t apid, struct dl_item *out, const struct can_record *recs,
		     size_t n)
{
	uint32_t counter = (uint32_t)atomic_inc(t->counter);

	out->apid = apid;
	out->data[0] = (uint8_t)(counter >> 24);
	out->data[1] = (uint8_t)(counter >> 16);
	out->data[2] = (uint8_t)(counter >> 8);
	out->data[3] = (uint8_t)counter;
	for (size_t i = 0; i < n; i++) {
		can_record_pack(&out->data[4 + i * CAN_RECORD_LEN], &recs[i]);
	}
	out->len = (uint16_t)(4 + n * CAN_RECORD_LEN);
	return out->len;
}

void tlm_build_telemetry(struct tlm *t, int64_t now_ms, struct dl_item *out)
{
	struct can_record recs[COMMS_TLM_MAX_RECORDS];
	size_t n = tlm_cache_collect(t->cache, now_ms, COMMS_TLM_MAX_AGE_MS, &recs[1], COMMS_TLM_MAX_RECORDS - 1);

	tlm_health_record(&recs[0], (uint32_t)(now_ms / 1000), (uint8_t)n);
	fill(t, APID_DL_TLM, out, recs, n + 1);
	t->stats.telemetry++;
}

void tlm_build_beacon(struct tlm *t, int64_t now_ms, struct dl_item *out)
{
	struct can_record recs[COMMS_BEACON_MAX_RECORDS];
	size_t n = 1;
	static const uint8_t boards[] = {CDH_ID, EPS_ID};

	tlm_health_record(&recs[0], (uint32_t)(now_ms / 1000), 0);
	for (size_t i = 0; i < ARRAY_SIZE(boards) && n < COMMS_BEACON_MAX_RECORDS; i++) {
		if (tlm_cache_get(t->cache, boards[i], CLS_HEARTBEAT, OP_HEARTBEAT, now_ms, COMMS_TLM_MAX_AGE_MS,
				  &recs[n])) {
			n++;
		}
	}
	/* The beacon's health record reports how many heartbeats it carries, as the other records' count. */
	recs[0].data[6] = (uint8_t)(n - 1);
	fill(t, APID_DL_BEACON, out, recs, n);
	t->stats.beacons++;
}
