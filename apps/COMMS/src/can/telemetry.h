/**
 * @file telemetry.h
 * @ingroup comms
 * @brief Cache of the latest telemetry each board broadcast on the CAN bus
 *
 * The boards broadcast heartbeats, telemetry and health records. COMMS keeps the most recent record for each
 * (source, class, opcode) with the time it arrived, and the scheduler sends them down periodically (app/tlm).
 * Only broadcast frames of the classes in `tlm_cache_wants()` are kept. Commands, answers, frames addressed
 * to one node and COMMS's own frames are not.
 *
 * The CAN thread stores and the scheduler thread reads, so every access holds a spinlock for a copy of a few
 * 12-byte records. The cache is fixed size and never allocates. When it is full a new key is dropped and
 * counted: that means more different records are broadcast than COMMS_TLM_MAX_RECORDS, and the table
 * size or the filter has to be revisited.
 */
#ifndef COMMS_TELEMETRY_H_
#define COMMS_TELEMETRY_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>

#include "can/can_record.h"
#include "comms_config.h"

/** Entries in the cache. One telemetry packet can carry COMMS_TLM_MAX_RECORDS of them, one of which is COMMS's
 *  own record, so this stays below that. */
#define TLM_CACHE_SLOTS (COMMS_TLM_MAX_RECORDS - 1)

struct tlm_cache_stats {
	uint32_t stored;   /**< records stored in a new slot */
	uint32_t updated;  /**< records that replaced an older one with the same key */
	uint32_t ignored;  /**< frames not wanted (wrong class, not broadcast, own frames, not 8-byte extended) */
	uint32_t overflow; /**< wanted records dropped because every slot was in use */
};

struct tlm_cache {
	struct k_spinlock lock;
	struct {
		bool used;
		int64_t t_ms;           /**< when it arrived */
		struct can_record rec;
	} slot[TLM_CACHE_SLOTS];
	struct tlm_cache_stats stats;
};

void tlm_cache_init(struct tlm_cache *c);

/** True if a record of this class from this source is the kind of record the cache keeps. */
bool tlm_cache_wants(const struct can_record *rec);

/**
 * Offer a frame from the CAN queue.
 * @param now_ms uptime in milliseconds
 * @return 1 if it was stored (new or replaced), 0 if ignored or dropped (counted)
 */
int tlm_cache_on_frame(struct tlm_cache *c, const struct can_frame *frame, int64_t now_ms);

/**
 * Copy the cached records that are not older than @p max_age_ms, in slot order, which is the order in which
 * each key first arrived.
 * @return the number of records written to @p out, at most @p max
 */
size_t tlm_cache_collect(struct tlm_cache *c, int64_t now_ms, uint32_t max_age_ms, struct can_record *out,
			 size_t max);

/**
 * Copy the record with this key if it is cached and fresh.
 * @return true if found
 */
bool tlm_cache_get(struct tlm_cache *c, uint8_t src, uint8_t cls, uint8_t op, int64_t now_ms,
		   uint32_t max_age_ms, struct can_record *out);

#endif /* COMMS_TELEMETRY_H_ */
