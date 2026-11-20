/** @file telemetry.c @ingroup comms @brief Telemetry cache, see telemetry.h */
#include "telemetry.h"

#include <string.h>

#include "common/can_proto.h"

void tlm_cache_init(struct tlm_cache *c)
{
	memset(c->slot, 0, sizeof(c->slot));
	memset(&c->stats, 0, sizeof(c->stats));
}

bool tlm_cache_wants(const struct can_record *rec)
{
	uint8_t src = can_record_src(rec);

	if (can_record_dst(rec) != CAN_BROADCAST) {
		return false;
	}
	/* COMMS's own frames never come back on the bus, and ground is not a board. This guards a test
	 * setup or a miswired bus, so COMMS never caches its own heartbeat as a second copy. */
	if (src == COMMS_ID || src == GROUND_ID) {
		return false;
	}
	switch (can_record_class(rec)) {
	case CLS_HEARTBEAT:
	case CLS_TELEMETRY:
	case CLS_HEALTH:
	case CLS_SOH_CORE:
	case CLS_SOH_CAN_STATS:
		return true;
	default:
		return false;
	}
}

static bool same_key(const struct can_record *a, const struct can_record *b)
{
	return can_record_src(a) == can_record_src(b) && can_record_class(a) == can_record_class(b) &&
	       can_record_op(a) == can_record_op(b);
}

int tlm_cache_on_frame(struct tlm_cache *c, const struct can_frame *frame, int64_t now_ms)
{
	struct can_record rec;
	int free_slot = -1;
	int result = 0;

	if (can_record_from_frame(frame, &rec) != REC_OK || !tlm_cache_wants(&rec)) {
		c->stats.ignored++;
		return 0;
	}

	K_SPINLOCK(&c->lock) {
		for (int i = 0; i < TLM_CACHE_SLOTS; i++) {
			if (!c->slot[i].used) {
				if (free_slot < 0) {
					free_slot = i;
				}
				continue;
			}
			if (same_key(&c->slot[i].rec, &rec)) {
				c->slot[i].rec = rec;
				c->slot[i].t_ms = now_ms;
				c->stats.updated++;
				result = 1;
				break;
			}
		}
		if (!result) {
			if (free_slot < 0) {
				c->stats.overflow++;
			} else {
				c->slot[free_slot].used = true;
				c->slot[free_slot].rec = rec;
				c->slot[free_slot].t_ms = now_ms;
				c->stats.stored++;
				result = 1;
			}
		}
	}
	return result;
}

static bool fresh(int64_t t_ms, int64_t now_ms, uint32_t max_age_ms)
{
	return (now_ms - t_ms) <= (int64_t)max_age_ms;
}

size_t tlm_cache_collect(struct tlm_cache *c, int64_t now_ms, uint32_t max_age_ms, struct can_record *out,
			 size_t max)
{
	size_t n = 0;

	K_SPINLOCK(&c->lock) {
		for (int i = 0; i < TLM_CACHE_SLOTS && n < max; i++) {
			if (c->slot[i].used && fresh(c->slot[i].t_ms, now_ms, max_age_ms)) {
				out[n++] = c->slot[i].rec;
			}
		}
	}
	return n;
}

bool tlm_cache_get(struct tlm_cache *c, uint8_t src, uint8_t cls, uint8_t op, int64_t now_ms,
		   uint32_t max_age_ms, struct can_record *out)
{
	bool found = false;

	K_SPINLOCK(&c->lock) {
		for (int i = 0; i < TLM_CACHE_SLOTS; i++) {
			struct can_record *r = &c->slot[i].rec;

			if (c->slot[i].used && can_record_src(r) == src && can_record_class(r) == cls &&
			    can_record_op(r) == op && fresh(c->slot[i].t_ms, now_ms, max_age_ms)) {
				*out = *r;
				found = true;
				break;
			}
		}
	}
	return found;
}
