/** @file scheduler.c @ingroup comms @brief Periodic jobs, see scheduler.h */
#include "scheduler.h"

#include <string.h>

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(scheduler, LOG_LEVEL_INF);

void scheduler_init(struct scheduler *s, int64_t now_ms, uint32_t stats_period_ms, struct tlm *tlm,
		    struct downlink *dl, struct can_bridge *bridge, const struct mvp_switches *sw)
{
	memset(&s->stats, 0, sizeof(s->stats));
	s->tlm = tlm;
	s->dl = dl;
	s->bridge = bridge;
	s->sw = sw;
	s->period_ms[JOB_HEARTBEAT] = COMMS_HEARTBEAT_PERIOD_MS;
	s->period_ms[JOB_TLM] = COMMS_TLM_PERIOD_MS;
	s->period_ms[JOB_BEACON] = COMMS_BEACON_PERIOD_MS;
	s->period_ms[JOB_STATS] = stats_period_ms;
	for (int j = 0; j < JOB_COUNT; j++) {
		s->next_ms[j] = now_ms + (j == JOB_HEARTBEAT ? 0 : (int64_t)s->period_ms[j]);
	}
}

static void run_job(struct scheduler *s, enum sched_job job, int64_t now_ms)
{
	struct dl_item item;

	switch (job) {
	case JOB_HEARTBEAT:
		(void)can_bridge_send_heartbeat(s->bridge);
		break;
	case JOB_TLM:
	case JOB_BEACON:
		if (!s->sw->tlm) {
			s->stats.tlm_off++;
			return;
		}
		if (job == JOB_TLM) {
			tlm_build_telemetry(s->tlm, now_ms, &item);
		} else {
			tlm_build_beacon(s->tlm, now_ms, &item);
		}
		(void)downlink_submit(s->dl, &item);
		break;
	default:
		break; /* JOB_STATS: the caller logs */
	}
	s->stats.run[job]++;
}

uint32_t scheduler_step(struct scheduler *s, int64_t now_ms)
{
	uint32_t due = 0;

	for (int j = 0; j < JOB_COUNT; j++) {
		if (now_ms < s->next_ms[j]) {
			continue;
		}
		due |= JOB_BIT(j);
		if (now_ms - s->next_ms[j] >= (int64_t)s->period_ms[j]) {
			s->stats.skipped[j]++;
			s->next_ms[j] = now_ms + (int64_t)s->period_ms[j];
		} else {
			s->next_ms[j] += (int64_t)s->period_ms[j];
		}
		run_job(s, (enum sched_job)j, now_ms);
	}
	return due;
}

uint32_t scheduler_wait_ms(const struct scheduler *s, int64_t now_ms)
{
	int64_t soonest = INT64_MAX;

	for (int j = 0; j < JOB_COUNT; j++) {
		if (s->next_ms[j] < soonest) {
			soonest = s->next_ms[j];
		}
	}
	return soonest <= now_ms ? 0 : (uint32_t)MIN(soonest - now_ms, (int64_t)UINT32_MAX);
}
