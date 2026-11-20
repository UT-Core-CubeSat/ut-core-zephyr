/**
 * @file scheduler.h
 * @ingroup comms
 * @brief Periodic jobs of COMMS: CAN heartbeat, telemetry, beacon, statistics log
 *
 * One thread (PRIO_SCHED, the lowest) calls `scheduler_step()` with the uptime in milliseconds. Each job has
 * a period. A due job runs, and the next due time moves forward by one period from the previous due time (so
 * the cadence does not drift). If a job is more than a whole period late, the missed runs are not made up
 * (a burst of telemetry after a stall would only fill the downlink queue): the next due time becomes
 * now + period and `skipped` counts it.
 *
 * Telemetry and the beacon go out only while the TLM test switch is on (`struct mvp_switches::tlm`, which is
 * the router's, so a runtime change shows here). They are queued with `downlink_submit()`, which never
 * waits, and a full queue is counted there. The heartbeat is not a test, it is always sent (CDH watches for
 * it). The statistics job only reports that it is due: the caller logs.
 *
 * The first heartbeat is due immediately, the other jobs one period after init.
 */
#ifndef COMMS_SCHEDULER_H_
#define COMMS_SCHEDULER_H_

#include <stdint.h>

#include "app/downlink.h"
#include "app/router.h"
#include "app/tlm.h"
#include "can/can_bridge.h"

enum sched_job {
	JOB_HEARTBEAT,
	JOB_TLM,
	JOB_BEACON,
	JOB_STATS,
	JOB_COUNT
};

/** Bit of a job in the mask returned by scheduler_step(). */
#define JOB_BIT(j) (1U << (j))

struct scheduler_stats {
	uint32_t run[JOB_COUNT];     /**< times each job was due and handled */
	uint32_t skipped[JOB_COUNT]; /**< runs not made up after a stall of more than one period */
	uint32_t tlm_off;            /**< telemetry or beacon runs skipped because the switch is off */
};

struct scheduler {
	struct scheduler_stats stats;
	uint32_t period_ms[JOB_COUNT];
	int64_t next_ms[JOB_COUNT];
	struct tlm *tlm;
	struct downlink *dl;
	struct can_bridge *bridge;
	const struct mvp_switches *sw;
};

/** Set the periods from comms_config.h (COMMS_*_PERIOD_MS, and STATS_LOG_INTERVAL_MS from the caller). */
void scheduler_init(struct scheduler *s, int64_t now_ms, uint32_t stats_period_ms, struct tlm *tlm,
		    struct downlink *dl, struct can_bridge *bridge, const struct mvp_switches *sw);

/**
 * Run whatever is due at @p now_ms.
 * @return the mask of jobs that were due (JOB_BIT), including JOB_STATS, which the caller handles
 */
uint32_t scheduler_step(struct scheduler *s, int64_t now_ms);

/** Milliseconds until the next job is due, 0 if one is due now. For the thread's sleep. */
uint32_t scheduler_wait_ms(const struct scheduler *s, int64_t now_ms);

#endif /* COMMS_SCHEDULER_H_ */
