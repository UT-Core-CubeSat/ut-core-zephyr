/**
 * @file can_bridge.h
 * @ingroup comms
 * @brief Between the radio side and the CAN bus
 *
 * Ground to bus: `can_bridge_forward()` puts a record that already passed the uplink policy on the bus,
 * with the priority forced to COMMS_CAN_UPLINK_PRIO.
 * Bus to ground: `can_bridge_on_frame()` takes frames addressed to GROUND_ID (the replies of the boards) and
 * turns each into a telemetry packet item (APID 0x020): `tlm_counter` (4 bytes, big-endian) then the 12-byte
 * record (PROTOCOL.md section 14). The counter is shared with the periodic telemetry of stage 3b, so it is
 * an `atomic_t` owned by the caller.
 *
 * The CAN send goes through `struct can_bridge_ops`, so host tests can replace it. Not thread-safe by itself:
 * `forward` is called from the uplink thread and `on_frame` from the CAN thread, and they touch separate
 * counters.
 */
#ifndef COMMS_CAN_BRIDGE_H_
#define COMMS_CAN_BRIDGE_H_

#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

#include "app/downlink_item.h"
#include "can/can_record.h"

struct can_bridge_ops {
	/** Send one frame. Returns 0 or a negative errno. */
	int (*send)(const struct device *dev, const struct can_frame *frame);
};

struct can_bridge_stats {
	uint32_t forwarded;     /**< records put on the bus */
	uint32_t bad_record;    /**< records that could not be turned into a frame */
	uint32_t send_errors;   /**< CAN sends that failed */
	uint32_t replies;       /**< frames from the bus turned into a downlink item */
	uint32_t ignored_dst;   /**< frames not addressed to GROUND_ID */
	uint32_t ignored_frame; /**< frames that are not classic 8-byte extended data frames, or have a bad ID */
};

struct can_bridge {
	const struct device *dev;
	const struct can_bridge_ops *ops;
	atomic_t *tlm_counter;  /**< next `tlm_counter` value, shared with telemetry */
	struct can_bridge_stats stats;
};

/** Set up with the driver-backed ops (`can_send` with COMMS_CAN_SEND_TIMEOUT_MS). Does not touch the bus. */
void can_bridge_init(struct can_bridge *br, const struct device *dev, atomic_t *tlm_counter);

/**
 * Put @p rec on the bus. The record must already have passed policy_check(). The priority bits are replaced.
 * @return 0, -EINVAL if the record is not a valid CAN ID, or the send error (counted)
 */
int can_bridge_forward(struct can_bridge *br, const struct can_record *rec);

/**
 * Install the RX filter for frames addressed to GROUND_ID, into the same queue as the other filters.
 * can_setup() installs only this node and broadcast.
 * @return 0 or the negative filter id error
 */
int can_bridge_add_ground_filter(const struct device *can_dev, struct k_msgq *rxq);

/**
 * Handle a frame taken from the CAN queue.
 * @return 1 if @p out is filled and must be sent down, 0 if the frame was ignored (counted)
 */
int can_bridge_on_frame(struct can_bridge *br, const struct can_frame *frame, struct dl_item *out);

#endif /* COMMS_CAN_BRIDGE_H_ */
