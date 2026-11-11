/**
 * @file uplink.h
 * @ingroup comms
 * @brief Uplink pipeline: radio block, TC frame, Space Packet, router
 *
 * `uplink_handle_block()` is the logic and has no threads or queues, so host tests call it directly.
 * `uplink_service_one()` is what the uplink thread runs. Stage 4 inserts SDLS between the frame and the
 * packet: `struct tc_frame` already returns the data field and its length.
 */
#ifndef COMMS_UPLINK_H_
#define COMMS_UPLINK_H_

#include <stddef.h>
#include <stdint.h>

#include <zephyr/kernel.h>

#include "app/downlink.h"
#include "app/router.h"
#include "link/space_packet.h"
#include "link/tc_frame.h"

struct uplink_stats {
	uint32_t blocks;                    /**< blocks handed to uplink_handle_block() */
	uint32_t accepted;                  /**< blocks that reached the router */
	uint32_t tc_drop[TC_STATUS_COUNT];  /**< frame-layer drops, indexed by enum tc_status */
	uint32_t sp_drop[SP_STATUS_COUNT];  /**< packet-layer drops, indexed by enum sp_status */
	uint32_t not_tc;                    /**< packets whose type bit says telemetry, not command */
};

/**
 * Process one received block.
 *
 * @return ROUTE_REPLY (reply filled), ROUTE_DONE, or a negative value if the block was dropped. Every
 *         drop is counted in @p st and logged.
 */
int uplink_handle_block(struct uplink_stats *st, struct router *rt, const uint8_t *block, size_t len,
			struct dl_item *reply);

/** State of the uplink thread. */
struct uplink {
	struct uplink_stats stats;
	struct k_msgq *in_q;       /**< blocks from radio_port, item size COMMS_BLOCK_LEN */
	struct downlink *dl;       /**< where replies go */
	struct router *router;     /**< dispatch by APID, with its counters */
};

void uplink_init(struct uplink *u, struct k_msgq *in_q, struct downlink *dl, struct router *router);

/**
 * Take one block from the queue, handle it, queue any reply.
 *
 * @return 0 if a block was processed, -EAGAIN if none arrived within @p timeout
 */
int uplink_service_one(struct uplink *u, k_timeout_t timeout);

#endif /* COMMS_UPLINK_H_ */
