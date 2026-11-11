/**
 * @file downlink.h
 * @ingroup comms
 * @brief Downlink pipeline: queue of packets, Space Packet, block, radio
 *
 * Anything that wants to send (uplink replies now, telemetry and CAN replies later) calls
 * `downlink_submit()`, which only queues. `downlink_service_one()`, run by the radio_tx thread, builds the
 * packet with the next sequence count for its APID, packs the block and sends it. The sequence counters
 * are therefore touched by that one thread only, so they need no lock.
 */
#ifndef COMMS_DOWNLINK_H_
#define COMMS_DOWNLINK_H_

#include <stdint.h>

#include <zephyr/kernel.h>

#include "app/downlink_item.h"
#include "app/radio_port.h"
#include "link/space_packet.h"

struct downlink_stats {
	uint32_t queued;      /**< items accepted by downlink_submit() */
	uint32_t queue_full;  /**< items dropped because the queue was full */
	uint32_t build_errors; /**< items that could not be framed (bad length) */
	uint32_t sent;        /**< blocks handed to the radio successfully */
	uint32_t send_errors; /**< blocks the radio refused or failed */
};

struct downlink {
	struct sp_seq seq;      /**< 4 KB, owned by the transmit thread */
	struct k_msgq *q;       /**< item size sizeof(struct dl_item) */
	struct radio_port *port;
	struct downlink_stats stats;
};

void downlink_init(struct downlink *d, struct k_msgq *q, struct radio_port *port);

/** Queue an item without waiting. Returns 0, or -ENOMSG if the queue is full (counted). */
int downlink_submit(struct downlink *d, const struct dl_item *item);

/**
 * Take one item from the queue and transmit it.
 * @return 0 if an item was taken (even if framing or sending it failed, which is counted),
 *         -EAGAIN if none arrived within @p timeout
 */
int downlink_service_one(struct downlink *d, k_timeout_t timeout);

#endif /* COMMS_DOWNLINK_H_ */
