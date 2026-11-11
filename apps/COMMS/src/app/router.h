/**
 * @file router.h
 * @ingroup comms
 * @brief Dispatch an uplink Space Packet by APID
 *
 * Handled APIDs: 0x0F0 (ping, answered here) and 0x010 (command: a CAN record that must pass the uplink
 * policy and is then put on the CAN bus; the board's answer comes back later through the CAN bridge).
 * Everything else is counted and dropped. 0x030 (CFDP) comes in stage 5.
 *
 * Each test can be switched off on its own (`struct mvp_switches`). A switched-off APID is counted in
 * `disabled` and dropped, as if the board did not know it.
 */
#ifndef COMMS_ROUTER_H_
#define COMMS_ROUTER_H_

#include <stdbool.h>
#include <stdint.h>

#include "app/downlink_item.h"
#include "can/can_bridge.h"
#include "can/uplink_policy.h"
#include "link/space_packet.h"

/** Per-test switches (Kconfig COMMS_MVP_*, and settable at runtime). */
struct mvp_switches {
	bool ping;    /**< APID 0x0F0 */
	bool can_cmd; /**< APID 0x010 */
};

struct router_stats {
	uint32_t ping;                                /**< ping requests answered */
	uint32_t bad_ping;                            /**< ping requests with an invalid data field length */
	uint32_t unknown;                             /**< APIDs nobody handles, including 0x7FF */
	uint32_t disabled;                            /**< packets for a switched-off test */
	uint32_t cmd_forwarded;                       /**< records put on the CAN bus */
	uint32_t cmd_denied[POLICY_RESULT_COUNT];     /**< records refused by the policy, by reason */
	uint32_t cmd_bad_record;                      /**< command packets that are not exactly one valid record */
	uint32_t cmd_can_error;                       /**< records the bridge could not send */
};

struct router {
	struct router_stats stats;
	struct mvp_switches sw;
	struct can_bridge *bridge;
};

/** Result of router_dispatch(). */
enum route_result {
	ROUTE_REPLY = 1,    /**< handled, @p reply is filled and must be sent down */
	ROUTE_DONE = 0,     /**< handled, nothing to send now */
	ROUTE_UNKNOWN = -1, /**< no handler for this APID, or its test is off. Counted and dropped */
	ROUTE_BAD = -2,     /**< handler rejected the packet. Counted and dropped */
};

void router_init(struct router *r, struct can_bridge *bridge, struct mvp_switches sw);

/**
 * Hand a parsed packet to its handler.
 *
 * @param r     state and counters, updated here
 * @param pkt   parsed uplink packet
 * @param reply filled only when ROUTE_REPLY is returned
 */
int router_dispatch(struct router *r, const struct sp_packet *pkt, struct dl_item *reply);

#endif /* COMMS_ROUTER_H_ */
