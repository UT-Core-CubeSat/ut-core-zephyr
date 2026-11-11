/** @file uplink.c @ingroup comms @brief Uplink pipeline, see uplink.h */
#include "uplink.h"

#include <errno.h>
#include <string.h>

#include <zephyr/logging/log.h>

#include "comms_config.h"

LOG_MODULE_REGISTER(uplink, LOG_LEVEL_INF);

int uplink_handle_block(struct uplink_stats *st, struct router *rt, const uint8_t *block, size_t len,
			struct dl_item *reply)
{
	struct tc_frame frame;
	struct sp_packet pkt;
	enum tc_status ts;
	enum sp_status ss;

	st->blocks++;

	ts = tc_frame_parse(block, len, &frame);
	if (ts != TC_OK) {
		st->tc_drop[ts]++;
		LOG_DBG("frame dropped: %s", tc_status_str(ts));
		return ROUTE_BAD;
	}

	/* Stage 4: SDLS verify and decrypt goes here, between the frame and the packet. */

	ss = sp_parse(frame.data, frame.data_len, &pkt);
	if (ss != SP_OK) {
		st->sp_drop[ss]++;
		LOG_DBG("packet dropped: %s", sp_status_str(ss));
		return ROUTE_BAD;
	}
	if (!pkt.is_tc) {
		st->not_tc++;
		LOG_DBG("packet dropped: not a command packet");
		return ROUTE_BAD;
	}

	st->accepted++;
	return router_dispatch(rt, &pkt, reply);
}

void uplink_init(struct uplink *u, struct k_msgq *in_q, struct downlink *dl, struct router *router)
{
	memset(&u->stats, 0, sizeof(u->stats));
	u->in_q = in_q;
	u->dl = dl;
	u->router = router;
}

int uplink_service_one(struct uplink *u, k_timeout_t timeout)
{
	uint8_t block[COMMS_BLOCK_LEN];
	struct dl_item reply;

	if (k_msgq_get(u->in_q, block, timeout) != 0) {
		return -EAGAIN;
	}
	if (uplink_handle_block(&u->stats, u->router, block, sizeof(block), &reply) == ROUTE_REPLY) {
		downlink_submit(u->dl, &reply);
	}
	return 0;
}
