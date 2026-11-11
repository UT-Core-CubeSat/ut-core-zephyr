/** @file router.c @ingroup comms @brief APID dispatch, see router.h */
#include "router.h"

#include <string.h>

#include "app/ping.h"
#include "can/can_record.h"

void router_init(struct router *r, struct can_bridge *bridge, struct mvp_switches sw)
{
	memset(&r->stats, 0, sizeof(r->stats));
	r->sw = sw;
	r->bridge = bridge;
}

static int handle_ping(struct router *r, const struct sp_packet *pkt, struct dl_item *reply)
{
	int n = ping_make_reply(pkt->data, pkt->data_len, reply->data, sizeof(reply->data));

	if (n < 0) {
		r->stats.bad_ping++;
		return ROUTE_BAD;
	}
	reply->apid = APID_DL_PING;
	reply->len = (uint16_t)n;
	r->stats.ping++;
	return ROUTE_REPLY;
}

static int handle_command(struct router *r, const struct sp_packet *pkt)
{
	struct can_record rec;
	enum policy_result pr;

	if (can_record_unpack(pkt->data, pkt->data_len, &rec) != REC_OK) {
		r->stats.cmd_bad_record++;
		return ROUTE_BAD;
	}
	pr = policy_check(&rec);
	if (pr != POLICY_OK) {
		r->stats.cmd_denied[pr]++;
		return ROUTE_BAD;
	}
	if (can_bridge_forward(r->bridge, &rec) != 0) {
		r->stats.cmd_can_error++;
		return ROUTE_BAD;
	}
	r->stats.cmd_forwarded++;
	return ROUTE_DONE;
}

int router_dispatch(struct router *r, const struct sp_packet *pkt, struct dl_item *reply)
{
	switch (pkt->apid) {
	case APID_UL_PING:
		if (!r->sw.ping) {
			r->stats.disabled++;
			return ROUTE_UNKNOWN;
		}
		return handle_ping(r, pkt, reply);
	case APID_UL_CMD:
		if (!r->sw.can_cmd) {
			r->stats.disabled++;
			return ROUTE_UNKNOWN;
		}
		return handle_command(r, pkt);
	default:
		r->stats.unknown++;
		return ROUTE_UNKNOWN;
	}
}
