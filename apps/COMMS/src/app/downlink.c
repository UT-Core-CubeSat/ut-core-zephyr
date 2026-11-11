/** @file downlink.c @ingroup comms @brief Downlink pipeline, see downlink.h */
#include "downlink.h"

#include <errno.h>
#include <string.h>

#include <zephyr/logging/log.h>

#include "link/dl_block.h"

LOG_MODULE_REGISTER(downlink, LOG_LEVEL_INF);

void downlink_init(struct downlink *d, struct k_msgq *q, struct radio_port *port)
{
	memset(&d->stats, 0, sizeof(d->stats));
	sp_seq_init(&d->seq);
	d->q = q;
	d->port = port;
}

int downlink_submit(struct downlink *d, const struct dl_item *item)
{
	if (k_msgq_put(d->q, item, K_NO_WAIT) != 0) {
		d->stats.queue_full++;
		return -ENOMSG;
	}
	d->stats.queued++;
	return 0;
}

int downlink_service_one(struct downlink *d, k_timeout_t timeout)
{
	struct dl_item item;
	uint8_t packet[COMMS_DL_MAX_PACKET];
	uint8_t block[COMMS_BLOCK_LEN];
	int n;

	if (k_msgq_get(d->q, &item, timeout) != 0) {
		return -EAGAIN;
	}

	if (item.len == 0 || item.len > DL_ITEM_MAX_DATA || item.apid >= SP_APID_COUNT) {
		d->stats.build_errors++;
		return 0;
	}

	/* The sequence count is taken only once the packet is known to be valid, so a bad item does not
	 * leave a gap that ground would read as a lost packet. */
	n = sp_build(packet, sizeof(packet), false, false, item.apid, sp_seq_next(&d->seq, item.apid),
		     item.data, item.len);
	if (n < 0 || dl_block_pack(block, packet, (size_t)n) != 0) {
		d->stats.build_errors++;
		return 0;
	}

	if (radio_port_send(d->port, block) != 0) {
		d->stats.send_errors++;
		LOG_WRN("send failed, APID 0x%03x dropped", item.apid);
		return 0;
	}
	d->stats.sent++;
	return 0;
}
