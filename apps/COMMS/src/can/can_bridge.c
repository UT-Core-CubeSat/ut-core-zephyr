/** @file can_bridge.c @ingroup comms @brief CAN bridge, see can_bridge.h */
#include "can_bridge.h"

#include <errno.h>
#include <string.h>

#include <zephyr/logging/log.h>

#include "comms_config.h"
#include "common/can_proto.h"
#include "link/space_packet.h"

LOG_MODULE_REGISTER(can_bridge, LOG_LEVEL_INF);

static int driver_send(const struct device *dev, const struct can_frame *frame)
{
	return can_send(dev, frame, K_MSEC(COMMS_CAN_SEND_TIMEOUT_MS), NULL, NULL);
}

static const struct can_bridge_ops driver_ops = {.send = driver_send};

void can_bridge_init(struct can_bridge *br, const struct device *dev, atomic_t *tlm_counter)
{
	memset(&br->stats, 0, sizeof(br->stats));
	br->dev = dev;
	br->ops = &driver_ops;
	br->tlm_counter = tlm_counter;
}

int can_bridge_forward(struct can_bridge *br, const struct can_record *rec)
{
	struct can_record r = *rec;
	struct can_frame frame;
	int err;

	/* Ground does not choose its own arbitration priority. */
	r.id = (r.id & ~CAN_PRIO(0x07)) | CAN_PRIO(COMMS_CAN_UPLINK_PRIO);

	if (can_record_to_frame(&r, &frame) != REC_OK) {
		br->stats.bad_record++;
		return -EINVAL;
	}
	err = br->ops->send(br->dev, &frame);
	if (err) {
		br->stats.send_errors++;
		LOG_WRN("CAN send failed: %d", err);
		return err;
	}
	br->stats.forwarded++;
	return 0;
}

int can_bridge_add_ground_filter(const struct device *can_dev, struct k_msgq *rxq)
{
	const struct can_filter to_ground = {
		.id = CAN_DST(GROUND_ID),
		.mask = CAN_DST_MASK_29,
		.flags = CAN_FILTER_IDE,
	};
	int id = can_add_rx_filter_msgq(can_dev, rxq, &to_ground);

	return id < 0 ? id : 0;
}

int can_bridge_on_frame(struct can_bridge *br, const struct can_frame *frame, struct dl_item *out)
{
	struct can_record rec;
	uint32_t counter;

	if (can_record_from_frame(frame, &rec) != REC_OK) {
		br->stats.ignored_frame++;
		return 0;
	}
	if (can_record_dst(&rec) != GROUND_ID) {
		br->stats.ignored_dst++;
		return 0;
	}

	counter = (uint32_t)atomic_inc(br->tlm_counter);
	out->apid = APID_DL_TLM;
	out->len = 4 + CAN_RECORD_LEN;
	out->data[0] = (uint8_t)(counter >> 24);
	out->data[1] = (uint8_t)(counter >> 16);
	out->data[2] = (uint8_t)(counter >> 8);
	out->data[3] = (uint8_t)counter;
	can_record_pack(&out->data[4], &rec);
	br->stats.replies++;
	return 1;
}
