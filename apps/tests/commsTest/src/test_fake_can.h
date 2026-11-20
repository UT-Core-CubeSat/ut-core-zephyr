/**
 * @file test_fake_can.h
 * @brief A fake CAN bus for the host tests: records what the bridge sends, can be made to fail
 *
 * Each test file that includes this gets its own copy of the state (it is static), which is what the
 * suites want.
 */
#ifndef COMMS_TEST_FAKE_CAN_H_
#define COMMS_TEST_FAKE_CAN_H_

#include <string.h>

#include "can/can_bridge.h"

#define FAKE_CAN_MAX 8

struct fake_can {
	struct can_frame frames[FAKE_CAN_MAX]; /**< frames sent successfully */
	int count;                             /**< number of successful sends */
	int attempts;                          /**< all send calls */
	int result;                            /**< what send returns (0 = success) */
};

static struct fake_can fcan;
static atomic_t fcan_counter;

static int fake_can_send(const struct device *dev, const struct can_frame *f)
{
	ARG_UNUSED(dev);
	fcan.attempts++;
	if (fcan.result == 0) {
		if (fcan.count < FAKE_CAN_MAX) {
			fcan.frames[fcan.count] = *f;
		}
		fcan.count++;
	}
	return fcan.result;
}

static const struct can_bridge_ops fake_can_ops = {.send = fake_can_send};

/** Clear the fake bus and set up @p br to use it. The counter starts at 0. */
static inline void fake_can_attach(struct can_bridge *br)
{
	memset(&fcan, 0, sizeof(fcan));
	atomic_set(&fcan_counter, 0);
	can_bridge_init(br, NULL, &fcan_counter);
	br->ops = &fake_can_ops;
}

#endif /* COMMS_TEST_FAKE_CAN_H_ */
