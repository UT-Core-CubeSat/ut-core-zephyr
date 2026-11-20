/**
 * @file main.c
 * @brief Host tests for the COMMS firmware (native_sim)
 *
 * One suite per layer, in its own file, registered by the stage that adds the layer. This file holds only
 * the environment check. See apps/COMMS/PLANNED.md section 2 for the stage list.
 */
#include <zephyr/device.h>
#include <zephyr/ztest.h>

#include <ut/drivers/rfm26w.h>

#include "comms_config.h"
#include "rfm26w_emul.h"

#define RADIO_NODE DT_NODELABEL(rfm26w0)

/* The same check main.c in the firmware makes: protocol constant and radio node agree. */
BUILD_ASSERT(DT_PROP(RADIO_NODE, block_length) == COMMS_BLOCK_LEN, "block length mismatch");

static const struct device *const radio = DEVICE_DT_GET(RADIO_NODE);

ZTEST_SUITE(environment, NULL, NULL, NULL, NULL, NULL);

ZTEST(environment, test_protocol_constants_are_consistent)
{
	zassert_equal(COMMS_BLOCK_LEN, 223);
	zassert_equal(COMMS_UL_MAX_PACKET_CLEAR, 216, "PROTOCOL.md: 223 - 5 - 2");
	zassert_equal(COMMS_DL_MAX_PACKET, 219, "PROTOCOL.md: 223 - 2 - 2");
	zassert_equal(COMMS_SP_MIN_PACKET, 7);
	zassert_true(COMMS_TC_SCID <= 0x3FF, "spacecraft ID is 10 bits");
	zassert_true(COMMS_TC_VCID <= 0x3F, "virtual channel ID is 6 bits");
}

/* The radio driver and the software model of the radio are linked and work together. This is what the
 * later integration tests build on. */
ZTEST(environment, test_radio_driver_and_model_are_present)
{
	struct rfm26w_info info;

	zassert_true(device_is_ready(radio));
	(void)rfm26w_shutdown(radio);
	zassert_ok(rfm26w_start(radio));
	zassert_ok(rfm26w_get_info(radio, &info));
	zassert_equal(info.block_len, COMMS_BLOCK_LEN);
	zassert_ok(rfm26w_shutdown(radio));
	zassert_equal(RFM26W_EMUL_DATA()->bad_cmds, 0);
}
