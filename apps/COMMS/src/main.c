/**
 * @defgroup comms COMMS
 * @ingroup apps
 * @brief Communications firmware
 *
 * @include{doc} ./apps/COMMS/README.md
 */

/**
 * @file main.c
 * @ingroup comms
 * @brief Entry point for COMMS firmware: init order, threads and queues
 *
 * UT-CORE COMMS: the satellite's link to the ground.
 *
 * Hardware: STM32U5 microcontroller, Zephyr RTOS.
 * Radio:    UHF RFM26W, driver in drivers/rfm26w, used only through ut/drivers/rfm26w.h.
 * Bus:      FDCAN to the shared CAN bus, 29-bit extended IDs (common/can_proto.h).
 *
 * This file is the front door: it says what runs, in what order and at what priority, and each
 * thread body is a few lines that move data between queues and call a module. It holds no protocol
 * logic. Parsing, framing, routing and policy live in the modules under src/ (link/, can/, and
 * app/, sec/, cfdp/ as the stages add them). Those take buffers, return codes, have no threads and
 * no globals, and are unit-tested on the host (apps/tests/commsTest). Constants are in
 * comms_config.h (protocol) and board_config.h (hardware).
 *
 * Thread overview (highest to lowest priority; the radio driver's own work queue thread, priority
 * CONFIG_RFM26W_WORKQ_PRIORITY, sits above all of these):
 *   PRIO 0  watchdog     kick the hardware watchdog, never starve it          (stage 3, todo)
 *   PRIO 1  can_rx       drain the CAN hardware FIFO into a queue             (stage 3, todo)
 *   PRIO 2  can_proc     decode CAN frames, replies to ground, telemetry      (stage 3, todo)
 *   PRIO 4  radio_rx     radio block to the uplink queue                      (stage 2, todo)
 *   PRIO 5  uplink       tc_frame -> space packet -> router by APID           (stage 2, todo)
 *   PRIO 6  radio_tx     sole owner of UHF transmit                           (stage 2, todo)
 *   PRIO 7  beacon       periodic beacon                                      (stage 3, todo)
 *   PRIO 8  heartbeat    logs once a second, later statistics and scheduler   (stage 0, running)
 *
 * Data flow once stages 2 and 3 exist:
 *   Uplink    radio -> radio_rx -> uplink: tc_frame_parse, sp_parse -> router (by APID)
 *                -> 0x010 commands: policy -> CAN bus     -> 0x0F0 ping: reply to the TX queue
 *   Downlink  CAN replies, telemetry, beacon, ping replies -> sp_build -> dl_block_pack -> radio_tx -> radio
 *
 * Init order (main): radio device check, CAN setup, then the threads start themselves.
 *
 * Status: STAGE 0 SCAFFOLD. Only the heartbeat runs, and CAN is brought up. The link-layer modules
 * (stage 1) are written and tested but not called from here yet, so the linker drops them. Each
 * stage adds its threads to this file and its modules to src/modules.cmake.
 *
 * Docs: PROTOCOL.md (wire format), README.md (overview, architecture), PLANNED.md (stages, next
 * steps, open items).
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "board_config.h"
#include "comms_config.h"
#include "common/can_proto.h"

/** @cond */ /* Hidden from Doxygen, or it will mistake this as a function */
LOG_MODULE_REGISTER(comms, LOG_LEVEL_INF);
/** @endcond */

/* The block length is a protocol constant and also a property of the radio node. They must agree. */
BUILD_ASSERT(DT_PROP(RADIO_NODE, block_length) == COMMS_BLOCK_LEN,
	     "radio node block-length must equal COMMS_BLOCK_LEN in comms_config.h");

static const struct device *const can_dev = DEVICE_DT_GET(CAN_NODE);
static const struct device *const radio = DEVICE_DT_GET(RADIO_NODE);

/** Frames from the CAN driver. Nothing consumes them until stage 3. */
K_MSGQ_DEFINE(can_rx_q, sizeof(struct can_frame), CAN_RX_Q_LEN, 4);

/** Heartbeat log, so a stage 0 build visibly runs. Becomes the scheduler thread in stage 3. */
static void heartbeat_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	for (;;) {
		LOG_INF("alive, uptime %u s", k_uptime_get_32() / 1000U);
		k_msleep(HEARTBEAT_INTERVAL_MS);
	}
}
K_THREAD_DEFINE(heartbeat_tid, STACK_SIZE, heartbeat_thread, NULL, NULL, NULL, PRIO_SCHED, 0, 0);

int main(void)
{
	int err;

	LOG_INF("UT-CORE COMMS booting (stage 0 scaffold, see PLANNED.md)");

	if (!device_is_ready(radio)) {
		LOG_ERR("radio device not ready");
	}

	err = can_setup(can_dev, NODE_ID, &can_rx_q, NULL);
	if (err) {
		LOG_ERR("CAN setup failed: %d", err);
	}

	return 0;
}
