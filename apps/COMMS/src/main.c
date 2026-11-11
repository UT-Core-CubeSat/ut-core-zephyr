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
 *   PRIO 1  (unused)     there is no can_rx thread: the CAN driver's filters already buffer frames
 *                        into a queue (can_rx_q), which can_proc reads
 *   PRIO 2  can_proc     CAN frames for ground (board replies) -> downlink queue (stage 3a, running)
 *   PRIO 4  (unused)     there is no radio_rx thread: the driver's receive callback copies each
 *                        block straight into the uplink queue (app/radio_port)
 *   PRIO 5  uplink       tc_frame -> space packet -> router by APID           (stage 2, running)
 *   PRIO 6  radio_tx     sole owner of UHF transmit                           (stage 2, running)
 *   PRIO 7  beacon       periodic beacon                                      (stage 3, todo)
 *   PRIO 8  heartbeat    logs once a second, later statistics and scheduler   (stage 0, running)
 *
 * Data flow once stages 2 and 3 exist:
 *   Uplink    radio -> receive callback -> uplink: tc_frame_parse, sp_parse -> router (by APID)
 *                -> 0x010 commands: policy -> CAN bus     -> 0x0F0 ping: reply to the TX queue
 *   Downlink  CAN replies, telemetry, beacon, ping replies -> sp_build -> dl_block_pack -> radio_tx -> radio
 *
 * Init order (main): radio device check, CAN setup, radio start and configure, start receiving. The
 * threads are static and start themselves at boot; they only wait on their queues until main has run.
 *
 * Status: STAGE 3a. The uplink and downlink pipeline runs against the radio driver, answers PING
 * (0x0F0 -> 0x0F1), and bridges allowlisted commands (0x010) to the CAN bus, sending the board's answer
 * down on 0x020. There is no telemetry cache, TLM, beacon, scheduler or watchdog yet (stage 3b). Each stage adds its
 * threads to this file and its modules to src/modules.cmake.
 *
 * Docs: PROTOCOL.md (wire format), README.md (overview, architecture), PLANNED.md (stages, next
 * steps, open items).
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "app/downlink.h"
#include "can/can_bridge.h"
#include "app/radio_port.h"
#include "app/uplink.h"
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

/** Frames from the CAN driver, read by can_proc. */
K_MSGQ_DEFINE(can_rx_q, sizeof(struct can_frame), CAN_RX_Q_LEN, 4);

/** Blocks from the radio to the uplink thread, and packets from everyone to the transmit thread. */
K_MSGQ_DEFINE(uplink_q, COMMS_BLOCK_LEN, UPLINK_Q_LEN, 1);
K_MSGQ_DEFINE(tx_q, sizeof(struct dl_item), TX_Q_LEN, 4);

static struct radio_port radio_port;
static struct downlink downlink;
static struct uplink uplink;
static struct router router;
static struct can_bridge can_bridge;

/** Next `tlm_counter` value, shared by the CAN replies now and the periodic telemetry in stage 3b. */
static atomic_t tlm_counter;

static void uplink_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	for (;;) {
		(void)uplink_service_one(&uplink, K_FOREVER);
	}
}
K_THREAD_DEFINE(uplink_tid, STACK_UPLINK, uplink_thread, NULL, NULL, NULL, PRIO_UPLINK, 0, 0);

static void radio_tx_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	for (;;) {
		(void)downlink_service_one(&downlink, K_FOREVER);
	}
}
K_THREAD_DEFINE(radio_tx_tid, STACK_RADIO_TX, radio_tx_thread, NULL, NULL, NULL, PRIO_RADIO_TX, 0, 0);

/** Frames addressed to ground (the boards' answers) become telemetry packets. Other frames are ignored
 *  until the telemetry cache exists (stage 3b). */
static void can_proc_thread(void *a, void *b, void *c)
{
	struct can_frame frame;
	struct dl_item item;

	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	for (;;) {
		k_msgq_get(&can_rx_q, &frame, K_FOREVER);
		if (can_bridge_on_frame(&can_bridge, &frame, &item) == 1) {
			(void)downlink_submit(&downlink, &item);
		}
	}
}
K_THREAD_DEFINE(can_proc_tid, STACK_CAN_PROC, can_proc_thread, NULL, NULL, NULL, PRIO_CAN_PROC, 0, 0);

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

	LOG_INF("UT-CORE COMMS booting (stage 3a, see PLANNED.md)");

	radio_port_init(&radio_port, radio, &uplink_q);
	downlink_init(&downlink, &tx_q, &radio_port);
	can_bridge_init(&can_bridge, can_dev, &tlm_counter);
	router_init(&router, &can_bridge,
		    (struct mvp_switches){
			    .ping = IS_ENABLED(CONFIG_COMMS_MVP_PING),
			    .can_cmd = IS_ENABLED(CONFIG_COMMS_MVP_CAN_CMD),
		    });
	uplink_init(&uplink, &uplink_q, &downlink, &router);

	if (!device_is_ready(radio)) {
		LOG_ERR("radio device not ready");
	} else {
		const struct rfm26w_config radio_cfg = {
			.frequency_hz = COMMS_RADIO_FREQ_HZ,
			.tx_power_dbm = COMMS_RADIO_TX_DBM,
			.profile = RFM26W_PROFILE_GFSK_9K6,
		};

		err = rfm26w_start(radio);
		if (!err) {
			err = rfm26w_configure(radio, &radio_cfg);
		}
		if (!err) {
			err = radio_port_start(&radio_port);
		}
		if (err) {
			LOG_ERR("radio start failed: %d", err);
		}
	}

	err = can_setup(can_dev, NODE_ID, &can_rx_q, NULL);
	if (err) {
		LOG_ERR("CAN setup failed: %d", err);
	} else {
		/* can_setup() filters on this node and broadcast only. The boards' answers are addressed to
		 * ground. */
		err = can_bridge_add_ground_filter(can_dev, &can_rx_q);
		if (err) {
			LOG_ERR("CAN ground filter failed: %d", err);
		}
	}

	return 0;
}
