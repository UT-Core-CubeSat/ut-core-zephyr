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
 *   PRIO 0  watchdog     supervises the other threads' check-ins (common/health.h) and feeds the task
 *                        watchdog. Reports only: no reset yet, see the note at watchdog_thread (stage 3b)
 *   PRIO 1  (unused)     there is no can_rx thread: the CAN driver's filters already buffer frames
 *                        into a queue (can_rx_q), which can_proc reads
 *   PRIO 2  can_proc     CAN frames: answers for ground -> downlink queue, broadcasts -> telemetry cache
 *   PRIO 4  (unused)     there is no radio_rx thread: the driver's receive callback copies each
 *                        block straight into the uplink queue (app/radio_port)
 *   PRIO 5  uplink       tc_frame -> space packet -> router by APID
 *   PRIO 6  radio_tx     sole owner of UHF transmit
 *   PRIO 8  scheduler    CAN heartbeat, telemetry, beacon, statistics log (app/scheduler). There is no
 *                        separate beacon thread: queueing a packet never waits, so the beacon is just one
 *                        more job of the lowest-priority thread
 *
 * Data flow:
 *   Uplink    radio -> receive callback -> uplink: tc_frame_parse, sp_parse -> router (by APID)
 *                -> 0x010 commands: policy -> CAN bus     -> 0x0F0 ping: reply to the TX queue
 *   Downlink  CAN replies, telemetry, beacon, ping replies -> sp_build (callsign header first) ->
 *                dl_block_pack -> radio_tx -> radio
 *   Telemetry CAN broadcasts -> can_proc -> telemetry cache -> scheduler -> TLM (0x020) and beacon (0x021)
 *
 * Init order (main): radio device check, CAN setup, radio start and configure, start receiving. The
 * threads are static and start themselves at boot; they only wait on their queues until main has run.
 *
 * Status: STAGE 3b. The uplink and downlink pipeline runs against the radio driver, answers PING
 * (0x0F0 -> 0x0F1), bridges allowlisted commands (0x010) to the CAN bus, sends the board's answer
 * down on 0x020, sends periodic telemetry and a beacon, broadcasts COMMS's CAN heartbeat, and supervises
 * its own threads. Nothing has run on hardware. Each stage adds its threads to this file and its modules
 * to src/modules.cmake.
 *
 * Docs: PROTOCOL.md (wire format), README.md (overview, architecture), PLANNED.md (stages, next
 * steps, open items).
 */

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/task_wdt/task_wdt.h>

#include "app/downlink.h"
#include "app/radio_port.h"
#include "app/scheduler.h"
#include "app/tlm.h"
#include "app/uplink.h"
#include "board_config.h"
#include "can/can_bridge.h"
#include "can/telemetry.h"
#include "comms_config.h"
#include "common/can_proto.h"
#include "common/health.h"

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
static struct tlm_cache tlm_cache;
static struct tlm tlm;
static struct scheduler scheduler;

/** Next `tlm_counter` value, shared by the CAN replies and the periodic telemetry and beacon. */
static atomic_t tlm_counter;

/* ================= Thread supervision =================
 *
 * Each supervised thread checks in once per loop, so every loop waits at most a short time. The
 * supervisor (watchdog_thread) looks at the health table. The ids are made at APPLICATION init, before the
 * threads start. */
static int hb_can_proc = -1;
static int hb_uplink = -1;
static int hb_radio_tx = -1;
static int hb_sched = -1;

static int health_init(void)
{
	int err;

	hb_can_proc = health_register("can_proc", HEALTH_TIMEOUT_CAN_PROC_MS);
	hb_uplink = health_register("uplink", HEALTH_TIMEOUT_UPLINK_MS);
	hb_radio_tx = health_register("radio_tx", HEALTH_TIMEOUT_RADIO_TX_MS);
	hb_sched = health_register("scheduler", HEALTH_TIMEOUT_SCHED_MS);

	/* NULL: no hardware watchdog fallback (CONFIG_TASK_WDT_HW_FALLBACK is off), like CDH. */
	err = task_wdt_init(NULL);
	if (err) {
		LOG_ERR("task_wdt_init failed: %d", err);
	}
	return 0;
}
SYS_INIT(health_init, APPLICATION, 0);

static void uplink_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	for (;;) {
		health_checkin(hb_uplink);
		(void)uplink_service_one(&uplink, K_MSEC(SERVICE_WAIT_MS));
	}
}
K_THREAD_DEFINE(uplink_tid, STACK_UPLINK, uplink_thread, NULL, NULL, NULL, PRIO_UPLINK, 0, 0);

static void radio_tx_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	for (;;) {
		health_checkin(hb_radio_tx);
		(void)downlink_service_one(&downlink, K_MSEC(SERVICE_WAIT_MS));
	}
}
K_THREAD_DEFINE(radio_tx_tid, STACK_RADIO_TX, radio_tx_thread, NULL, NULL, NULL, PRIO_RADIO_TX, 0, 0);

/** Frames addressed to ground (the boards' answers) become telemetry packets. Broadcasts go to the
 *  telemetry cache, and anything else is ignored. */
static void can_proc_thread(void *a, void *b, void *c)
{
	struct can_frame frame;
	struct dl_item item;

	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	for (;;) {
		health_checkin(hb_can_proc);
		if (k_msgq_get(&can_rx_q, &frame, K_MSEC(SERVICE_WAIT_MS)) != 0) {
			continue;
		}
		if (can_bridge_on_frame(&can_bridge, &frame, &item) == 1) {
			(void)downlink_submit(&downlink, &item);
		} else {
			(void)tlm_cache_on_frame(&tlm_cache, &frame, k_uptime_get());
		}
	}
}
K_THREAD_DEFINE(can_proc_tid, STACK_CAN_PROC, can_proc_thread, NULL, NULL, NULL, PRIO_CAN_PROC, 0, 0);

static void log_stats(void)
{
	const struct uplink_stats *u = &uplink.stats;
	const struct downlink_stats *d = &downlink.stats;

	LOG_INF("alive %u s | up: blocks %u ok %u | down: sent %u err %u full %u | tlm %u beacon %u | cache %u",
		k_uptime_get_32() / 1000U, u->blocks, u->accepted, d->sent, d->send_errors, d->queue_full,
		tlm.stats.telemetry, tlm.stats.beacons, tlm_cache.stats.stored);
}

/** CAN heartbeat, telemetry, beacon, statistics. Lowest priority: what it does never waits. */
static void scheduler_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	for (;;) {
		int64_t now = k_uptime_get();

		health_checkin(hb_sched);
		if (scheduler_step(&scheduler, now) & JOB_BIT(JOB_STATS)) {
			log_stats();
		}
		k_msleep(MIN(scheduler_wait_ms(&scheduler, k_uptime_get()), SERVICE_WAIT_MS));
	}
}
K_THREAD_DEFINE(scheduler_tid, STACK_SCHED, scheduler_thread, NULL, NULL, NULL, PRIO_SCHED, 0, 0);

/** Fires only if the supervisor itself stops running. Timer interrupt context: no blocking, no logging. */
static atomic_t supervisor_wdt_expiries = ATOMIC_INIT(0);

static void supervisor_wdt_expired(int channel_id, void *user_data)
{
	ARG_UNUSED(channel_id);
	ARG_UNUSED(user_data);
	atomic_inc(&supervisor_wdt_expiries);
}

/**
 * Highest priority: supervises the other threads' check-ins, as CDH does (same common/health.h).
 *
 * Each pass it feeds its own task watchdog channel, logs when a thread goes late or recovers, and counts
 * the expiries of its own channel. **It only reports.** The hardware watchdog fallback is off (like CDH, a
 * started IWDG cannot be stopped), so a hung thread is logged and nothing resets the board. The reset
 * policy, RX progress checks and the CDH backstop are stage 7. The external watchdog IC on `WATCHDOG_PIN`
 * is not kicked either, because the schematic does not exist.
 */
static void watchdog_thread(void *a, void *b, void *c)
{
	bool was_late[HEALTH_MAX_ENTRIES] = {false};
	atomic_val_t reported = 0;
	int channel = task_wdt_add(SUPERVISOR_WDT_TIMEOUT_MS, supervisor_wdt_expired, NULL);

	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	if (channel < 0) {
		LOG_ERR("task_wdt_add failed: %d", channel);
	}
	for (;;) {
		atomic_val_t expiries;

		task_wdt_feed(channel);
		for (int id = 0; id < health_count(); id++) {
			bool late = health_is_late(id);

			if (late && !was_late[id]) {
				LOG_ERR("thread %s missed its check-in: %u ms ago (limit %u ms)", health_name(id),
					health_ms_since_checkin(id), health_timeout_ms(id));
			} else if (!late && was_late[id]) {
				LOG_WRN("thread %s is checking in again", health_name(id));
			}
			was_late[id] = late;
		}
		expiries = atomic_get(&supervisor_wdt_expiries);
		if (expiries != reported) {
			LOG_ERR("supervisor task watchdog expired (%ld total)", (long)expiries);
			reported = expiries;
		}
		k_msleep(WATCHDOG_KICK_MS);
	}
}
K_THREAD_DEFINE(watchdog_tid, STACK_SIZE, watchdog_thread, NULL, NULL, NULL, PRIO_WATCHDOG, 0, 0);

int main(void)
{
	int err;

	LOG_INF("UT-CORE COMMS booting (stage 3b, see PLANNED.md)");

	radio_port_init(&radio_port, radio, &uplink_q);
	downlink_init(&downlink, &tx_q, &radio_port);
	can_bridge_init(&can_bridge, can_dev, &tlm_counter);
	router_init(&router, &can_bridge,
		    (struct mvp_switches){
			    .ping = IS_ENABLED(CONFIG_COMMS_MVP_PING),
			    .can_cmd = IS_ENABLED(CONFIG_COMMS_MVP_CAN_CMD),
			    .tlm = IS_ENABLED(CONFIG_COMMS_MVP_TLM),
		    });
	uplink_init(&uplink, &uplink_q, &downlink, &router);
	tlm_cache_init(&tlm_cache);
	tlm_init(&tlm, &tlm_cache, &tlm_counter);
	scheduler_init(&scheduler, k_uptime_get(), STATS_LOG_INTERVAL_MS, &tlm, &downlink, &can_bridge,
		       &router.sw);

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
