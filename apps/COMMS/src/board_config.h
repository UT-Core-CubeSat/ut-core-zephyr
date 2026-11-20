/**
 * @file board_config.h
 * @ingroup comms
 * @brief UT-CORE COMMS: hardware configuration (devicetree nodes, pins, thread priorities, queue sizes)
 *
 * Hardware only. Protocol constants (block length, spacecraft ID, GROUND_ID, callsign) are in
 * comms_config.h.
 *
 * @warning PLACEHOLDER VALUES. The COMMS board is still being designed. The pins below were copied
 *          from the CDH board and are almost certainly wrong for COMMS. Everything marked
 *          [PLACEHOLDER] must be revisited when the schematic is final. PLANNED.md section 4 keeps the list.
 */

#pragma once

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/gpio.h>

#include "common/can_proto.h"

/* ================= Node identity ================= */

#define NODE_ID  COMMS_ID /**< This board's CAN node address (defined in can_proto.h). */
#define PRIO_LOW 3        /**< [PLACEHOLDER] Default CAN arbitration priority for outbound frames. */

/* ================= Devicetree nodes ================= */

#define CAN_NODE   DT_NODELABEL(fdcan1) /**< [PLACEHOLDER] FDCAN1, copied from CDH. */
#define RADIO_NODE DT_NODELABEL(rfm26w0) /**< The UHF radio. The node is a placeholder (see boards/ut_core.overlay). */

/* ================= GPIO pins ================= */

/** [PLACEHOLDER] copied from CDH, almost certainly wrong for COMMS. Not used until stage 3. */
#define PIN_SILENT   9  /**< CAN transceiver silent-mode control (active high). */
#define PIN_SHDN     10 /**< CAN transceiver shutdown control (active high). */
#define WATCHDOG_PIN 2  /**< Hardware watchdog kick pin. */

/* ================= Thread priorities =================
 *
 * Lower number is more urgent. The radio driver has its own work queue thread, whose priority is
 * CONFIG_RFM26W_WORKQ_PRIORITY (default 1). FIFO streaming has a deadline of a few milliseconds at the
 * higher data rates, so nothing that can hold the CPU for longer than that may run above it.
 * Only threads marked "stage 0" exist so far. See README.md for the thread table.
 */
#define PRIO_WATCHDOG 0 /**< Thread supervisor and task watchdog. Never starve it. (stage 3b) */
#define PRIO_CAN_RX   1 /**< Unused: the CAN driver's filters buffer into a queue. */
#define PRIO_CAN_PROC 2 /**< Decode and dispatch CAN frames. (stage 3a) */
#define PRIO_RADIO_RX 4 /**< Unused: the radio callback feeds the uplink queue directly. */
#define PRIO_UPLINK   5 /**< TC frame check, packet routing. (stage 2) */
#define PRIO_RADIO_TX 6 /**< Sole owner of UHF transmit. (stage 2) */
#define PRIO_SCHED    8 /**< CAN heartbeat, telemetry, beacon, statistics. (stage 3b) */

/** [PLACEHOLDER] Stack sizes. Measure with CONFIG_THREAD_STACK_INFO before trusting them. */
#define STACK_SIZE    1024
#define STACK_UPLINK  3072
#define STACK_RADIO_TX 2048
#define STACK_CAN_PROC 2048
#define STACK_SCHED    2048

/* ================= Timing ================= */

#define WATCHDOG_KICK_MS      250   /**< Supervisor pass period. Must be below half SUPERVISOR_WDT_TIMEOUT_MS. */
#define SUPERVISOR_WDT_TIMEOUT_MS 1000U /**< Task watchdog timeout of the supervisor thread itself (as CDH). */
#define STATS_LOG_INTERVAL_MS 10000 /**< How often the counters are logged. */

/** Longest a supervised thread waits on its queue, so it checks in at least this often. */
#define SERVICE_WAIT_MS 250U

/** Longest allowed gap between check-ins, per thread. radio_tx may sit in a send for up to
 *  COMMS_RADIO_SEND_TIMEOUT_MS (1 s) plus the wait. [PLACEHOLDER] until measured on the board. */
#define HEALTH_TIMEOUT_CAN_PROC_MS 1500U
#define HEALTH_TIMEOUT_UPLINK_MS   1500U
#define HEALTH_TIMEOUT_RADIO_TX_MS 4000U
#define HEALTH_TIMEOUT_SCHED_MS    1500U

/* ================= Queues ================= */

#define CAN_RX_Q_LEN 32 /**< [PLACEHOLDER] Frames waiting between the CAN driver and the processing thread. */
#define UPLINK_Q_LEN 4  /**< Blocks waiting for the uplink thread. (stage 2) */
#define TX_Q_LEN     4  /**< Blocks waiting for the transmit thread. (stage 2) */
