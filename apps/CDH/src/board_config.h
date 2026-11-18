/**
 * @file board_config.h
 * @ingroup cdh
 * @brief UT-CORE CDH — Board Configuration
 *
 * Hardware-specific definitions for the CDH flight computer.
 * MCU: STM32U5, RTOS: Zephyr, CAN transceiver: TCAN3403.
 *
 * Keeps the main application code free of raw pin numbers, UID
 * constants, and devicetree boilerplate.
 */

#pragma once

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/i2c.h>

/* ================= Node Identity ================= */

#define NODE_ID 0x01   /**< This board's CAN node address (== CDH_ID).          */
#define DST_ME NODE_ID /**< Alias for this board's own CAN destination address. */
#define PRIO_LOW 3     /**< Default CAN priority for outbound frames.           */

/* ================= Devicetree Nodes ================= */

#define CAN_NODE DT_NODELABEL(fdcan1)   /**< FDCAN1 devicetree node.    */
#define I2C_BUS_NODE DT_NODELABEL(i2c1) /**< I2C1 devicetree node.      */
#define GPIOA_NODE DT_NODELABEL(gpioa)  /**< GPIOA devicetree node.     */
#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)
#define LED2_NODE DT_ALIAS(led2)

/* ================= GPIO Pin Assignments ================= */

#define PIN_ROLE 6     /**< MCU role strap — 0 = MCU1, 1 = MCU2.         */
#define WATCHDOG_PIN 2 /**< Hardware watchdog kick pin (PA2).            */

/* ================= STM32U5 96-bit Factory UIDs ================= */
/*
 * @name STM32U5 96-bit Factory UIDs
 * @brief Used at boot to cross-check hardware identity against the role
 *        strap pin. Read from UID_BASE in flash info area.
 */

#define UID1_WORD0 0x00340016
#define UID1_WORD1 0x41425007
#define UID1_WORD2 0x20363651

#define UID2_WORD0 0x0012001B
#define UID2_WORD1 0x41425007
#define UID2_WORD2 0x20363651

/* ================= Thread Configuration ================= */

#define PRIO_WATCHDOG 0  /**< Highest — hardware watchdog kick           */
#define PRIO_CAN_RX 1    /**< Drain hardware FIFO immediately            */
#define PRIO_CAN_PROC 2  /**< Decode & dispatch frames from SW queue     */
#define PRIO_SCHED 3     /**< Heartbeat + mission scheduling             */
#define PRIO_MODE_MGR 3  /**< Mode state machine (shares PRIO_SCHED)     */
#define PRIO_GNSS 4      /**< Request & store GNSS position data         */
#define PRIO_SOH 5       /**< Read temps & check node timeouts           */
#define PRIO_TELEMETRY 6 /**< Lowest — assemble & downlink telemetry     */

#define STACK_SIZE 1024

/* ================= Timing Constants ================= */

#define HEARTBEAT_TIMEOUT_MS 5000 /**< Node declared dead after this silence */
#define WATCHDOG_KICK_MS 15       /**< Supervisor pass period; must be < half HW watchdog timeout */
#define GNSS_POLL_MS 5000         /**< How often to request a GNSS fix       */
#define TELEMETRY_CHECK_MS 1000   /**< Downlink window / SOH check cadence   */
#define MODE_MGR_PERIOD_MS 500U   /**< PLACEHOLDER: how often transitions are re-checked */

/* ================= Thread Health Monitoring ================= */
/*
 * Longest allowed gap between a thread's check-ins before the supervisor
 * reports it as late. These are generous first guesses based on each
 * thread's loop period; measure real loop times on hardware and tighten.
 */
#define HEALTH_TIMEOUT_CAN_RX_MS 1000U
#define HEALTH_TIMEOUT_CAN_PROC_MS 1000U
#define HEALTH_TIMEOUT_SCHED_MS 1000U
#define HEALTH_TIMEOUT_SOH_MS 2000U
#define HEALTH_TIMEOUT_GNSS_MS (3U * GNSS_POLL_MS)
#define HEALTH_TIMEOUT_TELEMETRY_MS (3U * TELEMETRY_CHECK_MS)
#define HEALTH_TIMEOUT_MODE_MGR_MS (3U * MODE_MGR_PERIOD_MS)

#define CAN_RX_WAIT_MS 100              /**< Max time a CAN thread blocks waiting for a frame, so it still checks in on a quiet bus */
#define SUPERVISOR_WDT_TIMEOUT_MS 1000U /**< Task watchdog timeout for the supervisor thread itself */

/* ================= Mode State Machine ================= */
/*
 * PLACEHOLDER values. The state machine diagram leaves every one of these
 * TBD except STORAGE_NEARLY_FULL_PCT; they exist so the state machine runs,
 * not because the team chose them. Replace as decisions are made.
 */

#define BATTERY_SAFE_PCT 20        /**< TBD: at or below this, operational modes drop to SAFE */
#define BATTERY_MIN_STANDBY_PCT 40 /**< TBD: SAFE -> STANDBY guard                           */
#define BATTERY_MIN_SCAN_PCT 60    /**< TBD: required to start EXPERIMENT                    */

#define TEMP_SAFE_MIN_C (-40) /**< TBD: onboard sensor below this is unsafe (whole degC) */
#define TEMP_SAFE_MAX_C 85    /**< TBD: onboard sensor above this is unsafe (whole degC) */

#define STORAGE_NEARLY_FULL_PCT 75 /**< TBD: to help decide whether to downlink before scanning */

#define PHOENIX_ATTEMPT_LIMIT 4      /**< TBD: one attempt per escalation step              */
#define PHOENIX_STEP_SETTLE_MS 5000U /**< TBD: wait after a step before escalating          */
#define PHOENIX_TIMEOUT_MS 60000U    /**< TBD: PHOENIX is time-limited, then exits to SAFE  */

#define REBOOT_LOCKOUT_COUNT 5   /**< TBD: rapid reboots before SAFE lockout             */
#define REBOOT_STABLE_MS 600000U /**< TBD: uptime after which a boot no longer counts as rapid */

#define DOWNLINK_TIMEOUT_MS 600000U   /**< TBD: no ground traffic this long ends DOWNLINK */
#define EXPERIMENT_TIMEOUT_MS 600000U /**< TBD: experiment timeout (always running)       */

/*
 * Utah pass detection: a GNSS fix inside this lat/lon box (1e-7 deg) counts
 * as "over Utah". Box = Utah's straight-line borders; it includes the
 * Wyoming notch in the NE corner. No look-ahead: a pass "starts" on the
 * first fresh fix inside the box.
 */
#define UTAH_LAT_MIN_E7 370000000               /**< 37.00 N  southern border */
#define UTAH_LAT_MAX_E7 420000000               /**< 42.00 N  northern border */
#define UTAH_LON_MIN_E7 -1140500000             /**< 114.05 W western border  */
#define UTAH_LON_MAX_E7 -1090500000             /**< 109.05 W eastern border  */
#define GNSS_FIX_MAX_AGE_MS (3U * GNSS_POLL_MS) /**< Older fixes are ignored for pass detection */

/* ================= CAN Processing Queue ================= */

#define CAN_PROC_Q_LEN 32 /**< SW queue depth between RX and process threads */
