#pragma once

/*
 * UT-CORE CDH — Board Configuration
 *
 * Hardware-specific definitions for the CDH flight computer.
 * MCU: STM32U5, RTOS: Zephyr, CAN transceiver: TCAN3403.
 *
 * Keeps the main application code free of raw pin numbers, UID
 * constants, and devicetree boilerplate.
 */

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/i2c.h>

/* ================= Node Identity ================= */

#define NODE_ID     0x01      /* This board's CAN node address (== CDH_ID) */
#define DST_ME      NODE_ID
#define PRIO_LOW    3         /* Default CAN priority for outbound frames  */

/* ================= Devicetree Nodes ================= */

#define CAN_NODE     DT_NODELABEL(fdcan1)
#define I2C_BUS_NODE DT_NODELABEL(i2c1)
#define GPIOA_NODE   DT_NODELABEL(gpioa)
#define LED0_NODE    DT_ALIAS(led0)
#define LED1_NODE    DT_ALIAS(led1)
#define LED2_NODE    DT_ALIAS(led2)

/* ================= GPIO Pin Assignments ================= */

#define PIN_SILENT   9        /* TCAN3403 silent-mode control (active high) */
#define PIN_SHDN     10       /* TCAN3403 shutdown control (active high)    */
#define PIN_ROLE     6        /* MCU role strap — 0 = MCU1, 1 = MCU2       */
#define WATCHDOG_PIN 2        /* Hardware watchdog kick pin (PA2)           */

/* ================= STM32U5 96-bit Factory UIDs ================= */
/*
 * Used at boot to cross-check hardware identity against the role strap pin.
 * Read from UID_BASE in flash info area.
 */

#define UID1_WORD0 0x00340016
#define UID1_WORD1 0x41425007
#define UID1_WORD2 0x20363651

#define UID2_WORD0 0x0012001B
#define UID2_WORD1 0x41425007
#define UID2_WORD2 0x20363651

/* ================= Thread Configuration ================= */
/*
 * Priority 0 is reserved for the watchdog — it must always be able to kick
 * the hardware timer regardless of what any other thread is doing.
 */

#define PRIO_WATCHDOG   0     /* Highest — hardware watchdog kick           */
#define PRIO_CAN_RX     1     /* Drain hardware FIFO immediately            */
#define PRIO_CAN_PROC   2     /* Decode & dispatch frames from SW queue     */
#define PRIO_SCHED      3     /* Heartbeat + mission scheduling             */
#define PRIO_GNSS       4     /* Request & store GNSS position data         */
#define PRIO_SOH        5     /* Read temps & check node timeouts           */
#define PRIO_TELEMETRY  6     /* Lowest — assemble & downlink telemetry     */

#define STACK_SIZE 1024

/* ================= Timing Constants ================= */

#define HEARTBEAT_TIMEOUT_MS  5000  /* Node declared dead after this silence */
#define WATCHDOG_KICK_MS      15    /* Must be < half HW watchdog timeout    */
#define GNSS_POLL_MS          5000  /* How often to request a GNSS fix       */
#define TELEMETRY_CHECK_MS    1000  /* Downlink window / SOH check cadence   */

/* ================= CAN Processing Queue ================= */

#define CAN_PROC_Q_LEN 32    /* SW queue depth between RX and process threads */