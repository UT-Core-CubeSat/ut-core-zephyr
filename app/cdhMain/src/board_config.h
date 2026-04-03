/* boards/cdh/board_config.h */
#pragma once

/* ── Node identity ── */
#define NODE_ID         0x01
#define NODE_NAME       "CDH"

/* ── Hardware UIDs ── */
#define UID1_WORD0      0x00340016
#define UID1_WORD1      0x41425007
#define UID1_WORD2      0x20363651
#define UID2_WORD0      0x0012001B
#define UID2_WORD1      0x41425007
#define UID2_WORD2      0x20363651

/* ── GPIO pin assignments ── */
#define PIN_SILENT      9
#define PIN_SHDN        10
#define PIN_ROLE        6
#define WATCHDOG_PIN    2

/* ── CAN bus ── */
#define CAN_BITRATE     500000

/* ── Temperature sensors ── */
#define TEMP_ADDRS      { 0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D }
#define NUM_TEMP_SENSORS 6

/* ── Thread config ── */
#define WATCHDOG_KICK_MS    500
#define HEARTBEAT_PERIOD_MS 1000
#define SOH_PERIOD_MS       500
#define GNSS_POLL_MS        5000
#define TELEMETRY_CHECK_MS  1000

/* ── Heartbeat timeout ── */
#define HEARTBEAT_TIMEOUT_MS 5000

/* ── Nodes this board tracks ── */
#define TRACKED_NODES { EPS_ID, COMMS_ID, ADCS_ID, MOTOR_ID, GNSS_ID, STAR_ID }
#define NUM_TRACKED_NODES 6