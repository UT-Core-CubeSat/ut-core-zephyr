/**
 * @file main.c
 * @brief CDH mode test: STANDBY + GNSS fix over Utah -> EXPERIMENT.
 *
 * Hardware-in-the-loop test of the CDH mode state machine. Flash this onto
 * the GNSS board in place of the GNSS firmware, with CDH running its normal
 * firmware on the same CAN bus. CDH only accepts positions from GNSS_ID, so
 * this board answers as GNSS_ID; it also sends ground OP_SET_MODE commands
 * as COMMS_ID (no COMMS board is on the bus yet). Results print over RTT.
 *
 * CDH's mode is read from its heartbeat (payload byte 0).
 *
 * Steps:
 *   0. Wait for a CDH heartbeat.
 *   1. Send a GNSS fix outside Utah, then ground SAFE followed by STANDBY.
 *      Expect STANDBY, and still STANDBY after STAY_IN_STANDBY_MS.
 *   2. Send a GNSS fix over Utah. Expect EXPERIMENT within
 *      MODE_CHANGE_TIMEOUT_MS.
 * Afterwards CDH is sent back to SAFE whether the test passed or not.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/can.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/printk.h>
#include <stdbool.h>

#include "common/can_proto.h"

/* ================= CAN PROTOCOL ================= */

#define NODE_ID GNSS_ID /**< Impersonate GNSS: CDH ignores positions from any other node */
#define PRIO_MED 2      /**< Priority the GNSS app uses for position frames */
#define PRIO_LOW 4

/* CDH modes as broadcast in its heartbeat; must match cdh_mode_t in apps/CDH/src/main.c */
#define CDH_MODE_SAFE 0
#define CDH_MODE_STANDBY 1
#define CDH_MODE_DOWNLINK 2
#define CDH_MODE_EXPERIMENT 3
#define CDH_MODE_PHOENIX 4
#define CDH_MODE_UNKNOWN 0xFF /**< No CDH heartbeat seen yet */

/* ================= TIMING ================= */

#define CDH_HEARTBEAT_TIMEOUT_MS 30000 /**< Allows for CDH booting at the same time as this board */
#define MODE_CHANGE_TIMEOUT_MS 5000    /**< CDH mode manager (500 ms) + heartbeat (1 s), with margin */
#define STAY_IN_STANDBY_MS 3000        /**< How long CDH must stay in STANDBY with a fix outside Utah */
#define FIX_PERIOD_MS 1000             /**< CDH ignores fixes older than 15 s */

/* ================= HW DEVICES ================= */

static const struct device *const can_dev = DEVICE_DT_GET(DT_NODELABEL(fdcan1));

/** @cond */ /* Hidden from Doxygen, or it will mistake this as a function */
CAN_MSGQ_DEFINE(rxq, 16);
/** @endcond */

/* ================= TEST STATE ================= */

typedef struct
{
    const char *name;
    int32_t lat_e7; /**< Latitude, 1e-7 deg */
    int32_t lon_e7; /**< Longitude, 1e-7 deg */
} test_fix_t;

static const test_fix_t fix_outside_utah = {"Las Vegas, NV", 361699000, -1151398000};
static const test_fix_t fix_over_utah = {"Salt Lake City, UT", 407608000, -1118910000};

/** Fix sent every FIX_PERIOD_MS while the test waits, or NULL for none. */
static const test_fix_t *active_fix;
static int64_t last_fix_ms;

/** CDH mode from its latest heartbeat. */
static uint8_t cdh_mode = CDH_MODE_UNKNOWN;

/* ================= HELPERS ================= */

static const char *mode_str(uint8_t mode)
{
    switch (mode)
    {
    case CDH_MODE_SAFE:
        return "SAFE";
    case CDH_MODE_STANDBY:
        return "STANDBY";
    case CDH_MODE_DOWNLINK:
        return "DOWNLINK";
    case CDH_MODE_EXPERIMENT:
        return "EXPERIMENT";
    case CDH_MODE_PHOENIX:
        return "PHOENIX";
    case CDH_MODE_UNKNOWN:
        return "UNKNOWN";
    default:
        return "INVALID";
    }
}

/**
 * @brief Send a GNSS position frame packed exactly like apps/GNSS
 *        send_gnss_position(): 12-byte CAN FD frame, int32 big-endian
 *        lat/lon (1e-7 deg) and altitude (cm, fixed at 0 here).
 */
static void send_fix(const test_fix_t *fix)
{
    uint8_t buf[12];

    can_put_be32(&buf[0], fix->lat_e7);
    can_put_be32(&buf[4], fix->lon_e7);
    can_put_be32(&buf[8], 0);

    int err = can_send12(can_dev, NODE_ID, CDH_ID, OP_GNSS_POS, PRIO_MED, buf);
    if (err)
    {
        printk("  warning: GNSS fix not queued: %d\n", err);
    }
}

/**
 * @brief Send a ground OP_SET_MODE command to CDH. Sent as COMMS_ID because
 *        ground commands reach CDH through COMMS.
 */
static void send_ground_mode(uint8_t mode)
{
    printk("  ground -> CDH: OP_SET_MODE %s\n", mode_str(mode));

    int err = can_send2(can_dev, COMMS_ID, CDH_ID, OP_SET_MODE, PRIO_LOW, mode, 0);
    if (err)
    {
        printk("  warning: OP_SET_MODE not queued: %d\n", err);
    }
}

/**
 * @brief Handle at most one received frame (tracking CDH's heartbeat mode),
 *        and keep sending active_fix every FIX_PERIOD_MS.
 * @param wait_ms Longest time to wait for a frame.
 */
static void pump(int32_t wait_ms)
{
    struct can_frame rx;

    if (k_msgq_get(&rxq, &rx, K_MSEC(wait_ms)) == 0)
    {
        can_packet_t pkt;
        can_decode(&rx, &pkt);

        if (pkt.src == CDH_ID && pkt.op == OP_HEARTBEAT &&
            pkt.dlc >= 1)
        {
            if (pkt.data[0] != cdh_mode)
            {
                printk("  CDH heartbeat: mode %s\n", mode_str(pkt.data[0]));
            }
            cdh_mode = pkt.data[0];
        }
    }

    int64_t now = k_uptime_get();

    if (active_fix != NULL && (now - last_fix_ms) >= FIX_PERIOD_MS)
    {
        send_fix(active_fix);
        last_fix_ms = now;
    }
}

/**
 * @brief Wait until a CDH heartbeat reports @p expected.
 * @return true if it did within @p timeout_ms.
 */
static bool wait_for_mode(uint8_t expected, int32_t timeout_ms)
{
    int64_t deadline = k_uptime_get() + timeout_ms;

    while (k_uptime_get() < deadline)
    {
        pump(50);

        if (cdh_mode == expected)
        {
            return true;
        }
    }

    return false;
}

/**
 * @brief Check that CDH's heartbeats keep reporting @p expected.
 * @return true if they did for the whole @p duration_ms.
 */
static bool stays_in_mode(uint8_t expected, int32_t duration_ms)
{
    int64_t deadline = k_uptime_get() + duration_ms;

    while (k_uptime_get() < deadline)
    {
        pump(50);

        if (cdh_mode != expected)
        {
            return false;
        }
    }

    return true;
}

/* ================= TEST ================= */

/**
 * @brief Run the test steps.
 * @return true on PASS.
 */
static bool run_test(void)
{
    printk("[0] Waiting for a CDH heartbeat\n");
    int64_t deadline = k_uptime_get() + CDH_HEARTBEAT_TIMEOUT_MS;

    while (cdh_mode == CDH_MODE_UNKNOWN && k_uptime_get() < deadline)
    {
        pump(50);
    }
    if (cdh_mode == CDH_MODE_UNKNOWN)
    {
        printk("FAIL: no CDH heartbeat within %d ms (CDH powered and flashed? CAN bus connected?)\n",
               CDH_HEARTBEAT_TIMEOUT_MS);
        return false;
    }

    printk("[1] CDH to STANDBY, GNSS fix outside Utah (%s)\n", fix_outside_utah.name);
    active_fix = &fix_outside_utah;

    /* STANDBY is only accepted from SAFE, so start from SAFE whatever mode
     * CDH is in (e.g. left over from an earlier run). */
    send_ground_mode(CDH_MODE_SAFE);
    if (!wait_for_mode(CDH_MODE_SAFE, MODE_CHANGE_TIMEOUT_MS))
    {
        printk("FAIL: CDH did not enter SAFE (mode %s)\n", mode_str(cdh_mode));
        return false;
    }

    send_ground_mode(CDH_MODE_STANDBY);
    if (!wait_for_mode(CDH_MODE_STANDBY, MODE_CHANGE_TIMEOUT_MS))
    {
        printk("FAIL: CDH did not enter STANDBY (mode %s); check the CDH log for which "
               "STANDBY guard rejected it\n",
               mode_str(cdh_mode));
        return false;
    }

    if (!stays_in_mode(CDH_MODE_STANDBY, STAY_IN_STANDBY_MS))
    {
        printk("FAIL: CDH left STANDBY for %s with a fix outside Utah\n", mode_str(cdh_mode));
        return false;
    }
    printk("  ok: CDH stayed in STANDBY for %d ms outside Utah\n", STAY_IN_STANDBY_MS);

    printk("[2] GNSS fix over Utah (%s)\n", fix_over_utah.name);
    active_fix = &fix_over_utah;
    last_fix_ms = 0; /* send the first one now */

    if (!wait_for_mode(CDH_MODE_EXPERIMENT, MODE_CHANGE_TIMEOUT_MS))
    {
        printk("FAIL: CDH did not enter EXPERIMENT within %d ms (mode %s)\n",
               MODE_CHANGE_TIMEOUT_MS, mode_str(cdh_mode));
        return false;
    }
    printk("  ok: CDH entered EXPERIMENT\n");

    return true;
}

int main(void)
{
    printk("\n=== CDH mode test: STANDBY + fix over Utah -> EXPERIMENT ===\n");

    int err = tcan3403_wakeup();
    if (err)
    {
        printk("TCAN3403 wakeup failed: %d\n", err);
    }
    err = can_setup(can_dev, NODE_ID, &rxq, NULL);
    if (err)
    {
        printk("FAIL: CAN setup failed: %d\n", err);
        return 0;
    }

    bool pass = run_test();

    /* Leave CDH in SAFE and stop sending fixes, pass or fail. */
    active_fix = NULL;
    send_ground_mode(CDH_MODE_SAFE);
    wait_for_mode(CDH_MODE_SAFE, MODE_CHANGE_TIMEOUT_MS);

    printk("\nCAN TX errors during test: %ld\n", (long)can_proto_tx_error_count());
    printk("=== %s ===\n", pass ? "PASS" : "FAIL");

    /* Keep draining rxq: the CAN filters stay active, and CDH's heartbeats
     * would otherwise overflow it once the test stops reading. */
    while (1)
    {
        pump(100);
    }

    return 0;
}
