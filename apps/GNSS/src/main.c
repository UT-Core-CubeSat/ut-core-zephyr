/**
 * @defgroup gnss GNSS
 * @ingroup apps
 * @brief Global Navigation Satellite System
 * @include{doc} ./apps/GNSS/README.md
 */

/**
 * @file main.c
 * @ingroup gnss
 * @brief Orion B16 GNSS driver + CAN telemetry for UT-CORE bus.
 *
 * Initializes the Orion B16 GNSS receiver over UART and a CAN transceiver,
 * then runs a loop broadcasting heartbeats and periodic GNSS state-of-health
 * and position telemetry, while a separate thread handles incoming CAN
 * commands (position queries, update-rate changes) from CDH.
 *
 * @todo move CAN code shared between apps into the `common/` directory at git root.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/can.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include "orion_b16_driver.h"
#include "orion_b16_protocol.h"
#include "orion_b16_messages.h"
#include "common/can_proto.h"

/** @cond */ /* Hidden from Doxygen, or it will mistake this as a function */
LOG_MODULE_REGISTER(gnss_app, CONFIG_LOG_DEFAULT_LEVEL);
/** @endcond */

/* ===================================================== */
/* ================= CAN PROTOCOL ====================== */
/* ===================================================== */
/*
 * @name CAN Protocol
 * @brief Node IDs, priorities, and timing constants used on the bus.
 * @{
 */

#define NODE_ID       GNSS_ID   /* 0x06 */
#define NODE_CDH      CDH_ID    /* 0x01 */

#define PRIO_HIGH  0
#define PRIO_MED   2
#define PRIO_LOW   4

#define TIME_INTERVAL_MS   (60*60*1000) /* Send GPS time to CDH every hour */


/* @} */

/* ===================================================== */
/* ================= HW DEVICES ======================== */
/* ===================================================== */
/*
 * @name Hardware Devices
 * @brief GPIO/CAN device handles, transceiver pins, GNSS driver instance,
 *        and the CAN RX message queue.
 * @{
 */

static const struct device *const gpioa  = DEVICE_DT_GET(DT_NODELABEL(gpioa));
static const struct device *const can_dev = DEVICE_DT_GET(DT_NODELABEL(fdcan1));

/* GNSS driver instance */
static orion_driver_t gnss_driver;

/** @cond */ /* Hidden from Doxygen, or it will mistake this as a function */
/* CAN RX queue */
CAN_MSGQ_DEFINE(rxq, 16);
/** @endcond */

/* @} */

/* ===================================================== */
/* ================= CAN TX ============================ */
/* ===================================================== */
/*
 * @name CAN TX
 * @brief Outbound CAN frame construction: heartbeat, state-of-health, and
 *        position telemetry.
 * @{
 */

/**
 * @brief Broadcast an OP_HEARTBEAT frame announcing this node is alive.
 *
 * Sent periodically from main()'s loop at HEARTBEAT_INTERVAL_MS.
 */
static void send_heartbeat(void)
{
    int err = can_send2(can_dev, NODE_ID, CAN_BROADCAST, OP_HEARTBEAT, PRIO_LOW, 0x00, 0x00);
    if (err) {
        LOG_WRN("Failed to send CAN reply: %d", err);
    }
    LOG_INF("TX heartbeat");
}

/**
 * @brief Send a state-of-health frame: fix status, SV count, and DOP.
 *
 * Packs fix mode, satellite count, HDOP/PDOP (scaled by 10 and clamped to
 * 255), configured update rate, and a driver-running status flag into an
 * 8-byte OP_SOH_GNSS frame addressed to CDH.
 *
 * Payload:
 *   [0] fix_mode  (0=none, 2=2D, 3=3D, 4=3D+DGPS)
 *   [1] sv_count
 *   [2] HDOP * 10  (clamped to 255)
 *   [3] PDOP * 10  (clamped to 255)
 *   [4] update_rate_hz
 *   [5] status flags (bit0 = driver running)
 *   [6..7] unused (0)
 *
 * @todo hdop/pdop are divided by 10 under a comment claiming the source
 *       value is already ×100 ("we want *10"), but the resulting math
 *       (÷10 on a ×100 value) yields ×10 scale only if that premise is
 *       correct — worth double-checking against orion_nav_data_t's actual
 *       documented units to confirm hdop/pdop aren't ×1000 or similar,
 *       since a silent scale error here wouldn't be caught by the clamp.
 */
static void send_gnss_soh(void)
{
    orion_nav_data_t nav;
    orion_driver_get_nav(&gnss_driver, &nav);

    uint16_t hdop10 = nav.hdop / 10;  /* hdop is *100, we want *10 */
    uint16_t pdop10 = nav.pdop / 10;
    if (hdop10 > 255) hdop10 = 255;
    if (pdop10 > 255) pdop10 = 255;

    uint8_t flags = gnss_driver.running ? 0x01 : 0x00;

    int err = can_send8(can_dev, NODE_ID, NODE_CDH, OP_SOH_GNSS, PRIO_LOW,
        nav.fix_mode,
        nav.sv_count,
        (uint8_t)hdop10,
        (uint8_t)pdop10,
        gnss_driver.cfg.update_rate_hz,
        flags,
        0,
        0
    );
    if (err) {
        LOG_WRN("Failed to send OP_SOH_GNSS: %d", err);
    }

    LOG_INF("TX SOH  fix=%u sv=%u hdop=%u pdop=%u",
            nav.fix_mode, nav.sv_count, (unsigned)hdop10, (unsigned)pdop10);
}

/**
 * @brief Send the current position to CDH as one 12-byte CAN FD frame.
 *
 * Does nothing if the current nav fix is not valid.
 *
 * Frame layout (OP_GNSS_POS), all signed 32-bit big-endian:
 *   [0..3]  latitude  (1e-7 degrees)
 *   [4..7]  longitude (1e-7 degrees)
 *   [8..11] ellipsoid altitude (cm)
 */
static void send_gnss_position(const orion_nav_data_t *nav)
{
    uint8_t buf[12];

    can_put_be32(&buf[0], nav->latitude_1e7);
    can_put_be32(&buf[4], nav->longitude_1e7);
    can_put_be32(&buf[8], (int32_t)nav->ellipsoid_alt_cm);

    int err = can_send12(can_dev, NODE_ID, NODE_CDH, OP_GNSS_POS, PRIO_MED, buf);
    if (err) {
        LOG_WRN("Failed to send OP_GNSS_POS: %d", err);
    }

    double lat = ORION_DEG_FROM_1E7(nav->latitude_1e7);
    double lon = ORION_DEG_FROM_1E7(nav->longitude_1e7);
    LOG_INF("TX POS  lat=%.7f lon=%.7f alt=%d cm", lat, lon, (int)nav->ellipsoid_alt_cm);
}


/**
 * @brief Send the ECEF velocity to CDH as one 12-byte CAN FD frame.
 *
 * All three axes come from the same nav snapshot in a single frame, so they
 * are consistent by construction (no sequence number needed).
 *
 * Frame layout (OP_GNSS_VEL), all signed 32-bit big-endian, cm/s:
 *   [0..3] ECEF vx    [4..7] ECEF vy    [8..11] ECEF vz
 */
static void send_gnss_velocity(const orion_nav_data_t *nav)
{
    uint8_t buf[12];

    can_put_be32(&buf[0], nav->ecef_vx_cms);
    can_put_be32(&buf[4], nav->ecef_vy_cms);
    can_put_be32(&buf[8], nav->ecef_vz_cms);

    int err = can_send12(can_dev, NODE_ID, NODE_CDH, OP_GNSS_VEL, PRIO_MED, buf);
    if (err) {
        LOG_WRN("Failed to send OP_GNSS_VEL: %d", err);
    }

    LOG_INF("TX VEL  vx=%d vy=%d vz=%d cm/s",
            (int)nav->ecef_vx_cms, (int)nav->ecef_vy_cms, (int)nav->ecef_vz_cms);
}

/**
 * @brief Send position and velocity for the current nav fix (two CAN FD
 *        frames). Does nothing if the nav data is not valid.
 */
static void send_gnss_master(void)
{
    orion_nav_data_t nav;
    orion_driver_get_nav(&gnss_driver, &nav);

    if (!nav.valid) {
        return;  /* don't send garbage */
    }

    send_gnss_position(&nav);
    send_gnss_velocity(&nav);
}


/**
 * @brief Send the current GPS time (week + time-of-week) to CDH.
 * @return true if a frame was sent; false if there is no usable fix, in
 *         which case the caller should retry on its next pass.
 *
 * The time is GPS time, not UTC (GPS is currently ahead of UTC by the
 * leap-second count). Conversion is deliberately left to CDH/ground. The
 * value is the time of the receiver's last nav solution, so it is up to
 * about one nav interval old when sent, plus bus latency.
 *
 * Frame layout:
 *   [0..1] GPS week, unsigned 16-bit big-endian
 *   [2..5] time of week in centiseconds, unsigned 32-bit big-endian
 */
static bool send_gnss_time(void)
{
    orion_nav_data_t nav;
    orion_driver_get_nav(&gnss_driver, &nav);

    if (!nav.valid || nav.fix_mode == 0) {
        return false; /*don't send a time if no fix found*/
    }

    /* week: 16-bit big-endian; time-of-week: 32-bit big-endian centiseconds;
     * bytes 6..7 unused (0) */
    int err = can_send8(can_dev, NODE_ID, NODE_CDH, OP_GNSS_TIME, PRIO_MED,
        (nav.gnss_week >> 8) & 0xFF,
        (nav.gnss_week) & 0xFF,
        (nav.tow_cs >> 24) & 0xFF,
        (nav.tow_cs >> 16) & 0xFF,
        (nav.tow_cs >> 8) & 0xFF,
        (nav.tow_cs) & 0xFF,
        0,
        0
    );
    if (err) {
        LOG_WRN("Failed to send OP_GNSS_TIME: %d", err);
    }
    LOG_INF("TX TIME week=%u tow=%u.%02u s", nav.gnss_week,
            nav.tow_cs / 100, nav.tow_cs % 100);
    return true;
}
/* @} */

/* ===================================================== */
/* ================= CAN RX ============================ */
/* ===================================================== */
/*
 * @name CAN RX
 * @brief Inbound CAN frame decoding and message dispatch.
 * @{
 */


/**
 * @brief Handle an incoming command frame from CDH.
 * @param pkt Decoded CAN packet; pkt->op is the opcode.
 *
 * OP_GET_POS replies immediately with the current position plus an
 * OP_RESP_GET_POS ack. OP_SET_UPDATE_RATE (data[0] = requested rate in
 * Hz) attempts to apply the new rate via the GNSS driver and replies with
 * OP_RESP_SET_UPDATE_RATE carrying the accepted rate on success or 0x00 on
 * failure.
 */
static void handle_command(const can_packet_t *pkt)
{
    switch (pkt->op) {
    case OP_GET_POS: {
        /* CDH asked for current position — reply immediately */
        send_gnss_master();
        int err = can_send2(can_dev, NODE_ID, pkt->src, OP_RESP_GET_POS, PRIO_LOW, 0x01, 0x00);
        if (err) {
            LOG_WRN("Failed to send CAN reply: %d", err);
        }
        LOG_INF("RX query position from 0x%02X", pkt->src);
        break;
    }

    case OP_SET_UPDATE_RATE: {
        /* data[0] = new rate in Hz */
        uint8_t rate = pkt->data[0];
        int rc = orion_driver_set_update_rate(&gnss_driver, rate);
        uint8_t ack_val = (rc == 1) ? rate : 0x00;
        int err = can_send2(can_dev, NODE_ID, pkt->src, OP_RESP_SET_UPDATE_RATE, PRIO_LOW, ack_val, 0x00);
        if (err) {
            LOG_WRN("Failed to send CAN reply: %d", err);
        }
        LOG_INF("RX set update rate=%u Hz → %s", rate, rc == 1 ? "ACK" : "NACK");
        break;
    }

    default:
        LOG_WRN("Unknown command opcode: 0x%03X", pkt->op);
        break;
    }
}

/**
 * @brief Handle a received OP_HEARTBEAT frame.
 * @param pkt Decoded CAN packet.
 *
 * Currently only logs the sender; no liveness table is maintained here.
 */
static void handle_heartbeat(const can_packet_t *pkt)
{
    LOG_INF("RX heartbeat from 0x%02X", pkt->src);
}

/**
 * @brief Route a decoded CAN packet to its handler by opcode.
 * @param pkt Decoded CAN packet.
 *
 * Dispatches OP_HEARTBEAT to handle_heartbeat() and OP_GET_POS /
 * OP_SET_UPDATE_RATE to handle_command(). Unrecognized opcodes are logged
 * as warnings and dropped.
 */
static void can_dispatch(const can_packet_t *pkt)
{
    switch (pkt->op) {
    case OP_HEARTBEAT:       handle_heartbeat(pkt); break;
    case OP_GET_POS:
    case OP_SET_UPDATE_RATE: handle_command(pkt);   break;
    default:
        LOG_WRN("Unhandled op: 0x%03X from 0x%02X", pkt->op, pkt->src);
        break;
    }
}

/* @} */

/* ===================================================== */
/* ================= CAN SETUP ========================= */
/* ===================================================== */
/*
 * @name CAN Setup
 * @brief Transceiver wakeup and CAN controller/filter initialization.
 * @{
 */

/* @} */

/* ===================================================== */
/* ================= CAN RX THREAD ===================== */
/* ===================================================== */
/*
 * @name CAN RX Thread
 * @brief Dedicated thread draining the CAN RX queue and dispatching
 *        received frames.
 * @{
 */

#define CAN_RX_STACK_SIZE  1024
#define CAN_RX_PRIORITY    5

/**
 * @brief Thread entry point: blocks on the CAN RX queue, decoding and
 *        dispatching each frame as it arrives.
 * @param a Unused thread argument.
 * @param b Unused thread argument.
 * @param c Unused thread argument.
 */
static void can_rx_thread(void *a, void *b, void *c)
{
    struct can_frame frame;
    can_packet_t pkt;

    while (1) {
        if (k_msgq_get(&rxq, &frame, K_FOREVER) == 0) {
            if (can_decode(&frame, &pkt) != 0) {
                continue;
            }
            can_dispatch(&pkt);
        }
    }
}

/** @cond */ /* Hidden from Doxygen, or it will mistake this as a function */
K_THREAD_DEFINE(can_rx_tid, CAN_RX_STACK_SIZE,
    can_rx_thread, NULL, NULL, NULL,
    CAN_RX_PRIORITY, 0, 0);
/** @endcond */

/* @} */

/* ===================================================== */
/* ============== GNSS NAV CALLBACK ==================== */
/* ===================================================== */
/*
 * @name GNSS Nav Callback
 * @brief Callback invoked by the Orion driver whenever a new nav fix is
 *        available.
 * @{
 */

/**
 * @brief Nav-update callback registered with the Orion driver.
 * @param nav  Latest nav fix data from the GNSS driver.
 * @param user Unused user-data pointer (registered as NULL).
 *
 * Logs fix mode, position, altitude, velocity, and satellite count when
 * the fix is valid; logs a warning and returns early otherwise.
 */
static void on_nav_update(const orion_nav_data_t *nav, void *user)
{
    (void)user;

    if (!nav->valid) {
        LOG_WRN("NAV callback: invalid fix");
        return;
    }

    double lat = ORION_DEG_FROM_1E7(nav->latitude_1e7);
    double lon = ORION_DEG_FROM_1E7(nav->longitude_1e7);
    double alt = ORION_M_FROM_CM(nav->msl_alt_cm);

    const char *fix_str;
    switch (nav->fix_mode) {
        case ORION_FIX_2D:      fix_str = "2D";       break;
        case ORION_FIX_3D:      fix_str = "3D";       break;
        case ORION_FIX_3D_DGPS: fix_str = "3D+DGPS";  break;
        default:                fix_str = "NONE";      break;
    }

    LOG_INF("FIX %s | lat=%.6f lon=%.6f alt=%.2fm | SVs=%u | vx=%d vy=%d vz=%d cm/s",
            fix_str, lat, lon, alt, nav->sv_count,
            nav->ecef_vx_cms, nav->ecef_vy_cms, nav->ecef_vz_cms);
}

/* @} */

/* ===================================================== */
/* ================= MAIN ============================== */
/* ===================================================== */
/*
 * @name Main
 * @brief Application entry point.
 * @{
 */

/* Forward declaration for config recipe */
extern int orion_config_cubesat_default(orion_driver_t *drv, uint8_t rate_hz, bool save);

#define HEARTBEAT_INTERVAL_MS   1000
#define SOH_INTERVAL_MS         5000
#define POS_INTERVAL_MS         5000

/**
 * @brief Entry point: initializes the GNSS driver and CAN bus, then runs
 *        the main loop broadcasting heartbeats and periodic telemetry.
 *
 * Boot sequence: verify GPIOA is ready, initialize and start the Orion
 * B16 GNSS driver over UART with a nav-update callback registered, apply
 * the default CubeSat GNSS config, then wake the CAN transceiver and
 * bring up the bus. The main loop sends a heartbeat every
 * HEARTBEAT_INTERVAL_MS, a state-of-health frame every SOH_INTERVAL_MS,
 * and position telemetry plus a detailed nav/stats log line every
 * POS_INTERVAL_MS.
 *
 * @return Returns the driver init/start error code early on failure, or 0
 *         if GPIOA isn't ready; otherwise does not return under normal
 *         operation.
 */
int main(void)
{
    LOG_INF("=== Orion B16 GNSS + CAN Application ===");

    /* ── GPIO check ── */
    if (!device_is_ready(gpioa)) {
        LOG_ERR("GPIOA not ready");
        return 0;
    }

    /* ── GNSS driver init ── */
    orion_driver_cfg_t cfg = ORION_DRIVER_CFG_DEFAULTS;
    cfg.uart_dev = DEVICE_DT_GET(DT_NODELABEL(lpuart1));

    int rc = orion_driver_init(&gnss_driver, &cfg);
    if (rc != 0) {
        LOG_ERR("GNSS driver init failed: %d", rc);
        return rc;
    }

    orion_driver_register_nav_callback(&gnss_driver, on_nav_update, NULL);

    rc = orion_driver_start(&gnss_driver);
    if (rc != 0) {
        LOG_ERR("GNSS driver start failed: %d", rc);
        return rc;
    }

    orion_config_cubesat_default(&gnss_driver, 1, false);
    LOG_INF("GNSS driver running, waiting for fix...");

    /* ── CAN bus init ── */
    tcan3403_wakeup();
    int err = can_setup(can_dev, NODE_ID, &rxq, NULL);
    if (err) {
        LOG_ERR("CAN Setup failed: %d", err);
        return 0;
    }

    LOG_INF("Running — GNSS + CAN active");

    /* ── Main loop ── */
    int64_t last_hb  = k_uptime_get();
    int64_t last_soh = k_uptime_get();
    int64_t last_pos = k_uptime_get();
    /* Start one interval in the past so the first time frame goes out on the
     * first good fix; later, last_time only resets after a successful send,
     * so a missed hour is retried on the next pass instead of waiting
     * another hour. */
    int64_t last_time = -(int64_t)TIME_INTERVAL_MS;

    while (1) {
        int64_t now = k_uptime_get();

        if ((now - last_hb) >= HEARTBEAT_INTERVAL_MS) {
            send_heartbeat();
            last_hb = now;
        }

        if ((now - last_soh) >= SOH_INTERVAL_MS) {
            send_gnss_soh();
            last_soh = now;
        }

        if ((now - last_pos) >= POS_INTERVAL_MS) {
            send_gnss_master();

            /* Print detailed nav + stats on the same 5s cadence */
            orion_nav_data_t nav;
            orion_driver_get_nav(&gnss_driver, &nav);
            if (nav.valid) {
                double lat = ORION_DEG_FROM_1E7(nav.latitude_1e7);
                double lon = ORION_DEG_FROM_1E7(nav.longitude_1e7);
                double alt = ORION_M_FROM_CM(nav.msl_alt_cm);
                LOG_INF("NAV fix=%u sv=%u lat=%.6f lon=%.6f alt=%.2fm vx=%d vy=%d vz=%d cm/s",
                        nav.fix_mode, nav.sv_count, lat, lon, alt,
                        nav.ecef_vx_cms, nav.ecef_vy_cms, nav.ecef_vz_cms);
            } else {
                LOG_INF("No valid fix yet.");
            }

            uint32_t rx_b, tx_b, nav_cnt, f_ok, f_err;
            orion_driver_get_stats(&gnss_driver, &rx_b, &tx_b, &nav_cnt, &f_ok, &f_err);
            LOG_INF("Stats: RX=%u TX=%u nav=%u ok=%u err=%u",
                    rx_b, tx_b, nav_cnt, f_ok, f_err);

            last_pos = now;
        }
        if ((now - last_time) >= TIME_INTERVAL_MS) {
            if (send_gnss_time()) {
                last_time = now;
            }
        }

        k_msleep(100);
    }

    return 0;
}

/* @} */
