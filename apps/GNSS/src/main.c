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
 * @brief Send a position telemetry frame with lat/lon packed as scaled
 *        24-bit integers.
 *
 * Converts nav.latitude_1e7/longitude_1e7 (1e-7 degree units) down to
 * 1e-4 degree resolution (~11m) and packs each as a signed 24-bit
 * big-endian value into an 8-byte OP_GNSS_POS frame addressed to CDH.
 * Does nothing if the current nav fix is not valid.
 *
 * Packs lat/lon as scaled int24 for decent resolution in 8 bytes:
 *   [0..2] latitude  × 1e4, signed 24-bit big-endian  (~11m resolution)
 *   [3..5] longitude × 1e4, signed 24-bit big-endian
 */
static void send_gnss_position(orion_nav_data_t *nav)
{
    /* lat/lon are in 1e-7 degrees; divide by 1000 → 1e-4 degrees */
    int32_t lat_1e4 = nav->latitude_1e7  / 1000;
    int32_t lon_1e4 = nav->longitude_1e7 / 1000;

    /* lat/lon: 24-bit big-endian each; bytes 6..7 unused (0) */
    int err = can_send8(can_dev, NODE_ID, NODE_CDH, OP_GNSS_POS, PRIO_MED,
        (lat_1e4 >> 16) & 0xFF,
        (lat_1e4 >>  8) & 0xFF,
        (lat_1e4      ) & 0xFF,
        (lon_1e4 >> 16) & 0xFF,
        (lon_1e4 >>  8) & 0xFF,
        (lon_1e4      ) & 0xFF,
        0,
        0
    );
    if (err) {
        LOG_WRN("Failed to send OP_GNSS_POS: %d", err);
    }

    double lat = ORION_DEG_FROM_1E7(nav->latitude_1e7);
    double lon = ORION_DEG_FROM_1E7(nav->longitude_1e7);
    LOG_INF("TX POS  lat=%.6f lon=%.6f", lat, lon);
}


/**
 * @brief Send one ECEF velocity component to CDH in its own telemetry frame.
 * @param v_data Velocity component in cm/s (signed 32-bit).
 * @param op     Which axis: OP_GNSS_VX, OP_GNSS_VY or OP_GNSS_VZ.
 * @param seq    Sequence number shared by the VX/VY/VZ frames of one nav
 *               snapshot, so CDH can tell whether the three axes belong
 *               together.
 *
 * Frame layout:
 *   [0..3] velocity, signed 32-bit big-endian (cm/s)
 *   [4] seq (uint8_t, wraps 255 -> 0)
 *   [5..7] unused (0)
 */
static void send_gnss_velocity(int32_t v_data, uint16_t op, uint8_t seq)
{
    /* velocity: 32-bit big-endian in [0..3]; [4] = sequence number shared by
     * VX/VY/VZ from the same nav snapshot; [5..7] unused (0) */
    int err = can_send8(can_dev, NODE_ID, NODE_CDH, op, PRIO_MED,
        (v_data >> 24) & 0xFF,
        (v_data >> 16) & 0xFF,
        (v_data >> 8) & 0xFF,
        (v_data     ) & 0xFF,
        seq,
        0,
        0,
        0
    );
    if (err) {
        LOG_WRN("Failed to send velocity op=0x%03X: %d", op, err);
    }

    LOG_INF("TX VEL  op=0x%03X seq=%u v=%d cm/s", op, seq, v_data);
}

/**
 * @brief Send position and all three velocity components for the current
 *        nav fix.
 *
 * Does nothing if the nav data is not valid. The three velocity frames
 * share one sequence number (vel_seq), which increments once per call, so
 * CDH can detect a dropped or mixed-up velocity set. The counter is
 * compared for equality only, so its 8-bit wrap needs no special handling.
 */
static void send_gnss_master(void)
{
    static uint8_t vel_seq;  /* wraps 255 -> 0; CDH only compares for equality */
    orion_nav_data_t nav;
    orion_driver_get_nav(&gnss_driver, &nav);

    if (!nav.valid) {
        return;  /* don't send garbage */
    }

    send_gnss_position(&nav);
    send_gnss_velocity(nav.ecef_vx_cms, OP_GNSS_VX, vel_seq);
    send_gnss_velocity(nav.ecef_vy_cms, OP_GNSS_VY, vel_seq);
    send_gnss_velocity(nav.ecef_vz_cms, OP_GNSS_VZ, vel_seq);
    vel_seq++;
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
