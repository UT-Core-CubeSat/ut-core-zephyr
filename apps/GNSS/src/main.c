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
 * @brief Node IDs, priorities, and GNSS-specific opcodes used on the bus.
 * @{
 */

#define NODE_ID       GNSS_ID   /* 0x06 */
#define NODE_CDH      CDH_ID    /* 0x01 */

#define PRIO_HIGH  0
#define PRIO_MED   2
#define PRIO_LOW   4

/* GNSS-specific opcodes */
#define OP_GNSS_SOH        0x01   /* periodic state-of-health        */
#define OP_GNSS_POS        0x02   /* position telemetry               */
#define OP_QUERY_POS       0x61   /* CDH requests current position    */
#define OP_SET_UPDATE_RATE 0x62   /* CDH sets GNSS update rate        */

/**
 * @brief Decoded CAN frame with routing fields unpacked from the 29-bit ID.
 */
typedef struct {
    uint8_t  priority;
    uint8_t  src;
    uint8_t  dst;
    uint8_t  msg_class;
    uint8_t  dlc;
    uint8_t  data[8];
} can_packet_t;

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

/* CAN transceiver pins — adjust to your schematic */
#define PIN_SHDN    10
#define PIN_SILENT  9

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
 * @brief Build and send a single-opcode CAN frame with one data byte.
 * @param dst Destination node ID (or CAN_BROADCAST).
 * @param cls Message class.
 * @param op  Opcode.
 * @param val Single data byte payload.
 */
static void send_simple(uint8_t dst, uint8_t cls, uint8_t op, uint8_t val)
{
    struct can_frame f = {0};
    f.id    = CAN_ID_FULL(PRIO_LOW, NODE_ID, dst, cls);
    f.flags = CAN_FRAME_IDE;
    can_fill_payload(&f, NODE_ID, op, val, 0, 0, 0, 0, 0);
    can_send(can_dev, &f, K_NO_WAIT, NULL, NULL);
}

/**
 * @brief Broadcast a CLS_HEARTBEAT frame announcing this node is alive.
 *
 * Sent periodically from main()'s loop at HEARTBEAT_INTERVAL_MS.
 */
static void send_heartbeat(void)
{
    send_simple(CAN_BROADCAST, CLS_HEARTBEAT, OP_HEARTBEAT, 0x00);
    LOG_INF("TX heartbeat");
}

/**
 * @brief Send a state-of-health frame: fix status, SV count, and DOP.
 *
 * Packs fix mode, satellite count, HDOP/PDOP (scaled by 10 and clamped to
 * 255), configured update rate, and a driver-running status flag into an
 * 8-byte CLS_HEALTH frame addressed to CDH.
 *
 * Payload:
 *   [0] src
 *   [1] OP_GNSS_SOH
 *   [2] fix_mode  (0=none, 2=2D, 3=3D, 4=3D+DGPS)
 *   [3] sv_count
 *   [4] HDOP * 10  (clamped to 255)
 *   [5] PDOP * 10  (clamped to 255)
 *   [6] update_rate_hz
 *   [7] status flags (bit0 = driver running)
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

    struct can_frame f = {0};
    f.id    = CAN_ID_FULL(PRIO_LOW, NODE_ID, NODE_CDH, CLS_HEALTH);
    f.flags = CAN_FRAME_IDE;
    can_fill_payload(&f, NODE_ID,
        OP_GNSS_SOH,
        nav.fix_mode,
        nav.sv_count,
        (uint8_t)hdop10,
        (uint8_t)pdop10,
        gnss_driver.cfg.update_rate_hz,
        flags
    );
    can_send(can_dev, &f, K_NO_WAIT, NULL, NULL);

    LOG_INF("TX SOH  fix=%u sv=%u hdop=%u pdop=%u",
            nav.fix_mode, nav.sv_count, (unsigned)hdop10, (unsigned)pdop10);
}

/**
 * @brief Send a position telemetry frame with lat/lon packed as scaled
 *        24-bit integers.
 *
 * Converts nav.latitude_1e7/longitude_1e7 (1e-7 degree units) down to
 * 1e-4 degree resolution (~11m) and packs each as a signed 24-bit
 * big-endian value into an 8-byte CLS_TELEMETRY frame addressed to CDH.
 * Does nothing if the current nav fix is not valid.
 *
 * Packs lat/lon as scaled int24 for decent resolution in 8 bytes:
 *   [0] src
 *   [1] OP_GNSS_POS
 *   [2..4] latitude  × 1e4, signed 24-bit big-endian  (~11m resolution)
 *   [5..7] longitude × 1e4, signed 24-bit big-endian
 */
static void send_gnss_position(void)
{
    orion_nav_data_t nav;
    orion_driver_get_nav(&gnss_driver, &nav);

    if (!nav.valid) {
        return;  /* don't send garbage */
    }

    /* lat/lon are in 1e-7 degrees; divide by 1000 → 1e-4 degrees */
    int32_t lat_1e4 = nav.latitude_1e7  / 1000;
    int32_t lon_1e4 = nav.longitude_1e7 / 1000;

    struct can_frame f = {0};
    f.id    = CAN_ID_FULL(PRIO_MED, NODE_ID, NODE_CDH, CLS_TELEMETRY);
    f.flags = CAN_FRAME_IDE;
    f.dlc   = 8;
    f.data[0] = NODE_ID;
    f.data[1] = OP_GNSS_POS;
    /* lat: 24-bit big-endian */
    f.data[2] = (lat_1e4 >> 16) & 0xFF;
    f.data[3] = (lat_1e4 >>  8) & 0xFF;
    f.data[4] = (lat_1e4      ) & 0xFF;
    /* lon: 24-bit big-endian */
    f.data[5] = (lon_1e4 >> 16) & 0xFF;
    f.data[6] = (lon_1e4 >>  8) & 0xFF;
    f.data[7] = (lon_1e4      ) & 0xFF;

    can_send(can_dev, &f, K_NO_WAIT, NULL, NULL);

    double lat = ORION_DEG_FROM_1E7(nav.latitude_1e7);
    double lon = ORION_DEG_FROM_1E7(nav.longitude_1e7);
    LOG_INF("TX POS  lat=%.6f lon=%.6f", lat, lon);
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
 * @brief Unpack a raw CAN frame's 29-bit extended ID and payload into a
 *        can_packet_t.
 * @param f   Raw CAN frame as received from the driver.
 * @param pkt Output decoded packet.
 */
static void can_decode(const struct can_frame *f, can_packet_t *pkt)
{
    uint32_t id    = f->id;
    pkt->priority  = (id >> 26) & 0x07;
    pkt->src       = (id >> 14) & 0xFF;
    pkt->dst       = (id >>  6) & 0xFF;
    pkt->msg_class =  id        & 0x3F;
    pkt->dlc       = f->dlc;
    memcpy(pkt->data, f->data, f->dlc);
}

/**
 * @brief Handle an incoming CLS_COMMAND frame from CDH.
 * @param pkt Decoded CAN packet; data[1] is the opcode.
 *
 * OP_QUERY_POS replies immediately with the current position plus a
 * command-response ack. OP_SET_UPDATE_RATE (data[2] = requested rate in
 * Hz) attempts to apply the new rate via the GNSS driver and replies with
 * the accepted rate on success or 0x00 on failure. Unknown opcodes are
 * logged and dropped.
 *
 * @bug handle_command() reads pkt->data[1]/[2] without checking pkt->dlc
 *      first — a malformed or truncated command frame could read past
 *      the valid portion of the payload.
 */
static void handle_command(const can_packet_t *pkt)
{
    uint8_t opcode = pkt->data[1];

    switch (opcode) {
    case OP_QUERY_POS:
        /* CDH asked for current position — reply immediately */
        send_gnss_position();
        send_simple(pkt->src, CLS_CMD_RESP, OP_QUERY_POS, 0x01);
        LOG_INF("RX query position from 0x%02X", pkt->src);
        break;

    case OP_SET_UPDATE_RATE: {
        /* data[2] = new rate in Hz */
        uint8_t rate = pkt->data[2];
        int rc = orion_driver_set_update_rate(&gnss_driver, rate);
        uint8_t ack_val = (rc == 1) ? rate : 0x00;
        send_simple(pkt->src, CLS_CMD_RESP, OP_SET_UPDATE_RATE, ack_val);
        LOG_INF("RX set update rate=%u Hz → %s", rate, rc == 1 ? "ACK" : "NACK");
        break;
    }

    default:
        LOG_WRN("Unknown command opcode: 0x%02X", opcode);
        break;
    }
}

/**
 * @brief Handle a received CLS_HEARTBEAT frame.
 * @param pkt Decoded CAN packet.
 *
 * Currently only logs the sender; no liveness table is maintained here.
 */
static void handle_heartbeat(const can_packet_t *pkt)
{
    LOG_INF("RX heartbeat from 0x%02X", pkt->src);
}

/**
 * @brief Route a decoded CAN packet to its message-class handler.
 * @param pkt Decoded CAN packet.
 *
 * Dispatches CLS_HEARTBEAT to handle_heartbeat() and CLS_COMMAND to
 * handle_command(). Unrecognized classes are logged as warnings and
 * dropped.
 */
static void can_dispatch(const can_packet_t *pkt)
{
    switch (pkt->msg_class) {
    case CLS_HEARTBEAT: handle_heartbeat(pkt);  break;
    case CLS_COMMAND:   handle_command(pkt);     break;
    default:
        LOG_WRN("Unhandled class: %d from 0x%02X", pkt->msg_class, pkt->src);
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

/**
 * @brief Bring the TCAN3403 transceiver out of shutdown/silent mode.
 *
 * Drives PIN_SHDN and PIN_SILENT inactive, then waits 1 ms for the
 * transceiver to become active.
 */
static void tcan3403_wakeup(void)
{
    gpio_pin_configure(gpioa, PIN_SHDN,   GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure(gpioa, PIN_SILENT,  GPIO_OUTPUT_INACTIVE);
    k_msleep(1);
    LOG_INF("TCAN3403 Awake");
}

/**
 * @brief Configure bitrate/mode, start the CAN controller, and install RX
 *        filters for this node's address and broadcast.
 *
 * Sets 500 kbit/s normal mode, then adds two extended-ID filters routing
 * matching frames into rxq: one for frames addressed to NODE_ID, one for
 * CAN_BROADCAST.
 */
static void can_setup(void)
{
    if (!device_is_ready(can_dev)) {
        LOG_ERR("CAN not ready");
        return;
    }

    can_set_bitrate(can_dev, 500000);
    can_set_mode(can_dev, CAN_MODE_NORMAL);
    can_start(can_dev);

    const struct can_filter to_me = {
        .id    = CAN_DST(NODE_ID),
        .mask  = CAN_DST_MASK_29,
        .flags = CAN_FILTER_IDE
    };
    const struct can_filter bcast = {
        .id    = CAN_DST(CAN_BROADCAST),
        .mask  = CAN_DST_MASK_29,
        .flags = CAN_FILTER_IDE
    };

    can_add_rx_filter_msgq(can_dev, &rxq, &to_me);
    can_add_rx_filter_msgq(can_dev, &rxq, &bcast);
    LOG_INF("CAN initialized (29-bit extended)");
}

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
            can_decode(&frame, &pkt);
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
 * Logs fix mode, position, altitude, and satellite count when the fix is
 * valid; logs a warning and returns early otherwise.
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

    LOG_INF("FIX %s | lat=%.6f lon=%.6f alt=%.2fm | SVs=%u",
            fix_str, lat, lon, alt, nav->sv_count);
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
    can_setup();

    LOG_INF("Running — GNSS + CAN active");

    /* ── Main loop ── */
    int64_t last_hb  = k_uptime_get();
    int64_t last_soh = k_uptime_get();
    int64_t last_pos = k_uptime_get();

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
            send_gnss_position();

            /* Print detailed nav + stats on the same 5s cadence */
            orion_nav_data_t nav;
            orion_driver_get_nav(&gnss_driver, &nav);
            if (nav.valid) {
                double lat = ORION_DEG_FROM_1E7(nav.latitude_1e7);
                double lon = ORION_DEG_FROM_1E7(nav.longitude_1e7);
                double alt = ORION_M_FROM_CM(nav.msl_alt_cm);
                LOG_INF("NAV fix=%u sv=%u lat=%.6f lon=%.6f alt=%.2fm",
                        nav.fix_mode, nav.sv_count, lat, lon, alt);
            } else {
                LOG_INF("No valid fix yet.");
            }

            uint32_t rx_b, tx_b, nav_cnt, f_ok, f_err;
            orion_driver_get_stats(&gnss_driver, &rx_b, &tx_b, &nav_cnt, &f_ok, &f_err);
            LOG_INF("Stats: RX=%u TX=%u nav=%u ok=%u err=%u",
                    rx_b, tx_b, nav_cnt, f_ok, f_err);

            last_pos = now;
        }

        k_msleep(100);
    }

    return 0;
}

/* @} */
