/**
 * @file gps_test_app.c
 * @brief Orion B16 GNSS driver + CAN telemetry for UT-CORE bus.
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
#include "can_proto.h"

LOG_MODULE_REGISTER(gnss_app, CONFIG_LOG_DEFAULT_LEVEL);

/* ===================================================== */
/* ================= CAN PROTOCOL ====================== */
/* ===================================================== */

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

typedef struct {
    uint8_t  priority;
    uint8_t  src;
    uint8_t  dst;
    uint8_t  msg_class;
    uint8_t  dlc;
    uint8_t  data[8];
} can_packet_t;

/* ===================================================== */
/* ================= HW DEVICES ======================== */
/* ===================================================== */

static const struct device *const gpioa  = DEVICE_DT_GET(DT_NODELABEL(gpioa));
static const struct device *const can_dev = DEVICE_DT_GET(DT_NODELABEL(fdcan1));

/* CAN transceiver pins — adjust to your schematic */
#define PIN_SHDN    10
#define PIN_SILENT  9

/* GNSS driver instance */
static orion_driver_t gnss_driver;

/* CAN RX queue */
CAN_MSGQ_DEFINE(rxq, 16);

/* ===================================================== */
/* ================= CAN TX ============================ */
/* ===================================================== */

static void send_simple(uint8_t dst, uint8_t cls, uint8_t op, uint8_t val)
{
    struct can_frame f = {0};
    f.id    = CAN_ID_FULL(PRIO_LOW, NODE_ID, dst, cls);
    f.flags = CAN_FRAME_IDE;
    can_fill_payload(&f, NODE_ID, op, val, 0, 0, 0, 0, 0);
    can_send(can_dev, &f, K_NO_WAIT, NULL, NULL);
}

static void send_heartbeat(void)
{
    send_simple(CAN_BROADCAST, CLS_HEARTBEAT, OP_HEARTBEAT, 0x00);
    LOG_INF("TX heartbeat");
}

/*
 * send_gnss_soh() - State-of-health: fix status + SV count.
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

/*
 * send_gnss_position() - Position telemetry.
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

/* ===================================================== */
/* ================= CAN RX ============================ */
/* ===================================================== */

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

static void handle_heartbeat(const can_packet_t *pkt)
{
    LOG_INF("RX heartbeat from 0x%02X", pkt->src);
}

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

/* ===================================================== */
/* ================= CAN SETUP ========================= */
/* ===================================================== */

static void tcan3403_wakeup(void)
{
    gpio_pin_configure(gpioa, PIN_SHDN,   GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure(gpioa, PIN_SILENT,  GPIO_OUTPUT_INACTIVE);
    k_msleep(1);
    LOG_INF("TCAN3403 Awake");
}

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

/* ===================================================== */
/* ================= CAN RX THREAD ===================== */
/* ===================================================== */

#define CAN_RX_STACK_SIZE  1024
#define CAN_RX_PRIORITY    5

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

K_THREAD_DEFINE(can_rx_tid, CAN_RX_STACK_SIZE,
    can_rx_thread, NULL, NULL, NULL,
    CAN_RX_PRIORITY, 0, 0);

/* ===================================================== */
/* ============== GNSS NAV CALLBACK ==================== */
/* ===================================================== */

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

/* ===================================================== */
/* ================= MAIN ============================== */
/* ===================================================== */

/* Forward declaration for config recipe */
extern int orion_config_cubesat_default(orion_driver_t *drv, uint8_t rate_hz, bool save);

#define HEARTBEAT_INTERVAL_MS   1000
#define SOH_INTERVAL_MS         5000
#define POS_INTERVAL_MS         5000

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