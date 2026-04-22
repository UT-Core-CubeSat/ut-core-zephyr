/*
 * UT-CORE GNSS Node — Threaded Flight Architecture
 *
 * Hardware: STM32U5 microcontroller, Zephyr RTOS.
 * GNSS:    Orion B16 on LPUART1 (PC1 TX / PC0 RX), 115200 baud, binary protocol.
 * Bus:     FDCAN1 at 500 kbit/s, 29-bit extended IDs.
 * Console: USART3 (PC4 TX / PC5 RX), 115200 baud.
 * Sensors: Up to 2 I2C temperature sensors (TMP1xx-family) on I2C1 (PB6/PB7).
 *
 * Thread overview (highest to lowest priority):
 *   PRIO 0  watchdog_thread    - kicks hardware watchdog; must never starve
 *   PRIO 1  can_rx_thread      - drains hardware FIFO into SW queue
 *   PRIO 2  can_process_thread - decodes & dispatches CAN frames
 *   PRIO 3  scheduler_thread   - 1 Hz heartbeat + periodic GNSS telemetry push
 *   PRIO 4  soh_thread         - reads temps, monitors GNSS receiver health
 *   (internal) orion_parser    - driver's own UART RX parser thread (prio 5)
 *
 * CAN telemetry pushed to CDH every GNSS_TELEM_MS:
 *   OP_GNSS_POS1  — lat_1e7 + fix_mode + sv_count
 *   OP_GNSS_POS2  — lon_1e7 + hdop + pdop
 *   OP_GNSS_ALT   — msl_alt_cm + gps_week
 *   OP_GNSS_TIME  — tow_cs (centiseconds)
 *   OP_GNSS_VEL   — ECEF vx/vy/vz for ADCS
 *
 * LED usage:
 *   led0 — pulses briefly on every received CAN frame (activity indicator)
 *   led1 — GNSS fix status: ON = 3D fix, OFF = no fix
 *   led2 — controlled remotely via OP_SET_LED ground command
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <stm32u5xx.h>
#include <stdbool.h>
#include <string.h>

#include "can_proto.h"
#include "orion_b16_driver.h"
#include "orion_b16_protocol.h"
#include "orion_b16_messages.h"

LOG_MODULE_REGISTER(gnss_node, LOG_LEVEL_INF);

/* ================= THREAD PRIORITIES ================= */

#define PRIO_WATCHDOG   0
#define PRIO_CAN_RX     1
#define PRIO_CAN_PROC   2
#define PRIO_SCHED      3
#define PRIO_SOH        4

#define STACK_SIZE      1024
#define GNSS_STACK_SIZE 2048   /* Orion driver uses doubles internally */

/* ================= HARDWARE / PROTOCOL DEFINITIONS ================= */

#define NODE_ID     GNSS_ID    /* 0x06 from can_proto.h */
#define DST_ME      NODE_ID
#define PRIO_LOW    3

#define CAN_NODE     DT_NODELABEL(fdcan1)
#define GNSS_UART    DT_NODELABEL(lpuart1)
#define I2C_BUS_NODE DT_NODELABEL(i2c1)
#define GPIOA_NODE   DT_NODELABEL(gpioa)
#define LED0_NODE    DT_ALIAS(led0)
#define LED1_NODE    DT_ALIAS(led1)
#define LED2_NODE    DT_ALIAS(led2)

#define PIN_SILENT   9     /* TCAN3403 silent mode pin (PA9)  */
#define PIN_SHDN     10    /* TCAN3403 shutdown pin (PA10)    */
#define PIN_ROLE     6     /* MCU role strap pin (PA6)        */

#define WATCHDOG_KICK_MS    15
#define CAN_PROC_Q_LEN      16
#define GNSS_TELEM_MS        5000   /* Push position telemetry every 5 s */
#define HEARTBEAT_MS         1000

/* TMP1xx I2C addresses — adjust to match your GNSS board strapping. */
static const uint8_t temp_addrs[] = { 0x48, 0x49 };
#define NUM_SENSORS ((uint8_t)(sizeof(temp_addrs) / sizeof(temp_addrs[0])))
#define TEMP_REG 0x00

/* ================= GNSS-SPECIFIC CAN OPCODES ================= */
/*
 * These extend the opcode space in can_proto.h for GNSS telemetry frames.
 * Each is sent as class = CLS_TELEMETRY, dst = CAN_BROADCAST.
 *
 * Payload layout (6 usable bytes after src + opcode):
 *   OP_GNSS_POS1:  lat_1e7(4, BE) + fix_mode(1) + sv_count(1)
 *   OP_GNSS_POS2:  lon_1e7(4, BE) + hdop_100(2, BE)
 *   OP_GNSS_ALT:   msl_alt_cm(4, BE) + gps_week(2, BE)
 *   OP_GNSS_TIME:  tow_cs(4, BE) + pdop_100(2, BE)
 *   OP_GNSS_VEL:   ecef_vx(2, BE, i16) + vy(2) + vz(2) [cm/s]
 *   OP_GNSS_TEMP:  temp0_q4(2, BE) + temp1_q4(2, BE) + reserved(2)
 *
 * TODO: migrate these into common/can_proto.h once the protocol is frozen.
 */
#define OP_GNSS_POS1    0x60
#define OP_GNSS_POS2    0x61
#define OP_GNSS_ALT     0x62
#define OP_GNSS_TIME    0x63
#define OP_GNSS_VEL     0x64
#define OP_GNSS_TEMP    0x66

/* ================= DEVICE HANDLES ================= */

static const struct device      *can_dev  = DEVICE_DT_GET(CAN_NODE);
static const struct device      *gpioa    = DEVICE_DT_GET(GPIOA_NODE);
static const struct device      *i2c_bus  = DEVICE_DT_GET(I2C_BUS_NODE);
static const struct gpio_dt_spec led0     = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec led1     = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
static const struct gpio_dt_spec led2     = GPIO_DT_SPEC_GET(LED2_NODE, gpios);

CAN_MSGQ_DEFINE(rxq, 16);

/* ================= DATA STRUCTURES ================= */

/*
 * gnss_state_t — latest position fix from the Orion B16.
 *
 * Written by the nav callback (runs in the driver's parser thread).
 * Read by scheduler_thread to build CAN telemetry frames.
 * Protected by gnss_mutex.
 */
typedef struct {
    int32_t  lat_1e7;
    int32_t  lon_1e7;
    uint32_t msl_alt_cm;
    uint8_t  fix_mode;
    uint8_t  sv_count;
    uint16_t hdop;
    uint16_t pdop;
    uint16_t gps_week;
    uint32_t tow_cs;
    int32_t  ecef_vx_cms;
    int32_t  ecef_vy_cms;
    int32_t  ecef_vz_cms;
    bool     valid;
    uint32_t last_fix_ms;
} gnss_state_t;

static gnss_state_t gnss_state;
static struct k_mutex gnss_mutex;

struct temp_sample {
    uint8_t addr;
    int16_t temp_q4;
    int     status;
};

struct temp_telemetry {
    uint32_t t_ms;
    struct temp_sample s[NUM_SENSORS];
};

typedef struct {
    uint8_t  priority;
    uint8_t  msg_class;
    uint8_t  src;
    uint8_t  dst;
    uint8_t  data[8];
    uint8_t  dlc;
} can_packet_t;

/* ================= MODE STATE MACHINE ================= */

typedef enum {
    MODE_SETUP,
    MODE_STANDARD,
    MODE_MISSION,
    MODE_ERROR
} gnss_mode_t;

volatile gnss_mode_t current_mode = MODE_SETUP;

/* ================= ORION B16 DRIVER INSTANCE ================= */

static orion_driver_t orion_drv;
extern int orion_config_cubesat_default(orion_driver_t *drv, uint8_t rate_hz, bool save);

/* ================= THREAD STACKS ================= */

K_THREAD_STACK_DEFINE(watchdog_stack,  STACK_SIZE);
K_THREAD_STACK_DEFINE(can_stack,       STACK_SIZE);
K_THREAD_STACK_DEFINE(can_proc_stack,  STACK_SIZE);
K_THREAD_STACK_DEFINE(sched_stack,     GNSS_STACK_SIZE);
K_THREAD_STACK_DEFINE(soh_stack,       STACK_SIZE);

static struct k_thread watchdog_td;
static struct k_thread can_td;
static struct k_thread can_proc_td;
static struct k_thread sched_td;
static struct k_thread soh_td;

K_MSGQ_DEFINE(can_proc_q, sizeof(struct can_frame), CAN_PROC_Q_LEN, 4);

/* ================= FORWARD DECLARATIONS ================= */

static void handle_heartbeat(const can_packet_t *pkt);
static void handle_command(const can_packet_t *pkt);
static void handle_cmd_response(const can_packet_t *pkt);
static void handle_telemetry(const can_packet_t *pkt);
static void handle_health(const can_packet_t *pkt);
static void can_dispatch(const can_packet_t *pkt);

/* ===================================================== */
/* ================= UTILITY FUNCTIONS ================== */
/* ===================================================== */

static void read_uid(uint32_t uid[3])
{
    uid[0] = *(uint32_t *)(UID_BASE + 0x0);
    uid[1] = *(uint32_t *)(UID_BASE + 0x4);
    uid[2] = *(uint32_t *)(UID_BASE + 0x8);
}

static inline void led_can_activity(void)
{
    gpio_pin_set_dt(&led0, 1);
    k_busy_wait(1000);
    gpio_pin_set_dt(&led0, 0);
}

static void leds_init(void)
{
    gpio_pin_configure_dt(&led0, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led1, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led2, GPIO_OUTPUT_INACTIVE);
}

static int temp_telemetry_read_all(const struct device *bus,
                                   struct temp_telemetry *out)
{
    if (!out || !bus) {
        return 0;
    }

    out->t_ms = k_uptime_get_32();
    int ok = 0;

    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        uint8_t reg    = TEMP_REG;
        uint8_t buf[2] = {0};

        out->s[i].addr = temp_addrs[i];
        int ret = i2c_write_read(bus, temp_addrs[i], &reg, 1, buf, 2);
        out->s[i].status = ret;

        if (ret == 0) {
            int16_t raw = (int16_t)((buf[0] << 8) | buf[1]);
            out->s[i].temp_q4 = (int16_t)(raw >> 4);
            ok++;
        } else {
            out->s[i].temp_q4 = 0;
        }
    }
    return ok;
}

/* ===================================================== */
/* ================= CAN FUNCTIONS ====================== */
/* ===================================================== */

static void send_raw(uint8_t dst, uint8_t cls, uint8_t op,
                     uint8_t p2, uint8_t p3, uint8_t p4,
                     uint8_t p5, uint8_t p6, uint8_t p7)
{
    struct can_frame f = {0};
    f.id = CAN_ID_FULL(PRIO_LOW, NODE_ID, dst, cls);
    can_fill_payload(&f, NODE_ID, op, p2, p3, p4, p5, p6, p7);
    can_send(can_dev, &f, K_NO_WAIT, NULL, NULL);
}

static void send_simple(uint8_t dst, uint8_t cls, uint8_t op, uint8_t val)
{
    send_raw(dst, cls, op, val, 0, 0, 0, 0, 0);
}

static void tcan3403_wakeup(void)
{
    gpio_pin_configure(gpioa, PIN_SHDN,   GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure(gpioa, PIN_SILENT, GPIO_OUTPUT_INACTIVE);
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
        .id    = CAN_DST(DST_ME),
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

    LOG_INF("CAN Initialized (29-bit extended, node=0x%02X)", NODE_ID);
}

static void can_decode(const struct can_frame *f, can_packet_t *pkt)
{
    uint32_t id   = f->id;
    pkt->priority  = (id >> 26) & 0x07;
    pkt->src       = (id >> 14) & 0xFF;
    pkt->dst       = (id >>  6) & 0xFF;
    pkt->msg_class =  id        & 0x3F;
    pkt->dlc       = f->dlc;
    memcpy(pkt->data, f->data, f->dlc);
}

static void can_dispatch(const can_packet_t *pkt)
{
    switch (pkt->msg_class) {
    case CLS_HEARTBEAT: handle_heartbeat(pkt);    break;
    case CLS_COMMAND:   handle_command(pkt);      break;
    case CLS_CMD_RESP:  handle_cmd_response(pkt); break;
    case CLS_TELEMETRY: handle_telemetry(pkt);    break;
    case CLS_HEALTH:    handle_health(pkt);       break;
    default:
        LOG_WRN("Unhandled class: %d from 0x%02X", pkt->msg_class, pkt->src);
        break;
    }
}

/* ===================================================== */
/* ================= MESSAGE HANDLERS =================== */
/* ===================================================== */

static void handle_heartbeat(const can_packet_t *pkt)
{
    LOG_DBG("Heartbeat from 0x%02X", pkt->src);
}

static void handle_cmd_response(const can_packet_t *pkt)
{
    LOG_INF("CmdResp from 0x%02X", pkt->src);
}

static void handle_telemetry(const can_packet_t *pkt)
{
    LOG_DBG("Telemetry from 0x%02X", pkt->src);
}

static void handle_health(const can_packet_t *pkt)
{
    LOG_DBG("Health from 0x%02X", pkt->src);
}

static void handle_command(const can_packet_t *pkt)
{
    uint8_t opcode = pkt->data[1];
    uint8_t val    = pkt->data[2];

    switch (opcode) {
    case OP_SET_MODE:
        LOG_INF("Mode change -> %d", val);
        current_mode = val;
        break;

    case OP_REBOOT:
        LOG_WRN("Reboot requested");
        /* NVIC_SystemReset(); */
        break;

    case OP_SET_LED:
        LOG_INF("LED command: led2 -> %d", val);
        gpio_pin_set_dt(&led2, val);
        break;

    default:
        LOG_WRN("Unknown opcode: 0x%02X", opcode);
        break;
    }
}

/* ===================================================== */
/* ================= GNSS TELEMETRY ===================== */
/* ===================================================== */

/*
 * Big-endian packing helpers for CAN payloads.
 */
static inline void pack_be32(uint8_t *dst, uint32_t val)
{
    dst[0] = (uint8_t)(val >> 24);
    dst[1] = (uint8_t)(val >> 16);
    dst[2] = (uint8_t)(val >>  8);
    dst[3] = (uint8_t)(val);
}

static inline void pack_be16(uint8_t *dst, uint16_t val)
{
    dst[0] = (uint8_t)(val >> 8);
    dst[1] = (uint8_t)(val);
}

/*
 * send_gnss_telemetry() — pack the latest fix into CAN frames and broadcast.
 *
 * Sends 5 frames per cycle:
 *   POS1: position + fix quality
 *   POS2: position + DOP
 *   ALT:  altitude + GPS week
 *   TIME: time-of-week + PDOP
 *   VEL:  ECEF velocity (for ADCS)
 */
static void send_gnss_telemetry(void)
{
    gnss_state_t snap;

    k_mutex_lock(&gnss_mutex, K_FOREVER);
    memcpy(&snap, &gnss_state, sizeof(snap));
    k_mutex_unlock(&gnss_mutex);

    struct can_frame f = {0};
    f.flags = CAN_FRAME_IDE;
    f.dlc   = 8;
    f.id    = CAN_ID_FULL(PRIO_LOW, NODE_ID, CAN_BROADCAST, CLS_TELEMETRY);

    /* Frame 1 — OP_GNSS_POS1: lat_1e7(4) + fix_mode(1) + sv_count(1) */
    f.data[0] = NODE_ID;
    f.data[1] = OP_GNSS_POS1;
    pack_be32(&f.data[2], (uint32_t)snap.lat_1e7);
    f.data[6] = snap.fix_mode;
    f.data[7] = snap.sv_count;
    can_send(can_dev, &f, K_NO_WAIT, NULL, NULL);

    /* Frame 2 — OP_GNSS_POS2: lon_1e7(4) + hdop(2) */
    f.data[1] = OP_GNSS_POS2;
    pack_be32(&f.data[2], (uint32_t)snap.lon_1e7);
    pack_be16(&f.data[6], snap.hdop);
    can_send(can_dev, &f, K_NO_WAIT, NULL, NULL);

    /* Frame 3 — OP_GNSS_ALT: msl_alt_cm(4) + gps_week(2) */
    f.data[1] = OP_GNSS_ALT;
    pack_be32(&f.data[2], snap.msl_alt_cm);
    pack_be16(&f.data[6], snap.gps_week);
    can_send(can_dev, &f, K_NO_WAIT, NULL, NULL);

    /* Frame 4 — OP_GNSS_TIME: tow_cs(4) + pdop(2) */
    f.data[1] = OP_GNSS_TIME;
    pack_be32(&f.data[2], snap.tow_cs);
    pack_be16(&f.data[6], snap.pdop);
    can_send(can_dev, &f, K_NO_WAIT, NULL, NULL);

    /* Frame 5 — OP_GNSS_VEL: vx(2) + vy(2) + vz(2) [cm/s, signed] */
    f.data[1] = OP_GNSS_VEL;
    pack_be16(&f.data[2], (uint16_t)(int16_t)snap.ecef_vx_cms);
    pack_be16(&f.data[4], (uint16_t)(int16_t)snap.ecef_vy_cms);
    pack_be16(&f.data[6], (uint16_t)(int16_t)snap.ecef_vz_cms);
    can_send(can_dev, &f, K_NO_WAIT, NULL, NULL);
}

/*
 * send_temp_telemetry() — broadcast local board temperatures over CAN.
 */
static void send_temp_telemetry(const struct temp_telemetry *t)
{
    struct can_frame f = {0};
    f.flags = CAN_FRAME_IDE;
    f.dlc   = 8;
    f.id    = CAN_ID_FULL(PRIO_LOW, NODE_ID, CAN_BROADCAST, CLS_HEALTH);

    f.data[0] = NODE_ID;
    f.data[1] = OP_GNSS_TEMP;

    for (uint8_t i = 0; i < NUM_SENSORS && i < 3; i++) {
        if (t->s[i].status == 0) {
            pack_be16(&f.data[2 + i * 2], (uint16_t)t->s[i].temp_q4);
        }
    }

    can_send(can_dev, &f, K_NO_WAIT, NULL, NULL);
}

/* ===================================================== */
/* ================= ORION B16 CALLBACK ================= */
/* ===================================================== */

/*
 * on_nav_update() — called from the Orion driver's parser thread every
 * time a binary Navigation Data (0xA8) message is decoded.
 *
 * Copies the decoded PVT into gnss_state under mutex so the scheduler
 * thread can read it safely.
 */
static void on_nav_update(const orion_nav_data_t *nav, void *user)
{
    (void)user;

    k_mutex_lock(&gnss_mutex, K_FOREVER);

    gnss_state.fix_mode    = nav->fix_mode;
    gnss_state.sv_count    = nav->sv_count;
    gnss_state.lat_1e7     = nav->latitude_1e7;
    gnss_state.lon_1e7     = nav->longitude_1e7;
    gnss_state.msl_alt_cm  = nav->msl_alt_cm;
    gnss_state.hdop        = nav->hdop;
    gnss_state.pdop        = nav->pdop;
    gnss_state.gps_week    = nav->gnss_week;
    gnss_state.tow_cs      = nav->tow_cs;
    gnss_state.ecef_vx_cms = nav->ecef_vx_cms;
    gnss_state.ecef_vy_cms = nav->ecef_vy_cms;
    gnss_state.ecef_vz_cms = nav->ecef_vz_cms;
    gnss_state.valid       = nav->valid;
    gnss_state.last_fix_ms = k_uptime_get_32();

    k_mutex_unlock(&gnss_mutex);

    /* Update fix LED: solid ON for 3D fix */
    bool has_3d = (nav->fix_mode == ORION_FIX_3D ||
                   nav->fix_mode == ORION_FIX_3D_DGPS);
    gpio_pin_set_dt(&led1, has_3d ? 1 : 0);
}

/* ===================================================== */
/* ================= SETUP MODE ========================= */
/* ===================================================== */

static void setup_mode_init(void)
{
    LOG_INF("Entering SETUP MODE");

    /* 1. Read and log hardware UID */
    uint32_t uid[3];
    read_uid(uid);
    printk("UID: %08X-%08X-%08X\n", uid[0], uid[1], uid[2]);

    gpio_pin_configure(gpioa, PIN_ROLE, GPIO_INPUT);
    int role_pin = gpio_pin_get(gpioa, PIN_ROLE);
    printk("PA6 Role Pin: %d\n", role_pin);

    /* 2. Wake CAN transceiver */
    tcan3403_wakeup();

    /* 3. Init FDCAN */
    can_setup();

    /* 4. Init Orion B16 GNSS driver */
    const struct device *gnss_uart = DEVICE_DT_GET(GNSS_UART);
    if (!device_is_ready(gnss_uart)) {
        LOG_ERR("GNSS UART (lpuart1) not ready!");
        current_mode = MODE_ERROR;
        return;
    }

    orion_driver_cfg_t cfg = ORION_DRIVER_CFG_DEFAULTS;
    cfg.uart_dev = gnss_uart;

    int rc = orion_driver_init(&orion_drv, &cfg);
    if (rc != 0) {
        LOG_ERR("Orion driver init failed: %d", rc);
        current_mode = MODE_ERROR;
        return;
    }

    orion_driver_register_nav_callback(&orion_drv, on_nav_update, NULL);

    rc = orion_driver_start(&orion_drv);
    if (rc != 0) {
        LOG_ERR("Orion driver start failed: %d", rc);
        current_mode = MODE_ERROR;
        return;
    }

    /* 5. Apply CubeSat configuration (constellation, power, masks) */
    orion_config_cubesat_default(&orion_drv, 1, false);

    LOG_INF("Setup complete — waiting 2 s for bus stabilisation");
    k_sleep(K_MSEC(2000));

    current_mode = MODE_STANDARD;
}

/* ===================================================== */
/* ================= THREADS ============================ */
/* ===================================================== */

void can_rx_thread(void *a, void *b, void *c)
{
    LOG_INF("can_rx_thread started");
    struct can_frame rx;

    while (1) {
        if (current_mode == MODE_STANDARD ||
            current_mode == MODE_MISSION)
        {
            if (k_msgq_get(&rxq, &rx, K_FOREVER) == 0) {
                led_can_activity();
                if (k_msgq_put(&can_proc_q, &rx, K_NO_WAIT) != 0) {
                    LOG_WRN("CAN proc queue full — frame dropped");
                }
            }
        } else {
            k_sleep(K_MSEC(200));
        }
    }
}

void can_process_thread(void *a, void *b, void *c)
{
    LOG_INF("can_process_thread started");
    struct can_frame frame;
    can_packet_t pkt;

    while (1) {
        if (k_msgq_get(&can_proc_q, &frame, K_FOREVER) == 0) {
            can_decode(&frame, &pkt);

            if (pkt.dst != NODE_ID && pkt.dst != CAN_BROADCAST) {
                continue;
            }

            can_dispatch(&pkt);
        }
    }
}

/*
 * scheduler_thread() — 1 Hz heartbeat + GNSS telemetry push.
 *
 * Heartbeat is broadcast every HEARTBEAT_MS.
 * GNSS telemetry is broadcast every GNSS_TELEM_MS.
 * Both use uptime deltas so jitter doesn't accumulate.
 */
void scheduler_thread(void *a, void *b, void *c)
{
    LOG_INF("scheduler_thread started");
    int64_t last_hb   = 0;
    int64_t last_telem = 0;

    while (1) {
        if (current_mode == MODE_STANDARD ||
            current_mode == MODE_MISSION)
        {
            int64_t now = k_uptime_get();

            /* 1 Hz heartbeat */
            if (now - last_hb >= HEARTBEAT_MS) {
                last_hb = now;
                send_simple(CAN_BROADCAST, CLS_HEARTBEAT, OP_HEARTBEAT, 0);
            }

            /* Periodic GNSS telemetry push */
            if (now - last_telem >= GNSS_TELEM_MS) {
                last_telem = now;
                send_gnss_telemetry();

                /* Log locally for debug */
                k_mutex_lock(&gnss_mutex, K_FOREVER);
                if (gnss_state.valid) {
                    double lat = ORION_DEG_FROM_1E7(gnss_state.lat_1e7);
                    double lon = ORION_DEG_FROM_1E7(gnss_state.lon_1e7);
                    LOG_INF("TELEM fix=%u sv=%u lat=%.6f lon=%.6f",
                            gnss_state.fix_mode, gnss_state.sv_count,
                            lat, lon);
                } else {
                    LOG_INF("TELEM: no fix (sv=%u)", gnss_state.sv_count);
                }
                k_mutex_unlock(&gnss_mutex);
            }
        }

        k_sleep(K_MSEC(50));
    }
}

/*
 * soh_thread() — State-of-Health: temperature reads + GNSS receiver monitoring.
 *
 * Reads local I2C temperature sensors and broadcasts via CAN.
 * Monitors GNSS fix age — if fix goes stale, logs a warning.
 */
void soh_thread(void *a, void *b, void *c)
{
    LOG_INF("soh_thread started");

    while (1) {
        if (current_mode == MODE_STANDARD ||
            current_mode == MODE_MISSION)
        {
            /* Read temperatures */
            struct temp_telemetry telem;
            int ok = temp_telemetry_read_all(i2c_bus, &telem);

            if (ok > 0) {
                send_temp_telemetry(&telem);
                for (uint8_t i = 0; i < NUM_SENSORS; i++) {
                    if (telem.s[i].status == 0) {
                        int32_t c = (telem.s[i].temp_q4 + 8) / 16;
                        LOG_DBG("Temp 0x%02X: %ldC", telem.s[i].addr, (long)c);
                    }
                }
            }

            /* Check GNSS fix staleness */
            k_mutex_lock(&gnss_mutex, K_FOREVER);
            uint32_t age = k_uptime_get_32() - gnss_state.last_fix_ms;
            bool was_valid = gnss_state.valid;
            k_mutex_unlock(&gnss_mutex);

            if (was_valid && age > 10000) {
                LOG_WRN("GNSS fix stale (%u ms old)", age);
            }

            /* Check Orion driver stats */
            uint32_t rx_b, tx_b, nav_cnt, f_ok, f_err;
            orion_driver_get_stats(&orion_drv,
                                   &rx_b, &tx_b, &nav_cnt, &f_ok, &f_err);
            if (f_err > 0) {
                LOG_WRN("Orion parse errors: %u (ok=%u)", f_err, f_ok);
            }
        }

        k_sleep(K_MSEC(2000));
    }
}

void watchdog_thread(void *a, void *b, void *c)
{
    LOG_INF("watchdog_thread started");

    while (1) {
        /* TODO: gpio_pin_toggle(gpioa, WATCHDOG_PIN); */
        LOG_DBG("watchdog kick");
        k_sleep(K_MSEC(WATCHDOG_KICK_MS));
    }
}

/* ===================================================== */
/* ================= MAIN =============================== */
/* ===================================================== */

int main(void)
{
    LOG_INF("UT-CORE GNSS Node Booting...");
    LOG_INF("  Node ID   : 0x%02X", NODE_ID);
    LOG_INF("  GNSS UART : LPUART1 PC1(TX) / PC0(RX)");
    LOG_INF("  Console   : USART3  PC4(TX) / PC5(RX)");
    LOG_INF("  CAN Bus   : FDCAN1  PA11(RX) / PA12(TX) @ 500 kbit/s");

    leds_init();
    k_mutex_init(&gnss_mutex);
    memset(&gnss_state, 0, sizeof(gnss_state));

    k_thread_create(&watchdog_td, watchdog_stack, STACK_SIZE,
                    watchdog_thread, NULL, NULL, NULL,
                    PRIO_WATCHDOG, 0, K_NO_WAIT);

    k_thread_create(&can_td, can_stack, STACK_SIZE,
                    can_rx_thread, NULL, NULL, NULL,
                    PRIO_CAN_RX, 0, K_NO_WAIT);

    k_thread_create(&can_proc_td, can_proc_stack, STACK_SIZE,
                    can_process_thread, NULL, NULL, NULL,
                    PRIO_CAN_PROC, 0, K_NO_WAIT);

    k_thread_create(&sched_td, sched_stack, GNSS_STACK_SIZE,
                    scheduler_thread, NULL, NULL, NULL,
                    PRIO_SCHED, 0, K_NO_WAIT);

    k_thread_create(&soh_td, soh_stack, STACK_SIZE,
                    soh_thread, NULL, NULL, NULL,
                    PRIO_SOH, 0, K_NO_WAIT);

    setup_mode_init();

    while (1) {
        k_sleep(K_FOREVER);
    }
}
