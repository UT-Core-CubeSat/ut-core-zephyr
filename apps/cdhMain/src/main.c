/*
 * UT-CORE CDH — Threaded Flight Architecture
 *
 * Hardware: STM32U5 microcontroller, Zephyr RTOS.
 * Bus:      FDCAN1 at 500 kbit/s, 29-bit extended IDs.
 * Sensors:  Up to 6 I2C temperature sensors (TMP1xx-family, 0x48-0x4D).
 *
 * Thread overview (highest to lowest priority):
 *   PRIO 0  watchdog_thread     — kicks hardware watchdog; must never starve
 *   PRIO 1  can_rx_thread       — drains hardware FIFO into SW queue
 *   PRIO 2  can_process_thread  — decodes & dispatches frames
 *   PRIO 3  scheduler_thread    — heartbeat + future mission scheduling
 *   PRIO 4  gnss_thread         — requests & stores GNSS position data
 *   PRIO 5  soh_thread          — reads temps & checks node-alive timeouts
 *   PRIO 6  telemetry_thread    — assembles & downlinks telemetry packets
 *
 * LED usage:
 *   led0 — pulses briefly on every received CAN frame (activity indicator)
 *   led1 — spare
 *   led2 — controlled remotely via OP_SET_LED ground command
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>
#include <stm32u5xx.h>
#include <stdbool.h>
#include <string.h>

#include "board_config.h"
#include "../../../common/can_proto.h"
#include "../../../common/temp_telemetry.h"

LOG_MODULE_REGISTER(cdh, LOG_LEVEL_INF);

/* ================= DEVICE HANDLES ================= */

static const struct device          *can_dev  = DEVICE_DT_GET(CAN_NODE);
static const struct device          *gpioa    = DEVICE_DT_GET(GPIOA_NODE);
static const struct device          *i2c_bus  = DEVICE_DT_GET(I2C_BUS_NODE);
static const struct gpio_dt_spec     led0     = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec     led1     = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
static const struct gpio_dt_spec     led2     = GPIO_DT_SPEC_GET(LED2_NODE, gpios);

/* Hardware-backed CAN RX queue — the driver ISR deposits frames here. */
CAN_MSGQ_DEFINE(rxq, 16);

/* SW queue between can_rx_thread and can_process_thread. */
K_MSGQ_DEFINE(can_proc_q, sizeof(struct can_frame), CAN_PROC_Q_LEN, 4);

/* ================= DATA STRUCTURES ================= */

/*
 * node_status_t — liveness record for one remote satellite subsystem.
 * Written by handle_heartbeat(), read by check_node_timeouts().
 */
typedef struct {
    bool    alive;
    int64_t last_seen_ms;
} node_status_t;

typedef struct {
    node_status_t cdh;
    node_status_t eps;
    node_status_t comms;
    node_status_t adcs;
    node_status_t motor;
    node_status_t gnss;
    node_status_t star;
    node_status_t solar;
} node_heartbeat_t;

static node_heartbeat_t node_alive = {0};

/*
 * can_packet_t — decoded CAN frame with routing fields unpacked from the ID.
 *
 * 29-bit ID layout (see can_proto.h):
 *   [28:26] priority | [21:14] src | [13:6] dst | [5:0] msg_class
 */
typedef struct {
    uint8_t  priority;
    uint8_t  msg_class;
    uint8_t  src;
    uint8_t  dst;
    uint16_t inst;
    uint8_t  data[8];
    uint8_t  dlc;
} can_packet_t;

/*
 * gnss_data_t — latest position fix received from the GNSS node.
 *
 * TODO: populate fields once the GNSS CAN telemetry format is defined.
 */
typedef struct {
    int32_t  lat_deg_e7;
    int32_t  lon_deg_e7;
    int32_t  alt_mm;
    uint32_t timestamp_ms;
    bool     valid;
} gnss_data_t;

static gnss_data_t gnss_latest = {0};

/* ================= MODE STATE MACHINE ================= */

typedef enum {
    MODE_SETUP,
    MODE_STANDARD,
    MODE_MISSION,
    MODE_ERROR
} cdh_mode_t;

volatile cdh_mode_t current_mode = MODE_SETUP;

/* ================= THREAD STACKS ================= */

K_THREAD_STACK_DEFINE(watchdog_stack,  STACK_SIZE);
K_THREAD_STACK_DEFINE(can_stack,       STACK_SIZE);
K_THREAD_STACK_DEFINE(sched_stack,     STACK_SIZE);
K_THREAD_STACK_DEFINE(soh_stack,       STACK_SIZE);
K_THREAD_STACK_DEFINE(can_proc_stack,  STACK_SIZE);
K_THREAD_STACK_DEFINE(gnss_stack,      STACK_SIZE);
K_THREAD_STACK_DEFINE(telemetry_stack, STACK_SIZE);

static struct k_thread watchdog_thread_data;
static struct k_thread can_thread_data;
static struct k_thread sched_thread_data;
static struct k_thread soh_thread_data;
static struct k_thread can_proc_thread_data;
static struct k_thread gnss_thread_data;
static struct k_thread telemetry_thread_data;

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
    LOG_INF("reading UID...");
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

static node_status_t *get_node_status(uint8_t node_id)
{
    switch (node_id) {
        case CDH_ID:   return &node_alive.cdh;
        case EPS_ID:   return &node_alive.eps;
        case COMMS_ID: return &node_alive.comms;
        case ADCS_ID:  return &node_alive.adcs;
        case MOTOR_ID: return &node_alive.motor;
        case GNSS_ID:  return &node_alive.gnss;
        case STAR_ID:  return &node_alive.star;
        case SOLAR_ID: return &node_alive.solar;
        default:       return NULL;
    }
}

/* ===================================================== */
/* ================= CAN FUNCTIONS ====================== */
/* ===================================================== */

static void send_simple(uint8_t dst, uint8_t cls, uint8_t op, uint8_t val)
{
    struct can_frame f = {0};
    f.id    = CAN_ID_FULL(PRIO_LOW, NODE_ID, dst, cls);
    f.flags = CAN_FRAME_IDE;
    can_fill_payload(&f, NODE_ID, op, val, 0, 0, 0, 0, 0);
    can_send(can_dev, &f, K_NO_WAIT, NULL, NULL);
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

    LOG_INF("Filter to_me:  id=0x%08X mask=0x%08X", to_me.id, to_me.mask);
    LOG_INF("Filter bcast:  id=0x%08X mask=0x%08X", bcast.id, bcast.mask);
    LOG_INF("CAN Initialized (29-bit extended)");
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
        case CLS_HEARTBEAT:  handle_heartbeat(pkt);    break;
        case CLS_COMMAND:    handle_command(pkt);      break;
        case CLS_CMD_RESP:   handle_cmd_response(pkt); break;
        case CLS_TELEMETRY:  handle_telemetry(pkt);    break;
        case CLS_HEALTH:     handle_health(pkt);       break;
        default:
            LOG_WRN("Unhandled class: %d", pkt->msg_class);
            break;
    }
}

/* ===================================================== */
/* ================= MESSAGE HANDLERS =================== */
/* ===================================================== */

static void handle_cmd_response(const can_packet_t *pkt)
{
    uint8_t op = pkt->data[1];

    switch (op) {
    case OP_SET_PWR_STATE: {
        uint8_t load_num = pkt->data[2];
        uint8_t state    = pkt->data[3];
        uint8_t status   = pkt->data[4];  /* 0x00 = success, 0x01 = error */
        if (status == 0x00) {
            LOG_INF("EPS confirmed: load %d %s",
                    load_num, state ? "ENABLED" : "DISABLED");
        } else {
            LOG_ERR("EPS rejected: load %d state=%d (error)", load_num, state);
        }
        break;
    }
    default:
        LOG_INF("CmdResp from node 0x%x, op=0x%02X", pkt->src, op);
        break;
    }
}

static void handle_telemetry(const can_packet_t *pkt)
{
    if (pkt->src == GNSS_ID && pkt->data[1] == 0x02 /* OP_GNSS_POS */) {
        /* Unpack signed 24-bit big-endian lat/lon (units: 1e-4 degrees) */
        int32_t lat_1e4 = (int32_t)((uint32_t)pkt->data[2] << 16 |
                                    (uint32_t)pkt->data[3] << 8  |
                                    (uint32_t)pkt->data[4]);
        if (lat_1e4 & 0x800000) lat_1e4 |= 0xFF000000;  /* sign-extend */

        int32_t lon_1e4 = (int32_t)((uint32_t)pkt->data[5] << 16 |
                                    (uint32_t)pkt->data[6] << 8  |
                                    (uint32_t)pkt->data[7]);
        if (lon_1e4 & 0x800000) lon_1e4 |= 0xFF000000;

        gnss_latest.lat_deg_e7  = lat_1e4 * 1000;  /* back to 1e-7 */
        gnss_latest.lon_deg_e7  = lon_1e4 * 1000;
        gnss_latest.timestamp_ms = k_uptime_get_32();
        gnss_latest.valid        = true;

        LOG_INF("GNSS fix: lat=%d lon=%d (1e-7 deg)",
                gnss_latest.lat_deg_e7, gnss_latest.lon_deg_e7);
        return;
    }

    LOG_INF("Telemetry from node 0x%x", pkt->src);
}

static void handle_health(const can_packet_t *pkt)
{
    LOG_INF("Health from node 0x%x", pkt->src);
}

/*
 * send_set_board_pwr() — Tell EPS to enable/disable a specific load switch.
 *
 * @load_num: Load number (1..12)
 * @state:    1 = enable, 0 = disable
 */
static void send_set_board_pwr(uint8_t load_num, uint8_t state)
{
    struct can_frame f = {0};
    f.id    = CAN_ID_FULL(PRIO_LOW, NODE_ID, EPS_ID, CLS_COMMAND);
    f.flags = CAN_FRAME_IDE;
    can_fill_payload(&f, NODE_ID, OP_SET_PWR_STATE, load_num, state, 0, 0, 0, 0);
    int ret = can_send(can_dev, &f, K_NO_WAIT, NULL, NULL);
    if (ret) {
        LOG_ERR("Failed to send SET_PWR_STATE (load=%d state=%d): %d",
                load_num, state, ret);
    } else {
        LOG_INF("TX SET_PWR_STATE load=%d state=%d -> EPS", load_num, state);
    }
}

static void handle_command(const can_packet_t *pkt)
{
    uint8_t opcode = pkt->data[1];
    uint8_t val    = pkt->data[2];

    switch (opcode) {
        case OP_SET_MODE:
            current_mode = val;
            break;

        case OP_REBOOT:
            /* NVIC_SystemReset(); — enable after watchdog sequencing confirmed */
            break;

        case OP_SET_LED:
            LOG_INF("LED command: led2 -> %d", val);
            gpio_pin_set_dt(&led2, val);
            break;

        case OP_SET_PWR_STATE:
            LOG_INF("Forwarding SET_PWR_STATE load=%d state=%d to EPS",
            pkt->data[2], pkt->data[3]);
            send_set_board_pwr(pkt->data[2], pkt->data[3]);
            break;

        default:
            LOG_WRN("Unknown opcode: %d", opcode);
            break;
    }
}

static void handle_heartbeat(const can_packet_t *pkt)
{
    node_status_t *node = get_node_status(pkt->src);

    if (node == NULL) {
        LOG_WRN("Heartbeat from unknown node: 0x%02X", pkt->src);
        return;
    }

    node->alive        = true;
    node->last_seen_ms = k_uptime_get();

    LOG_INF("Heartbeat from 0x%02X - alive", pkt->src);
}

/*
 * check_node_timeouts() — Warn about nodes that have gone silent.
 *
 * NOTE: SOLAR entry uses SOLAR_ID (not STAR_ID — this fixes the bug
 * from the original code).
 */
static void check_node_timeouts(void)
{
    int64_t now = k_uptime_get();

    struct {
        uint8_t        id;
        const char    *name;
        node_status_t *status;
    } nodes[] = {
        { EPS_ID,   "EPS",   &node_alive.eps   },
        { COMMS_ID, "COMMS", &node_alive.comms },
        { ADCS_ID,  "ADCS",  &node_alive.adcs  },
        { MOTOR_ID, "MOTOR", &node_alive.motor },
        { GNSS_ID,  "GNSS",  &node_alive.gnss  },
        { STAR_ID,  "STAR",  &node_alive.star  },
        { SOLAR_ID, "SOLAR", &node_alive.solar },
    };

    for (int i = 0; i < ARRAY_SIZE(nodes); i++) {
        node_status_t *n = nodes[i].status;

        if (!n->alive) {
            continue;
        }

        if ((now - n->last_seen_ms) > HEARTBEAT_TIMEOUT_MS) {
            LOG_WRN("Node %s (0x%02X) timed out!", nodes[i].name, nodes[i].id);
            n->alive = false;
        }
    }
}

/* ===================================================== */
/* ================= SETUP MODE ========================= */
/* ===================================================== */

static void setup_mode_init(void)
{
    LOG_INF("Entering SETUP MODE");

    uint32_t uid[3];
    read_uid(uid);
    printk("UID: %08X-%08X-%08X\n", uid[0], uid[1], uid[2]);

    gpio_pin_configure(gpioa, PIN_ROLE, GPIO_INPUT);
    int role_pin = gpio_pin_get(gpioa, PIN_ROLE);
    printk("PA6 Role Pin: %d\n", role_pin);

    bool uid_is_mcu1 = (uid[0] == UID1_WORD0 &&
                        uid[1] == UID1_WORD1 &&
                        uid[2] == UID1_WORD2);

    bool uid_is_mcu2 = (uid[0] == UID2_WORD0 &&
                        uid[1] == UID2_WORD1 &&
                        uid[2] == UID2_WORD2);

    bool pin_is_mcu1 = (role_pin == 0);
    bool pin_is_mcu2 = (role_pin == 1);

    if ((uid_is_mcu1 && pin_is_mcu1) ||
        (uid_is_mcu2 && pin_is_mcu2)) {
        LOG_INF("MCU Identity Verified");
    } else {
        LOG_ERR("UID and PA6 MISMATCH — trusting PA6 hardware pin.");
    }

    tcan3403_wakeup();
    can_setup();

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
                LOG_INF("RX: id=0x%08X dlc=%d d0=0x%02X d1=0x%02X",
                        rx.id, rx.dlc, rx.data[0], rx.data[1]);

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
            led_can_activity();
            can_decode(&frame, &pkt);

            if (pkt.dst != NODE_ID && pkt.dst != CAN_BROADCAST) {
                continue;
            }

            can_dispatch(&pkt);
        }
    }
}

void scheduler_thread(void *a, void *b, void *c)
{
    LOG_INF("scheduler_thread started");
    int64_t last_hb = 0;

    while (1) {
        if (current_mode == MODE_STANDARD ||
            current_mode == MODE_MISSION)
        {
            int64_t now = k_uptime_get();

            if (now - last_hb >= 1000) {
                last_hb = now;
                send_simple(CAN_BROADCAST, CLS_HEARTBEAT, OP_HEARTBEAT, 0);
            }
        }

        k_sleep(K_MSEC(50));
    }
}

void soh_thread(void *a, void *b, void *c)
{
    LOG_INF("soh_thread started");

    while (1) {
        if (current_mode == MODE_STANDARD ||
            current_mode == MODE_MISSION)
        {
            struct temp_telemetry telem;
            temp_telemetry_read_all(i2c_bus, &telem);
            temp_telemetry_print(&telem);
            check_node_timeouts();
        }

        k_sleep(K_MSEC(500));
    }
}

void watchdog_thread(void *a, void *b, void *c)
{
    LOG_INF("watchdog_thread started");

    while (1) {
        /* TODO: Replace with correct kick sequence for your watchdog IC */
        /* gpio_pin_toggle(gpioa, WATCHDOG_PIN); */

        LOG_DBG("watchdog kick");

        k_sleep(K_MSEC(WATCHDOG_KICK_MS));
    }
}

void gnss_thread(void *a, void *b, void *c)
{
    LOG_INF("gnss_thread started");

    while (1) {
        if (current_mode == MODE_STANDARD ||
            current_mode == MODE_MISSION)
        {
            /* Request position from GNSS node — it responds with OP_GNSS_POS
               on CLS_TELEMETRY, which handle_telemetry() parses above */
            send_simple(GNSS_ID, CLS_COMMAND, 0x61 /* OP_QUERY_POS */, 0);
            LOG_INF("GNSS position request sent");
        }

        k_sleep(K_MSEC(GNSS_POLL_MS));
    }
}

void telemetry_thread(void *a, void *b, void *c)
{
    LOG_INF("telemetry_thread started");

    while (1) {
        if (current_mode == MODE_STANDARD ||
            current_mode == MODE_MISSION)
        {
            /* TODO: Collect SOH, check thresholds, downlink if window open */
            LOG_INF("telemetry check - placeholder");
        }

        k_sleep(K_MSEC(TELEMETRY_CHECK_MS));
    }
}

/* ===================================================== */
/* ================= MAIN =============================== */
/* ===================================================== */

int main(void)
{
    LOG_INF("UT-CORE CDH Booting...");

    leds_init();

    k_thread_create(&watchdog_thread_data, watchdog_stack, STACK_SIZE,
                    watchdog_thread, NULL, NULL, NULL,
                    PRIO_WATCHDOG, 0, K_NO_WAIT);

    k_thread_create(&can_thread_data, can_stack, STACK_SIZE,
                    can_rx_thread, NULL, NULL, NULL,
                    PRIO_CAN_RX, 0, K_NO_WAIT);

    k_thread_create(&can_proc_thread_data, can_proc_stack, STACK_SIZE,
                    can_process_thread, NULL, NULL, NULL,
                    PRIO_CAN_PROC, 0, K_NO_WAIT);

    k_thread_create(&sched_thread_data, sched_stack, STACK_SIZE,
                    scheduler_thread, NULL, NULL, NULL,
                    PRIO_SCHED, 0, K_NO_WAIT);

    k_thread_create(&gnss_thread_data, gnss_stack, STACK_SIZE,
                    gnss_thread, NULL, NULL, NULL,
                    PRIO_GNSS, 0, K_NO_WAIT);

    k_thread_create(&soh_thread_data, soh_stack, STACK_SIZE,
                    soh_thread, NULL, NULL, NULL,
                    PRIO_SOH, 0, K_NO_WAIT);

    k_thread_create(&telemetry_thread_data, telemetry_stack, STACK_SIZE,
                    telemetry_thread, NULL, NULL, NULL,
                    PRIO_TELEMETRY, 0, K_NO_WAIT);

    setup_mode_init();

    while (1) {
        k_sleep(K_FOREVER);
    }
}