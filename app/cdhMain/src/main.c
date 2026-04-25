/*
 * UT-CORE CDH - Threaded Flight Architecture
 *
 * Hardware: STM32U5 microcontroller, Zephyr RTOS.
 * Bus:      FDCAN1 at 500 kbit/s, 29-bit extended IDs.
 * Sensors:  Up to 6 I2C temperature sensors (TMP1xx-family, 0x48-0x4D).
 *
 * Thread overview (highest to lowest priority):
 *   PRIO 0  watchdog_thread     - kicks hardware watchdog; must never starve
 *   PRIO 1  can_rx_thread       - drains hardware FIFO into SW queue
 *   PRIO 2  can_process_thread  - decodes & dispatches frames
 *   PRIO 3  scheduler_thread    - heartbeat + future mission scheduling
 *   PRIO 4  gnss_thread         - requests & stores GNSS position data
 *   PRIO 5  soh_thread          - reads temps & checks node-alive timeouts
 *   PRIO 6  telemetry_thread    - assembles & downlinks telemetry packets
 *
 * LED usage:
 *   led0 - pulses briefly on every received CAN frame (activity indicator)
 *   led1 - spare
 *   led2 - controlled remotely via OP_SET_LED ground command
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
#include <stdio.h>
#include <string.h>
#include "C:\Users\notbr\Documents\all_coding\ut-core\common\can_proto.h"
LOG_MODULE_REGISTER(cdh, LOG_LEVEL_INF);

/* ================= THREAD PRIORITIES ================= */
/*
 * Priority 0 is reserved for the watchdog — it must always be able to kick
 * the hardware timer regardless of what any other thread is doing.
 * Everything else is shifted down by one from the previous revision.
 */
#define PRIO_WATCHDOG   0   /* Highest - hardware watchdog kick             */
#define PRIO_CAN_RX     1   /* Drain hardware FIFO immediately              */
#define PRIO_CAN_PROC   2   /* Decode & dispatch frames from SW queue       */
#define PRIO_SCHED      3   /* Heartbeat + mission scheduling               */
#define PRIO_GNSS       4   /* Request & store GNSS position data           */
#define PRIO_SOH        5   /* Read temps & check node timeouts             */
#define PRIO_TELEMETRY  6   /* Lowest - assemble & downlink telemetry       */

#define STACK_SIZE 1024

/* ================= HARDWARE / PROTOCOL DEFINITIONS ================= */

#define NODE_ID     0x1
#define DST_ME      NODE_ID
#define PRIO_LOW    3

#define CAN_NODE     DT_NODELABEL(fdcan1)
#define I2C_BUS_NODE DT_NODELABEL(i2c1)
#define GPIOA_NODE   DT_NODELABEL(gpioa)
#define LED0_NODE    DT_ALIAS(led0)
#define LED1_NODE    DT_ALIAS(led1)
#define LED2_NODE    DT_ALIAS(led2)

#define PIN_SILENT   9
#define PIN_SHDN     10
#define PIN_ROLE     6
#define WATCHDOG_PIN 2      /* Hardware watchdog kick pin (not yet implemented) */

/* STM32U5 96-bit factory UID — used to verify board identity at boot. */
#define UID1_WORD0 0x00340016
#define UID1_WORD1 0x41425007
#define UID1_WORD2 0x20363651

#define UID2_WORD0 0x0012001B
#define UID2_WORD1 0x41425007
#define UID2_WORD2 0x20363651

/* TMP1xx I2C addresses (A0-A2 strap pins give 0x48-0x4D). */
static const uint8_t temp_addrs[] = { 0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D };
#define NUM_SENSORS ((uint8_t)(sizeof(temp_addrs) / sizeof(temp_addrs[0])))

#define TEMP_REG       0x00   /* TMP1xx temperature register address          */
#define CAN_PROC_Q_LEN 32     /* SW queue depth between RX and process threads */

/* ================= DEVICE HANDLES ================= */

static const struct device          *can_dev  = DEVICE_DT_GET(CAN_NODE);
static const struct device          *gpioa    = DEVICE_DT_GET(GPIOA_NODE);
static const struct device          *i2c_bus  = DEVICE_DT_GET(I2C_BUS_NODE);
static const struct gpio_dt_spec     led0     = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec     led1     = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
static const struct gpio_dt_spec     led2     = GPIO_DT_SPEC_GET(LED2_NODE, gpios);

/* Hardware-backed CAN RX queue — the driver ISR deposits frames here. */
CAN_MSGQ_DEFINE(rxq, 16);

/* ================= DATA STRUCTURES ================= */

/*
 * temp_sample - result of reading one I2C temperature sensor.
 *
 * Temperature is stored in Q4 fixed-point (value = degC * 16) to avoid
 * floating-point on the microcontroller.
 * Example: 25.5 degC -> stored as 408. To display: (408 + 8) / 16 = 26 degC.
 */
struct temp_sample {
    uint8_t addr;     /* Sensor I2C address (0x48-0x4D), for debug labeling  */
    int16_t temp_q4;  /* Temperature in Q4 units; 0 if read failed            */
    int     status;   /* 0 = success, negative errno on I2C failure           */
};

/* temp_telemetry - snapshot of all sensors at one instant. */
struct temp_telemetry {
    uint32_t t_ms;                    /* k_uptime_get_32() at time of read    */
    struct temp_sample s[NUM_SENSORS];/* Indexed same order as temp_addrs[]   */
};

/*
 * node_status_t - liveness record for one remote satellite subsystem.
 * Written by handle_heartbeat(), read by check_node_timeouts().
 */
typedef struct {
    bool    alive;         /* True if we have recently heard from this node   */
    int64_t last_seen_ms;  /* k_uptime_get() at time of last heartbeat        */
} node_status_t;

/* Liveness table covering every node on the satellite CAN bus. */
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

/* Zero-init = all nodes start as "not yet seen". */
static node_heartbeat_t node_alive = {0};

/* A node is declared dead if no heartbeat arrives within this window. */
#define HEARTBEAT_TIMEOUT_MS 5000

/*
 * WATCHDOG_KICK_MS - interval between hardware watchdog kicks (ms).
 *
 * Must be less than half the hardware watchdog timeout period to guarantee
 * a kick lands even if the thread wakes slightly late due to scheduler jitter.
 * Tune this once the STM32 IWDG timeout is finalized in the board config.
 *
 * Example: if hardware timeout = 1000 ms, kick every 500 ms.
 */
#define WATCHDOG_KICK_MS 15

/*
 * gnss_data_t - latest position fix received from the GNSS node.
 *
 * Written by gnss_thread when a GNSS telemetry frame arrives.
 * Read by scheduler_thread to determine experiment pass windows.
 *
 * TODO: populate fields once the GNSS CAN telemetry format is defined.
 */
typedef struct {
    int32_t  lat_deg_e7;   /* Latitude  * 1e7 (avoids float); e.g. 30.2849 -> 302849000  */
    int32_t  lon_deg_e7;   /* Longitude * 1e7                                             */
    int32_t  alt_mm;       /* Altitude above ellipsoid in millimetres                     */
    uint32_t timestamp_ms; /* k_uptime_get_32() at time of fix                            */
    bool     valid;        /* True if the fix is fresh and usable                         */
} gnss_data_t;

/* Shared GNSS state. Written by gnss_thread, read by scheduler_thread.
 * TODO: protect with a mutex once both threads are fully implemented. */
static gnss_data_t gnss_latest = {0};

/*
 * SW queue between can_rx_thread and can_process_thread.
 * Decoupling RX (high priority, must not stall) from dispatch (may be slow).
 */
K_MSGQ_DEFINE(can_proc_q, sizeof(struct can_frame), CAN_PROC_Q_LEN, 4);

/*
 * can_packet_t - decoded CAN frame with routing fields unpacked from the ID.
 *
 * 29-bit ID layout:
 *   [28:26] priority  (3 bits)
 *   [21:14] src       (8 bits)
 *   [13: 6] dst       (8 bits)
 *   [ 5: 0] msg_class (6 bits)
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

/* ================= MODE STATE MACHINE ================= */
/*
 *   MODE_SETUP    - Power-on init; threads idle until complete.
 *   MODE_STANDARD - Normal on-orbit ops.
 *   MODE_MISSION  - Active payload / high-data-rate phase.
 *   MODE_ERROR    - Fault state; minimal activity.
 */
typedef enum {
    MODE_SETUP,
    MODE_STANDARD,
    MODE_MISSION,
    MODE_ERROR
} cdh_mode_t;

/* Shared across threads; volatile prevents compiler from caching in a register. */
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

/* ===================================================== */
/* ================= FORWARD DECLARATIONS =============== */
/* ===================================================== */

static void handle_heartbeat(const can_packet_t *pkt);
static void handle_command(const can_packet_t *pkt);
static void handle_cmd_response(const can_packet_t *pkt);
static void handle_telemetry(const can_packet_t *pkt);
static void handle_health(const can_packet_t *pkt);
static void can_dispatch(const can_packet_t *pkt);

/* ===================================================== */
/* ================= UTILITY FUNCTIONS ================== */
/* ===================================================== */

/*
 * read_uid() - Read the STM32U5 96-bit factory Unique Device Identifier.
 * The UID is in the flash info area at UID_BASE and never changes in the field.
 *
 * @uid: Output array of 3 x uint32_t; caller must allocate.
 */
static void read_uid(uint32_t uid[3])
{
    LOG_INF("reading UID...");
    uid[0] = *(uint32_t *)(UID_BASE + 0x0);
    uid[1] = *(uint32_t *)(UID_BASE + 0x4);
    uid[2] = *(uint32_t *)(UID_BASE + 0x8);
}

/*
 * led_can_activity() - Pulse led0 briefly to indicate a CAN frame was received.
 *
 * Uses k_busy_wait (spin) so the pulse width is precise regardless of
 * scheduler state. Called from can_rx_thread immediately after a frame
 * arrives. The 1 ms spin is short enough not to meaningfully delay
 * forwarding the frame to the processing queue.
 */
static inline void led_can_activity(void)
{
    gpio_pin_set_dt(&led0, 1);
    k_busy_wait(1000);   /* 1 ms pulse — spin, not sleep */
    gpio_pin_set_dt(&led0, 0);
}

/*
 * leds_init() - Configure all LED GPIO pins as outputs, initially off.
 * Must be called once in main() before any thread touches the LEDs.
 */
static void leds_init(void)
{
    gpio_pin_configure_dt(&led0, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led1, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led2, GPIO_OUTPUT_INACTIVE);
}

/*
 * print_temp_q4() - Print a Q4 temperature as a rounded integer degC.
 *
 * Q4: stored value = actual_degC * 16. Adding 8 before dividing rounds to
 * the nearest integer rather than always truncating toward zero.
 */
static void print_temp_q4(int16_t t_q4)
{
    int32_t temp_c = (t_q4 + 8) / 16;
    printk("%ld", (long)temp_c);
}

/*
 * temp_telemetry_read_all() - Poll all temperature sensors over I2C.
 *
 * For each sensor:
 *   1. Write the temperature register address (0x00).
 *   2. Read back 2 bytes big-endian containing the 12-bit signed result.
 *   3. Reconstruct, sign-extend, right-shift by 4 to get Q4 fixed-point.
 *
 * Per-sensor failures are recorded in s[i].status; partial reads are still
 * useful — a dead sensor doesn't invalidate the rest of the snapshot.
 *
 * @bus:    Zephyr I2C device handle.
 * @out:    Caller-allocated struct to fill.
 * @return: Number of sensors read successfully (0 to NUM_SENSORS).
 */
static int temp_telemetry_read_all(const struct device *bus,
                                   struct temp_telemetry *out)
{
    if (out == NULL || bus == NULL) {
        return 0;
    }

    out->t_ms = (uint32_t)k_uptime_get_32();

    int ok = 0;

    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        uint8_t reg    = TEMP_REG;
        uint8_t buf[2] = {0};

        out->s[i].addr = temp_addrs[i];

        /*
         * i2c_write_read(): one transaction — write register pointer, then
         * read back 2 bytes. Standard TMP1xx access pattern.
         */
        int ret = i2c_write_read(bus, temp_addrs[i], &reg, 1, buf, 2);
        out->s[i].status = ret;

        if (ret == 0) {
            /*
             * Reconstruct big-endian 16-bit word, sign-extend via int16_t,
             * then drop the lower 4 reserved bits to get Q4 (degC * 16).
             * Example: 25.5 degC -> raw 0x1980 -> >>4 -> 408.
             */
            int16_t raw = (int16_t)((buf[0] << 8) | buf[1]);
            out->s[i].temp_q4 = (int16_t)(raw >> 4);
            ok++;
        } else {
            out->s[i].temp_q4 = 0; /* Sentinel: no valid data */
        }
    }

    return ok;
}

/*
 * temp_telemetry_print() - Print a full thermal snapshot to the console.
 * Format: "t=<ms> ms | 0x48:25C  0x49:ERR(-5) ..."
 */
static void temp_telemetry_print(const struct temp_telemetry *t)
{
    printk("t=%lu ms | ", (unsigned long)t->t_ms);

    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        printk("0x%02X:", t->s[i].addr);

        if (t->s[i].status == 0) {
            print_temp_q4(t->s[i].temp_q4);
            printk("C");
        } else {
            printk("ERR(%d)", t->s[i].status);
        }

        if (i + 1 < NUM_SENSORS) {
            printk("  ");
        }
    }

    printk("\n");
}

/*
 * get_node_status() - Map a CAN node ID to its liveness record.
 *
 * @node_id: src field from a decoded CAN packet.
 * @return:  Pointer into node_alive, or NULL for unrecognised IDs.
 */
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

/*
 * send_simple() - Transmit a one-opcode-one-value CAN frame.
 *
 * Convenience wrapper for sending a minimal command or status frame without
 * manually filling a can_frame struct each time.
 */
static void send_simple(uint8_t dst, uint8_t cls, uint8_t op, uint8_t val)
{
    struct can_frame f = {0};
    f.id    = CAN_ID_FULL(PRIO_LOW, NODE_ID, dst, cls);
    f.flags = CAN_FRAME_IDE;
    can_fill_payload(&f, NODE_ID, op, val, 0, 0, 0, 0, 0);
    can_send(can_dev, &f, K_NO_WAIT, NULL, NULL);
}

/*
 * tcan3403_wakeup() - Bring the CAN transceiver out of shutdown.
 *
 * Both SHDN and SILENT must be driven low before the TCAN3403 passes
 * traffic. 1 ms settle time is conservative but safe.
 */
static void tcan3403_wakeup(void)
{
    gpio_pin_configure(gpioa, PIN_SHDN,   GPIO_OUTPUT_INACTIVE); /* Power on  */
    gpio_pin_configure(gpioa, PIN_SILENT, GPIO_OUTPUT_INACTIVE); /* Normal TX */
    k_msleep(1);
    LOG_INF("TCAN3403 Awake");
}

/*
 * can_setup() - Initialise FDCAN peripheral and install RX filters.
 *
 * Two filters are installed:
 *   a) Frames explicitly addressed to this node (dst == NODE_ID)
 *   b) Broadcast frames (dst == CAN_BROADCAST)
 * All other frames are rejected in hardware with zero CPU cost.
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

/*
 * can_decode() - Unpack a 29-bit CAN ID into a can_packet_t.
 *
 * ID bit layout:
 *   [28:26] priority  | [21:14] src | [13:6] dst | [5:0] msg_class
 */
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

/*
 * can_dispatch() - Route a decoded packet to its message-class handler.
 * Add new message classes here as the protocol grows.
 */
static void can_dispatch(const can_packet_t *pkt)
{
    switch (pkt->msg_class) {
        case 0:  handle_heartbeat(pkt);    break;
        case 2:  handle_command(pkt);      break;
        case 3:  handle_cmd_response(pkt); break;
        case 4:  handle_telemetry(pkt);    break;
        case 10: handle_health(pkt);       break;
        default:
            LOG_WRN("Unhandled class: %d", pkt->msg_class);
            break;
    }
}

/* ===================================================== */
/* ================= MESSAGE HANDLERS =================== */
/* ===================================================== */

/*
 * handle_cmd_response() - Called when a node replies to a command we sent.
 * TODO: match reply to a pending-command table and mark it acknowledged.
 */
static void handle_cmd_response(const can_packet_t *pkt)
{
    LOG_INF("CmdResp from node 0x%x", pkt->src);
}

/*
 * handle_telemetry() - Called when a node broadcasts telemetry.
 * TODO: store or forward to COMMS for downlink.
 */
static void handle_telemetry(const can_packet_t *pkt)
{
    LOG_INF("Telemetry from node 0x%x", pkt->src);
}

/*
 * handle_health() - Called when a node sends a health/fault report.
 * TODO: parse fault flags and update system fault table.
 */
static void handle_health(const can_packet_t *pkt)
{
    LOG_INF("Health from node 0x%x", pkt->src);
}

/*
 * handle_command() - Execute a command frame addressed to this CDH node.
 *
 * Payload layout (can_proto.h convention):
 *   data[1] = opcode
 *   data[2] = value / argument
 *
 * Supported opcodes:
 *   OP_SET_MODE - change operating mode immediately.
 *   OP_REBOOT   - hardware reset (stubbed pending safety review).
 *   OP_SET_LED  - drive led2 on/off (ground lamp-test command).
 */
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
            /* Ground lamp-test: drives led2, not the CAN activity led0 */
            LOG_INF("LED command: led2 -> %d", val);
            gpio_pin_set_dt(&led2, val);
            break;

        default:
            LOG_WRN("Unknown opcode: %d", opcode);
            break;
    }
}

/*
 * handle_heartbeat() - Record that a remote node is alive.
 *
 * Stamps the node's entry in node_alive with the current uptime.
 * check_node_timeouts() later uses this timestamp to detect silence.
 */
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
 * check_node_timeouts() - Warn about nodes that have gone silent.
 *
 * Only checks nodes seen at least once (alive == true) to avoid log spam
 * for optional subsystems absent from a test configuration.
 * Clears alive after the first warning so it fires once per event.
 *
 * TODO: escalate to MODE_ERROR or trigger EPS power-cycle for critical nodes.
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
        { STAR_ID,  "SOLAR",  &node_alive.solar  },
    };

    for (int i = 0; i < ARRAY_SIZE(nodes); i++) {
        node_status_t *n = nodes[i].status;

        if (!n->alive) {
            continue; /* Never seen or already declared dead — skip */
        }

        if ((now - n->last_seen_ms) > HEARTBEAT_TIMEOUT_MS) {
            LOG_WRN("Node %s (0x%02X) timed out!", nodes[i].name, nodes[i].id);
            n->alive = false; /* Clear so warning fires only once per event */
        }
    }
}

/* ===================================================== */
/* ================= SETUP MODE ========================= */
/* ===================================================== */

/*
 * setup_mode_init() - One-shot power-on initialisation, runs in main() context.
 *
 * Steps:
 *   1. Read and verify the 96-bit hardware UID against the role strap pin.
 *   2. Wake the TCAN3403 transceiver.
 *   3. Initialise FDCAN + install RX filters.
 *   4. Sleep 2 s for bus stabilisation, then release threads via MODE_STANDARD.
 *
 * All threads are already spawned when this runs but idle on the mode check,
 * so they won't do anything until step 4 completes.
 */
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

    current_mode = MODE_STANDARD; /* Release all threads into operation */
}

/* ===================================================== */
/* ================= THREADS ============================ */
/* ===================================================== */

/*
 * can_rx_thread() - Drain the hardware CAN FIFO into the SW processing queue.
 *
 * Highest priority (PRIO_CAN_RX = 1) because the STM32 FDCAN hardware FIFO
 * is shallow — frames are silently lost if not drained quickly under load.
 *
 * On each received frame:
 *   1. Pulses led0 for 2 ms as a visible CAN activity indicator.
 *   2. Forwards the raw frame to can_proc_q for decoding.
 *
 * Blocks on K_FOREVER when the bus is idle — zero CPU when quiet.
 * Sleeps 200 ms in non-active modes rather than busy-waiting.
 */
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

                /* Forward to processing thread; drop and warn if queue full */
                if (k_msgq_put(&can_proc_q, &rx, K_NO_WAIT) != 0) {
                    LOG_WRN("CAN proc queue full — frame dropped");
                }
            }
        } else {
            k_sleep(K_MSEC(200)); /* Inactive mode — yield without spinning */
        }
    }
}

/*
 * can_process_thread() - Decode and dispatch frames from the SW queue.
 *
 * Runs one priority below can_rx_thread so the FIFO drain is never delayed
 * by slower dispatch logic (e.g. a command handler that talks to I2C).
 *
 * Secondary destination check guards against hardware filter misconfiguration.
 */
void can_process_thread(void *a, void *b, void *c)
{
    LOG_INF("can_process_thread started");
    struct can_frame frame;
    can_packet_t pkt;

    while (1) {
        if (k_msgq_get(&can_proc_q, &frame, K_FOREVER) == 0) {
            /* Pulse LED on every frame processed */
            led_can_activity();

            can_decode(&frame, &pkt);

            /* Belt-and-suspenders: discard anything not addressed to us */
            if (pkt.dst != NODE_ID && pkt.dst != CAN_BROADCAST) {
                continue;
            }

            can_dispatch(&pkt);
        }
    }
}

/*
 * scheduler_thread() - Send periodic 1 Hz heartbeat broadcast.
 *
 * Uses an uptime delta so execution-time jitter doesn't accumulate into
 * long-term heartbeat drift. Polls every 50 ms (20x oversampling).
 *
 * This thread will grow into the mission_scheduler_thread, also owning
 * downlink windows and experiment pass timers.
 */
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

/*
 * soh_thread() - State-of-Health: temperature reads + node liveness checks.
 *
 * Lowest priority — best-effort 500 ms cadence. A late read is harmless;
 * the 5 s heartbeat timeout gives 10x margin over the 500 ms poll rate.
 *
 * Future additions:
 *   - SOH out-of-range detection and COMMS downlink trigger
 *   - Telemetry packet assembly
 */
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

/*
 * watchdog_thread() - Kick the hardware watchdog on a fixed interval.
 *
 * This is the highest-priority thread (PRIO_WATCHDOG = 0). A missed kick
 * causes the hardware to reset the MCU — which is exactly what we want if
 * the system has locked up. It must never be starved by any other thread.
 *
 * Current behavior:
 *   Toggles WATCHDOG_PIN (PA2) every WATCHDOG_KICK_MS milliseconds.
 *   gpio_pin_toggle_dt() or a pulse may be required depending on your
 *   external watchdog IC's trigger spec — check the datasheet.
 *
 * TODO: Replace gpio toggle with the correct kick sequence for your WD IC.
 * TODO: Configure WATCHDOG_KICK_MS to be < half the hardware timeout period.
 * TODO: Consider using the STM32 IWDG peripheral directly instead of a GPIO.
 */
void watchdog_thread(void *a, void *b, void *c)
{
    LOG_INF("watchdog_thread started");

    /* TODO: Configure WATCHDOG_PIN as output if not already done in setup */
    /* gpio_pin_configure(gpioa, WATCHDOG_PIN, GPIO_OUTPUT_INACTIVE); */

    while (1) {
        /* TODO: Replace with correct kick sequence for your watchdog IC */
        /* gpio_pin_toggle(gpioa, WATCHDOG_PIN); */

        LOG_DBG("watchdog kick");  /* Downgrade to LOG_DBG to reduce log noise */

        k_sleep(K_MSEC(WATCHDOG_KICK_MS));
    }
}

/*
 * gnss_thread() - Request and store GNSS position data from the GNSS node.
 *
 * GNSS communication is asynchronous: we send a parameter request over CAN,
 * then wait for the GNSS node to respond with a telemetry frame. This async
 * nature is why GNSS lives in its own thread rather than inside soh_thread —
 * the wait for a CAN reply doesn't belong inside a fixed-cadence SOH loop.
 *
 * Planned behavior:
 *   1. Every GNSS_POLL_MS, send a paramget request to the GNSS node.
 *   2. The CAN response arrives asynchronously via handle_telemetry().
 *   3. handle_telemetry() (or a dedicated handle_gnss_telemetry()) writes
 *      the parsed fix into gnss_latest.
 *   4. gnss_thread reads gnss_latest and checks whether we are in the
 *      experiment pass window (target lat/lon within threshold).
 *   5. If in window, notify scheduler_thread to trigger the experiment.
 *
 * TODO: Define the GNSS CAN telemetry frame format in can_proto.h.
 * TODO: Implement handle_gnss_telemetry() to populate gnss_latest.
 * TODO: Implement pass-window geometry check (great-circle or bounding box).
 * TODO: Define inter-thread notification mechanism (k_sem or k_event).
 */
void gnss_thread(void *a, void *b, void *c)
{
    LOG_INF("gnss_thread started");

    /* How often to request a fresh position fix from the GNSS node (ms). */
    /* TODO: Tune based on GNSS update rate and orbit dynamics. */
    #define GNSS_POLL_MS 5000

    while (1) {
        if (current_mode == MODE_STANDARD ||
            current_mode == MODE_MISSION)
        {
            /* TODO: Send paramget request to GNSS node over CAN */
            /* send_simple(GNSS_ID, CLS_PARAMGET, OP_GNSS_POSITION, 0); */
            LOG_INF("GNSS poll - placeholder");

            /* TODO: Check gnss_latest.valid and run pass-window logic */
            /* if (gnss_latest.valid) { check_experiment_window(); } */
        }

        k_sleep(K_MSEC(GNSS_POLL_MS));
    }
}

/*
 * telemetry_thread() - Assemble and downlink telemetry packets via COMMS.
 *
 * This thread owns the outbound telemetry pipeline:
 *   1. Collects SOH data: temperatures, node liveness, mode, fault flags.
 *   2. Checks whether a downlink window is open (scheduler sets a flag/semaphore).
 *   3. If in window, packages data into the downlink frame format and sends
 *      it to the COMMS node over CAN for RF transmission to the ground station.
 *   4. If SOH is out of acceptable range (over-temp, node timeout, etc.),
 *      triggers an immediate emergency downlink regardless of window.
 *
 * Runs at the lowest priority (PRIO_TELEMETRY = 6) because downlink is
 * best-effort — missing one packet is fine as long as the trend is captured.
 *
 * TODO: Define the telemetry packet struct (all fields to downlink).
 * TODO: Define the downlink window flag/semaphore with scheduler_thread.
 * TODO: Define SOH threshold values for emergency downlink trigger.
 * TODO: Implement COMMS handoff — the CAN frame format for a downlink command.
 */
void telemetry_thread(void *a, void *b, void *c)
{
    LOG_INF("telemetry_thread started");

    /* How often to check if a downlink window is open and SOH is in range (ms). */
    /* TODO: Tune based on downlink schedule cadence. */
    #define TELEMETRY_CHECK_MS 1000

    while (1) {
        if (current_mode == MODE_STANDARD ||
            current_mode == MODE_MISSION)
        {
            /* TODO: Collect latest SOH data into a telemetry packet struct */
            /* telemetry_packet_t pkt = build_telemetry_packet(); */

            /* TODO: Check if SOH values are within acceptable range */
            /* if (!soh_in_range(&pkt)) { trigger_emergency_downlink(&pkt); } */

            /* TODO: Check downlink window flag set by scheduler_thread */
            /* if (downlink_window_open) { send_telemetry_to_comms(&pkt); } */

            LOG_INF("telemetry check - placeholder");
        }

        k_sleep(K_MSEC(TELEMETRY_CHECK_MS));
    }
}



/*
 * main() - Spawn all threads, run boot sequence, then park forever.
 *
 * Threads are created before setup_mode_init() so they're ready for any
 * CAN traffic arriving during boot. They idle on the mode check until
 * setup_mode_init() transitions to MODE_STANDARD (~2 s post-boot).
 *
 * Thread creation order matches priority order (highest first) as a
 * convention — Zephyr runs them by priority regardless of creation order.
 */
int main(void)
{
    LOG_INF("UT-CORE CDH Booting...");

    leds_init();

    /* Priority 0 — watchdog must be first and highest */
    k_thread_create(&watchdog_thread_data, watchdog_stack, STACK_SIZE,
                    watchdog_thread, NULL, NULL, NULL,
                    PRIO_WATCHDOG, 0, K_NO_WAIT);

    /* Priority 1 — CAN RX must be near the top to drain the hardware FIFO */
    k_thread_create(&can_thread_data, can_stack, STACK_SIZE,
                    can_rx_thread, NULL, NULL, NULL,
                    PRIO_CAN_RX, 0, K_NO_WAIT);

    /* Priority 2 — CAN frame decode and dispatch */
    k_thread_create(&can_proc_thread_data, can_proc_stack, STACK_SIZE,
                    can_process_thread, NULL, NULL, NULL,
                    PRIO_CAN_PROC, 0, K_NO_WAIT);

    /* Priority 3 — heartbeat + future mission scheduling */
    k_thread_create(&sched_thread_data, sched_stack, STACK_SIZE,
                    scheduler_thread, NULL, NULL, NULL,
                    PRIO_SCHED, 0, K_NO_WAIT);

    /* Priority 4 — GNSS position requests and pass-window checks */
    k_thread_create(&gnss_thread_data, gnss_stack, STACK_SIZE,
                    gnss_thread, NULL, NULL, NULL,
                    PRIO_GNSS, 0, K_NO_WAIT);

    /* Priority 5 — temperature reads and node liveness checks */
    k_thread_create(&soh_thread_data, soh_stack, STACK_SIZE,
                    soh_thread, NULL, NULL, NULL,
                    PRIO_SOH, 0, K_NO_WAIT);

    /* Priority 6 — lowest; telemetry assembly and COMMS downlink handoff */
    k_thread_create(&telemetry_thread_data, telemetry_stack, STACK_SIZE,
                    telemetry_thread, NULL, NULL, NULL,
                    PRIO_TELEMETRY, 0, K_NO_WAIT);

    setup_mode_init();

    while (1) {
        k_sleep(K_FOREVER);
    }
}