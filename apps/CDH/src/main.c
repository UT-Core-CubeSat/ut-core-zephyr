/**
 * @defgroup cdh CDH
 * @ingroup apps
 * @brief Command and Data Handling firmware
 *
 * @include{doc} ./apps/CDH/README.md
 */

/**
 * @file main.c
 * @ingroup cdh
 * @brief Entry point for CDH firmware
 *
 * UT-CORE CDH — Threaded Flight Architecture
 *
 * Hardware: STM32U5 microcontroller, Zephyr RTOS.
 * Bus:      FDCAN1 at 500 kbit/s, 29-bit extended IDs.
 * Sensors:  Up to 6 I2C temperature sensors (TMP1xx-family, 0x48-0x4D).
 *
 * Thread overview (highest to lowest priority):
 *   PRIO 0  watchdog_thread     — supervises thread check-ins; must never starve
 *   PRIO 1  can_rx_thread       — drains hardware FIFO into SW queue
 *   PRIO 2  can_process_thread  — decodes & dispatches frames
 *   PRIO 3  scheduler_thread    — heartbeat (carries current mode) + future mission scheduling
 *   PRIO 3  mode_manager_thread — mode state machine; the only writer of current_mode
 *   PRIO 4  gnss_thread         — requests & stores GNSS position data
 *   PRIO 5  soh_thread          — reads temps & checks node-alive timeouts
 *   PRIO 6  telemetry_thread    — assembles & downlinks telemetry packets
 *
 * LED usage:
 *   led0 — pulses briefly on every received CAN frame (activity indicator)
 *   led1 — spare
 *   led2 — controlled remotely via OP_SET_LED ground command
 *
 * Modes (sat-controls/cdh_state_machine.drawio):
 *   SAFE       — boot + minimal activity; entered on every reset
 *   STANDBY    — maintain readiness, watch for the next Utah pass
 *   DOWNLINK   — coordinate data transmission (placeholder)
 *   EXPERIMENT — scan Utah while over the target (placeholder)
 *   PHOENIX    — fault recovery; always exits to SAFE
 *
 * @todo move CAN code shared between apps into the `common/` directory at git root.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/task_wdt/task_wdt.h>
#include <zephyr/drivers/hwinfo.h>
// #include <stm32u5xx.h>
#include <stdbool.h>
#include <string.h>

#include "board_config.h"
#include "common/can_proto.h"
#include "common/health.h"
#include "common/temp_telemetry.h"

/** @cond */ /* Hidden from Doxygen, or it will mistake this as a function */
LOG_MODULE_REGISTER(cdh, LOG_LEVEL_INF);
/** @endcond */

/* ================= DEVICE HANDLES ================= */

static const struct device *can_dev = DEVICE_DT_GET(CAN_NODE);
static const struct device *gpioa = DEVICE_DT_GET(GPIOA_NODE);
static const struct device *i2c_bus = DEVICE_DT_GET(I2C_BUS_NODE);
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
static const struct gpio_dt_spec led2 = GPIO_DT_SPEC_GET(LED2_NODE, gpios);

/** @cond */ /* Hidden from Doxygen, or it will mistake this as a function */
/** Hardware-backed CAN RX queue — the driver ISR deposits frames here. */
CAN_MSGQ_DEFINE(rxq, 16);
/** @endcond */

/** @cond */ /* Hidden from Doxygen, or it will mistake this as a function */
/** SW queue between can_rx_thread and can_process_thread. */
K_MSGQ_DEFINE(can_proc_q, sizeof(struct can_frame), CAN_PROC_Q_LEN, 4);
/** @endcond */

/* ================= DATA STRUCTURES ================= */

/**
 * node_status_t — liveness record for one remote satellite subsystem.
 * Written by handle_heartbeat(), read by check_node_timeouts().
 */
typedef struct
{
    bool alive;
    int64_t last_seen_ms;
} node_status_t;

typedef struct
{
    node_status_t cdh;
    node_status_t eps;
    node_status_t comms;
    node_status_t adcs;
    node_status_t motor;
    node_status_t gnss;
    node_status_t star;
    node_status_t solar;
} node_heartbeat_t;

/** @brief Liveness table for every remote satellite subsystem, indexed by node. */
static node_heartbeat_t node_alive = {0};

/**
 * gnss_data_t — latest position fix received from the GNSS node.
 *
 * @todo TODO: populate fields once the GNSS CAN telemetry format is defined.
 */
typedef struct
{
    int32_t lat_deg_e7;
    int32_t lon_deg_e7;
    int32_t alt_mm;
    uint32_t timestamp_ms;
    bool valid;
} gnss_data_t;

/**
 * @brief Most recently received GNSS fix.
 * @todo Populate/verify all fields once the GNSS CAN telemetry format is
 *       finalized.
 */
static gnss_data_t gnss_latest = {0};

/* ================= MODE STATE MACHINE ================= */

/**
 * @brief CDH operating mode (see sat-controls/cdh_state_machine.drawio).
 *
 * Values are explicit because the mode is broadcast in every CDH heartbeat
 * (payload byte 0) and arrives as the OP_SET_MODE argument, so other boards
 * depend on them. Never reorder or renumber.
 */
typedef enum
{
    MODE_SAFE = 0,       /**< Boot + minimal activity; entered on every reset */
    MODE_STANDBY = 1,    /**< Maintain readiness, watch for the next Utah pass */
    MODE_DOWNLINK = 2,   /**< Coordinate data transmission */
    MODE_EXPERIMENT = 3, /**< Scan Utah while over the target */
    MODE_PHOENIX = 4,    /**< Fault recovery; always exits to SAFE */
    MODE_COUNT           /**< Number of modes; also "no request" in ground_mode_request */
} cdh_mode_t;

/** @brief Current mode. Only mode_manager_thread writes it; every other thread only reads. */
volatile cdh_mode_t current_mode = MODE_SAFE;

/** @brief Uptime when current_mode was last entered. Owned by mode_manager_thread. */
static int64_t mode_entered_ms;

/**
 * @brief Mode requested by ground via OP_SET_MODE, or MODE_COUNT for none.
 *        Written by can_process_thread, consumed by mode_manager_thread.
 */
static atomic_t ground_mode_request = ATOMIC_INIT(MODE_COUNT);

/**
 * @brief Faults that send CDH to PHOENIX.
 *
 * @todo PLACEHOLDER fault list: which faults go to PHOENIX is not decided
 *       yet. The two below come from the state machine diagram (SAFE's
 *       "boot failed" edge and PHOENIX step 1 "restart hung thread"). Add
 *       new faults here, then decide in fault_is_critical() whether each
 *       one may interrupt a DOWNLINK pass.
 */
typedef enum
{
    FAULT_NONE = 0,
    FAULT_BOOT_FAILED = 1, /**< SAFE boot sequence failed (CAN bring-up) */
    FAULT_THREAD_HUNG = 2, /**< A supervised thread missed its check-in */
} cdh_fault_t;

/** @brief First reported fault not yet taken by PHOENIX, or FAULT_NONE. See report_fault(). */
static atomic_t pending_fault = ATOMIC_INIT(FAULT_NONE);

/** @brief PHOENIX escalation ladder; attempt N runs step N, capped at the last step. */
typedef enum
{
    PHOENIX_STEP_RESTART_THREAD = 0,
    PHOENIX_STEP_POWER_CYCLE_BOARD,
    PHOENIX_STEP_REINIT_CAN,
    PHOENIX_STEP_REBOOT,
    PHOENIX_STEP_COUNT
} phoenix_step_t;

/** @brief Uptime when the last PHOENIX step ran. Owned by mode_manager_thread. */
static int64_t phoenix_last_step_ms;

/** @brief Ground plan for the upcoming Utah pass. */
typedef enum
{
    PASS_PLAN_NONE,
    PASS_PLAN_DOWNLINK,
    PASS_PLAN_SCAN
} pass_plan_t;

/** @brief Set once safe_mode_boot() has finished, whether or not it succeeded. */
static atomic_t boot_done = ATOMIC_INIT(0);

/** @brief Set once safe_mode_boot() has brought CAN up. */
static atomic_t boot_ok = ATOMIC_INIT(0);

/** @brief Set by soh_thread while any onboard temperature sensor is outside TEMP_SAFE_MIN/MAX_C. */
static atomic_t temp_unsafe = ATOMIC_INIT(0);

/* ================= PERSISTENT STATE (FRAM) ================= */

/**
 * @brief State that must survive a reset (diagram: "Saved to FRAM").
 *
 * The golden copy of the code and the downlink queue also live in FRAM per
 * the diagram, but are not part of this record. Written by
 * safe_mode_boot() until boot_done is set, then only by mode_manager_thread.
 */
typedef struct
{
    uint8_t last_mode;           /**< cdh_mode_t at last save; diagnostics only, boot always enters SAFE */
    uint32_t reset_cause;        /**< hwinfo RESET_* flags read at the last boot */
    uint32_t reboot_count;       /**< Total boots */
    uint32_t rapid_reboot_count; /**< Consecutive boots that ended before REBOOT_STABLE_MS of uptime */
    uint8_t recorded_fault;      /**< cdh_fault_t that sent CDH to PHOENIX and has not recovered */
    uint8_t phoenix_attempts;    /**< PHOENIX steps tried since the last recovery */
    uint8_t can_fault;           /**< @todo Recorded CAN fault: contents not defined yet */
} cdh_persist_t;

static cdh_persist_t persist = {0};

/* ================= THREAD HEALTH ================= */

/* health.h ids for each supervised thread, assigned in health_init(). */
static int hb_can_rx;
static int hb_can_proc;
static int hb_sched;
static int hb_soh;
static int hb_gnss;
static int hb_telemetry;
static int hb_mode_mgr;

/** Times the supervisor's own task watchdog channel expired (see supervisor_wdt_expired()). */
static atomic_t supervisor_wdt_expiries = ATOMIC_INIT(0);

/* ================= THREAD STACKS ================= */

K_THREAD_STACK_DEFINE(watchdog_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(can_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(sched_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(soh_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(can_proc_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(gnss_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(telemetry_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(mode_mgr_stack, STACK_SIZE);

static struct k_thread watchdog_thread_data;
static struct k_thread can_thread_data;
static struct k_thread sched_thread_data;
static struct k_thread soh_thread_data;
static struct k_thread can_proc_thread_data;
static struct k_thread gnss_thread_data;
static struct k_thread telemetry_thread_data;
static struct k_thread mode_mgr_thread_data;

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

/**
 * @brief Read this MCU's 96-bit factory UID from the flash info area.
 * @param uid Output array of 3 words (96 bits total).
 * @warning Relies on UID_BASE, which normally comes from
 *          <stm32u5xx.h> — that include is currently commented out
 *          in this file. Confirm UID_BASE is still defined somewhere,
 *          or this either fails to compile or silently uses a stale value.
 */
static void read_uid(uint32_t uid[3])
{
    LOG_INF("reading UID...");
    uid[0] = *(uint32_t *)(UID_BASE + 0x0);
    uid[1] = *(uint32_t *)(UID_BASE + 0x4);
    uid[2] = *(uint32_t *)(UID_BASE + 0x8);
}

/**
 * @brief Briefly pulse led0 to indicate a CAN frame was received.
 */
static inline void led_can_activity(void)
{
    gpio_pin_set_dt(&led0, 1);
    k_busy_wait(1000);
    gpio_pin_set_dt(&led0, 0);
}

/**
 * @brief Configure all three status LED GPIOs as outputs, initially off.
 */
static void leds_init(void)
{
    gpio_pin_configure_dt(&led0, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led1, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led2, GPIO_OUTPUT_INACTIVE);
}

/**
 * @brief Look up the liveness record for a given CAN node ID.
 * @param node_id Source node ID from a decoded CAN packet.
 * @return Pointer to the matching node_status_t, or NULL if unrecognized.
 */
static node_status_t *get_node_status(uint8_t node_id)
{
    switch (node_id)
    {
    case CDH_ID:
        return &node_alive.cdh;
    case EPS_ID:
        return &node_alive.eps;
    case COMMS_ID:
        return &node_alive.comms;
    case ADCS_ID:
        return &node_alive.adcs;
    case MOTOR_ID:
        return &node_alive.motor;
    case GNSS_ID:
        return &node_alive.gnss;
    case STAR_ID:
        return &node_alive.star;
    case SOLAR_ID:
        return &node_alive.solar;
    default:
        return NULL;
    }
}

/**
 * @brief Report a fault to the mode manager, which sends CDH to PHOENIX.
 *        Safe from any thread. Only the first fault is kept until PHOENIX
 *        takes it; later reports are dropped.
 * @param fault Fault to report.
 */
static void report_fault(cdh_fault_t fault)
{
    atomic_cas(&pending_fault, FAULT_NONE, fault);
}

/**
 * @brief Load persistent state from FRAM.
 * @param p Output record; zeroed when nothing could be read.
 * @return 0 on success, negative errno otherwise.
 * @todo PLACEHOLDER: no FRAM driver yet (3x TMR FRAM on SPI2, see
 *       ut_core.dts). Until there is one, state lives in RAM and is lost on
 *       every reset, so the reboot lockout and recorded faults never
 *       survive a reboot.
 */
static int fram_load(cdh_persist_t *p)
{
    memset(p, 0, sizeof(*p));
    return -ENOTSUP;
}

/**
 * @brief Write persistent state to FRAM.
 * @param p Record to write.
 * @return 0 on success, negative errno otherwise.
 * @todo PLACEHOLDER: see fram_load().
 */
static int fram_save(const cdh_persist_t *p)
{
    ARG_UNUSED(p);
    return -ENOTSUP;
}

/* ===================================================== */
/* ================= CAN FUNCTIONS ====================== */
/* ===================================================== */

/**
 * @brief Route a decoded packet to its handler by opcode.
 * @param pkt Decoded CAN packet.
 */
static void can_dispatch(const can_packet_t *pkt)
{
    switch (pkt->op)
    {
    case OP_HEARTBEAT:
        handle_heartbeat(pkt);
        break;

    case OP_SET_MODE:
    case OP_REBOOT:
    case OP_SET_LED:
    case OP_SET_PWR_STATE:
        handle_command(pkt);
        break;

    case OP_RESP_SET_PWR_STATE:
    case OP_RESP_GET_POS:
    case OP_RESP_SET_UPDATE_RATE:
    case OP_RESP_SET_WHEEL_RPM:
    case OP_RESP_SET_MAG_DIPOLE:
    case OP_RESP_SET_EPS_LOAD:
        handle_cmd_response(pkt);
        break;

    case OP_GNSS_POS:
    case OP_GNSS_VX:
    case OP_GNSS_VY:
    case OP_GNSS_VZ:
    case OP_GNSS_TIME:
    case OP_MOTOR_WHEEL_RPM:
    case OP_ADCS_WHEEL_RPM:
    case OP_ADCS_MTQ_DIPOLE:
        handle_telemetry(pkt);
        break;

    case OP_SOH_GNSS:
    case OP_SOH_SOLAR:
    case OP_SOH_MOTOR:
    case OP_ADCS_SOH_ATTITUDE:
        handle_health(pkt);
        break;

    default:
        LOG_WRN("Unhandled op: 0x%03X from 0x%02X", pkt->op, pkt->src);
        break;
    }
}

/* ===================================================== */
/* ================= MESSAGE HANDLERS =================== */
/* ===================================================== */

/**
 * @brief Handle a command-response (OP_RESP_*) frame — currently only logs/confirms EPS
 *        power-state acknowledgements.
 * @param pkt Decoded CAN packet.
 */
static void handle_cmd_response(const can_packet_t *pkt)
{
    uint16_t op = pkt->op;

    switch (op)
    {
    case OP_RESP_SET_PWR_STATE:
    {
        uint8_t load_num = pkt->data[0];
        uint8_t state = pkt->data[1];
        uint8_t status = pkt->data[2]; /* 0x00 = success, 0x01 = error */
        if (status == 0x00)
        {
            LOG_INF("EPS confirmed: load %d %s",
                    load_num, state ? "ENABLED" : "DISABLED");
        }
        else
        {
            LOG_ERR("EPS rejected: load %d state=%d (error)", load_num, state);
        }
        break;
    }
    default:
        LOG_INF("CmdResp from node 0x%x, op=0x%03X", pkt->src, op);
        break;
    }
}

/**
 * @brief Handle a telemetry frame; parses GNSS position reports and
 *        logs anything else.
 * @param pkt Decoded CAN packet.
 */
static void handle_telemetry(const can_packet_t *pkt)
{
    if (pkt->src == GNSS_ID && pkt->op == OP_GNSS_POS)
    {
        /* Unpack signed 24-bit big-endian lat/lon (units: 1e-4 degrees) */
        int32_t lat_1e4 = (int32_t)((uint32_t)pkt->data[0] << 16 |
                                    (uint32_t)pkt->data[1] << 8 |
                                    (uint32_t)pkt->data[2]);
        if (lat_1e4 & 0x800000)
            lat_1e4 |= 0xFF000000; /* sign-extend */

        int32_t lon_1e4 = (int32_t)((uint32_t)pkt->data[3] << 16 |
                                    (uint32_t)pkt->data[4] << 8 |
                                    (uint32_t)pkt->data[5]);
        if (lon_1e4 & 0x800000)
            lon_1e4 |= 0xFF000000;

        gnss_latest.lat_deg_e7 = lat_1e4 * 1000; /* back to 1e-7 */
        gnss_latest.lon_deg_e7 = lon_1e4 * 1000;
        gnss_latest.timestamp_ms = k_uptime_get_32();
        gnss_latest.valid = true;

        LOG_INF("GNSS fix: lat=%d lon=%d (1e-7 deg)",
                gnss_latest.lat_deg_e7, gnss_latest.lon_deg_e7);
        return;
    }

    LOG_INF("Telemetry from node 0x%x", pkt->src);
}

/**
 * @brief Handle a state-of-health frame.
 * @param pkt Decoded CAN packet.
 */
static void handle_health(const can_packet_t *pkt)
{
    LOG_INF("Health from node 0x%x", pkt->src);
}

/**
 * @brief Tell EPS to enable/disable a specific load switch.
 * @param load_num Load number (1..12).
 * @param state    1 = enable, 0 = disable.
 */
static void send_set_board_pwr(uint8_t load_num, uint8_t state)
{
    int ret = can_send2(can_dev, NODE_ID, EPS_ID, OP_SET_PWR_STATE, PRIO_LOW,
                        load_num, state);
    if (ret)
    {
        LOG_ERR("Failed to send SET_PWR_STATE (load=%d state=%d): %d",
                load_num, state, ret);
    }
    else
    {
        LOG_INF("TX SET_PWR_STATE load=%d state=%d -> EPS", load_num, state);
    }
}

/**
 * @brief Handle an incoming ground/CAN command and act on its opcode.
 * @param pkt Decoded CAN packet; op is the opcode, data[0..] are args.
 *
 * OP_SET_MODE only posts a request; mode_manager_thread decides whether to
 * act on it. Ground may request SAFE (from any mode) or STANDBY (from SAFE,
 * if the STANDBY guards pass). Every other value is rejected here.
 *
 * @todo OP_REBOOT is currently a no-op (NVIC_SystemReset() is commented
 *       out), pending watchdog kick-sequencing being confirmed safe.
 */
static void handle_command(const can_packet_t *pkt)
{
    uint16_t opcode = pkt->op;
    uint8_t val = pkt->data[0];

    switch (opcode)
    {
    case OP_SET_MODE:
        if (val == MODE_SAFE || val == MODE_STANDBY)
        {
            LOG_INF("Ground mode request: %d", val);
            atomic_set(&ground_mode_request, val);
        }
        else
        {
            LOG_WRN("Ground mode request %d rejected: only SAFE or STANDBY", val);
        }
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
                pkt->data[0], pkt->data[1]);
        send_set_board_pwr(pkt->data[0], pkt->data[1]);
        break;

    default:
        LOG_WRN("Unknown opcode: 0x%03X", opcode);
        break;
    }
}

/**
 * @brief Mark a node alive and record its last-seen timestamp.
 * @param pkt Decoded CAN packet from OP_HEARTBEAT.
 */
static void handle_heartbeat(const can_packet_t *pkt)
{
    node_status_t *node = get_node_status(pkt->src);

    if (node == NULL)
    {
        LOG_WRN("Heartbeat from unknown node: 0x%02X", pkt->src);
        return;
    }

    node->alive = true;
    node->last_seen_ms = k_uptime_get();

    LOG_INF("Heartbeat from 0x%02X - alive", pkt->src);
}

/**
 * @brief Warn about nodes that have gone silent.
 * @note NOTE: SOLAR entry uses SOLAR_ID (not STAR_ID — this fixes the bug
 *       from the original code).
 */
static void check_node_timeouts(void)
{
    int64_t now = k_uptime_get();

    struct
    {
        uint8_t id;
        const char *name;
        node_status_t *status;
    } nodes[] = {
        {EPS_ID, "EPS", &node_alive.eps},
        {COMMS_ID, "COMMS", &node_alive.comms},
        {ADCS_ID, "ADCS", &node_alive.adcs},
        {MOTOR_ID, "MOTOR", &node_alive.motor},
        {GNSS_ID, "GNSS", &node_alive.gnss},
        {STAR_ID, "STAR", &node_alive.star},
        {SOLAR_ID, "SOLAR", &node_alive.solar},
    };

    for (int i = 0; i < ARRAY_SIZE(nodes); i++)
    {
        node_status_t *n = nodes[i].status;

        if (!n->alive)
        {
            continue;
        }

        if ((now - n->last_seen_ms) > HEARTBEAT_TIMEOUT_MS)
        {
            LOG_WRN("Node %s (0x%02X) timed out!", nodes[i].name, nodes[i].id);
            n->alive = false;
        }
    }
}

/* ===================================================== */
/* ================= SAFE MODE BOOT ===================== */
/* ===================================================== */

/**
 * @brief SAFE entry action: payload and ADCS actuators off.
 * @todo PLACEHOLDER: EPS load numbers for the payload and ADCS actuators
 *       are not defined yet. Call send_set_board_pwr(load, 0) for each once
 *       they are.
 */
static void safe_mode_power_down(void)
{
    LOG_INF("SAFE: payload + ADCS actuator power-down (placeholder)");
}

/**
 * @brief Read and clear the hardware reset cause, and count this boot
 *        toward the reboot counters in persist.
 */
static void record_boot(void)
{
    uint32_t cause = 0;
    int err = hwinfo_get_reset_cause(&cause);

    if (err)
    {
        LOG_WRN("Reset cause unavailable: %d", err);
    }
    else
    {
        hwinfo_clear_reset_cause();
    }

    persist.reset_cause = cause;
    persist.reboot_count++;
    persist.rapid_reboot_count++;

    LOG_INF("Boot #%u, reset cause 0x%08X, rapid reboots %u",
            persist.reboot_count, cause, persist.rapid_reboot_count);
}

/**
 * @brief True while too many rapid reboots have happened; CDH then stays in
 *        SAFE until ground clears it (see cdh_clear_faults()).
 */
static bool reboot_lockout(void)
{
    return persist.rapid_reboot_count >= REBOOT_LOCKOUT_COUNT;
}

/**
 * @brief Clear the reboot lockout and the recorded PHOENIX fault.
 * @todo PLACEHOLDER: nothing calls this yet. Wire it to a ground command
 *       once one exists in can_proto.h, and route that command through
 *       mode_manager_thread (it owns persist).
 */
static void __unused cdh_clear_faults(void)
{
    persist.rapid_reboot_count = 0;
    persist.recorded_fault = FAULT_NONE;
    persist.phoenix_attempts = 0;
    atomic_set(&pending_fault, FAULT_NONE);
    fram_save(&persist);
}

/**
 * @brief SAFE-mode boot sequence, run once on every reset: load persistent
 *        state, record the reset cause, verify MCU identity against the
 *        role strap, and bring up CAN. Reports FAULT_BOOT_FAILED if CAN
 *        cannot be started.
 *
 * @todo First boot only: deployment hold (no RF). Needs a "deployed" flag
 *       in persist and RF control through COMMS; neither exists yet.
 */
static void safe_mode_boot(void)
{
    LOG_INF("Entering SAFE mode: boot sequence");

    int err = fram_load(&persist);
    if (err)
    {
        LOG_WRN("FRAM load failed (%d); persistent state starts empty", err);
    }

    record_boot();

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
        (uid_is_mcu2 && pin_is_mcu2))
    {
        LOG_INF("MCU Identity Verified");
    }
    else
    {
        LOG_ERR("UID and PA6 MISMATCH — trusting PA6 hardware pin.");
    }

    err = tcan3403_wakeup();
    if (err)
    {
        LOG_ERR("TCAN3403 wakeup failed: %d", err);
    }
    else
    {
        LOG_INF("TCAN3403 Awake");
    }
    err = can_setup(can_dev, NODE_ID, &rxq, NULL);
    if (err)
    {
        LOG_ERR("CAN Setup failed: %d", err);
        report_fault(FAULT_BOOT_FAILED);
    }
    else
    {
        LOG_INF("Setup complete — waiting 2 s for bus stabilisation");
        k_sleep(K_MSEC(2000));
        atomic_set(&boot_ok, 1);
        safe_mode_power_down();
    }

    if (reboot_lockout())
    {
        LOG_ERR("Reboot lockout: %u rapid reboots, staying in SAFE until ground clears it",
                persist.rapid_reboot_count);
    }

    persist.last_mode = MODE_SAFE;
    fram_save(&persist);

    mode_entered_ms = k_uptime_get();
    atomic_set(&boot_done, 1);
}

/* ===================================================== */
/* ================= DOWNLINK (PLACEHOLDER) ============= */
/* ===================================================== */
/*
 * How data is transferred, stored, and broadcast is not decided yet (COMMS
 * will send images over S-band later). These hooks mark where DOWNLINK
 * plugs into the state machine. Diagram rules to implement: send the
 * oldest un-ACKed data first; mark data sent; delete only after ground ACK.
 */

/** @brief DOWNLINK entry action. @todo PLACEHOLDER: start sending the oldest un-ACKed data. */
static void downlink_begin(void)
{
    LOG_INF("DOWNLINK: begin (placeholder)");
}

/** @brief DOWNLINK exit action. @todo PLACEHOLDER: stop sending; un-ACKed data stays queued. */
static void downlink_end(void)
{
    LOG_INF("DOWNLINK: end (placeholder)");
}

/** @brief True when no captured data is waiting to be sent. @todo PLACEHOLDER: no queue yet. */
static bool downlink_queue_empty(void)
{
    return true;
}

/** @brief True when the ground pass has ended. @todo PLACEHOLDER: pass end detection not defined. */
static bool ground_pass_ended(void)
{
    return false;
}

/**
 * @brief True when ground has been silent for DOWNLINK_TIMEOUT_MS.
 * @todo PLACEHOLDER: ground traffic is not tracked yet, so this measures
 *       time since DOWNLINK started instead of since the last ground traffic.
 */
static bool ground_traffic_timed_out(void)
{
    return (k_uptime_get() - mode_entered_ms) >= DOWNLINK_TIMEOUT_MS;
}

/** @brief True when ground contacts CDH outside a planned pass. @todo PLACEHOLDER: detection not defined. */
static bool unplanned_ground_contact(void)
{
    return false;
}

/** @brief Ground plan for the upcoming pass. @todo PLACEHOLDER: pass plan upload format not defined. */
static pass_plan_t ground_pass_plan(void)
{
    return PASS_PLAN_NONE;
}

/** @brief Percent of image storage (flash) in use. @todo PLACEHOLDER: no image storage yet. */
static uint8_t storage_used_pct(void)
{
    return 0;
}

/* ===================================================== */
/* ================= EXPERIMENT (PLACEHOLDER) =========== */
/* ===================================================== */

/** @brief EXPERIMENT entry action. @todo PLACEHOLDER: power the payload via EPS and start scanning. */
static void experiment_begin(void)
{
    LOG_INF("EXPERIMENT: begin (placeholder)");
}

/**
 * @brief EXPERIMENT exit action.
 * @todo PLACEHOLDER: save the coverage map, add captured data to the
 *       downlink queue, power the payload off.
 */
static void experiment_end(void)
{
    LOG_INF("EXPERIMENT: end (placeholder)");
}

/** @brief True when the scan task finished early. @todo PLACEHOLDER. */
static bool experiment_task_done(void)
{
    return false;
}

/** @brief True when every area of Utah is marked scanned. @todo PLACEHOLDER: no coverage map yet. */
static bool utah_coverage_complete(void)
{
    return false;
}

/**
 * @brief True when every board the experiment needs is alive.
 * @todo PLACEHOLDER: required boards not decided; check node_alive for them.
 */
static bool experiment_boards_alive(void)
{
    return true;
}

/* ===================================================== */
/* ================= MODE STATE MACHINE ================= */
/* ===================================================== */

/** @brief Short mode name for logs. */
static const char *mode_name(cdh_mode_t mode)
{
    static const char *const names[MODE_COUNT] = {
        [MODE_SAFE] = "SAFE",
        [MODE_STANDBY] = "STANDBY",
        [MODE_DOWNLINK] = "DOWNLINK",
        [MODE_EXPERIMENT] = "EXPERIMENT",
        [MODE_PHOENIX] = "PHOENIX",
    };

    return (mode < MODE_COUNT) ? names[mode] : "?";
}

/** @brief True for the operational modes (inside the diagram's OPERATIONAL MODES box). */
static bool mode_is_operational(cdh_mode_t mode)
{
    return mode == MODE_STANDBY || mode == MODE_DOWNLINK || mode == MODE_EXPERIMENT;
}

/** @brief Battery state of charge in percent. @todo PLACEHOLDER: no EPS battery telemetry over CAN yet. */
static uint8_t battery_pct(void)
{
    return 100;
}

/** @brief False on low battery or unsafe temperature (the "unsafe conditions = SAFE" rule). */
static bool power_temp_safe(void)
{
    return battery_pct() > BATTERY_SAFE_PCT && atomic_get(&temp_unsafe) == 0;
}

/** @brief True if the latest GNSS fix is fresh and inside the Utah box. */
static bool gnss_over_utah(void)
{
    gnss_data_t fix = gnss_latest;

    if (!fix.valid || (k_uptime_get_32() - fix.timestamp_ms) > GNSS_FIX_MAX_AGE_MS)
    {
        return false;
    }

    return fix.lat_deg_e7 >= UTAH_LAT_MIN_E7 && fix.lat_deg_e7 <= UTAH_LAT_MAX_E7 &&
           fix.lon_deg_e7 >= UTAH_LON_MIN_E7 && fix.lon_deg_e7 <= UTAH_LON_MAX_E7;
}

/** @brief True if any supervised thread has missed its check-in. */
static bool any_thread_late(void)
{
    for (int id = 0; id < health_count(); id++)
    {
        if (health_is_late(id))
        {
            return true;
        }
    }

    return false;
}

/**
 * @brief Whether a fault may interrupt a DOWNLINK pass. Non-critical faults
 *        wait until the pass ends.
 * @todo PLACEHOLDER critical-fault list: not decided yet. Until it is, no
 *       fault is critical, which matches the supervisor's previous
 *       behaviour of deferring all recovery during a downlink.
 */
static bool fault_is_critical(cdh_fault_t fault)
{
    switch (fault)
    {
    /* case FAULT_xxx: return true; */
    default:
        return false;
    }
}

/**
 * @brief Whether the fault that sent CDH to PHOENIX has gone away.
 * @param fault Recorded fault.
 */
static bool phoenix_fault_cleared(cdh_fault_t fault)
{
    switch (fault)
    {
    case FAULT_NONE:
        return true;
    case FAULT_BOOT_FAILED:
        return atomic_get(&boot_ok) != 0;
    case FAULT_THREAD_HUNG:
        return !any_thread_late();
    default:
        return false;
    }
}

/**
 * @brief Run the next PHOENIX escalation step and count the attempt in FRAM.
 *
 * The attempt is saved before acting so a step that resets the board still
 * counts.
 */
static void phoenix_run_next_step(void)
{
    phoenix_step_t step = MIN(persist.phoenix_attempts, PHOENIX_STEP_COUNT - 1);

    persist.phoenix_attempts++;
    fram_save(&persist);
    phoenix_last_step_ms = k_uptime_get();

    LOG_WRN("PHOENIX: attempt %u, step %d", persist.phoenix_attempts, step);

    switch (step)
    {
    case PHOENIX_STEP_RESTART_THREAD:
        /** @todo PLACEHOLDER: abort and re-create the hung thread. */
        break;
    case PHOENIX_STEP_POWER_CYCLE_BOARD:
        /** @todo PLACEHOLDER: power-cycle the faulty board via EPS (load numbers TBD). */
        break;
    case PHOENIX_STEP_REINIT_CAN:
        /** @todo PLACEHOLDER: stop CAN and re-run can_setup(). */
        break;
    case PHOENIX_STEP_REBOOT:
        /** @todo PLACEHOLDER: sys_reboot() once OP_REBOOT watchdog sequencing is confirmed. */
        break;
    default:
        break;
    }
}

/** @brief PHOENIX entry action: take the pending fault, record it, and run the first step. */
static void phoenix_enter(void)
{
    cdh_fault_t fault = (cdh_fault_t)atomic_set(&pending_fault, FAULT_NONE);

    persist.recorded_fault = fault;
    LOG_ERR("PHOENIX: fault %d, %u earlier attempts", fault, persist.phoenix_attempts);

    phoenix_run_next_step();
}

/**
 * @brief One PHOENIX pass: exit to SAFE once recovered, timed out, or out of
 *        attempts; otherwise escalate after each step has had time to settle.
 * @return Next mode.
 */
static cdh_mode_t phoenix_next(void)
{
    int64_t now = k_uptime_get();

    if (phoenix_fault_cleared((cdh_fault_t)persist.recorded_fault))
    {
        LOG_INF("PHOENIX: fault %d recovered", persist.recorded_fault);
        persist.recorded_fault = FAULT_NONE;
        persist.phoenix_attempts = 0;
        atomic_set(&pending_fault, FAULT_NONE);
        return MODE_SAFE;
    }

    if (persist.phoenix_attempts >= PHOENIX_ATTEMPT_LIMIT ||
        (now - mode_entered_ms) >= PHOENIX_TIMEOUT_MS)
    {
        LOG_ERR("PHOENIX: fault %d not recovered after %u attempts; recorded",
                persist.recorded_fault, persist.phoenix_attempts);
        return MODE_SAFE;
    }

    if ((now - phoenix_last_step_ms) >= PHOENIX_STEP_SETTLE_MS)
    {
        phoenix_run_next_step();
    }

    return MODE_PHOENIX;
}

/**
 * @brief SAFE -> STANDBY guards: boot OK + healthy + battery + no recorded
 *        fault or reboot lockout. Logs the first guard that fails.
 */
static bool standby_guards_pass(void)
{
    if (atomic_get(&boot_ok) == 0)
    {
        LOG_WRN("STANDBY rejected: boot did not complete");
        return false;
    }
    if (atomic_get(&pending_fault) != FAULT_NONE || atomic_get(&temp_unsafe) != 0)
    {
        LOG_WRN("STANDBY rejected: not healthy");
        return false;
    }
    if (battery_pct() < BATTERY_MIN_STANDBY_PCT)
    {
        LOG_WRN("STANDBY rejected: battery %u%% < %u%%", battery_pct(), BATTERY_MIN_STANDBY_PCT);
        return false;
    }
    if (persist.recorded_fault != FAULT_NONE || reboot_lockout())
    {
        LOG_WRN("STANDBY rejected: recorded fault or reboot lockout; ground must clear it");
        return false;
    }

    return true;
}

/** @brief Guards for starting a scan: battery, required boards alive, storage free. */
static bool scan_guards_pass(void)
{
    return battery_pct() >= BATTERY_MIN_SCAN_PCT &&
           experiment_boards_alive() &&
           storage_used_pct() < STORAGE_NEARLY_FULL_PCT;
}

/**
 * @brief Utah pass decision tree (diagram diamonds, top to bottom).
 * @return Mode to enter for this pass; MODE_STANDBY to skip it.
 */
static cdh_mode_t pass_decision(void)
{
    switch (ground_pass_plan())
    {
    case PASS_PLAN_DOWNLINK:
        return MODE_DOWNLINK;
    case PASS_PLAN_SCAN:
        return MODE_EXPERIMENT;
    default:
        break;
    }

    if (storage_used_pct() >= STORAGE_NEARLY_FULL_PCT)
    {
        return MODE_DOWNLINK; /* downlink first */
    }

    if (utah_coverage_complete() && !downlink_queue_empty())
    {
        return MODE_DOWNLINK;
    }

    /* Coverage incomplete, or complete with nothing to send: scan anyway,
     * since already-scanned areas are rescanned to collect more data. */
    return scan_guards_pass() ? MODE_EXPERIMENT : MODE_STANDBY;
}

/**
 * @brief Pick the next mode, in the diagram's priority order:
 *        1. fault -> PHOENIX, 2. unsafe power/temp or ground SAFE -> SAFE,
 *        3. unplanned ground contact -> DOWNLINK, 4. Utah pass -> pass
 *        decision, 5. otherwise stay. Per-mode exits are checked last.
 * @param mode Current mode.
 * @param over_utah Latest GNSS fix is over Utah.
 * @param pass_starting over_utah just became true.
 * @return Next mode (== mode for no transition).
 */
static cdh_mode_t mode_next(cdh_mode_t mode, bool over_utah, bool pass_starting)
{
    cdh_fault_t fault = (cdh_fault_t)atomic_get(&pending_fault);
    cdh_mode_t ground = (cdh_mode_t)atomic_set(&ground_mode_request, MODE_COUNT);

    /* 1. Fault -> PHOENIX. During DOWNLINK only critical faults interrupt
     *    the pass; the rest stay pending until it ends. Out of attempts,
     *    CDH stays in SAFE with the fault recorded. */
    if (fault != FAULT_NONE && mode != MODE_PHOENIX &&
        !(mode == MODE_DOWNLINK && !fault_is_critical(fault)))
    {
        return (persist.phoenix_attempts < PHOENIX_ATTEMPT_LIMIT) ? MODE_PHOENIX : MODE_SAFE;
    }

    /* 2. Unsafe power/temp or ground SAFE command -> SAFE. */
    if (mode_is_operational(mode) && (ground == MODE_SAFE || !power_temp_safe()))
    {
        return MODE_SAFE;
    }

    switch (mode)
    {
    case MODE_SAFE:
        if (ground == MODE_STANDBY && standby_guards_pass())
        {
            return MODE_STANDBY;
        }
        break;

    case MODE_STANDBY:
        if (unplanned_ground_contact()) /* 3 */
        {
            return MODE_DOWNLINK;
        }
        if (pass_starting) /* 4 */
        {
            return pass_decision();
        }
        break;

    case MODE_DOWNLINK:
        if (downlink_queue_empty() || ground_pass_ended() || ground_traffic_timed_out())
        {
            return MODE_STANDBY;
        }
        break;

    case MODE_EXPERIMENT:
        /* Abort comes from ground SAFE (step 2). */
        if (!over_utah || experiment_task_done() ||
            (k_uptime_get() - mode_entered_ms) >= EXPERIMENT_TIMEOUT_MS)
        {
            return MODE_STANDBY;
        }
        break;

    case MODE_PHOENIX:
        return phoenix_next();

    default:
        break;
    }

    if (ground == MODE_STANDBY && mode != MODE_SAFE)
    {
        LOG_WRN("Ground STANDBY request ignored: only valid from SAFE");
    }

    return mode; /* 5 */
}

/**
 * @brief Run exit actions for @p from, switch modes, run entry actions for
 *        @p to, and save the new mode to FRAM.
 */
static void mode_transition(cdh_mode_t from, cdh_mode_t to)
{
    LOG_INF("Mode %s -> %s", mode_name(from), mode_name(to));

    switch (from)
    {
    case MODE_DOWNLINK:
        downlink_end();
        break;
    case MODE_EXPERIMENT:
        experiment_end();
        break;
    default:
        break;
    }

    current_mode = to;
    mode_entered_ms = k_uptime_get();

    switch (to)
    {
    case MODE_SAFE:
        safe_mode_power_down();
        break;
    case MODE_DOWNLINK:
        downlink_begin();
        break;
    case MODE_EXPERIMENT:
        experiment_begin();
        break;
    case MODE_PHOENIX:
        phoenix_enter();
        break;
    default:
        break;
    }

    persist.last_mode = to;
    fram_save(&persist);
}

/**
 * @brief Once CDH has stayed up for REBOOT_STABLE_MS, this boot no longer
 *        counts as rapid. A locked-out board keeps its count until ground
 *        clears it.
 */
static void update_rapid_reboot_count(void)
{
    if (persist.rapid_reboot_count != 0 && !reboot_lockout() &&
        k_uptime_get() >= REBOOT_STABLE_MS)
    {
        persist.rapid_reboot_count = 0;
        fram_save(&persist);
    }
}

/* ===================================================== */
/* ================= THREADS ============================ */
/* ===================================================== */

/**
 * @brief Drains the hardware CAN RX FIFO into the software processing
 *        queue in every mode (SAFE still accepts ground commands).
 */
void can_rx_thread(void *a, void *b, void *c)
{
    LOG_INF("can_rx_thread started");
    struct can_frame rx;

    while (1)
    {
        health_checkin(hb_can_rx);

        if (k_msgq_get(&rxq, &rx, K_MSEC(CAN_RX_WAIT_MS)) == 0)
        {
            LOG_INF("RX: id=0x%08X dlc=%d op=0x%03X src=0x%02X",
                    rx.id, rx.dlc, (unsigned)CAN_ID_OP(rx.id), (unsigned)CAN_ID_SRC(rx.id));

            if (k_msgq_put(&can_proc_q, &rx, K_NO_WAIT) != 0)
            {
                LOG_WRN("CAN proc queue full — frame dropped");
            }
        }
    }
}

/**
 * @brief Decodes and dispatches frames handed off from can_rx_thread.
 */
void can_process_thread(void *a, void *b, void *c)
{
    LOG_INF("can_process_thread started");
    struct can_frame frame;
    can_packet_t pkt;

    while (1)
    {
        health_checkin(hb_can_proc);

        if (k_msgq_get(&can_proc_q, &frame, K_MSEC(CAN_RX_WAIT_MS)) == 0)
        {
            led_can_activity();
            if (can_decode(&frame, &pkt) != 0)
            {
                continue;
            }

            if (pkt.dst != NODE_ID && pkt.dst != CAN_BROADCAST)
            {
                continue;
            }

            can_dispatch(&pkt);
        }
    }
}

/**
 * @brief Broadcasts a heartbeat once per second once CAN is up, in every
 *        mode. Payload byte 0 carries current_mode; other boards follow it
 *        and fall back to SAFE if the heartbeat stops.
 */
void scheduler_thread(void *a, void *b, void *c)
{
    LOG_INF("scheduler_thread started");
    int64_t last_hb = 0;

    while (1)
    {
        health_checkin(hb_sched);

        if (atomic_get(&boot_ok) != 0)
        {
            int64_t now = k_uptime_get();

            if (now - last_hb >= 1000)
            {
                last_hb = now;
                int err = can_send2(can_dev, NODE_ID, CAN_BROADCAST, OP_HEARTBEAT,
                                    PRIO_LOW, (uint8_t)current_mode, 0);
                if (err)
                {
                    LOG_WRN("Failed to send OP_HEARTBEAT: %d", err);
                }
            }
        }

        k_sleep(K_MSEC(50));
    }
}

/**
 * @brief True if every sensor that read successfully is within
 *        TEMP_SAFE_MIN_C..TEMP_SAFE_MAX_C. Failed reads are skipped.
 * @todo Decide whether a failed sensor read is itself a fault.
 */
static bool temps_within_limits(const struct temp_telemetry *t)
{
    for (int i = 0; i < NUM_TEMP_SENSORS; i++)
    {
        if (t->s[i].status != 0)
        {
            continue;
        }

        int32_t temp_c = t->s[i].temp_q4 / 16;

        if (temp_c < TEMP_SAFE_MIN_C || temp_c > TEMP_SAFE_MAX_C)
        {
            return false;
        }
    }

    return true;
}

/**
 * @brief Periodically reads onboard temperature sensors, flags unsafe
 *        temperatures for the mode manager, and checks for timed-out
 *        remote nodes. Runs in every mode.
 */
void soh_thread(void *a, void *b, void *c)
{
    LOG_INF("soh_thread started");

    while (1)
    {
        health_checkin(hb_soh);

        struct temp_telemetry telem;
        temp_telemetry_read_all(i2c_bus, &telem);
        temp_telemetry_print(&telem);
        atomic_set(&temp_unsafe, temps_within_limits(&telem) ? 0 : 1);
        check_node_timeouts();

        k_sleep(K_MSEC(500));
    }
}

/**
 * @brief Register every supervised thread and start the task watchdog.
 *        Must run before any thread is created.
 */
static void health_init(void)
{
    hb_can_rx = health_register("can_rx", HEALTH_TIMEOUT_CAN_RX_MS);
    hb_can_proc = health_register("can_proc", HEALTH_TIMEOUT_CAN_PROC_MS);
    hb_sched = health_register("scheduler", HEALTH_TIMEOUT_SCHED_MS);
    hb_soh = health_register("soh", HEALTH_TIMEOUT_SOH_MS);
    hb_gnss = health_register("gnss", HEALTH_TIMEOUT_GNSS_MS);
    hb_telemetry = health_register("telemetry", HEALTH_TIMEOUT_TELEMETRY_MS);
    hb_mode_mgr = health_register("mode_mgr", HEALTH_TIMEOUT_MODE_MGR_MS);

    /* NULL = no hardware watchdog fallback yet (see prj.conf). */
    int ret = task_wdt_init(NULL);

    if (ret != 0)
    {
        LOG_ERR("task_wdt_init failed: %d", ret);
    }
}

/**
 * @brief Task watchdog callback for the supervisor's own channel. Fires only
 *        if watchdog_thread stops running, i.e. the supervisor itself hung.
 *
 * Runs in timer interrupt context: it must not block, sleep, or log.
 * Fires once per expiry; the channel re-arms on the next task_wdt_feed().
 */
static void supervisor_wdt_expired(int channel_id, void *user_data)
{
    ARG_UNUSED(channel_id);
    ARG_UNUSED(user_data);

    atomic_inc(&supervisor_wdt_expiries);

    /** @todo Reset logic for a hung supervisor is not decided yet. */
}

/**
 * @brief Called on every supervisor pass while a supervised thread is late.
 *        Reports FAULT_THREAD_HUNG; the mode manager decides when to enter
 *        PHOENIX (deferred until a DOWNLINK pass ends unless critical).
 * @param id health.h id of a thread that has missed its check-in.
 * @todo Pass the hung thread's id to PHOENIX for its "restart hung thread"
 *       step. If the hung thread is mode_mgr itself, nothing acts on the
 *       fault; that case needs its own handling.
 */
static void handle_thread_failure(int id)
{
    ARG_UNUSED(id);

    report_fault(FAULT_THREAD_HUNG);
}

/**
 * @brief Highest-priority thread: supervises the other threads' check-ins.
 *
 * Each pass it feeds its own task watchdog channel (proving the supervisor
 * itself is running), checks every thread in the health table, logs when a
 * thread goes late or recovers, and calls handle_thread_failure() while any
 * thread is late.
 */
void watchdog_thread(void *a, void *b, void *c)
{
    LOG_INF("watchdog_thread started");

    int wdt_channel = task_wdt_add(SUPERVISOR_WDT_TIMEOUT_MS,
                                   supervisor_wdt_expired, NULL);
    if (wdt_channel < 0)
    {
        LOG_ERR("task_wdt_add failed: %d", wdt_channel);
    }

    bool was_late[HEALTH_MAX_ENTRIES] = {false};
    atomic_val_t expiries_reported = 0;

    while (1)
    {
        task_wdt_feed(wdt_channel);

        int first_late = -1;

        for (int id = 0; id < health_count(); id++)
        {
            bool late = health_is_late(id);

            if (late && !was_late[id])
            {
                LOG_ERR("Thread %s missed its check-in: last seen %u ms ago (limit %u ms)",
                        health_name(id), health_ms_since_checkin(id),
                        health_timeout_ms(id));
            }
            else if (!late && was_late[id])
            {
                LOG_WRN("Thread %s is checking in again", health_name(id));
            }
            was_late[id] = late;

            if (late && first_late < 0)
            {
                first_late = id;
            }
        }

        if (first_late >= 0)
        {
            handle_thread_failure(first_late);
        }

        atomic_val_t expiries = atomic_get(&supervisor_wdt_expiries);

        if (expiries != expiries_reported)
        {
            LOG_ERR("Supervisor task watchdog expired (%ld total)", (long)expiries);
            expiries_reported = expiries;
        }

        /** @todo Kick the external watchdog IC here (gpio_pin_toggle(gpioa,
         *        WATCHDOG_PIN)) once EE confirms its kick requirements. */

        k_sleep(K_MSEC(WATCHDOG_KICK_MS));
    }
}

/**
 * @brief Periodically requests a position fix from the GNSS node.
 * Sends OP_GET_POS; GNSS replies with OP_GNSS_POS telemetry.
 */
void gnss_thread(void *a, void *b, void *c)
{
    LOG_INF("gnss_thread started");

    while (1)
    {
        health_checkin(hb_gnss);

        /* Operational modes only: STANDBY watches for passes, EXPERIMENT and
         * DOWNLINK need position to know when the pass ends. */
        if (mode_is_operational(current_mode))
        {
            /* Request position from GNSS node — it responds with OP_GNSS_POS
               as telemetry, which handle_telemetry() parses above */
            int err = can_send2(can_dev, NODE_ID, GNSS_ID, OP_GET_POS, PRIO_LOW, 0, 0);
            if (err)
            {
                LOG_WRN("Failed to send OP_GET_POS: %d", err);
            }
            LOG_INF("GNSS position request sent");
        }

        k_sleep(K_MSEC(GNSS_POLL_MS));
    }
}

/**
 * @brief Periodically assembles and downlinks telemetry (currently a
 *        placeholder). Runs in every mode once CAN is up: SAFE still sends
 *        health telemetry.
 * @todo Implement actual SOH collection, threshold checks, and
 *       downlink-window gating — body is currently just a log line.
 */
void telemetry_thread(void *a, void *b, void *c)
{
    LOG_INF("telemetry_thread started");

    while (1)
    {
        health_checkin(hb_telemetry);

        if (atomic_get(&boot_ok) != 0)
        {
            /** @todo TODO: Collect SOH, check thresholds, downlink if window open */
            LOG_INF("telemetry check - placeholder");
        }

        k_sleep(K_MSEC(TELEMETRY_CHECK_MS));
    }
}

/**
 * @brief Runs the mode state machine every MODE_MGR_PERIOD_MS. The only
 *        thread that writes current_mode or, after boot, persist.
 *
 * Waits (still checking in) until safe_mode_boot() finishes, since boot
 * owns persist until then.
 */
void mode_manager_thread(void *a, void *b, void *c)
{
    LOG_INF("mode_manager_thread started");

    while (atomic_get(&boot_done) == 0)
    {
        health_checkin(hb_mode_mgr);
        k_sleep(K_MSEC(MODE_MGR_PERIOD_MS));
    }

    bool was_over_utah = false;

    while (1)
    {
        health_checkin(hb_mode_mgr);

        update_rapid_reboot_count();

        bool over_utah = gnss_over_utah();
        bool pass_starting = over_utah && !was_over_utah;
        was_over_utah = over_utah;

        cdh_mode_t mode = current_mode;
        cdh_mode_t next = mode_next(mode, over_utah, pass_starting);

        if (next != mode)
        {
            mode_transition(mode, next);
        }

        k_sleep(K_MSEC(MODE_MGR_PERIOD_MS));
    }
}

/* ===================================================== */
/* ================= MAIN =============================== */
/* ===================================================== */

/**
 * @brief Entry point: initializes LEDs, spawns all threads, then runs the
 *        SAFE boot sequence (safe_mode_boot()) before idling forever.
 * @return Does not return under normal operation.
 */
int main(void)
{
    LOG_INF("UT-CORE CDH Booting...");

    leds_init();
    health_init();

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

    k_thread_create(&mode_mgr_thread_data, mode_mgr_stack, STACK_SIZE,
                    mode_manager_thread, NULL, NULL, NULL,
                    PRIO_MODE_MGR, 0, K_NO_WAIT);

    safe_mode_boot();

    while (1)
    {
        k_sleep(K_FOREVER);
    }
}
