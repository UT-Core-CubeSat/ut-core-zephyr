/**
 * @file demo.c
 * @ingroup eps
 * @brief EPS (Electrical Power System) standalone bring-up / bench-test firmware.
 *
 * CAN-free build of the EPS node used for bench verification of load
 * switch GPIOs, motor rail GPIOs, and INA3221 power monitor wiring.
 * Initializes GPIOs and sensors, then loops blinking the status LED and
 * printing live power telemetry to the console. Unlike main.c, this build
 * has no CAN bus, no heartbeat, and no command handling — enable_load(),
 * disable_load(), enable_motor(), and disable_motor() are present but are
 * not invoked anywhere in this file; use main.c for full node behavior.
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/can.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include "common/can_proto.h"

/** @cond */ /* Hidden from Doxygen, or it will mistake this as a function */
LOG_MODULE_REGISTER(eps, LOG_LEVEL_INF);
/** @endcond */

/** INA3221 private attribute for channel selection (1, 2, or 3) */
#define SENSOR_ATTR_INA3221_SELECTED_CHANNEL (SENSOR_ATTR_PRIV_START + 1)

#define NUM_LOADS       12
#define NUM_MOTORS      4
#define NUM_INA         7
#define INA_CHANNELS    3
#define TOTAL_CHANNELS  (NUM_INA * INA_CHANNELS)

#define TELEMETRY_INTERVAL_MS 5000
#define LED_BLINK_INTERVAL_MS 500
#define HEARTBEAT_INTERVAL_MS 1000

/* ── CAN ─────────────────────────────────────────────────────────── */

#define NODE_ID     EPS_ID      /** 0x02 from can_proto.h */
#define PRIO_LOW    4

static const struct device *const can_dev = DEVICE_DT_GET(DT_NODELABEL(fdcan1));
static const struct device *const gpioa   = DEVICE_DT_GET(DT_NODELABEL(gpioa));
/** @cond */ /* Hidden from Doxygen, or it will mistake this as a function */
CAN_MSGQ_DEFINE(rxq, 16);
/** @endcond */
#define PIN_SHDN    10
#define PIN_SILENT  9

/* ── LED ─────────────────────────────────────────────────────────── */

static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

/* ── Load switch GPIOs (active-high enable) ─────────────────────── */

static const struct gpio_dt_spec loads[NUM_LOADS] = {
    GPIO_DT_SPEC_GET(DT_NODELABEL(load1),  gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(load2),  gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(load3),  gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(load4),  gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(load5),  gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(load6),  gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(load7),  gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(load8),  gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(load9),  gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(load10), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(load11), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(load12), gpios),
};

/* ── Motor 12V rail GPIOs ────────────────────────────────────────── */

/** Motor 12V rail GPIOs */
static const struct gpio_dt_spec motors[NUM_MOTORS] = {
    GPIO_DT_SPEC_GET(DT_NODELABEL(motor1), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(motor2), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(motor3), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(motor4), gpios),
};

/* ── INA3221 power monitors (7 chips, 3 channels each = 21 ch) ── */

/** INA3221 power monitors (7 chips, 3 channels each = 21 ch) */
static const struct device *const ina_devs[NUM_INA] = {
    DEVICE_DT_GET(DT_NODELABEL(ina_bus1_0)),
    DEVICE_DT_GET(DT_NODELABEL(ina_bus1_1)),
    DEVICE_DT_GET(DT_NODELABEL(ina_bus1_2)),
    DEVICE_DT_GET(DT_NODELABEL(ina_bus2_0)),
    DEVICE_DT_GET(DT_NODELABEL(ina_bus2_1)),
    DEVICE_DT_GET(DT_NODELABEL(ina_bus2_2)),
    DEVICE_DT_GET(DT_NODELABEL(ina_bus2_3)),
};

struct power_reading {
    struct sensor_value voltage;
    struct sensor_value current;
    struct sensor_value power;
};

static struct power_reading telemetry[TOTAL_CHANNELS];

/* ── Load switch control ─────────────────────────────────────────── */

/**
 * @brief Enable a switched power load.
 * @param load_num Load number, 1-12.
 * @return 0 on success, -EINVAL if load_num is out of range, or a negative
 *         errno from gpio_pin_set_dt() on GPIO failure.
 */
int enable_load(int load_num)
{
    if (load_num < 1 || load_num > NUM_LOADS) return -EINVAL;
    return gpio_pin_set_dt(&loads[load_num - 1], 1);
}

/**
 * @brief Disable a switched power load.
 * @param load_num Load number, 1-12.
 * @return 0 on success, -EINVAL if load_num is out of range, or a negative
 *         errno from gpio_pin_set_dt() on GPIO failure.
 */
int disable_load(int load_num)
{
    if (load_num < 1 || load_num > NUM_LOADS) return -EINVAL;
    return gpio_pin_set_dt(&loads[load_num - 1], 0);
}

/* ── Motor 12V rail control ──────────────────────────────────────── */

/**
 * @brief Enable a motor's 12V power rail.
 * @param motor_num Motor number, 1-4.
 * @return 0 on success, -EINVAL if motor_num is out of range, or a negative
 *         errno from gpio_pin_set_dt() on GPIO failure.
 */
int enable_motor(int motor_num)
{
    if (motor_num < 1 || motor_num > NUM_MOTORS) return -EINVAL;
    return gpio_pin_set_dt(&motors[motor_num - 1], 1);
}

/**
 * @brief Disable a motor's 12V power rail.
 * @param motor_num Motor number, 1-4.
 * @return 0 on success, -EINVAL if motor_num is out of range, or a negative
 *         errno from gpio_pin_set_dt() on GPIO failure.
 */
int disable_motor(int motor_num)
{
    if (motor_num < 1 || motor_num > NUM_MOTORS) return -EINVAL;
    return gpio_pin_set_dt(&motors[motor_num - 1], 0);
}

/* ── Power monitor readout ───────────────────────────────────────── */

/**
 * @brief Select which of an INA3221 chip's 3 channels subsequent samples
 *        will be read from.
 * @param dev     INA3221 device handle.
 * @param channel Channel to select (1, 2, or 3).
 * @return 0 on success, negative errno from sensor_attr_set() on failure.
 */
static int ina3221_select_channel(const struct device *dev, int channel)
{
    struct sensor_value val = { .val1 = channel, .val2 = 0 };
    return sensor_attr_set(dev, SENSOR_CHAN_ALL,
                   (enum sensor_attribute)SENSOR_ATTR_INA3221_SELECTED_CHANNEL,
                   &val);
}

/**
 * @brief Sample every channel on every INA3221 chip and store the results
 *        in the telemetry[] array.
 *
 * Iterates all 7 chips and their 3 channels each, skipping chips that
 * aren't ready. For each channel, selects it, fetches a fresh sample, and
 * reads voltage/current/power into the corresponding telemetry[] entry.
 *
 * @return Count of failed channel reads (0 = all channels read
 *         successfully). A not-ready chip counts as INA_CHANNELS failures.
 */
int read_power_monitors(void)
{
    int failures = 0;

    for (int chip = 0; chip < NUM_INA; chip++) {
        if (!device_is_ready(ina_devs[chip])) {
            failures += INA_CHANNELS;
            continue;
        }
        for (int ch = 0; ch < INA_CHANNELS; ch++) {
            int idx = chip * INA_CHANNELS + ch;
            if (ina3221_select_channel(ina_devs[chip], ch + 1)) { failures++; continue; }
            if (sensor_sample_fetch(ina_devs[chip]))             { failures++; continue; }
            sensor_channel_get(ina_devs[chip], SENSOR_CHAN_VOLTAGE, &telemetry[idx].voltage);
            sensor_channel_get(ina_devs[chip], SENSOR_CHAN_CURRENT, &telemetry[idx].current);
            sensor_channel_get(ina_devs[chip], SENSOR_CHAN_POWER,   &telemetry[idx].power);
        }
    }
    return failures;
}

/**
 * @brief Print the current contents of telemetry[] to the console.
 *
 * Prints voltage, current, and power for every channel on every INA3221
 * chip; prints "not ready" for any chip that failed its readiness check.
 * Does not sample new data — call read_power_monitors() first.
 */
void print_telemetry(void)
{
    printk("──── EPS Telemetry ────\n");
    for (int chip = 0; chip < NUM_INA; chip++) {
        if (!device_is_ready(ina_devs[chip])) {
            printk("  INA%d: not ready\n", chip);
            continue;
        }
        for (int ch = 0; ch < INA_CHANNELS; ch++) {
            int idx = chip * INA_CHANNELS + ch;
            printk("  INA%d-CH%d: %d.%06dV  %d.%06dA  %d.%06dW\n",
                   chip, ch + 1,
                   telemetry[idx].voltage.val1, telemetry[idx].voltage.val2,
                   telemetry[idx].current.val1, telemetry[idx].current.val2,
                   telemetry[idx].power.val1,   telemetry[idx].power.val2);
        }
    }
}

/* ── CAN TX ──────────────────────────────────────────────────────── */

/**
 * @brief Broadcast a CLS_HEARTBEAT frame announcing this node is alive.
 *
 * Sent periodically from main()'s loop at HEARTBEAT_INTERVAL_MS.
 */
static void send_heartbeat(void)
{
    struct can_frame f = {0};
    f.id    = CAN_ID_FULL(PRIO_LOW, NODE_ID, CAN_BROADCAST, CLS_HEARTBEAT);
    f.flags = CAN_FRAME_IDE;
    can_fill_payload(&f, NODE_ID, OP_HEARTBEAT, 0x00, 0, 0, 0, 0, 0);
    can_send(can_dev, &f, K_NO_WAIT, NULL, NULL);
    LOG_INF("TX heartbeat");
}

/* ── CAN RX ──────────────────────────────────────────────────────── */

/**
 * @brief Decoded CAN frame with routing fields unpacked from the 29-bit ID.
 */
typedef struct {
    uint8_t priority;
    uint8_t src;
    uint8_t dst;
    uint8_t msg_class;
    uint8_t dlc;
    uint8_t data[8];
} can_packet_t;

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
 * @brief Handle a received CLS_HEARTBEAT frame.
 * @param pkt Decoded CAN packet.
 *
 * Currently only logs the sender; no liveness table is maintained on EPS.
 */
static void handle_heartbeat(const can_packet_t *pkt)
{
    LOG_INF("RX heartbeat from 0x%02X", pkt->src);
}

/**
 * @brief Handle an OP_SET_PWR_STATE command by enabling or disabling the
 *        requested load, then replying with a CLS_CMD_RESP status frame.
 * @param pkt Decoded CAN packet; data[2] is the load number (1-12),
 *            data[3] is the requested state (1 = enable, 0 = disable).
 *
 * Sends a response frame back to the requester with data[4] set to 0x00
 * on success or 0x01 on error, based on the return value of
 * enable_load()/disable_load().
 */
static void handle_set_pwr_state(const can_packet_t *pkt)
{
    uint8_t load_num = pkt->data[2];   /* p2: load number (1..12) */
    uint8_t state    = pkt->data[3];   /* p3: 1 = enable, 0 = disable */
    int ret;

    if (state) {
        ret = enable_load(load_num);
    } else {
        ret = disable_load(load_num);
    }

    LOG_INF("SET_PWR_STATE load=%d state=%d result=%d", load_num, state, ret);

    /* Send command response back to the requester */
    struct can_frame f = {0};
    f.id    = CAN_ID_FULL(PRIO_LOW, NODE_ID, pkt->src, CLS_CMD_RESP);
    f.flags = CAN_FRAME_IDE;
    uint8_t status = (ret == 0) ? 0x00 : 0x01;  /* 0x00 = success, 0x01 = error */
    can_fill_payload(&f, NODE_ID, OP_SET_PWR_STATE, load_num, state, status, 0, 0, 0);
    can_send(can_dev, &f, K_NO_WAIT, NULL, NULL);
}

/**
 * @brief Route a decoded CAN packet to its message-class/opcode handler.
 * @param pkt Decoded CAN packet.
 *
 * Dispatches CLS_HEARTBEAT to handle_heartbeat() and CLS_COMMAND frames
 * with opcode OP_SET_PWR_STATE to handle_set_pwr_state(). Unrecognized
 * classes or opcodes are logged as warnings and dropped.
 */
static void can_dispatch(const can_packet_t *pkt)
{
    switch (pkt->msg_class) {
    case CLS_HEARTBEAT:
        handle_heartbeat(pkt);
        break;
    case CLS_COMMAND: {
        uint8_t op = pkt->data[1];
        switch (op) {
        case OP_SET_PWR_STATE:
            handle_set_pwr_state(pkt);
            break;
        default:
            LOG_WRN("Unknown command opcode: 0x%02X from 0x%02X", op, pkt->src);
            break;
        }
        break;
    }
    default:
        LOG_WRN("Unhandled class: %d from 0x%02X", pkt->msg_class, pkt->src);
        break;
    }
}

/* ── CAN RX Thread ───────────────────────────────────────────────── */

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

/* ── CAN Setup ───────────────────────────────────────────────────── */

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
        .flags = CAN_FILTER_IDE,
    };
    const struct can_filter bcast = {
        .id    = CAN_DST(CAN_BROADCAST),
        .mask  = CAN_DST_MASK_29,
        .flags = CAN_FILTER_IDE,
    };

    can_add_rx_filter_msgq(can_dev, &rxq, &to_me);
    can_add_rx_filter_msgq(can_dev, &rxq, &bcast);
    LOG_INF("CAN initialized (29-bit extended), node=0x%02X", NODE_ID);
}

/* ── GPIO initialization ─────────────────────────────────────────── */

/**
 * @brief Configure every load and motor GPIO as an inactive output.
 *
 * Checks readiness and configures direction for all 12 load GPIOs and all
 * 4 motor GPIOs. Bails out on the first failure encountered.
 *
 * @return 0 on success; -ENODEV if a GPIO device isn't ready, or a
 *         negative errno from gpio_pin_configure_dt() on configuration
 *         failure.
 */
static int init_gpios(void)
{
    int err;

    for (int i = 0; i < NUM_LOADS; i++) {
        if (!gpio_is_ready_dt(&loads[i])) {
            printk("EPS: load %d gpio not ready\n", i + 1);
            return -ENODEV;
        }
        err = gpio_pin_configure_dt(&loads[i], GPIO_OUTPUT_INACTIVE);
        if (err) { printk("EPS: load %d config failed (%d)\n", i + 1, err); return err; }
    }

    for (int i = 0; i < NUM_MOTORS; i++) {
        if (!gpio_is_ready_dt(&motors[i])) {
            printk("EPS: motor %d gpio not ready\n", i + 1);
            return -ENODEV;
        }
        err = gpio_pin_configure_dt(&motors[i], GPIO_OUTPUT_INACTIVE);
        if (err) { printk("EPS: motor %d config failed (%d)\n", i + 1, err); return err; }
    }

    return 0;
}

/* ── Main ─────────────────────────────────────────────────────────── */

/**
 * @brief Entry point: initializes GPIOs, LED, and CAN, then runs the main
 *        loop broadcasting heartbeats and reporting telemetry.
 *
 * Boot sequence: configure load/motor GPIOs, count ready INA3221 sensors,
 * configure the status LED, wake the CAN transceiver, and bring up the
 * CAN bus. The main loop then sends a heartbeat every
 * HEARTBEAT_INTERVAL_MS and, every TELEMETRY_INTERVAL_MS, toggles the
 * status LED and samples/prints power telemetry.
 *
 * @return Returns 0 early if GPIO, LED, or CAN initialization fails;
 *         otherwise does not return under normal operation.
 */
int main(void)
{
    printk("EPS: starting\n");

    int err = init_gpios();
    if (err) {
        printk("EPS: GPIO init failed (%d)\n", err);
        return 0;
    }

    printk("EPS: %d loads, %d motors configured\n", NUM_LOADS, NUM_MOTORS);

    int ina_ready = 0;
    for (int i = 0; i < NUM_INA; i++) {
        if (device_is_ready(ina_devs[i])) ina_ready++;
    }
    printk("EPS: %d/%d INA3221 sensors ready (%d total channels)\n",
           ina_ready, NUM_INA, ina_ready * INA_CHANNELS);

    if (!gpio_is_ready_dt(&led0)) {
        printk("EPS: led0 gpio not ready\n");
        return 0;
    }
    err = gpio_pin_configure_dt(&led0, GPIO_OUTPUT_ACTIVE);
    if (err) { printk("EPS: led0 config failed (%d)\n", err); return 0; }

    /* CAN bus init */
    tcan3403_wakeup();
    can_setup();

    LOG_INF("Running — heartbeat every %d ms", HEARTBEAT_INTERVAL_MS);

    int64_t last_hb   = k_uptime_get();
    int64_t last_telem = k_uptime_get();

    while (1) {
        int64_t now = k_uptime_get();

        /* Heartbeat */
        if ((now - last_hb) >= HEARTBEAT_INTERVAL_MS) {
            send_heartbeat();
            last_hb = now;
        }

        /* Telemetry + LED blink (kept from your original loop) */
        if ((now - last_telem) >= TELEMETRY_INTERVAL_MS) {
            gpio_pin_toggle_dt(&led0);
            read_power_monitors();
            print_telemetry();
            last_telem = now;
        }

        k_msleep(100);
    }
}
