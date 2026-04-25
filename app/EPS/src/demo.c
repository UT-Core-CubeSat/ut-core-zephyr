/*
 * UT-CORE EPS — CAN-Integrated Power System
 *
 * Hardware: STM32U5 microcontroller, Zephyr RTOS.
 * Bus:      FDCAN1 at 500 kbit/s, 29-bit extended IDs.
 * Sensors:  7x INA3221 (21 power channels), 6x TMP1xx temp sensors.
 * Loads:    12 load switches (GPIO, active-high), 4 motor 12V rails.
 *
 * CAN handlers:
 *   RX: SetBoardPwrState — CDH commands a load or motor on/off
 *   TX: Heartbeat        — 1 Hz broadcast
 *   TX: SOH              — voltages, currents, temps to CDH every 5 s
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/can.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include "C:\Users\notbr\Documents\all_coding\ut-core\common\can_proto.h"
#include "C:\Users\notbr\Documents\all_coding\ut-core\common\temp_telemetry.h"

LOG_MODULE_REGISTER(eps, LOG_LEVEL_INF);

/* ===================================================== */
/* ================= NODE IDENTITY ===================== */
/* ===================================================== */

#define NODE_ID     EPS_ID      /* 0x02 */
#define PRIO_LOW    4

/* ===================================================== */
/* ================= INA3221 PRIVATE ATTR ============== */
/* ===================================================== */

#define SENSOR_ATTR_INA3221_SELECTED_CHANNEL (SENSOR_ATTR_PRIV_START + 1)

#define NUM_LOADS       12
#define NUM_MOTORS      4
#define NUM_INA         7
#define INA_CHANNELS    3
#define TOTAL_CHANNELS  (NUM_INA * INA_CHANNELS)

/* ===================================================== */
/* ================= HW DEVICES ======================== */
/* ===================================================== */

static const struct device *const can_dev  = DEVICE_DT_GET(DT_NODELABEL(fdcan1));
static const struct device *const gpioa    = DEVICE_DT_GET(DT_NODELABEL(gpioa));
static const struct device *const i2c_bus  = DEVICE_DT_GET(DT_NODELABEL(i2c1));

/* Transceiver pins — adjust to your schematic */
#define PIN_SHDN    10
#define PIN_SILENT  9

/* CAN RX queue */
CAN_MSGQ_DEFINE(rxq, 16);

/* ===================================================== */
/* ================= LOAD / MOTOR GPIOs ================ */
/* ===================================================== */

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

static const struct gpio_dt_spec motors[NUM_MOTORS] = {
    GPIO_DT_SPEC_GET(DT_NODELABEL(motor1), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(motor2), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(motor3), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(motor4), gpios),
};

/* ===================================================== */
/* ================= INA3221 SENSORS =================== */
/* ===================================================== */

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

/* ===================================================== */
/* ================= CAN PACKET TYPE =================== */
/* ===================================================== */

typedef struct {
    uint8_t  priority;
    uint8_t  src;
    uint8_t  dst;
    uint8_t  msg_class;
    uint8_t  dlc;
    uint8_t  data[8];
} can_packet_t;

/* ===================================================== */
/* ================= LOAD / MOTOR CONTROL ============== */
/* ===================================================== */

int enable_load(int load_num)
{
    if (load_num < 1 || load_num > NUM_LOADS) return -EINVAL;
    return gpio_pin_set_dt(&loads[load_num - 1], 1);
}

int disable_load(int load_num)
{
    if (load_num < 1 || load_num > NUM_LOADS) return -EINVAL;
    return gpio_pin_set_dt(&loads[load_num - 1], 0);
}

int enable_motor(int motor_num)
{
    if (motor_num < 1 || motor_num > NUM_MOTORS) return -EINVAL;
    return gpio_pin_set_dt(&motors[motor_num - 1], 1);
}

int disable_motor(int motor_num)
{
    if (motor_num < 1 || motor_num > NUM_MOTORS) return -EINVAL;
    return gpio_pin_set_dt(&motors[motor_num - 1], 0);
}

/* ===================================================== */
/* ================= POWER MONITOR READOUT ============= */
/* ===================================================== */

static int ina3221_select_channel(const struct device *dev, int channel)
{
    struct sensor_value val = { .val1 = channel, .val2 = 0 };
    return sensor_attr_set(dev, SENSOR_CHAN_ALL,
                           (enum sensor_attribute)SENSOR_ATTR_INA3221_SELECTED_CHANNEL,
                           &val);
}

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

            if (ina3221_select_channel(ina_devs[chip], ch + 1)) {
                failures++;
                continue;
            }
            if (sensor_sample_fetch(ina_devs[chip])) {
                failures++;
                continue;
            }
            sensor_channel_get(ina_devs[chip], SENSOR_CHAN_VOLTAGE,
                               &telemetry[idx].voltage);
            sensor_channel_get(ina_devs[chip], SENSOR_CHAN_CURRENT,
                               &telemetry[idx].current);
            sensor_channel_get(ina_devs[chip], SENSOR_CHAN_POWER,
                               &telemetry[idx].power);
        }
    }
    return failures;
}

void print_power_telemetry(void)
{
    printk("──── EPS Power Telemetry ────\n");
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

/* ===================================================== */
/* ================= GPIO INIT ========================= */
/* ===================================================== */

static int init_gpios(void)
{
    int err;

    for (int i = 0; i < NUM_LOADS; i++) {
        if (!gpio_is_ready_dt(&loads[i])) {
            LOG_ERR("load %d gpio not ready", i + 1);
            return -ENODEV;
        }
        err = gpio_pin_configure_dt(&loads[i], GPIO_OUTPUT_INACTIVE);
        if (err) {
            LOG_ERR("load %d config failed (%d)", i + 1, err);
            return err;
        }
    }

    for (int i = 0; i < NUM_MOTORS; i++) {
        if (!gpio_is_ready_dt(&motors[i])) {
            LOG_ERR("motor %d gpio not ready", i + 1);
            return -ENODEV;
        }
        err = gpio_pin_configure_dt(&motors[i], GPIO_OUTPUT_INACTIVE);
        if (err) {
            LOG_ERR("motor %d config failed (%d)", i + 1, err);
            return err;
        }
    }

    return 0;
}

/* ===================================================== */
/* ================= CAN TX ============================ */
/* ===================================================== */

static void send_simple(uint8_t dst, uint8_t cls, uint8_t op, uint8_t val)
{
    struct can_frame f = {0};
    f.id = CAN_ID_FULL(PRIO_LOW, NODE_ID, dst, cls);
    f.flags = CAN_FRAME_IDE;
    can_fill_payload(&f, NODE_ID, op, val, 0, 0, 0, 0, 0);
    can_send(can_dev, &f, K_NO_WAIT, NULL, NULL);
}

/*
 * send_heartbeat() - 1 Hz broadcast heartbeat.
 */
static void send_heartbeat(void)
{
    send_simple(CAN_BROADCAST, CLS_HEARTBEAT, OP_HEARTBEAT, 0x00);
    LOG_INF("TX heartbeat");
}

/*
 * send_soh() - State-of-health to CDH.
 *
 * We have 21 power channels + 6 temp sensors — far too much for one 8-byte
 * CAN frame. So we send multiple frames, each tagged with a sub-index.
 *
 * SOH frame layout:
 *   data[0] = src (NODE_ID)
 *   data[1] = opcode (0x01 = SOH)
 *   data[2] = sub-index (which chunk of data this frame carries)
 *   data[3..7] = payload bytes for that chunk
 *
 * Sub-indices:
 *   0x00       = summary: num_loads_on, num_motors_on, num_ina_ok, num_temp_ok
 *   0x01..0x15 = INA channel N (voltage mV high/low, current mA high/low)
 *   0x20       = temp sensors 0-3 (rounded °C, signed)
 *   0x21       = temp sensors 4-5 + load switch bitmask
 */
static void send_soh(void)
{
    /* ── Read all sensors ── */
    int ina_failures = read_power_monitors();
    struct temp_telemetry temps;
    int temp_ok = temp_telemetry_read_all(i2c_bus, &temps);

    /* ── Count active loads/motors for summary ── */
    uint8_t loads_on = 0, motors_on = 0;
    for (int i = 0; i < NUM_LOADS; i++) {
        if (gpio_pin_get_dt(&loads[i])) loads_on++;
    }
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (gpio_pin_get_dt(&motors[i])) motors_on++;
    }

    /* ── Frame 0x00: Summary ── */
    {
        struct can_frame f = {0};
        f.id = CAN_ID_FULL(PRIO_LOW, NODE_ID, CDH_ID, CLS_HEALTH);
        f.flags = CAN_FRAME_IDE;
        can_fill_payload(&f, NODE_ID,
            0x01,                                       /* opcode: SOH */
            0x00,                                       /* sub-index: summary */
            loads_on,
            motors_on,
            (uint8_t)(TOTAL_CHANNELS - ina_failures),   /* INA channels OK */
            (uint8_t)temp_ok,                           /* temp sensors OK */
            0x00                                        /* reserved */
        );
        can_send(can_dev, &f, K_NO_WAIT, NULL, NULL);
    }

    /* ── Frames 0x01..0x15: Per-channel INA data ── */
    for (int i = 0; i < TOTAL_CHANNELS; i++) {
        /* Convert sensor_value to millivolts and milliamps for compactness.
         * sensor_value: val1 = integer part, val2 = fractional in micro-units.
         * mV = val1 * 1000 + val2 / 1000  */
        int32_t mv = telemetry[i].voltage.val1 * 1000
                   + telemetry[i].voltage.val2 / 1000;
        int32_t ma = telemetry[i].current.val1 * 1000
                   + telemetry[i].current.val2 / 1000;

        /* Clamp to uint16_t range */
        if (mv < 0) mv = 0;
        if (mv > 65535) mv = 65535;
        if (ma < 0) ma = 0;
        if (ma > 65535) ma = 65535;

        struct can_frame f = {0};
        f.id = CAN_ID_FULL(PRIO_LOW, NODE_ID, CDH_ID, CLS_HEALTH);
        f.flags = CAN_FRAME_IDE;
        can_fill_payload(&f, NODE_ID,
            0x01,                       /* opcode: SOH */
            (uint8_t)(i + 1),           /* sub-index: channel 1..21 */
            (uint8_t)(mv >> 8),         /* voltage mV high byte */
            (uint8_t)(mv & 0xFF),       /* voltage mV low byte */
            (uint8_t)(ma >> 8),         /* current mA high byte */
            (uint8_t)(ma & 0xFF),       /* current mA low byte */
            0x00
        );
        can_send(can_dev, &f, K_NO_WAIT, NULL, NULL);
    }

    /* ── Frame 0x20: Temp sensors 0-3 ── */
    {
        uint8_t t[4] = {0};
        for (int i = 0; i < 4 && i < NUM_TEMP_SENSORS; i++) {
            if (temps.s[i].status == 0) {
                int32_t c = (temps.s[i].temp_q4 + 8) / 16;
                if (c > 127) c = 127;
                if (c < -128) c = -128;
                t[i] = (uint8_t)(int8_t)c;
            }
        }
        struct can_frame f = {0};
        f.id = CAN_ID_FULL(PRIO_LOW, NODE_ID, CDH_ID, CLS_HEALTH);
        f.flags = CAN_FRAME_IDE;
        can_fill_payload(&f, NODE_ID,
            0x01, 0x20,
            t[0], t[1], t[2], t[3], 0x00
        );
        can_send(can_dev, &f, K_NO_WAIT, NULL, NULL);
    }

    /* ── Frame 0x21: Temp sensors 4-5 + load bitmask ── */
    {
        uint8_t t4 = 0, t5 = 0;
        if (NUM_TEMP_SENSORS > 4 && temps.s[4].status == 0) {
            int32_t c = (temps.s[4].temp_q4 + 8) / 16;
            t4 = (uint8_t)(int8_t)(c > 127 ? 127 : (c < -128 ? -128 : c));
        }
        if (NUM_TEMP_SENSORS > 5 && temps.s[5].status == 0) {
            int32_t c = (temps.s[5].temp_q4 + 8) / 16;
            t5 = (uint8_t)(int8_t)(c > 127 ? 127 : (c < -128 ? -128 : c));
        }

        /* Pack 12 load states into a 12-bit bitmask (fits in 2 bytes) */
        uint16_t load_mask = 0;
        for (int i = 0; i < NUM_LOADS; i++) {
            if (gpio_pin_get_dt(&loads[i])) {
                load_mask |= (1U << i);
            }
        }

        struct can_frame f = {0};
        f.id = CAN_ID_FULL(PRIO_LOW, NODE_ID, CDH_ID, CLS_HEALTH);
        f.flags = CAN_FRAME_IDE;
        can_fill_payload(&f, NODE_ID,
            0x01, 0x21,
            t4, t5,
            (uint8_t)(load_mask >> 8),
            (uint8_t)(load_mask & 0xFF),
            0x00
        );
        can_send(can_dev, &f, K_NO_WAIT, NULL, NULL);
    }

    /* ── Console log ── */
    LOG_INF("TX SOH: %d/%d INA ok, %d/%d temps ok, loads=%u motors=%u",
            TOTAL_CHANNELS - ina_failures, TOTAL_CHANNELS,
            temp_ok, NUM_TEMP_SENSORS,
            loads_on, motors_on);
    print_power_telemetry();
    temp_telemetry_print(&temps);
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

/*
 * handle_command() - Process commands addressed to EPS.
 *
 * OP_SET_BOARD_PWR payload:
 *   data[2] = target type:  0x01 = load switch,  0x02 = motor rail
 *   data[3] = target number (1-based: load 1-12, motor 1-4)
 *   data[4] = state:  0x00 = off,  0x01 = on
 *
 * Responds with CMD_RESP containing the result.
 */
static void handle_command(const can_packet_t *pkt)
{
    uint8_t opcode = pkt->data[1];

    switch (opcode) {
    case OP_SET_BOARD_PWR: {
        uint8_t target_type = pkt->data[2];
        uint8_t target_num  = pkt->data[3];
        uint8_t state       = pkt->data[4];
        int ret = -EINVAL;

        if (target_type == 0x01) {
            /* Load switch */
            ret = state ? enable_load(target_num) : disable_load(target_num);
            LOG_INF("RX SetBoardPwr: load %d -> %s (%s)",
                    target_num, state ? "ON" : "OFF",
                    ret == 0 ? "OK" : "FAIL");
        } else if (target_type == 0x02) {
            /* Motor rail */
            ret = state ? enable_motor(target_num) : disable_motor(target_num);
            LOG_INF("RX SetBoardPwr: motor %d -> %s (%s)",
                    target_num, state ? "ON" : "OFF",
                    ret == 0 ? "OK" : "FAIL");
        } else {
            LOG_WRN("RX SetBoardPwr: unknown target type 0x%02X", target_type);
        }

        /* ACK: send back opcode + result (0 = success) */
        send_simple(pkt->src, CLS_CMD_RESP, OP_SET_BOARD_PWR,
                    (uint8_t)(ret == 0 ? 0x00 : 0xFF));
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
    case CLS_HEARTBEAT: handle_heartbeat(pkt); break;
    case CLS_COMMAND:   handle_command(pkt);   break;
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
    LOG_INF("CAN initialized (29-bit, filters: EPS + broadcast)");
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

            /* Only process frames addressed to us or broadcast */
            if (pkt.dst != NODE_ID && pkt.dst != CAN_BROADCAST) {
                continue;
            }

            can_dispatch(&pkt);
        }
    }
}

K_THREAD_DEFINE(can_rx_tid, CAN_RX_STACK_SIZE,
    can_rx_thread, NULL, NULL, NULL,
    CAN_RX_PRIORITY, 0, 0);

/* ===================================================== */
/* ================= MAIN ============================== */
/* ===================================================== */

#define HEARTBEAT_INTERVAL_MS   1000
#define SOH_INTERVAL_MS         5000

int main(void)
{
    LOG_INF("UT-CORE EPS starting");

    /* GPIO init for loads and motors */
    int err = init_gpios();
    if (err) {
        LOG_ERR("GPIO init failed (%d)", err);
        return 0;
    }
    LOG_INF("%d loads, %d motors configured", NUM_LOADS, NUM_MOTORS);

    /* Check INA3221 sensors */
    int ina_ready = 0;
    for (int i = 0; i < NUM_INA; i++) {
        if (device_is_ready(ina_devs[i])) ina_ready++;
    }
    LOG_INF("%d/%d INA3221 ready (%d channels)", ina_ready, NUM_INA,
            ina_ready * INA_CHANNELS);

    /* Check I2C for temp sensors */
    if (!device_is_ready(i2c_bus)) {
        LOG_ERR("I2C bus not ready — temps will fail");
    }

    /* CAN bus init */
    tcan3403_wakeup();
    can_setup();

    LOG_INF("Running — waiting for CAN commands");

    /* Main loop: heartbeat + SOH */
    int64_t last_hb  = k_uptime_get();
    int64_t last_soh = k_uptime_get();

    while (1) {
        int64_t now = k_uptime_get();

        if ((now - last_hb) >= HEARTBEAT_INTERVAL_MS) {
            send_heartbeat();
            last_hb = now;
        }

        if ((now - last_soh) >= SOH_INTERVAL_MS) {
            send_soh();
            last_soh = now;
        }

        k_msleep(100);
    }

    return 0;
}