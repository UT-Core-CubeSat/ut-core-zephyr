#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/can.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include "C:\Users\notbr\Documents\all_coding\ut-core\common\can_proto.h"
#include "C:\Users\notbr\Documents\all_coding\ut-core\common\temp_telemetry.h"

#include <stm32_ll_tim.h>

LOG_MODULE_REGISTER(solarcon, LOG_LEVEL_INF);

/* ===================================================== */
/* ================= CAN PROTOCOL ====================== */
/* ===================================================== */

/* Node IDs — match your bus allocation */
#define NODE_ID         0x08    /* Solar Conglomerator */
#define NODE_CDH        0x01
#define NODE_ADCS       0x04
#define CAN_BROADCAST   0xFF

/* Priority levels */
#define PRIO_HIGH  0
#define PRIO_MED   2
#define PRIO_LOW   4

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

static const struct device *const gpioa   = DEVICE_DT_GET(DT_NODELABEL(gpioa));
static const struct device *const pwm1_dev = DEVICE_DT_GET(DT_NODELABEL(pwm1));
static const struct device *const pwm3_dev = DEVICE_DT_GET(DT_NODELABEL(pwm3));
static const struct device *const can_dev  = DEVICE_DT_GET(DT_NODELABEL(fdcan1));
static const struct device *i2c_bus = DEVICE_DT_GET(DT_NODELABEL(i2c1));

/* Transceiver pins — adjust to your schematic */
#define PIN_SHDN    10
#define PIN_SILENT  9

#define PWM_FREQ    20000U
#define PWM_PERIOD  PWM_HZ(PWM_FREQ)

#define TIM1_CH4    4U
#define TIM3_CH1    1U
#define TIM3_CH2    2U
#define TIM3_CH4    4U

/* CAN RX queue */
CAN_MSGQ_DEFINE(rxq, 16);

/* Shared state — current duty for telemetry reporting */
static volatile uint32_t current_duty_percent = 50;

/* ===================================================== */
/* ================= PWM HELPERS ======================= */
/* ===================================================== */

static void tim1_enable_ch4n(void)
{
    TIM1->CCER |= TIM_CCER_CC4NE | TIM_CCER_CC4E;
    TIM1->BDTR |= TIM_BDTR_MOE;
}

static void set_all_duty(uint32_t duty_percent)
{
    if (duty_percent > 100) duty_percent = 100;

    uint32_t pulse_tim = ((uint64_t)PWM_PERIOD * duty_percent) / 100U;
    uint32_t pulse_inv = PWM_PERIOD - pulse_tim;

    pwm_set(pwm1_dev, TIM1_CH4, PWM_PERIOD, pulse_inv, PWM_POLARITY_NORMAL);
    TIM1->CCER |= TIM_CCER_CC4NE | TIM_CCER_CC4E;
    TIM1->BDTR |= TIM_BDTR_MOE;

    pwm_set(pwm3_dev, TIM3_CH1, PWM_PERIOD, pulse_tim, PWM_POLARITY_NORMAL);
    pwm_set(pwm3_dev, TIM3_CH2, PWM_PERIOD, pulse_tim, PWM_POLARITY_NORMAL);
    pwm_set(pwm3_dev, TIM3_CH4, PWM_PERIOD, pulse_tim, PWM_POLARITY_NORMAL);

    current_duty_percent = duty_percent;
}

/* ===================================================== */
/* =========== MAGNETIC DIPOLE → PWM STUB ============== */
/* ===================================================== */

/*
 * mag_dipole_to_duty() - Convert a magnetic dipole moment value to PWM duty %.
 *
 * The ADCS sends a signed 16-bit dipole value (data[2]:data[3], big-endian).
 * For now this is a linear mapping:
 *   dipole  0        → duty  0%
 *   dipole ±32767    → duty 100%
 *
 * TODO: replace with your actual torquer calibration curve.
 */
static uint32_t mag_dipole_to_duty(int16_t dipole)
{
    uint32_t mag = (dipole < 0) ? (uint32_t)(-dipole) : (uint32_t)dipole;
    /* Linear scale: |dipole| / 327.67  ≈  mag * 100 / 32767 */
    uint32_t duty = (mag * 100U) / 32767U;
    if (duty > 100) duty = 100;
    return duty;
}

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

/*
 * send_heartbeat() - Periodic heartbeat to CDH.
 * data[1] = 0x01 means "alive and nominal".
 */
static void send_heartbeat(void)
{
    send_simple(CAN_BROADCAST, CLS_HEARTBEAT, 0x30, 0x00);
    LOG_DBG("TX heartbeat");
}

/*
 * send_soh() - State-of-health telemetry to CDH.
 *
 * Payload layout:
 *   [0] src
 *   [1] opcode (0x01 = SOH)
 *   [2] current duty %
 *   [3] temp sensor 1  (placeholder — raw ADC / 2 or °C)
 *   [4] temp sensor 2
 *   [5] temp sensor 3
 *   [6] temp sensor 4
 *   [7] status flags
 *
 * TODO: wire in real temp sensor reads.
 */
static void send_soh(void)
{
    struct can_frame f = {0};
    f.id    = CAN_ID_FULL(PRIO_LOW, NODE_ID, NODE_CDH, CLS_HEALTH);
    f.flags = CAN_FRAME_IDE;

    uint8_t duty = (uint8_t)current_duty_percent;

    /* Read all temp sensors */
    struct temp_telemetry telem;
    int ok = temp_telemetry_read_all(i2c_bus, &telem);

    /* Pack first 4 sensor temps as rounded integer °C into payload.
     * Clamp to int8_t range (-128..127) since each slot is one byte. */
    uint8_t temps[4] = {0};
    for (int i = 0; i < 4 && i < NUM_TEMP_SENSORS; i++) {
        if (telem.s[i].status == 0) {
            int32_t c = (telem.s[i].temp_q4 + 8) / 16;
            if (c > 127) c = 127;
            if (c < -128) c = -128;
            temps[i] = (uint8_t)(int8_t)c;
        }
    }

    can_fill_payload(&f, NODE_ID,
        0x01,           /* opcode: SOH */
        duty,
        temps[0], temps[1], temps[2], temps[3],
        0x00            /* status flags */
    );

    can_send(can_dev, &f, K_NO_WAIT, NULL, NULL);

    /* Print to console */
    LOG_INF("TX SOH  duty=%u%%  temps: %d/%d ok", duty, ok, NUM_TEMP_SENSORS);
    temp_telemetry_print(&telem);
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
    case OP_SET_MAG_DIPOLE: {
        /* dipole value in data[2]:data[3], big-endian signed */
        int16_t dipole = (int16_t)((pkt->data[2] << 8) | pkt->data[3]);
        uint32_t duty  = mag_dipole_to_duty(dipole);
        set_all_duty(duty);
        LOG_INF("RX setMagDipoleMoment: dipole=%d → duty=%u%%", dipole, duty);

        /* ACK back to sender */
        send_simple(pkt->src, CLS_CMD_RESP, OP_SET_MAG_DIPOLE, (uint8_t)duty);
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
/* ================= MAIN ============================== */
/* ===================================================== */

/* Telemetry / heartbeat intervals */
#define HEARTBEAT_INTERVAL_MS   1000
#define SOH_INTERVAL_MS         5000

int main(void)
{
    int ret;

    printk("\n=== Solarconglomerator PWM + CAN ===\n");

    /* PA5: mux enable */
    if (!device_is_ready(gpioa)) {
        printk("ERROR: GPIOA not ready!\n");
        return 0;
    }

    if (!device_is_ready(i2c_bus)) {
    LOG_ERR("I2C bus not ready — SOH temps will fail");
    }

    gpio_pin_configure(gpioa, 5, GPIO_OUTPUT_ACTIVE);
    gpio_pin_set(gpioa, 5, 0);

    /* PWM devices */
    if (!device_is_ready(pwm1_dev)) { LOG_ERR("TIM1 not ready"); return 0; }
    if (!device_is_ready(pwm3_dev)) { LOG_ERR("TIM3 not ready"); return 0; }

    /* Initial 50% on all channels */
    uint32_t half = PWM_PERIOD / 2;

    ret = pwm_set(pwm1_dev, TIM1_CH4, PWM_PERIOD, half, PWM_POLARITY_NORMAL);
    tim1_enable_ch4n();
    LOG_INF("PC5 TIM1_CH4N: %s", ret ? "FAIL" : "OK");

    ret = pwm_set(pwm3_dev, TIM3_CH1, PWM_PERIOD, half, PWM_POLARITY_NORMAL);
    LOG_INF("PC6 TIM3_CH1:  %s", ret ? "FAIL" : "OK");

    ret = pwm_set(pwm3_dev, TIM3_CH2, PWM_PERIOD, half, PWM_POLARITY_NORMAL);
    LOG_INF("PA7 TIM3_CH2:  %s", ret ? "FAIL" : "OK");

    ret = pwm_set(pwm3_dev, TIM3_CH4, PWM_PERIOD, half, PWM_POLARITY_NORMAL);
    LOG_INF("PC9 TIM3_CH4:  %s", ret ? "FAIL" : "OK");

    current_duty_percent = 50;

    /* CAN bus init */
    tcan3403_wakeup();
    can_setup();

    LOG_INF("Running — PWM at 50%%, waiting for CAN commands");

    /* Main loop: heartbeat + SOH telemetry */
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