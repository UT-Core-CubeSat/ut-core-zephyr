#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/can.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include "C:\Users\notbr\Documents\all_coding\ut-core\common\can_proto.h"

LOG_MODULE_REGISTER(eps, LOG_LEVEL_INF);

/* INA3221 private attribute for channel selection (1, 2, or 3) */
#define SENSOR_ATTR_INA3221_SELECTED_CHANNEL (SENSOR_ATTR_PRIV_START + 1)

#define NUM_LOADS       12
#define NUM_MOTORS      4
#define NUM_INA         7
#define INA_CHANNELS    3
#define TOTAL_CHANNELS  (NUM_INA * INA_CHANNELS)

#define TELEMETRY_INTERVAL_MS 1000
#define LED_BLINK_INTERVAL_MS 500
#define HEARTBEAT_INTERVAL_MS 1000

/* ── CAN ─────────────────────────────────────────────────────────── */

#define NODE_ID     EPS_ID      /* 0x02 from can_proto.h */
#define PRIO_LOW    4

static const struct device *const can_dev = DEVICE_DT_GET(DT_NODELABEL(fdcan1));
static const struct device *const gpioa   = DEVICE_DT_GET(DT_NODELABEL(gpioa));
CAN_MSGQ_DEFINE(rxq, 16);
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

static const struct gpio_dt_spec motors[NUM_MOTORS] = {
	GPIO_DT_SPEC_GET(DT_NODELABEL(motor1), gpios),
	GPIO_DT_SPEC_GET(DT_NODELABEL(motor2), gpios),
	GPIO_DT_SPEC_GET(DT_NODELABEL(motor3), gpios),
	GPIO_DT_SPEC_GET(DT_NODELABEL(motor4), gpios),
};

/* ── INA3221 power monitors (7 chips, 3 channels each = 21 ch) ── */

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

/* ── Motor 12V rail control ──────────────────────────────────────── */

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

/* ── Power monitor readout ───────────────────────────────────────── */

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
			if (ina3221_select_channel(ina_devs[chip], ch + 1)) { failures++; continue; }
			if (sensor_sample_fetch(ina_devs[chip]))             { failures++; continue; }
			sensor_channel_get(ina_devs[chip], SENSOR_CHAN_VOLTAGE, &telemetry[idx].voltage);
			sensor_channel_get(ina_devs[chip], SENSOR_CHAN_CURRENT, &telemetry[idx].current);
			sensor_channel_get(ina_devs[chip], SENSOR_CHAN_POWER,   &telemetry[idx].power);
		}
	}
	return failures;
}

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

typedef struct {
	uint8_t priority;
	uint8_t src;
	uint8_t dst;
	uint8_t msg_class;
	uint8_t dlc;
	uint8_t data[8];
} can_packet_t;

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

static void handle_heartbeat(const can_packet_t *pkt)
{
	LOG_INF("RX heartbeat from 0x%02X", pkt->src);
}

static void can_dispatch(const can_packet_t *pkt)
{
	switch (pkt->msg_class) {
	case CLS_HEARTBEAT:
		handle_heartbeat(pkt);
		break;
	/* TODO: CLS_COMMAND handler for load/motor control */
	default:
		LOG_WRN("Unhandled class: %d from 0x%02X", pkt->msg_class, pkt->src);
		break;
	}
}

/* ── CAN RX Thread ───────────────────────────────────────────────── */

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

/* ── CAN Setup ───────────────────────────────────────────────────── */

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