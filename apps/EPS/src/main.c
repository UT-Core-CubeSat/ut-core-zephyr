#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>

/* INA3221 private attribute for channel selection (1, 2, or 3) */
#define SENSOR_ATTR_INA3221_SELECTED_CHANNEL (SENSOR_ATTR_PRIV_START + 1)

#define NUM_LOADS       12
#define NUM_MOTORS      4
#define NUM_INA         7
#define INA_CHANNELS    3
#define TOTAL_CHANNELS  (NUM_INA * INA_CHANNELS)

#define TELEMETRY_INTERVAL_MS 1000
#define LED_BLINK_INTERVAL_MS 500

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
	/* I2C3 bus (PC0/PC1) */
	DEVICE_DT_GET(DT_NODELABEL(ina_bus1_0)),
	DEVICE_DT_GET(DT_NODELABEL(ina_bus1_1)),
	DEVICE_DT_GET(DT_NODELABEL(ina_bus1_2)),
	/* I2C1 bus (PB6/PB7) */
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
	if (load_num < 1 || load_num > NUM_LOADS) {
		return -EINVAL;
	}
	return gpio_pin_set_dt(&loads[load_num - 1], 1);
}

int disable_load(int load_num)
{
	if (load_num < 1 || load_num > NUM_LOADS) {
		return -EINVAL;
	}
	return gpio_pin_set_dt(&loads[load_num - 1], 0);
}

/* ── Motor 12V rail control ──────────────────────────────────────── */

int enable_motor(int motor_num)
{
	if (motor_num < 1 || motor_num > NUM_MOTORS) {
		return -EINVAL;
	}
	return gpio_pin_set_dt(&motors[motor_num - 1], 1);
}

int disable_motor(int motor_num)
{
	if (motor_num < 1 || motor_num > NUM_MOTORS) {
		return -EINVAL;
	}
	return gpio_pin_set_dt(&motors[motor_num - 1], 0);
}

/* ── Power monitor readout ───────────────────────────────────────── */

static int ina3221_select_channel(const struct device *dev, int channel)
{
	struct sensor_value val = { .val1 = channel, .val2 = 0 };

	return sensor_attr_set(dev,
			       SENSOR_CHAN_ALL,
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
			       telemetry[idx].voltage.val1,
			       telemetry[idx].voltage.val2,
			       telemetry[idx].current.val1,
			       telemetry[idx].current.val2,
			       telemetry[idx].power.val1,
			       telemetry[idx].power.val2);
		}
	}
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
		if (err) {
			printk("EPS: load %d config failed (%d)\n", i + 1, err);
			return err;
		}
	}

	for (int i = 0; i < NUM_MOTORS; i++) {
		if (!gpio_is_ready_dt(&motors[i])) {
			printk("EPS: motor %d gpio not ready\n", i + 1);
			return -ENODEV;
		}
		err = gpio_pin_configure_dt(&motors[i], GPIO_OUTPUT_INACTIVE);
		if (err) {
			printk("EPS: motor %d config failed (%d)\n", i + 1, err);
			return err;
		}
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
		if (device_is_ready(ina_devs[i])) {
			ina_ready++;
		}
	}
	printk("EPS: %d/%d INA3221 sensors ready (%d total channels)\n",
	       ina_ready, NUM_INA, ina_ready * INA_CHANNELS);

	if (!gpio_is_ready_dt(&led0)) {
		printk("EPS: led0 gpio not ready\n");
		return 0;
	}
	err = gpio_pin_configure_dt(&led0, GPIO_OUTPUT_ACTIVE);
	if (err) {
		printk("EPS: led0 config failed (%d)\n", err);
		return 0;
	}

	while (1) {
		gpio_pin_toggle_dt(&led0);
		read_power_monitors();
		print_telemetry();
		k_msleep(LED_BLINK_INTERVAL_MS);
	}
}
