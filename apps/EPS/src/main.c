/**
 * @defgroup eps EPS
 * @ingroup apps
 * @brief EPS Electrical Power System
 *
 * @include{doc} ./apps/EPS/README.md
 */

/**
 * @file main.c
 * @ingroup eps
 * @brief EPS (Electrical Power System) firmware entry point and control logic.
 *
 * Initializes load switch, motor rail, and INA3221 power monitor GPIOs/devices,
 * then runs a loop blinking the status LED and reporting power telemetry.
 *
 * @todo move CAN code shared between apps into the `common/` directory at git root.
 */

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

/**
 * @brief Active-high GPIO specs for the 12 switched power loads, indexed
 *        0-11 corresponding to load numbers 1-12.
 */
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

/**
 * @brief GPIO specs controlling the 12V power rail feeding each of the
 *        4 motor channels.
 */
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

/**
 * @brief Single power-monitor channel reading: voltage, current, and power.
 *
 * Populated per-channel by read_power_monitors() and stored in the
 * telemetry[] array for later reporting via print_telemetry().
 */
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
    if (load_num < 1 || load_num > NUM_LOADS) {
        return -EINVAL;
    }
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
    if (load_num < 1 || load_num > NUM_LOADS) {
        return -EINVAL;
    }
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
    if (motor_num < 1 || motor_num > NUM_MOTORS) {
        return -EINVAL;
    }
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
    if (motor_num < 1 || motor_num > NUM_MOTORS) {
        return -EINVAL;
    }
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

    return sensor_attr_set(dev,
                   SENSOR_CHAN_ALL,
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

/**
 * @brief Entry point: initializes GPIOs and status LED, then runs the main
 *        loop reading and printing power telemetry.
 *
 * Boot sequence: configure load/motor GPIOs, count ready INA3221 sensors,
 * configure the status LED as active output. The main loop then toggles
 * the status LED, samples all power monitors, and prints telemetry once
 * every LED_BLINK_INTERVAL_MS.
 *
 * @return Returns 0 early if GPIO or LED initialization fails; otherwise
 *         does not return under normal operation.
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
