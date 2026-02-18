#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>
#include <inttypes.h>

#include "ADCSCore.hpp"
#include "adcs_platform.hpp"

struct LoopTimingStats {
    uint64_t cycle_count;
    uint64_t min_cycle_us;
    uint64_t max_cycle_us;
    uint64_t total_time_us;
    uint64_t window_start_ms;
};

static LoopTimingStats timing_stats = {
    .cycle_count = 0,
    .min_cycle_us = UINT64_MAX,
    .max_cycle_us = 0,
    .total_time_us = 0,
    .window_start_ms = 0,
};

#define LED_NODE DT_ALIAS(led0)
#if !DT_NODE_HAS_STATUS(LED_NODE, okay)
#error "led0 alias is not defined in the device tree"
#endif
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(LED_NODE, gpios);

static void controlLoopOnce(ADCS::Core &adcs) {
    ADCS::SensorData sensors{};
    sensors.unix_time     = ADCSPlatform::getUnixTime();
    sensors.gyro          = ADCSPlatform::getGyroData();
    sensors.star_quat     = ADCSPlatform::getStarTrackerQuaternion();
    sensors.css_currents  = ADCSPlatform::getCoarseSunSensorCurrents();
    sensors.magnetometer  = ADCSPlatform::getMagnetometerData();
    sensors.gps_ecef      = ADCSPlatform::getGPSECEFData();
    sensors.wheel_speeds  = ADCSPlatform::getReactionWheelSpeeds();

    ADCS::Command cmd{};
    cmd.mode = ADCS::MissionMode::STANDBY;

    ADCS::AdcsOutput out = adcs.update(sensors, cmd);

    ADCSPlatform::sendWheelTorques(out.wheel_torque);
    ADCSPlatform::sendMTQDipoles(out.mtq_dipole);
}

extern "C" int main(void) {
    printk("ADCS Zephyr stub booting...\n");

    if (!gpio_is_ready_dt(&led0)) {
        printk("LED0 GPIO device is not ready\n");
        return 0;
    }
    if (gpio_pin_configure_dt(&led0, GPIO_OUTPUT_INACTIVE) < 0) {
        printk("Failed to configure LED0\n");
        return 0;
    }

    ADCS::Core adcs;

    timing_stats.window_start_ms = k_uptime_get();

    while (1) {
        uint64_t start_cyc = k_cycle_get_64();

        // optional marker pulse
        gpio_pin_toggle_dt(&led0);
        gpio_pin_toggle_dt(&led0);

        controlLoopOnce(adcs);

        uint64_t end_cyc = k_cycle_get_64();
        uint64_t cycle_us = k_cyc_to_us_near64(end_cyc - start_cyc);

        timing_stats.cycle_count++;
        timing_stats.total_time_us += cycle_us;
        if (cycle_us < timing_stats.min_cycle_us) timing_stats.min_cycle_us = cycle_us;
        if (cycle_us > timing_stats.max_cycle_us) timing_stats.max_cycle_us = cycle_us;

        uint64_t now_ms = k_uptime_get();
        uint64_t elapsed_ms = now_ms - timing_stats.window_start_ms;

        if (elapsed_ms >= 1000) {
            uint64_t avg_cycle_us = timing_stats.total_time_us / timing_stats.cycle_count;

            // frequency with 0.1 Hz resolution WITHOUT floats:
            // freq_x10 = count / (elapsed_s) * 10 = count * 10000 / elapsed_ms
            uint64_t freq_x10 = (timing_stats.cycle_count * 10000ULL) / elapsed_ms;

            printk("[ADCS LOOP] Count: %" PRIu64
                   " | Freq: %" PRIu64 ".%" PRIu64 " Hz"
                   " | Cycle - Min: %" PRIu64 " us, Max: %" PRIu64 " us, Avg: %" PRIu64 " us\n",
                   timing_stats.cycle_count,
                   (freq_x10 / 10), (freq_x10 % 10),
                   timing_stats.min_cycle_us, timing_stats.max_cycle_us, avg_cycle_us);

            // reset window
            timing_stats.cycle_count = 0;
            timing_stats.total_time_us = 0;
            timing_stats.min_cycle_us = UINT64_MAX;
            timing_stats.max_cycle_us = 0;
            timing_stats.window_start_ms = now_ms;
        }

        // if you want to cap loop rate, add a sleep here
        // k_msleep(1);
    }
}
