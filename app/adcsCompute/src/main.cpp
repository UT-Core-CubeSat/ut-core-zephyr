#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>

#include "ADCSCore.hpp"
#include "adcs_platform.hpp"

// Timing measurement structure
struct LoopTimingStats {
    uint64_t cycle_count = 0;
    uint64_t min_cycle_us = UINT64_MAX;
    uint64_t max_cycle_us = 0;
    uint64_t total_time_us = 0;
    uint64_t last_print_time = 0;
};

static LoopTimingStats timing_stats;

#define LED_NODE DT_ALIAS(led0)
#if !DT_NODE_HAS_STATUS(LED_NODE, okay)
#error "led0 alias is not defined in the device tree"
#endif
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(LED_NODE, gpios);

// takes adcs by reference
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

    // Marker A - 2 Blinks
    for (int i = 0; i < 4; i++) {
        gpio_pin_toggle_dt(&led0);
        k_msleep(1000);
    }
    k_msleep(500);

    // local construction (not global)
    ADCS::Core adcs;

    // Marker B - 3 Blinks
    for (int i = 0; i < 6; i++) {
        gpio_pin_toggle_dt(&led0);
        k_msleep(1000);
    }
    k_msleep(500);

    while (1) {
        uint64_t loop_start = k_uptime_ticks();
        
        gpio_pin_toggle_dt(&led0);
        // k_msleep(5); 
        gpio_pin_toggle_dt(&led0);

        controlLoopOnce(adcs);
        
        uint64_t loop_end = k_uptime_ticks();
        uint64_t cycle_us = k_cyc_to_us_near64(loop_end - loop_start);
        
        // Update statistics
        timing_stats.cycle_count++;
        timing_stats.total_time_us += cycle_us;
        if (cycle_us < timing_stats.min_cycle_us) timing_stats.min_cycle_us = cycle_us;
        if (cycle_us > timing_stats.max_cycle_us) timing_stats.max_cycle_us = cycle_us;
        
        // Print stats every 1 second
        uint64_t now = k_uptime_get();
        if (now - timing_stats.last_print_time >= 1000) {
            uint64_t avg_cycle_us = timing_stats.total_time_us / timing_stats.cycle_count;
            float frequency = (timing_stats.cycle_count * 1000.0f) / (now - timing_stats.last_print_time);
            
            printk("[ADCS LOOP] Count: %lld | Freq: %.1f Hz | Cycle - Min: %llu us, Max: %llu us, Avg: %llu us\n",
                   timing_stats.cycle_count, frequency,
                   timing_stats.min_cycle_us, timing_stats.max_cycle_us, avg_cycle_us);
            
            // Reset for next window
            timing_stats.cycle_count = 0;
            timing_stats.total_time_us = 0;
            timing_stats.min_cycle_us = UINT64_MAX;
            timing_stats.max_cycle_us = 0;
            timing_stats.last_print_time = now;
        }
    }
}
