#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "ADCSCore.hpp"
#include "adcs_platform.hpp"

#include <zephyr/drivers/gpio.h>

#define LED_NODE DT_ALIAS(led0)

#if !DT_NODE_HAS_STATUS(LED_NODE, okay)
#error "led0 alias is not defined in the device tree"
#endif

static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(LED_NODE, gpios);

static ADCS::Core adcs;

static void controlLoopOnce() {
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

    int ret;
	bool led_state = true;

    printk("ADCS Zephyr stub booting...\n");

    /* Check if the GPIO driver is ready for LED0 */
	if (!gpio_is_ready_dt(&led0)) {
		printk("LED0 GPIO device is not ready\n");
		return 0;
	}

    /* Configure LED0 as a GPIO output pin, starting in active (on) state */
	ret = gpio_pin_configure_dt(&led0, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		printk("Failed to configure LED0\n");
		return 0;
	}

    while (1) {
        // Marker: one blink BEFORE update
        gpio_pin_set_dt(&led0, 1);
        k_msleep(50);
        gpio_pin_set_dt(&led0, 0);

        controlLoopOnce();   // call update() inside here

        // Marker: two quick blinks AFTER update
        for (int i=0; i<2; i++) {
            gpio_pin_toggle_dt(&led0);
            k_msleep(50);
        }

        k_msleep(500);
    }


    // while (true) {
    //     controlLoopOnce();

    //     ret = gpio_pin_toggle_dt(&led0);
	// 	if (ret < 0) {
	// 		printk("Failed to toggle LED0\n");
	// 		return 0;
	// 	}
	// 	printk("LED0 toggled\n");


    //     k_sleep(K_MSEC(100)); // 10 Hz
    // }

    while (true) {
        gpio_pin_set_dt(&led0, 1);
        k_msleep(250);
        gpio_pin_set_dt(&led0, 0);
        k_msleep(250);
    }

    return 0;
}
