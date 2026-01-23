#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/can.h>

/* LED aliases */
#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)

/* CAN device */
#define CAN_NODE DT_NODELABEL(fdcan1)

/* Startup delay */
#define STARTUP_DELAY_MS 2000

static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);

static const struct device *can_dev = DEVICE_DT_GET(CAN_NODE);

int main(void)
{
	int ret;

	printk("UT-CORE Booting...\n");

	/* ========================= */
	/* LED STARTUP TEST SEQUENCE */
	/* ========================= */

	if (!gpio_is_ready_dt(&led0) || !gpio_is_ready_dt(&led1)) {
		printk("LED GPIO not ready\n");
		return 0;
	}

	gpio_pin_configure_dt(&led0, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&led1, GPIO_OUTPUT_INACTIVE);

	/* Turn ALL LEDs ON */
	gpio_pin_set_dt(&led0, 1);
	gpio_pin_set_dt(&led1, 1);

	printk("Startup LED test ON\n");

	k_msleep(STARTUP_DELAY_MS);

	/* Turn ALL LEDs OFF */
	gpio_pin_set_dt(&led0, 0);
	gpio_pin_set_dt(&led1, 0);

	printk("Startup LED test OFF\n");

	/* ========================= */
	/* CAN INITIALIZATION        */
	/* ========================= */

	printk("UT-CORE CAN TX Test\n");

	if (!device_is_ready(can_dev)) {
		printk("CAN device not ready\n");
		return 0;
	}

	/* Set CAN bitrate */
	ret = can_set_bitrate(can_dev, 500000); /* 500 kbps */
	if (ret) {
		printk("Failed to set CAN bitrate (%d)\n", ret);
		return 0;
	}

	/* Start CAN controller */
	ret = can_start(can_dev);
	if (ret) {
		printk("Failed to start CAN (%d)\n", ret);
		return 0;
	}

	printk("CAN started successfully\n");

	/* ========================= */
	/* CAN FRAME SETUP           */
	/* ========================= */

	struct can_frame frame = {
		.id = 0x123,
		.flags = CAN_FRAME_IDE, /* Standard ID */
		.dlc = 8,
		.data = { 0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE, 0x12, 0x34 }
	};

	/* ========================= */
	/* MAIN LOOP                 */
	/* ========================= */

	while (1) {

		ret = can_send(can_dev, &frame, K_MSEC(100), NULL, NULL);

		if (ret == 0) {
			printk("CAN frame sent\n");

			/* TX OK: LED0 ON */
			gpio_pin_set_dt(&led0, 1);
			gpio_pin_set_dt(&led1, 0);
		} else {
			printk("CAN send failed (%d)\n", ret);

			/* TX FAIL: LED1 ON */
			gpio_pin_set_dt(&led0, 0);
			gpio_pin_set_dt(&led1, 1);
		}

		k_msleep(1000);
	}
}
