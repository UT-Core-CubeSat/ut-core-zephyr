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

    /* 1. Set CAN Bitrate */
    ret = can_set_bitrate(can_dev, 500000);
    if (ret) {
        printk("Failed to set CAN bitrate (%d)\n", ret);
        return 0;
    }

    /* 2. ENABLE LOOPBACK MODE (Critical for single-board testing) */
    ret = can_set_mode(can_dev, CAN_MODE_LOOPBACK);
    if (ret) {
        printk("Failed to set loopback mode (%d)\n", ret);
        return 0;
    }

    /* 3. Start CAN controller */
    ret = can_start(can_dev);
    if (ret) {
        printk("Failed to start CAN (%d)\n", ret);
        return 0;
    }
    
    printk("CAN started successfully in LOOPBACK mode\n");

    /* 4. Fix the Frame Flags */
    struct can_frame frame = {
        .id = 0x123,
        /* ERROR FIX: CAN_FRAME_IDE is for Extended (29-bit) IDs. 
           For Standard (11-bit) IDs like 0x123, use 0. */
        .flags = 0, 
        .dlc = 8,
        .data = { 0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE, 0x12, 0x34 }
    };

	/* ========================= */
	/* MAIN LOOP                 */
	/* ========================= */

/* ========================= */
    /* MAIN LOOP                 */
    /* ========================= */

    while (1) {
        printk("Sending CAN frame...\n");

        /* Try to send (Non-blocking) */
        ret = can_send(can_dev, &frame, K_NO_WAIT, NULL, NULL);

        if (ret == 0) {
            printk("CAN frame sent (Success)\n");

            /* SUCCESS INDICATION: Blink LED0 */
            /* 1. Turn LED0 ON (Green?) */
            gpio_pin_set_dt(&led0, 1);
            /* 2. Turn LED1 OFF (Red?) */
            gpio_pin_set_dt(&led1, 0);

            /* Keep it ON briefly so you can see the flash */
            k_msleep(100); 

            /* 3. Turn LED0 OFF */
            gpio_pin_set_dt(&led0, 0);

            /* Wait the rest of the second before next send */
            k_msleep(900);

        } else {
            printk("CAN send failed (%d)\n", ret);

            /* FAILURE INDICATION: Solid LED1 */
            /* 1. Turn LED0 OFF */
            gpio_pin_set_dt(&led0, 0);
            /* 2. Turn LED1 ON */
            gpio_pin_set_dt(&led1, 1);

            /* Wait 1 second before retrying */
            k_msleep(1000);
        }
    }
}
