/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

/* Sleep interval between LED toggles: 1000 msec = 1 sec */
#define SLEEP_TIME_MS   100

/* Devicetree node identifiers for the three LED aliases (led0, led1, led2) */
#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)
#define LED2_NODE DT_ALIAS(led2)

#define pb10_NODE DT_ALIAS(pb10)
#define pd2_NODE DT_ALIAS(pd2)

/*
 * GPIO device specifications initialized from devicetree.
 * Each gpio_dt_spec structure contains GPIO port info and pin configuration.
 * A build error here means your board doesn't have these LED aliases defined.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
static const struct gpio_dt_spec led2 = GPIO_DT_SPEC_GET(LED2_NODE, gpios);

static const struct gpio_dt_spec pb10 = GPIO_DT_SPEC_GET(pb10_NODE, gpios);
static const struct gpio_dt_spec pd2  = GPIO_DT_SPEC_GET(pd2_NODE, gpios);

int main(void)
{
	int ret;
	bool led_state = true;

	/* Check if the GPIO driver is ready for LED0 */
	if (!gpio_is_ready_dt(&led0)) {
		printf("LED0 GPIO device is not ready\n");
		return 0;
	}

	/* Check if the GPIO driver is ready for LED1 */
	if (!gpio_is_ready_dt(&led1)) {
		printf("LED1 GPIO device is not ready\n");
		return 0;
	}

	/* Check if the GPIO driver is ready for LED2 */
	if (!gpio_is_ready_dt(&led2)) {
		printf("LED2 GPIO device is not ready\n");
		return 0;
	}

	/* Check if the GPIO driver is ready for pb10 */
	if (!gpio_is_ready_dt(&pb10)) {
		printf("PB10 GPIO device is not ready\n");
		return 0;
	}

	/* Check if the GPIO driver is ready for pd2 */
	if (!gpio_is_ready_dt(&pd2)) {
		printf("PD2 GPIO device is not ready\n");
		return 0;
	}

	/* Configure LED0 as a GPIO output pin, starting in active (on) state */
	ret = gpio_pin_configure_dt(&led0, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		printf("Failed to configure LED0\n");
		return 0;
	}

	/* Configure LED1 as a GPIO output pin, starting in active (on) state */
	ret = gpio_pin_configure_dt(&led1, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		printf("Failed to configure LED1\n");
		return 0;
	}

	/* Configure LED2 as a GPIO output pin, starting in active (on) state */
	ret = gpio_pin_configure_dt(&led2, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		printf("Failed to configure LED2\n");
		return 0;
	}

	/* Configure pb10 as a GPIO output pin, starting in active (on) state */
	ret = gpio_pin_configure_dt(&pb10, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		printf("Failed to configure PB10\n");
		return 0;
	}

	/* Configure pd2 as a GPIO output pin, starting in active (on) state */
	ret = gpio_pin_configure_dt(&pd2, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		printf("Failed to configure PD2\n");
		return 0;
	}

	printf("All three LEDs configured successfully. Starting sequential blink cycle...\n");

	/* Main infinite loop that blinks each LED sequentially */
	while (1) {
		/* Toggle LED0 state (on to off, or off to on) */
		ret = gpio_pin_toggle_dt(&led0);
		if (ret < 0) {
			printf("Failed to toggle LED0\n");
			return 0;
		}
		printf("LED0 toggled\n");

		/* Sleep before moving to the next LED */
		k_msleep(SLEEP_TIME_MS);

		/* Toggle LED1 state (on to off, or off to on) */
		ret = gpio_pin_toggle_dt(&led1);
		if (ret < 0) {
			printf("Failed to toggle LED1\n");
			return 0;
		}
		printf("LED1 toggled\n");

		/* Sleep before moving to the next LED */
		k_msleep(SLEEP_TIME_MS);

		/* Toggle LED2 state (on to off, or off to on) */
		ret = gpio_pin_toggle_dt(&led2);
		if (ret < 0) {
			printf("Failed to toggle LED2\n");
			return 0;
		}
		printf("LED2 toggled\n");

		/* Sleep before the sequence repeats */
		k_msleep(SLEEP_TIME_MS);

		/* Toggle pb10 state (on to off, or off to on) */
		ret = gpio_pin_toggle_dt(&pb10);
		if (ret < 0) {
			printf("Failed to toggle PB10\n");
			return 0;
		}
		printf("PB10 toggled\n");

		/* Sleep before the sequence repeats */
		k_msleep(SLEEP_TIME_MS);

		/* Toggle pd2 state (on to off, or off to on) */
		ret = gpio_pin_toggle_dt(&pd2);
		if (ret < 0) {
			printf("Failed to toggle PD2\n");
			return 0;
		}
		printf("PD2 toggled\n");

		/* Sleep before the sequence repeats */
		k_msleep(SLEEP_TIME_MS);
	}
	return 0;
}
