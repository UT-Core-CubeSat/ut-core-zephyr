#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(wdt_pulse, LOG_LEVEL_INF);

#define WDT_PULSE_PERIOD_MS 10

static const struct gpio_dt_spec wdtpin =
	GPIO_DT_SPEC_GET(DT_NODELABEL(wdtpin), gpios);

static void wdt_timer_handler(struct k_timer *timer)
{
	gpio_pin_set_dt(&wdtpin, 0);
	gpio_pin_set_dt(&wdtpin, 1);
}

K_TIMER_DEFINE(wdt_timer, wdt_timer_handler, NULL);

int main(void)
{
	if (!gpio_is_ready_dt(&wdtpin)) {
		LOG_ERR("WDT GPIO (PB2) not ready");
		return -1;
	}

	gpio_pin_configure_dt(&wdtpin, GPIO_OUTPUT_HIGH);

	k_timer_start(&wdt_timer, K_MSEC(WDT_PULSE_PERIOD_MS),
		       K_MSEC(WDT_PULSE_PERIOD_MS));

	LOG_INF("Watchdog pulse started: %d ms period on PB2",
		WDT_PULSE_PERIOD_MS);

	while (1) {
		k_sleep(K_FOREVER);
	}
}
