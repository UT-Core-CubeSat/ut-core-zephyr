#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>

/* ===== Devicetree ===== */
#define SPI_BUS_NODE DT_NODELABEL(spi2)

/* Pick a spare GPIO for scope trigger: define this alias in your DTS overlay:
 * / {
 *   aliases { scope0 = &some_gpio_node; };
 * };
 *
 * Example (if you have a free LED alias already, you can just use led0):
 * #define SCOPE_NODE DT_ALIAS(led0)
 */
#define SCOPE_NODE DT_ALIAS(led0)

/* ===== Devices ===== */
static const struct device *spi_bus = DEVICE_DT_GET(SPI_BUS_NODE);
static const struct gpio_dt_spec cs_gpio =
	GPIO_DT_SPEC_GET_BY_IDX(SPI_BUS_NODE, cs_gpios, 0);

/* Scope trigger pin (optional but recommended) */
static const struct gpio_dt_spec scope_pin =
	GPIO_DT_SPEC_GET(SCOPE_NODE, gpios);

/* ===== SPI config ===== */
static struct spi_config spi_cfg = {
	.frequency = 20000000, /* 1 MHz bring-up */
	.operation = SPI_OP_MODE_MASTER |
		     SPI_TRANSFER_MSB |
		     SPI_WORD_SET(8), /* MODE 0 */
	.slave = 0,
	.cs = {
		.gpio = cs_gpio,
		.delay = 2,      /* us */
		.cs_is_gpio = true,
	},
};

int main(void)
{
	printk("FRAM RDID scope loop (mode0)\n");
	k_usleep(1000);

	if (!device_is_ready(spi_bus) || !gpio_is_ready_dt(&cs_gpio)) {
		printk("SPI bus or CS GPIO not ready\n");
		return 0;
	}

	if (!gpio_is_ready_dt(&scope_pin)) {
		printk("Scope GPIO not ready (check SCOPE_NODE)\n");
		return 0;
	}

	/* Configure scope trigger pin */
	gpio_pin_configure_dt(&scope_pin, GPIO_OUTPUT_INACTIVE);

	/* Buffers live outside the loop so they don't move around */
	uint8_t tx[10] = { 0x9F }; /* RDID + 9 dummy clocks */
	uint8_t rx[10];

	struct spi_buf tx_buf = { .buf = tx, .len = sizeof(tx) };
	struct spi_buf rx_buf = { .buf = rx, .len = sizeof(rx) };
	struct spi_buf_set tx_set = { .buffers = &tx_buf, .count = 1 };
	struct spi_buf_set rx_set = { .buffers = &rx_buf, .count = 1 };

	uint32_t count = 0;

	while (1) {
		/* Toggle a trigger edge for your scope */
		gpio_pin_set_dt(&scope_pin, 1);

		/* Do transfer */
		memset(rx, 0, sizeof(rx));
		int err = spi_transceive(spi_bus, &spi_cfg, &tx_set, &rx_set);

		gpio_pin_set_dt(&scope_pin, 0);

		/* Print occasionally so UART doesn't ruin timing */
		if (err) {
			printk("spi_transceive err=%d\n", err);
		} else if ((count % 100U) == 0U) {
			printk("#%u RX: ", count);
			for (int i = 0; i < 10; i++) {
				printk("%02X ", rx[i]);
			}
			printk("\n");
		}

		count++;

		/* Slow down the repetition so it's easy to trigger.
		 * Change to k_usleep(50) if you want it screaming fast.
		 */
		k_msleep(10);
	}

	return 0;
}