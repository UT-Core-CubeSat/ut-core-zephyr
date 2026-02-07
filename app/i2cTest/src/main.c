/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>
#include <stdint.h>
#include <stdbool.h>

/* ===================== Devicetree ===================== */

#define I2C_BUS_NODE DT_NODELABEL(i2c1)
static const struct device *const i2c_bus = DEVICE_DT_GET(I2C_BUS_NODE);

/* Most TMP/TI-style sensors use register pointer 0x00 for temperature */
#define TEMP_REG 0x00

/* 1001000..1001101 = 0x48..0x4D */
static const uint8_t temp_addrs[] = { 0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D };
#define NUM_SENSORS ((uint8_t)(sizeof(temp_addrs) / sizeof(temp_addrs[0])))

/* ===================== Telemetry Types ===================== */

struct temp_sample {
	uint8_t addr;     /* I2C 7-bit address */
	int16_t temp_q4;  /* temperature in Q4 (degC * 16) */
	int status;       /* 0 = OK, else i2c error */
};

struct temp_telemetry {
	uint32_t t_ms;                 /* when sampled (uptime ms) */
	struct temp_sample s[NUM_SENSORS];
};

/* ===================== Small Helpers ===================== */

static void print_temp_q4(int16_t t_q4)
{
	/* Q4 to integer: divide by 16 and round */
	int32_t temp_c = (t_q4 + 8) / 16;  /* +8 for rounding */
	printk("%ld", (long)temp_c);
}

/* ===================== The One Function You Wanted ===================== */
/*
 * Reads all sensors into 'out'.
 * Returns number of sensors that responded successfully (status == 0).
 */
static int temp_telemetry_read_all(const struct device *bus, struct temp_telemetry *out)
{
	if (out == NULL || bus == NULL) {
		return 0;
	}

	out->t_ms = (uint32_t)k_uptime_get_32();

	int ok = 0;

	for (uint8_t i = 0; i < NUM_SENSORS; i++) {
		uint8_t reg = TEMP_REG;
		uint8_t buf[2] = {0};

		out->s[i].addr = temp_addrs[i];

		int ret = i2c_write_read(bus, temp_addrs[i], &reg, 1, buf, 2);
		out->s[i].status = ret;

		if (ret == 0) {
			/*
			 * Common format: 12-bit signed temperature left-justified in 16-bit reg.
			 * raw >> 4 gives Q4 (degC * 16).
			 * If your sensor is different, this is the ONLY line you change.
			 */
			int16_t raw = (int16_t)((buf[0] << 8) | buf[1]);
			out->s[i].temp_q4 = (int16_t)(raw >> 4);
			ok++;
		} else {
			out->s[i].temp_q4 = 0;
		}
	}

	return ok;
}

static void temp_telemetry_print(const struct temp_telemetry *t)
{
	printk("t=%lu ms | ", (unsigned long)t->t_ms);

	for (uint8_t i = 0; i < NUM_SENSORS; i++) {
		printk("0x%02X:", t->s[i].addr);

		if (t->s[i].status == 0) {
			print_temp_q4(t->s[i].temp_q4);
			printk("C");
		} else {
			printk("ERR(%d)", t->s[i].status);
		}

		if (i + 1 < NUM_SENSORS) {
			printk("  ");
		}
	}

	printk("\n");
}

/* ===================== Main ===================== */

int main(void)
{
	if (!device_is_ready(i2c_bus)) {
		printk("I2C bus not ready\n");
		return 0;
	}

	printk("Temp poller: reading %d sensors on i2c1 (0x48..0x4D)\n", NUM_SENSORS);

	struct temp_telemetry telem;

	while (1) {
		(void)temp_telemetry_read_all(i2c_bus, &telem);
		temp_telemetry_print(&telem);
		k_msleep(500);
	}

	return 0;
}
