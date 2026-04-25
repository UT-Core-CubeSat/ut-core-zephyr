#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

static const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));

#define MLX90393_ADDR_A       0x0C
#define MLX90393_ADDR_B       0x0D
#define MLX90393_ADDR_C       0x0F

#define MLX90393_CMD_EX       0x80
#define MLX90393_CMD_RT       0xF0
#define MLX90393_CMD_SM_XYZT  0x3F
#define MLX90393_CMD_RM_XYZT  0x4F

#define SAMPLE_PERIOD_MS      50
#define STATS_EVERY_SAMPLES   100

static const uint8_t mlx_addrs[] = {
	MLX90393_ADDR_A, MLX90393_ADDR_B, MLX90393_ADDR_C
};

/* Latest calibration from MagCalTool 3-minute run. */
static const float MAG_BIAS[3][3] = {
	{-396.500f, 2064.500f, 1274.500f},
	{3192.000f, -2262.000f, 133.500f},
	{-375.500f, 2299.000f, -2352.000f},
};

static const float MAG_SCALE[3][3] = {
	{0.894505f, 0.966746f, 1.179710f},
	{0.923704f, 0.989683f, 1.102564f},
	{0.887704f, 0.950533f, 1.217349f},
};

struct mag_stats {
	bool initialized;
	int16_t min_xyz[3];
	int16_t max_xyz[3];
};

static int mlx90393_cmd(uint8_t addr, uint8_t cmd, uint8_t *status_out)
{
	uint8_t st = 0;
	int ret = i2c_write_read(i2c_dev, addr, &cmd, 1, &st, 1);
	if (ret) {
		return ret;
	}
	if (status_out) {
		*status_out = st;
	}
	return 0;
}

static int mlx90393_init(uint8_t addr)
{
	uint8_t st = 0;
	int ret = mlx90393_cmd(addr, MLX90393_CMD_EX, &st);
	if (ret) {
		printk("MLX90393 0x%02X no response (%d)\n", addr, ret);
		return ret;
	}

	uint8_t rt = MLX90393_CMD_RT;
	ret = i2c_write(i2c_dev, &rt, 1, addr);
	if (ret) {
		printk("MLX90393 0x%02X reset write failed (%d)\n", addr, ret);
		return ret;
	}
	k_msleep(15);

	for (int attempt = 0; attempt < 5; attempt++) {
		ret = mlx90393_cmd(addr, MLX90393_CMD_EX, &st);
		if (ret == 0) {
			printk("MLX90393 0x%02X ready, status=0x%02X\n", addr, st);
			return 0;
		}
		k_msleep(5);
	}

	printk("MLX90393 0x%02X failed after reset (%d)\n", addr, ret);
	return ret;
}

static int mlx90393_start(uint8_t addr)
{
	uint8_t st = 0;
	return mlx90393_cmd(addr, MLX90393_CMD_SM_XYZT, &st);
}

static int mlx90393_collect(uint8_t addr, int16_t mag[3], uint16_t *temp)
{
	uint8_t cmd = MLX90393_CMD_RM_XYZT;
	uint8_t buf[9];

	int ret = i2c_write_read(i2c_dev, addr, &cmd, 1, buf, sizeof(buf));
	if (ret) {
		return ret;
	}

	*temp = (uint16_t)(buf[1] << 8 | buf[2]);
	mag[0] = (int16_t)(buf[3] << 8 | buf[4]);
	mag[1] = (int16_t)(buf[5] << 8 | buf[6]);
	mag[2] = (int16_t)(buf[7] << 8 | buf[8]);
	return 0;
}

static void stats_update(struct mag_stats *s, const int16_t mag[3])
{
	if (!s->initialized) {
		for (int i = 0; i < 3; i++) {
			s->min_xyz[i] = mag[i];
			s->max_xyz[i] = mag[i];
		}
		s->initialized = true;
		return;
	}

	for (int i = 0; i < 3; i++) {
		if (mag[i] < s->min_xyz[i]) {
			s->min_xyz[i] = mag[i];
		}
		if (mag[i] > s->max_xyz[i]) {
			s->max_xyz[i] = mag[i];
		}
	}
}

static void stats_print_cal_line(int sensor_idx, uint8_t addr, const struct mag_stats *s)
{
	if (!s->initialized) {
		printk("CAL,%d,0x%02X,not_initialized\n", sensor_idx, addr);
		return;
	}

	float ofs[3];
	float rad[3];
	for (int i = 0; i < 3; i++) {
		ofs[i] = 0.5f * ((float)s->max_xyz[i] + (float)s->min_xyz[i]);
		rad[i] = 0.5f * ((float)s->max_xyz[i] - (float)s->min_xyz[i]);
	}

	float avg_rad = (rad[0] + rad[1] + rad[2]) / 3.0f;
	float scl[3];
	for (int i = 0; i < 3; i++) {
		scl[i] = (rad[i] > 1e-6f) ? (avg_rad / rad[i]) : 1.0f;
	}

	printk("CAL,%d,0x%02X,ofs,%.2f,%.2f,%.2f,scl,%.4f,%.4f,%.4f,min,%d,%d,%d,max,%d,%d,%d\n",
	       sensor_idx, addr,
	       (double)ofs[0], (double)ofs[1], (double)ofs[2],
	       (double)scl[0], (double)scl[1], (double)scl[2],
	       s->min_xyz[0], s->min_xyz[1], s->min_xyz[2],
	       s->max_xyz[0], s->max_xyz[1], s->max_xyz[2]);
}

static void mag_apply_calibration(size_t sensor_idx, const int16_t raw[3], float cal[3])
{
	for (int ax = 0; ax < 3; ax++) {
		float centered = (float)raw[ax] - MAG_BIAS[sensor_idx][ax];
		cal[ax] = centered * MAG_SCALE[sensor_idx][ax];
	}
}

int main(void)
{
	if (!gpio_is_ready_dt(&led0)) {
		printk("LED GPIO not ready\n");
		return 0;
	}
	gpio_pin_configure_dt(&led0, GPIO_OUTPUT_ACTIVE);

	const struct device *gpioc = DEVICE_DT_GET(DT_NODELABEL(gpioc));
	if (!device_is_ready(gpioc)) {
		printk("GPIOC not ready\n");
		return 0;
	}
	gpio_pin_configure(gpioc, 9, GPIO_OUTPUT_ACTIVE);
	gpio_pin_set(gpioc, 9, 1);

	if (!device_is_ready(i2c_dev)) {
		printk("I2C1 not ready\n");
		return 0;
	}

	printk("\n=== magCalTest (MLX90393 x3) ===\n");
	printk("CSV data lines:\n");
	printk("MAGCSV,t_ms,sensor,addr,mx,my,mz,temp,status\n");
	printk("MAGCALCSV,t_ms,sensor,addr,mx_cal,my_cal,mz_cal,status\n");
	printk("Calibration estimate lines every %d samples:\n", STATS_EVERY_SAMPLES);
	printk("CAL,sensor,addr,ofs,x,y,z,scl,x,y,z,min,x,y,z,max,x,y,z\n\n");

	for (size_t i = 0; i < ARRAY_SIZE(mlx_addrs); i++) {
		(void)mlx90393_init(mlx_addrs[i]);
	}

	struct mag_stats stats[ARRAY_SIZE(mlx_addrs)] = {0};
	uint32_t sample_count = 0;

	while (1) {
		int16_t mag[ARRAY_SIZE(mlx_addrs)][3] = {0};
		uint16_t temp[ARRAY_SIZE(mlx_addrs)] = {0};
		bool ok[ARRAY_SIZE(mlx_addrs)] = {0};

		for (size_t i = 0; i < ARRAY_SIZE(mlx_addrs); i++) {
			if (mlx90393_start(mlx_addrs[i]) == 0) {
				ok[i] = true;
			}
		}

		k_msleep(10);

		uint32_t t_ms = k_uptime_get_32();
		for (size_t i = 0; i < ARRAY_SIZE(mlx_addrs); i++) {
			if (ok[i]) {
				if (mlx90393_collect(mlx_addrs[i], mag[i], &temp[i]) == 0) {
					float mag_cal[3];
					mag_apply_calibration(i, mag[i], mag_cal);

					stats_update(&stats[i], mag[i]);
					printk("MAGCSV,%lu,%u,0x%02X,%d,%d,%d,%u,0\n",
					       (unsigned long)t_ms, (unsigned int)i, mlx_addrs[i],
					       mag[i][0], mag[i][1], mag[i][2], temp[i]);
					printk("MAGCALCSV,%lu,%u,0x%02X,%.3f,%.3f,%.3f,0\n",
					       (unsigned long)t_ms, (unsigned int)i, mlx_addrs[i],
					       (double)mag_cal[0], (double)mag_cal[1], (double)mag_cal[2]);
					continue;
				}
			}

			printk("MAGCSV,%lu,%u,0x%02X,0,0,0,0,1\n",
			       (unsigned long)t_ms, (unsigned int)i, mlx_addrs[i]);
			printk("MAGCALCSV,%lu,%u,0x%02X,0,0,0,1\n",
			       (unsigned long)t_ms, (unsigned int)i, mlx_addrs[i]);
		}

		sample_count++;
		if ((sample_count % STATS_EVERY_SAMPLES) == 0U) {
			printk("---- CAL ESTIMATE @ sample %lu ----\n", (unsigned long)sample_count);
			for (size_t i = 0; i < ARRAY_SIZE(mlx_addrs); i++) {
				stats_print_cal_line((int)i, mlx_addrs[i], &stats[i]);
			}
		}

		gpio_pin_toggle_dt(&led0);
		k_msleep(SAMPLE_PERIOD_MS);
	}

	return 0;
}
