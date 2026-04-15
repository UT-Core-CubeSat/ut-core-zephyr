#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>

#include "ADCSCore.hpp"

#define VERBOSE_EVERY    100  /* full sensor dump every Nth cycle */

/* ===================================================================
 *  LEDs
 * =================================================================== */
#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)
#define LED2_NODE DT_ALIAS(led2)

static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
static const struct gpio_dt_spec led2 = GPIO_DT_SPEC_GET(LED2_NODE, gpios);

/* ===================================================================
 *  I2C bus  (i2c1 remapped to PB8 SCL / PB9 SDA via overlay)
 * =================================================================== */
static const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));

/* ===================================================================
 *  LSM6DSV  —  6-axis IMU  (accel + gyro)
 *  2 sensors: SA0=0 -> 0x6A,  SA0=1 -> 0x6B
 * =================================================================== */
#define LSM6DSV_ADDR_A        0x6A
#define LSM6DSV_ADDR_B        0x6B

#define LSM6DSV_WHO_AM_I      0x0F
#define LSM6DSV_WHO_AM_I_VAL  0x70
#define LSM6DSV_CTRL1_XL      0x10
#define LSM6DSV_CTRL2_G       0x11
#define LSM6DSV_CTRL3         0x12
#define LSM6DSV_OUTX_L_G      0x22
#define LSM6DSV_OUTX_L_A      0x28

static const uint8_t lsm6dsv_addrs[] = { LSM6DSV_ADDR_A, LSM6DSV_ADDR_B };

static int lsm6dsv_init(uint8_t addr)
{
	uint8_t who;
	int ret = i2c_reg_read_byte(i2c_dev, addr, LSM6DSV_WHO_AM_I, &who);
	if (ret) {
		printk("  LSM6DSV@0x%02X: no response (%d)\n", addr, ret);
		return ret;
	}
	if (who != LSM6DSV_WHO_AM_I_VAL) {
		printk("  LSM6DSV@0x%02X: unexpected WHO_AM_I 0x%02X\n", addr, who);
		return -EIO;
	}

	/* BDU=1, IF_INC=1 */
	i2c_reg_write_byte(i2c_dev, addr, LSM6DSV_CTRL3, 0x44);
	/* Accel: 60 Hz, +/-4 g */
	i2c_reg_write_byte(i2c_dev, addr, LSM6DSV_CTRL1_XL, 0x54);
	/* Gyro:  60 Hz, +/-250 dps */
	i2c_reg_write_byte(i2c_dev, addr, LSM6DSV_CTRL2_G, 0x51);

	printk("  LSM6DSV@0x%02X: OK  (WHO=0x%02X)\n", addr, who);
	return 0;
}

static int lsm6dsv_read(uint8_t addr, int16_t accel[3], int16_t gyro[3])
{
	uint8_t buf[12];
	uint8_t reg = LSM6DSV_OUTX_L_G;

	/* Burst-read gyro (0x22-0x27) + accel (0x28-0x2D) in one I2C transaction.
	   IF_INC=1 in CTRL3 enables auto-increment across the contiguous block. */
	int ret = i2c_write_read(i2c_dev, addr, &reg, 1, buf, 12);
	if (ret) return ret;

	gyro[0]  = (int16_t)(buf[1]  << 8 | buf[0]);
	gyro[1]  = (int16_t)(buf[3]  << 8 | buf[2]);
	gyro[2]  = (int16_t)(buf[5]  << 8 | buf[4]);
	accel[0] = (int16_t)(buf[7]  << 8 | buf[6]);
	accel[1] = (int16_t)(buf[9]  << 8 | buf[8]);
	accel[2] = (int16_t)(buf[11] << 8 | buf[10]);

	return 0;
}

/* ===================================================================
 *  I3G4250D  —  3-axis gyroscope
 *  2 sensors: SA0=0 -> 0x68,  SA0=1 -> 0x69
 * =================================================================== */
#define I3G4250D_ADDR_A        0x68
#define I3G4250D_ADDR_B        0x69

#define I3G4250D_WHO_AM_I      0x0F
#define I3G4250D_WHO_AM_I_VAL  0xD3
#define I3G4250D_CTRL_REG1     0x20
#define I3G4250D_CTRL_REG4     0x23
#define I3G4250D_OUT_X_L       0x28

static const uint8_t i3g4250d_addrs[] = { I3G4250D_ADDR_A, I3G4250D_ADDR_B };

static int i3g4250d_init(uint8_t addr)
{
	uint8_t who;
	int ret = i2c_reg_read_byte(i2c_dev, addr, I3G4250D_WHO_AM_I, &who);
	if (ret) {
		printk("  I3G4250D@0x%02X: no response (%d)\n", addr, ret);
		return ret;
	}
	if (who != I3G4250D_WHO_AM_I_VAL) {
		printk("  I3G4250D@0x%02X: unexpected WHO_AM_I 0x%02X\n", addr, who);
		return -EIO;
	}

	/* PD=1, 100 Hz, all axes enabled */
	i2c_reg_write_byte(i2c_dev, addr, I3G4250D_CTRL_REG1, 0x0F);
	/* BDU=1, +/-245 dps */
	i2c_reg_write_byte(i2c_dev, addr, I3G4250D_CTRL_REG4, 0x80);

	printk("  I3G4250D@0x%02X: OK  (WHO=0x%02X)\n", addr, who);
	return 0;
}

static int i3g4250d_read(uint8_t addr, int16_t gyro[3])
{
	uint8_t buf[6];
	uint8_t reg = I3G4250D_OUT_X_L | 0x80; /* MSB set = auto-increment */
	int ret = i2c_write_read(i2c_dev, addr, &reg, 1, buf, 6);
	if (ret) return ret;

	gyro[0] = (int16_t)(buf[1] << 8 | buf[0]);
	gyro[1] = (int16_t)(buf[3] << 8 | buf[2]);
	gyro[2] = (int16_t)(buf[5] << 8 | buf[4]);
	return 0;
}

/* ===================================================================
 *  MLX90393  —  3-axis magnetometer  (command-based I2C protocol)
 * =================================================================== */
#define MLX90393_ADDR_A       0x0C
#define MLX90393_ADDR_B       0x0D
#define MLX90393_ADDR_C       0x0F

#define MLX90393_CMD_EX       0x80
#define MLX90393_CMD_RT       0xF0
#define MLX90393_CMD_SM_XYZT  0x3F
#define MLX90393_CMD_RM_XYZT  0x4F

static const uint8_t mlx90393_addrs[] = {
	MLX90393_ADDR_A, MLX90393_ADDR_B, MLX90393_ADDR_C
};

static int mlx90393_cmd(uint8_t addr, uint8_t cmd, uint8_t *status_out)
{
	uint8_t st;
	int ret = i2c_write_read(i2c_dev, addr, &cmd, 1, &st, 1);
	if (ret) return ret;
	if (status_out) *status_out = st;
	return 0;
}

static int mlx90393_init(uint8_t addr)
{
	uint8_t st;

	int ret = mlx90393_cmd(addr, MLX90393_CMD_EX, &st);
	if (ret) {
		printk("  MLX90393@0x%02X: no response (%d)\n", addr, ret);
		return ret;
	}
	k_msleep(1);

	uint8_t rt = MLX90393_CMD_RT;
	i2c_write(i2c_dev, &rt, 1, addr);
	k_msleep(15);

	for (int attempt = 0; attempt < 5; attempt++) {
		ret = mlx90393_cmd(addr, MLX90393_CMD_EX, &st);
		if (ret == 0) break;
		k_msleep(5);
	}
	if (ret) {
		printk("  MLX90393@0x%02X: no response after reset (%d)\n", addr, ret);
		return ret;
	}

	printk("  MLX90393@0x%02X: OK  (status=0x%02X)\n", addr, st);
	return 0;
}

static int mlx90393_start(uint8_t addr)
{
	uint8_t st;
	return mlx90393_cmd(addr, MLX90393_CMD_SM_XYZT, &st);
}

static int mlx90393_collect(uint8_t addr, int16_t mag[3], uint16_t *temp)
{
	uint8_t cmd = MLX90393_CMD_RM_XYZT;
	uint8_t buf[9];
	int ret = i2c_write_read(i2c_dev, addr, &cmd, 1, buf, 9);
	if (ret) return ret;

	*temp  = (uint16_t)(buf[1] << 8 | buf[2]);
	mag[0] = (int16_t)(buf[3] << 8 | buf[4]);
	mag[1] = (int16_t)(buf[5] << 8 | buf[6]);
	mag[2] = (int16_t)(buf[7] << 8 | buf[8]);
	return 0;
}

/* ===================================================================
 *  Sensor conversion constants
 *
 *  LSM6DSV accel @ +/-4 g    : 0.122 mg/LSB -> m/s^2
 *  LSM6DSV gyro  @ +/-250 dps: 8.75 mdps/LSB -> rad/s
 *  I3G4250D      @ +/-245 dps: 8.75 mdps/LSB -> rad/s
 *  MLX90393      @ default gain: ~0.150 uT/LSB (XY), ~0.242 uT/LSB (Z)
 * =================================================================== */
static constexpr float ACCEL_SCALE = 0.122e-3f * 9.80665f;  /* LSB -> m/s^2 */
static constexpr float GYRO_SCALE  = 8.75e-3f * (Math::PI / 180.0f);
static constexpr float MAG_SCALE_XY = 0.150e-6f;
static constexpr float MAG_SCALE_Z  = 0.242e-6f;

/* ===================================================================
 *  main  —  Air Bearing Demo
 * =================================================================== */
int main(void)
{
	/* --- LEDs --- */
	if (!gpio_is_ready_dt(&led0) ||
	    !gpio_is_ready_dt(&led1) ||
	    !gpio_is_ready_dt(&led2)) {
		printk("LED GPIO not ready\n");
		return 0;
	}
	gpio_pin_configure_dt(&led0, GPIO_OUTPUT_ACTIVE);
	gpio_pin_configure_dt(&led1, GPIO_OUTPUT_ACTIVE);
	gpio_pin_configure_dt(&led2, GPIO_OUTPUT_ACTIVE);

	/* --- Sensor power enable (PC9 high) --- */
	const struct device *gpioc = DEVICE_DT_GET(DT_NODELABEL(gpioc));
	if (!device_is_ready(gpioc)) {
		printk("GPIOC not ready\n");
		return 0;
	}
	gpio_pin_configure(gpioc, 9, GPIO_OUTPUT_ACTIVE);
	gpio_pin_set(gpioc, 9, 1);
	printk("Sensor power rail enabled (PC9 HIGH)\n");
	k_msleep(50);

	/* --- I2C --- */
	if (!device_is_ready(i2c_dev)) {
		printk("I2C1 bus not ready\n");
		return 0;
	}

	printk("\n========== Air Bearing Demo ==========\n");
	printk("I2C1 bus on PB8(SCL)/PB9(SDA) ready\n\n");

	/* --- I2C bus scan --- */
	printk("[I2C SCAN] probing 0x08-0x77...\n");
	int found = 0;
	for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
		uint8_t dummy;
		int ret = i2c_read(i2c_dev, &dummy, 1, addr);
		if (ret == 0) {
			printk("  ACK at 0x%02X\n", addr);
			found++;
		}
	}
	printk("  %d device(s) found\n\n", found);

	/* --- Init sensors --- */
	printk("[LSM6DSV] 6-axis IMU x2\n");
	for (size_t i = 0; i < ARRAY_SIZE(lsm6dsv_addrs); i++) {
		lsm6dsv_init(lsm6dsv_addrs[i]);
	}

	printk("[I3G4250D] Gyro x2\n");
	for (size_t i = 0; i < ARRAY_SIZE(i3g4250d_addrs); i++) {
		i3g4250d_init(i3g4250d_addrs[i]);
	}

	printk("[MLX90393] scanning all possible addresses (0x0C-0x1B)...\n");
	for (uint8_t a = 0x0C; a <= 0x1B; a++) {
		uint8_t st;
		if (mlx90393_cmd(a, MLX90393_CMD_EX, &st) == 0) {
			printk("  MLX90393 found at 0x%02X (status=0x%02X)\n", a, st);
		}
	}

	printk("[MLX90393] Mag x3 init\n");
	for (size_t i = 0; i < ARRAY_SIZE(mlx90393_addrs); i++) {
		mlx90393_init(mlx90393_addrs[i]);
	}

	k_msleep(50);

	/* --- ADCS Core init --- */
	printk("\n[ADCS] Initializing ADCSCore (Air Bearing)...\n");
	ADCS::Core adcs_core;
	printk("[ADCS] Mode: POINT (NDI controller)\n");
	printk("[ADCS] Sensors: accel (2x avg), gyro (4x avg), mag (3x avg)\n");
	printk("[ADCS] Actuators: reaction wheels (wheel_speeds zeroed until connected)\n\n");

	printk("========== Read loop start ==========\n\n");

	int cycle = 0;
	int64_t prev_time = k_uptime_get();
	while (1) {
		int64_t now = k_uptime_get();
		int64_t loop_dt_ms = now - prev_time;
		prev_time = now;
		bool verbose = (cycle % VERBOSE_EVERY == 0);

		if (verbose) {
			printk("--- cycle %d  (loop dt %lld ms) ---\n", cycle, loop_dt_ms);
		}

		/* ============================================================
		 *  Read IMU + gyro (fast I2C, ~1ms each)
		 * ============================================================ */
		int16_t imu_accel[2][3], imu_gyro[2][3];
		bool imu_ok[2] = {false, false};
		for (size_t i = 0; i < ARRAY_SIZE(lsm6dsv_addrs); i++) {
			if (lsm6dsv_read(lsm6dsv_addrs[i], imu_accel[i], imu_gyro[i]) == 0) {
				imu_ok[i] = true;
			}
		}

		int16_t gyr_raw[2][3];
		bool gyr_ok[2] = {false, false};
		for (size_t i = 0; i < ARRAY_SIZE(i3g4250d_addrs); i++) {
			if (i3g4250d_read(i3g4250d_addrs[i], gyr_raw[i]) == 0) {
				gyr_ok[i] = true;
			}
		}

		/* ============================================================
		 *  Read magnetometers — parallel start, single wait, read all
		 * ============================================================ */
		bool mag_started[3] = {false, false, false};
		for (size_t i = 0; i < ARRAY_SIZE(mlx90393_addrs); i++) {
			if (mlx90393_start(mlx90393_addrs[i]) == 0) {
				mag_started[i] = true;
			}
		}
		k_msleep(10);

		int16_t mag_raw[3][3];
		uint16_t mag_temp[3];
		bool mag_ok[3] = {false, false, false};
		for (size_t i = 0; i < ARRAY_SIZE(mlx90393_addrs); i++) {
			if (mag_started[i] &&
			    mlx90393_collect(mlx90393_addrs[i], mag_raw[i], &mag_temp[i]) == 0) {
				mag_ok[i] = true;
			}
		}

		if (verbose) {
			for (int i = 0; i < 2; i++) {
				if (imu_ok[i])
					printk("IMU%d A:%6d %6d %6d G:%6d %6d %6d\n", i,
					       imu_accel[i][0], imu_accel[i][1], imu_accel[i][2],
					       imu_gyro[i][0], imu_gyro[i][1], imu_gyro[i][2]);
			}
			for (int i = 0; i < 2; i++) {
				if (gyr_ok[i])
					printk("GYR%d G:%6d %6d %6d\n", i,
					       gyr_raw[i][0], gyr_raw[i][1], gyr_raw[i][2]);
			}
			for (int i = 0; i < 3; i++) {
				if (mag_ok[i])
					printk("MAG%d M:%6d %6d %6d T:%u\n", i,
					       mag_raw[i][0], mag_raw[i][1], mag_raw[i][2],
					       mag_temp[i]);
			}
		}

		/* ============================================================
		 *  Convert raw readings to physical units & average
		 * ============================================================ */

		/* Accelerometer: average across both LSM6DSV IMUs */
		float accel_avg[3] = {0.0f, 0.0f, 0.0f};
		int accel_count = 0;
		for (int i = 0; i < 2; i++) {
			if (imu_ok[i]) {
				for (int ax = 0; ax < 3; ax++)
					accel_avg[ax] += (float)imu_accel[i][ax] * ACCEL_SCALE;
				accel_count++;
			}
		}
		if (accel_count > 0) {
			for (int ax = 0; ax < 3; ax++)
				accel_avg[ax] /= (float)accel_count;
		}

		/* Gyro: average across LSM6DSV (x2) + I3G4250D (x2) = up to 4 sources */
		float gyro_avg[3] = {0.0f, 0.0f, 0.0f};
		int gyro_count = 0;
		for (int i = 0; i < 2; i++) {
			if (imu_ok[i]) {
				for (int ax = 0; ax < 3; ax++)
					gyro_avg[ax] += (float)imu_gyro[i][ax] * GYRO_SCALE;
				gyro_count++;
			}
		}
		for (int i = 0; i < 2; i++) {
			if (gyr_ok[i]) {
				for (int ax = 0; ax < 3; ax++)
					gyro_avg[ax] += (float)gyr_raw[i][ax] * GYRO_SCALE;
				gyro_count++;
			}
		}
		if (gyro_count > 0) {
			for (int ax = 0; ax < 3; ax++)
				gyro_avg[ax] /= (float)gyro_count;
		}

		/* Magnetometer: average across 3 MLX90393 sensors */
		float mag_avg[3] = {0.0f, 0.0f, 0.0f};
		int mag_count = 0;
		for (int i = 0; i < 3; i++) {
			if (mag_ok[i]) {
				mag_avg[0] += (float)mag_raw[i][0] * MAG_SCALE_XY;
				mag_avg[1] += (float)mag_raw[i][1] * MAG_SCALE_XY;
				mag_avg[2] += (float)mag_raw[i][2] * MAG_SCALE_Z;
				mag_count++;
			}
		}
		if (mag_count > 0) {
			for (int ax = 0; ax < 3; ax++)
				mag_avg[ax] /= (float)mag_count;
		}

		/* ============================================================
		 *  Pack ADCS::SensorData & run core
		 *
		 *  Air bearing SensorData:
		 *    accelerometer  [m/s^2]  gravity vector in body frame
		 *    gyro           [rad/s]  body rates
		 *    magnetometer   [T]      B-field in body frame
		 *    wheel_speeds   [rad/s]  reaction wheel speeds (zeroed until connected)
		 * ============================================================ */
		ADCS::SensorData sd;
		sd.unix_time     = (double)k_uptime_get() / 1000.0;
		sd.accelerometer = Math::Vec<3>{accel_avg[0], accel_avg[1], accel_avg[2]};
		sd.gyro          = Math::Vec<3>{gyro_avg[0], gyro_avg[1], gyro_avg[2]};
		sd.magnetometer  = Math::Vec<3>{mag_avg[0], mag_avg[1], mag_avg[2]};
		sd.wheel_speeds  = Math::Vec<4>::Zero();

		ADCS::AdcsOutput out = adcs_core.update(sd);

		/* Print ADCS output only on verbose cycles to avoid burning
		   the entire loop budget on float-to-string formatting. */
		if (verbose) {
			printk("[ADCS] q: %.4f %.4f %.4f %.4f\n",
			       (double)out.attitude_est(0), (double)out.attitude_est(1),
			       (double)out.attitude_est(2), (double)out.attitude_est(3));
			printk("[ADCS] w: %.4f %.4f %.4f rad/s\n",
			       (double)out.rate_est(0), (double)out.rate_est(1),
			       (double)out.rate_est(2));
			printk("[ADCS] tau_w: %.6f %.6f %.6f %.6f\n",
			       (double)out.wheel_torque(0), (double)out.wheel_torque(1),
			       (double)out.wheel_torque(2), (double)out.wheel_torque(3));
			printk("[ADCS] mtq: %.6f %.6f %.6f  valid:%d\n",
			       (double)out.mtq_dipole(0), (double)out.mtq_dipole(1),
			       (double)out.mtq_dipole(2),
			       out.estimator_valid ? 1 : 0);
		}

		/* --- LEDs --- */
		gpio_pin_toggle_dt(&led0);
		if (cycle % 2 == 0) gpio_pin_toggle_dt(&led1);
		if (cycle % 3 == 0) gpio_pin_toggle_dt(&led2);

		cycle++;
	}

	return 0;
}
