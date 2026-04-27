#include <cmath>

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>

#include "ADCSCore.hpp"

#define VERBOSE_EVERY    100  /* full sensor dump every Nth cycle */
#define TELEMETRY_EVERY  10   /* ADCS estimator/controller telemetry cadence */

/* 1: command MissionMode::SAFE -> B-dot detumble (wheels zeroed, mtq active).
 * 0: default BEARING -> detumble until rates calm, then NDI point. */
#ifndef ADCS_AIR_BEARING_FORCE_DETUMBLE
#define ADCS_AIR_BEARING_FORCE_DETUMBLE 1
#endif

/* ADCS loop runs in its own preemptive thread (large stack: matrix math in Core). */
#define ADCS_LOOP_STACK_SIZE  16384
#define ADCS_LOOP_PRIORITY    K_PRIO_PREEMPT(5)

static struct k_thread adcs_loop_thread;
K_THREAD_STACK_DEFINE(adcs_loop_stack, ADCS_LOOP_STACK_SIZE);

/* File scope: loop thread must not use Core on main's stack after main returns. */
static ADCS::Core adcs_core;

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

/* ===================================================================
 *  CALIBRATION CONSTANTS — paste output of AirBearingCalibration.m here.
 *
 *  Build with -DCAL_EMIT=ON to dump raw CSV instead of normal telemetry:
 *    west build -p always -b ut_core ut-core/app/adcsComputeTest \
 *               -- -DAIR_BEARING=ON -DCAL_EMIT=ON
 *  In MATLAB:
 *    AirBearingCalibration('gyro',  "COM5")
 *    AirBearingCalibration('accel', "COM5", 'gravity', [0 0 -9.80665])
 *    AirBearingCalibration('mag',   "COM5", 'freq', 0.5, 'duration', 30)
 *  Paste the printed constants below, rebuild without -DCAL_EMIT.
 * =================================================================== */

/* Gyro bias [rad/s], applied AFTER GYRO_CORR_* scale correction.
 * Subtract from each per-sensor gyro vector. */
static constexpr float GYRO_BIAS_LSM0[3] = {-0.003214f, -0.003900f, -0.003059f};
static constexpr float GYRO_BIAS_LSM1[3] = {+0.003477f, -0.002018f, +0.001290f};
static constexpr float GYRO_BIAS_I3G0[3] = {-0.008273f, -0.001145f, -0.016675f};
static constexpr float GYRO_BIAS_I3G1[3] = {+0.003940f, -0.000403f, -0.008821f};

/* Accelerometer bias [m/s^2], in raw sensor frame, BEFORE the body remap.
 * Computed as mean(reading) - expected_gravity_vector. */
static constexpr float ACCEL_BIAS_LSM0[3] = {+0.101313f, -0.237740f, +0.269004f};
    // raw mean = [+0.1013 -0.2377 -9.5376]  bias = mean - g_expected
static constexpr float ACCEL_BIAS_LSM1[3] = {+0.155542f, -0.213896f, +0.056810f};
    // raw mean = [+0.1555 -0.2139 -9.7498]  bias = mean - g_expected

/* Magnetometer hard-iron bias and diagonal scale from magCalTest. */
static const float MAG_BIAS[3][3] = {
	{-396.500f, 2064.500f, 1274.500f},
	{3192.000f, -2262.000f, 133.500f},
	{-375.500f, 2299.000f, -2352.000f},
};

static const float MAG_CAL_SCALE[3][3] = {
	{0.894505f, 0.966746f, 1.179710f},
	{0.923704f, 0.989683f, 1.102564f},
	{0.887704f, 0.950533f, 1.217349f},
};

/* Per-sensor mag alignment 3x3 (row-major).  Applied AFTER bias+scale.
 * Identity = no alignment correction; replace with rotation that maps each
 * sensor's axes to the helmholtz/body frame, identified by sin/cos
 * demodulation in AirBearingCalibration('mag', ...). */
static const float MAG_ALIGN[3][3][3] = {
    /* sensor 0 */
    { {+0.268283f, +0.963232f, +0.014448f},
      {+0.986300f, +0.164773f, +0.007835f},
      {+0.058155f, +0.146699f, +0.987470f} },
    /* sensor 1 */
    { {+0.269730f, +0.962921f, +0.005324f},
      {+0.986200f, +0.165359f, +0.008164f},
      {+0.068165f, +0.172492f, +0.982649f} },
    /* sensor 2 */
    { {+0.262030f, +0.965039f, +0.006230f},
      {+0.984783f, +0.173571f, +0.008664f},
      {+0.086555f, +0.053610f, +0.994804f} },
};

/* Raw-CSV emit mode for AirBearingCalibration.m capture. When ON, suppresses
 * verbose dumps and ADCS telemetry so the serial stream is clean CSV. */
#ifndef CAL_EMIT
#define CAL_EMIT 0
#endif

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

static void mag_apply_calibration(size_t sensor_idx, const int16_t raw[3], float cal[3])
{
	float scaled[3];
	for (int ax = 0; ax < 3; ax++) {
		float centered = (float)raw[ax] - MAG_BIAS[sensor_idx][ax];
		scaled[ax] = centered * MAG_CAL_SCALE[sensor_idx][ax];
	}
	for (int i = 0; i < 3; i++) {
		cal[i] = MAG_ALIGN[sensor_idx][i][0] * scaled[0]
		       + MAG_ALIGN[sensor_idx][i][1] * scaled[1]
		       + MAG_ALIGN[sensor_idx][i][2] * scaled[2];
	}
}

/* ===================================================================
 *  Sensor conversion constants
 *
 *  LSM6DSV accel @ +/-4 g    : 0.122 mg/LSB -> m/s^2
 *  LSM6DSV gyro  @ +/-250 dps: 8.75 mdps/LSB -> rad/s
 *  I3G4250D      @ +/-245 dps: 8.75 mdps/LSB -> rad/s
 *  MLX90393      @ default gain: ~0.150 uT/LSB (XY), ~0.242 uT/LSB (Z)
 * =================================================================== */
static constexpr float ACCEL_SCALE = 0.061e-3f * 9.80665f;  /* LSB -> m/s^2  (+/-2g mode, 0.061 mg/LSB) */

/* Per-sensor gyro scales (LSB -> rad/s). Nominal is 8.75 mdps/LSB for both
 * (LSM6DSV @ +/-250 dps, I3G4250D @ +/-245 dps) but real-world scale factor
 * error on these parts (and/or undocumented register-encoding quirks on the
 * LSM6DSV variant) is far larger than the datasheet "1-3%" — calibration
 * against a known -180 deg Z rotation showed each sensor over-reporting by
 * 1.4x-2.3x, and with per-device spread. We correct each chip on each axis
 * independently. Calibrate by rotating the board a precisely known angle
 * about each body axis, integrating each sensor's output (MATLAB cumtrapz),
 * and setting the correction = true_angle / measured_angle. */
static constexpr float GYRO_LSB_RADPS = 8.75e-3f * (Math::PI / 180.0f);

/* Per-device, per-axis correction factors (dimensionless), calibrated against
 * +/-180 deg rotations about each body axis (short-window integrations, noise
 * bleed minimized).  I3G pair reads near true; LSM pair reads ~2x high -- a
 * strong indicator the LSM6DSV CTRL2_G actually selects a different full-scale
 * range than the datasheet variant we assumed (likely +/-125 dps, which would
 * halve real LSBs vs. the 8.75 mdps/LSB we apply).  Until the register write
 * at lsm6dsv_init() is fixed, these runtime corrections compensate.
 *
 * Calibration integrations (absolute-value measured / 180 deg):
 *            X            Y            Z
 *   LSM0:  349.9 -> 0.5144 | 371.3 -> 0.4848 | 353.9 -> 0.5086
 *   LSM1:  311.5 -> 0.5778 | 398.0 -> 0.4523 | 405.8 -> 0.4436
 *   I3G0:  177.1 -> 1.0163 | 179.2 -> 1.0045 | 171.4 -> 1.0502
 *   I3G1:  164.9 -> 1.0915 | 174.7 -> 1.0303 | 182.5 -> 0.9863
 */
static constexpr float GYRO_CORR_LSM0[3] = { 0.5144f, 0.4848f, 0.5086f };
static constexpr float GYRO_CORR_LSM1[3] = { 0.5778f, 0.4523f, 0.4436f };
static constexpr float GYRO_CORR_I3G0[3] = { 1.0163f, 1.0045f, 1.0502f };
static constexpr float GYRO_CORR_I3G1[3] = { 1.0915f, 1.0303f, 0.9863f };

/* Per-set enable flags for A/B testing — flip one off, rotate the board,
 * and watch whether the observer's integrated attitude matches truth. */
#ifndef GYRO_FUSE_LSM
#define GYRO_FUSE_LSM 1
#endif
#ifndef GYRO_FUSE_I3G
#define GYRO_FUSE_I3G 1
#endif

/* Print each sensor's converted gyro vector once every N cycles so you can
 * see live disagreement between the two sets on rotation. Set to 0 to disable. */
#ifndef GYRO_DIAG_EVERY
#define GYRO_DIAG_EVERY 30
#endif

static constexpr float MAG_SCALE_XY = 0.150e-6f;
static constexpr float MAG_SCALE_Z  = 0.242e-6f;

/* LSM6DSV reports specific force (reaction to gravity). The air-bearing observer
 * compares a *gravity-direction* unit vector to g_ref=(0,0,1); negate so rest
 * reading matches identity attitude (avoids huge accel innovation -> bogus β̂). */
#ifndef AIR_BEARING_ACCEL_AS_GRAVITY_NEGATE
#define AIR_BEARING_ACCEL_AS_GRAVITY_NEGATE 0
#endif

/* ===================================================================
 *  ADCS compute thread  —  sensors, Core::update, heartbeat LEDs
 * =================================================================== */
static void adcs_loop(void *, void *, void *)
{
	int cycle = 0;
	int64_t prev_time = k_uptime_get();
	float gyro_norm_max_window = 0.0f;

	while (1) {
		int64_t now = k_uptime_get();
		int64_t loop_dt_ms = now - prev_time;
		prev_time = now;
		bool verbose = (cycle % VERBOSE_EVERY == 0);
		bool telemetry = (cycle % TELEMETRY_EVERY == 0);

#if !CAL_EMIT
		if (verbose || telemetry) {
			printk("--- cycle %d  (loop dt %lld ms) ---\n", cycle, loop_dt_ms);
		}
#else
		(void)loop_dt_ms; (void)verbose; (void)telemetry;
#endif

		/* Read IMU + gyro (fast I2C, ~1ms each) */
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

		/* Magnetometers — parallel start, single wait, read all */
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

#if CAL_EMIT
		{
			int64_t t_ms = k_uptime_get();
			for (int i = 0; i < 2; i++) {
				if (imu_ok[i]) {
					printk("IMUCSV,%lld,%d,%d,%d,%d,%d,%d,%d\n",
					       t_ms, i,
					       imu_accel[i][0], imu_accel[i][1], imu_accel[i][2],
					       imu_gyro[i][0], imu_gyro[i][1], imu_gyro[i][2]);
				}
			}
			for (int i = 0; i < 2; i++) {
				if (gyr_ok[i]) {
					printk("GYRCSV,%lld,%d,%d,%d,%d\n",
					       t_ms, i,
					       gyr_raw[i][0], gyr_raw[i][1], gyr_raw[i][2]);
				}
			}
			for (int i = 0; i < 3; i++) {
				if (mag_ok[i]) {
					printk("MAGCSV,%lld,%d,0x%02X,%d,%d,%d,%u,0\n",
					       t_ms, i, mlx90393_addrs[i],
					       mag_raw[i][0], mag_raw[i][1], mag_raw[i][2],
					       mag_temp[i]);
				}
			}
		}
#else
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
#endif

		/* Convert raw readings to physical units & average */
		float accel_avg[3] = {0.0f, 0.0f, 0.0f};
		int accel_count = 0;
		for (int i = 0; i < 2; i++) {
			if (imu_ok[i]) {
				const float *abias = (i == 0) ? ACCEL_BIAS_LSM0 : ACCEL_BIAS_LSM1;
				for (int ax = 0; ax < 3; ax++)
					accel_avg[ax] += (float)imu_accel[i][ax] * ACCEL_SCALE - abias[ax];
				accel_count++;
			}
		}
		if (accel_count > 0) {
			for (int ax = 0; ax < 3; ax++)
				accel_avg[ax] /= (float)accel_count;
		}

		/* I2C frame glitches occasionally pass a successful read but with corrupt bytes
		 * (near-int16-max LSBs -> multi-rad/s spikes). Reject per-sensor outliers
		 * before averaging so one bad frame can't poison the fused gyro. */
		constexpr float GYRO_SANE_MAX = 5.0f;  /* rad/s; ~286 deg/s — far beyond bench use */
		auto gyro_sane_perdev = [](const int16_t raw[3], const float corr[3]) -> bool {
			for (int ax = 0; ax < 3; ax++) {
				float v = (float)raw[ax] * GYRO_LSB_RADPS * corr[ax];
				if (!(v >= -GYRO_SANE_MAX && v <= GYRO_SANE_MAX)) return false;
			}
			return true;
		};
		auto apply_perdev = [](float out[3], const int16_t raw[3], const float corr[3]) {
			for (int ax = 0; ax < 3; ax++) {
				out[ax] = (float)raw[ax] * GYRO_LSB_RADPS * corr[ax];
			}
		};

		/* Per-sensor converted vectors [rad/s], useful for A/B disagreement diagnosis. */
		float lsm_gyro_rps[2][3] = {{0,0,0},{0,0,0}};
		float i3g_gyro_rps[2][3] = {{0,0,0},{0,0,0}};

		auto debias = [](float v[3], const float bias[3]) {
			for (int ax = 0; ax < 3; ax++) v[ax] -= bias[ax];
		};

		float gyro_avg[3] = {0.0f, 0.0f, 0.0f};
		int gyro_count = 0;
#if GYRO_FUSE_LSM
		if (imu_ok[0] && gyro_sane_perdev(imu_gyro[0], GYRO_CORR_LSM0)) {
			apply_perdev(lsm_gyro_rps[0], imu_gyro[0], GYRO_CORR_LSM0);
			debias(lsm_gyro_rps[0], GYRO_BIAS_LSM0);
			for (int ax = 0; ax < 3; ax++) gyro_avg[ax] += lsm_gyro_rps[0][ax];
			gyro_count++;
		}
		if (imu_ok[1] && gyro_sane_perdev(imu_gyro[1], GYRO_CORR_LSM1)) {
			apply_perdev(lsm_gyro_rps[1], imu_gyro[1], GYRO_CORR_LSM1);
			debias(lsm_gyro_rps[1], GYRO_BIAS_LSM1);
			for (int ax = 0; ax < 3; ax++) gyro_avg[ax] += lsm_gyro_rps[1][ax];
			gyro_count++;
		}
#else
		if (imu_ok[0] && gyro_sane_perdev(imu_gyro[0], GYRO_CORR_LSM0)) { apply_perdev(lsm_gyro_rps[0], imu_gyro[0], GYRO_CORR_LSM0); debias(lsm_gyro_rps[0], GYRO_BIAS_LSM0); }
		if (imu_ok[1] && gyro_sane_perdev(imu_gyro[1], GYRO_CORR_LSM1)) { apply_perdev(lsm_gyro_rps[1], imu_gyro[1], GYRO_CORR_LSM1); debias(lsm_gyro_rps[1], GYRO_BIAS_LSM1); }
#endif
#if GYRO_FUSE_I3G
		if (gyr_ok[0] && gyro_sane_perdev(gyr_raw[0], GYRO_CORR_I3G0)) {
			apply_perdev(i3g_gyro_rps[0], gyr_raw[0], GYRO_CORR_I3G0);
			debias(i3g_gyro_rps[0], GYRO_BIAS_I3G0);
			for (int ax = 0; ax < 3; ax++) gyro_avg[ax] += i3g_gyro_rps[0][ax];
			gyro_count++;
		}
		if (gyr_ok[1] && gyro_sane_perdev(gyr_raw[1], GYRO_CORR_I3G1)) {
			apply_perdev(i3g_gyro_rps[1], gyr_raw[1], GYRO_CORR_I3G1);
			debias(i3g_gyro_rps[1], GYRO_BIAS_I3G1);
			for (int ax = 0; ax < 3; ax++) gyro_avg[ax] += i3g_gyro_rps[1][ax];
			gyro_count++;
		}
#else
		if (gyr_ok[0] && gyro_sane_perdev(gyr_raw[0], GYRO_CORR_I3G0)) { apply_perdev(i3g_gyro_rps[0], gyr_raw[0], GYRO_CORR_I3G0); debias(i3g_gyro_rps[0], GYRO_BIAS_I3G0); }
		if (gyr_ok[1] && gyro_sane_perdev(gyr_raw[1], GYRO_CORR_I3G1)) { apply_perdev(i3g_gyro_rps[1], gyr_raw[1], GYRO_CORR_I3G1); debias(i3g_gyro_rps[1], GYRO_BIAS_I3G1); }
#endif
		if (gyro_count > 0) {
			for (int ax = 0; ax < 3; ax++)
				gyro_avg[ax] /= (float)gyro_count;
		}

#if GYRO_DIAG_EVERY > 0 && !CAL_EMIT
		if ((cycle % GYRO_DIAG_EVERY) == 0) {
			printk("GYRO [rad/s] LSM0:%+6.3f %+6.3f %+6.3f | LSM1:%+6.3f %+6.3f %+6.3f | "
			       "I3G0:%+6.3f %+6.3f %+6.3f | I3G1:%+6.3f %+6.3f %+6.3f | FUSED:%+6.3f %+6.3f %+6.3f\n",
			       (double)lsm_gyro_rps[0][0], (double)lsm_gyro_rps[0][1], (double)lsm_gyro_rps[0][2],
			       (double)lsm_gyro_rps[1][0], (double)lsm_gyro_rps[1][1], (double)lsm_gyro_rps[1][2],
			       (double)i3g_gyro_rps[0][0], (double)i3g_gyro_rps[0][1], (double)i3g_gyro_rps[0][2],
			       (double)i3g_gyro_rps[1][0], (double)i3g_gyro_rps[1][1], (double)i3g_gyro_rps[1][2],
			       (double)gyro_avg[0], (double)gyro_avg[1], (double)gyro_avg[2]);
		}
#endif

		float mag_avg[3] = {0.0f, 0.0f, 0.0f};
		int mag_count = 0;
		for (int i = 0; i < 3; i++) {
			if (mag_ok[i]) {
				float mag_cal[3];
				mag_apply_calibration(i, mag_raw[i], mag_cal);
				mag_avg[0] += mag_cal[0] * MAG_SCALE_XY;
				mag_avg[1] += mag_cal[1] * MAG_SCALE_XY;
				mag_avg[2] += mag_cal[2] * MAG_SCALE_Z;
				mag_count++;
			}
		}
		if (mag_count > 0) {
			for (int ax = 0; ax < 3; ax++)
				mag_avg[ax] /= (float)mag_count;
		}

		ADCS::SensorData sd;
		sd.unix_time = (double)k_uptime_get() / 1000.0;
		/* Body-frame remap, identified by air-bearing test:
		 *   accel: x = +ay, y = -ax, z = -az  (X axis flipped vs. prior)
		 *   gyro : x = -gy, y = +gx, z = -gz  (Y axis flipped vs. prior)
		 *   mag  : identity on mag_avg (helmholtz +X/+Y/+Z reads +X/+Y/+Z)
		 * Per-sensor cross-stack alignment for the mag is in MAG_ALIGN. */
#if AIR_BEARING_ACCEL_AS_GRAVITY_NEGATE
		sd.accelerometer = Math::Vec<3>{accel_avg[1], accel_avg[0], -accel_avg[2]};
#else
		sd.accelerometer = Math::Vec<3>{accel_avg[1], -accel_avg[0], -accel_avg[2]};
#endif
		sd.gyro = Math::Vec<3>{-gyro_avg[1], gyro_avg[0], -gyro_avg[2]};

		/* Track worst-case gyro norm since last telemetry print, for motion-gate diagnosis. */
		float gn = sqrtf(gyro_avg[0]*gyro_avg[0] +
		                 gyro_avg[1]*gyro_avg[1] +
		                 gyro_avg[2]*gyro_avg[2]);
		if (gn > gyro_norm_max_window) gyro_norm_max_window = gn;
		sd.magnetometer  = Math::Vec<3>{mag_avg[0], mag_avg[1], mag_avg[2]};
		sd.wheel_speeds  = Math::Vec<4>::Zero();

		ADCS::Command adcs_cmd;
#if ADCS_AIR_BEARING_FORCE_DETUMBLE
		adcs_cmd.mode = ADCS::MissionMode::OFF;
#endif
		ADCS::AdcsOutput out = adcs_core.update(sd, adcs_cmd);

#if !CAL_EMIT
		if (telemetry) {
			const char *mode = "?";
			switch (out.current_mode) {
			case ADCS::MissionMode::OFF:
				mode = "OFF";
				break;
			case ADCS::MissionMode::SAFE:
				mode = "SAFE(detumble)"; /* wheels forced 0; B-dot drives mtq */
				break;
			case ADCS::MissionMode::BEARING:
				mode = "BEARING(point)"; /* NDI wheels; mtq only when desat active */
				break;
			}
			printk("[ADCS] mode=%s  meas|gy|=%.4f rad/s  valid:%d  init:%s (%d/%d)\n", mode,
			       (double)(std::fabs((double)sd.gyro(0)) +
					std::fabs((double)sd.gyro(1)) +
					std::fabs((double)sd.gyro(2))),
			       out.estimator_valid ? 1 : 0,
			       adcs_core.observerRunning() ? "RUN" : "COLLECT",
			       adcs_core.observerInitSamples(),
			       adcs_core.observerInitTarget());
			printk("[ADCS] q: %.4f %.4f %.4f %.4f\n",
			       (double)out.attitude_est(0), (double)out.attitude_est(1),
			       (double)out.attitude_est(2), (double)out.attitude_est(3));
			printk("[ADCS] w: %.4f %.4f %.4f rad/s\n",
			       (double)out.rate_est(0), (double)out.rate_est(1),
			       (double)out.rate_est(2));
			double bn = std::sqrt((double)sd.magnetometer(0) * (double)sd.magnetometer(0) +
			                      (double)sd.magnetometer(1) * (double)sd.magnetometer(1) +
			                      (double)sd.magnetometer(2) * (double)sd.magnetometer(2));
			printk("[ADCS] B [T]: %.4e %.4e %.4e  |B|=%.4e (%.1f uT)\n",
			       (double)sd.magnetometer(0), (double)sd.magnetometer(1),
			       (double)sd.magnetometer(2), bn, bn * 1e6);
			printk("[ADCS] a [m/s^2]: %.4e %.4e %.4e\n",
			       (double)sd.accelerometer(0), (double)sd.accelerometer(1),
			       (double)sd.accelerometer(2));
			printk("[ADCS] gy_raw [rad/s]: %.4f %.4f %.4f  |max_win|=%.4f\n",
			       (double)sd.gyro(0), (double)sd.gyro(1), (double)sd.gyro(2),
			       (double)gyro_norm_max_window);
			gyro_norm_max_window = 0.0f;
			/* %.6f hides sub-micro torques; dipole is [A·m²] not N·m */
			printk("[ADCS] tau_w [N*m]: %.4e %.4e %.4e %.4e\n",
			       (double)out.wheel_torque(0), (double)out.wheel_torque(1),
			       (double)out.wheel_torque(2), (double)out.wheel_torque(3));
			printk("[ADCS] mtq [A*m^2]: %.4e %.4e %.4e\n",
			       (double)out.mtq_dipole(0), (double)out.mtq_dipole(1),
			       (double)out.mtq_dipole(2));
		}
#else
		(void)out;
#endif

		gpio_pin_toggle_dt(&led0);
		if (cycle % 2 == 0) gpio_pin_toggle_dt(&led1);
		if (cycle % 3 == 0) gpio_pin_toggle_dt(&led2);

		cycle++;
	}
}

/* ===================================================================
 *  main  —  Air Bearing Demo (bring-up); ADCS loop in adcs_loop thread
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

	printk("\n[ADCS] ADCSCore (Air Bearing) — compute thread starting\n");
#if ADCS_AIR_BEARING_FORCE_DETUMBLE
	printk("[ADCS] Command: SAFE — forced B-dot detumble (tau_w=0 by policy)\n");
#else
	printk("[ADCS] Command: BEARING — detumble until arm, then NDI point\n");
#endif
	printk("[ADCS] Sensors: accel (2x avg), gyro (4x avg), mag (3x avg)\n");
	printk("[ADCS] Actuators: reaction wheels (wheel_speeds zeroed until connected)\n\n");

	k_thread_create(&adcs_loop_thread, adcs_loop_stack,
			K_THREAD_STACK_SIZEOF(adcs_loop_stack),
			adcs_loop, NULL, NULL, NULL,
			ADCS_LOOP_PRIORITY, 0, K_NO_WAIT);

	printk("========== ADCS loop thread started ==========\n\n");

	/* Main exits; ADCS runs on adcs_loop stack. */
	return 0;
}
