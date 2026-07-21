#include <cmath>
#include <cstring>

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/can.h>
#include <zephyr/logging/log.h>

#include "ADCSCore.hpp"
#include "../../../common/can_proto.h"

LOG_MODULE_REGISTER(adcs_air_bearing, LOG_LEVEL_INF);
 
#define VERBOSE_EVERY    100  /* full sensor dump every Nth cycle */
#define TELEMETRY_EVERY  10   /* ADCS estimator/controller telemetry cadence */
#define HEARTBEAT_INTERVAL_MS 1000
#define ADCS_TELEM_INTERVAL_MS 1000

/* CAN protocol defaults mirroring other UT-core boards. */
#define NODE_ID        ADCS_ID
#define PRIO_LOW       4
#define PIN_SHDN       10
#define PIN_SILENT      9

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
static const struct device *can_dev = DEVICE_DT_GET(DT_NODELABEL(fdcan1));
static const struct device *gpioa = DEVICE_DT_GET(DT_NODELABEL(gpioa));
CAN_MSGQ_DEFINE(rxq, 16);

typedef struct {
	uint8_t priority;
	uint8_t src;
	uint8_t dst;
	uint8_t msg_class;
	uint8_t dlc;
	uint8_t data[8];
} can_packet_t;

typedef struct {
	bool valid;
	float attitude_q[4];
	float wheel_rpm[4];
	float mtq_dipole[3];
} adcs_can_snapshot_t;

static adcs_can_snapshot_t g_can_snapshot;
static struct k_mutex g_can_snapshot_mutex;

static inline int16_t float_to_q15(float x)
{
	const float scaled = x * 10000.0f;
	if (scaled > 32767.0f) return 32767;
	if (scaled < -32768.0f) return -32768;
	return (int16_t)scaled;
}

static inline void pack_be16(uint8_t *dst, int16_t val)
{
	dst[0] = (uint8_t)(((uint16_t)val >> 8) & 0xFF);
	dst[1] = (uint8_t)((uint16_t)val & 0xFF);
}

static inline int16_t float_to_i16_rpm(float x)
{
	const float clamped = fminf(fmaxf(x, -32768.0f), 32767.0f);
	return (int16_t)lrintf(clamped);
}

static void send_simple(uint8_t dst, uint8_t cls, uint8_t op, uint8_t val)
{
	struct can_frame f = {0};
	f.id = CAN_ID_FULL(PRIO_LOW, NODE_ID, dst, cls);
	f.flags = CAN_FRAME_IDE;
	can_fill_payload(&f, NODE_ID, op, val, 0, 0, 0, 0, 0);
	can_send(can_dev, &f, K_NO_WAIT, NULL, NULL);
}

static void send_heartbeat(void)
{
	send_simple(CAN_BROADCAST, CLS_HEARTBEAT, OP_HEARTBEAT, 0x00);
}

static void send_soh_attitude(void)
{
	adcs_can_snapshot_t snap;
	k_mutex_lock(&g_can_snapshot_mutex, K_FOREVER);
	snap = g_can_snapshot;
	k_mutex_unlock(&g_can_snapshot_mutex);

	struct can_frame f = {0};
	f.id = CAN_ID_FULL(PRIO_LOW, NODE_ID, CAN_BROADCAST, CLS_HEALTH);
	f.flags = CAN_FRAME_IDE;
	f.dlc = 8;
	f.data[0] = NODE_ID;
	f.data[1] = OP_ADCS_SOH_ATTITUDE;

	/* Scaffold: pack qx, qy, qz as q15-ish fixed-point; q0 can be added in a follow-up frame. */
	pack_be16(&f.data[2], float_to_q15(snap.attitude_q[1]));
	pack_be16(&f.data[4], float_to_q15(snap.attitude_q[2]));
	pack_be16(&f.data[6], float_to_q15(snap.attitude_q[3]));
	can_send(can_dev, &f, K_NO_WAIT, NULL, NULL);
}

static void send_wheel_rpm_scaffold(void)
{
	adcs_can_snapshot_t snap;
	k_mutex_lock(&g_can_snapshot_mutex, K_FOREVER);
	snap = g_can_snapshot;
	k_mutex_unlock(&g_can_snapshot_mutex);

	struct can_frame f = {0};
	f.id = CAN_ID_FULL(PRIO_LOW, NODE_ID, CAN_BROADCAST, CLS_TELEMETRY);
	f.flags = CAN_FRAME_IDE;
	f.dlc = 8;
	f.data[0] = NODE_ID;
	f.data[1] = OP_ADCS_WHEEL_RPM;

	/* Scaffold: first 3 wheels in frame; wheel4 can be split into a second frame when finalized. */
	pack_be16(&f.data[2], (int16_t)snap.wheel_rpm[0]);
	pack_be16(&f.data[4], (int16_t)snap.wheel_rpm[1]);
	pack_be16(&f.data[6], (int16_t)snap.wheel_rpm[2]);
	can_send(can_dev, &f, K_NO_WAIT, NULL, NULL);
}

static void send_mtq_dipole_scaffold(void)
{
	adcs_can_snapshot_t snap;
	k_mutex_lock(&g_can_snapshot_mutex, K_FOREVER);
	snap = g_can_snapshot;
	k_mutex_unlock(&g_can_snapshot_mutex);

	struct can_frame f = {0};
	f.id = CAN_ID_FULL(PRIO_LOW, NODE_ID, CAN_BROADCAST, CLS_TELEMETRY);
	f.flags = CAN_FRAME_IDE;
	f.dlc = 8;
	f.data[0] = NODE_ID;
	f.data[1] = OP_ADCS_MTQ_DIPOLE;

	/* Scaffold: mx/my/mz packed as 1e-4 A*m^2 fixed-point. */
	pack_be16(&f.data[2], float_to_q15(snap.mtq_dipole[0]));
	pack_be16(&f.data[4], float_to_q15(snap.mtq_dipole[1]));
	pack_be16(&f.data[6], float_to_q15(snap.mtq_dipole[2]));
	can_send(can_dev, &f, K_NO_WAIT, NULL, NULL);
}

static void send_mtq_dipole_command_solar(float mx, float my, float mz)
{
	struct can_frame f = {0};
	f.id = CAN_ID_FULL(PRIO_LOW, NODE_ID, SOLAR_ID, CLS_COMMAND);
	f.flags = CAN_FRAME_IDE;
	f.dlc = 8;
	f.data[0] = NODE_ID;
	f.data[1] = OP_SET_MAG_DIPOLE;

	/* Pack commanded dipole [mx,my,mz] in one frame (p2..p7), 1e-4 A*m^2 per LSB. */
	pack_be16(&f.data[2], float_to_q15(mx));
	pack_be16(&f.data[4], float_to_q15(my));
	pack_be16(&f.data[6], float_to_q15(mz));
	can_send(can_dev, &f, K_NO_WAIT, NULL, NULL);
}

static void send_wheel_rpm_command_motor(uint8_t motor_sel, float rpm_cmd)
{
	struct can_frame f = {0};
	const int16_t rpm_i16 = float_to_i16_rpm(rpm_cmd);

	f.id = CAN_ID_FULL(PRIO_LOW, NODE_ID, MOTOR_ID, CLS_COMMAND);
	f.flags = CAN_FRAME_IDE;
	can_fill_payload(&f, NODE_ID, OP_SET_WHEEL_RPM,
			 motor_sel,
			 (uint8_t)(((uint16_t)rpm_i16 >> 8) & 0xFF),
			 (uint8_t)((uint16_t)rpm_i16 & 0xFF),
			 0, 0, 0);
	can_send(can_dev, &f, K_NO_WAIT, NULL, NULL);
}

static void send_wheel_rpm_commands_motor(const float wheel_rpm_cmd[4])
{
	for (uint8_t motor = 1; motor <= 4; motor++) {
		send_wheel_rpm_command_motor(motor, wheel_rpm_cmd[motor - 1]);
	}
}

static void tcan3403_wakeup(void)
{
	gpio_pin_configure(gpioa, PIN_SHDN, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure(gpioa, PIN_SILENT, GPIO_OUTPUT_INACTIVE);
	k_msleep(1);
	LOG_INF("TCAN3403 Awake");
}

static void can_decode(const struct can_frame *f, can_packet_t *pkt)
{
	uint32_t id = f->id;
	pkt->priority = (id >> 26) & 0x07;
	pkt->src = (id >> 14) & 0xFF;
	pkt->dst = (id >> 6) & 0xFF;
	pkt->msg_class = id & 0x3F;
	pkt->dlc = f->dlc;
	memcpy(pkt->data, f->data, f->dlc);
}

static void can_setup(void)
{
	if (!device_is_ready(can_dev)) {
		LOG_ERR("CAN not ready");
		return;
	}

	can_set_bitrate(can_dev, 500000);
	can_set_mode(can_dev, CAN_MODE_NORMAL);
	can_start(can_dev);

	const struct can_filter to_me = {
		.id = CAN_DST(NODE_ID),
		.mask = CAN_DST_MASK_29,
		.flags = CAN_FILTER_IDE,
	};
	const struct can_filter bcast = {
		.id = CAN_DST(CAN_BROADCAST),
		.mask = CAN_DST_MASK_29,
		.flags = CAN_FILTER_IDE,
	};

	can_add_rx_filter_msgq(can_dev, &rxq, &to_me);
	can_add_rx_filter_msgq(can_dev, &rxq, &bcast);
	LOG_INF("CAN initialized (29-bit extended), node=0x%02X", NODE_ID);
}

static void can_dispatch(const can_packet_t *pkt)
{
	switch (pkt->msg_class) {
	case CLS_HEARTBEAT:
		LOG_DBG("RX heartbeat from 0x%02X", pkt->src);
		break;
	case CLS_COMMAND:
		/* Scaffold hook for future ADCS command handling (mode changes, wheel commands, etc). */
		LOG_DBG("RX command from 0x%02X op=0x%02X", pkt->src, pkt->data[1]);
		break;
	default:
		break;
	}
}

#define CAN_RX_STACK_SIZE  1024
#define CAN_RX_PRIORITY    5

static void can_rx_thread(void *a, void *b, void *c)
{
	struct can_frame frame;
	can_packet_t pkt;
	while (1) {
		if (k_msgq_get(&rxq, &frame, K_FOREVER) == 0) {
			can_decode(&frame, &pkt);
			can_dispatch(&pkt);
		}
	}
}

K_THREAD_DEFINE(can_rx_tid, CAN_RX_STACK_SIZE,
	can_rx_thread, NULL, NULL, NULL,
	CAN_RX_PRIORITY, 0, 0);

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
	for (int ax = 0; ax < 3; ax++) {
		float centered = (float)raw[ax] - MAG_BIAS[sensor_idx][ax];
		cal[ax] = centered * MAG_CAL_SCALE[sensor_idx][ax];
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
static constexpr float GYRO_SCALE  = 8.75e-3f * (Math::PI / 180.0f);
static constexpr float MAG_SCALE_XY = 0.150e-6f;
static constexpr float MAG_SCALE_Z  = 0.242e-6f;

/* LSM6DSV reports specific force (reaction to gravity). The air-bearing observer
 * compares a *gravity-direction* unit vector to g_ref=(0,0,1); negate so rest
 * reading matches identity attitude (avoids huge accel innovation -> bogus β̂). */
#ifndef AIR_BEARING_ACCEL_AS_GRAVITY_NEGATE
#define AIR_BEARING_ACCEL_AS_GRAVITY_NEGATE 1
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

		if (verbose || telemetry) {
			printk("--- cycle %d  (loop dt %lld ms) ---\n", cycle, loop_dt_ms);
		}

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
		//k_msleep(10);

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

		/* Convert raw readings to physical units & average */
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

		/* I2C frame glitches occasionally pass a successful read but with corrupt bytes
		 * (near-int16-max LSBs -> multi-rad/s spikes). Reject per-sensor outliers
		 * before averaging so one bad frame can't poison the fused gyro. */
		constexpr float GYRO_SANE_MAX = 5.0f;  /* rad/s; ~286 deg/s — far beyond bench use */
		auto gyro_sane = [](const int16_t g[3]) -> bool {
			for (int ax = 0; ax < 3; ax++) {
				float v = (float)g[ax] * GYRO_SCALE;
				if (!(v >= -GYRO_SANE_MAX && v <= GYRO_SANE_MAX)) return false;
			}
			return true;
		};

		float gyro_avg[3] = {0.0f, 0.0f, 0.0f};
		int gyro_count = 0;
		for (int i = 0; i < 2; i++) {
			if (imu_ok[i] && gyro_sane(imu_gyro[i])) {
				for (int ax = 0; ax < 3; ax++)
					gyro_avg[ax] += (float)imu_gyro[i][ax] * GYRO_SCALE;
				gyro_count++;
			}
		}
		for (int i = 0; i < 2; i++) {
			if (gyr_ok[i] && gyro_sane(gyr_raw[i])) {
				for (int ax = 0; ax < 3; ax++)
					gyro_avg[ax] += (float)gyr_raw[i][ax] * GYRO_SCALE;
				gyro_count++;
			}
		}
		if (gyro_count > 0) {
			for (int ax = 0; ax < 3; ax++)
				gyro_avg[ax] /= (float)gyro_count;
		}

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
		#if AIR_BEARING_ACCEL_AS_GRAVITY_NEGATE
			sd.accelerometer = Math::Vec<3>{-accel_avg[0], -accel_avg[1], -accel_avg[2]};
		#else
			sd.accelerometer = Math::Vec<3>{accel_avg[0], accel_avg[1], accel_avg[2]};
		#endif
		sd.gyro = Math::Vec<3>{gyro_avg[0], gyro_avg[1], gyro_avg[2]};

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
		/* Motor command packing follows bldcHallL6234 contract:
		 * CLS_COMMAND + OP_SET_WHEEL_RPM, one frame per motor (p2=1..4, p3..p4=int16 rpm).
		 * Controller output currently exposes wheel torque only, so keep rpm commands zeroed
		 * until wheel-speed command path is promoted through ADCSCore.
		 */
		float wheel_rpm_cmd[4] = {0.0f, 0.0f, 0.0f, 0.0f};
		send_wheel_rpm_commands_motor(wheel_rpm_cmd);
		/* Command the Solar board at the same cadence as ADCS compute. */
		send_mtq_dipole_command_solar(out.mtq_dipole(0), out.mtq_dipole(1), out.mtq_dipole(2));
		k_mutex_lock(&g_can_snapshot_mutex, K_FOREVER);
		g_can_snapshot.valid = true;
		g_can_snapshot.attitude_q[0] = out.attitude_est(0);
		g_can_snapshot.attitude_q[1] = out.attitude_est(1);
		g_can_snapshot.attitude_q[2] = out.attitude_est(2);
		g_can_snapshot.attitude_q[3] = out.attitude_est(3);
		g_can_snapshot.wheel_rpm[0] = wheel_rpm_cmd[0];
		g_can_snapshot.wheel_rpm[1] = wheel_rpm_cmd[1];
		g_can_snapshot.wheel_rpm[2] = wheel_rpm_cmd[2];
		g_can_snapshot.wheel_rpm[3] = wheel_rpm_cmd[3];
		g_can_snapshot.mtq_dipole[0] = out.mtq_dipole(0);
		g_can_snapshot.mtq_dipole[1] = out.mtq_dipole(1);
		g_can_snapshot.mtq_dipole[2] = out.mtq_dipole(2);
		k_mutex_unlock(&g_can_snapshot_mutex);

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
	k_mutex_init(&g_can_snapshot_mutex);
	memset(&g_can_snapshot, 0, sizeof(g_can_snapshot));

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

	/* --- CAN init (common board protocol pattern) --- */
	if (!device_is_ready(gpioa)) {
		printk("GPIOA not ready (CAN transceiver pins)\n");
		return 0;
	}
	tcan3403_wakeup();
	can_setup();

	k_thread_create(&adcs_loop_thread, adcs_loop_stack,
			K_THREAD_STACK_SIZEOF(adcs_loop_stack),
			adcs_loop, NULL, NULL, NULL,
			ADCS_LOOP_PRIORITY, 0, K_NO_WAIT);

	printk("========== ADCS loop thread started ==========\n\n");
	LOG_INF("Running — heartbeat + ADCS telemetry scaffold active");

	int64_t last_hb = k_uptime_get();
	while (1) {
		const int64_t now = k_uptime_get();
		if ((now - last_hb) >= HEARTBEAT_INTERVAL_MS) {
			send_heartbeat();
			last_hb = now;
		}
		k_msleep(100);
	}
}
