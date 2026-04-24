#include "adcs_can.hpp"

extern "C" {
#include "can_wheel.h"
}

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/printk.h>

#include <cmath>
#include <string.h>

namespace {

constexpr int  CAN_RX_STACK_SIZE = 2048;
constexpr int  CAN_RX_THREAD_PRIO = 1;
constexpr int  CAN_RX_MSGQ_DEPTH = 4;
constexpr float RPM_TO_RAD_PER_S = (2.0f * 3.14159265358979323846f) / 60.0f;

const struct device *const g_can_dev = DEVICE_DT_GET(DT_NODELABEL(fdcan1));

CAN_MSGQ_DEFINE(wheel_tlm_q, CAN_RX_MSGQ_DEPTH);

K_THREAD_STACK_DEFINE(can_rx_stack, CAN_RX_STACK_SIZE);
struct k_thread can_rx_thread_data;

/*
 * RX state published to the ADCS loop. Updated from can_rx_thread,
 * read from the adcs compute thread. Per-element scalar writes; the
 * atomic age stamp is the freshness handshake.
 */
struct k_spinlock rx_lock;
float  latest_omega_rad_s[4] = {0.f, 0.f, 0.f, 0.f};
atomic_t last_rx_uptime_ms = ATOMIC_INIT(0);
bool     ever_received = false;

atomic_t tx_count = ATOMIC_INIT(0);
atomic_t rx_count = ATOMIC_INIT(0);

void can_rx_thread_entry(void *, void *, void *)
{
	struct can_frame rx;

	while (1) {
		if (k_msgq_get(&wheel_tlm_q, &rx, K_FOREVER) != 0) {
			continue;
		}

		if (rx.dlc != 8) {
			continue; /* malformed; drop */
		}

		struct wheel_rpm_frame f;
		memcpy(&f, rx.data, sizeof(f));

		float rpm[4];
		wheel_rpm_unpack(&f, rpm);

		k_spinlock_key_t key = k_spin_lock(&rx_lock);
		for (int i = 0; i < 4; i++) {
			latest_omega_rad_s[i] = rpm[i] * RPM_TO_RAD_PER_S;
		}
		ever_received = true;
		k_spin_unlock(&rx_lock, key);

		atomic_set(&last_rx_uptime_ms, (atomic_val_t)k_uptime_get());
		atomic_inc(&rx_count);
	}
}

} // namespace

int adcs_can_init(void)
{
	if (!device_is_ready(g_can_dev)) {
		printk("[CAN] fdcan1 not ready\n");
		return -ENODEV;
	}

	int ret = can_set_bitrate(g_can_dev, 500000);
	if (ret != 0) {
		printk("[CAN] set_bitrate err %d\n", ret);
		return ret;
	}

	ret = can_set_mode(g_can_dev, CAN_MODE_NORMAL);
	if (ret != 0) {
		printk("[CAN] set_mode err %d\n", ret);
		return ret;
	}

	ret = can_start(g_can_dev);
	if (ret != 0) {
		printk("[CAN] start err %d\n", ret);
		return ret;
	}

	/*
	 * Match only the motor board's wheel-telemetry frame: exact 29-bit ID
	 * including SRC=MOTOR_ID, DST=ADCS_ID, CLS=CLS_WHEEL_TLM.
	 */
	const struct can_filter filt = {
		.id    = WHEEL_TLM_ID,
		.mask  = WHEEL_ID_MASK_29,
		.flags = CAN_FILTER_IDE,
	};

	ret = can_add_rx_filter_msgq(g_can_dev, &wheel_tlm_q, &filt);
	if (ret < 0) {
		printk("[CAN] add_rx_filter err %d\n", ret);
		return ret;
	}

	k_thread_create(&can_rx_thread_data, can_rx_stack,
			K_THREAD_STACK_SIZEOF(can_rx_stack),
			can_rx_thread_entry, NULL, NULL, NULL,
			CAN_RX_THREAD_PRIO, 0, K_NO_WAIT);
	k_thread_name_set(&can_rx_thread_data, "adcs_can_rx");

	printk("[CAN] fdcan1 up @ 500 kbit/s, wheel_tlm filter id=0x%08X mask=0x%08X\n",
	       (unsigned)filt.id, (unsigned)filt.mask);
	return 0;
}

int adcs_can_send_wheel_rpm(const float rpm_ref[4])
{
	if (!device_is_ready(g_can_dev)) {
		return -ENODEV;
	}

	struct wheel_rpm_frame f;
	wheel_rpm_pack(rpm_ref, &f);

	struct can_frame tx = {};
	tx.id    = WHEEL_CMD_ID;
	tx.dlc   = 8;
	tx.flags = CAN_FRAME_IDE;
	memcpy(tx.data, &f, sizeof(f));

	int ret = can_send(g_can_dev, &tx, K_NO_WAIT, NULL, NULL);
	if (ret == 0) {
		atomic_inc(&tx_count);
	}
	return ret;
}

bool adcs_can_get_wheel_speeds(Math::Vec<4>& out, int64_t max_age_ms)
{
	int64_t rx_ts = (int64_t)atomic_get(&last_rx_uptime_ms);
	int64_t age = k_uptime_get() - rx_ts;

	k_spinlock_key_t key = k_spin_lock(&rx_lock);
	bool have_sample = ever_received;
	for (int i = 0; i < 4; i++) {
		out(i) = static_cast<Math::Real>(
			have_sample ? latest_omega_rad_s[i] : 0.f);
	}
	k_spin_unlock(&rx_lock, key);

	return have_sample && (age <= max_age_ms);
}

int64_t adcs_can_last_rx_age_ms(void)
{
	int64_t rx_ts = (int64_t)atomic_get(&last_rx_uptime_ms);
	return k_uptime_get() - rx_ts;
}

uint32_t adcs_can_tx_count(void) { return (uint32_t)atomic_get(&tx_count); }
uint32_t adcs_can_rx_count(void) { return (uint32_t)atomic_get(&rx_count); }
