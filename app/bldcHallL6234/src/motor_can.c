#include "motor_can.h"
#include "can_wheel.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/printk.h>

#include <string.h>

#define CAN_RX_STACK_SIZE 2048
#define CAN_TX_STACK_SIZE 1024
#define CAN_RX_THREAD_PRIO 2   /* above motor_ctrl (1)? no: motor_ctrl must
                                * preempt CAN. Keep RX just below it. */
#define CAN_TX_THREAD_PRIO 5   /* same tier as telemetry                    */
#define CAN_RX_MSGQ_DEPTH  4

#define WHEEL_TX_PERIOD_MS 50   /* 20 Hz measured-RPM telemetry              */

static const struct device *const can_dev = DEVICE_DT_GET(DT_NODELABEL(fdcan1));

CAN_MSGQ_DEFINE(wheel_cmd_q, CAN_RX_MSGQ_DEPTH);

static K_THREAD_STACK_DEFINE(can_rx_stack, CAN_RX_STACK_SIZE);
static K_THREAD_STACK_DEFINE(can_tx_stack, CAN_TX_STACK_SIZE);
static struct k_thread can_rx_thread_data;
static struct k_thread can_tx_thread_data;

static atomic_t rx_count = ATOMIC_INIT(0);
static atomic_t tx_count = ATOMIC_INIT(0);
static atomic_t last_rx_uptime_ms = ATOMIC_INIT(0);
static bool     ever_received = false;

static struct motor_can_cfg s_cfg;

static void can_rx_thread_entry(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	struct can_frame rx;

	while (1) {
		if (k_msgq_get(&wheel_cmd_q, &rx, K_FOREVER) != 0) {
			continue;
		}
		if (rx.dlc != 8) {
			continue;
		}

		struct wheel_rpm_frame f;
		memcpy(&f, rx.data, sizeof(f));

		float rpm[WHEEL_COUNT];
		wheel_rpm_unpack(&f, rpm);

		if (s_cfg.apply_setpoint != NULL) {
			for (unsigned i = 0; i < WHEEL_COUNT; i++) {
				s_cfg.apply_setpoint(i, rpm[i]);
			}
		}

		atomic_set(&last_rx_uptime_ms, (atomic_val_t)k_uptime_get());
		ever_received = true;
		atomic_inc(&rx_count);
	}
}

static void can_tx_thread_entry(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	int64_t next = k_uptime_get();

	while (1) {
		next += WHEEL_TX_PERIOD_MS;
		int64_t sleep_ms = next - k_uptime_get();
		if (sleep_ms < 1) {
			next = k_uptime_get();
			sleep_ms = 1;
		}
		k_sleep(K_MSEC(sleep_ms));

		if (!device_is_ready(can_dev)) {
			continue;
		}
		if (s_cfg.measure_rpm == NULL) {
			continue;
		}

		float rpm[WHEEL_COUNT];
		for (unsigned i = 0; i < WHEEL_COUNT; i++) {
			rpm[i] = s_cfg.measure_rpm(i);
		}

		struct wheel_rpm_frame f;
		wheel_rpm_pack(rpm, &f);

		struct can_frame tx = {0};
		tx.id    = WHEEL_TLM_ID;
		tx.dlc   = 8;
		tx.flags = CAN_FRAME_IDE;
		memcpy(tx.data, &f, sizeof(f));

		if (can_send(can_dev, &tx, K_NO_WAIT, NULL, NULL) == 0) {
			atomic_inc(&tx_count);
		}
	}
}

int motor_can_init(const struct motor_can_cfg *cfg)
{
	if (cfg == NULL || cfg->apply_setpoint == NULL || cfg->measure_rpm == NULL) {
		return -EINVAL;
	}
	s_cfg = *cfg;

	if (!device_is_ready(can_dev)) {
		printk("[CAN] fdcan1 not ready\n");
		return -ENODEV;
	}

	int ret = can_set_bitrate(can_dev, 500000);
	if (ret != 0) {
		printk("[CAN] set_bitrate err %d\n", ret);
		return ret;
	}

	ret = can_set_mode(can_dev, CAN_MODE_NORMAL);
	if (ret != 0) {
		printk("[CAN] set_mode err %d\n", ret);
		return ret;
	}

	ret = can_start(can_dev);
	if (ret != 0) {
		printk("[CAN] start err %d\n", ret);
		return ret;
	}

	const struct can_filter filt = {
		.id    = WHEEL_CMD_ID,
		.mask  = WHEEL_ID_MASK_29,
		.flags = CAN_FILTER_IDE,
	};

	ret = can_add_rx_filter_msgq(can_dev, &wheel_cmd_q, &filt);
	if (ret < 0) {
		printk("[CAN] add_rx_filter err %d\n", ret);
		return ret;
	}

	k_thread_create(&can_rx_thread_data, can_rx_stack,
			K_THREAD_STACK_SIZEOF(can_rx_stack),
			can_rx_thread_entry, NULL, NULL, NULL,
			CAN_RX_THREAD_PRIO, 0, K_NO_WAIT);
	k_thread_name_set(&can_rx_thread_data, "motor_can_rx");

	k_thread_create(&can_tx_thread_data, can_tx_stack,
			K_THREAD_STACK_SIZEOF(can_tx_stack),
			can_tx_thread_entry, NULL, NULL, NULL,
			CAN_TX_THREAD_PRIO, 0, K_NO_WAIT);
	k_thread_name_set(&can_tx_thread_data, "motor_can_tx");

	printk("[CAN] fdcan1 up @ 500 kbit/s\n");
	printk("[CAN]   rx filter WHEEL_CMD id=0x%08X mask=0x%08X\n",
	       (unsigned)filt.id, (unsigned)filt.mask);
	printk("[CAN]   tx WHEEL_TLM id=0x%08X every %u ms\n",
	       (unsigned)WHEEL_TLM_ID, (unsigned)WHEEL_TX_PERIOD_MS);
	return 0;
}

int64_t motor_can_rx_age_ms(void)
{
	if (!ever_received) {
		return INT64_MAX;
	}
	int64_t ts = (int64_t)atomic_get(&last_rx_uptime_ms);
	return k_uptime_get() - ts;
}

uint32_t motor_can_rx_count(void) { return (uint32_t)atomic_get(&rx_count); }
uint32_t motor_can_tx_count(void) { return (uint32_t)atomic_get(&tx_count); }
