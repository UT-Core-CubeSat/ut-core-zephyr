#include <errno.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/printk.h>

#if TEST_USE_MEMCPY_OVERRIDE
/*
 * Some MCAN/FDCAN message RAM regions are sensitive to access width.
 * Keep this override for test parity with known CAN diagnostics in this repo.
 */
void *memcpy(void *dst, const void *src, size_t n)
{
	uintptr_t d = (uintptr_t)dst;
	uintptr_t s = (uintptr_t)src;

	if (((d | s | n) & 0x3u) == 0u) {
		uint32_t *wd = (uint32_t *)dst;
		const uint32_t *ws = (const uint32_t *)src;

		for (size_t i = 0; i < (n >> 2); i++) {
			wd[i] = ws[i];
		}
		return dst;
	}

	uint8_t *bd = (uint8_t *)dst;
	const uint8_t *bs = (const uint8_t *)src;

	for (size_t i = 0; i < n; i++) {
		bd[i] = bs[i];
	}

	return dst;
}
#endif

#define CAN_NODE DT_NODELABEL(fdcan1)
#define CAN_STB_NODE DT_ALIAS(canstb)
#define LED_RX_NODE DT_ALIAS(led1)

/* Isolation toggles for fault triage */
#define TEST_ENABLE_TX_HEARTBEAT 1
#define TEST_ENABLE_EXT_FILTER   1
#define TEST_ENABLE_STD_FILTER   1
#define RX_MSGQ_LEN              16
#define TEST_USE_MEMCPY_OVERRIDE 0
#define TEST_USE_LOOPBACK_MODE   0

K_MSGQ_DEFINE(rx_msgq, sizeof(struct can_frame), RX_MSGQ_LEN, 4);
static atomic_t rx_total;
static atomic_t rx_q_drop;
static atomic_t rx_ext;
static atomic_t rx_std;
static atomic_t rx_fd;
static atomic_t rx_brs;
static atomic_t rx_esi;
static atomic_t rx_rtr;

#if !DT_NODE_HAS_STATUS(CAN_NODE, okay)
#error "DT node 'fdcan1' not okay"
#endif

static const struct device *const can_dev = DEVICE_DT_GET(CAN_NODE);

#if DT_NODE_HAS_STATUS(LED_RX_NODE, okay)
static const struct gpio_dt_spec led_rx = GPIO_DT_SPEC_GET(LED_RX_NODE, gpios);
#endif

#if DT_NODE_HAS_STATUS(CAN_STB_NODE, okay)
static const struct gpio_dt_spec can_stb = GPIO_DT_SPEC_GET(CAN_STB_NODE, gpios);
#endif

static const char *state_str(enum can_state s)
{
	switch (s) {
	case CAN_STATE_ERROR_ACTIVE:
		return "ERROR_ACTIVE";
	case CAN_STATE_ERROR_WARNING:
		return "ERROR_WARNING";
	case CAN_STATE_ERROR_PASSIVE:
		return "ERROR_PASSIVE";
	case CAN_STATE_BUS_OFF:
		return "BUS_OFF";
	case CAN_STATE_STOPPED:
		return "STOPPED";
	default:
		return "UNKNOWN";
	}
}

static void pulse_led(void)
{
#if DT_NODE_HAS_STATUS(LED_RX_NODE, okay)
	(void)gpio_pin_set_dt(&led_rx, 1);
	k_busy_wait(300);
	(void)gpio_pin_set_dt(&led_rx, 0);
#endif
}

static void rx_cb(const struct device *dev, struct can_frame *frame, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);

	/* ISR path must stay short; drain/print in thread context. */
	atomic_inc(&rx_total);
	if ((frame->flags & CAN_FRAME_IDE) != 0U) {
		atomic_inc(&rx_ext);
	} else {
		atomic_inc(&rx_std);
	}
	if ((frame->flags & CAN_FRAME_FDF) != 0U) {
		atomic_inc(&rx_fd);
	}
	if ((frame->flags & CAN_FRAME_BRS) != 0U) {
		atomic_inc(&rx_brs);
	}
	if ((frame->flags & CAN_FRAME_ESI) != 0U) {
		atomic_inc(&rx_esi);
	}
	if ((frame->flags & CAN_FRAME_RTR) != 0U) {
		atomic_inc(&rx_rtr);
	}

	if (k_msgq_put(&rx_msgq, frame, K_NO_WAIT) != 0) {
		atomic_inc(&rx_q_drop);
	}
}

static void print_can_state(const char *tag)
{
	enum can_state st = CAN_STATE_STOPPED;
	struct can_bus_err_cnt ec = {0};
	int ret = can_get_state(can_dev, &st, &ec);

	if (ret != 0) {
		printk("%s can_get_state ret=%d\n", tag, ret);
		return;
	}

	printk("%s state=%s tx_err=%u rx_err=%u\n", tag, state_str(st), ec.tx_err_cnt,
	       ec.rx_err_cnt);
}

int main(void)
{
	printk("\n=== motorBoardCanTest ===\n");
	printk("cfg: tx=%d ext_filter=%d std_filter=%d loopback=%d\n",
	       TEST_ENABLE_TX_HEARTBEAT, TEST_ENABLE_EXT_FILTER, TEST_ENABLE_STD_FILTER,
	       TEST_USE_LOOPBACK_MODE);

	if (!device_is_ready(can_dev)) {
		printk("ERR: can device not ready\n");
		return 0;
	}

#if DT_NODE_HAS_STATUS(LED_RX_NODE, okay)
	if (device_is_ready(led_rx.port)) {
		(void)gpio_pin_configure_dt(&led_rx, GPIO_OUTPUT_INACTIVE);
	}
#endif

#if DT_NODE_HAS_STATUS(CAN_STB_NODE, okay)
	if (!device_is_ready(can_stb.port)) {
		printk("ERR: canstb gpio device not ready\n");
		return 0;
	}
	(void)gpio_pin_configure_dt(&can_stb, GPIO_OUTPUT_INACTIVE);
	(void)gpio_pin_set_dt(&can_stb, 0);
	k_msleep(5);
	printk("canstb forced low\n");
#else
	printk("NOTE: no canstb alias; skipping transceiver standby control\n");
#endif

	int ret = can_set_bitrate(can_dev, 500000);
	printk("can_set_bitrate=%d\n", ret);
	if (ret != 0) {
		return 0;
	}

	ret = can_set_mode(can_dev,
#if TEST_USE_LOOPBACK_MODE
			   CAN_MODE_LOOPBACK
#else
			   CAN_MODE_NORMAL
#endif
	);
	printk("can_set_mode=%d\n", ret);
	if (ret != 0) {
		return 0;
	}

	ret = can_start(can_dev);
	printk("can_start=%d\n", ret);
	if (ret != 0) {
		return 0;
	}

	int fid_ext = -ENOTSUP;
	int fid_std = -ENOTSUP;

#if TEST_ENABLE_EXT_FILTER
	struct can_filter filt_ext = {
		.id = 0,
		.mask = 0,
		.flags = CAN_FILTER_IDE,
	};
	fid_ext = can_add_rx_filter(can_dev, rx_cb, NULL, &filt_ext);
#endif
	printk("rx_filter_ext=%d\n", fid_ext);

#if TEST_ENABLE_STD_FILTER
	struct can_filter filt_std = {
		.id = 0,
		.mask = 0,
		.flags = 0,
	};
	fid_std = can_add_rx_filter(can_dev, rx_cb, NULL, &filt_std);
#endif
	printk("rx_filter_std=%d\n", fid_std);

	uint8_t seq = 0;
	uint32_t prev_total = 0U, prev_drop = 0U, prev_ext = 0U, prev_std = 0U;
	uint32_t prev_fd = 0U, prev_brs = 0U, prev_esi = 0U, prev_rtr = 0U;
	while (1) {
		struct can_frame rx;
		while (k_msgq_get(&rx_msgq, &rx, K_NO_WAIT) == 0) {
			pulse_led();
			printk("RX id=0x%08x flags=0x%x dlc=%u d0=0x%02x d1=0x%02x d2=0x%02x d3=0x%02x\n",
			       rx.id, rx.flags, rx.dlc, rx.data[0], rx.data[1], rx.data[2], rx.data[3]);
		}

		int tx_ret = 0;

#if TEST_ENABLE_TX_HEARTBEAT
		struct can_frame tx = {
			.id = 0x123,
			.flags = 0,
			.dlc = 8,
			.data = {0xAA, 0x55, 0x01, 0x02, 0x03, 0x04, 0x05, seq++},
		};
		tx_ret = can_send(can_dev, &tx, K_MSEC(20), NULL, NULL);
		printk("TX ret=%d seq=%u\n", tx_ret, tx.data[7]);
#else
		printk("TX disabled\n");
#endif
		print_can_state("CAN");
		uint32_t cur_total = (uint32_t)atomic_get(&rx_total);
		uint32_t cur_drop = (uint32_t)atomic_get(&rx_q_drop);
		uint32_t cur_ext = (uint32_t)atomic_get(&rx_ext);
		uint32_t cur_std = (uint32_t)atomic_get(&rx_std);
		uint32_t cur_fd = (uint32_t)atomic_get(&rx_fd);
		uint32_t cur_brs = (uint32_t)atomic_get(&rx_brs);
		uint32_t cur_esi = (uint32_t)atomic_get(&rx_esi);
		uint32_t cur_rtr = (uint32_t)atomic_get(&rx_rtr);

		printk("RX/s total=%u drop=%u ext=%u std=%u fd=%u brs=%u esi=%u rtr=%u\n",
		       cur_total - prev_total, cur_drop - prev_drop, cur_ext - prev_ext,
		       cur_std - prev_std, cur_fd - prev_fd, cur_brs - prev_brs,
		       cur_esi - prev_esi, cur_rtr - prev_rtr);

		prev_total = cur_total;
		prev_drop = cur_drop;
		prev_ext = cur_ext;
		prev_std = cur_std;
		prev_fd = cur_fd;
		prev_brs = cur_brs;
		prev_esi = cur_esi;
		prev_rtr = cur_rtr;

		k_msleep(1000);
	}
}
