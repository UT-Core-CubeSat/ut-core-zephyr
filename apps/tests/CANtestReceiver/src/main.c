#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/can.h>

#include <stdint.h>
#include <stddef.h>

/*
 * ====== HARD FIX: override memcpy() ======
 * Some MCAN/FDCAN message RAM regions require 32-bit accesses only.
 * If libc memcpy uses byte writes, can_mcan will detect RAM access failure repeatedly.
 *
 * This override:
 *  - Uses 32-bit copies when src/dst/n are 4-byte aligned
 *  - Otherwise falls back to byte copy
 *
 * References: Zephyr can_mcan / fdcan message RAM byte-write corruption issues.
 */
void *memcpy(void *dst, const void *src, size_t n)
{
    uintptr_t d = (uintptr_t)dst;
    uintptr_t s = (uintptr_t)src;

    if (((d | s | n) & 0x3u) == 0u) {
        /* 32-bit aligned: do word copies */
        uint32_t *wd = (uint32_t *)dst;
        const uint32_t *ws = (const uint32_t *)src;

        for (size_t i = 0; i < (n >> 2); i++) {
            wd[i] = ws[i];
        }
        return dst;
    }

    /* Fallback: byte copy */
    uint8_t *bd = (uint8_t *)dst;
    const uint8_t *bs = (const uint8_t *)src;
    for (size_t i = 0; i < n; i++) {
        bd[i] = bs[i];
    }
    return dst;
}

/* ---------- Devicetree nodes ---------- */
#define LED_RX_NODE   DT_ALIAS(led1)
#define CAN_NODE      DT_NODELABEL(fdcan1)
#define CAN_EN_NODE   DT_ALIAS(canen)
#define CAN_STB_NODE  DT_ALIAS(canstb)

#if !DT_NODE_HAS_STATUS(LED_RX_NODE, okay)
#error "DT alias 'led1' not defined"
#endif
#if !DT_NODE_HAS_STATUS(CAN_EN_NODE, okay)
#error "DT alias 'canen' not defined"
#endif
#if !DT_NODE_HAS_STATUS(CAN_STB_NODE, okay)
#error "DT alias 'canstb' not defined"
#endif
#if !DT_NODE_HAS_STATUS(CAN_NODE, okay)
#error "DT node 'fdcan1' not okay"
#endif

static const struct gpio_dt_spec led_rx  = GPIO_DT_SPEC_GET(LED_RX_NODE, gpios);
static const struct gpio_dt_spec can_vio = GPIO_DT_SPEC_GET(CAN_EN_NODE, gpios);
static const struct gpio_dt_spec can_stb = GPIO_DT_SPEC_GET(CAN_STB_NODE, gpios);

static const struct device *const can_dev = DEVICE_DT_GET(CAN_NODE);

static void rx_cb(const struct device *dev, struct can_frame *frame, void *user_data)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(user_data);

    /* quick blink */
    gpio_pin_set_dt(&led_rx, 1);
    k_busy_wait(2000);  /* 2 ms */
    gpio_pin_set_dt(&led_rx, 0);

    /* print minimal info */
    printk("RX: id=0x%X dlc=%d data0=0x%02X data7=0x%02X\n",
           frame->id, frame->dlc, frame->data[0], frame->data[7]);
}

int main(void)
{
    printk("\n=== CAN RX test (with memcpy word-copy override) ===\n");

    if (!device_is_ready(led_rx.port) ||
        !device_is_ready(can_vio.port) ||
        !device_is_ready(can_stb.port) ||
        !device_is_ready(can_dev)) {
        printk("Device not ready\n");
        return 0;
    }

    gpio_pin_configure_dt(&led_rx, GPIO_OUTPUT_INACTIVE);

    /* Transceiver bring-up */
    gpio_pin_configure_dt(&can_vio, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&can_stb, GPIO_OUTPUT_INACTIVE);
    gpio_pin_set_dt(&can_vio, 1);
    k_msleep(5);
    gpio_pin_set_dt(&can_stb, 0);
    k_msleep(5);

    int ret = can_set_mode(can_dev, CAN_MODE_NORMAL);
    printk("can_set_mode=%d\n", ret);

    ret = can_start(can_dev);
    printk("can_start=%d\n", ret);

    /* Accept ALL standard IDs */
    struct can_filter filter = {
        .id = 0,
        .mask = 0,
        .flags = 0,
    };

    int fid = can_add_rx_filter(can_dev, rx_cb, NULL, &filter);
    printk("rx_filter_id=%d\n", fid);

    while (1) {
        k_msleep(1000);
    }
}
