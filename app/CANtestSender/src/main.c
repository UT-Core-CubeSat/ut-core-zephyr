#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/can.h>

#define LED_TX_NODE   DT_ALIAS(led0)
#define CAN_NODE      DT_NODELABEL(fdcan1)
#define CAN_EN_NODE   DT_ALIAS(canen)
#define CAN_STB_NODE  DT_ALIAS(canstb)

static const struct gpio_dt_spec led_tx  = GPIO_DT_SPEC_GET(LED_TX_NODE, gpios);
static const struct gpio_dt_spec can_vio = GPIO_DT_SPEC_GET(CAN_EN_NODE, gpios);
static const struct gpio_dt_spec can_stb = GPIO_DT_SPEC_GET(CAN_STB_NODE, gpios);

static const struct device *const can_dev = DEVICE_DT_GET(CAN_NODE);

static void pulse_tx_led(int ms)
{
    gpio_pin_set_dt(&led_tx, 1);
    k_msleep(ms);
    gpio_pin_set_dt(&led_tx, 0);
}

static const char *state_str(enum can_state s)
{
    switch (s) {
    case CAN_STATE_ERROR_ACTIVE:  return "ERROR_ACTIVE";
    case CAN_STATE_ERROR_PASSIVE: return "ERROR_PASSIVE";
    case CAN_STATE_BUS_OFF:       return "BUS_OFF";
    case CAN_STATE_STOPPED:       return "STOPPED";
    default: return "UNKNOWN";
    }
}

int main(void)
{
    printk("\n=== CAN TX blink test ===\n");

    gpio_pin_configure_dt(&led_tx, GPIO_OUTPUT_INACTIVE);

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

    struct can_frame frame = {
        .id = 0x123,
        .flags = 0,
        .dlc = 8,
        .data = { 1,2,3,4,5,6,7,0 }
    };

    uint8_t ctr = 0;

    while (1) {
        frame.data[7] = ctr++;

        ret = can_send(can_dev, &frame, K_MSEC(200), NULL, NULL);

        enum can_state st;
        struct can_bus_err_cnt ec;
        (void)can_get_state(can_dev, &st, &ec);

        printk("TX: ret=%d  state=%s  tx_err=%u  rx_err=%u\n",
               ret, state_str(st), ec.tx_err_cnt, ec.rx_err_cnt);

        if (ret == 0) {
            pulse_tx_led(30);
        } else {
            /* error: double blink */
            pulse_tx_led(20);
            k_msleep(50);
            pulse_tx_led(20);
        }

        k_msleep(300);
    }
}
