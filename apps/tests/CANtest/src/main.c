// CAN LOOP BACK TEST-----------------------------------------------------------------------------
// #include <zephyr/kernel.h>
// #include <zephyr/device.h>
// #include <zephyr/drivers/gpio.h>
// #include <zephyr/drivers/can.h>

// /* LED aliases */
// #define LED0_NODE DT_ALIAS(led0)
// #define LED1_NODE DT_ALIAS(led1)

// /* CAN device */
// #define CAN_NODE DT_NODELABEL(fdcan1)

// /* Startup delay */
// #define STARTUP_DELAY_MS 2000

// static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
// static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);

// static const struct device *can_dev = DEVICE_DT_GET(CAN_NODE);

// int main(void)
// {
// 	int ret;

// 	printk("UT-CORE Booting...\n");

// 	/* ========================= */
// 	/* LED STARTUP TEST SEQUENCE */
// 	/* ========================= */

// 	if (!gpio_is_ready_dt(&led0) || !gpio_is_ready_dt(&led1)) {
// 		printk("LED GPIO not ready\n");
// 		return 0;
// 	}

// 	gpio_pin_configure_dt(&led0, GPIO_OUTPUT_INACTIVE);
// 	gpio_pin_configure_dt(&led1, GPIO_OUTPUT_INACTIVE);

// 	/* Turn ALL LEDs ON */
// 	gpio_pin_set_dt(&led0, 1);
// 	gpio_pin_set_dt(&led1, 1);

// 	printk("Startup LED test ON\n");

// 	k_msleep(STARTUP_DELAY_MS);

// 	/* Turn ALL LEDs OFF */
// 	gpio_pin_set_dt(&led0, 0);
// 	gpio_pin_set_dt(&led1, 0);

// 	printk("Startup LED test OFF\n");

// 	/* ========================= */
// 	/* CAN INITIALIZATION        */
// 	/* ========================= */

// 	printk("UT-CORE CAN TX Test\n");

// 	if (!device_is_ready(can_dev)) {
// 		printk("CAN device not ready\n");
// 		return 0;
// 	}

//     /* 1. Set CAN Bitrate */
//     ret = can_set_bitrate(can_dev, 500000);
//     if (ret) {
//         printk("Failed to set CAN bitrate (%d)\n", ret);
//         return 0;
//     }

//     /* 2. ENABLE LOOPBACK MODE (Critical for single-board testing) */
//     ret = can_set_mode(can_dev, CAN_MODE_LOOPBACK);
//     if (ret) {
//         printk("Failed to set loopback mode (%d)\n", ret);
//         return 0;
//     }

//     /* 3. Start CAN controller */
//     ret = can_start(can_dev);
//     if (ret) {
//         printk("Failed to start CAN (%d)\n", ret);
//         return 0;
//     }
    
//     printk("CAN started successfully in LOOPBACK mode\n");

//     /* 4. Fix the Frame Flags */
//     struct can_frame frame = {
//         .id = 0x123,
//         /* ERROR FIX: CAN_FRAME_IDE is for Extended (29-bit) IDs. 
//            For Standard (11-bit) IDs like 0x123, use 0. */
//         .flags = 0, 
//         .dlc = 8,
//         .data = { 0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE, 0x12, 0x34 }
//     };

// 	/* ========================= */
// 	/* MAIN LOOP                 */
// 	/* ========================= */

//     while (1) {
//         printk("Sending CAN frame...\n");

//         /* Try to send (Non-blocking) */
//         ret = can_send(can_dev, &frame, K_NO_WAIT, NULL, NULL);

//         if (ret == 0) {
//             printk("CAN frame sent (Success)\n");

//             /* SUCCESS INDICATION: Blink LED0 */
//             /* 1. Turn LED0 ON (Green?) */
//             gpio_pin_set_dt(&led0, 1);
//             /* 2. Turn LED1 OFF (Red?) */
//             gpio_pin_set_dt(&led1, 0);

//             /* Keep it ON briefly so you can see the flash */
//             k_msleep(100); 

//             /* 3. Turn LED0 OFF */
//             gpio_pin_set_dt(&led0, 0);

//             /* Wait the rest of the second before next send */
//             k_msleep(900);

//         } else {
//             printk("CAN send failed (%d)\n", ret);

//             /* FAILURE INDICATION: Solid LED1 */
//             /* 1. Turn LED0 OFF */
//             gpio_pin_set_dt(&led0, 0);
//             /* 2. Turn LED1 ON */
//             gpio_pin_set_dt(&led1, 1);

//             /* Wait 1 second before retrying */
//             k_msleep(1000);
//         }
//     }
// }


//CAN DUAL BOARD TESTING-----------------------------------------------------------------------------------------
/* UT-CORE main.c (Node 0x1) - WITH TCAN340 WAKEUP */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/can.h>
#include <zephyr/logging/log.h>
#include "C:\Users\notbr\Documents\all_coding\ut-core\common\can_proto.h"

LOG_MODULE_REGISTER(ut_core, LOG_LEVEL_INF);

/* === CONFIG === */
#define NODE_ID     0x1 
#define DST_ME      NODE_ID
#define PRIO_LOW    3 
#define CLS_CORE    0 

/* Hardware Definitions */
#define LED0_NODE   DT_ALIAS(led0)
#define CAN_NODE    DT_NODELABEL(fdcan1)

/* TCAN340 Control Pins (Port A) */
/* We access GPIOA directly since these might not be in your DT aliases */
#define TCAN_PORT   DT_NODELABEL(gpioa)
#define PIN_SILENT  9   /* PA9 */
#define PIN_SHDN    10  /* PA10 */

static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct device *can_dev = DEVICE_DT_GET(CAN_NODE);
static const struct device *gpioa   = DEVICE_DT_GET(TCAN_PORT);

CAN_MSGQ_DEFINE(rxq, 16);

static void send_simple(uint8_t dst, uint8_t op, uint8_t val)
{
    struct can_frame f = {0};
    f.id = CAN_ID(PRIO_LOW, dst, CLS_CORE); 
    can_fill_payload(&f, NODE_ID, op, val, 0, 0, 0, 0, 0);
    can_send(can_dev, &f, K_NO_WAIT, NULL, NULL);
}

static void tcan330_wakeup(void)
{
    if (!device_is_ready(gpioa)) {
        LOG_ERR("GPIOA not ready!");
        return;
    }

    /* 1. Configure SHDN (PA10) as Output LOW (Normal Mode) */
    //gpio_pin_configure(gpioa, PIN_SHDN, GPIO_OUTPUT_INACTIVE);

    /* 2. Configure SILENT (PA9) as Output LOW (Normal Mode) */
    gpio_pin_configure(gpioa, PIN_SILENT, GPIO_OUTPUT_INACTIVE);

    /* 3. Wait for transceiver wake-up time (Datasheet says ~50us, we do 1ms to be safe) */
    k_msleep(1);

    LOG_INF("TCAN330 Transceiver Woken Up (SHDN=0, S=0)");
}

int main(void)
{
    LOG_INF("UT-CORE (Node 0x1) Booting...");

    /* === 1. WAKE UP THE TRANSCEIVER === */
    tcan330_wakeup();

    /* 2. Setup LEDs */
    gpio_pin_configure_dt(&led0, GPIO_OUTPUT_INACTIVE);

    /* 3. Setup CAN */
    if (!device_is_ready(can_dev)) {
        LOG_ERR("CAN device not ready");
        return 0;
    }

    can_set_bitrate(can_dev, 500000);
    can_set_mode(can_dev, CAN_MODE_NORMAL);
    can_start(can_dev);

    /* Filters */
    const struct can_filter to_me = { .id = CAN_DST(DST_ME), .mask = CAN_DST_MASK, .flags = 0 };
    const struct can_filter bcast = { .id = CAN_DST(CAN_BROADCAST), .mask = CAN_DST_MASK, .flags = 0 };
    
    can_add_rx_filter_msgq(can_dev, &rxq, &to_me);
    can_add_rx_filter_msgq(can_dev, &rxq, &bcast);

    LOG_INF("CAN Live. Sending Heartbeats...");

    int64_t last_hb = 0;

    while (1) {
        int64_t now = k_uptime_get();
        if (now - last_hb >= 1000) {
            last_hb = now;
            send_simple(CAN_BROADCAST, OP_HEARTBEAT, 0);
            LOG_INF("TX: Heartbeat");
        }

        struct can_frame rx;
        if (k_msgq_get(&rxq, &rx, K_MSEC(100)) == 0) {
            uint8_t src = rx.data[0];
            uint8_t op  = rx.data[1];
            uint8_t val = rx.data[2]; 

            if (op == OP_SET_LED) {
                LOG_INF("RX: CMD SET LED %d from 0x%x", val, src);
                gpio_pin_set_dt(&led0, val);
            }
        }
    }
}

