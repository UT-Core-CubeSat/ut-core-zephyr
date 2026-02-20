
//UID read code

// #include <zephyr/kernel.h>
// #include <zephyr/device.h>
// #include <zephyr/drivers/gpio.h>
// #include <zephyr/drivers/can.h>
// #include <stm32u5xx.h>

// int main(void)
// {
//     printk("\nUT-CORE boot OK\n");

//     uint32_t uid0 = *(uint32_t *)(UID_BASE + 0x0);
//     uint32_t uid1 = *(uint32_t *)(UID_BASE + 0x4);
//     uint32_t uid2 = *(uint32_t *)(UID_BASE + 0x8);

//     printk("STM32U5 UID:\n");
//     printk("%08X-%08X-%08X\n", uid0, uid1, uid2);

//     while (1) {
//         k_sleep(K_SECONDS(1));
//     }
// }
//------------------------------------------------------------------------


//MCU Determination code---------------------------------------------------------
// #include <zephyr/kernel.h>
// #include <zephyr/sys/printk.h>
// #include <zephyr/drivers/gpio.h>
// #include <stm32u5xx.h>
// #include <string.h>

// /* ========================= */
// /* EXPECTED UID DEFINITIONS */
// /* ========================= */

// static const uint32_t MCU1_UID[3] = {
//     0x00340016,
//     0x41425007,
//     0x20363651
// };

// static const uint32_t MCU2_UID[3] = {
//     0x0012001B,
//     0x41425007,
//     0x20363651
// };

// /* ========================= */
// /* GPIO ROLE PIN             */
// /* ========================= */

// #define MCU_ROLE_NODE DT_ALIAS(mcuselect)

// #if !DT_NODE_HAS_STATUS(MCU_ROLE_NODE, okay)
// #error "MCU role pin alias not defined"
// #endif

// static const struct gpio_dt_spec mcu_select_pin =
//     GPIO_DT_SPEC_GET(MCU_ROLE_NODE, gpios);

// /* ========================= */
// /* HELPER FUNCTIONS          */
// /* ========================= */

// static bool uid_matches(const uint32_t uid[3], const uint32_t expected[3])
// {
//     return (uid[0] == expected[0] &&
//             uid[1] == expected[1] &&
//             uid[2] == expected[2]);
// }

// /* ========================= */
// /* MAIN                      */
// /* ========================= */

// int main(void)
// {
//     int ret;

//     printk("\n============================\n");
//     printk("UT-CORE Setup Start\n");
//     printk("============================\n");

//     /* Configure PA6 as input */
//     ret = gpio_pin_configure_dt(&mcu_select_pin, GPIO_INPUT);
//     if (ret < 0) {
//         printk("ERROR: Failed to configure MCU role pin\n");
//         return 0;
//     }

//     /* Read PA6 */
//     int pa6_level = gpio_pin_get_dt(&mcu_select_pin);
//     bool pin_says_mcu2 = (pa6_level != 0); /* HIGH = MCU2 */

//     /* Read UID */
//     uint32_t uid[3];
//     uid[0] = *(uint32_t *)(UID_BASE + 0x0);
//     uid[1] = *(uint32_t *)(UID_BASE + 0x4);
//     uid[2] = *(uint32_t *)(UID_BASE + 0x8);

//     printk("Read UID: %08X-%08X-%08X\n",
//            uid[0], uid[1], uid[2]);

//     printk("PA6 state: %s\n", pa6_level ? "HIGH" : "LOW");

//     /* ========================= */
//     /* CONSISTENCY CHECK         */
//     /* ========================= */

//     bool uid_says_mcu1 = uid_matches(uid, MCU1_UID);
//     bool uid_says_mcu2 = uid_matches(uid, MCU2_UID);

//     if (pin_says_mcu2) {
//         printk("ROLE (from pin): MCU2\n");

//         if (!uid_says_mcu2) {
//             printk("WARNING: UID does NOT match expected MCU2 UID!\n");
//         }
//     } else {
//         printk("ROLE (from pin): MCU1\n");

//         if (!uid_says_mcu1) {
//             printk("WARNING: UID does NOT match expected MCU1 UID!\n");
//         }
//     }

//     /* Extra diagnostic: unknown UID */
//     if (!uid_says_mcu1 && !uid_says_mcu2) {
//         printk("WARNING: UID does not match any known MCU!\n");
//     }

//     printk("============================\n");
//     printk("Setup complete trusting PA6 for role\n");
//     printk("============================\n");

//     while (1) {
//         k_sleep(K_SECONDS(1));
//     }
// }
//--------------------------------------------------------------------------------

/*
 * UT-CORE CDH - Threaded Flight Architecture
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <stm32u5xx.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "C:\Users\notbr\Documents\all_coding\ut-core\common\can_proto.h"

LOG_MODULE_REGISTER(cdh, LOG_LEVEL_INF);

/* ================= PRIORITIES ================= */

#define PRIO_MODE       0
#define PRIO_CAN_RX     1
#define PRIO_WATCHDOG   2
#define PRIO_SCHED      3
#define PRIO_SOH        4

#define STACK_SIZE 1024

/* ================= HARDWARE DEFINITIONS ================= */

#define NODE_ID     0x1
#define DST_ME      NODE_ID
#define PRIO_LOW    3
#define CLS_CORE    0

#define CAN_NODE     DT_NODELABEL(fdcan1)
#define I2C_BUS_NODE DT_NODELABEL(i2c1)
#define GPIOA_NODE   DT_NODELABEL(gpioa)
#define GPIOB_NODE   DT_NODELABEL(gpiob)
#define LED0_NODE    DT_ALIAS(led0)
#define LED1_NODE    DT_ALIAS(led1)
#define LED2_NODE    DT_ALIAS(led2)

#define PIN_SILENT  9
#define PIN_SHDN    10
#define PIN_ROLE    6
#define WATCHDOG_PIN 2

#define UID1_WORD0 0x00340016
#define UID1_WORD1 0x41425007
#define UID1_WORD2 0x20363651

#define UID2_WORD0 0x0012001B
#define UID2_WORD1 0x41425007
#define UID2_WORD2 0x20363651

static const uint8_t temp_addrs[] = { 0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D };
#define NUM_SENSORS ((uint8_t)(sizeof(temp_addrs) / sizeof(temp_addrs[0])))

#define TEMP_REG 0x00

/* ================= DEVICES ================= */

static const struct device *can_dev   = DEVICE_DT_GET(CAN_NODE);
static const struct device *gpioa     = DEVICE_DT_GET(GPIOA_NODE);
static const struct device *gpiob     = DEVICE_DT_GET(GPIOB_NODE);
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
static const struct gpio_dt_spec led2 = GPIO_DT_SPEC_GET(LED2_NODE, gpios);
static const struct device *i2c_bus   = DEVICE_DT_GET(I2C_BUS_NODE);

CAN_MSGQ_DEFINE(rxq, 16);

struct temp_sample {
	uint8_t addr;     // I2C 7-bit address we read from (for logging/debug) 
	int16_t temp_q4;  // Temperature in Q4 format (see below); 0 if read failed
	int status;       // 0 = read OK, non-zero = I2C error code (e.g. -EIO)
};

struct temp_telemetry {
	uint32_t t_ms;                 	    // Time of sample: kernel uptime in milliseconds (probably going to change to RTC time? idk yet)
	struct temp_sample s[NUM_SENSORS];  // One entry per sensor, same order as temp_addrs[]
};

/* ================= MODE ================= */

typedef enum {
    MODE_SETUP,
    MODE_STANDARD,
    MODE_MISSION,
    MODE_ERROR
} cdh_mode_t;

volatile cdh_mode_t current_mode = MODE_SETUP;

/* ================= THREAD STACKS ================= */

K_THREAD_STACK_DEFINE(watchdog_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(can_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(mode_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(sched_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(soh_stack, STACK_SIZE);

static struct k_thread watchdog_thread_data;
static struct k_thread can_thread_data;
static struct k_thread mode_thread_data;
static struct k_thread sched_thread_data;
static struct k_thread soh_thread_data;

/* ===================================================== */
/* ================= UTIL FUNCTIONS ===================== */
/* ===================================================== */

static void read_uid(uint32_t uid[3])
{
    LOG_INF("reading UID...");
    uid[0] = *(uint32_t *)(UID_BASE + 0x0);
    uid[1] = *(uint32_t *)(UID_BASE + 0x4);
    uid[2] = *(uint32_t *)(UID_BASE + 0x8);

}

static inline void thread_pulse(const struct gpio_dt_spec *led)
{
    gpio_pin_set_dt(led, 1);
    k_busy_wait(2000);   // 2ms pulse
    gpio_pin_set_dt(led, 0);
}

static void leds_init(void)
{
    gpio_pin_configure_dt(&led0, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led1, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led2, GPIO_OUTPUT_INACTIVE);
}

static void leds_set(cdh_mode_t mode)
{
    gpio_pin_set_dt(&led0, 0);
    gpio_pin_set_dt(&led1, 0);
    gpio_pin_set_dt(&led2, 0);

    switch(mode)
    {
        case MODE_SETUP:    gpio_pin_set_dt(&led0, 1); break;
        case MODE_STANDARD: gpio_pin_set_dt(&led1, 1); break;
        case MODE_MISSION:  gpio_pin_set_dt(&led2, 1); break;
        case MODE_ERROR:    break;
    }
}

// Print a Q4 temperature as an integer degC (rounded).
// Q4 -> degC: divide by 16; +8 before /16 gives rounding to nearest integer.
static void print_temp_q4(int16_t t_q4) {
	int32_t temp_c = (t_q4 + 8) / 16;  // +8 for rounding
	printk("%ld", (long)temp_c);
}

static int temp_telemetry_read_all(const struct device *bus, struct temp_telemetry *out) {
	// Guard: don’t dereference null; return 0 successful reads
	if (out == NULL || bus == NULL) {
		return 0;
	}

	// When did we take this snapshot? (for ground station / logs)
	out->t_ms = (uint32_t)k_uptime_get_32();

	int ok = 0;  // Init count of sensors that read successfully to 0 at first

	for (uint8_t i = 0; i < NUM_SENSORS; i++) {
		uint8_t reg = TEMP_REG;   // Register 0x00 = temperature 
		uint8_t buf[2] = {0};     // MSB, LSB from sensor 

		// Remember which address we’re polling (for debugging / labels)
		// out->s[i].addr means: pointer out -> struct member s -> element [i] -> field addr
		out->s[i].addr = temp_addrs[i];

		// Single I2C transaction: write 1 byte (reg address), then read 2 bytes.
		// Device uses 7-bit address temp_addrs[i]; we get back the temp register.
		int ret = i2c_write_read(bus, temp_addrs[i], &reg, 1, buf, 2);
		out->s[i].status = ret;

		if (ret == 0) {
			// Sensor format: 12-bit signed temp, left-justified in 16 bits.
			// Big-endian: buf[0]=MSB, buf[1]=LSB. Cast to int16_t for sign.
			// Right-shift by 4 gives Q4 (value = degC * 16). This is the only
			// place that’s sensor-specific; different chip => change this block.

			// << 8 = shift left 8 bits (put first byte in high half). | = combine with second byte.
			// So: raw = [buf[0]][buf[1]] as one 16-bit number (big-endian). int16_t = signed.
			int16_t raw = (int16_t)((buf[0] << 8) | buf[1]);

			// >> 4 = shift right 4 bits. Sensor puts 12-bit temp in top bits; bottom 4 are fraction we drop.
			// Result is already Q4 (degC * 16). Example: 25.5 C -> raw 0x1980 -> 0x198 -> 408 = 25.5*16.
			out->s[i].temp_q4 = (int16_t)(raw >> 4);
			ok++;
		} else {
			// Read failed: leave temp at 0 so downstream knows we have no data
			out->s[i].temp_q4 = 0;
		}
	}

	return ok;
}

static void temp_telemetry_print(const struct temp_telemetry *t) {
	printk("t=%lu ms | ", (unsigned long)t->t_ms);

	for (uint8_t i = 0; i < NUM_SENSORS; i++) {
		printk("0x%02X:", t->s[i].addr);

		if (t->s[i].status == 0) {
			print_temp_q4(t->s[i].temp_q4);
			printk("C");
		} else {
			printk("ERR(%d)", t->s[i].status);
		}

		if (i + 1 < NUM_SENSORS) {
			printk("  ");
		}
	}

	printk("\n");
}

/* ===================================================== */
/* ================= CAN FUNCTIONS ====================== */
/* ===================================================== */

static void send_simple(uint8_t dst, uint8_t op, uint8_t val)
{
    struct can_frame f = {0};

    f.id = CAN_ID(PRIO_LOW, dst, CLS_CORE);
    can_fill_payload(&f, NODE_ID, op, val, 0, 0, 0, 0, 0);

    can_send(can_dev, &f, K_NO_WAIT, NULL, NULL);
}

static void tcan330_wakeup(void)
{
    gpio_pin_configure(gpioa, PIN_SHDN, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure(gpioa, PIN_SILENT, GPIO_OUTPUT_INACTIVE);
    k_msleep(1);
    LOG_INF("TCAN3403 Awake");
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
        .id = CAN_DST(DST_ME),
        .mask = CAN_DST_MASK,
        .flags = 0
    };

    const struct can_filter bcast = {
        .id = CAN_DST(CAN_BROADCAST),
        .mask = CAN_DST_MASK,
        .flags = 0
    };

    can_add_rx_filter_msgq(can_dev, &rxq, &to_me);
    can_add_rx_filter_msgq(can_dev, &rxq, &bcast);

    LOG_INF("CAN Initialized");
}

/* ===================================================== */
/* ================= SETUP MODE ========================= */
/* ===================================================== */

static void setup_mode_init(void)
{
    LOG_INF("Entering SETUP MODE");

    uint32_t uid[3];
    read_uid(uid);

    printk("UID: %08X-%08X-%08X\n", uid[0], uid[1], uid[2]);

    gpio_pin_configure(gpioa, PIN_ROLE, GPIO_INPUT);
    int role_pin = gpio_pin_get(gpioa, PIN_ROLE);

    printk("PA6 Role Pin: %d\n", role_pin);

    bool uid_is_mcu1 = (uid[0]==UID1_WORD0 &&
                        uid[1]==UID1_WORD1 &&
                        uid[2]==UID1_WORD2);

    bool uid_is_mcu2 = (uid[0]==UID2_WORD0 &&
                        uid[1]==UID2_WORD1 &&
                        uid[2]==UID2_WORD2);

    bool pin_is_mcu1 = (role_pin == 0);
    bool pin_is_mcu2 = (role_pin == 1);

    if ((uid_is_mcu1 && pin_is_mcu1) ||
        (uid_is_mcu2 && pin_is_mcu2))
    {
        LOG_INF("MCU Identity Verified");
    }
    else
    {
        LOG_ERR("UID and PA6 MISMATCH!");
        LOG_ERR("Trusting PA6 hardware pin.");
    }

    tcan330_wakeup();
    can_setup();

    LOG_INF("Setup Mode Initialization Complete");
    k_sleep(K_MSEC(2000));
    current_mode = MODE_STANDARD;
}

/* ===================================================== */
/* ================= THREADS ============================ */
/* ===================================================== */

void mode_thread(void *a, void *b, void *c)
{
    LOG_INF("mode thread started");
    cdh_mode_t last = MODE_ERROR;

    while (1) {

        thread_pulse(&led0);

        if (current_mode != last) {
            leds_set(current_mode);
            last = current_mode;
        }

        k_sleep(K_MSEC(100));
    }
}

void watchdog_thread(void *a, void *b, void *c)
{
    LOG_INF("watchdog thread started");
    gpio_pin_configure(gpiob, WATCHDOG_PIN, GPIO_OUTPUT_INACTIVE);

    while (1) {

        gpio_pin_set(gpiob, WATCHDOG_PIN, 1);
        gpio_pin_set(gpiob, WATCHDOG_PIN, 0);

        thread_pulse(&led0);
        thread_pulse(&led1);

        k_sleep(K_MSEC(50));
    }
}

void can_rx_thread(void *a, void *b, void *c)
{
    LOG_INF("can rx thread started");
    struct can_frame rx;

    while (1) {

        if (current_mode == MODE_STANDARD ||
            current_mode == MODE_MISSION)
        {
            if (k_msgq_get(&rxq, &rx, K_MSEC(100)) == 0) {
                thread_pulse(&led1);
            }
        }
        else {
            k_sleep(K_MSEC(200));
        }
    }
}

void scheduler_thread(void *a, void *b, void *c)
{
    LOG_INF("scheduler thread started");
    int64_t last = 0;

    while (1) {

        if (current_mode == MODE_STANDARD ||
            current_mode == MODE_MISSION)
        {
            int64_t now = k_uptime_get();

            if (now - last >= 1000) {
                last = now;
                send_simple(CAN_BROADCAST, OP_HEARTBEAT, 0);
                thread_pulse(&led2);
            }
        }

        k_sleep(K_MSEC(50));
    }
}

void soh_thread(void *a, void *b, void *c)
{
    LOG_INF("SOH thread started");

    while (1) {

        if (current_mode == MODE_STANDARD ||
            current_mode == MODE_MISSION)
        {

	    printk("Temp poller: reading %d sensors on i2c1 (0x48..0x4D)\n", NUM_SENSORS);

	    struct temp_telemetry telem;

	    while (1) {
		    (void)temp_telemetry_read_all(i2c_bus, &telem);
		    temp_telemetry_print(&telem);
		    k_msleep(500);
	    }

        /* SOH rate = 2 seconds */
        k_sleep(K_SECONDS(2));

        }
    }
}


/* ===================================================== */
/* ================= MAIN =============================== */
/* ===================================================== */

int main(void)
{
    LOG_INF("UT-CORE CDH Booting...");

    leds_init();

    k_thread_create(&mode_thread_data, mode_stack, STACK_SIZE,
                    mode_thread, NULL, NULL, NULL,
                    PRIO_MODE, 0, K_NO_WAIT);

    k_thread_create(&can_thread_data, can_stack, STACK_SIZE,
                    can_rx_thread, NULL, NULL, NULL,
                    PRIO_CAN_RX, 0, K_NO_WAIT);

    k_thread_create(&sched_thread_data, sched_stack, STACK_SIZE,
                    scheduler_thread, NULL, NULL, NULL,
                    PRIO_SCHED, 0, K_NO_WAIT);

    k_thread_create(&watchdog_thread_data, watchdog_stack, STACK_SIZE,
                    watchdog_thread, NULL, NULL, NULL,
                    PRIO_WATCHDOG, 0, K_NO_WAIT);

    k_thread_create(&soh_thread_data, soh_stack, STACK_SIZE,
                    soh_thread, NULL, NULL, NULL,
                    PRIO_SOH, 0, K_NO_WAIT);

    setup_mode_init();

    while (1) {
        k_sleep(K_FOREVER);
    }
}
