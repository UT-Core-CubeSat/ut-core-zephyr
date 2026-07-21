#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/can.h>
#include <stm32u5xx.h>

int main(void)
{
    printk("\nUT-CORE boot OK\n");

    uint32_t uid0 = *(uint32_t *)(UID_BASE + 0x0);
    uint32_t uid1 = *(uint32_t *)(UID_BASE + 0x4);
    uint32_t uid2 = *(uint32_t *)(UID_BASE + 0x8);

    printk("STM32U5 UID:\n");
    printk("%08X-%08X-%08X\n", uid0, uid1, uid2);

    while (1) {
        k_sleep(K_SECONDS(1));
    }
}

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>
#include <stm32u5xx.h>
#include <string.h>

/* ========================= */
/* EXPECTED UID DEFINITIONS */
/* ========================= */

static const uint32_t MCU1_UID[3] = {
    0x00340016,
    0x41425007,
    0x20363651
};

static const uint32_t MCU2_UID[3] = {
    0x0012001B,
    0x41425007,
    0x20363651
};

/* ========================= */
/* GPIO ROLE PIN             */
/* ========================= */

#define MCU_ROLE_NODE DT_ALIAS(mcuselect)

#if !DT_NODE_HAS_STATUS(MCU_ROLE_NODE, okay)
#error "MCU role pin alias not defined"
#endif

static const struct gpio_dt_spec mcu_select_pin =
    GPIO_DT_SPEC_GET(MCU_ROLE_NODE, gpios);

/* ========================= */
/* HELPER FUNCTIONS          */
/* ========================= */

static bool uid_matches(const uint32_t uid[3], const uint32_t expected[3])
{
    return (uid[0] == expected[0] &&
            uid[1] == expected[1] &&
            uid[2] == expected[2]);
}

/* ========================= */
/* MAIN                      */
/* ========================= */

int main(void)
{
    int ret;

    printk("\n============================\n");
    printk("UT-CORE Setup Start\n");
    printk("============================\n");

    /* Configure PA6 as input */
    ret = gpio_pin_configure_dt(&mcu_select_pin, GPIO_INPUT);
    if (ret < 0) {
        printk("ERROR: Failed to configure MCU role pin\n");
        return 0;
    }

    /* Read PA6 */
    int pa6_level = gpio_pin_get_dt(&mcu_select_pin);
    bool pin_says_mcu2 = (pa6_level != 0); /* HIGH = MCU2 */

    /* Read UID */
    uint32_t uid[3];
    uid[0] = *(uint32_t *)(UID_BASE + 0x0);
    uid[1] = *(uint32_t *)(UID_BASE + 0x4);
    uid[2] = *(uint32_t *)(UID_BASE + 0x8);

    printk("Read UID: %08X-%08X-%08X\n",
           uid[0], uid[1], uid[2]);

    printk("PA6 state: %s\n", pa6_level ? "HIGH" : "LOW");

    /* ========================= */
    /* CONSISTENCY CHECK         */
    /* ========================= */

    bool uid_says_mcu1 = uid_matches(uid, MCU1_UID);
    bool uid_says_mcu2 = uid_matches(uid, MCU2_UID);

    if (pin_says_mcu2) {
        printk("ROLE (from pin): MCU2\n");

        if (!uid_says_mcu2) {
            printk("WARNING: UID does NOT match expected MCU2 UID!\n");
        }
    } else {
        printk("ROLE (from pin): MCU1\n");

        if (!uid_says_mcu1) {
            printk("WARNING: UID does NOT match expected MCU1 UID!\n");
        }
    }

    /* Extra diagnostic: unknown UID */
    if (!uid_says_mcu1 && !uid_says_mcu2) {
        printk("WARNING: UID does not match any known MCU!\n");
    }

    printk("============================\n");
    printk("Setup complete trusting PA6 for role\n");
    printk("============================\n");

    while (1) {
        k_sleep(K_SECONDS(1));
    }
}