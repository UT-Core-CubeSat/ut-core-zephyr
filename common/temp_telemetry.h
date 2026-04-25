#pragma once

/*
 * UT-CORE Shared Temperature Telemetry
 *
 * I2C temperature sensor reader for TMP1xx/TMP275-family sensors.
 * All boards include this header and call the functions with their
 * own I2C bus device handle.
 *
 * Temps are stored in Q4 fixed-point (value = degC * 16) to avoid
 * floating-point on the MCU.
 *   Examples:  0°C → 0,  25.0°C → 400,  25.5°C → 408
 *   To integer °C (rounded):  (temp_q4 + 8) / 16
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>
#include <stdint.h>

/* ============== Sensor Configuration ============== */

/* TMP1xx I2C 7-bit addresses (A0-A2 strap pins → 0x48-0x4D). */
static const uint8_t temp_addrs[] = { 0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D };
#define NUM_TEMP_SENSORS ((uint8_t)(sizeof(temp_addrs) / sizeof(temp_addrs[0])))

/* TMP1xx temperature register address. */
#define TEMP_REG 0x00

/* ============== Data Structures ============== */

/* Single sensor reading. */
struct temp_sample {
    uint8_t addr;      /* I2C address (0x48-0x4D), for debug labelling   */
    int16_t temp_q4;   /* Temperature in Q4 (degC * 16); 0 if read failed */
    int     status;    /* 0 = success, negative errno on I2C failure      */
};

/* Snapshot of all sensors at one instant. */
struct temp_telemetry {
    uint32_t t_ms;                          /* k_uptime_get_32() timestamp */
    struct temp_sample s[NUM_TEMP_SENSORS]; /* Same order as temp_addrs[]  */
};

/* ============== Functions ============== */

/*
 * temp_telemetry_read_all() - Poll every configured sensor over I2C.
 *
 * @bus:  Zephyr I2C device handle (caller provides — each board may differ).
 * @out:  Caller-allocated telemetry struct to fill.
 * @return: Number of sensors read successfully (0 to NUM_TEMP_SENSORS).
 *
 * Per-sensor failures are recorded in s[i].status; a dead sensor does not
 * invalidate the rest of the snapshot.
 */
static inline int temp_telemetry_read_all(const struct device *bus,
                                          struct temp_telemetry *out)
{
    if (out == NULL || bus == NULL) {
        return 0;
    }

    out->t_ms = (uint32_t)k_uptime_get_32();
    int ok = 0;

    for (uint8_t i = 0; i < NUM_TEMP_SENSORS; i++) {
        uint8_t reg    = TEMP_REG;
        uint8_t buf[2] = {0};

        out->s[i].addr = temp_addrs[i];

        int ret = i2c_write_read(bus, temp_addrs[i], &reg, 1, buf, 2);
        out->s[i].status = ret;

        if (ret == 0) {
            /* Big-endian 16-bit → sign-extend → right-shift 4 → Q4 */
            int16_t raw = (int16_t)((buf[0] << 8) | buf[1]);
            out->s[i].temp_q4 = (int16_t)(raw >> 4);
            ok++;
        } else {
            out->s[i].temp_q4 = 0;
        }
    }

    return ok;
}

/*
 * print_temp_q4() - Print a Q4 value as a rounded integer °C.
 */
static inline void print_temp_q4(int16_t t_q4)
{
    int32_t temp_c = (t_q4 + 8) / 16;
    printk("%ld", (long)temp_c);
}

/*
 * temp_telemetry_print() - Print a full snapshot to console.
 * Format: "t=<ms> ms | 0x48:25C  0x49:ERR(-5) ..."
 */
static inline void temp_telemetry_print(const struct temp_telemetry *t)
{
    printk("t=%lu ms | ", (unsigned long)t->t_ms);

    for (uint8_t i = 0; i < NUM_TEMP_SENSORS; i++) {
        printk("0x%02X:", t->s[i].addr);

        if (t->s[i].status == 0) {
            print_temp_q4(t->s[i].temp_q4);
            printk("C");
        } else {
            printk("ERR(%d)", t->s[i].status);
        }

        if (i + 1 < NUM_TEMP_SENSORS) {
            printk("  ");
        }
    }

    printk("\n");
}