/*
 * fram_tmr.c — TMR FRAM test application
 *
 * Includes the header-only library (fram_tmr.h) which provides:
 *   - raw per-chip FRAM helpers (wren, read, write, rdid)
 *   - verified per-chip writes with retry
 *   - TMR write (all 3 chips), TMR read (vote + repair)
 *
 * This file wires up the 3 physical FRAM chips on SPI2 and
 * runs a continuous TMR write/read stress test.
 */

#include "fram_tmr.h"

/* =========================================================================
 * PRNG for test payloads
 * ========================================================================= */

static uint32_t xorshift32_state = 0xDEADBEEF;

static uint32_t xorshift32(void)
{
    uint32_t x = xorshift32_state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    xorshift32_state = x;
    return x;
}

static void rand_fill(uint8_t *buf, size_t len)
{
    while (len >= 4) {
        uint32_t r = xorshift32();
        memcpy(buf, &r, 4);
        buf += 4;
        len -= 4;
    }
    if (len) {
        uint32_t r = xorshift32();
        memcpy(buf, &r, len);
    }
}

/* =========================================================================
 * Devicetree bindings
 * ========================================================================= */

#define SPI_BUS_NODE DT_NODELABEL(spi2)
#define SCOPE_NODE   DT_ALIAS(led0)

static const struct device *spi_bus = DEVICE_DT_GET(SPI_BUS_NODE);

static const struct gpio_dt_spec cs_gpio_0 =
    GPIO_DT_SPEC_GET_BY_IDX(SPI_BUS_NODE, cs_gpios, 0);
static const struct gpio_dt_spec cs_gpio_1 =
    GPIO_DT_SPEC_GET_BY_IDX(SPI_BUS_NODE, cs_gpios, 1);
static const struct gpio_dt_spec cs_gpio_2 =
    GPIO_DT_SPEC_GET_BY_IDX(SPI_BUS_NODE, cs_gpios, 2);

static const struct gpio_dt_spec scope_pin =
    GPIO_DT_SPEC_GET(SCOPE_NODE, gpios);

/* =========================================================================
 * 3 physical FRAM device instances
 * ========================================================================= */

static struct fram_dev fram_a;
static struct fram_dev fram_b;
static struct fram_dev fram_c;

static void init_fram_dev(struct fram_dev *dev,
                          const char *name,
                          const struct gpio_dt_spec *cs)
{
    dev->name    = name;
    dev->spi_bus = spi_bus;

    dev->spi_cfg.frequency = 400000;
    dev->spi_cfg.operation = SPI_OP_MODE_MASTER |
                             SPI_TRANSFER_MSB   |
                             SPI_WORD_SET(8);
    dev->spi_cfg.slave     = 0;
    dev->spi_cfg.cs.gpio       = *cs;
    dev->spi_cfg.cs.delay      = 1;
    dev->spi_cfg.cs.cs_is_gpio = true;
}

/* =========================================================================
 * Test parameters
 * ========================================================================= */

#define TMR_TEST_ADDR  0x000100u
#define TMR_TEST_LEN   32u

/* =========================================================================
 * main
 * ========================================================================= */

int main(void)
{
    k_busy_wait(1000);

    /* ---- hardware readiness checks ---- */

    if (!device_is_ready(spi_bus)) {
        printk("SPI bus not ready\n");
        return 0;
    }

    if (!gpio_is_ready_dt(&cs_gpio_0) ||
        !gpio_is_ready_dt(&cs_gpio_1) ||
        !gpio_is_ready_dt(&cs_gpio_2)) {
        printk("One or more FRAM CS GPIOs not ready\n");
        return 0;
    }

    if (!gpio_is_ready_dt(&scope_pin)) {
        printk("Scope GPIO not ready\n");
        return 0;
    }

    gpio_pin_configure_dt(&scope_pin, GPIO_OUTPUT_INACTIVE);

    /* ---- initialise 3 FRAM devices ---- */

    init_fram_dev(&fram_a, "FRAM_A", &cs_gpio_0);
    init_fram_dev(&fram_b, "FRAM_B", &cs_gpio_1);
    init_fram_dev(&fram_c, "FRAM_C", &cs_gpio_2);

    printk("TMR FRAM initialised: A(CS0) B(CS1) C(CS2)\n");

    /* ---- RDID sanity check on each chip ---- */

    uint8_t id[FRAM_RDID_LEN];
    struct fram_dev *chips[] = { &fram_a, &fram_b, &fram_c };
    const char *labels[]     = { "A", "B", "C" };

    for (int i = 0; i < 3; i++) {
        int err = fram_rdid(chips[i], id);
        if (err) {
            printk("RDID FRAM_%s: FAILED err=%d\n", labels[i], err);
        } else {
            printk("RDID FRAM_%s:", labels[i]);
            for (int j = 0; j < FRAM_RDID_LEN; j++) {
                printk(" %02X", id[j]);
            }
            printk("\n");
        }
    }

    /* ---- TMR write / read stress test ---- */

    printk("TMR stress test: addr=0x%06X len=%u\n",
           (unsigned)TMR_TEST_ADDR, (unsigned)TMR_TEST_LEN);

    uint8_t wbuf[TMR_TEST_LEN];
    uint8_t rbuf[TMR_TEST_LEN];

    uint32_t total  = 0;
    uint32_t ok     = 0;
    uint32_t failed = 0;

    while (1) {
        rand_fill(wbuf, sizeof(wbuf));

        /* --- TMR write (scope-pin bracketed for timing) --- */
        gpio_pin_set_dt(&scope_pin, 1);
        int err = tmr_fram_write(&fram_a, &fram_b, &fram_c,
                                 TMR_TEST_ADDR, wbuf, sizeof(wbuf));
        gpio_pin_set_dt(&scope_pin, 0);

        total++;

        if (err) {
            failed++;
            printk("TMR WRITE FAIL #%u err=%d\n", total, err);
            k_msleep(10);
            continue;
        }

        /* --- TMR read + compare --- */
        err = tmr_fram_read(&fram_a, &fram_b, &fram_c,
                            TMR_TEST_ADDR, rbuf, sizeof(rbuf));

        if (err) {
            failed++;
            printk("TMR READ FAIL #%u err=%d\n", total, err);
        } else if (memcmp(wbuf, rbuf, sizeof(wbuf)) != 0) {
            failed++;
            printk("TMR DATA MISMATCH #%u\n", total);
        } else {
            ok++;
        }

        if (total % 100 == 0) {
            uint32_t rate = (total > 0) ? (ok * 10000 / total) : 0;
            printk("--- TMR: %u total | %u ok | %u fail (%u.%02u%% pass) ---\n",
                   total, ok, failed, rate / 100, rate % 100);
        }

        /* optional: verify all 3 chips hold identical data */
        if (total % 1000 == 0) {
            int veq = tmr_fram_verify_all_equal(&fram_a, &fram_b, &fram_c,
                                                TMR_TEST_ADDR, sizeof(wbuf));
            printk("--- TMR verify-all-equal: %s ---\n",
                   veq ? "MISMATCH" : "OK");
        }

        k_msleep(10);
    }
}
