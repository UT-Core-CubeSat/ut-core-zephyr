/*
 * flash_tmr.c — Flash-to-FRAM TMR application
 *
 * On boot:
 *   1. Reads the MCU internal flash (the running firmware image)
 *   2. TMR-writes it to all 3 FRAM chips (provisioning)
 *   3. Verifies the provisioned data matches flash
 *
 * Then runs a periodic scrub loop:
 *   - Votes across the 3 FRAMs and repairs any divergent chip
 *   - Compares the FRAM majority against internal flash
 *   - Reports any mismatches (FRAM is the golden reference)
 */

#include "flash_tmr.h"

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
 * Configuration
 * ========================================================================= */

#define SCRUB_INTERVAL_MS  10000u
#define INJECT_EVERY_N     3u        /* inject a fault every N scrub cycles */
#define INJECT_ADDR        0x0200u   /* FRAM address to corrupt */

/* =========================================================================
 * Fault injection — simulate a single-bit flip on one FRAM chip
 *
 * Bypasses TMR and writes directly to one chip so the other two
 * still hold the correct data.  The next scrub should detect the
 * divergence, outvote the bad chip, and repair it.
 * ========================================================================= */

static int inject_bitflip(struct fram_dev *target, uint32_t addr)
{
    uint8_t original;

    int err = fram_read(target, addr, &original, 1);
    if (err) {
        return err;
    }

    uint8_t corrupted = original ^ 0x01;

    err = fram_wren(target);
    if (err) {
        return err;
    }
    k_busy_wait(1);

    err = fram_write_fixed(target, addr, &corrupted, 1);
    if (err) {
        return err;
    }

    printk("INJECT: flipped bit0 on %s @ 0x%06x  (0x%02X -> 0x%02X)\n",
           target->name, (unsigned)addr, original, corrupted);
    return 0;
}

/* =========================================================================
 * main
 * ========================================================================= */

int main(void)
{
    k_busy_wait(1000);

    /* ---- hardware readiness ---- */

    if (!device_is_ready(spi_bus)) {
        printk("SPI bus not ready\n");
        return 0;
    }

    if (!gpio_is_ready_dt(&cs_gpio_0) ||
        !gpio_is_ready_dt(&cs_gpio_1) ||
        !gpio_is_ready_dt(&cs_gpio_2)) {
        printk("FRAM CS GPIOs not ready\n");
        return 0;
    }

    if (!gpio_is_ready_dt(&scope_pin)) {
        printk("Scope GPIO not ready\n");
        return 0;
    }

    gpio_pin_configure_dt(&scope_pin, GPIO_OUTPUT_INACTIVE);

    /* ---- init 3 FRAM devices ---- */

    init_fram_dev(&fram_a, "FRAM_A", &cs_gpio_0);
    init_fram_dev(&fram_b, "FRAM_B", &cs_gpio_1);
    init_fram_dev(&fram_c, "FRAM_C", &cs_gpio_2);

    printk("Flash-TMR: FRAMs initialised  A(CS0) B(CS1) C(CS2)\n");

    /* ---- RDID sanity check ---- */

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

    /* ---- get internal flash device ---- */

    const struct device *flash_dev = DEVICE_DT_GET(DT_PARENT(DT_CHOSEN(zephyr_flash)));

    printk("Firmware size: %u bytes\n", (unsigned)flash_tmr_fw_size());

    /* ---- Step 1:  provision  flash -> FRAM x3 ---- */

    gpio_pin_set_dt(&scope_pin, 1);
    int err = flash_tmr_provision(flash_dev, &fram_a, &fram_b, &fram_c);
    gpio_pin_set_dt(&scope_pin, 0);

    if (err) {
        printk("PROVISIONING FAILED: %d — halting\n", err);
        return 0;
    }

    /* ---- Step 2:  verify provision (first scrub pass) ---- */

    struct flash_tmr_scrub_result result;
    err = flash_tmr_scrub(flash_dev, &fram_a, &fram_b, &fram_c, &result);

    if (err) {
        printk("POST-PROVISION VERIFY FAILED: %d — halting\n", err);
        return 0;
    }

    if (result.flash_mismatches > 0) {
        printk("POST-PROVISION VERIFY: %u mismatches — halting\n",
               result.flash_mismatches);
        return 0;
    }

    printk("POST-PROVISION VERIFY: OK  (%u chunks)\n", result.chunks_checked);
    printk("Entering scrub loop (interval %u ms)...\n",
           (unsigned)SCRUB_INTERVAL_MS);

    /* ---- Step 3:  periodic scrub  (FRAM is golden) ---- */

    uint32_t scrub_count = 0;

    struct fram_dev *fault_targets[] = { &fram_a, &fram_b, &fram_c };

    while (1) {
        k_msleep(SCRUB_INTERVAL_MS);
        scrub_count++;

        /* inject a fault before the scrub so the scrub has to catch it */
        if (scrub_count % INJECT_EVERY_N == 0) {
            int target_idx = (scrub_count / INJECT_EVERY_N) % 3;
            inject_bitflip(fault_targets[target_idx], INJECT_ADDR);
        }

        gpio_pin_set_dt(&scope_pin, 1);
        err = flash_tmr_scrub(flash_dev, &fram_a, &fram_b, &fram_c, &result);
        gpio_pin_set_dt(&scope_pin, 0);

        if (err) {
            printk("SCRUB #%u ERROR: %d\n", scrub_count, err);
        } else if (result.flash_mismatches > 0) {
            printk("SCRUB #%u: %u flash mismatches detected  (%u chunks)\n",
                   scrub_count, result.flash_mismatches, result.chunks_checked);
        } else {
            printk("SCRUB #%u: OK  (%u chunks)\n",
                   scrub_count, result.chunks_checked);
        }
    }
}
