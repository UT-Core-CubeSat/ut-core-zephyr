/*
 * flash_tmr.c — Flash-to-FRAM Triple Modular Redundancy (TMR) application
 *
 * OVERVIEW
 * --------
 * This file ties together the lower-level FRAM and TMR libraries
 * (fram_tmr.h, flash_provision_tmr.h) into a runnable Zephyr application
 * with two execution phases:
 *
 *   Phase 1  (main, runs once at boot):
 *     1. Initialises SPI, GPIOs, and the 3 FRAM chips
 *     2. Reads the entire running firmware image out of the MCU's
 *        internal flash
 *     3. TMR-writes that image to all 3 external FRAM chips
 *        (this is called "provisioning")
 *     4. Immediately verifies the provision by running one scrub pass
 *     5. Releases the scrub thread
 *
 *   Phase 2  (scrub thread, runs forever in background):
 *     A dedicated low-priority Zephyr thread that periodically:
 *     1. Reads the same address range from all 3 FRAMs
 *     2. Majority-votes the data (2-of-3 wins)
 *     3. Auto-repairs whichever FRAM disagrees with the majority
 *     4. Compares the FRAM majority against MCU internal flash
 *        (FRAM is the golden reference after provisioning)
 *     5. Yields the CPU every N chunks so it never starves
 *        higher-priority work (comms, sensors, etc.)
 *
 * WHY A SEPARATE THREAD?
 * ----------------------
 * The scrub is important but not time-critical.  It can take seconds
 * to walk the entire firmware image.  Running it at a low priority
 * means the MCU is always free to service real-time tasks first.
 * The scrub just fills in idle time.  Zephyr's preemptive scheduler
 * ensures higher-priority threads always run immediately, even if
 * the scrub is mid-pass.
 *
 * FAULT INJECTION (testing only)
 * ------------------------------
 * For bench testing, inject_bitflip() deliberately corrupts one byte
 * on one FRAM chip every few scrub cycles.  The next scrub should
 * detect the divergence via majority vote and repair it automatically.
 * Remove or #ifdef this out for flight.
 */

#include "flash_provision_tmr.h"

/* =========================================================================
 * Devicetree bindings
 *
 * Zephyr's Devicetree (DTS) describes the hardware.  These macros
 * pull peripheral handles and GPIO specs directly from the board's
 * .dts file at compile time — no magic numbers needed.
 *
 *   DT_NODELABEL(spi2)    → the SPI2 peripheral defined in ut_core.dts
 *   DT_ALIAS(led0)        → the "led0" alias (we reuse it as a scope pin)
 *   DEVICE_DT_GET(...)    → gives us a const struct device* we can use
 *                            with Zephyr driver APIs (spi_transceive, etc.)
 *   GPIO_DT_SPEC_GET(...) → gives us a gpio_dt_spec (port + pin + flags)
 *                            ready for gpio_pin_set_dt() and friends
 * ========================================================================= */

/* SPI2 bus handle — all 3 FRAMs share this bus */
#define SPI_BUS_NODE DT_NODELABEL(spi2)

/* Scope/debug pin — directly aliases to a GPIO, which can be connected
 * to an oscilloscope or logic analyzer to time scrub/provision passes */
#define SCOPE_NODE   DT_ALIAS(led0)

static const struct device *spi_bus = DEVICE_DT_GET(SPI_BUS_NODE);

/*
 * Chip-select GPIOs — one per FRAM chip.
 * In the devicetree, spi2's cs-gpios property lists 3 GPIOs:
 *   index 0 → PB10  (FRAM_A)
 *   index 1 → PB8   (FRAM_B)
 *   index 2 → PB4   (FRAM_C)
 * The SPI driver asserts the correct CS automatically during each
 * transaction based on which spi_config we pass.
 */
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
 *
 * Each fram_dev holds a name (for debug prints), a pointer to the
 * shared SPI bus, and its own spi_config which includes the
 * chip-select GPIO.  When we call any fram_*() function with one of
 * these, the SPI driver knows which CS to assert.
 * ========================================================================= */

static struct fram_dev fram_a;
static struct fram_dev fram_b;
static struct fram_dev fram_c;

/*
 * init_fram_dev — populate one fram_dev struct.
 *
 * @dev   pointer to the fram_dev to initialise
 * @name  human-readable label (e.g. "FRAM_A") for printk messages
 * @cs    chip-select GPIO spec (from devicetree) for this particular chip
 *
 * All 3 FRAMs use the same SPI bus, same clock rate (400 kHz), same
 * SPI mode (MODE 0: CPOL=0 CPHA=0), same word size (8-bit).
 * The only difference between them is which CS pin gets asserted.
 */
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
    dev->spi_cfg.cs.delay      = 1;     /* 1 us CS setup time */
    dev->spi_cfg.cs.cs_is_gpio = true;  /* CS is a software-controlled GPIO */
}

/* =========================================================================
 * Configuration
 * ========================================================================= */

/* How often the scrub thread wakes up and walks the entire image (ms) */
#define SCRUB_INTERVAL_MS  10000u

/* --- Fault injection (testing only — remove for flight) --- */
#define INJECT_EVERY_N     3u        /* inject a fault every N scrub cycles */
#define INJECT_ADDR        0x0200u   /* FRAM address to corrupt */

/* =========================================================================
 * Scrub thread configuration
 *
 * TMR_SCRUB_STACK_SZ — bytes of stack for the scrub thread.  Needs to
 *   hold the scrub function's local buffers (~3×32 B for FRAM reads,
 *   32 B for flash read) plus whatever the SPI driver uses internally.
 *   2048 is generous; 1024 would likely work but no reason to cut it close.
 *
 * TMR_SCRUB_PRIORITY — Zephyr preemptive priority (0 = highest, 14 = lowest
 *   with default CONFIG_NUM_PREEMPT_PRIORITIES=15).  We pick 14 so the scrub
 *   only runs when no other preemptive thread needs the CPU.
 * ========================================================================= */

#define TMR_SCRUB_STACK_SZ  2048
#define TMR_SCRUB_PRIORITY  14

/*
 * flash_dev_ptr — set by main() after devicetree lookup, read by
 * the scrub thread.  Safe because the thread doesn't touch it until
 * after main() gives the semaphore.
 */
static const struct device *flash_dev_ptr;

/*
 * scrub_start_sem — binary semaphore that gates the scrub thread.
 *
 * Starts at count=0 (locked).  The scrub thread is created at boot
 * by K_THREAD_DEFINE and immediately blocks on k_sem_take().
 * main() does all one-time init and provisioning, then calls
 * k_sem_give() to release the thread.  This guarantees provisioning
 * finishes before any scrub runs.
 */
static K_SEM_DEFINE(scrub_start_sem, 0, 1);

/* =========================================================================
 * Fault injection — simulate a single-bit flip on one FRAM chip
 *
 * FOR TESTING ONLY.  This bypasses the TMR layer and writes directly
 * to one chip, flipping bit 0 of whatever byte is at `addr`.  The
 * other two chips still hold the correct data.  The next scrub pass
 * should detect the divergence via majority vote and repair it.
 *
 * @target  which FRAM chip to corrupt
 * @addr    FRAM address to flip
 * @return  0 on success, negative errno on SPI failure
 * ========================================================================= */

static int inject_bitflip(struct fram_dev *target, uint32_t addr)
{
    uint8_t original;

    int err = fram_read(target, addr, &original, 1);
    if (err) {
        return err;
    }

    uint8_t corrupted = original ^ 0x01;  /* flip least-significant bit */

    err = fram_wren(target);              /* WREN required before any write */
    if (err) {
        return err;
    }
    k_busy_wait(1);                       /* 1 us settle after WREN */

    err = fram_write_fixed(target, addr, &corrupted, 1);
    if (err) {
        return err;
    }

    printk("INJECT: flipped bit0 on %s @ 0x%06x  (0x%02X -> 0x%02X)\n",
           target->name, (unsigned)addr, original, corrupted);
    return 0;
}

/* =========================================================================
 * Scrub thread entry point
 *
 * This function is the body of a Zephyr thread created at boot via
 * K_THREAD_DEFINE (see below).  It:
 *
 *   1. Blocks until main() signals that provisioning is complete
 *   2. Enters an infinite loop:
 *      a. Sleeps for SCRUB_INTERVAL_MS (yields CPU entirely)
 *      b. Optionally injects a fault for testing
 *      c. Runs flash_tmr_scrub() which walks every chunk of the
 *         firmware image, majority-votes across 3 FRAMs, repairs
 *         any divergent chip, and compares against internal flash
 *      d. Reports results over printk
 *
 * The scrub function itself (in flash_provision_tmr.h) calls k_yield()
 * every SCRUB_YIELD_INTERVAL chunks, so even during a scrub pass
 * higher-priority threads can preempt without waiting for the full
 * pass to finish.
 *
 * The three void* parameters (p1, p2, p3) are required by Zephyr's
 * thread signature but unused here — we access everything through
 * file-scoped statics instead.
 * ========================================================================= */

static void tmr_scrub_thread_entry(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    /* Block here until main() finishes provisioning and gives the sem */
    k_sem_take(&scrub_start_sem, K_FOREVER);

    printk("TMR scrub thread: running  (priority %d, interval %u ms)\n",
           TMR_SCRUB_PRIORITY, (unsigned)SCRUB_INTERVAL_MS);

    uint32_t scrub_count = 0;
    struct fram_dev *fault_targets[] = { &fram_a, &fram_b, &fram_c };

    while (1) {
        /* Sleep between scrub passes — the thread is completely off the
         * ready queue during this time, consuming zero CPU. */
        k_msleep(SCRUB_INTERVAL_MS);
        scrub_count++;

        /* --- Fault injection (testing only) ---
         * Every INJECT_EVERY_N scrub cycles, corrupt one byte on one
         * FRAM chip.  Rotates through A → B → C so each chip gets a turn. */
        if (scrub_count % INJECT_EVERY_N == 0) {
            int target_idx = (scrub_count / INJECT_EVERY_N) % 3;
            inject_bitflip(fault_targets[target_idx], INJECT_ADDR);
        }

        struct flash_tmr_scrub_result result;

        /* Scope pin high for the duration of the scrub — lets you
         * measure scrub time on a logic analyser / oscilloscope */
        gpio_pin_set_dt(&scope_pin, 1);
        int err = flash_tmr_scrub(flash_dev_ptr,
                                  &fram_a, &fram_b, &fram_c, &result);
        gpio_pin_set_dt(&scope_pin, 0);

        if (err) {
            printk("SCRUB #%u ERROR: %d\n", scrub_count, err);
        } else if (result.flash_mismatches > 0) {
            printk("SCRUB #%u: %u flash mismatches detected  (%u chunks)\n",
                   scrub_count, result.flash_mismatches,
                   result.chunks_checked);
        } else {
            printk("SCRUB #%u: OK  (%u chunks)\n",
                   scrub_count, result.chunks_checked);
        }
    }
}

/*
 * K_THREAD_DEFINE — statically create the scrub thread at compile time.
 *
 * Zephyr spawns this thread automatically at boot.  It starts running
 * immediately but hits k_sem_take() and blocks until main() is done.
 *
 * Parameters:
 *   tmr_scrub_tid          — thread ID variable name (can be used to
 *                             suspend/resume the thread from elsewhere)
 *   TMR_SCRUB_STACK_SZ     — stack size in bytes
 *   tmr_scrub_thread_entry — entry point function
 *   NULL, NULL, NULL       — p1, p2, p3 arguments (unused)
 *   TMR_SCRUB_PRIORITY     — preemptive priority (14 = very low)
 *   0                      — options (none)
 *   0                      — startup delay in ms (0 = start immediately)
 */
K_THREAD_DEFINE(tmr_scrub_tid, TMR_SCRUB_STACK_SZ,
                tmr_scrub_thread_entry, NULL, NULL, NULL,
                TMR_SCRUB_PRIORITY, 0, 0);

/* =========================================================================
 * main — one-time init + provision, then hand off to scrub thread
 *
 * In Zephyr, main() runs on the "main thread" which has a default
 * priority of 0 (highest preemptive).  When main() returns, the main
 * thread terminates — but other threads (like our scrub thread)
 * keep running.  The RTOS scheduler is already active before main()
 * is even called.
 * ========================================================================= */

int main(void)
{
    /* Short delay for SPI / GPIO peripheral startup (belt-and-suspenders) */
    k_busy_wait(1000);

    /* ---- hardware readiness checks ----
     *
     * Zephyr's device model requires that we verify a device is ready
     * before using it.  DEVICE_DT_GET gives us a compile-time pointer,
     * but the device might fail its init at runtime (e.g. clock not
     * configured, pin conflict).  If any check fails, we bail. */

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

    /* ---- init 3 FRAM devices ----
     *
     * Each call populates a fram_dev struct with the shared SPI bus
     * handle and a unique chip-select.  After this, we can pass any
     * of these to the fram_*() / tmr_*() functions. */

    init_fram_dev(&fram_a, "FRAM_A", &cs_gpio_0);
    init_fram_dev(&fram_b, "FRAM_B", &cs_gpio_1);
    init_fram_dev(&fram_c, "FRAM_C", &cs_gpio_2);

    printk("Flash-TMR: FRAMs initialised  A(CS0) B(CS1) C(CS2)\n");

    /* ---- RDID sanity check ----
     *
     * Read the 9-byte device ID from each FRAM to confirm they're
     * alive and responding on the bus.  If RDID comes back wrong
     * (or fails), something is physically wrong — bad solder joint,
     * wrong CS wiring, etc. */

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

    /* ---- get internal flash device ----
     *
     * This retrieves the MCU's own internal flash controller.  We
     * need it to read out the running firmware image.
     *
     * DT_CHOSEN(zephyr_flash) is the flash *partition* (e.g. slot0).
     * DT_PARENT(...) gets the actual flash controller device so we
     * can call flash_read() on it. */

    flash_dev_ptr = DEVICE_DT_GET(DT_PARENT(DT_CHOSEN(zephyr_flash)));

    printk("Firmware size: %u bytes\n", (unsigned)flash_tmr_fw_size());

    /* ---- Step 1:  provision  flash -> FRAM x3 ----
     *
     * Walk the entire firmware image in FLASH_TMR_CHUNK_SZ-byte chunks,
     * reading each chunk from MCU flash and TMR-writing it to all 3
     * FRAMs.  After this, the FRAMs hold identical copies of the
     * running firmware and become the golden reference. */

    gpio_pin_set_dt(&scope_pin, 1);
    int err = flash_tmr_provision(flash_dev_ptr, &fram_a, &fram_b, &fram_c);
    gpio_pin_set_dt(&scope_pin, 0);

    if (err) {
        printk("PROVISIONING FAILED: %d — halting\n", err);
        return 0;
    }

    /* ---- Step 2:  verify provision (first scrub pass) ----
     *
     * Run one full scrub immediately to confirm the provision was
     * perfect.  If there are any mismatches between flash and FRAM
     * right after provisioning, something went seriously wrong
     * (e.g. FRAM hardware fault) and we should not continue. */

    struct flash_tmr_scrub_result result;
    err = flash_tmr_scrub(flash_dev_ptr, &fram_a, &fram_b, &fram_c, &result);

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

    /* ---- release scrub thread ----
     *
     * Give the semaphore.  The scrub thread has been blocked on
     * k_sem_take() since boot — this unblocks it and it enters
     * its infinite scrub loop.  main() then returns and the main
     * thread terminates, but the scrub thread (and any other
     * application threads) keeps running under the Zephyr scheduler. */

    k_sem_give(&scrub_start_sem);

    return 0;
}
