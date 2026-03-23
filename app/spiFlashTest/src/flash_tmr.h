/*
 * flash_tmr.h — Flash-to-FRAM TMR provisioning and scrub layer
 *
 * Builds on top of fram_tmr.h to provide:
 *   - One-time provisioning:  MCU internal flash  ->  FRAM x3 (TMR)
 *   - Periodic scrubbing:    vote + repair FRAMs, compare against flash
 *
 * Architecture:
 *   The MCU internal flash is the initial source of truth (it holds
 *   the firmware image at power-on).  After provisioning, the three
 *   FRAM copies become the golden reference.  TMR voting keeps them
 *   consistent.  If internal flash is later found to disagree with
 *   the FRAM majority, flash is the one that's wrong.
 *
 * FRAM address mapping:
 *   1:1 with flash offset.  Flash offset 0 -> FRAM address 0.
 *   The firmware image must fit within FRAM_SIZE (512 KiB).
 */

#ifndef FLASH_TMR_H_
#define FLASH_TMR_H_

#include "fram_tmr.h"
#include <zephyr/drivers/flash.h>

/* =========================================================================
 * Linker symbols — boundaries of the firmware image in flash
 * ========================================================================= */

extern char __rom_region_start[];
extern char __rom_region_end[];

/* =========================================================================
 * Tunables
 * ========================================================================= */

#define FLASH_TMR_CHUNK_SZ  32u

/* =========================================================================
 * Helpers
 * ========================================================================= */

static inline size_t flash_tmr_fw_size(void)
{
    return (size_t)(__rom_region_end - __rom_region_start);
}

/* =========================================================================
 * Provision:  read MCU flash  ->  TMR write to FRAM A, B, C
 *
 * Call once at boot.  After this returns successfully the three
 * FRAMs hold identical copies of the firmware image and become
 * the golden reference.
 * ========================================================================= */

static int flash_tmr_provision(const struct device *flash_dev,
                               struct fram_dev *fa,
                               struct fram_dev *fb,
                               struct fram_dev *fc)
{
    size_t fw_size = flash_tmr_fw_size();
    uint8_t buf[FLASH_TMR_CHUNK_SZ];

    if (!device_is_ready(flash_dev)) {
        printk("PROVISION: flash device not ready\n");
        return -ENODEV;
    }

    if (fw_size == 0) {
        printk("PROVISION: firmware size is 0 — linker symbols wrong?\n");
        return -EINVAL;
    }

    if (fw_size > FRAM_SIZE) {
        printk("PROVISION: firmware (%u B) exceeds FRAM capacity (%u B)\n",
               (unsigned)fw_size, (unsigned)FRAM_SIZE);
        return -ENOMEM;
    }

    printk("PROVISION: %u bytes  flash -> FRAM x3\n", (unsigned)fw_size);

    for (size_t off = 0; off < fw_size; off += FLASH_TMR_CHUNK_SZ) {
        size_t chunk = (fw_size - off < FLASH_TMR_CHUNK_SZ)
                     ? (fw_size - off)
                     : FLASH_TMR_CHUNK_SZ;

        int err = flash_read(flash_dev, off, buf, chunk);
        if (err) {
            printk("PROVISION: flash_read @ 0x%06x err=%d\n",
                   (unsigned)off, err);
            return err;
        }

        err = tmr_fram_write(fa, fb, fc, (uint32_t)off, buf, chunk);
        if (err) {
            printk("PROVISION: tmr_write @ 0x%06x err=%d\n",
                   (unsigned)off, err);
            return err;
        }

        if ((off & 0xFFF) == 0) {
            printk("  0x%06x / 0x%06x\n", (unsigned)off, (unsigned)fw_size);
        }
    }

    printk("PROVISION: complete\n");
    return 0;
}

/* =========================================================================
 * Scrub result
 * ========================================================================= */

struct flash_tmr_scrub_result {
    uint32_t chunks_checked;
    uint32_t flash_mismatches;
};

/* =========================================================================
 * Scrub:  FRAM vote + repair,  then compare flash against FRAM majority
 *
 * Two things happen in each chunk iteration:
 *
 *   1. tmr_fram_read() reads all 3 FRAMs, votes, and auto-repairs
 *      the odd chip out (FRAM-to-FRAM scrub).
 *
 *   2. The voted FRAM result is compared against the MCU internal
 *      flash.  Mismatches are reported — the FRAM majority is
 *      considered correct and flash is the suspect.
 *
 * Returns 0 on success, negative errno on hard failure.
 * ========================================================================= */

static int flash_tmr_scrub(const struct device *flash_dev,
                           struct fram_dev *fa,
                           struct fram_dev *fb,
                           struct fram_dev *fc,
                           struct flash_tmr_scrub_result *result)
{
    size_t fw_size = flash_tmr_fw_size();
    uint8_t fram_buf[FLASH_TMR_CHUNK_SZ];
    uint8_t flash_buf[FLASH_TMR_CHUNK_SZ];

    if (result) {
        memset(result, 0, sizeof(*result));
    }

    for (size_t off = 0; off < fw_size; off += FLASH_TMR_CHUNK_SZ) {
        size_t chunk = (fw_size - off < FLASH_TMR_CHUNK_SZ)
                     ? (fw_size - off)
                     : FLASH_TMR_CHUNK_SZ;

        /* FRAM majority vote (repairs divergent chip internally) */
        int err = tmr_fram_read(fa, fb, fc, (uint32_t)off, fram_buf, chunk);
        if (err) {
            printk("SCRUB: FRAM read error @ 0x%06x: %d\n",
                   (unsigned)off, err);
            return err;
        }

        /* read the same region from MCU internal flash */
        err = flash_read(flash_dev, off, flash_buf, chunk);
        if (err) {
            printk("SCRUB: flash_read error @ 0x%06x: %d\n",
                   (unsigned)off, err);
            return err;
        }

        if (memcmp(flash_buf, fram_buf, chunk) != 0) {
            printk("SCRUB: flash != FRAM @ 0x%06x  (FRAM is golden)\n",
                   (unsigned)off);
            if (result) {
                result->flash_mismatches++;
            }
        }

        if (result) {
            result->chunks_checked++;
        }
    }

    return 0;
}

#endif /* FLASH_TMR_H_ */
