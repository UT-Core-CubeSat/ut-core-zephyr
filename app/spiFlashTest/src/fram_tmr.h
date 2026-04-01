/*
 * fram_tmr.h / fram_tmr.c style implementation
 *
 * What this gives you:
 *  - raw per-chip FRAM read/write helpers
 *  - verified per-chip writes
 *  - TMR write to 3 physical FRAM chips
 *  - TMR read with whole-buffer majority vote
 *  - optional single-chip repair on read
 *
 * Assumptions:
 *  - all 3 FRAMs are on the same SPI bus
 *  - each FRAM has its own chip-select GPIO
 *  - same opcode set / same memory map on each chip
 *  - same max size on each chip
 *
 * Notes:
 *  - this intentionally does NOT include main()
 *  - this intentionally does NOT include random test code
 *  - this is the reusable layer you build on top of
 */

#ifndef FRAM_TMR_H_
#define FRAM_TMR_H_

 #include <zephyr/kernel.h>
 #include <zephyr/device.h>
 #include <zephyr/drivers/spi.h>
 #include <zephyr/drivers/gpio.h>
 #include <zephyr/sys/printk.h>
 #include <zephyr/cache.h>
 
 #include <string.h>
 #include <stdbool.h>
 #include <stdint.h>
 #include <errno.h>
 
 /* =========================
  * Tunables
  * ========================= */
 
 #define FRAM_SIZE             0x80000u   /* 512 KiB */
 #define FRAM_RW_MAX_LEN       256u       /* raw transaction max payload */
 #define FRAM_VERIFY_CHUNK_SZ  16u
 #define FRAM_MAX_RETRIES      5
 
 #define TMR_READ_CHUNK_SZ     32u
 #define TMR_ENABLE_REPAIR     1
 
 /* =========================
  * FRAM device abstraction
  * ========================= */
 
 struct fram_dev {
     const char *name;
     const struct device *spi_bus;
     struct spi_config spi_cfg;
 };
 
 /* =========================
  * Helpers
  * ========================= */
 
 static inline void addr_to_3b(uint32_t addr, uint8_t a[3])
 {
     a[0] = (addr >> 16) & 0xFF;
     a[1] = (addr >> 8)  & 0xFF;
     a[2] = (addr >> 0)  & 0xFF;
 }
 
 /* =========================
  * Raw single-chip helpers
  * ========================= */
 
 /*
  * Send WREN to one FRAM chip.
  * Must be called before every write transaction.
  */
 static int fram_wren(struct fram_dev *dev)
 {
     static uint8_t cmd[4] __aligned(32) = { 0x06 };
 
     sys_cache_data_flush_range(cmd, sizeof(cmd));
 
     struct spi_buf b = {
         .buf = cmd,
         .len = 1
     };
     struct spi_buf_set s = {
         .buffers = &b,
         .count = 1
     };
 
     return spi_write(dev->spi_bus, &dev->spi_cfg, &s);
 }
 
 /*
  * Raw write to one FRAM chip.
  * No verification here. Just pushes bytes.
  *
  * len must be <= FRAM_RW_MAX_LEN.
  */
 static int fram_write_fixed(struct fram_dev *dev,
                             uint32_t addr,
                             const uint8_t *data,
                             size_t len)
 {
     static uint8_t txbuf[4 + FRAM_RW_MAX_LEN] __aligned(32);
     uint8_t a[3];
 
     if (len > FRAM_RW_MAX_LEN) {
         return -EINVAL;
     }
 
     addr_to_3b(addr, a);
 
     txbuf[0] = 0x02;   /* WRITE */
     txbuf[1] = a[0];
     txbuf[2] = a[1];
     txbuf[3] = a[2];
     memcpy(&txbuf[4], data, len);
 
     sys_cache_data_flush_range(txbuf, 4 + len);
 
     struct spi_buf buf = {
         .buf = txbuf,
         .len = 4 + len
     };
     struct spi_buf_set tx = {
         .buffers = &buf,
         .count = 1
     };
 
     return spi_write(dev->spi_bus, &dev->spi_cfg, &tx);
 }
 
 /*
  * Raw read from one FRAM chip.
  * No voting, no CRC, no retry logic here.
  *
  * len must be <= FRAM_RW_MAX_LEN.
  */
 static int fram_read_fixed(struct fram_dev *dev,
                            uint32_t addr,
                            uint8_t *out,
                            size_t len)
 {
     static uint8_t tx[4 + FRAM_RW_MAX_LEN] __aligned(32);
     static uint8_t rx[4 + FRAM_RW_MAX_LEN] __aligned(32);
     uint8_t a[3];
 
     if (len > FRAM_RW_MAX_LEN) {
         return -EINVAL;
     }
 
     addr_to_3b(addr, a);
 
     memset(tx, 0x00, sizeof(tx));
     memset(rx, 0x00, sizeof(rx));
 
     tx[0] = 0x03;   /* READ */
     tx[1] = a[0];
     tx[2] = a[1];
     tx[3] = a[2];
 
     sys_cache_data_flush_range(tx, sizeof(tx));
     sys_cache_data_invd_range(rx, sizeof(rx));
 
     struct spi_buf txb = {
         .buf = tx,
         .len = 4 + len
     };
     struct spi_buf rxb = {
         .buf = rx,
         .len = 4 + len
     };
 
     struct spi_buf_set txs = {
         .buffers = &txb,
         .count = 1
     };
     struct spi_buf_set rxs = {
         .buffers = &rxb,
         .count = 1
     };
 
     int err = spi_transceive(dev->spi_bus, &dev->spi_cfg, &txs, &rxs);
     if (err) {
         return err;
     }
 
     sys_cache_data_invd_range(rx, sizeof(rx));
     memcpy(out, &rx[4], len);
 
     return 0;
 }
 
 /*
  * Optional chip ID read for one FRAM.
  * Handy for bring-up / sanity check.
  */
 #define FRAM_RDID_LEN 9
 
 static int fram_rdid(struct fram_dev *dev, uint8_t *out)
 {
     static uint8_t tx[1 + FRAM_RDID_LEN] __aligned(32);
     static uint8_t rx[1 + FRAM_RDID_LEN] __aligned(32);
 
     memset(tx, 0x00, sizeof(tx));
     memset(rx, 0x00, sizeof(rx));
 
     tx[0] = 0x9F;   /* RDID */
 
     sys_cache_data_flush_range(tx, sizeof(tx));
     sys_cache_data_invd_range(rx, sizeof(rx));
 
     struct spi_buf txb = {
         .buf = tx,
         .len = sizeof(tx)
     };
     struct spi_buf rxb = {
         .buf = rx,
         .len = sizeof(rx)
     };
 
     struct spi_buf_set txs = { .buffers = &txb, .count = 1 };
     struct spi_buf_set rxs = { .buffers = &rxb, .count = 1 };
 
     int err = spi_transceive(dev->spi_bus, &dev->spi_cfg, &txs, &rxs);
     if (err) {
         return err;
     }
 
     sys_cache_data_invd_range(rx, sizeof(rx));
     memcpy(out, &rx[1], FRAM_RDID_LEN);
 
     return 0;
 }
 
 /*
  * Verified write to one FRAM chip.
  *
  * This preserves the good part of your current code:
  * write a chunk, read it back, compare, retry if needed.
  */
 static int fram_write_verified(struct fram_dev *dev,
                                uint32_t addr,
                                const uint8_t *data,
                                size_t len)
 {
     if (len == 0) {
         return 0;
     }
 
     if (addr >= FRAM_SIZE || len > (FRAM_SIZE - addr)) {
         return -EINVAL;
     }
 
     uint8_t readback[FRAM_VERIFY_CHUNK_SZ];
 
     while (len > 0) {
         size_t chunk = (len < FRAM_VERIFY_CHUNK_SZ) ? len : FRAM_VERIFY_CHUNK_SZ;
         bool verified = false;
 
         for (int attempt = 0; attempt < FRAM_MAX_RETRIES; attempt++) {
             int err = fram_wren(dev);
             if (err) {
                 continue;
             }
 
             /* Tiny settle time between WREN and WRITE */
             k_busy_wait(1);
 
             err = fram_write_fixed(dev, addr, data, chunk);
             if (err) {
                 continue;
             }
 
             err = fram_read_fixed(dev, addr, readback, chunk);
             if (err) {
                 continue;
             }
 
             if (memcmp(data, readback, chunk) == 0) {
                 verified = true;
                 break;
             }
         }
 
         if (!verified) {
             return -EIO;
         }
 
         addr += chunk;
         data += chunk;
         len  -= chunk;
     }
 
     return 0;
 }
 
 /*
  * Plain read wrapper for one FRAM chip over multiple chunks.
  * Useful so higher layers do not need to care about the 256-byte raw limit.
  */
 static int fram_read(struct fram_dev *dev, uint32_t addr, uint8_t *out, size_t len)
 {
     if (len == 0) {
         return 0;
     }
 
     if (addr >= FRAM_SIZE || len > (FRAM_SIZE - addr)) {
         return -EINVAL;
     }
 
     while (len > 0) {
         size_t chunk = (len < FRAM_RW_MAX_LEN) ? len : FRAM_RW_MAX_LEN;
 
         int err = fram_read_fixed(dev, addr, out, chunk);
         if (err) {
             return err;
         }
 
         addr += chunk;
         out  += chunk;
         len  -= chunk;
     }
 
     return 0;
 }
 
 /* =========================
  * TMR voting helpers
  * ========================= */
 
 enum tmr_vote_result {
     TMR_VOTE_ALL_MATCH = 0,
     TMR_VOTE_AB_GOOD,
     TMR_VOTE_AC_GOOD,
     TMR_VOTE_BC_GOOD,
     TMR_VOTE_NO_MAJORITY,
 };
 
 static enum tmr_vote_result tmr_vote_whole_buffer(const uint8_t *a,
                                                   const uint8_t *b,
                                                   const uint8_t *c,
                                                   size_t len)
 {
     bool ab = (memcmp(a, b, len) == 0);
     bool ac = (memcmp(a, c, len) == 0);
     bool bc = (memcmp(b, c, len) == 0);
 
     if (ab && ac) {
         return TMR_VOTE_ALL_MATCH;
     }
     if (ab) {
         return TMR_VOTE_AB_GOOD;
     }
     if (ac) {
         return TMR_VOTE_AC_GOOD;
     }
     if (bc) {
         return TMR_VOTE_BC_GOOD;
     }
 
     return TMR_VOTE_NO_MAJORITY;
 }
 
 static const uint8_t *tmr_majority_buffer(const uint8_t *a,
                                           const uint8_t *b,
                                           const uint8_t *c,
                                           size_t len,
                                           enum tmr_vote_result *vote_out)
 {
     enum tmr_vote_result vote = tmr_vote_whole_buffer(a, b, c, len);
 
     if (vote_out) {
         *vote_out = vote;
     }
 
     switch (vote) {
     case TMR_VOTE_ALL_MATCH:
     case TMR_VOTE_AB_GOOD:
     case TMR_VOTE_AC_GOOD:
         return a;
 
     case TMR_VOTE_BC_GOOD:
         return b;
 
     case TMR_VOTE_NO_MAJORITY:
     default:
         return NULL;
     }
 }
 
 static int tmr_bad_index_from_vote(enum tmr_vote_result vote)
 {
     switch (vote) {
     case TMR_VOTE_AB_GOOD: return 2; /* C bad */
     case TMR_VOTE_AC_GOOD: return 1; /* B bad */
     case TMR_VOTE_BC_GOOD: return 0; /* A bad */
     default: return -1;
     }
 }
 
 /* =========================
  * Public TMR API
  * ========================= */
 
 /*
  * Write the same payload to all 3 FRAM chips.
  *
  * Strict policy:
  * all 3 must succeed or the call fails.
  */
 int tmr_fram_write(struct fram_dev *fram_a,
                    struct fram_dev *fram_b,
                    struct fram_dev *fram_c,
                    uint32_t addr,
                    const uint8_t *data,
                    size_t len)
 {
     if (!fram_a || !fram_b || !fram_c || !data) {
         return -EINVAL;
     }
 
     int err_a = fram_write_verified(fram_a, addr, data, len);
     int err_b = fram_write_verified(fram_b, addr, data, len);
     int err_c = fram_write_verified(fram_c, addr, data, len);
 
     if (err_a) {
         printk("TMR write failed on %s: %d\n", fram_a->name, err_a);
     }
     if (err_b) {
         printk("TMR write failed on %s: %d\n", fram_b->name, err_b);
     }
     if (err_c) {
         printk("TMR write failed on %s: %d\n", fram_c->name, err_c);
     }
 
     if (err_a || err_b || err_c) {
         return -EIO;
     }
 
     return 0;
 }
 
 /*
  * Read from all 3 FRAM chips, whole-buffer vote them,
  * and optionally repair the odd chip out.
  *
  * This starts simple:
  * - all 3 are read
  * - if 2 match, that copy wins
  * - if all 3 differ, fail
  *
  * Later you can upgrade this to CRC-backed records.
  */
 int tmr_fram_read(struct fram_dev *fram_a,
                   struct fram_dev *fram_b,
                   struct fram_dev *fram_c,
                   uint32_t addr,
                   uint8_t *out,
                   size_t len)
 {
     if (!fram_a || !fram_b || !fram_c || !out) {
         return -EINVAL;
     }
 
     uint8_t buf_a[TMR_READ_CHUNK_SZ];
     uint8_t buf_b[TMR_READ_CHUNK_SZ];
     uint8_t buf_c[TMR_READ_CHUNK_SZ];
 
     while (len > 0) {
         size_t chunk = (len < TMR_READ_CHUNK_SZ) ? len : TMR_READ_CHUNK_SZ;
 
         int err_a = fram_read(fram_a, addr, buf_a, chunk);
         int err_b = fram_read(fram_b, addr, buf_b, chunk);
         int err_c = fram_read(fram_c, addr, buf_c, chunk);
 
         if (err_a || err_b || err_c) {
             printk("TMR read raw failure at 0x%06x: A=%d B=%d C=%d\n",
                    (unsigned int)addr, err_a, err_b, err_c);
             return -EIO;
         }
 
         enum tmr_vote_result vote;
         const uint8_t *majority = tmr_majority_buffer(buf_a, buf_b, buf_c, chunk, &vote);
 
         if (majority == NULL) {
             printk("TMR read no majority at 0x%06x len=%u\n",
                    (unsigned int)addr, (unsigned int)chunk);
             return -EIO;
         }
 
         memcpy(out, majority, chunk);
 
 #if TMR_ENABLE_REPAIR
         int bad_idx = tmr_bad_index_from_vote(vote);
         if (bad_idx >= 0) {
             struct fram_dev *bad_dev =
                 (bad_idx == 0) ? fram_a :
                 (bad_idx == 1) ? fram_b : fram_c;
 
             printk("TMR: %s diverged at 0x%06x, repairing...\n",
                    bad_dev->name, (unsigned int)addr);

             int repair_err = fram_write_verified(bad_dev, addr, out, chunk);
             if (repair_err) {
                 printk("TMR: repair FAILED on %s at 0x%06x: %d\n",
                        bad_dev->name, (unsigned int)addr, repair_err);
             } else {
                 printk("TMR: %s repaired OK at 0x%06x\n",
                        bad_dev->name, (unsigned int)addr);
             }
         }
 #endif
 
         addr += chunk;
         out  += chunk;
         len  -= chunk;
     }
 
     return 0;
 }
 
 /*
  * Check whether all 3 FRAM chips contain exactly the same data
  * over a region. Good for bench testing and fault injection testing.
  */
 int tmr_fram_verify_all_equal(struct fram_dev *fram_a,
                               struct fram_dev *fram_b,
                               struct fram_dev *fram_c,
                               uint32_t addr,
                               size_t len)
 {
     if (!fram_a || !fram_b || !fram_c) {
         return -EINVAL;
     }
 
     uint8_t buf_a[TMR_READ_CHUNK_SZ];
     uint8_t buf_b[TMR_READ_CHUNK_SZ];
     uint8_t buf_c[TMR_READ_CHUNK_SZ];
 
     while (len > 0) {
         size_t chunk = (len < TMR_READ_CHUNK_SZ) ? len : TMR_READ_CHUNK_SZ;
 
         int err_a = fram_read(fram_a, addr, buf_a, chunk);
         int err_b = fram_read(fram_b, addr, buf_b, chunk);
         int err_c = fram_read(fram_c, addr, buf_c, chunk);
 
         if (err_a || err_b || err_c) {
             return -EIO;
         }
 
        if ((memcmp(buf_a, buf_b, chunk) != 0) ||
            (memcmp(buf_a, buf_c, chunk) != 0)) {
            return -EIO;
        }

        addr += chunk;
        len  -= chunk;
    }

    return 0;
}

#endif /* FRAM_TMR_H_ */