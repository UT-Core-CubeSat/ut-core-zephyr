#include <zephyr/kernel.h> 
#include <zephyr/device.h> 
#include <zephyr/drivers/spi.h> 
#include <zephyr/drivers/gpio.h> 
#include <zephyr/sys/printk.h> 
#include <string.h> 
#include <zephyr/cache.h> 
static uint32_t xorshift32_state = 0xDEADBEEF;

static uint32_t xorshift32(void) {
    uint32_t x = xorshift32_state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    xorshift32_state = x;
    return x;
}

static void rand_fill(uint8_t *buf, size_t len) {
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

#define SPI_BUS_NODE DT_NODELABEL(spi2) 
#define SCOPE_NODE DT_ALIAS(led0) 

/* Fixed address to hammer forever (0x00000..0x7FFFF valid) */ 
#define FRAM_ADDR 0x000100u 

static const struct device *spi_bus = DEVICE_DT_GET(SPI_BUS_NODE); 
static const struct gpio_dt_spec cs_gpio = GPIO_DT_SPEC_GET_BY_IDX(SPI_BUS_NODE, cs_gpios, 0); 
static const struct gpio_dt_spec scope_pin = GPIO_DT_SPEC_GET(SCOPE_NODE, gpios); 

/* MODE 0 @ 1 MHz: CPOL=0, CPHA=0 */ 
static struct spi_config spi_cfg = { 
    .frequency = 400000, 
    .operation = SPI_OP_MODE_MASTER | 
                 SPI_TRANSFER_MSB | 
                 SPI_WORD_SET(8),
                 //SPI_MODE_CPOL | SPI_MODE_CPHA, /* MODE 3 */ 
                 .slave = 0, 
                 .cs = { 
                    .gpio = cs_gpio, 
                    .delay = 1, /* us */ 
                    .cs_is_gpio = true, 
                }, 
}; 

static inline void addr_to_3b(uint32_t addr, uint8_t a[3]) { 
    a[0] = (addr >> 16) & 0xFF; 
    a[1] = (addr >> 8) & 0xFF; 
    a[2] = (addr >> 0) & 0xFF; 
} 

static int fram_wren(void) { 
    static uint8_t cmd[4] __aligned(32) = { 0x06 };
    sys_cache_data_flush_range(cmd, sizeof(cmd));

    struct spi_buf b = { .buf = cmd, .len = 1 };
    struct spi_buf_set s = { .buffers = &b, .count = 1 };

    return spi_write(spi_bus, &spi_cfg, &s); 
}

static int fram_write_fixed(uint32_t addr, const uint8_t *data, size_t len) { 
    static uint8_t txbuf[4 + 256] __aligned(32);
    uint8_t a[3];

    addr_to_3b(addr, a); 

    txbuf[0] = 0x02;
    txbuf[1] = a[0];
    txbuf[2] = a[1];
    txbuf[3] = a[2];
    memcpy(&txbuf[4], data, len); 

    sys_cache_data_flush_range(txbuf, 4 + len); 

    struct spi_buf buf = { .buf = txbuf, .len = 4 + len };
    struct spi_buf_set tx = { .buffers = &buf, .count = 1 }; 

    return spi_write(spi_bus, &spi_cfg, &tx); 
} 

static int fram_read_fixed(uint32_t addr, uint8_t *out, size_t len) { 
    static uint8_t tx[4 + 256] __aligned(32); 
    static uint8_t rx[4 + 256] __aligned(32); 
    uint8_t a[3];

    if (len > 256) return -EINVAL; 

    /* Convert the 32-bit address into three individual bytes (MSB first) */
    addr_to_3b(addr, a); 

    /* Zero both buffers — TX bytes after the header must be 0x00 so the
     * master clocks out dummy bytes while the FRAM drives data on MISO.
     * RX is cleared so any stale data doesn't contaminate the result. */
    memset(tx, 0x00, sizeof(tx)); 
    memset(rx, 0x00, sizeof(rx)); 

    /* Build the 4-byte command header in the TX buffer:
     * 0x03 = FRAM READ opcode, followed by 3 address bytes [A23..A0] */
    tx[0] = 0x03; /* FRAM READ opcode */
    tx[1] = a[0]; /* Address bits [23:16] (most significant) */
    tx[2] = a[1]; /* Address bits [15:8] */
    tx[3] = a[2]; /* Address bits [7:0]  (least significant) */
    /* tx[4..4+len-1] remain 0x00 — these are dummy bytes clocked out while
     * the FRAM responds with real data on MISO during those same cycles */
    
    /* Flush the TX buffer from CPU cache to main memory so the DMA
     * peripheral reads the correct command bytes we just wrote */
    sys_cache_data_flush_range(tx, sizeof(tx)); 
    /* Invalidate the RX buffer's cache lines so the CPU won't serve stale
     * cached data after DMA writes fresh bytes directly to RAM */
    sys_cache_data_invd_range(rx, sizeof(rx)); 

    /* TX SPI buffer: we send 4 header bytes + len dummy bytes.
     * The total length must match RX so the clock runs long enough
     * for the FRAM to shift out all requested data bytes. */
    struct spi_buf txb = { 
        .buf = tx, 
        .len = 4 + len 
    }; 

    /* RX SPI buffer: captures everything the FRAM sends back.
     * The first 4 bytes are garbage (FRAM outputs nothing useful while
     * receiving the opcode + address). Real data starts at rx[4]. */
    struct spi_buf rxb = { 
        .buf = rx, 
        .len = 4 + len 
    }; 

    /* Wrap TX and RX into buffer sets required by the Zephyr SPI API */
    struct spi_buf_set txs = { 
        .buffers = &txb, 
        .count = 1 
    }; 
    struct spi_buf_set rxs = { 
        .buffers = &rxb, 
        .count = 1 
    }; 

    /* Perform a full-duplex SPI transaction: TX (command + dummies) is
     * clocked out on MOSI while RX simultaneously captures MISO.
     * CS is asserted for the entire transaction, then de-asserted. */
    int err = spi_transceive(spi_bus, &spi_cfg, &txs, &rxs); 
    if (err) return err; 

    /* Invalidate the RX cache lines again — DMA has now written fresh data
     * to RAM, and the CPU cache may still hold the pre-transfer zeros.
     * This forces the CPU to re-read from RAM on the next access. */
    sys_cache_data_invd_range(rx, sizeof(rx)); 

    /* Copy only the data portion (skipping the 4-byte header region where
     * the FRAM hadn't started responding yet) into the caller's buffer */
    memcpy(out, &rx[4], len); 
    return 0; 
} 

/* RDID returns 9 bytes: 6x continuation code (0x7F), 1x manufacturer (0xC2),
 * and 2 bytes of product ID (family, density, sub, revision, etc.) */
#define FRAM_RDID_LEN 9

static int fram_rdid(uint8_t *out) {
    /* Static TX and RX buffers, 32-byte aligned for DMA cache-line safety.
     * Sized for 1-byte opcode + 9 bytes of device ID response = 10 total. */
    static uint8_t tx[1 + FRAM_RDID_LEN] __aligned(32);
    static uint8_t rx[1 + FRAM_RDID_LEN] __aligned(32);

    /* Zero both buffers — TX bytes after the opcode must be 0x00 (dummy bytes
     * clocked out on MOSI while the FRAM drives device ID data on MISO).
     * RX is cleared so stale data from a previous call can't leak through. */
    memset(tx, 0x00, sizeof(tx));
    memset(rx, 0x00, sizeof(rx));

    /* First byte is the RDID opcode (0x9F). No address bytes are needed —
     * the FRAM begins shifting out device ID immediately after the opcode. */
    tx[0] = 0x9F;

    /* Flush TX from CPU cache to main memory so DMA reads the correct opcode */
    sys_cache_data_flush_range(tx, sizeof(tx));
    /* Invalidate RX cache lines so the CPU won't serve stale cached zeros
     * after DMA writes the fresh device ID bytes directly to RAM */
    sys_cache_data_invd_range(rx, sizeof(rx));

    /* TX SPI buffer: 1 opcode byte + 9 dummy bytes = 10 bytes total.
     * The clock must run for all 10 bytes so the FRAM has enough cycles
     * to shift out every device ID byte on MISO. */
    struct spi_buf txb = {
        .buf = tx,
        .len = sizeof(tx)
    };

    /* RX SPI buffer: captures everything on MISO during the 10-byte transfer.
     * rx[0] is garbage (FRAM outputs nothing useful while receiving the opcode).
     * rx[1..9] contain the 9 device ID bytes. */
    struct spi_buf rxb = {
        .buf = rx,
        .len = sizeof(rx)
    };

    /* Wrap into buffer sets for the Zephyr SPI API */
    struct spi_buf_set txs = { .buffers = &txb, .count = 1 };
    struct spi_buf_set rxs = { .buffers = &rxb, .count = 1 };

    /* Full-duplex SPI transaction: opcode + dummies clocked out on MOSI
     * while MISO is simultaneously captured into rx[]. */
    int err = spi_transceive(spi_bus, &spi_cfg, &txs, &rxs);
    if (err) return err;

    /* Invalidate RX cache again — DMA has written fresh data to RAM and
     * the CPU cache may still hold the pre-transfer zeros. */
    sys_cache_data_invd_range(rx, sizeof(rx));

    /* Copy the 9 device ID bytes (skip rx[0] which was received while
     * the opcode was being sent — FRAM hadn't started responding yet) */
    memcpy(out, &rx[1], FRAM_RDID_LEN);
    return 0;
}

#define FRAM_SIZE        0x80000u   /* 512 KB */
#define FRAM_CHUNK_SZ    16u
#define FRAM_MAX_RETRIES 5

static int fram_write(uint32_t addr, const uint8_t *data, size_t len)
{
    if (len == 0) {
        return 0;
    }
    if (addr >= FRAM_SIZE || len > FRAM_SIZE - addr) {
        return -EINVAL;
    }

    uint8_t readback[FRAM_CHUNK_SZ];

    while (len > 0) {
        size_t chunk = (len < FRAM_CHUNK_SZ) ? len : FRAM_CHUNK_SZ;
        bool verified = false;

        for (int try = 0; try < FRAM_MAX_RETRIES; try++) {
            int err = fram_wren();
            if (err) {
                continue;
            }
            k_busy_wait(1);

            err = fram_write_fixed(addr, data, chunk);
            if (err) {
                continue;
            }

            err = fram_read_fixed(addr, readback, chunk);
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

int main(void) { 

    k_busy_wait(1000); 

    if (!device_is_ready(spi_bus) || !gpio_is_ready_dt(&cs_gpio)) { 
        printf("SPI bus or CS GPIO not ready\n"); 
        return 0; 
    } 

    if (!gpio_is_ready_dt(&scope_pin)) { 
        printf("Scope GPIO not ready\n"); 
        return 0; 
    } 

    gpio_pin_configure_dt(&scope_pin, GPIO_OUTPUT_INACTIVE); 

    #define PAYLOAD_LEN 32

    printf("FRAM random write/read addr=0x%06X len=%d\n",
           (unsigned)FRAM_ADDR, PAYLOAD_LEN);

    uint8_t w[PAYLOAD_LEN];
    uint32_t total = 0;
    uint32_t ok = 0;
    uint32_t failed = 0;

    while (1) { 
        rand_fill(w, sizeof(w));

        gpio_pin_set_dt(&scope_pin, 1); 
        int err = fram_write(FRAM_ADDR, w, sizeof(w)); 
        gpio_pin_set_dt(&scope_pin, 0); 

        total++;

        if (err) {
            failed++;
            printf("FAILED #%u err=%d\n", total, err);
        } else {
            ok++;
        }

        if (total % 100 == 0) {
            uint32_t rate = ok * 10000 / total;
            printf("--- stats: %u total | %u ok | %u failed (%u.%02u%% pass) ---\n",
                   total, ok, failed,
                   rate / 100, rate % 100);
        }

        k_msleep(10); 
    } 
}