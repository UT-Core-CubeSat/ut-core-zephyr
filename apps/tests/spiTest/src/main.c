/*
 * CY15B104QN / CY15V104QN FRAM bring-up test on SPI2.
 * - RDID (0x9F) should return 9 bytes
 * - RDSR (0x05) should return status byte
 * - WREN (0x06) then RDSR should show WEL=1
 * - WRITE (0x02) then READ (0x03) verify data
 *
 * Datasheet highlights:
 *  - SPI mode 0 or 3 supported
 *  - RDID = 0x9F (9 bytes), RDSR=0x05, WREN=0x06, WRITE=0x02, READ=0x03
 *  - 3-byte address (19 bits used)
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <string.h>

#define SPI_BUS_NODE DT_NODELABEL(spi2)

static const struct device *spi_bus = DEVICE_DT_GET(SPI_BUS_NODE);
static const struct gpio_dt_spec cs_gpio = GPIO_DT_SPEC_GET_BY_IDX(SPI_BUS_NODE, cs_gpios, 0);

static struct spi_config spi_cfg = {
    .frequency = 20000000, /* keep slow for bring-up */
    .operation = SPI_OP_MODE_MASTER |
                 SPI_TRANSFER_MSB   |
                 SPI_WORD_SET(8)    |
                 SPI_MODE_CPOL      |
                 SPI_MODE_CPHA,     /* Mode 3 */
    .slave = 0,
    .cs = {
        .gpio = cs_gpio,
        .delay = 0,
        .cs_is_gpio = true,
    },
};

/* ===== Helpers ===== */

static int spi_txrx(const uint8_t *tx, size_t tx_len, uint8_t *rx, size_t rx_len)
{
    struct spi_buf tx_bufs[1] = { { .buf = (void *)tx, .len = tx_len } };
    struct spi_buf_set tx_set = { .buffers = tx_bufs, .count = 1 };

    struct spi_buf rx_bufs[1] = { { .buf = rx, .len = rx_len } };
    struct spi_buf_set rx_set = { .buffers = rx_bufs, .count = 1 };

    return spi_transceive(spi_bus, &spi_cfg, &tx_set, &rx_set);
}

/* Send opcode only (no readback) */
static int fram_cmd(uint8_t opcode)
{
    uint8_t tx[1] = { opcode };
    return spi_write(spi_bus, &spi_cfg, &(struct spi_buf_set){
        .buffers = (struct spi_buf[]){{ .buf = tx, .len = sizeof(tx) }},
        .count = 1
    });
}

static int fram_rdsr(uint8_t *sr_out)
{
    uint8_t tx[2] = { 0x05, 0x00 }; /* RDSR + dummy */
    uint8_t rx[2] = { 0 };

    int rc = spi_txrx(tx, sizeof(tx), rx, sizeof(rx));
    if (rc == 0) {
        *sr_out = rx[1];
    }
    return rc;
}

static int fram_rdid(uint8_t id_out[9])
{
    uint8_t tx[1 + 9];
    uint8_t rx[1 + 9];
    memset(tx, 0, sizeof(tx));
    memset(rx, 0, sizeof(rx));

    tx[0] = 0x9F; /* RDID */
    /* remaining bytes are dummy clocks to shift out the 9-byte ID */

    int rc = spi_txrx(tx, sizeof(tx), rx, sizeof(rx));
    if (rc == 0) {
        memcpy(id_out, &rx[1], 9);
    }
    return rc;
}

static int fram_write(uint32_t addr, const uint8_t *data, size_t len)
{
    /* WRITE (0x02) + 3-byte address + payload */
    uint8_t hdr[4];
    hdr[0] = 0x02;
    hdr[1] = (addr >> 16) & 0xFF;
    hdr[2] = (addr >> 8)  & 0xFF;
    hdr[3] = (addr >> 0)  & 0xFF;

    struct spi_buf bufs[2] = {
        { .buf = hdr, .len = sizeof(hdr) },
        { .buf = (void *)data, .len = len },
    };
    struct spi_buf_set tx_set = { .buffers = bufs, .count = 2 };

    return spi_write(spi_bus, &spi_cfg, &tx_set);
}

static int fram_read(uint32_t addr, uint8_t *data, size_t len)
{
    /* READ (0x03) + 3-byte address + read bytes */
    uint8_t hdr[4];
    hdr[0] = 0x03;
    hdr[1] = (addr >> 16) & 0xFF;
    hdr[2] = (addr >> 8)  & 0xFF;
    hdr[3] = (addr >> 0)  & 0xFF;

    /* tx = header + dummy bytes; rx will contain garbage for header then data */
    size_t total = sizeof(hdr) + len;

    uint8_t tx[4 + 64];
    uint8_t rx[4 + 64];
    if (len > 64) return -EINVAL;

    memset(tx, 0, sizeof(tx));
    memset(rx, 0, sizeof(rx));
    memcpy(tx, hdr, sizeof(hdr));

    int rc = spi_txrx(tx, total, rx, total);
    if (rc == 0) {
        memcpy(data, &rx[sizeof(hdr)], len);
    }
    return rc;
}

/* ===== Main ===== */

int main(void)
{
    printk("\nFRAM bring-up test (CY15x104QN)\n");

    if (!device_is_ready(spi_bus)) {
        printk("SPI bus not ready\n");
        return 0;
    }
    if (!gpio_is_ready_dt(&cs_gpio)) {
        printk("CS GPIO not ready\n");
        return 0;
    }
    printk("CS from DT: port=%s pin=%d\n", cs_gpio.port->name, cs_gpio.pin);

    /* 1) RDID */
    uint8_t id[9] = {0};
    int rc = fram_rdid(id);
    printk("RDID rc=%d  ID:", rc);
    for (int i = 0; i < 9; i++) printk(" %02X", id[i]);
    printk("\n");

    /* 2) RDSR */
    uint8_t sr = 0;
    rc = fram_rdsr(&sr);
    printk("RDSR rc=%d  SR=0x%02X\n", rc, sr);

    /* 3) WREN then confirm WEL bit */
    rc = fram_cmd(0x06);
    printk("WREN rc=%d\n", rc);

    rc = fram_rdsr(&sr);
    printk("RDSR(after WREN) rc=%d  SR=0x%02X  (WEL=%d)\n", rc, sr, (sr >> 1) & 0x1);

    /* 4) Write + readback */
    const uint32_t addr = 0x000123;
    uint8_t tx_data[8] = { 'U','T','-','C','O','R','E','!' };
    uint8_t rx_data[8] = {0};

    rc = fram_write(addr, tx_data, sizeof(tx_data));
    printk("WRITE rc=%d (addr=0x%06X)\n", rc, addr);

    rc = fram_read(addr, rx_data, sizeof(rx_data));
    printk("READ  rc=%d (addr=0x%06X)  RX:", rc, addr);
    for (int i = 0; i < 8; i++) printk(" %02X", rx_data[i]);
    printk("  |ASCII: ");
    for (int i = 0; i < 8; i++) printk("%c", (rx_data[i] >= 32 && rx_data[i] <= 126) ? rx_data[i] : '.');
    printk("\n");

    while (1) {
        k_msleep(2000);
    }
}
