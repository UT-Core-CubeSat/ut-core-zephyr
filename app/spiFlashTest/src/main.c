/*
 * CY15x104QN FRAM diagnostic on SPI2 (Zephyr / STM32U5)
 *
 * Key changes vs your original:
 *  - WRITE uses spi_transceive (full duplex) so CS handling is identical to your working cmds
 *  - Drop speed to 1 MHz for stability (ramp later)
 *  - Read SR twice (checks for bit flips)
 *  - Read data twice (checks for transfer corruption)
 *
 * IMPORTANT build-side fix (recommended):
 *  - Disable SPI DMA for STM32U5 (see prj.conf below)
 */

 #include <zephyr/kernel.h>
 #include <zephyr/device.h>
 #include <zephyr/drivers/spi.h>
 #include <zephyr/drivers/gpio.h>
 #include <zephyr/sys/printk.h>
 #include <string.h>
 #include <errno.h>
 
 #define SPI_BUS_NODE    DT_NODELABEL(spi2)
 
 #define TEST_BUF_SIZE   4096
 
 /* WP pin: PB5 (optional) */
 #define WP_PORT_NODE    DT_NODELABEL(gpiob)
 #define WP_PIN          5
 
 /* Bring-up speed (raise later) */
 #define FRAM_SPI_HZ     1000000U
 
 static const struct device *spi_bus = DEVICE_DT_GET(SPI_BUS_NODE);
 static const struct device *wp_port = DEVICE_DT_GET(WP_PORT_NODE);
 
 static const struct gpio_dt_spec cs_gpio =
     GPIO_DT_SPEC_GET_BY_IDX(SPI_BUS_NODE, cs_gpios, 0);
 
 static struct spi_config spi_cfg = {
     .frequency = FRAM_SPI_HZ,
     .operation = SPI_OP_MODE_MASTER |
                  SPI_TRANSFER_MSB   |
                  SPI_WORD_SET(8)    |
                  SPI_MODE_CPOL      |
                  SPI_MODE_CPHA,       /* Mode 3 */
     .slave = 0,
     .cs = {
         .gpio = cs_gpio,
         .delay = 0,
         .cs_is_gpio = true,
     },
 };
 
 static uint8_t tx_buf[TEST_BUF_SIZE];
 static uint8_t rx_buf[TEST_BUF_SIZE];
 static uint8_t rx2_buf[TEST_BUF_SIZE];
 
 static uint8_t wr_tx[4 + TEST_BUF_SIZE];
 static uint8_t wr_rx[4 + TEST_BUF_SIZE];   /* dummy RX for full-duplex write */
 
 static uint8_t rd_tx[4 + TEST_BUF_SIZE];
 static uint8_t rd_rx[4 + TEST_BUF_SIZE];
 
 static int spi_txrx(const uint8_t *tx, size_t n, uint8_t *rx, size_t rn)
 {
     struct spi_buf tb = { .buf = (void *)tx, .len = n };
     struct spi_buf_set ts = { .buffers = &tb, .count = 1 };
 
     struct spi_buf rb = { .buf = rx, .len = rn };
     struct spi_buf_set rs = { .buffers = &rb, .count = 1 };
 
     int rc = spi_transceive(spi_bus, &spi_cfg, &ts, &rs);
 
     /* Force CS high / release bus */
     spi_release(spi_bus, &spi_cfg);
 
     /* Small guard time (helps if driver deassert is edgy) */
     k_busy_wait(2);
 
     return rc;
 }
 
 static int fram_cmd(uint8_t opcode)
 {
     uint8_t tx[1] = { opcode };
     uint8_t rx[1] = { 0 };
     return spi_txrx(tx, sizeof(tx), rx, sizeof(rx));
 }
 
 static int fram_rdsr(uint8_t *sr_out)
 {
     uint8_t tx[2] = { 0x05, 0x00 };
     uint8_t rx[2] = { 0 };
 
     int rc = spi_txrx(tx, sizeof(tx), rx, sizeof(rx));
     if (rc == 0) {
         *sr_out = rx[1];
     }
     return rc;
 }
 
 static int fram_rdid(uint8_t id_out[9])
 {
     uint8_t tx[10] = { 0 };
     uint8_t rx[10] = { 0 };
     tx[0] = 0x9F;
 
     int rc = spi_txrx(tx, sizeof(tx), rx, sizeof(rx));
     if (rc == 0) {
         memcpy(id_out, &rx[1], 9);
     }
     return rc;
 }
 
 /* WREN, and read SR twice to confirm stability */
 static int fram_wren(void)
 {
     int rc = fram_cmd(0x06);
     if (rc) {
         printk("  WREN SPI error: rc=%d\n", rc);
         return rc;
     }
 
     uint8_t sr1 = 0, sr2 = 0;
     rc = fram_rdsr(&sr1);
     if (rc) return rc;
     rc = fram_rdsr(&sr2);
     if (rc) return rc;
 
     printk("  SR after WREN: 0x%02X / 0x%02X\n", sr1, sr2);
 
     if (!(sr1 & 0x02) || !(sr2 & 0x02)) {
         printk("  WEL not set (or unstable) after WREN!\n");
         return -EIO;
     }
 
     return 0;
 }
 
 /*
  * WRITE FIX: use transceive, not spi_write.
  * This removes “write-only CS weirdness” as a variable.
  */
 static int fram_write(uint32_t addr, const uint8_t *data, size_t len)
 {
     if (len > TEST_BUF_SIZE) return -EINVAL;
 
     size_t total = 4 + len;
     wr_tx[0] = 0x02;
     wr_tx[1] = (addr >> 16) & 0xFF;
     wr_tx[2] = (addr >> 8)  & 0xFF;
     wr_tx[3] = (addr)       & 0xFF;
     memcpy(&wr_tx[4], data, len);
 
     memset(wr_rx, 0, total);
 
     return spi_txrx(wr_tx, total, wr_rx, total);
 }
 
 static int fram_read(uint32_t addr, uint8_t *data, size_t len)
 {
     if (len > TEST_BUF_SIZE) return -EINVAL;
 
     size_t total = 4 + len;
 
     memset(rd_tx, 0, total);
     memset(rd_rx, 0xCC, total);
 
     rd_tx[0] = 0x03;
     rd_tx[1] = (addr >> 16) & 0xFF;
     rd_tx[2] = (addr >> 8)  & 0xFF;
     rd_tx[3] = (addr)       & 0xFF;
 
     int rc = spi_txrx(rd_tx, total, rd_rx, total);
     if (rc == 0) {
         memcpy(data, &rd_rx[4], len);
     }
     return rc;
 }
 
 int main(void)
 {
     printk("\n========================================\n");
     printk("FRAM Diagnostic (CY15x104QN) @ %u Hz\n", (unsigned)FRAM_SPI_HZ);
     printk("========================================\n\n");
 
     if (!device_is_ready(spi_bus)) {
         printk("ERROR: SPI bus not ready\n");
         return 0;
     }
     if (!gpio_is_ready_dt(&cs_gpio)) {
         printk("ERROR: CS GPIO not ready\n");
         return 0;
     }
 
     printk("SPI bus ready. CS: port=%s pin=%d\n", cs_gpio.port->name, cs_gpio.pin);
 
     if (device_is_ready(wp_port)) {
         gpio_pin_configure(wp_port, WP_PIN, GPIO_OUTPUT_HIGH);
         printk("WP pin (PB5) driven HIGH\n\n");
     }
 
     /* RDID */
     printk("[1] RDID\n");
     uint8_t id[9] = {0};
     int rc = fram_rdid(id);
     printk("  rc=%d ID:", rc);
     for (int i = 0; i < 9; i++) printk(" %02X", id[i]);
     printk("\n\n");
 
     /* RDSR twice */
     printk("[2] RDSR\n");
     uint8_t sr1=0, sr2=0;
     rc = fram_rdsr(&sr1);
     rc |= fram_rdsr(&sr2);
     printk("  rc=%d SR1=0x%02X SR2=0x%02X (WEL=%d/%d)\n\n",
            rc, sr1, sr2, (sr1>>1)&1, (sr2>>1)&1);
 
     /* Write/read diagnostic */
     const uint32_t addr = 0x002000;
     const size_t len = 16;
 
     printk("[3] DIAGNOSTIC: write %u bytes @ 0x%06X\n", (unsigned)len, addr);
 
     for (size_t i = 0; i < len; i++) tx_buf[i] = (uint8_t)(0xA0 + i);
 
     printk("  TX:");
     for (size_t i = 0; i < len; i++) printk(" %02X", tx_buf[i]);
     printk("\n");
 
     rc = fram_wren();
     if (rc) {
         printk("  WREN FAILED: rc=%d\n", rc);
         goto done;
     }
 
     rc = fram_write(addr, tx_buf, len);
     printk("  WRITE rc=%d\n", rc);
 
     /* Read SR twice after write */
     sr1 = sr2 = 0;
     fram_rdsr(&sr1);
     fram_rdsr(&sr2);
     printk("  SR after WRITE: 0x%02X / 0x%02X (WEL=%d/%d)\n",
            sr1, sr2, (sr1>>1)&1, (sr2>>1)&1);
 
     /* Read twice */
     memset(rx_buf,  0xCC, len);
     memset(rx2_buf, 0xDD, len);
 
     rc = fram_read(addr, rx_buf, len);
     printk("  READ1 rc=%d\n", rc);
 
     rc = fram_read(addr, rx2_buf, len);
     printk("  READ2 rc=%d\n", rc);
 
     printk("  RX1:");
     for (size_t i = 0; i < len; i++) printk(" %02X", rx_buf[i]);
     printk("\n");
 
     printk("  RX2:");
     for (size_t i = 0; i < len; i++) printk(" %02X", rx2_buf[i]);
     printk("\n");
 
     printk("  MATCH1: %s\n", (memcmp(tx_buf, rx_buf,  len) == 0) ? "YES" : "NO");
     printk("  MATCH2: %s\n", (memcmp(tx_buf, rx2_buf, len) == 0) ? "YES" : "NO");
     printk("  RX1==RX2: %s\n", (memcmp(rx_buf, rx2_buf, len) == 0) ? "YES" : "NO");
 
 done:
     printk("\n========================================\n");
     printk("Done\n");
     printk("========================================\n");
 
     while (1) k_msleep(2000);
 }
 