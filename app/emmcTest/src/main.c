/*
 * GPIO bit-bang eMMC driver for STM32U5A5
 *
 * Implements the JEDEC eMMC protocol entirely through GPIO toggling,
 * operating in 1-bit data bus mode at ~100 kHz clock (~12 KB/s).
 *
 * Hardware connections:
 *   CLK      = PC10   (output  — host-generated clock)
 *   CMD      = PA15   (bidir   — command / response)
 *   DAT0     = PC2    (bidir   — 1-bit data)
 *   DAT1–7   = PC3–9  (unused in 1-bit mode)
 *   RST_n    = PA7    (output  — hardware reset, active low)
 *
 * DTS requirement: &gpioa and &gpioc must have status = "okay".
 *                  No overlay or special nodes needed.
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <string.h>

/* =====================================================================
 * Pin definitions
 * ===================================================================== */

static const struct device *port_a;
static const struct device *port_c;

#define PIN_CLK   10   /* PC10 */
#define PIN_CMD   15   /* PA15 */
#define PIN_DAT0   2   /* PC2  */
#define PIN_RST    7   /* PA7  — eMMC RST_n (active low) */

/*
 * Half-period of the bit-bang clock in microseconds.
 * 5 µs → 100 kHz → ~12 KB/s in 1-bit mode.
 * Decrease for higher throughput (stay ≤400 kHz during identification).
 */
#define T_HALF  5

/* Card state filled during init */
static uint16_t card_rca;
static bool     sector_addr;

/* =====================================================================
 * CRC-7  (command / response frames)
 * Generator: x^7 + x^3 + 1   →  0x09
 * ===================================================================== */

static uint8_t crc7(const uint8_t *buf, int nbits)
{
	uint8_t crc = 0;
	for (int i = 0; i < nbits; i++) {
		uint8_t bit = (buf[i >> 3] >> (7 - (i & 7))) & 1;
		uint8_t fb  = ((crc >> 6) ^ bit) & 1;
		crc = (crc << 1) & 0x7F;
		if (fb)
			crc ^= 0x09;
	}
	return crc;
}

/* =====================================================================
 * CRC-16 CCITT  (data blocks)
 * Generator: x^16 + x^12 + x^5 + 1   →  0x1021
 * ===================================================================== */

static uint16_t crc16(const uint8_t *buf, int nbytes)
{
	uint16_t crc = 0;
	for (int i = 0; i < nbytes; i++) {
		for (int b = 7; b >= 0; b--) {
			uint16_t fb = ((crc >> 15) ^ ((buf[i] >> b) & 1)) & 1;
			crc = (crc << 1) & 0xFFFF;
			if (fb)
				crc ^= 0x1021;
		}
	}
	return crc;
}

/* =====================================================================
 * Low-level GPIO helpers
 * ===================================================================== */

static inline void clk_lo(void) { gpio_pin_set(port_c, PIN_CLK, 0); }
static inline void clk_hi(void) { gpio_pin_set(port_c, PIN_CLK, 1); }
static inline void hperiod(void) { k_busy_wait(T_HALF); }

static void idle_clocks(int n)
{
	for (int i = 0; i < n; i++) {
		clk_lo(); hperiod();
		clk_hi(); hperiod();
	}
}

static void cmd_out(void)
{
	gpio_pin_configure(port_a, PIN_CMD, GPIO_OUTPUT_HIGH);
}

static void cmd_in(void)
{
	gpio_pin_configure(port_a, PIN_CMD, GPIO_INPUT | GPIO_PULL_UP);
}

static void dat0_out(void)
{
	gpio_pin_configure(port_c, PIN_DAT0, GPIO_OUTPUT_HIGH);
}

static void dat0_in(void)
{
	gpio_pin_configure(port_c, PIN_DAT0, GPIO_INPUT | GPIO_PULL_UP);
}

/* =====================================================================
 * Command / response layer
 * ===================================================================== */

/*
 * Build and clock out a 48-bit command frame on CMD.
 * Frame: start(0) | trans(1) | index(6) | arg(32) | crc7(7) | end(1)
 */
static void send_command(uint8_t idx, uint32_t arg)
{
	uint8_t f[6];
	f[0] = 0x40 | (idx & 0x3F);
	f[1] = (arg >> 24) & 0xFF;
	f[2] = (arg >> 16) & 0xFF;
	f[3] = (arg >>  8) & 0xFF;
	f[4] = arg & 0xFF;
	f[5] = (crc7(f, 40) << 1) | 0x01;

	cmd_out();
	for (int i = 0; i < 48; i++) {
		clk_lo();
		gpio_pin_set(port_a, PIN_CMD,
			     (f[i >> 3] >> (7 - (i & 7))) & 1);
		hperiod();
		clk_hi();
		hperiod();
	}
}

/*
 * Receive a 48-bit response (R1 / R3) on CMD.
 * Extracts the 32-bit payload (card-status for R1, OCR for R3).
 */
static int recv_r48(uint32_t *payload)
{
	cmd_in();

	/* Wait for start bit (CMD pulled low by card), Ncr ≤ 64 clocks */
	for (int t = 0; t < 256; t++) {
		clk_lo(); hperiod();
		clk_hi(); hperiod();
		if (gpio_pin_get(port_a, PIN_CMD) == 0)
			goto start;
	}
	return -ETIMEDOUT;
start:
	;
	uint64_t bits = 0;
	for (int i = 0; i < 47; i++) {
		clk_lo(); hperiod();
		clk_hi(); hperiod();
		bits = (bits << 1) | (gpio_pin_get(port_a, PIN_CMD) & 1);
	}
	/* [46]=trans [45:40]=idx [39:8]=payload [7:1]=crc [0]=end */
	*payload = (uint32_t)((bits >> 8) & 0xFFFFFFFF);
	return 0;
}

/*
 * Receive a 136-bit R2 response (CMD2 / CMD9).
 * Stores 16 bytes of CID/CSD in out[].
 */
static int recv_r136(uint8_t *out)
{
	cmd_in();

	for (int t = 0; t < 256; t++) {
		clk_lo(); hperiod();
		clk_hi(); hperiod();
		if (gpio_pin_get(port_a, PIN_CMD) == 0)
			goto start;
	}
	return -ETIMEDOUT;
start:
	/* 6 reserved bits (0b111111) */
	for (int i = 0; i < 6; i++) {
		clk_lo(); hperiod();
		clk_hi(); hperiod();
	}

	/* 128 bits of CID / CSD content */
	memset(out, 0, 16);
	for (int i = 0; i < 128; i++) {
		clk_lo(); hperiod();
		clk_hi(); hperiod();
		if (gpio_pin_get(port_a, PIN_CMD))
			out[i >> 3] |= 1 << (7 - (i & 7));
	}

	/* end bit */
	clk_lo(); hperiod();
	clk_hi(); hperiod();
	return 0;
}

/* Clock DAT0 until the card releases it (not busy). */
static int wait_dat0_ready(int max_ms)
{
	dat0_in();
	int64_t deadline = k_uptime_get() + max_ms;
	while (k_uptime_get() < deadline) {
		clk_lo(); hperiod();
		clk_hi(); hperiod();
		if (gpio_pin_get(port_c, PIN_DAT0))
			return 0;
	}
	return -ETIMEDOUT;
}

/* ---- Convenience wrappers ---- */

static int cmd_r1(uint8_t idx, uint32_t arg, uint32_t *st)
{
	send_command(idx, arg);
	int rc = recv_r48(st);
	idle_clocks(8);
	return rc;
}

static int cmd_r1b(uint8_t idx, uint32_t arg, uint32_t *st)
{
	send_command(idx, arg);
	int rc = recv_r48(st);
	if (rc == 0)
		rc = wait_dat0_ready(1000);
	idle_clocks(8);
	return rc;
}

static int cmd_r3(uint8_t idx, uint32_t arg, uint32_t *ocr)
{
	send_command(idx, arg);
	int rc = recv_r48(ocr);
	idle_clocks(8);
	return rc;
}

static int cmd_r2(uint8_t idx, uint32_t arg, uint8_t *out)
{
	send_command(idx, arg);
	int rc = recv_r136(out);
	idle_clocks(8);
	return rc;
}

static void cmd_none(uint8_t idx, uint32_t arg)
{
	send_command(idx, arg);
	cmd_in();
	idle_clocks(8);
}

/* =====================================================================
 * Card initialisation
 *
 * CMD0  → idle
 * CMD1  → poll until ready (sector-mode, 2.7-3.6 V, 1.7-1.95 V)
 * CMD2  → read CID
 * CMD3  → assign RCA  (eMMC: host picks the address)
 * CMD7  → select card  (R1b — may busy-wait on DAT0)
 * CMD16 → set 512-byte block length
 * ===================================================================== */

static int emmc_init(void)
{
	int rc;
	uint32_t resp;

	/* Hardware reset: RST_n low ≥1 µs, then high, wait ≥200 µs */
	gpio_pin_configure(port_a, PIN_RST, GPIO_OUTPUT_HIGH);
	k_busy_wait(100);
	gpio_pin_set(port_a, PIN_RST, 0);
	k_busy_wait(10);
	gpio_pin_set(port_a, PIN_RST, 1);
	k_busy_wait(500);

	cmd_out();
	dat0_in();
	idle_clocks(80);

	cmd_none(0, 0x00000000);
	k_msleep(10);

	/* CMD1 — OCR: bit30 = sector mode, bits23:15 = 2.7-3.6V, bit7 = 1.7-1.95V */
	for (int i = 0; i < 1000; i++) {
		rc = cmd_r3(1, 0x40FF8080, &resp);
		if (rc)
			return rc;
		if (resp & BIT(31))
			break;
		if (i == 999) {
			printk("CMD1: card never became ready\n");
			return -ETIMEDOUT;
		}
		k_msleep(1);
	}
	sector_addr = ((resp >> 29) & 3) == 2;
	printk("OCR=0x%08x  sector_addr=%d\n", resp, sector_addr);

	uint8_t cid[16];
	rc = cmd_r2(2, 0, cid);
	if (rc)
		return rc;
	printk("CID:");
	for (int i = 0; i < 16; i++)
		printk(" %02x", cid[i]);
	printk("\n");

	card_rca = 1;
	rc = cmd_r1(3, (uint32_t)card_rca << 16, &resp);
	if (rc)
		return rc;

	rc = cmd_r1b(7, (uint32_t)card_rca << 16, &resp);
	if (rc)
		return rc;
	printk("Selected  status=0x%08x\n", resp);

	rc = cmd_r1(16, 512, &resp);
	if (rc)
		return rc;

	return 0;
}

/* =====================================================================
 * Block read  (CMD17 — READ_SINGLE_BLOCK)
 * ===================================================================== */

static int emmc_read_block(uint32_t sector, uint8_t *buf)
{
	uint32_t addr = sector_addr ? sector : sector * 512;
	uint32_t st;
	int rc;

	rc = cmd_r1(17, addr, &st);
	if (rc)
		return rc;

	dat0_in();

	/* Wait for start bit (0) on DAT0 */
	for (int t = 0; t < 500000; t++) {
		clk_lo(); hperiod();
		clk_hi(); hperiod();
		if (gpio_pin_get(port_c, PIN_DAT0) == 0)
			goto data;
	}
	printk("read: timeout waiting for data\n");
	return -ETIMEDOUT;

data:
	for (int i = 0; i < 512; i++) {
		uint8_t val = 0;
		for (int b = 7; b >= 0; b--) {
			clk_lo(); hperiod();
			clk_hi(); hperiod();
			if (gpio_pin_get(port_c, PIN_DAT0))
				val |= 1 << b;
		}
		buf[i] = val;
	}

	/* CRC-16 (16 bits) */
	uint16_t got_crc = 0;
	for (int i = 15; i >= 0; i--) {
		clk_lo(); hperiod();
		clk_hi(); hperiod();
		if (gpio_pin_get(port_c, PIN_DAT0))
			got_crc |= 1 << i;
	}

	/* end bit */
	clk_lo(); hperiod();
	clk_hi(); hperiod();

	uint16_t exp_crc = crc16(buf, 512);
	if (got_crc != exp_crc) {
		printk("read CRC err: got=0x%04x exp=0x%04x\n",
		       got_crc, exp_crc);
		return -EIO;
	}

	return 0;
}

/* =====================================================================
 * Block write  (CMD24 — WRITE_BLOCK)
 * ===================================================================== */

static int emmc_write_block(uint32_t sector, const uint8_t *buf)
{
	uint32_t addr = sector_addr ? sector : sector * 512;
	uint32_t st;
	int rc;

	rc = cmd_r1(24, addr, &st);
	if (rc)
		return rc;

	/* Nwr gap (≥2 clocks, use 8) */
	dat0_in();
	idle_clocks(8);

	dat0_out();

	/* Start bit */
	clk_lo();
	gpio_pin_set(port_c, PIN_DAT0, 0);
	hperiod();
	clk_hi();
	hperiod();

	/* 512 bytes, MSB first */
	for (int i = 0; i < 512; i++) {
		for (int b = 7; b >= 0; b--) {
			clk_lo();
			gpio_pin_set(port_c, PIN_DAT0, (buf[i] >> b) & 1);
			hperiod();
			clk_hi();
			hperiod();
		}
	}

	/* CRC-16 */
	uint16_t c = crc16(buf, 512);
	for (int i = 15; i >= 0; i--) {
		clk_lo();
		gpio_pin_set(port_c, PIN_DAT0, (c >> i) & 1);
		hperiod();
		clk_hi();
		hperiod();
	}

	/* End bit */
	clk_lo();
	gpio_pin_set(port_c, PIN_DAT0, 1);
	hperiod();
	clk_hi();
	hperiod();

	/* Read CRC status token from card: start(0) + status(3) + end(1) */
	dat0_in();

	for (int t = 0; t < 1000; t++) {
		clk_lo(); hperiod();
		clk_hi(); hperiod();
		if (gpio_pin_get(port_c, PIN_DAT0) == 0)
			goto crc_st;
	}
	printk("write: timeout waiting for CRC status\n");
	return -ETIMEDOUT;

crc_st:
	;
	uint8_t crc_stat = 0;
	for (int i = 0; i < 3; i++) {
		clk_lo(); hperiod();
		clk_hi(); hperiod();
		crc_stat = (crc_stat << 1) |
			   (gpio_pin_get(port_c, PIN_DAT0) & 1);
	}
	/* end bit */
	clk_lo(); hperiod();
	clk_hi(); hperiod();

	/* 010 = accepted, 101 = CRC error, 110 = write error */
	if (crc_stat != 0x02) {
		printk("write CRC status rejected: 0x%x\n", crc_stat);
		return -EIO;
	}

	rc = wait_dat0_ready(1000);
	if (rc) {
		printk("write: busy timeout\n");
		return rc;
	}

	return 0;
}

/* =====================================================================
 * Test application
 * ===================================================================== */

static uint8_t wr_buf[512];
static uint8_t rd_buf[512];

int main(void)
{
	int rc;

	port_a = DEVICE_DT_GET(DT_NODELABEL(gpioa));
	port_c = DEVICE_DT_GET(DT_NODELABEL(gpioc));

	if (!device_is_ready(port_a) || !device_is_ready(port_c)) {
		printk("GPIO ports not ready\n");
		return -ENODEV;
	}

	gpio_pin_configure(port_c, PIN_CLK, GPIO_OUTPUT_LOW);

	printk("=== eMMC GPIO bit-bang test ===\n");

	rc = emmc_init();
	if (rc) {
		printk("Init FAILED (%d)\n", rc);
		return rc;
	}
	printk("Init OK\n\n");

	/* Fill with a recognisable pattern */
	for (int i = 0; i < 512; i++)
		wr_buf[i] = (uint8_t)(i & 0xFF);

	printk("Writing sector 1 ...\n");
	rc = emmc_write_block(1, wr_buf);
	if (rc) {
		printk("Write FAILED (%d)\n", rc);
		return rc;
	}
	printk("Write OK\n");

	printk("Reading sector 1 ...\n");
	rc = emmc_read_block(1, rd_buf);
	if (rc) {
		printk("Read FAILED (%d)\n", rc);
		return rc;
	}
	printk("Read OK\n");

	if (memcmp(wr_buf, rd_buf, 512) == 0) {
		printk("\n*** VERIFY PASS ***\n");
	} else {
		printk("\n*** VERIFY FAIL ***\n");
		int mis = 0;
		for (int i = 0; i < 512; i++) {
			if (wr_buf[i] != rd_buf[i]) {
				if (mis < 16)
					printk("  [%3d] wr=0x%02x rd=0x%02x\n",
					       i, wr_buf[i], rd_buf[i]);
				mis++;
			}
		}
		if (mis > 16)
			printk("  ... %d more mismatches\n", mis - 16);
	}

	return 0;
}
