/**
 * @file main.c
 * @brief Bring-up test for the RFM26W driver (phase 2: SPI command layer)
 *
 * Powers the radio up, reads PART_INFO and shuts it down again. With no radio connected,
 * rfm26w_start() is expected to fail with -ETIMEDOUT (no CTS). Transmit and receive are not
 * implemented yet.
 *
 * The radio's reference clock (30 MHz crystal) is an ASSUMPTION set in the devicetree overlay.
 * If it is wrong, PART_INFO can still read correctly, but the carrier frequency and data rate will
 * be off. See PLANNED.md section 5.4.
 */
#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <ut/drivers/rfm26w.h>

#define RADIO_NODE DT_NODELABEL(rfm26w0)

static const struct device *const radio = DEVICE_DT_GET(RADIO_NODE);

static const char *err_name(int err)
{
	switch (err) {
	case 0:
		return "ok";
	case -ETIMEDOUT:
		return "ETIMEDOUT (no CTS: radio absent, unpowered, or wrong reference clock/wiring)";
	case -EIO:
		return "EIO (SPI error or property read-back mismatch)";
	case -ESHUTDOWN:
		return "ESHUTDOWN (radio not started)";
	case -ENOTSUP:
		return "ENOTSUP (not implemented yet)";
	default:
		return "other error";
	}
}

int main(void)
{
	struct rfm26w_info info;
	int ret;

	if (!device_is_ready(radio)) {
		printk("rfm26w: device not ready\n");
		return -ENODEV;
	}

	printk("rfm26w: starting (reference clock is an ASSUMPTION, see the devicetree overlay)\n");
	ret = rfm26w_start(radio);
	printk("rfm26w: start returned %d: %s\n", ret, err_name(ret));
	if (ret < 0) {
		return ret;
	}

	ret = rfm26w_get_info(radio, &info);
	printk("rfm26w: get_info returned %d: %s\n", ret, err_name(ret));
	if (ret == 0) {
		printk("rfm26w: part 0x%04x (expect 0x4463, verify), chip rev 0x%02x, block %u B\n",
		       info.part, info.chip_rev, info.block_len);
	}

	/* Listen for a short while. With no transmitter nearby, -EAGAIN (timeout) is expected. */
	{
		uint8_t block[223];
		struct rfm26w_rx_info rx;
		struct rfm26w_stats st;
		int16_t rssi;

		ret = rfm26w_recv(radio, block, sizeof(block), &rx, K_MSEC(500));
		printk("rfm26w: recv returned %d (-EAGAIN means nothing was heard)\n", ret);
		if (rfm26w_read_rssi(radio, &rssi) == 0) {
			printk("rfm26w: channel RSSI %d.%d dBm (conversion unverified)\n", rssi / 10,
			       (rssi < 0 ? -rssi : rssi) % 10);
		}
		if (rfm26w_get_stats(radio, &st) == 0) {
			printk("rfm26w: rx %u, tx %u, overruns %u, underruns %u, cts timeouts %u, "
			       "spi errors %u\n", st.rx_blocks, st.tx_blocks, st.rx_overruns,
			       st.tx_underruns, st.cts_timeouts, st.spi_errors);
		}
	}

	ret = rfm26w_shutdown(radio);
	printk("rfm26w: shutdown returned %d: %s\n", ret, err_name(ret));
	return 0;
}
