/**
 * @file main.c
 * @brief Tests for the RFM26W driver against a software model of the radio (native_sim)
 *
 * What this covers: command sequencing and CTS handling, profile loading and read-back, start and
 * shutdown, configure, FIFO streaming for send and receive, the interrupt loop, RSSI timing, error
 * paths and blocked callers being woken by shutdown. What it cannot cover: how the real chip behaves.
 * See PLANNED.md section 5.5 for what needs the bench.
 */
#include <errno.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#include <ut/drivers/rfm26w.h>

#include "rfm26w_emul.h"

#define BLOCK 223

/* The RSSI offset the test profile below writes, so the dBm conversion has a known input. */
#define TEST_RSSI_COMP 0x40
#define EXPECT_DBM10(raw) ((int16_t)((raw) * 5 - (TEST_RSSI_COMP + 70) * 10))

static const struct device *const radio = DEVICE_DT_GET(DT_NODELABEL(rfm26w0));
static struct rfm26w_emul_data *emul;

/* ------------------------------------------------------------------------------------------ */
/* A small profile table, replacing the driver's (weak) empty placeholder                     */
/* ------------------------------------------------------------------------------------------ */

static const uint8_t test_profile[] = {
	/* POWER_UP with the 30 MHz reference: the driver must skip it and send its own */
	0x07, 0x02, 0x01, 0x00, 0x01, 0xC9, 0xC3, 0x80,
	/* GPIO_PIN_CFG, no change */
	0x08, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* GLOBAL_XO_TUNE and GLOBAL_CLK_CFG (group 0, two properties) */
	0x06, 0x11, 0x00, 0x02, 0x00, 0x52, 0x00,
	/* MODEM_RSSI_COMP = TEST_RSSI_COMP */
	0x05, 0x11, 0x20, 0x01, 0x4E, TEST_RSSI_COMP,
	/* PKT_TX_THRESHOLD, PKT_RX_THRESHOLD = 16, 16: the driver must overwrite these */
	0x06, 0x11, 0x12, 0x02, 0x0B, 0x10, 0x10,
	0x00,
};

int rfm26w_profile_table(enum rfm26w_profile profile, const uint8_t **tbl, size_t *len)
{
	if (profile != RFM26W_PROFILE_GFSK_9K6) {
		return -ENOTSUP;
	}
	*tbl = test_profile;
	*len = sizeof(test_profile);
	return 0;
}

/* ------------------------------------------------------------------------------------------ */
/* Helpers                                                                                    */
/* ------------------------------------------------------------------------------------------ */

#define WAIT_UNTIL(cond, ms)                                                                       \
	do {                                                                                       \
		for (int _i = 0; !(cond) && _i < (ms); _i++) {                                     \
			k_msleep(1);                                                               \
		}                                                                                  \
	} while (0)

static void fill(uint8_t *b, uint8_t seed)
{
	for (int i = 0; i < BLOCK; i++) {
		b[i] = (uint8_t)(seed + i * 7);
	}
}

struct rx_log {
	int n;
	uint8_t blocks[4][BLOCK];
	struct rfm26w_rx_info infos[4];
};
static struct rx_log rxlog;

static void rx_cb(const struct device *dev, const uint8_t *block, size_t len,
		  const struct rfm26w_rx_info *info, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);
	if (rxlog.n < 4 && len == BLOCK) {
		memcpy(rxlog.blocks[rxlog.n], block, len);
		rxlog.infos[rxlog.n] = *info;
	}
	rxlog.n++;
}

static void start_radio(void)
{
	zassert_ok(rfm26w_start(radio), "start failed");
}

/* ------------------------------------------------------------------------------------------ */
/* Suite                                                                                      */
/* ------------------------------------------------------------------------------------------ */

static void before(void *fixture)
{
	ARG_UNUSED(fixture);
	emul = RFM26W_EMUL_DATA();
	(void)rfm26w_shutdown(radio);
	rfm26w_emul_reset(emul);
	memset(&rxlog, 0, sizeof(rxlog));
}

static void after(void *fixture)
{
	ARG_UNUSED(fixture);
	/* Whatever the test did, the driver must never have broken the command protocol. */
	zassert_equal(emul->cmds_while_busy, 0, "command sent before CTS");
	zassert_equal(emul->bad_cmds, 0, "bad or unknown command sent");
	zassert_equal(emul->ant_violations, 0, "antenna pins wrong for the radio state");
	(void)rfm26w_shutdown(radio);
}

ZTEST_SUITE(rfm26w, NULL, NULL, before, after, NULL);

/* ---- start, shutdown, info ---- */

ZTEST(rfm26w, test_start_programs_the_chip)
{
	start_radio();

	zassert_equal(emul->power_ups, 1);
	zassert_equal(emul->xo_hz, 30000000U, "POWER_UP must carry the devicetree clock");
	zassert_false(emul->xo_tcxo);
	zassert_equal(emul->state, 3, "radio should be Ready");
	zassert_true(emul->cmd_count[0x01] >= 1, "PART_INFO not read");

	/* profile properties arrived, the POWER_UP entry was skipped, thresholds were overwritten */
	zassert_equal(rfm26w_emul_prop(emul, 0x00, 0x00), 0x52);
	zassert_equal(rfm26w_emul_prop(emul, 0x20, 0x4E), TEST_RSSI_COMP);
	zassert_equal(emul->cmd_count[0x02], 1, "only the driver's own POWER_UP");
	zassert_equal(emul->cmd_count[0x13], 1, "profile GPIO_PIN_CFG not sent");
	zassert_equal(rfm26w_emul_prop(emul, 0x12, 0x0B), CONFIG_RFM26W_TX_THRESHOLD);
	zassert_equal(rfm26w_emul_prop(emul, 0x12, 0x0C), CONFIG_RFM26W_RX_THRESHOLD);

	/* interrupts: all three groups, packet/FIFO bits, sync detect, FIFO error */
	zassert_equal(rfm26w_emul_prop(emul, 0x01, 0x00), 0x07);
	zassert_equal(rfm26w_emul_prop(emul, 0x01, 0x01), 0x33);
	zassert_equal(rfm26w_emul_prop(emul, 0x01, 0x02), 0x01);
	zassert_equal(rfm26w_emul_prop(emul, 0x01, 0x03), 0x20);
}

ZTEST(rfm26w, test_start_twice_is_harmless)
{
	start_radio();
	zassert_ok(rfm26w_start(radio));
	zassert_equal(emul->power_ups, 1);
}

ZTEST(rfm26w, test_get_info)
{
	struct rfm26w_info info;

	zassert_equal(rfm26w_get_info(radio, &info), -ESHUTDOWN);
	start_radio();
	zassert_ok(rfm26w_get_info(radio, &info));
	zassert_equal(info.part, 0x4463);
	zassert_equal(info.chip_rev, 0x03);
	zassert_equal(info.block_len, BLOCK);
	zassert_equal(info.profile, RFM26W_PROFILE_GFSK_9K6);
	zassert_equal(rfm26w_get_info(radio, NULL), -EINVAL);
}

ZTEST(rfm26w, test_start_without_cts_times_out_and_recovers)
{
	struct rfm26w_stats st;

	emul->never_cts = true;
	zassert_equal(rfm26w_start(radio), -ETIMEDOUT);
	zassert_ok(rfm26w_get_stats(radio, &st));
	zassert_equal(st.cts_timeouts, 1);

	emul->never_cts = false;
	zassert_ok(rfm26w_start(radio), "a failed start must leave the radio startable");
}

ZTEST(rfm26w, test_slow_cts_is_polled_not_failed)
{
	/* 60 polls: about 50 busy-wait polls and ten 1 ms sleeps, inside the 20 ms timeout */
	emul->cts_delay_polls = 60;
	start_radio();
}

ZTEST(rfm26w, test_spi_error_is_reported)
{
	struct rfm26w_stats st;

	emul->fail_next_io = true;
	zassert_equal(rfm26w_start(radio), -EIO);
	zassert_ok(rfm26w_get_stats(radio, &st));
	zassert_equal(st.spi_errors, 1);
	zassert_ok(rfm26w_start(radio));
}

ZTEST(rfm26w, test_profile_readback_catches_a_bad_write)
{
	emul->corrupt_props = true;
	zassert_equal(rfm26w_start(radio), -EIO, "read-back must notice the wrong values");
	emul->corrupt_props = false;
	zassert_ok(rfm26w_start(radio));
}

ZTEST(rfm26w, test_restart_after_shutdown)
{
	uint8_t blk[BLOCK];
	uint8_t got[BLOCK];
	size_t len = 0;

	fill(blk, 1);
	start_radio();
	zassert_ok(rfm26w_shutdown(radio));
	zassert_true(emul->resets >= 1, "SDN must have been raised");
	zassert_false(emul->powered_up, "chip loses its state in shutdown");

	zassert_ok(rfm26w_start(radio));
	zassert_equal(emul->power_ups, 2);
	zassert_equal(rfm26w_emul_prop(emul, 0x20, 0x4E), TEST_RSSI_COMP, "profile reloaded");
	zassert_ok(rfm26w_send(radio, blk, BLOCK, K_SECONDS(2)));
	zassert_ok(rfm26w_emul_take_tx(emul, got, &len));
	zassert_mem_equal(got, blk, BLOCK);
}

/* ---- configure and TX power ---- */

ZTEST(rfm26w, test_configure_writes_frequency_and_power)
{
	const struct rfm26w_config cfg = {
		.frequency_hz = 433920000,
		.tx_power_dbm = -30,
		.profile = RFM26W_PROFILE_GFSK_9K6,
	};

	zassert_equal(rfm26w_configure(radio, &cfg), -ESHUTDOWN);
	start_radio();
	zassert_ok(rfm26w_configure(radio, &cfg));

	/* the numbers a WDS header gives for 433.92 MHz with a 30 MHz clock */
	zassert_equal(rfm26w_emul_prop(emul, 0x40, 0x00), 0x38);
	zassert_equal(rfm26w_emul_prop(emul, 0x40, 0x01), 0x0E);
	zassert_equal(rfm26w_emul_prop(emul, 0x40, 0x02), 0xD9);
	zassert_equal(rfm26w_emul_prop(emul, 0x40, 0x03), 0x16);
	zassert_equal(rfm26w_emul_prop(emul, 0x20, 0x51), 0x0A);
	zassert_equal(rfm26w_emul_prop(emul, 0x22, 0x01), 24, "placeholder mapping of -30 dBm");
}

ZTEST(rfm26w, test_configure_rejects_bad_input)
{
	struct rfm26w_config cfg = {
		.frequency_hz = 600000000, /* between the module's bands */
		.tx_power_dbm = 0,
		.profile = RFM26W_PROFILE_GFSK_9K6,
	};

	start_radio();
	zassert_equal(rfm26w_configure(radio, NULL), -EINVAL);
	zassert_equal(rfm26w_configure(radio, &cfg), -EINVAL);
	cfg.frequency_hz = 433920000;
	cfg.tx_power_dbm = 12;
	zassert_equal(rfm26w_configure(radio, &cfg), -EINVAL);
	cfg.tx_power_dbm = 0;
	cfg.profile = RFM26W_PROFILE_GFSK_50K; /* no table */
	zassert_equal(rfm26w_configure(radio, &cfg), -ENOTSUP);
}

ZTEST(rfm26w, test_tx_power_is_capped)
{
	zassert_equal(rfm26w_set_tx_power(radio, 0), -ESHUTDOWN);
	start_radio();
	zassert_ok(rfm26w_set_tx_power(radio, 11));
	zassert_equal(rfm26w_emul_prop(emul, 0x22, 0x01), CONFIG_RFM26W_PA_LEVEL_MAX);
	zassert_equal(rfm26w_set_tx_power(radio, 12), -EINVAL);
	zassert_equal(rfm26w_set_tx_power(radio, -41), -EINVAL);
}

/* ---- send ---- */

ZTEST(rfm26w, test_send_streams_a_whole_block)
{
	uint8_t blk[BLOCK];
	uint8_t got[BLOCK];
	size_t len = 0;
	struct rfm26w_stats st;

	fill(blk, 9);
	start_radio();
	zassert_ok(rfm26w_send(radio, blk, BLOCK, K_SECONDS(2)));

	zassert_equal(emul->start_tx_count, 1);
	zassert_equal(emul->last_tx_len_arg, BLOCK);
	zassert_ok(rfm26w_emul_take_tx(emul, got, &len));
	zassert_equal(len, BLOCK);
	zassert_mem_equal(got, blk, BLOCK, "block must arrive intact (it is larger than the FIFO)");
	zassert_equal(emul->fifo_underflows, 0);
	zassert_equal(emul->state, 3, "back to Ready when not listening");
	zassert_ok(rfm26w_get_stats(radio, &st));
	zassert_equal(st.tx_blocks, 1);
	zassert_equal(st.tx_underruns, 0);
}

ZTEST(rfm26w, test_send_argument_and_state_errors)
{
	uint8_t blk[BLOCK] = {0};

	zassert_equal(rfm26w_send(radio, blk, BLOCK, K_MSEC(10)), -ESHUTDOWN);
	start_radio();
	zassert_equal(rfm26w_send(radio, blk, BLOCK - 1, K_MSEC(10)), -EMSGSIZE);
	zassert_equal(rfm26w_send(radio, NULL, BLOCK, K_MSEC(10)), -EMSGSIZE);
	zassert_equal(emul->start_tx_count, 0);
}

ZTEST(rfm26w, test_send_timeout_aborts_and_the_radio_recovers)
{
	uint8_t blk[BLOCK];

	fill(blk, 3);
	start_radio();
	emul->stall_tx = true;
	zassert_equal(rfm26w_send(radio, blk, BLOCK, K_MSEC(100)), -ETIMEDOUT);
	zassert_equal(emul->state, 3, "driver must stop the transmission");

	emul->stall_tx = false;
	zassert_ok(rfm26w_send(radio, blk, BLOCK, K_SECONDS(2)), "next send must work");
}

ZTEST(rfm26w, test_send_underrun_is_reported)
{
	uint8_t blk[BLOCK];
	struct rfm26w_stats st;

	fill(blk, 5);
	start_radio();
	/* The model drains the FIFO faster than the driver can possibly refill it. */
	emul->us_per_byte = 1;
	zassert_equal(rfm26w_send(radio, blk, BLOCK, K_SECONDS(2)), -EIO);
	zassert_ok(rfm26w_get_stats(radio, &st));
	zassert_equal(st.tx_underruns, 1);
	zassert_equal(st.tx_blocks, 0);

	emul->us_per_byte = 833;
	zassert_ok(rfm26w_send(radio, blk, BLOCK, K_SECONDS(2)), "recovered after the underrun");
}

/* ---- receive ---- */

ZTEST(rfm26w, test_async_receive_delivers_block_and_rssi)
{
	uint8_t blk[BLOCK];

	fill(blk, 11);
	start_radio();
	zassert_ok(rfm26w_recv_async(radio, rx_cb, NULL));
	zassert_equal(emul->start_rx_count, 1);
	zassert_equal(emul->last_rx_len_arg, BLOCK, "RX_LEN must be the block length");
	zassert_equal(emul->state, 8);

	zassert_ok(rfm26w_emul_inject_rx(emul, blk, BLOCK, 120));
	WAIT_UNTIL(rxlog.n >= 1, 1000);
	zassert_equal(rxlog.n, 1);
	zassert_mem_equal(rxlog.blocks[0], blk, BLOCK);
	/* The model clears its latched RSSI when the block ends. A driver that reads it late gets 0. */
	zassert_equal(rxlog.infos[0].rssi_dbm10, EXPECT_DBM10(120));
	zassert_true(rxlog.infos[0].uptime_ms > 0);
	zassert_equal(emul->state, 8, "still listening");

	zassert_ok(rfm26w_recv_async(radio, NULL, NULL));
	zassert_equal(emul->state, 3, "cancelling stops listening");
}

ZTEST(rfm26w, test_async_receive_back_to_back_blocks)
{
	uint8_t a[BLOCK];
	uint8_t b[BLOCK];
	uint8_t c[BLOCK];
	struct rfm26w_stats st;

	fill(a, 1);
	fill(b, 2);
	fill(c, 3);
	start_radio();
	zassert_ok(rfm26w_recv_async(radio, rx_cb, NULL));
	zassert_ok(rfm26w_emul_inject_rx(emul, a, BLOCK, 100));
	zassert_ok(rfm26w_emul_inject_rx(emul, b, BLOCK, 110));
	zassert_ok(rfm26w_emul_inject_rx(emul, c, BLOCK, 130));
	WAIT_UNTIL(rxlog.n >= 3, 2000);

	zassert_equal(rxlog.n, 3);
	zassert_mem_equal(rxlog.blocks[0], a, BLOCK);
	zassert_mem_equal(rxlog.blocks[1], b, BLOCK);
	zassert_mem_equal(rxlog.blocks[2], c, BLOCK);
	zassert_equal(rxlog.infos[1].rssi_dbm10, EXPECT_DBM10(110));
	zassert_equal(rxlog.infos[2].rssi_dbm10, EXPECT_DBM10(130));
	zassert_ok(rfm26w_get_stats(radio, &st));
	zassert_equal(st.rx_blocks, 3);
	zassert_equal(st.rx_overruns, 0);
	zassert_equal(emul->start_rx_count, 1, "receive never had to be restarted");
	zassert_equal(emul->cmd_count[0x33], 3, "one state check per block");
}

ZTEST(rfm26w, test_blocking_receive)
{
	uint8_t blk[BLOCK];
	uint8_t got[BLOCK];
	struct rfm26w_rx_info info;

	fill(blk, 21);
	zassert_equal(rfm26w_recv(radio, got, sizeof(got), &info, K_MSEC(10)), -ESHUTDOWN);
	start_radio();
	zassert_equal(rfm26w_recv(radio, got, BLOCK - 1, &info, K_MSEC(10)), -EMSGSIZE);

	zassert_ok(rfm26w_emul_inject_rx(emul, blk, BLOCK, 90));
	zassert_ok(rfm26w_recv(radio, got, sizeof(got), &info, K_SECONDS(1)));
	zassert_mem_equal(got, blk, BLOCK);
	zassert_equal(info.rssi_dbm10, EXPECT_DBM10(90));

	zassert_equal(rfm26w_recv(radio, got, sizeof(got), &info, K_MSEC(50)), -EAGAIN);
	zassert_equal(rfm26w_recv(radio, got, sizeof(got), NULL, K_NO_WAIT), -EAGAIN);
}

ZTEST(rfm26w, test_receive_keeps_one_block_and_counts_the_one_it_drops)
{
	uint8_t a[BLOCK];
	uint8_t b[BLOCK];
	uint8_t got[BLOCK];
	struct rfm26w_stats st;

	fill(a, 31);
	fill(b, 32);
	start_radio();
	/* the first recv() arms the radio */
	zassert_equal(rfm26w_recv(radio, got, sizeof(got), NULL, K_NO_WAIT), -EAGAIN);
	zassert_ok(rfm26w_emul_inject_rx(emul, a, BLOCK, 100));
	zassert_ok(rfm26w_emul_inject_rx(emul, b, BLOCK, 100));
	k_msleep(700); /* both blocks have been on the air, nobody called recv() */

	zassert_ok(rfm26w_get_stats(radio, &st));
	zassert_equal(st.rx_blocks, 1);
	zassert_equal(st.rx_overruns, 1, "the second block is dropped and counted");
	zassert_equal(emul->fifo_overflows, 0, "dropping must not overflow the radio FIFO");
	zassert_ok(rfm26w_recv(radio, got, sizeof(got), NULL, K_NO_WAIT));
	zassert_mem_equal(got, a, BLOCK, "the first block is the one kept");
	zassert_equal(rfm26w_recv(radio, got, sizeof(got), NULL, K_NO_WAIT), -EAGAIN);
}

ZTEST(rfm26w, test_receive_overflow_is_recovered)
{
	uint8_t bad[BLOCK];
	uint8_t good[BLOCK];
	struct rfm26w_stats st;

	fill(bad, 40);
	fill(good, 41);
	start_radio();
	zassert_ok(rfm26w_recv_async(radio, rx_cb, NULL));

	/* bytes arrive far faster than the driver can drain them */
	emul->us_per_byte = 1;
	zassert_ok(rfm26w_emul_inject_rx(emul, bad, BLOCK, 100));
	WAIT_UNTIL(emul->fifo_overflows > 0, 200);
	k_msleep(50);
	zassert_ok(rfm26w_get_stats(radio, &st));
	zassert_true(st.rx_overruns >= 1);
	zassert_equal(rxlog.n, 0, "a damaged block must not be delivered");

	emul->us_per_byte = 833;
	zassert_ok(rfm26w_emul_inject_rx(emul, good, BLOCK, 100));
	WAIT_UNTIL(rxlog.n >= 1, 1000);
	zassert_equal(rxlog.n, 1);
	zassert_mem_equal(rxlog.blocks[0], good, BLOCK);
}

ZTEST(rfm26w, test_receive_restarts_if_the_chip_leaves_rx)
{
	uint8_t a[BLOCK];
	uint8_t b[BLOCK];

	fill(a, 51);
	fill(b, 52);
	start_radio();
	emul->leave_rx_after_packet = true;
	zassert_ok(rfm26w_recv_async(radio, rx_cb, NULL));
	zassert_ok(rfm26w_emul_inject_rx(emul, a, BLOCK, 100));
	zassert_ok(rfm26w_emul_inject_rx(emul, b, BLOCK, 100));
	WAIT_UNTIL(rxlog.n >= 2, 1500);

	zassert_equal(rxlog.n, 2, "second block lost: receive was not restarted");
	zassert_mem_equal(rxlog.blocks[1], b, BLOCK);
	/* the initial START_RX, then one restart after each of the two blocks */
	zassert_equal(emul->start_rx_count, 3);
}

ZTEST(rfm26w, test_async_receive_argument_and_state_errors)
{
	uint8_t got[BLOCK];

	zassert_equal(rfm26w_recv_async(radio, rx_cb, NULL), -ESHUTDOWN);
	start_radio();
	zassert_ok(rfm26w_recv_async(radio, rx_cb, NULL));
	zassert_equal(rfm26w_recv_async(radio, rx_cb, NULL), -EBUSY, "already armed");
	zassert_equal(rfm26w_recv(radio, got, sizeof(got), NULL, K_NO_WAIT), -EBUSY,
		      "recv() and the callback are exclusive");
	zassert_ok(rfm26w_recv_async(radio, NULL, NULL));
	zassert_equal(rfm26w_recv(radio, got, sizeof(got), NULL, K_NO_WAIT), -EAGAIN,
		      "recv() works again once the callback is cancelled");
}

/* ---- sending and receiving together ---- */

ZTEST(rfm26w, test_send_waits_for_a_block_that_is_arriving)
{
	uint8_t in[BLOCK];
	uint8_t out[BLOCK];
	uint8_t got[BLOCK];
	size_t len = 0;

	fill(in, 61);
	fill(out, 62);
	start_radio();
	zassert_ok(rfm26w_recv_async(radio, rx_cb, NULL));
	zassert_ok(rfm26w_emul_inject_rx(emul, in, BLOCK, 100));
	k_msleep(60); /* sync word seen, block only partly received */

	zassert_equal(rfm26w_send(radio, out, BLOCK, K_MSEC(10)), -EBUSY);
	zassert_equal(emul->start_tx_count, 0, "must not transmit over an incoming block");

	WAIT_UNTIL(rxlog.n >= 1, 1000);
	zassert_equal(rxlog.n, 1);
	zassert_mem_equal(rxlog.blocks[0], in, BLOCK, "incoming block must be undamaged");

	/* Now the air is free. After the send, the radio listens again by itself. */
	zassert_ok(rfm26w_send(radio, out, BLOCK, K_SECONDS(2)));
	zassert_ok(rfm26w_emul_take_tx(emul, got, &len));
	zassert_mem_equal(got, out, BLOCK);
	zassert_equal(emul->state, 8, "receive must resume after the transmission");
	zassert_equal(emul->start_rx_count, 2);

	zassert_ok(rfm26w_emul_inject_rx(emul, in, BLOCK, 100));
	WAIT_UNTIL(rxlog.n >= 2, 1000);
	zassert_equal(rxlog.n, 2, "a block after the send is received");
}

/* ---- RSSI and statistics ---- */

ZTEST(rfm26w, test_read_rssi)
{
	int16_t rssi;

	zassert_equal(rfm26w_read_rssi(radio, &rssi), -ESHUTDOWN);
	start_radio();
	emul->curr_rssi = 100;
	zassert_ok(rfm26w_read_rssi(radio, &rssi));
	zassert_equal(rssi, EXPECT_DBM10(100));
	zassert_equal(rfm26w_read_rssi(radio, NULL), -EINVAL);
}

ZTEST(rfm26w, test_stats_argument_check)
{
	struct rfm26w_stats st;

	zassert_equal(rfm26w_get_stats(radio, NULL), -EINVAL);
	zassert_ok(rfm26w_get_stats(radio, &st));
}

/* ---- shutdown wakes anyone who is waiting ---- */

static K_THREAD_STACK_DEFINE(waiter_stack, 2048);
static struct k_thread waiter;
static volatile int waiter_ret;
static volatile bool waiter_is_send;

static void waiter_fn(void *a, void *b, void *c)
{
	uint8_t buf[BLOCK] = {0};

	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);
	if (waiter_is_send) {
		waiter_ret = rfm26w_send(radio, buf, BLOCK, K_FOREVER);
	} else {
		waiter_ret = rfm26w_recv(radio, buf, sizeof(buf), NULL, K_FOREVER);
	}
}

static void run_waiter_then_shutdown(bool is_send)
{
	waiter_is_send = is_send;
	waiter_ret = 12345;
	k_thread_create(&waiter, waiter_stack, K_THREAD_STACK_SIZEOF(waiter_stack), waiter_fn, NULL,
			NULL, NULL, K_PRIO_PREEMPT(5), 0, K_NO_WAIT);
	k_msleep(50);
	zassert_equal(waiter_ret, 12345, "the call should still be waiting");
	zassert_ok(rfm26w_shutdown(radio));
	zassert_ok(k_thread_join(&waiter, K_SECONDS(1)), "shutdown did not wake the waiter");
}

ZTEST(rfm26w, test_shutdown_wakes_a_blocked_recv)
{
	start_radio();
	run_waiter_then_shutdown(false);
	zassert_equal(waiter_ret, -ESHUTDOWN);
}

ZTEST(rfm26w, test_shutdown_wakes_a_blocked_send)
{
	start_radio();
	emul->stall_tx = true;
	run_waiter_then_shutdown(true);
	zassert_equal(waiter_ret, -ESHUTDOWN);
}
