/** @file test_radio_port.c @brief Tests for app/radio_port with the driver calls replaced */
#include <errno.h>
#include <zephyr/ztest.h>

#include "app/radio_port.h"
#include "test_util.h"

K_MSGQ_DEFINE(port_q, COMMS_BLOCK_LEN, 2, 1);

static struct radio_port port;
static rfm26w_rx_cb armed_cb;
static void *armed_arg;
static uint8_t sent[COMMS_BLOCK_LEN];
static size_t sent_len;
static int send_result;
static int send_calls;

static int fake_send(const struct device *dev, const uint8_t *block, size_t len, k_timeout_t timeout)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(timeout);
	send_calls++;
	sent_len = len;
	memcpy(sent, block, MIN(len, sizeof(sent)));
	return send_result;
}

static int fake_recv_async(const struct device *dev, rfm26w_rx_cb cb, void *arg)
{
	ARG_UNUSED(dev);
	armed_cb = cb;
	armed_arg = arg;
	return 0;
}

static const struct radio_port_ops fake_ops = {.send = fake_send, .recv_async = fake_recv_async};

static void before(void *f)
{
	ARG_UNUSED(f);
	k_msgq_purge(&port_q);
	radio_port_init(&port, NULL, &port_q);
	port.ops = &fake_ops;
	armed_cb = NULL;
	armed_arg = NULL;
	send_result = 0;
	send_calls = 0;
	sent_len = 0;
}

ZTEST_SUITE(radio_port, NULL, NULL, before, NULL, NULL);

static void deliver(uint8_t seed, size_t len)
{
	uint8_t b[COMMS_BLOCK_LEN];

	memset(b, seed, sizeof(b));
	armed_cb(NULL, b, len, NULL, armed_arg);
}

ZTEST(radio_port, test_start_arms_the_driver_callback)
{
	zassert_ok(radio_port_start(&port));
	zassert_not_null(armed_cb);
	zassert_equal(armed_arg, &port);
}

ZTEST(radio_port, test_received_block_is_copied_into_the_queue)
{
	uint8_t got[COMMS_BLOCK_LEN];
	uint8_t b[COMMS_BLOCK_LEN];

	zassert_ok(radio_port_start(&port));
	memset(b, 0x5A, sizeof(b));
	b[0] = 1;
	b[COMMS_BLOCK_LEN - 1] = 2;
	armed_cb(NULL, b, sizeof(b), NULL, armed_arg);
	memset(b, 0, sizeof(b)); /* the driver's buffer is only valid during the call */
	zassert_ok(k_msgq_get(&port_q, got, K_NO_WAIT));
	zassert_equal(got[0], 1);
	zassert_equal(got[1], 0x5A);
	zassert_equal(got[COMMS_BLOCK_LEN - 1], 2);
	zassert_equal(port.stats.rx_blocks, 1);
}

ZTEST(radio_port, test_full_queue_drops_and_counts_without_blocking)
{
	zassert_ok(radio_port_start(&port));
	deliver(1, COMMS_BLOCK_LEN);
	deliver(2, COMMS_BLOCK_LEN);
	int64_t t0 = k_uptime_get();

	deliver(3, COMMS_BLOCK_LEN);
	zassert_true(k_uptime_get() - t0 < 5, "the callback must not wait for queue space");
	zassert_equal(port.stats.rx_blocks, 2);
	zassert_equal(port.stats.rx_dropped, 1);
	zassert_equal(k_msgq_num_used_get(&port_q), 2);
}

ZTEST(radio_port, test_wrong_length_is_not_queued)
{
	zassert_ok(radio_port_start(&port));
	deliver(1, COMMS_BLOCK_LEN - 1);
	zassert_equal(port.stats.rx_bad_len, 1);
	zassert_equal(k_msgq_num_used_get(&port_q), 0);
}

ZTEST(radio_port, test_send_passes_one_full_block_to_the_driver)
{
	uint8_t b[COMMS_BLOCK_LEN];

	memset(b, 0xA5, sizeof(b));
	zassert_ok(radio_port_send(&port, b));
	zassert_equal(send_calls, 1);
	zassert_equal(sent_len, COMMS_BLOCK_LEN);
	zassert_mem_equal(sent, b, sizeof(b));
	zassert_equal(port.stats.tx_blocks, 1);
	zassert_equal(port.stats.tx_errors, 0);
}

ZTEST(radio_port, test_send_error_is_returned_and_counted)
{
	uint8_t b[COMMS_BLOCK_LEN] = {0};

	send_result = -EBUSY;
	zassert_equal(radio_port_send(&port, b), -EBUSY);
	zassert_equal(port.stats.tx_errors, 1);
	zassert_equal(port.stats.tx_blocks, 0);
}
