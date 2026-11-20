/**
 * @file radio_port.h
 * @ingroup comms
 * @brief Thin layer between the application and the RFM26W driver
 *
 * Receive: the driver calls back from its own work queue thread, which must not block. The callback
 * copies the block into a queue with K_NO_WAIT and counts a drop if the queue is full.
 * Send: `radio_port_send()` is the one function that transmits. Only the downlink (radio_tx) thread
 * calls it, and never from the receive callback.
 *
 * The driver calls sit behind `struct radio_port_ops`, so unit tests can replace them. The default ops
 * call ut/drivers/rfm26w.h.
 */
#ifndef COMMS_RADIO_PORT_H_
#define COMMS_RADIO_PORT_H_

#include <stddef.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>

#include <ut/drivers/rfm26w.h>

struct radio_port;

struct radio_port_ops {
	int (*send)(const struct device *dev, const uint8_t *block, size_t len, k_timeout_t timeout);
	int (*recv_async)(const struct device *dev, rfm26w_rx_cb cb, void *user_data);
};

struct radio_port_stats {
	uint32_t rx_blocks;  /**< blocks queued */
	uint32_t rx_dropped; /**< blocks lost because the queue was full */
	uint32_t rx_bad_len; /**< callbacks with a length other than COMMS_BLOCK_LEN */
	uint32_t tx_blocks;  /**< blocks sent */
	uint32_t tx_errors;  /**< sends that returned an error */
};

struct radio_port {
	const struct device *dev;
	struct k_msgq *rx_q; /**< item size COMMS_BLOCK_LEN */
	const struct radio_port_ops *ops;
	struct radio_port_stats stats;
};

/** Set up the port with the driver-backed ops. Does not touch the radio. */
void radio_port_init(struct radio_port *p, const struct device *dev, struct k_msgq *rx_q);

/** Start receiving into the queue. The radio must already be started (`rfm26w_start`). */
int radio_port_start(struct radio_port *p);

/**
 * Send one block (COMMS_BLOCK_LEN bytes). Blocks for the air time, about 200 ms at 9.6 kb/s.
 * @return 0 or the driver's negative errno (counted in tx_errors)
 */
int radio_port_send(struct radio_port *p, const uint8_t *block);

#endif /* COMMS_RADIO_PORT_H_ */
