/** @file radio_port.c @ingroup comms @brief Radio driver adapter, see radio_port.h */
#include "radio_port.h"

#include "comms_config.h"

static const struct radio_port_ops driver_ops = {
	.send = rfm26w_send,
	.recv_async = rfm26w_recv_async,
};

static void rx_cb(const struct device *dev, const uint8_t *block, size_t len,
		  const struct rfm26w_rx_info *info, void *user_data)
{
	struct radio_port *p = user_data;

	ARG_UNUSED(dev);
	ARG_UNUSED(info);

	if (len != COMMS_BLOCK_LEN) {
		p->stats.rx_bad_len++;
		return;
	}
	/* Runs in the driver's work queue: never wait here. */
	if (k_msgq_put(p->rx_q, block, K_NO_WAIT) != 0) {
		p->stats.rx_dropped++;
		return;
	}
	p->stats.rx_blocks++;
}

void radio_port_init(struct radio_port *p, const struct device *dev, struct k_msgq *rx_q)
{
	*p = (struct radio_port){.dev = dev, .rx_q = rx_q, .ops = &driver_ops};
}

int radio_port_start(struct radio_port *p)
{
	return p->ops->recv_async(p->dev, rx_cb, p);
}

int radio_port_send(struct radio_port *p, const uint8_t *block)
{
	int err = p->ops->send(p->dev, block, COMMS_BLOCK_LEN, K_MSEC(COMMS_RADIO_SEND_TIMEOUT_MS));

	if (err) {
		p->stats.tx_errors++;
	} else {
		p->stats.tx_blocks++;
	}
	return err;
}
