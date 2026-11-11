/**
 * @file rfm26w_emul.c
 * @brief Software model of the RFM26W / Si446x, see rfm26w_emul.h
 * @ingroup comms
 *
 * Written from AN625 (command and property tables) and the RFM26W datasheet. Constants are
 * deliberately NOT shared with the driver's rfm26w_priv.h.
 */
#define DT_DRV_COMPAT hoperf_rfm26w

#include <errno.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_emul.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/spi_emul.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include "rfm26w_emul.h"

LOG_MODULE_REGISTER(rfm26w_emul, LOG_LEVEL_INF);

/* ---- AN625 command codes ---- */
#define C_PART_INFO            0x01
#define C_POWER_UP             0x02
#define C_SET_PROPERTY         0x11
#define C_GET_PROPERTY         0x12
#define C_GPIO_PIN_CFG         0x13
#define C_FIFO_INFO            0x15
#define C_GET_INT_STATUS       0x20
#define C_GET_MODEM_STATUS     0x22
#define C_START_TX             0x31
#define C_START_RX             0x32
#define C_REQUEST_DEVICE_STATE 0x33
#define C_CHANGE_STATE         0x34
#define C_READ_CMD_BUFF        0x44
#define C_WRITE_TX_FIFO        0x66
#define C_READ_RX_FIFO         0x77

/* ---- AN625 state numbers ---- */
#define S_SLEEP 1
#define S_READY 3
#define S_TX    7
#define S_RX    8

/* ---- AN625 property addresses used by the model ---- */
#define G_INT_CTL        0x01
#define P_INT_CTL_ENABLE 0x00
#define P_INT_CTL_PH     0x01
#define P_INT_CTL_MODEM  0x02
#define P_INT_CTL_CHIP   0x03
#define G_PKT            0x12
#define P_PKT_TX_THR     0x0B
#define P_PKT_RX_THR     0x0C
#define G_MODEM          0x20
#define P_RSSI_COMP      0x4E

/* ---- AN625 interrupt bits ---- */
#define PH_RX_AF   0x01
#define PH_TX_AE   0x02
#define PH_PKT_RX  0x10
#define PH_PKT_TX  0x20
#define MODEM_SYNC 0x01
#define CHIP_FIFO  0x20

struct rfm26w_emul_cfg {
	struct gpio_dt_spec irq;
	struct gpio_dt_spec sdn;
	struct gpio_dt_spec tx_ant;
	struct gpio_dt_spec rx_ant;
};

/* One emulated radio only. */
static const struct rfm26w_emul_cfg *g_cfg;
static struct rfm26w_emul_data *g_data;

/* ------------------------------------------------------------------------------------------ */
/* Interrupt line and FIFO conditions (call with d->lock held)                                */
/* ------------------------------------------------------------------------------------------ */

static void irq_sync_locked(struct rfm26w_emul_data *d)
{
	const uint8_t en = d->props[G_INT_CTL][P_INT_CTL_ENABLE];
	const bool active = ((d->pend_ph != 0) && (en & 0x01)) ||
			    ((d->pend_modem != 0) && (en & 0x02)) ||
			    ((d->pend_chip != 0) && (en & 0x04));

	if (active != d->irq_asserted) {
		d->irq_asserted = active;
		/* nIRQ is active low */
		(void)gpio_emul_input_set(g_cfg->irq.port, g_cfg->irq.pin, active ? 0 : 1);
	}
}

static void raise_ph_locked(struct rfm26w_emul_data *d, uint8_t bit)
{
	if (d->props[G_INT_CTL][P_INT_CTL_PH] & bit) {
		d->pend_ph |= bit;
	}
	irq_sync_locked(d);
}

static void raise_modem_locked(struct rfm26w_emul_data *d, uint8_t bit)
{
	if (d->props[G_INT_CTL][P_INT_CTL_MODEM] & bit) {
		d->pend_modem |= bit;
	}
	irq_sync_locked(d);
}

static void raise_chip_locked(struct rfm26w_emul_data *d, uint8_t bit)
{
	if (d->props[G_INT_CTL][P_INT_CTL_CHIP] & bit) {
		d->pend_chip |= bit;
	}
	irq_sync_locked(d);
}

/* Fire the almost-empty and almost-full interrupts on the rising edge of their condition. */
static void fifo_update_locked(struct rfm26w_emul_data *d)
{
	const bool ae = (EMUL_FIFO_SIZE - d->tx_count) >= d->props[G_PKT][P_PKT_TX_THR];
	const bool af = d->rx_count >= d->props[G_PKT][P_PKT_RX_THR];

	if (ae && !d->ae_prev) {
		raise_ph_locked(d, PH_TX_AE);
	}
	if (af && !d->af_prev) {
		raise_ph_locked(d, PH_RX_AF);
	}
	d->ae_prev = ae;
	d->af_prev = af;
}

/* Power-on defaults. Everything the chip loses when SDN goes high. */
static void chip_reset_locked(struct rfm26w_emul_data *d)
{
	d->powered_up = false;
	d->state = S_READY;
	d->xo_hz = 0;
	d->xo_tcxo = false;
	memset(d->props, 0, sizeof(d->props));
	d->props[G_INT_CTL][P_INT_CTL_ENABLE] = 0x04;
	d->props[G_INT_CTL][P_INT_CTL_CHIP] = 0x04;
	d->props[G_PKT][P_PKT_TX_THR] = 0x30;
	d->props[G_PKT][P_PKT_RX_THR] = 0x30;
	d->props[G_MODEM][P_RSSI_COMP] = 0x32;
	d->tx_count = 0;
	d->rx_count = 0;
	d->ae_prev = true;
	d->af_prev = false;
	d->pend_ph = 0;
	d->pend_modem = 0;
	d->pend_chip = 0;
	d->latch_rssi = 0;
	d->reply_len = 0;
	d->cts_ready = true;
	d->polls_left = 0;
	d->tx_active = false;
	d->rx_session = false;
	irq_sync_locked(d);
}

/* SDN is wired to an output pin of the host. High means shut down. */
static void sdn_changed(const struct device *port, struct gpio_callback *cb, uint32_t pins)
{
	struct rfm26w_emul_data *d = g_data;
	k_spinlock_key_t key;

	ARG_UNUSED(port);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	if (gpio_emul_output_get(g_cfg->sdn.port, g_cfg->sdn.pin) == 1) {
		key = k_spin_lock(&d->lock);
		chip_reset_locked(d);
		d->resets++;
		k_spin_unlock(&d->lock, key);
	}
}

static bool in_shutdown(void)
{
	return gpio_emul_output_get(g_cfg->sdn.port, g_cfg->sdn.pin) != 0;
}

/* ------------------------------------------------------------------------------------------ */
/* Air interface                                                                              */
/* ------------------------------------------------------------------------------------------ */

static void queue_push(struct emul_block *q, size_t *head, size_t *count, const uint8_t *data,
		       size_t len, uint8_t rssi)
{
	if (*count == EMUL_QUEUE_DEPTH) {
		return; /* test queue full: drop the oldest-unread newcomer */
	}
	struct emul_block *b = &q[(*head + *count) % EMUL_QUEUE_DEPTH];

	memcpy(b->data, data, len);
	b->len = len;
	b->rssi_raw = rssi;
	(*count)++;
}

static void enter_rx_state_locked(struct rfm26w_emul_data *d)
{
	d->state = S_RX;
	d->rx_session = false;
	d->latch_rssi = 0; /* AN625: reset to 0 at the start of every RX */
}

static void packet_sent_locked(struct rfm26w_emul_data *d)
{
	queue_push(d->tx_done, &d->tx_d_head, &d->tx_d_count, d->tx_cur, d->tx_len, 0);
	d->tx_active = false;
	raise_ph_locked(d, PH_PKT_TX);
	if (d->tx_complete_state == S_RX) {
		enter_rx_state_locked(d);
	} else if (d->tx_complete_state != 0) {
		d->state = d->tx_complete_state;
	}
}

static void packet_received_locked(struct rfm26w_emul_data *d)
{
	d->rx_session = false;
	raise_ph_locked(d, PH_PKT_RX);
	if (d->leave_rx_after_packet) {
		d->state = S_READY;
	} else if (d->rx_state_valid == S_RX) {
		enter_rx_state_locked(d);
	} else if (d->rx_state_valid != 0) {
		d->state = d->rx_state_valid;
	}
}

/* One byte time on the air. */
static void byte_step_locked(struct rfm26w_emul_data *d)
{
	if (d->state == S_TX && d->tx_active) {
		if (d->stall_tx) {
			return;
		}
		if (d->tx_count == 0) {
			/* The host did not keep the FIFO fed. */
			d->fifo_underflows++;
			raise_chip_locked(d, CHIP_FIFO);
			d->tx_active = false;
			d->state = S_READY;
			return;
		}
		d->tx_cur[d->tx_sent++] = d->tx_fifo[0];
		memmove(d->tx_fifo, d->tx_fifo + 1, --d->tx_count);
		fifo_update_locked(d);
		if (d->tx_sent == d->tx_len) {
			packet_sent_locked(d);
		}
		return;
	}

	if (d->state != S_RX) {
		return;
	}
	if (!d->rx_session) {
		if (d->rx_q_count == 0) {
			return;
		}
		d->rx_cur = d->rx_queue[d->rx_q_head];
		d->rx_q_head = (d->rx_q_head + 1) % EMUL_QUEUE_DEPTH;
		d->rx_q_count--;
		d->rx_session = true;
		d->rx_delivered = 0;
		d->rx_preamble_left = d->preamble_bytes;
		if (d->rx_preamble_left == 0) {
			d->latch_rssi = d->rx_cur.rssi_raw;
			raise_modem_locked(d, MODEM_SYNC);
		}
		return;
	}
	if (d->rx_preamble_left > 0) {
		if (--d->rx_preamble_left == 0) {
			d->latch_rssi = d->rx_cur.rssi_raw;
			raise_modem_locked(d, MODEM_SYNC);
		}
		return;
	}

	if (d->rx_count < EMUL_FIFO_SIZE) {
		d->rx_fifo[d->rx_count++] =
			(d->rx_delivered < d->rx_cur.len) ? d->rx_cur.data[d->rx_delivered] : 0;
	} else {
		d->fifo_overflows++;
		raise_chip_locked(d, CHIP_FIFO);
	}
	d->rx_delivered++;
	fifo_update_locked(d);
	if (d->rx_delivered == d->rx_len) {
		packet_received_locked(d);
	}
}

static void tick(struct k_timer *timer)
{
	struct rfm26w_emul_data *d = CONTAINER_OF(timer, struct rfm26w_emul_data, timer);
	const uint32_t byte_us = (d->us_per_byte == 0) ? 1 : d->us_per_byte;
	k_spinlock_key_t key = k_spin_lock(&d->lock);

	if (d->powered_up && !in_shutdown()) {
		d->credit_us += 1000;
		while (d->credit_us >= byte_us) {
			d->credit_us -= byte_us;
			byte_step_locked(d);
		}
	}
	k_spin_unlock(&d->lock, key);
}

/* ------------------------------------------------------------------------------------------ */
/* Command interface                                                                          */
/* ------------------------------------------------------------------------------------------ */

static void check_antenna_locked(struct rfm26w_emul_data *d, bool tx)
{
	/* Datasheet pin table: RX state RX-ANT low and TX-ANT high, TX state the reverse. */
	if (g_cfg->tx_ant.port != NULL && g_cfg->rx_ant.port != NULL) {
		const int txp = gpio_emul_output_get(g_cfg->tx_ant.port, g_cfg->tx_ant.pin);
		const int rxp = gpio_emul_output_get(g_cfg->rx_ant.port, g_cfg->rx_ant.pin);
		const bool ok = tx ? (txp == 0 && rxp == 1) : (rxp == 0 && txp == 1);

		if (!ok) {
			d->ant_violations++;
		}
	}
}

static void exec_cmd_locked(struct rfm26w_emul_data *d, const uint8_t *tx, size_t n)
{
	const uint8_t cmd = tx[0];
	const uint8_t *a = tx + 1;
	const size_t na = n - 1;

	d->cmd_count[cmd]++;
	d->reply_len = 0;
	d->cts_ready = false;
	d->polls_left = d->cts_delay_polls;

	if (!d->powered_up && cmd != C_POWER_UP) {
		d->bad_cmds++; /* chip would raise CMD_ERROR */
		return;
	}

	switch (cmd) {
	case C_POWER_UP:
		if (na != 6) {
			d->bad_cmds++;
			return;
		}
		d->xo_tcxo = (a[1] & 0x01) != 0;
		d->xo_hz = sys_get_be32(&a[2]);
		d->powered_up = true;
		d->power_ups++;
		d->state = S_READY;
		break;
	case C_PART_INFO:
		d->reply[0] = 0x03; /* CHIPREV */
		d->reply[1] = 0x44; /* PART 0x4463 */
		d->reply[2] = 0x63;
		d->reply[3] = 0x00; /* PBUILD */
		d->reply[4] = 0x00; /* ID */
		d->reply[5] = 0x00;
		d->reply[6] = 0x00; /* CUSTOMER */
		d->reply[7] = 0x00; /* ROMID */
		d->reply_len = 8;
		break;
	case C_SET_PROPERTY:
		if (na < 4 || a[1] == 0 || a[1] > 12 || na != 3u + a[1] || a[0] >= EMUL_PROP_GROUPS) {
			d->bad_cmds++;
			return;
		}
		for (uint8_t i = 0; i < a[1]; i++) {
			const uint8_t idx = (uint8_t)(a[2] + i);

			d->props[a[0]][idx] = d->corrupt_props ? (uint8_t)(a[3 + i] ^ 0x01) : a[3 + i];
		}
		fifo_update_locked(d); /* a threshold may have changed */
		irq_sync_locked(d);    /* an enable may have changed */
		break;
	case C_GET_PROPERTY:
		if (na != 3 || a[1] == 0 || a[1] > 12 || a[0] >= EMUL_PROP_GROUPS) {
			d->bad_cmds++;
			return;
		}
		for (uint8_t i = 0; i < a[1]; i++) {
			d->reply[i] = d->props[a[0]][(uint8_t)(a[2] + i)];
		}
		d->reply_len = a[1];
		break;
	case C_GPIO_PIN_CFG:
		d->reply_len = 7;
		break;
	case C_FIFO_INFO:
		if (na != 1) {
			d->bad_cmds++;
			return;
		}
		if (a[0] & 0x02) {
			d->rx_count = 0;
		}
		if (a[0] & 0x01) {
			d->tx_count = 0;
		}
		d->reply[0] = d->rx_count;
		d->reply[1] = EMUL_FIFO_SIZE - d->tx_count;
		d->reply_len = 2;
		fifo_update_locked(d);
		break;
	case C_GET_INT_STATUS:
		if (na > 3) {
			d->bad_cmds++;
			return;
		}
		d->reply[0] = (d->pend_ph ? 0x01 : 0) | (d->pend_modem ? 0x02 : 0) |
			      (d->pend_chip ? 0x04 : 0);
		d->reply[1] = d->reply[0];
		d->reply[2] = d->pend_ph;
		d->reply[3] = d->pend_ph;
		d->reply[4] = d->pend_modem;
		d->reply[5] = d->pend_modem;
		d->reply[6] = d->pend_chip;
		d->reply[7] = d->pend_chip;
		d->reply_len = 8;
		/* AN625: a clear argument bit CLEARS that pending interrupt, a set bit leaves it. */
		d->pend_ph &= (na > 0) ? a[0] : 0xFF;
		d->pend_modem &= (na > 1) ? a[1] : 0xFF;
		d->pend_chip &= (na > 2) ? a[2] : 0xFF;
		irq_sync_locked(d);
		break;
	case C_GET_MODEM_STATUS:
		if (na > 1) {
			d->bad_cmds++;
			return;
		}
		memset(d->reply, 0, 8);
		d->reply[0] = d->pend_modem;
		d->reply[1] = d->pend_modem;
		d->reply[2] = d->curr_rssi;
		d->reply[3] = d->latch_rssi;
		d->reply_len = 8;
		break;
	case C_START_TX:
		if (na < 4) {
			d->bad_cmds++;
			return;
		}
		check_antenna_locked(d, true);
		d->start_tx_count++;
		d->tx_complete_state = a[1] >> 4;
		d->tx_len = sys_get_be16(&a[2]);
		d->last_tx_len_arg = d->tx_len;
		d->tx_sent = 0;
		d->tx_active = (d->tx_len != 0);
		d->rx_session = false;
		d->state = S_TX;
		break;
	case C_START_RX:
		if (na < 4) {
			d->bad_cmds++;
			return;
		}
		check_antenna_locked(d, false);
		d->start_rx_count++;
		d->rx_len = sys_get_be16(&a[2]);
		d->last_rx_len_arg = d->rx_len;
		d->rx_state_valid = (na >= 6) ? (a[5] & 0x0F) : S_READY;
		d->tx_active = false;
		enter_rx_state_locked(d);
		break;
	case C_REQUEST_DEVICE_STATE:
		d->reply[0] = d->state;
		d->reply[1] = 0;
		d->reply_len = 2;
		break;
	case C_CHANGE_STATE:
		if (na != 1) {
			d->bad_cmds++;
			return;
		}
		d->tx_active = false;
		d->rx_session = false;
		if (a[0] != 0) {
			d->state = a[0] & 0x0F;
		}
		break;
	default:
		d->bad_cmds++;
		break;
	}
}

static int flatten(const struct spi_buf_set *set, uint8_t *out, size_t cap, size_t *total)
{
	size_t n = 0;

	if (set != NULL) {
		for (size_t i = 0; i < set->count; i++) {
			if (n + set->buffers[i].len > cap) {
				return -EINVAL;
			}
			if (set->buffers[i].buf != NULL) {
				memcpy(out + n, set->buffers[i].buf, set->buffers[i].len);
			} else {
				memset(out + n, 0, set->buffers[i].len);
			}
			n += set->buffers[i].len;
		}
	}
	*total = n;
	return 0;
}

/* One chip-select window. */
static int rfm26w_emul_io(const struct emul *target, const struct spi_config *config,
			  const struct spi_buf_set *tx_bufs, const struct spi_buf_set *rx_bufs)
{
	struct rfm26w_emul_data *d = target->data;
	uint8_t tx[96];
	uint8_t rx[96] = {0};
	size_t ntx;
	size_t nrx;
	k_spinlock_key_t key;

	ARG_UNUSED(config);

	if (flatten(tx_bufs, tx, sizeof(tx), &ntx) < 0 || ntx == 0) {
		return -EINVAL;
	}
	nrx = 0;
	if (rx_bufs != NULL) {
		for (size_t i = 0; i < rx_bufs->count; i++) {
			nrx += rx_bufs->buffers[i].len;
		}
		if (nrx > sizeof(rx)) {
			return -EINVAL;
		}
	}

	key = k_spin_lock(&d->lock);
	if (d->fail_next_io) {
		d->fail_next_io = false;
		k_spin_unlock(&d->lock, key);
		return -EIO;
	}

	if (in_shutdown()) {
		/* No SPI access in shutdown: the bus reads back zeros, so CTS never appears. */
	} else if (tx[0] == C_READ_CMD_BUFF) {
		bool ready = false;

		if (d->never_cts) {
			ready = false;
		} else if (!d->cts_ready && d->polls_left > 0) {
			d->polls_left--;
		} else {
			d->cts_ready = true;
			ready = true;
		}
		if (nrx > 1) {
			rx[1] = ready ? 0xFF : 0x00;
		}
		for (size_t i = 0; ready && i < d->reply_len && 2 + i < nrx; i++) {
			rx[2 + i] = d->reply[i];
		}
	} else if (tx[0] == C_WRITE_TX_FIFO) {
		for (size_t i = 1; i < ntx; i++) {
			if (d->tx_count < EMUL_FIFO_SIZE) {
				d->tx_fifo[d->tx_count++] = tx[i];
			} else {
				d->fifo_overflows++;
				raise_chip_locked(d, CHIP_FIFO);
			}
		}
		fifo_update_locked(d);
	} else if (tx[0] == C_READ_RX_FIFO) {
		for (size_t i = 1; i < ntx; i++) {
			if (d->rx_count > 0) {
				rx[i] = d->rx_fifo[0];
				memmove(d->rx_fifo, d->rx_fifo + 1, --d->rx_count);
			} else {
				d->fifo_underflows++;
				raise_chip_locked(d, CHIP_FIFO);
			}
		}
		fifo_update_locked(d);
	} else if (!d->cts_ready && !d->never_cts) {
		d->cmds_while_busy++; /* the chip ignores a command sent before CTS */
	} else if (!d->never_cts || d->cts_ready) {
		exec_cmd_locked(d, tx, ntx);
	}
	k_spin_unlock(&d->lock, key);

	if (rx_bufs != NULL) {
		size_t off = 0;

		for (size_t i = 0; i < rx_bufs->count; i++) {
			if (rx_bufs->buffers[i].buf != NULL) {
				memcpy(rx_bufs->buffers[i].buf, rx + off, rx_bufs->buffers[i].len);
			}
			off += rx_bufs->buffers[i].len;
		}
	}
	return 0;
}

static const struct spi_emul_api rfm26w_emul_spi_api = {
	.io = rfm26w_emul_io,
};

/* ------------------------------------------------------------------------------------------ */
/* Test control                                                                               */
/* ------------------------------------------------------------------------------------------ */

void rfm26w_emul_reset(struct rfm26w_emul_data *d)
{
	k_spinlock_key_t key = k_spin_lock(&d->lock);

	d->us_per_byte = 833; /* 9.6 kb/s */
	d->preamble_bytes = 20;
	d->cts_delay_polls = 0;
	d->never_cts = false;
	d->fail_next_io = false;
	d->stall_tx = false;
	d->corrupt_props = false;
	d->leave_rx_after_packet = false;
	d->curr_rssi = 0;
	d->credit_us = 0;
	memset(d->cmd_count, 0, sizeof(d->cmd_count));
	d->cmds_while_busy = 0;
	d->bad_cmds = 0;
	d->ant_violations = 0;
	d->fifo_underflows = 0;
	d->fifo_overflows = 0;
	d->start_rx_count = 0;
	d->start_tx_count = 0;
	d->power_ups = 0;
	d->resets = 0;
	d->last_tx_len_arg = 0;
	d->last_rx_len_arg = 0;
	d->rx_q_head = d->rx_q_count = 0;
	d->tx_d_head = d->tx_d_count = 0;
	d->rx_session = false;
	/* leave the radio's own state alone: SDN and POR decide that */
	k_spin_unlock(&d->lock, key);
}

int rfm26w_emul_inject_rx(struct rfm26w_emul_data *d, const uint8_t *block, size_t len,
			  uint8_t rssi_raw)
{
	k_spinlock_key_t key;

	if (len == 0 || len > EMUL_MAX_BLOCK) {
		return -EINVAL;
	}
	key = k_spin_lock(&d->lock);
	if (d->rx_q_count == EMUL_QUEUE_DEPTH) {
		k_spin_unlock(&d->lock, key);
		return -ENOMEM;
	}
	queue_push(d->rx_queue, &d->rx_q_head, &d->rx_q_count, block, len, rssi_raw);
	k_spin_unlock(&d->lock, key);
	return 0;
}

int rfm26w_emul_take_tx(struct rfm26w_emul_data *d, uint8_t *out, size_t *len)
{
	k_spinlock_key_t key = k_spin_lock(&d->lock);
	int ret = -ENOENT;

	if (d->tx_d_count > 0) {
		struct emul_block *b = &d->tx_done[d->tx_d_head];

		memcpy(out, b->data, b->len);
		*len = b->len;
		d->tx_d_head = (d->tx_d_head + 1) % EMUL_QUEUE_DEPTH;
		d->tx_d_count--;
		ret = 0;
	}
	k_spin_unlock(&d->lock, key);
	return ret;
}

uint8_t rfm26w_emul_prop(struct rfm26w_emul_data *d, uint8_t group, uint8_t idx)
{
	k_spinlock_key_t key = k_spin_lock(&d->lock);
	uint8_t v = (group < EMUL_PROP_GROUPS) ? d->props[group][idx] : 0;

	k_spin_unlock(&d->lock, key);
	return v;
}

/* ------------------------------------------------------------------------------------------ */
/* Registration                                                                               */
/* ------------------------------------------------------------------------------------------ */

static int rfm26w_emul_init(const struct emul *target, const struct device *parent)
{
	struct rfm26w_emul_data *d = target->data;

	ARG_UNUSED(parent);
	g_cfg = target->cfg;
	g_data = d;

	chip_reset_locked(d);
	rfm26w_emul_reset(d);
	gpio_init_callback(&d->sdn_cb, sdn_changed, BIT(g_cfg->sdn.pin));
	(void)gpio_add_callback(g_cfg->sdn.port, &d->sdn_cb);
	k_timer_init(&d->timer, tick, NULL);
	k_timer_start(&d->timer, K_MSEC(1), K_MSEC(1));
	return 0;
}

#define RFM26W_EMUL(n)                                                                             \
	static struct rfm26w_emul_data rfm26w_emul_data_##n;                                       \
	static const struct rfm26w_emul_cfg rfm26w_emul_cfg_##n = {                                \
		.irq = GPIO_DT_SPEC_INST_GET(n, int_gpios),                                        \
		.sdn = GPIO_DT_SPEC_INST_GET(n, sdn_gpios),                                        \
		.tx_ant = GPIO_DT_SPEC_INST_GET_OR(n, tx_ant_gpios, {0}),                          \
		.rx_ant = GPIO_DT_SPEC_INST_GET_OR(n, rx_ant_gpios, {0}),                          \
	};                                                                                         \
	EMUL_DT_INST_DEFINE(n, rfm26w_emul_init, &rfm26w_emul_data_##n, &rfm26w_emul_cfg_##n,      \
			    &rfm26w_emul_spi_api, NULL)

DT_INST_FOREACH_STATUS_OKAY(RFM26W_EMUL)
