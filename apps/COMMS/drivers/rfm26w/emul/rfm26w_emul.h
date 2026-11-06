/**
 * @file rfm26w_emul.h
 * @brief Software model of the RFM26W / Si446x on an emulated SPI bus, for testing the driver
 * @ingroup comms
 *
 * The model is written from AN625 and the RFM26W datasheet, independently of the driver, so a
 * driver mistake and a model mistake are unlikely to be the same mistake. It is NOT the chip: it
 * checks the driver's logic (command sequencing, FIFO streaming, interrupt handling, locking and
 * error paths), not how the real chip behaves. PLANNED.md section 5.5 lists what only hardware can
 * settle.
 *
 * Time model: a 1 ms timer moves bytes through the FIFOs at `us_per_byte` (833 is 9.6 kb/s). The
 * model raises the same interrupts the chip would and drives the nIRQ GPIO.
 */
#ifndef RFM26W_EMUL_H_
#define RFM26W_EMUL_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi_emul.h>
#include <zephyr/kernel.h>

#define EMUL_FIFO_SIZE   64
#define EMUL_MAX_BLOCK   256
#define EMUL_QUEUE_DEPTH 8
#define EMUL_PROP_GROUPS 0x50

struct emul_block {
	uint8_t data[EMUL_MAX_BLOCK];
	size_t len;
	uint8_t rssi_raw; /**< latched RSSI once the sync word has been seen */
};

struct rfm26w_emul_data {
	struct k_spinlock lock;
	struct k_timer timer;
	struct gpio_callback sdn_cb;
	uint32_t credit_us;

	/* ---- chip state (lost when SDN goes high) ---- */
	bool powered_up;      /**< POWER_UP has been accepted */
	uint8_t state;        /**< 1 sleep, 2 SPI active, 3 ready, 5/6 tune, 7 TX, 8 RX */
	uint32_t xo_hz;       /**< from the last POWER_UP */
	bool xo_tcxo;
	uint8_t props[EMUL_PROP_GROUPS][256];
	uint8_t tx_fifo[EMUL_FIFO_SIZE];
	uint8_t tx_count;
	uint8_t rx_fifo[EMUL_FIFO_SIZE];
	uint8_t rx_count;
	bool ae_prev;         /**< TX almost-empty condition on the previous look */
	bool af_prev;         /**< RX almost-full condition on the previous look */
	uint8_t pend_ph;
	uint8_t pend_modem;
	uint8_t pend_chip;
	bool irq_asserted;    /**< what the nIRQ pin is being driven to */
	uint8_t latch_rssi;

	/* command channel */
	uint8_t reply[16];
	size_t reply_len;
	bool cts_ready;
	uint32_t polls_left;

	/* transmit session */
	bool tx_active;
	uint16_t tx_len;
	uint16_t tx_sent;
	uint8_t tx_complete_state;
	uint8_t tx_cur[EMUL_MAX_BLOCK];

	/* receive session */
	uint16_t rx_len;      /**< RX_LEN from START_RX */
	uint8_t rx_state_valid;
	bool rx_session;
	uint16_t rx_delivered;
	uint32_t rx_preamble_left;
	struct emul_block rx_cur;

	/* ---- queues the test uses ---- */
	struct emul_block rx_queue[EMUL_QUEUE_DEPTH];
	size_t rx_q_head, rx_q_count;
	struct emul_block tx_done[EMUL_QUEUE_DEPTH];
	size_t tx_d_head, tx_d_count;

	/* ---- knobs the test sets (cleared by rfm26w_emul_reset) ---- */
	uint32_t us_per_byte;       /**< byte time, 833 is 9.6 kb/s */
	uint32_t preamble_bytes;    /**< preamble and sync time before a block's data */
	uint32_t cts_delay_polls;   /**< CTS polls answered "not ready" after each command */
	bool never_cts;             /**< chip never reports CTS */
	bool fail_next_io;          /**< next SPI transfer fails with -EIO */
	bool stall_tx;              /**< a started transmission never drains or completes */
	bool corrupt_props;         /**< SET_PROPERTY stores the wrong value */
	bool leave_rx_after_packet; /**< chip drops to Ready after a block regardless of next state */
	uint8_t curr_rssi;          /**< CURR_RSSI reported by GET_MODEM_STATUS */

	/* ---- observations ---- */
	uint32_t cmd_count[256];
	uint32_t cmds_while_busy;   /**< a command arrived before CTS: a driver sequencing bug */
	uint32_t bad_cmds;          /**< unknown command, bad arguments, or command before POWER_UP */
	uint32_t ant_violations;    /**< START_TX/START_RX with the antenna pins in the wrong state */
	uint32_t fifo_underflows;
	uint32_t fifo_overflows;
	uint32_t start_rx_count;
	uint32_t start_tx_count;
	uint32_t power_ups;
	uint32_t resets;            /**< times SDN went high */
	uint16_t last_tx_len_arg;
	uint16_t last_rx_len_arg;
};

/** The one emulated radio, from the devicetree node `rfm26w0`. */
#define RFM26W_EMUL_DATA() ((struct rfm26w_emul_data *)EMUL_DT_GET(DT_NODELABEL(rfm26w0))->data)

/** Clear knobs, observations and queues, and set the nIRQ pin to its idle level. */
void rfm26w_emul_reset(struct rfm26w_emul_data *d);

/** Queue a block to arrive over the air. It is received only while the chip is in RX. */
int rfm26w_emul_inject_rx(struct rfm26w_emul_data *d, const uint8_t *block, size_t len,
			  uint8_t rssi_raw);

/** Take the oldest block the radio has transmitted. Returns 0, or -ENOENT if none. */
int rfm26w_emul_take_tx(struct rfm26w_emul_data *d, uint8_t *out, size_t *len);

/** Read a property from the model. */
uint8_t rfm26w_emul_prop(struct rfm26w_emul_data *d, uint8_t group, uint8_t idx);

#endif /* RFM26W_EMUL_H_ */
