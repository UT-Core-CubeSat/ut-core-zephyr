/**
 * @file rfm26w.c
 * @brief RFM26W (Si446x-class) radio driver
 * @ingroup comms
 *
 * Implements the SPI command layer (CTS polling, properties, POWER_UP, PART_INFO), start/shutdown,
 * configure, TX power, block transmit and receive through the FIFOs with nIRQ handling in a
 * dedicated work queue, and RSSI. Still stubs that return -ENOTSUP: the CW test mode
 * (rfm26w_drv_test_cw) and the power management action, both marked "phase 4".
 *
 * !!! ASSUMPTIONS THAT MAY NEED ADJUSTING !!!
 *  - The reference clock is ASSUMED to be a 30 MHz crystal (devicetree `xo-frequency-hz`,
 *    `xo-tcxo`). It is not confirmed for the module. If it differs, change the devicetree AND
 *    regenerate the radio profile (PLANNED.md section 5.4, "Regenerating the radio profile").
 *  - The radio profile (rfm26w_profiles.c) is an empty placeholder until WDS output exists.
 *  - The dBm to PA level conversion is an unmeasured placeholder.
 *
 * Locking: every public operation (rfm26w_drv_*) takes data->lock. The helpers that touch the
 * radio (SPI, FIFO, state and interrupt handling) expect the caller to hold it. The exceptions are
 * rfm26w_irq_isr() (takes no lock, only defers), rfm26w_irq_work() and rfm26w_init().
 */
#define DT_DRV_COMPAT hoperf_rfm26w

#include <errno.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/byteorder.h>

#include <ut/drivers/rfm26w.h>

#include "rfm26w_priv.h"
#include "rfm26w_profiles.h"

/** @cond */ /* Hidden from Doxygen, or it will mistake this as a function */
LOG_MODULE_REGISTER(rfm26w, CONFIG_RFM26W_LOG_LEVEL);
/** @endcond */

/** Constant per-instance configuration, from the devicetree. */
struct rfm26w_cfg {
	struct spi_dt_spec bus;      /**< SPI device (chip select, frequency) */
	struct gpio_dt_spec irq;     /**< nIRQ, active low */
	struct gpio_dt_spec sdn;     /**< SDN, active (high) means shut down */
	struct gpio_dt_spec tx_ant;  /**< optional TX-ANT switch pin */
	struct gpio_dt_spec rx_ant;  /**< optional RX-ANT switch pin */
	uint32_t xo_hz;              /**< reference clock, ASSUMED 30 MHz, see the file comment */
	bool xo_tcxo;                /**< reference clock is a TCXO instead of a crystal */
	uint16_t block_len;          /**< fixed block length in bytes */
	k_thread_stack_t *workq_stack; /**< stack of the radio work queue thread */
	size_t workq_stack_size;
};

/** What the radio is doing with its FIFOs right now. */
enum rfm26w_op {
	RFM26W_OP_IDLE,
	RFM26W_OP_TX, /**< a block is being sent, the work handler feeds the TX FIFO */
};

/** Mutable per-instance state. */
struct rfm26w_data {
	const struct device *dev;     /**< back pointer for the work handler */
	struct gpio_callback irq_cb;  /**< nIRQ GPIO callback */
	struct k_work irq_work;       /**< runs the SPI side of interrupt handling */
	struct k_work_q workq;        /**< own thread: the FIFO deadlines must not wait behind other work */
	/** Serializes SPI command sequences and protects the state below. Held briefly, never while
	 *  waiting for a block to finish, because the work handler needs it. Reentrant. */
	struct k_mutex lock;
	struct k_mutex tx_lock;       /**< one send() at a time */
	struct rfm26w_stats stats;    /**< counters since start */
	enum rfm26w_profile profile;  /**< profile loaded by start or configure */
	bool started;                 /**< SDN released and the chip is powered up */
	uint8_t rssi_comp;            /**< MODEM_RSSI_COMP read at start, for the dBm conversion */

	/* transmit */
	enum rfm26w_op op;
	const uint8_t *tx_buf;        /**< caller's block, valid while op == TX */
	size_t tx_off;                /**< bytes already written to the TX FIFO */
	int tx_result;                /**< outcome handed back to send() */
	struct k_sem tx_done;

	/* receive */
	bool rx_armed;                /**< the radio is listening (or will be once a TX ends) */
	bool rx_in_progress;          /**< sync word seen, block not complete */
	uint8_t *rx_buf;              /**< block being assembled, or held while rx_pending */
	size_t rx_off;
	uint8_t rx_rssi_raw;          /**< latched RSSI read at sync detect */
	bool rx_pending;              /**< rx_buf holds a complete block for recv() */
	struct rfm26w_rx_info rx_info;/**< info for the pending block */
	struct k_sem rx_sem;
	rfm26w_rx_cb rx_cb;           /**< asynchronous receive callback, or NULL */
	void *rx_cb_data;
};

/* ------------------------------------------------------------------------------------------ */
/* SPI command layer                                                                          */
/* ------------------------------------------------------------------------------------------ */

/* Time spent busy-waiting between CTS polls before falling back to sleeping 1 ms. CTS is
 * typically ready after about 20 us (datasheet section 3.1). */
#define CTS_SPIN_US      1000
#define CTS_SPIN_STEP_US 20

/* Write one command (command byte plus arguments) in a single chip-select window. */
static int rfm26w_spi_write(const struct device *dev, const uint8_t *buf, size_t len)
{
	const struct rfm26w_cfg *cfg = dev->config;
	struct rfm26w_data *data = dev->data;
	const struct spi_buf b = {.buf = (void *)buf, .len = len};
	const struct spi_buf_set set = {.buffers = &b, .count = 1};
	int ret = spi_write_dt(&cfg->bus, &set);

	if (ret < 0) {
		data->stats.spi_errors++;
		LOG_ERR("SPI write failed: %d", ret);
		return -EIO;
	}
	return 0;
}

/*
 * Poll READ_CMD_BUFF until CTS (byte 1) reads 0xFF, then copy out the reply bytes that follow it.
 * Each poll is one chip-select window of 2 + nreply bytes, which is how AN625 describes reading a
 * reply (command byte, CTS, reply). timeout_ms of 0 means a single attempt.
 */
static int rfm26w_read_reply(const struct device *dev, uint8_t *reply, size_t nreply,
			     uint32_t timeout_ms)
{
	const struct rfm26w_cfg *cfg = dev->config;
	struct rfm26w_data *data = dev->data;
	uint8_t tx[2 + RFM26W_REPLY_MAX] = {RFM26W_CMD_READ_CMD_BUFF};
	uint8_t rx[2 + RFM26W_REPLY_MAX];
	const size_t xfer = 2 + nreply;
	const struct spi_buf tx_buf = {.buf = tx, .len = xfer};
	const struct spi_buf rx_buf = {.buf = rx, .len = xfer};
	const struct spi_buf_set tx_set = {.buffers = &tx_buf, .count = 1};
	const struct spi_buf_set rx_set = {.buffers = &rx_buf, .count = 1};
	const int64_t deadline = k_uptime_get() + timeout_ms;
	uint32_t spun_us = 0;

	for (;;) {
		int ret = spi_transceive_dt(&cfg->bus, &tx_set, &rx_set);

		if (ret < 0) {
			data->stats.spi_errors++;
			LOG_ERR("SPI CTS poll failed: %d", ret);
			return -EIO;
		}
		if (rx[1] == RFM26W_CTS_READY) {
			if (nreply > 0) {
				memcpy(reply, &rx[2], nreply);
			}
			return 0;
		}
		if (k_uptime_get() >= deadline) {
			data->stats.cts_timeouts++;
			return -ETIMEDOUT;
		}
		if (spun_us < CTS_SPIN_US) {
			k_busy_wait(CTS_SPIN_STEP_US);
			spun_us += CTS_SPIN_STEP_US;
		} else {
			k_msleep(1);
		}
	}
}

/*
 * Send a command, wait for CTS, and read nreply bytes of reply. Every command waits for CTS even
 * when it has no reply, so the next command can follow immediately.
 */
static int rfm26w_cmd(const struct device *dev, uint8_t cmd, const uint8_t *args, size_t nargs,
		      uint8_t *reply, size_t nreply, uint32_t timeout_ms)
{
	uint8_t buf[RFM26W_CMD_MAX_LEN];
	int ret;

	if (nargs > RFM26W_CMD_MAX_ARGS || nreply > RFM26W_REPLY_MAX) {
		return -EINVAL;
	}
	buf[0] = cmd;
	if (nargs > 0) {
		memcpy(&buf[1], args, nargs);
	}
	ret = rfm26w_spi_write(dev, buf, 1 + nargs);
	if (ret < 0) {
		return ret;
	}
	ret = rfm26w_read_reply(dev, reply, nreply, timeout_ms);
	if (ret == -ETIMEDOUT) {
		LOG_ERR("CTS timeout after command 0x%02x", cmd);
	}
	return ret;
}

/* SET_PROPERTY: `n` consecutive properties of one group starting at `start`. */
static int rfm26w_set_props(const struct device *dev, uint8_t group, uint8_t start,
			    const uint8_t *vals, uint8_t n)
{
	uint8_t args[3 + RFM26W_PROP_MAX];

	if (n == 0 || n > RFM26W_PROP_MAX) {
		return -EINVAL;
	}
	args[0] = group;
	args[1] = n;
	args[2] = start;
	memcpy(&args[3], vals, n);
	return rfm26w_cmd(dev, RFM26W_CMD_SET_PROPERTY, args, 3u + n, NULL, 0,
			  CONFIG_RFM26W_CTS_TIMEOUT_MS);
}

/* GET_PROPERTY: read back `n` consecutive properties. */
static int rfm26w_get_props(const struct device *dev, uint8_t group, uint8_t start, uint8_t *out,
			    uint8_t n)
{
	const uint8_t args[3] = {group, n, start};

	if (n == 0 || n > RFM26W_PROP_MAX) {
		return -EINVAL;
	}
	return rfm26w_cmd(dev, RFM26W_CMD_GET_PROPERTY, args, sizeof(args), out, n,
			  CONFIG_RFM26W_CTS_TIMEOUT_MS);
}

/*
 * POWER_UP with the reference clock from the devicetree. The chip does not check the value, so
 * a wrong xo-frequency-hz gives a wrong carrier and data rate with no error (see the file comment).
 */
static int rfm26w_power_up(const struct device *dev)
{
	const struct rfm26w_cfg *cfg = dev->config;
	uint8_t args[6];

	args[0] = RFM26W_BOOT_MAIN_IMAGE;
	args[1] = cfg->xo_tcxo ? RFM26W_XTAL_TCXO : 0;
	sys_put_be32(cfg->xo_hz, &args[2]);
	return rfm26w_cmd(dev, RFM26W_CMD_POWER_UP, args, sizeof(args), NULL, 0,
			  CONFIG_RFM26W_POWER_UP_TIMEOUT_MS);
}

/* PART_INFO: reply is CHIPREV, PART[15:8], PART[7:0], PBUILD, ID[2], CUSTOMER, ROMID. */
static int rfm26w_part_info(const struct device *dev, uint16_t *part, uint8_t *chip_rev)
{
	uint8_t r[8];
	int ret = rfm26w_cmd(dev, RFM26W_CMD_PART_INFO, NULL, 0, r, sizeof(r),
			     CONFIG_RFM26W_CTS_TIMEOUT_MS);

	if (ret < 0) {
		return ret;
	}
	*chip_rev = r[0];
	*part = sys_get_be16(&r[1]);
	return 0;
}

/* GET_INT_STATUS with every clear argument 0: a clear bit clears that pending interrupt, a set
 * bit leaves it pending (AN625 3.2.12). This drops all pending interrupts and releases nIRQ. */
static const uint8_t rfm26w_clear_all_args[3] = {0x00, 0x00, 0x00};

/* GET_INT_STATUS reply: INT_PEND, INT_STATUS, PH_PEND, PH_STATUS, MODEM_PEND, MODEM_STATUS,
 * CHIP_PEND, CHIP_STATUS. */
#define RFM26W_INT_REPLY_LEN 8

static int rfm26w_clear_interrupts(const struct device *dev)
{
	uint8_t r[RFM26W_INT_REPLY_LEN];

	return rfm26w_cmd(dev, RFM26W_CMD_GET_INT_STATUS, rfm26w_clear_all_args,
			  sizeof(rfm26w_clear_all_args), r, sizeof(r), CONFIG_RFM26W_CTS_TIMEOUT_MS);
}

/* ------------------------------------------------------------------------------------------ */
/* Profile loading                                                                            */
/* ------------------------------------------------------------------------------------------ */

/*
 * Send a WDS-format table to the radio, then (optionally) read every SET_PROPERTY back.
 * A POWER_UP entry is not sent: the driver powers up from the devicetree. If the entry names a
 * different reference clock than the devicetree, the table and the devicetree disagree and the
 * table needs regenerating, so that is reported.
 */
static int rfm26w_load_profile(const struct device *dev, enum rfm26w_profile profile)
{
	const struct rfm26w_cfg *cfg = dev->config;
	const uint8_t *tbl;
	size_t len;
	size_t pos = 0;
	struct rfm26w_tbl_entry e;
	int r;
	int ret = rfm26w_profile_table(profile, &tbl, &len);

	if (ret < 0) {
		LOG_ERR("no table for profile %d", (int)profile);
		return ret;
	}

	while ((r = rfm26w_tbl_next(tbl, len, &pos, &e)) == 1) {
		if (e.cmd == RFM26W_CMD_POWER_UP) {
			if (e.nargs == 6 && (sys_get_be32(&e.args[2]) != cfg->xo_hz ||
					     (bool)(e.args[1] & RFM26W_XTAL_TCXO) != cfg->xo_tcxo)) {
				LOG_WRN("profile was generated for XO %u Hz but the devicetree says "
					"%u Hz: regenerate the profile (PLANNED.md 5.4)",
					(unsigned int)sys_get_be32(&e.args[2]),
					(unsigned int)cfg->xo_hz);
			}
			continue;
		}
		ret = rfm26w_cmd(dev, e.cmd, e.args, e.nargs, NULL, 0, CONFIG_RFM26W_CTS_TIMEOUT_MS);
		if (ret < 0) {
			LOG_ERR("profile command 0x%02x failed: %d", e.cmd, ret);
			return ret;
		}
	}
	if (r < 0) {
		LOG_ERR("profile table is malformed at offset %u", (unsigned int)pos);
		return -EINVAL;
	}

	if (IS_ENABLED(CONFIG_RFM26W_VERIFY_PROFILE)) {
		pos = 0;
		while ((r = rfm26w_tbl_next(tbl, len, &pos, &e)) == 1) {
			uint8_t back[RFM26W_PROP_MAX];

			if (e.cmd != RFM26W_CMD_SET_PROPERTY || e.nargs < 4 ||
			    e.args[1] == 0 || e.args[1] > RFM26W_PROP_MAX ||
			    e.nargs != 3u + e.args[1]) {
				continue;
			}
			ret = rfm26w_get_props(dev, e.args[0], e.args[2], back, e.args[1]);
			if (ret < 0) {
				return ret;
			}
			if (memcmp(back, &e.args[3], e.args[1]) != 0) {
				LOG_ERR("property read-back mismatch, group 0x%02x start 0x%02x",
					e.args[0], e.args[2]);
				return -EIO;
			}
		}
	}
	return 0;
}

/*
 * The settings the driver owns, applied after every profile load (start and configure): FIFO
 * thresholds and interrupt enables, which the WDS output does not set the way the driver needs
 * (the sample output switches every interrupt off), and the RSSI offset the profile uses, needed to
 * turn readings into dBm. A profile load overwrites all of these, so this must follow each one.
 */
static int rfm26w_apply_driver_props(const struct device *dev)
{
	struct rfm26w_data *data = dev->data;
	const uint8_t thr[2] = {CONFIG_RFM26W_TX_THRESHOLD, CONFIG_RFM26W_RX_THRESHOLD};
	const uint8_t ints[4] = {
		RFM26W_INT_EN_PH | RFM26W_INT_EN_MODEM | RFM26W_INT_EN_CHIP,
		RFM26W_PH_RX_FIFO_ALMOST_FULL | RFM26W_PH_TX_FIFO_ALMOST_EMPTY | RFM26W_PH_PACKET_RX |
			RFM26W_PH_PACKET_SENT,
		RFM26W_MODEM_SYNC_DETECT,
		RFM26W_CHIP_FIFO_ERROR,
	};
	int ret;

	ret = rfm26w_set_props(dev, RFM26W_PROP_GROUP_PKT, RFM26W_PROP_PKT_TX_THRESHOLD, thr,
			       sizeof(thr));
	if (ret < 0) {
		return ret;
	}
	ret = rfm26w_set_props(dev, RFM26W_PROP_GROUP_INT_CTL, RFM26W_PROP_INT_CTL_ENABLE, ints,
			       sizeof(ints));
	if (ret < 0) {
		return ret;
	}
	return rfm26w_get_props(dev, RFM26W_PROP_GROUP_MODEM, RFM26W_PROP_MODEM_RSSI_COMP,
				&data->rssi_comp, 1);
}

/* ------------------------------------------------------------------------------------------ */
/* TX power (placeholder mapping)                                                             */
/* ------------------------------------------------------------------------------------------ */

/* Datasheet table 4 power range. The features list says +20 dBm, so the real range is open
 * (PLANNED.md open item 17). */
#define TX_POWER_MIN_DBM (-40)
#define TX_POWER_MAX_DBM 11

/* PLACEHOLDER. A straight line from the table 4 range onto PA_PWR_LVL 0..127. The real curve
 * is nonlinear and depends on the matching network, and has not been measured. Do not trust the
 * resulting dBm. CONFIG_RFM26W_PA_LEVEL_MAX is what limits the actual output for now. */
static uint8_t rfm26w_dbm_to_pa_level(int8_t dbm)
{
	int lvl = ((dbm - TX_POWER_MIN_DBM) * 127) / (TX_POWER_MAX_DBM - TX_POWER_MIN_DBM);

	if (lvl > CONFIG_RFM26W_PA_LEVEL_MAX) {
		LOG_WRN("PA level %d capped to %d (CONFIG_RFM26W_PA_LEVEL_MAX)", lvl,
			CONFIG_RFM26W_PA_LEVEL_MAX);
		lvl = CONFIG_RFM26W_PA_LEVEL_MAX;
	}
	return (uint8_t)lvl;
}

static int rfm26w_set_tx_power_locked(const struct device *dev, int8_t dbm)
{
	uint8_t lvl;

	if (dbm < TX_POWER_MIN_DBM || dbm > TX_POWER_MAX_DBM) {
		return -EINVAL;
	}
	lvl = rfm26w_dbm_to_pa_level(dbm);
	return rfm26w_set_props(dev, RFM26W_PROP_GROUP_PA, RFM26W_PROP_PA_PWR_LVL, &lvl, 1);
}

/* ------------------------------------------------------------------------------------------ */
/* Interrupt path                                                                             */
/* ------------------------------------------------------------------------------------------ */

/* nIRQ interrupt: only defer. The SPI work must not run in interrupt context. */
static void rfm26w_irq_isr(const struct device *port, struct gpio_callback *cb, uint32_t pins)
{
	struct rfm26w_data *data = CONTAINER_OF(cb, struct rfm26w_data, irq_cb);

	ARG_UNUSED(port);
	ARG_UNUSED(pins);
	k_work_submit_to_queue(&data->workq, &data->irq_work);
}

/* ---- FIFO access (no CTS needed for 0x66 / 0x77, AN625 3.2.25 and 3.2.26) ---- */

/* Append up to 64 bytes to the TX FIFO in one chip-select window. */
static int rfm26w_fifo_write(const struct device *dev, const uint8_t *buf, size_t n)
{
	const struct rfm26w_cfg *cfg = dev->config;
	struct rfm26w_data *data = dev->data;
	const uint8_t cmd = RFM26W_CMD_WRITE_TX_FIFO;
	const struct spi_buf bufs[2] = {
		{.buf = (void *)&cmd, .len = 1},
		{.buf = (void *)buf, .len = n},
	};
	const struct spi_buf_set set = {.buffers = bufs, .count = 2};
	int ret;

	if (n == 0 || n > RFM26W_FIFO_SIZE) {
		return -EINVAL;
	}
	ret = spi_write_dt(&cfg->bus, &set);
	if (ret < 0) {
		data->stats.spi_errors++;
		LOG_ERR("TX FIFO write failed: %d", ret);
		return -EIO;
	}
	return 0;
}

/* Take up to 64 bytes from the RX FIFO in one chip-select window. */
static int rfm26w_fifo_read(const struct device *dev, uint8_t *buf, size_t n)
{
	const struct rfm26w_cfg *cfg = dev->config;
	struct rfm26w_data *data = dev->data;
	uint8_t tx[1 + RFM26W_FIFO_SIZE] = {RFM26W_CMD_READ_RX_FIFO};
	uint8_t rx[1 + RFM26W_FIFO_SIZE];
	const struct spi_buf tx_buf = {.buf = tx, .len = 1 + n};
	const struct spi_buf rx_buf = {.buf = rx, .len = 1 + n};
	const struct spi_buf_set tx_set = {.buffers = &tx_buf, .count = 1};
	const struct spi_buf_set rx_set = {.buffers = &rx_buf, .count = 1};
	int ret;

	if (n == 0 || n > RFM26W_FIFO_SIZE) {
		return -EINVAL;
	}
	ret = spi_transceive_dt(&cfg->bus, &tx_set, &rx_set);
	if (ret < 0) {
		data->stats.spi_errors++;
		LOG_ERR("RX FIFO read failed: %d", ret);
		return -EIO;
	}
	memcpy(buf, &rx[1], n);
	return 0;
}

/* FIFO_INFO without a reset: how many bytes wait in the RX FIFO and how much room the TX FIFO has. */
static int rfm26w_fifo_info(const struct device *dev, uint8_t *rx_count, uint8_t *tx_space)
{
	static const uint8_t args[1] = {0x00};
	uint8_t r[2];
	int ret = rfm26w_cmd(dev, RFM26W_CMD_FIFO_INFO, args, sizeof(args), r, sizeof(r),
			     CONFIG_RFM26W_CTS_TIMEOUT_MS);

	if (ret == 0) {
		*rx_count = r[0];
		*tx_space = r[1];
	}
	return ret;
}

/* Reset only the FIFOs that are named (RFM26W_FIFO_RESET_RX / _TX). Normally only needed for
 * error recovery and when a block starts (AN625 3.2.8). */
static int rfm26w_fifo_reset_mask(const struct device *dev, uint8_t mask)
{
	uint8_t r[2];

	return rfm26w_cmd(dev, RFM26W_CMD_FIFO_INFO, &mask, 1, r, sizeof(r),
			  CONFIG_RFM26W_CTS_TIMEOUT_MS);
}

/* ---- Antenna switch: the two pins are driven to opposite levels (datasheet pin table) ----
 * RX state: RX-ANT low, TX-ANT high. TX state: TX-ANT low, RX-ANT high. Assumes GPIO_ACTIVE_HIGH.
 * Polarity to be confirmed on the board (PLANNED.md open item 17). */
static void rfm26w_set_antenna(const struct device *dev, bool tx)
{
	const struct rfm26w_cfg *cfg = dev->config;

	if (gpio_is_ready_dt(&cfg->tx_ant) && gpio_is_ready_dt(&cfg->rx_ant)) {
		(void)gpio_pin_set_dt(&cfg->tx_ant, tx ? 0 : 1);
		(void)gpio_pin_set_dt(&cfg->rx_ant, tx ? 1 : 0);
	}
}

/* ---- State changes ---- */

static int rfm26w_change_state(const struct device *dev, enum rfm26w_state state)
{
	const uint8_t arg = (uint8_t)state;

	return rfm26w_cmd(dev, RFM26W_CMD_CHANGE_STATE, &arg, 1, NULL, 0,
			  CONFIG_RFM26W_CTS_TIMEOUT_MS);
}

/*
 * START_RX: continuous receive of fixed-length blocks. RX_LEN is the block length, so the radio
 * receives with field 1 only and no packet handler features (AN625 3.2.17). All three next states
 * are RX, so the radio keeps listening after a block, a timeout or an invalid packet. Called with
 * the lock held. CTS does not return until the chip is in the RX state.
 */
static int rfm26w_start_rx(const struct device *dev)
{
	const struct rfm26w_cfg *cfg = dev->config;
	uint8_t args[7];

	args[0] = 0; /* channel, the frequency comes from FREQ_CONTROL */
	args[1] = 0; /* start immediately */
	sys_put_be16(cfg->block_len, &args[2]);
	args[4] = RFM26W_STATE_RX; /* RXTIMEOUT_STATE */
	args[5] = RFM26W_STATE_RX; /* RXVALID_STATE */
	args[6] = RFM26W_STATE_RX; /* RXINVALID_STATE */
	return rfm26w_cmd(dev, RFM26W_CMD_START_RX, args, sizeof(args), NULL, 0,
			  CONFIG_RFM26W_CTS_TIMEOUT_MS);
}

/*
 * Begin listening, unless a transmission is under way (it re-arms receive when it ends). Does
 * nothing if already armed: START_RX again would restart the receiver and destroy a block that is
 * arriving.
 */
static int rfm26w_rx_arm_locked(const struct device *dev)
{
	struct rfm26w_data *data = dev->data;
	int ret = 0;

	if (data->rx_armed) {
		return 0;
	}
	if (data->op != RFM26W_OP_TX) {
		rfm26w_set_antenna(dev, false);
		ret = rfm26w_start_rx(dev);
		if (ret < 0) {
			return ret;
		}
	}
	data->rx_armed = true;
	return 0;
}

/* Stop listening and drop any partial block. */
static int rfm26w_rx_disarm_locked(const struct device *dev)
{
	struct rfm26w_data *data = dev->data;
	int ret = 0;

	if (data->rx_armed && data->op != RFM26W_OP_TX) {
		ret = rfm26w_change_state(dev, RFM26W_STATE_READY);
		(void)rfm26w_fifo_reset_mask(dev, RFM26W_FIFO_RESET_RX);
	}
	data->rx_armed = false;
	data->rx_in_progress = false;
	data->rx_off = 0;
	return ret;
}

/*
 * Check the radio is still in the RX state after a block, and start receive again if it is not.
 * The next-state fields should keep it in RX, but how the chip reuses RX_LEN across that
 * transition is unverified, so this makes the receive path independent of that detail.
 * One cheap command per block. Called with the lock held.
 */
static void rfm26w_rx_check_state_locked(const struct device *dev)
{
	uint8_t r[2];
	int ret = rfm26w_cmd(dev, RFM26W_CMD_REQUEST_DEVICE_STATE, NULL, 0, r, sizeof(r),
			     CONFIG_RFM26W_CTS_TIMEOUT_MS);

	if (ret == 0 && (r[0] & 0x0F) != RFM26W_STATE_RX) {
		LOG_DBG("radio left RX (state %u), restarting receive", r[0] & 0x0F);
		(void)rfm26w_start_rx(dev);
	}
}

/* ---- Interrupt handling (runs in the radio's work queue thread, lock held) ---- */

/* A TX or RX error ended the current operation. Put the radio back to a known state. */
static void rfm26w_recover_locked(const struct device *dev)
{
	struct rfm26w_data *data = dev->data;

	(void)rfm26w_change_state(dev, RFM26W_STATE_READY);
	(void)rfm26w_fifo_reset_mask(dev, RFM26W_FIFO_RESET_RX | RFM26W_FIFO_RESET_TX);
	data->rx_in_progress = false;
	data->rx_off = 0;
	rfm26w_set_antenna(dev, false);
	if (data->rx_armed) {
		(void)rfm26w_start_rx(dev);
	}
}

/* TX_FIFO_ALMOST_EMPTY: top the FIFO up with the rest of the block. */
static void rfm26w_tx_refill_locked(const struct device *dev)
{
	struct rfm26w_data *data = dev->data;
	const struct rfm26w_cfg *cfg = dev->config;
	size_t remaining = cfg->block_len - data->tx_off;
	uint8_t rx_count;
	uint8_t space;

	if (remaining == 0) {
		return; /* everything is already in the FIFO, waiting for PACKET_SENT */
	}
	if (rfm26w_fifo_info(dev, &rx_count, &space) < 0) {
		return;
	}
	if (space > remaining) {
		space = (uint8_t)remaining;
	}
	if (space > 0 && rfm26w_fifo_write(dev, &data->tx_buf[data->tx_off], space) == 0) {
		data->tx_off += space;
	}
}

/* PACKET_SENT: the whole block has left the antenna. */
static void rfm26w_tx_complete_locked(const struct device *dev, int result)
{
	struct rfm26w_data *data = dev->data;

	if (result == 0) {
		data->stats.tx_blocks++;
		rfm26w_set_antenna(dev, false);
		if (data->rx_armed) {
			(void)rfm26w_start_rx(dev);
		}
	} else {
		data->op = RFM26W_OP_IDLE; /* so recover re-arms receive */
		rfm26w_recover_locked(dev);
	}
	data->op = RFM26W_OP_IDLE;
	data->tx_buf = NULL;
	data->tx_result = result;
	k_sem_give(&data->tx_done);
}

/* Move whatever the RX FIFO holds into the block being assembled. */
static void rfm26w_rx_drain_locked(const struct device *dev)
{
	const struct rfm26w_cfg *cfg = dev->config;
	struct rfm26w_data *data = dev->data;
	uint8_t count;
	uint8_t space;
	uint8_t tmp[RFM26W_FIFO_SIZE];

	if (rfm26w_fifo_info(dev, &count, &space) < 0) {
		return;
	}
	while (count > 0 && data->rx_off < cfg->block_len) {
		size_t n = MIN((size_t)count, cfg->block_len - data->rx_off);

		if (rfm26w_fifo_read(dev, tmp, n) < 0) {
			return;
		}
		/* While the previous block is still waiting for recv(), the new one is read out of the
		 * FIFO and thrown away, so the FIFO cannot overflow. */
		if (!data->rx_pending) {
			memcpy(&data->rx_buf[data->rx_off], tmp, n);
		}
		data->rx_off += n;
		count -= n;
	}
}

/* A full block has been received (PACKET_RX). Hand it on, then get ready for the next one. */
static void rfm26w_rx_complete_locked(const struct device *dev)
{
	const struct rfm26w_cfg *cfg = dev->config;
	struct rfm26w_data *data = dev->data;
	struct rfm26w_rx_info info;
	rfm26w_rx_cb cb = data->rx_cb;
	void *cb_data = data->rx_cb_data;

	data->rx_in_progress = false;
	if (data->rx_off != cfg->block_len) {
		LOG_WRN("block ended with %u of %u bytes, dropped", (unsigned int)data->rx_off,
			(unsigned int)cfg->block_len);
		data->rx_off = 0;
		rfm26w_rx_check_state_locked(dev);
		return;
	}
	data->rx_off = 0;

	if (data->rx_pending) {
		/* recv() has not picked up the last block yet: this one was discarded. */
		data->stats.rx_overruns++;
		rfm26w_rx_check_state_locked(dev);
		return;
	}

	info.rssi_dbm10 = rfm26w_rssi_to_dbm10(data->rx_rssi_raw, data->rssi_comp);
	info.uptime_ms = k_uptime_get_32();
	data->stats.rx_blocks++;
	rfm26w_rx_check_state_locked(dev);

	if (cb != NULL) {
		/* Called with the lock released so the callback may use the other API calls. It must
		 * not block, and must not call rfm26w_send(), which waits for this thread. */
		k_mutex_unlock(&data->lock);
		cb(dev, data->rx_buf, cfg->block_len, &info, cb_data);
		k_mutex_lock(&data->lock, K_FOREVER);
	} else {
		data->rx_info = info;
		data->rx_pending = true;
		k_sem_give(&data->rx_sem);
	}
}

/* Read and clear the interrupt status, then act on every pending source. */
static void rfm26w_handle_irq_locked(const struct device *dev)
{
	struct rfm26w_data *data = dev->data;
	uint8_t st[RFM26W_INT_REPLY_LEN];
	uint8_t ph;
	uint8_t modem;
	uint8_t chip;

	if (rfm26w_cmd(dev, RFM26W_CMD_GET_INT_STATUS, rfm26w_clear_all_args,
		       sizeof(rfm26w_clear_all_args), st, sizeof(st),
		       CONFIG_RFM26W_CTS_TIMEOUT_MS) < 0) {
		return;
	}
	ph = st[RFM26W_INT_REPLY_PH_PEND];
	modem = st[RFM26W_INT_REPLY_MODEM_PEND];
	chip = st[RFM26W_INT_REPLY_CHIP_PEND];

	if (chip & RFM26W_CHIP_FIFO_ERROR) {
		if (data->op == RFM26W_OP_TX) {
			LOG_ERR("TX FIFO underrun");
			data->stats.tx_underruns++;
			rfm26w_tx_complete_locked(dev, -EIO);
		} else {
			LOG_ERR("RX FIFO overflow, block lost");
			data->stats.rx_overruns++;
			rfm26w_recover_locked(dev);
		}
		return;
	}

	/* The latched RSSI is read at sync detect: the chip clears it at the start of the next RX. */
	if ((modem & RFM26W_MODEM_SYNC_DETECT) && data->op != RFM26W_OP_TX) {
		uint8_t ms[8];

		data->rx_in_progress = true;
		data->rx_off = 0;
		if (rfm26w_cmd(dev, RFM26W_CMD_GET_MODEM_STATUS, NULL, 0, ms, sizeof(ms),
			       CONFIG_RFM26W_CTS_TIMEOUT_MS) == 0) {
			data->rx_rssi_raw = ms[3]; /* LATCH_RSSI */
		}
	}

	if (data->op == RFM26W_OP_TX) {
		if (ph & RFM26W_PH_TX_FIFO_ALMOST_EMPTY) {
			rfm26w_tx_refill_locked(dev);
		}
		if (ph & RFM26W_PH_PACKET_SENT) {
			rfm26w_tx_complete_locked(dev, 0);
		}
		return;
	}

	if (ph & (RFM26W_PH_RX_FIFO_ALMOST_FULL | RFM26W_PH_PACKET_RX)) {
		rfm26w_rx_drain_locked(dev);
	}
	if (ph & RFM26W_PH_PACKET_RX) {
		rfm26w_rx_complete_locked(dev);
	}
}

/*
 * nIRQ is active low and stays low while anything is pending, but the GPIO interrupt is edge
 * triggered. So keep handling until the pin goes inactive. An edge that arrives while this runs
 * submits the work again, so nothing is lost.
 */
static void rfm26w_irq_work(struct k_work *work)
{
	struct rfm26w_data *data = CONTAINER_OF(work, struct rfm26w_data, irq_work);
	const struct rfm26w_cfg *cfg = data->dev->config;
	int guard = 64; /* never spin here forever if the line is stuck low */

	k_mutex_lock(&data->lock, K_FOREVER);
	while (data->started && guard-- > 0 && gpio_pin_get_dt(&cfg->irq) > 0) {
		rfm26w_handle_irq_locked(data->dev);
	}
	k_mutex_unlock(&data->lock);
}

/* ------------------------------------------------------------------------------------------ */
/* Driver API                                                                                 */
/* ------------------------------------------------------------------------------------------ */

static int rfm26w_start_locked(const struct device *dev)
{
	const struct rfm26w_cfg *cfg = dev->config;
	struct rfm26w_data *data = dev->data;
	uint16_t part;
	uint8_t rev;
	int ret;

	if (data->started) {
		return 0;
	}

	ret = gpio_pin_set_dt(&cfg->sdn, 0);
	if (ret < 0) {
		return ret;
	}
	/* Power-on reset and internal calibration (datasheet table 9: 20 ms to a ready radio). */
	k_msleep(CONFIG_RFM26W_STARTUP_DELAY_MS);

	LOG_INF("powering up with reference clock %u Hz (%s), ASSUMED, see the devicetree binding",
		(unsigned int)cfg->xo_hz, cfg->xo_tcxo ? "TCXO" : "crystal");

	ret = rfm26w_power_up(dev);
	if (ret < 0) {
		goto fail;
	}
	ret = rfm26w_part_info(dev, &part, &rev);
	if (ret < 0) {
		goto fail;
	}
	LOG_INF("part 0x%04x, chip revision 0x%02x", part, rev);

	ret = rfm26w_load_profile(dev, data->profile);
	if (ret < 0) {
		goto fail;
	}

	ret = rfm26w_apply_driver_props(dev);
	if (ret < 0) {
		goto fail;
	}

	ret = rfm26w_clear_interrupts(dev);
	if (ret < 0) {
		goto fail;
	}
	ret = rfm26w_fifo_reset_mask(dev, RFM26W_FIFO_RESET_RX | RFM26W_FIFO_RESET_TX);
	if (ret < 0) {
		goto fail;
	}

	memset(&data->stats, 0, sizeof(data->stats));
	data->op = RFM26W_OP_IDLE;
	data->tx_buf = NULL;
	data->rx_armed = false;
	data->rx_in_progress = false;
	data->rx_pending = false;
	data->rx_off = 0;
	data->rx_cb = NULL;
	k_sem_reset(&data->tx_done);
	k_sem_reset(&data->rx_sem);
	data->started = true;

	ret = gpio_pin_interrupt_configure_dt(&cfg->irq, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret < 0) {
		data->started = false;
		goto fail;
	}
	/* An interrupt that was already pending before the edge interrupt was enabled. */
	if (gpio_pin_get_dt(&cfg->irq) > 0) {
		k_work_submit_to_queue(&data->workq, &data->irq_work);
	}
	return 0;

fail:
	(void)gpio_pin_set_dt(&cfg->sdn, 1);
	return ret;
}

static int rfm26w_drv_start(const struct device *dev)
{
	struct rfm26w_data *data = dev->data;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = rfm26w_start_locked(dev);
	k_mutex_unlock(&data->lock);
	return ret;
}

static int rfm26w_drv_shutdown(const struct device *dev)
{
	const struct rfm26w_cfg *cfg = dev->config;
	struct rfm26w_data *data = dev->data;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);
	(void)gpio_pin_interrupt_configure_dt(&cfg->irq, GPIO_INT_DISABLE);
	ret = gpio_pin_set_dt(&cfg->sdn, 1);
	data->started = false;
	data->op = RFM26W_OP_IDLE;
	data->rx_armed = false;
	data->rx_in_progress = false;
	data->rx_pending = false;
	data->rx_cb = NULL;
	/* Wake anyone blocked in send() or recv(): they see the radio is shut down. */
	data->tx_result = -ESHUTDOWN;
	k_sem_give(&data->tx_done);
	k_sem_give(&data->rx_sem);
	k_mutex_unlock(&data->lock);
	return ret;
}

/*
 * Write the profile (if it changed), the driver's own properties, the carrier frequency and the TX
 * power. The radio must not be receiving or transmitting. Called with the lock held.
 */
static int rfm26w_apply_config_locked(const struct device *dev, const struct rfm26w_config *rcfg,
				      uint8_t inte, uint32_t frac, uint8_t band)
{
	struct rfm26w_data *data = dev->data;
	uint8_t freq_props[4];
	int ret;

	if (rcfg->profile != data->profile) {
		ret = rfm26w_load_profile(dev, rcfg->profile);
		if (ret < 0) {
			return ret;
		}
		data->profile = rcfg->profile;
		/* The profile load overwrote the thresholds, interrupt enables and RSSI offset. */
		ret = rfm26w_apply_driver_props(dev);
		if (ret < 0) {
			return ret;
		}
	}

	/* The carrier frequency overrides whatever frequency the profile was generated for. */
	freq_props[0] = inte;
	freq_props[1] = (uint8_t)((frac >> 16) & 0x0F);
	freq_props[2] = (uint8_t)((frac >> 8) & 0xFF);
	freq_props[3] = (uint8_t)(frac & 0xFF);
	ret = rfm26w_set_props(dev, RFM26W_PROP_GROUP_FREQ_CONTROL, 0x00, freq_props,
			       sizeof(freq_props));
	if (ret < 0) {
		return ret;
	}
	ret = rfm26w_set_props(dev, RFM26W_PROP_GROUP_MODEM, RFM26W_PROP_MODEM_CLKGEN_BAND, &band,
			       1);
	if (ret < 0) {
		return ret;
	}

	return rfm26w_set_tx_power_locked(dev, rcfg->tx_power_dbm);
}

/*
 * Apply frequency, TX power and profile. If the radio is listening it is stopped for the change
 * and started again afterwards (even when the change failed), because the new frequency and
 * profile only take effect at the next START_RX. A block that is arriving at that moment is lost.
 * Returns -EBUSY while a block is being transmitted.
 */
static int rfm26w_drv_configure(const struct device *dev, const struct rfm26w_config *rcfg)
{
	const struct rfm26w_cfg *cfg = dev->config;
	struct rfm26w_data *data = dev->data;
	uint8_t inte;
	uint8_t band;
	uint32_t frac;
	bool rearm;
	int ret;

	if (rcfg == NULL) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	if (!data->started) {
		ret = -ESHUTDOWN;
		goto out;
	}
	if (data->op == RFM26W_OP_TX) {
		ret = -EBUSY;
		goto out;
	}

	/* Check everything that can be checked before touching the radio. */
	ret = rfm26w_freq_calc(rcfg->frequency_hz, cfg->xo_hz, &inte, &frac, &band);
	if (ret < 0) {
		LOG_ERR("frequency %u Hz not usable (error %d)", (unsigned int)rcfg->frequency_hz,
			ret);
		ret = (ret == -ERANGE) ? -EINVAL : ret;
		goto out;
	}

	rearm = data->rx_armed;
	if (rearm) {
		ret = rfm26w_rx_disarm_locked(dev);
		if (ret < 0) {
			data->rx_armed = rearm; /* still listening as far as we know */
			goto out;
		}
	}

	ret = rfm26w_apply_config_locked(dev, rcfg, inte, frac, band);

	if (rearm) {
		int rc = rfm26w_rx_arm_locked(dev);

		if (ret == 0) {
			ret = rc;
		}
	}
out:
	k_mutex_unlock(&data->lock);
	return ret;
}

static int rfm26w_drv_get_info(const struct device *dev, struct rfm26w_info *info)
{
	const struct rfm26w_cfg *cfg = dev->config;
	struct rfm26w_data *data = dev->data;
	int ret;

	if (info == NULL) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	if (!data->started) {
		ret = -ESHUTDOWN;
	} else {
		ret = rfm26w_part_info(dev, &info->part, &info->chip_rev);
		if (ret == 0) {
			info->block_len = cfg->block_len;
			info->profile = data->profile;
		}
	}
	k_mutex_unlock(&data->lock);
	return ret;
}

static int rfm26w_drv_set_tx_power(const struct device *dev, int8_t dbm)
{
	struct rfm26w_data *data = dev->data;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = data->started ? rfm26w_set_tx_power_locked(dev, dbm) : -ESHUTDOWN;
	k_mutex_unlock(&data->lock);
	return ret;
}

/*
 * Send one block. The block is written into the TX FIFO a piece at a time: the first 64 bytes
 * here, the rest from the work handler on the almost-empty interrupt. Waits for PACKET_SENT.
 */
static int rfm26w_drv_send(const struct device *dev, const uint8_t *block, size_t len,
			   k_timeout_t timeout)
{
	const struct rfm26w_cfg *cfg = dev->config;
	struct rfm26w_data *data = dev->data;
	const k_timepoint_t deadline = sys_timepoint_calc(timeout);
	uint8_t args[6];
	size_t first;
	int ret;

	if (block == NULL || len != cfg->block_len) {
		return -EMSGSIZE;
	}
	if (k_mutex_lock(&data->tx_lock, timeout) != 0) {
		return -EBUSY; /* another send() is in progress */
	}

	/* Wait for a block that is already coming in. Sending now would destroy it. */
	for (;;) {
		k_mutex_lock(&data->lock, K_FOREVER);
		if (!data->started) {
			ret = -ESHUTDOWN;
			goto unlock_both;
		}
		if (!data->rx_in_progress) {
			break; /* keep the lock */
		}
		k_mutex_unlock(&data->lock);
		if (sys_timepoint_expired(deadline)) {
			ret = -EBUSY;
			goto unlock_tx;
		}
		k_msleep(1);
	}

	/* Lock held, radio idle or listening. Load the first part of the block and start. */
	k_sem_reset(&data->tx_done);
	data->op = RFM26W_OP_TX;
	data->tx_buf = block;
	data->tx_result = 0;
	first = MIN((size_t)RFM26W_FIFO_SIZE, len);

	rfm26w_set_antenna(dev, true);
	ret = rfm26w_fifo_reset_mask(dev, RFM26W_FIFO_RESET_TX);
	if (ret == 0) {
		ret = rfm26w_fifo_write(dev, block, first);
	}
	if (ret == 0) {
		data->tx_off = first;
		args[0] = 0;                                  /* channel */
		args[1] = (uint8_t)(RFM26W_STATE_READY << 4); /* TXCOMPLETE_STATE, start immediately */
		sys_put_be16((uint16_t)len, &args[2]);        /* TX_LEN, nonzero: field 1 only */
		args[4] = 0;                                  /* TX_DELAY */
		args[5] = 0;                                  /* NUM_REPEAT */
		ret = rfm26w_cmd(dev, RFM26W_CMD_START_TX, args, sizeof(args), NULL, 0,
				 CONFIG_RFM26W_CTS_TIMEOUT_MS);
	}
	if (ret < 0) {
		data->op = RFM26W_OP_IDLE;
		data->tx_buf = NULL;
		rfm26w_recover_locked(dev);
		goto unlock_both;
	}
	k_mutex_unlock(&data->lock);

	/* Not holding the lock: the work handler needs it to feed the FIFO and finish. */
	if (k_sem_take(&data->tx_done, sys_timepoint_timeout(deadline)) == 0) {
		ret = data->tx_result;
		goto unlock_tx;
	}

	/* Timed out. The handler may have finished in the meantime, so look at the state again. */
	k_mutex_lock(&data->lock, K_FOREVER);
	if (data->op == RFM26W_OP_TX) {
		data->op = RFM26W_OP_IDLE;
		data->tx_buf = NULL;
		rfm26w_recover_locked(dev);
		ret = -ETIMEDOUT;
	} else {
		ret = data->tx_result;
	}
unlock_both:
	k_mutex_unlock(&data->lock);
unlock_tx:
	k_mutex_unlock(&data->tx_lock);
	return ret;
}

/*
 * Receive one block. The first call (or recv_async) makes the radio listen, and it keeps listening
 * after that. One block is kept for the next recv(). If a second arrives before recv() is called it
 * is dropped and counted in rx_overruns.
 */
static int rfm26w_drv_recv(const struct device *dev, uint8_t *block, size_t len,
			   struct rfm26w_rx_info *info, k_timeout_t timeout)
{
	const struct rfm26w_cfg *cfg = dev->config;
	struct rfm26w_data *data = dev->data;
	int ret;

	if (block == NULL || len < cfg->block_len) {
		return -EMSGSIZE;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	if (!data->started) {
		ret = -ESHUTDOWN;
		goto out;
	}
	if (data->rx_cb != NULL) {
		ret = -EBUSY; /* asynchronous receive is armed */
		goto out;
	}
	ret = rfm26w_rx_arm_locked(dev);
	if (ret < 0) {
		goto out;
	}
	if (!data->rx_pending) {
		k_mutex_unlock(&data->lock);
		if (k_sem_take(&data->rx_sem, timeout) != 0) {
			return -EAGAIN;
		}
		k_mutex_lock(&data->lock, K_FOREVER);
		if (!data->started) {
			ret = -ESHUTDOWN;
			goto out;
		}
		if (!data->rx_pending) {
			ret = -EAGAIN;
			goto out;
		}
	}
	memcpy(block, data->rx_buf, cfg->block_len);
	if (info != NULL) {
		*info = data->rx_info;
	}
	data->rx_pending = false;
	k_sem_reset(&data->rx_sem);
	ret = 0;
out:
	k_mutex_unlock(&data->lock);
	return ret;
}

static int rfm26w_drv_recv_async(const struct device *dev, rfm26w_rx_cb cb, void *user_data)
{
	struct rfm26w_data *data = dev->data;
	int ret = 0;

	k_mutex_lock(&data->lock, K_FOREVER);
	if (!data->started) {
		ret = -ESHUTDOWN;
	} else if (cb == NULL) {
		/* Cancel only an asynchronous receive. Without one, the radio may have been armed by
		 * recv(), and a thread may be waiting in it. */
		if (data->rx_cb != NULL) {
			data->rx_cb = NULL;
			ret = rfm26w_rx_disarm_locked(dev);
		}
	} else if (data->rx_cb != NULL) {
		ret = -EBUSY; /* already armed: cancel with NULL first */
	} else {
		data->rx_pending = false; /* a block left over from recv() is dropped */
		data->rx_cb = cb;
		data->rx_cb_data = user_data;
		ret = rfm26w_rx_arm_locked(dev);
		if (ret < 0) {
			data->rx_cb = NULL;
		}
	}
	k_mutex_unlock(&data->lock);
	return ret;
}

/* Instantaneous RSSI (CURR_RSSI). Only meaningful while the radio is listening. */
static int rfm26w_drv_read_rssi(const struct device *dev, int16_t *rssi_dbm10)
{
	struct rfm26w_data *data = dev->data;
	uint8_t ms[8];
	int ret;

	if (rssi_dbm10 == NULL) {
		return -EINVAL;
	}
	k_mutex_lock(&data->lock, K_FOREVER);
	if (!data->started) {
		ret = -ESHUTDOWN;
	} else {
		ret = rfm26w_cmd(dev, RFM26W_CMD_GET_MODEM_STATUS, NULL, 0, ms, sizeof(ms),
				 CONFIG_RFM26W_CTS_TIMEOUT_MS);
		if (ret == 0) {
			*rssi_dbm10 = rfm26w_rssi_to_dbm10(ms[2], data->rssi_comp);
		}
	}
	k_mutex_unlock(&data->lock);
	return ret;
}

static int rfm26w_drv_get_stats(const struct device *dev, struct rfm26w_stats *stats)
{
	struct rfm26w_data *data = dev->data;

	if (stats == NULL) {
		return -EINVAL;
	}
	k_mutex_lock(&data->lock, K_FOREVER);
	*stats = data->stats;
	k_mutex_unlock(&data->lock);
	return 0;
}

static int rfm26w_drv_test_cw(const struct device *dev, uint32_t frequency_hz, int8_t dbm,
			      uint32_t duration_ms)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(frequency_hz);
	ARG_UNUSED(dbm);
	ARG_UNUSED(duration_ms);
	return -ENOTSUP; /* phase 4 */
}

/* Plain static const API struct (not DEVICE_API): this is an out-of-tree API class. */
static const struct rfm26w_driver_api rfm26w_api = {
	.configure = rfm26w_drv_configure,
	.get_info = rfm26w_drv_get_info,
	.send = rfm26w_drv_send,
	.recv = rfm26w_drv_recv,
	.recv_async = rfm26w_drv_recv_async,
	.set_tx_power = rfm26w_drv_set_tx_power,
	.read_rssi = rfm26w_drv_read_rssi,
	.get_stats = rfm26w_drv_get_stats,
	.shutdown = rfm26w_drv_shutdown,
	.start = rfm26w_drv_start,
	.test_cw = rfm26w_drv_test_cw,
};

#ifdef CONFIG_PM_DEVICE
/* Phase 4: SUSPEND shuts the radio down (SDN high), RESUME starts it again. */
static int rfm26w_pm_action(const struct device *dev, enum pm_device_action action)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(action);
	return -ENOTSUP;
}
#endif

/* The radio is left in shutdown (SDN high) until rfm26w_start() is called. */
static int rfm26w_init(const struct device *dev)
{
	const struct rfm26w_cfg *cfg = dev->config;
	struct rfm26w_data *data = dev->data;
	int ret;

	if (!spi_is_ready_dt(&cfg->bus)) {
		LOG_ERR("SPI bus not ready");
		return -ENODEV;
	}
	if (!gpio_is_ready_dt(&cfg->irq) || !gpio_is_ready_dt(&cfg->sdn)) {
		LOG_ERR("GPIO not ready");
		return -ENODEV;
	}

	data->dev = dev;
	data->profile = RFM26W_PROFILE_GFSK_9K6;
	data->started = false;
	k_mutex_init(&data->lock);
	k_mutex_init(&data->tx_lock);
	k_sem_init(&data->tx_done, 0, 1);
	k_sem_init(&data->rx_sem, 0, 1);
	k_work_init(&data->irq_work, rfm26w_irq_work);
	k_work_queue_init(&data->workq);
	k_work_queue_start(&data->workq, cfg->workq_stack, cfg->workq_stack_size,
			   CONFIG_RFM26W_WORKQ_PRIORITY, NULL);
	k_thread_name_set(&data->workq.thread, "rfm26w");

	ret = gpio_pin_configure_dt(&cfg->sdn, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		LOG_ERR("SDN configure failed: %d", ret);
		return ret;
	}

	/* Optional antenna switch pins. The datasheet drives them opposite to each other: in RX
	 * state RX-ANT low and TX-ANT high, in TX state the reverse. Start in the receive posture.
	 * GPIO_ACTIVE_HIGH is required in the devicetree (checked below). The driver switches them
	 * on state changes in rfm26w_set_antenna(). The polarity must be confirmed on the board
	 * (PLANNED.md open item 17). */
	if (gpio_is_ready_dt(&cfg->tx_ant) && gpio_is_ready_dt(&cfg->rx_ant)) {
		if (((cfg->tx_ant.dt_flags | cfg->rx_ant.dt_flags) & GPIO_ACTIVE_LOW) != 0) {
			LOG_ERR("tx-ant-gpios and rx-ant-gpios must be GPIO_ACTIVE_HIGH");
			return -EINVAL;
		}
		ret = gpio_pin_configure_dt(&cfg->tx_ant, GPIO_OUTPUT_ACTIVE);
		if (ret < 0) {
			LOG_ERR("TX-ANT configure failed: %d", ret);
			return ret;
		}
		ret = gpio_pin_configure_dt(&cfg->rx_ant, GPIO_OUTPUT_INACTIVE);
		if (ret < 0) {
			LOG_ERR("RX-ANT configure failed: %d", ret);
			return ret;
		}
	}

	ret = gpio_pin_configure_dt(&cfg->irq, GPIO_INPUT);
	if (ret < 0) {
		LOG_ERR("nIRQ configure failed: %d", ret);
		return ret;
	}
	gpio_init_callback(&data->irq_cb, rfm26w_irq_isr, BIT(cfg->irq.pin));
	ret = gpio_add_callback(cfg->irq.port, &data->irq_cb);
	if (ret < 0) {
		LOG_ERR("nIRQ callback failed: %d", ret);
		return ret;
	}
	/* The interrupt stays disabled until rfm26w_start() has configured the radio. */
	ret = gpio_pin_interrupt_configure_dt(&cfg->irq, GPIO_INT_DISABLE);
	if (ret < 0) {
		return ret;
	}

	LOG_INF("RFM26W ready, radio held in shutdown until rfm26w_start()");
	return 0;
}

#define RFM26W_INIT(inst)                                                                          \
	BUILD_ASSERT(DT_INST_PROP(inst, block_length) > 0 &&                                       \
			     DT_INST_PROP(inst, block_length) <= UINT16_MAX,                       \
		     "block-length must be 1 to 65535 (TX_LEN and RX_LEN are 16 bits)");           \
	BUILD_ASSERT((DT_INST_GPIO_FLAGS(inst, int_gpios) & GPIO_ACTIVE_LOW) != 0,                  \
		     "int-gpios (nIRQ) must be GPIO_ACTIVE_LOW");                                  \
	BUILD_ASSERT((DT_INST_GPIO_FLAGS(inst, sdn_gpios) & GPIO_ACTIVE_LOW) == 0,                  \
		     "sdn-gpios must be GPIO_ACTIVE_HIGH");                                        \
	K_THREAD_STACK_DEFINE(rfm26w_workq_stack_##inst, CONFIG_RFM26W_WORKQ_STACK_SIZE);          \
	static uint8_t rfm26w_rx_buf_##inst[DT_INST_PROP(inst, block_length)];                     \
	static struct rfm26w_data rfm26w_data_##inst = {.rx_buf = rfm26w_rx_buf_##inst};           \
	static const struct rfm26w_cfg rfm26w_cfg_##inst = {                                       \
		.workq_stack = rfm26w_workq_stack_##inst,                                          \
		.workq_stack_size = K_THREAD_STACK_SIZEOF(rfm26w_workq_stack_##inst),              \
		.bus = SPI_DT_SPEC_INST_GET(inst, SPI_WORD_SET(8)),                                \
		.irq = GPIO_DT_SPEC_INST_GET(inst, int_gpios),                                     \
		.sdn = GPIO_DT_SPEC_INST_GET(inst, sdn_gpios),                                     \
		.tx_ant = GPIO_DT_SPEC_INST_GET_OR(inst, tx_ant_gpios, {0}),                       \
		.rx_ant = GPIO_DT_SPEC_INST_GET_OR(inst, rx_ant_gpios, {0}),                       \
		.xo_hz = DT_INST_PROP(inst, xo_frequency_hz),                                      \
		.xo_tcxo = DT_INST_PROP(inst, xo_tcxo),                                            \
		.block_len = DT_INST_PROP(inst, block_length),                                     \
	};                                                                                         \
	PM_DEVICE_DT_INST_DEFINE(inst, rfm26w_pm_action);                                          \
	DEVICE_DT_INST_DEFINE(inst, rfm26w_init, PM_DEVICE_DT_INST_GET(inst), &rfm26w_data_##inst, \
			      &rfm26w_cfg_##inst, POST_KERNEL, CONFIG_RFM26W_INIT_PRIORITY,        \
			      &rfm26w_api);

DT_INST_FOREACH_STATUS_OKAY(RFM26W_INIT)
