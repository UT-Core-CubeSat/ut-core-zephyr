/**
 * @file rfm26w_priv.h
 * @brief RFM26W driver internals that need no Zephyr: command codes, property addresses, the profile
 *        table walker and the carrier frequency calculation.
 * @ingroup comms
 *
 * Kept free of Zephyr headers so the pure helpers can be compiled and checked on the host.
 * Command codes and reply layouts are from AN625. The frequency equation and the band codes are NOT
 * in the documents kept in external_documents/ (AN625 defers to the Si446x datasheet), so they are
 * marked "verify" below.
 */
#ifndef RFM26W_PRIV_H_
#define RFM26W_PRIV_H_

#include <errno.h>
#include <stddef.h>
#include <stdint.h>

/* ---- Commands (AN625 section 3, PLANNED.md section 5.3) ---- */
#define RFM26W_CMD_PART_INFO            0x01
#define RFM26W_CMD_POWER_UP             0x02
#define RFM26W_CMD_SET_PROPERTY         0x11
#define RFM26W_CMD_GET_PROPERTY         0x12
#define RFM26W_CMD_GPIO_PIN_CFG         0x13
#define RFM26W_CMD_FIFO_INFO            0x15
#define RFM26W_CMD_GET_INT_STATUS       0x20
#define RFM26W_CMD_GET_MODEM_STATUS     0x22
#define RFM26W_CMD_START_TX             0x31
#define RFM26W_CMD_START_RX             0x32
#define RFM26W_CMD_REQUEST_DEVICE_STATE 0x33
#define RFM26W_CMD_CHANGE_STATE         0x34
#define RFM26W_CMD_READ_CMD_BUFF        0x44 /**< CTS poll; also returns the reply bytes */
#define RFM26W_CMD_WRITE_TX_FIFO        0x66 /**< does not use CTS */
#define RFM26W_CMD_READ_RX_FIFO         0x77 /**< does not use CTS */

/** CTS value that means "ready, the reply (if any) follows". */
#define RFM26W_CTS_READY 0xFF

/** Radio states used by START_RX / START_TX next-state fields and CHANGE_STATE. */
enum rfm26w_state {
	RFM26W_STATE_NOCHANGE = 0,
	RFM26W_STATE_SLEEP = 1,
	RFM26W_STATE_SPI_ACTIVE = 2,
	RFM26W_STATE_READY = 3,
	RFM26W_STATE_READY2 = 4,
	RFM26W_STATE_TX_TUNE = 5,
	RFM26W_STATE_RX_TUNE = 6,
	RFM26W_STATE_TX = 7,
	RFM26W_STATE_RX = 8,
};

/* ---- Sizes ---- */
/** A command is at most 16 bytes including the command byte (SET_PROPERTY with 12 properties). */
#define RFM26W_CMD_MAX_LEN   16
#define RFM26W_CMD_MAX_ARGS  (RFM26W_CMD_MAX_LEN - 1)
/** READ_CMD_BUFF returns at most 16 bytes after CTS. */
#define RFM26W_REPLY_MAX     16
/** One SET_PROPERTY / GET_PROPERTY call covers at most 12 properties. */
#define RFM26W_PROP_MAX      12

/* POWER_UP arguments (AN625 3.1.1) */
#define RFM26W_BOOT_MAIN_IMAGE 0x01 /**< BOOT_OPTIONS: FUNC = 1, no patch */
#define RFM26W_XTAL_TCXO       0x01 /**< XTAL_OPTIONS bit 0 */

/* ---- Property addresses (group, start index) ---- */
#define RFM26W_PROP_GROUP_FREQ_CONTROL 0x40 /**< INTE, FRAC[2..0], step size ... start 0x00 */
#define RFM26W_PROP_GROUP_MODEM        0x20
#define RFM26W_PROP_MODEM_CLKGEN_BAND  0x51
#define RFM26W_PROP_GROUP_PA           0x22
#define RFM26W_PROP_PA_PWR_LVL         0x01

/* Phase 3: packet handler thresholds, interrupt enables, RSSI compensation */
#define RFM26W_PROP_GROUP_PKT          0x12
#define RFM26W_PROP_PKT_TX_THRESHOLD   0x0B /**< followed by PKT_RX_THRESHOLD at 0x0C */
#define RFM26W_PROP_GROUP_INT_CTL      0x01
#define RFM26W_PROP_INT_CTL_ENABLE     0x00 /**< then PH, MODEM, CHIP enables at 0x01, 0x02, 0x03 */
#define RFM26W_PROP_MODEM_RSSI_COMP    0x4E /**< in group 0x20, default 0x32 */

/* INT_CTL_ENABLE (0x0100): top-level groups */
#define RFM26W_INT_EN_PH    0x01
#define RFM26W_INT_EN_MODEM 0x02
#define RFM26W_INT_EN_CHIP  0x04
/* PH enable / pending bits (INT_CTL_PH_ENABLE 0x0101, GET_INT_STATUS PH_PEND) */
#define RFM26W_PH_RX_FIFO_ALMOST_FULL  0x01
#define RFM26W_PH_TX_FIFO_ALMOST_EMPTY 0x02
#define RFM26W_PH_PACKET_RX            0x10
#define RFM26W_PH_PACKET_SENT          0x20
/* MODEM enable / pending bits (INT_CTL_MODEM_ENABLE 0x0102, MODEM_PEND) */
#define RFM26W_MODEM_SYNC_DETECT       0x01
/* CHIP enable / pending bits (INT_CTL_CHIP_ENABLE 0x0103, CHIP_PEND) */
#define RFM26W_CHIP_FIFO_ERROR         0x20 /**< FIFO underflow or overflow */

/* GET_INT_STATUS reply byte offsets (after CTS) */
#define RFM26W_INT_REPLY_PH_PEND    2
#define RFM26W_INT_REPLY_MODEM_PEND 4
#define RFM26W_INT_REPLY_CHIP_PEND  6

/* FIFO_INFO reset bits (AN625 3.2.8) */
#define RFM26W_FIFO_RESET_TX 0x01
#define RFM26W_FIFO_RESET_RX 0x02

/** The radio's FIFOs are 64 bytes each (GLOBAL_CONFIG.FIFO_MODE = 0). */
#define RFM26W_FIFO_SIZE 64

/**
 * @brief Convert a raw RSSI reading to tenths of a dBm.
 *
 * VERIFY: the formula `dBm = raw / 2 - MODEM_RSSI_COMP - 70` is from Silicon Labs' Si446x
 * material and is NOT in AN625 or the RFM26W datasheet kept in external_documents/
 * (PLANNED.md open item 23). `comp` is the MODEM_RSSI_COMP value in the radio.
 */
static inline int16_t rfm26w_rssi_to_dbm10(uint8_t raw, uint8_t comp)
{
	return (int16_t)((int)raw * 5 - ((int)comp + 70) * 10);
}

/** MODEM_CLKGEN_BAND bit 3 (SY_SEL) is set in all WDS output seen so far (value 0x0A at 433 MHz). */
#define RFM26W_CLKGEN_SY_SEL 0x08

/* ---- Profile table walker ----
 *
 * A profile table is the WDS "RADIO_CONFIGURATION_DATA_ARRAY": entries of
 *   len, cmd, args...        (len counts the command byte and the arguments)
 * closed by a zero length byte.
 */
struct rfm26w_tbl_entry {
	uint8_t cmd;
	const uint8_t *args;
	uint8_t nargs;
};

/**
 * @brief Fetch the next entry of a profile table.
 * @param tbl table start
 * @param tbl_len table size in bytes
 * @param pos in/out offset of the next entry, start at 0
 * @param entry filled on a return of 1
 * @retval 1 an entry was returned
 * @retval 0 end of table (terminator, or the end of the buffer)
 * @retval -EINVAL malformed entry (too long, or runs past the end of the buffer)
 */
static inline int rfm26w_tbl_next(const uint8_t *tbl, size_t tbl_len, size_t *pos,
				  struct rfm26w_tbl_entry *entry)
{
	size_t p = *pos;
	uint8_t len;

	if (p >= tbl_len) {
		return 0;
	}
	len = tbl[p];
	if (len == 0) {
		return 0;
	}
	if (len > RFM26W_CMD_MAX_LEN || (p + 1u + len) > tbl_len) {
		return -EINVAL;
	}
	entry->cmd = tbl[p + 1];
	entry->args = &tbl[p + 2];
	entry->nargs = (uint8_t)(len - 1u);
	*pos = p + 1u + len;
	return 1;
}

/* ---- Carrier frequency ----
 *
 * VERIFY against the Si446x datasheet: this equation and the band table are not in AN625 or the
 * RFM26W datasheet kept in external_documents/. Cross-checked against one WDS header (433.92 MHz,
 * 30 MHz reference, divider 8): INTE = 0x38, FRAC = 0x0ED916, MODEM_CLKGEN_BAND = 0x0A.
 *
 *   ratio = F_rf * outdiv / (2 * F_xo)
 *   INTE  = floor(ratio) - 1
 *   FRAC  = floor((ratio - INTE) * 2^19)      (20 bits, 2^19 .. 2^20 - 1)
 *
 * The reference clock F_xo is the devicetree value. The result is only right if the chip was
 * powered up with that same value.
 */
struct rfm26w_band {
	uint32_t lo_hz;
	uint32_t hi_hz;
	uint8_t outdiv;
	uint8_t code; /**< MODEM_CLKGEN_BAND BAND[2:0] */
};

/** The RFM26W's four bands (datasheet section 3). Dividers and codes: verify. */
static const struct rfm26w_band rfm26w_bands[] = {
	{142000000u, 175000000u, 24, 5},
	{283000000u, 350000000u, 12, 3},
	{425000000u, 525000000u, 8, 2},
	{850000000u, 1050000000u, 4, 0},
};

/**
 * @brief Work out the synthesizer settings for a carrier frequency.
 * @param freq_hz carrier frequency
 * @param xo_hz reference clock frequency (must match what POWER_UP was given)
 * @param inte out: FREQ_CONTROL_INTE (7 bits)
 * @param frac out: FREQ_CONTROL_FRAC (20 bits)
 * @param clkgen_band out: MODEM_CLKGEN_BAND value
 * @retval 0 success
 * @retval -EINVAL xo_hz is zero, or the frequency is outside all four bands
 * @retval -ERANGE the integer part does not fit the property
 */
static inline int rfm26w_freq_calc(uint32_t freq_hz, uint32_t xo_hz, uint8_t *inte,
				   uint32_t *frac, uint8_t *clkgen_band)
{
	const struct rfm26w_band *band = NULL;
	uint64_t val;
	uint32_t i_part;

	if (xo_hz == 0u) {
		return -EINVAL;
	}
	for (size_t i = 0; i < sizeof(rfm26w_bands) / sizeof(rfm26w_bands[0]); i++) {
		if (freq_hz >= rfm26w_bands[i].lo_hz && freq_hz <= rfm26w_bands[i].hi_hz) {
			band = &rfm26w_bands[i];
			break;
		}
	}
	if (band == NULL) {
		return -EINVAL;
	}

	/* ratio * 2^19 = F * outdiv * 2^19 / (2 * xo) = (F * outdiv << 18) / xo, truncated like WDS */
	val = (((uint64_t)freq_hz * band->outdiv) << 18) / xo_hz;
	i_part = (uint32_t)(val >> 19);
	if (i_part < 1u || (i_part - 1u) > 0x7Fu) {
		return -ERANGE;
	}
	*inte = (uint8_t)(i_part - 1u);
	*frac = (uint32_t)(val - ((uint64_t)(i_part - 1u) << 19));
	*clkgen_band = (uint8_t)(RFM26W_CLKGEN_SY_SEL | band->code);
	return 0;
}

#endif /* RFM26W_PRIV_H_ */
