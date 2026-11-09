/**
 * @file soh.h
  * @ingroup common
 * @brief Shared board State of Health (SOH) message format.
 *
 * Defines the SOH Core and SOH CAN Stats CAN payloads (8 bytes each)
 * and the functions to pack/unpack them. Used by every board and by
 * host-side tests. Must not depend on Zephyr.
 *
 * SOH Core payload:
 *   Bytes 0-3  uptime_s      (u32, seconds since boot)
 *   Bytes 4-5  vdd_mv        (u16, mV; SOH_VDD_INVALID = read failed)
 *   Byte  6    temp_c        (i8, deg C; SOH_TEMP_INVALID = read failed)
 *   Byte  7    version/reset (high nibble = schema version,
 *                             low nibble = soh_reset_cause_t)
 *
 * SOH CAN Stats payload:
 *   Byte  0    tec               (u8, current transmit error counter)
 *   Byte  1    rec               (u8, current receive error counter)
 *   Byte  2    can_flags         (u8, SOH_CANF_* bits)
 *   Byte  3    bus_off_count     (u8, saturating)
 *   Bytes 4-5  error_irq_count   (u16, saturating)
 *   Bytes 6-7  rx_overrun_count  (u16, saturating)
 *
 * Multi-byte fields use the byte order defined in soh.c.
 *
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Current SOH payload format version (packed into the high nibble of Core byte 7). */
#define SOH_SCHEMA_VERSION   1U

/** Length of each SOH CAN payload in bytes. */
#define SOH_FRAME_LEN        8U

/** How often each node sends SOH, in seconds. */
#define SOH_PERIOD_S         5U

/** A node is stale if no SOH Core arrives within this many seconds. */
#define SOH_STALE_S          15U

/** Sentinel: supply voltage read failed. */
#define SOH_VDD_INVALID      0xFFFFU

/** Sentinel: temperature read failed. */
#define SOH_TEMP_INVALID     INT8_MIN

/** can_flags bit: controller is in error-warning state. */
#define SOH_CANF_ERR_WARNING (1U << 0)

/** can_flags bit: controller is in error-passive state. */
#define SOH_CANF_ERR_PASSIVE (1U << 1)

/** Why the board last reset. Packed into the low nibble of Core byte 7. */
typedef enum {
    SOH_RESET_UNKNOWN       = 0,
    SOH_RESET_POWER_ON      = 1,
    SOH_RESET_BROWNOUT      = 2,
    SOH_RESET_WATCHDOG      = 3,
    SOH_RESET_SOFTWARE      = 4,
    SOH_RESET_EXTERNAL_PIN  = 5,
    SOH_RESET_LOCKUP_OTHER  = 6,
    SOH_RESET_COUNT         /**< Number of valid causes; not a cause itself. */
} soh_reset_cause_t;

/** Decoded SOH Core frame: is the board itself healthy? */
typedef struct {
    uint32_t          uptime_s;    /**< Seconds since boot. */
    uint16_t          vdd_mv;      /**< Supply voltage in mV, or SOH_VDD_INVALID. */
    int8_t            temp_c;      /**< MCU temperature in deg C, or SOH_TEMP_INVALID. */
    uint8_t           version;     /**< Schema version (set by unpack; pack always writes SOH_SCHEMA_VERSION). */
    soh_reset_cause_t reset_cause; /**< Cause of the most recent reset. */
} soh_core_t;

/** Decoded SOH CAN Stats frame: is the board's CAN link healthy? */
typedef struct {
    uint8_t  tec;              /**< Current transmit error counter. */
    uint8_t  rec;              /**< Current receive error counter. */
    uint8_t  can_flags;        /**< SOH_CANF_* bits. */
    uint8_t  bus_off_count;    /**< Bus-off events since boot (saturating). */
    uint16_t error_irq_count;  /**< CAN error events since boot (saturating). */
    uint16_t rx_overrun_count; /**< Frames dropped since boot (saturating). */
} soh_can_t;

/**
 * @brief Pack an SOH Core struct into an 8-byte CAN payload.
 *
 * The version nibble is always written as SOH_SCHEMA_VERSION;
 * in->version is ignored.
 *
 * @param in  Values to pack. Must not be NULL.
 * @param out Destination buffer of SOH_FRAME_LEN bytes. Must not be NULL.
 */
void soh_pack_core(const soh_core_t *in, uint8_t out[SOH_FRAME_LEN]);

/**
 * @brief Unpack and validate an SOH Core CAN payload.
 *
 * Sentinel values (SOH_VDD_INVALID, SOH_TEMP_INVALID) are valid and
 * are returned as-is.
 *
 * @param in  Received payload bytes.
 * @param len Number of bytes in @p in.
 * @param out Destination struct.
 * @return true if valid; false if any pointer is NULL,
 *         len != SOH_FRAME_LEN, the version is not SOH_SCHEMA_VERSION,
 *         or the reset cause is >= SOH_RESET_COUNT.
 */
bool soh_unpack_core(const uint8_t *in, size_t len, soh_core_t *out);

/**
 * @brief Pack an SOH CAN Stats struct into an 8-byte CAN payload.
 *
 * @param in  Values to pack. Must not be NULL.
 * @param out Destination buffer of SOH_FRAME_LEN bytes. Must not be NULL.
 */
void soh_pack_can(const soh_can_t *in, uint8_t out[SOH_FRAME_LEN]);

/**
 * @brief Unpack and validate an SOH CAN Stats CAN payload.
 *
 * @param in  Received payload bytes.
 * @param len Number of bytes in @p in.
 * @param out Destination struct.
 * @return true if valid; false if any pointer is NULL or
 *         len != SOH_FRAME_LEN.
 */
bool soh_unpack_can(const uint8_t *in, size_t len, soh_can_t *out);

#ifdef __cplusplus
}
#endif