/**
 * @file orion_b16_protocol.h
 * @brief SkyTraq Binary Protocol constants and frame definitions for the Orion B16 GNSS.
 *
 * Frame format (per vendor manual):
 *   [0xA0][0xA1][PL_hi][PL_lo][MessageID][Body...][CS][0x0D][0x0A]
 *
 *   - PL  = payload length = sizeof(MessageID) + sizeof(Body)
 *   - CS  = XOR of every byte in payload (MessageID + Body)
 *   - All multi-byte fields are BIG-ENDIAN
 *
 * This header is pure C, no external dependencies.
 */

#ifndef ORION_B16_PROTOCOL_H
#define ORION_B16_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ──────────────────────── Frame Delimiters ──────────────────────── */

#define ORION_SOF0  0xA0u   /**< Start-of-frame byte 0 */
#define ORION_SOF1  0xA1u   /**< Start-of-frame byte 1 */
#define ORION_EOF0  0x0Du   /**< End-of-frame byte 0   */
#define ORION_EOF1  0x0Au   /**< End-of-frame byte 1   */

/** Maximum payload we'll accept (MessageID + Body).  Keeps buffers bounded. */
#define ORION_MAX_PAYLOAD   512u

/** Overhead bytes around the payload: SOF(2) + LEN(2) + CS(1) + EOF(2) = 7 */
#define ORION_FRAME_OVERHEAD 7u

/** Maximum full-frame size */
#define ORION_MAX_FRAME_SIZE (ORION_MAX_PAYLOAD + ORION_FRAME_OVERHEAD)

/* ──────────────── Host → Receiver  Message IDs (Input) ─────────── */

#define ORION_CMD_RESTART                      0x01u
#define ORION_CMD_QUERY_SW_VERSION             0x02u
#define ORION_CMD_QUERY_SW_CRC                 0x03u
#define ORION_CMD_SET_FACTORY_DEFAULTS         0x04u
#define ORION_CMD_CFG_SERIAL_PORT              0x05u
#define ORION_CMD_CFG_NMEA                     0x08u
#define ORION_CMD_CFG_MSG_TYPE                 0x09u
#define ORION_CMD_CFG_POWER_MODE               0x0Cu
#define ORION_CMD_CFG_UPDATE_RATE              0x0Eu
#define ORION_CMD_QUERY_UPDATE_RATE            0x10u
#define ORION_CMD_CFG_DATUM                    0x29u
#define ORION_CMD_CFG_DOP_MASK                 0x2Au
#define ORION_CMD_CFG_ELEV_CNR_MASK            0x2Bu
#define ORION_CMD_QUERY_DATUM                  0x2Du
#define ORION_CMD_QUERY_DOP_MASK               0x2Eu
#define ORION_CMD_QUERY_ELEV_CNR_MASK          0x2Fu
#define ORION_CMD_QUERY_POWER_MODE             0x15u
#define ORION_CMD_QUERY_MSG_TYPE               0x16u
#define ORION_CMD_CFG_NAV_DATA_MSG_INTERVAL    0x11u
#define ORION_CMD_CFG_GNSS_CONSTELLATION       0x64u
#define ORION_CMD_QUERY_GNSS_CONSTELLATION     0x63u
#define ORION_CMD_QUERY_NMEA_TALKER_ID         0x4Fu
#define ORION_CMD_CFG_NMEA_TALKER_ID           0x50u
#define ORION_CMD_CFG_1PPS_MODE                0x65u
#define ORION_CMD_QUERY_1PPS_MODE              0x66u

/* ──────────── Receiver → Host  Message IDs (Output) ────────────── */

#define ORION_RSP_SW_VERSION                   0x80u
#define ORION_RSP_SW_CRC                       0x81u
#define ORION_RSP_ACK                          0x83u
#define ORION_RSP_NACK                         0x84u
#define ORION_RSP_UPDATE_RATE                  0x86u
#define ORION_RSP_MSG_TYPE                     0x8Cu
#define ORION_RSP_NMEA_TALKER_ID               0x93u
#define ORION_RSP_NAV_DATA                     0xA8u
#define ORION_RSP_GNSS_DATUM                   0xAEu
#define ORION_RSP_DOP_MASK                     0xAFu
#define ORION_RSP_ELEV_CNR_MASK                0xB0u
#define ORION_RSP_GPS_EPHEMERIS                0xB1u
#define ORION_RSP_GNSS_CONSTELLATION           0xB3u
#define ORION_RSP_POWER_MODE                   0xB9u
#define ORION_RSP_1PPS_MODE                    0xC4u

/* ──────────────── Save/Attribute options ────────────────────────── */

#define ORION_ATTR_SRAM_ONLY     0x00u  /**< Change in SRAM (lost on reboot) */
#define ORION_ATTR_SRAM_FLASH    0x01u  /**< Change in SRAM + save to flash  */
#define ORION_ATTR_TEMPORARY     0x02u  /**< Temporary (some commands)       */

/* ──────────────── Message Type constants ────────────────────────── */

#define ORION_MSG_TYPE_NMEA      0x01u
#define ORION_MSG_TYPE_BINARY    0x02u

/* ──────────────── Fix Mode constants ────────────────────────────── */

#define ORION_FIX_NONE           0x00u
#define ORION_FIX_2D             0x01u
#define ORION_FIX_3D             0x02u
#define ORION_FIX_3D_DGPS        0x03u

/* ──────────────── Power Mode constants ──────────────────────────── */

#define ORION_POWER_NORMAL       0x00u
#define ORION_POWER_SAVE         0x01u

/* ──────────────── Raw Frame Structure ───────────────────────────── */

/**
 * @brief Represents a single decoded SkyTraq binary frame.
 *
 * After successful parsing, msg_id holds the Message ID and body[]
 * holds the remaining payload bytes (body_len of them).
 */
typedef struct {
    uint16_t payload_len;                   /**< Full payload length (1 + body_len) */
    uint8_t  msg_id;                        /**< Message ID (first payload byte)    */
    uint8_t  body[ORION_MAX_PAYLOAD - 1];   /**< Body (payload without msg_id)      */
    uint16_t body_len;                      /**< Number of valid bytes in body[]    */
    uint8_t  checksum;                      /**< Received checksum byte             */
} orion_frame_t;


/* ──────────────── Utility Functions ─────────────────────────────── */

/**
 * @brief Compute XOR checksum over a payload buffer.
 * @param payload Pointer to the payload (MessageID + Body).
 * @param len     Number of bytes.
 * @return XOR checksum.
 */
static inline uint8_t orion_xor_checksum(const uint8_t *payload, size_t len)
{
    uint8_t cs = 0;
    for (size_t i = 0; i < len; i++) {
        cs ^= payload[i];
    }
    return cs;
}

/**
 * @brief Read a big-endian uint16 from a byte buffer.
 */
static inline uint16_t orion_read_be16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] << 8) | (uint16_t)p[1];
}

/**
 * @brief Read a big-endian int16 from a byte buffer.
 */
static inline int16_t orion_read_be16s(const uint8_t *p)
{
    uint16_t u = orion_read_be16(p);
    return (int16_t)u;
}

/**
 * @brief Read a big-endian uint32 from a byte buffer.
 */
static inline uint32_t orion_read_be32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) |
           ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] <<  8) |
           ((uint32_t)p[3]);
}

/**
 * @brief Read a big-endian int32 from a byte buffer.
 */
static inline int32_t orion_read_be32s(const uint8_t *p)
{
    uint32_t u = orion_read_be32(p);
    return (int32_t)u;
}

/**
 * @brief Write a big-endian uint16 into a byte buffer.
 */
static inline void orion_write_be16(uint8_t *p, uint16_t val)
{
    p[0] = (uint8_t)(val >> 8);
    p[1] = (uint8_t)(val & 0xFF);
}

/**
 * @brief Write a big-endian uint32 into a byte buffer.
 */
static inline void orion_write_be32(uint8_t *p, uint32_t val)
{
    p[0] = (uint8_t)((val >> 24) & 0xFF);
    p[1] = (uint8_t)((val >> 16) & 0xFF);
    p[2] = (uint8_t)((val >>  8) & 0xFF);
    p[3] = (uint8_t)((val)       & 0xFF);
}

#ifdef __cplusplus
}
#endif

#endif /* ORION_B16_PROTOCOL_H */
