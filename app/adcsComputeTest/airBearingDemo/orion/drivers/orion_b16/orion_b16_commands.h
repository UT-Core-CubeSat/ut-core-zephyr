/**
 * @file orion_b16_commands.h
 * @brief Command builders for Host → Orion B16 messages.
 *
 * Each `orion_cmd_build_*()` function serializes a complete SkyTraq binary
 * frame (SOF, length, payload, checksum, EOF) into the caller's buffer and
 * returns the number of bytes written. The caller then sends these bytes
 * over UART.
 *
 * All functions are zero-allocation — they write into the provided buffer.
 */

#ifndef ORION_B16_COMMANDS_H
#define ORION_B16_COMMANDS_H

#include "orion_b16_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Build a generic SkyTraq command frame from a message ID and body.
 *
 * @param buf      Output buffer (must be >= 7 + 1 + body_len bytes).
 * @param buf_size Size of output buffer.
 * @param msg_id   Message ID byte.
 * @param body     Body bytes (may be NULL if body_len == 0).
 * @param body_len Number of body bytes.
 *
 * @return Number of bytes written into buf, or -1 on overflow.
 */
int orion_cmd_build_raw(uint8_t *buf, size_t buf_size,
                        uint8_t msg_id,
                        const uint8_t *body, size_t body_len);


/* ──────────── Convenience Builders ──────────────────────────────── */

/** Query Software Version (0x02). sw_type: 0x00 = system code. */
int orion_cmd_query_sw_version(uint8_t *buf, size_t buf_size, uint8_t sw_type);

/** Configure Message Type (0x09). type: NMEA=0x01, Binary=0x02. */
int orion_cmd_cfg_msg_type(uint8_t *buf, size_t buf_size,
                           uint8_t type, uint8_t attr);

/** Configure Position Update Rate (0x0E). rate_hz: 1,2,4,5,8,10,20,25,40,50 */
int orion_cmd_cfg_update_rate(uint8_t *buf, size_t buf_size,
                              uint8_t rate_hz, uint8_t attr);

/** Query Position Update Rate (0x10). */
int orion_cmd_query_update_rate(uint8_t *buf, size_t buf_size);

/** Query Power Mode (0x15). */
int orion_cmd_query_power_mode(uint8_t *buf, size_t buf_size);

/** Configure Power Mode (0x0C). mode: 0x00=Normal, 0x01=PowerSave. */
int orion_cmd_cfg_power_mode(uint8_t *buf, size_t buf_size,
                             uint8_t mode, uint8_t attr);

/** Query Message Type (0x16). */
int orion_cmd_query_msg_type(uint8_t *buf, size_t buf_size);

/** Query NMEA Talker ID (0x4F). */
int orion_cmd_query_nmea_talker_id(uint8_t *buf, size_t buf_size);

/** Configure NMEA Talker ID (0x50). mode: 0x00=GP, 0x01=GN, 0x02=Auto. */
int orion_cmd_cfg_nmea_talker_id(uint8_t *buf, size_t buf_size,
                                 uint8_t mode, uint8_t attr);

/** System Restart (0x01). start_mode: 0x00=hot, 0x01=warm, 0x02=cold, 0x03=factory cold. */
int orion_cmd_restart(uint8_t *buf, size_t buf_size,
                      uint8_t start_mode, uint8_t attr);

/** Configure Navigation Data Message Interval (0x11). interval: output interval in seconds. */
int orion_cmd_cfg_nav_interval(uint8_t *buf, size_t buf_size,
                               uint8_t interval, uint8_t attr);

/** Configure GNSS Constellation (0x64). Each flag: 0=disable, 1=enable. */
int orion_cmd_cfg_constellation(uint8_t *buf, size_t buf_size,
                                uint8_t gps, uint8_t glonass,
                                uint8_t galileo, uint8_t beidou,
                                uint8_t attr);

/** Query GNSS Constellation (0x63). */
int orion_cmd_query_constellation(uint8_t *buf, size_t buf_size);

/** Configure DOP Mask (0x2A). */
int orion_cmd_cfg_dop_mask(uint8_t *buf, size_t buf_size,
                           uint8_t mode, uint16_t pdop, uint16_t hdop,
                           uint16_t gdop, uint8_t attr);

/** Query DOP Mask (0x2E). */
int orion_cmd_query_dop_mask(uint8_t *buf, size_t buf_size);

/** Configure Elevation & CNR Mask (0x2B). */
int orion_cmd_cfg_elev_cnr_mask(uint8_t *buf, size_t buf_size,
                                uint8_t elev_deg, uint8_t cnr_dbhz,
                                uint8_t attr);

/** Query Elevation & CNR Mask (0x2F). */
int orion_cmd_query_elev_cnr_mask(uint8_t *buf, size_t buf_size);

/** Set Factory Defaults (0x04). */
int orion_cmd_set_factory_defaults(uint8_t *buf, size_t buf_size, uint8_t type);

#ifdef __cplusplus
}
#endif

#endif /* ORION_B16_COMMANDS_H */
