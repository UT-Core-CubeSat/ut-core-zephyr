/**
 * @file orion_b16_commands.c
 * @brief Command builder implementations for Host → Orion B16 messages.
 */

#include "orion_b16_commands.h"
#include <string.h>

/* ──────────────── Generic Frame Builder ─────────────────────────── */

int orion_cmd_build_raw(uint8_t *buf, size_t buf_size,
                        uint8_t msg_id,
                        const uint8_t *body, size_t body_len)
{
    /* payload = [msg_id] + [body...] */
    uint16_t pl_len = (uint16_t)(1 + body_len);
    size_t   frame_len = ORION_FRAME_OVERHEAD + pl_len;

    if (buf_size < frame_len) {
        return -1;  /* buffer too small */
    }

    size_t idx = 0;

    /* Start of Frame */
    buf[idx++] = ORION_SOF0;
    buf[idx++] = ORION_SOF1;

    /* Payload length (big-endian) */
    buf[idx++] = (uint8_t)(pl_len >> 8);
    buf[idx++] = (uint8_t)(pl_len & 0xFF);

    /* Payload: message ID */
    buf[idx++] = msg_id;

    /* Payload: body */
    if (body && body_len > 0) {
        memcpy(&buf[idx], body, body_len);
        idx += body_len;
    }

    /* Checksum: XOR of payload bytes (everything between PL and CS) */
    uint8_t cs = msg_id;
    for (size_t i = 0; i < body_len; i++) {
        cs ^= body[i];
    }
    buf[idx++] = cs;

    /* End of Frame */
    buf[idx++] = ORION_EOF0;
    buf[idx++] = ORION_EOF1;

    return (int)idx;
}


/* ──────────────── Convenience Builders ──────────────────────────── */

int orion_cmd_query_sw_version(uint8_t *buf, size_t buf_size, uint8_t sw_type)
{
    uint8_t body[] = { sw_type };
    return orion_cmd_build_raw(buf, buf_size, ORION_CMD_QUERY_SW_VERSION, body, sizeof(body));
}

int orion_cmd_cfg_msg_type(uint8_t *buf, size_t buf_size,
                           uint8_t type, uint8_t attr)
{
    uint8_t body[] = { type, attr };
    return orion_cmd_build_raw(buf, buf_size, ORION_CMD_CFG_MSG_TYPE, body, sizeof(body));
}

int orion_cmd_cfg_update_rate(uint8_t *buf, size_t buf_size,
                              uint8_t rate_hz, uint8_t attr)
{
    uint8_t body[] = { rate_hz, attr };
    return orion_cmd_build_raw(buf, buf_size, ORION_CMD_CFG_UPDATE_RATE, body, sizeof(body));
}

int orion_cmd_query_update_rate(uint8_t *buf, size_t buf_size)
{
    return orion_cmd_build_raw(buf, buf_size, ORION_CMD_QUERY_UPDATE_RATE, NULL, 0);
}

int orion_cmd_query_power_mode(uint8_t *buf, size_t buf_size)
{
    return orion_cmd_build_raw(buf, buf_size, ORION_CMD_QUERY_POWER_MODE, NULL, 0);
}

int orion_cmd_cfg_power_mode(uint8_t *buf, size_t buf_size,
                             uint8_t mode, uint8_t attr)
{
    uint8_t body[] = { mode, attr };
    return orion_cmd_build_raw(buf, buf_size, ORION_CMD_CFG_POWER_MODE, body, sizeof(body));
}

int orion_cmd_query_msg_type(uint8_t *buf, size_t buf_size)
{
    return orion_cmd_build_raw(buf, buf_size, ORION_CMD_QUERY_MSG_TYPE, NULL, 0);
}

int orion_cmd_query_nmea_talker_id(uint8_t *buf, size_t buf_size)
{
    return orion_cmd_build_raw(buf, buf_size, ORION_CMD_QUERY_NMEA_TALKER_ID, NULL, 0);
}

int orion_cmd_cfg_nmea_talker_id(uint8_t *buf, size_t buf_size,
                                 uint8_t mode, uint8_t attr)
{
    uint8_t body[] = { mode, attr };
    return orion_cmd_build_raw(buf, buf_size, ORION_CMD_CFG_NMEA_TALKER_ID, body, sizeof(body));
}

int orion_cmd_restart(uint8_t *buf, size_t buf_size,
                      uint8_t start_mode, uint8_t attr)
{
    /* Restart command: [start_mode]             *
     * 0x00=hot, 0x01=warm, 0x02=cold, 0x03=factory cold */
    uint8_t body[] = { start_mode, attr };
    return orion_cmd_build_raw(buf, buf_size, ORION_CMD_RESTART, body, sizeof(body));
}

int orion_cmd_cfg_nav_interval(uint8_t *buf, size_t buf_size,
                               uint8_t interval, uint8_t attr)
{
    uint8_t body[] = { interval, attr };
    return orion_cmd_build_raw(buf, buf_size, ORION_CMD_CFG_NAV_DATA_MSG_INTERVAL,
                               body, sizeof(body));
}

int orion_cmd_cfg_constellation(uint8_t *buf, size_t buf_size,
                                uint8_t gps, uint8_t glonass,
                                uint8_t galileo, uint8_t beidou,
                                uint8_t attr)
{
    uint8_t body[] = { gps, glonass, galileo, beidou, attr };
    return orion_cmd_build_raw(buf, buf_size, ORION_CMD_CFG_GNSS_CONSTELLATION,
                               body, sizeof(body));
}

int orion_cmd_query_constellation(uint8_t *buf, size_t buf_size)
{
    return orion_cmd_build_raw(buf, buf_size, ORION_CMD_QUERY_GNSS_CONSTELLATION,
                               NULL, 0);
}

int orion_cmd_cfg_dop_mask(uint8_t *buf, size_t buf_size,
                           uint8_t mode, uint16_t pdop, uint16_t hdop,
                           uint16_t gdop, uint8_t attr)
{
    uint8_t body[8];
    body[0] = mode;
    orion_write_be16(&body[1], pdop);
    orion_write_be16(&body[3], hdop);
    orion_write_be16(&body[5], gdop);
    body[7] = attr;
    return orion_cmd_build_raw(buf, buf_size, ORION_CMD_CFG_DOP_MASK,
                               body, sizeof(body));
}

int orion_cmd_query_dop_mask(uint8_t *buf, size_t buf_size)
{
    return orion_cmd_build_raw(buf, buf_size, ORION_CMD_QUERY_DOP_MASK, NULL, 0);
}

int orion_cmd_cfg_elev_cnr_mask(uint8_t *buf, size_t buf_size,
                                uint8_t elev_deg, uint8_t cnr_dbhz,
                                uint8_t attr)
{
    uint8_t body[] = { elev_deg, cnr_dbhz, attr };
    return orion_cmd_build_raw(buf, buf_size, ORION_CMD_CFG_ELEV_CNR_MASK,
                               body, sizeof(body));
}

int orion_cmd_query_elev_cnr_mask(uint8_t *buf, size_t buf_size)
{
    return orion_cmd_build_raw(buf, buf_size, ORION_CMD_QUERY_ELEV_CNR_MASK, NULL, 0);
}

int orion_cmd_set_factory_defaults(uint8_t *buf, size_t buf_size, uint8_t type)
{
    uint8_t body[] = { type };
    return orion_cmd_build_raw(buf, buf_size, ORION_CMD_SET_FACTORY_DEFAULTS,
                               body, sizeof(body));
}
