/**
 * @file orion_b16_parser.c
 * @brief State-machine parser and message decoder for the SkyTraq Binary Protocol.
 */

#include "orion_b16_parser.h"
#include <string.h>

/* ──────────────── Parser Init ───────────────────────────────────── */

void orion_parser_init(orion_parser_t *p)
{
    memset(p, 0, sizeof(*p));
    p->state = ORION_PS_SEEK_SOF0;
}

/* ──────────────── Parser Feed Byte ──────────────────────────────── */

orion_parse_result_t orion_parser_feed_byte(orion_parser_t *p,
                                            uint8_t byte,
                                            orion_frame_t *out)
{
    switch (p->state) {

    /* ── Synchronization ─────────────────────────────────────────── */

    case ORION_PS_SEEK_SOF0:
        if (byte == ORION_SOF0) {
            p->state = ORION_PS_SEEK_SOF1;
        }
        return ORION_PARSE_NEED_MORE;

    case ORION_PS_SEEK_SOF1:
        if (byte == ORION_SOF1) {
            p->state = ORION_PS_LEN_HI;
        } else {
            /* False SOF0, resync */
            p->state = (byte == ORION_SOF0) ? ORION_PS_SEEK_SOF1
                                             : ORION_PS_SEEK_SOF0;
        }
        return ORION_PARSE_NEED_MORE;

    /* ── Payload Length (big-endian 16-bit) ───────────────────────── */

    case ORION_PS_LEN_HI:
        p->payload_len = (uint16_t)byte << 8;
        p->state = ORION_PS_LEN_LO;
        return ORION_PARSE_NEED_MORE;

    case ORION_PS_LEN_LO:
        p->payload_len |= byte;
        p->payload_idx = 0;

        if (p->payload_len == 0 || p->payload_len > ORION_MAX_PAYLOAD) {
            /* Invalid length — resync */
            p->state = ORION_PS_SEEK_SOF0;
            p->frames_err++;
            return ORION_PARSE_ERROR;
        }
        p->state = ORION_PS_PAYLOAD;
        return ORION_PARSE_NEED_MORE;

    /* ── Payload Accumulation ────────────────────────────────────── */

    case ORION_PS_PAYLOAD:
        p->payload_buf[p->payload_idx++] = byte;
        if (p->payload_idx >= p->payload_len) {
            p->state = ORION_PS_CHECKSUM;
        }
        return ORION_PARSE_NEED_MORE;

    /* ── Checksum ────────────────────────────────────────────────── */

    case ORION_PS_CHECKSUM:
        p->checksum_rx = byte;
        p->state = ORION_PS_EOF0;
        return ORION_PARSE_NEED_MORE;

    /* ── End-of-Frame ────────────────────────────────────────────── */

    case ORION_PS_EOF0:
        if (byte != ORION_EOF0) {
            p->state = ORION_PS_SEEK_SOF0;
            p->frames_err++;
            return ORION_PARSE_ERROR;
        }
        p->state = ORION_PS_EOF1;
        return ORION_PARSE_NEED_MORE;

    case ORION_PS_EOF1:
        if (byte != ORION_EOF1) {
            p->state = ORION_PS_SEEK_SOF0;
            p->frames_err++;
            return ORION_PARSE_ERROR;
        }

        /* Validate checksum */
        {
            uint8_t cs_calc = orion_xor_checksum(p->payload_buf, p->payload_len);
            if (cs_calc != p->checksum_rx) {
                p->state = ORION_PS_SEEK_SOF0;
                p->frames_err++;
                return ORION_PARSE_ERROR;
            }
        }

        /* Build output frame */
        if (out) {
            out->payload_len = p->payload_len;
            out->msg_id      = p->payload_buf[0];
            out->body_len    = p->payload_len - 1;
            out->checksum    = p->checksum_rx;
            if (out->body_len > 0) {
                memcpy(out->body, &p->payload_buf[1], out->body_len);
            }
        }

        p->frames_ok++;
        p->state = ORION_PS_SEEK_SOF0;
        return ORION_PARSE_FRAME_READY;

    default:
        p->state = ORION_PS_SEEK_SOF0;
        return ORION_PARSE_ERROR;
    }
}


/* ──────────────── Navigation Data Decoder ────────────────────────── */

int orion_decode_nav_data(const uint8_t *body, uint16_t body_len,
                          orion_nav_data_t *out)
{
    memset(out, 0, sizeof(*out));

    if (body_len < ORION_NAV_DATA_BODY_LEN) {
        out->valid = false;
        return -1;
    }

    out->fix_mode         = body[0];
    out->sv_count         = body[1];
    out->gnss_week        = orion_read_be16(&body[2]);
    out->tow_cs           = orion_read_be32(&body[4]);
    out->latitude_1e7     = orion_read_be32s(&body[8]);
    out->longitude_1e7    = orion_read_be32s(&body[12]);
    out->ellipsoid_alt_cm = orion_read_be32(&body[16]);
    out->msl_alt_cm       = orion_read_be32(&body[20]);
    out->gdop             = orion_read_be16(&body[24]);
    out->pdop             = orion_read_be16(&body[26]);
    out->hdop             = orion_read_be16(&body[28]);
    out->vdop             = orion_read_be16(&body[30]);
    out->tdop             = orion_read_be16(&body[32]);
    out->ecef_x_cm        = orion_read_be32s(&body[34]);
    out->ecef_y_cm        = orion_read_be32s(&body[38]);
    out->ecef_z_cm        = orion_read_be32s(&body[42]);
    out->ecef_vx_cms      = orion_read_be32s(&body[46]);
    out->ecef_vy_cms      = orion_read_be32s(&body[50]);
    out->ecef_vz_cms      = orion_read_be32s(&body[54]);
    out->valid            = true;

    return 0;
}


/* ──────────────── Generic Message Decoder ────────────────────────── */

int orion_decode_message(const orion_frame_t *frame, orion_message_t *msg)
{
    memset(msg, 0, sizeof(*msg));
    msg->msg_id = frame->msg_id;

    switch (frame->msg_id) {

    /* ── Navigation Data (0xA8) — the big one ────────────────────── */
    case ORION_RSP_NAV_DATA:
        return orion_decode_nav_data(frame->body, frame->body_len,
                                     &msg->nav_data);

    /* ── Software Version (0x80) ─────────────────────────────────── */
    case ORION_RSP_SW_VERSION:
        if (frame->body_len < ORION_SW_VERSION_BODY_LEN) {
            msg->sw_version.valid = false;
            return -1;
        }
        msg->sw_version.sw_type = frame->body[0];
        memcpy(msg->sw_version.kernel_ver, &frame->body[1], 4);
        memcpy(msg->sw_version.odm_ver,    &frame->body[5], 4);
        memcpy(msg->sw_version.revision,   &frame->body[9], 4);
        msg->sw_version.valid = true;
        return 0;

    /* ── ACK (0x83) ──────────────────────────────────────────────── */
    case ORION_RSP_ACK:
        if (frame->body_len < 1) return -1;
        msg->ack.acked_msg_id = frame->body[0];
        msg->ack.is_ack = true;
        return 0;

    /* ── NACK (0x84) ─────────────────────────────────────────────── */
    case ORION_RSP_NACK:
        if (frame->body_len < 1) return -1;
        msg->ack.acked_msg_id = frame->body[0];
        msg->ack.is_ack = false;
        return 0;

    /* ── Position Update Rate (0x86) ─────────────────────────────── */
    case ORION_RSP_UPDATE_RATE:
        if (frame->body_len < 1) return -1;
        msg->update_rate.rate_hz = frame->body[0];
        msg->update_rate.valid = true;
        return 0;

    /* ── Message Type (0x8C) ─────────────────────────────────────── */
    case ORION_RSP_MSG_TYPE:
        if (frame->body_len < 1) return -1;
        msg->msg_type.msg_type = frame->body[0];
        msg->msg_type.valid = true;
        return 0;

    /* ── NMEA Talker ID (0x93) ───────────────────────────────────── */
    case ORION_RSP_NMEA_TALKER_ID:
        if (frame->body_len < 1) return -1;
        msg->talker_id.talker_mode = frame->body[0];
        msg->talker_id.valid = true;
        return 0;

    /* ── Power Mode (0xB9) ───────────────────────────────────────── */
    case ORION_RSP_POWER_MODE:
        if (frame->body_len < 1) return -1;
        msg->power_mode.power_mode = frame->body[0];
        msg->power_mode.valid = true;
        return 0;

    /* ── GNSS Constellation (0xB3) ───────────────────────────────── */
    case ORION_RSP_GNSS_CONSTELLATION:
        if (frame->body_len < 4) return -1;
        msg->constellation.gps_enabled     = frame->body[0];
        msg->constellation.glonass_enabled = frame->body[1];
        msg->constellation.galileo_enabled = frame->body[2];
        msg->constellation.beidou_enabled  = frame->body[3];
        msg->constellation.valid = true;
        return 0;

    /* ── DOP Mask (0xAF) ─────────────────────────────────────────── */
    case ORION_RSP_DOP_MASK:
        if (frame->body_len < 7) return -1;
        msg->dop_mask.dop_mode = frame->body[0];
        msg->dop_mask.pdop_val = orion_read_be16(&frame->body[1]);
        msg->dop_mask.hdop_val = orion_read_be16(&frame->body[3]);
        msg->dop_mask.gdop_val = orion_read_be16(&frame->body[5]);
        msg->dop_mask.valid = true;
        return 0;

    /* ── Elevation & CNR Mask (0xB0) ─────────────────────────────── */
    case ORION_RSP_ELEV_CNR_MASK:
        if (frame->body_len < 2) return -1;
        msg->elev_cnr_mask.elev_mask = frame->body[0];
        msg->elev_cnr_mask.cnr_mask  = frame->body[1];
        msg->elev_cnr_mask.valid = true;
        return 0;

    default:
        /* Unknown message ID — not decoded, but not an error */
        return -1;
    }
}
