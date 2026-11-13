#pragma once

/**
 * @file ops.h
 * @ingroup common
 * @brief Central registry of every UT-CORE CAN opcode.
 *
 * The opcode is a 12-bit field in the 29-bit CAN ID (see can_proto.h), so
 * each value identifies exactly one message type. There is no separate
 * message class: the high nibble of the opcode is the *category*, kept only
 * so bus dumps stay readable:
 *
 * ```
 * 0x0xx  system / heartbeat
 * 0x1xx  command            (sender -> target)
 * 0x2xx  command response   (command opcode + 0x100)
 * 0x3xx  telemetry
 * 0x4xx  state of health
 * ```
 *
 * Payload byte layouts below use p0..p7 = CAN data[0]..data[7]; all 8 data
 * bytes are application payload (src and op live in the ID, not the data).
 *
 * Frame length: every opcode has a fixed payload length of 2, 4 or 8 bytes
 * (smallest that fits), given by can_op_len() at the bottom of this file.
 * Senders use can_send2/can_send4/can_send8; can_decode() rejects frames
 * whose DLC does not match, so handlers can trust data[0..len-1]. Unused
 * trailing bytes within the length are zero.
 *
 * Rules:
 *  - Add every new opcode here AND give it a length in can_op_len(). Never define opcodes locally in an app.
 *  - Values must be unique and <= OP_MAX (0xFFF).
 *  - A response to command OP_X is OP_RESP(OP_X).
 */

#include <stdint.h>

#define OP_MAX          0xFFFU
#define OP_RESP(cmd)    ((cmd) + 0x100U)

typedef enum {
    /* ---- 0x0xx system / heartbeat ---- */
    OP_HEARTBEAT = 0x001,   /**< Node alive. p0 = current mode (CDH) or 0. */
    OP_PING      = 0x002,   /**< Test ping. */
    OP_PONG      = 0x003,   /**< Test pong. */
    OP_BUTTON    = 0x004,   /**< Test button event. */

    /* ---- 0x1xx commands ---- */
    OP_SET_LED       = 0x101,   /**< p0 = led state. */
    OP_SET_MODE      = 0x102,   /**< p0 = mode. */
    OP_REBOOT        = 0x103,   /**< No payload. */
    OP_SET_PWR_STATE = 0x104,   /**< CDH -> EPS. p0, p1 = opcode-specific power state params. */
    OP_GET_POS       = 0x105,   /**< CDH -> GNSS: request position. No payload. */
    OP_SET_UPDATE_RATE = 0x106, /**< CDH -> GNSS: p0 = update rate. */
    /** Set wheel RPM reference: p0 = motor index (1..4, 0 = all), p1..p2 = int16 RPM (big-endian). */
    OP_SET_WHEEL_RPM  = 0x110,
    /** Magnetorquer dipole: p0..p1 = mx, p2..p3 = my, p4..p5 = mz (big-endian int16, 1e-4 A*m^2/LSB). */
    OP_SET_MAG_DIPOLE = 0x111,
    /** EPS load switch: p0 = load index (1..12, 0 = all), p1 = enable (0 = off, nonzero = on). */
    OP_SET_EPS_LOAD   = 0x112,

    /* ---- 0x2xx command responses (OP_RESP(command)) ---- */
    OP_RESP_SET_PWR_STATE   = OP_RESP(OP_SET_PWR_STATE),   /**< EPS -> CDH: p0..p2 = status. */
    OP_RESP_GET_POS         = OP_RESP(OP_GET_POS),         /**< GNSS -> CDH: p0 = 1 ack. */
    OP_RESP_SET_UPDATE_RATE = OP_RESP(OP_SET_UPDATE_RATE), /**< GNSS -> CDH: p0 = ack value. */
    OP_RESP_SET_WHEEL_RPM   = OP_RESP(OP_SET_WHEEL_RPM),
    OP_RESP_SET_MAG_DIPOLE  = OP_RESP(OP_SET_MAG_DIPOLE),  /**< SOLAR -> sender: p0 = duty percent. */
    OP_RESP_SET_EPS_LOAD    = OP_RESP(OP_SET_EPS_LOAD),

    /* ---- 0x3xx telemetry ---- */
    OP_GNSS_POS  = 0x301,   /**< GNSS -> CDH: p0..p2 = lat, p3..p5 = lon (big-endian 24-bit). */
    OP_GNSS_VX   = 0x302,   /**< p0..p3 = 32-bit value (big-endian), p4 = seq. */
    OP_GNSS_VY   = 0x303,   /**< same layout as OP_GNSS_VX. */
    OP_GNSS_VZ   = 0x304,   /**< same layout as OP_GNSS_VX. */
    OP_GNSS_TIME = 0x305,   /**< p0..p1 = GPS week, p2..p5 = time of week (big-endian). */
    OP_MOTOR_WHEEL_RPM = 0x310, /**< MOTOR wheel RPM telemetry (was OP_SET_WHEEL_RPM in telemetry class). */
    OP_ADCS_WHEEL_RPM  = 0x311, /**< wheel1..wheel3 rpm packed in p0..p5. */
    OP_ADCS_MTQ_DIPOLE = 0x312, /**< mx/my/mz packed in p0..p5. */

    /* ---- 0x4xx state of health ---- */
    OP_SOH_GNSS  = 0x401,   /**< GNSS SOH, up to 8 bytes p0..p7. */
    OP_SOH_SOLAR = 0x402,   /**< SOLAR SOH, p0..p7. */
    OP_SOH_MOTOR = 0x403,   /**< MOTOR SOH, p0..p7. */
    OP_ADCS_SOH_ATTITUDE = 0x404, /**< qx/qy/qz packed in p0..p5. */
} can_op_t;

/**
 * @brief Fixed payload length (DLC) for an opcode.
 * @param op Opcode (12-bit value from the CAN ID).
 * @return 2, 4 or 8; 0 if the opcode is unknown (such frames are rejected).
 */
static inline uint8_t can_op_len(uint16_t op)
{
    switch (op) {
    /* 2 bytes */
    case OP_HEARTBEAT:
    case OP_PING:
    case OP_PONG:
    case OP_BUTTON:
    case OP_SET_LED:
    case OP_SET_MODE:
    case OP_REBOOT:
    case OP_SET_PWR_STATE:
    case OP_GET_POS:
    case OP_SET_UPDATE_RATE:
    case OP_SET_EPS_LOAD:
    case OP_RESP_GET_POS:
    case OP_RESP_SET_UPDATE_RATE:
    case OP_RESP_SET_WHEEL_RPM:
    case OP_RESP_SET_MAG_DIPOLE:
    case OP_RESP_SET_EPS_LOAD:
        return 2;
    /* 4 bytes */
    case OP_SET_WHEEL_RPM:
    case OP_RESP_SET_PWR_STATE:
    case OP_MOTOR_WHEEL_RPM:
    case OP_SOH_MOTOR:
        return 4;
    /* 8 bytes */
    case OP_SET_MAG_DIPOLE:
    case OP_GNSS_POS:
    case OP_GNSS_VX:
    case OP_GNSS_VY:
    case OP_GNSS_VZ:
    case OP_GNSS_TIME:
    case OP_ADCS_WHEEL_RPM:
    case OP_ADCS_MTQ_DIPOLE:
    case OP_SOH_GNSS:
    case OP_SOH_SOLAR:
    case OP_ADCS_SOH_ATTITUDE:
        return 8;
    default:
        return 0;
    }
}
