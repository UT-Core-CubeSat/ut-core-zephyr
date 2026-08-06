#pragma once

/**
 * @file can_proto.h
 * @ingroup common
 * @brief Shared UT-CORE CAN bus protocol: ID layout, node IDs, message
 *        classes, opcodes, and payload packing.
 *
 * Defines the 29-bit extended CAN ID bit layout (priority/src/dst/class),
 * macros to construct and mask that ID, every subsystem's fixed node ID,
 * the message-class taxonomy, the shared opcode enum used across all
 * UT-CORE apps, and a helper for packing the standard 8-byte payload.
 * Included by every board's firmware (CDH, EPS, GNSS, MOTOR, SOLAR) to
 * keep CAN traffic consistent across the satellite.
 *
 * UT-CORE CAN Protocol  —  29-bit Extended Frame ID Layout
 * ```
 * [28:26]  PRIORITY  (3 bits)   0 = highest
 * [25:22]  RESERVED  (4 bits)   must be 0
 * [21:14]  SRC       (8 bits)   source node ID
 * [13:6]   DST       (8 bits)   destination node ID; 0xFF = broadcast
 * [5:0]    CLASS     (6 bits)   message class
 * ```
 */


#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/drivers/can.h>
#include <stdint.h>

#define CAN_PRIO(p)    (((uint32_t)(p)  & 0x07U) << 26)
#define CAN_SRC(s)     (((uint32_t)(s)  & 0xFFU) << 14)
#define CAN_DST(d)     (((uint32_t)(d)  & 0xFFU) <<  6)
#define CAN_CLASS(c)   (((uint32_t)(c)  & 0x3FU)      )

#define CAN_ID(prio, dst, cls) \
    (CAN_PRIO(prio) | CAN_DST(dst) | CAN_CLASS(cls))

/** Use this variant when you want SRC baked into the ID (optional, useful for filters) */
#define CAN_ID_FULL(prio, src, dst, cls) \
    (CAN_PRIO(prio) | CAN_SRC(src) | CAN_DST(dst) | CAN_CLASS(cls))

/* Masks for filtering */
#define CAN_DST_SHIFT       6
#define CAN_DST_MASK_29     (0xFFU << CAN_DST_SHIFT)   /* 0x00003FC0 */
#define CAN_BROADCAST       0xFF

/* Node IDs */
#define CDH_ID      0x01
#define EPS_ID      0x02
#define COMMS_ID    0x03
#define ADCS_ID     0x04
#define MOTOR_ID    0x05
#define GNSS_ID     0x06
#define STAR_ID     0x07
#define SOLAR_ID    0x08

/* Message Classes */
#define CLS_HEARTBEAT   0
#define CLS_COMMAND     2
#define CLS_CMD_RESP    3
#define CLS_TELEMETRY   4
#define CLS_HEALTH      10

/**
 * @brief Opcodes used across all UT-CORE CAN message classes.
 *
 * Payload byte layout for multi-byte opcodes is opcode-specific; see the
 * per-opcode comments above each value for p2..p7 field meaning (e.g.
 * OP_SET_WHEEL_RPM, OP_SET_MAG_DIPOLE, OP_SET_EPS_LOAD, and the
 * OP_ADCS_* telemetry opcodes).
 */
typedef enum {
    OP_PING      = 0x01,    /**< _ */
    OP_PONG      = 0x02,    /**< _ */
    OP_BUTTON    = 0x10,    /**< _ */
    OP_SET_LED   = 0x20,    /**< _ */
    OP_HEARTBEAT = 0x30,    /**< _ */
    OP_SET_MODE  = 0x40,    /**< _ */
    OP_REBOOT    = 0x41,    /**< _ */
    OP_SET_PWR_STATE = 0x60,/**< _ */
    OP_GET_POS = 0x61,      /**< _ */
    /**
     * Set wheel RPM reference (CLS_COMMAND):
     *   p2 = motor index (1..4), or 0 for all motors
     *   p3..p4 = signed RPM reference (big-endian int16_t)
     */
    OP_SET_WHEEL_RPM = 0x50,
    /**
     * Set magnetorquer commanded dipole vector (CLS_COMMAND):
     *   p2..p3 = mx (big-endian int16_t, 1e-4 A*m^2 per LSB)
     *   p4..p5 = my (big-endian int16_t, 1e-4 A*m^2 per LSB)
     *   p6..p7 = mz (big-endian int16_t, 1e-4 A*m^2 per LSB)
     */
    OP_SET_MAG_DIPOLE = 0x51,
    /**
     * EPS load switch command (CLS_COMMAND):
     *   p2 = load index (1..12), or 0 for all loads
     *   p3 = enable flag (0 = disable, nonzero = enable)
     */
    OP_SET_EPS_LOAD = 0x52,
    /*
     * ADCS app telemetry/scaffold opcodes:
     *   OP_ADCS_SOH_ATTITUDE (CLS_HEALTH): qx/qy/qz packed in p2..p7
     *   OP_ADCS_WHEEL_RPM    (CLS_TELEMETRY): wheel1..wheel3 rpm packed in p2..p7
     *   OP_ADCS_MTQ_DIPOLE   (CLS_TELEMETRY): mx/my/mz packed in p2..p7
     */
    OP_ADCS_SOH_ATTITUDE = 0x70, /**< OP_ADCS_SOH_ATTITUDE (CLS_HEALTH): qx/qy/qz packed in p2..p7 */
    OP_ADCS_WHEEL_RPM    = 0x71, /**< OP_ADCS_WHEEL_RPM (CLS_TELEMETRY): wheel1..wheel3 rpm packed in p2..p7 */
    OP_ADCS_MTQ_DIPOLE   = 0x72, /**< OP_ADCS_MTQ_DIPOLE (CLS_TELEMETRY): mx/my/mz packed in p2..p7 */
} can_op_t;

/**
 * @brief Standard 8-byte CAN application payload layout: source node,
 *        opcode, and 6 opcode-specific data bytes.
 */
struct can_payload {
    uint8_t src;
    uint8_t op;
    uint8_t p2, p3, p4, p5, p6, p7;
};

/**
 * @brief Pack a can_frame's payload with the standard [src, op, p2..p7]
 *        layout and mark it as an extended (29-bit) frame.
 * @param f   CAN frame to populate (id must already be set separately).
 * @param src Source node ID.
 * @param op  Opcode.
 * @param p2  Payload byte 2.
 * @param p3  Payload byte 3.
 * @param p4  Payload byte 4.
 * @param p5  Payload byte 5.
 * @param p6  Payload byte 6.
 * @param p7  Payload byte 7.
 *
 * Always sets dlc to 8, even for opcodes that use fewer meaningful bytes.
 */
static inline void can_fill_payload(struct can_frame *f,
                                    uint8_t src, uint8_t op,
                                    uint8_t p2, uint8_t p3, uint8_t p4,
                                    uint8_t p5, uint8_t p6, uint8_t p7)
{
    f->dlc    = 8;
    f->flags  = CAN_FRAME_IDE;   /* <-- Mark as extended frame */
    f->data[0] = src;
    f->data[1] = op;
    f->data[2] = p2; f->data[3] = p3;
    f->data[4] = p4; f->data[5] = p5;
    f->data[6] = p6; f->data[7] = p7;
}

#ifdef __cplusplus
}
#endif
