#pragma once
#include <zephyr/drivers/can.h>
#include <stdint.h>

/*
 * UT-CORE CAN Protocol  —  29-bit Extended Frame ID Layout
 *
 * [28:26]  PRIORITY  (3 bits)   0 = highest
 * [25:22]  RESERVED  (4 bits)   must be 0
 * [21:14]  SRC       (8 bits)   source node ID
 * [13:6]   DST       (8 bits)   destination node ID; 0xFF = broadcast
 * [5:0]    CLASS     (6 bits)   message class
 */

#define CAN_PRIO(p)    (((uint32_t)(p)  & 0x07U) << 26)
#define CAN_SRC(s)     (((uint32_t)(s)  & 0xFFU) << 14)
#define CAN_DST(d)     (((uint32_t)(d)  & 0xFFU) <<  6)
#define CAN_CLASS(c)   (((uint32_t)(c)  & 0x3FU)      )

#define CAN_ID(prio, dst, cls) \
    (CAN_PRIO(prio) | CAN_DST(dst) | CAN_CLASS(cls))

/* Use this variant when you want SRC baked into the ID (optional, useful for filters) */
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

/* Opcodes */
typedef enum {
    OP_PING      = 0x01,
    OP_PONG      = 0x02,
    OP_BUTTON    = 0x10,
    OP_SET_LED   = 0x20,
    OP_HEARTBEAT = 0x30,
    OP_SET_MODE  = 0x40,
    OP_REBOOT    = 0x41,
} can_op_t;

/* 8-byte app payload */
struct can_payload {
    uint8_t src;
    uint8_t op;
    uint8_t p2, p3, p4, p5, p6, p7;
};

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

