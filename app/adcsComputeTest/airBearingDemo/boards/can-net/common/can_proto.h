#pragma once
#include <zephyr/drivers/can.h>
#include <stdint.h>

/* ID helpers (standard 11-bit) */
#define CAN_PRIO(p)  (((p) & 0x7) << 8)
#define CAN_DST(d)   (((d) & 0xF) << 4)
#define CAN_CLASS(c) ((c) & 0xF)
#define CAN_ID(prio, dst, cls) (CAN_PRIO(prio) | CAN_DST(dst) | CAN_CLASS(cls))

#define CAN_DST_MASK  0x0F0
#define CAN_BROADCAST 0xF

/* Opcodes */
enum can_op {
  OP_PING      = 0x01,
  OP_PONG      = 0x02,
  OP_BUTTON    = 0x10,   /* button event from EPS -> CDH */
  OP_SET_LED   = 0x20,   /* set LED state on EPS */
  OP_HEARTBEAT = 0x30,   /* periodic heartbeat */
};

/* Simple 8-byte app payload */
struct can_payload {
  uint8_t src;     /* sender node id */
  uint8_t op;      /* enum can_op */
  uint8_t p2, p3, p4, p5, p6, p7; /* free */
};

static inline void can_fill_payload(struct can_frame *f,
                                    uint8_t src, uint8_t op,
                                    uint8_t p2,uint8_t p3,uint8_t p4,
                                    uint8_t p5,uint8_t p6,uint8_t p7)
{
  f->dlc = 8; f->flags = 0;
  f->data[0]=src; f->data[1]=op; f->data[2]=p2; f->data[3]=p3;
  f->data[4]=p4;  f->data[5]=p5; f->data[6]=p6; f->data[7]=p7;
}
