#pragma once
#include <stdint.h>
#include <zephyr/drivers/can.h>
#ifdef __cplusplus
extern "C" {
    #endif


    /* 11-bit ID layout: prio[10:8] | dst[7:4] | cls[3:0] */
    #define CAN_PRIO(p) (((p) & 0x7) << 8)
    #define CAN_DST(d) (((d) & 0xF) << 4)
    #define CAN_CLS(c) ((c) & 0xF)
    #define CAN_ID(prio, dst, cls) (CAN_PRIO(prio) | CAN_DST(dst) | CAN_CLS(cls))


    #define CAN_DST_MASK 0x0F0
    #define CAN_BROADCAST 0xF


    /* classes */
    enum can_cls {
        CLS_CORE = 0x0,
        CLS_TLM = 0x1,
        CLS_CMD = 0x2,
        CLS_SOH = 0x3,
    };


    /* opcodes in payload[1] */
    enum can_op {
        OP_PING = 0x01,
        OP_PONG = 0x02,
        OP_BUTTON = 0x10,
        OP_SET_LED = 0x20,
        OP_SOH_FRAME = 0x30,
        OP_GROUND_CMD= 0x40,
    };


    struct can_payload {
        uint8_t src; /* payload[0] */
        uint8_t op; /* payload[1] */
        uint8_t p2, p3, p4, p5, p6, p7;
    };


    static inline void can_fill_payload(struct can_frame *f,
    uint8_t src, uint8_t op,
    uint8_t p2,uint8_t p3,uint8_t p4,
    uint8_t p5,uint8_t p6,uint8_t p7)
    {
    f->dlc = 8; f->flags = 0;
    f->data[0]=src; f->data[1]=op; f->data[2]=p2; f->data[3]=p3;
    f->data[4]=p4; f->data[5]=p5; f->data[6]=p6; f->data[7]=p7;
    }


    #ifdef __cplusplus
}
#endif