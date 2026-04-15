#pragma once
#include <zephyr/drivers/can.h>
#include "cdh_can_proto.h"
#include "cdh_events.h"
#ifdef __cplusplus
extern "C" {
    #endif


    #define CDH_NODE_ID 0x1 /* logical CDH node id */


    int can_bus_init(const struct device **out_dev);
    int can_bus_send_u8(const struct device *dev, uint16_t id,
    uint8_t src, uint8_t op,
    uint8_t a,uint8_t b,uint8_t c,uint8_t d,uint8_t e,uint8_t f);


    /* Non-blocking get of an incoming frame; returns 0 if got one */
    int can_bus_try_get(struct can_frame *out);


    #ifdef __cplusplus
}
#endif