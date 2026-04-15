#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/logging/log.h>
#include "can_bus.h"


LOG_MODULE_REGISTER(can_bus, LOG_LEVEL_INF);


CAN_MSGQ_DEFINE(cdh_can_rxq, 16);


int can_bus_init(const struct device **out_dev) {
    const struct device *can_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));
    if (!device_is_ready(can_dev)) return -ENODEV;


    (void)can_stop(can_dev);
    (void)can_set_mode(can_dev, CAN_MODE_NORMAL); /* CDH is active node */
    if (can_start(can_dev)) return -EIO;


    /* Receive frames addressed to CDH and broadcasts */
    struct can_filter to_me = { .id = CAN_DST(CDH_NODE_ID), .mask = CAN_DST_MASK, .flags = 0 };
    struct can_filter bcast = { .id = CAN_DST(CAN_BROADCAST), .mask = CAN_DST_MASK, .flags = 0 };
    can_add_rx_filter_msgq(can_dev, &cdh_can_rxq, &to_me);
    can_add_rx_filter_msgq(can_dev, &cdh_can_rxq, &bcast);


    if (out_dev) *out_dev = can_dev;
    return 0;
}


int can_bus_send_u8(const struct device *dev, uint16_t id, uint8_t src, uint8_t op, uint8_t a,uint8_t b,uint8_t c,uint8_t d,uint8_t e,uint8_t f) {
    struct can_frame f = { .id = id };
    can_fill_payload(&f, src, op, a,b,c,d,e,f);
    return can_send(dev, &f, K_MSEC(200), NULL, NULL);
}


int can_bus_try_get(struct can_frame *out) {
    if (!out) return -EINVAL;
    return k_msgq_get(&cdh_can_rxq, out, K_NO_WAIT);
}