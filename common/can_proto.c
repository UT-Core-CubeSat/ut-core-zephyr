/**
 * @file can_proto.c
 * @ingroup common
 * @brief Shared UT-CORE CAN bus helpers: frame decode, simple frame send,
 *        and common transceiver/controller bring-up.
 *
 * Plans to Implement the CAN handling logic previously duplicated across CDH,
 * EPS, GNSS, MOTOR, and SOLAR (frame decoding, single-opcode frame
 * construction, TCAN3403 wakeup, and CAN controller/filter setup), so a
 * protocol-level fix only needs to be made in one place.
 *
 * @todo move shared can source code to this file.
 */

#include "can_proto.h"
#include <zephyr/logging/log.h>

/** @cond */ /* Hidden from Doxygen, or it will mistake this as a function */
LOG_MODULE_REGISTER(can_proto, LOG_LEVEL_INF);
/** @endcond */

/**
 * @brief Configure bitrate/mode, start the CAN controller, and install
 *        RX filters for this node's address and broadcast.
 */

int can_setup(const struct device *can_dev, uint8_t my_id_node, struct k_msgq *rxq, const struct gpio_dt_spec *can_stb) {
    
    if (!device_is_ready(can_dev)) {
        LOG_ERR("CAN not ready: %s", can_dev->name);
        return -ENODEV;
    }

    if (rxq == NULL) {
        LOG_ERR("can_setup: rxq is null");
        return -EINVAL;
    }

    if (can_stb != NULL) {
        if (!device_is_ready(can_stb->port)) {
            LOG_ERR("CAN STB GPIO device not ready");
            return -ENODEV;
        }

        gpio_pin_configure_dt(can_stb, GPIO_OUTPUT_INACTIVE);
        gpio_pin_set_dt(can_stb, 0);
        k_msleep(5);
    } else {
        LOG_WRN("No CAN STB Pin provided; assuming transceiver needs no standby control");
    }

    int err;

    err = can_set_bitrate(can_dev, 500000);
    if (err){
        LOG_ERR("can_set_bitrate failed: %d", err);
        return err;
    }

    err = can_set_mode(can_dev, CAN_MODE_NORMAL);
    if (err){
        LOG_ERR("can_set_mode failed: %d", err);
        return err; 
    }

    err = can_start(can_dev);
    if (err != 0 && can_stb != NULL) {
        /* Some transceivers use opposite standby polarity; retry once with STB=1 */
        LOG_WRN("CAN start failed (%d), retrying with STB=1", err);
        gpio_pin_set_dt(can_stb, 1);
        k_msleep(5);
        err = can_start(can_dev);
        if (err == 0) {
            LOG_INF("CAN recovered with STB=1 (transceiver standby polarity is inverted)");
        }
    }
    if (err){
        LOG_ERR("can_start failed: %d", err);
        return err;
    }

    const struct can_filter to_me = {
        .id    = CAN_DST(my_id_node),
        .mask  = CAN_DST_MASK_29,
        .flags = CAN_FILTER_IDE
    };

    const struct can_filter bcast = {
        .id    = CAN_DST(CAN_BROADCAST),
        .mask  = CAN_DST_MASK_29,
        .flags = CAN_FILTER_IDE
    };

    int filter_id = can_add_rx_filter_msgq(can_dev, rxq, &to_me);
    if (filter_id < 0) {
        LOG_ERR("Failed to add to_me filter: %d", filter_id);
    }
    filter_id = can_add_rx_filter_msgq(can_dev, rxq, &bcast);
    if (filter_id < 0) {
        LOG_ERR("Failed to add bcast filter: %d", filter_id);
    }

    LOG_INF("Filter to_me:  id=0x%08X mask=0x%08X", to_me.id, to_me.mask);
    LOG_INF("Filter bcast:  id=0x%08X mask=0x%08X", bcast.id, bcast.mask);
    LOG_INF("CAN Initialized (29-bit extended)");

    return 0;
}

/**
 * @brief Unpack a raw CAN frame's 29-bit ID and payload into a can_packet_t.
 * @param f   Raw CAN frame as received from the driver.
 * @param pkt Output decoded packet.
 */

void can_decode(const struct can_frame *f, can_packet_t *pkt)
{
    uint32_t id   = f->id;
    pkt->priority  = (id >> 26) & 0x07;
    pkt->src       = (id >> 14) & 0xFF;
    pkt->dst       = (id >>  6) & 0xFF;
    pkt->msg_class =  id        & 0x3F;
    pkt->dlc       = f->dlc;
    memcpy(pkt->data, f->data, f->dlc);
}


void send_simple(const struct device *can_dev, uint8_t src, uint8_t dst,
                  uint8_t cls, uint8_t op, uint8_t val, uint8_t prio)
{
    struct can_frame f = {0};

    f.id = CAN_ID_FULL(prio, src, dst, cls);
    f.flags = CAN_FRAME_IDE;

    can_fill_payload(&f, src, op, val, 0, 0, 0, 0, 0);

    int err = can_send(can_dev, &f, K_NO_WAIT, NULL, NULL);
    if (err) {
        LOG_ERR("can_send failed: %d", err);
    }
}
