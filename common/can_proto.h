#pragma once

/**
 * @file can_proto.h
 * @ingroup common
 * @brief Shared UT-CORE CAN bus protocol: ID layout, node IDs, and payload
 *        packing (opcodes live in ops.h).
 *
 * Defines the 29-bit extended CAN ID bit layout (priority/src/dst/op),
 * macros to construct and mask that ID, every subsystem's fixed node ID,
 * and a helper for packing the 8-byte payload.
 * Included by every board's firmware (CDH, EPS, GNSS, MOTOR, SOLAR) to
 * keep CAN traffic consistent across the satellite.
 *
 * UT-CORE CAN Protocol  —  29-bit Extended Frame ID Layout
 * ```
 * [28:26]  PRIORITY  (3 bits)   0 = highest
 * [25:21]  SRC       (5 bits)   source node ID
 * [20:16]  DST       (5 bits)   destination node ID; 0x1F = broadcast
 * [15:4]   OP        (12 bits)  opcode, one per message type (see ops.h)
 * [3:0]    RESERVED  (4 bits)   must be 0
 * ```
 * All 8 data bytes are application payload; the source node and opcode are
 * carried only in the ID.
 */


#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/drivers/can.h>
#include <stdint.h>
#include <zephyr/drivers/gpio.h>
#include "ops.h"

#define CAN_SRC_SHIFT       21
#define CAN_DST_SHIFT       16
#define CAN_OP_SHIFT        4

#define CAN_PRIO(p)    (((uint32_t)(p)  & 0x07U) << 26)
#define CAN_SRC(s)     (((uint32_t)(s)  & 0x1FU) << CAN_SRC_SHIFT)
#define CAN_DST(d)     (((uint32_t)(d)  & 0x1FU) << CAN_DST_SHIFT)
#define CAN_OP(o)      (((uint32_t)(o)  & 0xFFFU) << CAN_OP_SHIFT)

/** Build a full 29-bit ID. */
#define CAN_ID_FULL(prio, src, dst, op) \
    (CAN_PRIO(prio) | CAN_SRC(src) | CAN_DST(dst) | CAN_OP(op))

/* Field extraction */
#define CAN_ID_PRIO(id)  (((id) >> 26) & 0x07U)
#define CAN_ID_SRC(id)   (((id) >> CAN_SRC_SHIFT) & 0x1FU)
#define CAN_ID_DST(id)   (((id) >> CAN_DST_SHIFT) & 0x1FU)
#define CAN_ID_OP(id)    (((id) >> CAN_OP_SHIFT) & 0xFFFU)

/* Masks for filtering */
#define CAN_DST_MASK_29     (0x1FU << CAN_DST_SHIFT)   /* 0x001F0000 */
#define CAN_BROADCAST       0x1F

/* Node IDs */
#define CDH_ID      0x01
#define EPS_ID      0x02
#define COMMS_ID    0x03
#define ADCS_ID     0x04
#define MOTOR_ID    0x05
#define GNSS_ID     0x06
#define STAR_ID     0x07
#define SOLAR_ID    0x08
#define GROUND_ID   0x10


typedef struct {
    uint8_t  priority;
    uint8_t  src;
    uint8_t  dst;
    uint16_t op;
    uint8_t  data[8];
    uint8_t  dlc;
} can_packet_t;

/**
 * @brief Configure bitrate/mode, start the CAN controller, and install
 *        RX filters for this node's address and broadcast.
 * @param can_dev    CAN controller device.
 * @param my_id_node This board's node ID (used for the to-me filter).
 * @param rxq        Message queue to receive filtered frames into.
 * @return 0 on success, negative errno on failure.
 */

int can_setup(const struct device *can_dev, uint8_t my_id_node, struct k_msgq *rxq, const struct gpio_dt_spec *can_stb);

/* TCAN3403 transceiver control pins (GPIOA), identical on every UT-CORE board
 * that uses it. Override before including this header if a board differs. */
#ifndef CAN_TCAN_SILENT_PIN
#define CAN_TCAN_SILENT_PIN 9    /**< PA9  - TCAN3403 silent mode (active high)  */
#endif
#ifndef CAN_TCAN_SHDN_PIN
#define CAN_TCAN_SHDN_PIN   10   /**< PA10 - TCAN3403 shutdown (active high)     */
#endif

/**
 * @brief Bring the TCAN3403 transceiver out of shutdown/silent mode.
 *
 * Drives CAN_TCAN_SHDN_PIN and CAN_TCAN_SILENT_PIN low on GPIOA, then waits
 * 1 ms for the transceiver to become active. Call before can_setup(). Boards
 * that control the transceiver with a devicetree STB pin (MOTOR) pass it to
 * can_setup() instead and do not call this.
 *
 * @return 0 on success, -ENODEV if GPIOA is not ready, -ENOTSUP if the board
 *         has no gpioa node.
 */
int tcan3403_wakeup(void);

/**
 * @brief Unpack a raw CAN frame's 29-bit ID and payload into a can_packet_t.
 *
 * pkt->data is zeroed first, then the frame's payload is copied in, so bytes
 * beyond dlc are always 0. The frame is rejected if its DLC does not equal
 * can_op_len(op) (which also rejects unknown opcodes).
 *
 * @param f   Raw CAN frame as received from the driver.
 * @param pkt Output decoded packet (fields are filled even on rejection).
 * @return 0 on success, -EMSGSIZE if the DLC does not match the opcode's
 *         fixed length (counted in can_proto_rx_len_error_count()).
 */
int can_decode(const struct can_frame *f, can_packet_t *pkt);

/**
 * @brief Build and send one CAN frame of a fixed payload length.
 *
 * @param can_dev CAN controller device to send on.
 * @param src     Source node ID (this board's own ID).
 * @param dst     Destination node ID, or CAN_BROADCAST for all nodes.
 * @param op      Opcode (see ops.h); carried in the CAN ID.
 * @param prio    Arbitration priority (0 = highest; see CAN_PRIO()).
 * @param data    Payload bytes (at least len bytes).
 * @param len     Payload length; must equal can_op_len(op) (2, 4 or 8).
 *
 * Prefer the can_send2/can_send4/can_send8 wrappers below.
 *
 * @note Returns as soon as the frame is handed to the CAN driver, not once
 *       it is acknowledged on the bus. A 0 return means the frame was
 *       queued; it does not mean a peer received it. If no peer acknowledges
 *       the frame, the failure shows up asynchronously in
 *       can_proto_tx_error_count() rather than blocking this call.
 * @return 0 if queued, -EINVAL if len does not match can_op_len(op), or a
 *         negative errno from the driver (invalid ID, no free TX mailbox).
 */
int can_send_frame(const struct device *can_dev, uint8_t src, uint8_t dst,
                   uint16_t op, uint8_t prio, const uint8_t *data, uint8_t len);

/** @brief Send a 2-byte frame. @see can_send_frame */
static inline int can_send2(const struct device *can_dev, uint8_t src, uint8_t dst,
                            uint16_t op, uint8_t prio, uint8_t p0, uint8_t p1)
{
    const uint8_t d[2] = {p0, p1};
    return can_send_frame(can_dev, src, dst, op, prio, d, 2);
}

/** @brief Send a 4-byte frame. @see can_send_frame */
static inline int can_send4(const struct device *can_dev, uint8_t src, uint8_t dst,
                            uint16_t op, uint8_t prio,
                            uint8_t p0, uint8_t p1, uint8_t p2, uint8_t p3)
{
    const uint8_t d[4] = {p0, p1, p2, p3};
    return can_send_frame(can_dev, src, dst, op, prio, d, 4);
}

/** @brief Send an 8-byte frame. @see can_send_frame */
static inline int can_send8(const struct device *can_dev, uint8_t src, uint8_t dst,
                            uint16_t op, uint8_t prio,
                            uint8_t p0, uint8_t p1, uint8_t p2, uint8_t p3,
                            uint8_t p4, uint8_t p5, uint8_t p6, uint8_t p7)
{
    const uint8_t d[8] = {p0, p1, p2, p3, p4, p5, p6, p7};
    return can_send_frame(can_dev, src, dst, op, prio, d, 8);
}

/**
 * @brief Number of CAN sends that failed to complete since boot, as
 *        reported asynchronously by the CAN driver after can_send_frame()
 *        has already returned.
 *
 * A failure here most often means a frame went unacknowledged on the bus
 * (no other node received it) rather than a local driver error. Intended
 * for future SOH/diagnostics reporting.
 *
 * @return Rolling count of failed sends. Wraps like any other 32-bit
 *         counter; treat it as a diagnostic value, not a one-time
 *         threshold to alarm on directly.
 */
uint32_t can_proto_tx_error_count(void);

/**
 * @brief Number of received frames dropped by can_decode() because their DLC
 *        did not match can_op_len() for the opcode (or the opcode is unknown).
 * @return Rolling count; wraps like any 32-bit counter.
 */
uint32_t can_proto_rx_len_error_count(void);

#ifdef __cplusplus
}
#endif
