/**
 * @file orion_b16_parser.h
 * @brief Zero-allocation state-machine parser for the SkyTraq Binary Protocol.
 *
 * Feed raw UART bytes one at a time into `orion_parser_feed_byte()`.
 * When a complete, checksum-validated frame arrives, the parser fills the
 * provided `orion_frame_t` and returns `ORION_PARSE_FRAME_READY`.
 *
 * The parser also provides `orion_decode_message()` which takes a raw frame
 * and populates an `orion_message_t` with fully decoded, native-typed fields.
 *
 * This module is pure C with no OS dependencies — it can be unit-tested on
 * any host, or dropped into any RTOS.
 */

#ifndef ORION_B16_PARSER_H
#define ORION_B16_PARSER_H

#include "orion_b16_protocol.h"
#include "orion_b16_messages.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ──────────────── Parser State Machine ──────────────────────────── */

/** Return values from the byte-feeding function. */
typedef enum {
    ORION_PARSE_NEED_MORE   = 0,  /**< Byte consumed, need more data   */
    ORION_PARSE_FRAME_READY = 1,  /**< A complete valid frame is ready */
    ORION_PARSE_ERROR       = -1, /**< Checksum fail or framing error  */
} orion_parse_result_t;

/** Internal parser states. */
typedef enum {
    ORION_PS_SEEK_SOF0 = 0,
    ORION_PS_SEEK_SOF1,
    ORION_PS_LEN_HI,
    ORION_PS_LEN_LO,
    ORION_PS_PAYLOAD,
    ORION_PS_CHECKSUM,
    ORION_PS_EOF0,
    ORION_PS_EOF1,
} orion_parser_state_t;

/**
 * @brief Parser context — holds intermediate state between bytes.
 *
 * Allocate one of these (stack or static) and call `orion_parser_init()`,
 * then repeatedly call `orion_parser_feed_byte()`.
 */
typedef struct {
    orion_parser_state_t state;
    uint16_t             payload_len;   /**< Expected payload length         */
    uint16_t             payload_idx;   /**< Bytes of payload received so far */
    uint8_t              payload_buf[ORION_MAX_PAYLOAD]; /**< Accumulation buffer */
    uint8_t              checksum_rx;   /**< Received checksum byte          */
    uint32_t             frames_ok;     /**< Successful frame count          */
    uint32_t             frames_err;    /**< Framing/checksum error count    */
} orion_parser_t;


/**
 * @brief Initialize (or reset) a parser context.
 */
void orion_parser_init(orion_parser_t *p);

/**
 * @brief Feed one byte into the parser state machine.
 *
 * @param p    Parser context.
 * @param byte The next byte from the UART stream.
 * @param out  Pointer to a frame struct that will be filled on FRAME_READY.
 *             May be NULL if you only want to detect completion.
 *
 * @return ORION_PARSE_NEED_MORE  — keep feeding bytes.
 *         ORION_PARSE_FRAME_READY — `*out` now holds a valid decoded frame.
 *         ORION_PARSE_ERROR       — bad checksum or EOF mismatch; parser resynced.
 */
orion_parse_result_t orion_parser_feed_byte(orion_parser_t *p,
                                            uint8_t byte,
                                            orion_frame_t *out);

/* ──────────────── Message Decoder ───────────────────────────────── */

/**
 * @brief Decode a raw frame into a typed message struct.
 *
 * @param frame  Input: a frame returned by the parser.
 * @param msg    Output: decoded message with native-typed fields.
 *
 * @return 0 on success, -1 if the message ID is unknown or the body length
 *         doesn't match the expected layout.
 */
int orion_decode_message(const orion_frame_t *frame, orion_message_t *msg);

/**
 * @brief Decode just the Navigation Data (0xA8) body into the nav_data struct.
 *
 * This is the hot path — called frequently on every PVT update.
 *
 * @param body     Pointer to body bytes (after msg_id).
 * @param body_len Length of body.
 * @param out      Pointer to destination struct.
 *
 * @return 0 on success, -1 on length mismatch.
 */
int orion_decode_nav_data(const uint8_t *body, uint16_t body_len,
                          orion_nav_data_t *out);

#ifdef __cplusplus
}
#endif

#endif /* ORION_B16_PARSER_H */
