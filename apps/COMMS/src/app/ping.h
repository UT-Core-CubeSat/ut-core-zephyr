/**
 * @file ping.h
 * @ingroup comms
 * @brief PING test packet (APID 0x0F0 request, 0x0F1 reply), PROTOCOL.md section 14
 *
 * Data field: `ping_id` (4 bytes, big-endian, ground counts up from 0) then 0 to 200 payload bytes. The
 * reply carries the same bytes unchanged and the board adds nothing. The module only checks the length,
 * so any `ping_id` value and any payload content are echoed.
 */
#ifndef COMMS_PING_H_
#define COMMS_PING_H_

#include <stddef.h>
#include <stdint.h>

#define PING_ID_LEN      4   /**< `ping_id`, big-endian. */
#define PING_MAX_PAYLOAD 200 /**< Largest payload ground may send. */
#define PING_MAX_DATA    (PING_ID_LEN + PING_MAX_PAYLOAD)

/**
 * Build the reply data field for a ping request data field.
 *
 * @param req     request data field (`ping_id` + payload)
 * @param req_len its length
 * @param out     reply buffer
 * @param cap     size of @p out
 * @return reply length, -EINVAL if the arguments are NULL or the request is shorter than `ping_id` or
 *         longer than PING_MAX_DATA, -EMSGSIZE if @p cap is too small
 */
int ping_make_reply(const uint8_t *req, size_t req_len, uint8_t *out, size_t cap);

#endif /* COMMS_PING_H_ */
