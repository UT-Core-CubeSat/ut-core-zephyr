/** @file ping.c @ingroup comms @brief PING reply, see ping.h */
#include "ping.h"

#include <errno.h>
#include <string.h>

int ping_make_reply(const uint8_t *req, size_t req_len, uint8_t *out, size_t cap)
{
	if (req == NULL || out == NULL || req_len < PING_ID_LEN || req_len > PING_MAX_DATA) {
		return -EINVAL;
	}
	if (cap < req_len) {
		return -EMSGSIZE;
	}
	memcpy(out, req, req_len);
	return (int)req_len;
}
