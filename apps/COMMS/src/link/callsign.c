/** @file callsign.c @ingroup comms @brief Callsign secondary header, see callsign.h */
#include "callsign.h"

#include <errno.h>
#include <string.h>

int callsign_pack(uint8_t *out, const char *call, uint8_t ssid)
{
	size_t n;

	if (out == NULL || call == NULL) {
		return -EINVAL;
	}
	n = strlen(call);
	if (n == 0 || n > COMMS_CALLSIGN_LEN) {
		return -EINVAL;
	}
	for (size_t i = 0; i < n; i++) {
		char c = call[i];

		if (!((c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9'))) {
			return -EINVAL;
		}
	}
	memset(out, ' ', COMMS_CALLSIGN_LEN);
	memcpy(out, call, n);
	out[COMMS_CALLSIGN_LEN] = ssid;
	return 0;
}
