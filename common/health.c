/**
 * @file health.c
 * @ingroup common
 * @brief Thread health table implementation. See health.h.
 */

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

#include "health.h"

struct health_entry {
    const char *name;
    uint32_t    timeout_ms;
    atomic_t    last_checkin_ms;
};

static struct health_entry entries[HEALTH_MAX_ENTRIES];
static int entry_count;

static bool valid_id(int id)
{
    return id >= 0 && id < entry_count;
}

int health_register(const char *name, uint32_t timeout_ms)
{
    if (timeout_ms == 0U) {
        return -EINVAL;
    }
    if (entry_count >= HEALTH_MAX_ENTRIES) {
        return -ENOMEM;
    }

    struct health_entry *e = &entries[entry_count];

    e->name = name;
    e->timeout_ms = timeout_ms;
    /* Start the clock at registration so a thread isn't late before its first loop. */
    atomic_set(&e->last_checkin_ms, (atomic_val_t)k_uptime_get_32());

    return entry_count++;
}

void health_checkin(int id)
{
    if (!valid_id(id)) {
        return;
    }
    atomic_set(&entries[id].last_checkin_ms, (atomic_val_t)k_uptime_get_32());
}

int health_count(void)
{
    return entry_count;
}

const char *health_name(int id)
{
    return valid_id(id) ? entries[id].name : "?";
}

uint32_t health_timeout_ms(int id)
{
    return valid_id(id) ? entries[id].timeout_ms : 0U;
}

uint32_t health_ms_since_checkin(int id)
{
    if (!valid_id(id)) {
        return 0U;
    }

    uint32_t now  = k_uptime_get_32();
    uint32_t last = (uint32_t)atomic_get(&entries[id].last_checkin_ms);

    /* Unsigned subtraction stays correct when the 32-bit uptime wraps (~49 days). */
    return now - last;
}

bool health_is_late(int id)
{
    return valid_id(id) && health_ms_since_checkin(id) > entries[id].timeout_ms;
}
