#pragma once

/**
 * @file health.h
 * @ingroup common
 * @brief Thread health table: tracks when each monitored thread last
 *        checked in, so a supervisor thread can detect threads that hang.
 *
 * Usage:
 *   1. Before starting any monitored thread, register each one with
 *      health_register() and keep the returned id.
 *   2. Each monitored thread calls health_checkin(id) once per loop.
 *   3. A supervisor thread periodically calls health_is_late() for every
 *      id from 0 to health_count() - 1.
 *
 * Registration is not thread-safe and must finish before the monitored
 * threads and the supervisor start running. Check-ins and queries are
 * safe from any thread.
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Maximum number of threads that can be registered. */
#define HEALTH_MAX_ENTRIES 8

/**
 * @brief Register a thread to be monitored.
 * @param name       Name used in log messages. Must remain valid forever
 *                   (use a string literal).
 * @param timeout_ms Longest allowed gap between check-ins.
 * @return Non-negative id on success, -ENOMEM if the table is full, or
 *         -EINVAL if timeout_ms is 0.
 */
int health_register(const char *name, uint32_t timeout_ms);

/** @brief Record that the thread with this id is still running. */
void health_checkin(int id);

/** @brief Number of registered threads. Valid ids are 0..count-1. */
int health_count(void);

/** @brief Name passed to health_register(), or "?" for an invalid id. */
const char *health_name(int id);

/** @brief Allowed gap passed to health_register(), or 0 for an invalid id. */
uint32_t health_timeout_ms(int id);

/** @brief Milliseconds since the thread last checked in (0 for an invalid id). */
uint32_t health_ms_since_checkin(int id);

/** @brief True if the thread has gone longer than its timeout without checking in. */
bool health_is_late(int id);

#ifdef __cplusplus
}
#endif
