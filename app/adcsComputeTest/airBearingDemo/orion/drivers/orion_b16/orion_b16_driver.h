/**
 * @file orion_b16_driver.h
 * @brief High-level Orion B16 GNSS driver for ZephyrRTOS.
 *
 * This driver manages UART communication with the Orion B16, provides a
 * background parsing thread, and exposes a clean C API for the CubeSat
 * flight software to:
 *
 *   1. Initialize and configure the receiver (binary mode, update rate, etc.)
 *   2. Read the latest navigation solution (lat, lon, alt, vel, time)
 *   3. Register callbacks for real-time PVT updates
 *   4. Send arbitrary commands and wait for ACK/NACK
 *
 * All public functions are thread-safe (Zephyr mutexes + semaphores).
 *
 * Usage:
 * @code
 *     static orion_driver_t gnss;
 *
 *     // In your init task:
 *     orion_driver_cfg_t cfg = ORION_DRIVER_CFG_DEFAULTS;
 *     cfg.uart_dev_name = "UART_1";   // or from DTS alias
 *     cfg.update_rate_hz = 1;
 *     orion_driver_init(&gnss, &cfg);
 *     orion_driver_start(&gnss);
 *
 *     // Anywhere later — non-blocking snapshot:
 *     orion_nav_data_t nav;
 *     if (orion_driver_get_nav(&gnss, &nav) == 0 && nav.valid) {
 *         double lat = ORION_DEG_FROM_1E7(nav.latitude_1e7);
 *         double lon = ORION_DEG_FROM_1E7(nav.longitude_1e7);
 *         // ...
 *     }
 * @endcode
 */

#ifndef ORION_B16_DRIVER_H
#define ORION_B16_DRIVER_H

#include "orion_b16_protocol.h"
#include "orion_b16_messages.h"
#include "orion_b16_parser.h"
#include "orion_b16_commands.h"

#ifdef __ZEPHYR__
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* ──────────────── Configuration ─────────────────────────────────── */

/** Maximum number of navigation data callbacks that can be registered. */
#define ORION_MAX_NAV_CALLBACKS  4

/** Stack size for the parser background thread. */
#define ORION_PARSER_STACK_SIZE  2048

/** Parser thread priority (cooperative or preemptive — your choice). */
#define ORION_PARSER_THREAD_PRIO 5

/** Size of the UART receive ring buffer. */
#define ORION_UART_RX_BUF_SIZE  1024

/** Timeout for ACK/NACK after sending a command, in milliseconds. */
#define ORION_CMD_TIMEOUT_MS    1000

/**
 * @brief Driver configuration — pass to `orion_driver_init()`.
 */
typedef struct {
    const char *uart_dev_name;    /**< Zephyr UART device name (e.g. "UART_1") */
    uint8_t     update_rate_hz;   /**< Desired position update rate in Hz      */
    uint8_t     msg_type;         /**< ORION_MSG_TYPE_BINARY recommended       */
    uint8_t     save_attr;        /**< ORION_ATTR_SRAM_ONLY or SRAM_FLASH      */
    uint32_t    baud_rate;        /**< UART baud rate (default 115200)         */
} orion_driver_cfg_t;

/** Sensible defaults. */
#define ORION_DRIVER_CFG_DEFAULTS {        \
    .uart_dev_name  = "UART_1",            \
    .update_rate_hz = 1,                   \
    .msg_type       = ORION_MSG_TYPE_BINARY, \
    .save_attr      = ORION_ATTR_SRAM_ONLY,  \
    .baud_rate      = 115200,              \
}

/* ──────────────── Callback Types ────────────────────────────────── */

/**
 * @brief Called every time a new Navigation Data message is decoded.
 * @param nav  Pointer to the decoded nav data (valid for this call's duration).
 * @param user User context pointer (registered with the callback).
 */
typedef void (*orion_nav_callback_t)(const orion_nav_data_t *nav, void *user);

/**
 * @brief Called for every fully-decoded message of any type.
 * @param msg  Pointer to the decoded message (valid for this call's duration).
 * @param user User context pointer.
 */
typedef void (*orion_msg_callback_t)(const orion_message_t *msg, void *user);

/* ──────────────── Driver State ──────────────────────────────────── */

/** Internal driver state — treat as opaque. */
typedef struct {
    /* Configuration */
    orion_driver_cfg_t  cfg;

#ifdef __ZEPHYR__
    /* Zephyr resources */
    const struct device *uart_dev;
    struct k_thread      parser_thread;
    struct k_mutex       nav_mutex;           /**< Protects latest_nav   */
    struct k_mutex       cmd_mutex;           /**< Serializes commands   */
    struct k_sem         ack_sem;             /**< Signaled on ACK/NACK  */
    K_KERNEL_STACK_MEMBER(parser_stack, ORION_PARSER_STACK_SIZE);
#endif

    /* Parser */
    orion_parser_t      parser;

    /* Latest navigation solution (double-buffered via mutex) */
    orion_nav_data_t    latest_nav;

    /* ACK state for command-response pairing */
    volatile uint8_t    pending_cmd_id;       /**< Which command we're waiting ACK for */
    volatile bool       ack_received;         /**< ACK (true) or NACK (false)          */
    volatile bool       ack_valid;            /**< An ACK/NACK arrived                 */

    /* Callbacks */
    struct {
        orion_nav_callback_t fn;
        void                *user;
    } nav_callbacks[ORION_MAX_NAV_CALLBACKS];
    uint8_t             nav_cb_count;

    orion_msg_callback_t msg_callback;
    void                *msg_callback_user;

    /* UART RX ring buffer for interrupt-driven mode */
#ifdef __ZEPHYR__
    uint8_t             rx_ring_buf[ORION_UART_RX_BUF_SIZE];
    struct ring_buf     rx_rb;
#endif

    /* Statistics */
    uint32_t            rx_bytes;
    uint32_t            tx_bytes;
    uint32_t            nav_count;            /**< Total 0xA8 messages decoded */

    /* Running flag */
    volatile bool       running;
} orion_driver_t;


/* ──────────────── Public API ────────────────────────────────────── */

/**
 * @brief Initialize the driver (does NOT start communication).
 *
 * Call this once at boot. It sets up internal structures, opens the UART
 * device, and configures the parser. Does not send any commands yet.
 *
 * @param drv  Driver instance (caller-allocated, e.g. static).
 * @param cfg  Configuration. A copy is stored internally.
 *
 * @return 0 on success, negative errno on failure.
 */
int orion_driver_init(orion_driver_t *drv, const orion_driver_cfg_t *cfg);

/**
 * @brief Start the driver — begins UART RX, spawns the parser thread,
 *        and configures the receiver for binary PVT output.
 *
 * After this call, `orion_driver_get_nav()` will begin returning data
 * as soon as the receiver achieves a fix.
 *
 * @return 0 on success, negative errno on failure.
 */
int orion_driver_start(orion_driver_t *drv);

/**
 * @brief Stop the driver — halts the parser thread and UART interrupts.
 */
int orion_driver_stop(orion_driver_t *drv);

/**
 * @brief Get a snapshot of the latest navigation solution.
 *
 * Thread-safe. Non-blocking. Copies the latest decoded PVT into `out`.
 * Check `out->valid` to know if a fix has been obtained.
 *
 * @param drv  Driver instance.
 * @param out  Destination for the navigation data copy.
 *
 * @return 0 on success (data copied), -EAGAIN if no data available yet.
 */
int orion_driver_get_nav(orion_driver_t *drv, orion_nav_data_t *out);

/**
 * @brief Get latitude in degrees (double). Returns 0.0 if no fix.
 */
double orion_driver_get_latitude(orion_driver_t *drv);

/**
 * @brief Get longitude in degrees (double). Returns 0.0 if no fix.
 */
double orion_driver_get_longitude(orion_driver_t *drv);

/**
 * @brief Get altitude above mean sea level in meters. Returns 0.0 if no fix.
 */
double orion_driver_get_altitude_msl(orion_driver_t *drv);

/**
 * @brief Register a callback for real-time navigation data updates.
 *
 * The callback fires from the parser thread context every time a new
 * 0xA8 Navigation Data message is decoded.
 *
 * @param drv  Driver instance.
 * @param cb   Callback function.
 * @param user User context pointer (passed to callback).
 *
 * @return 0 on success, -ENOMEM if max callbacks reached.
 */
int orion_driver_register_nav_callback(orion_driver_t *drv,
                                       orion_nav_callback_t cb,
                                       void *user);

/**
 * @brief Register a catch-all callback for every decoded message.
 *
 * Useful for logging, debugging, or building higher-level abstractions.
 * Only one catch-all is supported; subsequent calls overwrite previous.
 *
 * @param drv  Driver instance.
 * @param cb   Callback function (or NULL to clear).
 * @param user User context pointer.
 */
void orion_driver_register_msg_callback(orion_driver_t *drv,
                                        orion_msg_callback_t cb,
                                        void *user);

/**
 * @brief Send a raw command and wait for ACK/NACK.
 *
 * Thread-safe. Blocks up to ORION_CMD_TIMEOUT_MS for a response.
 *
 * @param drv     Driver instance.
 * @param cmd_buf Pre-built command frame bytes.
 * @param cmd_len Number of bytes in cmd_buf.
 *
 * @return 1 on ACK, 0 on NACK, -ETIMEDOUT on timeout, negative errno on error.
 */
int orion_driver_send_command(orion_driver_t *drv,
                              const uint8_t *cmd_buf, size_t cmd_len);

/**
 * @brief Query the software version and wait for the response.
 *
 * @param drv Driver instance.
 * @param out Destination for decoded SW version.
 *
 * @return 0 on success, negative errno on failure.
 */
int orion_driver_query_sw_version(orion_driver_t *drv, orion_sw_version_t *out);

/**
 * @brief Reconfigure the update rate at runtime.
 *
 * @param drv     Driver instance.
 * @param rate_hz New update rate in Hz (1, 2, 4, 5, 8, 10, 20).
 *
 * @return 1 on ACK, 0 on NACK, negative errno on failure.
 */
int orion_driver_set_update_rate(orion_driver_t *drv, uint8_t rate_hz);

/**
 * @brief Get the fix mode of the latest navigation solution.
 * @return ORION_FIX_NONE/2D/3D/3D_DGPS, or ORION_FIX_NONE if no data.
 */
uint8_t orion_driver_get_fix_mode(orion_driver_t *drv);

/**
 * @brief Check if the receiver currently has a valid 3D fix.
 */
bool orion_driver_has_fix(orion_driver_t *drv);

/**
 * @brief Get driver statistics.
 */
void orion_driver_get_stats(orion_driver_t *drv,
                            uint32_t *rx_bytes,
                            uint32_t *tx_bytes,
                            uint32_t *nav_count,
                            uint32_t *frames_ok,
                            uint32_t *frames_err);

#ifdef __cplusplus
}
#endif

#endif /* ORION_B16_DRIVER_H */
