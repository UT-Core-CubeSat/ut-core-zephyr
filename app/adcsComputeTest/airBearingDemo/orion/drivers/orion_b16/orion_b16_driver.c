/**
 * @file orion_b16_driver.c
 * @brief ZephyrRTOS driver implementation for the Orion B16 GNSS receiver.
 *
 * HOW THIS WORKS (the big picture):
 *
 *   The STM32 has a UART (serial port) connected to the Orion B16 chip.
 *   Bytes stream in constantly from the Orion over that wire.
 *
 *   This file sets up:
 *     1. A UART interrupt — every time a byte arrives, it gets stuffed
 *        into a ring buffer (a circular queue in RAM).
 *     2. A background thread — continuously pulls bytes out of the ring
 *        buffer and feeds them one-by-one into the parser state machine.
 *     3. When the parser detects a complete, valid frame, it decodes the
 *        message and updates a "latest_nav" struct that any other code
 *        can read at any time.
 *
 *   For sending commands TO the Orion (e.g., "switch to binary mode"),
 *   we just write bytes out the same UART and wait for an ACK/NACK reply.
 */

#include "orion_b16_driver.h"

/* Only compile this file when building under Zephyr */
#ifdef __ZEPHYR__

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <errno.h>

LOG_MODULE_REGISTER(orion_b16, CONFIG_LOG_DEFAULT_LEVEL);


/* ──────────────── UART Interrupt Callback ────────────────────────── */

static void orion_uart_isr(const struct device *dev, void *user_data)
{
    orion_driver_t *drv = (orion_driver_t *)user_data;

    if (!uart_irq_update(dev)) {
        return;
    }

    while (uart_irq_rx_ready(dev)) {
        uint8_t buf[64];
        int len = uart_fifo_read(dev, buf, sizeof(buf));
        if (len > 0) {
            ring_buf_put(&drv->rx_rb, buf, len);
            drv->rx_bytes += len;
        }
    }
}


/* ──────────────── Parser Thread ─────────────────────────────────── */

static void orion_dispatch_frame(orion_driver_t *drv, const orion_frame_t *frame)
{
    orion_message_t msg;

    if (orion_decode_message(frame, &msg) != 0) {
        LOG_DBG("Unknown/malformed msg_id=0x%02x body_len=%u",
                frame->msg_id, frame->body_len);
        return;
    }

    /* Fire the catch-all callback */
    if (drv->msg_callback) {
        drv->msg_callback(&msg, drv->msg_callback_user);
    }

    switch (msg.msg_id) {

    case ORION_RSP_NAV_DATA:
        /* Update the latest navigation solution under lock */
        k_mutex_lock(&drv->nav_mutex, K_FOREVER);
        memcpy(&drv->latest_nav, &msg.nav_data, sizeof(orion_nav_data_t));
        k_mutex_unlock(&drv->nav_mutex);
        drv->nav_count++;

        LOG_DBG("NAV fix=%u sv=%u lat=%d lon=%d alt=%u",
                msg.nav_data.fix_mode, msg.nav_data.sv_count,
                msg.nav_data.latitude_1e7, msg.nav_data.longitude_1e7,
                msg.nav_data.msl_alt_cm);

        /* Fire registered nav callbacks */
        for (int i = 0; i < drv->nav_cb_count; i++) {
            if (drv->nav_callbacks[i].fn) {
                drv->nav_callbacks[i].fn(&msg.nav_data,
                                         drv->nav_callbacks[i].user);
            }
        }
        break;

    case ORION_RSP_ACK:
    case ORION_RSP_NACK:
        /* Pair with pending command */
        if (msg.ack.acked_msg_id == drv->pending_cmd_id) {
            drv->ack_received = msg.ack.is_ack;
            drv->ack_valid = true;
            k_sem_give(&drv->ack_sem);
        }
        LOG_INF("%s for cmd 0x%02x",
                msg.ack.is_ack ? "ACK" : "NACK",
                msg.ack.acked_msg_id);
        break;

    case ORION_RSP_SW_VERSION:
        LOG_INF("SW version: type=%u kernel=%02x%02x%02x%02x",
                msg.sw_version.sw_type,
                msg.sw_version.kernel_ver[0], msg.sw_version.kernel_ver[1],
                msg.sw_version.kernel_ver[2], msg.sw_version.kernel_ver[3]);
        break;

    case ORION_RSP_UPDATE_RATE:
        LOG_INF("Update rate: %u Hz", msg.update_rate.rate_hz);
        break;

    case ORION_RSP_MSG_TYPE:
        LOG_INF("Message type: %s",
                msg.msg_type.msg_type == ORION_MSG_TYPE_BINARY ? "Binary" : "NMEA");
        break;

    case ORION_RSP_POWER_MODE:
        LOG_INF("Power mode: %s",
                msg.power_mode.power_mode == ORION_POWER_NORMAL ? "Normal" : "PowerSave");
        break;

    default:
        LOG_DBG("Decoded msg_id=0x%02x", msg.msg_id);
        break;
    }
}

static void orion_parser_thread_fn(void *p1, void *p2, void *p3)
{
    orion_driver_t *drv = (orion_driver_t *)p1;

    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    LOG_INF("Orion B16 parser thread started");

    orion_frame_t frame;

    while (drv->running) {
        uint8_t byte;
        uint32_t got = ring_buf_get(&drv->rx_rb, &byte, 1);

        if (got == 0) {
            /* No data — sleep briefly and retry */
            k_sleep(K_MSEC(1));
            continue;
        }

        orion_parse_result_t res = orion_parser_feed_byte(&drv->parser, byte, &frame);

        if (res == ORION_PARSE_FRAME_READY) {
            orion_dispatch_frame(drv, &frame);
        } else if (res == ORION_PARSE_ERROR) {
            LOG_WRN("Frame parse error (total err=%u)", drv->parser.frames_err);
        }
    }

    LOG_INF("Orion B16 parser thread stopped");
}


/* ──────────────── UART TX Helper ────────────────────────────────── */

static int orion_uart_send(orion_driver_t *drv, const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        uart_poll_out(drv->uart_dev, data[i]);
    }
    drv->tx_bytes += len;
    return 0;
}


/* ──────────────── Public API Implementation ─────────────────────── */

int orion_driver_init(orion_driver_t *drv, const orion_driver_cfg_t *cfg)
{
    memset(drv, 0, sizeof(*drv));
    memcpy(&drv->cfg, cfg, sizeof(*cfg));

    /* Initialize parser state machine */
    orion_parser_init(&drv->parser);

    /* Get UART device */
    drv->uart_dev = device_get_binding(cfg->uart_dev_name);
    if (!drv->uart_dev) {
        LOG_ERR("UART device '%s' not found", cfg->uart_dev_name);
        return -ENODEV;
    }

    if (!device_is_ready(drv->uart_dev)) {
        LOG_ERR("UART device '%s' not ready", cfg->uart_dev_name);
        return -ENODEV;
    }

    /* Initialize Zephyr primitives */
    k_mutex_init(&drv->nav_mutex);
    k_mutex_init(&drv->cmd_mutex);
    k_sem_init(&drv->ack_sem, 0, 1);
    ring_buf_init(&drv->rx_rb, sizeof(drv->rx_ring_buf), drv->rx_ring_buf);

    LOG_INF("Orion B16 driver initialized (uart=%s baud=%u)",
            cfg->uart_dev_name, cfg->baud_rate);

    return 0;
}

int orion_driver_start(orion_driver_t *drv)
{
    if (drv->running) {
        return -EALREADY;
    }

    drv->running = true;

    /* Set up interrupt-driven UART RX */
    uart_irq_callback_user_data_set(drv->uart_dev, orion_uart_isr, drv);
    uart_irq_rx_enable(drv->uart_dev);

    /* Spawn parser thread */
    k_thread_create(&drv->parser_thread,
                    drv->parser_stack,
                    K_KERNEL_STACK_SIZEOF(drv->parser_stack),
                    orion_parser_thread_fn,
                    drv, NULL, NULL,
                    ORION_PARSER_THREAD_PRIO, 0, K_NO_WAIT);
    k_thread_name_set(&drv->parser_thread, "orion_parser");

    LOG_INF("Orion B16 driver started — configuring receiver...");

    /* Configure the receiver:
     * 1. Switch to binary message output
     * 2. Set the desired update rate
     */
    uint8_t cmd_buf[32];
    int len;

    /* Switch to Binary mode */
    len = orion_cmd_cfg_msg_type(cmd_buf, sizeof(cmd_buf),
                                 drv->cfg.msg_type, drv->cfg.save_attr);
    if (len > 0) {
        int rc = orion_driver_send_command(drv, cmd_buf, len);
        if (rc == 1) {
            LOG_INF("Binary mode configured (ACK)");
        } else {
            LOG_WRN("Binary mode config: rc=%d", rc);
        }
    }

    /* Set update rate */
    len = orion_cmd_cfg_update_rate(cmd_buf, sizeof(cmd_buf),
                                    drv->cfg.update_rate_hz, drv->cfg.save_attr);
    if (len > 0) {
        int rc = orion_driver_send_command(drv, cmd_buf, len);
        if (rc == 1) {
            LOG_INF("Update rate %u Hz configured (ACK)", drv->cfg.update_rate_hz);
        } else {
            LOG_WRN("Update rate config: rc=%d", rc);
        }
    }

    return 0;
}

int orion_driver_stop(orion_driver_t *drv)
{
    if (!drv->running) {
        return -EALREADY;
    }

    drv->running = false;

    /* Disable UART interrupts */
    uart_irq_rx_disable(drv->uart_dev);

    /* The parser thread will exit on its next loop iteration */
    k_thread_join(&drv->parser_thread, K_MSEC(500));

    LOG_INF("Orion B16 driver stopped");
    return 0;
}

int orion_driver_get_nav(orion_driver_t *drv, orion_nav_data_t *out)
{
    k_mutex_lock(&drv->nav_mutex, K_FOREVER);
    memcpy(out, &drv->latest_nav, sizeof(orion_nav_data_t));
    k_mutex_unlock(&drv->nav_mutex);

    return out->valid ? 0 : -EAGAIN;
}

double orion_driver_get_latitude(orion_driver_t *drv)
{
    orion_nav_data_t nav;
    if (orion_driver_get_nav(drv, &nav) != 0) return 0.0;
    return ORION_DEG_FROM_1E7(nav.latitude_1e7);
}

double orion_driver_get_longitude(orion_driver_t *drv)
{
    orion_nav_data_t nav;
    if (orion_driver_get_nav(drv, &nav) != 0) return 0.0;
    return ORION_DEG_FROM_1E7(nav.longitude_1e7);
}

double orion_driver_get_altitude_msl(orion_driver_t *drv)
{
    orion_nav_data_t nav;
    if (orion_driver_get_nav(drv, &nav) != 0) return 0.0;
    return ORION_M_FROM_CM(nav.msl_alt_cm);
}

uint8_t orion_driver_get_fix_mode(orion_driver_t *drv)
{
    orion_nav_data_t nav;
    if (orion_driver_get_nav(drv, &nav) != 0) return ORION_FIX_NONE;
    return nav.fix_mode;
}

bool orion_driver_has_fix(orion_driver_t *drv)
{
    uint8_t fm = orion_driver_get_fix_mode(drv);
    return (fm == ORION_FIX_3D || fm == ORION_FIX_3D_DGPS);
}

int orion_driver_register_nav_callback(orion_driver_t *drv,
                                       orion_nav_callback_t cb,
                                       void *user)
{
    if (drv->nav_cb_count >= ORION_MAX_NAV_CALLBACKS) {
        return -ENOMEM;
    }
    drv->nav_callbacks[drv->nav_cb_count].fn   = cb;
    drv->nav_callbacks[drv->nav_cb_count].user = user;
    drv->nav_cb_count++;
    return 0;
}

void orion_driver_register_msg_callback(orion_driver_t *drv,
                                        orion_msg_callback_t cb,
                                        void *user)
{
    drv->msg_callback      = cb;
    drv->msg_callback_user = user;
}

int orion_driver_send_command(orion_driver_t *drv,
                              const uint8_t *cmd_buf, size_t cmd_len)
{
    k_mutex_lock(&drv->cmd_mutex, K_FOREVER);

    /* Extract msg_id from the command frame (byte index 4 = first payload byte) */
    if (cmd_len < 5) {
        k_mutex_unlock(&drv->cmd_mutex);
        return -EINVAL;
    }
    drv->pending_cmd_id = cmd_buf[4];
    drv->ack_valid = false;
    drv->ack_received = false;

    /* Reset the ACK semaphore */
    k_sem_reset(&drv->ack_sem);

    /* Transmit the command */
    int rc = orion_uart_send(drv, cmd_buf, cmd_len);
    if (rc != 0) {
        k_mutex_unlock(&drv->cmd_mutex);
        return rc;
    }

    /* Wait for ACK/NACK */
    rc = k_sem_take(&drv->ack_sem, K_MSEC(ORION_CMD_TIMEOUT_MS));
    int result;
    if (rc == -EAGAIN) {
        LOG_WRN("Timeout waiting for ACK (cmd=0x%02x)", drv->pending_cmd_id);
        result = -ETIMEDOUT;
    } else if (drv->ack_valid) {
        result = drv->ack_received ? 1 : 0;
    } else {
        result = -EIO;
    }

    drv->pending_cmd_id = 0;
    k_mutex_unlock(&drv->cmd_mutex);
    return result;
}

int orion_driver_query_sw_version(orion_driver_t *drv, orion_sw_version_t *out)
{
    /* TODO: To properly return the decoded version, we'd need a temporary
     * callback or a response mailbox.  For now, we just send and check ACK. */
    uint8_t buf[16];
    int len = orion_cmd_query_sw_version(buf, sizeof(buf), 0x00);
    if (len <= 0) return -EINVAL;

    /* The response is SW Version (0x80), not ACK.
     * The generic send_command waits for ACK, which won't come for queries.
     * So just send and let the parser thread log/dispatch it. */
    k_mutex_lock(&drv->cmd_mutex, K_FOREVER);
    orion_uart_send(drv, buf, len);
    k_mutex_unlock(&drv->cmd_mutex);

    /* Give the receiver time to respond */
    k_sleep(K_MSEC(200));

    /* out is populated by the msg_callback if the user registered one */
    memset(out, 0, sizeof(*out));
    return 0;
}

int orion_driver_set_update_rate(orion_driver_t *drv, uint8_t rate_hz)
{
    uint8_t buf[16];
    int len = orion_cmd_cfg_update_rate(buf, sizeof(buf),
                                        rate_hz, drv->cfg.save_attr);
    if (len <= 0) return -EINVAL;

    int rc = orion_driver_send_command(drv, buf, len);
    if (rc == 1) {
        drv->cfg.update_rate_hz = rate_hz;
    }
    return rc;
}

void orion_driver_get_stats(orion_driver_t *drv,
                            uint32_t *rx_bytes,
                            uint32_t *tx_bytes,
                            uint32_t *nav_count,
                            uint32_t *frames_ok,
                            uint32_t *frames_err)
{
    if (rx_bytes)   *rx_bytes   = drv->rx_bytes;
    if (tx_bytes)   *tx_bytes   = drv->tx_bytes;
    if (nav_count)  *nav_count  = drv->nav_count;
    if (frames_ok)  *frames_ok  = drv->parser.frames_ok;
    if (frames_err) *frames_err = drv->parser.frames_err;
}

#endif /* __ZEPHYR__ */
