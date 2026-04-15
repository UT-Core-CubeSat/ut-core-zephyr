/**
 * @file gps_simulator.c
 * @brief Orion B16 GNSS simulator for ZephyrRTOS.
 *
 * Simulates the Orion B16 receiver over a UART loopback (or second UART).
 * This is invaluable for:
 *   - Testing the parser/driver without hardware
 *   - HIL (hardware-in-the-loop) testing in the flat-sat
 *   - CI/CD pipeline validation
 *
 * The simulator:
 *   1. Listens for incoming SkyTraq binary commands on a UART
 *   2. Responds with appropriate ACK/NACK frames
 *   3. Periodically transmits simulated Navigation Data (0xA8) messages
 *      with configurable position/velocity that drifts over time
 *
 * To use with the driver, connect two UARTs back-to-back (TX↔RX),
 * or use Zephyr's native_posix UART pipe for pure-software testing.
 *
 * Build: Include this file in your test application's CMakeLists.txt
 *        (do NOT include in the flight build).
 */

#include "orion_b16_protocol.h"
#include "orion_b16_commands.h"
#include "orion_b16_parser.h"
#include <string.h>
#include <math.h>

#ifdef __ZEPHYR__
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(orion_sim, CONFIG_LOG_DEFAULT_LEVEL);
#else
/* Standalone / host build stubs for testing outside Zephyr */
#include <stdio.h>
#define LOG_INF(...)  do { printf("[SIM] "); printf(__VA_ARGS__); printf("\n"); } while(0)
#define LOG_DBG(...)  do { printf("[SIM-D] "); printf(__VA_ARGS__); printf("\n"); } while(0)
#define LOG_WRN(...)  do { printf("[SIM-W] "); printf(__VA_ARGS__); printf("\n"); } while(0)
#endif


/* ──────────────── Simulator State ───────────────────────────────── */

/**
 * @brief Simulated satellite state.
 */
typedef struct {
    /* Simulated position (1e-7 degrees, cm) */
    int32_t  latitude_1e7;
    int32_t  longitude_1e7;
    uint32_t altitude_cm;

    /* Simulated velocity (cm/s) */
    int32_t  vel_north_cms;
    int32_t  vel_east_cms;
    int32_t  vel_down_cms;

    /* ECEF (will be computed from geodetic) */
    int32_t  ecef_x_cm;
    int32_t  ecef_y_cm;
    int32_t  ecef_z_cm;
    int32_t  ecef_vx_cms;
    int32_t  ecef_vy_cms;
    int32_t  ecef_vz_cms;

    /* GPS Time */
    uint16_t gnss_week;
    uint32_t tow_cs;            /* Time-of-week in centiseconds */

    /* Receiver state */
    uint8_t  fix_mode;          /* 0=none, 1=2D, 2=3D, 3=3D+DGPS */
    uint8_t  sv_count;
    uint8_t  update_rate_hz;
    uint8_t  msg_type;          /* 1=NMEA, 2=Binary */
    uint8_t  power_mode;        /* 0=Normal, 1=PowerSave */
    uint8_t  talker_id;         /* 0=GP, 1=GN, 2=Auto */

    /* Simulation control */
    uint32_t tick_count;
    bool     running;
} orion_sim_state_t;


/* ──────────────── Default Initial State (ISS-like orbit) ────────── */

static void orion_sim_init_state(orion_sim_state_t *sim)
{
    memset(sim, 0, sizeof(*sim));

    /* Start position: roughly over Texas (lat 30.2672, lon -97.7431) */
    sim->latitude_1e7  =  302672000;    /* 30.2672 deg × 1e7 */
    sim->longitude_1e7 = -977431000;    /* -97.7431 deg × 1e7 */
    sim->altitude_cm   = 40800000;      /* 408 km (ISS altitude) in cm */

    /* ISS-like velocity ~7.66 km/s, mostly eastward */
    sim->vel_north_cms =  10000;        /* 100 m/s north */
    sim->vel_east_cms  = 760000;        /* 7600 m/s east */
    sim->vel_down_cms  =  0;

    /* Approximate ECEF for the above geodetic position */
    sim->ecef_x_cm  = -74186900;        /* approximate */
    sim->ecef_y_cm  = -515389700;
    sim->ecef_z_cm  =  319882500;
    sim->ecef_vx_cms =  760000;
    sim->ecef_vy_cms =  10000;
    sim->ecef_vz_cms =  0;

    /* GPS time: arbitrary starting point */
    sim->gnss_week = 2350;
    sim->tow_cs    = 30000000;          /* ~300000 seconds into the week */

    /* Receiver defaults */
    sim->fix_mode       = ORION_FIX_3D;
    sim->sv_count       = 12;
    sim->update_rate_hz = 1;
    sim->msg_type       = ORION_MSG_TYPE_BINARY;
    sim->power_mode     = ORION_POWER_NORMAL;
    sim->talker_id      = 0x01;         /* GN */

    sim->running = true;
}


/* ──────────────── Orbit Propagation (simplified) ────────────────── */

/**
 * @brief Advance the simulated position by one tick.
 *
 * This is a *very* simplified propagator that drifts the lat/lon based
 * on the velocity. Good enough for protocol testing. For real orbit
 * simulation, use the ADCSCore two-body propagator.
 */
static void orion_sim_propagate(orion_sim_state_t *sim)
{
    /* dt = 1 / update_rate_hz (in seconds) */
    if (sim->update_rate_hz == 0) return;
    double dt = 1.0 / (double)sim->update_rate_hz;

    /* Approximate: 1 degree latitude ≈ 111,320 m
     *              1 degree longitude ≈ 111,320 × cos(lat) m
     * We work in 1e-7 deg and cm/s. */
    double lat_rad = (double)sim->latitude_1e7 * 1e-7 * 3.14159265358979 / 180.0;
    double cos_lat = cos(lat_rad);
    if (cos_lat < 0.01) cos_lat = 0.01;  /* clamp near poles */

    /* cm/s → 1e-7 deg/s : factor = 1e7 / (111320 × 100) = 1e7 / 11132000 ≈ 0.8983 */
    double factor_lat = 0.8983;
    double factor_lon = 0.8983 / cos_lat;

    sim->latitude_1e7  += (int32_t)((double)sim->vel_north_cms * factor_lat * dt);
    sim->longitude_1e7 += (int32_t)((double)sim->vel_east_cms  * factor_lon * dt);

    /* Wrap longitude to ±180° */
    if (sim->longitude_1e7 > 1800000000)   sim->longitude_1e7 -= 3600000000;
    if (sim->longitude_1e7 < -1800000000)  sim->longitude_1e7 += 3600000000;

    /* Wrap latitude (simple clamp) */
    if (sim->latitude_1e7 >  900000000)  sim->latitude_1e7 =  900000000;
    if (sim->latitude_1e7 < -900000000)  sim->latitude_1e7 = -900000000;

    /* Advance ECEF proportionally (rough) */
    sim->ecef_x_cm += (int32_t)((double)sim->ecef_vx_cms * dt);
    sim->ecef_y_cm += (int32_t)((double)sim->ecef_vy_cms * dt);
    sim->ecef_z_cm += (int32_t)((double)sim->ecef_vz_cms * dt);

    /* Advance GPS time */
    uint32_t dt_cs = (uint32_t)(dt * 100.0);
    sim->tow_cs += dt_cs;
    if (sim->tow_cs >= 60480000) {  /* 604800 seconds/week × 100 */
        sim->tow_cs -= 60480000;
        sim->gnss_week++;
    }

    /* Vary SV count slightly for realism */
    sim->tick_count++;
    sim->sv_count = 10 + (sim->tick_count % 5);

    /* Occasionally simulate brief fix loss */
    if (sim->tick_count % 120 == 0) {
        sim->fix_mode = ORION_FIX_NONE;
        sim->sv_count = 0;
    } else if (sim->tick_count % 120 == 1) {
        sim->fix_mode = ORION_FIX_2D;
        sim->sv_count = 3;
    } else {
        sim->fix_mode = ORION_FIX_3D;
    }
}


/* ──────────────── Frame Builders (Simulator Responses) ──────────── */

/**
 * @brief Build a Navigation Data Message (0xA8) frame from simulator state.
 * @return Number of bytes written into buf, or -1 on overflow.
 */
static int orion_sim_build_nav_data(const orion_sim_state_t *sim,
                                     uint8_t *buf, size_t buf_size)
{
    /* Body = 58 bytes (as specified in orion_b16_messages.h) */
    uint8_t body[ORION_NAV_DATA_BODY_LEN];
    memset(body, 0, sizeof(body));

    body[0] = sim->fix_mode;
    body[1] = sim->sv_count;
    orion_write_be16(&body[2],  sim->gnss_week);
    orion_write_be32(&body[4],  sim->tow_cs);
    orion_write_be32(&body[8],  (uint32_t)sim->latitude_1e7);
    orion_write_be32(&body[12], (uint32_t)sim->longitude_1e7);
    orion_write_be32(&body[16], sim->altitude_cm);
    orion_write_be32(&body[20], sim->altitude_cm);  /* MSL ≈ ellipsoid for sim */

    /* DOP values (×100) — reasonable for 10+ SVs */
    orion_write_be16(&body[24], 150);   /* GDOP = 1.50 */
    orion_write_be16(&body[26], 120);   /* PDOP = 1.20 */
    orion_write_be16(&body[28], 80);    /* HDOP = 0.80 */
    orion_write_be16(&body[30], 90);    /* VDOP = 0.90 */
    orion_write_be16(&body[32], 70);    /* TDOP = 0.70 */

    /* ECEF position */
    orion_write_be32(&body[34], (uint32_t)sim->ecef_x_cm);
    orion_write_be32(&body[38], (uint32_t)sim->ecef_y_cm);
    orion_write_be32(&body[42], (uint32_t)sim->ecef_z_cm);

    /* ECEF velocity */
    orion_write_be32(&body[46], (uint32_t)sim->ecef_vx_cms);
    orion_write_be32(&body[50], (uint32_t)sim->ecef_vy_cms);
    orion_write_be32(&body[54], (uint32_t)sim->ecef_vz_cms);

    return orion_cmd_build_raw(buf, buf_size, ORION_RSP_NAV_DATA,
                               body, sizeof(body));
}

/**
 * @brief Build an ACK frame for a given command message ID.
 */
static int orion_sim_build_ack(uint8_t cmd_id, uint8_t *buf, size_t buf_size)
{
    uint8_t body[] = { cmd_id };
    return orion_cmd_build_raw(buf, buf_size, ORION_RSP_ACK, body, sizeof(body));
}

/**
 * @brief Build a NACK frame.
 */
static int orion_sim_build_nack(uint8_t cmd_id, uint8_t *buf, size_t buf_size)
{
    uint8_t body[] = { cmd_id };
    return orion_cmd_build_raw(buf, buf_size, ORION_RSP_NACK, body, sizeof(body));
}

/**
 * @brief Build a Software Version response (0x80).
 */
static int orion_sim_build_sw_version(uint8_t *buf, size_t buf_size)
{
    uint8_t body[ORION_SW_VERSION_BODY_LEN];
    memset(body, 0, sizeof(body));
    body[0] = 0x00;  /* sw_type = system */
    /* Kernel version: "SIM1" */
    body[1] = 'S'; body[2] = 'I'; body[3] = 'M'; body[4] = '1';
    /* ODM version: "V001" */
    body[5] = 'V'; body[6] = '0'; body[7] = '0'; body[8] = '1';
    /* Revision: "2025" */
    body[9] = '2'; body[10] = '0'; body[11] = '2'; body[12] = '5';
    return orion_cmd_build_raw(buf, buf_size, ORION_RSP_SW_VERSION,
                               body, sizeof(body));
}

/**
 * @brief Build an Update Rate response (0x86).
 */
static int orion_sim_build_update_rate(uint8_t rate, uint8_t *buf, size_t buf_size)
{
    uint8_t body[] = { rate };
    return orion_cmd_build_raw(buf, buf_size, ORION_RSP_UPDATE_RATE,
                               body, sizeof(body));
}

/**
 * @brief Build a Message Type response (0x8C).
 */
static int orion_sim_build_msg_type(uint8_t type, uint8_t *buf, size_t buf_size)
{
    uint8_t body[] = { type };
    return orion_cmd_build_raw(buf, buf_size, ORION_RSP_MSG_TYPE,
                               body, sizeof(body));
}

/**
 * @brief Build a Power Mode response (0xB9).
 */
static int orion_sim_build_power_mode(uint8_t mode, uint8_t *buf, size_t buf_size)
{
    uint8_t body[] = { mode };
    return orion_cmd_build_raw(buf, buf_size, ORION_RSP_POWER_MODE,
                               body, sizeof(body));
}


/* ──────────────── Command Handler ───────────────────────────────── */

/**
 * @brief Handle an incoming command from the host and generate a response.
 *
 * @param sim      Simulator state (may be modified by config commands).
 * @param frame    The decoded incoming command frame.
 * @param resp_buf Output buffer for the response frame(s).
 * @param resp_size Size of the response buffer.
 *
 * @return Number of response bytes to send, or 0 if no response needed.
 */
static int orion_sim_handle_command(orion_sim_state_t *sim,
                                    const orion_frame_t *frame,
                                    uint8_t *resp_buf, size_t resp_size)
{
    int total = 0;
    uint8_t *ptr = resp_buf;
    size_t remaining = resp_size;
    int len;

    switch (frame->msg_id) {

    case ORION_CMD_QUERY_SW_VERSION:
        len = orion_sim_build_sw_version(ptr, remaining);
        if (len > 0) { total += len; ptr += len; remaining -= len; }
        break;

    case ORION_CMD_CFG_MSG_TYPE:
        if (frame->body_len >= 1) {
            sim->msg_type = frame->body[0];
            LOG_INF("SIM: msg_type set to %s",
                    sim->msg_type == ORION_MSG_TYPE_BINARY ? "Binary" : "NMEA");
        }
        len = orion_sim_build_ack(frame->msg_id, ptr, remaining);
        if (len > 0) { total += len; ptr += len; remaining -= len; }
        break;

    case ORION_CMD_CFG_UPDATE_RATE:
        if (frame->body_len >= 1) {
            sim->update_rate_hz = frame->body[0];
            LOG_INF("SIM: update rate set to %u Hz", sim->update_rate_hz);
        }
        len = orion_sim_build_ack(frame->msg_id, ptr, remaining);
        if (len > 0) { total += len; ptr += len; remaining -= len; }
        break;

    case ORION_CMD_QUERY_UPDATE_RATE:
        len = orion_sim_build_update_rate(sim->update_rate_hz, ptr, remaining);
        if (len > 0) { total += len; ptr += len; remaining -= len; }
        break;

    case ORION_CMD_QUERY_MSG_TYPE:
        len = orion_sim_build_msg_type(sim->msg_type, ptr, remaining);
        if (len > 0) { total += len; ptr += len; remaining -= len; }
        break;

    case ORION_CMD_QUERY_POWER_MODE:
        len = orion_sim_build_power_mode(sim->power_mode, ptr, remaining);
        if (len > 0) { total += len; ptr += len; remaining -= len; }
        break;

    case ORION_CMD_CFG_POWER_MODE:
        if (frame->body_len >= 1) {
            sim->power_mode = frame->body[0];
            LOG_INF("SIM: power mode set to %s",
                    sim->power_mode == ORION_POWER_NORMAL ? "Normal" : "PowerSave");
        }
        len = orion_sim_build_ack(frame->msg_id, ptr, remaining);
        if (len > 0) { total += len; ptr += len; remaining -= len; }
        break;

    case ORION_CMD_CFG_ELEV_CNR_MASK:
    case ORION_CMD_CFG_DOP_MASK:
    case ORION_CMD_CFG_GNSS_CONSTELLATION:
    case ORION_CMD_CFG_NMEA_TALKER_ID:
    case ORION_CMD_CFG_NAV_DATA_MSG_INTERVAL:
    case ORION_CMD_CFG_DATUM:
    case ORION_CMD_CFG_1PPS_MODE:
        /* Accept all config commands with ACK */
        len = orion_sim_build_ack(frame->msg_id, ptr, remaining);
        if (len > 0) { total += len; ptr += len; remaining -= len; }
        break;

    case ORION_CMD_RESTART:
        LOG_INF("SIM: Restart command received (mode=%u)",
                frame->body_len > 0 ? frame->body[0] : 0);
        len = orion_sim_build_ack(frame->msg_id, ptr, remaining);
        if (len > 0) { total += len; ptr += len; remaining -= len; }
        break;

    default:
        LOG_WRN("SIM: Unknown command 0x%02x — sending NACK", frame->msg_id);
        len = orion_sim_build_nack(frame->msg_id, ptr, remaining);
        if (len > 0) { total += len; ptr += len; remaining -= len; }
        break;
    }

    return total;
}


/* ──────────────── Public Simulator API ──────────────────────────── */

#ifdef __ZEPHYR__

#define SIM_STACK_SIZE  2048
#define SIM_THREAD_PRIO 6
#define SIM_RX_BUF_SIZE 512

typedef struct {
    orion_sim_state_t  state;
    const struct device *uart_dev;
    orion_parser_t      parser;
    uint8_t             rx_ring_data[SIM_RX_BUF_SIZE];
    struct ring_buf     rx_rb;
    struct k_thread     thread;
    K_KERNEL_STACK_MEMBER(stack, SIM_STACK_SIZE);
} orion_sim_ctx_t;

/* Forward decl */
static void orion_sim_uart_isr(const struct device *dev, void *user_data);
static void orion_sim_thread_fn(void *p1, void *p2, void *p3);

/**
 * @brief Start the GNSS simulator on the given UART.
 *
 * @param ctx       Caller-allocated simulator context.
 * @param uart_name Zephyr UART device name (the "receiver side" of the link).
 *
 * @return 0 on success, negative errno on failure.
 */
int orion_sim_start(orion_sim_ctx_t *ctx, const char *uart_name)
{
    memset(ctx, 0, sizeof(*ctx));
    orion_sim_init_state(&ctx->state);
    orion_parser_init(&ctx->parser);
    ring_buf_init(&ctx->rx_rb, sizeof(ctx->rx_ring_data), ctx->rx_ring_data);

    ctx->uart_dev = device_get_binding(uart_name);
    if (!ctx->uart_dev || !device_is_ready(ctx->uart_dev)) {
        LOG_ERR("SIM: UART device '%s' not found/ready", uart_name);
        return -ENODEV;
    }

    uart_irq_callback_user_data_set(ctx->uart_dev, orion_sim_uart_isr, ctx);
    uart_irq_rx_enable(ctx->uart_dev);

    k_thread_create(&ctx->thread, ctx->stack, K_KERNEL_STACK_SIZEOF(ctx->stack),
                    orion_sim_thread_fn, ctx, NULL, NULL,
                    SIM_THREAD_PRIO, 0, K_NO_WAIT);
    k_thread_name_set(&ctx->thread, "orion_sim");

    LOG_INF("Orion B16 simulator started on %s", uart_name);
    return 0;
}

static void orion_sim_uart_isr(const struct device *dev, void *user_data)
{
    orion_sim_ctx_t *ctx = (orion_sim_ctx_t *)user_data;

    if (!uart_irq_update(dev)) return;

    while (uart_irq_rx_ready(dev)) {
        uint8_t buf[64];
        int len = uart_fifo_read(dev, buf, sizeof(buf));
        if (len > 0) {
            ring_buf_put(&ctx->rx_rb, buf, len);
        }
    }
}

static void orion_sim_send(orion_sim_ctx_t *ctx, const uint8_t *data, int len)
{
    for (int i = 0; i < len; i++) {
        uart_poll_out(ctx->uart_dev, data[i]);
    }
}

static void orion_sim_thread_fn(void *p1, void *p2, void *p3)
{
    orion_sim_ctx_t *ctx = (orion_sim_ctx_t *)p1;
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    uint32_t last_nav_ms = 0;
    orion_frame_t frame;
    uint8_t resp_buf[256];

    while (ctx->state.running) {
        /* Process incoming commands */
        uint8_t byte;
        while (ring_buf_get(&ctx->rx_rb, &byte, 1) == 1) {
            orion_parse_result_t res = orion_parser_feed_byte(&ctx->parser, byte, &frame);
            if (res == ORION_PARSE_FRAME_READY) {
                int resp_len = orion_sim_handle_command(&ctx->state, &frame,
                                                        resp_buf, sizeof(resp_buf));
                if (resp_len > 0) {
                    orion_sim_send(ctx, resp_buf, resp_len);
                }
            }
        }

        /* Periodically send Navigation Data messages */
        uint32_t now_ms = k_uptime_get_32();
        uint32_t interval_ms = (ctx->state.update_rate_hz > 0)
                               ? (1000 / ctx->state.update_rate_hz) : 1000;

        if ((now_ms - last_nav_ms) >= interval_ms) {
            last_nav_ms = now_ms;

            /* Propagate the simulated orbit */
            orion_sim_propagate(&ctx->state);

            /* Only send nav data if in binary mode */
            if (ctx->state.msg_type == ORION_MSG_TYPE_BINARY) {
                int nav_len = orion_sim_build_nav_data(&ctx->state,
                                                        resp_buf, sizeof(resp_buf));
                if (nav_len > 0) {
                    orion_sim_send(ctx, resp_buf, nav_len);
                }
            }
        }

        k_sleep(K_MSEC(1));
    }

    LOG_INF("Orion B16 simulator stopped");
}

#endif /* __ZEPHYR__ */


/* ──────────────── Standalone Test Helper (non-Zephyr) ───────────── */

#ifndef __ZEPHYR__

/**
 * @brief Generate a single Navigation Data frame into a buffer.
 *
 * Useful for host-side unit testing of the parser — build a known-good
 * frame and feed it byte-by-byte into the parser.
 *
 * @param buf      Output buffer.
 * @param buf_size Buffer size.
 * @param lat_1e7  Latitude in 1e-7 degrees.
 * @param lon_1e7  Longitude in 1e-7 degrees.
 * @param alt_cm   Altitude in cm.
 *
 * @return Frame length, or -1 on overflow.
 */
int orion_sim_generate_nav_frame(uint8_t *buf, size_t buf_size,
                                  int32_t lat_1e7, int32_t lon_1e7,
                                  uint32_t alt_cm)
{
    orion_sim_state_t sim;
    orion_sim_init_state(&sim);
    sim.latitude_1e7  = lat_1e7;
    sim.longitude_1e7 = lon_1e7;
    sim.altitude_cm   = alt_cm;
    sim.fix_mode      = ORION_FIX_3D;
    sim.sv_count      = 12;
    return orion_sim_build_nav_data(&sim, buf, buf_size);
}

/**
 * @brief Generate an ACK frame for testing.
 */
int orion_sim_generate_ack_frame(uint8_t *buf, size_t buf_size, uint8_t cmd_id)
{
    return orion_sim_build_ack(cmd_id, buf, buf_size);
}

/**
 * @brief Generate a NACK frame for testing.
 */
int orion_sim_generate_nack_frame(uint8_t *buf, size_t buf_size, uint8_t cmd_id)
{
    return orion_sim_build_nack(cmd_id, buf, buf_size);
}

#endif /* !__ZEPHYR__ */
