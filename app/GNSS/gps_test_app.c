/**
 * @file gps_test_app.c
 * @brief Test application demonstrating full Orion B16 GNSS driver usage.
 *
 * This is the "main()" of your Zephyr firmware — the entry point.
 *
 * WHAT IT DOES, step by step:
 *   1. Opens UART1 and sets up the Orion B16 binary protocol driver
 *   2. Tells the Orion to output binary navigation data at 1 Hz
 *   3. Registers a callback so we get notified on every new GPS fix
 *   4. Sits in a loop, printing the latest position every 5 seconds
 *
 * WHAT YOU'LL SEE on the serial console (UART0 / USB):
 *   [00:00:01.234] <inf> gnss_app: === Orion B16 GNSS Test Application ===
 *   [00:00:02.456] <inf> gnss_app: No valid fix yet.
 *   ... (until the Orion gets satellite lock) ...
 *   [00:00:32.789] <inf> gnss_app: FIX 3D | lat=30.267200 lon=-97.743100 alt=408.00m
 *
 * If you don't have the Orion B16 connected, you'll just see
 * "No valid fix yet" forever — that's expected and means the driver
 * is running correctly but has no data to parse.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

/* These headers are found via the include path set in CMakeLists.txt */
#include "orion_b16_driver.h"
#include "orion_b16_protocol.h"
#include "orion_b16_messages.h"

LOG_MODULE_REGISTER(gnss_app, CONFIG_LOG_DEFAULT_LEVEL);

/* Driver instance (static allocation — no heap) */
static orion_driver_t gnss_driver;

/* Forward declaration for config recipe */
extern int orion_config_cubesat_default(orion_driver_t *drv, uint8_t rate_hz, bool save);

/* ──────────────── Navigation Callback ───────────────────────────── */

static void on_nav_update(const orion_nav_data_t *nav, void *user)
{
    (void)user;

    if (!nav->valid) {
        LOG_WRN("NAV callback: invalid fix");
        return;
    }

    double lat = ORION_DEG_FROM_1E7(nav->latitude_1e7);
    double lon = ORION_DEG_FROM_1E7(nav->longitude_1e7);
    double alt = ORION_M_FROM_CM(nav->msl_alt_cm);

    const char *fix_str;
    switch (nav->fix_mode) {
        case ORION_FIX_2D:      fix_str = "2D";       break;
        case ORION_FIX_3D:      fix_str = "3D";       break;
        case ORION_FIX_3D_DGPS: fix_str = "3D+DGPS";  break;
        default:                fix_str = "NONE";      break;
    }

    /* NOTE: Zephyr LOG doesn't support %f on all platforms.
     * For production, use integer-scaled prints or printk with custom formatting.
     * This works on ESP32 with CONFIG_NEWLIB_LIBC=y. */
    LOG_INF("FIX %s | lat=%.6f lon=%.6f alt=%.2fm | SVs=%u",
            fix_str, lat, lon, alt, nav->sv_count);
}

/* ──────────────── Detailed Status Print ─────────────────────────── */

static void print_full_nav(const orion_nav_data_t *nav)
{
    if (!nav->valid) {
        LOG_INF("No valid fix yet.");
        return;
    }

    double lat   = ORION_DEG_FROM_1E7(nav->latitude_1e7);
    double lon   = ORION_DEG_FROM_1E7(nav->longitude_1e7);
    double alt   = ORION_M_FROM_CM(nav->msl_alt_cm);
    double ex    = ORION_M_FROM_CM(nav->ecef_x_cm);
    double ey    = ORION_M_FROM_CM(nav->ecef_y_cm);
    double ez    = ORION_M_FROM_CM(nav->ecef_z_cm);
    double evx   = ORION_MS_FROM_CMS(nav->ecef_vx_cms);
    double evy   = ORION_MS_FROM_CMS(nav->ecef_vy_cms);
    double evz   = ORION_MS_FROM_CMS(nav->ecef_vz_cms);
    double gdop  = ORION_DOP_FROM_100(nav->gdop);
    double pdop  = ORION_DOP_FROM_100(nav->pdop);
    double hdop  = ORION_DOP_FROM_100(nav->hdop);
    double vdop  = ORION_DOP_FROM_100(nav->vdop);
    double tdop  = ORION_DOP_FROM_100(nav->tdop);
    double tow_s = (double)nav->tow_cs * 0.01;

    LOG_INF("--- Navigation Data ---");
    LOG_INF("  Fix:  mode=%u  SVs=%u", nav->fix_mode, nav->sv_count);
    LOG_INF("  Pos:  lat=%.7f  lon=%.7f  alt=%.2f m", lat, lon, alt);
    LOG_INF("  ECEF: x=%.2f  y=%.2f  z=%.2f m", ex, ey, ez);
    LOG_INF("  Vel:  vx=%.2f  vy=%.2f  vz=%.2f m/s", evx, evy, evz);
    LOG_INF("  DOP:  G=%.2f P=%.2f H=%.2f V=%.2f T=%.2f",
            gdop, pdop, hdop, vdop, tdop);
    LOG_INF("  Time: week=%u  tow=%.2f s", nav->gnss_week, tow_s);
}

/* ──────────────── Main ──────────────────────────────────────────── */

int main(void)
{
    LOG_INF("=== Orion B16 GNSS Test Application ===");

    /* 1. Initialize driver */
    orion_driver_cfg_t cfg = ORION_DRIVER_CFG_DEFAULTS;
    /*
     * "UART_1" must match the Zephyr device label for your UART.
     * On ESP32, UART1 becomes "UART_1" thanks to our overlay.
     * On the STM32U5 flight board, you'd change this to "USART_2"
     * or whatever UART the Orion is wired to.
     */

    int rc = orion_driver_init(&gnss_driver, &cfg);
    if (rc != 0) {
        LOG_ERR("Driver init failed: %d", rc);
        return rc;
    }

    /* 2. Register navigation callback */
    orion_driver_register_nav_callback(&gnss_driver, on_nav_update, NULL);

    /* 3. Start the driver (spawns parser thread, configures receiver) */
    rc = orion_driver_start(&gnss_driver);
    if (rc != 0) {
        LOG_ERR("Driver start failed: %d", rc);
        return rc;
    }

    /* 4. Optionally apply full CubeSat configuration */
    orion_config_cubesat_default(&gnss_driver, 1, false);

    LOG_INF("Driver running. Waiting for fix...");

    /* 5. Main loop — periodically print detailed status */
    while (1) {
        k_sleep(K_SECONDS(5));

        orion_nav_data_t nav;
        orion_driver_get_nav(&gnss_driver, &nav);
        print_full_nav(&nav);

        /* Print stats */
        uint32_t rx_b, tx_b, nav_cnt, f_ok, f_err;
        orion_driver_get_stats(&gnss_driver, &rx_b, &tx_b, &nav_cnt, &f_ok, &f_err);
        LOG_INF("Stats: RX=%u TX=%u nav=%u frames_ok=%u frames_err=%u",
                rx_b, tx_b, nav_cnt, f_ok, f_err);
    }

    return 0;
}
