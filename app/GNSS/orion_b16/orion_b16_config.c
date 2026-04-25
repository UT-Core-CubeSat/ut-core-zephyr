/**
 * @file orion_b16_config.c
 * @brief Higher-level configuration recipes for the Orion B16.
 *
 * Provides "one-call" convenience functions that string together multiple
 * commands to put the receiver into a desired operational state. These are
 * built on top of the driver's send_command API and are intended for use
 * during CubeSat initialization.
 */

#ifdef __ZEPHYR__

#include "orion_b16_driver.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(orion_b16_cfg, CONFIG_LOG_DEFAULT_LEVEL);

/**
 * @brief Full "CubeSat-ready" configuration sequence.
 *
 * This sends a series of commands to the Orion B16 to put it into the
 * recommended operational state for a CubeSat mission:
 *
 *   1. Switch to binary output mode
 *   2. Set position update rate (default 1 Hz)
 *   3. Enable GPS + GLONASS constellations
 *   4. Set normal power mode
 *   5. Set reasonable DOP and elevation masks
 *
 * @param drv      Initialized and started driver instance.
 * @param rate_hz  Desired update rate in Hz (1-10 recommended for CubeSat).
 * @param save     true = save to flash (persists across reboots).
 *
 * @return 0 if all commands succeeded, negative on first failure.
 */
int orion_config_cubesat_default(orion_driver_t *drv, uint8_t rate_hz, bool save)
{
    uint8_t attr = save ? ORION_ATTR_SRAM_FLASH : ORION_ATTR_SRAM_ONLY;
    uint8_t buf[32];
    int len, rc;

    LOG_INF("Applying CubeSat default configuration...");

    /* 1. Binary output mode */
    len = orion_cmd_cfg_msg_type(buf, sizeof(buf), ORION_MSG_TYPE_BINARY, attr);
    if (len > 0) {
        rc = orion_driver_send_command(drv, buf, len);
        if (rc != 1) {
            LOG_ERR("Failed to set binary mode (rc=%d)", rc);
            return -EIO;
        }
        LOG_INF("  Binary mode: OK");
    }

    /* 2. Update rate */
    len = orion_cmd_cfg_update_rate(buf, sizeof(buf), rate_hz, attr);
    if (len > 0) {
        rc = orion_driver_send_command(drv, buf, len);
        if (rc != 1) {
            LOG_ERR("Failed to set update rate %u Hz (rc=%d)", rate_hz, rc);
            return -EIO;
        }
        LOG_INF("  Update rate %u Hz: OK", rate_hz);
    }

    /* 3. Enable GPS + GLONASS (Galileo/BeiDou optional based on mission) */
    len = orion_cmd_cfg_constellation(buf, sizeof(buf),
                                      1, /* GPS */
                                      1, /* GLONASS */
                                      0, /* Galileo */
                                      0, /* BeiDou */
                                      attr);
    if (len > 0) {
        rc = orion_driver_send_command(drv, buf, len);
        if (rc != 1) {
            LOG_WRN("Constellation config: rc=%d (may not be supported)", rc);
        } else {
            LOG_INF("  Constellation GPS+GLONASS: OK");
        }
    }

    /* 4. Normal power mode */
    len = orion_cmd_cfg_power_mode(buf, sizeof(buf), ORION_POWER_NORMAL, attr);
    if (len > 0) {
        rc = orion_driver_send_command(drv, buf, len);
        if (rc != 1) {
            LOG_WRN("Power mode config: rc=%d", rc);
        } else {
            LOG_INF("  Power mode Normal: OK");
        }
    }

    /* 5. Elevation mask = 5 degrees, CNR mask = 15 dB-Hz */
    len = orion_cmd_cfg_elev_cnr_mask(buf, sizeof(buf), 5, 15, attr);
    if (len > 0) {
        rc = orion_driver_send_command(drv, buf, len);
        if (rc != 1) {
            LOG_WRN("Elev/CNR mask config: rc=%d", rc);
        } else {
            LOG_INF("  Elevation mask 5 deg, CNR mask 15 dB-Hz: OK");
        }
    }

    LOG_INF("CubeSat default configuration applied.");
    return 0;
}

/**
 * @brief Low-power configuration for eclipse / low-activity periods.
 *
 * Reduces update rate to 1 Hz and enables power-save mode.
 */
int orion_config_low_power(orion_driver_t *drv)
{
    uint8_t buf[32];
    int len, rc;

    LOG_INF("Applying low-power GNSS configuration...");

    /* Reduce to 1 Hz */
    len = orion_cmd_cfg_update_rate(buf, sizeof(buf), 1, ORION_ATTR_SRAM_ONLY);
    if (len > 0) {
        rc = orion_driver_send_command(drv, buf, len);
        LOG_INF("  Update rate 1 Hz: %s", rc == 1 ? "OK" : "FAIL");
    }

    /* Power save mode */
    len = orion_cmd_cfg_power_mode(buf, sizeof(buf),
                                   ORION_POWER_SAVE, ORION_ATTR_SRAM_ONLY);
    if (len > 0) {
        rc = orion_driver_send_command(drv, buf, len);
        LOG_INF("  Power mode PowerSave: %s", rc == 1 ? "OK" : "FAIL");
    }

    return 0;
}

#endif /* __ZEPHYR__ */
