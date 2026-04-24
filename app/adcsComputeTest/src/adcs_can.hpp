#ifndef ADCS_CAN_HPP
#define ADCS_CAN_HPP

/*
 * ADCS-side CAN layer.
 *
 * - Init FDCAN1 (500 kbit/s, classical CAN, 29-bit IDs).
 * - TX: one wheel-cmd frame per ADCS tick with 4 x int16 RPM setpoints.
 * - RX: dedicated msgq + background thread that decodes wheel telemetry
 *   from the motor board and publishes the latest measured omega to
 *   shared state consumable from the ADCS loop.
 *
 * All RPM values are signed mechanical RPM at the wheel rotor.
 */

#include "core_Math.hpp"
#include <stdint.h>

int  adcs_can_init(void);

int  adcs_can_send_wheel_rpm(const float rpm_ref[4]);

/*
 * Copy the most-recent wheel speed telemetry into out[] (rad/s).
 * Returns true if the latest sample is younger than max_age_ms;
 * otherwise out[] is filled with the last valid sample (zeros if none).
 */
bool adcs_can_get_wheel_speeds(Math::Vec<4>& out, int64_t max_age_ms);

int64_t adcs_can_last_rx_age_ms(void);
uint32_t adcs_can_tx_count(void);
uint32_t adcs_can_rx_count(void);

#endif // ADCS_CAN_HPP
