#ifndef WHEEL_BRIDGE_HPP
#define WHEEL_BRIDGE_HPP

/*
 * Torque -> Reference-RPM bridge for the app layer.
 *
 * ADCSCore produces per-wheel torque commands in N*m. The motor driver
 * board expects per-wheel RPM setpoints over CAN (its PI speed loop
 * consumes those). This bridge converts between the two once per ADCS
 * tick, without leaking speed-space bookkeeping into the ADCS core.
 *
 * Math (per wheel, each tick):
 *   tau_sat   = clamp(tau_cmd, -tau_max, +tau_max)
 *   omega_ref = clamp(omega_meas + (dt / J_w) * tau_sat,
 *                     -omega_cap, +omega_cap)
 *   rpm_ref   = omega_ref * 60 / (2*pi)
 *
 * omega_meas comes from the motor board's latest telemetry. The saturation
 * cap is the min of ADCS Param::omega_w_max and the motor board's CAN
 * limit (WHEEL_RPM_LIMIT) so ADCS never accumulates commands the motor
 * side will clip away.
 */

#include "ADCSCore.hpp"

namespace ADCS {

void wheel_bridge_step(const Math::Vec<4>& tau_cmd_Nm,
		       const Math::Vec<4>& omega_meas_rad_s,
		       float dt_s,
		       float rpm_ref_out[4]);

} // namespace ADCS

#endif // WHEEL_BRIDGE_HPP
