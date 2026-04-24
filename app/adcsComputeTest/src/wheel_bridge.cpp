#include "wheel_bridge.hpp"

#include "core_Parameters.hpp"

#include <algorithm>
#include <cmath>

namespace ADCS {

namespace {

constexpr float RAD_PER_S_TO_RPM = 60.0f / (2.0f * static_cast<float>(Param::PI));

/*
 * Saturate at whichever cap is tighter: the control-law speed limit
 * (Param::omega_w_max) or the motor-board hardware RPM cap expressed in
 * rad/s. Converting WHEEL_RPM_LIMIT here avoids a build-time dependency
 * on can_wheel.h from this TU.
 */
constexpr float WHEEL_RPM_HW_CAP = 10000.0f;
constexpr float OMEGA_HW_CAP_RAD_S =
	WHEEL_RPM_HW_CAP * (2.0f * static_cast<float>(Param::PI)) / 60.0f;

inline float clampf(float x, float lo, float hi)
{
	return std::max(lo, std::min(hi, x));
}

} // namespace

void wheel_bridge_step(const Math::Vec<4>& tau_cmd_Nm,
		       const Math::Vec<4>& omega_meas_rad_s,
		       float dt_s,
		       float rpm_ref_out[4])
{
	const float tau_max   = static_cast<float>(Param::Actuators::tau_w_max);
	const float omega_cap = std::min(
		static_cast<float>(Param::Actuators::omega_w_max),
		OMEGA_HW_CAP_RAD_S);

	/* dt<=0 degenerates to pure hold-at-measured; harmless but defensive. */
	const float dt = (dt_s > 0.f && std::isfinite(dt_s)) ? dt_s : 0.f;
	const float inv_Jw = 1.0f / static_cast<float>(Param::Actuators::I_wheel);

	for (int i = 0; i < 4; i++) {
		float tau = clampf(static_cast<float>(tau_cmd_Nm(i)), -tau_max, tau_max);
		float omega = static_cast<float>(omega_meas_rad_s(i));
		float omega_next = clampf(omega + dt * inv_Jw * tau,
					  -omega_cap, omega_cap);

		rpm_ref_out[i] = omega_next * RAD_PER_S_TO_RPM;
	}
}

} // namespace ADCS
