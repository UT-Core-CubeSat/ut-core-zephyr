#include "ADCSCore.hpp"
#include <cmath>

namespace ADCS {

Core::Core()
    : observer_(), controller_(), last_time(static_cast<Param::TimeReal>(0.0)), first_update(true),
    workspace_meas_(Param::Vector13::Zero()),
    bearing_mode_armed(false)
{
}

AdcsOutput Core::update(const SensorData& sensors, const Command& command)
{

    // Compute variable dt 
    Param::TimeReal dt;
    if (first_update) {
        dt = static_cast<Param::TimeReal>(0.0);
        first_update = false;
    } else {
        dt = sensors.unix_time - last_time;
    }
    last_time = sensors.unix_time;

    // 1. Pack measurements for observer (reuse workspace buffer to avoid per-call allocation)
    // [0-2]: accelerometer (gravity in body frame)
    // [3-5]: gyro (body rates)
    // [6-8]: magnetometer (B-field in body frame)
    // [9-12]: wheel speeds
    workspace_meas_.setSegment(0, sensors.accelerometer);
    workspace_meas_.setSegment(3, sensors.gyro);
    workspace_meas_.setSegment(6, sensors.magnetometer);
    workspace_meas_.setSegment(9, sensors.wheel_speeds);

    // 2. Run observer
    Param::Real dt_scalar = static_cast<Param::Real>(dt);  // Safe: dt is small (0.025s), no precision loss
    Param::Vector11 states_hat = observer_.update(workspace_meas_, sensors.unix_time, dt_scalar);

    // 3. Build reference (identity pointing: unit quaternion, zero rates/accel)
    Param::Vector10 reference = Param::Vector10::Zero();
    reference(0) = 1; // unit quaternion, no rotation

    if (command.mode != MissionMode::BEARING) {
        bearing_mode_armed = false;
    }

    Param::PointingMode mode = Param::PointingMode::POINT;
    if (command.mode == MissionMode::OFF) {
        mode = Param::PointingMode::OFF;
    } else if (command.mode == MissionMode::SAFE) {
        mode = Param::PointingMode::DETUMBLE;
    } else if (command.mode == MissionMode::BEARING) {
        if (!bearing_mode_armed) {
            Param::Real w_thresh = Param::Controller::bearing_entry_axis_rate_threshold;
            bool rates_ready = (std::abs(sensors.gyro(0)) <= w_thresh) &&
                               (std::abs(sensors.gyro(1)) <= w_thresh) &&
                               (std::abs(sensors.gyro(2)) <= w_thresh);
            if (rates_ready) {
                bearing_mode_armed = true;
            }
        }
        mode = bearing_mode_armed ? Param::PointingMode::POINT : Param::PointingMode::DETUMBLE;
    }

    // 4. Run controller
    auto ctrl_out = controller_.update(states_hat, reference, workspace_meas_, mode, dt_scalar);

    // 5. Pack output
    // states_hat layout (ObserverClass::StateVector = Vector11):
    //   [0..3]   quaternion q_hat
    //   [4..6]   bias-corrected body rate omega_b_hat
    //   [7..10]  wheel speeds omega_w (from measurements)
    AdcsOutput out;
    out.wheel_torque = ctrl_out.tau.segment<4>(0);
    out.mtq_dipole = ctrl_out.tau.segment<3>(4);
    out.attitude_est = states_hat.segment<4>(0);
    out.rate_est = states_hat.segment<3>(4);
    out.estimator_valid = !states_hat.hasNaN();
    if (mode == Param::PointingMode::OFF) {
        out.current_mode = MissionMode::OFF;
    } else if (mode == Param::PointingMode::DETUMBLE) {
        out.current_mode = MissionMode::SAFE;
    } else {
        out.current_mode = MissionMode::BEARING;
    }

    return out;
}

AdcsOutput Core::update(const SensorData& sensors)
{
    Command default_command;
    return update(sensors, default_command);
}

void Core::reset() {
    observer_ = ObserverClass();
    controller_ = ControllerManager();
    first_update = true;
    last_time = static_cast<Param::TimeReal>(0.0);
    bearing_mode_armed = false;
}

} // namespace ADCS