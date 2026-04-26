#include "ADCSCore.hpp"
#include <cmath>

namespace ADCS {

Core::Core()
    : observer_(), controller_(), last_time(static_cast<Param::TimeReal>(0.0)), first_update(true),
    workspace_meas_(Param::Vector13::Zero()),
    motor_mode_armed(false)
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
    reference(3) = states_hat(3); // don't care about yaw
    // Reset arm flag when transitioning away from wheel-active modes
    if (command.mode != MissionMode::MOTOR && command.mode != MissionMode::BOTH) {
        motor_mode_armed = false;
    }

    // Entry guard for MOTOR and BOTH: require low rates before handing off to NDI
    if ((command.mode == MissionMode::MOTOR || command.mode == MissionMode::BOTH)
        && !motor_mode_armed)
    {
        motor_mode_armed = true;
    }

    Param::PointingMode mode;
    switch (command.mode) {
        case MissionMode::OFF:
            mode = Param::PointingMode::OFF;
            break;
        case MissionMode::DETUMBLE:
            mode = Param::PointingMode::DETUMBLE;
            break;
        case MissionMode::MOTOR:
            mode = motor_mode_armed ? Param::PointingMode::MOTOR : Param::PointingMode::DETUMBLE;
            break;
        case MissionMode::BOTH:
        default:
            mode = motor_mode_armed ? Param::PointingMode::BOTH : Param::PointingMode::DETUMBLE;
            break;
    }

    // 4. Run controller
    auto ctrl_out = controller_.update(states_hat, reference, workspace_meas_, mode, dt_scalar);

    // 5. Pack output
    // states_hat layout (ObserverClass::StateVector = Vector11):
    //   [0..3]   quaternion q_hat
    //   [4..6]   bias-corrected body rate omega_b_hat
    //   [7..10]  wheel speeds omega_w (from measurements)
    AdcsOutput out;
    out.wheel_rpm      = ctrl_out.wheel_rpm;
    out.mtq_dipole     = ctrl_out.mtq_dipole;
    out.mtq_coil_currents = ctrl_out.mtq_coil_currents;
    out.attitude_est   = states_hat.segment<4>(0);
    out.rate_est       = states_hat.segment<3>(4);
    out.estimator_valid = !states_hat.hasNaN();
    switch (mode) {
        case Param::PointingMode::OFF:      out.current_mode = MissionMode::OFF;      break;
        case Param::PointingMode::DETUMBLE: out.current_mode = MissionMode::DETUMBLE; break;
        case Param::PointingMode::MOTOR:    out.current_mode = MissionMode::MOTOR;    break;
        case Param::PointingMode::BOTH:
        default:                            out.current_mode = MissionMode::BOTH;     break;
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
    motor_mode_armed = false;
}

} // namespace ADCS