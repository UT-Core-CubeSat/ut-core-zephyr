#include "ADCSCore.hpp"

namespace ADCS {

Core::Core()
    : observer_(), controller_(), first_update(true), last_time(static_cast<Param::TimeReal>(0.0)), mode(Param::PointingMode::POINT)
{
}

AdcsOutput Core::update(const SensorData& sensors)
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

    // 1. Pack measurements for observer (Vector13)
    // [0-2]: accelerometer (gravity in body frame)
    // [3-5]: gyro (body rates)
    // [6-8]: magnetometer (B-field in body frame)
    // [9-12]: wheel speeds
    Param::Vector13 meas;
    meas.setSegment(0, sensors.accelerometer);
    meas.setSegment(3, sensors.gyro);
    meas.setSegment(6, sensors.magnetometer);
    meas.setSegment(9, sensors.wheel_speeds);

    // 2. Run observer
    Param::Real dt_scalar = static_cast<Param::Real>(dt);  // Safe: dt is small (0.025s), no precision loss
    Param::Vector11 states_hat = observer_.update(meas, sensors.unix_time, dt_scalar);

    // 3. Run reference generator with command
    Param::Vector10 reference = Param::Vector10::Zero();
    reference(0) = 1; // unit quaternion, no rotation

    // 4. Run controller
    auto ctrl_out = controller_.update(states_hat, reference, meas, mode, dt_scalar);

    // 5. Pack output
    AdcsOutput out;
    out.wheel_torque = ctrl_out.tau.segment<4>(0);
    out.mtq_dipole = ctrl_out.tau.segment<3>(4);
    out.attitude_est = states_hat.segment<4>(6);
    out.rate_est = states_hat.segment<3>(10);
    out.estimator_valid = !states_hat.hasNaN();

    // Adding equivalence variables, won't be here later 
    out.reference = reference; 
    out.states_m = ctrl_out.states_m;
    out.states_hat = states_hat;

    return out;
}

void Core::reset() {
    observer_ = ObserverClass();
    controller_ = ControllerManager();
    first_update = true;
    last_time = static_cast<Param::TimeReal>(0.0);
    mode = Param::PointingMode::POINT;
}

} // namespace ADCS