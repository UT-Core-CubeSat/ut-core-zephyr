#include "ADCSCore.hpp"

namespace ADCS {

Core::Core()
    : observer_(), refgen_(), controller_()
{
}

AdcsOutput Core::update(const SensorData& sensors, const Command& cmd)
{
    // 1. Pack measurements for observer (Vector29)
    Param::Vector29 meas;
    meas.setSegment(0, Math::Vec<3>::Zero());  // accelerometer placeholder
    meas.setSegment(3, sensors.gyro);
    meas.setSegment(6, sensors.star_quat);
    meas.setSegment(10, sensors.css_currents);
    meas.setSegment(16, sensors.magnetometer);
    meas.setSegment(19, sensors.gps_ecef);
    meas.setSegment(25, sensors.wheel_speeds);

    // 2. Run observer
    Param::Vector17 states_hat = observer_.update(meas, sensors.unix_time);

    // 3. Run reference generator with command
    auto ref_out = refgen_.update(states_hat, cmd, sensors.unix_time);
    Param::Vector10 reference = ref_out.reference;
    Param::PointingMode internal_mode = ref_out.mode;

    // 4. Run controller
    auto ctrl_out = controller_.update(states_hat, reference, meas, internal_mode);

    // 5. Pack output
    AdcsOutput out;
    out.wheel_torque = ctrl_out.tau.segment<4>(0);
    out.mtq_dipole = ctrl_out.tau.segment<3>(4);
    out.attitude_est = states_hat.segment<4>(6);
    out.rate_est = states_hat.segment<3>(10);
    out.estimator_valid = !states_hat.hasNaN();
    out.current_mode = cmd.mode;  // Echo commanded mode

    // Adding equivalence variables, won't be here later 
    out.reference = reference; 
    out.states_m = ctrl_out.states_m;
    out.states_hat = states_hat;

    return out;
}

void Core::reset() {
    observer_ = ObserverClass();
    refgen_ = ReferenceGenerator();
    controller_ = ControllerManager();
}

} // namespace ADCS