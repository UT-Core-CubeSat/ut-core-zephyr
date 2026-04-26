#include "core_ControllerManager.hpp"

// Constructor 
ControllerManager::ControllerManager() 
    : // Initialize Members
    NDI(),
    Bdot(),
    prev_mode(Param::PointingMode::OFF)
{

}

ControllerManager::ControlOutput ControllerManager::update(const Param::Vector11& states_hat,
                                                            const Param::Vector10& reference,
                                                            const Param::Vector13& measurements,
                                                            const Param::PointingMode mode, Param::Real dt) {
    ControlOutput output;
    output.wheel_rpm  = Param::Vector4::Zero();
    output.mtq_dipole = Param::Vector3::Zero();

    if (mode == Param::PointingMode::DETUMBLE) {
        // BDot controller — MTQ only, wheels idle (0 RPM)
        auto bdot_out = Bdot.update(measurements, states_hat, dt);
        output.mtq_dipole = bdot_out;
        output.mtq_coil_currents = Bdot.getCoilCurrents();

    } else if (mode == Param::PointingMode::MOTOR) {
        // NDI attitude hold — reaction wheels only, no MTQ desaturation
        auto ndi_out = NDI.update(states_hat, reference, measurements, dt,
                                  /*enable_desat=*/false, /*apply_deadband=*/true);
        output.wheel_rpm  = ndi_out.rpm_cmd;
        output.mtq_dipole = Param::Vector3::Zero();
        output.mtq_coil_currents = Param::Vector4::Zero();
        
    } else if (mode == Param::PointingMode::BOTH) {
        // NDI attitude hold + MTQ wheel desaturation
        auto ndi_out = NDI.update(states_hat, reference, measurements, dt,
                                  /*enable_desat=*/true, /*apply_deadband=*/true);
        output.wheel_rpm  = ndi_out.rpm_cmd;
        output.mtq_dipole = ndi_out.mtq_cmd;
        output.mtq_coil_currents = Bdot.getCoilCurrents();

    } else {
        // OFF: all actuators idle
        output.wheel_rpm  = Param::Vector4::Zero();
        output.mtq_dipole = Param::Vector3::Zero();
        output.mtq_coil_currents = Param::Vector4::Zero();
    }

    prev_mode = mode;
    return output;
}