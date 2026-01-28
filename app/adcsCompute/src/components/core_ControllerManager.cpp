#include "core_ControllerManager.hpp"

// Constructor 
ControllerManager::ControllerManager() 
    : // Initialize Members
    NDI(),
    Bdot(),
    Ts(Param::SimTime::Ts),
    prev_mode(Param::PointingMode::OFF)
{

}

ControllerManager::ControlOutput ControllerManager::update(const Param::Vector17& states_hat, 
                                                            const Param::Vector10& reference,
                                                            const Param::Vector29& measurements,
                                                            const Param::PointingMode mode) {
    ControlOutput output;

    // Depending on mode, select controller
    if (mode == Param::PointingMode::DETUMBLE) {
        // B-Dot Controller
        auto bdot_out = Bdot.update(measurements, states_hat);
        
        // FIX: output.tau is Vector7. bdot_out.tau_sat is Vector3 (MTQ only).
        // Structure is [Wheel(4); MTQ(3)]. Set Wheels to 0.
        //output.tau << 0.0, 0.0, 0.0, 0.0, bdot_out.tau_sat;
        output.tau = Param::Vector7::Zero();
        output.tau(4) = bdot_out.tau_sat(0);
        output.tau(5) = bdot_out.tau_sat(1);
        output.tau(6) = bdot_out.tau_sat(2);
        output.states_m = bdot_out.states_m;

    } else if (mode == Param::PointingMode::POINT) {
        // NDI Controller
        auto ndi_out = NDI.update(states_hat, reference, measurements);
        
        // FIX: output.tau is Vector7. NDI returns separate Wheel(4) and MTQ(3).
        // Concatenate them.
        //output.tau << ndi_out.tau_wheel, ndi_out.tau_mtq;
        output.tau(0) = ndi_out.tau_wheel(0);
        output.tau(1) = ndi_out.tau_wheel(1);
        output.tau(2) = ndi_out.tau_wheel(2);
        output.tau(3) = ndi_out.tau_wheel(3);
        output.tau(4) = ndi_out.tau_mtq(0);
        output.tau(5) = ndi_out.tau_mtq(1);
        output.tau(6) = ndi_out.tau_mtq(2);
        
        output.states_m = ndi_out.states_m;

    } else {
        // OFF Mode: Zero torques
        output.tau = Param::Vector7::Zero();
        output.states_m = Param::Vector7::Zero();
    }

    // Update previous mode
    prev_mode = mode;

    return output;
}