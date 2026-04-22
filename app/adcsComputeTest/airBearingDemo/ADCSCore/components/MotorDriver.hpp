#ifndef CORE_MOTORDRIVER_HPP
#define CORE_MOTORDRIVER_HPP

#include "core_Parameters.hpp"
#include "ADCSCore.hpp"
#include "core_Saturate.hpp"

class MotorDriver {
public: 

    // Constructor 
    MotorDriver();
    using Vector4 = Param::Vector4;
    using Scalar = Param::Real;
    using MissionMode = ADCS::MissionMode;
    Vector4 computeMotorCommands(const Vector4& tau_w_cmd, const Vector4& omega_w, const Scalar& dt, const MissionMode& mission_mode);

private:
    // Private Members
    Scalar k_p, k_d; // PD control gains 
    Scalar I_wheel; // Wheel inertia
    Scalar M_PI; // Pi constant
    Vector4 b; // Intercept for duty cycle mapping
    Vector4 m; // Slope for duty cycle mapping
    Scalar omega_w_max; // Maximum wheel speed (RPM)
    Vector4 prev_omega_w; // Previous wheel speeds for acceleration calculation
    Vector4 omega_target; // Target wheel speeds for PD control
    MissionMode last_mode; // To detect mode changes for potential reinitialization
};

#endif // CORE_MOTORDRIVER_HPP